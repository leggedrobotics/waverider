#include <gflags/gflags.h>
#include <wavemap_ros/ros_server.h>

#include "waverider_ros/waverider_server.h"

class MapChangedCallbackOperation : public wavemap::MapOperationBase {
 public:
  using Callback = std::function<void(const wavemap::MapBase&)>;
  MapChangedCallbackOperation(wavemap::MapBase::Ptr occupancy_map,
                              Callback callback)
      : wavemap::MapOperationBase(std::move(occupancy_map)),
        callback_(std::move(callback)) {}

  void run(bool /*force_run*/) override {
    CHECK_NOTNULL(occupancy_map_);
    CHECK_NOTNULL(callback_);
    std::invoke(callback_, *occupancy_map_);
  }

 private:
  Callback callback_;
};

int main(int argc, char** argv) {
  // Register with ROS
  ros::init(argc, argv, "waverider");

  // Setup GLOG and register a failure signal handler that prints the callstack
  // if the program is killed (e.g. SIGSEGV)
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);
  google::InstallFailureSignalHandler();
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;

  // Create the mapper
  ros::CallbackQueue mapper_callback_queue;
  ros::AsyncSpinner mapper_spinner{1, &mapper_callback_queue};
  std::unique_ptr<wavemap::RosServer> wavemap_server;
  {
    ros::NodeHandle nh_mapper;
    ros::NodeHandle nh_mapper_private{"~/mapper"};
    nh_mapper.setCallbackQueue(&mapper_callback_queue);
    nh_mapper_private.setCallbackQueue(&mapper_callback_queue);
    wavemap_server =
        std::make_unique<wavemap::RosServer>(nh_mapper, nh_mapper_private);
  }

  // Create the planner
  ros::CallbackQueue planner_callback_queue;
  ros::AsyncSpinner planner_spinner{1, &planner_callback_queue};
  std::unique_ptr<waverider::WaveriderServer> waverider_server;
  {
    ros::NodeHandle nh_planner;
    ros::NodeHandle nh_planner_private{"~/planner"};
    nh_planner.setCallbackQueue(&planner_callback_queue);
    nh_planner_private.setCallbackQueue(&planner_callback_queue);
    waverider_server = std::make_unique<waverider::WaveriderServer>(
        nh_planner, nh_planner_private);
  }

  // Subscribe waverider to wavemap map updates
  {
    auto map_changed_callback_op =
        std::make_unique<MapChangedCallbackOperation>(
            wavemap_server->getMap(),
            [&waverider_server](const wavemap::MapBase& map) {
              std::cout << "EVAL\t" << ros::Time::now()
                        << "\tSTARTED obstacle cells update" << std::endl;
              waverider_server->updateMap(map);
              std::cout << "EVAL\t" << ros::Time::now()
                        << "\tFINISHED obstacle cells update" << std::endl;
            });
    wavemap_server->getPipeline().addOperation(
        std::move(map_changed_callback_op));
  }

  // Start processing inputs
  mapper_spinner.start();
  planner_spinner.start();

  // Start publishing planning policies
  waverider_server->startPlanningAsync();

  // Keep running until a shutdown request is received
  ros::waitForShutdown();
  return 0;
}
