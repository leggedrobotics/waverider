#!/usr/bin/env python3

import rospy
import yaml
import tf2_ros
import geometry_msgs.msg
from tf.transformations import quaternion_from_euler
import os
import rospkg

class YamlTfStaticPublisher:
    def __init__(self):
        rospy.init_node('yaml_tf_static_publisher', anonymous=True)
        
        # Get the YAML file path from a parameter
        yaml_file = rospy.get_param('~yaml_file', '')
        if not yaml_file:
            # Try to find a default path using the package path
            try:
                rospack = rospkg.RosPack()
                package_path = rospack.get_path('wavemap_tf_publisher')
                yaml_file = os.path.join(package_path, 'config', 'transforms.yaml')
                rospy.loginfo(f"No YAML file specified, using default: {yaml_file}")
            except rospkg.common.ResourceNotFound:
                rospy.logerr("No YAML file specified and couldn't find default. Please provide a path using the 'yaml_file' parameter.")
                return
        
        if not os.path.exists(yaml_file):
            rospy.logerr(f"YAML file not found: {yaml_file}")
            return
        
        # Static transform broadcaster
        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster()
        
        # Load and publish transforms
        self.load_and_publish_transforms(yaml_file)
        
        rospy.loginfo(f"Successfully loaded transforms from {yaml_file}")

    def load_and_publish_transforms(self, yaml_file):
        try:
            with open(yaml_file, 'r') as file:
                transforms_data = yaml.safe_load(file)
                
            if not transforms_data or not isinstance(transforms_data, list):
                rospy.logerr("Invalid YAML format. Expected a list of transforms.")
                return
                
            static_transforms = []
            
            for transform in transforms_data:
                if not self.validate_transform(transform):
                    continue
                
                # Create transform message
                static_transform = geometry_msgs.msg.TransformStamped()
                
                static_transform.header.stamp = rospy.Time.now()
                static_transform.header.frame_id = transform['parent_frame']
                static_transform.child_frame_id = transform['child_frame']
                
                # Set translation
                static_transform.transform.translation.x = transform['translation']['x']
                static_transform.transform.translation.y = transform['translation']['y']
                static_transform.transform.translation.z = transform['translation']['z']
                
                # Set rotation from either quaternion or Euler angles
                if 'quaternion' in transform:
                    static_transform.transform.rotation.x = transform['quaternion']['x']
                    static_transform.transform.rotation.y = transform['quaternion']['y']
                    static_transform.transform.rotation.z = transform['quaternion']['z']
                    static_transform.transform.rotation.w = transform['quaternion']['w']
                elif 'rotation' in transform:
                    # Convert Euler angles to quaternion
                    roll = transform['rotation'].get('roll', 0.0)
                    pitch = transform['rotation'].get('pitch', 0.0)
                    yaw = transform['rotation'].get('yaw', 0.0)
                    
                    q = quaternion_from_euler(roll, pitch, yaw)
                    static_transform.transform.rotation.x = q[0]
                    static_transform.transform.rotation.y = q[1]
                    static_transform.transform.rotation.z = q[2]
                    static_transform.transform.rotation.w = q[3]
                
                static_transforms.append(static_transform)
                rospy.loginfo(f"Added transform: {transform['parent_frame']} -> {transform['child_frame']}")
            
            # Publish all transforms
            if static_transforms:
                self.static_broadcaster.sendTransform(static_transforms)
                rospy.loginfo(f"Published {len(static_transforms)} static transforms")
            else:
                rospy.logwarn("No valid transforms found in the YAML file")
                
        except yaml.YAMLError as e:
            rospy.logerr(f"Error parsing YAML file: {e}")
        except Exception as e:
            rospy.logerr(f"Error loading transforms: {e}")

    def validate_transform(self, transform):
        required_fields = ['parent_frame', 'child_frame', 'translation']
        
        # Check required fields
        for field in required_fields:
            if field not in transform:
                rospy.logerr(f"Missing required field '{field}' in transform")
                return False
        
        # Check that at least one rotation representation is provided
        if 'quaternion' not in transform and 'rotation' not in transform:
            rospy.logerr("Transform must include either 'quaternion' or 'rotation' (Euler angles)")
            return False
            
        # Check translation fields
        if not all(key in transform['translation'] for key in ['x', 'y', 'z']):
            rospy.logerr("Transform 'translation' must include x, y, and z values")
            return False
            
        # If quaternion is provided, check its fields
        if 'quaternion' in transform:
            if not all(key in transform['quaternion'] for key in ['x', 'y', 'z', 'w']):
                rospy.logerr("Transform 'quaternion' must include x, y, z, and w values")
                return False
        
        return True

if __name__ == '__main__':
    try:
        publisher = YamlTfStaticPublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass