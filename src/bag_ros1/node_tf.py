#!/usr/bin/env python3

import geometry_msgs.msg
import rospy
import tf2_ros


def publish_static_transform():
    rospy.init_node('base_to_laser_broadcaster', anonymous=True)
    
    broadcaster = tf2_ros.StaticTransformBroadcaster()
    
    static_transform_stamped = geometry_msgs.msg.TransformStamped()
    
    static_transform_stamped.header.stamp = rospy.Time.now()
    static_transform_stamped.header.frame_id = "base_link"
    static_transform_stamped.child_frame_id = "laser"
    
    static_transform_stamped.transform.translation.x = 0.0
    static_transform_stamped.transform.translation.y = 0.0
    static_transform_stamped.transform.translation.z = 0.0
    
    static_transform_stamped.transform.rotation.x = 0.0
    static_transform_stamped.transform.rotation.y = 0.0
    static_transform_stamped.transform.rotation.z = 0.0
    static_transform_stamped.transform.rotation.w = 1.0
    
    broadcaster.sendTransform(static_transform_stamped)
    
    rospy.loginfo("Publishing static transform from base_link to laser")
    
    rospy.spin()

if __name__ == '__main__':
    try:
        publish_static_transform()
    except rospy.ROSInterruptException:
        pass
