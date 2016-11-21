from geometry_msgs.msg import Pose as ROSPose
from geometry_msgs.msg import PoseStamped as ROSPoseStamped
from std_msgs.msg import Header
from Context import Pose as ContextPose
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion
from rospkg import RosPack
import rospy
import numpy


def contextPoseToROSPose(cPose, bStamped=False, frame_id='/world'):
    pos = Vector3()
    pos.x = cPose.position[0]
    pos.y = cPose.position[1]
    pos.z = cPose.position[2]

    quat = Quaternion()
    quat.x = cPose.orientation[0]
    quat.y = cPose.orientation[1]
    quat.z = cPose.orientation[2]
    quat.w = cPose.orientation[3]
    pose = ROSPose(position=pos, orientation=quat)
    if bStamped:
        header = Header(stamp=rospy.Time.now(), frame_id=frame_id)
        pose = ROSPoseStamped(pose=pose, header=header)
    return pose


def rosPoseToContextPose(rosPose):
    position = numpy.array([rosPose.position.x, rosPose.position.y, rosPose.position.z])
    orientation = numpy.array([rosPose.orientation.x, rosPose.orientation.y,
                              rosPose.orientation.z, rosPose.orientation.w])
    return ContextPose(position=position, orientation=orientation)


def resolvePath(path):
    """
        If the given path contains $ROS_PACKAGE_PATH macros, these macros are resolved and replaced
        by the absolute path. (e.g. ROS_PACKAGE_PATH(manipulation_optimizer)/data becomes
            /home/userName/catkin_ws/src/manipulation_optimizer/data)
    """
    package_key_word = '$ROS_PACKAGE_PATH('
    # or there is a ros package specified

    firstOccurrence = path.find(package_key_word)
    while firstOccurrence != -1:
        beginPackageName = firstOccurrence + len(package_key_word)
        endPackageName = path.find(')', beginPackageName)
        package_name = path[beginPackageName:endPackageName]
        rp = RosPack()
        prefix = path[0:firstOccurrence]
        postfix = path[endPackageName+1:len(path)]
        package_path = rp.get_path(package_name)
        path = prefix + package_path + postfix
        firstOccurrence = path.find(package_key_word)
    return path
