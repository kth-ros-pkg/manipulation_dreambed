from geometry_msgs.msg import Pose as ROSPose
from geometry_msgs.msg import PoseStamped as ROSPoseStamped
from std_msgs.msg import Header
from Context import Pose as ContextPose
from Context import ConfigurationWrapper as ContextConfiguration
from MethodTypes import Trajectory as ContextTrajectory
from MethodTypes import Waypoint as ContextWaypoint
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion
from rospkg import RosPack
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
import rospy
import numpy


def toROSPose(cPose, bStamped=False):
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
        header = Header(stamp=rospy.Time.now(), frame_id=cPose.frame)
        pose = ROSPoseStamped(pose=pose, header=header)
    return pose


def toContextPose(rosPose, frame='world'):
    if type(rosPose) is ROSPoseStamped:
        frame = rosPose.header.frame_id
        rosPose = rosPose.pose
    position = numpy.array([rosPose.position.x, rosPose.position.y, rosPose.position.z])
    orientation = numpy.array([rosPose.orientation.x, rosPose.orientation.y,
                              rosPose.orientation.z, rosPose.orientation.w])
    return ContextPose(position=position, orientation=orientation, frame=frame)


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


def toROSTrajectory(trajectory):
    points = []
    for wp in trajectory.waypoints:
        time_stamp = rospy.Duration(secs=wp.timestamp[0], nsecs=wp.timestamp[1])
        jtp = JointTrajectoryPoint(positions=wp.positions, velocities=wp.velocities,
                                   accelerations=wp.accelerations, time_from_start=time_stamp)
        points.append(jtp)
    joint_traj = JointTrajectory(joint_names=trajectory.joint_names, points=points)
    return joint_traj


def toContextTrajectory(ros_trajectory):
    context_trajectory = ContextTrajectory(group_name='', joint_names=ros_trajectory.joint_names)
    for wp in ros_trajectory.points:
        context_waypoint = ContextWaypoint(timestamp=(wp.time_from_start.secs, wp.time_from_start.nsecs),
                                           positions=wp.positions, velocities=wp.velocities,
                                           accelerations=wp.accelerations)
        context_trajectory.appendWaypoint(context_waypoint)
    return context_trajectory


def toJointState(configuration, names=None):
    if names is None:
        names = configuration.keys()
    joint_state = JointState()
    joint_state.header.frame_id = ''
    joint_state.header.stamp = rospy.Time.now()
    joint_state.name = names
    joint_state.position = [configuration[name] for name in names]
    return joint_state


def extractPartialJointState(joint_state, names):
    new_joint_state = JointState()
    indices = [i for i in range(len(joint_state.name)) if joint_state.name[i] in names]
    new_joint_state.name = [joint_state.name[i] for i in indices]
    new_joint_state.position = [joint_state.position[i] for i in indices]
    if len(joint_state.velocity) > 0:
        new_joint_state.velocity = [joint_state.velocity[i] for i in indices]
    if len(joint_state.effort) > 0:
        new_joint_state.effort = [joint_state.effort[i] for i in indices]
    return new_joint_state


def toContextConfiguration(joint_state):
    config = dict(zip(joint_state.name, joint_state.position))
    return config


def extract_part_trajectory(joint_names, trajectory):
    """
    Returns a copy of the given trajectory that is reduced to the given set of joints.
    :param joint_names: list of joints to keep.
    :param trajectory: input trajectory
    :return: a copy of the input trajectory that only contains the joints defined in joint_names
    """
    output_traj = JointTrajectory()
    output_traj.header = trajectory.header
    indices = []
    num_joints = len(trajectory.joint_names)
    for name_idx in range(num_joints):
        if trajectory.joint_names[name_idx] in joint_names:
            indices.append(name_idx)
    output_traj.joint_names = [trajectory.joint_names[i] for i in range(num_joints) if i in indices]
    for point in trajectory.points:
        new_point = JointTrajectoryPoint()
        new_point.positions = [point.positions[i] for i in range(len(point.positions)) if i in indices]
        new_point.velocities = [point.velocities[i] for i in range(len(point.velocities)) if i in indices]
        new_point.accelerations = [point.accelerations[i] for i in range(len(point.accelerations)) if i in indices]
        new_point.time_from_start = point.time_from_start
        output_traj.points.append(new_point)
    return output_traj



