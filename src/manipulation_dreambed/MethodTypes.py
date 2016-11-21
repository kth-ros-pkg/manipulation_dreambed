"""
    This file is part of the RobDREAM (http://robdream.eu/) deliverable D4.2.
    It contains the abstract interfaces of each method type as defined in deliverable D4.1.

    @author: Joshua Haustein (haustein@kth.se)
"""
import abc


class MethodType(object):
    """ Abstract super class of each method type. """
    __metaclass__ = abc.ABCMeta

    @abc.abstractmethod
    def getName(self):
        """ Returns the name of this method. """
        pass

    @abc.abstractmethod
    def getParameters(self, role, paramPrefix):
        """ Returns a pySMAC compatible parameter definition (see deliverable D2.1)
            @param role - Since a single method may implement multiple interfaces
                          (for instance an ArmPlanner and ArmController) this parameter specifies
                          for which role the parameters should be returned. In the example, if this
                          method has the parameters plannerParamA, plannerParamB, controllerParamA
                          it would return plannerParamA and plannerParamB in case role = ArmPlanner
                          and controllerParamA in case role == ArmController
            @param paramPrefix - a string containing the prefix used for the parameters keys
        """
        pass

    @abc.abstractmethod
    def getConditionals(self, role, paramPrefix):
        """ Returns a pySMAC compatible definition of conditional parameters (see deliverable D2.1)
            @param role - see getParameters()
            @param paramPrefix - a string containing the prefix used for the parameters keys
        """
        pass

    @abc.abstractmethod
    def getForbiddenConfigurations(self, role, paramPrefix):
        """ Returns a pySMAC compatible definition of forbidden parameters (see deliverable D2.1)
            @param role - see getParameters()
            @param paramPrefix - a string containing the prefix used for the parameters keys
        """
        pass

    @abc.abstractmethod
    def initialize(self):
        """ Intializes this method type so that it is ready to use afterwards.
            Note that any resources that would prevent other method types from succesfully
            executing should be acquired in @allocateResources."""
        pass

    @abc.abstractmethod
    def allocateResources(self):
        """ Allocates all resources that are required to run this method but can interfere
            with other method types if loaded. This should be the same resources as they are
            released in @releaseResources. """
        pass

    @abc.abstractmethod
    def releaseResources(self):
        """ Releases all resources that may interfere with other method types.
            This should be the same resources as the ones acquired in @allocateResources."""
        pass

    @abc.abstractmethod
    def destroy(self):
        """ Notifies the method type to destroy any allocated resources/ terminate any subprocesses. """
        pass

    @abc.abstractmethod
    def getType(self):
        """ Returns a string representing the type of this method. """
        pass

    @abc.abstractmethod
    def hasResourceConflict(self, otherMethod):
        """ Returns whether this method type may have resource conflicts with the given method. """
        pass


class ArmPlanner(MethodType):
    """ Plans an arm trajectory to a specified goal. """
    # @abc.abstractmethod
    # def preparePlanning(self, context):
    #     pass

    @abc.abstractmethod
    def plan(self, goal, context, paramPrefix, parameters):
        """
            Plans an arm trajectory to the given goal.
            @param goal - can either be a configuration, a pose or a position
            @param context - the context
            @param paramPrefix - a string containing the prefix used for the parameters keys
            @param parameters - a dict containing the parameters for this method.
            @return Trajectory - The planned trajectory, None if no solution was found.
        """
        pass

    def getType(self):
        return 'ArmPlanner'

    def hasResourceConflict(self, otherMethod):
        return False


class ArmController(MethodType):
    """ Executes a given arm trajectory. """

    @abc.abstractmethod
    def execute(self, trajectory, context, paramPrefix, parameters):
        """
            Executes the given trajectory.
            @param trajectory - of type Trajectory
            @param context - the context
            @param paramPrefix - a string containing the prefix used for the parameters keys
            @param parameters - a dict containing the parameters for this method.
            @return boolean - True if success, else False
        """
        pass

    def getType(self):
        return 'ArmController'

    def hasResourceConflict(self, otherMethod):
        return otherMethod.getType() == 'COMController' \
               or otherMethod.getType() == 'ToolUseController' \
               or otherMethod.getType() == 'GraspController' \
               or otherMethod.getType() == 'PlaceController'


class GraspPlanner(MethodType):
    """ Plans a grasp for a given object. """

    @abc.abstractmethod
    def plan(self, object, context, paramPrefix, parameters):
        """
            Plans a grasp for the given object.
            @param object - object information containing the pose and the name of the object.
            @param context - the context
            @param paramPrefix - a string containing the prefix used for the parameters keys
            @param parameters - a dict containing the parameters for this method.
            @return GraspResult - The planned grasp, None if failed
        """
        pass

    def getType(self):
        return 'GraspPlanner'

    def hasResourceConflict(self, otherMethod):
        return False


class GraspController(MethodType):
    """ Executes a grasp and controls the robot hand while an object is grasped. """

    @abc.abstractmethod
    def startExecution(self, grasp, context, paramPrefix, parameters):
        """
            Starts executing the given grasp. This function is only blocking until the grasp
            is established. The controller may continue running (e.g. for adaptive grasping)
            until stopExecution() is called.
            @param grasp - the GraspResult to execute.
            @param context - the context
            @param paramPrefix - a string containing the prefix used for the parameters keys
            @param parameters - a dict containing the parameters for this method.
            @return boolean - True if grasp established, else False
        """
        pass

    @abc.abstractmethod
    def stopExecution(self):
        """
            Stops the execution of the grasp controller. Note, that the gripper should remain
            stiff after this call, as a grasped object may be dropped otherwise. The purpose of
            this function is to notify the grasp controller that another process is going to control
            the robot hand from now on.
            @return boolean - False in case of any severe error, else True
        """
        pass

    def getType(self):
        return 'GraspController'

    def hasResourceConflict(self, otherMethod):
            return otherMethod.getType() == 'COMController' \
                   or otherMethod.getType() == 'ToolUseController' \
                   or otherMethod.getType() == 'GraspController' \
                   or otherMethod.getType() == 'PlaceController'


class PlacePlanner(MethodType):
    """ Plans to place an object. """

    @abc.abstractmethod
    def plan(self, placementInfo, context, paramPrefix, parameters):
        # TODO
        pass

    def getType(self):
        return 'PlacePlanner'

    def hasResourceConflict(self, otherMethod):
        return False


class PlaceController(MethodType):
    """ Executes the placement of an object. """

    @abc.abstractmethod
    def execute(self, trajectory, context, paramPrefix, parameters):
        # TODO
        pass

    def getType(self):
        return 'PlaceController'

    def hasResourceConflict(self, otherMethod):
        return otherMethod.getType() == 'COMController' \
               or otherMethod.getType() == 'ToolUseController' \
               or otherMethod.getType() == 'GraspController' \
               or otherMethod.getType() == 'PlaceController'


class COMPlanner(MethodType):
    """ Plans to manipulate a constrained object (e.g. a door handle). """

    @abc.abstractmethod
    def plan(self):
        # TODO
        pass

    def getType(self):
        return 'COMPlanner'

    def hasResourceConflict(self, otherMethod):
        return False


class COMController(MethodType):
    """ Executes the manipulation planned by a COMPlanner. """

    @abc.abstractmethod
    def execute(self):
        # TODO
        pass

    def getType(self):
        return 'COMController'

    def hasResourceConflict(self, otherMethod):
        return otherMethod.getType() == 'COMController' \
               or otherMethod.getType() == 'ToolUseController' \
               or otherMethod.getType() == 'GraspController' \
               or otherMethod.getType() == 'PlaceController'


class ToolUsePlanner(MethodType):
    """ Plans to use a tool. """

    @abc.abstractmethod
    def plan(self):
        # TODO
        pass

    def getType(self):
        return 'ToolUsePlanner'

    def hasResourceConflict(self, otherMethod):
        return False


class ToolUseController(MethodType):
    """ Executes a trajectory planned by a ToolUsePlanner. """

    @abc.abstractmethod
    def execute(self):
        # TODO
        pass

    def getType(self):
        return 'ToolUseController'

    def hasResourceConflict(self, otherMethod):
        return otherMethod.getType() == 'COMController' \
               or otherMethod.getType() == 'ToolUseController' \
               or otherMethod.getType() == 'GraspController' \
               or otherMethod.getType() == 'PlaceController'


class GraspResult(object):

    def __init__(self, grasp_position, approach_vector, orientation):
        self.grasp_position = grasp_position
        self.approach_vector = approach_vector
        self.orientation = orientation


class Waypoint(object):
    """ A waypoint on a trajectory. """
    def __init__(self, timestamp, positions, velocities, accelerations):
        """ Creates a new waypoint.
            @param timeStamp - a tuple (sec, nanosec) that denotes the timestamp of this waypoint.
            @param position - the joint positions (n-tuple) where the robot is at time timestamp
            @param velocities - the joint velocities (n-tuple)the robot has at time timestamp
            @param accelerations - the joint accelerations(n-tuple) the robot has at time timestamp
        """
        self.timestamp = timestamp
        self.positions = positions
        self.velocities = velocities
        self.accelerations = accelerations


class Trajectory(object):
    """ A robot joint space trajectory. """
    def __init__(self, group_name, joint_names, waypoints=None):
        """ Creates a new robot joint space trajectory.
            @param group_name - the name of the move group as defined in the urdf
            @param joint_names - the names of the joints involved.
            @param waypoints - a list of waypoints that make up the trajectory.
        """
        # TODO the group_name is a bit MoveIt specific (it is defined in the MoveIt srdf)
        self.group_name = group_name
        self.joint_names = joint_names
        if waypoints is None:
            self.waypoints = []
        else:
            self.waypoints = waypoints

    def getPathLength(self):
        """ Returns the path length (covered distance in configuration space) of this trajectory """
        if len(self.waypoints) <= 1:
            return 0.0
        totalDist = 0.0
        prevWp = self.waypoints[0]
        for wp in self.waypoints:
            deltaWp = map(lambda x, y: y - x, prevWp.positions, wp.positions)
            distWp = reduce(lambda x, y: x + y * y, deltaWp)
            totalDist += distWp
        return totalDist

    def appendWaypoint(self, waypoint):
        """ Appends the given waypoint to the trajectory.
            Note that there is sanity check performed to ensure the timestamp makes sense.
        """
        self.waypoints.append(waypoint)
