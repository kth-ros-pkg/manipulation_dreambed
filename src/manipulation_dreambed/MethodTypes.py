"""
    This file is part of the RobDREAM (http://robdream.eu/) deliverable D4.2.
    It contains the abstract interfaces of each method type as defined in deliverable D4.1.

    @author: Joshua Haustein (haustein@kth.se)
"""
import abc


class PortfolioMethod(object):
    """ Abstract super class of each method. """
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
                          it would return plannerParamA and plannerParamB in case role == ArmPlanner
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
        """ Initializes this method so that it is ready to use afterwards.
            Note that any resources that would prevent other method types from successfully
            executing should be acquired in @allocateResources."""
        pass

    @abc.abstractmethod
    def allocateResources(self, roles=None):
        """ Allocates all resources that are required to run this method but can interfere
            with other method types if loaded. This should be the same resources as they are
            released in @releaseResources.
            @param roles - see getParameters(). If None, all the resources for all roles are allocated.
        """
        pass

    @abc.abstractmethod
    def releaseResources(self, roles=None):
        """ Releases all resources that may interfere with other method types.
            This should be the same resources as the ones acquired in @allocateResources.
            @param roles - see getParameters(). If None, the resources for all roles are released, else only
                    for the roles specified.
        """
        pass

    @abc.abstractmethod
    def destroy(self):
        """ Notifies the method type to destroy any allocated resources/ terminate any subprocesses. """
        pass

    def getRoles(self):
        """ Returns a list of all roles implemented by this method. """
        parentClasses = [x.__name__ for x in self.__class__.__mro__]
        roles = [x for x in parentClasses if x in METHOD_TYPES_STRING]
        return roles

    @abc.abstractmethod
    def hasResourceConflict(self, activeRoles):
        """ Returns whether this method may have resource conflicts with any method in any of
            the given roles. """
        pass

    @abc.abstractmethod
    def supportsBatchProcessing(self):
        """ Returns whether this method supports batch processing. If a method implements multiple
            different method types (e.g. ArmPlanner and GraspPlanner) this method can be queried to
            process multiple planning/control requests at once rather than in sequence. For instance,
            if a method provides integrated grasp and motion planning, it is desirable to process
            the inputs for both roles at once rather than in sequence. This allows the method to tackle
            both problems at the same time instead of sequentially (i.e. plan the grasp such that it can
            move the arm to the location).
            If a method supports batch processing, it must support this for any sequence of its roles.
        """
        pass

    @abc.abstractmethod
    def executeBatch(self, startContext, batchInput, parameters):
        """ If this method supports batch processing, this function executes a whole batch of tasks.
            @param TODO startContext
            @param batchInput - a list of tuples (role_name, role_input), where role_name is the name of
            the role (e.g. ArmPlanner, GraspController), and role_input is a dictionary of inputs for
            that respective role. Each input is stored under the same key as the respective function
            argument.
            @param TODO parameters
            @return - an ordered list that contains for each task a tuple (success, result).
        """
        raise NotImplementedError('executeBatch-function of root class MethodType was called: You need to override this' + \
                                  ' function, if your method type supports batch processing.')


class ArmPlanner(object):
    """ Plans an arm trajectory to a specified goal. """
    __metaclass__ = abc.ABCMeta
    # @abc.abstractmethod
    # def preparePlanning(self, context):
    #     pass
    STRING_REPRESENTATION = 'ArmPlanner'

    @abc.abstractmethod
    def planArmTrajectory(self, goal, context, paramPrefix, parameters):
        """
            Plans an arm trajectory to the given goal.
            @param goal - can either be a configuration, a pose or a position
            @param context - the context
            @param paramPrefix - a string containing the prefix used for the parameters keys
            @param parameters - a dict containing the parameters for this method.
            @return Trajectory - The planned trajectory, None if no solution was found.
        """
        pass


class ArmController(object):
    """ Executes a given arm trajectory. """
    __metaclass__ = abc.ABCMeta
    STRING_REPRESENTATION = 'ArmController'

    @abc.abstractmethod
    def executeArmTrajectory(self, trajectory, context, paramPrefix, parameters):
        """
            Executes the given trajectory.
            @param trajectory - of type Trajectory
            @param context - the context
            @param paramPrefix - a string containing the prefix used for the parameters keys
            @param parameters - a dict containing the parameters for this method.
            @return boolean - True if success, else False
        """
        pass

    # def hasResourceConflict(self, otherMethod):
    #     return otherMethod.getType() == 'COMController' \
    #            or otherMethod.getType() == 'ToolUseController' \
    #            or otherMethod.getType() == 'GraspController' \
    #            or otherMethod.getType() == 'PlaceController'


class GraspPlanner(object):
    """ Plans a grasp for a given object. """
    __metaclass__ = abc.ABCMeta
    STRING_REPRESENTATION = 'GraspPlanner'

    @abc.abstractmethod
    def planGrasp(self, object, context, paramPrefix, parameters):
        """
            Plans a grasp for the given object.
            @param object - object information containing the pose and the name of the object.
            @param context - the context
            @param paramPrefix - a string containing the prefix used for the parameters keys
            @param parameters - a dict containing the parameters for this method.
            @return GraspResult - The planned grasp, None if failed
        """
        pass


class GraspController(object):
    """ Executes a grasp and controls the robot hand while an object is grasped. """
    __metaclass__ = abc.ABCMeta
    STRING_REPRESENTATION = 'GraspController'

    @abc.abstractmethod
    def startGraspExecution(self, grasp, context, paramPrefix, parameters):
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
    def stopGraspExecution(self):
        """
            Stops the execution of the grasp controller. Note, that the gripper should remain
            stiff after this call, as a grasped object may be dropped otherwise. The purpose of
            this function is to notify the grasp controller that another process is going to control
            the robot hand from now on.
            @return boolean - False in case of any severe error, else True
        """
        pass

    # def hasResourceConflict(self, otherMethod):
    #         return otherMethod.getType() == 'COMController' \
    #                or otherMethod.getType() == 'ToolUseController' \
    #                or otherMethod.getType() == 'GraspController' \
    #                or otherMethod.getType() == 'PlaceController'


class PlacePlanner(object):
    """ Plans to place an object. """
    __metaclass__ = abc.ABCMeta
    STRING_REPRESENTATION = 'PlacePlanner'

    @abc.abstractmethod
    def planPlacement(self, placementInfo, context, paramPrefix, parameters):
        # TODO
        pass


class PlaceController(object):
    """ Executes the placement of an object. """
    __metaclass__ = abc.ABCMeta
    STRING_REPRESENTATION = 'PlaceController'

    @abc.abstractmethod
    def executePlacement(self, trajectory, context, paramPrefix, parameters):
        # TODO
        pass

    # def hasResourceConflict(self, otherMethod):
    #     return otherMethod.getType() == 'COMController' \
    #            or otherMethod.getType() == 'ToolUseController' \
    #            or otherMethod.getType() == 'GraspController' \
    #            or otherMethod.getType() == 'PlaceController'


class COMPlanner(object):
    """ Plans to manipulate a constrained object (e.g. a door handle). """
    __metaclass__ = abc.ABCMeta
    STRING_REPRESENTATION = 'COMPlanner'

    @abc.abstractmethod
    def planCOM(self):
        # TODO
        pass


class COMController(object):
    """ Executes the manipulation planned by a COMPlanner. """
    __metaclass__ = abc.ABCMeta
    STRING_REPRESENTATION = 'COMController'

    @abc.abstractmethod
    def executeCOM(self):
        # TODO
        pass

    # def hasResourceConflict(self, otherMethod):
    #     return otherMethod.getType() == 'COMController' \
    #            or otherMethod.getType() == 'ToolUseController' \
    #            or otherMethod.getType() == 'GraspController' \
    #            or otherMethod.getType() == 'PlaceController'


class ToolUsePlanner(object):
    """ Plans to use a tool. """
    __metaclass__ = abc.ABCMeta
    STRING_REPRESENTATION = 'ToolUsePlanner'

    @abc.abstractmethod
    def planToolUse(self):
        # TODO
        pass


class ToolUseController(object):
    """ Executes a trajectory planned by a ToolUsePlanner. """
    __metaclass__ = abc.ABCMeta
    STRING_REPRESENTATION = 'ToolUseController'

    @abc.abstractmethod
    def executeToolUse(self):
        # TODO
        pass

    # def hasResourceConflict(self, otherMethod):
    #     return otherMethod.getType() == 'COMController' \
    #            or otherMethod.getType() == 'ToolUseController' \
    #            or otherMethod.getType() == 'GraspController' \
    #            or otherMethod.getType() == 'PlaceController'


class GraspResult(object):

    def __init__(self, grasp_pose, approach_vector, hand_configuration, grasped_object=None):
        self.grasp_pose = grasp_pose
        self.approach_vector = approach_vector
        self.hand_configuration = hand_configuration
        self.grasped_object = grasped_object


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

    def getTotalDuration(self):
        """ Returns the total duration of this trajectory"""
        return self.waypoints[-1].timestamp[0] + self.waypoints[-1].timestamp[1] * 10e9

    def appendWaypoint(self, waypoint):
        """ Appends the given waypoint to the trajectory.
            Note that there is no sanity check performed to ensure the timestamp makes sense.
        """
        self.waypoints.append(waypoint)

METHOD_TYPES_STRING = [ArmPlanner.STRING_REPRESENTATION,
                       ArmController.STRING_REPRESENTATION,
                       GraspPlanner.STRING_REPRESENTATION,
                       GraspController.STRING_REPRESENTATION,
                       PlacePlanner.STRING_REPRESENTATION,
                       PlaceController.STRING_REPRESENTATION,
                       COMPlanner.STRING_REPRESENTATION,
                       COMController.STRING_REPRESENTATION,
                       ToolUsePlanner.STRING_REPRESENTATION,
                       ToolUseController.STRING_REPRESENTATION]
