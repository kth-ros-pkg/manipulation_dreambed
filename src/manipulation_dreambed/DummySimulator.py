"""
    This file is part of the RobDREAM (http://robdream.eu/) deliverable D4.3.
    It contains a dummy implementation of the simulator interface to allow evaluation
    of planning algorithms in the manipulation dreambed .

    @author: Joshua Haustein (haustein@kth.se)
"""
from ManipulationDreamBed import Simulator
from MethodTypes import ArmController, GraspController, PortfolioMethod
import copy
import time
import sys

THIS = sys.modules[__name__]
THIS.SIMULATOR_INSTANCE = None
THIS.CONTROLLER_INSTANCE = None


def getSimulatorInstance(logger=None):
    if THIS.SIMULATOR_INSTANCE is None:
        THIS.SIMULATOR_INSTANCE = DummySimulator(logger)
    return THIS.SIMULATOR_INSTANCE


def getControllerInstance():
    if THIS.CONTROLLER_INSTANCE is None:
        simulator = THIS.getSimulatorInstance()
        THIS.CONTROLLER_INSTANCE = DummyController(simulator)
    return THIS.CONTROLLER_INSTANCE


class DummySimulator(Simulator):

    def __init__(self, logger):
        self._last_set_world_state = None
        self._current_world_state = None
        self._timer_start = None
        self._b_paused = False
        self._logger = logger

    @staticmethod

    def init(self):
        pass

    def isInitialized(self):
        return True

    def isGrasped(self, object_name):
        ri = self._current_world_state.getRobotInfo()
        return ri.grasped_object == object_name

    def setWorldState(self, si):
        self._last_set_world_state = copy.deepcopy(si)
        self._current_world_state = copy.deepcopy(si)

    def getWorldState(self, si):
        si.copy(self._current_world_state)

    def reset(self):
        self._current_world_state = copy.deepcopy(self._last_set_world_state)

    def startTimer(self):
        self._timer_start = time.time()

    def stopTimer(self):
        if self._timer_start is not None:
            elapsed_time = time.time() - self._timer_start
            self._timer_start = None
            return elapsed_time
        return 0.0

    def pause(self, bPause):
        self._b_paused = bPause

    def terminate(self):
        pass

    def getLogger(self):
        return self._logger


class DummyController(PortfolioMethod, ArmController, GraspController):
    """
        Implements a dummy controller operating on the dummy simulator.
        It essentially just sets the robot and environment state to the
        planned outcome of the respective action.
    """
    # TODO interface also other controllers
    def __init__(self, simulator):
        self._dummy_simulator = simulator
        self._logger = simulator.getLogger()

    def executeBatch(self, startContext, batchInput, parameters):
        pass

    def supportsBatchProcessing(self):
        return False

    def getForbiddenConfigurations(self, role, paramPrefix):
        return []

    def allocateResources(self, roles=None):
        pass

    def getParameters(self, role, paramPrefix):
        return {}

    def destroy(self):
        pass

    def getConditionals(self, role, paramPrefix):
        return {}

    def initialize(self):
        pass

    def releaseResources(self, roles=None):
        pass

    def hasResourceConflict(self, activeRoles):
        return False

    def getName(self):
        return 'DummyController'

    def setRobotConfiguration(self, configuration):
        world_state = self._dummy_simulator._current_world_state
        ri = world_state.getRobotInfo()
        for name, value in configuration.iteritems():
            ri.configuration[name] = value

    def executeArmTrajectory(self, trajectory, context, paramPrefix, parameters):
        last_wp = trajectory.waypoints[-1]
        last_config = dict(zip(trajectory.joint_names, last_wp.positions))
        self._logger.loginfo('[DummyController::executeArmTrajectory] Setting arm configuration to %s' %
                             str(last_config))
        self.setRobotConfiguration(last_config)
        # TODO we should move grabbed objects here, but for that we would need
        # TODO the end-effector pose; or the grabbed object pose would need to be in eef frame
        return True

    def startGraspExecution(self, grasp, context, paramPrefix, parameters):
        self.setRobotConfiguration(grasp.hand_configuration)
        self._logger.loginfo('[DummyController::startGraspExecution] Setting hand configuration to %s' %
                             str(grasp.hand_configuration))
        robot_info = self._dummy_simulator._current_world_state.getRobotInfo()
        robot_info.grasped_object = grasp.grasped_object
        return True

    def stopGraspExecution(self):
        return True

