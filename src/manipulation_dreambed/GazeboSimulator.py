"""
    This file is part of the RobDREAM (http://robdream.eu/) deliverable D4.2.
    It contains an implementation of the abstract robot simulator interface defined in ManipulationDreamBed.py.
    As underlying simulator we utilize the open source robot simulator gazebo (http://gazebosim.org/).

    @author: Joshua Haustein (haustein@kth.se)
"""
from manipulation_dreambed import Simulator
from std_srvs.srv import Empty as EmptyService
from std_msgs.msg import Empty as EmptyMessage
from std_msgs.msg import Bool as BoolMessage
from gazebo_msgs.srv import SpawnModel, DeleteModel, GetWorldProperties, GetModelState, SetModelState
from gazebo_msgs.srv import GetJointProperties, GetModelProperties, SetModelConfiguration, JointRequest, BodyRequest
from gazebo_msgs.msg import ModelState
import rospy
import roslaunch
import ROSUtils
import yaml
import abc
import time


class RobotSimulator(object):
    """ Abstract super class of a robot simulator. """
    __metaclass__ = abc.ABCMeta

    @abc.abstractmethod
    def init(self):
        """ Blocks the execution until the robot simulator is ready to use. """
        return False

    @abc.abstractmethod
    def reset(self):
        """ Resets the robot simulator to its initial state. """
        pass

    @abc.abstractmethod
    def cleanUp(self):
        """ Clean up when the program is stopped. """
        pass

class ROSRobotSimulator(RobotSimulator):
    """ An implementation of the robot simulator interface that sends the issued
        commands through ROS services to a remote ROS node"""
    def __init__(self, rosServicePrefix='/robot_simulator/'):
        self._servicePrefix = rosServicePrefix
        self._resetRobotService = None
        self._cleanUpService = None
        self._initService = None

    #TODO implement the interface and call respective services


class GazeboSimulatorWrapper(Simulator):
    # TODO: do we need to write a gazebo plugin for this? -> measure time + maybe something else
    def __init__(self, modelPathFile, robotSimulator):
        self.bPaused = False
        # Gazebo services
        self._unpauseService = None
        self._pauseService = None
        self._clearJointForcesService = None
        self._clearBodyWrenchesService = None
        self._resetWorldService = None
        self._resetSimulationService = None
        self._getWorldPropertiesService = None
        self._spawnModelService = None
        self._removeModelService = None
        self._setModelStateService = None
        self._getModelStateService = None
        self._getJointPropertiesService = None
        self._getModelPropertiesService = None
        self._setModelConfigurationService = None
        self._robotSimulator = robotSimulator

        # self.modelRootPath = modelPath
        # if self.modelRootPath[-1] != '/':
        #     self.modelRootPath.append('/')
        f = open(modelPathFile, 'r')
        fileContent = f.read()
        if fileContent == "":
            raise IOError('The model path file is empty: %s' % modelPathFile)
        self.modelPaths = yaml.load(fileContent)
        self.bInit = False

    def init(self):
        rospy.loginfo('GazeboSimulatorWrapper: Initializing...')
        rospy.wait_for_service('/gazebo/pause_physics')
        self._pauseService = rospy.ServiceProxy('/gazebo/pause_physics', EmptyService)
        rospy.wait_for_service('/gazebo/unpause_physics')
        self._unpauseService = rospy.ServiceProxy('/gazebo/unpause_physics', EmptyService)
        rospy.wait_for_service('/gazebo/reset_world')
        self._resetWorldService = rospy.ServiceProxy('/gazebo/reset_world', EmptyService)
        rospy.wait_for_service('/gazebo/reset_simulation')
        self._resetSimulationService = rospy.ServiceProxy('/gazebo/reset_simulation', EmptyService)
        rospy.wait_for_service('/gazebo/clear_joint_forces')
        self._clearJointForcesService = rospy.ServiceProxy('/gazebo/clear_joint_forces', JointRequest)
        rospy.wait_for_service('/gazebo/clear_body_wrenches')
        self._clearBodyWrenchesService = rospy.ServiceProxy('/gazebo/clear_body_wrenches', BodyRequest)
        rospy.wait_for_service('/gazebo/get_world_properties')
        self._getWorldPropertiesService = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)
        rospy.wait_for_service('/gazebo/spawn_sdf_model')
        self._spawnModelService = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        rospy.wait_for_service('/gazebo/delete_model')
        self._removeModelService = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        rospy.wait_for_service('/gazebo/get_model_state')
        self._getModelStateService = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        rospy.wait_for_service('/gazebo/set_model_state')
        self._setModelStateService = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        rospy.wait_for_service('/gazebo/get_joint_properties')
        self._getJointPropertiesService = rospy.ServiceProxy('/gazebo/get_joint_properties', GetJointProperties)
        rospy.wait_for_service('/gazebo/get_model_properties')
        self._getModelPropertiesService = rospy.ServiceProxy('/gazebo/get_model_properties', GetModelProperties)
        rospy.wait_for_service('/gazebo/set_model_configuration')
        self._setModelConfigurationService = rospy.ServiceProxy('/gazebo/set_model_configuration', SetModelConfiguration)
        self.bInit = self._robotSimulator.init()
        if not self.bInit:
            raise RuntimeError('Could not initialize Gazebo wrapper.')
        rospy.loginfo('GazeboSimulatorWrapper: ...initialization complete.')

    def isInitialized(self):
        return self.bInit

    def setWorldState(self, sceneInfo):
        def setStuff():
            self.reset()
            self._synchModels(sceneInfo)
            # set object poses
            for obj in sceneInfo.objects:
                self._sendModelState(obj, sceneInfo.frameName)
            # and robot pose
            self._sendModelState(sceneInfo.robot, sceneInfo.frameName)
            # set robot configuration
            self._sendRobotConfiguration(sceneInfo.robot)
        self._executePaused(setStuff, True)

    def getWorldState(self, sceneInfo):
        def getStuff():
            self._synchModels(sceneInfo, scene2Gazebo=False)
            # update object poses
            for obj in sceneInfo.objects:
                self._getModelState(obj)
            # update robot pose
            self._getModelState(sceneInfo.robot)
            # update robot configuration
            self._receiveRobotConfiguration(sceneInfo.robot)
        self._executePaused(getStuff, True)

    def _sendRobotConfiguration(self, robot):
        config = robot.configuration
        joints = config.keys()
        jointValues = config.values()
        # first reset joint values:
        for jointName in joints:
            self._clearJointForcesService(jointName)
        print joints, jointValues
        result = self._setModelConfigurationService(model_name=robot.name, joint_names=joints, joint_positions=jointValues)
        if not result.success:
            raise RuntimeError('Could not send robot configuration. Message:%s' % result.status_message)

    def _receiveRobotConfiguration(self, robot):
        properties = self._getModelPropertiesService(robot.name)
        if not properties.success:
            raise RuntimeError('Could not receive robot information from gazebo. Message:%s' % properties.status_message)
        for joint in properties.joint_names:
            jp = self._getJointPropertiesService(joint)
            if not jp.success:
                raise RuntimeError('Could not receive joint information for joint %s from gazebo. Message:%s' %
                                   (joint, jp.status_message))
            robot.configuration[joint] = jp.position[0]

    def _sendModelState(self, modelInfo, frameName):
        # first reset all wrenches
        self._clearBodyWrenchesService(modelInfo.name)
        mstate = ModelState(pose=ROSUtils.contextPoseToROSPose(modelInfo.pose),
                            model_name=modelInfo.name, reference_frame=frameName)
        result = self._setModelStateService(mstate)
        if not result.success:
            raise RuntimeError('Could not set the pose of model %s. The error message is: %s' %
                               (modelInfo.name, result.status_message))

    def _getModelState(self, modelInfo):
        mstate = self._getModelStateService(model_name=modelInfo.name)
        if not mstate.success:
            raise RuntimeError('Could not receive model state of model %s. The error message is: %s' %
                               (modelInfo.name, mstate.status_message))
        modelInfo.pose = ROSUtils.rosPoseToContextPose(mstate.pose)

    def _synchModels(self, sceneInfo, scene2Gazebo=True):
        """
            Adds/removes models to/from gazebo if they are/are not specified in the sceneInfo,
            but are not/are in gazebo.
            If scene2Gazebo is False, the opposite is done, i.e. models are added/removed
            from the sceneInfo according to what is in gazebo.
        """
        worldProps = self._getWorldPropertiesService()
        if not worldProps.success:
            raise RuntimeError('Could not retrieve world properties from gazebo!')
        sceneModels = sceneInfo.getObjectNames()
        # TODO: for now we assume gazebo always contains the robot
        if sceneInfo.getRobotName() not in worldProps.model_names:
            raise RuntimeError('Robot is not in gazebo, adding robot is not supported yet.')
        worldProps.model_names.remove(sceneInfo.getRobotName())
        # compute diff between gazebo and scene info
        missingModelsInGazebo = [x for x in sceneModels if x not in worldProps.model_names]
        additionalModelsInGazebo = [x for x in worldProps.model_names if x not in sceneModels]

        if scene2Gazebo:
            for missingModel in missingModelsInGazebo:
                success = self._loadModel(missingModel, sceneInfo)
                if not success:
                    raise RuntimeError('Could not load model %s.' % missingModel)
            for additionalModel in additionalModelsInGazebo:
                success = self._removeModel(additionalModel)
                if not success:
                    raise RuntimeError('Could not remove model %s.' % additionalModel)
        else:
            for missingModel in additionalModelsInGazebo:
                sceneInfo.addObjectInfo(missingModel)
            for additionalModel in missingModelsInGazebo:
                sceneInfo.removeObjectInfo(additionalModel)

    def _loadModel(self, missingModel, sceneInfo):
        objectInfo = sceneInfo.getObjectInfo(missingModel)
        if missingModel not in self.modelPaths:
            raise RuntimeError('There is no gazebo model file specified for model %s' % missingModel)
        # modelPath = self.modelRootPath + self.modelPaths[missingModel]
        modelPath = ROSUtils.resolvePath(self.modelPaths[missingModel])
        f = open(modelPath, 'r')
        model_xml = f.read()
        if model_xml == "":
            f.close()
            raise IOError('The given model %s is empty.' % modelPath)
        pose = ROSUtils.contextPoseToROSPose(objectInfo.pose)
        result = self._spawnModelService(model_name=missingModel, model_xml=model_xml, robot_namespace=rospy.get_namespace(),
                                         initial_pose=pose, reference_frame=sceneInfo.frameName)
        f.close()
        return result.success

    def _removeModel(self, additionalModel):
        result = self._removeModelService(additionalModel)
        return result.success

    def reset(self):
        # self._resetWorldService()
        # self._resetSimulationService()
        self._executePaused(self._robotSimulator.reset, False)

    def startTimer(self):
        # TODO
        pass

    def stopTimer(self):
        # TODO
        return 0.0

    def pause(self, bPause):
        """ Pauses the simulation. WARNING: This pauses ROS time! That means all
            ROS time functions stop working until the simulation is unpaused again! """
        # if not self.bPaused and bPause:
        if bPause:
            rospy.loginfo('Pausing simulator.')
            # pause the simulator
            self._pauseService()
            self.bPaused = True
        # elif self.bPaused and not bPause:
        else:
            # unpause the simulator
            rospy.loginfo('Unpausing simulator.')
            self._unpauseService()
            self.bPaused = False

    def terminate(self):
        self._robotSimulator.cleanUp()

    def _executePaused(self, function, paused=True):
        """ Executes the given function either while the simulator is paused or unpaused.
            After the execution, the simulator is set to pause if it was paused before
            and vice versa.
            @param function The function to execute.
            @param paused   If true, function is executed while the simulator is paused,
            else while unpaused.
        """
        wasPaused = self.bPaused
        self.pause(paused)
        function()
        self.pause(wasPaused)


if __name__ == "__main__":
    import IPython
    import rospy

    rospy.init_node('GazeboSimulatorWrapperTest')
    simulator = GazeboSimulatorWrapper()
    simulator.init()
    IPython.embed()
