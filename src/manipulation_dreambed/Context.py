"""
    This file is part of the RobDREAM (http://robdream.eu/) deliverable D4.2.
    It contains an implementation of the context for manipulation tasks.

    @author: Joshua Haustein (haustein@kth.se)
"""
import yaml
import numpy
import copy
from ActionPrimitives import (Task, GraspReference, MoveArmAction, GraspAction, PlaceAction)


def numpyArray_representer(dumper, data):
    """ Produces a yaml node representing a numpy array """
    return dumper.represent_scalar(u'!ndarray', u'%s' % data.tolist())


def numpyArray_constructor(loader, node):
    """ Creates a numpy array from its yaml description """
    value = loader.construct_scalar(node)
    return numpy.array(map(float, value.replace(']', '').replace('[', '').split(',')))


def addNumpyYaml():
    yaml.add_representer(numpy.ndarray, numpyArray_representer)
    yaml.add_constructor(u'!ndarray', numpyArray_constructor)


class ConfigurationWrapper(yaml.YAMLObject):
    yaml_tag = u'!ConfigurationWrapper'

    def __init__(self, configuration):
        """
            Creates a wrapper for a robot configuration.
            @param configuration - a dict where the keys are joint names and the values joint values
        """
        self.configuration = configuration

    def __repr__(self):
        return "%s(configuration=%r)" % (self.__class__.__name__, self.configuration)


class PositionWrapper(yaml.YAMLObject):
    yaml_tag = u'!PositionWrapper'

    def __init__(self, position):
        """
            Creates a wrapper for a position.
            @param position - a numpy array [x,y,z].
        """
        self.position = position

    def __repr__(self):
        return "%s(position=%r)" % (self.__class__.__name__, self.position)


class Pose(yaml.YAMLObject):
    yaml_tag = u'!Pose'

    def __init__(self, position=numpy.array([0.0, 0.0, 0.0]),
                 orientation=numpy.array([0.0, 0.0, 0.0, 1.0]),
                 frame='world'):
        self.position = position
        self.orientation = orientation
        self.frame = frame

    def __repr__(self):
        return "%s(position=%r, orientation=%r, frame=%r)" % (self.__class__.__name__, self.position, self.orientation, self.frame)


class RobotInformation(yaml.YAMLObject):
    """ Stores information about the robot. """
    yaml_tag = u'!Robot'

    def __init__(self, name, basePose, configuration, grasped_object):
        self.name = name
        # self.modelFile = modelFile
        self.pose = basePose
        self.configuration = configuration
        self.grasped_object = grasped_object
        if grasped_object == 'None':
            self.grasped_object = None

    def __repr__(self):
        return "%s(name=%s, pose=%r, configuration=%s, grasped_object=%s)" % (
            self.__class__.__name__, self.name, self.pose, self.configuration, self.grasped_object)


class ObjectInformation(yaml.YAMLObject):
    """ Stores information about an object in the scene. """
    yaml_tag = u'!Object'

    def __init__(self, name, pose, object_class=None):
        self.name = name
        if object_class is None:
            object_class = name
        self.object_class = object_class
        # self.modelFile = modelFile
        self.pose = pose

    def __repr__(self):
        return "%s(name=%s, pose=%s)" % (
            self.__class__.__name__, self.name, self.pose)
    # TODO: make attachable to robot! -> pose relative to link in that case


class SceneInformation(object):
    """ Stores information about the scene"""
    def __init__(self, sceneDescriptionFile):
        self.robot = None
        self.objects = []
        self.frameName = None
        self._loadSceneInformation(sceneDescriptionFile)

    def getRobotInfo(self):
        return self.robot

    def getObjectNames(self):
        return map(lambda x: x.name, self.objects)

    def getObjectInfo(self, name):
        for obj in self.objects:
            if obj.name == name:
                return obj
        return None

    def addObjectInfo(self, name, pose=None):
        if pose is None:
            pose = Pose()
        self.objects.append(ObjectInformation(name, pose))

    def getObjects(self):
        return self.objects

    def removeObjectInfo(self, name):
        toRemove = None
        for obj in self.objects:
            if obj.name == name:
                toRemove = obj
                break
        if toRemove is not None:
            self.objects.remove(toRemove)

    def getRobotName(self):
        return self.robot.name

    def copy(self, scene_info):
        self.robot = copy.deepcopy(scene_info.getRobotInfo())
        self.objects = copy.deepcopy(scene_info.getObjects())
        self.frameName = scene_info.frameName

    def _loadSceneInformation(self, sceneDescriptionFile):
        stream = file(sceneDescriptionFile, 'r')
        addNumpyYaml()
        sceneDescription = yaml.load(stream)
        if 'robot' not in sceneDescription:
            raise IOError('The given scene description file does not contain a robot.')
        else:
            self.robot = sceneDescription['robot']
        if 'worldFrame' not in sceneDescription:
            raise IOError('The given scene description file does not a world frame name.')
        else:
            self.frameName = sceneDescription['worldFrame']
        if 'objects' in sceneDescription:
            self.objects = sceneDescription['objects']

        # for entity in allEntities:
        #     if isinstance(entity, RobotInformation):
        #         if self.robot is not None:
        #             print 'Warning: Multiple robots declared. Newest defintion.'
        #         self.robot = entity
        #     elif isinstance(entity, ObjectInformation):
        #         self.objects.append(entity)
        #     elif isinstance(entity, str):


class Context(object):
    """ The context as we use it in workpackage 4. This class may be replaced
        by a more general class that serves the requirements for all workpackages.
    """
    def __init__(self):
        self.task = None
        self.sceneInfo = None

    def setTask(self, task):
        self.task = task

    def getTask(self):
        return self.task

    def readSceneDescription(self, filename):
        self.sceneInfo = SceneInformation(filename)

    def readTask(self, filename):
        f = open(filename, 'r')
        content = f.read()
        if content == "":
            raise IOError('The given task file %s is empty.' % filename)
        self.task = yaml.load(content)

    def getSceneInformation(self):
        return self.sceneInfo

    def getRobotInformation(self):
        return self.sceneInfo.robot

if __name__ == '__main__':
    pose = Pose(numpy.array([1.0, 2.0, 1.0]), numpy.array([0.0, 1.0, 1.1, 1.0]))
    config = {'j0':0.1, 'j1': 1.0}
    ri = RobotInformation('C3P0', pose, config)
    # print ri
    yaml.add_representer(numpy.ndarray, numpyArray_representer)
    yaml.add_constructor(u'!ndarray', numpyArray_constructor)
    import IPython
    IPython.embed()
