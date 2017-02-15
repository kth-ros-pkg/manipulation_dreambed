"""
    This file is part of the RobDREAM (http://robdream.eu/) deliverable D4.2.
    It contains the implementation action primitives in the context of manipulation tasks
    as well as a general implementation of a task class.

    @author: Joshua Haustein (haustein@kth.se)
"""
import yaml


class Task(yaml.YAMLObject):
    """ A task contains a list of action primitives. """
    yaml_tag = u'!Task'

    def __init__(self):
        self.actions = []

    def addAction(self, action):
        self.actions.append(action)

    def __repr__(self):
        return "%s(actions=%s)" % (self.__class__.__name__, self.actions)


class GraspReference(yaml.YAMLObject):
    yaml_tag = u'!GraspReference'

    def __init__(self, graspActionName):
        self.graspActionName = graspActionName

    def __repr__(self):
        return "%s(graspActionName=%s)" % (self.__class__.__name__, self.graspActionName)


class MoveArmAction(object):
    # yaml_tag = u'!MoveArmAction'

    def __init__(self, name, goal, optional=False, parameterGroup=None, constraints=None):
        self.name = name
        self.goal = goal
        if constraints is None:
            constraints = {}
        self.constraints = constraints
        self.optional = optional
        self.parameterGroup = parameterGroup

    def getInputs(self):
        return {'goal': self.goal, 'constraints': self.constraints}

    def isOptional(self):
        return self.optional

    def getParameterPrefix(self):
        if self.parameterGroup is None:
            return self.name
        return self.parameterGroup

    def __repr__(self):
        return "%s(name=%s, goal=%s, optional=%s, paramaterGroup=%s, constraints=%s)" % (self.__class__.__name__, self.name,
                                                         self.goal, self.optional,
                                                         self.parameterGroup, self.constraints)


class GraspAction(object):
    # yaml_tag = u'!GraspAction'

    def __init__(self, name, objectName, optional=False, parameterGroup=None, constraints=None):
        self.name = name
        self.objectName = objectName
        self.optional = optional
        if constraints is None:
            constraints = {}
        self.constraints = constraints
        self.parameterGroup = parameterGroup

    def getInputs(self):
        return {'objectName': self.objectName, 'constraints': self.constraints}

    def isOptional(self):
        return self.optional

    def getParameterPrefix(self):
        if self.parameterGroup is None:
            return self.name
        return self.parameterGroup

    def __repr__(self):
        return "%s(name=%s, objectName=%s, optional=%s, paramaterGroup=%s, constraints=%s)" % (self.__class__.__name__, self.name,
                                                               self.objectName, self.optional,
                                                               self.parameterGroup, self.constraints)


class PlaceAction(object):
    # yaml_tag = u'!PlaceAction'

    def __init__(self, name, goal, optional=False, parameterGroup=None, constraints=None):
        self.name = name
        self.goal = goal
        self.optional = optional
        if constraints is None:
            constraints = {}
        self.constraints = constraints
        self.parameterGroup = parameterGroup

    def getInputs(self):
        return {'goal': self.goal, 'constraints': self.constraints}

    def isOptional(self):
        return self.optional

    def getParameterPrefix(self):
        if self.parameterGroup is None:
            return self.name
        return self.parameterGroup

    def __repr__(self):
        return "%s(name=%s, goal=%s, optional=%s, paramaterGroup=%s, constraints=%s)" % (self.__class__.__name__, self.name,
                                                         self.goal, self.optional,
                                                         self.parameterGroup, self.constraints)


class COMAction(object):
    # yaml_tag = u'!COMAction'

    def __init__(self, name, objectName, goal, optional=False, parameterGroup=None, constraints=None):
        self.name = name
        self.objectName = objectName
        self.optional = optional
        self.goal = goal
        if constraints is None:
            constraints = {}
        self.constraints = constraints
        self.parameterGroup = parameterGroup

    def isOptional(self):
        return self.optional

    def getParameterPrefix(self):
        if self.parameterGroup is None:
            return self.name
        return self.parameterGroup

    def getInputs(self):
        return {'goal': self.goal, 'objectName': self.objectName, 'constraints': self.constraints}


class ToolUseAction(object):
    # yaml_tag = u'!ToolUseAction'

    def __init__(self, name, toolName, toolTargetName, optional=False, parameterGroup=None, constraints=None):
        self.name = name
        self.toolName = toolName
        self.toolTargetName = toolTargetName
        self.optional = optional
        if constraints is None:
            constraints = {}
        self.constraints = constraints
        self.parameterGroup = parameterGroup

    def isOptional(self):
        return self.optional

    def getParameterPrefix(self):
        if self.parameterGroup is None:
            return self.name
        return self.parameterGroup

    def getInputs(self):
        return {'toolName': self.toolName, 'toolTargetName': self.toolTargetName, 'constraints': self.constraints}

def toolUseFactory(loader, node):
    fields = loader.construct_mapping(node)
    return ToolUseAction(**fields)

def comFactory(loader, node):
    fields = loader.construct_mapping(node)
    return COMAction(**fields)

def placeFactory(loader, node):
    fields = loader.construct_mapping(node)
    return PlaceAction(**fields)

def graspFactory(loader, node):
    fields = loader.construct_mapping(node)
    return GraspAction(**fields)

def moveArmFactory(loader, node):
    fields = loader.construct_mapping(node)
    return MoveArmAction(**fields)


yaml.add_constructor('!ToolUseAction', toolUseFactory)
yaml.add_constructor('!COMAction', comFactory)
yaml.add_constructor('!PlaceAction', placeFactory)
yaml.add_constructor('!GraspAction', graspFactory)
yaml.add_constructor('!MoveArmAction', moveArmFactory)
