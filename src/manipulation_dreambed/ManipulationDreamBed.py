"""
    This file is part of the RobDREAM (http://robdream.eu/) deliverable D4.2.
    It contains the implementation of the DreamBed for manipulation. The DreamBed provides an interface
    for the optimizer to select from a variety of manipulation methods for a given task.

    @author: Joshua Haustein (haustein@kth.se)
"""
import abc
import ROSUtils
import yaml
import sys
import importlib
import ActionPrimitives
import time
import copy
import math
from MethodTypes import (Trajectory, GraspResult)


class Logger(object):
    def loginfo(self, message):
        """Log message on info level """
        pass

    def logwarn(self, message):
        """Log message on warning level """
        pass

    def logerr(self, message):
        """Log message on error level """
        pass


class Simulator(object):
    """ Abstract interface of a robot simulator. """
    __metaclass__ = abc.ABCMeta

    @abc.abstractmethod
    def init(self):
        """ Initializes the simulator. This function may be computationally expensive/or
            blocking until a network connection is established.
            @returns True iff the simulator is ready for execution. """
        pass

    @abc.abstractmethod
    def isInitialized(self):
        """ Returns True iff the simulator is initialized and ready to use. """
        pass

    @abc.abstractmethod
    def setWorldState(self, si):
        """ Sets the world state of the simulator to represent the state specified in the given scene information. """
        pass

    @abc.abstractmethod
    def getWorldState(self, si):
        """ Updates the given scene information to represent the current state in the simulator. """
        pass

    @abc.abstractmethod
    def reset(self):
        """ Resets the world state to its initial state (that was specified in setWorldState(...)) """
        pass

    @abc.abstractmethod
    def startTimer(self):
        """ Starts a timer that measures the time passed in simulation. """
        pass

    @abc.abstractmethod
    def stopTimer(self):
        """ If a timer is running, stops the timer and returns the passed time in seconds. """
        pass

    @abc.abstractmethod
    def pause(self, bPause):
        """ Pauses the simulator if bPause is true, else unpauses."""
        pass

    @abc.abstractmethod
    def terminate(self):
        """ Notifies the simulator that the process is terminated. Can be used for cleanups."""
        pass
    # TODO ...


class PerformanceJudge(object):
    def reset(self):
        raise NotImplementedError('You need to implement a reset function!')

    def start(self):
        raise NotImplementedError('You need to implement a start function!')

    def stop(self):
        raise NotImplementedError('You need to implement a stop function!')

    def methodFailed(self, methodDesc, methodResult):
        """
            Called when a method failed in either planning or control.
            @param methodDesc - instance of class MethodDescription containing information about
                                the method that failed.
            @param methodResult - the result of the method. Its type depends on the method.
        """
        raise NotImplementedError('You need to implement a methodFailed function!')

    def methodSucceeded(self, methodDesc, methodResult):
        """
            Called when a method succeeded in either planning or control.
            @param methodDesc - instance of class MethodDescription containing information about
                                the method that claims to have succeeded.
            @param methodResult - the result of the method. Its type depends on the method.
        """
        raise NotImplementedError('You need to implement a methodSucceeded function!')

    def getPerformanceMeasure(self):
        raise NotImplementedError('You need to implement a getPerformanceMeasure function!')


class DefaultPerformanceJudge(PerformanceJudge):
    def __init__(self, simulator):
        self.simulator = simulator
        self.reset()

    def reset(self):
        self.startTime = 0.0
        self.runtime = 0.0
        self.pathLength = 0.0
        self.failed = False
        self.numOptionalFails = 0
        self.numSuccess = 0

    def start(self):
        self.startTime = time.time()

    def stop(self):
        self.runtime = time.time() - self.startTime

    def methodFailed(self, methodDesc, methodResult):
        self.failed = self.failed or not methodDesc.optional
        if methodDesc.optional:
            self.numOptionalFails += 1

    def methodSucceeded(self, methodDesc, methodResult):
        self.numSuccess += 1
        if methodDesc.type == "ArmPlanner":
            # methodResult should be of type Trajectory
            self.pathLength += methodResult.getPathLength()

    def getPerformanceMeasure(self):
        if self.failed or self.numSuccess == 0:
            return float('inf')
        return -(math.pow(self.numSuccess, 3) / (self.pathLength * self.runtime))
        # return self.numOptionalFails * self.optionalFailPenalty + self.pathLength + self.runtime / 60.0


class MethodDescription:
    def __init__(self, name, type, method_choice, inputs, finished,
                 result, isOptional, paramPrefix):
        self.name = name
        self.type = type
        self.method_choice = method_choice
        self.inputs = inputs
        self.finished = finished
        self.result = result
        self.optional = isOptional
        self.paramPrefix = paramPrefix

    def isPlanner(self):
        return "Planner" in self.type

    def isController(self):
        return not self.isPlanner()


class ManipulationDreamBed(object):
    def __init__(self, simulator, portfolioDescription, logFileName=None,
                 judge=None, logger=Logger()):
        """ Creates a new instance of a manipulation dreambed.
            @param simulator - the simulator to use (must implement the simulator interface)
            @param portfolioDescription - path to a file containing the portfolio description
            @param judge - an instance of a class that implements the PerformanceJudge interface;
                            if None, a default judge is used.
            @param logger - a logger to use for log messages.
        """
        self.simulator = simulator
        self.portfolioDescription = portfolioDescription
        self.bPortfolioLoaded = False
        self.bInit = False
        self.context = None
        self._runningGraspController = None  # TODO: this is not particularly nice to have
        self._allocatedMethods = []
        self.methodTypes = ('ArmController', 'ArmPlanner', 'COMController',
                            'COMPlanner', 'GraspPlanner', 'GraspController', 'PlaceController',
                            'PlacePlanner', 'ToolUsePlanner', 'ToolUseController')
        self.methodPortfolio = {}
        if judge is None:
            self._judge = DefaultPerformanceJudge(simulator)
        else:
            self._judge = judge
        self._logger = logger
        if logFileName is None:
            self._logFileName = ''
        else:
            self._logFileName = logFileName
        for mt in self.methodTypes:
            self.methodPortfolio[mt] = {}

    def init(self):
        """ Initializes the dream bed. Call this before evaluating any method! """
        if not self.simulator.isInitialized():
            self.simulator.init()
        if not self.bPortfolioLoaded:
            self._createPortfolio()
        self._initPortfolio()
        self.bInit = True

    def setContext(self, context):
        """ Sets the context to the given context. """
        if not self.bInit:
            self.init()
        print 'Setting context world state ...'
        self.simulator.setWorldState(context.getSceneInformation())
        print '... done'
        self.context = context
        self._checkTaskSolvable(context.task)

    def getParameters(self):
        parameters = {}
        conditionals = {}

        for action in self.context.getTask().actions:
            (plannerType, controllerType) = self._getMethodTypes(action)
            # Set parameter choices for method types for the given task
            planners = self.methodPortfolio[plannerType].keys()
            controllers = self.methodPortfolio[controllerType].keys()
            plannerParamPrefix = action.getParameterPrefix() + '_planner'
            controllerParamPrefix = action.getParameterPrefix() + '_controller'
            parameters[plannerParamPrefix] = ('categorical', planners, planners[0])
            parameters[controllerParamPrefix] = ('categorical', controllers, controllers[0])
            for planner in planners:
                self._extractParameters(methodType=plannerType, methodName=planner,
                                        methodParamPrefix=plannerParamPrefix,
                                        globalParameters=parameters,
                                        globalConditionals=conditionals)
            for controller in controllers:
                self._extractParameters(methodType=controllerType, methodName=controller,
                                        methodParamPrefix=controllerParamPrefix,
                                        globalParameters=parameters,
                                        globalConditionals=conditionals)

        return (parameters, conditionals.values(), [])

    def evaluate(self, **kwargs):
        # First copy the scene so we can modify it
        originalScene = self.context.getSceneInformation()
        modifiableScene = copy.deepcopy(originalScene)
        self.context.sceneInfo = modifiableScene

        # Now, build up a list of all methods we need to run
        methods = self._assembleMethodsList(kwargs)
        methods = self._resolveDependencies(methods)
        # Set the simulator up and let the judge know execution is starting.
        self.simulator.setWorldState(self.context.getSceneInformation())
        self._judge.reset()
        self._judge.start()
        # Initialize some variables
        numExecutedMethods = 0
        (success, result) = (True, None)
        # Run over task and execute methods with their respective parameters
        # and collect performance measures
        for method in methods:
            self._logger.logdebug('Running method ' + str(numExecutedMethods))
            # The current method might be a method that is duplicated due to dependencies.
            # In that case it has already been computed before and we can skip it.
            if method.finished:
                result = method.result
                success = result is not None
                self._logger.logdebug('Current method (must be a grasp planner) ' +
                                      'already executed before, skipping it.')
                continue

            # Execute the current method and pass it the result of the previous method
            (success, result) = self._executeMethod(method, kwargs, result)
            if not success:
                self._judge.methodFailed(method, result)
                msg = 'Current method ' + method.name + ' of type ' + method.type + ' failed.'
                self._logger.logwarn(msg)
                result = None
                # we can skip this method if it is optional
                if not method.optional:
                    self._logger.logwarn('The failed method was not optional, aborting execution.')
                    break
            else:
                self.simulator.getWorldState(self.context.getSceneInformation())
                self._judge.methodSucceeded(method, result)
            numExecutedMethods += 1

        self._judge.stop()
        self.context.sceneInfo = originalScene
        self._releaseResources()
        self.simulator.setWorldState(originalScene)
        objectiveValue = self._judge.getPerformanceMeasure()
        if len(self._logFileName) > 0:
            try:
                logFile = open(self._logFileName, "a")
                yaml.dump((kwargs, objectiveValue), logFile)
                logFile.close()
            except IOError as e:
                self._logger.logerr('Could not write information to optimization log file: ' + repr(e))
        return objectiveValue

    def destroy(self):
        self.simulator.terminate()
        self._destroyPortfolio()

    def _assembleMethodsList(self, kwargs):
        methods = []
        for action in self.context.task.actions:
            plannerName = action.name + '_planner'
            plannerKey = action.getParameterPrefix() + '_planner'
            controllerName = action.name + '_controller'
            controllerKey = action.getParameterPrefix() + '_controller'
            (plannerType, controllerType) = self._getMethodTypes(action)
            # save the planner method for this action
            methods.append(MethodDescription(name=plannerName,
                                             type=plannerType,
                                             method_choice=kwargs[plannerKey],
                                             inputs=action.getInputs(), finished=False,
                                             result=None, isOptional=action.isOptional(),
                                             paramPrefix=plannerKey))
            # save the controller method for this action
            methods.append(MethodDescription(name=controllerName,
                                             type=controllerType,
                                             method_choice=kwargs[controllerKey],
                                             inputs=action.getInputs(),
                                             finished=False,
                                             result=None,
                                             isOptional=action.isOptional(),
                                             paramPrefix=controllerKey))
        return methods

    def _createPortfolio(self):
        try:
            f = open(self.portfolioDescription, 'r')
            fileContent = f.read()
            if fileContent == "":
                f.close()
                raise IOError('The provided portfolio description is empty. (File: %s)' % self.portfolioDescription)
            portfolioInfo = yaml.load(fileContent)
            if 'paths' in portfolioInfo:
                self._logger.logwarn('Loading additional paths to the system path.' +
                              ' Please note that this is a potential security hazard.')
                paths = map(ROSUtils.resolvePath, portfolioInfo['paths'].split(';'))
                self._logger.logwarn('Adding paths %s to system path.' % str(paths))
                sys.path.extend(paths)
            if 'portfolio' not in portfolioInfo:
                f.close()
                raise RuntimeError('The given portfolio file does not contain any algorithm.')
            methodDescription = portfolioInfo['portfolio']
            for methodDesc in methodDescription:
                self._createMethod(methodDesc)
            self.bPortfolioLoaded = True
            f.close()
        except IOError as err:
            self._logger.logerr('Could not load portfolio')
            raise err

    def _createMethod(self, methodDesc):
        new_module = importlib.import_module(methodDesc['module'], methodDesc['package'])
        method_class = new_module.__getattribute__(methodDesc['class'])
        method_instance = method_class(methodDesc['parameters'])
        types = methodDesc['types'].split(';')
        self._logger.loginfo('ManipulationDreamBed: Created method %s' % method_instance.getName())
        for t in types:
            self.methodPortfolio[t].update([(method_instance.getName(), method_instance)])

    def _extractParameters(self, methodType, methodName, methodParamPrefix,
                           globalParameters, globalConditionals):
        paramsMethod = self.methodPortfolio[methodType][methodName].getParameters(role=methodType, paramPrefix=methodParamPrefix)
        for (paramName, paramDef) in paramsMethod.items():
            # first add the parameters to the normal parameter
            globalParameters[paramName] = paramDef
            # now make them condition on the choice of this method
            globalConditionals[paramName] = paramName + ' | ' + methodParamPrefix + ' == ' + methodName
        conditionalsMethod = self.methodPortfolio[methodType][methodName].getConditionals(role=methodType, paramPrefix=methodParamPrefix)
        for (paramName, condition) in conditionalsMethod.items():
            globalConditionals[paramName] = condition + ' && ' + methodParamPrefix + ' == ' + methodName

    def _initPortfolio(self):
        for t in self.methodPortfolio.keys():
            for m in self.methodPortfolio[t].values():
                m.initialize()

    def _getMethodTypes(self, action):
        if isinstance(action, ActionPrimitives.MoveArmAction):
            plannerType = "ArmPlanner"
            controllerType = "ArmController"
        elif isinstance(action, ActionPrimitives.GraspAction):
            plannerType = "GraspPlanner"
            controllerType = "GraspController"
        elif isinstance(action, ActionPrimitives.PlaceAction):
            plannerType = "PlacePlanner"
            controllerType = "PlaceController"
        else:
            raise NotImplementedError('%a - only MoveArm-, Grasp-, and PlaceAction are implemented yet.' % action)
        return (plannerType, controllerType)

    def _needsGrasp(self, currentMethod, methods):
        if currentMethod.type != 'ArmPlanner':
            return (False, 0)
        goal = currentMethod.inputs['goal']
        if isinstance(goal, ActionPrimitives.GraspReference):
            index = 0
            for m in methods:
                plannerName = goal.graspActionName + '_planner'
                if m.name == plannerName:
                    return (not m.finished, index)
                index += 1
            raise RuntimeError('There is no planner selected for the referenced grasp action %s.' %
                               goal.graspActionName)
        return (False, 0)

    def _executeMethod(self, currentMethodDesc, parameters, previousResult):
        self._logger.loginfo('Executing method ' + str(currentMethodDesc))
        # get the instance of the selected method
        method = self.methodPortfolio[currentMethodDesc.type][currentMethodDesc.method_choice]
        self._resolveResourceAllocationConflicts(method)
        inputs = currentMethodDesc.inputs
        # It is possible that the previous method failed and we do not have a previous result.
        # If this method is a controller it always needs a result, hence we can just abort if
        # we don't have a previous result.
        if currentMethodDesc.isController() and previousResult is None:
            return (False, None)

        # Else switch case method types
        if currentMethodDesc.type == 'ArmPlanner':
            solution = self._executeArmPlanner(planner=method, inputs=inputs,
                                               paramPrefix=currentMethodDesc.paramPrefix,
                                               parameters=parameters, graspResult=previousResult)
            success = solution is not None
        elif currentMethodDesc.type == 'ArmController':
            solution = None
            success = self._executeArmController(controller=method, inputs=inputs,
                                                 paramPrefix=currentMethodDesc.paramPrefix,
                                                 parameters=parameters, traj=previousResult)
        elif currentMethodDesc.type == 'GraspPlanner':
            solution = self._executeGraspPlanner(planner=method, inputs=inputs,
                                                 paramPrefix=currentMethodDesc.paramPrefix,
                                                 parameters=parameters)
            success = solution is not None
        elif currentMethodDesc.type == 'GraspController':
            solution = None
            if not isinstance(previousResult, GraspResult):
                raise ValueError('Attempting to execute a grasp controller, but no grasp given.')
            success = self._executeGraspController(controller=method, inputs=inputs,
                                                   paramPrefix=currentMethodDesc.paramPrefix,
                                                   parameters=parameters, graspResult=previousResult)

        currentMethodDesc.finished = True
        if solution is not None:
            currentMethodDesc.result = solution
        return (success, solution)

    def _executeArmPlanner(self, planner, inputs, paramPrefix, parameters, graspResult):
        # planner.preparePlanning(self.context)
        goal = inputs['goal']
        if isinstance(goal, ActionPrimitives.GraspReference):
            if graspResult is None or not isinstance(graspResult, GraspResult):
                # raise RuntimeError('Could not resolve goal for arm planner.')
                self._logger.logwarn('Could not resolve goal grasp for arm planner. Arm planner failed')
                return None
            # TODO add constraints (orientation and approach vector)
            goal = graspResult.position

        solution = planner.plan(goal=goal, context=self.context, paramPrefix=paramPrefix, parameters=parameters)
        return solution

    def _executeArmController(self, controller, inputs, paramPrefix, parameters, traj):
        if traj is None or not isinstance(traj, Trajectory):
            raise ValueError('Attempting to execute an arm controller, but no trajectory given, instead: %s' % traj)
        success = controller.execute(trajectory=traj, context=self.context,
                                     paramPrefix=paramPrefix, parameters=parameters)
        return success

    def _executeGraspPlanner(self, planner, inputs, paramPrefix, parameters):
        graspResult = planner.plan(object=inputs['objectName'], context=self.context, paramPrefix=paramPrefix,
                                   parameters=parameters)
        return graspResult

    def _executeGraspController(self, controller, inputs, paramPrefix, parameters, graspResult):
        self._stopGraspController()
        success = controller.startExecution(grasp=graspResult, context=self.context, paramPrefix=paramPrefix,
                                            parameters=parameters)
        if success:
            self._runningGraspController = controller
        return success

    def _stopGraspController(self):
        if self._runningGraspController is not None:
            success = self._runningGraspController.stopExecution()
            if not success:
                raise RuntimeError('Could not stop previously started grasp controller.')
            self._runningGraspController = None

    def _destroyPortfolio(self):
        for t in self.methodPortfolio.keys():
            for m in self.methodPortfolio[t].values():
                m.destroy()

    def _checkTaskSolvable(self, task):
        for action in task.actions:
            hasMethods = False
            (plannerType, controllerType) = self._getMethodTypes(action)
            hasMethods = len(self.methodPortfolio[plannerType]) > 0 and len(self.methodPortfolio[controllerType]) > 0
            if not hasMethods:
                raise ValueError('The action %s in the given task requires methods that are not available.' % action)

        return True

    def _resolveDependencies(self, methods):
        """ Adds a duplicate reference for each grasp planning method that an arm planning
            method depends on in front of the respective arm planning method."""
        extendedMethods = []
        for method in methods:
            (bNeedsGrasp, graspMethodIndex) = self._needsGrasp(method, methods)
            if bNeedsGrasp:
                extendedMethods.append(methods[graspMethodIndex])
            extendedMethods.append(method)
        return extendedMethods

    def _resolveResourceAllocationConflicts(self, newMethod):
        remainingAllocatedMethods = []
        for allocMeth in self._allocatedMethods:
            if allocMeth.hasResourceConflict(newMethod):
                allocMeth.releaseResources()
            else:
                remainingAllocatedMethods.append(allocMeth)
        newMethod.allocateResources()
        remainingAllocatedMethods.append(newMethod)
        self._allocatedMethods = remainingAllocatedMethods

    def _releaseResources(self):
        for ameth in self._allocatedMethods:
            ameth.releaseResources()
