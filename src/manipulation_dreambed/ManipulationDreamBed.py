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
from MethodTypes import (Trajectory, GraspResult, METHOD_TYPES_STRING, ArmPlanner,
                         ArmController, GraspPlanner, GraspController, PlacePlanner,
                         PlaceController)


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
        if methodDesc.type == ArmPlanner.STRING_REPRESENTATION:
            # methodResult should be of type Trajectory
            self.pathLength += methodResult.getPathLength()

    def getPerformanceMeasure(self):
        if self.failed or self.numSuccess == 0:
            return float('inf')
        return -(math.pow(self.numSuccess, 3) / (self.pathLength * self.runtime))
        # return self.numOptionalFails * self.optionalFailPenalty + self.pathLength + self.runtime / 60.0


class MethodDescriptionBatch:
    def __init__(self, methodName):
        self._descriptions = []
        self._types = []
        self._arguments = []
        self._results = []
        self._methodName = methodName

    def getMethodName(self):
        return self._methodName

    def getBatchSize(self):
        return len(self._descriptions)

    def addMethodDescription(self, methodDesc):
        self._descriptions.append(methodDesc)
        self._types.append(methodDesc.type)
        inputs = methodDesc.inputs.copy()
        inputs['paramPrefix'] = methodDesc.paramPrefix
        self._arguments.append(inputs)
        self._results.append((False, None))

    def getType(self, idx):
        return self._types[idx]

    def getArguments(self, idx):
        return self._arguments[idx]

    def setResult(self, idx, bSuccess, result):
        self._results[idx] = (bSuccess, result)

    def getMethodDescriptions(self):
        return self._descriptions

    def getFailedMethods(self):
        failedMethods = []
        for idx in range(len(self._results)):
            if not self._results[idx][0]:
                failedMethods.append(self._descriptions[idx])
        return failedMethods


class MethodDescription:
    def __init__(self, name, type, method_choice, inputs, finished,
                 result, isOptional, paramPrefix, supplyMethodDesc=None):
        self.name = name
        self.type = type
        self.method_choice = method_choice
        self.inputs = inputs
        self.finished = finished
        self.result = result
        self.optional = isOptional
        self.paramPrefix = paramPrefix
        self.supplyMethodDesc = supplyMethodDesc

    def isPlanner(self):
        return "Planner" in self.type

    def isController(self):
        return not self.isPlanner()

    # TODO???
    # def getInputs(self):
    #     if self.type == 'ArmPlanner':
    #         inputs = {'goal': self.inputs['goal']}
    #     solution = planner.plan(goal=goal, context=self.context, paramPrefix=paramPrefix, parameters=parameters)


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
        for mt in METHOD_TYPES_STRING:
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
        success = True
        # Run over task and execute methods with their respective parameters
        # and collect performance measures
        while numExecutedMethods < len(methods) and success:
            self._logger.logdebug('Running method ' + str(numExecutedMethods))
            batch = None # set batch to None so we don't execute an old batch again
            currentMethodDesc = methods[numExecutedMethods]
            currentMethod = self.methodPortfolio[currentMethodDesc.type][currentMethodDesc.method_choice]
            # Check whether we are using a method that supports batch processing
            if currentMethod.supportsBatchProcessing():
                # in this case create a batch and execute it as batch
                batch = self._assembleBatch(methods[numExecutedMethods:])
            if batch is not None:
                # Execute the current batch
                (success, result) = self._executeBatch(batch, kwargs)
                numExecutedMethods += batch.getBatchSize()
            else:
                # Execute the current method and pass it the result of the previous method
                (success, result) = self._executeMethod(currentMethodDesc, kwargs)
                numExecutedMethods += 1
            if not success:
                failedMethods = [currentMethodDesc]
                if batch is not None:
                    failedMethods = batch.getFailedMethods()
                for failedMethod in failedMethods:
                    self._judge.methodFailed(failedMethod, result)
                    msg = 'The method ' + failedMethod.name + ' of type ' + failedMethod.type + ' failed.'
                    self._logger.logwarn(msg)
                    result = None
                    # we can skip this method if it is optional
                    if not failedMethod.optional:
                        self._logger.logwarn('The failed method was not optional, aborting execution.')
                        success = False
                    else:
                        success = True
            else:
                self.simulator.getWorldState(self.context.getSceneInformation())
                successfulMethods = [currentMethodDesc]
                if batch is not None:
                    successfulMethods = batch.getMethodDescriptions()
                for successfulMethod in successfulMethods:
                    self._judge.methodSucceeded(successfulMethod, result)

        # We are done executing. Let the judge know and reset everything
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
                                             paramPrefix=controllerKey,
                                             supplyMethodDesc=methods[-1]))
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
            methodDescriptions = portfolioInfo['portfolio']
            for methodDesc in methodDescriptions:
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
            plannerType = ArmPlanner.STRING_REPRESENTATION
            controllerType = ArmController.STRING_REPRESENTATION
        elif isinstance(action, ActionPrimitives.GraspAction):
            plannerType = GraspPlanner.STRING_REPRESENTATION
            controllerType = GraspController.STRING_REPRESENTATION
        elif isinstance(action, ActionPrimitives.PlaceAction):
            plannerType = PlacePlanner.STRING_REPRESENTATION
            controllerType = PlaceController.STRING_REPRESENTATION
        else:
            raise NotImplementedError('%a - only MoveArm-, Grasp-, and PlaceAction are implemented yet.' % action)
        return (plannerType, controllerType)

    def _executeBatch(self, batch, parameters):
        self._logger.loginfo('Executing batch. Batch method is ' + batch.getMethodName())
        # First do some sanity checks for the first role
        firstMethodRole = batch.getMethodDescriptions()[0]
        previousResult = firstMethodRole.supplyMethodDesc.result
        if firstMethodRole.isController() and previousResult is None:
            return (False, None)
        if firstMethodRole.type == GraspController.STRING_REPRESENTATION and\
                not isinstance(previousResult, GraspResult):
            raise ValueError('Attempting to execute a grasp controller, but no grasp given')
        # If all these conditions are fulfilled, let's create the arguments for batch execution
        # First, get the method that is going to execute the batch
        types = [batch.getType(idx) for idx in range(len(batch.getBatchSize()))]
        # TODO we could do a sanity check here, whether this method is really registered for all assigned roles
        method = self.methodPortfolio[types[0]][batch.getMethodName()]
        # Create the inputs for all roles
        batchInput = [(batch.getType(idx), batch.getArguments(idx)) for idx in range(len(batch.getBatchSize()))]
        # Make sure nothing is conflicting
        self._resolveResourceAllocationConflicts(method, types)
        # Now execute the batch
        results = method.executeBatch(startContext=self.context, batchInput=batchInput, parameters=parameters)
        # Next, save the results in our batch data structure
        idx = 0
        bAllSuccess = True
        for (bSuccess, result) in results:
            batch.setResult(idx, bSuccess=bSuccess, result=result)
            idx += 1
            bAllSuccess = bAllSuccess and bSuccess

        # Return success if we have a success for all batch elements
        bAllSuccess = bAllSuccess and idx == batch.getBatchSize() - 1
        return (bAllSuccess, results[-1][1]) # the result is the result of the last batch element

    def _executeMethod(self, currentMethodDesc, parameters):
        self._logger.loginfo('Executing method ' + currentMethodDesc.method_choice)
        # get the instance of the selected method
        method = self.methodPortfolio[currentMethodDesc.type][currentMethodDesc.method_choice]
        self._resolveResourceAllocationConflicts(method, [currentMethodDesc.type])
        inputs = currentMethodDesc.inputs
        if currentMethodDesc.supplyMethodDesc is not None:
            previousResult = currentMethodDesc.supplyMethodDesc.result
        else:
            previousResult = None
        # It is possible that the previous method failed and we do not have a previous result.
        # If this method is a controller it always needs a result, hence we can just abort if
        # we don't have a previous result.
        if currentMethodDesc.isController() and previousResult is None:
            return (False, None)

        # Else switch case method types
        if currentMethodDesc.type == ArmPlanner.STRING_REPRESENTATION:
            solution = self._executeArmPlanner(planner=method, inputs=inputs,
                                               paramPrefix=currentMethodDesc.paramPrefix,
                                               parameters=parameters, graspResult=previousResult)
            success = solution is not None
        elif currentMethodDesc.type == ArmController.STRING_REPRESENTATION:
            solution = None
            success = self._executeArmController(controller=method, inputs=inputs,
                                                 paramPrefix=currentMethodDesc.paramPrefix,
                                                 parameters=parameters, traj=previousResult)
        elif currentMethodDesc.type == GraspPlanner.STRING_REPRESENTATION:
            solution = self._executeGraspPlanner(planner=method, inputs=inputs,
                                                 paramPrefix=currentMethodDesc.paramPrefix,
                                                 parameters=parameters)
            success = solution is not None
        elif currentMethodDesc.type == GraspController.STRING_REPRESENTATION:
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
        reorderedMethods = []
        skipIndex = -1
        # Run over all methods and check whether its an ArmPlanner depending on a grasp planner
        for i in range(len(methods)):
            method = methods[i]
            # Check whether we have to move a Grasp planner to the front.
            if method.type == ArmPlanner.STRING_REPRESENTATION:
                goal = method.inputs['goal']
                if isinstance(goal, ActionPrimitives.GraspReference):
                    reorderedMethods.append(methods[i + 2])
                    method.supplyMethodDesc = methods[i + 2]
                    skipIndex = i + 2

            if i != skipIndex:
                reorderedMethods.append(method)
        return reorderedMethods

    def _resolveResourceAllocationConflicts(self, newMethod, activeRoles):
        remainingAllocatedMethods = []
        for allocatedMethod in self._allocatedMethods:
            if allocatedMethod.hasResourceConflict(activeRoles):
                allocatedMethod.releaseResources(roles=activeRoles)
            else:
                remainingAllocatedMethods.append(allocatedMethod)
        newMethod.allocateResources(roles=activeRoles)
        remainingAllocatedMethods.append(newMethod)
        self._allocatedMethods = remainingAllocatedMethods

    def _releaseResources(self):
        for ameth in self._allocatedMethods:
            ameth.releaseResources()

    def _assembleBatch(self, methods):
        batchSize = 0
        methodName = methods[0].getName()
        batch = MethodDescriptionBatch(methodName=methodName)
        # count how many roles this method takes in a sequence
        while batchSize < len(methods) and methodName == methods[batchSize].getName():
            batch.addMethodDescription(methods[batchSize])
            batchSize += 1
        # A batch has to have size > 1, else it's just a single method
        if batchSize <= 1:
            return None
        return batch

