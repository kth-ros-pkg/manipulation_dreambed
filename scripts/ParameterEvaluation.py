#!/usr/bin/env python
"""
    This file is part of the RobDREAM (http://robdream.eu/) deliverable D4.3.
    Given a task within a context, the ROS node defined in this script utilizes the ManipulationOptimizer
    to evaluate an algorithm for a given parameter configuration.

    @author: Joshua Haustein (haustein@kth.se)
"""
import rospy
import sys
import argparse
import yaml
from manipulation_dreambed.ManipulationOptimizer import ManipulationOptimizer
from manipulation_dreambed.ManipulationDreamBed import ManipulationDreamBed, Logger, OptimizationLogger, PlanningPerformanceJudge
from manipulation_dreambed.GazeboSimulator import GazeboSimulatorWrapper
import manipulation_dreambed.DummySimulator as DummySimulator
from manipulation_dreambed.Context import Context


class ROSLogger(Logger):
    def loginfo(self, message):
        rospy.loginfo(message)

    def logerr(self, message):
        rospy.logerr(message)

    def logwarn(self, message):
        rospy.logwarn(message)

    def logdebug(self, message):
        rospy.logdebug(message)


class DummyLogger(OptimizationLogger):
    def __init__(self):
        pass

    def recordEvaluation(self, value, params):
        pass

    def savePlot(self):
        pass

    def reset(self):
        pass


if __name__ == "__main__":
    rospy.init_node('manipulation_parameter_evaluation')
    argv = rospy.myargv(argv=sys.argv)
    parser = argparse.ArgumentParser(description='Script to test a set of parameters found by the manipulation dream bed.')
    parser.add_argument('parameters_file', nargs='?', type=str, help='Path to a yaml file containing parameters to evaluate. Must be a list')
    parser.add_argument('--gzModelPathsFile', nargs='?', default='../data/gazebo_model_paths.yaml',
                        help='Path to gazebo object models.')
    parser.add_argument('--nosim', dest='nosim', action='store_true')
    parser.add_argument('--portfolio', nargs='?', default='../data/portfolio.yaml',
                        help='File that contains information about the method portfolio')
    parser.add_argument('--scene', nargs='?', default='../data/shelfWorld.yaml',
                        help='File that contains a scene description.')
    parser.add_argument('--task', nargs='?', default='../data/mini_shelf_task.yaml',
                        help='File that contains a task description.')
    parser.add_argument('--numRounds', type=int, default=5,
                        help='Number of averaging rounds to compensate for randomness.')
    # rospy.loginfo('The filtered arguments are: %s', argv)
    args = parser.parse_args(args=argv[1:])
    portfolioDescription = args.portfolio

    # TODO: listen to service calls of optimizing a task within a context (at first load context from file)
    # Set up the context.
    context = Context()
    context.readSceneDescription(args.scene)
    context.readTask(args.task)
    logger = ROSLogger()
    # Set up simulator.
    rospy.loginfo('ParameterEvaluation: Setting up simulator...')
    if args.nosim:
        simulator = DummySimulator.getSimulatorInstance(logger)
        additional_portfolio_methods = [DummySimulator.getControllerInstance()]
    else:
        simulator = GazeboSimulatorWrapper(args.gzModelPathsFile)
        additional_portfolio_methods = None
    # Set up dreambed.
    rospy.loginfo('ParameterEvaluation: Setting up dreambed...')
    optimizationLogger = DummyLogger()
    judge = None
    if args.nosim:
        judge = PlanningPerformanceJudge()
    dreamBed = ManipulationDreamBed(simulator, portfolioDescription,
                                    optimizationLogger,
                                    numAveragingSteps=args.numRounds,
                                    judge=judge,
                                    logger=logger)
    dreamBed.init(additional_portfolio_methods)
    rospy.loginfo('ParameterEvaluation: Setting context...')
    dreamBed.setContext(context)
    import IPython
    IPython.embed()
    # Call evaluation function once for each parameters set in the list
    afile = open(args.parameters_file, 'r')
    parameters_list = yaml.load(afile)
    afile.close()
    for i in range(len(parameters_list)):
        parameters = parameters_list[i]
        rospy.loginfo('ParameterEvaluation: Evaluating parameters %s' % str(parameters))
        score = dreamBed.evaluate(**parameters)
        rospy.loginfo('ParameterEvaluation: Parameters %i achieve a score of %f' %(i, score))
    # debugFunction(dreamBed, context)
    # Done, clean up.
    rospy.loginfo('ParameterEvaluation: Evaluation finished. Terminating.')
    dreamBed.destroy()
