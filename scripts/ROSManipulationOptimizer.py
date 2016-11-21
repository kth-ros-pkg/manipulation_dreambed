#!/usr/bin/env python
"""
    This file is part of the RobDREAM (http://robdream.eu/) deliverable D4.2.
    Given a task within a context, the ROS node defined in this script utilizes the ManipulationOptimizer
    to select appropriate algorithms and optimize their respective performance. The best performing choice of
    methods as well as their respective parameters are TODO: stored? published?

    Note, this node may be replaced by a more general script/node that accesses the
    DreamBeds of all work packages.

    @author: Joshua Haustein (haustein@kth.se)
"""
import rospy
import sys
import argparse
import pysmac
from manipulation_dreambed.ManipulationOptimizer import ManipulationOptimizer
from manipulation_dreambed.ManipulationDreamBed import ManipulationDreamBed, Logger
from manipulation_dreambed.GazeboSimulator import GazeboSimulatorWrapper
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


if __name__ == "__main__":
    rospy.init_node('manipulation_optimizer_node')
    argv = rospy.myargv(argv=sys.argv)
    parser = argparse.ArgumentParser(description='Script to test the manipulation dream bed.')
    parser.add_argument('--gzModelPathsFile', nargs='?', default='../data/gazebo_model_paths.yaml',
                        help='Path to gazebo object models.')
    parser.add_argument('--portfolio', nargs='?', default='../data/portfolio.yaml',
                        help='File that contains information about the method portfolio')
    parser.add_argument('--scene', nargs='?', default='../data/shelfWorld.yaml',
                        help='File that contains a scene description.')
    parser.add_argument('--task', nargs='?', default='../data/mini_shelf_task.yaml',
                        help='File that contains a task description.')
    parser.add_argument('--log', nargs='?', default='../result.yaml',
                        help='File for logging objective values.')
    # rospy.loginfo('The filtered arguments are: %s', argv)
    args = parser.parse_args(args=argv[1:])
    portfolioDescription = args.portfolio

    # TODO: listen to service calls of optimizing a task within a context (at first load context from file)
    # Set up the context.
    context = Context()
    context.readSceneDescription(args.scene)
    context.readTask(args.task)
    # Set up simulator.
    rospy.loginfo('ROSManipulationOptimizer: Setting up simulator...')
    simulator = GazeboSimulatorWrapper(args.gzModelPathsFile)
    # Set up dreambed.
    rospy.loginfo('ROSManipulationOptimizer: Setting up dreambed...')
    dreamBed = ManipulationDreamBed(simulator, portfolioDescription, logFileName=args.log, logger=ROSLogger())
    dreamBed.init()
    rospy.loginfo('ROSManipulationOptimizer: Setting context...')
    dreamBed.setContext(context)
    # Create optimizer.
    rospy.loginfo('ROSManipulationOptimizer: Creating optimizer...')
    manipOptimizer = ManipulationOptimizer(dreamBed)
    # Do the job, optimize!
    rospy.loginfo('ROSManipulationOptimizer: Starting optimizer...')
    optimalSolution = manipOptimizer.run()
    # import IPython
    # IPython.embed()
    # Done, clean up.
    rospy.loginfo('ROSManipulationOptimizer: Optimization finished. Terminating.')
    dreamBed.destroy()
    # TODO: return results somehow
