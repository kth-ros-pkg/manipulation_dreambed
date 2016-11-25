"""
    This file is part of the RobDREAM (http://robdream.eu/) deliverable D4.2.
    The ManipulationOptimizer provides utilizes pySMAC as well as the ManipulationDreamBed
    to determine the optimal algorithms and respective parameters for a given task
    within a given context.

    @author: Joshua Haustein (haustein@kth.se)
"""
import pysmac
import IPython
import rospy
import PySMACROSInterface
from manipulation_dreambed.srv import DreamBedEvaluation, DreamBedEvaluationResponse
import yaml


class ManipulationOptimizer(object):
    def __init__(self, dreamBed):
        self.dreamBed = dreamBed
        self.optimizer = pysmac.SMAC_optimizer()
        self.iterationCounter = 0
        self.totalNumIterations = 0
        self.optimizer_service = rospy.Service('dream_bed_evaluation', DreamBedEvaluation, self.serviceCallBack)

    def run(self, numEvals=100, deterministic=False):
        self.iterationCounter = 0
        self.totalNumIterations = numEvals
        (paramDef, condParams, forbiddenClauses) = self.dreamBed.getParameters()
        # paramDef = dict(x=('real', [0.9, 1.5], 1.2),
        #                 y=('real', [-0.5, 0.5], 0.0),
        #                 phi=('real', [-0.3, 0.3], 0.0))
        # IPython.embed()

        score, bestParameters = self.optimizer.minimize(func=PySMACROSInterface.func,
                                                        max_evaluations=numEvals,
                                                        parameter_dict=paramDef,
                                                        deterministic=deterministic)
        print 'Best score is ', score
        print 'Best parameters are ', bestParameters

    def serviceCallBack(self, req):
        kwargs = yaml.load(req.dictionary)
        print 'Iteration ', self.iterationCounter, ' out of ', self.totalNumIterations
        self.iterationCounter += 1
        # TODO: we probably want to catch errors here
        val = self.dreamBed.evaluate(**kwargs)
        return DreamBedEvaluationResponse(val)
