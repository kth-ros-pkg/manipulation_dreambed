#!/usr/bin/env python
"""
    This file is part of the RobDREAM (http://robdream.eu/) deliverable D4.2.

    @author: Silvia Cruciani (cruciani@kth.se)
"""
import rospy
import yaml
from manipulation_dreambed.srv import DreamBedEvaluation

def func(**kwargs):
    rospy.wait_for_service('dream_bed_evaluation')
    try:
        dream_bed_evaluation = rospy.ServiceProxy('dream_bed_evaluation', DreamBedEvaluation)
        resp1 = dream_bed_evaluation(yaml.dump(kwargs))
        return resp1.value
    except rospy.ServiceException, e:
        print "Service call failed: %s" % repr(e)
