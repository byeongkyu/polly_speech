#!/usr/bin/env python
#-*- encoding: utf8 -*-

import sys
import rospy
import actionlib
from polly_speech.msg import SpeechAction, SpeechGoal

def func_done(state, result):
    pass

def func_feedback(feedback):
    rospy.loginfo(feedback.is_speaking)

def func_active():
    pass

def main(say_text):
    client = actionlib.SimpleActionClient('speech_action', SpeechAction)
    client.wait_for_server()

    goal = SpeechGoal()
    goal.text = say_text
    client.send_goal(goal, done_cb=func_done, feedback_cb=func_feedback, active_cb=func_active)
    client.wait_for_result()
    exit(1)

if __name__ == '__main__':
    rospy.init_node('test_speech', anonymous=False)

    if len(sys.argv) < 2:
        print "Usage: rosrun polly_speech test_speech <Say Somthing>"
        exit(-1)

    m = main(sys.argv[1])
