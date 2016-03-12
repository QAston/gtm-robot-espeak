#! /usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import actionlib

from gtm_speech_msgs.msg import SpeakAction, SpeakGoal, SpeakResult, SpeakFeedback
from gtm_speech_msgs.cfg import SpeechConfig

def feedback_cb(feedback):
  rospy.loginfo("Speaking feedback: processed %s characters", feedback.processed_characters)
  
def done_cb(state, result):
  rospy.loginfo("Speaking result - finished: %s, goal state %s", result.finished, state)
  
def active_cb():
  rospy.loginfo("Speaking action active now")
  
import dynamic_reconfigure.client

orig_config = None

def config_callback(config):
  global orig_config
  if orig_config is None:
    orig_config = config
  rospy.loginfo("Config is {volume}, {pitch}, {rate}, {pitch_range}, {announce_punctuation}, {wordgap}".format(**config))

rospy.init_node('speech_client')
client = actionlib.SimpleActionClient('speak', SpeakAction)
client.wait_for_server()

while not rospy.is_shutdown():
  goal = SpeakGoal()
  goal.text = "Witaj świecie raz dwa"
  client.send_goal(goal, feedback_cb=feedback_cb, done_cb=done_cb, active_cb=active_cb)
  
  #interrupt immediately
  goal = SpeakGoal()
  goal.text = "Witaj świecie raz dwa trzy cztery"
  client.send_goal(goal, feedback_cb=feedback_cb, done_cb=done_cb, active_cb=active_cb)
  rospy.sleep(1)
  
  #interrupt aftet a second
  goal = SpeakGoal()
  goal.text = "Witaj ponownie świecie, raz dwa trzy cztery"
  client.send_goal(goal, feedback_cb=feedback_cb, done_cb=done_cb, active_cb=active_cb)
  
  # no interruption
  client.wait_for_result()

  reconfigure = dynamic_reconfigure.client.Client("speech_synthesizer", timeout=30, config_callback=config_callback)

  #test configuration change
  reconfigure.update_configuration({"volume":200, "pitch":200, "rate":210, "announce_punctuation":True})
  rospy.sleep(2)
  goal = SpeakGoal()
  goal.text = "Witaj ponownie świecie, raz dwa trzy cztery"
  client.send_goal(goal, feedback_cb=feedback_cb, done_cb=done_cb, active_cb=active_cb)
  rospy.sleep(5)

  reconfigure.update_configuration(orig_config)