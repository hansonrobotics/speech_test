#!/usr/bin/env python
import rospy
from basic_head_api.srv import ValidFaceExprs
from basic_head_api.msg import MakeFaceExpr, PointHead
from std_msgs.msg import String
from pau2motors.msg import fsMsgTrackingState
import random
import ShapekeyStore
import time
import copy
from SoundFile import SoundFile
from geometry_msgs.msg import Vector3
import threading
import pyglet
import math

class ApplyJaw:
  """
  This class will listen to remapped basic_head_api output, apply jaw position
  and send the modified message to pau2motors.
  """

  rms_params = {"scale": 1.0/1500, "min": 0.3, "max": 0.7}

  soundfile = None

  def hit(self, rms):
    p = self.rms_params
    jaw_coeff = min(max(math.sqrt(rms * p["scale"]), p["min"]), p["max"])

    cmd = copy.deepcopy(self.facepau)
    coeffs = list(cmd.m_coeffs)
    coeffs[ShapekeyStore.getIndex("JawOpen")] = jaw_coeff
    cmd.m_coeffs = coeffs
    self.pub.publish(cmd)

  def stop(self):
    if self.soundfile:
      self.soundfile.stop()

  def play(self, filename):
    self.stop()
    self.soundfile = SoundFile(filename)
    self.soundfile.on_playmore = self.hit
    self.soundfile.play()

  def handle_face_in(self, msg):
    self.facepau = msg

  def __init__(self):
    self.facepau = fsMsgTrackingState()
    rospy.Subscriber("speechtest_face_in", fsMsgTrackingState, self.handle_face_in)
    self.pub = rospy.Publisher("cmd_face_pau", fsMsgTrackingState, queue_size=10)

class SpeechTest:

  def handle_cmd_speak(self, msg):
    self.applyjaw.play(msg.data)

  def handle_face_in(self, msg):
    jaw_coeff = time.time() % 2
    if jaw_coeff > 1:
      jaw_coeff = 2 - jaw_coeff

    coeffs = list(msg.m_coeffs)
    coeffs[ShapekeyStore.getIndex("JawOpen")] = jaw_coeff
    msg.m_coeffs = coeffs
    self.pub_face_pau.publish(msg)

  @staticmethod
  def call_service(servicename):
    rospy.wait_for_service(servicename)
    try:
        return rospy.ServiceProxy(servicename, ValidFaceExprs)()
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

  def __init__(self):
    rospy.init_node("speech_test")

    self.applyjaw = ApplyJaw()
    rospy.Subscriber("cmd_speak", String, self.handle_cmd_speak)

    #Fix to get the pyglet in background thread exit gracefully
    pyglet.clock._get_sleep_time = pyglet.clock.get_sleep_time
    pyglet.clock.get_sleep_time = lambda sleep_idle: pyglet.clock._get_sleep_time(False)

    threading.Timer(0.0, pyglet.app.run).start()
    rospy.on_shutdown(pyglet.app.exit)

if __name__ == '__main__':
  SpeechTest()
  rospy.loginfo("Started")
  rospy.spin()
