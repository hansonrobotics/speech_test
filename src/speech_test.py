#!/usr/bin/env python
import rospy
from basic_head_api.srv import ValidFaceExprs
from basic_head_api.msg import MakeFaceExpr, PointHead
from std_msgs.msg import String
from pau2motors.msg import pau
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

  rms_params = {"scale": 1.0/5000, "min": 0.3, "max": 0.7}

  soundfile = None

  def hit(self, rms):
    """
    Publishes the current jaw-modified expression, given rms (root mean squared),
    the volume or the power of a small segment in the file.
    """

    # Map the power number to the jaw coefficient.
    # Note that coefficient can't effectively go below 0 or over 1.
    # It will be cut to this range at the receiving end (pau2motors node)
    p = self.rms_params
    jaw_coeff = min(max(math.sqrt(rms * p["scale"]), p["min"]), p["max"])

    # Copy pau expression message stored during handle_face_in(),
    # modify jaw and publish.
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
    self.soundfile.on_playmore = self.hit # Set callback
    self.soundfile.play()

  def handle_face_in(self, msg):
    # Save expression message for later transmission and modification.
    self.facepau = msg

    # If no sound file is playing, pass the message unmodified
    if not self.soundfile or not self.soundfile.is_playing:
      self.pub.publish(msg)

  def __init__(self):
    self.facepau = pau()

    #Face expression pau message comes in.
    rospy.Subscriber("speechtest_face_in", pau, self.handle_face_in)
    #Modified jaw position pau message comes out.
    self.pub = rospy.Publisher("cmd_face_pau", pau, queue_size=10)

class SpeechTest:

  def handle_cmd_speak(self, msg):
    self.applyjaw.play(msg.data)

  def __init__(self):
    rospy.init_node("speech_test")

    self.applyjaw = ApplyJaw()

    #Listens to filenames to play, if it can be found in the 'res' folder
    rospy.Subscriber("cmd_speak", String, self.handle_cmd_speak)

    #Fix to get the pyglet in background thread exit gracefully on keyboard
    #interrupt (CTRL+C)
    pyglet.clock._get_sleep_time = pyglet.clock.get_sleep_time
    pyglet.clock.get_sleep_time = lambda sleep_idle: pyglet.clock._get_sleep_time(False)

    threading.Timer(0.0, pyglet.app.run).start()
    rospy.on_shutdown(pyglet.app.exit)

if __name__ == '__main__':
  SpeechTest()
  rospy.loginfo("Started")
  rospy.spin()
