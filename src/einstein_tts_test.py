#!/usr/bin/env python
import rospy
from basic_head_api.srv import ValidFaceExprs
from basic_head_api.msg import MakeFaceExpr, PointHead
from std_msgs.msg import String
from pau2motors.msg import fsMsgTrackingState
import threading
import random
import ShapekeyStore
import time
import copy

class MakeRandomExprs:

  def hit(self):
    cmd = MakeFaceExpr(
      self.exprnames[random.randint(0, len(self.exprnames)-1)],
      1.0
    )
    self.pub.publish(cmd)

    self.timer = threading.Timer(random.uniform(2.0, 5.0), self.hit)
    self.timer.start()

  def start(self):
    self.hit()

  def stop(self):
    self.pub.publish(MakeFaceExpr("neutral", 0))
    self.timer.cancel()

  def __init__(self, exprnames):
    self.exprnames = exprnames
    self.pub = rospy.Publisher('make_face_expr', MakeFaceExpr, queue_size=10)

class PointHeadRandom:
  def hit(self):
    cmd = PointHead(
      random.gauss(0, 0.25),
      random.gauss(0, 0.1),
      random.gauss(0, 0.1)
    )
    self.pub.publish(cmd)

    self.timer = threading.Timer(random.uniform(2.0, 5.0), self.hit)
    self.timer.start()

  def start(self):
    self.hit()

  def stop(self):
    self.pub.publish(PointHead(0, 0, 0))
    self.timer.cancel()

  def __init__(self):
    self.pub = rospy.Publisher('point_head', PointHead, queue_size=10)

class ApplyJaw:
  """
  This class will listen to remapped basic_head_api output, apply jaw position
  and send the modified message to pau2motors.
  """

  def hit(self):
    cmd = copy.deepcopy(self.facepau)

    jaw_coeff = time.time() % 2
    if jaw_coeff > 1:
      jaw_coeff = 2 - jaw_coeff

    coeffs = list(cmd.m_coeffs)
    coeffs[ShapekeyStore.getIndex("JawOpen")] = jaw_coeff
    cmd.m_coeffs = coeffs
    self.pub.publish(cmd)

    self.timer = threading.Timer(0.03, self.hit)
    self.timer.start()

  def start(self):
    self.hit()

  def stop(self):
    self.pub.publish(fsMsgTrackingState())
    self.timer.cancel()

  def handle_face_in(self, msg):
    self.facepau = msg

  def __init__(self):
    self.facepau = fsMsgTrackingState()
    rospy.Subscriber("ttstest_face_in", fsMsgTrackingState, self.handle_face_in)
    self.pub = rospy.Publisher("cmd_face_pau", fsMsgTrackingState, queue_size=10)

class TtsTest:

  def handle_cmd(self, msg):
    msg = msg.data
    if msg == "start":
      self.facemaker.start()
      self.headpointer.start()
      self.applyjaw.start()
    elif msg == "stop":
      self.facemaker.stop()
      self.headpointer.stop()
      self.applyjaw.stop()

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
    rospy.init_node("tts_test")

    exprnames = self.call_service("valid_face_exprs").exprnames
    self.facemaker = MakeRandomExprs(exprnames)
    self.headpointer = PointHeadRandom()
    self.applyjaw = ApplyJaw()

    rospy.Subscriber("cmd_ttstest", String, self.handle_cmd)

if __name__ == '__main__':
  TtsTest()
  rospy.loginfo("Started")
  rospy.spin()
