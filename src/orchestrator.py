#!/usr/bin/env python
import rospy
from basic_head_api.msg import PointHead
from basic_head_api.msg import MakeFaceExpr
from pau2motors.msg import fsMsgTrackingState
from std_msgs.msg import String

class Switch():

  _is_excited = False

  def excite(self, b):
    self._is_excited = b
    rospy.loginfo("%s excited: %s", self.name, b)
    pub_switched.publish(self.name+"-on" if b else self.name+"-off")

  def handle_ground_msg(self, msg):
    if self._is_excited:
      self.excite(False)
    self.ground["pub"].publish(msg)

  def handle_excited_msg(self, msg):
    if self._is_excited:
      self.excited["pub"].publish(msg)

  def __init__(self, name, ground, excited):
    self.name = name

    rospy.Subscriber(*(ground["subargs"] + [self.handle_ground_msg]))
    rospy.Subscriber(*(excited["subargs"] + [self.handle_excited_msg]))
    self.ground = ground
    self.excited = excited

class Orchestrator:

  def handle_switch(self, msg):
    if msg.data == "transmit":
      for switch in self.switches.values():
        pub_switched.publish(switch.name+"-on" if switch._is_excited else switch.name+"-off")
    else:
      msg = msg.data.split("-")
      self.switches[msg[0]].excite(msg[1] == "on")

  def __init__(self):
    rospy.init_node("orchestrator")

    switches = {}

    switches["neck"] = Switch("neck", {
      "subargs": ["bridge/point_head", PointHead],
      "pub": rospy.Publisher("orch/point_head", PointHead, queue_size=10)
    }, {
      "subargs": ["blender/cmd_neck_pau", fsMsgTrackingState],
      "pub": rospy.Publisher("orch/cmd_neck_pau", fsMsgTrackingState, queue_size=10)
    })

    switches["face"] = Switch("face", {
      "subargs": ["bridge/make_face_expr", MakeFaceExpr],
      "pub": rospy.Publisher("orch/make_face_expr", MakeFaceExpr, queue_size=10)
    }, {
      "subargs": ["blender/make_face_expr", MakeFaceExpr],
      "pub": rospy.Publisher("orch/make_face_expr", MakeFaceExpr, queue_size=10)
    })

    self.switches = switches

    global pub_switched
    pub_switched = rospy.Publisher("orch_switched", String, queue_size=10)
    rospy.Subscriber("orch_switch", String, self.handle_switch)


if __name__ == '__main__':
  Orchestrator()
  rospy.loginfo("Started")
  rospy.spin()
