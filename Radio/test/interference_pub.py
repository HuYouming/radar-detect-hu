#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import json
import rospy
from std_msgs.msg import String

rospy.init_node('test_interference_pub')
pub = rospy.Publisher('/receiver/state', String, queue_size=1)

seq = 0
rate = rospy.Rate(1)  # 1Hz

while not rospy.is_shutdown():
    msg_data = {
        "seq": seq,
        "stamp": rospy.Time.now().to_sec(),
        "type": "interference_status",
        "state": {
            "is_activating_double_effect": False,
            "my_health": [100, 100, 100, 100, 100, 0, 1500, 5000, 1500, 5000],
            "mark_progress": [0, 0, 0, 0, 0, 0],
            "have_double_effect_times": 0,
            "time_left": -1,
            "dart_target": 0,
            "interference_level": 3,
            "is_key_update": 0,
        },
        "payload": {
            "interference_level": 3,   # 0~3
            "is_key_update": 1         # 0 或 1
        }
    }

    msg = String(data=json.dumps(msg_data, separators=(',', ':')))
    pub.publish(msg)
    rospy.loginfo(f"Published interference_status, seq={seq}")
    seq += 1
    rate.sleep()