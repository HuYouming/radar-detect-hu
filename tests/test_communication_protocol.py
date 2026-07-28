import json
import struct
import time
import unittest
from types import SimpleNamespace
from unittest.mock import call, patch

from communication.Messager import Messager
from communication.Receiver import Receiver
from communication.Sender import Sender
from Tools.Tools import Tools


class StubLogger:
    def __init__(self):
        self.messages = []

    def log(self, message):
        self.messages.append(message)


class StubSender:
    def __init__(self):
        self.frames = []

    def send_double_effect_analysis_result_info(self, request_id, key):
        self.frames.append((request_id, key))


def build_messager_for_decision():
    messager = Messager.__new__(Messager)
    messager.logger = StubLogger()
    messager.sender = StubSender()
    messager.receiver_state_topic = "/receiver/state"
    messager._jam_key = "ABC123"
    messager.send_double_time_threshold = 240
    messager.my_health_info = [100, 100, 100, 100, 100, 0, 1500, 5000, 1500, 5000]
    messager.have_double_effect_times = 0
    messager.is_activating_double_effect = False
    messager.double_effect_request_id = 0
    messager.double_effect_request_pending = False
    messager.time_left = 300
    messager.dart_target = 0
    messager.mark_progress = [0, 0, 0, 0, 0, 0]
    messager.interference_level = 1
    messager.is_key_update = 0
    return messager


def build_messager_for_drone():
    messager = Messager.__new__(Messager)
    messager.logger = StubLogger()
    messager.my_color = "Red"
    messager.enemy_id = [101, 102, 103, 104, 106, 107]
    messager.enemy_car_infos = []
    messager.our_car_infos = []
    messager.car_life_infos = {}
    messager.send_map_infos = [[0.0, 0.0] for _ in range(12)]
    messager.send_map_info_is_latest = [0] * 12
    messager.drone_fixed_y = 6.5
    messager._drone_field_xyz = [12.25, 3.0, 2.0]
    messager._drone_ros_update_seq = 1
    messager._drone_last_map_update_seq = 0
    messager._drone_missed_map_cycles = 0
    messager._drone_use_ros = True
    messager.guess_points = {
        106: {
            "name": "drone",
            "x": 20.0,
            "y": 1.0,
            "active": True,
            "stamp": 0.0,
        }
    }
    return messager


class ReceiverProtocolTest(unittest.TestCase):
    def setUp(self):
        self.receiver = Receiver.__new__(Receiver)
        self.receiver.logger = StubLogger()
        self.receiver.receiver_state = {}
        self.published = []
        self.receiver.publish_state = lambda event, payload=None: self.published.append((event, payload))

    def test_robot_status_contains_ten_uint16_values(self):
        values = tuple(range(10))
        result = self.receiver.parse_robot_status(struct.pack("<10H", *values))

        self.assertEqual(result, list(values))
        self.assertEqual(self.receiver.receiver_state["my_health"], list(values))

    def test_dart_target_accepts_all_four_values(self):
        for target in range(4):
            data = bytes([0]) + struct.pack("<H", target << 6)
            self.assertEqual(self.receiver.parse_dart_target(data), target)
            self.assertEqual(self.receiver.receiver_state["dart_target"], target)

    def test_radar_status_flags_share_the_same_byte(self):
        radar_status = 2 | (1 << 2) | (3 << 3) | (1 << 5)

        self.assertEqual(self.receiver.parse_double_effect(bytes([radar_status])), (2, 1))
        self.assertEqual(self.receiver.parse_interference_status(bytes([radar_status])), (3, 1))
        self.assertEqual(self.receiver.receiver_state["have_double_effect_times"], 2)
        self.assertTrue(self.receiver.receiver_state["is_activating_double_effect"])
        self.assertEqual(self.receiver.receiver_state["interference_level"], 3)
        self.assertEqual(self.receiver.receiver_state["is_key_update"], 1)


class MessagerProtocolTest(unittest.TestCase):
    def test_drone_uses_ros_x_and_fixed_y_until_five_missed_map_cycles(self):
        messager = build_messager_for_drone()

        for _ in range(6):
            messager._advance_drone_source_for_map_cycle()
            messager.apply_drone_ros_point()
            self.assertEqual(messager.send_map_infos[4], [12.25, 6.5])

        messager.guess_points[106]["stamp"] = time.time()
        messager._advance_drone_source_for_map_cycle()
        messager.apply_guess_points()
        self.assertEqual(messager.send_map_infos[4], [20.0, 1.0])

    def test_drone_ros_update_interrupts_guessing(self):
        messager = build_messager_for_drone()
        messager._drone_use_ros = False
        messager._drone_missed_map_cycles = 6
        messager.guess_points[106]["stamp"] = time.time()
        messager.apply_guess_points()
        self.assertEqual(messager.send_map_infos[4], [20.0, 1.0])

        messager.my_color = "Blue"
        with patch(
            "communication.Messager.pc2.read_points",
            return_value=[(19.25, 4.0, 2.0)],
        ):
            messager._drone_field_callback(SimpleNamespace())

        messager._advance_drone_source_for_map_cycle()
        messager.apply_drone_ros_point()
        self.assertEqual(messager.send_map_infos[4], [8.75, 6.5])

    def test_full_state_synchronizes_dart_target_and_ten_health_values(self):
        messager = build_messager_for_decision()
        health = list(range(10))
        state = {
            "my_health": health,
            "dart_target": 3,
            "have_double_effect_times": 2,
            "is_activating_double_effect": False,
        }

        messager._receiver_state_callback(SimpleNamespace(data=json.dumps({"state": state})))

        self.assertEqual(messager.my_health_info, health)
        self.assertEqual(messager.dart_target, 3)

    def test_only_dart_targets_one_and_two_trigger(self):
        messager = build_messager_for_decision()
        results = {}
        for target in range(4):
            messager.dart_target = target
            results[target] = messager.should_request_double_effect()

        self.assertEqual(results, {0: False, 1: True, 2: True, 3: False})

    def test_double_effect_request_id_is_monotonic(self):
        messager = build_messager_for_decision()
        messager.dart_target = 1
        messager.have_double_effect_times = 2

        messager.send_double_effect_decision()
        messager.send_double_effect_decision()
        self.assertEqual([frame[0] for frame in messager.sender.frames], [1, 1])

        state = {
            "have_double_effect_times": 1,
            "is_activating_double_effect": True,
            "dart_target": 1,
        }
        messager._receiver_state_callback(SimpleNamespace(data=json.dumps({"state": state})))
        messager.send_double_effect_decision()

        state["have_double_effect_times"] = 0
        messager._receiver_state_callback(SimpleNamespace(data=json.dumps({"state": state})))
        messager.send_double_effect_decision()

        self.assertEqual([frame[0] for frame in messager.sender.frames], [1, 1, 2, 2])
        self.assertEqual(messager.double_effect_request_id, 2)

    def test_unknown_time_does_not_trigger(self):
        messager = build_messager_for_decision()
        messager.time_left = -1
        self.assertFalse(messager.should_request_double_effect())

    def test_sentry_position_and_enemy_hp_use_independent_rates(self):
        messager = Messager.__new__(Messager)
        messager.sentry_hz = 5.0
        messager.enemy_hp_hz = 4.9
        messager.last_send_sentry_time = 10.0
        messager.last_send_enemy_hp_time = 20.0
        messager.send_map_infos = [[0.0, 0.0]] * 12
        messager.enemy_health_info = [100] * 5
        sent = []
        messager.send_sentry_perception = lambda infos: sent.append(("position", infos))
        messager.send_sentinel_enemy_HP = lambda infos: sent.append(("hp", infos))

        with patch.object(
            Tools,
            "frame_control_skip",
            side_effect=[(False, 11.0), (False, 21.0)],
        ) as frame_control:
            messager.send_sentry_updates()

        self.assertEqual(
            frame_control.call_args_list,
            [call(5.0, 10.0), call(4.9, 20.0)],
        )
        self.assertEqual([item[0] for item in sent], ["position", "hp"])
        self.assertEqual(messager.last_send_sentry_time, 11.0)
        self.assertEqual(messager.last_send_enemy_hp_time, 21.0)


class SenderProtocolTest(unittest.TestCase):
    def test_radar_decision_payload_has_monotonic_id_then_seven_key_bytes(self):
        sender = Sender.__new__(Sender)
        sender.my_id = 109
        sender.get_frame_header = lambda data_len: b"H" * 5
        sender.get_frame_tail = lambda frame: b"T" * 2

        frame = sender.generate_double_effect_analysis_result_info(1, "ABC123")
        interaction_data = frame[7:-2]

        self.assertEqual(struct.unpack("<H", interaction_data[:2])[0], 0x0121)
        self.assertEqual(interaction_data[6:], b"\x01\x02ABC123")


if __name__ == "__main__":
    unittest.main()
