import struct
import socket
import threading
import queue
import time
from dataclasses import dataclass, field
from typing import Callable, Dict, Optional, List, Tuple
from enum import Enum
from multiprocessing import Array
import rospy


# ==================== 命令字定义 ====================
class CommandType(Enum):
    POSITION = 0x0A01      # 对方机器人位置坐标 (24字节)
    HEALTH = 0x0A02        # 对方机器人血量信息 (12字节)
    AMMO = 0x0A03          # 对方机器人剩余发弹量信息 (10字节)
    TEAM_STATUS = 0x0A04   # 对方队伍宏观状态信息 (8字节)
    BUFF = 0x0A05          # 对方各机器人当前增益效果 (36字节)
    JAM_KEY = 0x0A06       # 对方干扰波密钥 (6字节)


COMMAND_LENGTH_MAP = {
    CommandType.POSITION: 24,
    CommandType.HEALTH: 12,
    CommandType.AMMO: 10,
    CommandType.TEAM_STATUS: 8,
    CommandType.BUFF: 36,
    CommandType.JAM_KEY: 6,
}


# ==================== CRC8查表 ====================
CRC8_TAB = [
    0x00, 0x5E, 0xBC, 0xE2, 0x61, 0x3F, 0xDD, 0x83, 0xC2, 0x9C, 0x7E, 0x20, 0xA3, 0xFD, 0x1F, 0x41,
    0x9D, 0xC3, 0x21, 0x7F, 0xFC, 0xA2, 0x40, 0x1E, 0x5F, 0x01, 0xE3, 0xBD, 0x3E, 0x60, 0x82, 0xDC,
    0x23, 0x7D, 0x9F, 0xC1, 0x42, 0x1C, 0xFE, 0xA0, 0xE1, 0xBF, 0x5D, 0x03, 0x80, 0xDE, 0x3C, 0x62,
    0xBE, 0xE0, 0x02, 0x5C, 0xDF, 0x81, 0x63, 0x3D, 0x7C, 0x22, 0xC0, 0x9E, 0x1D, 0x43, 0xA1, 0xFF,
    0x46, 0x18, 0xFA, 0xA4, 0x27, 0x79, 0x9B, 0xC5, 0x84, 0xDA, 0x38, 0x66, 0xE5, 0xBB, 0x59, 0x07,
    0xDB, 0x85, 0x67, 0x39, 0xBA, 0xE4, 0x06, 0x58, 0x19, 0x47, 0xA5, 0xFB, 0x78, 0x26, 0xC4, 0x9A,
    0x65, 0x3B, 0xD9, 0x87, 0x04, 0x5A, 0xB8, 0xE6, 0xA7, 0xF9, 0x1B, 0x45, 0xC6, 0x98, 0x7A, 0x24,
    0xF8, 0xA6, 0x44, 0x1A, 0x99, 0xC7, 0x25, 0x7B, 0x3A, 0x64, 0x86, 0xD8, 0x5B, 0x05, 0xE7, 0xB9,
    0x8C, 0xD2, 0x30, 0x6E, 0xED, 0xB3, 0x51, 0x0F, 0x4E, 0x10, 0xF2, 0xAC, 0x2F, 0x71, 0x93, 0xCD,
    0x11, 0x4F, 0xAD, 0xF3, 0x70, 0x2E, 0xCC, 0x92, 0xD3, 0x8D, 0x6F, 0x31, 0xB2, 0xEC, 0x0E, 0x50,
    0xAF, 0xF1, 0x13, 0x4D, 0xCE, 0x90, 0x72, 0x2C, 0x6D, 0x33, 0xD1, 0x8F, 0x0C, 0x52, 0xB0, 0xEE,
    0x32, 0x6C, 0x8E, 0xD0, 0x53, 0x0D, 0xEF, 0xB1, 0xF0, 0xAE, 0x4C, 0x12, 0x91, 0xCF, 0x2D, 0x73,
    0xCA, 0x94, 0x76, 0x28, 0xAB, 0xF5, 0x17, 0x49, 0x08, 0x56, 0xB4, 0xEA, 0x69, 0x37, 0xD5, 0x8B,
    0x57, 0x09, 0xEB, 0xB5, 0x36, 0x68, 0x8A, 0xD4, 0x95, 0xCB, 0x29, 0x77, 0xF4, 0xAA, 0x48, 0x16,
    0xE9, 0xB7, 0x55, 0x0B, 0x88, 0xD6, 0x34, 0x6A, 0x2B, 0x75, 0x97, 0xC9, 0x4A, 0x14, 0xF6, 0xA8,
    0x74, 0x2A, 0xC8, 0x96, 0x15, 0x4B, 0xA9, 0xF7, 0xB6, 0xE8, 0x0A, 0x54, 0xD7, 0x89, 0x6B, 0x35,
]


def crc8(data):
    value = 0xFF
    for byte in data:
        value = CRC8_TAB[value ^ byte]
    return value & 0xFF


CRC16_INIT = 0xFFFF

WCRC_TABLE = [
    0x0000, 0x1189, 0x2312, 0x329B, 0x4624, 0x57AD, 0x6536, 0x74BF,
    0x8C48, 0x9DC1, 0xAF5A, 0xBED3, 0xCA6C, 0xDBE5, 0xE97E, 0xF8F7,
    0x1081, 0x0108, 0x3393, 0x221A, 0x56A5, 0x472C, 0x75B7, 0x643E,
    0x9CC9, 0x8D40, 0xBFDB, 0xAE52, 0xDAED, 0xCB64, 0xF9FF, 0xE876,
    0x2102, 0x308B, 0x0210, 0x1399, 0x6726, 0x76AF, 0x4434, 0x55BD,
    0xAD4A, 0xBCC3, 0x8E58, 0x9FD1, 0xEB6E, 0xFAE7, 0xC87C, 0xD9F5,
    0x3183, 0x200A, 0x1291, 0x0318, 0x77A7, 0x662E, 0x54B5, 0x453C,
    0xBDCB, 0xAC42, 0x9ED9, 0x8F50, 0xFBEF, 0xEA66, 0xD8FD, 0xC974,
    0x4204, 0x538D, 0x6116, 0x709F, 0x0420, 0x15A9, 0x2732, 0x36BB,
    0xCE4C, 0xDFC5, 0xED5E, 0xFCD7, 0x8868, 0x99E1, 0xAB7A, 0xBAF3,
    0x5285, 0x430C, 0x7197, 0x601E, 0x14A1, 0x0528, 0x37B3, 0x263A,
    0xDECD, 0xCF44, 0xFDDF, 0xEC56, 0x98E9, 0x8960, 0xBBFB, 0xAA72,
    0x6306, 0x728F, 0x4014, 0x519D, 0x2522, 0x34AB, 0x0630, 0x17B9,
    0xEF4E, 0xFEC7, 0xCC5C, 0xDDD5, 0xA96A, 0xB8E3, 0x8A78, 0x9BF1,
    0x7387, 0x620E, 0x5095, 0x411C, 0x35A3, 0x242A, 0x16B1, 0x0738,
    0xFFCF, 0xEE46, 0xDCDD, 0xCD54, 0xB9EB, 0xA862, 0x9AF9, 0x8B70,
    0x8408, 0x9581, 0xA71A, 0xB693, 0xC22C, 0xD3A5, 0xE13E, 0xF0B7,
    0x0840, 0x19C9, 0x2B52, 0x3ADB, 0x4E64, 0x5FED, 0x6D76, 0x7CFF,
    0x9489, 0x8500, 0xB79B, 0xA612, 0xD2AD, 0xC324, 0xF1BF, 0xE036,
    0x18C1, 0x0948, 0x3BD3, 0x2A5A, 0x5EE5, 0x4F6C, 0x7DF7, 0x6C7E,
    0xA50A, 0xB483, 0x8618, 0x9791, 0xE32E, 0xF2A7, 0xC03C, 0xD1B5,
    0x2942, 0x38CB, 0x0A50, 0x1BD9, 0x6F66, 0x7EEF, 0x4C74, 0x5DFD,
    0xB58B, 0xA402, 0x9699, 0x8710, 0xF3AF, 0xE226, 0xD0BD, 0xC134,
    0x39C3, 0x284A, 0x1AD1, 0x0B58, 0x7FE7, 0x6E6E, 0x5CF5, 0x4D7C,
    0xC60C, 0xD785, 0xE51E, 0xF497, 0x8028, 0x91A1, 0xA33A, 0xB2B3,
    0x4A44, 0x5BCD, 0x6956, 0x78DF, 0x0C60, 0x1DE9, 0x2F72, 0x3EFB,
    0xD68D, 0xC704, 0xF59F, 0xE416, 0x90A9, 0x8120, 0xB3BB, 0xA232,
    0x5AC5, 0x4B4C, 0x79D7, 0x685E, 0x1CE1, 0x0D68, 0x3FF3, 0x2E7A,
    0xE70E, 0xF687, 0xC41C, 0xD595, 0xA12A, 0xB0A3, 0x8238, 0x93B1,
    0x6B46, 0x7ACF, 0x4854, 0x59DD, 0x2D62, 0x3CEB, 0x0E70, 0x1FF9,
    0xF78F, 0xE606, 0xD49D, 0xC514, 0xB1AB, 0xA022, 0x92B9, 0x8330,
    0x7BC7, 0x6A4E, 0x58D5, 0x495C, 0x3DE3, 0x2C6A, 0x1EF1, 0x0F78,
]


def crc16(data: bytes, crc: int = CRC16_INIT) -> int:
    for byte in data:
        crc = ((crc >> 8) ^ WCRC_TABLE[(crc ^ byte) & 0x00FF]) & 0xFFFF
    return crc


# ==================== 数据类 ====================
@dataclass
class RobotPositions:
    hero: Tuple[int, int] = (0, 0)
    engineer: Tuple[int, int] = (0, 0)
    infantry_3: Tuple[int, int] = (0, 0)
    infantry_4: Tuple[int, int] = (0, 0)
    aerial: Tuple[int, int] = (0, 0)
    sentry: Tuple[int, int] = (0, 0)

    @classmethod
    def from_bytes(cls, data: bytes) -> 'RobotPositions':
        if len(data) < 24:
            raise ValueError(f"数据长度不足24字节，实际{len(data)}字节")
        coords = struct.unpack('>12h', data[:24])
        return cls(
            hero=(coords[0], coords[1]),
            engineer=(coords[2], coords[3]),
            infantry_3=(coords[4], coords[5]),
            infantry_4=(coords[6], coords[7]),
            aerial=(coords[8], coords[9]),
            sentry=(coords[10], coords[11])
        )

    def get_position_dict(self) -> Dict[int, Tuple[int, int]]:
        return {1: self.hero, 2: self.engineer, 3: self.infantry_3, 
                4: self.infantry_4, 6: self.aerial, 7: self.sentry}


@dataclass
class RobotHealths:
    hero: int = 0
    engineer: int = 0
    infantry_3: int = 0
    infantry_4: int = 0
    reserved: int = 0
    sentry: int = 0

    @classmethod
    def from_bytes(cls, data: bytes) -> 'RobotHealths':
        if len(data) < 12:
            raise ValueError(f"数据长度不足12字节，实际{len(data)}字节")
        values = struct.unpack('>6H', data[:12])
        return cls(
            hero=values[0], engineer=values[1], infantry_3=values[2],
            infantry_4=values[3], reserved=values[4], sentry=values[5]
        )

    def get_health_dict(self) -> Dict[int, int]:
        return {1: self.hero, 2: self.engineer, 3: self.infantry_3, 
                4: self.infantry_4, 7: self.sentry}


@dataclass
class RobotAmmos:
    hero: int = 0
    infantry_3: int = 0
    infantry_4: int = 0
    aerial: int = 0
    sentry: int = 0

    @classmethod
    def from_bytes(cls, data: bytes) -> 'RobotAmmos':
        if len(data) < 10:
            raise ValueError(f"数据长度不足10字节，实际{len(data)}字节")
        values = struct.unpack('>5H', data[:10])
        return cls(
            hero=values[0], infantry_3=values[1], infantry_4=values[2],
            aerial=values[3], sentry=values[4]
        )

    def get_ammo_dict(self) -> Dict[int, int]:
        return {1: self.hero, 3: self.infantry_3, 4: self.infantry_4, 
                6: self.aerial, 7: self.sentry}


@dataclass
class TeamModuleStatus:
    base_shield: int = 0
    outpost_shield: int = 0
    hero_shield: int = 0
    engineer_shield: int = 0
    infantry_3_shield: int = 0
    infantry_4_shield: int = 0
    sentry_shield: int = 0
    base_occupied: bool = False
    outpost_occupied: bool = False
    power_rune: int = 0
    flyover_buff: bool = False
    flyover_cooldown: bool = False
    center_buff: int = 0
    resource_island_buff: int = 0
    power_rune_point: bool = False
    trapezoid_highland: bool = False
    ring_highland: int = 0

    @classmethod
    def from_uint32(cls, status_bits: int) -> 'TeamModuleStatus':
        return cls(
            base_shield=(status_bits >> 0) & 0x3,
            outpost_shield=(status_bits >> 2) & 0x3,
            hero_shield=(status_bits >> 4) & 0x3,
            engineer_shield=(status_bits >> 6) & 0x3,
            infantry_3_shield=(status_bits >> 8) & 0x3,
            infantry_4_shield=(status_bits >> 10) & 0x3,
            sentry_shield=(status_bits >> 12) & 0x3,
            base_occupied=bool((status_bits >> 14) & 0x1),
            outpost_occupied=bool((status_bits >> 15) & 0x1),
            power_rune=(status_bits >> 16) & 0x3,
            flyover_buff=bool((status_bits >> 18) & 0x1),
            flyover_cooldown=bool((status_bits >> 19) & 0x1),
            center_buff=(status_bits >> 20) & 0x3,
            resource_island_buff=(status_bits >> 24) & 0x3,
            power_rune_point=bool((status_bits >> 28) & 0x1),
            trapezoid_highland=bool((status_bits >> 29) & 0x1),
            ring_highland=(status_bits >> 30) & 0x3,
        )


@dataclass
class TeamStatus:
    remaining_coins: int = 0
    destroy_count: int = 0
    module_status: TeamModuleStatus = field(default_factory=TeamModuleStatus)

    @classmethod
    def from_bytes(cls, data: bytes) -> 'TeamStatus':
        if len(data) < 8:
            raise ValueError(f"数据长度不足8字节，实际{len(data)}字节")
        remaining_coins = struct.unpack('>H', data[0:2])[0]
        destroy_count = struct.unpack('>H', data[2:4])[0]
        status_bits = struct.unpack('>I', data[4:8])[0]
        return cls(
            remaining_coins=remaining_coins,
            destroy_count=destroy_count,
            module_status=TeamModuleStatus.from_uint32(status_bits)
        )


@dataclass
class RobotBuffData:
    hp_percent: int = 0
    cooling_percent: int = 0
    defense_percent: int = 0
    attack_percent: int = 0
    hp_value: int = 0


@dataclass
class TeamBuffs:
    hero: RobotBuffData = field(default_factory=RobotBuffData)
    engineer: RobotBuffData = field(default_factory=RobotBuffData)
    infantry_3: RobotBuffData = field(default_factory=RobotBuffData)
    infantry_4: RobotBuffData = field(default_factory=RobotBuffData)
    sentry: RobotBuffData = field(default_factory=RobotBuffData)
    aerial: RobotBuffData = field(default_factory=RobotBuffData)

    @classmethod
    def from_bytes(cls, data: bytes) -> 'TeamBuffs':
        if len(data) < 30:
            raise ValueError(f"数据长度不足30字节，实际{len(data)}字节")
        def parse_buff(offset: int) -> RobotBuffData:
            return RobotBuffData(
                hp_percent=data[offset],
                cooling_percent=data[offset + 1],
                defense_percent=data[offset + 2],
                attack_percent=data[offset + 3],
                hp_value=data[offset + 4]
            )
        return cls(
            hero=parse_buff(0), engineer=parse_buff(5), infantry_3=parse_buff(10),
            infantry_4=parse_buff(15), sentry=parse_buff(20), aerial=parse_buff(25)
        )

    def get_buff_dict(self) -> Dict[int, RobotBuffData]:
        return {1: self.hero, 2: self.engineer, 3: self.infantry_3,
                4: self.infantry_4, 7: self.sentry, 6: self.aerial}


@dataclass
class JamKey:
    key: str = ""

    @classmethod
    def from_bytes(cls, data: bytes) -> 'JamKey':
        if len(data) < 6:
            raise ValueError(f"数据长度不足6字节，实际{len(data)}字节")
        try:
            key = data[:6].decode('ascii')
        except UnicodeDecodeError:
            key = data[:6].hex()
        return cls(key=key)


# ==================== 雷达数据帧解析 ====================
class RadarDataFrame:
    FRAME_HEADER = 0xA5

    def __init__(self):
        self.sof: int = 0
        self.data_length: int = 0
        self.seq: int = 0
        self.header_crc8: int = 0
        self.cmd_id: int = 0
        self.command: Optional[CommandType] = None
        self.data: bytes = b''
        self.frame_crc16: int = 0
        self.raw_frame: bytes = b''

    def parse(self, raw_data: bytes) -> bool:
        if len(raw_data) < 9:
            return False

        pos = 0
        self.sof = raw_data[pos]
        if self.sof != self.FRAME_HEADER:
            return False
        pos += 1

        self.data_length = struct.unpack('>H', raw_data[pos:pos+2])[0]
        pos += 2

        self.seq = raw_data[pos]
        pos += 1

        self.header_crc8 = raw_data[pos]
        pos += 1

        header = bytes([0xA5]) + struct.pack('>H', self.data_length) + bytes([self.seq])
        calc_crc8 = crc8(header)
        if self.header_crc8 != calc_crc8:
            return False

        self.cmd_id = struct.unpack('>H', raw_data[pos:pos+2])[0]
        pos += 2

        try:
            self.command = CommandType(self.cmd_id)
        except ValueError:
            return False

        if len(raw_data) < pos + self.data_length + 2:
            return False
        self.data = raw_data[pos:pos + self.data_length]
        pos += self.data_length

        self.frame_crc16 = struct.unpack('>H', raw_data[pos:pos+2])[0]

        frame_for_crc = raw_data[:pos]
        calc_crc16 = crc16(frame_for_crc)
        if self.frame_crc16 != calc_crc16:
            return False

        self.raw_frame = raw_data
        return True

    def to_dataclass(self):
        if not self.command or not self.data:
            return None

        parsers = {
            CommandType.POSITION: RobotPositions,
            CommandType.HEALTH: RobotHealths,
            CommandType.AMMO: RobotAmmos,
            CommandType.TEAM_STATUS: TeamStatus,
            CommandType.BUFF: TeamBuffs,
            CommandType.JAM_KEY: JamKey,
        }

        parser = parsers.get(self.command)
        if parser:
            try:
                expected_len = COMMAND_LENGTH_MAP[self.command]
                if len(self.data) >= expected_len:
                    return parser.from_bytes(self.data)
                else:
                    return None
            except Exception as e:
                return None
        return None


class RadarUDPReceiver:
    # 帧格式：SOF(1) + data_length(2) + seq(1) + CRC8(1) + cmd_id(2) + data(n) + CRC16(2)
    # 总长度 = 9 + data_length
    # 0x0A01: 33B | 0x0A02: 21B | 0x0A03: 19B | 0x0A04: 17B | 0x0A05: 45B | 0x0A06: 15B

    def __init__(self, host: str = "127.0.0.1", port: int = 40001,
                 shared_enemy_health_list=None, shared_enemy_position_list=None):
        self.host = host
        self.port = port
        self.sock: Optional[socket.socket] = None
        self.running = False
        self.receive_thread: Optional[threading.Thread] = None

        # 共享内存引用（与Messager/Receiver共用）
        self.shared_enemy_health_list = shared_enemy_health_list
        self.shared_enemy_position_list = shared_enemy_position_list

        # 内部数据缓存（用于接口读取）
        self._health_lock = threading.Lock()
        self._position_lock = threading.Lock()
        self._latest_health: Optional[RobotHealths] = None
        self._latest_positions: Optional[RobotPositions] = None

        # 回调
        self.callbacks: Dict[CommandType, List[Callable]] = {
            cmd: [] for cmd in CommandType
        }

        self.data_queue: queue.Queue = queue.Queue(maxsize=100)

        # 统计
        self.stats = {
            'total_packets': 0,
            'data': 0,
            'valid_frames': 0,
            'error_frames': 0,
        }

        # 字节流拼接缓冲区（关键修改）
        self._buffer = bytearray()
        self._buffer_lock = threading.Lock()
        self._max_buffer_size = 2048  # 防止内存无限增长

    def register_callback(self, cmd_type: CommandType, callback: Callable):
        self.callbacks[cmd_type].append(callback)

    def _update_shared_health(self, health_data: RobotHealths):
        """更新共享内存中的血量"""
        health_dict = health_data.get_health_dict()
        mapping = {1: 0, 2: 1, 3: 2, 4: 3, 7: 5}
        
        with self._health_lock:
            self._latest_health = health_data
            if self.shared_enemy_health_list is not None:
                for robot_id, hp in health_dict.items():
                    if robot_id in mapping:
                        self.shared_enemy_health_list[mapping[robot_id]] = hp

    def _update_shared_positions(self, pos_data: RobotPositions):
        """更新共享内存中的位置"""
        pos_dict = pos_data.get_position_dict()
        mapping = {1: 0, 2: 2, 3: 4, 4: 6, 6: 8, 7: 10}
        
        with self._position_lock:
            self._latest_positions = pos_data
            if self.shared_enemy_position_list is not None:
                for robot_id, (x, y) in pos_dict.items():
                    if robot_id in mapping:
                        idx = mapping[robot_id]
                        self.shared_enemy_position_list[idx] = float(x)
                        self.shared_enemy_position_list[idx + 1] = float(y)

    def get_latest_health(self) -> Optional[RobotHealths]:
        """线程安全获取最新血量"""
        with self._health_lock:
            return self._latest_health

    def get_latest_positions(self) -> Optional[RobotPositions]:
        """线程安全获取最新位置"""
        with self._position_lock:
            return self._latest_positions

    def start(self) -> bool:
        """启动UDP接收子线程"""
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.bind((self.host, self.port))
            self.sock.settimeout(1.0)
            self.running = True

            self.receive_thread = threading.Thread(target=self._receive_loop, daemon=True)
            self.receive_thread.start()

            print(f"[UDP] 接收器已启动: {self.host}:{self.port}")
            return True
        except Exception as e:
            print(f"[UDP] 启动失败: {e}")
            return False

    def stop(self):
        """停止UDP接收"""
        self.running = False
        if self.receive_thread and self.receive_thread.is_alive():
            self.receive_thread.join(timeout=2)
        if self.sock:
            self.sock.close()
        print("[UDP] 接收器已停止")

    def _receive_loop(self):
        """接收线程主循环：拼接15字节包，解析完整DJI帧"""
        while self.running:
            try:
                data, addr = self.sock.recvfrom(1024)
                self.stats['total_packets'] += 1
                rospy.loginfo("data: ", data)
                self.stats['data'] = data

                # 将收到的数据追加到拼接缓冲区
                with self._buffer_lock:
                    self._buffer.extend(data)
                    if len(self._buffer) > self._max_buffer_size:
                        self._buffer = self._buffer[-self._max_buffer_size//2:]
                        self.stats['error_frames'] += 1

                # 从拼接后的字节流中提取所有完整帧
                frames = self._extract_frames()
                for frame in frames:
                    self.data_queue.put((frame, addr))

                self._process_queue()

            except socket.timeout:
                continue
            except Exception as e:
                pass

    def _extract_frames(self) -> List[RadarDataFrame]:
        """从拼接缓冲区中提取所有校验通过的完整DJI帧"""
        frames: List[RadarDataFrame] = []

        with self._buffer_lock:
            while len(self._buffer) >= 9:  # 最小帧长：header(5) + cmd_id(2) + crc16(2)
                # 查找 SOF 0xA5
                try:
                    sof_idx = self._buffer.index(0xA5)
                except ValueError:
                    # 缓冲区中不存在合法帧头，清空
                    if len(self._buffer) > 0:
                        self.stats['error_frames'] += 1
                    self._buffer.clear()
                    break

                # 丢弃 SOF 之前的垃圾字节
                if sof_idx > 0:
                    self.stats['error_frames'] += 1
                    self._buffer = self._buffer[sof_idx:]

                # 检查是否足以读取 data_length
                if len(self._buffer) < 5:
                    break

                # 读取 data_length（大端）
                data_length = struct.unpack('>H', bytes(self._buffer[1:3]))[0]
                total_frame_len = 9 + data_length  # 5(header) + 2(cmd_id) + n(data) + 2(crc16)

                # 防御异常长度（根据协议最大约 45+9=54，设个安全上限）
                if total_frame_len > 256:
                    self.stats['error_frames'] += 1
                    self._buffer = self._buffer[1:]  # 丢弃这个假 SOF
                    continue

                # 数据尚未收齐，等待下一次 UDP 包
                if len(self._buffer) < total_frame_len:
                    break

                # 提取候选帧
                candidate = bytes(self._buffer[:total_frame_len])

                # 使用 RadarDataFrame 完整校验（CRC8 + CRC16 + cmd_id）
                frame = RadarDataFrame()
                if frame.parse(candidate):
                    frames.append(frame)
                    self.stats['valid_frames'] += 1
                    # 从缓冲区移除已解析帧
                    self._buffer = self._buffer[total_frame_len:]
                else:
                    # 校验失败，说明这个 0xA5 不是真帧头，从下一个字节重新搜索
                    self.stats['error_frames'] += 1
                    self._buffer = self._buffer[1:]

        return frames

    def _process_queue(self):
        """处理数据帧队列"""
        while not self.data_queue.empty():
            try:
                frame, addr = self.data_queue.get_nowait()
                parsed_data = frame.to_dataclass()

                if parsed_data and frame.command in self.callbacks:
                    # 自动更新共享内存
                    if frame.command == CommandType.HEALTH and isinstance(parsed_data, RobotHealths):
                        self._update_shared_health(parsed_data)
                    elif frame.command == CommandType.POSITION and isinstance(parsed_data, RobotPositions):
                        self._update_shared_positions(parsed_data)

                    # 执行回调
                    for callback in self.callbacks[frame.command]:
                        try:
                            callback(parsed_data, addr, frame.seq)
                        except Exception as e:
                            pass
            except queue.Empty:
                break

    def get_stats(self) -> dict:
        return {
            'total_packets': self.stats['total_packets'],
            'data': self.stats['data'],
            'valid_frames': self.stats['valid_frames'],
            'error_frames': self.stats['error_frames'],
            'success_rate': f"{(self.stats['valid_frames'] / max(self.stats['total_packets'], 1) * 100):.1f}%"
        }