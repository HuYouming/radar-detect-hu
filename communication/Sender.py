# 定义一个Communicator用于串口通信
import serial
import serial.tools.list_ports
import struct
from Radio.interferance_level_sender import InterferenceSender
from Log.Log import RadarLog
class Sender:
    def __init__(self , cfg ):
        # 我方颜色
        self.my_color = cfg['global']['my_color']

        self.logger = RadarLog("Sender")

        if self.my_color == 'Red':
            self.my_id = 9
            self.my_sentinel_id = 7
        else:
            self.my_id = 109
            self.my_sentinel_id = 107

        self.enabled = cfg.get('communication', {}).get('enabled', True)
        self.port = cfg['communication'].get('port', '/dev/ttyUSB0')
        self.bps = cfg['communication']['bps']
        self.timex = cfg['communication']['timex']
        # self.SOF = b'\xA5'
        self.SOF = struct.pack('B',0xa5)
        self.seq = 0  # 目前均为单包数据，且无重发机制?
        self.ser = self.serial_init()
        # UDP发送器
        self.udp_sender = InterferenceSender("127.0.0.1", 40003)
        self.CRC8_TABLE = [
            0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83, 0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41,
            0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e, 0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc,
            0x23, 0x7d, 0x9f, 0xc1, 0x42, 0x1c, 0xfe, 0xa0, 0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62,
            0xbe, 0xe0, 0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d, 0x7c, 0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff,
            0x46, 0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5, 0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07,
            0xdb, 0x85, 0x67, 0x39, 0xba, 0xe4, 0x06, 0x58, 0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a,
            0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 0xb8, 0xe6, 0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98, 0x7a, 0x24,
            0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b, 0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9,
            0x8c, 0xd2, 0x30, 0x6e, 0xed, 0xb3, 0x51, 0x0f, 0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd,
            0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92, 0xd3, 0x8d, 0x6f, 0x31, 0xb2, 0xec, 0x0e, 0x50,
            0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c, 0x6d, 0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee,
            0x32, 0x6c, 0x8e, 0xd0, 0x53, 0x0d, 0xef, 0xb1, 0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73,
            0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49, 0x08, 0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b,
            0x57, 0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4, 0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16,
            0xe9, 0xb7, 0x55, 0x0b, 0x88, 0xd6, 0x34, 0x6a, 0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8,
            0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7, 0xb6, 0xe8, 0x0a, 0x54, 0xd7, 0x89, 0x6b, 0x35,
        ]

        self.CRC16_TABLE = [
            0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
            0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
            0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
            0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
            0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
            0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
            0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
            0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
            0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
            0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
            0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
            0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
            0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
            0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
            0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
            0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
            0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
            0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
            0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
            0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
            0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
            0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
            0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
            0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
            0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
            0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
            0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
            0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
            0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
            0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
            0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
            0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78,
        ]


    # crc校验
    def get_crc16_check_byte(self,data):
        crc = 0xffff
        for byte in data:
            crc = ((crc >> 8) ^ self.CRC16_TABLE[(crc ^ byte & 0xff) & 0xff])
        return crc

    def get_crc8_check_byte(self,data):
        crc = 0xff
        for byte in data:
            crc_index = crc ^ byte
            crc = self.CRC8_TABLE[crc_index]
        return crc
    # 串口初始化
    def serial_init(self):

        if not self.enabled:
            print('通信串口已禁用，Sender 不打开串口')
            return None

        port_list = list(serial.tools.list_ports.comports())

        if len(port_list) == 0:
            print('无可用串口!')
            # 停止程序
            exit()
        else:
            for i in range(0, len(port_list)):
                print(port_list[i])

        ser = serial.Serial(self.port, self.bps, timeout=self.timex)

        return ser

    # 帧尾获取 , 传入整包数据 , 通用方法
    def get_frame_tail(self , tx_buff):

        CRC16 = self.get_crc16_check_byte(tx_buff)
        frame_tail = bytes([CRC16 & 0x00ff, (CRC16 & 0xff00) >> 8])

        return frame_tail


    # 帧头获取 , 通用方法
    def get_frame_header(self,data_length=14):
        # frame header
        # +--------+--------------+--------+--------+
        # | SOF    | data_length  | seq    | CRC8   |
        # +--------+--------------+--------+--------+
        # | 1-byte | 2-byte       | 1-byte | 1-byte |
        # +--------+--------------+--------+--------+
        #
        # SOF: start of frame, a fixed byte at the beginnig of frame header
        #      the value is 0xA5 in v1.4 protocol
        #      单字节，接收的数据应为 A5
        #
        # data_length: 不包含 cmd_id 和 frame_tail
        #              (construct of a frame:
        #                   [ frame_head  | cmd_id  | data    | frame_tail ]
        #                     5-byte        2-byte    n-byte    2-byte
        #              )
        #              双字节，以data_length=14(10)为例，接收时表现为 0E 00，低字节在前
        #
        # seq: packet sequence number
        #      not used now?
        #
        # struct.py
        # https://docs.python.org/3.8/library/struct.html#struct-format-strings
        # format: struct member type -> size
        # 'H': unsigned short -> 2 bytes
        # 'h': short -> 2 bytes
        # 'B': unsigned char -> 1 byte
        # 'f': float -> 4 bytes
        # 'fff' or '3f' means continuous 3 float values
        # 'I': unsigned int -> 4 bytes
        # global SOF, seq

        _SOF = self.SOF
        _data_length = struct.pack('H', data_length)
        # print(_data_length)
        _seq = struct.pack('B', self.seq)
        _frame_header =  _SOF + _data_length + _seq
        frame_header = _frame_header + struct.pack('B', self.get_crc8_check_byte(_frame_header))
        return frame_header

    # 创建新地方车辆小地图信息，注意单位从m变为了cm
    # 新小地图信息包格式：
    '''
        雷达可通过常规链路向己方所有选手端发送对方机器人的坐标数据，该位置会在己方选手端小地图显示。
    表 3-2 命令码 ID：0x0305
    字节偏移量 大小 说明
    0 2 英雄机器人 x 位置坐标，单位：cm
    2 2 英雄机器人 y 位置坐标，单位：cm
    4 2 工程机器人 x 位置坐标，单位：cm
    6 2 工程机器人 y 位置坐标，单位：cm
    8 2 3 号步兵机器人 x 位置坐标，单位：cm
    10 2 3 号步兵机器人 y 位置坐标，单位：cm
    12 2 4 号步兵机器人 x 位置坐标，单位：cm
    14 2 4 号步兵机器人 y 位置坐标，单位：cm
    16 2 5 号步兵机器人 x 位置坐标，单位：cm
    18 2 5 号步兵机器人 y 位置坐标，单位：cm
    20 2 哨兵机器人 x 位置坐标，单位：cm
    22 2 哨兵机器人 y 位置坐标，单位：cm
    备注
    当 x、y 超出边界时显示在对应边缘处，
    当 x、y 均为 0 时，视为未发送此机器人坐标。
    typedef _packed struct
    {
    uint16_t hero_position_x;
    uint16_t hero_position_y;
    uint16_t engineer_position_x;
    uint16_t engineer_position_y;
    uint16_t infantry_3_position_x;
    uint16_t infantry_3_position_y;
    uint16_t infantry_4_position_x;
    uint16_t infantry_4_position_y;
    uint16_t infantry_5_position_x;
    uint16_t infantry_5_position_y;
    uint16_t sentry_position_x;
    uint16_t sentry_position_y;
    } map_robot_data_t;
    '''
    def generate_all_location_info(self , infos): # for info in infos 有6个, info是一个list，里面为[x , y] , 单位为m，需要转换为cm并以uint16_t形式打包
        cmd_id = struct.pack('H', 0x0305)
        # 初始化data
        data = b''
        for info in infos:
            x = int(info[0]*100)
            y = int(info[1]*100)
            # print("map ",x,y)
            data += struct.pack('HH', x, y) # 单位转换为cm

        data_len = len(data)
        # print("data len ",data_len)

        frame_head = self.get_frame_header(data_len)

        tx_buff = frame_head + cmd_id + data

        frame_tail = self.get_frame_tail(tx_buff)

        tx_buff += frame_tail

        return tx_buff


    # 发送Info , 通用方法
    def send_info(self,tx_buff):
        if not self.enabled or self.ser is None:
            return
        self.ser.write(tx_buff)

    # 发送所有车辆位置信息 , 调用方法
    def send_all_location(self , infos):
        self.logger.log(f"Preparing to send all location info: {', '.join(f'[ {x:.2f}, {y:.2f} ]' for x, y in infos)}")
        tx_buff = self.generate_all_location_info(infos)
        # print("send all location",tx_buff)

        self.send_info(tx_buff)



    # 构建机器人交互数据，主 cmd_id 为 0x0301。
    '''
字节偏移量 大小    说明             备注
0         2    子内容 ID   需为开放的子内容 ID
2         2    发送者 ID   需与自身 ID 匹配，ID 编号详见附录
4         2    接收者 ID   需为规则允许的多机通讯接收者，若接收者为选手端，则仅可发送至发送者对应的选手端，仅限己方通信，ID 编号详见附录，
6         x    内容数据段    x 最大为 112
    子内容 ID  内容数据段长度     功能说明
    0x0200~0x02FF     x≤112      机器人之间通信
    typedef _packed struct{
    uint16_t data_cmd_id;
    uint16_t sender_id;
    uint16_t receiver_id;
    uint8_t user_data[x];
    }robot_interaction_data_t;
    '''
    # 组织哨兵赛场坐标信息，传入 6 辆敌方车辆的 [x, y]。
    # car_info in car_infos: [[x , y]]
    def generate_sentinel_field_info(self , car_infos):
        cmd_id = struct.pack('H', 0x0301)
        data_cmd_id = struct.pack('H', 0x0202)
        sender_id = struct.pack('H', self.my_id)
        receiver_id = struct.pack('H', self.my_sentinel_id)
        data = data_cmd_id + sender_id + receiver_id
        for car_info in car_infos:
            data += struct.pack('2f', car_info[0], car_info[1])
            
        data_len = len(data)
        frame_head = self.get_frame_header(data_len)

        tx_buff = frame_head + cmd_id + data

        frame_tail = self.get_frame_tail(tx_buff)

        tx_buff += frame_tail

        return tx_buff

    # 发送哨兵赛场坐标信息。
    def send_sentinel_field_info(self , car_infos):
        tx_buff = self.generate_sentinel_field_info(car_infos)
        self.logger.log(f"Generated sentinel field info: {' '.join(f'0x{b:02x}' for b in tx_buff)}")

        self.send_info(tx_buff)

    # 组织雷达自主决策信息。请求序号开局为0，每次请求只能增加1，不能回退。
    def generate_double_effect_analysis_result_info(self, request_id=0, analysis_result='000000'):
        if not 0 <= int(request_id) <= 0xFF:
            raise ValueError("double effect request_id must fit in uint8")
        cmd_id = struct.pack('H', 0x0301)
        data_cmd_id = struct.pack('H', 0x0121)
        sender_id = struct.pack('H', self.my_id)
        receiver_id = struct.pack('H', 0x8080)
        request_data = struct.pack('B', int(request_id))
        password_cmd = struct.pack('B', 2)
        password = analysis_result.encode('ascii', 'ignore')[:6].ljust(6, b'\x00')

        data = data_cmd_id + sender_id + receiver_id + request_data + password_cmd + password

        data_len = len(data)
        frame_head = self.get_frame_header(data_len)

        tx_buff = frame_head + cmd_id + data

        frame_tail = self.get_frame_tail(tx_buff)

        tx_buff += frame_tail

        return tx_buff

    def send_double_effect_analysis_result_info(self, request_id=0, analysis_result='000000'):
        tx_buff = self.generate_double_effect_analysis_result_info(request_id, analysis_result)
        # print("send double",tx_buff)
        # print("send double length",len(tx_buff))
        self.logger.log(f"Send self decision info: {' '.join(f'0x{b:02x}' for b in tx_buff)}")

        self.send_info(tx_buff)

    # 向哨兵发送敌方血量信息，中间方法，初始值均为100，每个血量值为两个字节
    def generate_enemy_HP_info(self, enemy_hp_list):
        cmd_id = struct.pack('H', 0x0301)
        data_cmd_id = struct.pack('H', 0x0205)
        sender_id = struct.pack('H', self.my_id)
        receiver_id = struct.pack('H', self.my_sentinel_id)
        data = data_cmd_id + sender_id + receiver_id
        for hp in enemy_hp_list:
            data += struct.pack('H', hp)
        data_len = len(data)
        frame_head = self.get_frame_header(data_len)

        tx_buff = frame_head + cmd_id + data

        frame_tail = self.get_frame_tail(tx_buff)

        tx_buff += frame_tail

        return tx_buff
    
    def send_enemy_HP_info(self, enemy_hp_list):
        tx_buff = self.generate_enemy_HP_info(enemy_hp_list)
        self.send_info(tx_buff)

    # 干扰波等级数据帧格式
    def generate_interferance_level_info(self, level):
        cmd_id = struct.pack('B', 0xAA) + struct.pack('B', 0x55)
        data = struct.pack('B', level)
        tx_buff = cmd_id + data
        frame_tail = struct.pack('B', 0x0D) + struct.pack('B', 0x0A)
        tx_buff += frame_tail

        return tx_buff
    
    def send_interferance_level_info(self, level):
        tx_buff = self.generate_interferance_level_info(level)
        self.udp_sender.send_value(tx_buff)
