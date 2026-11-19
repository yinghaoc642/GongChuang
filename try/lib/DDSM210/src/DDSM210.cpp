#include "DDSM210.h"

/**
 * @file DDSM210.cpp
 * @brief DDSM_CTRL 的 10 字节协议组帧、CRC 校验与反馈解析实现。
 *
 * 实现约定：
 *
 * - 所有多字节整数均按高字节在前（大端）传输；
 * - 有符号 16 位量以补码表示；
 * - 每帧 DATA[0]～DATA[8] 参与 CRC-8/MAXIM，DATA[9] 存放 CRC；
 * - 通信采用“一问一答”，发送一台电机的命令后立即等待这一台的回复。
 *
 * DDSM210 可直接使用 TTL UART。DDSM115 的物理层是 RS485，本类只负责
 * 通过 HardwareSerial 收发协议字节，不包含 RS485 DE/RE 方向控制。
 *
 * 这里保留上游库的原有控制流程，只补充说明，不改变协议或超时行为。
 */

/**
 * @brief 初始化协议状态和公用发送缓冲区。
 *
 * packet_length 是 const 成员，必须在初始化列表中赋值。默认使用 DDSM210
 * 协议；pSerial 留空，等待使用者在 setup() 中绑定实际硬件串口。
 *
 * packet_move 的初始内容是一帧发给 ID 1 的示例速度命令。后续每个发送函数
 * 都会覆盖其所需字段，再重新计算 CRC。
 */
DDSM_CTRL::DDSM_CTRL()
    : packet_length(10), ddsm_type(TYPE_DDSM210), get_info_flag(false),
      pSerial(nullptr) {
  // DATA[0]：目标电机 ID。
  packet_move[0] = 0x01;

  // DATA[1]：0x64 表示常规控制命令。
  packet_move[1] = 0x64;

  // DATA[2..3]：示例目标 0xFFCE，即有符号数 -50。
  packet_move[2] = 0xff;
  packet_move[3] = 0xce;

  // DATA[4..8]：反馈选择、加速时间、刹车及保留字段的初始值。
  packet_move[4] = 0x00;
  packet_move[5] = 0x00;
  packet_move[6] = 0x00;
  packet_move[7] = 0x00;
  packet_move[8] = 0x00;

  // DATA[9]：上述 9 字节对应的 CRC-8/MAXIM。
  packet_move[9] = 0xda;
}

/**
 * @brief 对一个字节执行 CRC-8/MAXIM 的逐位更新。
 *
 * 标准多项式为 x^8+x^5+x^4+1；由于这里按最低位优先右移，代码使用其
 * 反射形式 0x8C。计算一整帧时，初值取 0，并连续调用本函数 9 次。
 */
uint8_t DDSM_CTRL::crc8_update(uint8_t crc, uint8_t data) {
  // 先把新字节异或进当前 CRC。
  crc ^= data;

  // 每次处理最低位，共处理 8 位。
  for (uint8_t i = 0; i < 8; ++i) {
    if (crc & 0x01) {
      // 最低位为 1：右移后再异或反射多项式。
      crc = (crc >> 1) ^ 0x8c;
    } else {
      // 最低位为 0：只右移。
      crc >>= 1;
    }
  }
  return crc;
}

/**
 * @brief 选择 DDSM115 或 DDSM210 协议格式。
 *
 * 同时兼容直观型号数字 115/210 和头文件中的枚举式宏 1/2。
 * 非法输入不修改当前 ddsm_type。
 */
int DDSM_CTRL::set_ddsm_type(int inputType) {
  if (inputType == 115 || inputType == TYPE_DDSM115) {
    ddsm_type = TYPE_DDSM115;
    return TYPE_DDSM115;
  }
  if (inputType == 210 || inputType == TYPE_DDSM210) {
    ddsm_type = TYPE_DDSM210;
    return TYPE_DDSM210;
  }
  return -1;
}

/**
 * @brief 读空当前串口接收 FIFO。
 *
 * 该操作常用于初始化阶段重新建立 10 字节帧边界。这里故意不调用 flush()：
 * Arduino 的 flush() 处理的是等待发送完成，并不负责清空接收缓冲区。
 * 本函数也不会等待仍在传输途中的字节到达；在正常通信中调用可能丢掉合法
 * 反馈，应主要用于初始化或明确的通信重同步。
 *
 * @warning pSerial 必须已经绑定；本库沿用原实现，不做空指针检查。
 */
void DDSM_CTRL::clear_ddsm_buffer() {
  while (pSerial->available() > 0) {
    pSerial->read();
  }
}

/**
 * @brief 接收并解析一帧 DDSM210 反馈。
 *
 * 接收流程为：等待至少 10 字节 → 一次读出完整帧 → 校验 CRC → 根据
 * DATA[1] 的反馈类型拆包。函数采用忙等待，最多占用当前任务 TIME0UT ms。
 * 它固定按每 10 字节切帧，不搜索帧头；一旦接收流错位，本次通常会因 CRC
 * 错误失败，但函数不会自动滑动窗口寻找下一帧。
 *
 * @return CRC 正确时返回 1；等待超时或 CRC 不匹配时返回 -1。
 */
int DDSM_CTRL::ddsm210_fb() {
  // 记录开始等待的毫秒计数。无符号减法可自然处理 millis() 回绕。
  unsigned long startTime = millis();

  // 协议固定一帧 10 字节；不足一帧时持续等待。
  while (pSerial->available() < 10) {
    if (millis() - startTime >= TIME0UT) {
      return -1;
    }
  }

  // available() 已确认至少 10 字节，正常情况下 readBytes() 会立即读满。
  uint8_t data[10];
  pSerial->readBytes(data, 10);

  // DATA[0]～DATA[8] 参与计算，结果必须等于 DATA[9]。
  uint8_t crc = 0;
  for (size_t i = 0; i < packet_length - 1; ++i) {
    crc = crc8_update(crc, data[i]);
  }
  if (crc != data[9]) {
    return -1;
  }

  // DATA[1] 同时用于区分常规控制反馈和附加信息反馈。
  int feedback_type = data[1];
  if (feedback_type == 0x64) {
    /*
     * 0x64 常规控制反馈：
     * DATA[2..3]：本实现按有符号速度值解析；
     * DATA[4..5]：本实现按有符号总线电流值解析；
     * DATA[6]   ：电机实际返回的加速时间参数；
     * DATA[7]   ：温度（℃）；
     * DATA[8]   ：故障位掩码。
     */

    // 合并高、低字节，再手工把 16 位补码转换成 int。
    speed_data = (data[2] << 8) | data[3];
    if (speed_data & 0x8000) {
      speed_data = -(0x10000 - speed_data);
    }

    // 电流同样是有符号 16 位补码。
    current = (data[4] << 8) | data[5];
    if (current & 0x8000) {
      current = -(0x10000 - current);
    }

    acceleration_time = data[6];
    temperature = data[7];
    fault_code = data[8];
  } else if (feedback_type == 0x74) {
    /*
     * 0x74 附加信息反馈：
     * DATA[2..5]：有符号 32 位累计圈数，高字节在前；
     * DATA[6..7]：单圈位置，0～65535 对应 0～360°；
     * DATA[8]   ：故障位掩码。
     */

    // 先转为无符号数再移位，避免有符号左移带来的未定义/实现相关行为。
    mileage =
        (int32_t)((uint32_t)data[2] << 24 | (uint32_t)data[3] << 16 |
                  (uint32_t)data[4] << 8 | (uint32_t)data[5]);
    ddsm_pos = (data[6] << 8) | data[7];
    fault_code = data[8];
  }

  /*
   * 注意：原实现只以“长度足够且 CRC 正确”作为成功标准。
   * 它不验证 DATA[0] 是否为期望 ID；遇到未知 DATA[1] 也仍返回 1。
   */
  return 1;
}

/**
 * @brief 接收并解析一帧 DDSM115 反馈。
 *
 * DDSM115 的 DATA[1] 是当前模式，不像 DDSM210 那样作为 0x64/0x74
 * 反馈类型。DATA[6..7] 的含义由 get_info_flag 决定。
 *
 * 若等待超时或 CRC 错误，函数会在清除 get_info_flag 之前返回。因此，
 * ddsm_get_info() 失败后到来的下一帧普通反馈可能被当作附加信息解析。
 */
int DDSM_CTRL::ddsm115_fb() {
  unsigned long startTime = millis();

  // 等待完整的 10 字节反馈；等待过程会阻塞当前任务。
  while (pSerial->available() < 10) {
    if (millis() - startTime >= TIME0UT) {
      return -1;
    }
  }

  uint8_t data[10];
  pSerial->readBytes(data, 10);

  // 校验 DATA[0]～DATA[8]，接收 CRC 位于 DATA[9]。
  uint8_t crc = 0;
  for (size_t i = 0; i < packet_length - 1; ++i) {
    crc = crc8_update(crc, data[i]);
  }
  if (crc != data[9]) {
    return -1;
  }

  /*
   * DDSM115 常规字段：
   * DATA[0]   ：电机 ID（本实现未保存、未核对）；
   * DATA[1]   ：控制模式；
   * DATA[2..3]：有符号转矩电流；
   * DATA[4..5]：有符号速度；
   * DATA[8]   ：故障位掩码。
   */
  ddsm_mode = data[1];

  // DATA[2..3] 按大端有符号 16 位补码解码。
  ddsm_torque = (data[2] << 8) | data[3];
  if (ddsm_torque & 0x8000) {
    ddsm_torque = -(0x10000 - ddsm_torque);
  }

  // DATA[4..5] 按大端有符号 16 位补码解码。
  speed_data = (data[4] << 8) | data[5];
  if (speed_data & 0x8000) {
    speed_data = -(0x10000 - speed_data);
  }

  if (get_info_flag) {
    /*
     * ddsm_get_info() 置位后，附加反馈把 DATA[6] 解释为绕组温度，
     * DATA[7] 解释为 8 位位置（0～255 对应 0～360°）。
     */
    get_info_flag = false;
    temperature = data[6];
    ddsm_u8 = data[7];
    fault_code = data[8];
  } else {
    // 普通控制反馈把 DATA[6..7] 解释为 16 位位置原始值。
    ddsm_pos = (data[6] << 8) | data[7];
    fault_code = data[8];
  }
  return 1;
}

/**
 * @brief 发送广播 ID 查询帧并读取一帧回答。
 *
 * 查询帧是协议规定的固定值：
 * C8 64 00 00 00 00 00 00 00 DE。
 *
 * @warning 总线上有多台电机时会同时应答，造成帧冲突；查询时应只保留一台。
 */
int DDSM_CTRL::ddsm_id_check() {
  // DATA[0]=0xC8 是 ID 查询的广播地址，DATA[1]=0x64 是固定命令字。
  packet_move[0] = 0xC8;
  packet_move[1] = 0x64;
  packet_move[2] = 0x00;
  packet_move[3] = 0x00;
  packet_move[4] = 0x00;
  packet_move[5] = 0x00;
  packet_move[6] = 0x00;
  packet_move[7] = 0x00;
  packet_move[8] = 0x00;

  // 该固定帧 DATA[0..8] 的 CRC 预计算结果为 0xDE。
  packet_move[9] = 0xDE;
  pSerial->write(packet_move, packet_length);

  // 查询采用同步等待；超过 TIME0UT ms 仍不足一帧即失败。
  unsigned long startTime = millis();
  while (pSerial->available() < 10) {
    if (millis() - startTime >= TIME0UT) {
      return -1;
    }
  }

  uint8_t data[10];
  pSerial->readBytes(data, 10);

  // 对应答进行 CRC 校验，但不检查其命令/模式字段。
  uint8_t crc = 0;
  for (size_t i = 0; i < packet_length - 1; ++i) {
    crc = crc8_update(crc, data[i]);
  }
  if (crc != data[9]) {
    return -1;
  }

  // 按协议，应答 DATA[0] 就是当前电机 ID。
  return data[0];
}

/**
 * @brief 连续发送 5 次 ID 设置帧，并尝试核对新 ID。
 *
 * 设置帧格式：
 * AA 55 53 [新ID] 00 00 00 00 00 [CRC]。
 *
 * 这是会写入电机配置的操作。必须确保总线上只有一台电机，否则所有在线
 * 电机可能收到相同的新 ID。
 *
 * @note 本实现不区分 DDSM115/DDSM210，统一生成 CRC 形式的 ID 设置帧；
 *       不同固件的具体要求应以对应电机手册为准。
 * @note 本实现不检查 ID 合法范围或总线上是否已经存在同名 ID。
 */
int DDSM_CTRL::ddsm_change_id(uint8_t id) {
  // 前三个字节是协议规定的 ID 设置帧头。
  packet_move[0] = 0xAA;
  packet_move[1] = 0x55;
  packet_move[2] = 0x53;

  // DATA[3] 携带要写入的新 ID。
  packet_move[3] = id;
  packet_move[4] = 0x00;
  packet_move[5] = 0x00;
  packet_move[6] = 0x00;
  packet_move[7] = 0x00;
  packet_move[8] = 0x00;

  // ID 不同会改变 CRC，因此这里动态计算 DATA[9]。
  uint8_t crc = 0;
  for (size_t i = 0; i < packet_length - 1; ++i) {
    crc = crc8_update(crc, packet_move[i]);
  }
  packet_move[9] = crc;

  // 协议要求连续收到 5 帧后才执行 ID 设置。
  for (int i = 0; i < 5; i++) {
    pSerial->write(packet_move, packet_length);
    delay(TIME_BETWEEN_CMD);
  }

  /*
   * 原实现先调用 ddsm_id_check()；该函数会发送查询、等待并消耗一帧反馈，
   * 但这里没有使用它的返回值。
   */
  ddsm_id_check();

  /*
   * 随后原实现又等待第二个 10 字节帧并将其作为验证结果。
   * 若设备对一次查询只返回一帧，第二次等待会超时；这里保留原有行为。
   */
  unsigned long startTime = millis();
  while (pSerial->available() < 10) {
    if (millis() - startTime >= TIME0UT) {
      return -1;
    }
  }

  uint8_t data[10];
  pSerial->readBytes(data, 10);

  // 验证第二帧 CRC。
  crc = 0;
  for (size_t i = 0; i < packet_length - 1; ++i) {
    crc = crc8_update(crc, data[i]);
  }
  if (crc != data[9]) {
    return -1;
  }

  // 第二帧 DATA[0] 必须与请求的新 ID 相同。
  uint8_t returnedId = data[0];
  return id == returnedId ? returnedId : -1;
}

/**
 * @brief 按当前型号组装并发送模式切换帧。
 *
 * DDSM115 和 DDSM210 的模式字段位置以及末字节含义不同，所以必须先正确
 * 调用 set_ddsm_type()。本函数不等待应答；若固件主动返回确认帧，该帧会
 * 残留在接收缓冲区，可能干扰下一次同步接收。
 */
void DDSM_CTRL::ddsm_change_mode(uint8_t id, uint8_t mode) {
  if (ddsm_type == TYPE_DDSM115) {
    /*
     * DDSM115：
     * DATA[0]=ID，DATA[1]=0xA0，DATA[2..8]=0，DATA[9]=模式码。
     * 其模式帧末字节直接存 mode，不使用 DDSM210 的 CRC 布局。
     */
    packet_move[0] = id;
    packet_move[1] = 0xA0;
    packet_move[2] = 0x00;
    packet_move[3] = 0x00;
    packet_move[4] = 0x00;
    packet_move[5] = 0x00;
    packet_move[6] = 0x00;
    packet_move[7] = 0x00;
    packet_move[8] = 0x00;
    packet_move[9] = mode;
  } else if (ddsm_type == TYPE_DDSM210) {
    /*
     * DDSM210：
     * DATA[0]=ID，DATA[1]=0xA0，DATA[2]=模式码，
     * DATA[3..8]=0，DATA[9]=CRC。
     */
    packet_move[0] = id;
    packet_move[1] = 0xA0;
    packet_move[2] = mode;
    packet_move[3] = 0x00;
    packet_move[4] = 0x00;
    packet_move[5] = 0x00;
    packet_move[6] = 0x00;
    packet_move[7] = 0x00;
    packet_move[8] = 0x00;

    // DDSM210 模式帧仍使用 CRC-8/MAXIM。
    uint8_t crc = 0;
    for (size_t i = 0; i < packet_length - 1; ++i) {
      crc = crc8_update(crc, packet_move[i]);
    }
    packet_move[9] = crc;
  }

  // 只发送，不 flush、不等待反馈；调用方需要自行安排模式稳定时间。
  pSerial->write(packet_move, packet_length);
}

/**
 * @brief 发送一帧控制命令，并同步读取 DDSM210 格式反馈。
 *
 * 常规控制帧布局：
 *
 * DATA[0] ID
 * DATA[1] 0x64
 * DATA[2..3] 目标值（大端 16 位）
 * DATA[4..5] 反馈内容选择/默认值（本实现固定为 0）
 * DATA[6] 加速时间参数 act
 * DATA[7] 刹车字段（本实现固定为 0，即不启用 0xFF 电刹）
 * DATA[8] 保留
 * DATA[9] CRC-8/MAXIM
 *
 * 本函数不会限制 cmd 范围。写入帧时只保留低 16 位，超范围参数可能回绕
 * 成完全不同的有符号目标，调用方必须在进入本函数前完成限幅。
 */
void DDSM_CTRL::ddsm_ctrl(uint8_t id, int cmd, uint8_t act) {
  packet_move[0] = id;
  packet_move[1] = 0x64;

  /*
   * cmd 取低 16 位并拆成高、低字节。
   * 对负数右移的行为在常见 ARM GCC 上是算术右移，再与 0xFF 得到补码高字节。
   */
  packet_move[2] = (cmd >> 8) & 0xFF;
  packet_move[3] = cmd & 0xFF;

  /*
   * DDSM210 协议把这里定义为两个反馈内容选择码（1=速度、2=母线电流、
   * 3=位置）。本库保留上游实现，固定写 0、0；后面的 ddsm210_fb() 却仍
   * 假定反馈 DATA[2..3] 是速度、DATA[4..5] 是电流。这一假定依赖电机固件
   * 对选择码 0 的默认处理方式。
   */
  packet_move[4] = 0x00;
  packet_move[5] = 0x00;

  // act 是速度环加速时间参数，不是“是否使能电机”的布尔开关。
  packet_move[6] = act;

  // 协议中 DATA[7]=0xFF 才是电刹；这里 0 表示不刹车。
  packet_move[7] = 0x00;

  // packet_move[8] 在构造和其他组帧函数中保持为保留值 0。

  // 每次目标、ID 或 act 改变后，都必须重新计算 CRC。
  uint8_t crc = 0;
  for (size_t i = 0; i < packet_length - 1; ++i) {
    crc = crc8_update(crc, packet_move[i]);
  }
  packet_move[9] = crc;

  // 写入整帧；flush() 会等待硬件把发送缓冲区中的字节发送完毕。
  pSerial->write(packet_move, packet_length);
  pSerial->flush();

  /*
   * DDSM210 是一问一答协议，因此发送后立刻接收，防止下一台电机的反馈
   * 与当前反馈混在一起。
   *
   * 注意：即使 ddsm_type 选择为 DDSM115，原实现这里仍固定调用
   * ddsm210_fb()，而不是 ddsm115_fb()。
   */
  if (ddsm210_fb() == -1) {
    /*
     * 超时或 CRC 错误时，用目标值代替反馈值，保证上层数组仍有数值。
     * 因而 speed_data == cmd 并不代表电机确实达到了目标速度。
     */
    speed_data = cmd;
  }
}

/**
 * @brief 按固定 ID 1、2 依次发送两个目标。
 *
 * 每次 ddsm_ctrl() 都会完成“发送→等待反馈”，所以不会并行发送。保存到
 * speed_data_4[] 的值可能是真实反馈，也可能是反馈失败后的 cmd 替代值。
 */
void DDSM_CTRL::ddsm210_ctrl_2(int cmd1, int cmd2, uint8_t act) {
  ddsm_ctrl(1, cmd1, act);
  speed_data_4[0] = speed_data;
  ddsm_ctrl(2, cmd2, act);
  speed_data_4[1] = speed_data;
}

/**
 * @brief 按固定 ID 1、2、3 依次发送三个目标。
 *
 * 最坏情况下，仅等待反馈就可能累计约 3×TIME0UT ms；实际总耗时还包括
 * 串口发送时间。speed_data_4[3] 不会更新，读取它可能得到旧值或未初始化值。
 */
void DDSM_CTRL::ddsm210_ctrl_3(int cmd1, int cmd2, int cmd3, uint8_t act) {
  ddsm_ctrl(1, cmd1, act);
  speed_data_4[0] = speed_data;
  ddsm_ctrl(2, cmd2, act);
  speed_data_4[1] = speed_data;
  ddsm_ctrl(3, cmd3, act);
  speed_data_4[2] = speed_data;
}

/**
 * @brief 按固定 ID 1、2、3、4 依次发送四个目标。
 *
 * 参数顺序严格对应电机 ID，而不是自动对应车辆方位。麦轮底盘应由上层代码
 * 明确约定 ID 1～4 分别是哪一个轮子，并处理各电机安装方向的正负号。
 *
 * 最坏情况下，仅等待反馈就可能累计约 4×TIME0UT ms。
 */
void DDSM_CTRL::ddsm210_ctrl_4(int cmd1, int cmd2, int cmd3, int cmd4,
                              uint8_t act) {
  ddsm_ctrl(1, cmd1, act);
  speed_data_4[0] = speed_data;
  ddsm_ctrl(2, cmd2, act);
  speed_data_4[1] = speed_data;
  ddsm_ctrl(3, cmd3, act);
  speed_data_4[2] = speed_data;
  ddsm_ctrl(4, cmd4, act);
  speed_data_4[3] = speed_data;
}

/**
 * @brief 发送 0x74 附加信息查询并按当前型号解析回答。
 *
 * 查询帧 DATA[2..8] 全部为 0，DATA[9] 为动态计算的 CRC。DDSM115 需要先
 * 置 get_info_flag，使其解析器把 DATA[6..7] 解释为温度和 U8 位置。
 * 若 DDSM115 接收失败，该标志仍保持 true，可能影响下一帧的解释。
 */
void DDSM_CTRL::ddsm_get_info(uint8_t id) {
  packet_move[0] = id;
  if (ddsm_type == TYPE_DDSM115) {
    // 仅影响下一次成功进入 ddsm115_fb() 的 DATA[6..7] 解释方式。
    get_info_flag = true;
  }

  packet_move[1] = 0x74;
  packet_move[2] = 0x00;
  packet_move[3] = 0x00;
  packet_move[4] = 0x00;
  packet_move[5] = 0x00;
  packet_move[6] = 0x00;
  packet_move[7] = 0x00;
  packet_move[8] = 0x00;

  // 0x74 查询帧也使用 DATA[0..8] 的 CRC-8/MAXIM。
  uint8_t crc = 0;
  for (size_t i = 0; i < packet_length - 1; ++i) {
    crc = crc8_update(crc, packet_move[i]);
  }
  packet_move[9] = crc;
  pSerial->write(packet_move, packet_length);

  /*
   * 立即同步接收。解析函数的返回值在原接口中被忽略，所以调用方无法从本
   * 函数直接判断本次是否成功；失败时公开状态字段可能仍是上一次的值。
   */
  if (ddsm_type == TYPE_DDSM115) {
    ddsm115_fb();
  } else if (ddsm_type == TYPE_DDSM210) {
    ddsm210_fb();
  }
}

/**
 * @brief 发送 0x74 查询并返回经过 ID、命令字和 CRC 校验的结构化结果。
 *
 * 接收端使用 10 字节滑动窗口寻找合法帧，因此即使缓冲区开头存在少量残留
 * 或错位字节，也有机会在超时前重新找到帧边界。遇到 CRC 正确但 ID/命令字
 * 不匹配的完整旧帧时会整帧丢弃，再继续等待目标电机的回答。
 */
bool DDSM_CTRL::ddsm210_get_odometry(
    uint8_t id, DDSM210OdometryFeedback &feedback) {
  if (pSerial == nullptr) {
    return false;
  }

  // 构造 DDSM210 的 0x74 附加信息查询帧。
  uint8_t request[10] = {id, 0x74, 0x00, 0x00, 0x00,
                         0x00, 0x00, 0x00, 0x00, 0x00};
  uint8_t requestCrc = 0;
  for (size_t i = 0; i < packet_length - 1; ++i) {
    requestCrc = crc8_update(requestCrc, request[i]);
  }
  request[9] = requestCrc;

  pSerial->write(request, packet_length);
  pSerial->flush();

  uint8_t window[10];
  size_t bytesInWindow = 0;
  const unsigned long startTime = millis();

  while (millis() - startTime < TIME0UT) {
    while (pSerial->available() > 0) {
      const uint8_t incoming = static_cast<uint8_t>(pSerial->read());

      if (bytesInWindow < packet_length) {
        window[bytesInWindow++] = incoming;
      } else {
        // CRC 错误时滑动一个字节，继续搜索下一个可能的 10 字节帧边界。
        for (size_t i = 0; i < packet_length - 1; ++i) {
          window[i] = window[i + 1];
        }
        window[packet_length - 1] = incoming;
      }

      if (bytesInWindow < packet_length) {
        continue;
      }

      uint8_t responseCrc = 0;
      for (size_t i = 0; i < packet_length - 1; ++i) {
        responseCrc = crc8_update(responseCrc, window[i]);
      }

      if (responseCrc != window[9]) {
        // 当前窗口不是合法帧；保留窗口，下一字节到来后继续滑动搜索。
        continue;
      }

      if (window[0] != id || window[1] != 0x74) {
        /*
         * 这是一个 CRC 正确但不属于本次查询的完整帧。整帧丢弃，避免把它的
         * 尾部与后续帧拼接。
         */
        bytesInWindow = 0;
        continue;
      }

      DDSM210OdometryFeedback parsed;
      parsed.id = window[0];
      parsed.mileage =
          static_cast<int32_t>((static_cast<uint32_t>(window[2]) << 24) |
                               (static_cast<uint32_t>(window[3]) << 16) |
                               (static_cast<uint32_t>(window[4]) << 8) |
                               static_cast<uint32_t>(window[5]));
      parsed.position =
          static_cast<uint16_t>((static_cast<uint16_t>(window[6]) << 8) |
                                static_cast<uint16_t>(window[7]));
      parsed.faultCode = window[8];

      // 只有完整验证成功后才写调用者输出，失败不会破坏上一次有效数据。
      feedback = parsed;

      // 同步更新旧接口公开字段，保持向后兼容。
      mileage = parsed.mileage;
      ddsm_pos = parsed.position;
      fault_code = parsed.faultCode;
      return true;
    }
  }

  return false;
}

/**
 * @brief 给指定 ID 发送零目标并继续执行原实现的反馈读取流程。
 *
 * ddsm_ctrl(id, 0, 0) 会发送零命令并已经调用一次 ddsm210_fb()。随后本函数
 * 又按当前型号调用一次反馈解析。因此在严格“一问一答”的设备上，第二次读取
 * 通常没有对应的新请求，可能等待至 TIME0UT 超时。
 *
 * @note DATA[7] 没有被设置为 0xFF，所以这是“零目标/减速停止”，不是电刹。
 */
void DDSM_CTRL::ddsm_stop(uint8_t id) {
  ddsm_ctrl(id, 0, 0);

  // 保留上游库的第二次反馈读取行为。
  if (ddsm_type == TYPE_DDSM115) {
    ddsm115_fb();
  } else if (ddsm_type == TYPE_DDSM210) {
    ddsm210_fb();
  }
}
