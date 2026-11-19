/*
 * @Author: igcxl acer5502@gmail.com
 * @Date: 2024-07-28 20:41:08
 * @LastEditors: igcxl acer5502@gmail.com
 *
 * @file DDSM210.h
 * @brief DDSM115 / DDSM210 直驱伺服电机通信控制类。
 *
 * 本库按照电机的 10 字节“一问一答”协议工作。使用前必须先完成：
 *
 * 1. 创建并初始化一个 HardwareSerial；
 * 2. 将该串口地址赋给 DDSM_CTRL::pSerial；
 * 3. 通过 set_ddsm_type() 选择电机型号；
 * 4. 等待电机上电稳定后调用 clear_ddsm_buffer() 清理残留数据。
 *
 * 典型初始化方式：
 *
 * @code
 * HardwareSerial motorSerial(PA3, PA2);  // 参数顺序：RX、TX
 * DDSM_CTRL motor;
 *
 * void setup() {
 *   motorSerial.begin(DDSM_BAUDRATE);
 *   motor.pSerial = &motorSerial;
 *   motor.set_ddsm_type(TYPE_DDSM210);
 *   delay(100);
 *   motor.clear_ddsm_buffer();
 * }
 * @endcode
 *
 * @warning 本类不会自动创建或 begin() 串口。pSerial 未赋值时调用通信函数
 *          会解引用空指针。
 * @warning DDSM210 使用 TTL UART；DDSM115 使用 RS485。使用 DDSM115 时仍需
 *          外接合适的 RS485 收发器，并由硬件或其他代码处理收发方向。本类
 *          只读写 HardwareSerial，不控制收发器的 DE/RE 引脚。
 * @warning 本类复用同一个发送缓冲区和同一组公开反馈变量，不支持在中断、
 *          多线程或多个任务中并发调用。
 */
#ifndef _DDSM210_H
#define _DDSM210_H

#include "Arduino.h"

/** 电机协议的固定串口波特率；帧格式为 115200、8 数据位、无校验、1 停止位。 */
#define DDSM_BAUDRATE 115200

/** set_ddsm_type() 使用的 DDSM115 型号标识。 */
#define TYPE_DDSM115 1

/** set_ddsm_type() 使用的 DDSM210 型号标识，也是构造后的默认型号。 */
#define TYPE_DDSM210 2

/** 修改电机 ID 时，连续两帧设置命令之间的等待时间，单位 ms。 */
#define TIME_BETWEEN_CMD 4

/**
 * 等待一帧反馈的最长时间，单位 ms。
 *
 * @note 宏名 TIME0UT 中间是数字 0，这是上游库保留的历史命名。
 */
#define TIME0UT 4

/**
 * @brief 一台 DDSM210 电机的 0x74 里程反馈。
 *
 * 该结构保存协议返回的原始数据，不在驱动层换算成米或弧度：
 *
 * - mileage：上电后的有符号整圈计数；
 * - position：当前单圈内的位置原始值；
 * - faultCode：故障位掩码。
 *
 * 上层里程计应同时使用 mileage 和 position，不能只使用整数圈数。
 */
struct DDSM210OdometryFeedback {
  /** 实际返回该帧的电机 ID。 */
  uint8_t id;

  /** 上电后的有符号累计整圈计数。 */
  int32_t mileage;

  /** 当前单圈位置原始值，量程由电机固件协议决定。 */
  uint16_t position;

  /** 电机故障位掩码。 */
  uint8_t faultCode;

  /** 构造一个清零的里程反馈对象。 */
  DDSM210OdometryFeedback()
      : id(0), mileage(0), position(0), faultCode(0) {}
};

/**
 * @brief DDSM115 / DDSM210 电机协议控制器。
 *
 * 一个对象对应一条通信总线，而不是只对应一台电机。总线上可按 ID 访问多台
 * 电机；ddsm210_ctrl_2/3/4() 则约定依次控制 ID 1、2、3、4。
 *
 * 除 CRC 辅助函数和型号选择函数外，所有通信函数都要求 pSerial 已指向一个
 * 完成 begin() 的 HardwareSerial。
 */
class DDSM_CTRL {
public:
  /**
   * @brief 构造控制器。
   *
   * 默认选择 TYPE_DDSM210，发送缓冲区长度固定为 10 字节，pSerial 初始化
   * 为 nullptr。构造函数不会初始化串口，也不会与电机通信。
   *
   * @note speed_data 等公开反馈字段只有在成功解析相应反馈后才有有效含义。
   */
  DDSM_CTRL();

  /**
   * @brief 丢弃电机串口接收缓冲区中当前已有的全部字节。
   *
   * 适合在电机上电稳定后、开始第一次控制前调用，以免残留字节破坏后续
   * 10 字节帧的对齐。函数只清接收缓冲区，不会发送命令。
   *
   * @note 它只丢弃调用时已经到达的数据，不等待仍在传输途中的字节；若在
   *       正常通信过程中调用，也会丢弃尚未处理的合法反馈。
   * @warning 调用前必须设置 pSerial。
   */
  virtual void clear_ddsm_buffer();

  /**
   * @brief 将一个字节累加到 CRC-8/MAXIM 校验值中。
   *
   * 完整帧 CRC 的通常用法是令初始 crc=0，依次传入 DATA[0]～DATA[8]，
   * 最终结果写入或对比 DATA[9]。
   *
   * @param crc 前一个字节处理后的 CRC；处理首字节时传 0。
   * @param data 本次要加入校验的一个字节。
   * @return 加入 data 后的新 CRC 值。
   */
  virtual uint8_t crc8_update(uint8_t crc, uint8_t data);

  /**
   * @brief 选择后续命令和反馈所采用的电机协议型号。
   *
   * @param inputType 可传 115、TYPE_DDSM115、210 或 TYPE_DDSM210。
   * @return 成功时返回 TYPE_DDSM115 或 TYPE_DDSM210；参数不支持时返回 -1，
   *         并保持原型号不变。
   *
   * @note 本函数只改变本地组帧/解析方式，不会自动识别电机，也不会改变
   *       电机当前的开环、速度环、电流环或位置环模式。
   */
  virtual int set_ddsm_type(int inputType);

  /**
   * @brief 广播查询当前总线上电机的 ID。
   *
   * 函数发送固定 ID 查询帧，最多等待 TIME0UT ms，校验返回帧的 CRC，
   * 然后返回反馈帧 DATA[0] 中的 ID。
   *
   * @return 查询成功时返回 0～255 的电机 ID；超时或 CRC 错误时返回 -1。
   * @warning 查询时总线上应只连接一台电机，否则多台电机同时回复会造成冲突。
   */
  virtual int ddsm_id_check();

  /**
   * @brief 修改当前唯一在线电机的 ID，并尝试查询新 ID 进行确认。
   *
   * ID 设置帧会连续发送 5 次，每次间隔 TIME_BETWEEN_CMD ms。电机 ID
   * 通常会掉电保存。
   *
   * @param id 要写入的新 ID。
   * @return 验证成功时返回新 ID；等待反馈超时、CRC 错误或返回 ID 不一致时
   *         返回 -1。
   * @warning 设置 ID 时总线上必须只有一台电机；同一次上电周期通常只允许
   *          设置一次，具体限制以对应型号协议为准。
   * @warning 本函数不检查 ID 是否位于设备允许范围内，也不检查新 ID 是否
   *          与总线上其他电机重复。
   */
  virtual int ddsm_change_id(uint8_t id);

  /**
   * @brief 切换指定电机的控制模式。
   *
   * DDSM115 模式码：
   * - 1：电流环；
   * - 2：速度环；
   * - 3：位置环。
   *
   * DDSM210 模式码：
   * - 0：开环；
   * - 2：速度环；
   * - 3：位置环。
   *
   * @param id 目标电机 ID。
   * @param mode 对应型号的模式码。
   *
   * @note 本函数只发送模式切换帧，不读取反馈，也不报告切换是否成功。
   *       如果某版固件会返回模式切换应答，该帧会留在接收缓冲区中，后续
   *       接收函数可能把它误当作下一条命令的反馈。
   * @warning 切换模式前应先令电机停止；发送后建议至少等待约 5 ms。
   */
  virtual void ddsm_change_mode(uint8_t id, uint8_t mode);

  /**
   * @brief 向一台电机发送控制目标，并立即尝试接收反馈。
   *
   * cmd 的含义由当前电机型号及已经设置的工作模式决定：
   *
   * DDSM210：
   * - 开环：-32767～32767；
   * - 速度环：-2100～2100，对应 -210.0～210.0 RPM，即 1 单位=0.1 RPM；
   * - 位置环：0～32767 对应 0～360°。
   *
   * DDSM115：
   * - 电流环：-32767～32767 对应约 -8～8 A；
   * - 速度环：命令值单位为 RPM，范围以电机手册为准；
   * - 位置环：0～32767 对应 0～360°。
   *
   * @param id 目标电机 ID。
   * @param cmd 16 位协议目标值；本函数本身不会做范围限制。
   * @param act 速度环的加速时间参数。每增加 1 RPM 所用时间约为
   *            act × 0.1 ms；传 0 时电机按默认值 1 处理。
   *
   * @warning 本函数不做限幅。超出协议范围的 int 最终只发送低 16 位，可能
   *          回绕为符号和大小完全不同的危险命令；调用者必须先限幅。
   * @note 本实现发送后固定调用 ddsm210_fb()。反馈成功时 speed_data 保存
   *       解析值；超时或 CRC 错误时 speed_data 会被写成 cmd，因此
   *       speed_data 不能单独用来证明真实反馈有效。
   * @note 函数会等待串口发送完成并等待反馈，属于阻塞式调用。
   */
  virtual void ddsm_ctrl(uint8_t id, int cmd, uint8_t act);

  /**
   * @brief 依次控制 ID 1～4 的四台 DDSM210 电机。
   *
   * @param cmd1 ID 1 的协议目标值。
   * @param cmd2 ID 2 的协议目标值。
   * @param cmd3 ID 3 的协议目标值。
   * @param cmd4 ID 4 的协议目标值。
   * @param act 四台电机共用的加速时间参数，默认 1。
   *
   * 每次子命令完成后会把 speed_data 复制到 speed_data_4[0..3]。
   * 因为每台电机都可能等待 TIME0UT ms，整次调用的阻塞时间会累加。
   */
  virtual void ddsm210_ctrl_4(int cmd1, int cmd2, int cmd3, int cmd4,
                             uint8_t act = 1);

  /**
   * @brief 依次控制 ID 1～3 的三台 DDSM210 电机。
   *
   * 参数和反馈规则与 ddsm210_ctrl_4() 相同，只更新 speed_data_4[0..2]。
   * speed_data_4[3] 不会被改写，可能仍未初始化或保留旧值。
   */
  virtual void ddsm210_ctrl_3(int cmd1, int cmd2, int cmd3, uint8_t act = 1);

  /**
   * @brief 依次控制 ID 1～2 的两台 DDSM210 电机。
   *
   * 参数和反馈规则与 ddsm210_ctrl_4() 相同，只更新 speed_data_4[0..1]。
   * speed_data_4[2..3] 不会被改写，可能仍未初始化或保留旧值。
   */
  virtual void ddsm210_ctrl_2(int cmd1, int cmd2, uint8_t act = 1);

  /**
   * @brief 请求指定电机的附加状态信息并立即解析反馈。
   *
   * DDSM210 成功后主要更新 mileage、ddsm_pos、fault_code；DDSM115 成功后
   * 主要更新 temperature、ddsm_u8、fault_code，同时也更新模式、电流和速度。
   *
   * @param id 目标电机 ID。
   *
   * @note 本函数没有返回值，内部解析失败时已有字段可能保持旧值。
   */
  virtual void ddsm_get_info(uint8_t id);

  /**
   * @brief 可靠读取一台 DDSM210 的整圈计数、单圈位置和故障码。
   *
   * 本函数发送 0x74 查询帧，并验证反馈的帧长、CRC、ID 和命令字。与
   * ddsm_get_info() 不同，它通过 bool 明确报告本次通信是否成功，也不会
   * 让调用者依赖可能陈旧的共享成员。
   *
   * @param id 要查询的电机 ID。
   * @param feedback 成功时写入本次反馈；失败时保持调用前内容不变。
   * @return 收到 ID 匹配且 CRC 正确的 0x74 帧时返回 true，否则返回 false。
   *
   * @note 函数仍会同步等待最多 TIME0UT ms，属于阻塞式通信。
   * @note 成功后也会同步更新兼容旧接口的 mileage、ddsm_pos 和 fault_code。
   */
  virtual bool
  ddsm210_get_odometry(uint8_t id, DDSM210OdometryFeedback &feedback);

  /**
   * @brief 给指定电机发送零目标。
   *
   * @param id 目标电机 ID。
   *
   * @note 当前实现发送的是 cmd=0、act=0，DATA[7] 仍为 0；这表示零目标，
   *       不是协议中的 0xFF 电刹命令。
   * @note 本函数会额外再次尝试读取反馈，可能再阻塞最多 TIME0UT ms。
   */
  virtual void ddsm_stop(uint8_t id);

  /**
   * @brief 等待并解析一帧 DDSM210 反馈。
   *
   * 支持解析：
   * - 0x64 常规控制反馈：speed_data、current、acceleration_time、
   *   temperature、fault_code；
   * - 0x74 附加信息反馈：mileage、ddsm_pos、fault_code。
   *
   * @return 收到 10 字节且 CRC 正确时返回 1；超时或 CRC 错误时返回 -1。
   *
   * @note 当前实现不检查反馈 ID 是否为预期 ID；CRC 正确但 DATA[1] 不是
   *       0x64/0x74 时也会返回 1，但不会更新上述数据字段。
   * @note 接收器固定每 10 字节切分一帧，不搜索帧头，也不自动恢复字节错位；
   *       出现错帧后可在合适时机调用 clear_ddsm_buffer() 重新同步。
   */
  virtual int ddsm210_fb();

  /**
   * @brief 等待并解析一帧 DDSM115 反馈。
   *
   * 常规控制反馈更新 ddsm_mode、ddsm_torque、speed_data、ddsm_pos 和
   * fault_code；由 ddsm_get_info() 发起的附加反馈则把 DATA[6]/DATA[7]
   * 解析为 temperature 和 ddsm_u8。
   *
   * @return 收到 10 字节且 CRC 正确时返回 1；超时或 CRC 错误时返回 -1。
   *
   * @note 若 ddsm_get_info() 后接收超时或 CRC 错误，get_info_flag 不会清除，
   *       下一帧普通反馈可能被误按附加信息格式解释。
   */
  virtual int ddsm115_fb();

private:
  /** DDSM 协议固定帧长：10 字节。 */
  const size_t packet_length;

  /** 所有命令共同复用的 10 字节发送缓冲区。 */
  uint8_t packet_move[10];

  /** 当前选择的型号，值为 TYPE_DDSM115 或 TYPE_DDSM210。 */
  uint8_t ddsm_type;

  /**
   * DDSM115 解析辅助标志。
   * true 表示下一帧来自 ddsm_get_info()，DATA[6]/DATA[7] 应按温度/U8位置解析。
   */
  bool get_info_flag;

public:
  /**
   * @brief 电机通信串口，由使用者在 setup() 中赋值。
   *
   * 示例：`dc.pSerial = &Serial_M0603C;`
   */
  HardwareSerial *pSerial;

  /**
   * 最近一次解析或替代得到的速度值。
   *
   * DDSM210 速度环中 1 单位通常为 0.1 RPM；DDSM115 速度值单位为 RPM。
   * 如果 ddsm_ctrl() 等待反馈失败，该字段会被目标 cmd 覆盖。
   */
  int speed_data;

  /**
   * ID 1～4 的最近速度值/替代目标，由 ddsm210_ctrl_2/3/4() 更新。
   *
   * 索引 0、1、2、3 分别对应电机 ID 1、2、3、4。反馈失败时相应元素可能
   * 是发送目标而非真实转速。
   */
  int speed_data_4[4];

  /** DDSM210 0x64 反馈 DATA[4..5] 的有符号电流原始值。 */
  int current;

  /**
   * DDSM210 返回的加速时间参数原始值；每单位表示每增加 1 RPM 使用 0.1 ms。
   */
  int acceleration_time;

  /**
   * 最近一次状态反馈中的温度原始字节，正温度时数值单位为 ℃。
   *
   * 当前实现直接把 uint8_t 赋给 int，没有按 int8_t 对负温度做符号扩展。
   */
  int temperature;

  /** DDSM115 当前模式码：1 电流环、2 速度环、3 位置环。 */
  int ddsm_mode;

  /** DDSM115 反馈中的有符号转矩电流原始值。 */
  int ddsm_torque;

  /** DDSM115 附加反馈的 U8 位置值；0～255 对应 0～360°。 */
  int ddsm_u8;

  /** DDSM210 上电后的累计圈数计数；重新上电后清零。 */
  int32_t mileage;

  /**
   * 位置反馈原始值。
   *
   * DDSM210 的 0～65535 对应 0～360°；DDSM115 的具体缩放取决于反馈类型。
   */
  int ddsm_pos;

  /**
   * 最近一次反馈的故障位掩码。
   *
   * DDSM210 常用位：bit1=过流，bit4=过温；DDSM115 的位定义不同，应按
   * 对应型号手册解释。DDSM115 常见定义为 bit0=传感器故障、bit1=过流、
   * bit2=相电流过流、bit3=堵转、bit4=过温。
   */
  int fault_code;
};

#endif
