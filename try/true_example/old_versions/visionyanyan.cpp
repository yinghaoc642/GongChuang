/*
 * 粗加工区视觉定位 + 三物料放置测量模式。
 *
 * 使用方法：
 *   1. 保留 src/yanyanversionnew.inc；
 *   2. 将本文件复制并覆盖 src/main.cpp；
 *   3. 同时给 MaixCAM 部署 vision/yanyanview5.py；
 *   4. 上电前人工确认 M5 旧坐标 0°、M6 最短、M7 最高；
 *   5. 料盘槽 0/1/2 各装一个物料，车按正式朝向放在粗加工三圆前；
 *   6. 上电初始化完成后单击 PB9。
 *
 * 程序只执行：
 *   底盘锁向并冻结 -> 看1号端点 -> 看3号端点 -> 中点计算2号
 *   -> 动态计算三组M5/M6 -> 槽0/1/2末段慢速放入1/2/3号圆
 *   -> 机械臂和料盘回行驶姿态 -> 停在MEASURE。
 * 不扫码、不跑路线、不把三个物料重新抓回。
 *
 * 圆内印刷的数字 1/2/3 必须保留，它们是比赛真实视觉条件。
 */
#define GONGCHUANG_VISION_YANYAN_TEST 1
#include "../src/yanyanversionnew.inc"
