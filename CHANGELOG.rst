^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package rcf_standard_hardware
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.2 (2026-11-16)
------------------
* Fix/joint defaule keep pos (`#14 <https://github.com/robot-control-frame/rcf_ros2/issues/14>`_)
  * 在没有控制器接管的情况下，关节默认锁位。
  * 修复没设置模式的bug
  * 修复没有覆盖控制模式的bug
  ---------
  Co-authored-by: 廖伦稼 <liaolunjia@gmail.com>
* Feat/rm rcf driver control modes (`#13 <https://github.com/robot-control-frame/rcf_ros2/issues/13>`_)
  * 删除驱动端冗余的control_mode
  * 优化imu超时检测逻辑
  * format
  ---------
  Co-authored-by: 廖伦稼 <liaolunjia@gmail.com>
* Contributors: 廖伦稼

0.0.1 (2026-11-13)
------------------
* 基于伺服的 RCF 机器人 ros2_control system 硬件接口，含 topic 版 IMU 硬件组件
* Contributors: 廖伦稼
