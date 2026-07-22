# 部署视图

生产进程运行在 Raspberry Pi 4B。串口 I/O 由 Bridge 的有界线程拥有；
ROS executor 不执行阻塞串口操作。固件运行在 STM32F407，并以 Upper v3
HELLO 身份、capability mask、session/sequence ACK 和 200 ms watchdog
形成独立安全边界。

发布证据必须绑定上位机 commit、下位机 commit、硬件 revision、配置
SHA-256、标定版本、地图 hash、HIL 与实车报告。
