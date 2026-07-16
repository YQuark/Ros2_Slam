# ADR 0001：Platform API v3

背景：裸 Twist 不能表达 enable、来源、会话和顺序。决策：唯一控制接口为相对名称 `chassis/command`，消息包含 session_id 和 sequence；v0.4.0 只运行 wire v3。后果：beta4/v2 不兼容，必须完成下位机 v3 和 HIL。验证：接口、协议和 ROS 图测试。
