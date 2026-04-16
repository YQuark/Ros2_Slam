#!/usr/bin/env python3
"""Generate the updated static re-audit report as a Word document."""

from __future__ import annotations

from datetime import UTC, datetime
from pathlib import Path
from xml.sax.saxutils import escape
import zipfile


OUT = Path("/mnt/d/document/projects/上位机与下位机代码全面审计说明.docx")


def p(text: str = "", style: str = "BodyText") -> str:
    runs = []
    for idx, line in enumerate(text.split("\n")):
        if idx:
            runs.append("<w:r><w:br/></w:r>")
        runs.append(f"<w:r><w:t>{escape(line)}</w:t></w:r>")
    return f'<w:p><w:pPr><w:pStyle w:val="{style}"/></w:pPr>{"".join(runs)}</w:p>'


def h(text: str, level: int = 1) -> str:
    return p(text, f"Heading{level}")


def b(text: str) -> str:
    return p("• " + text)


def body_xml(content: str) -> str:
    return f'''<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<w:document xmlns:w="http://schemas.openxmlformats.org/wordprocessingml/2006/main">
  <w:body>
    {content}
    <w:sectPr>
      <w:pgSz w:w="11906" w:h="16838"/>
      <w:pgMar w:top="1440" w:right="1440" w:bottom="1440" w:left="1440" w:header="720" w:footer="720" w:gutter="0"/>
    </w:sectPr>
  </w:body>
</w:document>
'''


STYLES = '''<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<w:styles xmlns:w="http://schemas.openxmlformats.org/wordprocessingml/2006/main">
  <w:style w:type="paragraph" w:default="1" w:styleId="Normal">
    <w:name w:val="Normal"/>
    <w:rPr><w:rFonts w:ascii="Arial" w:eastAsia="Microsoft YaHei"/><w:sz w:val="21"/></w:rPr>
  </w:style>
  <w:style w:type="paragraph" w:styleId="Title">
    <w:name w:val="Title"/>
    <w:pPr><w:jc w:val="center"/><w:spacing w:after="240"/></w:pPr>
    <w:rPr><w:rFonts w:ascii="Arial" w:eastAsia="Microsoft YaHei"/><w:b/><w:sz w:val="36"/></w:rPr>
  </w:style>
  <w:style w:type="paragraph" w:styleId="Heading1">
    <w:name w:val="heading 1"/>
    <w:pPr><w:spacing w:before="360" w:after="160"/></w:pPr>
    <w:rPr><w:rFonts w:ascii="Arial" w:eastAsia="Microsoft YaHei"/><w:b/><w:sz w:val="30"/></w:rPr>
  </w:style>
  <w:style w:type="paragraph" w:styleId="Heading2">
    <w:name w:val="heading 2"/>
    <w:pPr><w:spacing w:before="240" w:after="120"/></w:pPr>
    <w:rPr><w:rFonts w:ascii="Arial" w:eastAsia="Microsoft YaHei"/><w:b/><w:sz w:val="25"/></w:rPr>
  </w:style>
  <w:style w:type="paragraph" w:styleId="BodyText">
    <w:name w:val="Body Text"/>
    <w:pPr><w:spacing w:after="90" w:line="300" w:lineRule="auto"/></w:pPr>
    <w:rPr><w:rFonts w:ascii="Arial" w:eastAsia="Microsoft YaHei"/><w:sz w:val="21"/></w:rPr>
  </w:style>
</w:styles>
'''


def build_report() -> str:
    now = datetime.now().strftime("%Y-%m-%d %H:%M")
    parts: list[str] = [
        p("上位机与下位机代码全面审计说明（二次审计更新版）", "Title"),
        p(f"生成时间：{now}"),
        p("审计方式：静态代码审计。当前 /mnt/ros2_ws 是远程机器人工作区的 SMB 挂载，本地未执行 colcon build、cmake、make、ninja、ros2 launch/run、topic/node/tf 检查或任何硬件相关程序。所有运行结论需在远程机器人环境验证。"),
        h("一、审计范围"),
    ]

    for item in [
        "上位机自研代码：/mnt/ros2_ws/src/stm32_robot_bridge、/mnt/ros2_ws/src/robot_bringup、/mnt/ros2_ws/launch_scripts。",
        "上位机集成边界：/mnt/ros2_ws/src/ydlidar_ros2_driver，重点审计 LaserScan 发布和参数语义。",
        "下位机 STM32：/mnt/d/Document/Work/projects/Clion/Core，重点审计 robot_control、link_proto、pc_link、robot_config、main、串口和安全保护。",
        "下位机 ESP01S：/mnt/d/Document/Work/projects/Clion/ESP01S/ESP01S/ESP01S.ino，重点审计 Web 控制、Wi-Fi、状态 JSON 和串口协议。",
    ]:
        parts.append(b(item))

    parts.append(h("二、本轮改动后的总体判断"))
    for item in [
        "上一轮审计指出的速度量纲不一致已经明显改善：bridge_node.py、stm32_bridge.launch.py、base.launch.py、system.launch.py 默认 max_linear/max_angular 已改为 1.204277 / 19.268435，与 STM32 robot_config.h 中 MAX_CPS、DIFF_WHEEL_RADIUS_M、DIFF_TRACK_WIDTH_M、ENCODER_COUNTS_PER_REV 派生值一致。",
        "上一轮审计指出的 odom 在状态过期时积分 /cmd_vel 的问题已经改善：bridge_node.py 当前在 status_timeout 之外将 odom_vx/odom_wz 置 0，并提高 twist covariance，不再把命令速度当作测量。",
        "上位机状态解析增强：bridge_node.py 支持 45 字节扩展状态，解析 yaw_est、raw accel、cmd_semantics、raw_left/raw_right；IMU 线加速度增加 raw/valid/invalid covariance 参数。",
        "启动编排更偏向真实底盘：system.launch.py 默认 base_mode=real、base_fusion_mode=ekf、base_imu_enabled=true，并在 EKF 模式下关闭 bridge TF，由 EKF 发布 odom->base_link。",
        "同时引入新风险：robot.sh 和 system.launch.py 默认 base_port 从 auto 变为 /dev/ttyUSB0。由于 resolve_base_port 只有 port=auto 才会做协议级探测，固定默认值会绕过 detect_base_port 与 excluded_ports 保护。"
    ]:
        parts.append(b(item))

    findings = [
        ("高", "固定 /dev/ttyUSB0 默认底盘串口绕过协议级探测",
         "证据：robot.sh 中 mapping/navigation/base/system 默认 base_port=${BASE_PORT:-/dev/ttyUSB0}；resolve_base_port 仅在 port=auto 时调用 detect_base_port.sh。system.launch.py 默认 base_port=/dev/ttyUSB0。bridge_node.py 的 excluded_ports 也只在 auto 扫描候选串口时发挥作用。",
         "影响：若 CP210x 枚举顺序变化，或者雷达/底盘串口互换，底盘桥接可能直接打开错误设备。轻则桥接持续失败，重则误占用雷达串口，导致雷达不可用或底盘无法控制。这个风险是本轮默认值修改后新增的。",
         "建议：将 robot.sh 和 system.launch.py 的底盘默认值恢复为 auto；需要固定端口时通过 BASE_PORT 或 --base-port 显式指定。保留 ROBOT_LIDAR_PORT_HINT/excluded_ports 作为 auto 探测时的排除条件。"),
        ("高", "SET_MODE ACK 仍不能证明真实模式切换成功",
         "证据：RobotControl_SetMode 在编码器故障时直接 return，函数无返回值；link_proto.c 的 SET_MODE 调用 RobotControl_SetMode 后直接 ACK；pc_link.c 的 MODE 命令也直接返回 OK MODE <请求值>；ESP /mode 只等待 ACK。",
         "影响：上位机、串口终端或 Web 页面可能显示切换成功，但下位机实际仍保持 IDLE 或原模式。后续 DRIVE/RAW 被拒绝或不动作时，操作者会被 ACK/OK 误导。",
         "建议：让 RobotControl_SetMode 返回结果和失败原因；SET_MODE 响应中回传 actual_mode/reason；PC 和 ESP 端按 actual_mode 返回，而不是按请求值返回。"),
        ("高", "ESP01S Web 控制仍无认证且 AP 密码硬编码",
         "证据：ESP01S.ino 中 AP_SSID=ESP01S_CTRL、AP_PASS=12345678；/cmd、/raw、/mode、/wifi_set、/status、/health、/wifi_status 均注册为无认证 HTTP 路由。",
         "影响：连接到 AP 或同网段的任意设备可控制底盘、切换模式或改写 Wi-Fi 配置。这是移动机器人现场安全边界问题。",
         "建议：加入维护 token、唯一强密码、危险操作二次确认和 /wifi_set 认证；生产模式默认关闭 AP 控制或仅允许物理维护开关开启。"),
        ("中", "PC 文本 DRIVE 在 IDLE 下仍可能返回 OK",
         "证据：pc_link.c 的 DRIVE/CTRL 只拒绝 MODE_OPEN_LOOP；MODE_IDLE 时会 RobotControl_SetCmd_PC 并返回 OK DRIVE。RobotControl_Update 在 MODE_IDLE 分支清空输出并 return。",
         "影响：串口调试或上层脚本看到 OK 后可能误以为闭环命令已执行，实际底盘不会运动，增加排障成本。",
         "建议：DRIVE 必须要求 RobotControl_GetMode()==MODE_CLOSED_LOOP；否则返回 ERR DRIVE requires MODE CLOSED actual=<mode>。"),
        ("中", "YDLidar invalid_range_is_inf 参数读取后未应用",
         "证据：ydlidar_ros2_driver_node.cpp 声明并读取 invalid_range_is_inf；随后发布 LaserScan 时 ranges/intensities 只是 resize，未命中的 bin 默认 0。robot_bringup/config/ydlidar_X2_mapping.yaml 设置 invalid_range_is_inf=true。",
         "影响：未命中激光点可能以 0 距离进入 /scan，下游 costmap/SLAM 可能将其解释为近距离障碍或异常扫描，尤其与 fixed_resolution=true 搭配时更明显。",
         "建议：resize 后根据 invalid_range_is_inf 将 ranges 初始化为 inf 或 0，并显式初始化 intensities；再填充有效点。"),
        ("中", "ESP 状态 JSON 仍缺少 yaw/raw accel/IMU accel 有效性",
         "证据：ESP parseStatusExtra 对 45 字节状态只解析 cmdSemantics/rawLeft/rawRight；buildStatusJson 未暴露 yaw_est、raw_ax/raw_ay/raw_az、imu_accel_valid。",
         "影响：Web 端无法判断姿态融合、原始加速度和 IMU 加速度有效性，现场排障时仍需依赖 ROS bridge 或串口日志。",
         "建议：ESP StatusData 和 JSON 增加 yaw_deg、raw_accel_mg、imu_accel_valid、enc_fault_mask、actual_mode_age 等字段。"),
        ("中", "robot.sh 的 yaw 来源开关只有开启路径，缺少关闭路径",
         "证据：robot.sh 默认 base_use_status_yaw=true/use_status_yaw=true，参数 --base-use-status-yaw 也只是设置 true，没有 --no-base-use-status-yaw。",
         "影响：如果下位机 yaw_est 在某个现场状态下不稳定，运维入口无法通过统一脚本快速回退到桥接端 w_est 积分，只能绕过脚本直接传 launch 参数。",
         "建议：增加 --no-base-use-status-yaw，并在日志中明确当前使用下位机 yaw_est 或桥接积分。"),
        ("中", "stop_all.sh 仍使用宽泛 pkill 模式",
         "证据：stop_all.sh 先枚举 ros2 node list，再执行 pkill -f 'ros2 run'、'ros2 launch'、rviz2、rtabmap、slam_toolbox 等宽匹配。",
         "影响：共享机器人或同机多任务场景下可能误杀非本项目 ROS 进程。",
         "建议：优先按 tmux session、PID 文件、namespace 或进程组停止；把宽匹配放到 --force。"),
        ("低", "save_map.sh 仍使用固定 /tmp/map_probe.txt",
         "证据：save_map.sh 检查 /map 数据时将输出重定向到 /tmp/map_probe.txt。",
         "影响：并发或多用户执行时可能互相覆盖，旧文件也会干扰排障。",
         "建议：使用 mktemp，并通过 trap 清理临时文件。"),
        ("低", "配置/launch 文件出现大量 100755 模式变化",
         "证据：git diff 显示多项 yaml、launch、package.xml、README 和图片文件只有文件模式或零内容变化。",
         "影响：不会直接改变运行行为，但会增加审计、合并和回滚噪声，掩盖真正代码变化。",
         "建议：恢复非脚本文件为 100644，仅 shell/python 可执行入口保持 100755。"),
        ("低", "main.c 仍保留 return 后不可达显示代码",
         "证据：main.c 的 System_ShowStatus 前半段 return 后仍保留旧 OLED 显示逻辑。",
         "影响：运行风险低，但维护者容易误判显示路径。",
         "建议：删除不可达旧代码，或用编译期开关明确选择显示实现。"),
    ]

    parts.append(h("三、主要发现"))
    for idx, (sev, title, evidence, impact, advice) in enumerate(findings, 1):
        parts.append(h(f"{idx}. [{sev}] {title}", 2))
        parts.append(p(evidence))
        parts.append(p(impact))
        parts.append(p(advice))

    parts.append(h("四、已改善项"))
    for item in [
        "速度/角速度上限从 0.50/1.50 改为 1.204277/19.268435，和 STM32 物理参数派生值一致。",
        "上位机 /odom 不再在 STM32 状态 stale 时继续积分 /cmd_vel，降低 SLAM/EKF 被假里程计污染的风险。",
        "桥接节点扩展状态解析覆盖 45 字节 payload，raw 命令、cmd_semantics、yaw、raw accel 等状态已进入 ROS 侧。",
        "system.launch.py 在 EKF 模式下关闭 bridge TF，避免默认编排中的 odom->base_link 双发布。",
        "雷达默认配置从通用 ydlidar X2 参数转到 robot_bringup/config/ydlidar_X2_mapping.yaml，并明确 /dev/ttyUSB1、fixed_resolution、invalid_range_is_inf。"
    ]:
        parts.append(b(item))

    parts.append(h("五、修复优先级"))
    for item in [
        "P0：恢复底盘串口默认 auto 探测；修复 SET_MODE 真实结果 ACK；为 ESP Web 控制加入认证。",
        "P1：修复 PC DRIVE 的 IDLE OK；应用 YDLidar invalid_range_is_inf；补齐 ESP status JSON；增加 --no-base-use-status-yaw。",
        "P2：收窄 stop_all.sh；save_map.sh 使用 mktemp；清理文件模式噪声和 main.c 死代码。"
    ]:
        parts.append(b(item))

    parts.append(h("六、远程验证要求"))
    parts.append(p("当前目录是挂载环境，不能在本地执行，请在远程机器人通过 SSH 执行。"))
    parts.append(p("建议命令：\ncd ~/ros2_ws\ncolcon build --packages-select stm32_robot_bridge robot_bringup ydlidar_ros2_driver\nsource install/setup.bash\nros2 launch robot_bringup system.launch.py mode:=mapping mapping_source:=lidar base_mode:=real base_port:=auto"))
    parts.append(p("可选检查：\nros2 topic list\nros2 node list\nros2 topic echo /odom\nros2 topic echo /imu/data\nros2 topic echo /scan\nros2 topic echo /stm32_bridge/status"))

    parts.append(h("七、审计限制"))
    for item in [
        "本报告未声明运行验证通过，只基于当前文件内容、差异和静态证据。",
        "未逐行审计 vendor/third_party 全量源码，只覆盖与当前系统集成直接相关的边界。",
        "下位机工程不在当前 Git 工作区内，无法用同一份 git diff 精确区分每次修改，只按当前文件内容审计。"
    ]:
        parts.append(b(item))

    return "\n".join(parts)


def write_docx() -> None:
    OUT.parent.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now(UTC).isoformat().replace("+00:00", "Z")
    core = f'''<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<cp:coreProperties xmlns:cp="http://schemas.openxmlformats.org/package/2006/metadata/core-properties"
 xmlns:dc="http://purl.org/dc/elements/1.1/"
 xmlns:dcterms="http://purl.org/dc/terms/"
 xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance">
  <dc:title>上位机与下位机代码全面审计说明（二次审计更新版）</dc:title>
  <dc:creator>Codex</dc:creator>
  <cp:lastModifiedBy>Codex</cp:lastModifiedBy>
  <dcterms:created xsi:type="dcterms:W3CDTF">{stamp}</dcterms:created>
  <dcterms:modified xsi:type="dcterms:W3CDTF">{stamp}</dcterms:modified>
</cp:coreProperties>
'''
    with zipfile.ZipFile(OUT, "w", zipfile.ZIP_DEFLATED) as zf:
        zf.writestr("[Content_Types].xml", '''<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">
  <Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/>
  <Default Extension="xml" ContentType="application/xml"/>
  <Override PartName="/word/document.xml" ContentType="application/vnd.openxmlformats-officedocument.wordprocessingml.document.main+xml"/>
  <Override PartName="/word/styles.xml" ContentType="application/vnd.openxmlformats-officedocument.wordprocessingml.styles+xml"/>
  <Override PartName="/docProps/core.xml" ContentType="application/vnd.openxmlformats-package.core-properties+xml"/>
  <Override PartName="/docProps/app.xml" ContentType="application/vnd.openxmlformats-officedocument.extended-properties+xml"/>
</Types>''')
        zf.writestr("_rels/.rels", '''<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">
  <Relationship Id="rId1" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/officeDocument" Target="word/document.xml"/>
  <Relationship Id="rId2" Type="http://schemas.openxmlformats.org/package/2006/relationships/metadata/core-properties" Target="docProps/core.xml"/>
  <Relationship Id="rId3" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/extended-properties" Target="docProps/app.xml"/>
</Relationships>''')
        zf.writestr("word/_rels/document.xml.rels", '''<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">
  <Relationship Id="rId1" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/styles" Target="styles.xml"/>
</Relationships>''')
        zf.writestr("word/document.xml", body_xml(build_report()))
        zf.writestr("word/styles.xml", STYLES)
        zf.writestr("docProps/core.xml", core)
        zf.writestr("docProps/app.xml", '''<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<Properties xmlns="http://schemas.openxmlformats.org/officeDocument/2006/extended-properties">
  <Application>Codex OpenXML Generator</Application>
</Properties>''')


if __name__ == "__main__":
    write_docx()
    print(OUT)
