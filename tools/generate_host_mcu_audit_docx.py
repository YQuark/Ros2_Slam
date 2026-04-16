#!/usr/bin/env python3
"""Generate the host/MCU static audit report as a minimal Word document."""

from __future__ import annotations

from datetime import UTC, datetime
from pathlib import Path
from xml.sax.saxutils import escape
import zipfile


OUT = Path("/mnt/d/document/projects/上位机与下位机代码全面审计说明.docx")


def para(text: str = "", style: str | None = None) -> str:
    style_xml = f'<w:pPr><w:pStyle w:val="{style}"/></w:pPr>' if style else ""
    lines = text.split("\n") or [""]
    runs = []
    for idx, line in enumerate(lines):
        if idx:
            runs.append("<w:r><w:br/></w:r>")
        runs.append(f"<w:r><w:t>{escape(line)}</w:t></w:r>")
    return f"<w:p>{style_xml}{''.join(runs)}</w:p>"


def bullet(text: str) -> str:
    return para("• " + text, "BodyText")


def heading(text: str, level: int = 1) -> str:
    return para(text, f"Heading{level}")


def document_xml(body: str) -> str:
    return f'''<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<w:document xmlns:w="http://schemas.openxmlformats.org/wordprocessingml/2006/main"
 xmlns:r="http://schemas.openxmlformats.org/officeDocument/2006/relationships">
  <w:body>
    {body}
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
    <w:pPr><w:spacing w:after="80" w:line="300" w:lineRule="auto"/></w:pPr>
    <w:rPr><w:rFonts w:ascii="Arial" w:eastAsia="Microsoft YaHei"/><w:sz w:val="21"/></w:rPr>
  </w:style>
</w:styles>
'''


def build_body() -> str:
    parts: list[str] = []
    now = datetime.now().strftime("%Y-%m-%d %H:%M")
    parts.append(para("上位机与下位机代码全面审计说明", "Title"))
    parts.append(para(f"生成时间：{now}", "BodyText"))
    parts.append(para("审计方式：静态代码审计。当前 /mnt/ros2_ws 是远程机器人工作区的 SMB 挂载，本地未执行构建、运行、ROS topic/node/tf 检查或硬件相关程序。所有运行结论需在远程机器人环境验证。", "BodyText"))

    parts.append(heading("一、审计范围", 1))
    for item in [
        "上位机自研代码：/mnt/ros2_ws/src/stm32_robot_bridge、/mnt/ros2_ws/src/robot_bringup、/mnt/ros2_ws/launch_scripts。",
        "上位机集成边界：ydlidar_ros2_driver、ros2_astra_camera、third_party 下游依赖；对 vendor 代码主要审计启动、配置、边界和局部改动风险。",
        "下位机 STM32：/mnt/d/Document/Work/projects/Clion/Core，重点覆盖 Drivers、main、usart、tim、gpio、adc 等。",
        "下位机 ESP01S：/mnt/d/Document/Work/projects/Clion/ESP01S/ESP01S/ESP01S.ino。",
        "主审计代码量约 12,374 行，不包含大量第三方 SDK 与生成缓存。"
    ]:
        parts.append(bullet(item))

    parts.append(heading("二、总体结论", 1))
    for item in [
        "系统的链路分层比较清晰：ROS2 上位机通过 stm32_robot_bridge 以二进制帧协议连接 STM32；ESP01S 作为 Wi-Fi/Web 控制入口复用相同协议语义；robot_bringup 负责传感器、底盘、EKF、SLAM、导航编排。",
        "主要风险集中在跨端协议契约不完全统一、里程计降级策略过于乐观、模式切换 ACK 不能表达真实执行结果、ESP01S Web 控制缺少访问控制，以及若干启动脚本对本机运行环境和进程范围假设较强。",
        "下位机基础安全保护有明显设计：STM32 侧有独立看门狗、电池欠压保护、命令超时、链路 active 超时、UART 错误恢复和编码器故障制动。但这些保护状态没有完整向上位机形成闭环。",
        "本次没有发现会在静态层面直接导致必然编译失败的自研代码语法问题；但没有本地构建验证，仍需远程机器人环境验证。"
    ]:
        parts.append(bullet(item))

    findings = [
        ("高", "上位机与下位机速度/角速度契约不一致",
         "上位机 bridge_node.py 默认 max_linear=0.50、max_angular=1.50，并将 cmd_vel 按该上限归一化为 Q15 后发送。launch/stm32_bridge.launch.py 与 robot_bringup/base.launch.py 也沿用该默认值。STM32 robot_config.h 中 WHEEL_RADIUS_M=0.0325、ENCODER_CPR_X4=2340、MOTOR_MAX_CPS=13800，折算轮边速度约 1.20 m/s，小车理论角速度上限约 19.3 rad/s。",
         "同一个 Q15 值在两端对应的物理量不同，会导致控制输出、反馈速度和里程计尺度不一致。表现可能是 ROS 侧认为速度较小，但 STM32 控制器按自身电机上限解释后输出过大；或 STM32 状态反馈被上位机按 0.50/1.50 反缩放，造成 odom/ekf 偏差。",
         "建立单一物理契约：建议协议直接传 SI 单位定点值，或将 STM32 的线速度/角速度物理上限作为状态能力上报，上位机启动时读取并校验。至少应统一 max_linear/max_angular 与 robot_config.h 派生值，并在 README/参数文件中写明。"),
        ("高", "上位机 odom 在 STM32 状态过期时回退到命令速度",
         "bridge_node.py 的 publish_odom 先使用 current_vx/current_wz，也就是最近发送的命令速度；只有在状态新鲜且反馈有效时才改用 feedback_vx/feedback_wz。",
         "串口异常、STM32 未回状态、底盘故障或运动被下位机安全逻辑抑制时，/odom 仍可能继续积分命令速度，污染 map/odom、EKF、SLAM 和导航闭环。该问题比单纯缺少状态更危险，因为它制造了看似连续但实际错误的位姿。",
         "将 /odom 默认改为反馈优先；状态过期后停止积分或显著增大协方差，并发布诊断状态。若确需命令速度 fallback，应作为显式参数关闭默认，并在状态中暴露降级原因。"),
        ("高", "模式切换 ACK/OK 不能代表真实模式已经切换",
         "STM32 RobotControl_SetMode 在编码器故障时直接 return，函数无返回值。link_proto.c 的 SET_MODE 处理调用该函数后仍发送 ACK；pc_link.c 的 MODE 命令也在调用后直接输出 OK。ESP01S /mode 只等待 ACK。",
         "上位机、串口控制台或 Web 控制端可能看到切换成功，但下位机实际保持原模式。这会造成 DRIVE/RAW 被拒绝、空转等待，或操作者误判安全状态。",
         "让 RobotControl_SetMode 返回 bool/错误码；协议 ACK 中包含 actual_mode 和 reason；ESP 与 PC 文本命令以实际模式为准返回。"),
        ("高", "ESP01S Web 控制入口缺少认证且 AP 密码硬编码",
         "ESP01S.ino 中 AP_SSID/AP_PASS 固定写在代码内；/cmd、/raw、/mode、/wifi_set、/status 等 HTTP 接口均可直接访问，未见 token、会话、来源限制或 CSRF 防护。",
         "任何连接到 AP 或同网段的设备都可以控制底盘、切换模式或修改 Wi-Fi 配置。对移动机器人而言，这是直接的安全边界问题。",
         "至少增加本机唯一强密码、配置化 token、危险操作二次确认、/wifi_set 认证和访问日志。生产现场建议默认关闭 AP 控制或限制到维护模式。"),
        ("中", "PC 文本 DRIVE 在 IDLE 模式下可能返回 OK 但不会执行",
         "pc_link.c 的 DRIVE 命令只拒绝 MODE_OPEN_LOOP；若当前是 MODE_IDLE，它会接受并设置 PC 命令，然后返回 OK DRIVE。RobotControl_Update 在 IDLE 模式下不会按闭环命令驱动电机。",
         "串口调试者看到 OK 后可能误以为底盘应该运动，排障方向会被误导；自动化脚本也可能把 OK 当作控制闭环成功。",
         "DRIVE 应要求 actual_mode == MODE_CLOSED_LOOP；否则返回 ERR MODE 并附当前模式。"),
        ("中", "TF 发布职责依赖启动组合，单独启动存在重复 odom->base_link 风险",
         "system.launch.py 在 EKF 模式下将 stm32_bridge 的 publish_tf 设为 false，这是正确的；但 stm32_bridge.launch.py 与 base.launch.py 默认 publish_tf=true，ekf_base.yaml 也 publish_tf=true。",
         "如果用户绕过 system.launch.py 单独组合 base.launch.py 与 robot_localization，可能同时有 bridge 和 EKF 发布 odom->base_link，导致 TF 抖动或树冲突。",
         "把 base.launch.py 的 publish_tf 默认改为 false，或在文档中明确：使用 EKF 时只能由 EKF 发布 odom->base_link。"),
        ("中", "ESP01S 状态接口没有完整暴露 STM32 扩展状态",
         "STM32 status payload 已包含 yaw、raw accel、cmd_semantics、raw 命令等字段；ROS bridge 会解析 yaw 和扩展状态。ESP01S parseStatusExtra 只解析到 cmd_semantics/raw 等部分字段，/status JSON 没有暴露 yaw、raw accel 和 IMU 有效性细节。",
         "现场通过 Web 页面排障时，看不到姿态和原始惯性状态，容易误判为底盘控制问题或串口问题。",
         "扩展 ESP status JSON，至少加入 yaw、imu_valid、raw_ax/raw_ay/raw_az、last_ack_cmd、link_error 等字段。"),
        ("中", "YDLidar LaserScan 未填充未命中点的 invalid 值",
         "ydlidar_ros2_driver_node.cpp 创建 ranges/intensities 后只 resize，随后只对 scan.points 命中的 index 写入 range/intensity。未命中的 ranges 默认是 0，而配置中存在 invalid_range_is_inf 语义。",
         "LaserScan 中 0 距离可能被下游误认为近距离障碍，影响建图、避障或导航成本地图。",
         "发布前根据配置将 ranges 初始化为 inf 或 NaN，再填充有效点。"),
        ("中", "停止脚本进程匹配范围过宽",
         "launch_scripts/stop_all.sh 在停止 tmux 会话后使用 pkill -f 匹配 ros2 run、ros2 launch、rviz2、rtabmap、slam_toolbox 等进程名。",
         "如果同一机器人或开发环境中有其他 ROS 会话，脚本可能误杀非本系统进程。该脚本适合单任务机器人，但不适合共享环境。",
         "优先按 tmux session、launch PID 文件、ROS namespace 或进程组停止；保留 --force 选项再执行宽匹配。"),
        ("中", "地图保存脚本使用固定 /tmp/map_probe.txt",
         "save_map.sh 检查 /map 数据时将 ros2 topic echo 输出写入 /tmp/map_probe.txt。",
         "并发或多用户执行时可能互相覆盖；旧文件也可能干扰排障判断。虽然这不是核心控制风险，但会降低运维可靠性。",
         "改用 mktemp 并在 exit trap 中清理。"),
        ("低", "main.c 中 System_ShowStatus 存在 return 后死代码",
         "main.c 的 System_ShowStatus 前半段通过 OLED_Printf_Ascii 输出状态后直接 return，后续旧版 OLED 绘制逻辑不可达。",
         "不会直接影响运行，但增加维护成本，也可能让后续修改者误以为两套显示逻辑都有效。",
         "删除不可达旧代码，或用编译期开关明确选择显示实现。"),
        ("低", "第三方和生成目录混入大量修改状态",
         "git status 显示 YDLidar-SDK、ros2_astra_camera、third_party/robot_localization、third_party/geographic_info 等 vendor 目录大量修改，同时自研目录下有 __pycache__、build/install/log/COLCON_IGNORE 等生成文件。",
         "审计、回滚和升级第三方依赖时边界不清，容易把 vendor 代码误当作业务变更，或在合并时丢失本地修补。",
         "将第三方依赖固定到子模块、vendor patch 或明确版本目录；清理生成缓存并把 build/install/log 保持在忽略规则中。")
    ]

    parts.append(heading("三、主要发现", 1))
    for idx, (sev, title, evidence, impact, recommendation) in enumerate(findings, start=1):
        parts.append(heading(f"{idx}. [{sev}] {title}", 2))
        parts.append(para(f"证据：{evidence}", "BodyText"))
        parts.append(para(f"影响：{impact}", "BodyText"))
        parts.append(para(f"建议：{recommendation}", "BodyText"))

    parts.append(heading("四、跨端协议与控制链路观察", 1))
    for item in [
        "二进制协议具备 COBS 编码、CRC、序号、ACK、重复请求缓存、TX 队列和 UART storm 保护，基础链路设计完整。",
        "当前协议的核心缺口不是帧格式，而是语义 ACK：ACK 表示“收到并处理函数返回”，但不一定表示“执行结果符合请求”。SET_MODE、DRIVE、RAW、状态降级都需要引入实际执行状态。",
        "PS2、PC、ESP 三路控制源在 RobotControl choose_cmd 中做了优先级仲裁，非零命令优先。建议把当前控制源、命令年龄、被拒绝原因也上报给上位机，便于现场判断。"
    ]:
        parts.append(bullet(item))

    parts.append(heading("五、建议修复优先级", 1))
    for item in [
        "P0：统一速度/角速度物理契约；修复 odom 状态过期时仍积分命令速度；修复 SET_MODE/ACK 真实结果表达；给 ESP Web 控制加认证。",
        "P1：修正 PC DRIVE 的模式判断；补齐 ESP status 可观测性；初始化 LaserScan 无效点；约束 TF 发布职责。",
        "P2：收窄 stop_all.sh 的进程匹配范围；save_map.sh 改用 mktemp；清理 main.c 死代码；整理 vendor 与生成目录。"
    ]:
        parts.append(bullet(item))

    parts.append(heading("六、远程验证建议", 1))
    parts.append(para("当前目录是挂载环境，不能在本地执行，请在远程机器人通过 SSH 执行。", "BodyText"))
    parts.append(para("建议在远程机器人上执行：\ncd ~/ros2_ws\ncolcon build --packages-select stm32_robot_bridge robot_bringup\nsource install/setup.bash\nros2 launch robot_bringup system.launch.py mode:=mapping use_lidar:=true base_mode:=real", "BodyText"))
    parts.append(para("可选检查：\nros2 topic list\nros2 node list\nros2 topic echo /odom\nros2 topic echo /scan\nros2 topic echo /stm32_bridge/status", "BodyText"))

    parts.append(heading("七、审计限制", 1))
    for item in [
        "未在本地执行构建、启动、topic/node/tf 检查或硬件控制，报告不声明验证通过。",
        "第三方 SDK/驱动未做逐行安全审计，只审计与当前系统集成相关的配置和局部风险。",
        "下位机工程位于 Windows 路径挂载目录，报告基于当前文件内容；若 IDE 工程或生成代码另有配置，需要同步纳入版本管理后再审。"
    ]:
        parts.append(bullet(item))

    return "\n".join(parts)


def write_docx(path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    body = build_body()
    content_types = '''<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">
  <Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/>
  <Default Extension="xml" ContentType="application/xml"/>
  <Override PartName="/word/document.xml" ContentType="application/vnd.openxmlformats-officedocument.wordprocessingml.document.main+xml"/>
  <Override PartName="/word/styles.xml" ContentType="application/vnd.openxmlformats-officedocument.wordprocessingml.styles+xml"/>
  <Override PartName="/docProps/core.xml" ContentType="application/vnd.openxmlformats-package.core-properties+xml"/>
  <Override PartName="/docProps/app.xml" ContentType="application/vnd.openxmlformats-officedocument.extended-properties+xml"/>
</Types>
'''
    rels = '''<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">
  <Relationship Id="rId1" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/officeDocument" Target="word/document.xml"/>
  <Relationship Id="rId2" Type="http://schemas.openxmlformats.org/package/2006/relationships/metadata/core-properties" Target="docProps/core.xml"/>
  <Relationship Id="rId3" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/extended-properties" Target="docProps/app.xml"/>
</Relationships>
'''
    doc_rels = '''<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">
  <Relationship Id="rId1" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/styles" Target="styles.xml"/>
</Relationships>
'''
    stamp = datetime.now(UTC).isoformat().replace("+00:00", "Z")
    core = f'''<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<cp:coreProperties xmlns:cp="http://schemas.openxmlformats.org/package/2006/metadata/core-properties"
 xmlns:dc="http://purl.org/dc/elements/1.1/"
 xmlns:dcterms="http://purl.org/dc/terms/"
 xmlns:dcmitype="http://purl.org/dc/dcmitype/"
 xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance">
  <dc:title>上位机与下位机代码全面审计说明</dc:title>
  <dc:creator>Codex</dc:creator>
  <cp:lastModifiedBy>Codex</cp:lastModifiedBy>
  <dcterms:created xsi:type="dcterms:W3CDTF">{stamp}</dcterms:created>
  <dcterms:modified xsi:type="dcterms:W3CDTF">{stamp}</dcterms:modified>
</cp:coreProperties>
'''
    app = '''<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<Properties xmlns="http://schemas.openxmlformats.org/officeDocument/2006/extended-properties"
 xmlns:vt="http://schemas.openxmlformats.org/officeDocument/2006/docPropsVTypes">
  <Application>Codex OpenXML Generator</Application>
</Properties>
'''
    with zipfile.ZipFile(path, "w", zipfile.ZIP_DEFLATED) as zf:
        zf.writestr("[Content_Types].xml", content_types)
        zf.writestr("_rels/.rels", rels)
        zf.writestr("word/document.xml", document_xml(body))
        zf.writestr("word/styles.xml", STYLES)
        zf.writestr("word/_rels/document.xml.rels", doc_rels)
        zf.writestr("docProps/core.xml", core)
        zf.writestr("docProps/app.xml", app)


if __name__ == "__main__":
    write_docx(OUT)
    print(OUT)
