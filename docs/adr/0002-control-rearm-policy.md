# ADR 0002：故障后重新使能

背景：缓存命令可能在故障恢复后复动。决策：断连、STATUS/command timeout、ESTOP、fault 或 supervisor critical 后要求新的 disable→enable 边沿。远程 ESTOP 只能置位不能释放。验证：BridgeCore 单测与 HIL fault injection。
