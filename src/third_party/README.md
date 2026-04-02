# third_party

本目录用于收口当前工作空间需要直接参与构建的第三方源码依赖。

## 当前包含

- `robot_localization`
- `geographic_info`

## 为什么放在这里

- 当前环境不依赖系统 `apt` 安装即可完成 `robot_localization` 相关构建
- 第三方依赖的来源、版本和本地兼容修改可以和主仓库一起管理
- 便于在 `robot_bringup` 中直接声明依赖并参与工作空间构建

## 使用约定

- 不在根目录再额外复制同名依赖
- 只在这里保存需要参与本工程构建的 vendored 源码
- 如果对第三方源码做了本地兼容修改，必须同步更新根目录和 `docs/` 中的架构说明

## 维护边界

- `third_party/` 属于源码层，不属于 `build/ install/ log/` 这类生成目录
- 这里的包可以被 `colcon build` 直接扫描
- 如果未来改回系统包依赖，应先更新 `README.md`、`docs/02-系统架构.md` 和相关 `package.xml`
