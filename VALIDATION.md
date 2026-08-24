# 验证记录

验证日期：2026-08-24

## 精确实验版本

E2E01、AD04-R1、AD04-R2、AD04-R3 与 CMP01-SLAMONLY 的源码清单均与
冻结前工作区完全匹配。

## 构建与测试

- `semantic_perception` 与 `semantic_mapping_bringup` 均成功完成构建。
- `semantic_perception` 包级测试：24 项通过、1 项跳过、0 项功能失败。

## 精确版本保留的问题

为了保持正式实验源码哈希不变，精确实验标签保留两处格式提示和
`setup.py` 中缺少的 pytest 集成声明。这些非功能性问题在后续
`release-clean-20260824` 标签中修复。

## 清理发布版本

`release-clean-20260824` 在精确实验提交之后仅修复两处格式问题及 pytest
测试集成配置。清理版本已重新通过包级 pytest、colcon build 和 colcon test。
精确实验源码仍由 `experiment-exact-20260816` 标签永久保留。
