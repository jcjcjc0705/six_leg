# -*- coding: utf-8 -*-
"""
test / pose_stand

由 Fusion 的 JointSetup 產生於 2026-10-02 18:28。
角度基準是 URDF 的零位，不是 Fusion 的 joint 零位。
"""

# joint 名稱 -> 角度（度）。USD 的角度單位是度，不是弧度。
JOINT_POS_DEG = {
    "base_leg_rf": 0.0000,
    "base_leg_rr": 0.0000,
    "base_leg_lf": 0.0000,
    "base_leg_lr": 0.0000,
    "base_hand_r": 0.0000,
    "base_hand_l": 0.0000,
    "leg_rf1_rf2": -45.0000,
    "leg_rf2_rf3": -45.0000,
    "leg_rf_wheel": 0.0000,
    "leg_rr1_rr2": -45.0000,
    "leg_rr2_rr3": -45.0000,
    "leg_rr_wheel": 0.0000,
    "leg_lf1_lf2": -45.0000,
    "leg_lf2_lf3": -45.0000,
    "leg_lf_wheel": 0.0000,
    "leg_lr1_lr2": -45.0000,
    "leg_lr2_lr3": -45.0000,
    "leg_lr_wheel": 0.0000,
    "hand_r1_r2": -45.0000,
    "hand_r2_r3": -45.0000,
    "hand_r3_gripper_r1": 110.0000,
    "gripper_r1_r2": 0.0000,
    "hand_l1_l2": -45.0000,
    "hand_l2_l3": -45.0000,
    "hand_l3_gripper_l1": 110.0000,
    "gripper_l1_l2": 0.0000,
}

# 給 Isaac Lab 的 ArticulationCfg.InitialStateCfg 用（弧度）：
# joint_pos={
#     "base_leg_rf": 0.000000,
#     "base_leg_rr": 0.000000,
#     "base_leg_lf": 0.000000,
#     "base_leg_lr": 0.000000,
#     "base_hand_r": 0.000000,
#     "base_hand_l": 0.000000,
#     "leg_rf1_rf2": -0.785398,
#     "leg_rf2_rf3": -0.785398,
#     "leg_rf_wheel": 0.000000,
#     "leg_rr1_rr2": -0.785398,
#     "leg_rr2_rr3": -0.785398,
#     "leg_rr_wheel": 0.000000,
#     "leg_lf1_lf2": -0.785398,
#     "leg_lf2_lf3": -0.785398,
#     "leg_lf_wheel": 0.000000,
#     "leg_lr1_lr2": -0.785398,
#     "leg_lr2_lr3": -0.785398,
#     "leg_lr_wheel": 0.000000,
#     "hand_r1_r2": -0.785398,
#     "hand_r2_r3": -0.785398,
#     "hand_r3_gripper_r1": 1.919862,
#     "gripper_r1_r2": 0.000000,
#     "hand_l1_l2": -0.785398,
#     "hand_l2_l3": -0.785398,
#     "hand_l3_gripper_l1": 1.919862,
#     "gripper_l1_l2": 0.000000,
# }

import omni.usd
from pxr import UsdPhysics, PhysxSchema

stage = omni.usd.get_context().get_stage()

found = {}
for prim in stage.Traverse():
    name = prim.GetName()
    if name in JOINT_POS_DEG and (prim.IsA(UsdPhysics.RevoluteJoint)
                                  or prim.IsA(UsdPhysics.PrismaticJoint)):
        found[name] = prim

ok_state, ok_drive, missing, errors = 0, 0, [], []

for name, deg in JOINT_POS_DEG.items():
    prim = found.get(name)
    if prim is None:
        missing.append(name)
        continue

    axis_token = 'angular' if prim.IsA(UsdPhysics.RevoluteJoint) else 'linear'

    # 1) 關節狀態 —— 決定 reset 之後回到哪裡
    try:
        state = PhysxSchema.JointStateAPI.Apply(prim, axis_token)
        state.CreatePositionAttr().Set(float(deg))
        ok_state += 1
    except Exception as e:
        errors.append('%s state: %s' % (name, e))

    # 2) drive 目標 —— 位置控制時追蹤的目標
    try:
        drive = UsdPhysics.DriveAPI.Get(prim, axis_token)
        if not drive:
            drive = UsdPhysics.DriveAPI.Apply(prim, axis_token)
        drive.CreateTargetPositionAttr().Set(float(deg))
        ok_drive += 1
    except Exception as e:
        errors.append('%s drive: %s' % (name, e))

print('[pose_stand] joint 表 %d 個，stage 找到 %d 個'
      % (len(JOINT_POS_DEG), len(found)))
print('  關節狀態設定 %d 個、drive 目標設定 %d 個' % (ok_state, ok_drive))
if missing:
    print('  stage 裡找不到 %d 個: %s' % (len(missing), ', '.join(missing)))
    print('  -> 確認 URDF 已匯入，而且 joint 名稱沒有被改過')
for e in errors[:10]:
    print('  錯誤: %s' % e)
