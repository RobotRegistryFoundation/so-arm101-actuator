---
rcan_version: '3.0'
metadata:
  robot_name: so-arm101-test-rig
  model: so-arm101
  rrn: 'RRN-000000000000'
physics:
  type: arm
  dof: 6
  solver:
    convention: DH
    base_frame:
      up: z
      forward: x
    encoder:
      steps_per_rev: 4096
    gripper:
      joint_id: gripper
      tip_offset_mm:
        - 30
        - 0
        - 0
      open_steps: 1700
      close_steps: 1200
    ik_provider: inhouse-so-arm101
    ik_frame: ready
  kinematics:
    - id: shoulder_pan
      axis: z
      limits_deg:
        - -180
        - 180
      length_mm: 60
      a_mm: 0
      d_mm: 60
      servo_id: 1
      encoder_sign: 1
      zero_pose_steps: 1970
    - id: shoulder_lift
      axis: y
      limits_deg:
        - -90
        - 90
      length_mm: 125
      a_mm: 125
      d_mm: 0
      servo_id: 2
      encoder_sign: 1
      zero_pose_steps: 2230
    - id: elbow_flex
      axis: y
      limits_deg:
        - -90
        - 90
      length_mm: 125
      a_mm: 125
      d_mm: 0
      servo_id: 3
      encoder_sign: 1
      zero_pose_steps: 2047
    - id: wrist_flex
      axis: y
      limits_deg:
        - -90
        - 90
      length_mm: 60
      a_mm: 60
      d_mm: 0
      servo_id: 4
      encoder_sign: 1
      zero_pose_steps: 2048
    - id: wrist_roll
      axis: x
      limits_deg:
        - -180
        - 180
      length_mm: 30
      a_mm: 30
      d_mm: 0
      servo_id: 5
      encoder_sign: 1
      zero_pose_steps: 2188
    - id: gripper
      axis: y
      limits_deg:
        - 0
        - 90
      length_mm: 40
      a_mm: 0
      d_mm: 0
      servo_id: 6
      encoder_sign: 1
      zero_pose_steps: 1539
  poses:
    ready:
      description: Forward-extended pose, gripper horizontal.
      joints:
        shoulder_pan: 2048
        shoulder_lift: 1800
        elbow_flex: 2300
        wrist_flex: 2048
        wrist_roll: 2048
        gripper: 1700
      source: declared
  workspace:
    from_pose: ready
    bounds_mm:
      x:
        - -200
        - 340
      y:
        - -340
        - 340
      z:
        - 0
        - 250
capabilities:
  - arm.reach
  - arm.home
  - arm.move_to
  - arm.state
  - status.report
---

# so-arm101-test-rig

The geometry of the SO-ARM101 this driver was written against, copied verbatim
from that robot's own manifest so the tests reason about a real arm rather than
a convenient one. Unsigned and not registered: it exists to be read, never to
authorize anything.
