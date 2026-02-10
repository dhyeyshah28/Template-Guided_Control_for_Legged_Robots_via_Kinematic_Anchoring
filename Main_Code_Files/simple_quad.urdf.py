<?xml version="1.0"?>
<robot name="simple_quad">

  <!-- ================= BASE ================= -->
  <link name="base">
    <inertial>
      <mass value="5"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.2" iyy="0.2" izz="0.2"/>
    </inertial>
    <visual>
      <geometry><box size="0.4 0.1 0.1"/></geometry>
      <origin xyz="0 0 0"/>
      <material name="gray"><color rgba="0.8 0.8 0.8 1"/></material>
    </visual>
    <collision>
      <geometry><box size="0.4 0.1 0.1"/></geometry>
    </collision>
  </link>

  <!-- PARAMETERS -->
  <!-- Hip spacing: +/- 0.2 m forward/back -->
  <!-- Leg lateral offset ignored → 2D model -->

  <!-- ============ LEG TEMPLATE ============ -->
  <!-- Each leg has:
        hip (revolute Y-axis → sagittal)
        thigh link
        knee (revolute Y-axis)
        lower leg link
  -->

  <!-- **** FRONT RIGHT LEG **** -->
  <joint name="FR_hip" type="revolute">
    <parent link="base"/>
    <child link="FR_thigh"/>
    <origin xyz="0.2 0 0" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.5" upper="1.5"/>
  </joint>

  <link name="FR_thigh">
    <visual>
      <origin xyz="0 0 -0.075"/>
      <geometry><capsule radius="0.015" length="0.15"/></geometry>
      <material name="blue"><color rgba="0 0 1 1"/></material>
    </visual>
    <collision>
      <origin xyz="0 0 -0.075"/>
      <geometry><capsule radius="0.015" length="0.15"/></geometry>
    </collision>
    <inertial><mass value="0.3"/></inertial>
  </link>

  <joint name="FR_knee" type="revolute">
    <parent link="FR_thigh"/>
    <child link="FR_shin"/>
    <origin xyz="0 0 -0.15" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-2" upper="0.0"/>
  </joint>

  <link name="FR_shin">
    <visual>
      <origin xyz="0 0 -0.15"/>
      <geometry><capsule radius="0.015" length="0.3"/></geometry>
      <material name="green"><color rgba="0 1 0 1"/></material>
    </visual>
    <collision>
      <origin xyz="0 0 -0.15"/>
      <geometry><capsule radius="0.015" length="0.3"/></geometry>
    </collision>
    <inertial><mass value="0.2"/></inertial>
  </link>

  <!-- Copy-pasted legs: FL, RR, RL are identical but mirrored -->

  <!-- **** FRONT LEFT **** -->
  <joint name="FL_hip" type="revolute">
    <parent link="base"/>
    <child link="FL_thigh"/>
    <origin xyz="0.2 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.5" upper="1.5"/>
  </joint>

  <link name="FL_thigh">
    <visual><origin xyz="0 0 -0.075"/>
      <geometry><capsule radius="0.015" length="0.15"/></geometry>
      <material name="blue"/>
    </visual>
    <collision><origin xyz="0 0 -0.075">
      <geometry><capsule radius="0.015" length="0.15"/></geometry></origin>
    </collision>
    <inertial><mass value="0.3"/></inertial>
  </link>

  <joint name="FL_knee" type="revolute">
    <parent link="FL_thigh"/>
    <child link="FL_shin"/>
    <origin xyz="0 0 -0.15"/>
    <axis xyz="0 1 0"/>
    <limit lower="-2" upper="0.0"/>
  </joint>

  <link name="FL_shin">
    <visual><origin xyz="0 0 -0.15"/>
      <geometry><capsule radius="0.015" length="0.3"/></geometry>
      <material name="green"/>
    </visual>
    <collision><origin xyz="0 0 -0.15">
      <geometry><capsule radius="0.015" length="0.3"/></geometry></origin>
    </collision>
    <inertial><mass value="0.2"/></inertial>
  </link>

  <!-- **** REAR RIGHT **** -->
  <joint name="RR_hip" type="revolute">
    <parent link="base"/>
    <child link="RR_thigh"/>
    <origin xyz="-0.2 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.5" upper="1.5"/>
  </joint>

  <link name="RR_thigh">
    <visual><origin xyz="0 0 -0.075"/>
      <geometry><capsule radius="0.015" length="0.15"/></geometry>
      <material name="blue"/>
    </visual>
    <collision><origin xyz="0 0 -0.075">
      <geometry><capsule radius="0.015" length="0.15"/></geometry></origin>
    </collision>
    <inertial><mass value="0.3"/></inertial>
  </link>

  <joint name="RR_knee" type="revolute">
    <parent link="RR_thigh"/>
    <child link="RR_shin"/>
    <origin xyz="0 0 -0.15"/>
    <axis xyz="0 1 0"/>
    <limit lower="-2" upper="0.0"/>
  </joint>

  <link name="RR_shin">
    <visual><origin xyz="0 0 -0.15"/>
      <geometry><capsule radius="0.015" length="0.3"/></geometry>
      <material name="green"/>
    </visual>
    <collision><origin xyz="0 0 -0.15">
      <geometry><capsule radius="0.015" length="0.3"/></geometry></origin>
    </collision>
    <inertial><mass value="0.2"/></inertial>
  </link>

  <!-- **** REAR LEFT **** -->
  <joint name="RL_hip" type="revolute">
    <parent link="base"/>
    <child link="RL_thigh"/>
    <origin xyz="-0.2 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.5" upper="1.5"/>
  </joint>

  <link name="RL_thigh">
    <visual><origin xyz="0 0 -0.075"/>
      <geometry><capsule radius="0.015" length="0.15"/></geometry>
    </visual>
    <collision><origin xyz="0 0 -0.075">
      <geometry><capsule radius="0.015" length="0.15"/></geometry></origin>
    </collision>
    <inertial><mass value="0.3"/></inertial>
  </link>

  <joint name="RL_knee" type="revolute">
    <parent link="RL_thigh"/>
    <child link="RL_shin"/>
    <origin xyz="0 0 -0.15"/>
    <axis xyz="0 1 0"/>
    <limit lower="-2" upper="0.0"/>
  </joint>

  <link name="RL_shin">
    <visual><origin xyz="0 0 -0.15"/>
      <geometry><capsule radius="0.015" length="0.3"/></geometry>
    </visual>
    <collision><origin xyz="0 0 -0.15">
      <geometry><capsule radius="0.015" length="0.3"/></geometry></origin>
    </collision>
    <inertial><mass value="0.2"/></inertial>
  </link>

</robot>
