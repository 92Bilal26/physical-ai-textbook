---
title: Section 1 - Links and Joints
---

# Section 1: Links and Joints

**Estimated Reading Time**: 8 minutes
**Bloom's Level**: Understand, Apply
**Key Learning**: URDF structure, kinematic relationships

---

## What Are Links and Joints?

Every robot is made of **two fundamental building blocks**: links and joints.

### Link = Rigid Body

A **link** is a **rigid body**—a solid part of the robot that doesn't bend or deform.

Examples:
- Robot arm segment (metal rod or pipe)
- Robot torso (main body)
- Wheel (can spin but doesn't change shape)
- Gripper finger (solid piece)

**In URDF**: A link has:
- **Mass** (weight in kg)
- **Inertia** (resistance to spinning)
- **Visual geometry** (how it looks in RViz)
- **Collision geometry** (shape for physics)

### Joint = Connection Between Links

A **joint** is a **connection** between two links that allows relative motion.

Examples:
- **Revolute joint**: Robot arm elbow (rotates around an axis)
- **Prismatic joint**: Robot gripper (slides/extends)
- **Fixed joint**: Mounted camera (no motion)
- **Continuous joint**: Robot wheel (rotates freely)

**In URDF**: A joint has:
- **Type** (revolute, prismatic, fixed, continuous)
- **Parent link** (fixed link)
- **Child link** (moving link)
- **Axis** (direction of motion)
- **Limits** (minimum and maximum angles/distances)

---

## The Kinematic Tree

A robot's structure is described as a **kinematic tree**: each link has parents and children connected by joints.

### Example: Simple 2-Link Robot

```
        Joint 1 (Revolute)
           ↙↖ (Z-axis rotation)

      base_link
           ↓
      (mass=2.0 kg)
           ↓
      link_1
           ↓
      Joint 2 (Revolute)
      (Y-axis rotation)
           ↓
      link_2
           ↓
  end_effector (fixed)
```

**Read as**:
- `base_link` is the root (doesn't move)
- `link_1` connects to `base_link` via `joint_1`
- `link_2` connects to `link_1` via `joint_2`
- `end_effector` is a fixed reference frame on `link_2`

---

## URDF Syntax Basics

URDF files are **XML** (eXtensible Markup Language). If you've seen HTML, XML is similar:

```xml
<tag attribute="value">
  Content here
</tag>
```

### URDF Structure

```xml
<?xml version="1.0"?>
<robot name="my_robot">

  <!-- Define links -->
  <link name="base_link">
    <!-- link properties -->
  </link>

  <link name="link_1">
    <!-- link properties -->
  </link>

  <!-- Define joints (connections) -->
  <joint name="joint_1" type="revolute">
    <parent link="base_link"/>
    <child link="link_1"/>
    <!-- joint properties -->
  </joint>

</robot>
```

### Key XML Rules
- Every `<tag>` must have a closing `</tag>`
- Attributes use quotes: `name="value"`
- Nesting must be proper (no crossing)
- XML is case-sensitive: `<link>` ≠ `<Link>`

---

## Code Example 1: Links in URDF

Let's look at a link from `simple_robot.urdf`:

```xml
<link name="base_link">
  <inertial>
    <!-- Mass of this link (kg) -->
    <mass value="2.0"/>

    <!-- Center of mass location -->
    <origin xyz="0 0 0.05" rpy="0 0 0"/>

    <!-- Inertia matrix (how hard to spin) -->
    <inertia
      ixx="0.0167" ixy="0.0" ixz="0.0"
      iyy="0.0167" iyz="0.0"
      izz="0.0167"/>
  </inertial>

  <!-- Visual appearance (how RViz shows it) -->
  <visual>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
    <geometry>
      <!-- Box: width=0.2m, depth=0.2m, height=0.1m -->
      <box size="0.2 0.2 0.1"/>
    </geometry>
    <material name="gray">
      <color rgba="0.5 0.5 0.5 1.0"/>
    </material>
  </visual>

  <!-- Collision shape (for physics simulation) -->
  <collision>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
    <geometry>
      <box size="0.2 0.2 0.1"/>
    </geometry>
  </collision>
</link>
```

**What this describes**:
- A box-shaped link named `base_link`
- Mass: 2.0 kg
- Dimensions: 0.2m × 0.2m × 0.1m (width × depth × height)
- Color: Gray (in RViz)
- Can collide in simulation (Gazebo)

---

## Code Example 2: Joints in URDF

Now let's see how joints connect links:

```xml
<joint name="joint_1" type="revolute">
  <!-- Parent link (doesn't move with this joint) -->
  <parent link="base_link"/>

  <!-- Child link (moves with this joint) -->
  <child link="link_1"/>

  <!-- Position of joint relative to parent -->
  <origin xyz="0 0 0.1" rpy="0 0 0"/>

  <!-- Axis of rotation: X=1,0,0 (rotate around X-axis) -->
  <axis xyz="0 0 1"/>

  <!-- Joint limits: min angle, max angle, effort, velocity -->
  <limit lower="-3.14159" upper="3.14159" effort="10" velocity="1.0"/>

  <!-- Friction and damping -->
  <dynamics damping="0.1" friction="0.1"/>
</joint>
```

**What this describes**:
- A revolute (rotational) joint named `joint_1`
- Connects `base_link` (parent) to `link_1` (child)
- Rotates around the **Z-axis** (xyz="0 0 1")
- Can rotate from -180° to +180° (radians)
- Max force: 10 Nm (torque)
- Max speed: 1.0 rad/s

---

## Joint Types

### Revolute Joint
**Rotates** around an axis with **limits** (like a hinge).

```xml
<joint name="elbow" type="revolute">
  <axis xyz="1 0 0"/>  <!-- Rotate around X-axis -->
  <limit lower="0" upper="2.0" effort="10" velocity="1.5"/>
</joint>
```

**Real examples**:
- Robot arm elbow
- Robot neck (yaw rotation)
- Door hinge

### Continuous Joint
**Rotates freely** (no angle limits, like a wheel).

```xml
<joint name="wheel" type="continuous">
  <axis xyz="0 0 1"/>  <!-- Spin around Z-axis -->
</joint>
```

**Real examples**:
- Robot wheel
- Robot joint that spins 360°
- Turntable

### Prismatic Joint
**Slides** (translates) along an axis with limits (like a drawer).

```xml
<joint name="elevator" type="prismatic">
  <axis xyz="0 0 1"/>  <!-- Move up/down (Z-axis) -->
  <limit lower="0" upper="1.0" effort="100" velocity="0.5"/>
</joint>
```

**Real examples**:
- Robot gripper that extends
- Elevator lift
- Sliding drawer

### Fixed Joint
**No motion** allowed (rigid connection).

```xml
<joint name="mount" type="fixed">
  <parent link="arm"/>
  <child link="camera"/>
</joint>
```

**Real examples**:
- Camera mounted on robot
- Gripper tool mounted on arm
- Sensor bolted to frame

---

## Coordinate Frames: XYZ and RPY

Every link and joint has an **origin**—a position and orientation in 3D space.

### XYZ: Position
- **X**: Left (-) / Right (+) [red axis]
- **Y**: Back (-) / Forward (+) [green axis]
- **Z**: Down (-) / Up (+) [blue axis]

```
Origin xyz="0.5 0.2 0.1"
  means: 0.5m right, 0.2m forward, 0.1m up
```

### RPY: Rotation
- **R (Roll)**: Rotate around X-axis (like rolling a ball forward)
- **P (Pitch)**: Rotate around Y-axis (like nodding your head)
- **Y (Yaw)**: Rotate around Z-axis (like turning left/right)

All angles in **radians**:
- π (pi) ≈ 3.14159 = 180°
- π/2 ≈ 1.5708 = 90°

```
Origin rpy="0 1.5708 0"
  means: 0° roll, 90° pitch (nodding), 0° yaw
```

### Full Origin Example

```xml
<origin xyz="0.5 0 0" rpy="0 0 0"/>
  Position: 0.5m to the right, no height change
  Rotation: No rotation (aligned with parent frame)

<origin xyz="0 0 0.1" rpy="0 1.5708 0"/>
  Position: 0.1m upward
  Rotation: 90° pitch (tilted forward)
```

---

## Mental Model: Kinematic Chain

Think of a robot like a **chain**:
- Each link is a **link of the chain**
- Each joint is a **connection point**
- Motion in one joint affects all children

```
Head (can look left/right)
  ↓ (neck joint)
Torso (main body)
  ↙       ↘
L-Shoulder  R-Shoulder
  ↓          ↓
L-Arm      R-Arm
  ↓          ↓
L-Hand     R-Hand
```

**Rule**: When you move a parent, all children move too.
- Rotate the torso → head and arms rotate with it
- Rotate the shoulder → arm and hand move
- Rotate the wrist → only the hand moves

This is called **forward kinematics**: parent motion affects children.

---

## Common Mistakes

### ❌ Mistake 1: Circular Kinematic Tree

```xml
<joint name="joint_1">
  <parent link="A"/>
  <child link="B"/>
</joint>

<joint name="joint_2">
  <parent link="B"/>
  <child link="A"/>  <!-- ERROR: A is now child of B AND parent! -->
</joint>
```

**Error**: Circular reference. Links should form a **tree**, not a loop.

**Fix**: Keep parent-child relationships strict. One parent per link.

### ❌ Mistake 2: Missing Links

```xml
<joint name="joint_1">
  <parent link="base_link"/>
  <child link="link_1"/>  <!-- OK -->
</joint>

<joint name="joint_2">
  <parent link="link_2"/>  <!-- ERROR: link_2 was never defined! -->
  <child link="link_3"/>
</joint>
```

**Error**: `link_2` doesn't exist. Every parent and child in a joint must be defined.

**Fix**: Use `check_urdf simple_robot.urdf` to catch these errors:
```
ERROR: link [link_2] not found
```

### ❌ Mistake 3: Wrong Axis Direction

```xml
<joint name="elbow" type="revolute">
  <axis xyz="0 0 1"/>  <!-- Z-axis (up/down rotation) -->
  <!-- But this is supposed to be left/right rotation! -->
</joint>
```

**Error**: Robot rotates the wrong way.

**Fix**: Think about which direction the joint should rotate:
- **Left/Right (yaw)**: Use Z-axis `xyz="0 0 1"`
- **Up/Down (pitch)**: Use Y-axis `xyz="0 1 0"`
- **Forward/Back (roll)**: Use X-axis `xyz="1 0 0"`

---

## Try It Yourself

Open `simple_robot.urdf` and:
- [ ] Find all `<link>` definitions. How many are there?
- [ ] Find all `<joint>` definitions. What types are they?
- [ ] Trace the kinematic tree: base_link → link_1 → link_2 → end_effector
- [ ] Change a link's color in the visual material
- [ ] Change a joint's rotation axis (then validate with `check_urdf`)

---

## Key Takeaways

✅ **Links** are rigid bodies with mass and geometry
✅ **Joints** connect links and define how they move
✅ **Kinematic trees** show parent-child relationships
✅ **XYZ** describes position; **RPY** describes rotation
✅ Use `check_urdf` to validate URDF syntax
✅ Every parent and child link must be defined
✅ Circular references cause errors

---

## Next: Gazebo Properties

Now that you understand URDF structure, let's add **physical properties** for simulation!

👉 **Next**: [Section 2: Gazebo Properties](./02-gazebo-properties.md)

Or review:
- [Chapter 2 Intro](./index.md)
- [Learning Objectives](./learning-objectives.md)

---

**Section Status**: Complete ✅
**Skills Used**: Explanation, diagrams, mental models
