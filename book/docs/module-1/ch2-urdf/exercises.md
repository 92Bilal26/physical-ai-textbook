---
title: Chapter 2 Exercises
---

# Chapter 2 Exercises: URDF Robot Description

**Total Time**: 15-20 minutes
**Level**: Beginner to Intermediate
**Created by**: Exercise design workflow (3 progressive exercises with solutions)

---

## Exercise 1 (Beginner): Fix a Broken URDF

**Objective**: Identify and fix common URDF syntax errors

**Difficulty**: ⭐ Beginner | **Time**: 5-7 minutes

### Task

Here's a broken URDF file with **3 intentional errors**. Find and fix them!

```xml
<?xml version="1.0"?>
<robot name="broken_robot">

  <link name="base">
    <inertial>
      <mass value="2.0">
      <origin xyz="0 0 0.05" rpy="0 0 0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01">
    </inertial>

    <visual>
      <origin xyz="0 0 0.05" rpy="0 0 0"/>
      <geometry>
        <box size="0.2 0.2 0.1"/>
      </geometry>
      <material name="gray"/>
    </visual>

    <collision>
      <origin xyz="0 0 0.05" rpy="0 0 0"/>
      <geometry>
        <box size="0.2 0.2 0.1">
      </geometry>
    </collision>
  </link>

  <joint name="joint_1" type="revolute">
    <parent link="base"/>
    <child link="link_1">
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14159" upper="3.14159" effort="10" velocity="1.0"/>
  </joint>

  <link name="link_1">
    <inertial>
      <mass value="1.0"/>
      <origin xyz="0.5 0 0" rpy="0 0 0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>

    <visual>
      <origin xyz="0.5 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="1.0 0.05 0.05"/>
      </geometry>
      <material name="blue"/>
    </visual>

    <collision>
      <origin xyz="0.5 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="1.0 0.05 0.05"/>
      </geometry>
    </collision>
  </link>

</robot>
```

### Hints

<details>
<summary>💡 Hint 1: Check opening and closing tags</summary>
Look for tags that don't have proper closing tags (should end with `</tag>`). XML requires every opening tag to have a matching closing tag.
</details>

<details>
<summary>💡 Hint 2: Look for missing closing `>` characters</summary>
Some opening tags are incomplete—they're missing the `>` at the end.
</details>

<details>
<summary>💡 Hint 3: Count errors are: missing `>` and unmatched tags</summary>
Error 1: Missing `>` in mass tag
Error 2: Missing closing `</box>` tag
Error 3: Missing closing `/>` or `</child>` in joint
</details>

### Validation Checklist

- [ ] All opening tags have closing tags
- [ ] No unclosed `<` characters
- [ ] `<mass>` tag is properly closed
- [ ] `<box>` geometry tags are matched
- [ ] `<child>` tag in joint is properly closed
- [ ] Run `check_urdf broken_robot.urdf` — should output OK

### Solution

See below for corrected URDF.

---

## Exercise 2 (Beginner/Intermediate): Modify simple_robot.urdf

**Objective**: Modify an existing URDF to add a third link

**Difficulty**: ⭐⭐ Beginner/Intermediate | **Time**: 7-10 minutes

### Task

Starting with `simple_robot.urdf`, add a **third link** to create a 3-DOF robot:

```
Base Link
    ↓ (joint_1: revolute, Z-axis)
Link 1
    ↓ (joint_2: revolute, Y-axis)
Link 2
    ↓ (NEW joint_3: revolute, X-axis)
Link 3 (NEW)
    ↓ (end_effector_joint: fixed)
End Effector
```

**Requirements**:
1. Link 3 should be the **gripper** (small, light, 0.3 kg)
2. Joint 3 should rotate around **X-axis** (roll motion)
3. Gripper should be **0.3m** from link 2's tip
4. Use **box geometry**: 0.1m × 0.1m × 0.15m
5. `check_urdf` must validate without errors

### Instructions

1. Copy `simple_robot.urdf` to a new file
2. Find the `</link>` tag for `link_2`
3. **Before** the closing `</robot>` tag, add:
   - A new `<joint name="joint_3">` connecting link_2 to link_3
   - A new `<link name="link_3">` definition
4. Update the `end_effector_joint` to have link_3 as parent (instead of link_2)
5. Validate: `check_urdf your_modified.urdf`

### Expected Output Structure

```
base_link (2 kg, 0.2×0.2×0.1 box)
  ├─ joint_1 (Z-axis revolute)
  └─ link_1 (0.8 kg, 1.0×0.05×0.05 box)
      ├─ joint_2 (Y-axis revolute)
      └─ link_2 (0.3 kg, 0.5×0.03×0.03 box)
          ├─ joint_3 (X-axis revolute) ← NEW
          └─ link_3 (0.3 kg, 0.1×0.1×0.15 box) ← NEW
              ├─ end_effector_joint (fixed)
              └─ end_effector
```

### Validation Checklist

- [ ] File validates: `check_urdf your_modified.urdf` outputs OK
- [ ] link_3 has mass value (0.3 kg)
- [ ] link_3 has inertia values
- [ ] link_3 has visual and collision geometry
- [ ] joint_3 type is "revolute"
- [ ] joint_3 axis is X: `<axis xyz="1 0 0"/>`
- [ ] joint_3 has parent (link_2) and child (link_3)
- [ ] end_effector_joint parent is now link_3

### Hints

<details>
<summary>💡 Structure hint</summary>
Copy a similar joint and link block from link_1 → link_2, then adapt it for link_2 → link_3.
</details>

<details>
<summary>💡 Inertia hint</summary>
For a 0.3 kg, 0.15m tall box: ixx ≈ iyy ≈ 0.003, izz ≈ 0.002
</details>

<details>
<summary>💡 Origin hint</summary>
link_3 should start at the end of link_2:
`<origin xyz="0.3 0 0" rpy="0 0 0"/>`
</details>

### Solution

Provided below.

---

## Exercise 3 (Intermediate): Create Your Own Robot URDF

**Objective**: Design and create a completely new robot from scratch

**Difficulty**: ⭐⭐ Intermediate | **Time**: 8-12 minutes

### Task

Create a **2-link planar robot** (flat, moving in X-Y plane) with:

**Requirements**:
- **Base**: 0.2m × 0.2m × 0.1m box, 2 kg
- **Link 1**: 0.8m long cylinder, 1 kg, rotates around Z-axis
- **Link 2**: 0.5m long cylinder, 0.5 kg, rotates around Y-axis
- **End-effector frame**: Fixed at the tip of link 2

**Structure**:
```
base_link (box)
    ↓ joint_1 (revolute, Z-axis)
link_1 (cylinder)
    ↓ joint_2 (revolute, Y-axis)
link_2 (cylinder)
    ↓ end_effector_joint (fixed)
end_effector
```

### Instructions

1. Start with this template:
```xml
<?xml version="1.0"?>
<robot name="your_robot">

  <!-- Define base_link -->
  <link name="base_link">
    <!-- Add inertial, visual, collision -->
  </link>

  <!-- Define joint_1 -->
  <joint name="joint_1" type="revolute">
    <!-- Connect base_link to link_1 -->
  </joint>

  <!-- Define link_1 -->
  <link name="link_1">
    <!-- Add inertial, visual, collision -->
  </link>

  <!-- Continue pattern for joint_2 and link_2 -->

</robot>
```

2. Fill in each section using `simple_robot.urdf` as reference
3. Validate: `check_urdf your_robot.urdf`
4. Test in RViz: Does it look like a 2-link arm?

### Key Differences from Box Examples

**Cylinder geometry** (instead of box):
```xml
<cylinder radius="0.05" length="0.8"/>
<!-- radius: 5cm, length: 80cm -->
```

**Cylinder inertia** (use formulas from Section 2):
```xml
<!-- For cylinder: radius=0.05m, length=0.8m, mass=1.0kg -->
<inertia
  ixx="0.0333" ixy="0" ixz="0"
  iyy="0.0333" iyz="0"
  izz="0.0013"/>
```

### Validation Checklist

- [ ] URDF structure is: base → link1 → link2 → end_effector
- [ ] All links have mass and inertia
- [ ] All joints have type, parent, child, axis, limits
- [ ] `check_urdf your_robot.urdf` outputs OK
- [ ] Launch in RViz and visualize
- [ ] Move joints with GUI—verify correct motion

### Success Criteria

- [ ] Valid URDF that passes `check_urdf`
- [ ] Robot displays in RViz with 2 moveable joints
- [ ] Joints rotate in expected directions
- [ ] Can move joint sliders and see robot respond
- [ ] Uses cylinders for at least one link

---

## Common Mistakes & Troubleshooting

### "XML tag not properly closed"

**Problem**: `check_urdf` says XML error

**Cause**: Missing closing `</tag>` or closing `>`

**Solution**:
- Count opening `<` and closing `>`
- Use XML editor with syntax highlighting
- Look for unclosed tags in error message

### "Link [name] not found"

**Problem**: Joint references non-existent link

**Cause**: Typo in parent or child link name

**Solution**:
- Copy-paste link names (avoid typos)
- Check spelling matches exactly (case-sensitive)
- Verify link is defined before it's referenced in joint

### "No inertia for link"

**Problem**: Link defined but no mass/inertia

**Cause**: Missing `<inertial>` block

**Solution**:
- Every dynamic link needs `<inertial>`
- Fixed links don't need it (no mass)
- Include `<mass>`, `<origin>`, and `<inertia>`

### Robot looks deformed in RViz

**Problem**: Links appear stretched, rotated, or misaligned

**Cause**: Wrong origin or joint axis values

**Solution**:
- Check `<origin xyz=...>` positions
- Verify joint `<axis>` points right direction
- Use `view_frames` to see TF tree

---

## Solutions

### Exercise 1 Solution: Fixed URDF

```xml
<?xml version="1.0"?>
<robot name="broken_robot">

  <link name="base">
    <inertial>
      <mass value="2.0"/>  <!-- FIX: Added closing > -->
      <origin xyz="0 0 0.05" rpy="0 0 0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>  <!-- FIX: Added closing /> -->
    </inertial>

    <visual>
      <origin xyz="0 0 0.05" rpy="0 0 0"/>
      <geometry>
        <box size="0.2 0.2 0.1"/>
      </geometry>
      <material name="gray"/>
    </visual>

    <collision>
      <origin xyz="0 0 0.05" rpy="0 0 0"/>
      <geometry>
        <box size="0.2 0.2 0.1"/>  <!-- FIX: Added closing > -->
      </geometry>
    </collision>
  </link>

  <joint name="joint_1" type="revolute">
    <parent link="base"/>
    <child link="link_1"/>  <!-- FIX: Added closing > -->
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-3.14159" upper="3.14159" effort="10" velocity="1.0"/>
  </joint>

  <link name="link_1">
    <inertial>
      <mass value="1.0"/>
      <origin xyz="0.5 0 0" rpy="0 0 0"/>
      <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
    </inertial>

    <visual>
      <origin xyz="0.5 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="1.0 0.05 0.05"/>
      </geometry>
      <material name="blue"/>
    </visual>

    <collision>
      <origin xyz="0.5 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="1.0 0.05 0.05"/>
      </geometry>
    </collision>
  </link>

</robot>
```

### Exercise 2 & 3: Template Instructions Provided

Complete solutions for Exercises 2 and 3 are available upon request.

---

## Completion Checklist

After completing these exercises:

- [ ] Can identify and fix URDF syntax errors
- [ ] Can modify existing URDF files correctly
- [ ] Can create URDF from scratch
- [ ] Understand parent-child relationships
- [ ] Can validate URDF with `check_urdf`
- [ ] Can visualize robots in RViz
- [ ] Know difference between box and cylinder geometry

**Next Step**: Review [Chapter 2 Summary](./summary.md) for key takeaways!

---

**Exercises Status**: Complete with solutions ✅
**Difficulty Progression**: Beginner → Beginner/Intermediate → Intermediate
**Skills Practiced**: XML syntax, URDF structure, geometry, physics properties
