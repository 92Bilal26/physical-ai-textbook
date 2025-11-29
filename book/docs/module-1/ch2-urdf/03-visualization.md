---
title: Section 3 - Visualizing Robots
---

# Section 3: Visualizing Robots

**Estimated Reading Time**: 9 minutes
**Bloom's Level**: Apply, Analyze
**Key Learning**: RViz visualization, debugging URDF issues

---

## From URDF File to 3D Visualization

You've created URDF files with links, joints, and physics properties. But how do you actually **see** your robot?

Two tools do this:
1. **RViz** (Visualization): Shows 3D robot structure, TF frames, sensor data
2. **Gazebo** (Simulation): Simulates physics, gravity, collisions

Both read the URDF file and render the robot in 3D.

---

## RViz: Visualizing Robot Structure

### Launching RViz with Your Robot

```bash
# Method 1: Using a launch file (easiest)
ros2 launch simple_robot_description display.launch.py

# Method 2: Manual launch
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(cat simple_robot.urdf)"
ros2 run joint_state_publisher_gui joint_state_publisher_gui
ros2 run rviz2 rviz2
```

The launch file does **three things**:
1. **robot_state_publisher**: Publishes TF transforms from URDF
2. **joint_state_publisher_gui**: Let you move joints with sliders
3. **rviz2**: Visualizes everything in 3D

### What You'll See

```
RViz window
├─ 3D view: Your robot rendered in 3D
├─ Sliders: Move each joint
├─ TF tree: Shows frame hierarchy
└─ Property panel: Configure visualization
```

### Interacting with RViz

**Navigation**:
- **Left click + drag**: Rotate view
- **Middle click + drag**: Pan view
- **Scroll wheel**: Zoom in/out

**Joint Control** (with GUI):
- **Sliders**: Move each joint angle
- Watch the robot move in real-time
- Verify that joint axes are correct

---

## Understanding Coordinate Frames (TF)

### What are Frames?

Every link in your URDF has a **coordinate frame**—a 3D origin point with axes (X, Y, Z).

```
Base Link Frame
    ↓ (origin)
    X (red) → right
    Y (green) → forward
    Z (blue) → up
```

ROS 2 maintains the relationship between all frames using **TF (Transform Framework)**.

### Visualizing TF in RViz

1. Launch RViz with your robot
2. In left panel: **Add** → **By topic** → **/tf_static** (or **/tf**)
3. In left panel: Check "**TF**" checkbox
4. In the 3D view, you'll see coordinate axes at each joint

**What you see**:
- Red arrow: X-axis
- Green arrow: Y-axis
- Blue arrow: Z-axis

Each axis pair shows that frame's origin and orientation.

### Debugging Frame Issues

If your robot looks wrong:

**❌ Robot is upside down**
- Check Z-axis (blue) in base_link. Should point UP, not down.
- Fix: Adjust origin or joint axes

**❌ Robot appears stretched**
- Some joint axes might be pointing wrong direction
- Check that revolute joint axes align with visual geometry

**❌ Links overlap weirdly**
- Check origin values (xyz) for each link
- Verify parent-child relationships make sense

---

## Visualization Errors and Fixes

### Error 1: URDF Won't Load

```
Error: Failed to load URDF
```

**Causes**:
- XML syntax error (missing closing tag)
- Missing parent/child link definition
- File path doesn't exist

**Fix**:
```bash
check_urdf simple_robot.urdf
# Shows exactly where the error is
```

### Error 2: Robot Doesn't Appear in RViz

```
RViz empty 3D window (no robot)
```

**Causes**:
- `robot_state_publisher` not running
- TF frames not being published
- URDF has zero-sized links

**Fix**:
```bash
# Check if robot_state_publisher is running
ros2 topic list | grep tf

# Manually start it
ros2 run robot_state_publisher robot_state_publisher \
  --ros-args -p robot_description:="$(cat simple_robot.urdf)"
```

### Error 3: Joint Rotates Wrong Direction

```
Expected: Move elbow up/down
Actual: Moves left/right
```

**Cause**: Joint axis points wrong direction

**Fix**: Check the `<axis>` tag:
```xml
<!-- ❌ Wrong: Rotates side-to-side -->
<axis xyz="1 0 0"/>  <!-- X-axis -->

<!-- ✅ Correct: Rotates up/down -->
<axis xyz="0 1 0"/>  <!-- Y-axis -->
```

### Error 4: Collision Geometry Doesn't Match Visual

```
RViz shows a thin box, but collision sphere blocks movement
```

**Cause**: Visual and collision geometries are too different

**Fix**: Keep them similar (or identical):
```xml
<visual>
  <geometry>
    <box size="0.1 0.1 0.4"/>  <!-- 1m long box -->
  </geometry>
</visual>

<collision>
  <geometry>
    <box size="0.1 0.1 0.4"/>  <!-- Same as visual -->
  </geometry>
</collision>
```

---

## Gazebo: Physics Simulation

### Launching Gazebo with Your Robot

```bash
# With Gazebo
ros2 launch simple_robot_description display.launch.py use_gazebo:=true

# Gazebo window opens showing your robot
```

### What Happens in Gazebo

1. **Physics**: Gravity pulls robot down
2. **Collision**: Links can't pass through objects
3. **Motors**: Joints can move with simulated forces
4. **Sensors**: Simulated lidar, camera, etc.

### Gazebo vs. RViz

| Feature | RViz | Gazebo |
|---------|------|--------|
| **Visualization** | ✅ Yes | ✅ Yes |
| **Physics** | ❌ No | ✅ Yes |
| **Gravity** | ❌ No | ✅ Yes |
| **Collisions** | ❌ No | ✅ Yes |
| **Joint Control** | ✅ Via GUI | ✅ Via ROS 2 commands |
| **Sensor Simulation** | ❌ No | ✅ Yes |

### Debugging in Gazebo

**Robot falls through the ground**
- Cause: Bad collision geometry or bad inertia
- Fix: Check `<collision>` shapes and `<inertia>` values

**Robot vibrates or explodes**
- Cause: Inertia values too small
- Fix: Increase inertia matrix values

**Joint moves but doesn't look right**
- Cause: Collision geometry blocking movement
- Fix: Simplify collision shapes or increase joint limits

---

## Visualizing the Humanoid Robot

The humanoid robot is more complex—let's understand its structure:

```
            head (sphere)
             ↓ (neck_joint)
          base_link (torso)
          /      \
    l_shoulder  r_shoulder
      ↓           ↓
   l_upper_arm  r_upper_arm
      ↓           ↓
   l_elbow      r_elbow
      ↓           ↓
   l_lower_arm  r_lower_arm
      ↓           ↓
    l_hand      r_hand
```

To visualize the humanoid:

```bash
# Set up the robot description
export ROBOT_URDF="humanoid.urdf"

# Launch RViz
ros2 launch humanoid_robot_description display.launch.py

# In RViz:
# 1. Look at the left panel (tree view)
# 2. You should see: world → base_link → head, l_shoulder, r_shoulder, l_hip, r_hip
# 3. Use sliders to move each joint
# 4. Watch how child links move with parents
```

---

## Common Visualization Issues

### Issue: All Links Appear at Origin

**Problem**: All robot parts appear stacked at (0, 0, 0)

**Cause**: Missing `<origin>` tags in joints

**Example of problem**:
```xml
<joint name="shoulder_joint" type="revolute">
  <parent link="torso"/>
  <child link="arm"/>
  <!-- ❌ Missing: <origin xyz="0.15 0 0" rpy="0 0 0"/> -->
  <axis xyz="0 1 0"/>
</joint>
```

**Fix**: Add origin to specify where joint connects
```xml
<joint name="shoulder_joint" type="revolute">
  <parent link="torso"/>
  <child link="arm"/>
  <!-- ✅ Specifies arm starts 0.15m to the side -->
  <origin xyz="0.15 0 0" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>
</joint>
```

### Issue: Robot Has Floating/Broken Appearance

**Problem**: Links look disconnected or floating

**Cause**: Joint parent/child order is wrong

**Fix**: Check parent/child links exist:
```bash
# List all links
grep '<link name' simple_robot.urdf

# List all joints and check parent/child
grep -A 2 '<joint name' simple_robot.urdf | grep -E 'parent|child'
```

---

## Debugging Workflow

When something looks wrong:

**Step 1**: Validate URDF syntax
```bash
check_urdf humanoid.urdf
# Should output: OK
```

**Step 2**: Visualize in RViz
```bash
ros2 launch humanoid_robot_description display.launch.py
# Look for missing parts, strange positions, or disconnected links
```

**Step 3**: Enable TF visualization
- In RViz: Add → By topic → **/tf**
- Look at coordinate frame axes
- Check if they align with visual geometry

**Step 4**: Test in Gazebo
```bash
ros2 launch humanoid_robot_description display.launch.py use_gazebo:=true
# Watch if robot falls realistically
# Check if joints move smoothly
```

**Step 5**: Check console for error messages
- ROS 2 logs show problems
- Look for warnings about missing frames or bad inertia

---

## Mental Model: From URDF to Screen

Here's how URDF becomes a 3D visualization:

```
simple_robot.urdf (XML file)
        ↓
   URDF Parser
        ↓
  Robot Model
  ├─ base_link (2.0 kg box)
  ├─ link_1 (0.8 kg box)
  ├─ link_2 (0.3 kg box)
  └─ Joints (connect them)
        ↓
   TF Transform Tree
   base → link_1 → link_2
        ↓
   RViz Renderer
        ↓
   3D View on Screen
```

Each step transforms the data until you see the robot.

---

## Try It Yourself

**Exercise 1**: Visualize simple_robot.urdf
- [ ] Build the package: `colcon build --packages-select simple_robot_description`
- [ ] Source install: `source install/setup.bash`
- [ ] Launch: `ros2 launch simple_robot_description display.launch.py`
- [ ] Move the joint sliders—does the robot move correctly?

**Exercise 2**: Find URDF Errors
- [ ] Break the URDF: Change a parent link name in a joint
- [ ] Run `check_urdf simple_robot.urdf` — what error appears?
- [ ] Fix it and validate again

**Exercise 3**: Visualize TF Tree
- [ ] In RViz, add **TF** visualization
- [ ] See the coordinate axes at each joint
- [ ] Verify that base_link → link_1 → link_2 sequence makes sense

---

## Key Takeaways

✅ **RViz** visualizes robot structure without physics
✅ **Gazebo** simulates with physics (gravity, collisions)
✅ **TF (transforms)** connects all coordinate frames
✅ Use `check_urdf` to validate before visualizing
✅ Visual and collision geometries should be similar
✅ Joint axes must point the right direction
✅ RViz errors usually mean URDF problems—check with `check_urdf`

---

## Next: Hands-On Exercises

Now it's time to practice! Create, modify, and debug your own URDF files.

👉 **Next**: [Exercises](./exercises.md)

Or review:
- [Section 2: Gazebo Properties](./02-gazebo-properties.md)
- [Section 1: Links and Joints](./01-links-joints.md)
- [Chapter 2 Intro](./index.md)

---

**Section Status**: Complete ✅
