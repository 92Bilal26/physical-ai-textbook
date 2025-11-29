---
title: Section 2 - Gazebo Properties
---

# Section 2: Gazebo Properties

**Estimated Reading Time**: 9 minutes
**Bloom's Level**: Understand, Apply
**Key Learning**: Physics simulation, inertia, collision geometry

---

## From Visualization to Simulation

In **Section 1**, you learned to describe robot structure with links and joints. In **RViz**, this works great—you can see the robot in 3D.

But what if you want to **simulate** the robot? What if you want to:
- ✅ Drop a robot and have it fall realistically
- ✅ Simulate motors pushing against gravity
- ✅ Detect when the gripper hits an object
- ✅ Calculate how long it takes to move

**RViz can't do this**—it's just a visualization tool. You need **Gazebo**, a physics simulator.

Gazebo needs to know:
- **How much does each part weigh?** (mass)
- **How hard is it to spin this part?** (inertia)
- **What shape is this part for collisions?** (collision geometry)

Without this information:
- ❌ Robots fall through the floor
- ❌ Motors spin indefinitely (no gravity resistance)
- ❌ Collisions don't work
- ❌ Physics calculations fail

With this information:
- ✅ Realistic gravity and weight
- ✅ Motors need real force to move parts
- ✅ Objects bounce and collide correctly
- ✅ Simulation matches real robot behavior

---

## Mass and Weight

### What is Mass?

**Mass** is how much material is in a link (measured in kilograms).

```
Light link (easy to move):     mass="0.5"  (500 grams)
Heavy link (hard to move):     mass="5.0"  (5 kilograms)
```

### Adding Mass to URDF

```xml
<link name="arm_segment">
  <inertial>
    <!-- Mass in kilograms -->
    <mass value="1.5"/>

    <!-- Where is the mass centered? -->
    <origin xyz="0.5 0 0" rpy="0 0 0"/>
    <!-- This says: center of mass is 0.5m along X-axis -->

    <!-- Inertia matrix (next section) -->
    <inertia .../>
  </inertial>

  <!-- Visual and collision geometry -->
  <visual>...</visual>
  <collision>...</collision>
</link>
```

### Estimating Mass

**Real robots**: Weigh each part on a scale
**Simulation**: Estimate based on size:

```
Material density guide:
- Plastic:     ~1000 kg/m³
- Aluminum:    ~2700 kg/m³
- Steel:       ~7850 kg/m³

Example: Aluminum box 0.1m × 0.1m × 0.4m
Volume = 0.004 m³
Mass = 0.004 × 2700 = 10.8 kg

For safety, use:
- Motors/servos: ~0.5-2.0 kg each
- Robot links: ~1-5 kg each
- Gripper: ~0.5-1.0 kg
```

---

## Inertia Matrix

### What is Inertia?

**Inertia** is resistance to spinning. Just like mass resists acceleration, inertia resists angular acceleration.

**Simple analogy**:
- Spinning a **pencil** around its long axis: Easy (low inertia)
- Spinning a **pencil** around its short axis: Hard (high inertia)

Same mass, different shapes → different inertia.

### The Inertia Matrix

```xml
<inertia
  ixx="0.0167" ixy="0.0" ixz="0.0"
  iyy="0.0167" iyz="0.0"
  izz="0.0167"/>
```

**What each means**:
- **ixx**: Resistance to rolling (rotation around X-axis)
- **iyy**: Resistance to pitching (rotation around Y-axis)
- **izz**: Resistance to yawing (rotation around Z-axis)
- **ixy, ixz, iyz**: Off-diagonal terms (usually 0 for symmetric shapes)

### Computing Inertia for Common Shapes

**Solid Box** (width W, depth D, height H, mass M):
```
ixx = (1/12) × M × (D² + H²)
iyy = (1/12) × M × (W² + H²)
izz = (1/12) × M × (W² + D²)
```

**Example**: Box 0.2m × 0.2m × 0.1m, mass 2.0 kg
```
ixx = (1/12) × 2.0 × (0.2² + 0.1²) = 0.0167
iyy = (1/12) × 2.0 × (0.2² + 0.1²) = 0.0167
izz = (1/12) × 2.0 × (0.2² + 0.2²) = 0.0267
```

**Solid Cylinder** (radius R, height H, mass M):
```
ixx = (1/12) × M × (3×R² + H²)
iyy = (1/12) × M × (3×R² + H²)
izz = (1/2) × M × R²
```

**For quick estimates**: Use online inertia calculators or approximations in your CAD software.

### Common Mistake: Bad Inertia Values

```xml
<!-- ❌ WRONG: Inertia too small -->
<inertia
  ixx="0.00001" ixy="0.0" ixz="0.0"
  iyy="0.00001" iyz="0.0"
  izz="0.00001"/>
```

**Problem**: Gazebo simulator becomes unstable. Robot vibrates or explodes.

```xml
<!-- ✅ CORRECT: Reasonable inertia values -->
<inertia
  ixx="0.01" ixy="0.0" ixz="0.0"
  iyy="0.01" iyz="0.0"
  izz="0.01"/>
```

**Rule**: Inertia should be roughly: `0.001 × (mass in kg) × (size in meters)²`

---

## Collision Geometry

### Visual vs. Collision Geometry

A robot link has **two** geometries:

1. **Visual geometry**: What RViz shows (can be fancy meshes)
2. **Collision geometry**: What Gazebo uses for physics (must be simple shapes)

```
      VISUAL (RViz)              COLLISION (Gazebo)
   Detailed 3D mesh            Simple box/cylinder
   ~Pretty to look at          ~Fast to compute
   ~High detail                ~Physics accurate
```

### Why Two Geometries?

Visual meshes are complex (thousands of triangles) → slow physics calculations.
Collision shapes are simple (box, cylinder, sphere) → fast physics.

```xml
<link name="arm">
  <!-- Visual: detailed mesh -->
  <visual>
    <geometry>
      <mesh filename="package://my_robot/meshes/arm.obj"/>
    </geometry>
  </visual>

  <!-- Collision: simple box -->
  <collision>
    <geometry>
      <box size="0.05 0.05 0.4"/>  <!-- Approximate with box -->
    </geometry>
  </collision>
</link>
```

### Collision Shape Types

**Box** (rectangular)
```xml
<box size="width depth height"/>
<!-- Example: 0.2m × 0.2m × 0.1m -->
```

**Cylinder** (circular pipe)
```xml
<cylinder radius="0.05" length="0.4"/>
<!-- Example: 5cm radius, 40cm tall -->
```

**Sphere** (ball)
```xml
<sphere radius="0.1"/>
<!-- Example: 10cm radius -->
```

---

## Complete Example: Link with Physics

Here's a complete link from `simple_robot.urdf` with all physics properties:

```xml
<link name="link_1">
  <!-- Physics properties -->
  <inertial>
    <!-- Mass in kilograms -->
    <mass value="0.8"/>

    <!-- Center of mass (middle of the segment) -->
    <origin xyz="0.5 0 0" rpy="0 0 0"/>

    <!-- Inertia for elongated rod -->
    <inertia
      ixx="0.0133" ixy="0.0" ixz="0.0"
      iyy="0.0533" iyz="0.0"
      izz="0.0533"/>
  </inertial>

  <!-- Appearance (what RViz shows) -->
  <visual>
    <origin xyz="0.5 0 0" rpy="0 0 0"/>
    <geometry>
      <box size="1.0 0.05 0.05"/>  <!-- 1m × 5cm × 5cm -->
    </geometry>
    <material name="blue">
      <color rgba="0.0 0.0 1.0 1.0"/>  <!-- Blue color -->
    </material>
  </visual>

  <!-- Physics shape (what Gazebo uses) -->
  <collision>
    <origin xyz="0.5 0 0" rpy="0 0 0"/>
    <geometry>
      <box size="1.0 0.05 0.05"/>  <!-- Same as visual -->
    </geometry>
  </collision>
</link>
```

**This describes**:
- An elongated link (like a robot arm segment)
- Mass: 0.8 kg
- Shape: Box 1m long, 5cm × 5cm thick
- Inertia calculated for this shape
- Blue color in RViz
- Can collide with other objects in Gazebo

---

## Gazebo-Specific Tags

Gazebo physics simulation uses some special tags:

### Surface Properties

```xml
<gazebo reference="link_1">
  <!-- Friction: how much the surface grips -->
  <mu1>0.8</mu1>  <!-- Friction coefficient: 0.8 (realistic) -->

  <!-- Bounciness: how much it bounces -->
  <restitution_coefficient>0.1</restitution_coefficient>
</gazebo>
```

**Common values**:
- Rubber on concrete: μ = 0.8-1.0 (grippy)
- Metal on metal: μ = 0.3-0.5 (slippery)
- Ice: μ = 0.02-0.1 (very slippery)

### Material Definition

```xml
<gazebo reference="link_1">
  <material>Gazebo/Blue</material>  <!-- Gazebo material -->
</gazebo>
```

---

## Validation Checklist

When you add physics properties to URDF:

- [ ] Every `<link>` with motion has `<mass>` and `<inertia>`
- [ ] Inertia is symmetric: ixx, iyy, izz are all positive
- [ ] Off-diagonal inertia (ixy, ixz, iyz) are ~0 for simple shapes
- [ ] Collision geometry is simple (box, cylinder, sphere)
- [ ] Visual and collision geometry are similar
- [ ] Mass values are reasonable (0.1 - 100 kg for most robots)
- [ ] `check_urdf` passes without errors

```bash
# Validate URDF with physics properties
check_urdf simple_robot.urdf
# Output: OK  (no errors)
```

---

## Try It Yourself

Modify `simple_robot.urdf`:
- [ ] Change the mass of `base_link` from 2.0 to 5.0 kg
- [ ] Calculate new inertia values (heavier = larger inertia)
- [ ] Add collision geometry to `link_2` (currently has none)
- [ ] Change a collision shape from box to cylinder
- [ ] Validate with `check_urdf`

---

## Key Takeaways

✅ **Mass** is weight; inertia is resistance to spinning
✅ **Inertia matrix** has three diagonal values (ixx, iyy, izz)
✅ **Collision geometry** is simple shapes for fast physics
✅ **Visual geometry** can be detailed (meshes) for appearance
✅ Gazebo uses both to simulate robot behavior realistically
✅ Use `check_urdf` to validate physics properties

---

## Next: Visualizing Robots

Now that your URDF has physics properties, let's **visualize** it in RViz and Gazebo!

👉 **Next**: [Section 3: Visualizing Robots](./03-visualization.md)

Or review:
- [Section 1: Links and Joints](./01-links-joints.md)
- [Chapter 2 Intro](./index.md)

---

**Section Status**: Complete ✅
