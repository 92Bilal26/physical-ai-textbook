# Exercise Template

Generated using `exercise-designer` skill. Follows pedagogical progression (Beginner → Intermediate → Advanced).

---

## Exercise [N]: [Title]

**Difficulty**: Beginner | Intermediate | Advanced
**Time Estimate**: 10-15 | 15-30 | 30-45 minutes
**Related Concept**: [Chapter section where this applies]
**Learning Outcome**: Student can [Bloom's verb] [concept]

---

## Task Description

**Objective**: What should the student accomplish?

Clear, specific task statement. Should answer:
- What code do I write?
- What should it do?
- What's the success criteria?

**Example**:
"Create a ROS 2 node that publishes 'Hello, World!' to topic `/greeting` once per second. The message should be of type `std_msgs/String`."

---

## Instructions

Step-by-step instructions without giving away the answer:

1. **Setup**: What files/packages to create?
   - `ros2 pkg create my_package --build-type ament_python`
   - Copy template from Chapter X

2. **Implement**: What code patterns to follow?
   - Use pattern from Example Y in Chapter X
   - Refer to [specific section] for explanation

3. **Test**: How to verify it works?
   - `ros2 run my_package node_name`
   - `ros2 topic echo /greeting`
   - Verify output matches expectation

4. **Extend**: (Optional) How to go further?
   - Modify to use parameter `greeting_rate`
   - Add logging to see what's happening

---

## Hints (Collapsed in final version)

Provide 2-3 hints at increasing specificity. Students reveal hints only if stuck.

**Hint 1** (If stuck on step 1):
- "You need to create a Python class that inherits from [class name]"
- "Look at Example Y for the structure"

**Hint 2** (If stuck on step 2):
- "The callback function should do X and Y"
- "Remember to call self.create_publisher() with these arguments..."

**Hint 3** (If stuck on step 3):
- "You need to use `self.get_logger().info()` to see debug output"
- "The timer interval should be 1.0 second for once-per-second publishing"

---

## Solution

**Full Solution Code**: (In separate `exercises.md` with all solutions)

```python
# Example solution for reference
# Generated and tested with ros2-code-generator

from rclpy.node import Node
from std_msgs.msg import String
import rclpy

class GreeterNode(Node):
    def __init__(self):
        super().__init__('greeter_node')
        self.publisher_ = self.create_publisher(String, '/greeting', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello, World!'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = GreeterNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Explanation of Solution**:
- Line X: Does Y because Z
- Line A: Implements pattern from Chapter N
- Common variations: [Alternative approaches]

---

## Validation Checklist

How to check if the solution is correct:

- [ ] Code builds without errors: `colcon build` succeeds
- [ ] Code runs without crashes: `ros2 run` completes
- [ ] Output matches specification: `ros2 topic echo` shows expected messages
- [ ] Code follows ROS 2 style guide: Comments, naming conventions
- [ ] Performance acceptable: No excessive logging, reasonable CPU usage

**Expected Output Example**:
```
[INFO] Publishing: "Hello, World!"
[INFO] Publishing: "Hello, World!"
```

---

## Extension Challenges (Optional)

For students who want to push further:

**Challenge 1**: Modify the code to [more complex requirement]
- Hints: Consider [advanced concept]
- Success criteria: [specific output or behavior]

**Challenge 2**: Combine this with code from Chapter X
- Integration point: [where to connect]
- Expected behavior: [what should happen]

**Challenge 3**: Performance optimization
- Current issue: [hypothetical problem]
- Solution approach: [general guidance]

---

## Common Mistakes

Errors students might encounter (from `content-evaluation-framework` feedback):

**Mistake 1**: [Description]
- Symptom: Code builds but doesn't publish
- Root cause: [explanation]
- Fix: [corrected code snippet]

**Mistake 2**: [Description]
- Symptom: ModuleNotFoundError
- Root cause: Missing import or package configuration
- Fix: Ensure setup.py includes all dependencies

**Mistake 3**: [Description]
- Symptom: Topic never receives messages
- Root cause: Publisher created after subscription, or wrong QoS settings
- Fix: [specific code change]

---

## Related Concepts

Link to other parts of the textbook:

- **Previous**: Chapter N, Section X covers prerequisites
- **Next**: Chapter M, Section Y builds on this
- **Related**: Alternative approach in Chapter P, Exercise Q

---

## References

- ROS 2 API: [link to rclpy API]
- Example Repo: [link to code-examples/ros2_packages/]
- Tutorial: [link to external resource]

---

## Metadata

**Skills Used**: ros2-code-generator, exercise-designer
**Package Location**: `code-examples/ros2_packages/[package-name]/`
**Solution File**: `book/docs/module-1/ch[N]/exercises.md`
**Est. Completion Time**: [Minutes]
**Bloom's Level**: Remember | Understand | Apply | Analyze | Evaluate | Create
