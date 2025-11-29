---
title: Section 3 - Actions for Goal-Reaching
---

# Section 3: Actions for Goal-Reaching Behaviors

**Estimated Reading Time**: 10 minutes

## When to Use Actions

Use **actions** for goal-oriented tasks that:
- Take time to complete (not instantaneous)
- Need progress feedback during execution
- Might be canceled mid-task
- Have a final result

Examples:
- Navigate robot to a location (get position feedback)
- Pick and place object (report gripper position)
- Wait for X seconds (report elapsed time)
- Any long-running goal with feedback

Don't use actions for:
- One-shot computations (use services)
- Continuous streams (use topics)

## Action Pattern vs Alternatives

### Service Pattern
```
Client -> Request
Server -> Response (waits for answer)
```
Problem: No feedback during execution, no cancel.

### Topic Pattern
```
Publisher -> Message -> Subscriber (continuous stream)
```
Problem: Not goal-oriented, no goal completion signal.

### Action Pattern
```
Client -> Goal
Server -> [Feedback, Feedback, ...] -> Result
         (can cancel mid-execution)
```
Solution: Progress updates, cancellation, goal completion.

## Action Components

**Goal**: What the client requests
```python
goal: target_position = 10.0
```

**Feedback**: Progress updates during execution
```python
feedback: current_position = 3.5  # "Currently at 3.5"
feedback: current_position = 5.0  # "Currently at 5.0"
```

**Result**: Final outcome
```python
result: success = True
result: final_position = 10.0
```

## Action Server

An action server:
1. Accepts goals from clients
2. Validates the goal
3. Executes the action (possibly over time)
4. Sends feedback during execution
5. Returns a result when done

```python
class MoveActionServer(Node):
    def __init__(self):
        super().__init__('move_server')
        # Create action server here

    def execute_goal(self, goal):
        # Execute the action
        # Send feedback periodically
        # Return result when complete
```

## Action Client

An action client:
1. Sends a goal to the server
2. Receives feedback updates
3. Gets the final result
4. Can cancel the action

```python
class MoveActionClient(Node):
    def __init__(self):
        super().__init__('move_client')
        # Create action client here

    def send_goal(self, target_position):
        # Send goal to server
        # Wait for result
        # Handle feedback
```

## Complete Example

See `move_action_server.py` and `move_action_client.py` in code-examples:

**Server Flow:**
1. Accept goal (e.g., "move to position 10")
2. Execute action (move robot step by step)
3. Send feedback ("Now at position 3", "Now at position 5")
4. Return result when done ("Reached position 10")

**Client Flow:**
1. Send goal to server
2. Receive feedback updates
3. Monitor progress
4. Get final result

## Real-World Use Cases

### Navigation
```
Client goal: "Navigate to (x=10, y=20)"
Server feedback: "Currently at (x=2, y=5)"
Server feedback: "Currently at (x=5, y=10)"
Server result: "Arrived at destination" or "Blocked by obstacle"
```

### Manipulation
```
Client goal: "Pick up object at position X"
Server feedback: "Gripper moving to X"
Server feedback: "Gripper at target, closing"
Server result: "Object grasped" or "Grasp failed"
```

### Waiting
```
Client goal: "Wait 5 seconds"
Server feedback: "1 second elapsed"
Server feedback: "2 seconds elapsed"
Server feedback: "5 seconds elapsed"
Server result: "Wait complete"
```

## Key Differences from Chapter 1

**Chapter 1 (Services):**
- Synchronous: Client waits for immediate response
- No feedback during execution
- Used for request/response patterns

**Chapter 3 (Actions):**
- Asynchronous: Client sends goal and continues
- Gets feedback during execution
- Used for goal-oriented behaviors
- Supports cancellation

## Key Takeaways

- Actions are for goal-oriented, long-running tasks
- Action pattern: Goal -> [Feedback...] -> Result
- Action servers execute goals and send feedback
- Action clients send goals and receive results
- Use actions for navigation, manipulation, waiting
- Actions support progress feedback and cancellation

**Review**: [Summary](./summary.md)

