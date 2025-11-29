# Contributing to Physical AI Textbook

Thank you for your interest in contributing! This document provides guidelines for participating in the project.

## Ways to Contribute

### 1. Report Issues
- Found a bug? [Open an issue](https://github.com/92Bilal26/physical-ai-textbook/issues)
- Suggest improvements or new features
- Report outdated or incorrect information

### 2. Add Content
- Write new chapters or sections
- Create additional exercises
- Add code examples
- Improve existing explanations

### 3. Fix Bugs
- Correct typos and grammar
- Fix broken links
- Update outdated information
- Improve code examples

### 4. Improve Quality
- Suggest pedagogical improvements
- Test code examples
- Improve diagrams and visuals
- Enhance accessibility

## Getting Started

### Fork and Clone
```bash
git clone https://github.com/YOUR_USERNAME/physical-ai-textbook.git
cd physical-ai-textbook
git checkout -b feature/your-feature-name
```

### Build Locally
```bash
# Documentation
cd book && npm install && npm run build

# Code examples
cd code-examples/ros2_packages
colcon build
```

### Make Changes
- Edit files in `book/docs/` for content
- Edit files in `code-examples/` for code
- Update README.md or other docs as needed

### Test Changes
```bash
# Build documentation
cd book && npm run build

# Test code examples
cd code-examples
colcon build
colcon test  # Run tests
```

### Commit and Push
```bash
git add .
git commit -m "Brief description of changes"
git push origin feature/your-feature-name
```

### Create Pull Request
1. Go to GitHub and create a pull request
2. Describe your changes in detail
3. Reference any related issues
4. Wait for review

## Content Guidelines

### Writing Style
- Clear, concise explanations
- Beginner-friendly language
- Use analogies and mental models
- Avoid jargon without explanation

### Code Quality
- Follow Python PEP 8 style guide
- Add docstrings to functions
- Include comments explaining concepts
- Test code before submitting

### Learning Objectives
- Use Bloom's taxonomy (Remember → Create)
- Align with CEFR proficiency levels
- Ensure progressive difficulty
- Define clear success criteria

### Exercises
- Beginner (⭐), Intermediate (⭐⭐), Advanced (⭐⭐⭐)
- Include templates and hints
- Provide complete solutions
- Add success criteria

## Chapter Structure

Each chapter should include:
```
chapter-name/
├── index.md                    # Chapter introduction
├── learning-objectives.md      # 5 learning objectives
├── 01-section-one.md          # Section 1
├── 02-section-two.md          # Section 2
├── 03-section-three.md        # Section 3
├── exercises.md               # 3 progressive exercises
├── summary.md                 # Key takeaways & mental models
└── _category_.json            # Docusaurus navigation
```

## Code Example Template

```python
#!/usr/bin/env python3
"""
Module description: Brief description of what this code demonstrates.

Key Concepts:
  1. Concept 1 explanation
  2. Concept 2 explanation

Usage:
  ros2 run package_name script_name
"""

import rclpy
from rclpy.node import Node

class ExampleNode(Node):
    """
    Example node class description.
    
    This class demonstrates [specific concept].
    """
    
    def __init__(self):
        """Initialize the node with explanation of setup."""
        super().__init__('example_node')
        # Include detailed comments explaining setup
```

## Documentation Standards

### Markdown Style
- Use headers for structure (# Main, ## Sub, ### Detail)
- Bold for **important terms**
- Code blocks with language specification
- Bullet lists for options

### Code Comments
- Explain WHY, not just WHAT
- Use Socratic approach (guide thinking)
- Include example usage
- Reference pedagogical concepts

## Testing Requirements

Before submitting a pull request:

### Documentation
- [ ] Builds without warnings: `npm run build`
- [ ] All links are valid
- [ ] All code blocks are correct
- [ ] All exercises have complete solutions

### Code Examples
- [ ] All packages build: `colcon build`
- [ ] All tests pass: `colcon test`
- [ ] Code follows PEP 8
- [ ] Includes docstrings and comments

### Content
- [ ] Spelling and grammar checked
- [ ] Consistent formatting throughout
- [ ] No broken references
- [ ] Accurate technical content

## Review Process

1. **Automated Checks**: GitHub Actions runs build tests
2. **Code Review**: Maintainers review for quality and pedagogy
3. **Feedback**: Constructive comments and suggestions
4. **Approval**: Merge when requirements met

## Code of Conduct

- Be respectful and inclusive
- Assume good intentions
- Provide constructive criticism
- Help others learn

## Questions?

- Open a [GitHub Discussion](https://github.com/92Bilal26/physical-ai-textbook/discussions)
- Ask on [ROS Discourse](https://discourse.ros.org/)
- Email the maintainers

## Recognition

Contributors will be:
- Added to contributors list in README
- Acknowledged in commit messages
- Credited in release notes

## License

All contributions must be compatible with MIT License. By contributing, you agree that your work will be licensed under the same terms.

---

Thank you for contributing to making robotics education accessible to everyone!
