# Implementation Plan: Docusaurus Textbook (Phase 1 - Module 1)

**Branch**: `001-docusaurus-textbook` | **Date**: 2025-11-29 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/001-docusaurus-textbook/spec.md`

**Phase 1 Scope**: Create Docusaurus-based textbook with Module 1 (3 chapters) using Claude Code skills/subagents. Chatbot, authentication, and bonus features deferred to Phase 2+.

---

## Summary

Create a high-quality educational textbook for Physical AI & Humanoid Robotics using Docusaurus 3.x, deployed to GitHub Pages. **Phase 1 delivers Module 1** with 3 complete chapters covering ROS 2 fundamentals, created systematically using Claude Code skills (`exercise-designer`, `code-example-generator`, `ros2-code-generator`) and subagents (`code-explainer`, `urdf-designer`).

**Primary Requirement**: Deliver Module 1 (Ch1: ROS 2 Basics, Ch2: URDF Robot Description, Ch3: Python Integration) with tested code examples, exercises, diagrams, and deployment to GitHub Pages.

**Technical Approach**:
1. Set up Docusaurus 3.x site with custom theme
2. Create content templates for chapters, code examples, exercises
3. Use `ros2-code-generator` skill to create tested ROS 2 code
4. Use `exercise-designer` skill to create progressive exercises
5. Use `code-explainer` subagent to annotate code pedagogically
6. Deploy to GitHub Pages via GitHub Actions workflow

---

## Technical Context

**Language/Version**:
- Node.js 18+ (Docusaurus build environment)
- TypeScript 5.x (custom React components)
- Python 3.10+ (ROS 2 code examples)
- C++ 17 (ROS 2 C++ code examples)

**Primary Dependencies**:
- Docusaurus 3.x (static site generator)
- React 18+ (Docusaurus framework)
- Prism.js (syntax highlighting for code blocks)
- Mermaid.js (diagrams - optional for Phase 1)
- rclpy (ROS 2 Python library - for code examples)
- rclcpp (ROS 2 C++ library - for code examples)

**Storage**:
- Static markdown files in `book/docs/`
- Code examples in `code-examples/ros2_packages/`
- Images/diagrams in `book/static/img/`
- Git repository for version control

**Testing**:
- Jest (React component tests - optional for Phase 1)
- pytest (ROS 2 Python code validation)
- `colcon test` (ROS 2 package testing)
- Manual: Deploy preview and visual QA

**Target Platform**:
- Web browsers (Chrome, Firefox, Safari, Edge)
- Mobile responsive (320px minimum width)
- GitHub Pages hosting (static HTML/CSS/JS)

**Project Type**:
- Static documentation site (Docusaurus)
- Separate code examples repository structure
- Web application architecture (frontend only for Phase 1)

**Performance Goals**:
- Page load <2 seconds (Lighthouse score >90)
- Build time <60 seconds for incremental builds
- Search index generation <30 seconds
- Image optimization (WebP, lazy loading)

**Constraints**:
- GitHub Pages max 1GB site size (well within limits)
- Free tier hosting (no server-side processing)
- No database (static content only for Phase 1)
- Mobile-first responsive design

**Scale/Scope**:
- **Phase 1**: 3 chapters (~5,000-7,500 words total)
- **Phase 1**: 10-15 code examples (ROS 2 Python/C++)
- **Phase 1**: 5-10 exercises with solutions
- **Phase 1**: 10-15 diagrams/screenshots
- **Future**: Expand to 20-25 chapters (Modules 2-4)

---

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Gate 1: Technical Accuracy
- [x] All ROS 2 code specifies version (Humble or Iron on Ubuntu 22.04)
- [x] Gazebo Classic 11 or Gazebo Sim specified for simulations
- [x] Hardware specifications verified (deferred to future modules)
- [x] All code examples will be tested before publication

**Status**: ✅ PASS - Version requirements clear, testing workflow defined

### Gate 2: Learning Progression
- [x] Chapter 1 prerequis dependencies: None (introduction)
- [x] Chapter 2 prerequisites: Chapter 1 (ROS 2 basics required)
- [x] Chapter 3 prerequisites: Chapters 1-2 (URDF + ROS 2)
- [x] Complexity appropriate: Weeks 1-2 tier (foundational)
- [x] Learning objectives: TBD in content creation (use `learning-objectives` skill)
- [x] Exercises integrate theory and practice

**Status**: ✅ PASS - Natural progression validated, prerequisites clear

### Gate 3: Simulation-First
- [x] ROS 2 concepts introduced with simulation examples
- [x] Gazebo integration in Chapter 2 (URDF visualization)
- [x] Chapter 3 Python code controls simulated robots
- [x] No physical hardware required for Phase 1

**Status**: ✅ PASS - Simulation-first workflow documented

### Gate 4: AI-Native Workflow
- [x] Spec.md complete with learning objectives per chapter
- [x] Code examples reference specifications
- [x] Skills/subagents used for content creation:
  - `ros2-code-generator`: Generate tested ROS 2 code
  - `exercise-designer`: Create exercises from learning objectives
  - `code-explainer`: Annotate code with pedagogical comments
  - `urdf-designer`: Create valid URDF robot descriptions
- [x] Content created using spec-driven methodology

**Status**: ✅ PASS - AI-native workflow central to Phase 1

### Gate 5: Hardware Reality
- [x] Phase 1 requires NO physical hardware (simulation only)
- [x] Software requirements clearly stated (ROS 2 Humble, Ubuntu 22.04)
- [x] Cloud alternatives documented (deferred to later chapters)
- [x] Docker images planned for environment setup

**Status**: ✅ PASS - Software-only Phase 1, accessible to all students

### Gate 6: RAG Integration
- [ ] RAG chatbot deferred to Phase 2
- [ ] Content structured for future RAG indexing (512 token chunks)
- [ ] Markdown format compatible with chunking

**Status**: ⏸️ DEFERRED - Chatbot in Phase 2, content structured for future compatibility

### Gate 7: Personalization
- [ ] Personalization deferred to Phase 2
- [x] Content written at beginner-intermediate level
- [x] Optional advanced sections noted for future personalization

**Status**: ⏸️ DEFERRED - Phase 2 feature, content structured for future adaptation

### Gate 8: Multilingual Access
- [ ] Urdu translation deferred to Phase 3
- [x] Technical terms documented in glossary for future translation
- [x] Content structure translation-friendly (markdown)

**Status**: ⏸️ DEFERRED - Phase 3 feature, translation-ready structure

### Gate 9: Reusable Intelligence
- [x] **PRIMARY FOCUS**: Skills/subagents used extensively
- [x] Skills to use:
  - `ros2-code-generator`: Generate ROS 2 packages
  - `exercise-designer`: Create progressive exercises
  - `code-explainer`: Annotate code pedagogically
  - `urdf-designer`: Create robot URDF files
  - `learning-objectives`: Map to Bloom's taxonomy
  - `summary-generator`: Generate lesson summaries
  - `content-evaluation-framework`: Evaluate chapter quality
- [x] Subagents documented for reuse
- [x] Workflow captured in planning artifacts

**Status**: ✅ PASS - Skills/subagents central to content creation strategy

---

**Overall Constitution Check**: ✅ **PASS with deferrals**

- **Passed**: 6 gates (1, 2, 3, 4, 5, 9)
- **Deferred**: 3 gates (6, 7, 8) - Phase 2+ features as planned
- **Failed**: 0 gates

**Justification for deferrals**: Chatbot (Gate 6), personalization (Gate 7), and translation (Gate 8) explicitly scoped to Phase 2+ per clarified specification. Phase 1 focuses on high-quality Module 1 content creation using skills/subagents.

---

## Project Structure

### Documentation (this feature)

```text
specs/001-docusaurus-textbook/
├── spec.md              # Feature specification (complete)
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (Docusaurus best practices)
├── data-model.md        # Phase 1 output (content structure model)
├── quickstart.md        # Phase 1 output (developer quickstart)
├── contracts/           # Phase 1 output (content contracts/templates)
│   ├── chapter-template.md
│   ├── exercise-template.md
│   └── code-example-template.md
└── tasks.md             # Phase 2 output (/sp.tasks command)
```

### Source Code (repository root)

```text
physical-ai-textbook/
├── book/                           # Docusaurus site
│   ├── docs/                      # Chapter markdown files
│   │   ├── intro.md              # Homepage introduction
│   │   └── module-1/             # ROS 2 Fundamentals
│   │       ├── index.md          # Module 1 overview
│   │       ├── ch1-ros2-basics/
│   │       │   ├── index.md      # Chapter 1 introduction
│   │       │   ├── 01-what-is-ros2.md
│   │       │   ├── 02-nodes-topics.md
│   │       │   ├── 03-services-actions.md
│   │       │   └── exercises.md
│   │       ├── ch2-urdf/
│   │       │   ├── index.md
│   │       │   ├── 01-robot-description.md
│   │       │   ├── 02-joints-links.md
│   │       │   ├── 03-gazebo-integration.md
│   │       │   └── exercises.md
│   │       └── ch3-python-integration/
│   │           ├── index.md
│   │           ├── 01-rclpy-basics.md
│   │           ├── 02-publishers-subscribers.md
│   │           ├── 03-control-robot.md
│   │           └── exercises.md
│   ├── src/                       # Custom React components
│   │   ├── components/
│   │   │   ├── CodeExample.tsx   # Enhanced code block
│   │   │   └── ExerciseBlock.tsx # Interactive exercise
│   │   ├── css/
│   │   │   └── custom.css        # Theme customization
│   │   └── theme/                # Docusaurus theme overrides
│   ├── static/                    # Static assets
│   │   ├── img/                  # Images, diagrams
│   │   └── examples/             # Downloadable code examples
│   ├── docusaurus.config.js      # Site configuration
│   ├── sidebars.js               # Navigation structure
│   └── package.json              # Dependencies
│
├── code-examples/                 # ROS 2 code examples (separate from book)
│   └── ros2_packages/
│       ├── hello_world_py/       # Chapter 1 example
│       │   ├── package.xml
│       │   ├── setup.py
│       │   └── hello_world_py/
│       │       └── hello_node.py
│       ├── simple_robot_description/  # Chapter 2 example
│       │   ├── urdf/
│       │   │   └── simple_robot.urdf
│       │   └── launch/
│       │       └── display.launch.py
│       └── robot_control_py/     # Chapter 3 example
│           ├── package.xml
│           ├── setup.py
│           └── robot_control_py/
│               └── velocity_controller.py
│
├── .github/
│   └── workflows/
│       └── deploy.yml            # GitHub Pages deployment
│
├── .specify/                      # SpecKit Plus (existing)
├── .claude/                       # Skills and subagents (existing)
├── specs/                         # Specifications (this folder)
└── README.md                      # Project overview
```

**Structure Decision**: **Web application (static documentation site)** structure selected. Docusaurus site in `book/` directory with docs, custom components, and static assets. ROS 2 code examples separated in `code-examples/` for clarity and independent testing. This structure supports:
- Clear separation of content (markdown) and presentation (React components)
- Easy navigation for students (module → chapter → section hierarchy)
- Independent ROS 2 package testing via `colcon`
- GitHub Pages deployment from `book/build/` output

---

## Complexity Tracking

> **No violations** - All constitution gates passed or deferred per plan. No complexity justification required.

---

## Phase 0: Research & Discovery

**Goal**: Resolve all "NEEDS CLARIFICATION" items and establish best practices for Docusaurus + ROS 2 content creation.

### Research Tasks

#### R1: Docusaurus 3.x Best Practices for Technical Documentation

**Question**: What are the best practices for structuring technical educational content in Docusaurus 3.x?

**Research Approach**:
- Review official Docusaurus documentation (https://docusaurus.io/docs)
- Analyze successful technical documentation sites using Docusaurus:
  - React documentation (https://react.dev)
  - Jest documentation (https://jestjs.io)
  - ROS 2 documentation style (https://docs.ros.org)
- Identify patterns for code examples, exercises, and progressive learning

**Key Questions**:
1. How to structure multi-level hierarchy (module → chapter → section)?
2. Best practices for code example presentation?
3. Plugin ecosystem for enhanced features (tabs, admonitions, diagrams)?
4. Performance optimization for technical content?

**Output**: `research.md` section on Docusaurus architecture decisions

---

#### R2: ROS 2 Educational Content Structure

**Question**: How should ROS 2 concepts be taught progressively to beginners?

**Research Approach**:
- Review ROS 2 official tutorials (https://docs.ros.org/en/humble/Tutorials.html)
- Analyze "The Construct" online courses structure
- Review academic ROS 2 course syllabi
- Identify common student pain points and effective teaching sequences

**Key Questions**:
1. What's the optimal order: nodes → topics → services or different?
2. When to introduce URDF (before or after basic ROS 2 concepts)?
3. Python-first vs C++-first approach for beginners?
4. Balance between theory and hands-on practice?

**Output**: `research.md` section on pedagogical structure for Module 1

---

#### R3: Claude Code Skills/Subagents Workflow for Content Creation

**Question**: What's the optimal workflow for using Claude Code skills to create educational content systematically?

**Research Approach**:
- Review existing skills documentation in `.claude/skills/`
- Map skills to content creation tasks:
  - `ros2-code-generator` → Generate tested ROS 2 packages
  - `exercise-designer` → Create exercises from learning objectives
  - `code-explainer` → Annotate code with pedagogical comments
  - `urdf-designer` → Create valid URDF files
  - `learning-objectives` → Define learning objectives per chapter
  - `summary-generator` → Generate chapter summaries
- Define workflow: spec → learning objectives → content → code → exercises → review

**Key Questions**:
1. In what order should skills be invoked?
2. How to ensure consistency across chapters?
3. How to validate skill outputs (code testing, exercise quality)?
4. Integration points between skills (e.g., code-explainer after ros2-code-generator)?

**Output**: `research.md` section on skills-driven content creation workflow

---

#### R4: GitHub Pages Deployment for Docusaurus

**Question**: What's the best GitHub Actions workflow for deploying Docusaurus to GitHub Pages?

**Research Approach**:
- Review Docusaurus deployment documentation
- Analyze existing GitHub Actions workflows for Docusaurus sites
- Identify caching strategies for faster builds
- Determine branch strategy (main vs gh-pages branch)

**Key Questions**:
1. Build on push to main or manual trigger?
2. Deploy to gh-pages branch or GitHub Pages from build artifact?
3. Caching strategy for `node_modules` and build artifacts?
4. Environment variables needed (base URL, etc.)?

**Output**: `research.md` section on deployment workflow

---

#### R5: Code Example Testing Strategy for ROS 2 Packages

**Question**: How to ensure all ROS 2 code examples are tested and executable before publication?

**Research Approach**:
- Review ROS 2 testing best practices (`ament_python`, `gtest`)
- Identify minimal testing for educational examples (not production code)
- Define acceptance criteria: code must build and run, basic functionality validated
- Determine Docker strategy for reproducible testing environment

**Key Questions**:
1. Test every code example or sample subset?
2. Unit tests vs integration tests for educational code?
3. Docker image for consistent ROS 2 Humble environment?
4. CI/CD for code examples (separate from book build)?

**Output**: `research.md` section on code validation strategy

---

### Research Consolidation

**Decision Summary** (to be filled after research):

| Topic | Decision | Rationale | Alternatives Considered |
|-------|----------|-----------|-------------------------|
| Docusaurus Structure | TBD | TBD | TBD |
| ROS 2 Teaching Order | TBD | TBD | TBD |
| Skills Workflow | TBD | TBD | TBD |
| Deployment Strategy | TBD | TBD | TBD |
| Testing Strategy | TBD | TBD | TBD |

**Output File**: `specs/001-docusaurus-textbook/research.md`

---

## Phase 1: Design & Contracts

**Goal**: Design content structure, chapter templates, and deployment configuration.

**Prerequisites**: `research.md` complete with decisions documented

### D1: Content Data Model

**Task**: Define the structure of educational content (chapters, sections, exercises, code examples)

**Entity: Chapter**
- `id`: string (e.g., "module-1-ch1-ros2-basics")
- `title`: string
- `module`: string (e.g., "module-1")
- `order`: number (1, 2, 3)
- `prerequisites`: string[] (IDs of prerequisite chapters)
- `learning_objectives`: LearningObjective[]
- `sections`: Section[]
- `exercises`: Exercise[]
- `estimated_time`: number (minutes)

**Entity: Section**
- `id`: string
- `title`: string
- `content`: markdown string
- `code_examples`: CodeExample[]
- `diagrams`: Diagram[]

**Entity: CodeExample**
- `id`: string
- `language`: "python" | "cpp" | "bash"
- `title`: string
- `description`: string
- `code`: string
- `file_path`: string (in code-examples/)
- `ros_version`: "humble" | "iron"
- `tested`: boolean
- `dependencies`: string[]

**Entity: Exercise**
- `id`: string
- `type`: "conceptual" | "coding" | "simulation"
- `difficulty`: "beginner" | "intermediate" | "advanced"
- `prompt`: string
- `hints`: string[]
- `solution`: string
- `validation_criteria`: string[]

**Output**: `specs/001-docusaurus-textbook/data-model.md`

---

### D2: Chapter Template Contract

**Task**: Define the standard structure for every chapter markdown file

**Template Structure**:
```markdown
---
sidebar_position: {order}
---

# {Chapter Title}

## Learning Objectives

By the end of this chapter, you will:
- [Objective 1 using Bloom's taxonomy action verb]
- [Objective 2]
- [Objective 3]

**Prerequisites**: [List prerequisite chapters]
**Estimated Time**: [X minutes]

---

## Introduction

[Brief overview of what this chapter covers and why it matters]

---

## Section 1: {Section Title}

[Content with explanations, diagrams, code examples]

### Code Example: {Example Title}

```python
# Code with inline comments explaining each part
```

**Explanation**: [Detailed explanation of the code]

**Try it yourself**: [Instructions for running the code]

---

## Section 2: {Section Title}

[Continue pattern...]

---

## Exercises

### Exercise 1: {Title} (Difficulty: Beginner)

**Task**: [Clear description of what to build/implement]

**Hints**:
1. [Hint 1]
2. [Hint 2]

<details>
<summary>Solution</summary>

[Complete solution with explanation]

</details>

---

## Summary

[Key takeaways from this chapter]

---

## Next Steps

Continue to [Next Chapter Title](../link)
```

**Output**: `specs/001-docusaurus-textbook/contracts/chapter-template.md`

---

### D3: Exercise Template Contract

**Task**: Standardize exercise structure for consistency and pedagogical effectiveness

**Template** (to be filled by `exercise-designer` skill):
```markdown
### Exercise {N}: {Title} (Difficulty: {Level})

**Learning Objective**: [Which objective from chapter this exercise addresses]

**Task**:
[Clear, actionable description of what student should accomplish]

**Requirements**:
- [ ] Requirement 1
- [ ] Requirement 2
- [ ] Requirement 3

**Starter Code** (if applicable):
```python
# Partial implementation to guide students
```

**Hints**:
1. [Progressive hint 1 - conceptual guidance]
2. [Progressive hint 2 - implementation direction]
3. [Progressive hint 3 - specific code guidance]

**Expected Output**:
[Description or example of correct output/behavior]

**Validation**:
- Test case 1: [Input] → [Expected output]
- Test case 2: [Input] → [Expected output]

<details>
<summary>Solution</summary>

**Explanation**:
[Step-by-step explanation of solution approach]

**Complete Code**:
```python
# Fully working solution with detailed comments
```

**Why this works**:
[Conceptual explanation reinforcing learning objective]

</details>
```

**Output**: `specs/001-docusaurus-textbook/contracts/exercise-template.md`

---

### D4: Code Example Template Contract

**Task**: Standardize ROS 2 code example structure for testing and pedagogy

**Template**:
```markdown
### Code Example: {Title}

**Concept Demonstrated**: [What ROS 2 concept this illustrates]

**ROS 2 Version**: Humble (Ubuntu 22.04)

**Package**: `{package_name}`

**File**: `code-examples/ros2_packages/{package_name}/{file_path}`

**Code**:
```python
"""
{Package description}

This example demonstrates:
- [Key concept 1]
- [Key concept 2]
"""

import rclpy
from rclpy.node import Node

class ExampleNode(Node):
    def __init__(self):
        super().__init__('example_node')
        # [Pedagogical comment explaining initialization]

    def example_method(self):
        # [Pedagogical comment explaining this method]
        pass

def main(args=None):
    rclpy.init(args=args)
    node = ExampleNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Line-by-Line Explanation**:
- **Line X**: [Explanation of what this line does and why]
- **Line Y**: [Explanation]

**How to Run**:
```bash
# Build the package
cd ~/ros2_ws
colcon build --packages-select {package_name}

# Source the workspace
source install/setup.bash

# Run the node
ros2 run {package_name} {node_name}
```

**Expected Output**:
```
[INFO] [timestamp]: Expected log message...
```

**Troubleshooting**:
- **Error X**: [Common error and solution]
- **Error Y**: [Common error and solution]

**Further Exploration**:
- Try modifying [specific part] to [suggestion]
- Combine with [related concept] from [other example]
```

**Output**: `specs/001-docusaurus-textbook/contracts/code-example-template.md`

---

### D5: Docusaurus Configuration

**Task**: Create complete `docusaurus.config.js` for the site

**Key Configuration**:
```javascript
module.exports = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'From Simulation to Reality',
  url: 'https://92Bilal26.github.io',
  baseUrl: '/physical-ai-textbook/',
  organizationName: '92Bilal26',
  projectName: 'physical-ai-textbook',

  themeConfig: {
    navbar: {
      title: 'Physical AI',
      items: [
        {to: '/docs/intro', label: 'Start Learning', position: 'left'},
        {to: '/docs/module-1', label: 'Module 1: ROS 2', position: 'left'},
        {href: 'https://github.com/92Bilal26/physical-ai-textbook', label: 'GitHub', position: 'right'},
      ],
    },
    footer: {
      copyright: `Built with Docusaurus. © ${new Date().getFullYear()}`,
    },
    prism: {
      theme: require('prism-react-renderer/themes/github'),
      darkTheme: require('prism-react-renderer/themes/dracula'),
      additionalLanguages: ['python', 'cpp', 'bash', 'xml'],
    },
  },

  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: require.resolve('./sidebars.js'),
          editUrl: 'https://github.com/92Bilal26/physical-ai-textbook/edit/main/book/',
        },
        theme: {
          customCss: require.resolve('./src/css/custom.css'),
        },
      },
    ],
  ],
};
```

**Output**: Documented in `quickstart.md`

---

### D6: GitHub Pages Deployment Workflow

**Task**: Create `.github/workflows/deploy.yml` for automated deployment

**Workflow**:
```yaml
name: Deploy to GitHub Pages

on:
  push:
    branches: [main]
  workflow_dispatch:

jobs:
  deploy:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v3

      - name: Setup Node.js
        uses: actions/setup-node@v3
        with:
          node-version: 18
          cache: 'npm'
          cache-dependency-path: book/package-lock.json

      - name: Install dependencies
        run: cd book && npm ci

      - name: Build website
        run: cd book && npm run build

      - name: Deploy to GitHub Pages
        uses: peaceiris/actions-gh-pages@v3
        with:
          github_token: ${{ secrets.GITHUB_TOKEN }}
          publish_dir: ./book/build
```

**Output**: Documented in `quickstart.md`

---

### D7: Developer Quickstart Guide

**Task**: Create `quickstart.md` for developers to set up and contribute

**Content Outline**:
1. Prerequisites (Node.js 18+, ROS 2 Humble, Ubuntu 22.04)
2. Repository setup (`git clone`, `npm install`)
3. Local development (`npm start` for live preview)
4. Content creation workflow (using skills)
5. Testing ROS 2 code examples
6. Submitting changes (PR process)

**Output**: `specs/001-docusaurus-textbook/quickstart.md`

---

### D8: Skills-Driven Content Creation Workflow

**Task**: Document the systematic workflow for creating chapters using Claude Code skills

**Workflow Steps**:

1. **Define Learning Objectives** (use `learning-objectives` skill)
   - Input: Chapter topic (e.g., "ROS 2 Basics")
   - Output: 3-5 measurable learning objectives using Bloom's taxonomy
   - Example: "Students will be able to: Create a basic ROS 2 node using rclpy (Apply level)"

2. **Generate ROS 2 Code Examples** (use `ros2-code-generator` skill)
   - Input: Learning objectives + ROS 2 concept
   - Output: Tested ROS 2 package with proper structure
   - Validation: `colcon build` and `ros2 run` successful

3. **Annotate Code Pedagogically** (use `code-explainer` subagent)
   - Input: Generated ROS 2 code
   - Output: Code with inline pedagogical comments explaining concepts
   - Focus: Why not just what, common pitfalls, design decisions

4. **Create Progressive Exercises** (use `exercise-designer` skill)
   - Input: Learning objectives + code examples
   - Output: 3-5 exercises with clear success criteria, hints, solutions
   - Difficulty: Beginner → Intermediate progression

5. **Generate URDF (Chapter 2)** (use `urdf-designer` subagent)
   - Input: Robot description requirements
   - Output: Valid URDF with proper joints, links, collision models
   - Validation: `check_urdf` and Gazebo visualization

6. **Write Chapter Content** (manual with AI assistance)
   - Follow chapter-template.md
   - Integrate generated code examples and exercises
   - Add explanations, diagrams, context

7. **Generate Summary** (use `summary-generator` skill)
   - Input: Completed chapter markdown
   - Output: Key takeaways, mental models, common mistakes

8. **Evaluate Quality** (use `content-evaluation-framework` skill)
   - Input: Completed chapter
   - Output: Quality score across 6 categories
   - Iterate if score <80%

**Output**: Documented in `data-model.md` and `quickstart.md`

---

## Phase 2: Implementation Preparation

**Output**: All design artifacts complete, ready for `/sp.tasks` command

**Deliverables from Phase 1**:
1. ✅ `research.md` - Decisions on Docusaurus, ROS 2 teaching, skills workflow, deployment, testing
2. ✅ `data-model.md` - Content structure (Chapter, Section, Exercise, CodeExample entities)
3. ✅ `contracts/chapter-template.md` - Standard chapter structure
4. ✅ `contracts/exercise-template.md` - Standard exercise structure
5. ✅ `contracts/code-example-template.md` - Standard code example structure
6. ✅ `quickstart.md` - Developer setup and workflow guide
7. ✅ Agent context updated with Docusaurus, ROS 2 Humble, skills workflow

**Next Command**: `/sp.tasks`

This will generate `tasks.md` with specific implementation tasks for:
- Docusaurus site setup
- Module 1 Chapter 1 content creation
- Module 1 Chapter 2 content creation
- Module 1 Chapter 3 content creation
- ROS 2 code examples
- Exercises and solutions
- GitHub Pages deployment
- Quality validation

---

## Risk Analysis

### Technical Risks

| Risk | Impact | Mitigation | Owner |
|------|--------|------------|-------|
| ROS 2 code examples fail to build/run | High | Test all code in Docker container with ROS 2 Humble before publication | Developer |
| Docusaurus build failures | Medium | Use stable Docusaurus 3.x version, pin dependencies in package-lock.json | Developer |
| Skills produce inconsistent output | Medium | Define clear input contracts, validate outputs against templates | Developer |
| GitHub Pages deployment issues | Low | Test deployment workflow in fork first, have Vercel as backup | Developer |

### Content Risks

| Risk | Impact | Mitigation | Owner |
|------|--------|------------|-------|
| Chapters too advanced for beginners | High | Use `content-evaluation-framework` skill, test with beginner students | Content Creator |
| Code examples become outdated | Medium | Pin to ROS 2 Humble LTS (support until 2027), document versions clearly | Developer |
| Exercises too difficult/too easy | Medium | Use `exercise-designer` skill with difficulty calibration, get feedback | Content Creator |

### Timeline Risks

| Risk | Impact | Mitigation | Owner |
|------|--------|------------|-------|
| Content creation slower than expected | Medium | Start with 3-chapter MVP (Phase 1), expand iteratively | Project Manager |
| Skills/subagents require debugging | Low | Allocate buffer time for skill refinement, have manual fallback | Developer |

---

## Success Criteria (Phase 1)

**Must Have (MVP)**:
- [ ] Docusaurus site deployed to https://92Bilal26.github.io/physical-ai-textbook/
- [ ] Module 1 Chapter 1 complete (ROS 2 Basics) with 3-4 sections, 3+ code examples, 3+ exercises
- [ ] Module 1 Chapter 2 complete (URDF) with 3-4 sections, 2+ URDF examples, 3+ exercises
- [ ] Module 1 Chapter 3 complete (Python Integration) with 3-4 sections, 3+ code examples, 3+ exercises
- [ ] All ROS 2 code examples tested and executable (build + run successful)
- [ ] Mobile-responsive design (320px+)
- [ ] Page load <2 seconds

**Should Have (Quality)**:
- [ ] All chapters evaluated with `content-evaluation-framework` skill (score ≥80%)
- [ ] Learning objectives defined for each chapter (using Bloom's taxonomy)
- [ ] Code examples annotated with pedagogical comments (via `code-explainer` subagent)
- [ ] Exercises have progressive difficulty (beginner → intermediate)
- [ ] Chapter summaries generated (via `summary-generator` skill)

**Nice to Have (Polish)**:
- [ ] Custom CSS theme (robotics-themed colors)
- [ ] Diagrams for key concepts (Mermaid.js or static images)
- [ ] Interactive code examples (copy button, language tabs)
- [ ] Search functionality working

---

## Appendix: Skills & Subagents Reference

### Skills Used in Phase 1

| Skill | Purpose | Input | Output |
|-------|---------|-------|--------|
| `learning-objectives` | Define learning objectives per chapter | Chapter topic | 3-5 measurable objectives (Bloom's taxonomy) |
| `ros2-code-generator` | Generate tested ROS 2 code | Requirements, ROS 2 concept | Python/C++ package with proper structure |
| `exercise-designer` | Create progressive exercises | Learning objectives | Exercises with hints, solutions, validation |
| `summary-generator` | Generate chapter summaries | Completed chapter markdown | Key takeaways, mental models |
| `content-evaluation-framework` | Evaluate chapter quality | Completed chapter | Quality score (6 categories) |

### Subagents Used in Phase 1

| Subagent | Purpose | Input | Output |
|----------|---------|-------|--------|
| `code-explainer` | Annotate code with pedagogical comments | ROS 2 code | Code with explanatory comments |
| `urdf-designer` | Create valid URDF robot descriptions | Robot requirements | URDF XML file |

---

**Plan Status**: ✅ COMPLETE - Ready for Phase 0 (Research) execution

**Next Action**: Execute Phase 0 research tasks to fill `research.md`, then proceed to Phase 1 design to create contracts and data model.
