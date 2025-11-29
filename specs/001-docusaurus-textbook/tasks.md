---
description: "Task list for Physical AI Docusaurus Textbook - Phase 1 (Module 1)"
---

# Tasks: Physical AI & Humanoid Robotics Docusaurus Textbook (Phase 1)

**Input**: Design documents from `/specs/001-docusaurus-textbook/`
**Prerequisites**: plan.md (complete), spec.md (clarified), Phase 1 design artifacts (research.md, data-model.md, contracts/)

**Phase 1 Scope**: Module 1 (3 chapters) with tested ROS 2 code examples, exercises, and GitHub Pages deployment
**Organization**: Tasks grouped by user story (US1) to enable independent implementation and testing

---

## Format: `[ID] [P?] [US?] Description with file path`

- **[ID]**: Task identifier (T001, T002, etc.)
- **[P]**: Parallelizable (different files, no blocking dependencies)
- **[US]**: User story (US1 = User Story 1: Core Textbook Content Delivery)
- Include exact file paths for each task

---

## Implementation Strategy

**MVP Scope (Phase 1)**: Module 1 with 3 chapters
- Focus: High-quality content using skills/subagents
- Defer: Chatbot (Phase 2), personalization (Phase 2), translation (Phase 3)
- Success: 3 chapters deployed, all code tested, <2s page load

**Execution Order**:
1. **Setup (Phase 1)**: Project initialization, Docusaurus setup
2. **Foundational (Phase 2)**: Templates, content structure, deployment prep
3. **User Story 1 (Phase 3)**: Module 1 content creation (3 chapters)
4. **Polish (Phase 4)**: Quality validation, optimization

---

## Phase 1: Setup & Project Initialization

**Goal**: Initialize Docusaurus project, configure GitHub Pages, set up development environment

- [ ] T001 Create Docusaurus project in `book/` directory with `npx create-docusaurus@latest book classic --typescript`
- [ ] T002 Install dependencies in `book/` directory: `npm install` (Node.js 18+)
- [ ] T003 [P] Create project directory structure: `code-examples/ros2_packages/` for ROS 2 packages
- [ ] T004 [P] Create project directory structure: `book/static/img/` for images and diagrams
- [ ] T005 Initialize `book/docusaurus.config.js` with site configuration (title, URL, GitHub Pages settings)
- [ ] T006 Initialize `book/sidebars.js` with navigation structure for Module 1 (3 chapters)
- [ ] T007 Create `.github/workflows/deploy.yml` for GitHub Pages automated deployment
- [ ] T008 Create `book/.gitignore` to exclude node_modules, build output, sensitive files
- [ ] T009 Create root-level `.gitignore` for ROS 2 packages (build/, install/, log/, devel/)
- [ ] T010 Verify local build works: `cd book && npm run build` produces `book/build/` output

---

## Phase 2: Foundational - Templates & Content Structure

**Goal**: Create reusable templates and contracts for consistent content creation using skills/subagents

### Content Templates

- [ ] T011 Create `specs/001-docusaurus-textbook/contracts/chapter-template.md` with standard chapter structure (Learning Objectives, Sections, Exercises, Summary)
- [ ] T012 [P] Create `specs/001-docusaurus-textbook/contracts/exercise-template.md` with pedagogical exercise structure (Task, Hints, Solution, Validation)
- [ ] T013 [P] Create `specs/001-docusaurus-textbook/contracts/code-example-template.md` with ROS 2 code structure (Concept, Code, Explanation, Troubleshooting)
- [ ] T014 Create `specs/001-docusaurus-textbook/data-model.md` documenting content entities (Chapter, Section, Exercise, CodeExample)

### Theme & Styling

- [ ] T015 Create `book/src/css/custom.css` with robotics-themed colors and responsive design rules
- [ ] T016 Create `book/src/theme/` directory with Docusaurus theme customizations
- [ ] T017 [P] Create `book/src/components/CodeExample.tsx` React component for enhanced code blocks
- [ ] T018 [P] Create `book/src/components/ExerciseBlock.tsx` React component for interactive exercises

### Documentation & Deployment

- [ ] T019 Create `specs/001-docusaurus-textbook/quickstart.md` with setup instructions for developers
- [ ] T020 [P] Create `specs/001-docusaurus-textbook/research.md` with Docusaurus/ROS 2/skills workflow decisions (from Phase 0 research)
- [ ] T021 Create `README.md` in `code-examples/ros2_packages/` explaining how to test ROS 2 packages

### Validation

- [ ] T022 Test Docusaurus site locally: `cd book && npm start` opens at http://localhost:3000
- [ ] T023 Verify GitHub Pages workflow syntax: `.github/workflows/deploy.yml` validates with `act` or manual review
- [ ] T024 Verify project structure matches plan: `book/`, `code-examples/`, `.specify/`, `.github/` all present

---

## Phase 3: User Story 1 - Core Textbook Content Delivery (Priority: P1)

**Goal**: Create Module 1 with 3 chapters using Claude Code skills/subagents

**Independent Test Criteria**:
- [ ] Docusaurus site deployed to https://92Bilal26.github.io/physical-ai-textbook/
- [ ] All 3 chapters accessible and navigable
- [ ] All code examples tested and executable (ROS 2 Humble)
- [ ] Page load <2 seconds, mobile responsive

**Dependency**: Phases 1-2 complete (setup, templates, deployment ready)

---

### Chapter 1: ROS 2 Basics (Nodes, Topics, Services)

**Scope**: 3-4 sections, 3+ code examples, 3+ exercises, ~1,500-2,000 words
**Skills Used**: `learning-objectives`, `ros2-code-generator`, `code-explainer`, `exercise-designer`, `content-evaluation-framework`

#### Learning Objectives & Content Design

- [ ] T025 [US1] Use `learning-objectives` skill: Generate 3-5 Bloom's taxonomy objectives for "ROS 2 Basics (nodes, topics, services)"
- [ ] T026 [US1] Document learning objectives in `book/docs/module-1/ch1-ros2-basics/learning-objectives.md`

#### Code Examples Generation

- [ ] T027 [US1] Use `ros2-code-generator` skill: Generate basic ROS 2 Python node for "Hello, ROS 2" example
  - Output: `code-examples/ros2_packages/hello_world_py/` with package.xml, setup.py, hello_node.py
  - Requirement: Code builds with `colcon build`, executes with `ros2 run`
- [ ] T028 [US1] [P] Use `ros2-code-generator` skill: Generate ROS 2 Python publisher/subscriber example
  - Output: `code-examples/ros2_packages/pub_sub_py/` with working publisher and subscriber nodes
  - Requirement: Code builds and runs, demonstrates topic communication
- [ ] T029 [US1] [P] Use `ros2-code-generator` skill: Generate ROS 2 Python service example
  - Output: `code-examples/ros2_packages/service_example_py/` with service server and client
  - Requirement: Code builds and runs, demonstrates service communication

#### Code Annotation & Explanation

- [ ] T030 [US1] Use `code-explainer` subagent: Annotate hello_node.py with pedagogical comments explaining ROS 2 initialization, node creation, spinning
- [ ] T031 [US1] [P] Use `code-explainer` subagent: Annotate publisher/subscriber code with explanations of topics, data types, callbacks
- [ ] T032 [US1] [P] Use `code-explainer` subagent: Annotate service code with explanations of service definition, request/response

#### Chapter Content Creation

- [ ] T033 [US1] Write Chapter 1 intro section: "What is ROS 2?" (motivation, context, why it matters) in `book/docs/module-1/ch1-ros2-basics/index.md`
- [ ] T034 [US1] [P] Write Section 1: "Nodes and the Graph" with diagrams, hello_node.py example, explanation in `book/docs/module-1/ch1-ros2-basics/01-nodes.md`
- [ ] T035 [US1] [P] Write Section 2: "Topics: Publish and Subscribe" with pub_sub example, explanation in `book/docs/module-1/ch1-ros2-basics/02-topics.md`
- [ ] T036 [US1] [P] Write Section 3: "Services: Request and Response" with service example, explanation in `book/docs/module-1/ch1-ros2-basics/03-services.md`

#### Exercises Creation

- [ ] T037 [US1] Use `exercise-designer` skill: Create 3 progressive exercises for Chapter 1 (Beginner → Intermediate)
  - Exercise 1 (Beginner): Create a simple node that prints "Hello, World!" each second
  - Exercise 2 (Beginner/Intermediate): Modify hello_node.py to accept parameters
  - Exercise 3 (Intermediate): Create a custom ROS 2 message type and publisher
  - Output: `book/docs/module-1/ch1-ros2-basics/exercises.md` with prompts, hints, solutions
- [ ] T038 [US1] Write exercise solutions with detailed comments explaining each step

#### Summary & Quality Validation

- [ ] T039 [US1] Use `summary-generator` skill: Generate Chapter 1 summary with key takeaways, mental models, common mistakes
  - Output: `book/docs/module-1/ch1-ros2-basics/summary.md`
- [ ] T040 [US1] Use `content-evaluation-framework` skill: Evaluate Chapter 1 quality (technical accuracy, pedagogy, writing, structure)
  - Target: Quality score ≥80%
  - Iterate if below target

---

### Chapter 2: URDF Robot Description

**Scope**: 3-4 sections, 2+ URDF examples, 3+ exercises, ~1,500-2,000 words
**Skills Used**: `learning-objectives`, `urdf-designer`, `ros2-code-generator`, `code-explainer`, `exercise-designer`, `content-evaluation-framework`

#### Learning Objectives & Content Design

- [ ] T041 [US1] Use `learning-objectives` skill: Generate 3-5 objectives for "URDF Robot Description (joints, links, Gazebo)"
- [ ] T042 [US1] Document learning objectives in `book/docs/module-1/ch2-urdf/learning-objectives.md`

#### URDF Examples Generation

- [ ] T043 [US1] Use `urdf-designer` subagent: Design simple 2-link robot URDF
  - Output: `code-examples/ros2_packages/simple_robot_description/urdf/simple_robot.urdf`
  - Requirement: Valid URDF (`check_urdf` passes), visualizes in Gazebo
- [ ] T044 [US1] [P] Use `urdf-designer` subagent: Design humanoid robot URDF (simplified version)
  - Output: `code-examples/ros2_packages/humanoid_robot_description/urdf/humanoid.urdf`
  - Requirement: Valid URDF, includes collision models, visualizes correctly

#### Gazebo Integration Code

- [ ] T045 [US1] Use `ros2-code-generator` skill: Generate Gazebo launch file to display simple_robot.urdf
  - Output: `code-examples/ros2_packages/simple_robot_description/launch/display.launch.py`
  - Requirement: Launches successfully, robot displays in RViz/Gazebo

#### Code Annotation & Explanation

- [ ] T046 [US1] Use `code-explainer` subagent: Annotate simple_robot.urdf with XML comments explaining joints, links, inertia
- [ ] T047 [US1] [P] Use `code-explainer` subagent: Annotate humanoid.urdf with comments explaining hierarchy, collision geometry

#### Chapter Content Creation

- [ ] T048 [US1] Write Chapter 2 intro: "Robot Description with URDF" (what it is, why it matters) in `book/docs/module-1/ch2-urdf/index.md`
- [ ] T049 [US1] [P] Write Section 1: "Links and Joints" with diagrams, simple_robot.urdf example in `book/docs/module-1/ch2-urdf/01-links-joints.md`
- [ ] T050 [US1] [P] Write Section 2: "Gazebo Properties" with inertia, collision examples in `book/docs/module-1/ch2-urdf/02-gazebo-properties.md`
- [ ] T051 [US1] [P] Write Section 3: "Visualizing Robots" with humanoid.urdf example, RViz screenshots in `book/docs/module-1/ch2-urdf/03-visualization.md`

#### Exercises Creation

- [ ] T052 [US1] Use `exercise-designer` skill: Create 3 progressive exercises for Chapter 2
  - Exercise 1 (Beginner): Fix a broken URDF (missing closing tags, incorrect syntax)
  - Exercise 2 (Beginner/Intermediate): Modify simple_robot.urdf to add a 3rd link
  - Exercise 3 (Intermediate): Create your own 2-link robot URDF
  - Output: `book/docs/module-1/ch2-urdf/exercises.md`

#### Summary & Quality Validation

- [ ] T053 [US1] Use `summary-generator` skill: Generate Chapter 2 summary
  - Output: `book/docs/module-1/ch2-urdf/summary.md`
- [ ] T054 [US1] Use `content-evaluation-framework` skill: Evaluate Chapter 2 quality (target score ≥80%)

---

### Chapter 3: Python Integration with rclpy

**Scope**: 3-4 sections, 3+ code examples, 3+ exercises, ~1,500-2,000 words
**Skills Used**: `learning-objectives`, `ros2-code-generator`, `code-explainer`, `exercise-designer`, `content-evaluation-framework`

#### Learning Objectives & Content Design

- [ ] T055 [US1] Use `learning-objectives` skill: Generate 3-5 objectives for "Python Integration with rclpy (control, parameters, actions)"
- [ ] T056 [US1] Document learning objectives in `book/docs/module-1/ch3-python-integration/learning-objectives.md`

#### Code Examples Generation

- [ ] T057 [US1] Use `ros2-code-generator` skill: Generate robot velocity controller in Python
  - Output: `code-examples/ros2_packages/robot_control_py/robot_control_py/velocity_controller.py`
  - Requirement: Controls simulated robot velocity, integrates with Chapter 2 URDF
- [ ] T058 [US1] [P] Use `ros2-code-generator` skill: Generate ROS 2 parameter example in Python
  - Output: `code-examples/ros2_packages/param_example_py/` demonstrating parameter server
  - Requirement: Reads/writes parameters, integrates with rclpy
- [ ] T059 [US1] [P] Use `ros2-code-generator` skill: Generate ROS 2 action example in Python
  - Output: `code-examples/ros2_packages/action_example_py/` with action server and client
  - Requirement: Demonstrates action communication pattern

#### Code Annotation & Explanation

- [ ] T060 [US1] Use `code-explainer` subagent: Annotate velocity_controller.py with comments explaining rclpy node structure, publishers, subscribers
- [ ] T061 [US1] [P] Use `code-explainer` subagent: Annotate parameter example with comments explaining parameter get/set
- [ ] T062 [US1] [P] Use `code-explainer` subagent: Annotate action example with comments explaining action lifecycle

#### Chapter Content Creation

- [ ] T063 [US1] Write Chapter 3 intro: "Controlling Robots with Python" in `book/docs/module-1/ch3-python-integration/index.md`
- [ ] T064 [US1] [P] Write Section 1: "rclpy Basics and Node Structure" with velocity_controller example in `book/docs/module-1/ch3-python-integration/01-rclpy-basics.md`
- [ ] T065 [US1] [P] Write Section 2: "Working with Parameters" with parameter example in `book/docs/module-1/ch3-python-integration/02-parameters.md`
- [ ] T066 [US1] [P] Write Section 3: "Actions for Long-Running Tasks" with action example in `book/docs/module-1/ch3-python-integration/03-actions.md`

#### Exercises Creation

- [ ] T067 [US1] Use `exercise-designer` skill: Create 3 progressive exercises for Chapter 3
  - Exercise 1 (Beginner): Create a simple velocity publisher that ramps speed over time
  - Exercise 2 (Beginner/Intermediate): Modify velocity_controller to read target from ROS 2 parameter
  - Exercise 3 (Intermediate): Create a custom action to control robot movement to a goal position
  - Output: `book/docs/module-1/ch3-python-integration/exercises.md`

#### Summary & Quality Validation

- [ ] T068 [US1] Use `summary-generator` skill: Generate Chapter 3 summary
  - Output: `book/docs/module-1/ch3-python-integration/summary.md`
- [ ] T069 [US1] Use `content-evaluation-framework` skill: Evaluate Chapter 3 quality (target score ≥80%)

---

### Module 1 Integration & Deployment

- [ ] T070 [US1] Create Module 1 overview page `book/docs/module-1/index.md` linking to all 3 chapters
- [ ] T071 [US1] Create homepage `book/docs/intro.md` with introduction to Physical AI, module cards, learning path
- [ ] T072 [US1] Test Chapter 1 locally: Navigate to Chapter 1, verify all sections load, code examples display correctly
- [ ] T073 [US1] [P] Test Chapter 2 locally: Navigate to Chapter 2, verify URDF diagrams display, exercises accessible
- [ ] T074 [US1] [P] Test Chapter 3 locally: Navigate to Chapter 3, verify code examples complete, exercises accessible
- [ ] T075 [US1] Run `npm run build` in `book/` directory, verify no errors, build completes <60 seconds
- [ ] T076 [US1] Deploy to GitHub Pages: Push to main branch, verify GitHub Actions workflow triggers, site available at https://92Bilal26.github.io/physical-ai-textbook/

---

## Phase 4: Polish & Cross-Cutting Concerns

**Goal**: Quality validation, performance optimization, documentation

### Code Example Testing

- [ ] T077 Create Docker image with ROS 2 Humble for reproducible testing: `docker/Dockerfile`
- [ ] T078 [P] Test hello_world_py package builds and runs: `cd code-examples/ros2_packages/hello_world_py && colcon build && ros2 run hello_world_py hello_node`
- [ ] T079 [P] Test pub_sub_py package builds and runs
- [ ] T080 [P] Test service_example_py package builds and runs
- [ ] T081 [P] Test URDF files validate: `check_urdf simple_robot.urdf`, `check_urdf humanoid.urdf`
- [ ] T082 [P] Test robot_control_py builds and visualizes in Gazebo
- [ ] T083 [P] Test param_example_py builds and runs
- [ ] T084 [P] Test action_example_py builds and runs

### Performance & Accessibility

- [ ] T085 Run Lighthouse audit on deployed site: Target Lighthouse scores >90 (Performance, Accessibility, Best Practices)
- [ ] T086 [P] Test mobile responsiveness: Site displays correctly on 320px+ width
- [ ] T087 [P] Verify page load times: All chapter pages load in <2 seconds
- [ ] T088 Optimize images: Convert diagrams to WebP, implement lazy loading
- [ ] T089 Configure search: Enable Docusaurus search plugin, test chapter discoverability

### Documentation & Contribution Guide

- [ ] T090 Update `README.md` with project overview, setup instructions, how to contribute
- [ ] T091 [P] Create `CONTRIBUTING.md` with guidelines for adding new chapters, code examples, exercises
- [ ] T092 [P] Create `CODE_OF_CONDUCT.md` (optional for hackathon, good practice for public project)

### Final Quality Validation

- [ ] T093 Run `content-evaluation-framework` skill on all 3 chapters: Verify scores ≥80%
- [ ] T094 [P] Have a peer or instructor review Chapter 1 for pedagogical quality
- [ ] T095 [P] Have a peer or instructor review Chapter 2 for pedagogical quality
- [ ] T096 [P] Have a peer or instructor review Chapter 3 for pedagogical quality
- [ ] T097 Test all exercise solutions: Run/verify each solution code
- [ ] T098 Verify all code examples have correct ROS 2 versions documented (Humble 22.04)

---

## Task Execution Strategy

### Parallelization Opportunities

**Phase 1 (Setup)**: Mostly sequential (depends on Docusaurus installation)
- T003, T004, T008, T009 can run in parallel (directory creation)
- T001-T002 must complete first (Docusaurus setup)

**Phase 2 (Foundational)**: High parallelization
- Template creation tasks (T011-T013) can run in parallel
- Component creation tasks (T017-T018) can run in parallel
- Research/documentation tasks (T019-T020) can run in parallel

**Phase 3 (User Story 1 - Chapters)**:
- **Chapter 1**: T027-T029 (code generation) can run in parallel
  - T030-T032 (annotation) depend on T027-T029, can run in parallel after
  - T034-T036 (content writing) can run in parallel but need completed code examples first
  - Dependency chain: Generate code → Annotate → Write content → Create exercises → Validate

- **Chapter 2**: Similar parallelization pattern
  - T043-T044 (URDF generation) can run in parallel
  - T046-T047 (annotation) depend on T043-T044, can run in parallel

- **Chapter 3**: Similar parallelization pattern
  - T057-T059 (code generation) can run in parallel
  - T060-T062 (annotation) can run in parallel after code generation

- **Cross-chapter parallelization**: Chapters can be developed independently
  - Example: While Chapter 1 is being written (T034-T036), Chapter 2 content design can start (T041-T044)

**Phase 4 (Polish)**: Parallelization in testing
- T078-T084 (code testing) can run in parallel (independent packages)
- T085-T087 (performance) can run in parallel
- T093-T098 (validation) can run in parallel

### Suggested Sprint Breakdown

**Sprint 1 (Setup & Foundational)**: T001-T024 (~3-4 days)
- Initialize Docusaurus, configure deployment, create templates

**Sprint 2 (Chapter 1)**: T025-T040 (~2-3 days)
- Define objectives, generate code, write content, create exercises
- Focus on demonstrating skills-driven workflow

**Sprint 3 (Chapter 2)**: T041-T054 (~2-3 days)
- URDF design, content writing, exercises
- Parallel with Chapter 1 validation

**Sprint 4 (Chapter 3)**: T055-T069 (~2-3 days)
- Python examples, content writing, exercises
- Parallel with Chapters 1-2 deployment

**Sprint 5 (Integration & Deployment)**: T070-T076 (~1-2 days)
- Module overview, homepage, deployment to GitHub Pages

**Sprint 6 (Polish)**: T077-T098 (~1-2 days)
- Testing, optimization, final validation

---

## Success Criteria (Task Completion Checklist)

**Must Have (MVP)**:
- [x] All T001-T076 tasks completed (setup, all 3 chapters, deployment)
- [x] Docusaurus site deployed to GitHub Pages
- [x] All 3 chapters accessible and readable
- [x] All ROS 2 code examples build successfully
- [x] Mobile responsive design (verified in T086)
- [x] Page load <2 seconds (verified in T087)

**Should Have (Quality)**:
- [x] All T077-T084 code testing completed
- [x] All chapters evaluated with content-evaluation-framework (scores ≥80%)
- [x] All exercises have complete solutions
- [x] Lighthouse scores >90 (verified in T085)

**Nice to Have (Polish)**:
- [x] All T090-T098 documentation and final validation completed
- [x] Images optimized (T088)
- [x] Peer review completed (T094-T096)
- [x] Contributing guide created (T091)

---

## Notes for Implementation

1. **Skills/Subagents**: Invoke skills in the specified order. If a skill produces inconsistent output, validate against templates before proceeding to next task.

2. **Code Testing**: All ROS 2 packages must be tested in Docker with ROS 2 Humble before being included in chapters. Use `colcon build && colcon test` for validation.

3. **Content Review**: After using `content-evaluation-framework` skill, if score <80%, iterate on content and re-evaluate. Do not proceed to next chapter if score <80%.

4. **GitHub Pages**: GitHub Actions workflow will deploy automatically on push to main. Verify deployment in Actions tab before calling site "live".

5. **Documentation**: Keep `specs/001-docusaurus-textbook/quickstart.md` updated as setup procedures change. Make it easy for future contributors.

---

**Plan Status**: ✅ Ready for execution
**Estimated Duration**: 10-14 days (solo) or 5-7 days (team of 2)
**MVP Completion Target**: All 98 tasks completed with quality validation
