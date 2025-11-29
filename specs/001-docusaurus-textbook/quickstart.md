# Quickstart Guide: Physical AI Textbook Development

**Target Audience**: Developers and content creators contributing to the Physical AI textbook

**Time to Complete**: 15-20 minutes

---

## Prerequisites

Before starting, ensure you have:

- **Git**: Version control ([install](https://git-scm.com/))
- **Node.js**: 18+ ([install](https://nodejs.org/))
- **npm**: Comes with Node.js
- **Text Editor**: VS Code recommended ([download](https://code.visualstudio.com/))
- **ROS 2 Humble**: For testing code examples ([install guide](https://docs.ros.org/en/humble/Installation.html))

---

## Project Structure

```
physical-ai-textbook/
├── book/                          # Docusaurus website
│   ├── docs/                      # Markdown content
│   │   ├── intro.md               # Homepage
│   │   └── tutorial-basics/       # Template structure
│   ├── src/
│   │   ├── components/            # React components (CodeExample, ExerciseBlock)
│   │   ├── css/custom.css         # Robotics theme
│   │   └── pages/                 # Custom pages
│   ├── package.json               # Dependencies
│   ├── docusaurus.config.ts       # Docusaurus configuration
│   └── sidebars.ts                # Navigation structure
│
├── code-examples/                 # ROS 2 code examples
│   └── ros2_packages/             # Separate ROS 2 packages
│       ├── hello_world_py/        # Example package structure
│       ├── pub_sub_py/
│       └── ...
│
├── specs/                         # Specification and planning
│   └── 001-docusaurus-textbook/
│       ├── spec.md                # Feature requirements
│       ├── plan.md                # Implementation plan
│       ├── tasks.md               # 98 implementation tasks
│       ├── data-model.md          # Content entities
│       ├── contracts/             # Content templates
│       │   ├── chapter-template.md
│       │   ├── exercise-template.md
│       │   └── code-example-template.md
│       ├── quickstart.md          # This file
│       └── research.md            # Design decisions
│
└── .github/
    └── workflows/
        └── deploy.yml             # GitHub Pages deployment
```

---

## Getting Started (5 minutes)

### 1. Clone the Repository

```bash
git clone https://github.com/92Bilal26/physical-ai-textbook.git
cd physical-ai-textbook
```

### 2. Create a Feature Branch

```bash
git checkout -b feature/your-feature-name
# Example: git checkout -b feature/add-chapter-1
```

### 3. Navigate to Book Directory

```bash
cd book
```

### 4. Install Dependencies

```bash
npm install
```

### 5. Start Development Server

```bash
npm start
```

Visit http://localhost:3000 to see your site!

---

## Common Workflows

### Adding a New Chapter

**Location**: `book/docs/module-1/ch[N]-[topic-slug]/`

**Steps**:

1. Create chapter directory:
   ```bash
   mkdir -p book/docs/module-1/ch3-python-integration
   ```

2. Use chapter template:
   ```bash
   cp specs/001-docusaurus-textbook/contracts/chapter-template.md \
      book/docs/module-1/ch3-python-integration/index.md
   ```

3. Edit `index.md` with your content

4. Create section files:
   ```bash
   touch book/docs/module-1/ch3-python-integration/01-rclpy-basics.md
   touch book/docs/module-1/ch3-python-integration/02-parameters.md
   touch book/docs/module-1/ch3-python-integration/03-actions.md
   ```

5. Update `book/sidebars.ts` to include new chapter

6. Test locally:
   ```bash
   npm start
   ```

### Creating a ROS 2 Code Example

**Location**: `code-examples/ros2_packages/[package-name]/`

**Steps**:

1. Create a ROS 2 Python package:
   ```bash
   cd code-examples/ros2_packages
   ros2 pkg create my_example_package --build-type ament_python
   ```

2. Create your nodes in `my_example_package/my_example_package/`

3. Build and test:
   ```bash
   cd ../../..
   colcon build --packages-select my_example_package
   source install/setup.bash
   ros2 run my_example_package my_node
   ```

4. Document with code-example-template.md

5. Commit the working package

### Writing an Exercise

**Location**: `book/docs/module-1/ch[N]/exercises.md`

**Steps**:

1. Use exercise-template.md as reference:
   ```bash
   cat specs/001-docusaurus-textbook/contracts/exercise-template.md
   ```

2. Create exercise in chapter's exercises.md:
   ```markdown
   ## Exercise 1: [Title]

   **Difficulty**: Beginner
   **Time**: 10-15 minutes

   ### Task
   [Clear objective]
   ```

3. Provide instructions, hints, solution code

4. Include validation checklist

5. Test solution works before committing

---

## Skills-Driven Workflow (Phase 3)

The textbook uses Claude Code skills for systematic content creation:

### 1. Learning Objectives
```bash
# Use learning-objectives skill to generate Bloom's taxonomy objectives
# Generates file: book/docs/module-1/ch[N]/learning-objectives.md
```

### 2. Code Generation
```bash
# Use ros2-code-generator skill to create tested ROS 2 packages
# Output: code-examples/ros2_packages/[package-name]/
```

### 3. Code Annotation
```bash
# Use code-explainer subagent to add pedagogical comments
# Updates: [package-name]/[node-file].py with inline explanations
```

### 4. Exercise Design
```bash
# Use exercise-designer skill to create progressive exercises
# Output: book/docs/module-1/ch[N]/exercises.md
```

### 5. Summary Generation
```bash
# Use summary-generator skill to create chapter summaries
# Output: book/docs/module-1/ch[N]/summary.md
```

### 6. Quality Evaluation
```bash
# Use content-evaluation-framework skill to score content
# Target: Quality score ≥80%
# Output: Quality report with feedback
```

---

## Building and Deployment

### Local Build

```bash
cd book
npm run build
```

Output appears in `book/build/` directory.

### Preview Build

```bash
cd book
npm run serve
```

Visit http://localhost:3000 for preview of production build.

### GitHub Pages Deployment

Push to `main` branch:

```bash
git add .
git commit -m "Add Chapter 1 content"
git push origin your-feature-branch
```

Then create a Pull Request on GitHub. Once merged to `main`, GitHub Actions automatically:
1. Builds the Docusaurus site
2. Deploys to GitHub Pages
3. Site live at https://92Bilal26.github.io/physical-ai-textbook/

---

## Key Files to Know

| File | Purpose |
|------|---------|
| `book/docusaurus.config.ts` | Site configuration, navbar, footer, theme |
| `book/sidebars.ts` | Navigation structure and chapter ordering |
| `book/src/css/custom.css` | Robotics theme colors and styling |
| `specs/data-model.md` | Content entity definitions |
| `specs/contracts/` | Template contracts for consistency |
| `.github/workflows/deploy.yml` | Automated deployment configuration |

---

## Troubleshooting

### "npm start" shows blank page

**Solution**: Clear cache and rebuild:
```bash
rm -rf .docusaurus build node_modules/.cache
npm start
```

### Code examples not building

**Solution**: Ensure ROS 2 Humble installed:
```bash
source /opt/ros/humble/setup.bash
cd code-examples/ros2_packages/[package-name]
colcon build
```

### "Module not found" errors in React components

**Solution**: Ensure CSS modules imported correctly:
```typescript
import styles from './MyComponent.module.css';
// Not: import './MyComponent.css'
```

### Git conflicts when merging

**Solution**: Pull latest, resolve conflicts:
```bash
git fetch origin
git rebase origin/main
# Resolve conflicts in your editor
git add .
git rebase --continue
```

---

## Useful Commands

```bash
# Development
npm start              # Start dev server
npm run build         # Build for production
npm run serve         # Serve production build

# Docusaurus
npm run docusaurus clear  # Clear cache
npm run swizzle       # Eject Docusaurus components

# Git
git status            # See changes
git add -A            # Stage all changes
git commit -m "..."   # Create commit
git push origin branch # Push to GitHub

# ROS 2
colcon build          # Build all packages
colcon build --packages-select pkg  # Build single package
ros2 run pkg node     # Run a node
ros2 topic echo /topic  # Listen to topic
```

---

## Learning Resources

- **Docusaurus Docs**: https://docusaurus.io/
- **ROS 2 Documentation**: https://docs.ros.org/en/humble/
- **React**: https://react.dev/
- **Markdown Guide**: https://www.markdownguide.org/

---

## Getting Help

- **Project Issues**: GitHub Issues on repo
- **ROS 2 Help**: https://index.ros.org/search/?type=forum
- **Docusaurus Help**: https://docusaurus.io/docs

---

## Next Steps

1. ✅ Complete this quickstart
2. Create a feature branch for your contribution
3. Choose your task from `specs/tasks.md`
4. Use appropriate skill/template for your task
5. Test your changes locally
6. Commit and push to GitHub
7. Create a Pull Request for review

**Happy contributing! 🚀**
