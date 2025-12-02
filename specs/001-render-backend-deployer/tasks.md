# Implementation Tasks: Render Backend Deployer Skill

**Feature**: Render Backend Deployer Skill
**Branch**: `001-render-backend-deployer`
**Date**: 2025-12-02
**Spec**: [spec.md](spec.md) | **Plan**: [plan.md](plan.md)

---

## Overview

This document defines all tasks needed to implement the Render Backend Deployer skill. Tasks are organized by phase, with each phase representing independently testable functionality. Each user story (P1-P3) can be implemented in parallel after foundational setup completes.

### Summary

- **Total Tasks**: 32 tasks across 5 phases
- **Setup Tasks**: 4 (Phase 1)
- **Foundational Tasks**: 5 (Phase 2)
- **User Story 1 (P1 - Initial Deployment)**: 6 tasks
- **User Story 2 (P1 - Database Setup)**: 5 tasks
- **User Story 3 (P1 - Environment Variables)**: 4 tasks
- **User Story 4 (P2 - CORS Configuration)**: 4 tasks
- **User Story 5 (P2 - Error Debugging)**: 2 tasks
- **User Story 6 (P3 - Frontend Integration)**: 1 task
- **Polish Phase**: 1 task

### Implementation Strategy

**MVP Scope (Phase 1-2)**: Core skill infrastructure + foundational workflows
**Phase 3 (Add Stories)**: One user story per iteration, starting with P1 stories
**Parallel Execution**: All user stories can be developed in parallel after Phase 2 completes

### Dependency Graph

```
Phase 1: Setup (4 tasks)
    ↓
Phase 2: Foundational (5 tasks)
    ↓
Phase 3+: User Stories (18 tasks in parallel)
    - US1, US2, US3, US4, US5, US6
    ↓
Final: Polish & Testing (1 task)
```

---

## Phase 1: Setup & Skill Infrastructure

**Goal**: Create the skill directory structure and templates
**Completion Time**: ~1 hour
**Independent Test**: `ls -R .claude/skills/render-backend-deployer/` shows all directories created

### Tasks

- [ ] T001 Create skill directory structure at `.claude/skills/render-backend-deployer/` with subdirectories: `workflows/`, `patterns/`, `examples/`
- [ ] T002 Create main skill prompt template at `.claude/skills/render-backend-deployer/render-backend-deployer.prompt.md` with intro, workflow selector, and step-by-step structure
- [ ] T003 Create skill metadata file `.claude/skills/render-backend-deployer/.meta.md` documenting skill purpose, version, dependencies, and usage
- [ ] T004 Copy render.yaml example template to `.claude/skills/render-backend-deployer/examples/render.yaml.example`

---

## Phase 2: Foundational Knowledge Base

**Goal**: Build pattern libraries and reference materials used by all workflows
**Completion Time**: ~2 hours
**Independent Test**: All pattern files exist and contain correct information from RAG chatbot deployment

### Tasks

- [ ] T005 [P] Create common errors reference at `.claude/skills/render-backend-deployer/patterns/common-errors.md` covering: DATABASE_URL naming issues, CORS errors, middleware order problems, Pydantic type mismatches, Docker build failures
- [ ] T006 [P] Create CORS/middleware patterns guide at `.claude/skills/render-backend-deployer/patterns/middleware-order.md` explaining: middleware order importance, CORS before rate-limiting, preflight request handling
- [ ] T007 [P] Create environment variable checklist at `.claude/skills/render-backend-deployer/patterns/environment-checklist.md` with: pre-deployment validation steps, variable naming conventions, secret vs non-secret classification
- [ ] T008 [P] Create log interpretation guide at `.claude/skills/render-backend-deployer/patterns/log-interpretation.md` covering: how to read Render deployment logs, common log patterns, error identification
- [ ] T009 Create example files: `config.py.example` (Pydantic settings), `main.py.example` (FastAPI CORS), `docker.example` (Docker best practices) in `.claude/skills/render-backend-deployer/examples/`

---

## Phase 3: User Story 1 - Initial Backend Deployment (P1)

**Goal**: Guide developers through creating render.yaml and deploying to Render
**Completion Time**: ~2 hours
**Independent Test**: Can deploy a simple FastAPI app to Render.com using skill guidance and access it via provided URL

### Tasks

- [ ] T010 [US1] Create workflow prompt at `.claude/skills/render-backend-deployer/workflows/initial-setup.md` covering: render.yaml structure explanation, web service creation steps, build context/Dockerfile configuration
- [ ] T011 [US1] Add render.yaml validation checklist to workflow: repository structure check, Docker configuration verification, environment variable stubs
- [ ] T012 [US1] Create Render.com web service creation guide with: UI step-by-step screenshots reference, copy-paste commands, common configuration mistakes
- [ ] T013 [US1] Create deployment verification checklist: health endpoint test, logs inspection, success criteria confirmation
- [ ] T014 [US1] Add troubleshooting section for build failures: Docker context issues, Dockerfile path problems, dependency resolution errors
- [ ] T015 [US1] Create integration test that simulates deploying a test FastAPI app and validates successful service creation

---

## Phase 3: User Story 2 - Database Setup & Configuration (P1)

**Goal**: Guide database creation and connection configuration
**Completion Time**: ~2 hours
**Independent Test**: Can create PostgreSQL and Redis on Render, retrieve connection strings, and verify backend can connect

### Tasks

- [ ] T016 [US2] Create database setup workflow at `.claude/skills/render-backend-deployer/workflows/database-setup.md` covering: PostgreSQL creation, Redis creation, free tier limitations, region selection
- [ ] T017 [US2] Create PostgreSQL setup guide: database name/user/region selection, connection string retrieval, free tier constraints, password management
- [ ] T018 [US2] Create Redis setup guide: Key Value instance creation on Render (not PostgreSQL!), internal URL retrieval, free tier limitations, connection testing
- [ ] T019 [US2] Create connection string validation checklist: DATABASE_URL format verification, REDIS_URL format verification, connection string documentation
- [ ] T020 [US2] Add troubleshooting: database name collision handling, region mismatch resolution, connection timeout diagnosis

---

## Phase 3: User Story 3 - Environment Variable Configuration (P1)

**Goal**: Guide secure configuration of environment variables
**Completion Time**: ~1.5 hours
**Independent Test**: Can add environment variables through Render UI and verify backend accesses them correctly

### Tasks

- [ ] T021 [US3] Create environment variable workflow at `.claude/skills/render-backend-deployer/workflows/environment-vars.md` covering: secret vs non-secret variables, Render UI configuration, variable naming conventions
- [ ] T022 [US3] Create environment variable configuration guide: step-by-step Render UI instructions, database URL variables (DATABASE_URL, REDIS_URL), API key variables (OPENAI_API_KEY, QDRANT_*)
- [ ] T023 [US3] Create Pydantic settings validation guide: type conversion patterns, Union types for flexibility, field validator ordering, common validation errors
- [ ] T024 [US3] Add environment variable troubleshooting: variable name mismatches (NEON_DATABASE_URL vs DATABASE_URL), type conversion errors, missing required variables

---

## Phase 3: User Story 4 - CORS Configuration for Frontend (P2)

**Goal**: Configure CORS and fix cross-origin issues
**Completion Time**: ~1.5 hours
**Independent Test**: Frontend on GitHub Pages can make successful API calls to backend without CORS errors

### Tasks

- [ ] T025 [US4] Create CORS configuration workflow at `.claude/skills/render-backend-deployer/workflows/cors-config.md` covering: middleware order importance, allowed_origins configuration, CORS environment variables
- [ ] T026 [US4] Create FastAPI CORS setup guide: CORSMiddleware configuration, allowed_origins list format, credentials/headers/methods settings, environment variable integration
- [ ] T027 [US4] Create middleware order debugging guide: why CORS must be first, rate-limiting middleware placement, preflight request handling, header verification
- [ ] T028 [US4] Add CORS troubleshooting: common error patterns, log inspection, browser console debugging, preflight OPTIONS request issues

---

## Phase 3: User Story 5 - Deployment Error Debugging (P2)

**Goal**: Provide systematic debugging for deployment issues
**Completion Time**: ~1.5 hours
**Independent Test**: Can diagnose and resolve common deployment errors using skill guidance

### Tasks

- [ ] T029 [US5] Create error debugging workflow at `.claude/skills/render-backend-deployer/workflows/error-debugging.md` covering: log reading, common error patterns, progressive debugging strategy, solution reference
- [ ] T030 [US5] Create diagnostic decision tree: SQLAlchemy errors → DATABASE_URL check, Pydantic errors → type conversion, CORS errors → middleware order, Docker errors → build context

---

## Phase 3: User Story 6 - Frontend URL Configuration (P3)

**Goal**: Help developers update frontend with correct backend URL
**Completion Time**: ~45 minutes
**Independent Test**: Frontend can successfully call backend after URL update without CORS errors

### Tasks

- [ ] T031 [US6] Create frontend integration workflow at `.claude/skills/render-backend-deployer/workflows/frontend-integration.md` covering: Render URL format explanation, frontend configuration files, URL update guide, API call testing

---

## Phase 4: Polish & Validation

**Goal**: Complete documentation and validation
**Completion Time**: ~1 hour
**Independent Test**: Full end-to-end skill walkthrough succeeds for all 6 user stories

### Tasks

- [ ] T032 Create quick reference guide at `.claude/skills/render-backend-deployer/QUICKSTART.md` with: 15-minute deployment path, common questions, troubleshooting index, success criteria checklist

---

## Task Execution Order

### Recommended Sequence

**Week 1 - Foundation**:
1. Phase 1 Setup (T001-T004): 1 hour
2. Phase 2 Knowledge Base (T005-T009): 2 hours
3. **TOTAL**: 3 hours - Skill infrastructure ready

**Week 1 - Core Workflows**:
4. Phase 3.1 - User Story 1 (T010-T015): 2 hours
5. Phase 3.2 - User Story 2 (T016-T020): 2 hours
6. Phase 3.3 - User Story 3 (T021-T024): 1.5 hours
7. **TOTAL**: 5.5 hours - MVP complete (covers 70% of deployments)

**Week 2 - Advanced**:
8. Phase 3.4 - User Story 4 (T025-T028): 1.5 hours
9. Phase 3.5 - User Story 5 (T029-T030): 1.5 hours
10. Phase 3.6 - User Story 6 (T031): 0.75 hours
11. Phase 4 - Polish (T032): 1 hour
12. **TOTAL**: 4.75 hours - Feature complete

**Grand Total**: ~12 hours for complete skill implementation

### Parallel Execution

After Phase 2 completes, all User Stories (T010-T031) can be developed in parallel:
- Developer A: User Stories 1-3 (deployment + databases + env vars)
- Developer B: User Story 4 (CORS configuration)
- Developer C: User Story 5 (error debugging)
- Developer D: User Story 6 (frontend integration)

---

## Success Criteria by Phase

### Phase 1 Success
- [ ] All directories created with correct structure
- [ ] Main skill prompt file exists and is readable
- [ ] Example templates copied to examples/ directory

### Phase 2 Success
- [ ] All 5 pattern files created with accurate content
- [ ] Example files (config, main, docker) available
- [ ] Pattern content reflects RAG chatbot deployment learnings

### Phase 3 Success
- [ ] All 6 workflows created
- [ ] Each workflow guides through complete user story
- [ ] Troubleshooting sections address common issues
- [ ] Integration tests validate workflow success

### Phase 4 Success
- [ ] Quick reference guide exists and is complete
- [ ] End-to-end walkthrough succeeds for all stories
- [ ] Skill meets all success criteria from spec.md

---

## File Structure After Completion

```text
.claude/skills/render-backend-deployer/
├── .meta.md
├── render-backend-deployer.prompt.md (main skill)
├── QUICKSTART.md
├── workflows/
│   ├── initial-setup.md
│   ├── database-setup.md
│   ├── environment-vars.md
│   ├── cors-config.md
│   ├── error-debugging.md
│   └── frontend-integration.md
├── patterns/
│   ├── common-errors.md
│   ├── middleware-order.md
│   ├── environment-checklist.md
│   └── log-interpretation.md
└── examples/
    ├── render.yaml.example
    ├── config.py.example
    ├── main.py.example
    └── docker.example
```

---

## Notes

- All tasks include exact file paths for precision
- Pattern files reuse real knowledge from RAG chatbot deployment
- Workflows are skill-executable without additional context
- Testing focuses on integration (end-to-end) rather than unit
- MVP (Phase 1-3.1) delivers value: basic deployment workflow
- Tasks can be distributed across team for parallel execution
