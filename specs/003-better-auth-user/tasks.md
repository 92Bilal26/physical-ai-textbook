# Implementation Tasks: Better Auth User Authentication & Personalization

**Feature**: 003-better-auth-user (User Authentication & Profile Management)
**Status**: Task Generation Complete
**Date**: 2025-12-02
**Branch**: `003-better-auth-user`

---

## Overview

This document contains all implementation tasks organized by user story and phase. Tasks are prioritized, parallelizable where possible, and include specific file paths for independent execution.

**Total Tasks**: 68
**Phases**: 5
- Phase 1: Setup (5 tasks)
- Phase 2: Foundational Infrastructure (8 tasks)
- Phase 3: User Story 1 - Authentication Core (15 tasks)
- Phase 4: User Story 2 - User Profiles (18 tasks)
- Phase 5: User Story 3 - Content Personalization (16 tasks)
- Phase 6: Polish & Cross-Cutting (6 tasks)

**MVP Scope**: Phase 1-4 (40 tasks, ~25-30 hours)
- Core authentication working
- User profiles and backgrounds collected
- Basic session management
- Database schema complete

---

## Dependency Graph

```
Phase 1: Setup
  ↓ (blocking)
Phase 2: Foundational Infrastructure
  ├─→ Phase 3: Authentication (US1) [Independent]
  ├─→ Phase 4: User Profiles (US2) [Depends on US1]
  └─→ Phase 5: Personalization (US3) [Depends on US1 + US2]
  ↓ (sequential)
Phase 6: Polish & Cross-Cutting
```

**Parallel Opportunities**:
- Phase 3 & 4: Can run in parallel if both teams available (different routes/components)
- Within each phase: Multiple [P] tasks can run in parallel (different files)

---

## Phase 1: Setup (Project Initialization)

**Goal**: Initialize Node.js backend project with Better Auth framework and PostgreSQL connection

**Success Criteria**:
- ✅ Project structure created per implementation plan
- ✅ All dependencies installed and compatible
- ✅ Environment variables configured
- ✅ Database connection verified
- ✅ Better Auth initialized with PostgreSQL adapter

---

### Phase 1 Tasks

- [ ] T001 Create auth-backend project directory structure with src/, tests/, and config files per plan at `auth-backend/`

- [ ] T002 [P] Initialize Node.js project: `npm init -y` in `auth-backend/`, set name to "physical-ai-auth", version "0.1.0", main "dist/index.js" at `auth-backend/package.json`

- [ ] T003 [P] Create TypeScript configuration at `auth-backend/tsconfig.json` with target "ES2020", module "commonjs", strict mode, and outDir "dist"

- [ ] T004 [P] Install core dependencies: `npm install express cors uuid dotenv` and dev dependencies: `npm install -D typescript ts-node @types/express @types/node prettier eslint` at `auth-backend/`

- [ ] T005 [P] Install Better Auth stack: `npm install better-auth drizzle-orm postgres dotenv` at `auth-backend/`

---

## Phase 2: Foundational Infrastructure

**Goal**: Set up database, middleware, and core utilities shared by all user stories

**Success Criteria**:
- ✅ PostgreSQL database connection working
- ✅ Drizzle ORM configured with migrations
- ✅ CORS middleware protecting endpoints
- ✅ Error handling middleware consistent
- ✅ Authentication guard middleware operational
- ✅ All utility functions available (validators, loggers)
- ✅ Environment variables loaded and validated

---

### Phase 2 Tasks

- [ ] T006 Create `.env.example` at `auth-backend/.env.example` with all required variables: DATABASE_URL, BETTER_AUTH_SECRET, BETTER_AUTH_URL, NODE_ENV, LOG_LEVEL, CORS_ORIGIN, SESSION_EXPIRY_DAYS

- [ ] T007 [P] Create environment loader at `auth-backend/src/config/env.ts` that validates required variables using zod and throws on missing secrets

- [ ] T008 [P] Create logger utility at `auth-backend/src/utils/logger.ts` with debug(), info(), warn(), error() functions that respect LOG_LEVEL

- [ ] T009 [P] Create validators at `auth-backend/src/utils/validators.ts` with functions: validateEmail(), validatePassword(), validateLanguages(), validateDifficultyLevel()

- [ ] T010 [P] Create error handling middleware at `auth-backend/src/middleware/error-handler.ts` that catches exceptions and returns user-friendly error responses with proper HTTP status codes

- [ ] T011 [P] Create CORS middleware at `auth-backend/src/middleware/cors.ts` that reads CORS_ORIGIN from environment and configures Express CORS

- [ ] T012 Create Drizzle database schema at `auth-backend/src/db/schema.ts` with tables: users (Better Auth), user_backgrounds, user_preferences, chapter_metadata, user_progress (see plan.md for full schema)

- [ ] T013 [P] Create database client at `auth-backend/src/db/client.ts` using Drizzle ORM with PostgreSQL adapter, export `db` instance for use in other modules

---

## Phase 3: User Story 1 - Authentication Core (US1)

**User Story**: "As a new user, I want to sign up with email/password and have my account created securely"

**Priority**: P1 (Critical for MVP)

**Success Criteria**:
- ✅ Signup form submits without errors
- ✅ Passwords stored as scrypt hashes (verified in DB)
- ✅ Session token created and returned
- ✅ User automatically signed in post-signup
- ✅ Duplicate emails rejected with 409 status
- ✅ Weak passwords rejected (<8 chars)
- ✅ Email verification tokens generated (optional feature)
- ✅ Login/logout flows work end-to-end
- ✅ Session validation middleware protects endpoints
- ✅ Rate limiting prevents brute force (5 attempts → 15 min lockout)

---

### Phase 3 Tasks: Authentication Setup & Better Auth

- [ ] T014 [P] Set up Better Auth server at `auth-backend/src/auth.ts`: Initialize betterAuth() with PostgreSQL adapter, email/password auth, and database connection from env

- [ ] T015 [P] Create Better Auth routes at `auth-backend/src/routes/auth.ts`: Mount all Better Auth endpoints at `/api/auth/*` using Express router

- [ ] T016 [P] Create authentication middleware at `auth-backend/src/middleware/auth-guard.ts` that validates session tokens from cookies, extracts userId, and attaches to request.user

- [ ] T017 [P] Create password validation at `auth-backend/src/utils/validators.ts`: Add validatePassword() that enforces 8-128 char requirement, returns helpful error messages

- [ ] T018 Create email validation at `auth-backend/src/utils/validators.ts`: Add validateEmail() using regex, check uniqueness in database before signup

- [ ] T019 [P] Create User service at `auth-backend/src/services/user.service.ts` with methods: createUser(), getUser(), getUserById(), checkEmailExists(), updateUser()

- [ ] T020 [P] Create error types at `auth-backend/src/types/errors.ts`: Define AuthError, ValidationError, NotFoundError with proper HTTP status codes

- [ ] T021 Create signup handler at `auth-backend/src/routes/users.ts` with endpoint `POST /api/auth/sign-up` that: validates input, calls Better Auth signup, creates user record, returns session token

- [ ] T022 [P] Create signin handler at `auth-backend/src/routes/users.ts` with endpoint `POST /api/auth/sign-in` that: validates email/password, calls Better Auth, returns session with "remember me" option

- [ ] T023 [P] Create signout handler at `auth-backend/src/routes/users.ts` with endpoint `POST /api/auth/sign-out` that: terminates session, clears cookies, returns success

- [ ] T024 Create session validation at `auth-backend/src/routes/users.ts` with endpoint `GET /api/auth/session` that: returns current user info if authenticated, 401 if not

- [ ] T025 [P] Create password reset request at `auth-backend/src/routes/users.ts` with endpoint `POST /api/auth/password-reset-request` that: validates email, generates token, sends reset email (mock for now)

- [ ] T026 [P] Create password reset confirmation at `auth-backend/src/routes/users.ts` with endpoint `POST /api/auth/password-reset` that: validates token, updates password, returns success

- [ ] T027 Create main Express server at `auth-backend/src/index.ts`: Initialize Express app, register all middleware (CORS, error handler, auth guard), mount routes, start listening on PORT

- [ ] T028 Create npm scripts in `auth-backend/package.json`: "dev" (ts-node src/index.ts), "build" (tsc), "start" (node dist/index.js), "test" (jest)

---

## Phase 4: User Story 2 - User Profiles & Background (US2)

**User Story**: "As a user, I want to complete my profile with background information so content can be personalized"

**Priority**: P1 (Critical for MVP)

**Depends On**: US1 (Authentication)

**Success Criteria**:
- ✅ Background questionnaire collected (software dev + robotics + languages)
- ✅ User can edit background anytime
- ✅ Preferences stored and retrieved correctly
- ✅ Profile page displays editable fields
- ✅ Updates persist to database
- ✅ Default preferences created for new users
- ✅ User can export their data (GDPR)
- ✅ User can request account deletion with 30-day grace period

---

### Phase 4 Tasks: User Profiles & Preferences

- [ ] T029 [P] Create background types at `auth-backend/src/types/index.ts`: Define UserBackground, UserPreferences interfaces matching database schema

- [ ] T030 [P] Create profile service at `auth-backend/src/services/profile.service.ts` with methods: createBackground(), updateBackground(), getBackground(), createPreferences(), updatePreferences(), getPreferences()

- [ ] T031 [P] Create background questionnaire data at `auth-backend/src/data/questionnaire.ts`: Define questions, options, and validation rules for software dev / robotics / languages

- [ ] T032 Create background endpoint at `auth-backend/src/routes/profiles.ts` with endpoint `POST /api/profiles/background` that: validates input, saves UserBackground record, creates default preferences, returns background object

- [ ] T033 [P] Create profile GET endpoint at `auth-backend/src/routes/profiles.ts` with endpoint `GET /api/profiles/me` that: requires auth, returns user + background + preferences

- [ ] T034 [P] Create preferences update endpoint at `auth-backend/src/routes/profiles.ts` with endpoint `PUT /api/profiles/preferences` that: requires auth, validates input, saves preferences, returns updated object

- [ ] T035 [P] Create profile edit endpoint at `auth-backend/src/routes/profiles.ts` with endpoint `PUT /api/users/me` that: requires auth, validates input, updates name/email/picture, returns user object

- [ ] T036 [P] Create data export endpoint at `auth-backend/src/routes/users.ts` with endpoint `GET /api/users/me/export` that: requires auth, returns JSON with all user data (GDPR compliance)

- [ ] T037 Create account deletion endpoint at `auth-backend/src/routes/users.ts` with endpoint `DELETE /api/users/me` that: requires auth, marks for deletion, returns grace period (30 days), actually deletes after 30 days (cron job)

- [ ] T038 [P] Create profile picture upload handler (placeholder) at `auth-backend/src/routes/profiles.ts`: Accept image file, validate type/size, store path/URL, return image URL

- [ ] T039 Create React signup form at `book/src/components/Auth/SignupForm.tsx` with fields: email, password, confirm password, background questionnaire (3 questions), submit button

- [ ] T040 [P] Create React signin form at `book/src/components/Auth/SigninForm.tsx` with fields: email, password, "remember me" checkbox, signin button, "forgot password" link

- [ ] T041 [P] Create profile edit page at `book/src/components/Auth/ProfilePage.tsx` with: current user info, background summary, edit form, preferences section, logout button

- [ ] T042 [P] Create background questionnaire component at `book/src/components/Auth/BackgroundQuestionnaire.tsx` with: 3 questions (software dev, robotics, languages), validation, submit handler

- [ ] T043 [P] Create auth context hook at `book/src/components/hooks/useAuth.ts` that: manages auth state, login/logout functions, session check, returns user info

- [ ] T044 [P] Create profile context hook at `book/src/components/hooks/useProfile.ts` that: manages profile state, fetch background/preferences, update functions, returns profile info

- [ ] T045 Create dashboard page at `book/src/pages/dashboard.tsx` with: welcome message, background summary, recommended chapters (placeholder), progress tracker, profile link

- [ ] T046 Create profile page route at `book/src/pages/profile.tsx` that: protects with auth check, renders ProfilePage component, handles save/logout

---

## Phase 5: User Story 3 - Content Personalization (US3)

**User Story**: "As a logged-in user, I want chapter recommendations based on my background so I can learn at my level"

**Priority**: P2 (High for MVP+)

**Depends On**: US1 + US2

**Success Criteria**:
- ✅ Chapter metadata loaded from database
- ✅ Recommendation algorithm scores chapters
- ✅ Top recommendations returned with relevance scores
- ✅ Difficulty filtering works
- ✅ Dashboard shows personalized recommendations
- ✅ "Perfect for your level" badges appear
- ✅ Difficulty ratings display on chapters
- ✅ Progress tracking works (chapters started/completed)
- ✅ User progress persists across sessions
- ✅ Personalization button on chapter headers works

---

### Phase 5 Tasks: Personalization Engine

- [ ] T047 [P] Create chapter metadata data at `auth-backend/src/data/chapters.ts`: Define all chapters with difficulty, prerequisites, target audience, estimated time

- [ ] T048 Create seed script at `auth-backend/scripts/seed-chapters.ts`: Read chapter metadata, insert into chapter_metadata table, handle duplicates gracefully

- [ ] T049 [P] Create personalization service at `auth-backend/src/services/personalize.service.ts` with methods: getRecommendations(), calculateRelevanceScore(), filterByDifficulty(), updateProgress()

- [ ] T050 [P] Implement recommendation algorithm at `auth-backend/src/services/personalize.service.ts`: calculateRelevanceScore() that scores chapters based on difficulty match (0.3), audience match (0.3), prerequisites met (0.2), progress (0.2)

- [ ] T051 [P] Create recommendation endpoint at `auth-backend/src/routes/personalize.ts` with endpoint `POST /api/personalize/recommend-chapters` that: requires auth, calculates scores, returns top-N chapters with explanations

- [ ] T052 [P] Create difficulty info endpoint at `auth-backend/src/routes/personalize.ts` with endpoint `GET /api/personalize/chapter-difficulty/:chapterId` that: returns chapter difficulty, recommendations, alternatives (auth optional)

- [ ] T053 [P] Create chapter filter endpoint at `auth-backend/src/routes/personalize.ts` with endpoint `POST /api/personalize/filter-chapters` that: filters by difficulty, keywords, tags (auth optional)

- [ ] T054 [P] Create progress tracking endpoint at `auth-backend/src/routes/personalize.ts` with endpoint `POST /api/personalize/progress/:chapterId` that: requires auth, updates user_progress record with status/completion

- [ ] T055 [P] Create progress GET endpoint at `auth-backend/src/routes/personalize.ts` with endpoint `GET /api/personalize/progress` that: requires auth, returns all chapters user has started/completed

- [ ] T056 Create React personalization hook at `book/src/components/hooks/usePersonalization.ts` that: fetches recommendations, filters chapters, returns personalized data, caches results

- [ ] T057 [P] Create recommended chapters component at `book/src/components/Dashboard/RecommendedChapters.tsx` with: list of recommended chapters, relevance score badges, "Perfect for your level" badges, click to navigate

- [ ] T058 [P] Create personalize button at `book/src/components/Auth/PersonalizeButton.tsx` that: appears on chapter headers, shows current personalization level, allows changing difficulty preference

- [ ] T059 [P] Create progress tracker component at `book/src/components/Dashboard/ProgressTracker.tsx` with: visual progress bars, chapters completed count, time invested, achievement badges

- [ ] T060 Create Docusaurus integration for personalization at `book/docusaurus.config.js`: Add auth client initialization, environment variable setup, API base URL configuration

- [ ] T061 [P] Create chapter metadata JSON at `auth-backend/src/data/chapter-metadata.json`: List all 40+ chapters with difficulty, prerequisites, target audience (reference from Physical AI Textbook structure)

- [ ] T062 Create progress persistence at `book/src/components/hooks/useProgress.ts`: Track chapter viewing time, update progress endpoint on unload, store local cache

---

## Phase 6: Polish & Cross-Cutting Concerns

**Goal**: Security hardening, performance optimization, testing, documentation

**Success Criteria**:
- ✅ All endpoints tested (unit + integration + API)
- ✅ Security vulnerabilities addressed (npm audit passes)
- ✅ Performance meets targets (<3s signup, <2s dashboard)
- ✅ Documentation complete (README, API docs, deployment guide)
- ✅ Environment setup automated
- ✅ Deployment to Render tested and working
- ✅ Monitoring and logging operational

---

### Phase 6 Tasks: Testing, Security, Deployment

- [ ] T063 [P] Create unit test suite at `auth-backend/tests/unit/validators.test.ts`: Test validateEmail(), validatePassword(), validateLanguages() with valid/invalid inputs

- [ ] T064 [P] Create integration tests at `auth-backend/tests/integration/auth.test.ts`: Test signup flow, signin flow, password reset, session validation end-to-end

- [ ] T065 [P] Create API tests at `auth-backend/tests/integration/api.test.ts`: Test all endpoints with Supertest, verify response codes, error messages, auth requirements

- [ ] T066 Create security audit at `auth-backend/`: Run `npm audit`, fix any vulnerabilities, document security measures (HTTPS, hashing, rate limiting, GDPR)

- [ ] T067 Create API documentation at `auth-backend/API.md`: Document all endpoints, request/response examples, auth requirements, error codes

- [ ] T068 Create deployment guide at `auth-backend/DEPLOYMENT.md`: Step-by-step instructions for deploying to Render, environment setup, database migrations, testing post-deploy

---

## Implementation Strategy

### MVP Scope (Phases 1-4): 40 tasks, ~25-30 hours

**Deliverables**:
- ✅ Working signup/signin
- ✅ User profiles with backgrounds
- ✅ Database schema complete
- ✅ Authentication middleware
- ✅ Basic React components
- ✅ Session management

**Benefits**:
- Core feature complete and testable
- Can deploy and gather user feedback
- Foundation for personalization (Phase 5)
- Meets core hackathon requirements

### Phase 5+: Personalization & Polish

**Timeline**: After MVP validated
- Personalization algorithm (Phase 5)
- Testing & security (Phase 6)
- Performance optimization
- Documentation

### Parallel Execution

**Teams of 2-3**:

**Team 1 (Backend Auth)**:
- Tasks T001-T028 (Phase 1-3)
- Delivers: Working auth API

**Team 2 (Backend Profiles)**:
- Tasks T029-T046 (Phase 4 backend)
- Delivers: Profile API
- Can start after T015 (Better Auth routes)

**Team 3 (Frontend)**:
- Tasks T039-T046 (Phase 4 frontend)
- Starts after T001 (project structure exists)
- Can work in parallel with Teams 1-2

**Team 4 (Personalization)**:
- Tasks T047-T062 (Phase 5)
- Starts after US1+US2 complete
- Independent from other teams

---

## Testing Strategy

### Recommended TDD Approach (Optional)

If following Test-Driven Development:

1. Before implementing service methods: Write unit tests
2. Before implementing endpoints: Write API contract tests
3. Before merging: Run full integration suite
4. Before deployment: End-to-end tests

### Testing Commands

```bash
# Run all tests
npm test

# Run specific suite
npm test -- auth.test.ts

# Run with coverage
npm test -- --coverage

# Run integration tests
npm test -- tests/integration/

# Security audit
npm audit
npm audit fix
```

---

## Success Metrics

### Per-Task Validation

Each task includes:
- Specific file path (testable location)
- Clear success criteria (checkable deliverable)
- Optional dependencies shown

### Cross-Story Validation

- [ ] US1 (Auth) completes before US2 can be tested
- [ ] US2 (Profiles) completes before US3 can be tested
- [ ] All tasks in Phase 6 pass security/performance checks
- [ ] All npm scripts work (dev, build, test, start)

### Deployment Validation

- [ ] Render environment configured with BETTER_AUTH_SECRET
- [ ] PostgreSQL migrations applied
- [ ] Health endpoint returns 200
- [ ] Signup/signin flows work in production
- [ ] Session tokens persist across restarts

---

## Task Checklist Format

Each task follows this strict format:

```
- [ ] TASKID [P] [Story] Description with file path
```

- **Checkbox**: `- [ ]` (unchecked), `- [x]` (checked when complete)
- **Task ID**: T001-T068 (sequential)
- **[P]**: Parallelizable (optional marker)
- **[Story]**: [US1], [US2], [US3] (optional, user story phase only)
- **Description**: Action + exact file path

---

## Next Steps

1. ✅ **Tasks Generated** (this file)
2. 🔄 **Commit Tasks**: Push to GitHub branch
3. 🔄 **Begin Implementation**: Start Phase 1 tasks
4. 🔄 **Iterate**: Complete phase, test, move to next
5. 🔄 **Deploy**: Render deployment after MVP complete

**Ready to implement?** Follow tasks in order, mark with [x] as complete.

---

**Tasks Version**: 1.0 | **Generated**: 2025-12-02 | **Status**: Ready for Implementation

Total Estimated Effort: 40-60 hours (MVP 25-30h, Full 60h with testing)
