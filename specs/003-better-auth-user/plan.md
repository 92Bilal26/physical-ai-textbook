# Implementation Plan: Better Auth User Authentication & Personalization

**Branch**: `003-better-auth-user` | **Date**: 2025-12-02 | **Spec**: [specs/003-better-auth-user/spec.md](./spec.md)
**Input**: Feature specification from `/specs/003-better-auth-user/spec.md`

---

## Summary

Add user authentication to the Physical AI Textbook using Better Auth framework, enabling personalized learning experiences. Users sign up with email/password and background questionnaire (software dev + robotics + languages). The system stores profiles, manages sessions, and personalizes chapter recommendations and content based on user background. This transforms the static textbook into an adaptive learning platform while maintaining the existing Docusaurus structure.

**Key Deliverables**:
1. Better Auth backend integration with TypeScript/Node.js
2. User signup/signin/logout flows with profile management
3. Content personalization engine (recommendations + difficulty filtering)
4. Database schema for user profiles and preferences
5. Frontend integration with Docusaurus React components

---

## Technical Context

**Language/Version**: TypeScript 5.0+, Node.js 18+ (Better Auth requirement)
**Primary Dependencies**:
- Better Auth (authentication framework)
- Express.js (HTTP server for auth API)
- PostgreSQL (existing Render database)
- React/Docusaurus (frontend framework)
- OpenAI Agents SDK (for personalization recommendations)

**Storage**: PostgreSQL (existing Render managed database)
**Testing**: Jest (TypeScript testing), Supertest (API testing)
**Target Platform**: Web (Docusaurus frontend + Node.js backend)
**Project Type**: Web application (frontend + backend)
**Performance Goals**:
- Signup/signin: <3 seconds (p95)
- Dashboard load: <2 seconds (p95)
- Profile update: <1 second
- Content filtering: <500ms

**Constraints**:
- <100ms additional latency for personalization (transparent to user)
- Secure session management (HTTPS only)
- GDPR compliance (user data export/deletion)
- Backward compatibility with existing Docusaurus site

**Scale/Scope**:
- Expected users: 100-1000 during MVP
- Database: <10MB for user/profile data
- Session storage: Redis or PostgreSQL
- Personalization coverage: All chapters (40+ chapters in modules)

---

## Constitution Check

*GATE: Must pass before Phase 1 design. Re-check after Phase 1 complete.*

### Gate 4: AI-Native Workflow ✅
- [x] Feature spec.md completed with 40 functional requirements
- [x] Code examples will reference specifications
- [x] AI chatbot integration points identified (personalization queries)
- [x] User stories prioritized in specification (P1-P3)

### Gate 7: Personalization ✅
- [x] Content adaptable for beginner/intermediate/advanced levels
- [x] Code examples available in Python, C++, TypeScript (framework-dependent)
- [x] Hardware-specific guidance identified in recommendations
- [x] Personalization button placement defined (chapter headers)

### Gate 9: Reusable Intelligence ✅
- [x] Better Auth subagent/skill can be reused for other textbooks
- [x] User profiling pattern applicable to future projects
- [x] Personalization algorithm documented for extension
- [x] Reusability across all Panaversity courses planned

**All gates: PASS ✅ Ready for Phase 1 design**

---

## Architecture Overview

### System Components

```
Physical AI Textbook System
│
├─ Frontend (Docusaurus React)
│  ├─ Signup/Signin Components (Better Auth UI)
│  ├─ User Profile Page
│  ├─ Personalization Engine (client-side filtering)
│  └─ Content Components (personalization aware)
│
├─ Backend (Node.js + Better Auth)
│  ├─ Better Auth Server (/api/auth/*)
│  ├─ User API (/api/users/*)
│  ├─ Profile API (/api/profiles/*)
│  ├─ Personalization API (/api/personalize/*)
│  └─ Middleware (CORS, auth validation, error handling)
│
└─ Database (PostgreSQL)
   ├─ users (Better Auth managed)
   ├─ sessions (Better Auth managed)
   ├─ user_backgrounds (custom)
   ├─ user_preferences (custom)
   └─ chapter_metadata (for personalization)
```

### Technology Stack Decision

| Layer | Technology | Rationale |
|-------|-----------|-----------|
| Frontend | React/Docusaurus | Already in use; minimal changes |
| Auth Framework | Better Auth | TypeScript-first, minimal config, session mgmt |
| Backend | Node.js + Express | Better Auth native support |
| Backend Language | TypeScript | Type safety, Better Auth examples, Node.js standard |
| Database | PostgreSQL | Existing Render infrastructure |
| Session Storage | PostgreSQL + Redis (optional) | Built-in Better Auth support |
| ORM | Drizzle ORM | Better Auth recommended, SQL-first approach |
| API Style | REST | Simple, proven pattern for personalization |

**Key Decision**: Use TypeScript/Node.js backend separate from Python services (RAG chatbot backend) to follow Better Auth's native stack. Both backends communicate via REST API.

---

## Project Structure

### Backend (New Directory)

```
auth-backend/                          # New service
├── src/
│   ├── index.ts                       # Entry point
│   ├── auth.ts                        # Better Auth setup
│   ├── routes/
│   │   ├── auth.ts                    # Auth endpoints
│   │   ├── users.ts                   # User management
│   │   ├── profiles.ts                # Profile operations
│   │   └── personalize.ts             # Personalization engine
│   ├── db/
│   │   ├── schema.ts                  # Drizzle schema
│   │   ├── client.ts                  # DB connection
│   │   └── migrations/                # SQL migrations
│   ├── middleware/
│   │   ├── auth-guard.ts              # Session validation
│   │   ├── error-handler.ts           # Error handling
│   │   └── cors.ts                    # CORS configuration
│   ├── services/
│   │   ├── profile.service.ts         # Profile operations
│   │   ├── personalize.service.ts     # Recommendation logic
│   │   └── background.service.ts      # Background questions
│   ├── types/
│   │   └── index.ts                   # TypeScript types
│   └── utils/
│       ├── logger.ts                  # Logging
│       └── validators.ts              # Input validation
├── tests/
│   ├── unit/                          # Unit tests
│   └── integration/                   # API tests
├── .env.example                       # Environment template
├── package.json
├── tsconfig.json
└── README.md
```

### Frontend (Docusaurus Modifications)

```
book/src/
├── components/
│   ├── Auth/
│   │   ├── SignupForm.tsx
│   │   ├── SigninForm.tsx
│   │   ├── ProfilePage.tsx
│   │   ├── BackgroundQuestionnaire.tsx
│   │   └── PersonalizeButton.tsx
│   ├── Dashboard/
│   │   ├── UserDashboard.tsx
│   │   ├── RecommendedChapters.tsx
│   │   └── ProgressTracker.tsx
│   └── hooks/
│       ├── useAuth.ts                 # Auth context hook
│       ├── useProfile.ts              # Profile context hook
│       └── usePersonalization.ts      # Personalization hook
└── pages/
    ├── dashboard.tsx                  # Dashboard page
    └── profile.tsx                    # Profile edit page
```

### Database Schema

```sql
-- Better Auth manages:
-- - users (id, email, name, emailVerified, createdAt, updatedAt)
-- - accounts (userId, accountId, provider, providerAccountId, accessToken, etc.)
-- - sessions (id, userId, token, expiresAt)

-- Custom tables:
CREATE TABLE user_backgrounds (
  id UUID PRIMARY KEY,
  user_id UUID REFERENCES users(id) ON DELETE CASCADE,
  software_dev_level VARCHAR(20), -- None, Beginner, Intermediate, Advanced
  robotics_level VARCHAR(20),      -- Same levels
  languages TEXT[],                -- Array: [Python, C++, etc]
  created_at TIMESTAMP,
  updated_at TIMESTAMP,
  UNIQUE(user_id)
);

CREATE TABLE user_preferences (
  id UUID PRIMARY KEY,
  user_id UUID REFERENCES users(id) ON DELETE CASCADE,
  preferred_language VARCHAR(10),     -- Python, C++, TypeScript, etc
  preferred_difficulty VARCHAR(20),   -- Beginner, Intermediate, Advanced
  content_types TEXT[],               -- [text, video, interactive]
  learning_path VARCHAR(50),          -- default, difficulty-based, etc
  notifications_enabled BOOLEAN,
  created_at TIMESTAMP,
  updated_at TIMESTAMP,
  UNIQUE(user_id)
);

CREATE TABLE chapter_metadata (
  id UUID PRIMARY KEY,
  chapter_id TEXT UNIQUE,             -- e.g., "module-1/ch1-basics"
  title VARCHAR(255),
  difficulty VARCHAR(20),             -- Beginner, Intermediate, Advanced
  prerequisites TEXT[],               -- Chapter IDs
  target_audience TEXT[],             -- Programmer, Roboticist, Beginner
  estimated_time_minutes INTEGER,
  created_at TIMESTAMP
);

CREATE TABLE user_progress (
  id UUID PRIMARY KEY,
  user_id UUID REFERENCES users(id),
  chapter_id TEXT,
  status VARCHAR(20),                 -- not_started, started, completed, mastered
  time_spent_seconds INTEGER,
  completion_percentage DECIMAL,
  last_accessed TIMESTAMP,
  created_at TIMESTAMP,
  updated_at TIMESTAMP,
  UNIQUE(user_id, chapter_id)
);
```

---

## API Contracts

### Authentication Endpoints (Better Auth)

All Better Auth endpoints are automatically generated at `/api/auth/*`

**Key endpoints**:
- `POST /api/auth/sign-up/email` - Register new user (Better Auth)
- `POST /api/auth/sign-in/email` - Login user (Better Auth)
- `GET /api/auth/session` - Get current session (Better Auth)
- `POST /api/auth/sign-out` - Logout user (Better Auth)

### Custom API Endpoints

#### User Profile Management

**GET /api/users/me**
- Returns: `{ id, email, name, image, createdAt }`
- Auth: Required (JWT session)
- Purpose: Get current user info

**POST /api/profiles/background**
- Body: `{ softwareDevLevel, roboticsLevel, languages }`
- Returns: Updated background object
- Auth: Required
- Purpose: Set/update user background

**GET /api/profiles/me**
- Returns: `{ userId, softwareDevLevel, roboticsLevel, languages, preferences }`
- Auth: Required
- Purpose: Get complete user profile

**PUT /api/profiles/preferences**
- Body: `{ preferredLanguage, preferredDifficulty, contentTypes, learningPath }`
- Returns: Updated preferences
- Auth: Required
- Purpose: Update learning preferences

**DELETE /api/users/me**
- Returns: `{ status: "deletion_requested", gracePeriod: "30 days" }`
- Auth: Required
- Purpose: Request account deletion (with 30-day grace)

#### Personalization Engine

**POST /api/personalize/recommend-chapters**
- Body: Optional `{ limit: 5 }`
- Returns: `[{ chapterId, title, difficulty, relevanceScore, reason }]`
- Auth: Required
- Purpose: Get personalized chapter recommendations

**GET /api/personalize/chapter-difficulty/:chapterId**
- Returns: `{ difficulty, recommendations, alternatives }`
- Auth: Optional (works for anonymous users)
- Purpose: Get difficulty info for a chapter

**POST /api/personalize/filter-chapters**
- Body: `{ difficulty?, keywords?, tags? }`
- Returns: Filtered chapters with personalization info
- Auth: Optional
- Purpose: Filter/search chapters with personalization

---

## Data Model

### Core Entities

**User** (from Better Auth)
```typescript
interface User {
  id: string;        // UUID
  email: string;     // Unique
  name: string;      // Display name
  image?: string;    // Profile picture URL
  emailVerified: boolean;
  createdAt: Date;
  updatedAt: Date;
}
```

**UserBackground**
```typescript
interface UserBackground {
  id: string;
  userId: string;
  softwareDevLevel: 'None' | 'Beginner' | 'Intermediate' | 'Advanced';
  roboticsLevel: 'None' | 'Beginner' | 'Intermediate' | 'Advanced';
  languages: string[]; // ['Python', 'C++', 'Java', 'Go', 'JavaScript', 'Other']
  createdAt: Date;
  updatedAt: Date;
}
```

**UserPreferences**
```typescript
interface UserPreferences {
  id: string;
  userId: string;
  preferredLanguage: string;        // Python, C++, TypeScript, etc
  preferredDifficulty: 'Beginner' | 'Intermediate' | 'Advanced' | 'Mixed';
  contentTypes: string[];           // text, video, interactive
  learningPath: string;             // default, difficulty-based, interest-based
  notificationsEnabled: boolean;
  createdAt: Date;
  updatedAt: Date;
}
```

**ChapterMetadata**
```typescript
interface ChapterMetadata {
  id: string;
  chapterId: string;                // e.g., module-1/ch1-basics
  title: string;
  difficulty: 'Beginner' | 'Intermediate' | 'Advanced';
  prerequisites: string[];          // chapterIds
  targetAudience: string[];         // Programmer, Roboticist, Beginner
  estimatedTimeMinutes: number;
}
```

**UserProgress**
```typescript
interface UserProgress {
  id: string;
  userId: string;
  chapterId: string;
  status: 'not_started' | 'started' | 'completed' | 'mastered';
  timeSpentSeconds: number;
  completionPercentage: number;  // 0-100
  lastAccessed: Date;
}
```

---

## Integration Points

### 1. Frontend ↔ Backend Communication

**Authentication Flow**:
```
Docusaurus Component
  ↓ (form submit)
Better Auth Client SDK
  ↓ (POST /api/auth/sign-up)
Backend Better Auth Handler
  ↓ (stores user + session)
Return session token
  ↓ (store in client cookie)
Redirect to profile completion
```

**Profile Completion**:
```
ProfilePage Component
  ↓ (background questionnaire)
POST /api/profiles/background
  ↓ (store in DB)
ProfileService.createBackground()
  ↓ (save UserBackground record)
Return success
  ↓ (redirect to dashboard)
```

### 2. Personalization Data Flow

```
Chapter Load in Docusaurus
  ↓ usePersonalization() hook
Check if user logged in
  ├─ Yes: GET /api/personalize/recommend-chapters
  │   ↓
  │   → Get recommendations based on background
  │   → Filter chapters by difficulty
  │   → Show "Perfect for your level" badges
  │
  └─ No: Show generic recommendations
      ↓
      → Suggest signup for personalization
```

### 3. Backend ↔ Database

```
User Signs Up
  → Better Auth inserts into users table
  → User completes background questionnaire
  → POST /api/profiles/background
  → Insert UserBackground record
  → Create default UserPreferences record
  → (Optional) Index chapter metadata if first user
```

### 4. Content Personalization Algorithm

```
GET /api/personalize/recommend-chapters
  ↓
1. Load user background + preferences
2. Load chapter metadata
3. For each chapter:
   a) Calculate relevance score:
      - Difficulty match: Is chapter difficulty near user level?
      - Background match: Does target audience include user type?
      - Prerequisites: Has user completed prerequisites?
      - Progress: What's completion percentage?
   b) Score = (difficulty_match * 0.3) + (audience_match * 0.3) +
              (prerequisites_met * 0.2) + (progress * 0.2)
4. Sort by score (descending)
5. Return top N chapters with explanations
```

---

## Error Handling & Edge Cases

### Authentication Errors

| Error | HTTP Status | User Message |
|-------|------------|--------------|
| Invalid email format | 400 | "Please enter a valid email address" |
| Email already registered | 409 | "This email is already registered" |
| Weak password (< 8 chars) | 400 | "Password must be at least 8 characters" |
| Invalid credentials | 401 | "Incorrect email or password" |
| Email not verified | 403 | "Please verify your email before signing in" |
| Session expired | 401 | "Your session expired. Please sign in again" |
| Account locked (5+ failed attempts) | 429 | "Account temporarily locked. Try again in 15 minutes" |

### Profile/Preference Errors

| Error | HTTP Status | User Message |
|-------|------------|--------------|
| Invalid background selection | 400 | "Please select valid options" |
| Duplicate email on update | 409 | "This email is already in use" |
| Failed to save preferences | 500 | "Could not save preferences. Please try again" |
| Background not found | 404 | "User background not found" |

### Data Integrity

- **Orphaned sessions**: Cron job deletes sessions older than 30 days
- **Missing backgrounds**: Default to "None/Beginner" if missing (allow optional completion)
- **Duplicate profiles**: Unique constraint on (user_id) prevents duplicates
- **Race conditions**: Database transactions ensure atomicity

---

## Security Considerations

### Authentication & Authorization

✅ **Implemented by Better Auth**:
- Secure password hashing (scrypt, OWASP recommended)
- Session tokens encrypted and HTTP-only
- CSRF protection for form submissions
- Rate limiting on auth endpoints

✅ **Custom Implementation**:
- All API endpoints require valid session token
- Middleware validates JWT before processing request
- User can only access their own profile (not others')
- Rate limiting on failed login attempts (5 attempts → 15 min lockout)

### Data Protection

- All passwords stored as scrypt hashes (never plain text)
- User data encrypted at rest (PostgreSQL SSL)
- API endpoints use HTTPS only (enforced)
- Sensitive fields (API keys) never logged
- User can export/delete all their data (GDPR)

### Privacy & Compliance

- User consent collected at signup (checkbox)
- Clear privacy policy (link in signup)
- No third-party data sharing without explicit consent
- Audit log of data access (future enhancement)

---

## Testing Strategy

### Unit Tests

- Background questionnaire validation
- Preference parsing and defaults
- Recommendation algorithm accuracy
- Error message formatting

### Integration Tests

- Signup flow (email → password → background → account created)
- Signin/signout flows
- Session management (expiry, concurrent sessions)
- Profile CRUD operations
- Personalization API responses

### API Tests (Supertest)

- All endpoints return correct status codes
- Error responses have proper error messages
- Auth-required endpoints reject unauthenticated requests
- Session validation works

### End-to-End Tests (Future)

- User signup → complete profile → see personalized dashboard
- User signin → modify preferences → recommendations change
- Personalization badges appear on chapters

---

## Deployment Strategy

### Environment Configuration

```env
# .env.example
DATABASE_URL=postgresql://user:pass@host/dbname
BETTER_AUTH_URL=http://localhost:3000
BETTER_AUTH_SECRET=your-secret-key-32-chars-min
NODE_ENV=production
LOG_LEVEL=info
CORS_ORIGIN=https://yourdomain.com
SESSION_EXPIRY_DAYS=30
```

### Deploy Process

1. **Backend Setup**:
   ```bash
   cd auth-backend
   npm install
   npm run db:migrate  # Apply database migrations
   npm run build       # Compile TypeScript
   npm start           # Start server
   ```

2. **Frontend Integration**:
   ```bash
   cd book
   npm install better-auth  # Add Better Auth client
   # Update Docusaurus config with auth URLs
   npm run build
   ```

3. **Database Migrations**:
   - Drizzle will auto-generate migration files
   - Run `npm run db:migrate` on deployment
   - Rollback: `npm run db:rollback` if needed

4. **Post-Deployment**:
   - Test signup/signin flows
   - Verify recommendations engine
   - Check session persistence
   - Monitor auth API response times

---

## Phased Implementation

### Phase 1: Core Authentication (Priority 1)
- ✅ Better Auth setup with PostgreSQL
- ✅ Signup/signin forms
- ✅ User profile table
- ✅ Session management
- ✅ Basic auth middleware

### Phase 2: User Profiles (Priority 1)
- ✅ Background questionnaire during signup
- ✅ Profile edit page
- ✅ Preference management
- ✅ Chapter metadata table

### Phase 3: Personalization (Priority 2)
- ✅ Recommendation algorithm
- ✅ Difficulty filtering
- ✅ Dashboard with recommendations
- ✅ Progress tracking

### Phase 4: Polish & Optimization (Priority 3)
- Rate limiting optimization
- Performance profiling
- Security audit
- User feedback integration

---

## Success Criteria Verification

| Criterion | How to Verify |
|-----------|--------------|
| 70% signup completion | Monitor analytics: signups / signup form loads |
| <3s signup latency | Load test with Apache Bench, measure p95 |
| GDPR compliance | Test data export/deletion flows |
| 80% recommendation relevance | User survey on recommendation usefulness |
| Zero password leaks | Code review + security scan (npm audit) |
| 40% return rate in 7 days | Track user login frequency |
| <500ms recommendation generation | Monitor API response times |

---

## Known Risks & Mitigations

| Risk | Probability | Impact | Mitigation |
|------|-------------|--------|-----------|
| TypeScript/Node.js unfamiliar | Medium | High | Use Better Auth docs + examples, pair programming |
| Database migration issues | Low | High | Test migrations in staging first, backup database |
| Session management bugs | Medium | Medium | Comprehensive integration tests, audit session code |
| Personalization inaccuracy | Medium | Low | Start simple (rule-based), improve iteratively |
| Email delivery failures | Low | Medium | Implement retry logic, provide backup verification |
| User privacy concerns | Medium | Low | Clear privacy policy, transparent data use |

---

## Next Steps

1. ✅ **Specification Complete** (spec.md)
2. ✅ **Planning Complete** (this file)
3. 🔄 **Next**: `/sp.tasks` - Generate implementation tasks
4. 🔄 **Then**: `/sp.implement` - Execute tasks
5. 🔄 **Finally**: Test + Deploy

**Ready to generate tasks?** Run `/sp.tasks` when ready!

---

**Plan Version**: 1.0 | **Last Updated**: 2025-12-02 | **Status**: Ready for Task Generation

## Project Structure

### Documentation (this feature)

```text
specs/[###-feature]/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)
<!--
  ACTION REQUIRED: Replace the placeholder tree below with the concrete layout
  for this feature. Delete unused options and expand the chosen structure with
  real paths (e.g., apps/admin, packages/something). The delivered plan must
  not include Option labels.
-->

```text
# [REMOVE IF UNUSED] Option 1: Single project (DEFAULT)
src/
├── models/
├── services/
├── cli/
└── lib/

tests/
├── contract/
├── integration/
└── unit/

# [REMOVE IF UNUSED] Option 2: Web application (when "frontend" + "backend" detected)
backend/
├── src/
│   ├── models/
│   ├── services/
│   └── api/
└── tests/

frontend/
├── src/
│   ├── components/
│   ├── pages/
│   └── services/
└── tests/

# [REMOVE IF UNUSED] Option 3: Mobile + API (when "iOS/Android" detected)
api/
└── [same as backend above]

ios/ or android/
└── [platform-specific structure: feature modules, UI flows, platform tests]
```

**Structure Decision**: [Document the selected structure and reference the real
directories captured above]

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| [e.g., 4th project] | [current need] | [why 3 projects insufficient] |
| [e.g., Repository pattern] | [specific problem] | [why direct DB access insufficient] |
