# Specification: User Authentication & Personalization with Better Auth

**Feature Name**: Better Auth User Authentication & Profile Management
**Feature ID**: 003-better-auth-user
**Status**: Specification Phase
**Date Created**: 2025-12-02
**Version**: 1.0

---

## Executive Summary

Add user authentication to the Physical AI Textbook using Better Auth framework, enabling users to create accounts, sign in, and receive personalized learning content based on their background (software development experience, hardware/robotics knowledge, programming language proficiency). This feature transforms the static textbook into a personalized learning platform where content recommendations adapt to individual user backgrounds.

---

## Problem Statement

Currently, the Physical AI Textbook is a static website accessible to all visitors equally. There's no way to:
- Track user progress through the textbook
- Collect user background information for personalization
- Recommend relevant chapters based on user expertise level
- Provide personalized learning paths

Users with different backgrounds (software developers learning robotics vs. roboticists learning Python) receive identical content, regardless of their starting point.

---

## Feature Overview

### What We're Building

A complete authentication and user profile system that:

1. **Allows users to sign up** with email/password and background questionnaire
2. **Collects user background** during signup (3 key dimensions):
   - Software development experience (None / Beginner / Intermediate / Advanced)
   - Hardware/robotics background (None / Beginner / Intermediate / Advanced)
   - Programming language proficiency (Python, C++, Java, Go, Other)
3. **Enables user sign in** with session management
4. **Supports user sign out** with session cleanup
5. **Manages user profiles** - edit background information and preferences
6. **Personalizes content** - recommend chapters based on background
7. **Stores user preferences** - track reading preferences and interests

### Core User Flows

**Flow 1: New User Signup**
```
Visitor → Click "Sign Up" → Fill signup form (email, password) → Answer background questions → Create account → Redirected to dashboard → See personalized content
```

**Flow 2: Existing User Sign In**
```
User → Click "Sign In" → Enter email/password → Authenticated → Redirected to dashboard → See personalized content
```

**Flow 3: User Profile Management**
```
Logged-in user → Click profile → View/edit background → Update preferences → Save changes → See updated recommendations
```

**Flow 4: Content Personalization**
```
Logged-in user → Browse chapters → System recommends based on background → User sees difficulty rating + prerequisites + "perfect for your level" badges
```

---

## User Scenarios

### Scenario 1: Software Developer Learning Robotics
**User**: Alice, experienced Python developer, no robotics background

**Goal**: Learn ROS 2 and robotics in the context of her programming expertise

**Interaction**:
1. Alice visits textbook, sees "Sign Up" button
2. Clicks "Sign Up", enters alice@example.com, password
3. Answers: Software Dev = Advanced, Robotics = None, Languages = Python
4. Account created, redirected to dashboard
5. System shows: "Chapter 1 is perfect for your level" (emphasizes programming concepts)
6. Recommended next: "Skip basic Python, start with ROS 2 architecture"
7. Chapter content shows: "As a Python developer, you'll recognize..." callouts

**Success**: Alice finds content relevant to her existing skills without feeling talked down to

---

### Scenario 2: Roboticist Learning Python
**User**: Bob, hardware engineer, beginner Python, intermediate robotics

**Goal**: Learn to program robots in Python with ROS 2

**Interaction**:
1. Bob signs up: Software Dev = Beginner, Robotics = Intermediate, Languages = C++
2. System creates profile with background
3. Dashboard shows: "Start with Python Integration chapter" (bridges gap)
4. Chapter 1 shows: "As a roboticist transitioning to Python, note these differences..." callouts
5. Code examples emphasize hardware control vs. software architecture
6. Recommendations suggest: "Your robotics knowledge means you can focus on syntax"

**Success**: Bob gets Python fundamentals with robotics context, not abstract programming theory

---

### Scenario 3: Complete Beginner
**User**: Carol, learning robotics and programming for first time

**Goal**: Learn everything from basics

**Interaction**:
1. Carol signs up: All = Beginner
2. System shows chapter progression path: Start with Chapter 1 (beginner-friendly)
3. Content includes: All fundamentals, no assumed knowledge
4. Callouts explain concepts from scratch
5. Can view "advanced" sections but marked as optional

**Success**: Carol doesn't feel overwhelmed; content matches her level

---

### Scenario 4: Profile Update
**User**: Dave, initially beginner, now intermediate

**Goal**: Update profile to reflect growth

**Interaction**:
1. Dave logs in, clicks "Profile"
2. Updates "Software Dev" from Beginner to Intermediate
3. System refreshes recommendations
4. Dashboard now shows more advanced chapters first
5. Previously completed chapters show "You mastered this"

**Success**: Platform adapts to user growth

---

## Functional Requirements

### Authentication (FR-001 to FR-010)

**FR-001: User Signup with Email/Password**
- System accepts email and password during signup
- Password must be 8-128 characters
- Validates email format
- Stores password securely (scrypt hashing)
- Automatically signs in user post-signup
- Redirects to profile completion (background questionnaire)

**FR-002: Email Verification (Optional)**
- System can send verification email to new users
- Users can verify email via link
- Unverified users can still sign in (but marked as unverified)
- Admin can mandate verification before accessing content

**FR-003: User Sign In**
- Existing users can sign in with email/password
- Creates session on successful authentication
- "Remember me" option persists sessions across browser closures
- Redirects to dashboard on success
- Shows error message on failed auth

**FR-004: User Sign Out**
- Authenticated users can sign out
- Terminates session
- Clears authentication cookies
- Redirects to home page

**FR-005: Session Management**
- Sessions persist across browser closures (configurable)
- Sessions expire after inactivity (30 days default)
- Multiple concurrent sessions from same user supported
- User can view all active sessions
- User can logout from specific sessions

**FR-006: Password Reset**
- Users can request password reset via email
- System sends reset link valid for 1 hour
- User clicks link, sets new password
- Old password no longer valid
- Sessions continue after reset (user doesn't need to re-login)

**FR-007: Password Change**
- Authenticated users can change password
- Requires current password verification
- New password must meet requirements (8-128 chars)
- Option to logout all other sessions after change

**FR-008: Social Sign-In (Future)**
- Framework supports GitHub, Google, Discord OAuth
- Not implemented in MVP but architecture supports it

**FR-009: Error Handling**
- User-friendly error messages (don't expose system details)
- Rate limiting on failed login attempts (5 attempts → 15 min lockout)
- Email validation before sending reset links

**FR-010: Security**
- All authentication endpoints use HTTPS only
- Passwords never logged or exposed in error messages
- Database stores hashed passwords only
- Session tokens are cryptographically secure

---

### User Profile & Preferences (FR-011 to FR-020)

**FR-011: User Profile Creation**
- Signup process creates user profile with email
- Profile stores: email, name, profile picture (optional)
- Default values: created_at, updated_at timestamps

**FR-012: Background Information Collection**
- Signup includes questionnaire with 3 dimensions:

  **Dimension 1: Software Development Experience**
  - Options: None, Beginner, Intermediate, Advanced
  - Determines depth of programming concepts explained

  **Dimension 2: Hardware/Robotics Background**
  - Options: None, Beginner, Intermediate, Advanced
  - Determines robotics context and prerequisites

  **Dimension 3: Programming Language Proficiency**
  - Checkboxes: Python, C++, Java, Go, JavaScript, Other
  - Allows multiple selections
  - Determines code example language preferences

**FR-013: Profile Edit & Update**
- Authenticated users can view their profile
- Can edit name, email, profile picture
- Can update background information (all 3 dimensions)
- Changes take effect immediately
- Recommendations refresh on background update

**FR-014: User Preferences**
- Store preferred programming language for code examples
- Store preferred chapter order (default, difficulty-based, etc.)
- Store content types preferred (text, video, interactive, etc.)
- Preferences persist across sessions

**FR-015: Account Management**
- Users can view account details
- Users can download their data (GDPR compliance)
- Users can request account deletion (30-day grace period)
- Users can view all active sessions and logout from any

**FR-016: Privacy & Data Consent**
- Signup includes consent checkbox for data collection
- Users can view what data is collected
- Users can control what data is used for personalization

**FR-017: User Roles (Optional for MVP)**
- Default role: Student
- Future roles: Instructor, Admin
- Role-based content access control

**FR-018: Profile Picture Management**
- Users can upload profile picture (JPG, PNG, max 5MB)
- System stores in cloud storage or local file system
- Picture displays in user dashboard and comments

**FR-019: Notification Preferences**
- Users can opt-in/out of email notifications
- Users can choose notification frequency
- Users can manage notification types (progress updates, recommendations, etc.)

**FR-020: Account Recovery**
- Users with forgotten email can recover via phone verification
- Users can add backup email address
- Two-factor authentication available (future)

---

### Content Personalization (FR-021 to FR-030)

**FR-021: Personalized Chapter Recommendations**
- System analyzes user background (software dev + robotics + languages)
- Recommends chapter order based on background
- Shows difficulty rating (Beginner / Intermediate / Advanced) per chapter
- Shows "Perfect for your level" badge for chapters matching background
- Shows prerequisites for chapters

**FR-022: Background-Specific Content Notes**
- Chapter content includes contextual callouts based on user background
- Examples:
  - For experienced programmers: "As a Python dev, you'll recognize this pattern"
  - For roboticists: "Your hardware knowledge means you can skip the physical basics"
  - For beginners: "Don't worry if this seems complex, we'll explain step-by-step"
- System conditionally renders callouts based on user background

**FR-023: Preferred Code Examples**
- User's preferred language (from FR-014) drives code example selection
- Chapter shows code in user's preferred language first
- Other language examples available but secondary
- User can switch preferred language anytime

**FR-024: Reading Progress Tracking**
- System tracks which chapters user has visited
- Tracks time spent per chapter
- Tracks completion status (started / completed / mastered)
- Dashboard shows progress visualization

**FR-025: Personalized Dashboard**
- Shows recommended next chapters based on background + progress
- Shows learning statistics (chapters completed, time invested, etc.)
- Shows achievements (completed module, perfect exercises, etc.)
- Shows user's background summary with option to edit

**FR-026: Content Difficulty Filtering**
- Users can filter chapters by difficulty
- Users can request "show only beginner" or "show advanced"
- System respects user's current background level as default filter

**FR-027: Learning Path Suggestions**
- System suggests optimal learning order based on background
- For programmers learning robotics: "Start with software concepts, then ROS 2"
- For roboticists learning Python: "Start with language basics, then robotics framework"
- User can follow suggested path or customize

**FR-028: Related Content Recommendations**
- After completing chapter, system recommends next chapters
- Recommendations based on background + completion pattern
- Shows relevance score (85% match, etc.)

**FR-029: Content Tagging by Background**
- Each chapter tagged with difficulty, prerequisites, target audience
- Examples: "Requires Python", "Assumes URDF knowledge", "For programmers"
- Tags drive personalization algorithms

**FR-030: Future Personalization Features** (out of scope for MVP)
- Quiz-based skill assessment to validate background
- Adaptive difficulty based on exercise performance
- Machine learning recommendations based on similar users
- Video content with user-selected language/speed

---

### Data & Storage (FR-031 to FR-035)

**FR-031: User Data Storage**
- Store in PostgreSQL database (Render managed)
- Tables: users, sessions, user_backgrounds, user_preferences
- Encrypted fields: passwords (scrypt hashing)

**FR-032: Session Storage**
- Sessions stored in database or Redis
- Session tokens encrypted
- Session expiry enforcement (30 days default)

**FR-033: User Backgrounds Table**
- Columns: user_id, software_dev_level, robotics_level, languages (JSON array)
- Tracks creation and update timestamps
- Supports versioning for future analytics

**FR-034: User Preferences Table**
- Columns: user_id, preferred_language, preferred_chapter_order, content_types (JSON)
- Defaults to system-wide preferences if not set
- Updates in real-time

**FR-035: Data Privacy & Compliance**
- GDPR compliance: Users can export/delete data
- No third-party data sharing without consent
- Audit log of data access
- No tracking of user behavior beyond progress

---

### Integration Points (FR-036 to FR-040)

**FR-036: Frontend Integration**
- Docusaurus site includes auth UI components
- Signup/signin forms accessible from navbar
- Dashboard accessible after login
- Profile management page
- Content personalization integrated into chapter rendering

**FR-037: Backend Integration**
- Better Auth server running on separate domain or same domain
- Authentication endpoints at `/api/auth/*`
- Session verification for every protected request
- User data available to content service for personalization

**FR-038: Database Integration**
- Better Auth manages user/session tables
- Custom user_backgrounds and user_preferences tables created
- Migrations run automatically on deployment

**FR-039: Environment Configuration**
- BETTER_AUTH_URL environment variable
- BETTER_AUTH_SECRET (32+ character key)
- DATABASE_URL for user/session storage
- Optional SMTP for email verification/password reset

**FR-040: Analytics Integration (Optional)**
- Track signup/signin conversion rate
- Track chapter completion rate by background
- Track recommendation engagement
- No PII in analytics

---

## User Interface Requirements

### Signup & Signin Pages

**Signup Form**:
- Email field with validation
- Password field with strength indicator
- Confirm password field
- Background questionnaire (3 questions)
- Terms of service checkbox
- "Sign Up" button
- Link to signin page

**Signin Form**:
- Email field
- Password field
- "Remember me" checkbox
- "Forgot password?" link
- "Sign In" button
- Link to signup page

**Background Questionnaire** (part of signup):
1. "What's your software development experience?"
   - Dropdown: None / Beginner / Intermediate / Advanced
2. "What's your hardware/robotics background?"
   - Dropdown: None / Beginner / Intermediate / Advanced
3. "Which programming languages do you know?"
   - Checkboxes: Python, C++, Java, Go, JavaScript, Other

---

### Dashboard & Profile Pages

**User Dashboard** (after login):
- Welcome message with user's name
- Background summary card (editable link)
- Recommended chapters section
- Progress visualization (chapters completed, time invested)
- Learning path suggestion
- Recent activity
- Logout button

**Profile Edit Page**:
- Current background information
- Editable fields for name, email, picture
- Background questionnaire (update options)
- Preferences section (preferred language, content types)
- Privacy settings
- Session management
- Account deletion option
- Save changes button

---

## Success Criteria

### User Engagement
1. ✅ New user signup completion rate ≥ 70% (at least 70% of visitors who start signup finish it)
2. ✅ User returns within 7 days ≥ 40% (returning user rate)
3. ✅ Average session duration increases 20% post-launch (vs. pre-launch baseline)

### Authentication & Security
4. ✅ All authentication endpoints protected with HTTPS
5. ✅ Password reset email delivered within 2 minutes 100% of the time
6. ✅ Session hijacking attempts detected and logged
7. ✅ Zero unencrypted password storage (100% scrypt hashing)

### Content Personalization
8. ✅ 80% of users see chapter recommendations relevant to their background (rated by user feedback)
9. ✅ Personalized chapter recommendations have 40% higher engagement than generic recommendations
10. ✅ Users with matching background update see recommendation refresh within 5 seconds

### Performance
11. ✅ Signup/signin completes in <3 seconds (p95 latency)
12. ✅ Dashboard loads in <2 seconds (p95 latency)
13. ✅ Profile edit saves in <1 second
14. ✅ Content personalization (chapter filtering) happens in <500ms

### Data & Privacy
15. ✅ 100% GDPR compliance (data export works, deletion works)
16. ✅ User can view all their data collected
17. ✅ No third-party data sharing without explicit user consent

### User Satisfaction
18. ✅ Onboarding experience rated ≥4/5 stars (user survey)
19. ✅ Content personalization rated helpful by ≥75% of users
20. ✅ Login/signup process rated simple by ≥80% of users

---

## Edge Cases & Error Handling

### Authentication Errors
- Incorrect password: "Incorrect email or password"
- Email not found: "No account found with this email"
- Weak password: "Password must be at least 8 characters"
- Duplicate email: "This email is already registered"
- Account locked (too many failed attempts): "Account temporarily locked. Try again in 15 minutes"
- Email verification required: "Please verify your email before signing in"

### Session Errors
- Expired session: "Your session expired. Please sign in again"
- Invalid session token: "Session not found or invalid"
- Concurrent session limit: "Too many active sessions. Logout from another device"

### Profile/Preference Errors
- Invalid background selection: Show field-level validation
- Email already in use: "This email is already registered"
- Failed preference save: "Could not save preferences. Please try again"
- Failed background update: "Could not update background. Please try again"

### Rate Limiting & Security
- Failed login attempts (5+): Lock account for 15 minutes, notify user
- Password reset spam: Max 3 reset emails per hour
- Session theft detection: Log suspicious login patterns (new device/location)

---

## Assumptions

1. **Database**: PostgreSQL database already available (Render managed DB from previous deployments)
2. **Frontend Framework**: Docusaurus React-based site with ability to add custom components
3. **Backend**: Node.js/Express or similar for Better Auth API (not Python, Better Auth is TypeScript-first)
4. **Email Service**: SMTP configured for verification/reset emails (can use Render's environment)
5. **HTTPS**: All authentication endpoints require HTTPS (enforced)
6. **User Anonymity Default**: Non-authenticated visitors can still browse chapters (auth optional for MVP)
7. **Content Personalization Scope**: Initial MVP only personalizes chapter recommendations; doesn't modify chapter content itself
8. **No Social Auth**: MVP uses email/password only; social auth (GitHub, Google) can be added later
9. **Single Organization**: Single-tenant (one textbook), no multi-tenancy
10. **No Two-Factor Auth**: MVP uses single-factor (email/password); MFA can be added later
11. **Default Language**: Content examples default to Python; other languages available

---

## Out of Scope (Future Features)

- Social sign-in (GitHub, Google, etc.)
- Two-factor authentication
- Multi-language UI (English only for MVP)
- Mobile app (web-only)
- Real-time progress sync across devices
- Video tutorials with personalization
- Community features (forums, discussions)
- Instructor dashboard
- Certificates of completion
- Payment/subscriptions
- Advanced analytics & reporting
- Adaptive difficulty based on quiz performance

---

## Dependencies & Risks

### External Dependencies
- **Better Auth Framework**: Open-source, well-maintained, TypeScript-based
- **PostgreSQL Database**: Already deployed on Render
- **Email Service**: SMTP (can use Render environment or Sendgrid)
- **Docusaurus**: React-based static site generator

### Technical Risks
1. **Risk**: TypeScript/Node.js unfamiliar to Python-focused team
   - **Mitigation**: Use Better Auth's documentation + RAG chatbot backend skills
2. **Risk**: Session management complexity in static site
   - **Mitigation**: Better Auth handles this; use client SDK correctly
3. **Risk**: Personalization recommendations could be inaccurate
   - **Mitigation**: Start simple (rule-based), improve with ML later
4. **Risk**: Email delivery failures for verification/reset
   - **Mitigation**: Implement retry logic, provide backup verification methods

### User Experience Risks
1. **Risk**: Users might not complete signup due to background questions
   - **Mitigation**: Make questions optional, show progress indicator
2. **Risk**: Personalization doesn't match user expectations
   - **Mitigation**: Allow users to override recommendations, provide feedback mechanism

---

## Known Questions & Clarifications

- ✅ **Framework**: Better Auth chosen (TypeScript-based, not Python)
- ✅ **Database**: Using existing Render PostgreSQL
- ✅ **Authentication Method**: Email/password (social auth future)
- ✅ **Personalization Scope**: Recommendations only (MVP), not full content modification
- ✅ **Email Service**: Existing SMTP/environment configuration

---

## Next Steps

1. **Clarify** (if needed): Ask any clarification questions
2. **Plan**: Design technical architecture (Better Auth + Docusaurus integration)
3. **Tasks**: Generate implementation tasks
4. **Implement**: Build signup, signin, profile, personalization
5. **Test**: User acceptance testing
6. **Deploy**: Launch to production

---

## Related Documents

- Better Auth Docs: https://www.better-auth.com/docs
- Docusaurus Docs: https://docusaurus.io/
- Current Deployment: specs/001-render-backend-deployer/
- Project Structure: README.md
