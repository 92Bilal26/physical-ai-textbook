# Implementation Tasks: RAG Chatbot for Physical AI Textbook

**Feature**: 002-rag-chatbot
**Branch**: `002-rag-chatbot`
**Created**: 2025-11-30
**Status**: Ready for Implementation

**Plan Reference**: [plan.md](./plan.md) | **Spec Reference**: [spec.md](./spec.md)

---

## Task Execution Strategy

**MVP Scope (P1)**: User Stories 1-3 (113 tasks, ~8-10 weeks for 2-person team)
- US1: Natural language Q&A with citations
- US2: Selected text questions
- US3: Embedded widget on all pages

**Phase 2 (P2)**: User Stories 4-5 (18 tasks, future enhancement)
- US4: Multi-turn conversation context
- US5: Advanced source references

**Key Dependencies**:
1. Setup phase → blocks all (infrastructure, dependencies)
2. Foundational phase → blocks user stories (database, core services)
3. User stories → independent (can parallelize)

**Parallel Opportunities**:
- Backend API (US1-3) parallel with frontend widget (US3)
- Database migrations parallel with dependency installation
- Model creation (T019-T023) all parallel
- Tests can run parallel with implementation

---

## Phase 1: Project Setup (10 tasks)

### Goal: Initialize project structure, install dependencies, configure environment

### Independent Test Criteria
- All dependencies installed and verified
- Environment configuration secure (no secrets exposed)
- Project structure matches plan.md
- CI/CD pipeline operational

### Tasks

- [ ] T001 Create project directory structure: chatbot-backend/, book/src/components/ChatWidget/
- [ ] T002 Create backend structure: src/ (models, services, api, utils), tests/ (unit, integration, fixtures), migrations/
- [ ] T003 [P] Create requirements.txt with: FastAPI 0.109+, pydantic v2, asyncpg, psycopg2-binary, sentence-transformers, python-dotenv, aiohttp
- [ ] T004 [P] Create .env.example template with: NEON_DATABASE_URL, QDRANT_URL, QDRANT_API_KEY, GEMINI_API_KEY
- [ ] T005 [P] Create Dockerfile and docker-compose.yml for local development
- [ ] T006 [P] Create React/TypeScript frontend structure with src/, tests/, types/
- [ ] T007 Create conftest.py with pytest fixtures for mock Qdrant/Neon responses
- [ ] T008 Setup Alembic migrations: alembic/ directory with env.py and script.py.mako
- [ ] T009 Create .github/workflows/ci.yml for GitHub Actions: lint, test, build on PR
- [ ] T010 Create README.md with setup, testing, and deployment instructions

---

## Phase 2: Foundational Infrastructure (27 tasks)

### Goal: Build shared services all user stories depend on

### Independent Test Criteria
- Database schema created (6 tables with proper indexes)
- All foundational services have unit tests (>80% coverage)
- Qdrant connection validated
- Session service creates/retrieves sessions
- Gemini API connectivity tested (with mock)

### Database Schema & Migrations

- [ ] T011 Create Alembic migration for Message table: id, conversation_id (FK), role (enum), content, source_references (JSON), timestamp, expires_at
- [ ] T012 Create Alembic migration for Conversation table: id, user_session_id (FK), page_context, created_at, updated_at, expires_at
- [ ] T013 Create Alembic migration for TextbookContent: id, chapter, section, content_text, vector_embedding_id, metadata (JSONB)
- [ ] T014 Create Alembic migration for UserSession: session_id, anonymous_browser_id, user_id, page_context, created_at, updated_at, conversation_ids (JSON), expires_at
- [ ] T015 Create Alembic migration for Citation: id, message_id (FK), chapter, section, content_excerpt, link, expires_at
- [ ] T016 Create Alembic migration indexes: (conversation_id, timestamp), (user_session_id), (expires_at) for cleanup
- [ ] T017 Run migrations: alembic upgrade head
- [ ] T018 Create seed migration: insert sample Module 1 content into TextbookContent table

### Core Models & Schemas

- [ ] T019 [P] Create src/models/message.py with SQLAlchemy Message model
- [ ] T020 [P] Create src/models/conversation.py with SQLAlchemy Conversation model
- [ ] T021 [P] Create src/models/citation.py with SQLAlchemy Citation model
- [ ] T022 [P] Create src/models/session.py with SQLAlchemy UserSession model
- [ ] T023 [P] Create src/models/schemas.py with Pydantic request/response schemas

### Foundational Services

- [ ] T024 Create src/services/neon_service.py: connection pool, CRUD operations for all models
- [ ] T025 Create src/services/qdrant_service.py: Qdrant Cloud connection, vector storage/retrieval, search
- [ ] T026 Create src/services/embedding_service.py: text embedding (sentence-transformers), caching
- [ ] T027 Create src/services/gemini_service.py: Gemini API client, response generation, rate limit retry logic
- [ ] T028 Create src/services/session_service.py: create/retrieve/delete sessions, TTL management
- [ ] T029 Create src/utils/chunking.py: split content into 512-token chunks with metadata preservation
- [ ] T030 Create src/utils/errors.py: custom exception classes (RAGException, DatabaseException, QdrantException)
- [ ] T031 Create src/utils/logging.py: structured logging setup

### Tests for Foundational

- [ ] T032 Create tests/unit/test_neon_service.py: test CRUD, connection, error handling
- [ ] T033 Create tests/unit/test_qdrant_service.py: test vector storage/retrieval with mocks
- [ ] T034 Create tests/unit/test_session_service.py: test creation, anonymous vs auth, TTL
- [ ] T035 [P] Create tests/integration/test_neon_integration.py: integration with test database
- [ ] T036 [P] Create tests/integration/test_qdrant_integration.py: integration with Qdrant Cloud (test vectors)
- [ ] T037 Create tests/fixtures/mock_data.py: mock Qdrant responses, test conversations

---

## Phase 3: User Story 1 - Natural Language Q&A (24 tasks, P1)

### Goal: Accept questions, retrieve content, generate answers with citations

### User Story
"As a reader, I want to ask questions about ROS 2 content in natural language and receive accurate answers with citations."

### Independent Test Criteria
- Submit question via POST /api/v1/chat/query
- Question embedded and Qdrant searched
- Top-5 content chunks passed to Gemini
- Response includes answer + Citation array
- Each citation: chapter, section, excerpt, link
- Conversation stored with 30-day expiration
- Out-of-scope questions rejected gracefully
- Response time < 5 seconds (P95)

### Tasks - API Endpoint

- [ ] T038 [US1] Create src/api/routes.py with POST /api/v1/chat/query endpoint
- [ ] T039 [US1] Implement query validation: {question: string (1-500), session_id: UUID, page_context?: string}
- [ ] T040 [US1] Implement session retrieval/creation in query endpoint
- [ ] T041 [US1] Implement embedding generation for question
- [ ] T042 [US1] Implement Qdrant search: top-5 similar content chunks
- [ ] T043 [US1] Implement Gemini API call with RAG prompt: "Answer using ONLY: [retrieved content]. Question: [user_question]"
- [ ] T044 [US1] Implement Citation extraction from retrieved chunks
- [ ] T045 [US1] Implement Message storage: question + response in database
- [ ] T046 [US1] Implement endpoint response: {answer, sources: Citation[], session_id, message_id}
- [ ] T047 [US1] Implement error handling: 400 (invalid), 401 (session expired), 429 (rate limited), 503 (unavailable)
- [ ] T048 [US1] Implement rate limit handling: exponential backoff, circuit breaker for Gemini API

### Tasks - RAG Service

- [ ] T049 [US1] Create src/services/rag_service.py: orchestrate embedding+retrieval+generation
- [ ] T050 [US1] Implement scope validation: detect out-of-scope questions (France capital → reject)
- [ ] T051 [US1] Implement hallucination prevention: reject if retrieved_score < threshold
- [ ] T052 [US1] Implement RAG prompt formatting with clear context boundaries

### Tasks - Tests

- [ ] T053 [US1] Create tests/unit/test_rag_service.py with mock Qdrant/Gemini
- [ ] T054 [US1] Create tests/integration/test_query_endpoint.py: full flow test
- [ ] T055 [US1] Create tests/integration/test_hallucination_prevention.py: verify scope checking
- [ ] T056 [US1] Create tests/unit/test_citation_accuracy.py: verify citation extraction

---

## Phase 4: User Story 2 - Selected Text Questions (16 tasks, P1)

### Goal: Enable focused Q&A on highlighted passages

### User Story
"As a reader, I want to select text and ask a question about just that passage for focused explanations."

### Independent Test Criteria
- Accept {selected_text, question, session_id, chapter}
- Validate selection > 5 chars, < 5000 chars
- Answer using ONLY selected text (no Qdrant retrieval)
- Response indicates focused on selection
- Citations point to selection only

### Tasks

- [ ] T057 [US2] Create POST /api/v1/chat/selection endpoint in src/api/routes.py
- [ ] T058 [US2] Implement selection validation (length check)
- [ ] T059 [US2] Implement selection passage as exclusive context (no Qdrant search)
- [ ] T060 [US2] Implement Gemini call with selection: "Answer using ONLY this text: [selected_text]. Question: [user_question]"
- [ ] T061 [US2] Implement Citation creation with selection excerpt
- [ ] T062 [US2] Implement Message storage marked as selection-based
- [ ] T063 [US2] Implement response: {answer, sources, focused_on_selection: true}
- [ ] T064 [US2] Create useChat hook in book/src/components/ChatWidget/useChat.ts with selection() function
- [ ] T065 [US2] Create InputArea.tsx: detect text selection via window.getSelection()
- [ ] T066 [US2] Implement InputArea: "Ask about this" button on selection detection
- [ ] T067 [US2] Implement InputArea: send to /chat/selection endpoint when selection exists
- [ ] T068 [US2] Create tests/unit/test_selection_rag.py: test selection-constrained RAG
- [ ] T069 [US2] Create tests/integration/test_selection_endpoint.py: full selection flow
- [ ] T070 [US2] Create tests/frontend/InputArea.test.tsx: test text selection detection
- [ ] T071 [US2] Create tests/frontend/useChat.test.ts: test selection() hook function
- [ ] T072 [US2] Implement selection endpoint: suggest full-textbook search if info not in selection

---

## Phase 5: User Story 3 - Embedded Widget (35 tasks, P1)

### Goal: Create chat widget, embed in Docusaurus, ensure fast loading

### User Story
"As a reader, I want the chat available on every page without leaving the textbook."

### Independent Test Criteria
- Widget loads in < 3 seconds
- Widget persists across page navigation
- localStorage session storage working
- No interference with page rendering
- Responsive on desktop/tablet
- Graceful error handling

### Tasks - React Components

- [ ] T073 [US3] Create book/src/components/ChatWidget/types.ts with TypeScript interfaces
- [ ] T074 [US3] Create ChatContainer.tsx: main widget state management, session lifecycle
- [ ] T075 [US3] Create MessageList.tsx: render conversation with proper formatting
- [ ] T076 [US3] Create InputArea.tsx: text input, send button, selection detection (reuse from US2)
- [ ] T077 [US3] Create SourceCitation.tsx: display Citation with chapter, section, excerpt, link
- [ ] T078 [US3] Create ErrorBoundary.tsx: catch errors, show recovery UI
- [ ] T079 [US3] Create ChatWidget.css: styling (dark mode compatible, no page interference)

### Tasks - Custom Hook

- [ ] T080 [US3] Create useChat.ts hook: manage API state, responses, errors
- [ ] T081 [US3] Implement useChat query() function: POST /chat/query
- [ ] T082 [US3] Implement useChat selection() function: POST /chat/selection
- [ ] T083 [US3] Implement useChat history() function: GET /chat/history
- [ ] T084 [US3] Implement useChat deleteSession() function: DELETE /chat/session/{id}
- [ ] T085 [US3] Implement useChat session management: init session_id, store in localStorage
- [ ] T086 [US3] Implement useChat localStorage persistence: save session_id and last 50 messages
- [ ] T087 [US3] Implement useChat error handling: network errors, retry button
- [ ] T088 [US3] Implement useChat retry logic: exponential backoff for 429/503

### Tasks - Docusaurus Integration

- [ ] T089 [US3] Modify book/src/pages/_app.tsx or theme layout: embed <ChatWidget /> on all pages
- [ ] T090 [US3] Configure lazy-loading: ChatWidget script doesn't block page render
- [ ] T091 [US3] Implement toggle button: chat widget hidden by default
- [ ] T092 [US3] Position fixed bottom-right, z-index 1000+ (above page content)
- [ ] T093 [US3] Add keyboard shortcut (Cmd+/ or Ctrl+?) to open chat
- [ ] T094 [US3] Ensure widget CSS scoped (no page style bleed)
- [ ] T095 [US3] Implement conversation persistence: load history on page load

### Tasks - Performance

- [ ] T096 [US3] Lazy-load ChatWidget on first user interaction
- [ ] T097 [US3] Implement message virtualization (render only visible messages)
- [ ] T098 [US3] Implement input debounce (500ms) before API call
- [ ] T099 [US3] Implement response caching in localStorage
- [ ] T100 [US3] Benchmark: widget load < 3s, interaction < 500ms

### Tasks - Tests

- [ ] T101 [US3] Create ChatWidget.test.tsx: render, message display, interaction
- [ ] T102 [US3] Create useChat.test.ts: hook lifecycle, API calls, localStorage sync
- [ ] T103 [US3] Create InputArea.test.tsx: input field, send, selection
- [ ] T104 [US3] Create SourceCitation.test.tsx: citation rendering
- [ ] T105 [US3] Create tests/e2e/chatbot-widget.spec.ts: Playwright E2E test on actual page
- [ ] T106 [US3] Create tests/performance/widget-load-time.spec.ts: benchmark loading time
- [ ] T107 [US3] Test conversation history persistence: localStorage + Neon sync

---

## Phase 6: User Story 4 - Conversation Context (9 tasks, P2)

### Goal: Enable multi-turn conversations with context awareness

### User Story
"As a reader, I want follow-up questions to reference previous answers naturally."

### Independent Test Criteria
- Chatbot retrieves conversation history
- Previous 5 messages included in LLM context
- Responses contextually aware
- Full conversation history retrievable

### Tasks

- [ ] T108 [US4] Extend POST /api/v1/chat/query: include previous 5 messages from database
- [ ] T109 [US4] Modify rag_service: format multi-turn prompt with conversation history
- [ ] T110 [US4] Implement conversation context caching (Redis optional)
- [ ] T111 [US4] Extend GET /api/v1/chat/history: full conversation with pagination (limit 50)
- [ ] T112 [US4] Enhance ChatContainer: fetch full history on mount
- [ ] T113 [US4] Enhance MessageList: display all historical messages
- [ ] T114 [US4] Create tests/unit/test_conversation_context.py: context inclusion in prompt
- [ ] T115 [US4] Create tests/integration/test_followup_questions.py: multi-turn conversation flow
- [ ] T116 [US4] Create tests/frontend/conversation-history.test.tsx: history display

---

## Phase 7: User Story 5 - Advanced Citations (9 tasks, P2)

### Goal: Enhance source display with previews and confidence scores

### User Story
"As a reader, I want source previews and confidence indicators to verify answer quality."

### Independent Test Criteria
- Citations show chapter + section + excerpt (100-200 chars)
- Confidence scores displayed (0-1)
- Link navigates to textbook section
- Hover shows preview modal

### Tasks

- [ ] T117 [US5] Extend Citation model: add confidence_score (float 0-1)
- [ ] T118 [US5] Modify rag_service: calculate confidence from retrieval score
- [ ] T119 [US5] Extend POST /api/v1/chat/query response: include confidence_score
- [ ] T120 [US5] Enhance SourceCitation: display confidence as visual indicator (stars, percentage)
- [ ] T121 [US5] Extend TextbookContent: add section_url field
- [ ] T122 [US5] Implement: clicking citation navigates to textbook section
- [ ] T123 [US5] Implement: hover modal shows preview with link
- [ ] T124 [US5] Create tests/unit/test_confidence_scoring.py: confidence calculation
- [ ] T125 [US5] Create tests/frontend/SourceCitation.test.tsx: citation with confidence display

---

## Phase 8: Polish & Cross-Cutting (25 tasks)

### Tasks - Monitoring

- [ ] T126 Add structured logging to all services: request ID tracking, response times
- [ ] T127 Create src/utils/metrics.py: track API latency, error rates, hallucination attempts
- [ ] T128 Create /api/v1/health endpoint: return service status (Neon, Qdrant, Gemini, API)
- [ ] T129 Setup error alerting: Sentry integration (optional)

### Tasks - Maintenance

- [ ] T130 Create src/scripts/cleanup_expired_data.py: delete messages/conversations > 30 days
- [ ] T131 Create src/scripts/index_textbook.py: load Module 1 into Qdrant (512-token chunks)
- [ ] T132 Configure database connection pooling: PgBouncer or asyncpg settings
- [ ] T133 Implement circuit breaker for external APIs: fail gracefully when down

### Tasks - Documentation

- [ ] T134 Create SETUP.md: local development setup (backend + frontend)
- [ ] T135 Create TESTING.md: run test suites (unit, integration, E2E)
- [ ] T136 Create DEPLOYMENT.md: deploy backend (serverless), deploy widget
- [ ] T137 Create API.md: OpenAPI docs with endpoints, schemas, errors
- [ ] T138 Create book/docs/chatbot/guide.md: user guide for textbook readers

### Tasks - Deployment

- [ ] T139 Create GitHub Actions: on PR - lint, test, build
- [ ] T140 Create deployment workflow: on merge - deploy to staging, run integration tests
- [ ] T141 Setup environment configuration: dev, staging, production secrets
- [ ] T142 Create Docker image: build and push to registry
- [ ] T143 Setup monitoring: alerts for error rate > 5%, latency > 5s P95

### Tasks - QA & Validation

- [ ] T144 End-to-end test: session → question → answer → database verification
- [ ] T145 Hallucination QA: verify out-of-scope questions rejected
- [ ] T146 Load test: 50 concurrent users, verify < 5s P95 latency
- [ ] T147 Citation accuracy QA: manually verify 100+ answers have correct citations
- [ ] T148 Performance tuning: optimize slow queries, cache frequently asked questions
- [ ] T149 Security review: no API key exposure, SQL injection prevention, input validation
- [ ] T150 Accessibility review: keyboard navigation, screen reader (WCAG AA)

---

## Summary

**Total Tasks**: 150

**By Phase**:
- Setup: 10 tasks
- Foundational: 27 tasks
- US1 Q&A: 24 tasks
- US2 Selection: 16 tasks
- US3 Widget: 35 tasks
- US4 Context: 9 tasks
- US5 Citations: 9 tasks
- Polish: 20 tasks

**MVP Scope (P1)**:
- Setup + Foundational + US1-US3 = **112 tasks**
- Estimate: **8-10 weeks for 2-person team**

**Parallelizable**:
- Setup: 10 tasks (T001-T010) run in parallel
- Models: 5 tasks (T019-T023) run in parallel
- Services: 8 tasks (T024-T031) run in parallel
- Tests: 6 tasks (T032-T037) run in parallel
- API endpoints (US1-3) parallel with React components

**Blocking Dependencies**:
1. Setup (T001-T010) → everything
2. Foundational (T011-T037) → user stories
3. Database migrations (T011-T018) → foundational services

---

**Status**: ✅ READY FOR IMPLEMENTATION

**Next**: Begin Phase 1 or jump to specific tasks

Prepared By: Claude Code (Haiku 4.5)
Date: 2025-11-30
Branch: `002-rag-chatbot`
