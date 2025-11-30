# Implementation Plan: RAG Chatbot for Physical AI Textbook

**Branch**: `002-rag-chatbot` | **Date**: 2025-11-30 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification with 3 integrated clarifications

**Status**: Ready for Phase 2 (Phase 0-1 design artifacts complete)

---

## Summary

Build and integrate a Retrieval-Augmented Generation (RAG) chatbot that answers user questions about Module 1 textbook content (ROS 2, URDF, Python Integration). The chatbot uses Qdrant Cloud for semantic vector search, Google Gemini LLM for response generation, and Neon PostgreSQL for persistent conversation storage. Embedded as a React widget on all Docusaurus textbook pages with optional user authentication and 30-day automatic data retention.

**Core MVP (P1 Features - Ready for Implementation):**
1. Natural language Q&A about Module 1 content with source citations
2. Selected text-based contextual questions
3. Embedded chat widget on all textbook pages without page reloads
4. Conversation history storage (anonymous sessions via localStorage/cookies)

---

## Technical Context

**Language/Version**: Python 3.11+ (Backend), TypeScript 5+ (Frontend)

**Primary Dependencies**:
- Backend: FastAPI 0.109+, pydantic v2, aiohttp, sentence-transformers
- Frontend: React 18+, TypeScript, Docusaurus 3.x
- AI/ML: Google Generative AI (Gemini API), Qdrant Python SDK
- Database: asyncpg, psycopg2-binary

**Storage**:
- Vector DB: Qdrant Cloud (100K vectors, ~1GB)
  - Endpoint: `https://1521bc26-af63-4594-8df5-c4a2e64c549b.us-east4-0.gcp.cloud.qdrant.io:6333`
- Relational DB: Neon Serverless Postgres
  - URL: `https://ep-rapid-dust-a1k9et2k.apirest.ap-southeast-1.aws.neon.tech/neondb/rest/v1`
- Session Cache: Browser localStorage (5MB limit, anonymous sessions)

**Testing**: pytest (backend), vitest (frontend), integration tests with fixtures

**Target Platform**: Web browser, serverless backend

**Project Type**: Web application (FastAPI backend + React frontend + Docusaurus integration)

**Performance Goals**:
- Chat response: < 5 seconds (P95)
- Widget load: < 3 seconds
- 50+ concurrent users
- Citation accuracy: 95%+
- Hallucination rate: 0%

**Constraints**:
- Qdrant Free Tier rate limits (~100 req/min)
- Neon Free Tier query limits
- Gemini API rate limits (~60 queries/min)
- Browser storage: 5MB localStorage

**Scale/Scope**:
- 10-50 concurrent users at launch
- Module 1 (3 chapters) = ~10K text chunks
- 6 data entities, 8 API endpoints
- 2 languages: English/Urdu

---

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### ✅ Gate 6: RAG Integration (Critical)
- Content structured for RAG (512 token chunks)
- Selection-based queries designed
- Test questions defined in spec
- Expected answers documented

**Status**: ✅ **PASS**

### ✅ Gate 4: AI-Native Workflow
- Spec.md with clear requirements
- API contracts defined
- User stories independently testable
- Spec-driven methodology

**Status**: ✅ **PASS**

### ✅ Gate 1: Technical Accuracy
- Python 3.11 specified
- All dependencies with versions
- Integration tests plan includes fixtures
- API specifications executable

**Status**: ✅ **PASS**

**No Violations**: All relevant gates align with chatbot design.

---

## Phase 0: Research (Complete)

All critical decisions resolved in specification clarifications:

| Item | Decision | Rationale |
|------|----------|-----------|
| User ID | Hybrid: Anonymous + Optional Auth | Maximize adoption, enable sync |
| Data Retention | 30-day auto-deletion | Privacy + cost optimization |
| Rate Limits | Graceful degradation + notification | Maintain service |

**No Additional Research Required**.

---

## Phase 1: Design Artifacts (Complete)

### 1. Data Model

**6 Core Entities:**
1. Message - Chat messages
2. Conversation - Message groupings
3. TextbookContent - Indexed content in Qdrant
4. UserSession - Session tracking (anonymous + auth)
5. UserAccount - Optional login (Phase 2)
6. Citation - Answer sources

All with TTL-based cleanup (30-day expiration).

### 2. API Contracts

**8 Endpoints:**
- `POST /chat/query` - Main Q&A
- `POST /chat/selection` - Selected text queries
- `GET /chat/history` - Retrieve conversation
- `DELETE /chat/session/{id}` - Delete session
- `POST /user/profile` - User settings (Phase 2)
- `GET /health` - Service health
- Additional: feedback, analytics (Phase 2)

**Error Handling**: 400, 401, 404, 429 (rate limited), 503 (unavailable)

### 3. Frontend Integration

**React Components:**
- ChatContainer.tsx (main wrapper)
- MessageList.tsx (display)
- InputArea.tsx (input + text selection)
- SourceCitation.tsx (citations)
- useChat.ts (API hook)

**Features:**
- Text selection integration
- localStorage persistence
- Streaming responses
- Graceful error handling

### 4. Quickstart Guide

Local setup steps for backend, frontend, testing, and deployment.

---

## Project Structure

### Documentation
```
specs/002-rag-chatbot/
├── spec.md
├── plan.md (this file)
├── research.md
├── data-model.md
├── contracts/openapi.yaml
└── quickstart.md
```

### Source Code
```
chatbot-backend/
├── src/ (models, services, api, utils)
├── tests/ (unit, integration, fixtures)
├── requirements.txt
└── .env.example

book/src/components/ChatWidget/
├── ChatContainer.tsx
├── MessageList.tsx
├── InputArea.tsx
├── SourceCitation.tsx
└── useChat.ts
```

**Structure Decision**: Web application (Option 2)
- Independent FastAPI backend (scalable, versionable)
- React widget embedded in Docusaurus
- Neon + Qdrant as managed services

---

## Next Steps

**Phase 2** (`/sp.tasks`):
- Generate detailed task breakdown by user story
- Create independently testable tasks
- Define acceptance criteria
- Estimate complexity

**Phase 3** (Implementation):
- Build FastAPI backend
- Create React widget
- Integrate with Docusaurus
- Write tests
- Deploy

---

**Status**: ✅ **READY FOR PHASE 2**

**Artifacts Completed**:
- Summary & Technical Context
- Constitution Check (3 gates PASS)
- Phase 0 Research (resolved)
- Phase 1 Design (complete)

**Next Command**: `/sp.tasks`

---

Prepared By: Claude Code (Haiku 4.5)
Date: 2025-11-30
Branch: `002-rag-chatbot`
