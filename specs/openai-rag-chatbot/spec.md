# OpenAI ChatKit RAG Chatbot - Specification

## Overview
Physical AI Textbook RAG (Retrieval-Augmented Generation) chatbot powered by OpenAI's ChatKit SDK, Vector database (Qdrant), and PostgreSQL.

## Architecture

### Core Components
1. **Backend**: FastAPI (Python) with async support
2. **LLM**: OpenAI ChatKit SDK (GPT-4/GPT-3.5)
3. **Vector DB**: Qdrant Cloud (semantic search)
4. **Database**: PostgreSQL (sessions, conversations, citations)
5. **Cache**: Redis (query results, embeddings)
6. **Frontend**: React ChatWidget (TypeScript)

### Data Flow
```
User Question
    ↓
Session Validation
    ↓
Cache Check (Redis)
    ↓
Query Embedding (OpenAI API)
    ↓
Vector Search (Qdrant)
    ↓
Context Building
    ↓
LLM Processing (OpenAI ChatKit)
    ↓
Response Generation + Citations
    ↓
Cache Storage
    ↓
Response to User
```

## API Endpoints

### 1. Health Check
- **GET** `/api/v1/health`
- Returns service status including OpenAI API, Qdrant, PostgreSQL

### 2. Query Endpoint
- **POST** `/api/v1/chat/query`
- Input: `{question, session_id, page_context}`
- Output: `{answer, sources[], confidence, message_id}`

### 3. Selection Endpoint
- **POST** `/api/v1/chat/selection`
- Input: `{selected_text, question, session_id, chapter, section}`
- Output: `{answer, sources[], confidence, message_id}`

## Database Schema

### Tables
1. **user_sessions**
   - session_id (UUID, PK)
   - created_at, expires_at (30-day TTL)
   - browser_id (for client tracking)

2. **conversations**
   - id (PK)
   - session_id (FK)
   - created_at

3. **messages**
   - id (PK)
   - conversation_id (FK)
   - role (USER/ASSISTANT/ERROR)
   - content, timestamp

4. **citations**
   - id (PK)
   - message_id (FK)
   - chapter, section, excerpt
   - confidence_score

## RAG Pipeline

### Step 1: Query Processing
- Validate session
- Clean/normalize question (1-500 chars)

### Step 2: Embedding Generation
- Use OpenAI's embedding API (text-embedding-3-small)
- Cache embeddings in Redis

### Step 3: Vector Search
- Query Qdrant collection with embeddings
- Retrieve top-5 matching chunks (min score: 0.5)

### Step 4: Context Building
- Format retrieved chunks as context
- Build prompt with system instructions

### Step 5: LLM Processing
- Call OpenAI ChatKit SDK
- System prompt: "Answer only based on provided context, no general knowledge"
- Detect hallucinations (phrases like "I believe", "might be")

### Step 6: Citation Extraction
- Parse citations from LLM response
- Link to source chunks
- Calculate confidence scores

### Step 7: Response Assembly
- Format answer with sources
- Calculate overall confidence (average of citations)
- Store in PostgreSQL

## Key Features

### Caching Strategy
- Query results: 1 hour (Redis)
- Embeddings: 24 hours (Redis)
- Cache key: MD5(question + session_id)

### Rate Limiting
- 60 requests/minute per session (token bucket)
- 1000 requests/hour per session
- Returns 429 with Retry-After header

### Session Management
- 30-day TTL for sessions
- localStorage persistence on client
- Auto-cleanup on expiry

### Error Handling
- 401: Invalid/expired session
- 422: Validation error (question length, format)
- 429: Rate limited
- 500: Service error with context

## Configuration

### Environment Variables
```
# OpenAI
OPENAI_API_KEY=sk-...
OPENAI_MODEL=gpt-4  # or gpt-3.5-turbo
OPENAI_EMBEDDING_MODEL=text-embedding-3-small

# Qdrant
QDRANT_URL=https://...
QDRANT_API_KEY=...
QDRANT_COLLECTION=textbook_content

# Database
DATABASE_URL=postgresql://user:pass@host/db

# Redis
REDIS_URL=redis://:password@host:6379

# Server
HOST=0.0.0.0
PORT=8000
ENVIRONMENT=production
DEBUG=false
```

## Testing Requirements

### Unit Tests
- Embedding service (OpenAI API mock)
- RAG service (vector search mock)
- Session management
- Citation extraction

### Integration Tests
- Full query pipeline
- Selection queries
- Error handling
- Database operations

### E2E Tests
- Health check
- Complete chat flow
- Rate limiting
- Cache hits/misses

## Deployment

### Local Development
```bash
docker-compose up -d
alembic upgrade head
python -m uvicorn src.main:app --reload
```

### Production
```bash
docker build -t rag-chatbot:latest .
docker-compose -f docker-compose.prod.yml up -d
```

## Success Criteria

- [ ] OpenAI API integration working
- [ ] Vector search returning relevant results
- [ ] Citations properly extracted and formatted
- [ ] Cache reducing API calls by 30%+
- [ ] Response time < 2 seconds (p95)
- [ ] Rate limiting preventing abuse
- [ ] All tests passing (>90% coverage)
- [ ] ChatWidget rendering correctly
- [ ] Session management functioning
