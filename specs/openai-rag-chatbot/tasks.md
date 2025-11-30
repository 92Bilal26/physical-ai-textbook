# OpenAI ChatKit RAG Chatbot - Actionable Tasks

## Phase 1: Backend Setup (8 tasks)

### Task 1.1: Update Dependencies
**Objective**: Switch from Gemini to OpenAI SDK

```python
# Update chatbot-backend/requirements.txt
# Remove: google-generativeai, sentence-transformers
# Add: openai==1.29.0
```

**Acceptance Criteria**:
- [ ] requirements.txt has openai package
- [ ] Removed all google-generativeai dependencies
- [ ] Removed sentence-transformers
- [ ] `pip install -r requirements.txt` succeeds locally

---

### Task 1.2: Create Config Management
**File**: `src/config.py`

```python
class Settings(BaseSettings):
    # OpenAI
    openai_api_key: str
    openai_model: str = "gpt-4"
    openai_embedding_model: str = "text-embedding-3-small"

    # Qdrant
    qdrant_url: str
    qdrant_api_key: str
    qdrant_collection: str = "textbook_content"

    # Database
    database_url: str

    # Redis
    redis_url: str

    # Server
    host: str = "0.0.0.0"
    port: int = 8000
    environment: str = "production"

    class Config:
        env_file = ".env"
```

**Acceptance Criteria**:
- [ ] Config loads from .env correctly
- [ ] All required variables have defaults or are required
- [ ] Can instantiate Settings() without errors
- [ ] Type checking passes

---

### Task 1.3: Database Models & Schema
**Files**: `src/models/models.py`, `migrations/`

```python
# User Sessions
class UserSession(Base):
    session_id: UUID
    browser_id: str
    created_at: datetime
    expires_at: datetime (30 days from created)

# Conversations
class Conversation(Base):
    id: int
    session_id: UUID (FK)
    created_at: datetime

# Messages
class Message(Base):
    id: int
    conversation_id: int (FK)
    role: Enum(USER, ASSISTANT, ERROR)
    content: str
    timestamp: datetime

# Citations
class Citation(Base):
    id: int
    message_id: int (FK)
    chapter: str
    section: str
    excerpt: str
    confidence_score: float
```

**Acceptance Criteria**:
- [ ] All models defined in SQLAlchemy
- [ ] Alembic migration created
- [ ] Migration runs successfully
- [ ] Tables created in PostgreSQL
- [ ] Foreign keys properly configured

---

### Task 1.4: OpenAI Client Wrapper
**File**: `src/services/openai_service.py`

```python
class OpenAIService:
    def __init__(self, api_key: str, model: str):
        self.client = OpenAI(api_key=api_key)
        self.model = model

    async def chat_completion(self, messages: list, system_prompt: str) -> str:
        # Call OpenAI ChatGPT API
        # Return assistant message
        pass

    async def embed_text(self, text: str) -> list[float]:
        # Use text-embedding-3-small
        # Return embedding vector
        pass

    async def embed_texts(self, texts: list[str]) -> list[list[float]]:
        # Batch embedding
        pass
```

**Acceptance Criteria**:
- [ ] Chat completion works with mock API key
- [ ] Embedding API works
- [ ] Error handling for rate limits
- [ ] Proper async/await
- [ ] Unit tests pass

---

### Task 1.5: Qdrant Integration
**File**: `src/services/qdrant_service.py`

```python
class QdrantService:
    def __init__(self, url: str, api_key: str, collection: str):
        self.client = QdrantClient(url=url, api_key=api_key)
        self.collection = collection

    async def search(self, vector: list[float], limit: int = 5) -> list:
        # Query collection
        # Return top results with scores
        pass

    async def upsert(self, points: list):
        # Add/update points in collection
        pass
```

**Acceptance Criteria**:
- [ ] Connects to Qdrant Cloud
- [ ] Collection exists or created
- [ ] Search returns results with scores
- [ ] Error handling for connection failures

---

### Task 1.6: Redis Cache Service
**File**: `src/services/cache_service.py`

```python
class CacheService:
    def __init__(self, redis_url: str):
        self.redis = redis.from_url(redis_url)

    async def get(self, key: str) -> Any:
        pass

    async def set(self, key: str, value: Any, ttl: int) -> None:
        pass

    async def delete(self, key: str) -> None:
        pass
```

**Acceptance Criteria**:
- [ ] Connects to Redis
- [ ] Set/Get operations work
- [ ] TTL properly configured
- [ ] Graceful fallback if Redis unavailable

---

### Task 1.7: FastAPI Application Setup
**File**: `src/main.py`

```python
app = FastAPI(title="RAG Chatbot")

# CORS middleware
app.add_middleware(CORSMiddleware, ...)

# Health check endpoint
@app.get("/api/v1/health")
async def health_check():
    pass
```

**Acceptance Criteria**:
- [ ] FastAPI app starts without errors
- [ ] CORS properly configured
- [ ] Health endpoint returns 200
- [ ] Documentation available at /docs

---

### Task 1.8: Database Session Management
**File**: `src/db.py`

```python
DATABASE_URL = "postgresql://..."
engine = create_async_engine(DATABASE_URL)
AsyncSessionLocal = sessionmaker(engine, class_=AsyncSession, expire_on_commit=False)

async def get_db():
    async with AsyncSessionLocal() as session:
        yield session
```

**Acceptance Criteria**:
- [ ] Connects to PostgreSQL
- [ ] Async session management works
- [ ] Dependency injection pattern working
- [ ] Connection pooling configured

---

## Phase 2: RAG Pipeline (12 tasks)

### Task 2.1: Embedding Service
**File**: `src/services/embedding_service.py`

```python
class EmbeddingService:
    def __init__(self, openai_service: OpenAIService, cache: CacheService):
        self.openai = openai_service
        self.cache = cache

    async def embed(self, text: str) -> list[float]:
        # Check cache first
        # If miss, call OpenAI API
        # Cache result
        # Return embedding
        pass
```

**Acceptance Criteria**:
- [ ] Calls OpenAI embedding API
- [ ] Results cached in Redis
- [ ] Cache hits reduce API calls
- [ ] Proper error handling

---

### Task 2.2: Vector Search Service
**File**: `src/services/vector_search_service.py`

```python
class VectorSearchService:
    def __init__(self, qdrant: QdrantService):
        self.qdrant = qdrant

    async def search(self, embedding: list[float]) -> list[dict]:
        # Query Qdrant
        # Filter by score threshold (0.5)
        # Return top-5 results
        pass
```

**Acceptance Criteria**:
- [ ] Searches Qdrant collection
- [ ] Filters by confidence score
- [ ] Returns properly formatted results
- [ ] Handles empty results

---

### Task 2.3: RAG Service - Core
**File**: `src/services/rag_service.py`

```python
class RAGService:
    async def process_query(self, question: str, session_id: UUID) -> QueryResponse:
        # Validate input
        # Get embedding
        # Search vectors
        # Build context
        # Call OpenAI
        # Extract citations
        # Calculate confidence
        # Store in DB
        # Return response
        pass
```

**Acceptance Criteria**:
- [ ] Full pipeline works end-to-end
- [ ] Returns structured response
- [ ] Citations properly extracted
- [ ] Confidence calculated correctly
- [ ] Message stored in database

---

### Task 2.4: Context Building
**File**: `src/services/rag_service.py` (method)

```python
def _build_context(self, search_results: list) -> str:
    # Format results into readable context
    # Include source attribution
    # Handle empty results
    pass
```

**Acceptance Criteria**:
- [ ] Context properly formatted
- [ ] Source information included
- [ ] No hardcoded sections missing
- [ ] Readable prompt format

---

### Task 2.5: OpenAI Chat Integration
**File**: `src/services/openai_service.py` (expanded)

```python
async def chat_completion(self,
    messages: list,
    system_prompt: str,
    temperature: float = 0.7
) -> str:
    # Call OpenAI ChatGPT
    # Handle streaming (optional)
    # Return full response
    pass
```

**Acceptance Criteria**:
- [ ] Generates coherent responses
- [ ] Respects system prompt
- [ ] Handles errors gracefully
- [ ] Proper token management

---

### Task 2.6: Citation Extraction
**File**: `src/services/citation_service.py`

```python
class CitationService:
    def extract_citations(self, answer: str, sources: list) -> list[Citation]:
        # Parse answer for source references
        # Match to retrieved sources
        # Calculate confidence
        # Return Citation objects
        pass
```

**Acceptance Criteria**:
- [ ] Citations extracted from answer
- [ ] Matched to source documents
- [ ] Confidence scores calculated
- [ ] Handles ambiguous citations

---

### Task 2.7: Hallucination Detection
**File**: `src/services/hallucination_service.py`

```python
class HallucinationService:
    def detect(self, answer: str) -> float:
        # Look for uncertain language
        # Calculate hallucination probability
        # Return confidence penalty
        pass
```

**Acceptance Criteria**:
- [ ] Detects hedging language
- [ ] Returns confidence score
- [ ] Properly weights signals
- [ ] Doesn't penalize valid uncertainty

---

### Task 2.8: Input Validation
**File**: `src/services/validation_service.py`

```python
class ValidationService:
    def validate_question(self, question: str) -> None:
        # Check length (1-500)
        # Check for malicious input
        # Raise ValueError if invalid
        pass
```

**Acceptance Criteria**:
- [ ] Validates question length
- [ ] Detects SQL injection attempts
- [ ] Detects XSS attempts
- [ ] Proper error messages

---

### Task 2.9: Selection Service
**File**: `src/services/selection_service.py`

```python
class SelectionService:
    async def process_selection(self,
        selected_text: str,
        question: str,
        session_id: UUID,
        chapter: str
    ) -> QueryResponse:
        # Embed selected text + question
        # Search with chapter filter
        # Primary citation = selected text (confidence 1.0)
        # Secondary = related results
        # Weighted confidence: 60% primary, 40% secondary
        pass
```

**Acceptance Criteria**:
- [ ] Handles selected text properly
- [ ] Filters by chapter
- [ ] Correct confidence weighting
- [ ] Returns properly formatted response

---

### Task 2.10: Message Storage
**File**: `src/services/message_service.py`

```python
class MessageService:
    async def save_user_message(self,
        conversation_id: int,
        question: str,
        db: AsyncSession
    ) -> Message:
        pass

    async def save_assistant_message(self,
        conversation_id: int,
        answer: str,
        citations: list[Citation],
        db: AsyncSession
    ) -> Message:
        pass
```

**Acceptance Criteria**:
- [ ] Messages stored in PostgreSQL
- [ ] Citations linked to messages
- [ ] Proper transaction handling
- [ ] Timestamps recorded

---

### Task 2.11: Session Service
**File**: `src/services/session_service.py`

```python
class SessionService:
    async def create_session(self, db: AsyncSession) -> UserSession:
        # Generate UUID
        # Set created_at
        # Set expires_at (now + 30 days)
        # Store in DB
        # Return session
        pass

    async def validate_session(self, session_id: UUID, db: AsyncSession) -> bool:
        # Check exists
        # Check not expired
        # Return bool
        pass
```

**Acceptance Criteria**:
- [ ] Creates sessions with UUID
- [ ] 30-day TTL properly set
- [ ] Session validation works
- [ ] Expired sessions rejected

---

### Task 2.12: Response Formatting
**File**: `src/models/schemas.py`

```python
class QueryResponse(BaseModel):
    answer: str
    sources: list[CitationSchema]
    confidence: float
    message_id: UUID
    session_id: UUID

class CitationSchema(BaseModel):
    id: UUID
    chapter: str
    section: Optional[str]
    excerpt: str
    confidence_score: float
```

**Acceptance Criteria**:
- [ ] Response schemas defined
- [ ] Proper JSON serialization
- [ ] Type validation working
- [ ] API documentation generated

---

## Phase 3: API Endpoints (6 tasks)

### Task 3.1: Health Check Endpoint
**Endpoint**: `GET /api/v1/health`

```python
@app.get("/api/v1/health")
async def health_check(db: AsyncSession = Depends(get_db)):
    return {
        "status": "ok",
        "openai": check_openai_status(),
        "qdrant": check_qdrant_status(),
        "database": check_db_status(db),
        "redis": check_redis_status()
    }
```

**Acceptance Criteria**:
- [ ] Returns 200 on healthy state
- [ ] Checks all service statuses
- [ ] Proper error messages
- [ ] No performance impact

---

### Task 3.2: Query Endpoint
**Endpoint**: `POST /api/v1/chat/query`

```python
@app.post("/api/v1/chat/query")
async def query(
    request: QueryRequest,
    db: AsyncSession = Depends(get_db),
    cache: CacheService = Depends(get_cache),
    rag: RAGService = Depends(get_rag)
) -> QueryResponse:
    # Validate session
    # Execute RAG pipeline
    # Return response
    pass
```

**Acceptance Criteria**:
- [ ] Accepts QueryRequest format
- [ ] Returns QueryResponse format
- [ ] Proper error handling
- [ ] Rate limiting applied

---

### Task 3.3: Selection Endpoint
**Endpoint**: `POST /api/v1/chat/selection`

```python
@app.post("/api/v1/chat/selection")
async def selection(
    request: SelectionRequest,
    db: AsyncSession = Depends(get_db),
    service: SelectionService = Depends(get_selection_service)
) -> QueryResponse:
    pass
```

**Acceptance Criteria**:
- [ ] Handles selected text
- [ ] Returns proper response
- [ ] Confidence weighting correct
- [ ] Citations properly linked

---

### Task 3.4: Chat History Endpoint
**Endpoint**: `GET /api/v1/chat/history/{session_id}`

```python
@app.get("/api/v1/chat/history/{session_id}")
async def get_history(
    session_id: UUID,
    skip: int = 0,
    limit: int = 50,
    db: AsyncSession = Depends(get_db)
) -> list[MessageSchema]:
    pass
```

**Acceptance Criteria**:
- [ ] Returns conversation history
- [ ] Pagination works
- [ ] Proper filtering
- [ ] Secure access

---

### Task 3.5: Session Management
**Endpoint**: `POST /api/v1/chat/session` / `GET /api/v1/chat/session/{session_id}`

```python
@app.post("/api/v1/chat/session")
async def create_session(db: AsyncSession = Depends(get_db)) -> SessionSchema:
    pass

@app.get("/api/v1/chat/session/{session_id}")
async def get_session(session_id: UUID, db: AsyncSession = Depends(get_db)) -> SessionSchema:
    pass
```

**Acceptance Criteria**:
- [ ] Create new session
- [ ] Retrieve session info
- [ ] Validate session
- [ ] Return proper schema

---

### Task 3.6: Error Handling & Responses
**File**: `src/api/error_handlers.py`

```python
# 401 - Invalid/expired session
# 422 - Validation error
# 429 - Rate limited
# 500 - Service error
```

**Acceptance Criteria**:
- [ ] All error codes implemented
- [ ] Proper error messages
- [ ] Error logging
- [ ] User-friendly responses

---

## Phase 4: Caching & Performance (5 tasks)

### Task 4.1: Query Result Caching
**Implementation**: Redis, MD5 hash keys, 1-hour TTL

**Acceptance Criteria**:
- [ ] Query results cached
- [ ] Cache keys properly generated
- [ ] 1-hour TTL working
- [ ] Cache hits reduce latency

---

### Task 4.2: Embedding Caching
**Implementation**: Redis, 24-hour TTL

**Acceptance Criteria**:
- [ ] Embeddings cached
- [ ] Reused for same text
- [ ] 24-hour TTL working
- [ ] Reduces OpenAI API calls

---

### Task 4.3: Rate Limiting
**Algorithm**: Token bucket (60 req/min per session)

**Acceptance Criteria**:
- [ ] Limits enforced
- [ ] 429 responses on limit
- [ ] Retry-After header included
- [ ] Doesn't block legitimate users

---

### Task 4.4: Database Optimization
**Indexes**: session_id, created_at, conversation_id

**Acceptance Criteria**:
- [ ] Indexes created
- [ ] Query performance improved
- [ ] Connection pooling working
- [ ] No N+1 queries

---

### Task 4.5: Monitoring & Metrics
**Prometheus metrics**:
- Query latency (p50, p95, p99)
- Cache hit rate
- API calls count
- Error rate

**Acceptance Criteria**:
- [ ] Metrics exposed at /metrics
- [ ] Prometheus scraping works
- [ ] Grafana dashboards created
- [ ] Alerts configured

---

## Phases 5-8: Frontend, Testing, Docker, Documentation

(Continue with similar detailed breakdown for remaining phases...)

## Total Tasks: 52

## Current Status
- Phase 1: Ready to implement
- Phase 2: Ready to implement
- Phase 3: Ready to implement
- Phase 4-8: Pending Phase 1-3 completion

