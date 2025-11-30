# OpenAI ChatKit RAG Chatbot - Implementation Plan

## Phase 1: Backend Setup & OpenAI Integration (8 tasks)

### 1.1 Project Structure & Dependencies
- Update requirements.txt with OpenAI SDK
- Install dependencies
- Set up virtual environment
- Create project structure

### 1.2 Configuration Management
- Create config.py with Pydantic settings
- Load environment variables (OPENAI_API_KEY, QDRANT_URL, etc.)
- Add validation for required settings

### 1.3 Database Setup
- Initialize PostgreSQL connection
- Create SQLAlchemy models (sessions, conversations, messages, citations)
- Set up Alembic migrations
- Create initial schema

### 1.4 OpenAI Client Setup
- Initialize OpenAI ChatKit client
- Create wrapper for chat completions
- Create wrapper for embedding API
- Add error handling

### 1.5 Qdrant Integration
- Initialize Qdrant client
- Create collection (if not exists)
- Test vector search functionality

### 1.6 Redis Cache Setup
- Initialize Redis connection
- Create cache service
- Implement TTL management (1hr queries, 24hr embeddings)

### 1.7 FastAPI Application
- Create main.py with FastAPI app
- Add CORS middleware
- Add error handling middleware
- Add health check endpoint

### 1.8 Database Utilities
- Create db.py with session management
- Set up async database operations
- Create dependency injection pattern

## Phase 2: RAG Pipeline Implementation (12 tasks)

### 2.1 Embedding Service
- Create embedding_service.py
- Use OpenAI's text-embedding-3-small API
- Cache embeddings in Redis
- Handle API errors gracefully

### 2.2 Vector Search Service
- Create vector_search_service.py
- Query Qdrant with embeddings
- Score threshold filtering (0.5)
- Top-5 result selection

### 2.3 RAG Service - Core
- Create rag_service.py
- Implement main RAG pipeline
- Coordinate embedding → search → generation flow
- Add hallucination detection

### 2.4 Context Building
- Format search results into prompt context
- Add source attribution markers
- Handle empty results gracefully

### 2.5 OpenAI Chat Service
- Create openai_chat_service.py
- Call ChatGPT with context
- System prompt optimization
- Stream response handling (optional)

### 2.6 Citation Extraction
- Parse answer for citations
- Link to source chunks
- Calculate confidence scores
- Handle malformed responses

### 2.7 Hallucination Detection
- Detect uncertain language patterns
- Flag high-risk responses
- Add confidence penalty

### 2.8 Query Validation
- Validate question length (1-500 chars)
- Check for malicious input
- Sanitize input

### 2.9 Selection Service
- Create selection_service.py
- Implement weighted confidence (60% selected, 40% secondary)
- Same-chapter filtering
- Selection-specific prompts

### 2.10 Message Storage
- Store user messages in PostgreSQL
- Store assistant messages with sources
- Link citations to messages
- Set up conversation threads

### 2.11 Session Management
- Create session_service.py
- Generate UUIDs for sessions
- Handle 30-day TTL
- Track browser IDs

### 2.12 Response Formatting
- Create response schemas
- Format citations
- Calculate confidence scores
- Create consistent response format

## Phase 3: API Endpoints (6 tasks)

### 3.1 Health Check Endpoint
- GET /api/v1/health
- Check OpenAI API connectivity
- Check Qdrant status
- Check PostgreSQL connection
- Check Redis connection

### 3.2 Query Endpoint
- POST /api/v1/chat/query
- Full RAG pipeline
- Error handling
- Response formatting

### 3.3 Selection Endpoint
- POST /api/v1/chat/selection
- Selection-specific RAG flow
- Weighted confidence
- Citation handling

### 3.4 Chat History Endpoint
- GET /api/v1/chat/history/{session_id}
- Retrieve conversation history
- Pagination support
- Message filtering

### 3.5 Session Endpoint
- GET /api/v1/chat/session
- Create new session
- Validate existing session
- Return session details

### 3.6 Error Handling
- Custom error responses
- Proper HTTP status codes
- Error logging
- User-friendly messages

## Phase 4: Caching & Performance (5 tasks)

### 4.1 Query Result Caching
- Cache query results in Redis
- 1-hour TTL
- MD5 hash for cache key
- Bypass/invalidate logic

### 4.2 Embedding Caching
- Cache embeddings in Redis
- 24-hour TTL
- Reuse embeddings for same text
- Size limits

### 4.3 Rate Limiting
- Token bucket algorithm per session
- 60 requests/minute limit
- Temporary blocking mechanism
- Rate limit headers in responses

### 4.4 Query Optimization
- Database index on session_id
- Index on created_at for TTL cleanup
- Optimize vector search parameters
- Connection pooling

### 4.5 Monitoring & Metrics
- Track query latency
- Monitor cache hit rate
- Count API calls
- Track errors

## Phase 5: Frontend - ChatWidget (8 tasks)

### 5.1 React Component Structure
- Create ChatWidget.tsx main component
- Set up TypeScript configuration
- Create component hierarchy

### 5.2 Session Management (Frontend)
- Create SessionManager service
- localStorage integration
- Session ID generation (UUID)
- 30-day TTL tracking

### 5.3 API Communication
- Create ChatAPI service
- Query endpoint integration
- Selection endpoint integration
- Error handling

### 5.4 UI Components
- MessageList component
- MessageBubble component
- CitationList component
- InputBox with character counter
- Header with menu

### 5.5 Text Selection Handling
- Detect selected text
- Auto-open on selection (>5 chars)
- Selection context capture
- Preview display

### 5.6 Styling & Theming
- CSS custom properties
- Light/dark theme support
- Responsive design (mobile-first)
- Animation and transitions

### 5.7 State Management
- React Hooks for state
- useCallback for memoization
- useEffect for side effects
- Error state handling

### 5.8 Testing
- Unit tests for services
- Component snapshot tests
- Integration tests
- E2E tests with mock API

## Phase 6: Testing & Quality (6 tasks)

### 6.1 Unit Tests - Backend
- Test embedding service
- Test vector search
- Test RAG pipeline
- Test citation extraction
- Test session management
- Test cache operations

### 6.2 Integration Tests
- End-to-end RAG flow
- Database operations
- Redis caching
- OpenAI API integration

### 6.3 API Tests
- Test all endpoints
- Validate responses
- Error scenarios
- Rate limiting

### 6.4 Frontend Tests
- Component rendering
- User interactions
- Session persistence
- API error handling

### 6.5 Load Testing
- Concurrent requests
- Cache effectiveness
- Database performance
- OpenAI API quotas

### 6.6 Code Quality
- Type checking (mypy)
- Linting (ruff)
- Formatting (black)
- Coverage >90%

## Phase 7: Docker & Deployment (5 tasks)

### 7.1 Dockerfile
- Multi-stage build
- Minimal image size
- Security best practices
- Health checks

### 7.2 Docker Compose
- Development configuration
- Production configuration
- Service dependencies
- Volume management

### 7.3 Database Migrations
- Alembic setup
- Initial migration
- Schema versioning
- Rollback procedures

### 7.4 Environment Configuration
- .env template
- Production secrets
- Different environments (dev, staging, prod)

### 7.5 Health Checks & Monitoring
- Kubernetes probes (if applicable)
- Docker health checks
- Prometheus metrics
- Logging setup

## Phase 8: Documentation & Deployment (5 tasks)

### 8.1 API Documentation
- OpenAPI/Swagger schema
- Endpoint descriptions
- Request/response examples
- Error codes reference

### 8.2 Deployment Guide
- Local development setup
- Docker deployment
- Production checklist
- Troubleshooting guide

### 8.3 README
- Project overview
- Quick start
- API usage examples
- Contributing guidelines

### 8.4 Configuration Documentation
- Environment variables
- Required API keys
- Database setup
- Redis configuration

### 8.5 Runbooks
- Common issues
- Debugging procedures
- Rollback procedures
- Monitoring alerts

## Total Tasks: 52

## Implementation Order
1. Phase 1: Backend Setup (foundation)
2. Phase 2: RAG Pipeline (core logic)
3. Phase 3: API Endpoints (integration)
4. Phase 4: Caching (optimization)
5. Phase 5: Frontend (user interface)
6. Phase 6: Testing (quality assurance)
7. Phase 7: Docker (containerization)
8. Phase 8: Documentation (deployment)

## Key Dependencies
- OpenAI API key (you will provide)
- Qdrant Cloud credentials
- PostgreSQL database
- Redis instance
