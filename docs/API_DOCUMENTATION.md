# RAG Chatbot API Documentation

Complete API reference for the Physical AI Textbook RAG Chatbot.

## Overview

The RAG Chatbot API provides two primary endpoints for asking questions about textbook content:
- **Query Endpoint**: General questions about textbook content
- **Selection Endpoint**: Questions about highlighted/selected text

All requests require a valid `session_id` (UUID format).

## Authentication

Sessions are managed via localStorage on the client. Each user gets a unique `session_id` that persists for 30 days.

```bash
# Example session creation
curl -X GET http://localhost:8000/api/v1/health \
  -H "Content-Type: application/json"
```

## Base URL

```
http://localhost:8000/api/v1
```

## Endpoints

### 1. Health Check

**GET** `/health`

Check API and service health status.

**Response (200 OK):**
```json
{
  "status": "ok",
  "version": "0.1.0",
  "qdrant_status": "healthy",
  "neon_status": "healthy",
  "gemini_status": "healthy"
}
```

**Possible Status Values:**
- `ok`: All services healthy
- `degraded`: Some services unavailable
- `down`: Critical services down

---

### 2. Query Endpoint

**POST** `/chat/query`

Ask a general question about textbook content.

**Request Body:**
```json
{
  "question": "What is robotics?",
  "session_id": "550e8400-e29b-41d4-a716-446655440000",
  "page_context": "Chapter 1"
}
```

**Parameters:**
| Parameter | Type | Required | Description |
|-----------|------|----------|-------------|
| `question` | string | Yes | Question (1-500 characters) |
| `session_id` | UUID | Yes | User session ID |
| `page_context` | string | No | Current page/chapter context |

**Response (200 OK):**
```json
{
  "answer": "Robotics is the integration of mechanical engineering, electronics, and software to create autonomous systems...",
  "sources": [
    {
      "id": "550e8400-e29b-41d4-a716-446655440001",
      "chapter": "Chapter 1: Introduction",
      "section": "What is Robotics",
      "content_excerpt": "Robotics is the integration of mechanical engineering, electronics, and software...",
      "link": null,
      "confidence_score": 0.95
    }
  ],
  "session_id": "550e8400-e29b-41d4-a716-446655440000",
  "message_id": "550e8400-e29b-41d4-a716-446655440002",
  "confidence": 0.92
}
```

**Error Responses:**

| Status | Error | Description |
|--------|-------|-------------|
| 400 | Validation Error | Invalid request format |
| 401 | Invalid Session | Session expired or invalid |
| 422 | Validation Error | Question too short/long |
| 429 | Rate Limited | Too many requests |
| 500 | Service Error | API or LLM error |

**Example cURL:**
```bash
curl -X POST http://localhost:8000/api/v1/chat/query \
  -H "Content-Type: application/json" \
  -d '{
    "question": "What is robotics?",
    "session_id": "550e8400-e29b-41d4-a716-446655440000",
    "page_context": "Chapter 1"
  }'
```

---

### 3. Selection Endpoint

**POST** `/chat/selection`

Ask a question about selected/highlighted text from the textbook.

**Request Body:**
```json
{
  "selected_text": "Robots are autonomous machines that perform tasks.",
  "question": "What are the key characteristics?",
  "session_id": "550e8400-e29b-41d4-a716-446655440000",
  "chapter": "Chapter 1",
  "section": "Basics"
}
```

**Parameters:**
| Parameter | Type | Required | Description |
|-----------|------|----------|-------------|
| `selected_text` | string | Yes | Highlighted text (5-5000 chars) |
| `question` | string | Yes | Question about selection (1-500 chars) |
| `session_id` | UUID | Yes | User session ID |
| `chapter` | string | Yes | Chapter name |
| `section` | string | No | Section name (optional) |

**Response (200 OK):**
```json
{
  "answer": "Based on the selected text, robots have several key characteristics...",
  "sources": [
    {
      "id": "550e8400-e29b-41d4-a716-446655440003",
      "chapter": "Chapter 1",
      "section": "Basics",
      "content_excerpt": "Robots are autonomous machines that perform tasks.",
      "link": null,
      "confidence_score": 1.0
    }
  ],
  "session_id": "550e8400-e29b-41d4-a716-446655440000",
  "message_id": "550e8400-e29b-41d4-a716-446655440004",
  "confidence": 0.95
}
```

**Example cURL:**
```bash
curl -X POST http://localhost:8000/api/v1/chat/selection \
  -H "Content-Type: application/json" \
  -d '{
    "selected_text": "Robots are autonomous machines that perform tasks.",
    "question": "What are the key characteristics?",
    "session_id": "550e8400-e29b-41d4-a716-446655440000",
    "chapter": "Chapter 1",
    "section": "Basics"
  }'
```

---

### 4. API Documentation

**GET** `/docs`

Get available API endpoints and descriptions.

**Response (200 OK):**
```json
{
  "title": "RAG Chatbot API Documentation",
  "version": "0.1.0",
  "endpoints": {
    "POST /api/v1/chat/query": "Send question about textbook content",
    "POST /api/v1/chat/selection": "Ask question about selected text",
    "GET /api/v1/chat/history": "Retrieve conversation history",
    "GET /api/v1/health": "Service health status"
  }
}
```

---

## Rate Limiting

The API implements rate limiting to prevent abuse.

**Default Limits:**
- 60 requests per minute per session
- 1000 requests per hour per session

**Rate Limit Headers:**
```
X-RateLimit-Limit: 60
X-RateLimit-Remaining: 45
X-RateLimit-Reset: 1699000000
Retry-After: 30
```

**429 Response:**
```json
{
  "detail": "Rate limit exceeded",
  "retry_after": 30
}
```

---

## Response Codes

| Code | Meaning |
|------|---------|
| 200 | Success |
| 400 | Bad Request |
| 401 | Unauthorized (invalid session) |
| 422 | Unprocessable Entity (validation) |
| 429 | Too Many Requests (rate limited) |
| 500 | Internal Server Error |

---

## Confidence Scores

Each response includes a confidence score (0.0-1.0) indicating answer reliability:

- **0.9-1.0**: High confidence (direct match)
- **0.7-0.9**: Good confidence (multiple supporting sources)
- **0.5-0.7**: Moderate confidence (single source)
- **<0.5**: Low confidence (weak match)

---

## Caching

Responses are cached for:
- **Query results**: 1 hour
- **Embeddings**: 24 hours

Identical questions within the cache window return cached results.

---

## Webhooks (Advanced)

Register webhooks for event notifications (available in admin API).

**Event Types:**
- `query.completed`: Query processed
- `error.occurred`: Error during processing
- `session.created`: New session created
- `ratelimit.exceeded`: Rate limit hit

---

## Best Practices

1. **Session Management**
   - Store session_id in localStorage
   - Reuse session_id within 30-day window
   - Clear session on logout

2. **Error Handling**
   - Implement exponential backoff for retries
   - Handle 429 responses with Retry-After
   - Log 500 errors for debugging

3. **Performance**
   - Batch similar questions to benefit from caching
   - Use page_context for better results
   - Limit question length to 500 characters

4. **Data**
   - Don't send PII in questions
   - Use generic page references in page_context
   - Store conversation history locally

---

## SDKs

Official SDKs available for:
- JavaScript/TypeScript (React, Vue, Angular)
- Python
- Java
- C#

See repository for SDK documentation.

---

## Support

For issues or questions:
1. Check [Troubleshooting Guide](./TROUBLESHOOTING.md)
2. Review [Deployment Guide](./DEPLOYMENT.md)
3. Check service health endpoint
4. Submit issue on GitHub

---

## Version History

| Version | Date | Changes |
|---------|------|---------|
| 0.1.0 | 2024-01 | Initial release |

---

**Last Updated:** January 2024
**Maintainer:** Physical AI Team
