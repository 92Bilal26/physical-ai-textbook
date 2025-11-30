# Phase 4: Database Optimization Guide

## Indexes Implemented

### UserSession Table
- `ix_user_sessions_session_id` - Primary lookup for session validation
- `ix_user_sessions_user_id` - Lookup for authenticated users
- `ix_user_sessions_expires_at` - TTL cleanup queries

### Conversation Table
- `ix_conversations_user_session_id` - FK lookup for conversation retrieval
- `ix_conversations_created_at` - Chronological queries
- `ix_conversations_expires_at` - TTL cleanup queries

### Message Table
- `ix_messages_conversation_id` - FK lookup for message retrieval
- `ix_messages_timestamp` - Chronological queries
- `ix_messages_expires_at` - TTL cleanup queries

## Connection Pooling Configuration

### Settings
```python
DATABASE_POOL_SIZE: 10          # Default pool size
DATABASE_MAX_OVERFLOW: 20       # Additional connections when pool exhausted
pool_recycle: 3600              # Recycle connections after 1 hour
pool_pre_ping: True             # Test connections before using them
```

### Benefits
- **Pool Recycling**: Handles database idle timeouts gracefully
- **Pre-ping**: Eliminates stale connection errors
- **Overflow**: Handles traffic spikes without connection exhaustion

## Query Optimization Tips

### 1. N+1 Query Prevention
```python
# Bad: N+1 queries
conversations = await db.query(Conversation).filter(...).all()
for conv in conversations:
    messages = await db.query(Message).filter(Message.conversation_id == conv.id).all()

# Good: Joined load
from sqlalchemy.orm import selectinload
conversations = await db.query(Conversation).filter(...).options(selectinload(Conversation.messages)).all()
```

### 2. Batch Operations
```python
# Bulk insert citations instead of one-by-one
citations = [Citation(...), Citation(...), ...]
db.add_all(citations)
await db.flush()
```

### 3. Pagination
```python
# For history endpoints
LIMIT = 50
offset = (page - 1) * LIMIT
messages = await db.query(Message).filter(...).offset(offset).limit(LIMIT).all()
```

## Performance Monitoring

### Key Metrics to Track
1. **Query Latency**: p50, p95, p99 (see Monitoring & Metrics)
2. **Connection Pool Usage**: Monitor from application logs
3. **Cache Hit Rate**: Embeddings (24h), Query results (1h)

### TTL Cleanup

```python
# Run periodically (e.g., daily at 2 AM)
async def cleanup_expired_data():
    """Delete expired data"""
    from datetime import datetime

    # Clean up expired sessions
    await db.query(UserSession).filter(
        UserSession.expires_at <= datetime.utcnow()
    ).delete()

    # Clean up expired messages
    await db.query(Message).filter(
        Message.expires_at <= datetime.utcnow()
    ).delete()

    await db.commit()
    logger.info("Cleaned up expired data")
```

## Redis Cache Strategy

### Query Results
- **Key**: MD5(question + session_id)
- **TTL**: 1 hour
- **Hit Benefit**: Avoid entire RAG pipeline (500ms+ saved)

### Embeddings
- **Key**: MD5(text)
- **TTL**: 24 hours
- **Hit Benefit**: Avoid OpenAI API call ($0.02 per 1M tokens saved)

## Load Testing Benchmarks

After implementing Phase 4, expected performance:

```
Sequential requests (1 req/sec):
- Without cache: ~800ms per query
- With query cache hit: ~50ms (96% faster)
- With embedding cache hit: ~200ms (75% faster)

Concurrent requests (10 simultaneous):
- Connection pool handles without degradation
- Rate limiting prevents overload (429 responses)
- Cache reduces OpenAI API usage by ~60%
```

## Monitoring Alerts

Set up alerts for:
1. **High Database Latency** (> 500ms for p95)
2. **Connection Pool Exhaustion** (> 95% utilization)
3. **Cache Failures** (Redis unavailable)
4. **Rate Limit Hits** (> 5% of requests)

## Next Steps (Phase 5-6)

After Phase 4 optimization is validated:
1. **Phase 5**: Frontend ChatWidget implementation
2. **Phase 6**: Comprehensive testing and quality metrics
3. Implement Prometheus metrics (detailed in Phase 6)
