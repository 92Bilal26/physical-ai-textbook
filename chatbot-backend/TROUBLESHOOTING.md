# Troubleshooting Guide

Common issues and solutions for the RAG Chatbot.

## Installation Issues

### Issue: "ModuleNotFoundError: No module named 'qdrant_client'"

**Solution:**
```bash
pip install -r requirements.txt
pip install --upgrade qdrant-client
```

### Issue: "psycopg2 compilation failed"

**Solution:**
```bash
# Install system dependencies
# Ubuntu/Debian
sudo apt-get install postgresql-client libpq-dev

# macOS
brew install postgresql

# Windows
# Download Visual C++ Build Tools

# Then reinstall
pip install psycopg2-binary
```

### Issue: Python version mismatch

**Solution:**
```bash
python --version  # Should be 3.11+
python3.11 -m venv venv
source venv/bin/activate
```

## Runtime Issues

### Issue: "Connection refused" when connecting to database

**Solution:**
```bash
# Check if PostgreSQL is running
sudo systemctl status postgresql  # Linux
brew services list | grep postgres  # macOS

# Start PostgreSQL
sudo systemctl start postgresql  # Linux
brew services start postgresql  # macOS

# Verify connection
psql postgresql://user:pass@localhost:5432/chatbot
```

### Issue: "Failed to connect to Qdrant"

**Solution:**
```bash
# Check Qdrant URL and API key
echo $QDRANT_URL
echo $QDRANT_API_KEY

# Test connection
curl -X GET "$QDRANT_URL/health" \
  -H "api-key: $QDRANT_API_KEY"

# If using local Qdrant
docker ps | grep qdrant
docker logs qdrant-service
```

### Issue: "Gemini API authentication failed"

**Solution:**
```bash
# Verify API key
echo $GEMINI_API_KEY

# Test API key
curl -X POST "https://generativelanguage.googleapis.com/v1/models/gemini-pro:generateContent?key=$GEMINI_API_KEY" \
  -H "Content-Type: application/json" \
  -d '{"contents":[{"parts":[{"text":"test"}]}]}'

# Regenerate key in Google Cloud Console
# https://console.cloud.google.com/
```

## API Issues

### Issue: "401 Unauthorized - Invalid Session"

**Solution:**
```python
# Client-side: Create new session
session_id = str(uuid.uuid4())
localStorage.setItem('rag_session_id', session_id)

# Server-side: Check session expiry
SELECT expires_at FROM user_sessions WHERE session_id = 'xxx'
```

### Issue: "422 Unprocessable Entity - Validation Error"

**Solution:**
```bash
# Check request format
curl -X POST http://localhost:8000/api/v1/chat/query \
  -H "Content-Type: application/json" \
  -d '{
    "question": "What is robotics?",
    "session_id": "550e8400-e29b-41d4-a716-446655440000"
  }'

# Question must be 1-500 characters
# Session ID must be UUID format
# All required fields must be present
```

### Issue: "429 Too Many Requests - Rate Limited"

**Solution:**
```python
# Check rate limit headers
X-RateLimit-Remaining: 0
Retry-After: 30

# Wait before retrying
import time
time.sleep(30)

# Or increase rate limit
# In rate_limit_service.py:
DEFAULT_REQUESTS_PER_MINUTE = 120  # Increase from 60
```

### Issue: "500 Internal Server Error"

**Solution:**
```bash
# Check server logs
docker logs chatbot-backend

# Check database connection
SELECT 1;  # Should return 1

# Check Redis connection
redis-cli ping  # Should return PONG

# Check Qdrant/Gemini availability
curl http://localhost:8000/api/v1/health
```

## Performance Issues

### Issue: "Response time is very slow (>2 seconds)"

**Solution:**
```bash
# Check cache hit rate
# Monitor: chatbot_cache_hits_total / chatbot_cache_misses_total

# Check database query times
EXPLAIN ANALYZE SELECT * FROM messages WHERE conversation_id = 'xxx';

# Monitor Gemini API latency
# Check query complexity
# Upgrade server resources
```

### Issue: "High memory usage"

**Solution:**
```bash
# Check memory usage
docker stats chatbot-backend

# Clear old cache entries
redis-cli FLUSHDB  # WARNING: Clears all cache

# Reduce cache TTL
# In cache_service.py:
self.cache_ttl = 1800  # Reduce from 3600

# Monitor memory in CloudWatch/Prometheus
```

### Issue: "Database connection pool exhausted"

**Solution:**
```bash
# Check connection count
SELECT count(*) FROM pg_stat_activity WHERE datname = 'chatbot';

# Increase pool size
# In db.py:
pool_size=20  # Increase from 10

# Kill idle connections
SELECT pg_terminate_backend(pid)
FROM pg_stat_activity
WHERE state = 'idle';
```

## Deployment Issues

### Issue: "Container won't start"

**Solution:**
```bash
# Check logs
docker logs container_id

# Check environment variables
docker inspect container_id | grep Env

# Verify port is available
lsof -i :8000
kill -9 <PID>

# Rebuild image
docker build --no-cache -t rag-chatbot:latest .
```

### Issue: "Kubernetes pod keeps restarting"

**Solution:**
```bash
# Check pod status
kubectl describe pod pod_name

# Check logs
kubectl logs pod_name --previous  # Previous crashed instance
kubectl logs pod_name  # Current instance

# Check liveness/readiness probes
kubectl get pod pod_name -o yaml | grep -A 5 livenessProbe

# Check resource limits
kubectl top node
kubectl top pod pod_name
```

### Issue: "DNS resolution fails in container"

**Solution:**
```yaml
# Add to docker-compose.yml
services:
  backend:
    dns:
      - 8.8.8.8
      - 8.8.4.4

# Or test DNS inside container
docker exec container_id nslookup google.com
```

## Data Issues

### Issue: "No search results found"

**Solution:**
```bash
# Check if content is indexed
curl -X GET "$QDRANT_URL/collections/textbook_content" \
  -H "api-key: $QDRANT_API_KEY"

# Check embedding generation
curl -X POST http://localhost:8000/api/v1/chat/query \
  -H "Content-Type: application/json" \
  -d '{"question": "test", "session_id": "xxx"}'

# Rebuild vector index
# Contact admin to re-index content
```

### Issue: "Outdated search results"

**Solution:**
```bash
# Clear cache
redis-cli FLUSHDB

# Or clear specific session cache
redis-cli KEYS "query:*session_id*" | xargs redis-cli DEL

# Refresh embeddings
# Admin: Run content ingestion script
python scripts/ingest_content.py
```

## Security Issues

### Issue: "Unauthorized API access"

**Solution:**
```bash
# Check CORS settings
# In main.py:
allow_origins=["https://textbook.example.com"]

# Check API key exposure
grep -r "GEMINI_API_KEY" .
grep -r "QDRANT_API_KEY" .
# Should only appear in .env (which is .gitignored)

# Rotate keys immediately
```

### Issue: "Suspicious activity detected"

**Solution:**
```bash
# Check rate limit logs
docker logs chatbot-backend | grep "Rate limited"

# Block IP if needed
# Add to firewall/WAF rules

# Review webhook activity
SELECT * FROM webhooks WHERE status = 'failed';

# Check authentication logs
SELECT * FROM audit_log WHERE action = 'failed_auth';
```

## Monitoring Issues

### Issue: "Metrics not appearing"

**Solution:**
```bash
# Check if metrics endpoint is available
curl http://localhost:8000/metrics

# Check Prometheus scrape config
cat /etc/prometheus/prometheus.yml

# Verify service is being scraped
curl http://localhost:9090/api/v1/targets
```

### Issue: "Alerts not triggering"

**Solution:**
```bash
# Check alert rules
curl http://localhost:9090/api/v1/rules

# Verify alert configuration
cat /etc/prometheus/rules.yml

# Test alert manually
curl -X POST http://localhost:9093/api/v1/alerts \
  -H "Content-Type: application/json" \
  -d '[{"labels": {"alertname": "HighErrorRate"}}]'
```

## Network Issues

### Issue: "CORS errors in browser"

**Solution:**
```bash
# Check CORS headers
curl -I -X OPTIONS http://localhost:8000/api/v1/chat/query \
  -H "Origin: https://textbook.example.com"

# Should show:
# Access-Control-Allow-Origin: https://textbook.example.com

# If not, update CORS settings
# In main.py, update allowed_origins
```

### Issue: "Timeout errors"

**Solution:**
```bash
# Increase timeout in FastAPI
# In main.py:
uvicorn.run(app, timeout_keep_alive=30)

# Increase client timeout
# In JavaScript:
fetch(url, { signal: AbortSignal.timeout(30000) })

# Check network latency
ping api.example.com

# Check firewall rules
ufw status
sudo ufw allow 8000
```

## Getting Help

1. Check logs first
2. Verify environment variables
3. Test individual services
4. Check [Deployment Guide](./DEPLOYMENT.md)
5. Review [API Documentation](./API_DOCUMENTATION.md)
6. Submit issue on GitHub with:
   - Error logs
   - Environment (OS, Python version)
   - Steps to reproduce
   - Expected vs actual behavior

---

**Last Updated:** January 2024
