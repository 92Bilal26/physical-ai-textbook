# Deployment Guide

Production deployment guide for the RAG Chatbot.

## Prerequisites

- Docker & Docker Compose
- Python 3.11+
- Node.js 18+ (for ChatWidget)
- PostgreSQL 13+
- Redis 7+
- Qdrant Cloud account
- Google Gemini API key

## Quick Start

### 1. Local Development

```bash
# Setup
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
pip install -r requirements.txt

# Configure
cp .env.example .env
# Edit .env with your credentials

# Run
docker-compose up -d
alembic upgrade head
python -m uvicorn src.main:app --reload

# Access API
open http://localhost:8000
```

### 2. Docker Deployment

```bash
# Build image
docker build -t rag-chatbot:latest .

# Run container
docker run -p 8000:8000 \
  -e GEMINI_API_KEY=$GEMINI_API_KEY \
  -e QDRANT_URL=$QDRANT_URL \
  -e QDRANT_API_KEY=$QDRANT_API_KEY \
  -e DATABASE_URL=$DATABASE_URL \
  rag-chatbot:latest

# Access
curl http://localhost:8000/api/v1/health
```

### 3. Production Deployment

#### Option A: Docker Swarm

```bash
# Initialize swarm
docker swarm init

# Deploy stack
docker stack deploy -c docker-compose.prod.yml rag-chatbot

# Verify
docker service ls
docker service logs rag-chatbot_backend
```

#### Option B: Kubernetes

```bash
# Create namespace
kubectl create namespace rag-chatbot

# Create secrets
kubectl create secret generic rag-credentials \
  --from-literal=gemini_api_key=$GEMINI_API_KEY \
  --from-literal=qdrant_api_key=$QDRANT_API_KEY \
  -n rag-chatbot

# Deploy
kubectl apply -f k8s/ -n rag-chatbot

# Verify
kubectl get pods -n rag-chatbot
kubectl logs -f deployment/rag-chatbot -n rag-chatbot
```

#### Option C: Cloud (AWS, GCP, Azure)

**AWS ECS:**
```bash
# Create task definition
aws ecs register-task-definition --cli-input-json file://task-definition.json

# Create service
aws ecs create-service \
  --cluster rag-chatbot-cluster \
  --service-name rag-chatbot-service \
  --task-definition rag-chatbot:1 \
  --desired-count 3
```

**Google Cloud Run:**
```bash
# Build image
gcloud builds submit --tag gcr.io/$PROJECT_ID/rag-chatbot

# Deploy
gcloud run deploy rag-chatbot \
  --image gcr.io/$PROJECT_ID/rag-chatbot \
  --platform managed \
  --region us-central1 \
  --set-env-vars GEMINI_API_KEY=$GEMINI_API_KEY
```

## Configuration

### Environment Variables

```env
# Database
DATABASE_URL=postgresql://user:password@host:5432/chatbot
REDIS_URL=redis://localhost:6379

# Qdrant Vector DB
QDRANT_URL=https://your-instance.qdrant.io
QDRANT_API_KEY=your-api-key
QDRANT_COLLECTION_NAME=textbook_content

# Google Gemini
GEMINI_API_KEY=your-gemini-key
GEMINI_MODEL=gemini-pro

# Server
HOST=0.0.0.0
PORT=8000
ENVIRONMENT=production
DEBUG=false
LOG_LEVEL=INFO

# Session & Security
SESSION_EXPIRY_DAYS=30
SECRET_KEY=your-secret-key
ALLOWED_ORIGINS=https://textbook.example.com

# Features
ENABLE_HALLUCINATION_CHECK=true
ENABLE_RATE_LIMIT_GRACEFUL_DEGRADATION=true
```

### Security

**1. HTTPS/TLS**
```nginx
server {
    listen 443 ssl http2;
    ssl_certificate /etc/ssl/certs/cert.pem;
    ssl_certificate_key /etc/ssl/private/key.pem;

    location / {
        proxy_pass http://localhost:8000;
        proxy_set_header X-Forwarded-Proto https;
    }
}
```

**2. API Keys**
- Store in environment variables or secrets manager
- Rotate quarterly
- Never commit to version control

**3. CORS**
```python
app.add_middleware(
    CORSMiddleware,
    allow_origins=settings.allowed_origins,
    allow_methods=["POST", "GET"],
    allow_headers=["Content-Type"],
)
```

## Monitoring

### Health Checks

```bash
# Kubernetes liveness probe
curl -f http://localhost:8000/api/v1/health || exit 1

# Docker healthcheck
HEALTHCHECK --interval=30s --timeout=3s --start-period=5s --retries=3 \
  CMD curl -f http://localhost:8000/api/v1/health || exit 1
```

### Logging

```bash
# View logs
docker logs -f container_id
kubectl logs -f deployment/rag-chatbot

# Log aggregation (ELK Stack)
# Logs sent to Elasticsearch
# Visualize in Kibana dashboard
```

### Metrics

**Prometheus metrics at `/metrics`:**
```
chatbot_queries_total
chatbot_selection_queries_total
chatbot_errors_total
chatbot_cache_hits_total
chatbot_cache_misses_total
chatbot_response_time_seconds
```

### Alerting

**Alert conditions:**
- Error rate > 5% (5-minute window)
- Response time p95 > 2 seconds
- Rate limit violations > 100/minute
- Service down (health check fails)

## Database Management

### Migrations

```bash
# Create migration
alembic revision --autogenerate -m "Add new column"

# Apply migrations
alembic upgrade head

# Rollback
alembic downgrade -1

# History
alembic history
```

### Backup

```bash
# PostgreSQL backup
pg_dump -U user -h host chatbot > backup.sql

# Restore
psql -U user -h host chatbot < backup.sql

# Automated backups (AWS RDS)
aws rds create-db-snapshot \
  --db-instance-identifier chatbot-db \
  --db-snapshot-identifier chatbot-backup-$(date +%Y%m%d)
```

## Performance Optimization

### Caching Strategy
1. Query results cached 1 hour
2. Embeddings cached 24 hours
3. Clear cache on content updates

### Database Optimization
```sql
-- Add indexes
CREATE INDEX idx_session_created ON user_sessions(created_at);
CREATE INDEX idx_conversation_session ON conversations(user_session_id);
CREATE INDEX idx_message_timestamp ON messages(timestamp);
```

### Load Balancing
```yaml
# Nginx upstream
upstream backend {
    least_conn;
    server backend1:8000;
    server backend2:8000;
    server backend3:8000;
}
```

## Troubleshooting

### Common Issues

**1. Service won't start**
```bash
# Check logs
docker logs container_id

# Check port binding
lsof -i :8000

# Check environment variables
env | grep GEMINI
```

**2. Database connection fails**
```bash
# Test connection
psql postgresql://user:pass@host:5432/chatbot

# Check credentials
echo $DATABASE_URL
```

**3. Rate limiting too strict**
- Adjust `DEFAULT_REQUESTS_PER_MINUTE` in rate_limit_service.py
- Increase Redis memory
- Implement custom limits per endpoint

**4. Slow responses**
- Check cache hit rate
- Monitor Qdrant performance
- Analyze Gemini API latency
- Check database query times

See [TROUBLESHOOTING.md](./TROUBLESHOOTING.md) for more.

## Rollback Procedure

```bash
# Docker Swarm
docker service rollback rag-chatbot_backend

# Kubernetes
kubectl rollout undo deployment/rag-chatbot

# Database (if needed)
alembic downgrade -1
```

## Scaling

### Horizontal Scaling
```bash
# Docker Swarm
docker service scale rag-chatbot_backend=5

# Kubernetes
kubectl scale deployment rag-chatbot --replicas=5

# AWS ECS
aws ecs update-service \
  --cluster cluster-name \
  --service rag-chatbot-service \
  --desired-count 10
```

### Load Testing
```bash
# Using Apache Bench
ab -n 1000 -c 10 http://localhost:8000/api/v1/health

# Using wrk
wrk -t12 -c400 -d30s http://localhost:8000/api/v1/health
```

## Maintenance

### Weekly
- Review error logs
- Check disk space
- Verify backups

### Monthly
- Update dependencies
- Rotate API keys
- Review analytics

### Quarterly
- Security audit
- Performance review
- Capacity planning

---

**Last Updated:** January 2024
**Version:** 0.1.0
