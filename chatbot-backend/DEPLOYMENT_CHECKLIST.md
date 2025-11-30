# Deployment Checklist

Pre-deployment verification checklist for production deployment.

## Pre-Deployment

- [ ] All tests passing
  ```bash
  pytest --cov=src tests/
  ```

- [ ] Code review completed
  - [ ] Security review
  - [ ] Performance review
  - [ ] Architecture review

- [ ] Dependencies updated
  ```bash
  pip list --outdated
  npm outdated
  ```

- [ ] Environment variables configured
  - [ ] Database URL set
  - [ ] Redis URL set
  - [ ] Qdrant credentials set
  - [ ] Gemini API key set
  - [ ] Secret key generated
  - [ ] Allowed origins set

- [ ] Secrets secured
  - [ ] .env not committed
  - [ ] API keys not in logs
  - [ ] Credentials in secrets manager
  - [ ] Database password changed

- [ ] Database ready
  - [ ] Migrations tested locally
  - [ ] Backup strategy in place
  - [ ] Connection string verified

- [ ] External services verified
  - [ ] Qdrant cluster accessible
  - [ ] Gemini API key valid
  - [ ] Redis accessible
  - [ ] Database accessible

## Deployment Steps

### 1. Pre-deployment
- [ ] Create backup of production database
  ```bash
  pg_dump production_db > backup_$(date +%Y%m%d).sql
  ```

- [ ] Notify team of deployment
- [ ] Enable maintenance mode (optional)
- [ ] Set deployment start time

### 2. Deployment
- [ ] Build Docker image
  ```bash
  docker build -t rag-chatbot:latest .
  ```

- [ ] Push to registry
  ```bash
  docker push registry.example.com/rag-chatbot:latest
  ```

- [ ] Deploy to production
  ```bash
  docker-compose -f docker-compose.prod.yml pull
  docker-compose -f docker-compose.prod.yml up -d
  ```

- [ ] Run database migrations
  ```bash
  docker exec rag-chatbot-backend alembic upgrade head
  ```

- [ ] Verify services are running
  ```bash
  docker ps | grep rag-chatbot
  ```

### 3. Health Checks
- [ ] API health check passes
  ```bash
  curl http://localhost:8000/api/v1/health
  ```

- [ ] Database connection works
  ```bash
  curl -X POST http://localhost:8000/api/v1/chat/query \
    -H "Content-Type: application/json" \
    -d '{"question": "test", "session_id": "test-uuid"}'
  ```

- [ ] External services accessible
  - [ ] Qdrant reachable
  - [ ] Gemini API responding
  - [ ] Redis connected

- [ ] Logs show no errors
  ```bash
  docker logs rag-chatbot-backend | grep ERROR
  ```

### 4. Smoke Testing
- [ ] General query works
- [ ] Selection query works
- [ ] Rate limiting works
- [ ] Caching works
- [ ] Error handling works
- [ ] Webhooks trigger

### 5. Monitoring Setup
- [ ] Prometheus collecting metrics
  ```bash
  curl http://localhost:9090/api/v1/query?query=up
  ```

- [ ] Grafana dashboards loaded
- [ ] Alerts configured
- [ ] Log aggregation active
- [ ] APM instrumentation working

### 6. Performance Validation
- [ ] Response time < 2 seconds
  ```bash
  curl -w "@curl-format.txt" http://localhost:8000/api/v1/health
  ```

- [ ] Memory usage stable
- [ ] CPU usage < 80%
- [ ] Database queries optimized

### 7. Security Verification
- [ ] HTTPS/TLS enabled
- [ ] API keys not exposed
- [ ] CORS properly configured
- [ ] Rate limiting active
- [ ] Input validation working
- [ ] SQL injection protection
- [ ] XSS protection enabled

### 8. User Testing
- [ ] End-to-end test passed
- [ ] UI loads correctly
- [ ] ChatWidget renders
- [ ] Message submission works
- [ ] Results display correctly
- [ ] Citations show properly

### 9. Documentation
- [ ] API docs updated
- [ ] Deployment guide updated
- [ ] Troubleshooting guide updated
- [ ] Version bump completed
- [ ] CHANGELOG updated

### 10. Post-Deployment
- [ ] Monitor error rate (< 1%)
- [ ] Monitor response time (p95 < 2s)
- [ ] Monitor uptime (> 99.9%)
- [ ] Check user feedback
- [ ] Review analytics
- [ ] Validate backups work

## Rollback Plan

If issues occur:

1. **Quick Rollback (< 5 minutes)**
   ```bash
   docker-compose -f docker-compose.prod.yml down
   docker-compose -f docker-compose.prod.yml up -d
   # This reverts to previous image if already pulled
   ```

2. **Container Rollback**
   ```bash
   docker service rollback rag-chatbot_backend
   # Kubernetes: kubectl rollout undo deployment/rag-chatbot
   ```

3. **Database Rollback**
   ```bash
   alembic downgrade -1  # One migration back
   # Or full restore from backup
   psql production_db < backup_20240101.sql
   ```

4. **Full Rollback**
   - Stop new deployment
   - Restore from backup
   - Revert to previous version
   - Notify stakeholders

## Post-Deployment Validation

### Day 1
- [ ] All services running
- [ ] No error spikes
- [ ] Performance metrics normal
- [ ] User queries working

### Week 1
- [ ] Error rate stable
- [ ] Response times consistent
- [ ] Cache hit rates good (> 30%)
- [ ] Database performing well

### Month 1
- [ ] Uptime > 99.9%
- [ ] No critical bugs
- [ ] Performance optimized
- [ ] Analytics collected

## Sign-off

- **Deployed by:** _________________ (Name)
- **Reviewed by:** _________________ (Name)
- **Date:** _________________
- **Time:** _________________
- **Result:** [ ] Success [ ] Rollback

**Notes:** _________________________________________________________________

---

**Last Updated:** January 2024
