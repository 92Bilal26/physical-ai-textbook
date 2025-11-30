# Phase 4: Monitoring & Metrics Setup Guide

## Prometheus Metrics Endpoint

The application now exposes metrics at `GET /metrics` for Prometheus scraping.

### Available Metrics

#### Query Metrics
- `chatbot_query_latency_seconds` - Histogram with buckets: [0.1s, 0.25s, 0.5s, 1s, 2.5s, 5s, 10s]
- `chatbot_queries_total` - Counter by endpoint and status (success/error/rate_limited)

#### Cache Metrics
- `chatbot_cache_hits_total` - Counter by cache type (query/embedding)
- `chatbot_cache_misses_total` - Counter by cache type (query/embedding)
- Derive **cache hit rate** = hits / (hits + misses)

#### OpenAI API Metrics
- `chatbot_openai_api_calls_total` - Counter by operation (chat_completion/embedding)
- `chatbot_openai_rate_limit_errors_total` - Counter of rate limit errors

#### Qdrant Search Metrics
- `chatbot_qdrant_searches_total` - Counter of vector searches
- `chatbot_qdrant_search_latency_seconds` - Histogram with buckets: [0.01s, 0.025s, 0.05s, 0.1s, 0.25s, 0.5s]

#### Rate Limiting Metrics
- `chatbot_rate_limit_hits_total` - Counter by session_id

#### Database Metrics
- `chatbot_database_query_latency_seconds` - Histogram for DB performance

#### Error Metrics
- `chatbot_errors_total` - Counter by error type

#### Session Metrics
- `chatbot_active_sessions` - Gauge for active sessions

#### Cache/Redis Metrics
- `chatbot_redis_connection_errors_total` - Counter of connection errors

## Prometheus Configuration

### docker-compose Addition

```yaml
prometheus:
  image: prom/prometheus:latest
  ports:
    - "9090:9090"
  volumes:
    - ./prometheus.yml:/etc/prometheus/prometheus.yml
    - prometheus_data:/prometheus
  command:
    - '--config.file=/etc/prometheus/prometheus.yml'
    - '--storage.tsdb.path=/prometheus'
```

### prometheus.yml

```yaml
global:
  scrape_interval: 15s
  evaluation_interval: 15s

scrape_configs:
  - job_name: 'rag-chatbot'
    static_configs:
      - targets: ['localhost:8000']
    metrics_path: '/metrics'
    scrape_interval: 10s
```

## Grafana Setup

### Key Dashboards to Create

#### 1. Query Performance Dashboard
```
- Query Latency (p50, p95, p99)
- Queries per second
- Error rate by type
- Rate limit hits over time
```

#### 2. Cache Effectiveness Dashboard
```
- Cache hit rate (%)
- Hit/miss ratio trend
- Cache savings ($ saved by not hitting OpenAI)
- Embedding cache hits vs query cache hits
```

#### 3. Resource Utilization Dashboard
```
- Active sessions
- Database connection pool usage
- Redis memory usage (if available)
- OpenAI API usage
```

#### 4. Error Monitoring Dashboard
```
- Error rate
- Error types breakdown
- OpenAI rate limit errors
- Redis connection errors
```

## Key Alerts to Set

### P1 (Critical)
```
- Query latency p99 > 5s for 2 minutes
- Error rate > 5%
- Cache unavailable for 1 minute
- Database connection pool exhausted
```

### P2 (Warning)
```
- Cache hit rate < 40%
- OpenAI API latency > 2s
- Qdrant search latency > 500ms
- Active sessions > 1000
```

## Metrics Usage Examples

### Query Performance Analysis
```promql
# Average query latency
rate(chatbot_query_latency_seconds_sum[5m]) / rate(chatbot_query_latency_seconds_count[5m])

# P95 query latency
histogram_quantile(0.95, rate(chatbot_query_latency_seconds_bucket[5m]))

# Error rate
rate(chatbot_queries_total{status="error"}[5m]) / rate(chatbot_queries_total[5m])
```

### Cache Efficiency
```promql
# Cache hit rate
rate(chatbot_cache_hits_total[5m]) / (rate(chatbot_cache_hits_total[5m]) + rate(chatbot_cache_misses_total[5m]))

# Queries served from cache vs API
rate(chatbot_cache_hits_total{cache_type="query"}[5m])
```

### OpenAI Cost Analysis
```promql
# Total embedding API calls (cost = count * 0.00002)
rate(chatbot_openai_api_calls_total{operation="embedding"}[1h])

# Total chat completion calls (cost varies by model)
rate(chatbot_openai_api_calls_total{operation="chat_completion"}[1h])
```

## Data Retention & Cleanup

### Prometheus Storage
- Default retention: 15 days
- Storage format: TSDB (time-series database)
- Adjust with `--storage.tsdb.retention.time=30d` if needed

### Application Metrics Cleanup
- Rate limit buckets cleaned daily (max 24h old)
- Expired session tracking removed periodically
- Database TTL cleanup: runs nightly for expired data

## Integration with Alerting

### For Slack Integration (example)
```yaml
# In alertmanager config
receivers:
  - name: 'slack'
    slack_configs:
      - api_url: 'YOUR_SLACK_WEBHOOK'
        channel: '#alerts'
        title: 'RAG Chatbot Alert'
        text: '{{ .GroupLabels.alertname }}'
```

## Performance Benchmarks (Phase 4 Target)

After implementing caching and optimization:

```
Metric                  | Without Optimization | After Phase 4 | Target SLA
Query P95 Latency      | 800ms               | 400ms         | < 500ms
Cache Hit Rate         | 0%                  | 60%           | > 50%
OpenAI API Calls/day   | 10,000              | 4,000         | 50% reduction
Error Rate             | 2%                  | < 0.5%        | < 1%
Active Sessions        | 500                 | 500           | Stable
```

## Testing Metrics

### Load Test Query
```bash
# Query /metrics endpoint
curl http://localhost:8000/metrics | grep chatbot_

# Filter by specific metric
curl http://localhost:8000/metrics | grep chatbot_queries_total
```

### Manual Verification
```python
from prometheus_client import REGISTRY

# Get all metrics
for metric in REGISTRY.collect():
    print(f"Metric: {metric.name}")
    for sample in metric.samples:
        print(f"  {sample.name}: {sample.value}")
```

## Next Steps

1. **Phase 5**: Frontend implementation (metrics remain active)
2. **Phase 6**: Add comprehensive testing (mock metrics for tests)
3. **Phase 7**: Docker deployment (Prometheus in docker-compose)
4. **Phase 8**: Documentation includes Grafana dashboard templates
