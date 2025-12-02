# Common Errors & Solutions

This guide covers the most frequent Render.com deployment errors and their solutions based on real-world experience.

---

## Error 1: SQLAlchemy - "Can't load plugin: sqlalchemy.dialects:driver"

**When It Happens**: During `alembic upgrade head` in deployment logs

**Root Cause**: DATABASE_URL environment variable is missing, malformed, or uses wrong name

**Solution**:

1. **Check Variable Names**:
   - Your code might expect `DATABASE_URL`
   - Old code might expect `NEON_DATABASE_URL`
   - Check your migrations/env.py or src/config.py for the variable name

2. **Verify Connection String**:
   ```
   postgresql://user:password@host/dbname    ✅ CORRECT
   postgresql://user@host/dbname              ✅ OK (no password)
   postgres://user:password@host/dbname       ❌ WRONG (postgresql, not postgres)
   ```

3. **Check Environment Variables in Render**:
   - Go to rag-chatbot-backend → Environment tab
   - Search for DATABASE_URL or NEON_DATABASE_URL
   - If missing, add it manually
   - If wrong, click Edit and fix it

4. **Verify Database Exists**:
   - Check your databases list in Render dashboard
   - Make sure database is "Available" status
   - Copy the Internal Database URL
   - Add to environment variables

**Example Fix**:
```
If using migrations/env.py that checks for NEON_DATABASE_URL:
  sqlalchemy_url = os.getenv('NEON_DATABASE_URL') or ...

Either:
1. Rename env var to NEON_DATABASE_URL, OR
2. Change code to accept both: DATABASE_URL or NEON_DATABASE_URL
```

---

## Error 2: Pydantic - "Input should be a valid string [type=string_type]"

**When It Happens**: During app startup, Pydantic loads environment variables

**Root Cause**: Environment variable type mismatch (e.g., `allowed_origins` field expects string but gets list)

**Common Culprit**: `allowed_origins` field in Pydantic Settings

**Solution**:

1. **Check Field Type**:
   ```python
   # ❌ WRONG - Pydantic tries to JSON-parse the string
   allowed_origins: list = "https://example.com,https://other.com"

   # ✅ CORRECT - Accept string, convert in validator
   allowed_origins: Union[str, list] = "https://example.com,https://other.com"

   @field_validator("allowed_origins", mode="before")
   def parse_origins(cls, v):
       if isinstance(v, str):
           return [o.strip() for o in v.split(",")]
       return v if isinstance(v, list) else []
   ```

2. **Default Value Format**:
   - If your Pydantic field has a default, make it match the expected format
   - For `allowed_origins`, make the default a comma-separated string
   - The validator converts it to a list

3. **Environment Variable Format in Render**:
   - For list-like values, use **comma-separated strings**:
     ```
     https://example.com,https://other.com   ✅ CORRECT
     ["https://example.com"]                  ❌ WRONG (don't use JSON syntax)
     ```

**Example Output**:
```python
# In config.py
allowed_origins: Union[str, list] = "http://localhost:3000,https://example.com"

# Rendered in Render env var:
ALLOWED_ORIGINS=https://example.com,https://other.com

# Loaded as:
['https://example.com', 'https://other.com']
```

---

## Error 3: CORS - "Access to fetch... blocked by CORS policy"

**When It Happens**: Frontend makes API request, browser blocks it

**Error Message**:
```
Access to fetch at 'https://backend-XXXX.onrender.com/api/query'
from origin 'https://frontend.github.io'
has been blocked by CORS policy
```

**Common Causes**:
1. Middleware order - rate limiting before CORS
2. allowed_origins doesn't include frontend URL
3. Frontend using wrong backend URL
4. Preflight OPTIONS request blocked

**Solution**:

1. **Check Middleware Order** (Most Common):
   ```python
   # ❌ WRONG - rate limiting first, CORS second
   app.add_middleware(RateLimitMiddleware)
   app.add_middleware(CORSMiddleware, ...)

   # ✅ CORRECT - CORS first, then rate limiting
   app.add_middleware(CORSMiddleware, ...)
   app.add_middleware(RateLimitMiddleware)
   ```

2. **Verify Frontend URL in ALLOWED_ORIGINS**:
   - Check Render Environment tab
   - Look for ALLOWED_ORIGINS
   - Make sure frontend URL is included
   - Example: `https://92bilal26.github.io`

3. **Check Backend URL in Frontend**:
   - Verify frontend is calling `https://backend-XXXX.onrender.com`
   - Not `https://backend.onrender.com` (missing random suffix)
   - Copy exact URL from Render dashboard

4. **Test Preflight Request**:
   ```bash
   # Test OPTIONS request (preflight)
   curl -X OPTIONS https://backend-XXXX.onrender.com/api/query \
        -H "Origin: https://frontend.github.io"

   # Should include headers like:
   # access-control-allow-origin: https://frontend.github.io
   # access-control-allow-methods: GET, POST, PUT, DELETE, ...
   ```

---

## Error 4: Docker Build - "failed to build image"

**When It Happens**: During deployment build phase

**Common Causes**:
1. Dockerfile path incorrect
2. Build context wrong
3. Requirements file not found
4. Dockerfile syntax error

**Solution**:

1. **Check Dockerfile Path**:
   - In render.yaml: `dockerfilePath: ./chatbot-backend/Dockerfile`
   - File must exist at exactly that path
   - Relative to repository root

2. **Check Build Context**:
   - In render.yaml or Render UI: Docker Build Context Directory
   - If set to `chatbot-backend`, then dockerfilePath should be `Dockerfile`
   - If set to `.`, then dockerfilePath should be `chatbot-backend/Dockerfile`

3. **Verify Requirements.txt**:
   - Must exist in same directory as Dockerfile
   - Dockerfile line: `COPY requirements.txt .`
   - Check file isn't corrupted (invalid package names)

4. **Check Dockerfile Syntax**:
   ```dockerfile
   # Common errors:
   FROM python:3.11-slim          # ✅ OK
   RUN pip install -r requirements.txt  # ✅ OK
   COPY . .                       # ✅ OK (AFTER pip install for caching)

   # ❌ Missing command
   RUN
   ```

---

## Error 5: Middleware Order - "No CORS headers in response"

**When It Happens**: OPTIONS preflight request doesn't return CORS headers

**Root Cause**: CORS middleware added after rate limiting or other middleware

**Solution**: See Error 3 (CORS) above - verify middleware order

---

## Error 6: Health Check Timeout

**When It Happens**: Service is marked "unhealthy" after deployment

**Error**: Render gives up trying to connect to health endpoint

**Root Cause**:
1. Health check path is wrong
2. Endpoint doesn't return 200 OK
3. App didn't start properly

**Solution**:

1. **Verify Health Endpoint Exists**:
   ```python
   @app.get("/api/v1/health")
   async def health_check():
       return {"status": "ok"}
   ```

2. **Check Health Path in Configuration**:
   - In render.yaml: `healthCheckPath: /api/v1/health`
   - Must exactly match your endpoint
   - No trailing slash issues

3. **Test Locally**:
   ```bash
   curl http://localhost:8000/api/v1/health
   # Should return 200 with JSON
   ```

4. **Check Deployment Logs**:
   - Render → Logs tab
   - Look for startup errors
   - Check if app is actually starting

---

## Error 7: Database Connection Pool Exhaustion (Free Tier)

**When It Happens**: "too many connections" error after 5-10 requests

**Root Cause**: Free tier PostgreSQL limits connections to ~10

**Solution**:

1. **Use Connection Pooling**:
   ```python
   from sqlalchemy import create_engine

   engine = create_engine(
       DATABASE_URL,
       poolclass=StaticPool,        # For SQLite
       pool_pre_ping=True,           # Test connections before use
       pool_recycle=3600,            # Recycle after 1 hour
       max_overflow=5,               # Max overflow connections
       pool_size=5                   # Base pool size
   )
   ```

2. **Use Render PostgreSQL Properly**:
   - Free tier: 10 connections max
   - Don't create new engine per request
   - Reuse single engine instance
   - Close connections properly

3. **For Production**:
   - Upgrade to Starter tier (20 connections)
   - Or use external connection pooler (PgBouncer)

---

## Error 8: Alembic - "ImportError: cannot import name 'X'"

**When It Happens**: During `alembic upgrade head`

**Root Cause**: Alembic can't find your models because imports fail

**Solution**:

1. **Check Python Path**:
   - `migrations/env.py` needs to import your models
   - Make sure `sys.path` includes your src directory:
   ```python
   import sys
   sys.path.insert(0, '/app')  # or wherever src is
   ```

2. **Verify Import Statements**:
   - Check `from myapp.models import *` in env.py
   - Make sure models.py exists and has no import errors

3. **Run Locally First**:
   ```bash
   alembic upgrade head
   # If it works locally, should work on Render
   ```

---

## Quick Diagnosis Checklist

When deployment fails:

- [ ] Check Render → Logs tab (most informative)
- [ ] Look for "Can't load plugin" → DATABASE_URL issue
- [ ] Look for "Input should be valid" → Pydantic type issue
- [ ] Look for "CORS policy blocked" → Middleware order issue
- [ ] Look for "failed to build" → Dockerfile/path issue
- [ ] Look for "health check" → Endpoint issue
- [ ] Look for "too many connections" → Connection pooling issue
- [ ] Look for "ImportError" → Alembic/imports issue

If still stuck:
1. Read the full error message (not just first line)
2. Check the Render documentation for that error
3. Search the error online with "Render.com"
4. Review the example configurations in this skill
