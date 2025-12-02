# Workflow 5: Error Debugging - Diagnose and Fix Deployment Issues

This workflow guides you through systematically debugging and fixing common deployment errors.

**Time estimate**: 10-30 minutes depending on error
**Prerequisites**: Completed Workflow 1 (attempted deployment)
**Success indicator**: Service deployed successfully or error resolved

---

## Overview

Deployment errors usually fall into these categories:

1. **Build errors** - Docker build fails (Dockerfile or dependencies)
2. **Startup errors** - App starts but crashes immediately
3. **Runtime errors** - App runs but endpoints fail
4. **Configuration errors** - Environment variables missing or wrong type
5. **Health check failures** - Health endpoint not responding

This workflow helps you find and fix them systematically.

---

## Step 1: Find the Error

### 1.1 Check Service Status

Go to Render Dashboard:

1. Click your service
2. Check status:
   - **Live** ✅ - Service is healthy
   - **Building** 🔨 - Docker build in progress
   - **Deploying** 📦 - Running your app
   - **Unhealthy** ❌ - Service failed or crashing
   - **Failure** ❌ - Build or startup failed

### 1.2 Read the Logs

Click **Logs** tab to see detailed deployment logs.

### 1.3 Search for Errors

Use browser Ctrl+F to search for:

```
ERROR        # Deployment failed
CRITICAL     # Critical error
Traceback    # Python exception
failed       # Operation failed
exit code    # Non-zero exit code (failure)
```

Look for the FIRST error (not necessarily the last line).

**Check**:
- [ ] Identified error message
- [ ] Found relevant error context (5 lines before and after)

---

## Step 2: Categorize the Error

Find which error type you have, then jump to the corresponding section:

### Build Errors (Check Section 3)

Symptoms:
```
failed to build image
ERROR: Could not find a version that satisfies the requirement
RUN pip install -r requirements.txt (failed)
```

### Startup Errors (Check Section 4)

Symptoms:
```
ImportError: cannot import name 'X'
ModuleNotFoundError: No module named 'X'
SyntaxError: invalid syntax
Can't load plugin: sqlalchemy.dialects:driver
```

### Configuration Errors (Check Section 5)

Symptoms:
```
Input should be a valid string [type=string_type]
Required variable X not found
AttributeError: 'NoneType' object has no attribute
```

### CORS Errors (Check Section 6)

Symptoms:
```
Access to fetch blocked by CORS policy
No 'Access-Control-Allow-Origin' header
```

### Health Check Errors (Check Section 7)

Symptoms:
```
Deployment failed. Service unhealthy.
Health check failed after retries
```

---

## Section 3: Build Errors

**When it happens**: During Docker build phase

**Common causes**:
- Dockerfile path incorrect
- requirements.txt has invalid packages
- Python version incompatibility
- Missing build dependencies

### 3.1 Error: "failed to build image"

**Diagnosis**:
1. Check error message for details (e.g., pip failure)
2. Look for line with ERROR in logs
3. Read context around error

**Solution**:

1. **Verify Dockerfile path**:
   - Check render.yaml: `dockerfilePath: ./Dockerfile`
   - Verify file exists at that path
   - File must be at repository root or correct subdirectory

2. **Test locally**:
   ```bash
   docker build -t test .
   # If fails, fix locally first
   ```

3. **Check requirements.txt**:
   - Ensure all package names are correct
   - Run locally: `pip install -r requirements.txt`
   - Look for packages with typos or invalid versions

4. **Check Dockerfile syntax**:
   - Verify COPY paths are correct
   - Check RUN commands are valid
   - Ensure WORKDIR exists before COPY

### 3.2 Error: "Could not find a version that satisfies the requirement"

**Example**:
```
Could not find a version that satisfies the requirement FastAP==0.104.0
```

**Diagnosis**: Package name or version is wrong

**Solution**:
1. Check package name spelling
2. Verify version exists (check PyPI)
3. Fix in requirements.txt:
   ```
   # ❌ Wrong
   FastAP==0.104.0

   # ✅ Correct
   FastAPI==0.104.0
   ```

### 3.3 Error: "python: command not found"

**Diagnosis**: Python not installed in Docker image

**Solution**:
1. Ensure Dockerfile starts with: `FROM python:3.11-slim`
2. Add python install if using different base image
3. RUN python before using it

---

## Section 4: Startup Errors

**When it happens**: After Docker build succeeds, app tries to start

**Common causes**:
- Import errors in Python code
- Missing modules or dependencies
- Syntax errors
- Database connection issues

### 4.1 Error: "ImportError" or "ModuleNotFoundError"

**Example**:
```
ModuleNotFoundError: No module named 'src'
ImportError: cannot import name 'app' from 'src.main'
```

**Diagnosis**: Python import path issue

**Solution**:

1. **Check file structure**:
   ```
   src/
   ├── __init__.py         ← Required!
   ├── main.py
   └── config.py
   ```
   Every directory needs `__init__.py`

2. **Test locally**:
   ```bash
   python -c "from src.main import app"
   # Should work without error
   ```

3. **Check working directory in Dockerfile**:
   ```dockerfile
   WORKDIR /app
   # Then your imports work from /app/src/main.py
   ```

4. **Fix in Dockerfile**:
   ```dockerfile
   # Set working directory before running
   WORKDIR /app
   COPY . .
   CMD ["uvicorn", "src.main:app", "--host", "0.0.0.0", "--port", "8000"]
   ```

### 4.2 Error: "Can't load plugin: sqlalchemy.dialects:driver"

**Diagnosis**: DATABASE_URL missing or malformed

**Solution**:

1. **Check DATABASE_URL in render.yaml**:
   ```yaml
   - key: DATABASE_URL
     fromDatabase:
       name: my-postgres-db    # ← Must match database name
       property: connectionString
   ```

2. **Verify database exists**:
   - Go to Render Dashboard → Databases tab
   - Check database is "Available" (not "Creating")
   - Database name matches `name` in render.yaml

3. **Check format if manually set**:
   ```
   postgresql://user:password@host:5432/dbname  ✅
   postgres://user:password@host:5432/dbname    ❌ (use postgresql)
   ```

4. **Add to error handler if optional**:
   ```python
   if not settings.DATABASE_URL:
       logger.warning("DATABASE_URL not set, skipping DB init")
   ```

### 4.3 Error: "SyntaxError: invalid syntax"

**Diagnosis**: Python syntax error in your code

**Solution**:
1. Check the file and line number in error
2. Look for common syntax issues:
   - Missing colons (`:`)
   - Mismatched parentheses
   - Indentation errors
3. Test locally: `python -m py_compile src/main.py`
4. Fix and recommit

---

## Section 5: Configuration Errors

**When it happens**: During app startup, usually in Pydantic Settings validation

**Common causes**:
- Environment variable type mismatch
- Required variable not set
- Variable format wrong (e.g., list formatted as JSON)

### 5.1 Error: "Input should be a valid string [type=string_type]"

**Diagnosis**: Pydantic type validation failure, usually with ALLOWED_ORIGINS

**Solution**:

1. **Check field type in config.py**:
   ```python
   # ❌ Wrong - expects JSON list
   ALLOWED_ORIGINS: list = "https://example.com,https://other.com"

   # ✅ Correct - accepts both string and list
   ALLOWED_ORIGINS: Union[str, list] = "http://localhost:3000"
   ```

2. **Add field validator**:
   ```python
   @field_validator("ALLOWED_ORIGINS", mode="before")
   @classmethod
   def parse_origins(cls, v):
       if isinstance(v, str):
           return [o.strip() for o in v.split(",")]
       return v if isinstance(v, list) else []
   ```

3. **Check environment variable format**:
   - ✅ `https://example.com,https://other.com` (comma-separated)
   - ❌ `["https://example.com"]` (JSON array)

4. **Fix in Render Environment tab**:
   - Remove JSON brackets
   - Use comma-separated format
   - Redeploy

### 5.2 Error: "Required variable X not found"

**Diagnosis**: Environment variable not set

**Solution**:
1. Check variable name in error message
2. Go to Render Dashboard → Service → Environment tab
3. Search for the variable
4. If missing, add it:
   - For non-secrets: Update render.yaml and push
   - For secrets: Add manually in dashboard
5. Redeploy service

### 5.3 Error: "ValueError: invalid literal for int()"

**Diagnosis**: Environment variable has wrong type

**Solution**:
```python
# In config.py, ensure correct types:
PORT: int = 8000  # ← Pydantic auto-converts "8000" string to 8000 int

# For custom types, add validator:
@field_validator("PORT", mode="before")
def validate_port(cls, v):
    return int(v) if isinstance(v, str) else v
```

---

## Section 6: CORS Errors

**Symptoms**: Frontend can't call backend, browser CORS error

See **Workflow 4: CORS Configuration** for detailed CORS debugging.

**Quick fix**:
1. Check middleware order (CORS last)
2. Verify ALLOWED_ORIGINS in Render Environment
3. Include frontend URL
4. Redeploy

---

## Section 7: Health Check Errors

**Symptoms**:
```
Deployment failed. Service unhealthy.
Health check failed
```

**Diagnosis**: Health endpoint not responding 200 OK

**Solution**:

1. **Verify health endpoint exists**:
   ```python
   # In src/main.py
   @app.get("/api/v1/health")
   async def health_check():
       return {"status": "ok"}
   ```

2. **Check health path in render.yaml**:
   ```yaml
   healthCheckPath: /api/v1/health
   ```
   Must match exactly (case-sensitive, no trailing slash)

3. **Test locally**:
   ```bash
   curl http://localhost:8000/api/v1/health
   # Should return 200 with JSON
   ```

4. **Check logs for startup errors**:
   - If app didn't start, health endpoint won't respond
   - Look for Import/Config errors above health check message

5. **If endpoint has dependencies**:
   - Health endpoint should work even if database is down
   - Or handle database errors gracefully:
   ```python
   @app.get("/api/v1/health")
   async def health_check():
       try:
           # Optional: test database
           db_status = "healthy"
       except:
           db_status = "checking"

       return {
           "status": "ok",
           "database": db_status
       }
   ```

---

## Step 3: Search for Help

### Common Error Patterns

See **Pattern: Common Errors** for detailed solutions to 8 common errors:
- SQLAlchemy plugin errors
- Pydantic validation errors
- CORS errors
- Docker build errors
- Connection pooling errors
- Alembic migration errors

### Log Reading

See **Pattern: Log Interpretation** for how to read Render deployment logs and understand each stage.

---

## Step 4: Reproduce Locally

Most errors are easier to fix locally:

```bash
# Test Docker build
docker build -t test-app .

# Run locally with environment
docker run -p 8000:8000 \
  -e DATABASE_URL="postgresql://user:pass@localhost/db" \
  -e ENVIRONMENT=development \
  test-app

# Check logs
docker logs <container-id>

# Test health endpoint
curl http://localhost:8000/api/v1/health
```

If it works locally, it should work on Render.

---

## Step 5: Fix and Redeploy

### 5.1 Code Changes

For code/config changes:

```bash
# Fix the issue
# (update Dockerfile, requirements.txt, src/config.py, etc.)

git add <files>
git commit -m "Fix: <error description>"
git push origin main
# Render auto-redeploys
```

### 5.2 Environment Variable Changes

For environment variable changes (don't require code changes):

1. Go to Render Dashboard → Service → **Environment** tab
2. Update/add variables
3. Click **Deploy** button (no rebuild needed)
4. Service restarts with new variables

### 5.3 Check Redeployment

1. Monitor **Logs** tab
2. Look for "Building" → "Deploying" → "Live"
3. Check logs for the same error
4. If gone, success!

---

## Systematic Debugging Flowchart

```
Error in Render logs?
  ├─ "failed to build" → Check Dockerfile path and requirements.txt
  ├─ "ImportError" → Check imports, __init__.py, working directory
  ├─ "Can't load plugin" → Check DATABASE_URL, database exists
  ├─ "Input should be valid" → Check Pydantic type, format of env var
  ├─ "CORS policy blocked" → Check middleware order, ALLOWED_ORIGINS
  ├─ "unhealthy" → Check health endpoint, app startup errors
  └─ Unknown error → Search error in Pattern: Common Errors

Still stuck?
  ├─ Re-read full error (not just first line)
  ├─ Check logs for context (5 lines before/after)
  ├─ Test locally with docker
  ├─ Read related pattern (middleware, logs, env)
  └─ Search error online with "Render.com"
```

---

## Success Checklist

- [ ] Error identified and understood
- [ ] Error categorized (build/startup/config/etc)
- [ ] Cause identified
- [ ] Fix applied (code or environment)
- [ ] Tested locally if applicable
- [ ] Redeployed
- [ ] Service shows "Live" status
- [ ] Logs show no ERROR messages
- [ ] Functionality works

**Excellent!** You've debugged and fixed the deployment issue. 🎉

---

## Next Steps

1. **Connect frontend to backend** → Workflow 6: Frontend Integration
2. **Learn more** → Pattern: Common Errors, Pattern: Log Interpretation
