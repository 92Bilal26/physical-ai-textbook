# Workflow 1: Initial Setup - Deploy Your FastAPI Backend to Render

This workflow guides you through the first deployment of a FastAPI backend to Render.com. Follow these steps in order to get your service running.

**Time estimate**: 10-15 minutes
**Prerequisites**: GitHub account, Render.com free account, FastAPI application with Docker setup
**Success indicator**: Service shows "Live" in Render dashboard

---

## Step 1: Prepare Your Repository

### 1.1 Verify Project Structure

Your repository should have this structure:

```
your-repo/
├── Dockerfile              # From examples/Dockerfile.example
├── requirements.txt        # Python dependencies
├── src/
│   ├── __init__.py
│   ├── main.py            # From examples/main.py.example
│   └── config.py          # From examples/config.py.example
├── migrations/            # Alembic migrations (optional)
│   └── env.py
└── render.yaml            # From examples/render.yaml.example
```

**Check**:
- [ ] Dockerfile exists at repository root
- [ ] requirements.txt exists with all dependencies
- [ ] src/main.py has FastAPI app instance
- [ ] src/config.py loads environment variables

### 1.2 Verify Dockerfile

Your Dockerfile should:
1. Use `python:3.11-slim` base image
2. Install dependencies from `requirements.txt`
3. Copy application code
4. Expose port 8000
5. Run uvicorn on `0.0.0.0:8000`

```bash
# Test locally
docker build -t test-app .
docker run -p 8000:8000 test-app
curl http://localhost:8000/api/v1/health
# Should return: {"status":"ok",...}
```

If this works locally, it will work on Render.

**Check**:
- [ ] Docker build succeeds: `docker build -t test-app .`
- [ ] Docker run succeeds: `docker run -p 8000:8000 test-app`
- [ ] Health endpoint responds: `curl http://localhost:8000/api/v1/health` returns 200

### 1.3 Verify Requirements.txt

Your `requirements.txt` should include:

```
FastAPI>=0.104.0
uvicorn[standard]>=0.24.0
pydantic>=2.0.0
python-dotenv>=1.0.0
```

Plus any other dependencies (sqlalchemy, openai, etc.)

**Check**:
- [ ] requirements.txt exists
- [ ] No version conflicts (test locally: `pip install -r requirements.txt`)
- [ ] All imports in src/ are in requirements.txt

---

## Step 2: Create render.yaml

Create `render.yaml` at your repository root:

```yaml
services:
  - type: web
    name: my-fastapi-backend
    runtime: docker
    dockerfilePath: ./Dockerfile
    healthCheckPath: /api/v1/health

    envVars:
      - key: ENVIRONMENT
        value: production
      - key: LOG_LEVEL
        value: INFO
      - key: ALLOWED_ORIGINS
        value: https://your-frontend.com,http://localhost:3000

databases:
  - name: my-postgres-db
    databaseName: appdb
    user: appuser
    plan: free
```

### 2.1 Customize Your render.yaml

1. **Service Name** (line 4): Replace `my-fastapi-backend`
   - Must be unique in your Render account
   - Used in URL: `https://my-fastapi-backend-XXXX.onrender.com`
   - Only alphanumeric and hyphens

2. **Health Check Path** (line 7): Match your endpoint
   - Default: `/api/v1/health`
   - Must return 200 OK
   - Must exist in your src/main.py

3. **ALLOWED_ORIGINS** (line 14): Add your frontend URLs
   - Comma-separated, no brackets
   - Example: `https://example.github.io,https://example.com`
   - For development: `http://localhost:3000,http://localhost:5173`

4. **Database Names** (lines 19-22): Choose database names
   - `name`: Service identifier (unique)
   - `databaseName`: Actual database name
   - `user`: Database user name

**Check**:
- [ ] render.yaml is valid YAML (no syntax errors)
- [ ] Service name is unique (no spaces, lowercase)
- [ ] Health check path matches your code
- [ ] Database names match between envVars and databases sections

### 2.2 Optional: Add Redis for Caching

If you need Redis (optional):

```yaml
databases:
  - name: my-postgres-db
    # ... postgres config
  - name: my-redis-db
    plan: free
```

Then in envVars:

```yaml
  - key: REDIS_URL
    fromDatabase:
      name: my-redis-db
      property: connectionString
```

---

## Step 3: Push to GitHub

Commit your changes and push:

```bash
git add Dockerfile requirements.txt src/main.py src/config.py render.yaml
git commit -m "Add Render deployment configuration"
git push origin main
```

**Check**:
- [ ] Files committed to repository
- [ ] render.yaml is visible on GitHub
- [ ] Dockerfile is visible on GitHub

---

## Step 4: Create Render Service

1. Go to [Render Dashboard](https://dashboard.render.com)

2. Click **New +** → **Web Service**

3. Select your GitHub repository

4. Render will automatically detect `render.yaml`

5. Review the configuration:
   - Service name
   - Docker runtime
   - Health check path
   - Environment variables
   - Database plan

6. Click **Create Web Service**

Render will:
- Build your Docker image
- Create PostgreSQL database (if specified)
- Create Redis database (if specified)
- Deploy your application
- Start health checks

---

## Step 5: Monitor Deployment

### 5.1 Watch the Logs

1. Render → Your Service → **Logs** tab
2. Watch for these success indicators:

```
Upload succeeded
==> Deploying...
INFO:     Started server process
INFO:     Application startup complete.
INFO:     Uvicorn running on http://0.0.0.0:8000
```

### 5.2 Wait for Service Status

Service will show:
- **Building** (1-3 minutes) - Docker image building
- **Deploying** (1-2 minutes) - Running your app
- **Live** ✅ - Service is healthy and running

**Check**:
- [ ] Service shows "Live" status
- [ ] No ERROR messages in logs
- [ ] "Uvicorn running" message appears

### 5.3 Test Health Endpoint

Once "Live", test your health endpoint:

```bash
curl https://my-fastapi-backend-XXXX.onrender.com/api/v1/health
# Should return: {"status":"ok","environment":"production",...}
```

Copy your service URL from Render dashboard (looks like `https://my-fastapi-backend-XXXX.onrender.com`)

**Check**:
- [ ] Health endpoint returns 200 OK
- [ ] Response contains `"status":"ok"`

---

## Common Issues During Initial Setup

### Issue: "Service is unhealthy"

**Cause**: Health check endpoint not responding

**Solution**:
1. Check render.yaml `healthCheckPath` matches your code
2. Verify endpoint exists: Search src/main.py for the path
3. Test locally: `curl http://localhost:8000/api/v1/health`
4. Check logs: Look for app startup errors

### Issue: "failed to build image"

**Cause**: Docker build failed

**Solution**:
1. Check Dockerfile path is correct in render.yaml
2. Test locally: `docker build -t test .`
3. Verify requirements.txt exists
4. Check logs for actual error

### Issue: "Cannot import module"

**Cause**: Import error in your code

**Solution**:
1. Check src/main.py imports work locally
2. Verify all dependencies in requirements.txt
3. Check file paths and __init__.py files exist

See **Workflow 5: Error Debugging** if you encounter deployment errors.

---

## Next Steps

After successful deployment:

1. **Add a database** → Workflow 2: Database Setup
2. **Configure environment variables** → Workflow 3: Environment Variables
3. **Fix CORS errors** → Workflow 4: CORS Configuration
4. **Debug errors** → Workflow 5: Error Debugging
5. **Connect frontend** → Workflow 6: Frontend Integration

---

## Success Checklist

- [ ] Service shows "Live" in Render dashboard
- [ ] Health endpoint returns 200 OK
- [ ] Logs show "Uvicorn running"
- [ ] No ERROR messages in logs
- [ ] Service URL is accessible (not timeout)

**Congratulations!** Your FastAPI backend is deployed to Render! 🎉
