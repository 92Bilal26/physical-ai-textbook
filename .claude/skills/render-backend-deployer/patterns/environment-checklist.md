# Pre-Deployment Environment Variable Checklist

Use this checklist before deploying to Render.com to catch configuration issues early.

---

## Database Variables

- [ ] **DATABASE_URL** exists in Render Environment tab
- [ ] DATABASE_URL format is correct: `postgresql://user:password@host/dbname`
- [ ] REDIS_URL exists (if using Redis)
- [ ] REDIS_URL format is correct: `redis://host:port`
- [ ] Database service is marked "Available" in Render dashboard

## API Key Variables

- [ ] **OPENAI_API_KEY** is set (if using OpenAI)
- [ ] OPENAI_API_KEY starts with `sk-`
- [ ] **QDRANT_URL** is set (if using Qdrant)
- [ ] QDRANT_API_KEY is set (if using Qdrant)
- [ ] All API keys are marked as "secret" (not exposed in logs)

## Application Variables

- [ ] **ENVIRONMENT** is set to "production"
- [ ] LOG_LEVEL is set appropriately (INFO or DEBUG)
- [ ] **ALLOWED_ORIGINS** includes all frontend URLs
- [ ] ALLOWED_ORIGINS format: comma-separated (no quotes, no brackets)
- [ ] health check endpoint path is correct in render.yaml

## Variable Format Validation

- [ ] All variable names use UPPERCASE_WITH_UNDERSCORES
- [ ] No spaces in variable names or values
- [ ] List variables use comma-separated format: `url1,url2,url3`
- [ ] No JSON syntax for lists (no brackets or quotes)
- [ ] Special characters properly escaped (if any)

## Pydantic Settings Validation

- [ ] Your `src/config.py` loads all required variables
- [ ] All variables have proper type hints
- [ ] List types use `Union[str, list]` for flexibility
- [ ] Validators properly convert string lists to Python lists
- [ ] No missing required variables (should error in startup tests)

## Docker & Build Validation

- [ ] Dockerfile path is correct in render.yaml
- [ ] Build context is set correctly (or omitted if at root)
- [ ] Requirements.txt includes all Python dependencies
- [ ] Dockerfile installs requirements correctly
- [ ] CMD runs uvicorn with correct app path

## Health Check Validation

- [ ] Health endpoint exists at specified path
- [ ] Health endpoint returns 200 OK
- [ ] Health path in render.yaml matches your code
- [ ] Health endpoint doesn't require database (or handles errors gracefully)
- [ ] Tested locally: `curl http://localhost:8000/health`

## Frontend Integration Validation

- [ ] Frontend has correct backend URL
- [ ] Backend URL includes random suffix: `https://service-XXXX.onrender.com`
- [ ] Frontend is sending CORS headers correctly
- [ ] Backend has CORS middleware configured
- [ ] CORS middleware is in correct position (before rate limiting)

## Secret Security Validation

- [ ] API keys are NOT in your repository
- [ ] API keys are NOT in render.yaml (use render.yaml only for non-secrets)
- [ ] Secret variables are marked with `sync: false` in render.yaml
- [ ] All secrets added manually in Render UI
- [ ] Render logs don't expose secrets (check "View Logs" tab)

---

## Pre-Deployment Testing Checklist

Before clicking "Deploy" on Render:

- [ ] Local build succeeds: `docker build -t myapp .`
- [ ] Local startup works: `docker run -p 8000:8000 myapp`
- [ ] Health endpoint works locally: `curl http://localhost:8000/health`
- [ ] render.yaml syntax is valid (no JSON errors)
- [ ] All database names match between envVars and databases sections
- [ ] No typos in environment variable names
- [ ] All secret values are ready (not placeholder text)

## Post-Deployment Validation Checklist

After deployment completes:

- [ ] Service shows "Live" in Render dashboard
- [ ] Deployment logs show no errors (Logs tab)
- [ ] Health endpoint returns 200: `curl https://service-XXXX.onrender.com/health`
- [ ] Environment variables are accessible in app (test log output)
- [ ] Database connections work (test query if possible)
- [ ] Frontend can make API calls without CORS errors
- [ ] API endpoints respond correctly

---

## Quick Diagnosis

If deployment fails, check these first:

1. **Logs Tab** - Most informative, read full error
2. **DATABASE_URL** - Most common issue, verify it exists
3. **ALLOWED_ORIGINS** - Comma-separated format, no brackets
4. **Middleware Order** - CORS must be first
5. **Health Endpoint** - Path must be exact match
6. **Docker** - Dockerfile path and build context

---

## Variable Reference

| Variable | Type | Example | Secret? | Required? |
|----------|------|---------|---------|-----------|
| ENVIRONMENT | string | production | No | Yes |
| LOG_LEVEL | string | INFO | No | No |
| DEBUG | boolean | false | No | No |
| DATABASE_URL | string | postgresql://user:pass@host/db | Yes | Conditional |
| REDIS_URL | string | redis://host:6379 | Yes | Conditional |
| ALLOWED_ORIGINS | string (comma-separated) | https://example.com,https://other.com | No | No |
| OPENAI_API_KEY | string | sk-... | Yes | Conditional |
| QDRANT_URL | string | https://host:6333 | Yes | Conditional |
| QDRANT_API_KEY | string | api-key | Yes | Conditional |

---

Use this checklist every deployment to prevent common issues!
