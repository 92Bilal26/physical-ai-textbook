# Workflow 3: Environment Variables - Configure Your Application

This workflow guides you through setting up all necessary environment variables for your FastAPI application on Render.

**Time estimate**: 5-10 minutes
**Prerequisites**: Completed Workflow 1 (service deployed)
**Success indicator**: Application loads all variables without errors

---

## Overview

Environment variables configure your application without code changes:

- **Non-secrets** (in render.yaml): ENVIRONMENT, LOG_LEVEL, ALLOWED_ORIGINS
- **Secrets** (Render dashboard): API keys, database credentials
- **Auto-linked** (from databases): DATABASE_URL, REDIS_URL

---

## Step 1: Review render.yaml Variables

Your `render.yaml` should include non-secret variables:

```yaml
envVars:
  - key: ENVIRONMENT
    value: production
  - key: LOG_LEVEL
    value: INFO
  - key: DEBUG
    value: "false"
  - key: ALLOWED_ORIGINS
    value: https://example.com,https://example.github.io
  - key: DATABASE_URL
    fromDatabase:
      name: my-postgres-db
      property: connectionString
```

### 1.1 Standard Variables

These should be in render.yaml:

| Variable | Value | Example |
|----------|-------|---------|
| ENVIRONMENT | production | production |
| LOG_LEVEL | INFO or DEBUG | INFO |
| DEBUG | false | false |
| ALLOWED_ORIGINS | Comma-separated URLs | https://example.com,https://example.github.io |

### 1.2 Database Variables

These link to your databases:

```yaml
- key: DATABASE_URL
  fromDatabase:
    name: my-postgres-db        # Must match database name
    property: connectionString

- key: REDIS_URL
  fromDatabase:
    name: my-redis-db
    property: connectionString
```

**Check**:
- [ ] ENVIRONMENT is set to "production"
- [ ] LOG_LEVEL is INFO (or DEBUG for troubleshooting)
- [ ] ALLOWED_ORIGINS includes all frontend URLs
- [ ] Database names match (in databases section)

---

## Step 2: Add Secret Variables in Render Dashboard

Secret variables (API keys, etc.) are NOT committed to code. Add them manually:

### 2.1 Go to Service Environment Tab

1. Render Dashboard → Your Service
2. Click **Environment** tab
3. Click **Add Environment Variable**

### 2.2 Add Each Secret Variable

For OpenAI (if using):

```
Key: OPENAI_API_KEY
Value: sk-...
```

For Qdrant (if using):

```
Key: QDRANT_URL
Value: https://host:6333

Key: QDRANT_API_KEY
Value: api-key-value
```

For any other API key:

```
Key: API_KEY_NAME
Value: actual-key-value
```

### 2.3 Verify Secret Variables

After adding, your Environment tab should show:

```
ENVIRONMENT=production
LOG_LEVEL=INFO
DEBUG=false
ALLOWED_ORIGINS=https://example.com
DATABASE_URL=***hidden***
OPENAI_API_KEY=***hidden***
QDRANT_URL=***hidden***
QDRANT_API_KEY=***hidden***
```

**Check**:
- [ ] All secret variables added
- [ ] Variables show as "***hidden***" (security)
- [ ] No secrets in render.yaml file
- [ ] No secrets in your code repository

---

## Step 3: Verify Variable Format

### 3.1 Correct Format

✅ **CORRECT**:
```
ALLOWED_ORIGINS=https://example.com,https://other.com
ENVIRONMENT=production
DEBUG=false
```

❌ **WRONG**:
```
ALLOWED_ORIGINS=[https://example.com, https://other.com]  # Brackets
ALLOWED_ORIGINS="https://example.com"                     # Quotes
ENVIRONMENT=Production                                     # Wrong case
DEBUG=True                                                  # Wrong format
```

### 3.2 List Variables

For variables containing lists (like ALLOWED_ORIGINS):

✅ **CORRECT**: Comma-separated without spaces
```
https://example.com,https://other.com
```

❌ **WRONG**: JSON format with brackets
```
["https://example.com", "https://other.com"]
```

**Check**:
- [ ] No brackets in list variables
- [ ] No quotes around values
- [ ] Comma-separated with optional spaces
- [ ] All uppercase with underscores

---

## Step 4: Update Your Application Configuration

Your `src/config.py` should load these variables:

```python
from typing import Union
from pydantic import BaseSettings, field_validator

class Settings(BaseSettings):
    ENVIRONMENT: str = "development"
    LOG_LEVEL: str = "INFO"
    DEBUG: bool = False

    DATABASE_URL: str
    REDIS_URL: str = None

    # ✅ Handle comma-separated string, convert to list
    ALLOWED_ORIGINS: Union[str, list] = "http://localhost:3000"

    @field_validator("ALLOWED_ORIGINS", mode="before")
    @classmethod
    def parse_origins(cls, v):
        if isinstance(v, str):
            return [o.strip() for o in v.split(",")]
        return v if isinstance(v, list) else []

    OPENAI_API_KEY: str = None
    QDRANT_URL: str = None
    QDRANT_API_KEY: str = None

    class Config:
        env_file = ".env"
        case_sensitive = True
```

**Key Pattern**:
- Use `Union[str, list]` for list-type variables
- Use `@field_validator` to convert string to list
- This handles Render's comma-separated format

See `examples/config.py.example` for complete example.

**Check**:
- [ ] config.py loads all variables
- [ ] List variables use Union[str, list]
- [ ] Validators convert string to list
- [ ] Defaults provided for optional variables

---

## Step 5: Test Variable Loading

Add a debug endpoint to verify variables load:

```python
# In src/main.py
from src.config import settings

@app.get("/api/v1/config-debug", tags=["Debug"])
async def config_debug():
    """Debug endpoint to verify environment variables (development only)"""
    if settings.ENVIRONMENT == "production":
        return {"error": "Not available in production"}

    return {
        "environment": settings.ENVIRONMENT,
        "log_level": settings.LOG_LEVEL,
        "allowed_origins": settings.ALLOWED_ORIGINS,
        "has_database_url": bool(settings.DATABASE_URL),
        "has_openai_key": bool(settings.OPENAI_API_KEY),
    }
```

Test it:

```bash
curl https://your-service.onrender.com/api/v1/config-debug
# Should return variables you configured
```

**Check**:
- [ ] All variables load without error
- [ ] Types are correct (string, list, bool)
- [ ] Secret variables show as `True` (exists)
- [ ] No validation errors

---

## Step 6: Commit and Redeploy

If you updated `src/config.py`:

```bash
git add src/config.py
git commit -m "Update environment variable configuration"
git push origin main
```

Render will automatically redeploy with the new configuration.

If you only changed environment variables in Render dashboard:

1. Go to Service → **Environment** tab
2. Click **Deploy** button next to your service
3. Render will restart with new variables (no rebuild needed)

**Check**:
- [ ] Changes committed and pushed
- [ ] Service redeployed
- [ ] New variables active

---

## Common Issues

### Issue: "Input should be a valid string [type=string_type]"

**Cause**: ALLOWED_ORIGINS field expects string but gets list

**Solution**:
```python
# ✅ CORRECT
ALLOWED_ORIGINS: Union[str, list] = "http://localhost:3000"

@field_validator("ALLOWED_ORIGINS", mode="before")
def parse_origins(cls, v):
    if isinstance(v, str):
        return [o.strip() for o in v.split(",")]
    return v
```

### Issue: "Environment variable X is required"

**Cause**: Required variable not set in Render

**Solution**:
1. Go to Service → Environment tab
2. Add the missing variable
3. Click **Deploy** or **Redeploy**

### Issue: "Can't load plugin: sqlalchemy.dialects:driver"

**Cause**: DATABASE_URL is missing or malformed

**Solution**:
1. Check Environment tab for DATABASE_URL
2. If missing, add it via `fromDatabase` in render.yaml
3. Verify format: `postgresql://user:password@host/dbname`
4. Redeploy

### Issue: "API key not working"

**Cause**: API key value is wrong or expired

**Solution**:
1. Verify API key value in Render Environment tab
2. Regenerate key if expired
3. Update in Render dashboard
4. Redeploy service

---

## Environment Variable Reference

### Application Settings

| Variable | Type | Example | Required | Secret |
|----------|------|---------|----------|--------|
| ENVIRONMENT | string | production | Yes | No |
| LOG_LEVEL | string | INFO | No | No |
| DEBUG | boolean | false | No | No |

### Database

| Variable | Type | Example | Required | Secret |
|----------|------|---------|----------|--------|
| DATABASE_URL | string | postgresql://... | Conditional | Yes |
| REDIS_URL | string | redis://... | Conditional | Yes |

### CORS

| Variable | Type | Example | Required | Secret |
|----------|------|---------|----------|--------|
| ALLOWED_ORIGINS | string (comma-separated) | https://example.com | No | No |

### APIs

| Variable | Type | Example | Required | Secret |
|----------|------|---------|----------|--------|
| OPENAI_API_KEY | string | sk-... | Conditional | Yes |
| QDRANT_URL | string | https://host:6333 | Conditional | Yes |
| QDRANT_API_KEY | string | api-key | Conditional | Yes |

---

## Success Checklist

- [ ] All variables appear in Environment tab
- [ ] Secret variables are marked as "***hidden***"
- [ ] Config debug endpoint shows all variables
- [ ] No validation errors in logs
- [ ] Application loads without "missing variable" errors
- [ ] No secrets visible in logs or code

**Excellent!** Your application is fully configured. 🎉

---

## Next Steps

1. **Handle CORS errors** → Workflow 4: CORS Configuration
2. **Debug errors** → Workflow 5: Error Debugging
3. **Connect frontend** → Workflow 6: Frontend Integration
