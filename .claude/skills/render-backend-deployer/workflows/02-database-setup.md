# Workflow 2: Database Setup - Create and Configure PostgreSQL and Redis

This workflow guides you through setting up databases on Render and connecting them to your FastAPI application.

**Time estimate**: 5-10 minutes
**Prerequisites**: Completed Workflow 1 (service deployed and "Live")
**Success indicator**: Database connections work in your application

---

## Overview

Your FastAPI application needs databases to store data:

- **PostgreSQL**: Primary relational database for application data
- **Redis** (optional): Key-value store for caching and sessions

Render can create and manage these automatically via `render.yaml`.

---

## Step 1: Verify Your Service is Live

Before adding databases, ensure your service is deployed and healthy:

```bash
curl https://your-service.onrender.com/api/v1/health
# Should return: {"status":"ok",...}
```

If not live, complete **Workflow 1: Initial Setup** first.

---

## Step 2: Update render.yaml with Databases

Your `render.yaml` should already have a databases section. If not, add it:

```yaml
services:
  - type: web
    name: my-fastapi-backend
    runtime: docker
    dockerfilePath: ./Dockerfile
    healthCheckPath: /api/v1/health

    envVars:
      # ... other variables ...
      - key: DATABASE_URL
        fromDatabase:
          name: my-postgres-db
          property: connectionString

      - key: REDIS_URL
        fromDatabase:
          name: my-redis-db
          property: connectionString

databases:
  - name: my-postgres-db
    databaseName: appdb
    user: appuser
    plan: free

  - name: my-redis-db
    plan: free
```

### 2.1 PostgreSQL Configuration

```yaml
- name: my-postgres-db        # Service identifier (unique)
  databaseName: appdb         # Actual database name
  user: appuser               # Database user
  plan: free                  # free or starter
```

**Customize**:
- `name`: Unique identifier for this service (used in envVars)
- `databaseName`: Actual database name (appears in PostgreSQL)
- `user`: Database user name (used for connections)
- `plan`: free (10 connections) or starter (20 connections)

**Check**:
- [ ] PostgreSQL `name` matches `fromDatabase.name` in envVars
- [ ] No spaces or special characters in database/user names
- [ ] Plan chosen based on needs (free for development/testing)

### 2.2 Redis Configuration (Optional)

If you need caching or sessions:

```yaml
- name: my-redis-db
  plan: free
```

Redis shows as "Key Value" in Render dashboard.

**Check**:
- [ ] Redis `name` matches `fromDatabase.name` in envVars
- [ ] Plan chosen (free or starter+)

---

## Step 3: Commit and Push

After updating `render.yaml`:

```bash
git add render.yaml
git commit -m "Add PostgreSQL and Redis database configuration"
git push origin main
```

Render will automatically detect the change and redeploy.

**Check**:
- [ ] render.yaml updated in GitHub
- [ ] Changes pushed to main branch

---

## Step 4: Verify Database Creation

After Render redeploys:

1. Go to Render Dashboard
2. Check **Databases** tab
3. You should see:
   - **PostgreSQL**: my-postgres-db (Status: Available)
   - **Redis**: my-redis-db (Status: Available)

Wait for status to change from "Creating" to "Available" (usually 30-60 seconds).

**Check**:
- [ ] PostgreSQL shows "Available"
- [ ] Redis shows "Available" (if configured)
- [ ] Database names match your render.yaml

---

## Step 5: Access Database Information

### 5.1 View Connection Strings

1. Click on the PostgreSQL database
2. **Internal Connection String** (for your Render service):
   ```
   postgresql://user:password@host/dbname
   ```
3. **External Connection String** (for local testing):
   ```
   postgresql://user:password@external-host/dbname
   ```

### 5.2 Copy Connection Strings

**PostgreSQL Internal Connection String**:
- Used automatically in `render.yaml` → `envVars` → `DATABASE_URL`
- Your app receives this automatically

**Redis Internal Connection String**:
- Used automatically in `render.yaml` → `envVars` → `REDIS_URL`
- Format: `redis://host:port`

**Check**:
- [ ] PostgreSQL connection string visible
- [ ] Connection string starts with `postgresql://`
- [ ] Redis connection string visible (if using Redis)
- [ ] Connection strings are marked as internal (safer)

---

## Step 6: Verify Your Application Receives Connection Strings

Check your application logs:

1. Go to Service → **Logs** tab
2. Search for "DATABASE_URL" in your startup logs
3. You should see something like:
   ```
   INFO: DATABASE_URL=postgresql://appuser:***@host/appdb
   ```

If you see this in your logs (with *** masking the password), database is connected!

**Add logging to verify** (in src/main.py):

```python
import logging
from src.config import settings

logger = logging.getLogger(__name__)

@asynccontextmanager
async def lifespan(app: FastAPI):
    logger.info(f"DATABASE_URL: {settings.DATABASE_URL[:30]}...")  # Log first 30 chars
    logger.info(f"REDIS_URL: {settings.REDIS_URL[:30]}...")
    yield
```

**Check**:
- [ ] Logs show database URL is set
- [ ] URL format is correct (`postgresql://`)
- [ ] Password is masked (*** in logs)

---

## Step 7: Test Database Connection (Optional)

If you have database connection code, test it:

```python
# In src/main.py or a test file
from sqlalchemy import create_engine, text

@app.get("/api/v1/test-db")
async def test_db():
    try:
        engine = create_engine(settings.DATABASE_URL)
        with engine.connect() as conn:
            result = conn.execute(text("SELECT 1"))
            return {"database": "connected", "result": result.fetchone()[0]}
    except Exception as e:
        return {"error": str(e)}
```

Then test:

```bash
curl https://your-service.onrender.com/api/v1/test-db
# Should return: {"database":"connected","result":1}
```

**Check**:
- [ ] Database connection test succeeds
- [ ] Response shows connection is working

---

## Step 8: Run Database Migrations (If Using Alembic)

If your application uses Alembic migrations:

### 8.1 Verify Migrations Exist

```
migrations/
├── env.py
├── alembic.ini
└── versions/
    ├── 001_initial.py
    └── ...
```

### 8.2 Add Migration Command to Dockerfile

Update your `Dockerfile` to run migrations on startup:

```dockerfile
# Before the CMD line, add:
RUN alembic upgrade head || echo "No migrations to run"

# Then the normal CMD:
CMD ["uvicorn", "src.main:app", "--host", "0.0.0.0", "--port", "8000"]
```

Or alternatively, run migrations in your app startup:

```python
# In src/main.py
from alembic.config import Config
from alembic.script import ScriptDirectory
from alembic.runtime.migration import MigrationContext
from alembic.operations import Operations

@asynccontextmanager
async def lifespan(app: FastAPI):
    # Run migrations
    alembic_cfg = Config("alembic.ini")
    alembic_cfg.set_main_option("sqlalchemy.url", settings.DATABASE_URL)

    with engine.begin() as connection:
        context = MigrationContext.configure(connection)
        op = Operations(context)
        # Run pending migrations

    logger.info("Migrations completed")
    yield
```

### 8.3 Check Migration Logs

After redeployment, check logs for migration status:

```
INFO [alembic.runtime.migration] Context impl PostgresqlImpl.
INFO [alembic.runtime.migration] Running upgrade  -> 001_initial
INFO [alembic.runtime.migration] Done
```

**Check**:
- [ ] Migrations run successfully
- [ ] No "Can't load plugin" errors
- [ ] Database schema created (if migration creates tables)

---

## Common Issues

### Issue: "Database is not available"

**Cause**: Database still creating or connection string incorrect

**Solution**:
1. Wait 30-60 seconds for "Available" status
2. Check render.yaml database names match
3. Verify `fromDatabase.name` matches the database `name`

### Issue: "too many connections"

**Cause**: Free tier PostgreSQL limited to ~10 connections

**Solution**:
1. Use connection pooling in your app
2. Limit concurrent requests
3. Upgrade to Starter tier

See **Pattern: Connection Pooling** for code example.

### Issue: "Can't load plugin: sqlalchemy.dialects:driver"

**Cause**: DATABASE_URL missing or malformed

**Solution**:
1. Check render.yaml `fromDatabase` configuration
2. Verify `fromDatabase.property` is `connectionString`
3. Check database names match
4. Redeploy service

### Issue: "Connection refused"

**Cause**: Using external connection string in container (wrong)

**Solution**:
1. Always use **Internal Connection String** in Render service
2. External string is for local testing only
3. Verify render.yaml uses correct string link

---

## Success Checklist

- [ ] PostgreSQL shows "Available" in Render dashboard
- [ ] Redis shows "Available" (if configured)
- [ ] Database URL appears in application logs
- [ ] Database connection test succeeds (if configured)
- [ ] Migrations complete successfully (if using Alembic)
- [ ] No "connection refused" errors

**Great!** Your databases are set up and connected. 🎉

---

## Next Steps

1. **Configure environment variables** → Workflow 3: Environment Variables
2. **Handle CORS errors** → Workflow 4: CORS Configuration
3. **Debug errors** → Workflow 5: Error Debugging
