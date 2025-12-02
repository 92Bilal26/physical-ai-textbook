# Render Backend Deployer - Quick Start Guide

Deploy your FastAPI backend to Render.com in 5 minutes.

---

## 📋 Pre-Flight Checklist (5 minutes)

```bash
# ✅ Verify project structure
ls Dockerfile requirements.txt src/main.py

# ✅ Test locally
docker build -t test .
docker run -p 8000:8000 test
curl http://localhost:8000/api/v1/health
# Should return: {"status":"ok",...}
```

---

## 🚀 Deploy in 5 Steps

### Step 1: Create render.yaml (2 minutes)

Copy from `examples/render.yaml.example` to repository root:

```yaml
services:
  - type: web
    name: my-fastapi-backend        # Change this
    runtime: docker
    dockerfilePath: ./Dockerfile
    healthCheckPath: /api/v1/health

    envVars:
      - key: ENVIRONMENT
        value: production
      - key: ALLOWED_ORIGINS
        value: https://yourname.github.io,http://localhost:3000

databases:
  - name: my-postgres-db
    databaseName: appdb
    user: appuser
    plan: free
```

**Customize**:
- `name`: Your service name (unique)
- `ALLOWED_ORIGINS`: Your frontend URL(s)
- Database names (must be unique)

### Step 2: Push to GitHub (1 minute)

```bash
git add render.yaml Dockerfile requirements.txt src/
git commit -m "Add Render deployment configuration"
git push origin main
```

### Step 3: Create Render Service (1 minute)

1. Go to [Render Dashboard](https://dashboard.render.com)
2. Click **New +** → **Web Service**
3. Select your repository
4. Click **Create Web Service**

Render auto-detects `render.yaml` and deploys.

### Step 4: Wait for Deployment (2-5 minutes)

Watch **Logs** tab:

```
Building Docker image...
Upload succeeded
Starting application...
Uvicorn running on http://0.0.0.0:8000
```

Wait for status: **Live** ✅

### Step 5: Test It Works (30 seconds)

```bash
# Copy URL from Render dashboard
curl https://my-fastapi-backend-XXXX.onrender.com/api/v1/health
# Should return: {"status":"ok",...}
```

**Done!** 🎉 Backend deployed.

---

## 🐛 Common Issues

### "Service is unhealthy"
→ Check health endpoint exists in code at `/api/v1/health`

### "failed to build"
→ Verify `Dockerfile` and `requirements.txt` exist locally

### "Can't load plugin"
→ Check `DATABASE_URL` in Render Environment tab

### "CORS policy blocked"
→ Add frontend URL to `ALLOWED_ORIGINS` in Environment tab

### "ImportError"
→ Ensure `src/__init__.py` exists and imports work locally

---

## 📚 Full Workflows

For step-by-step guides, see:

1. **Workflow 1: Initial Setup** - Complete deployment walkthrough
2. **Workflow 2: Database Setup** - Add PostgreSQL/Redis
3. **Workflow 3: Environment Variables** - Configure secrets and variables
4. **Workflow 4: CORS Configuration** - Fix cross-origin errors
5. **Workflow 5: Error Debugging** - Diagnose and fix issues
6. **Workflow 6: Frontend Integration** - Connect frontend to backend

---

## 📖 Reference Patterns

- **Pattern: Common Errors** - 8 error patterns with solutions
- **Pattern: Middleware Order** - CORS configuration (critical!)
- **Pattern: Environment Checklist** - Pre/post-deployment validation
- **Pattern: Log Interpretation** - How to read Render logs

---

## 📝 Example Files

Copy and customize for your project:

- `examples/render.yaml.example` - Blueprint configuration
- `examples/config.py.example` - Pydantic Settings pattern
- `examples/main.py.example` - FastAPI CORS setup
- `examples/Dockerfile.example` - Production Docker config

---

## ✅ Success Indicators

Your deployment is successful when:

1. ✅ Service shows "Live" in Render dashboard
2. ✅ Health endpoint returns 200: `curl https://your-service.onrender.com/api/v1/health`
3. ✅ Logs show "Uvicorn running on http://0.0.0.0:8000"
4. ✅ No ERROR messages in logs
5. ✅ Frontend can call backend without CORS errors

---

## 🔗 Key URLs

Once deployed, use:

```
Health:     https://my-fastapi-backend-XXXX.onrender.com/api/v1/health
Docs:       https://my-fastapi-backend-XXXX.onrender.com/docs
Logs:       https://dashboard.render.com → Service → Logs
Environment:https://dashboard.render.com → Service → Environment
```

Replace `my-fastapi-backend-XXXX` with your actual service name + random suffix.

---

## ⏱️ Timeline

| Step | Time | Status |
|------|------|--------|
| Prepare repo | 2 min | Verify Docker builds locally |
| Create render.yaml | 2 min | Customize service name, origins |
| Push to GitHub | 1 min | `git push` |
| Create service | 1 min | Click buttons in Render |
| Wait for deploy | 5 min | Watch "Building" → "Live" |
| Test | 1 min | `curl health endpoint` |
| **Total** | **~12 minutes** | ✅ Live! |

---

## 🆘 Need Help?

**Deployment issue?**
1. Check **Workflow 5: Error Debugging**
2. Look up error in **Pattern: Common Errors**
3. Read deployment logs carefully

**CORS problem?**
→ **Workflow 4: CORS Configuration**

**Environment variable issue?**
→ **Workflow 3: Environment Variables**

**Database error?**
→ **Workflow 2: Database Setup**

**Can't get started?**
→ **Workflow 1: Initial Setup**

---

## 📊 What This Deploys

```
Your FastAPI App
  ├─ Docker container (Python 3.11)
  ├─ Uvicorn ASGI server (port 8000)
  ├─ PostgreSQL database (optional)
  ├─ Redis cache (optional)
  ├─ CORS configuration for frontend
  └─ Health monitoring and auto-restart
```

Render manages:
- Infrastructure (servers, networking)
- Docker building and deployment
- Health checks (restarts if unhealthy)
- Logs and monitoring
- SSL/HTTPS certificates

You manage:
- Application code
- Dependencies (requirements.txt)
- Environment variables (secrets)
- Database migrations

---

## 💡 Pro Tips

1. **Test locally first** - Always run `docker build` and `docker run` locally before deploying
2. **Use environment variables** - Never hardcode URLs, keys, or config
3. **Monitor logs** - Check logs tab when something goes wrong
4. **Store secrets securely** - Use Render dashboard, never commit to git
5. **Redeploy for env changes** - Click Deploy button when you update environment variables
6. **Start with free tier** - PostgreSQL and Redis both have free options

---

## 🎯 Next Steps

After successful deployment:

1. ✅ **Connect frontend** - Use backend URL in your frontend code
2. ✅ **Test API endpoints** - Make sure your endpoints work
3. ✅ **Monitor in production** - Check Render logs occasionally
4. ✅ **Add more features** - Deploy new endpoints and features
5. ✅ **Scale when needed** - Upgrade to paid tier for more power

---

## 📚 Full Documentation

This guide is a quick overview. For complete step-by-step walkthroughs:

```
.claude/skills/render-backend-deployer/
├── QUICKSTART.md          ← You are here
├── workflows/
│   ├── 01-initial-setup.md
│   ├── 02-database-setup.md
│   ├── 03-environment-variables.md
│   ├── 04-cors-configuration.md
│   ├── 05-error-debugging.md
│   └── 06-frontend-integration.md
├── patterns/
│   ├── common-errors.md
│   ├── middleware-order.md
│   ├── environment-checklist.md
│   └── log-interpretation.md
└── examples/
    ├── render.yaml.example
    ├── config.py.example
    ├── main.py.example
    └── Dockerfile.example
```

**Each file is self-contained and can be read independently.**

---

## 🎉 Congratulations!

You've deployed a production-ready FastAPI application to Render.com!

Your backend is now:
- ✅ Running on Render's global infrastructure
- ✅ Auto-scaling and monitoring with health checks
- ✅ SSL/HTTPS encrypted
- ✅ Connected to your database
- ✅ Ready for your frontend to call

**What's next?**
- Add more endpoints to your FastAPI app
- Deploy additional features
- Monitor in Render dashboard
- Scale to paid tier if needed

Happy coding! 🚀
