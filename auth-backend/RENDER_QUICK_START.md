# Deploy to Render - Quick Start (You Already Have Everything!)

Since you're already using Render for your chatbot service backend, deployment is **super simple**!

## ✅ What You Already Have

```
Your Render Account:
├── rag-chatbot-backend (running)
│   ├── Service: rag-chatbot-backend
│   ├── Database: rag-chatbot-db (PostgreSQL)
│   └── Cache: rag-chatbot-redis
│
└── (NEW) physical-ai-auth (ready to deploy)
    ├── Service: physical-ai-auth
    ├── Database: physical-ai-auth-db (PostgreSQL - auto-created)
    └── Configuration: render.yaml (already created!)
```

## 🚀 Deploy in 5 Steps

### Step 1: Push Code to GitHub
```bash
cd auth-backend
git add .
git commit -m "Add Better Auth backend with Render configuration"
git push origin 003-better-auth-user
```

### Step 2: Go to Render Dashboard
1. Open https://render.com/dashboard
2. Click "New" → "Web Service"

### Step 3: Connect GitHub
1. Click "Connect repository"
2. Select `physical-ai-textbook`
3. Choose branch: `003-better-auth-user`
4. Click "Connect"

### Step 4: Render Auto-Configures
**Render reads `render.yaml` and automatically sets up:**
- Service name: `physical-ai-auth`
- Runtime: Docker (uses Dockerfile)
- Database: PostgreSQL (physical-ai-auth-db)
- Health check: `/health`
- Environment variables: Pre-configured!

**Just click "Create Web Service"** - No manual configuration needed! ✨

### Step 5: Add Secrets
Before deploying, add these in Render Dashboard → Environment:

```
BETTER_AUTH_SECRET=8aUZWMstbytTyJ8KtQZ8TVjz4KetZo1y

# Optional - for OAuth later:
GITHUB_CLIENT_ID=
GITHUB_CLIENT_SECRET=
GOOGLE_CLIENT_ID=
GOOGLE_CLIENT_SECRET=
```

Then click **"Deploy"**!

## ⏱️ What Happens Next

Render automatically:
1. Clones GitHub repo
2. Builds Docker image (~2 min)
3. Creates PostgreSQL database (~1 min)
4. Runs migrations automatically
5. Starts service (~1 min)
6. Runs health check

**Total time: ~5-10 minutes**

## ✅ Verify It Works

Once deployed:

1. **Get Service URL:**
   - Render Dashboard → Services → physical-ai-auth
   - Copy the URL (e.g., `https://physical-ai-auth.onrender.com`)

2. **Test health endpoint:**
   ```bash
   curl https://physical-ai-auth.onrender.com/health

   # Response:
   {"status":"ok","timestamp":"2025-12-02T..."}
   ```

3. **Check logs:**
   - Render Dashboard → physical-ai-auth → Logs
   - Should show: "Auth server running on http://0.0.0.0:3001"

## 🔗 Use in Frontend

Update your frontend to use this URL:

```javascript
// frontend/src/api.js
export const API_URL = 'https://physical-ai-auth.onrender.com'

// Now use it:
async function signup(email, password) {
  const response = await fetch(`${API_URL}/api/auth/sign-up`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ email, password })
  })
  return response.json()
}
```

## 🔄 Automatic Redeploy on Changes

When you push changes:
```bash
git commit -m "Update auth features"
git push origin 003-better-auth-user
```

Render automatically:
1. Detects commit
2. Rebuilds
3. Redeploys
4. Zero downtime! ✨

## 📊 Side by Side

Your existing Render services:

```
Chatbot Service:
URL: https://rag-chatbot-backend.onrender.com
Port: 8000
Database: rag-chatbot-db
Cache: rag-chatbot-redis

Auth Service (NEW):
URL: https://physical-ai-auth.onrender.com
Port: 3001
Database: physical-ai-auth-db
(No cache needed)
```

Both running on the same Render account! 🎉

## 🆘 If Something Goes Wrong

**Check Render Logs:**
1. Dashboard → physical-ai-auth
2. Click "Logs" tab
3. Look for error messages

**Common fixes:**
- Build error → Check `npm run build` works locally
- Database error → Wait 2-3 min for DB creation
- Port issue → Render auto-assigns, should be fine
- Timeout → Check health endpoint locally

## 📝 Files Render Uses

```
auth-backend/
├── render.yaml           ← Render reads this
├── Dockerfile            ← Render builds this
├── package.json          ← Dependencies
├── src/                  ← Application code
└── migrations/           ← Database schema
```

**You don't need to do anything with these files** - Render handles everything!

## 🎯 Summary

**What I've done for you:**
- ✅ Created `render.yaml` with all Render configuration
- ✅ Created `Dockerfile` for containerization
- ✅ Configured PostgreSQL database auto-creation
- ✅ Set up automatic migrations
- ✅ Configured health checks
- ✅ Set up CORS for your frontend

**What you need to do:**
1. Push code to GitHub
2. Create Web Service in Render
3. Add 1-2 secret variables
4. Click Deploy
5. Wait 5-10 minutes
6. Done! 🎉

## 🚀 Ready to Deploy?

**Next step:**
```bash
cd auth-backend
git push origin 003-better-auth-user
```

Then go to https://render.com/dashboard and create the Web Service!

---

**Questions?** See `RENDER_DEPLOY.md` for detailed instructions and troubleshooting.

**Time to deploy:** 5 steps, ~5-10 minutes ⚡
