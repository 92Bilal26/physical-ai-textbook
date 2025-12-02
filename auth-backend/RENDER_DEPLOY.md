# Deploy Better Auth Backend to Render.com

Since you're already using Render for your chatbot service backend, this guide shows how to deploy the Better Auth backend to the same Render account.

## 📋 Overview

You have:
- ✅ `render.yaml` - Blueprint configuration (already created)
- ✅ `Dockerfile` - Container configuration (already created)
- ✅ All source code ready

This makes deployment straightforward!

## 🚀 Deployment Steps

### Step 1: Push Code to GitHub

First, push the auth-backend to your GitHub repository:

```bash
cd auth-backend

# If not already in git
git init
git add .
git commit -m "Add Better Auth backend with Render configuration"
git push origin 003-better-auth-user
```

### Step 2: Login to Render.com

1. Go to https://render.com
2. Sign in with your existing account (same one as chatbot-backend)
3. Go to Dashboard

### Step 3: Create Web Service from Blueprint

**Option A: Using GitHub Integration (Recommended)**

1. Click "New" → "Web Service"
2. Select "Connect a repository"
3. Choose your repository with auth-backend code
4. Select "physical-ai-textbook" repository
5. Click "Connect"

**Option B: Using Blueprint**

1. Click "New" → "Blueprint"
2. Use existing GitHub repository
3. Select your repo
4. Render will auto-detect `render.yaml`

### Step 4: Render Configuration

When creating the service, Render will show:

```
Service Name: physical-ai-auth
Runtime: Docker
Root Directory: ./auth-backend
Build Command: (automatic - uses Dockerfile)
Start Command: (automatic - uses Dockerfile)
```

**The render.yaml handles all of this automatically!**

### Step 5: Database Setup

The `render.yaml` automatically creates a PostgreSQL database:

```yaml
databases:
  - name: physical-ai-auth-db
    databaseName: physical_ai_auth
    user: authuser
    plan: free
```

Render will:
- ✅ Create the database
- ✅ Set connection string automatically
- ✅ Inject into `DATABASE_URL` environment variable
- ✅ Run migrations automatically

### Step 6: Set Environment Variables

Before deploying, set these secrets in Render Dashboard:

**In Render Service Dashboard → Environment:**

```
BETTER_AUTH_SECRET=8aUZWMstbytTyJ8KtQZ8TVjz4KetZo1y

GITHUB_CLIENT_ID=your_github_id
GITHUB_CLIENT_SECRET=your_github_secret

GOOGLE_CLIENT_ID=your_google_id
GOOGLE_CLIENT_SECRET=your_google_secret
```

(Leave blank if you don't have OAuth credentials yet)

### Step 7: Deploy

Click "Create Web Service" or "Deploy"

Render will:
1. ✅ Clone your GitHub repository
2. ✅ Build Docker image
3. ✅ Create PostgreSQL database
4. ✅ Run migrations
5. ✅ Start web service
6. ✅ Run health checks

**Deployment time: ~5-10 minutes**

### Step 8: Verify Deployment

Once deployed:

1. **Find your service URL:**
   - Render Dashboard → Services → physical-ai-auth
   - Look for "Service URL" (e.g., https://physical-ai-auth.onrender.com)

2. **Test health endpoint:**
   ```bash
   curl https://physical-ai-auth.onrender.com/health

   # Should return:
   # {"status":"ok","timestamp":"2025-12-02T..."}
   ```

3. **Check logs:**
   - In Render Dashboard → Logs tab
   - Should show:
     - "Setting up user profile schema..."
     - "Database schema initialized successfully"
     - "Auth server running on http://0.0.0.0:3001"

## 🔄 Automatic Migrations

The `render.yaml` includes:

```yaml
healthCheckPath: /health
```

This tells Render your app is ready after `/health` returns 200.

The app automatically runs migrations on startup via `src/index.ts`:

```typescript
async function initializeDatabase() {
  await setupUserProfileSchema();
  // Migrations run here
}
```

## 🌍 Update Frontend URL

Update `ALLOWED_ORIGINS` in render.yaml to match your frontend:

```yaml
ALLOWED_ORIGINS: http://localhost:3000,https://92bilal26.github.io
```

## 📊 Comparing with Chatbot Backend

Your chatbot backend uses:
```yaml
services:
  - name: rag-chatbot-backend
    dockerfilePath: ./Dockerfile
    databases:
      - name: rag-chatbot-db
      - name: rag-chatbot-redis
```

Better Auth backend uses:
```yaml
services:
  - name: physical-ai-auth
    dockerfilePath: ./Dockerfile
    databases:
      - name: physical-ai-auth-db
```

Same pattern, just simpler (no Redis needed).

## 🔧 Troubleshooting

### Deployment Failed

Check logs in Render Dashboard:
1. Click service → Logs tab
2. Look for error messages
3. Common issues:
   - `DATABASE_URL not set` - Database creation failed
   - `Port already in use` - Change PORT in .env
   - `TypeScript error` - Check build logs

**Fix:**
1. Check logs
2. Fix issue
3. Push to GitHub
4. Render auto-redeploys

### Health Check Failing

If health check fails:
1. Service might not be starting
2. Check logs for startup errors
3. Verify `/health` endpoint is working

```bash
# Test locally first
npm run dev
curl http://localhost:3001/health
```

### Database Not Connecting

If DATABASE_URL is missing:
1. Check Render dashboard - is database shown?
2. Wait 2-3 minutes - databases take time to initialize
3. Manually set DATABASE_URL if needed:
   - In Render Dashboard → Environment
   - Add: `DATABASE_URL=postgresql://...`

### Build Failures

If Docker build fails:
1. Check `npm install` works locally: `npm install`
2. Check `npm run build` works: `npm run build`
3. Check Dockerfile syntax
4. View build logs in Render Dashboard

## 📈 After Deployment

### Frontend Integration

Update your frontend to use the Render URL:

```javascript
// Before (local)
const API_URL = 'http://localhost:3001'

// After (production)
const API_URL = 'https://physical-ai-auth.onrender.com'
```

### Monitor Usage

In Render Dashboard:
- View logs real-time
- Check build history
- Monitor resource usage
- See deployment history

### Redeploy on Changes

When you push code to GitHub:
```bash
git commit -m "Update auth features"
git push origin 003-better-auth-user
```

Render automatically:
1. Detects new commit
2. Rebuilds Docker image
3. Redeploys service
4. Zero-downtime updates

## 🔐 Security Notes

For production:

1. **Generate strong BETTER_AUTH_SECRET:**
   ```bash
   node -e "console.log(require('crypto').randomBytes(32).toString('hex'))"
   ```

2. **Use environment-specific URLs:**
   - Development: `localhost`
   - Production: `render.com` domain

3. **Enable OAuth securely:**
   - Store credentials in Render secrets (not in code)
   - Use OAuth redirect URIs

4. **Database backups:**
   - Render PostgreSQL has automatic backups
   - Can be restored from dashboard

## 📚 Reference

- **render.yaml path:** `auth-backend/render.yaml`
- **Dockerfile path:** `auth-backend/Dockerfile`
- **Render docs:** https://render.com/docs
- **Better Auth docs:** https://better-auth.com/docs

## 🎯 Next Steps

1. **Push code:**
   ```bash
   git add .
   git commit -m "Add Render configuration"
   git push origin 003-better-auth-user
   ```

2. **Create Web Service in Render Dashboard**
   - GitHub repo → physical-ai-textbook
   - Branch → 003-better-auth-user

3. **Set environment variables in Render**
   - BETTER_AUTH_SECRET
   - OAuth credentials (if available)

4. **Deploy**
   - Click "Deploy"
   - Wait for completion (~5-10 min)
   - Test health endpoint

5. **Update Frontend**
   - Change API_URL to Render domain
   - Test authentication

That's it! Your auth backend is now on Render, same as your chatbot service! 🚀

---

**Questions?** Check the Render dashboard logs or refer to:
- Render docs: https://render.com/docs
- This guide's troubleshooting section
