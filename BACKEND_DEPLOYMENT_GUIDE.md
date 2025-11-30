# 🚀 BACKEND DEPLOYMENT GUIDE - DEPLOY IN 10 MINUTES

**Status**: Ready to deploy the RAG chatbot backend
**Goal**: Get your backend live so the ChatWidget works on GitHub Pages

---

## 🎯 OPTION 1: Railway.app (RECOMMENDED - EASIEST)

Railway is the fastest way to deploy. Free tier available.

### Step 1: Sign Up for Railway

1. Go to https://railway.app/
2. Click "Start a New Project"
3. Sign in with GitHub

### Step 2: Create New Project

1. Click "New Project"
2. Select "Deploy from GitHub repo"
3. Choose your repository: `92Bilal26/physical-ai-textbook`
4. Select the `main` branch

### Step 3: Configure Root Directory

1. In Railway dashboard, click on your service
2. Go to "Settings" tab
3. Set **Root Directory**: `chatbot-backend`
4. Click "Save"

### Step 4: Add Environment Variables

Click "Variables" tab and add these:

```bash
# Required Variables
DATABASE_URL=postgresql://user:password@host:5432/dbname
REDIS_URL=redis://default:password@host:6379
OPENAI_API_KEY=your-openai-api-key-here
QDRANT_URL=your-qdrant-url-here
QDRANT_API_KEY=your-qdrant-api-key-here

# Optional but Recommended
ENVIRONMENT=production
LOG_LEVEL=INFO
CORS_ORIGINS=https://92bilal26.github.io,https://92Bilal26.github.io
```

### Step 5: Add PostgreSQL Database

1. In Railway dashboard, click "New"
2. Select "Database" → "Add PostgreSQL"
3. Railway will auto-create `DATABASE_URL` variable
4. Your service will automatically link to it

### Step 6: Add Redis Cache

1. Click "New" → "Database" → "Add Redis"
2. Railway will auto-create `REDIS_URL` variable
3. Your service will automatically link to it

### Step 7: Deploy

1. Click "Deploy" button
2. Railway will:
   - Build Docker image
   - Run migrations
   - Start the server
3. Wait 2-3 minutes for deployment

### Step 8: Get Your Backend URL

1. Go to "Settings" tab
2. Click "Generate Domain"
3. Copy the URL (e.g., `https://your-app.railway.app`)
4. This is your `BACKEND_URL`

### Step 9: Update ChatWidget to Use Production Backend

You need to configure the ChatWidget to use your deployed backend.

**Option A: Add to Docusaurus Config** (Recommended)

Edit `book/docusaurus.config.ts`:

```typescript
export default {
  // ... existing config
  scripts: [
    {
      src: '/chatbot-config.js',
      async: false,
    }
  ],
  // ... rest of config
}
```

Create `book/static/chatbot-config.js`:

```javascript
// Configure ChatWidget to use production backend
window.__CHATBOT_API_URL__ = 'https://your-app.railway.app';
```

**Option B: Environment Variable at Build Time**

Create `book/.env.production`:

```bash
REACT_APP_CHATBOT_API_URL=https://your-app.railway.app
```

Update `book/src/components/ChatWidget/ChatWidget.tsx`:

```typescript
const API_URL = typeof window !== 'undefined'
  ? (window as any).__CHATBOT_API_URL__ || process.env.REACT_APP_CHATBOT_API_URL || 'http://localhost:8000'
  : 'http://localhost:8000';
```

### Step 10: Test Your Deployment

1. Visit your Railway URL: `https://your-app.railway.app/health`
2. Should see: `{"status": "healthy"}`
3. Visit: `https://your-app.railway.app/docs`
4. Should see FastAPI Swagger documentation

### Step 11: Test ChatWidget Integration

1. Rebuild Docusaurus with backend URL configured
2. Deploy to GitHub Pages
3. Visit: https://92Bilal26.github.io/physical-ai-textbook/
4. Click the 💬 button
5. Ask a question
6. You should get a response!

---

## 🎯 OPTION 2: Render.com (ALSO EASY)

### Step 1: Sign Up

1. Go to https://render.com/
2. Sign in with GitHub

### Step 2: Create Web Service

1. Click "New +"
2. Select "Web Service"
3. Connect your GitHub repo: `92Bilal26/physical-ai-textbook`

### Step 3: Configure Service

```
Name: rag-chatbot-backend
Root Directory: chatbot-backend
Environment: Docker
Region: Choose closest to you
Branch: main
```

### Step 4: Add Environment Variables

In "Environment" section:

```bash
DATABASE_URL=postgresql://...
REDIS_URL=redis://...
OPENAI_API_KEY=your-key
QDRANT_URL=your-url
QDRANT_API_KEY=your-key
ENVIRONMENT=production
CORS_ORIGINS=https://92bilal26.github.io
```

### Step 5: Add PostgreSQL Database

1. Go to Dashboard → "New +"
2. Select "PostgreSQL"
3. Copy the "Internal Database URL"
4. Add to your Web Service as `DATABASE_URL`

### Step 6: Add Redis

1. Go to Dashboard → "New +"
2. Select "Redis"
3. Copy the "Internal Redis URL"
4. Add to your Web Service as `REDIS_URL`

### Step 7: Deploy

1. Click "Create Web Service"
2. Render will build and deploy
3. Wait 5-10 minutes

### Step 8: Get Your URL

Your backend will be at:
`https://rag-chatbot-backend.onrender.com`

---

## 🎯 OPTION 3: Heroku (CLASSIC OPTION)

### Prerequisites

Install Heroku CLI:
```bash
# Windows
choco install heroku-cli

# Or download from https://devcenter.heroku.com/articles/heroku-cli
```

### Step 1: Login to Heroku

```bash
heroku login
```

### Step 2: Create Heroku App

```bash
cd chatbot-backend
heroku create rag-chatbot-backend
```

### Step 3: Add PostgreSQL

```bash
heroku addons:create heroku-postgresql:mini
```

### Step 4: Add Redis

```bash
heroku addons:create heroku-redis:mini
```

### Step 5: Set Environment Variables

```bash
heroku config:set OPENAI_API_KEY=your-key
heroku config:set QDRANT_URL=your-url
heroku config:set QDRANT_API_KEY=your-key
heroku config:set ENVIRONMENT=production
heroku config:set CORS_ORIGINS=https://92bilal26.github.io
```

### Step 6: Create Procfile

Already exists at `chatbot-backend/Procfile`:

```
web: uvicorn src.main:app --host 0.0.0.0 --port $PORT
release: alembic upgrade head
```

### Step 7: Deploy

```bash
git subtree push --prefix chatbot-backend heroku main
```

### Step 8: Get Your URL

```bash
heroku open
```

URL will be: `https://rag-chatbot-backend.herokuapp.com`

---

## ⚙️ ENVIRONMENT VARIABLES EXPLAINED

### Required Variables

**OPENAI_API_KEY**
- Get from: https://platform.openai.com/api-keys
- Format: `sk-...`
- Used for: Embeddings and chat completions

**QDRANT_URL**
- Get from: https://cloud.qdrant.io/
- Format: `https://xxx.qdrant.io`
- Used for: Vector similarity search

**QDRANT_API_KEY**
- Get from Qdrant dashboard
- Format: Long alphanumeric string
- Used for: Qdrant authentication

**DATABASE_URL**
- Auto-generated by Railway/Render/Heroku
- Format: `postgresql://user:pass@host:port/db`
- Used for: Storing conversations, sessions

**REDIS_URL**
- Auto-generated by Railway/Render/Heroku
- Format: `redis://default:pass@host:port`
- Used for: Query caching, rate limiting

### Optional Variables

**CORS_ORIGINS**
- Default: `*`
- Production: `https://92bilal26.github.io`
- Comma-separated for multiple origins

**ENVIRONMENT**
- Values: `development`, `production`
- Affects logging and debug mode

**LOG_LEVEL**
- Values: `DEBUG`, `INFO`, `WARNING`, `ERROR`
- Default: `INFO`

---

## 🧪 TESTING YOUR DEPLOYMENT

### 1. Health Check

```bash
curl https://your-backend-url.com/health
```

Expected response:
```json
{"status": "healthy"}
```

### 2. API Documentation

Visit: `https://your-backend-url.com/docs`

Should see interactive Swagger UI with all endpoints.

### 3. Test Query Endpoint

```bash
curl -X POST https://your-backend-url.com/api/query \
  -H "Content-Type: application/json" \
  -d '{
    "question": "What is ROS 2?",
    "session_id": "test-session",
    "context": "/docs/module-1"
  }'
```

Expected response:
```json
{
  "answer": "ROS 2 is...",
  "sources": [...],
  "confidence": 0.85,
  "message_id": "..."
}
```

### 4. Test from ChatWidget

1. Open browser DevTools (F12)
2. Go to https://92Bilal26.github.io/physical-ai-textbook/
3. Click 💬 button
4. Open "Network" tab in DevTools
5. Ask a question
6. Check request to your backend URL
7. Should see 200 response with answer

---

## 🐛 TROUBLESHOOTING

### Error: "CORS error"

**Problem**: Browser blocks request from GitHub Pages to backend

**Solution**: Add CORS origins to backend

In `chatbot-backend/src/main.py`:

```python
from fastapi.middleware.cors import CORSMiddleware

app.add_middleware(
    CORSMiddleware,
    allow_origins=[
        "https://92bilal26.github.io",
        "https://92Bilal26.github.io",  # Case variations
        "http://localhost:3000",  # For local dev
    ],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)
```

Redeploy after change.

### Error: "Connection refused" or "Network error"

**Problem**: ChatWidget can't reach backend

**Solution**:
1. Check backend is running: Visit `https://your-backend-url.com/health`
2. Check ChatWidget has correct URL configured
3. Check browser DevTools Console for errors

### Error: "OpenAI API key not found"

**Problem**: Missing environment variable

**Solution**:
1. Go to deployment platform dashboard
2. Add `OPENAI_API_KEY` to environment variables
3. Restart service

### Error: "Database connection failed"

**Problem**: DATABASE_URL incorrect or DB not running

**Solution**:
1. Check DATABASE_URL in environment variables
2. Make sure PostgreSQL addon is running
3. Check database logs in platform dashboard

### Error: "Rate limit exceeded"

**Problem**: Too many requests

**Solution**: This is normal! Backend rate limiter is working. Wait 60 seconds or clear session.

---

## 📊 POST-DEPLOYMENT CHECKLIST

After deploying, verify:

- [ ] Backend health endpoint responds: `/health`
- [ ] API docs accessible: `/docs`
- [ ] Database migrations ran successfully
- [ ] Redis connection working
- [ ] Qdrant connection working
- [ ] CORS configured for GitHub Pages
- [ ] Environment variables set correctly
- [ ] ChatWidget shows backend URL in DevTools
- [ ] Can ask questions and get responses
- [ ] Source citations appear
- [ ] Session persistence works

---

## 🎯 QUICK DEPLOYMENT SUMMARY

**Fastest Path (Railway):**

1. Sign up at Railway.app (2 min)
2. Connect GitHub repo (1 min)
3. Add PostgreSQL + Redis (2 min)
4. Set environment variables (3 min)
5. Deploy (2 min)
6. Update ChatWidget config (2 min)
7. Rebuild & redeploy GitHub Pages (5 min)

**Total time: ~15 minutes** ⏱️

---

## 📞 NEED HELP?

### Check Deployment Logs

**Railway**: Dashboard → Select service → "Logs" tab
**Render**: Dashboard → Service → "Logs" tab
**Heroku**: `heroku logs --tail`

### Common Issues

1. **Build fails**: Check Dockerfile and requirements.txt
2. **Migrations fail**: Check DATABASE_URL is correct
3. **App crashes**: Check environment variables
4. **CORS errors**: Add GitHub Pages to allowed origins

### Debugging Commands

```bash
# Check environment variables (Railway CLI)
railway variables

# View logs (Railway)
railway logs

# Check service status (Render)
# Use dashboard

# Restart service (Heroku)
heroku restart
```

---

## ✅ SUCCESS CRITERIA

Your deployment is successful when:

✅ Backend URL returns healthy status
✅ API docs load at `/docs`
✅ ChatWidget connects without CORS errors
✅ Questions return answers with citations
✅ No errors in browser console
✅ Session history persists

---

## 🚀 FINAL STEP: UPDATE CHATWIDGET

After deploying backend, you MUST update the ChatWidget to use your production URL.

**Create this file**: `book/static/chatbot-config.js`

```javascript
// Replace with your actual Railway/Render/Heroku URL
window.__CHATBOT_API_URL__ = 'https://your-actual-backend-url.railway.app';
```

**Update**: `book/docusaurus.config.ts`

```typescript
export default {
  // ... existing config
  scripts: [
    {
      src: '/chatbot-config.js',
      async: false,
    }
  ],
}
```

**Rebuild and deploy:**

```bash
cd book
npm run build
cd ..
git checkout gh-pages
rm -rf assets docs *.html
cp -r book/build/* .
git add -A
git commit -m "Update ChatWidget with production backend URL"
git push origin gh-pages
```

**Wait 2-3 minutes for GitHub Pages to update, then test!**

---

**Your backend will be live and the ChatWidget will be fully functional!** 🎉
