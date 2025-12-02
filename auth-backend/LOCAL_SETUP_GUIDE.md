# Better Auth - Local Development Setup Guide

This guide will help you run the Better Auth backend locally with a test frontend, so you can develop and test authentication features before deploying to production.

---

## What You'll Get

✅ Local PostgreSQL database
✅ Running auth backend on http://localhost:3001
✅ Beautiful test UI on http://localhost:3001/test-client.html
✅ Full authentication flow: signup → signin → profile
✅ Ready to test everything before production deployment

---

## Prerequisites

- **Node.js** 20.x or higher
- **Docker** (easiest) OR local PostgreSQL installation
- **Git** (already have this)
- A text editor (VS Code recommended)

Check versions:
```bash
node --version  # Should be v20+
npm --version
docker --version  # If using Docker
```

---

## Step 1: Start PostgreSQL Database

### Option A: Docker (Recommended - Easiest)

```bash
docker run --name physical-ai-auth-db \
  -e POSTGRES_USER=authuser \
  -e POSTGRES_PASSWORD=authpass123 \
  -e POSTGRES_DB=physical_ai_auth \
  -p 5432:5432 \
  -d postgres:16-alpine
```

Verify it's running:
```bash
docker ps
# You should see: physical-ai-auth-db
```

### Option B: Local PostgreSQL

See `START_LOCAL_DB.md` for detailed instructions on setting up local PostgreSQL.

---

## Step 2: Install Dependencies

```bash
cd auth-backend
npm install
```

This installs:
- Express.js (web framework)
- Better Auth (authentication library)
- Drizzle ORM (database access)
- PostgreSQL driver
- And more...

Expected output:
```
added 554 packages, and audited 555 packages in X seconds
```

---

## Step 3: Verify Environment Configuration

Check `.env` file has correct database URL:

```bash
cat .env
```

Should show:
```
DATABASE_URL=postgresql://authuser:authpass123@localhost:5432/physical_ai_auth
PORT=3001
NODE_ENV=development
FRONTEND_URL=http://localhost:3000,http://localhost:5173
BETTER_AUTH_SECRET=dev-secret-change-in-production-32-chars-min
```

If not, update it:
```bash
# On Windows
notepad .env

# On Mac/Linux
nano .env
```

---

## Step 4: Run the Backend

```bash
npm run dev
```

You should see output like:

```
> physical-ai-auth@0.1.0 dev
> ts-node src/index.ts

Starting auth server...
ℹ️  Environment check:
   - PORT: 3001
   - FRONTEND_URL: http://localhost:3000,http://localhost:5173
   - NODE_ENV: development
   - BETTER_AUTH_SECRET: ✅ Set
   - DATABASE_URL: ✅ Set

📡 Validating database connection...
✅ Database connection successful
✅ Setting up user profile schema...
✅ Database schema initialized successfully

🚀 Auth server running on http://localhost:3001
📚 API Routes:
   - POST /api/auth/sign-up
   - POST /api/auth/sign-in
   - POST /api/auth/sign-out
   - GET /api/auth/session
   - GET /api/users/profile
   - PUT /api/users/profile
   - GET /health
```

**✅ If you see "Auth server running" - it's working!**

---

## Step 5: Test with the Browser UI

Open this file in your browser:
```
auth-backend/test-client.html
```

**Or use the direct path:**
```
file:///F:/ai_projects/physical-ai-textbook/auth-backend/test-client.html
```

You should see a beautiful blue interface with:
- Backend Status checker
- Sign Up form
- Sign In form
- Session info
- User Profile section

---

## Step 6: Test the Authentication Flow

### 1. Check Backend Status
Click "Check Health" button
- Should show ✅ Connected
- Shows backend is running

### 2. Sign Up
Fill in the form:
- Email: `test@example.com`
- Name: `Test User`
- Password: `Test123!@`
- Click "Sign Up"

Should see: ✅ Account created!

### 3. Sign In
Click "Sign In" button
- Uses same email/password from signup
- Should see: ✅ Signed in successfully!

### 4. View Session
Click "Get Session" button
- Shows your user ID, email, name
- This proves you're authenticated

### 5. Initialize Profile
Scroll down to "Initialize Profile"
- Dev Experience: `beginner`
- Robotics Background: `beginner`
- Programming Languages: `Python, JavaScript`
- Click "Initialize Profile"

### 6. View Profile
Click "Get Profile"
- Shows your personalization data
- Proves profile was saved to database

### 7. Sign Out
Click "Sign Out" button
- Clears session
- You'll need to sign in again to use protected routes

---

## API Endpoints Reference

All these endpoints work with your test client!

### Auth Routes (Public)
```
POST /api/auth/sign-up
  Body: { email, password, name }
  Returns: { user, session }

POST /api/auth/sign-in
  Body: { email, password }
  Returns: { user, session }

POST /api/auth/sign-out
  No body needed
  Clears session

GET /api/auth/session
  Returns: Current session or null
```

### User Profile Routes (Protected - Need Auth)
```
GET /api/users/profile
  Returns: User's profile or 404

PUT /api/users/profile
  Body: { softwareDevelopmentExperience, roboticsHardwareBackground, ... }
  Returns: Updated profile

POST /api/users/profile/init
  Body: { softwareDevelopmentExperience, roboticsHardwareBackground, programmingLanguages }
  Returns: Newly created profile
```

### Health Check
```
GET /health
  Returns: { status: "ok", timestamp: "..." }
```

---

## Testing with curl (Advanced)

If you prefer command line:

### Sign Up
```bash
curl -X POST http://localhost:3001/api/auth/sign-up \
  -H "Content-Type: application/json" \
  -d '{
    "email": "test@example.com",
    "password": "Test123!@",
    "name": "Test User"
  }'
```

### Sign In
```bash
curl -X POST http://localhost:3001/api/auth/sign-in \
  -H "Content-Type: application/json" \
  -d '{
    "email": "test@example.com",
    "password": "Test123!@"
  }'
```

### Get Session
```bash
curl http://localhost:3001/api/auth/session
```

---

## Troubleshooting

### Error: Cannot connect to backend
```
❌ Cannot reach backend at http://localhost:3001
```

**Solutions:**
1. Is PostgreSQL running?
   ```bash
   docker ps
   # Should show: physical-ai-auth-db
   ```

2. Is backend running?
   - Check terminal - should see "Auth server running"
   - Make sure `npm run dev` is still running

3. Check backend is on right port:
   ```bash
   # Mac/Linux
   lsof -i :3001

   # Windows
   netstat -ano | findstr :3001
   ```

### Error: Database connection failed
```
❌ Database connection failed: ECONNREFUSED
```

**Solutions:**
1. Start PostgreSQL:
   ```bash
   docker start physical-ai-auth-db
   ```

2. Check database is running:
   ```bash
   docker ps
   ```

3. Verify connection string in `.env`:
   ```
   DATABASE_URL=postgresql://authuser:authpass123@localhost:5432/physical_ai_auth
   ```

### Error: "Cannot find module 'express'"
```
Error: Cannot find module 'express'
```

**Solution:**
```bash
npm install
npm run dev
```

### Port 5432 Already in Use
```
Error: listen EADDRINUSE :::5432
```

**Solution - Option 1: Use different port**
```bash
docker run ... -p 5433:5432 postgres:16-alpine
# Update .env: DATABASE_URL=postgresql://authuser:authpass123@localhost:5433/...
```

**Solution - Option 2: Kill existing process**
```bash
# Mac/Linux
lsof -i :5432 | grep LISTEN | awk '{print $2}' | xargs kill -9

# Windows - Find and stop Docker container
docker stop <container-id>
```

### CORS Error in Browser
```
Access to XMLHttpRequest blocked by CORS policy
```

**This should not happen** because we set:
```
FRONTEND_URL=http://localhost:3000,http://localhost:5173
```

But if it does:
1. Make sure you're accessing from localhost (not 127.0.0.1)
2. Check `.env` has correct FRONTEND_URL
3. Restart backend: Ctrl+C and `npm run dev` again

---

## File Structure

```
auth-backend/
├── src/
│   ├── index.ts              ← Main server
│   ├── auth/index.ts         ← Authentication setup
│   ├── db/
│   │   ├── index.ts          ← Database connection
│   │   ├── schema.ts         ← Database tables
│   │   └── userProfileSchema.ts  ← User profile functions
│   └── routes/
│       └── userProfile.ts    ← API routes
├── dist/                     ← Compiled JavaScript
├── .env                      ← Configuration (don't commit!)
├── package.json              ← Dependencies
├── tsconfig.json             ← TypeScript config
├── Dockerfile                ← Docker build
├── test-client.html          ← 👈 TEST UI (Open in browser)
└── LOCAL_SETUP_GUIDE.md      ← This file!
```

---

## Next: Connect to Your Physical AI Textbook Frontend

Once local testing works, you can integrate with the book frontend:

1. **Add auth SDK to your book frontend** (React, Vue, etc.)
2. **Import auth client**:
   ```javascript
   const auth = new BetterAuth({
     baseURL: 'http://localhost:3001'
   });
   ```

3. **Use in your components**:
   ```javascript
   // Sign up
   const { user, session } = await auth.signUp({
     email: 'user@example.com',
     password: 'password',
     name: 'User Name'
   });

   // Sign in
   const { user, session } = await auth.signIn({
     email: 'user@example.com',
     password: 'password'
   });

   // Get session
   const session = await auth.getSession();

   // Sign out
   await auth.signOut();
   ```

---

## Production Deployment (After Testing)

Once everything works locally:

1. ✅ Verify all features work (signup, signin, profile)
2. ✅ Test with actual data
3. ✅ Check logs for any issues
4. ✅ Then deploy to Render using `DEPLOY_NOW.md`

---

## Quick Reference

| Command | Purpose |
|---------|---------|
| `docker run ...` | Start PostgreSQL database |
| `npm install` | Install dependencies |
| `npm run dev` | Run backend locally |
| `npm run build` | Compile TypeScript |
| `docker stop physical-ai-auth-db` | Stop database |
| `docker ps` | Check running containers |

---

## Support

- Check logs in terminal running `npm run dev`
- Check browser console (F12) for JavaScript errors
- Test with `test-client.html` UI first
- If still stuck, check `.env` file is correct

**You're all set! Start with Step 1 and follow the guide.** 🚀
