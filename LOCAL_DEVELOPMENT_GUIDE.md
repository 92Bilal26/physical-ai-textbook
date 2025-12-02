# 🚀 Complete Local Development Guide

Run the entire Physical AI Textbook with Authentication locally!

---

## What You'll Have

✅ **Backend**: Better Auth server with user management
✅ **Frontend**: Book with integrated signup/signin button
✅ **Database**: PostgreSQL database
✅ **Testing**: Beautiful auth modal in the book
✅ **Full auth flow**: Signup → Signin → Profile → Signout

---

## Prerequisites

- **Node.js** 20.x or higher
- **Docker** (for PostgreSQL)
- **2 Terminal Windows** (one for backend, one for book)

Check you have them:
```bash
node --version    # Should be v20+
docker --version
```

---

## Step 1️⃣: Start the Backend

**Terminal 1:**

```bash
cd auth-backend
npm install
npm run dev
```

Wait for this message:
```
🚀 Auth server running on http://localhost:3001
```

---

## Step 2️⃣: Start the Book

**Terminal 2:**

```bash
cd book
npm install
npm start
```

Wait for this message:
```
[INFO] Docusaurus server started on http://localhost:3000
```

---

## Step 3️⃣: Open in Browser

Go to: **http://localhost:3000**

You should see the Physical AI Textbook homepage with:
- "Start Learning 📚" button
- **NEW: "🔐 Sign In / Sign Up" button**

---

## Step 4️⃣: Test Authentication

### Click the "🔐 Sign In / Sign Up" Button

A beautiful modal appears with:
- **Sign Up** tab (create new account)
- **Sign In** tab (login)
- Pre-filled test data

### Try Sign Up:
```
Email: test@example.com
Password: Test123!@
Name: Test User
```

The form will:
1. Connect to backend at http://localhost:3001
2. Create user in PostgreSQL
3. Establish session
4. Show success message
5. Close modal
6. Button changes to "👤 test@example.com"

### Click Button Again:
Shows your session info:
- Email
- User ID
- "Sign Out" button

### Try Sign In:
1. Sign out first
2. Enter email/password
3. Gets logged in
4. Session appears

### View Your Profile:
After signup/signin, user data is stored:
- In PostgreSQL database
- In browser session
- Button shows your name

---

## What's Running

```
┌─────────────────────────────────────┐
│      Browser (localhost:3000)       │
│                                     │
│  Physical AI Textbook (Docusaurus) │
│  ├── Homepage with auth button      │
│  ├── AuthModal component (React)    │
│  └── Beautiful signup/signin forms  │
└─────────────┬───────────────────────┘
              │ HTTP
              ▼
┌─────────────────────────────────────┐
│   Backend (localhost:3001)          │
│                                     │
│  Express.js + Better Auth           │
│  ├── POST /api/auth/sign-up         │
│  ├── POST /api/auth/sign-in         │
│  ├── POST /api/auth/sign-out        │
│  └── GET /api/auth/session          │
└─────────────┬───────────────────────┘
              │ TCP
              ▼
┌─────────────────────────────────────┐
│   PostgreSQL (localhost:5432)       │
│                                     │
│  Database in Docker                 │
│  ├── users table                    │
│  ├── sessions table                 │
│  └── user_profiles table            │
└─────────────────────────────────────┘
```

---

## File Structure

```
physical-ai-textbook/
│
├── auth-backend/                 ← Backend
│   ├── src/
│   │   ├── index.ts             (Express server)
│   │   ├── auth/                (Better Auth setup)
│   │   ├── db/                  (Database layer)
│   │   └── routes/              (API endpoints)
│   ├── .env                     (Configuration)
│   ├── package.json             (Dependencies)
│   ├── START.bat                (Windows starter)
│   └── start.sh                 (Mac/Linux starter)
│
├── book/                         ← Frontend
│   ├── src/
│   │   ├── pages/
│   │   │   └── index.tsx        (Homepage with auth button)
│   │   └── components/
│   │       ├── AuthModal.tsx    (NEW: Auth component)
│   │       └── AuthModal.module.css
│   ├── package.json
│   ├── docusaurus.config.ts
│   └── AUTH_SETUP.md            (Integration guide)
│
└── LOCAL_DEVELOPMENT_GUIDE.md    (This file!)
```

---

## Testing Checklist

✅ **Backend checks:**
- [ ] `npm run dev` shows "Auth server running"
- [ ] Visit http://localhost:3001/health → shows status
- [ ] PostgreSQL running (docker ps shows container)

✅ **Frontend checks:**
- [ ] `npm start` shows Docusaurus message
- [ ] Book loads at http://localhost:3000
- [ ] Auth button visible on homepage
- [ ] Clicking button opens modal

✅ **Auth flow checks:**
- [ ] Can sign up with new email
- [ ] Success message appears
- [ ] Button shows user name
- [ ] Can click button again and see session
- [ ] Can sign out
- [ ] Button resets

✅ **Data flow checks:**
- [ ] User data saved in database
- [ ] Session cookie set in browser
- [ ] Modal closes on success
- [ ] Error messages show on failure

---

## Common Issues & Fixes

### Backend won't start

**Error**: `Cannot find module 'express'`
```bash
cd auth-backend
npm install
npm run dev
```

**Error**: `listen EADDRINUSE :::3001`
```bash
# Port 3001 already in use
docker stop physical-ai-auth-db  # Stop other services
# Or use different port - see port config in .env
```

**Error**: `DATABASE_URL not set`
```bash
# Check .env file has DATABASE_URL
cat .env
# Should show: postgresql://authuser:authpass123@localhost:5432/physical_ai_auth
```

### Book won't start

**Error**: `Cannot find module '@docusaurus/...'`
```bash
cd book
npm install
npm start
```

**Error**: `Port 3000 in use`
```bash
# Kill process on port 3000
lsof -i :3000 | grep LISTEN | awk '{print $2}' | xargs kill -9
# Or it will start on next available port
```

### Auth button doesn't work

**Error**: `Cannot connect to backend`
1. Check backend is running (Terminal 1)
2. Check PostgreSQL running: `docker ps`
3. Browser console (F12) for error details

**Error**: `ReferenceError: process is not defined`
- This has been fixed in latest code
- Clear browser cache (Ctrl+Shift+Delete)
- Refresh page

**Error**: Modal won't open
- Clear localStorage: Open DevTools → Console → `localStorage.clear()`
- Refresh page
- Try different browser

### Database connection failed

**Error**: `ECONNREFUSED` when signing up/in
1. PostgreSQL not running
   ```bash
   docker ps
   # Should show: physical-ai-auth-db
   ```

2. Start PostgreSQL:
   ```bash
   docker start physical-ai-auth-db
   ```

3. Wait 5 seconds for database to initialize

4. Try again

---

## Useful Commands

| Command | Purpose |
|---------|---------|
| `docker ps` | Check if PostgreSQL is running |
| `docker logs physical-ai-auth-db` | See database logs |
| `npm run dev` (in auth-backend) | Start backend |
| `npm start` (in book) | Start book frontend |
| `npm run build` (in book) | Build for production |
| `localhost:3001/health` | Check backend health |

---

## Architecture Overview

### Frontend (React Component)
```
AuthModal.tsx
├── State: email, password, name, user
├── Mode: signin or signup
├── API calls to backend
└── Displays modal with forms
```

### Backend (Express Server)
```
index.ts
├── Express app setup
├── Better Auth initialization
├── Database connection
├── CORS configuration
└── Route handlers
```

### Database (PostgreSQL)
```
physical_ai_auth
├── users (id, email, name, emailVerified, image)
├── sessions (id, userId, token, expiresAt)
├── accounts (for OAuth)
└── verification_tokens
```

---

## Real-Time Testing

While running locally, you can:

1. **Edit backend code** → Restart `npm run dev` → Changes apply
2. **Edit frontend code** → Browser auto-refreshes → Changes apply
3. **Check logs** → Terminal shows request/error logs
4. **Debug** → Browser DevTools (F12) shows network requests

---

## Next Steps

Once everything works locally:

1. **Verify all features work**
   - Signup
   - Signin
   - View session
   - Signout
   - Clear session

2. **Test error handling**
   - Wrong password
   - Email already exists
   - Missing fields

3. **Check database**
   ```bash
   docker exec -it physical-ai-auth-db psql -U authuser -d physical_ai_auth -c "SELECT * FROM users;"
   ```

4. **Ready for production?**
   - Yes! Follow `auth-backend/DEPLOY_NOW.md`
   - Deploy backend to Render
   - Deploy book to GitHub Pages
   - Connect to production backend

---

## Production Deployment

After local testing works:

### Backend
```bash
cd auth-backend
# Follow: DEPLOY_NOW.md
```

### Frontend (Book)
```bash
cd book
# Update API URL to production backend
# Deploy to GitHub Pages or Render
```

See `auth-backend/DEPLOY_NOW.md` for full production guide.

---

## Recap - Quick Start

**Terminal 1:**
```bash
cd auth-backend && npm install && npm run dev
```

**Terminal 2:**
```bash
cd book && npm install && npm start
```

**Browser:**
```
http://localhost:3000
```

**Then:**
- Click "🔐 Sign In / Sign Up" button
- Test signup
- Test signin
- Test signout

**Done!** 🎉

---

## Support

For issues:
1. Check error messages in console
2. Read troubleshooting section above
3. Check `auth-backend/LOCAL_SETUP_GUIDE.md`
4. Check `book/AUTH_SETUP.md`

**You've got everything you need!** 🚀
