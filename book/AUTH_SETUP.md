# 🔐 Authentication Setup - Physical AI Textbook

This guide explains how to use the Better Auth system integrated into the Physical AI Textbook.

---

## Quick Start - Run Everything Locally

You need **2 terminal windows**:

### Terminal 1: Start the Backend

```bash
cd auth-backend
npm install  # First time only
npm run dev
```

You should see:
```
🚀 Auth server running on http://localhost:3001
```

### Terminal 2: Start the Book

```bash
cd book
npm install  # First time only
npm start
```

You should see:
```
[INFO] Docusaurus server started on http://localhost:3000
```

---

## Open in Browser

Go to: **http://localhost:3000**

You should see the Physical AI Textbook homepage with a new button:

**🔐 Sign In / Sign Up**

Click it! A modal will appear.

---

## Test the Authentication Flow

### 1. Sign Up
```
Email: your@email.com
Password: Password123!
Name: Your Name
```
Click "Create Account"

### 2. See Success Message
Should show: ✅ Account created! Signing you in...

### 3. Button Changes
The button now shows: 👤 your@email.com

Click it again to see your session info.

### 4. Sign Out
Click the red "Sign Out" button

### 5. Button Resets
Button goes back to: 🔐 Sign In / Sign Up

---

## Features

✅ **Sign Up**: Create new account with email/password/name
✅ **Sign In**: Login with email and password
✅ **Session**: View current user info
✅ **Sign Out**: Logout and clear session
✅ **Modal**: Beautiful popup interface
✅ **State Management**: User stays logged in
✅ **Responsive**: Works on mobile too

---

## Test Data (Pre-filled)

Quick testing without typing:
```
Email: test@example.com
Password: Test123!@
Name: Test User
```

---

## How It Works

1. **Frontend** (React component in `book/src/components/AuthModal.tsx`)
   - Beautiful modal with signup/signin forms
   - Connects to backend at `http://localhost:3001`
   - Stores session in browser cookies

2. **Backend** (Express server in `auth-backend/src/`)
   - Handles user signup and signin
   - Manages sessions
   - Uses PostgreSQL database

3. **Database** (PostgreSQL in Docker)
   - Stores users
   - Stores sessions
   - Auto-created by startup script

---

## Components Added to Book

### AuthModal.tsx
- Signup form
- Signin form
- Session display
- User info display
- Fully responsive

### Updated index.tsx
- Added auth button to homepage
- Shows user name when logged in
- Opens modal on click

---

## API Endpoints Used

The frontend uses these backend endpoints:

```
POST /api/auth/sign-up
POST /api/auth/sign-in
POST /api/auth/sign-out
GET  /api/auth/session
```

All configured to run on `http://localhost:3001`

---

## Environment Configuration

The auth URL is configured in:
```
book/src/components/AuthModal.tsx
Line 21: const API_URL = process.env.REACT_APP_AUTH_URL || 'http://localhost:3001';
```

For production, set:
```
REACT_APP_AUTH_URL=https://physical-ai-auth-xxxx.onrender.com
```

---

## Troubleshooting

### "Cannot connect to backend"
1. Is backend running? (`npm run dev` in `auth-backend` folder)
2. Is PostgreSQL running? (`docker ps` should show container)
3. Check browser console (F12) for errors

### "Sign up/sign in not working"
1. Check backend is responding: Visit `http://localhost:3001/health`
2. Should show: `{"status":"ok",...}`
3. Check browser console for network errors

### Button not appearing
1. Save and rebuild book: `npm start`
2. Clear browser cache (Ctrl+Shift+Delete)
3. Refresh page

### Modal not opening
1. Check JavaScript console for errors (F12)
2. Try opening in different browser
3. Clear localStorage: `localStorage.clear()`

---

## Files Modified

**Added:**
- `book/src/components/AuthModal.tsx` - Auth component
- `book/src/components/AuthModal.module.css` - Auth styles

**Modified:**
- `book/src/pages/index.tsx` - Added auth button
- `book/src/pages/index.module.css` - Added button styles

---

## Next Steps

1. ✅ Run backend: `npm run dev` in `auth-backend`
2. ✅ Run book: `npm start` in `book`
3. ✅ Click "Sign In / Sign Up" button
4. ✅ Test signup and signin
5. ✅ Verify user data appears
6. ✅ Sign out

---

## For Production Deployment

After testing locally:

1. Update `REACT_APP_AUTH_URL` to production backend URL
2. Deploy book to GitHub Pages or Render
3. Deploy backend to Render
4. Update CORS settings in backend
5. Test authentication end-to-end

See `DEPLOY_NOW.md` in `auth-backend` for production deployment.

---

## Technology Stack

- **Frontend**: React + TypeScript (Docusaurus)
- **Backend**: Express.js + Better Auth
- **Database**: PostgreSQL
- **Auth**: Better Auth (email/password)
- **UI**: CSS Modules (responsive design)

---

## Questions?

Check these files:
- `auth-backend/LOCAL_SETUP_GUIDE.md` - Backend setup
- `auth-backend/RUN_LOCALLY.md` - Local development
- `auth-backend/DEPLOYMENT_SUMMARY.md` - Feature overview

---

**Ready? Start the backend and book, then click the "Sign In / Sign Up" button!** 🚀
