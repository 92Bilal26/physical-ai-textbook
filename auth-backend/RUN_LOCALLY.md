# 🚀 Run Better Auth Locally - Quick Start

This is the **fastest way** to get Better Auth running on your machine with a beautiful test UI!

---

## 🎯 One Command to Start Everything

### Windows Users
```bash
START.bat
```

### Mac/Linux Users
```bash
bash start.sh
```

That's it! The script will:
1. ✅ Start PostgreSQL database (or use existing)
2. ✅ Install dependencies
3. ✅ Start the backend server
4. ✅ Show you where to open the test UI

---

## What Happens When You Run It

```
╔════════════════════════════════════════════════════════════╗
║         Better Auth - Local Development Starter            ║
║          Physical AI Textbook Authentication               ║
╚════════════════════════════════════════════════════════════╝

✅ PostgreSQL started!
📦 Installing dependencies...
🚀 Starting Better Auth Backend...

    Backend URL: http://localhost:3001
    Test UI: Open test-client.html in your browser

📖 For detailed guide, see: LOCAL_SETUP_GUIDE.md
```

---

## 🌐 Open the Test UI

After the script starts the backend, open this file in your browser:

```
auth-backend/test-client.html
```

**Or use the direct path:**
```
file:///F:/ai_projects/physical-ai-textbook/auth-backend/test-client.html
```

You'll see a beautiful interface to:
- ✅ Check backend status
- ✅ Sign up new accounts
- ✅ Sign in with email/password
- ✅ View your session
- ✅ Create user profile
- ✅ Test all authentication features

---

## 📋 Pre-filled Test Data

The test UI comes with default values to make testing easy:

### Sign Up (Default Values)
- **Email**: test@example.com
- **Name**: Test User
- **Password**: Test123!@

### User Profile
- **Dev Experience**: beginner
- **Robotics Background**: beginner
- **Languages**: Python, JavaScript

Just click the buttons - no need to type anything!

---

## 🧪 Full Testing Flow (5 minutes)

1. **Start the script** (Windows: `START.bat` or Mac/Linux: `bash start.sh`)

2. **Wait for startup message** showing "Auth server running"

3. **Open test-client.html** in your browser

4. **Click "Check Health"** - should show ✅ Connected

5. **Click "Sign Up"** - creates new account

6. **Click "Sign In"** - logs in with credentials

7. **Click "Get Session"** - shows you're authenticated

8. **Scroll down and click "Initialize Profile"** - sets user preferences

9. **Click "Get Profile"** - shows your saved profile

10. **Click "Sign Out"** - logs out

**If all steps work - your auth system is ready! 🎉**

---

## 📂 What's Included

```
auth-backend/
├── START.bat              👈 Click this (Windows)
├── start.sh               👈 Run this (Mac/Linux)
├── test-client.html       👈 Open in browser
├── LOCAL_SETUP_GUIDE.md   👈 Detailed guide
├── START_LOCAL_DB.md      👈 Database help
│
├── src/
│   ├── index.ts           (Express server)
│   ├── auth/index.ts      (Auth setup)
│   ├── db/                (Database layer)
│   └── routes/            (API endpoints)
│
├── Dockerfile             (For production)
├── render.yaml            (Render config)
├── package.json           (Dependencies)
└── .env                   (Configuration)
```

---

## 🛠️ What You Need

- **Node.js** 20.x or higher
- **Docker** (for database)
- **Browser** (any modern browser)

Check you have them:
```bash
node --version
docker --version
```

---

## 🚨 Troubleshooting

### "Cannot connect to backend"
1. Is the script still running? (should see "Auth server running")
2. Is PostgreSQL running? (`docker ps` should show container)
3. Wait 5 seconds and refresh the browser

### "Database connection failed"
1. Start Docker first
2. Run the script again
3. Wait 10 seconds for database to initialize

### "Port 5432 already in use"
The script will skip starting Docker and use existing database
- Or manually run: `docker stop physical-ai-auth-db` and try again

### Still having issues?
1. Check `LOCAL_SETUP_GUIDE.md` for detailed help
2. Look at terminal output for error messages
3. Check `.env` file has correct DATABASE_URL

---

## 📝 Next Steps After Testing

Once local testing works:

1. ✅ **Check all features work** (signup, signin, profile)
2. ✅ **Test with different data**
3. ✅ **Verify logs show no errors**
4. ✅ **Then deploy to production** using `DEPLOY_NOW.md`

---

## 🎓 Learning Resources

Inside this folder:
- `LOCAL_SETUP_GUIDE.md` - Complete step-by-step guide
- `DEPLOYMENT_SUMMARY.md` - Feature overview
- `FIX_EXIT_CODE_128.md` - Technical deep dive
- `DEPLOY_NOW.md` - Production deployment guide

---

## 🎯 Quick Commands

| Action | Command |
|--------|---------|
| Start everything | `START.bat` (Windows) or `bash start.sh` (Mac/Linux) |
| Open test UI | Double-click `test-client.html` |
| Stop backend | Press `Ctrl+C` in terminal |
| Stop database | `docker stop physical-ai-auth-db` |
| View database | `psql -h localhost -U authuser -d physical_ai_auth` |
| Check Docker | `docker ps` |

---

## 💻 API Quick Reference

The test UI tests these endpoints:

```
POST   /api/auth/sign-up       Create account
POST   /api/auth/sign-in       Login
POST   /api/auth/sign-out      Logout
GET    /api/auth/session       Get current session
GET    /api/users/profile      Get user profile (protected)
PUT    /api/users/profile      Update profile (protected)
POST   /api/users/profile/init Initialize profile (protected)
GET    /health                 Health check
```

---

## ✨ Features Included

✅ User authentication (email/password)
✅ User sessions
✅ User profiles with personalization
✅ CORS configured for local development
✅ PostgreSQL database
✅ Beautiful test UI
✅ Detailed logging

---

## 📱 Browser Support

Works in all modern browsers:
- Chrome/Edge (✅ tested)
- Firefox (✅ tested)
- Safari (✅ tested)
- Mobile browsers (✅ responsive design)

---

## 🔐 Security Note

This is for **local development only**. For production:
1. Generate a new `BETTER_AUTH_SECRET`
2. Use strong passwords
3. Enable HTTPS
4. Set proper CORS origins
5. Use environment variables for secrets

See `DEPLOY_NOW.md` for production setup.

---

## 📞 Need Help?

1. **Check the logs** - terminal output shows what's happening
2. **Read LOCAL_SETUP_GUIDE.md** - has detailed troubleshooting
3. **Check .env file** - make sure DATABASE_URL is correct
4. **Verify Docker** - run `docker ps` to check PostgreSQL

---

## 🎉 You're Ready!

```bash
# Windows
START.bat

# Mac/Linux
bash start.sh

# Then open test-client.html in your browser
```

**Happy testing!** 🚀
