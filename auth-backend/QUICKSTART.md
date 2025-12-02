# 🚀 Quick Start - Physical AI Auth Backend

## 5-Minute Setup

### 1. Configure Database (Choose One)

#### Option A: Docker (Easiest - Recommended)
```bash
# Create and start PostgreSQL container
docker run --name postgres_auth \
  -e POSTGRES_USER=authuser \
  -e POSTGRES_PASSWORD=authpass123 \
  -e POSTGRES_DB=physical_ai_auth \
  -p 5432:5432 \
  -d postgres:16

# Update .env file:
# DATABASE_URL=postgresql://authuser:authpass123@localhost:5432/physical_ai_auth
```

#### Option B: Local PostgreSQL
```bash
# Create database
createdb physical_ai_auth

# Update .env file:
# DATABASE_URL=postgresql://username:password@localhost:5432/physical_ai_auth
```

#### Option C: Cloud PostgreSQL (Render/Railway/Vercel)
1. Create PostgreSQL database on your chosen platform
2. Copy connection URL
3. Update `.env` → `DATABASE_URL=your_url_here`

### 2. Run Database Migrations

**Linux/macOS:**
```bash
cd auth-backend
bash setup-db.sh
```

**Windows:**
```bash
cd auth-backend
setup-db.bat
```

**Manual (All platforms):**
```bash
psql $DATABASE_URL < migrations/0001_initial_schema.sql
```

### 3. Start the Server

```bash
cd auth-backend

# Install dependencies (if not already done)
npm install

# Start development server
npm run dev

# Server runs at http://localhost:3001
```

### 4. Test It Works

```bash
# Health check
curl http://localhost:3001/health

# Expected response:
# {"status":"ok","timestamp":"2025-12-02T..."}
```

## ✅ You're All Set!

Your auth backend is now running. Next steps:

1. **Sign Up Test** (via frontend or curl)
2. **Test Endpoints** (see API Reference below)
3. **Deploy** (when ready - see SETUP_GUIDE.md)

## 📚 Files You Need to Know

| File | Purpose |
|------|---------|
| `.env` | Configuration (DATABASE_URL, secrets) |
| `migrations/0001_initial_schema.sql` | Database schema |
| `setup-db.sh` / `setup-db.bat` | Automated setup scripts |
| `SETUP_GUIDE.md` | Detailed setup instructions |
| `README.md` | Full documentation |

## 🔌 API Reference

### Authentication
```bash
# Sign up
POST /api/auth/sign-up
Content-Type: application/json

{
  "email": "user@example.com",
  "password": "password123",
  "name": "John Doe"
}
```

```bash
# Sign in
POST /api/auth/sign-in
Content-Type: application/json

{
  "email": "user@example.com",
  "password": "password123"
}
```

```bash
# Get current session
GET /api/auth/session
Cookie: <session_token>
```

### User Profile
```bash
# Get profile
GET /api/users/profile
Authorization: Bearer <token>

# Update profile
PUT /api/users/profile
Authorization: Bearer <token>
Content-Type: application/json

{
  "softwareDevelopmentExperience": "intermediate",
  "roboticsHardwareBackground": "basic",
  "programmingLanguages": ["Python", "JavaScript"],
  "preferredLanguage": "en"
}

# Initialize profile (after signup)
POST /api/users/profile/init
Authorization: Bearer <token>
Content-Type: application/json

{
  "softwareDevelopmentExperience": "beginner",
  "roboticsHardwareBackground": "none",
  "programmingLanguages": ["Python"]
}
```

## 🐛 Troubleshooting

| Issue | Solution |
|-------|----------|
| `Connection refused` | Ensure PostgreSQL is running |
| `Database does not exist` | Run migrations: `psql $DATABASE_URL < migrations/0001_initial_schema.sql` |
| `Port 3001 in use` | Change `PORT=3002` in `.env` |
| `psql not found` | Install PostgreSQL client |

## 📖 Learn More

- **Better Auth docs**: https://better-auth.com/docs
- **PostgreSQL docs**: https://www.postgresql.org/docs/
- **Express.js docs**: https://expressjs.com/
- **Drizzle ORM docs**: https://orm.drizzle.team/

## 🎯 Next Steps

1. **Frontend Integration**: Connect your React/Vue frontend to auth endpoints
2. **Environment Variables**: Set up different `.env` for dev/production
3. **OAuth Setup**: Add GitHub/Google credentials for social login
4. **Deployment**: Deploy to Render, Railway, or Vercel

## 💡 Pro Tips

1. **Keep secrets safe**: Never commit `.env` to git (already in `.gitignore`)
2. **Use strong passwords**: Generate with `openssl rand -base64 32`
3. **Enable HTTPS**: Required for production (handled by deployment platform)
4. **Backup database**: Set up automated backups for production

---

**Questions?** Check SETUP_GUIDE.md for detailed instructions or Better Auth docs at https://better-auth.com
