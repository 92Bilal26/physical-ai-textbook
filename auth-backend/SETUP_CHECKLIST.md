# Setup Checklist - Physical AI Auth Backend

Complete these steps to get your auth backend running.

## Pre-Setup (Verify You Have These)

- [ ] Node.js 18+ installed (`node --version`)
- [ ] npm installed (`npm --version`)
- [ ] Docker installed OR PostgreSQL installed (`psql --version`)
- [ ] Code editor/IDE
- [ ] Terminal/Command Prompt access

## Step 1: Environment Setup

- [ ] Navigate to auth-backend directory
  ```bash
  cd auth-backend
  ```

- [ ] `.env` file exists (should be created already)
  ```bash
  ls .env
  ```

- [ ] `.env` contains required keys:
  - [ ] `DATABASE_URL` (ready to update)
  - [ ] `BETTER_AUTH_SECRET` (already generated)
  - [ ] `PORT=3001`
  - [ ] `FRONTEND_URL=http://localhost:3000`

## Step 2: Dependencies

- [ ] npm dependencies installed
  ```bash
  npm install
  ```

- [ ] Build compiles successfully
  ```bash
  npm run build
  ```

- [ ] No TypeScript errors in output

## Step 3: Database Setup

### Option A: Docker (Recommended)

- [ ] Docker installed and running
- [ ] PostgreSQL container created and running
  ```bash
  docker run --name postgres_auth \
    -e POSTGRES_USER=authuser \
    -e POSTGRES_PASSWORD=authpass123 \
    -e POSTGRES_DB=physical_ai_auth \
    -p 5432:5432 \
    -d postgres:16
  ```

- [ ] Container is running
  ```bash
  docker ps | grep postgres_auth
  ```

### Option B: Local PostgreSQL

- [ ] PostgreSQL server installed and running
- [ ] Database created
  ```bash
  createdb physical_ai_auth
  ```

- [ ] Can connect to database
  ```bash
  psql -d physical_ai_auth -c "SELECT 1;"
  ```

### Option C: Cloud PostgreSQL (Render/Railway/Vercel)

- [ ] Database instance created on platform
- [ ] Connection URL obtained
- [ ] Firewall rules allow connections

## Step 4: Configure Connection

- [ ] Open `.env` file in editor

- [ ] Update `DATABASE_URL` with your connection string:
  ```
  # Docker example:
  DATABASE_URL=postgresql://authuser:authpass123@localhost:5432/physical_ai_auth

  # Local PostgreSQL example:
  DATABASE_URL=postgresql://username:password@localhost:5432/physical_ai_auth

  # Cloud example:
  DATABASE_URL=postgresql://user:password@host.render.internal:5432/physical_ai_auth
  ```

- [ ] Save `.env` file

- [ ] Test connection
  ```bash
  psql $DATABASE_URL -c "SELECT 1;"
  ```
  Should output: `?column?` `1` with success

## Step 5: Run Migrations

### Method A: Automated Setup (Recommended)

**Linux/macOS:**
- [ ] Make script executable
  ```bash
  chmod +x setup-db.sh
  ```

- [ ] Run setup script
  ```bash
  ./setup-db.sh
  ```

- [ ] Verify output shows:
  - [ ] "Database connection successful"
  - [ ] "Migrations completed successfully"
  - [ ] List of tables created

**Windows:**
- [ ] Run setup script
  ```bash
  setup-db.bat
  ```

- [ ] Verify output shows:
  - [ ] "Database connection successful"
  - [ ] "Migrations completed successfully"
  - [ ] List of tables created

### Method B: Manual Migration

- [ ] Run migration file directly
  ```bash
  psql $DATABASE_URL < migrations/0001_initial_schema.sql
  ```

- [ ] Verify tables created
  ```bash
  psql $DATABASE_URL -c "\dt"
  ```

- [ ] Should see these tables:
  - [ ] `users`
  - [ ] `user_profiles`
  - [ ] `sessions`
  - [ ] `accounts`
  - [ ] `verification_tokens`

## Step 6: Start Server

- [ ] Install dependencies (if not done)
  ```bash
  npm install
  ```

- [ ] Start development server
  ```bash
  npm run dev
  ```

- [ ] Server output shows:
  - [ ] "🚀 Auth server running on http://localhost:3001"
  - [ ] List of available routes
  - [ ] No error messages

- [ ] Leave server running (keep terminal open)

## Step 7: Test It Works

**In a new terminal window:**

- [ ] Test health endpoint
  ```bash
  curl http://localhost:3001/health
  ```

- [ ] Response should be:
  ```json
  {"status":"ok","timestamp":"2025-12-02T..."}
  ```

- [ ] Try signing up (example)
  ```bash
  curl -X POST http://localhost:3001/api/auth/sign-up \
    -H "Content-Type: application/json" \
    -d '{
      "email": "test@example.com",
      "password": "password123",
      "name": "Test User"
    }'
  ```

- [ ] Verify no errors in response

## Troubleshooting Checks

If something doesn't work:

### Connection Issues
- [ ] PostgreSQL is running
  ```bash
  # Docker:
  docker ps | grep postgres_auth

  # Local:
  psql -l | grep physical_ai_auth
  ```

- [ ] DATABASE_URL in `.env` is correct
  ```bash
  grep DATABASE_URL .env
  ```

- [ ] Can connect manually
  ```bash
  psql $DATABASE_URL -c "SELECT 1;"
  ```

### Migration Issues
- [ ] Migrations file exists
  ```bash
  ls migrations/0001_initial_schema.sql
  ```

- [ ] Migrations ran successfully (check for tables)
  ```bash
  psql $DATABASE_URL -c "\dt"
  ```

- [ ] No syntax errors in migration file

### Server Issues
- [ ] Dependencies installed
  ```bash
  npm ls | head -20
  ```

- [ ] Build succeeds
  ```bash
  npm run build
  ```

- [ ] Port 3001 not in use
  ```bash
  # Linux/macOS:
  lsof -i :3001

  # Windows:
  netstat -ano | findstr :3001
  ```

- [ ] Environment variables loaded
  ```bash
  grep BETTER_AUTH_SECRET .env
  ```

## After Setup

- [ ] Server is running: `npm run dev`
- [ ] Health check passes: `curl http://localhost:3001/health`
- [ ] Database connected and working
- [ ] Can make API requests

## Next Steps (When Ready)

- [ ] Test frontend integration
- [ ] Set up OAuth credentials (GitHub/Google)
- [ ] Deploy to production (Render/Railway/Vercel)
- [ ] Set up monitoring and logging
- [ ] Configure automated backups

## Documentation Reference

| Document | Use For |
|----------|---------|
| `QUICKSTART.md` | Fast 5-minute setup |
| `SETUP_GUIDE.md` | Detailed instructions |
| `README.md` | API documentation |
| `IMPLEMENTATION_SUMMARY.md` | Technical details |
| This file | Setup verification |

## Getting Help

1. **Check documentation** - SETUP_GUIDE.md has troubleshooting
2. **Check logs** - Server output shows errors
3. **Test database** - `psql $DATABASE_URL -c "SELECT 1;"`
4. **Check ports** - Make sure 3001 isn't in use
5. **Better Auth docs** - https://better-auth.com/docs

---

**Completed all checks?** Your auth backend is ready to use! 🎉

Move on to frontend integration or OAuth setup as needed.
