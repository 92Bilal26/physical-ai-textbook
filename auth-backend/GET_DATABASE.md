# How to Get a PostgreSQL Database

Choose one of these options based on your situation.

## 🎯 Quick Comparison

| Option | Best For | Setup Time | Cost | Requires |
|--------|----------|-----------|------|----------|
| **Docker** | Development/Testing | 2 min | Free | Docker installed |
| **Local Install** | Development | 10 min | Free | PostgreSQL installed |
| **Render** | Production/Staging | 3 min | Free tier available | Render account |
| **Railway** | Production/Staging | 3 min | Free tier available | Railway account |
| **Vercel** | Production/Staging | 3 min | Free tier available | Vercel account |
| **AWS RDS** | Large projects | 10 min | Pay as you go | AWS account |

---

## ✅ Option 1: Docker (Easiest - Recommended for Development)

**Perfect if:** You want the fastest setup with zero installation hassle.

### Requirements
- Docker installed ([Download Docker Desktop](https://www.docker.com/products/docker-desktop))

### Setup (2 minutes)

**Step 1: Start PostgreSQL Container**
```bash
docker run --name postgres_auth \
  -e POSTGRES_USER=authuser \
  -e POSTGRES_PASSWORD=authpass123 \
  -e POSTGRES_DB=physical_ai_auth \
  -p 5432:5432 \
  -d postgres:16
```

**Step 2: Verify It's Running**
```bash
docker ps | grep postgres_auth
```

You should see the container listed.

**Step 3: Update .env**
```
DATABASE_URL=postgresql://authuser:authpass123@localhost:5432/physical_ai_auth
```

**Step 4: Test Connection**
```bash
psql postgresql://authuser:authpass123@localhost:5432/physical_ai_auth -c "SELECT 1;"
```

### Useful Docker Commands
```bash
# Stop the container
docker stop postgres_auth

# Start it again later
docker start postgres_auth

# See logs
docker logs postgres_auth

# Delete everything when done
docker rm postgres_auth
```

### Pros
✅ Super fast setup
✅ No installation needed
✅ Easy cleanup
✅ Works on Windows, Mac, Linux
✅ Perfect for development

### Cons
❌ Database deleted when container removed
❌ Requires Docker installed

---

## ✅ Option 2: Local PostgreSQL (Best for Always-On Development)

**Perfect if:** You want a permanent local database that persists between sessions.

### Requirements
- PostgreSQL installed on your computer

### Installation

**Windows:**
1. Download: https://www.postgresql.org/download/windows/
2. Run installer
3. Remember the password you set
4. Choose default port 5432

**macOS:**
```bash
# Using Homebrew
brew install postgresql@16
brew services start postgresql@16
```

**Linux (Ubuntu/Debian):**
```bash
sudo apt-get update
sudo apt-get install postgresql postgresql-contrib
```

### Setup (5 minutes)

**Step 1: Create Database**
```bash
# Windows/Mac/Linux
createdb physical_ai_auth
```

**Step 2: Verify It Works**
```bash
psql -d physical_ai_auth -c "SELECT 1;"
```

**Step 3: Update .env**

If using default local PostgreSQL:
```
DATABASE_URL=postgresql://postgres@localhost:5432/physical_ai_auth
```

If you set a password during install:
```
DATABASE_URL=postgresql://postgres:YOUR_PASSWORD@localhost:5432/physical_ai_auth
```

**Step 4: Test Connection**
```bash
psql $DATABASE_URL -c "SELECT 1;"
```

### Find Your Connection Details

**Windows (in PowerShell):**
```powershell
# Default user
$env:PGUSER = "postgres"
$env:PGPASSWORD = "your_password"

psql -h localhost -d physical_ai_auth
```

**macOS/Linux:**
```bash
# Check if PostgreSQL is running
sudo service postgresql status

# Get connection info
psql -U postgres -c "SELECT current_user, inet_client_addr();"
```

### Useful Commands
```bash
# List all databases
psql -l

# Connect to a database
psql -d physical_ai_auth

# Create a new database
createdb my_database

# Delete a database
dropdb my_database

# Start PostgreSQL (if stopped)
# Windows: pg_ctl -D "C:\Program Files\PostgreSQL\16\data" start
# macOS: brew services start postgresql@16
# Linux: sudo service postgresql start
```

### Pros
✅ Persistent database
✅ No cloud account needed
✅ Works offline
✅ No subscription costs

### Cons
❌ Takes longer to install
❌ Need to manage yourself
❌ Uses your computer resources

---

## ✅ Option 3: Render.com (Best for Production)

**Perfect if:** You want a cloud database that's easy to share with your team.

### Setup (3 minutes)

**Step 1: Create Account**
1. Go to https://render.com
2. Click "Get Started"
3. Sign up (free)

**Step 2: Create PostgreSQL Database**
1. Dashboard → New → PostgreSQL
2. Fill in details:
   - Name: `physical-ai-auth`
   - Database: `physical_ai_auth`
   - User: `authuser`
   - Region: Choose closest to you
   - PostgreSQL Version: 16
3. Click "Create Database"
4. Wait 2-3 minutes for setup

**Step 3: Copy Connection String**
1. On the database page, find "Connections"
2. Copy the "External Database URL"
3. It looks like: `postgresql://user:password@host.render.internal:5432/database`

**Step 4: Update .env**
```
DATABASE_URL=postgresql://user:password@host.render.internal:5432/physical_ai_auth
```

**Step 5: Test Connection**
```bash
psql $DATABASE_URL -c "SELECT 1;"
```

### Free Tier Details
- 1 database free
- 90 days inactive = deletion
- Keep it active (run migrations regularly)

### Pros
✅ Free tier available
✅ Cloud-hosted (always up)
✅ Easy to scale later
✅ Team-friendly (share URL)
✅ Perfect for production

### Cons
❌ Requires internet connection
❌ Database deleted if inactive 90 days
❌ Need to manage credentials

### Deploy Your Backend to Render Too
Later, you can deploy your auth backend to Render:
1. Push code to GitHub
2. Create new Web Service on Render
3. Connect GitHub repository
4. Set environment variables
5. Deploy!

---

## ✅ Option 4: Railway.app (Modern & Easy)

**Perfect if:** You want something between Docker and Render, very developer-friendly.

### Setup (3 minutes)

**Step 1: Create Account**
1. Go to https://railway.app
2. Sign up with GitHub (easiest)

**Step 2: Create PostgreSQL**
1. New Project → Add Service → PostgreSQL
2. Wait for it to start (should be instant)

**Step 3: Copy Connection String**
1. Click on PostgreSQL service
2. Go to "Connect" tab
3. Copy the Connection String
4. It looks like: `postgresql://user:password@host:5432/railway`

**Step 4: Update .env**
```
DATABASE_URL=<paste_the_connection_string>
```

**Step 5: Test Connection**
```bash
psql $DATABASE_URL -c "SELECT 1;"
```

### Free Tier
- $5/month free credits
- Usually covers development database

### Pros
✅ Very easy setup
✅ Good free tier ($5/month)
✅ Great for startups
✅ GitHub integration

### Cons
❌ Credits expire if not used
❌ Requires account

---

## ✅ Option 5: Vercel Postgres (If Using Vercel for Frontend)

**Perfect if:** You already use or plan to use Vercel for your frontend.

### Setup (3 minutes)

**Step 1: Create Database**
1. Go to https://vercel.com/storage/postgres
2. Create new Postgres database
3. Select region

**Step 2: Copy Connection String**
1. Click "Connect" → "Vercel Postgres"
2. Copy the connection string

**Step 3: Update .env**
```
DATABASE_URL=<paste_the_connection_string>
```

**Step 4: Test Connection**
```bash
psql $DATABASE_URL -c "SELECT 1;"
```

### Free Tier
- Free tier available with limits
- Great if frontend is on Vercel

### Pros
✅ Integrated with Vercel
✅ Easy if using Vercel frontend
✅ Great free tier

---

## ✅ Option 6: AWS RDS (For Large/Serious Projects)

**Perfect if:** You need enterprise features and don't mind complexity.

### Setup (10 minutes)

[See AWS RDS documentation](https://docs.aws.amazon.com/rds/latest/UserGuide/CHAP_PostgreSQL.html)

### General Steps
1. Create AWS account
2. Go to RDS service
3. Create database → PostgreSQL
4. Set master username/password
5. Copy endpoint
6. Update .env with connection string

### Free Tier
- 750 hours/month free for 12 months
- `db.t3.micro` instance

### Pros
✅ Most powerful option
✅ Enterprise features
✅ Auto backups
✅ High availability options

### Cons
❌ Complex setup
❌ More expensive after free tier
❌ Overkill for small projects

---

## 🎯 My Recommendation

**For Right Now (Testing):**
👉 **Use Docker** - Fastest, no installation, works great for development

**For Team Development:**
👉 **Use Render.com** - Free, cloud-hosted, easy to share

**For Production:**
👉 **Use Railway or Vercel Postgres** - Good balance of ease and features

---

## 🔄 Switching Databases Later

Good news! You can switch databases anytime:

1. **Export data** (if needed)
2. **Update DATABASE_URL in .env**
3. **Run migrations again** on new database
4. **Restart server** - Done!

The migration file works on any PostgreSQL database.

---

## ✅ Testing Your Database Connection

Once you've set up your database, verify it works:

```bash
# Set the DATABASE_URL (or it should be in .env)
export DATABASE_URL="postgresql://user:password@host:5432/database"

# Test connection
psql $DATABASE_URL -c "SELECT 1;"

# Should output something like:
# ?column?
# ----------
#        1
# (1 row)
```

If this works, you're good to go! 🎉

---

## Common Issues

### "psql: command not found"
- PostgreSQL client not installed
- **Fix:** Install: `sudo apt install postgresql-client` (Linux) or `brew install postgresql` (Mac)

### "Connection refused"
- Database not running
- **Fix:** Start PostgreSQL (Docker: `docker start postgres_auth`, Local: `brew services start postgresql`)

### "FATAL: role 'authuser' does not exist"
- Wrong username in connection string
- **Fix:** Check your DATABASE_URL matches your database setup

### "database 'physical_ai_auth' does not exist"
- Database not created
- **Fix:** Create it: `createdb physical_ai_auth`

---

## Next Steps After Getting Database

1. **Update .env** with your DATABASE_URL
2. **Run migrations:**
   ```bash
   psql $DATABASE_URL < migrations/0001_initial_schema.sql
   ```
3. **Start server:**
   ```bash
   npm run dev
   ```
4. **Test:**
   ```bash
   curl http://localhost:3001/health
   ```

---

## Resources

- **Docker:** https://www.docker.com/products/docker-desktop
- **PostgreSQL:** https://www.postgresql.org/download/
- **Render:** https://render.com
- **Railway:** https://railway.app
- **Vercel Postgres:** https://vercel.com/storage/postgres
- **AWS RDS:** https://aws.amazon.com/rds/

---

**Questions?** See SETUP_GUIDE.md or QUICKSTART.md in the auth-backend directory.
