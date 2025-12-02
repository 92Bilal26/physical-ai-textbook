# Start Local PostgreSQL Database

## Option 1: Using Docker (Easiest - Recommended)

### Step 1: Install Docker
If you don't have Docker installed, download from: https://www.docker.com/products/docker-desktop

### Step 2: Run PostgreSQL Container

```bash
docker run --name physical-ai-auth-db \
  -e POSTGRES_USER=authuser \
  -e POSTGRES_PASSWORD=authpass123 \
  -e POSTGRES_DB=physical_ai_auth \
  -p 5432:5432 \
  -d postgres:16-alpine
```

This creates a PostgreSQL container with:
- **User**: authuser
- **Password**: authpass123
- **Database**: physical_ai_auth
- **Port**: 5432

### Step 3: Verify Connection

```bash
# Install psql if you don't have it
# On Windows: Download from https://www.postgresql.org/download/windows/
# On Mac: brew install postgresql
# On Linux: sudo apt install postgresql-client

psql -h localhost -U authuser -d physical_ai_auth -c "SELECT 1"
```

Should show: `?column?` with value `1`

---

## Option 2: Using Local PostgreSQL Installation

### Step 1: Install PostgreSQL
- **Windows**: https://www.postgresql.org/download/windows/
- **Mac**: `brew install postgresql@16`
- **Linux**: `sudo apt install postgresql`

### Step 2: Start PostgreSQL Service
```bash
# Mac
brew services start postgresql@16

# Linux
sudo systemctl start postgresql

# Windows
# Should start automatically after installation
```

### Step 3: Create Database
```bash
createdb -U postgres physical_ai_auth
```

### Step 4: Create User
```bash
psql -U postgres -d physical_ai_auth -c "CREATE USER authuser WITH PASSWORD 'authpass123';"
psql -U postgres -d physical_ai_auth -c "GRANT ALL PRIVILEGES ON DATABASE physical_ai_auth TO authuser;"
```

---

## Connection String

After setup, your `DATABASE_URL` should be:

```
postgresql://authuser:authpass123@localhost:5432/physical_ai_auth
```

Save this - you'll need it for the `.env` file!

---

## Stop & Clean Up

### Docker
```bash
# Stop the container
docker stop physical-ai-auth-db

# Remove the container (if you want to start fresh)
docker rm physical-ai-auth-db

# Restart the same container
docker start physical-ai-auth-db
```

### Local PostgreSQL
```bash
# Mac
brew services stop postgresql@16

# Linux
sudo systemctl stop postgresql
```

---

## Troubleshooting

### Port 5432 Already in Use
If you get "port 5432 is already in use":
```bash
# Find process using port 5432
lsof -i :5432  # Mac/Linux
netstat -ano | findstr :5432  # Windows

# Kill the process or use different port
docker run ... -p 5433:5432 postgres:16-alpine
# Then use: postgresql://authuser:authpass123@localhost:5433/physical_ai_auth
```

### Can't Connect to Database
1. Verify PostgreSQL is running: `docker ps` or `pg_isready`
2. Check credentials match what you set
3. Check firewall isn't blocking port 5432
4. Try connection with psql first before running the app

### "database does not exist" error
Run the database creation command again or check spelling of database name

---

## Next Step

Once your local database is running:
1. Update `auth-backend/.env` with your `DATABASE_URL`
2. Run: `npm run dev` in auth-backend folder
3. Backend will start on http://localhost:3001
