# Better Auth Backend - Setup Guide

## Quick Start

This guide walks you through setting up the Physical AI Textbook authentication backend.

## Prerequisites

- Node.js 18+ (already installed)
- PostgreSQL (or Docker)
- npm (already installed)

## Step 1: Environment Configuration

The `.env` file has been created with defaults. Update it with your database connection:

```bash
cd auth-backend

# If using local PostgreSQL
DATABASE_URL=postgresql://username:password@localhost:5432/physical_ai_auth

# If using cloud PostgreSQL (Render example)
DATABASE_URL=postgresql://user:password@host.render.internal:5432/physical_ai_auth

# Other values are already configured
PORT=3001
BETTER_AUTH_SECRET=8aUZWMstbytTyJ8KtQZ8TVjz4KetZo1y
FRONTEND_URL=http://localhost:3000
```

## Step 2: Create PostgreSQL Database

### Option A: Local PostgreSQL (Linux/Mac)

```bash
# Create database
createdb physical_ai_auth

# Check connection
psql physical_ai_auth -c "SELECT 1;"
```

### Option B: Local PostgreSQL (Windows with PostgreSQL installed)

```bash
# Using psql
psql -U postgres -c "CREATE DATABASE physical_ai_auth;"

# Connection string format:
# DATABASE_URL=postgresql://postgres:password@localhost:5432/physical_ai_auth
```

### Option C: Docker (Recommended)

```bash
# Start PostgreSQL container
docker run --name postgres_auth \
  -e POSTGRES_USER=authuser \
  -e POSTGRES_PASSWORD=authpass123 \
  -e POSTGRES_DB=physical_ai_auth \
  -p 5432:5432 \
  -d postgres:16

# Update .env with:
# DATABASE_URL=postgresql://authuser:authpass123@localhost:5432/physical_ai_auth
```

### Option D: Cloud PostgreSQL (Render.com - Recommended for Production)

1. Go to https://render.com
2. Create account → Create Database
3. Copy the External Database URL
4. Update `.env` with the URL

## Step 3: Run Database Migrations

```bash
# Apply the initial schema
psql $DATABASE_URL < migrations/0001_initial_schema.sql

# Verify tables were created
psql $DATABASE_URL -c "\dt"
```

## Step 4: Install Dependencies

```bash
npm install
```

## Step 5: Start the Server

```bash
# Development mode (with auto-reload)
npm run dev

# Production mode (build then run)
npm run build
npm start
```

## Step 6: Test the Server

The server will start on `http://localhost:3001`

```bash
# Health check
curl http://localhost:3001/health

# Response should be:
# {"status":"ok","timestamp":"2025-12-02T..."}
```

## Common Issues

### Database Connection Refused

**Problem**: `Error: connect ECONNREFUSED 127.0.0.1:5432`

**Solution**:
- Check PostgreSQL is running
- Verify DATABASE_URL in .env
- Test connection: `psql $DATABASE_URL -c "SELECT 1;"`

### Missing Database

**Problem**: `error: database "physical_ai_auth" does not exist`

**Solution**:
```bash
# Create the database
createdb physical_ai_auth

# Or with psql
psql -U postgres -c "CREATE DATABASE physical_ai_auth;"
```

### Port Already in Use

**Problem**: `Error: listen EADDRINUSE :::3001`

**Solution**:
```bash
# Change PORT in .env to 3002, 3003, etc.
# Or kill process using port 3001:
lsof -i :3001  # (macOS/Linux)
netstat -ano | findstr :3001  # (Windows)
```

### Better Auth Secret Not Set

**Problem**: Warning about using default secret

**Solution**:
- A random secret has been generated in .env
- For production, generate a new secret:
  ```bash
  node -e "console.log(require('crypto').randomBytes(32).toString('hex'))"
  ```

## Available API Endpoints

### Authentication (via Better Auth)
- `POST /api/auth/sign-up` - Register new user
- `POST /api/auth/sign-in` - Login user
- `POST /api/auth/sign-out` - Logout
- `GET /api/auth/session` - Get current session

### User Profile (requires authentication)
- `GET /api/users/profile` - Get user profile
- `PUT /api/users/profile` - Update profile
- `POST /api/users/profile/init` - Initialize profile on signup

### Health
- `GET /health` - Server status

## Example Requests

### Sign Up

```bash
curl -X POST http://localhost:3001/api/auth/sign-up \
  -H "Content-Type: application/json" \
  -d '{
    "email": "user@example.com",
    "password": "password123",
    "name": "John Doe"
  }'
```

### Initialize Profile

```bash
curl -X POST http://localhost:3001/api/users/profile/init \
  -H "Content-Type: application/json" \
  -H "Authorization: Bearer YOUR_SESSION_TOKEN" \
  -d '{
    "softwareDevelopmentExperience": "intermediate",
    "roboticsHardwareBackground": "basic",
    "programmingLanguages": ["Python", "JavaScript", "C++"]
  }'
```

## Next Steps

1. **Frontend Integration**: Connect your frontend to these endpoints
2. **OAuth Setup**: Add GitHub/Google credentials for OAuth
3. **Deployment**: Deploy to Render, Railway, or similar
4. **Database Backups**: Set up automated backups for production

## Support

For issues with Better Auth, see:
- https://better-auth.com/docs
- https://better-auth.com/docs/api-reference

For database issues:
- PostgreSQL docs: https://www.postgresql.org/docs/
- Render docs: https://render.com/docs
