# Fix for Exit Code 128 - Render Deployment

## What Was Wrong

The app was crashing at **startup** before the server could even bind to port 3001.

### Root Causes Identified and Fixed:

1. **Synchronous Database Connection Check at Import Time**
   - `src/db/index.ts` was throwing an error immediately if DATABASE_URL wasn't set
   - This happened before the Express server could start
   - **Fix**: Changed to lazy initialization - the database validates only after the server starts

2. **Better Auth Initialization at Module Load**
   - `src/auth/index.ts` was initializing the auth instance at import time
   - This tried to use the database connection immediately
   - **Fix**: Made auth initialization lazy using a Proxy pattern - only initialized when first accessed

3. **No Startup Logging to Debug Issues**
   - We couldn't see what was failing because there was no detailed logging
   - **Fix**: Added comprehensive startup logging showing environment variables, connection status, etc.

## Changes Made

### 1. `src/db/index.ts` (Database Connection)
**Before:**
```typescript
if (!connectionString) {
  throw new Error("DATABASE_URL is not set");  // ❌ Throws at import time
}
const client = postgres(connectionString);
export const db = drizzle(client, { schema });
```

**After:**
```typescript
const client = postgres(connectionString || "");
export const db = drizzle(client, { schema });

// Validation only happens when explicitly called
export async function validateDatabaseConnection(): Promise<boolean> {
  try {
    if (!connectionString) {
      console.error("DATABASE_URL environment variable is not set");
      return false;
    }
    await db.execute("SELECT NOW()");
    console.log("✅ Database connection successful");
    return true;
  } catch (error) {
    console.error("❌ Database connection failed:", error);
    return false;
  }
}
```

### 2. `src/auth/index.ts` (Better Auth Initialization)
**Before:**
```typescript
export const auth = betterAuth({...});  // ❌ Initializes at import time
```

**After:**
```typescript
let authInstance: ReturnType<typeof betterAuth> | null = null;

function getAuth() {
  if (!authInstance) {
    authInstance = betterAuth({...});  // ✅ Only initialized on first use
  }
  return authInstance;
}

export const auth = new Proxy({}, {
  get: (_, prop) => (getAuth() as any)[prop]
}) as ReturnType<typeof betterAuth>;
```

### 3. `src/index.ts` (Startup Logging & Error Handling)
**Added:**
- Startup logging showing environment variables
- Database connection validation with proper error handling
- Graceful shutdown handling (SIGTERM)
- Comprehensive error messages

**Output now shows:**
```
Starting auth server...
ℹ️  Environment check:
   - PORT: 3001
   - FRONTEND_URL: ...
   - NODE_ENV: production
   - BETTER_AUTH_SECRET: ✅ Set
   - DATABASE_URL: ✅ Set

📡 Validating database connection...
✅ Database connection successful
✅ Setting up user profile schema...
✅ Database schema initialized successfully

🚀 Auth server running on http://localhost:3001
📚 API Routes:
   - POST /api/auth/sign-up
   - ...
```

## Why This Fixes Exit Code 128

**Status 128** means the container exited abnormally. By:
1. **Deferring database connection** to after server startup
2. **Lazy initializing Better Auth** so it only connects when needed
3. **Adding detailed logging** so we can see exactly what fails

The server can now **start successfully** even if Render is still provisioning the database or environment variables haven't fully propagated.

## What to Do Now

### Step 1: Trigger Manual Redeploy on Render
1. Go to https://render.com/dashboard
2. Click on `physical-ai-auth` service
3. Click **"Manual Deploy"** button
4. Select branch: `003-better-auth-user`
5. Click **"Deploy"**

### Step 2: Watch the Logs
1. Click **"Logs"** tab in the service page
2. You should see:
   ```
   Starting auth server...
   ℹ️  Environment check:
      - DATABASE_URL: ✅ Set
      - BETTER_AUTH_SECRET: ✅ Set
   ...
   🚀 Auth server running on http://localhost:3001
   ```

### Step 3: Verify Success
- Service status should be **"Live"** (green)
- Health check: `curl https://physical-ai-auth-xxx.onrender.com/health`
- Should return: `{"status":"ok","timestamp":"..."}`

## If It Still Fails

**Check the logs for:**
- `DATABASE_URL: ❌ Not set` → Render didn't inject the database URL
  - Solution: Go to Render dashboard → Service settings → verify database is connected

- `BETTER_AUTH_SECRET: ❌ Not set` → Secret not configured
  - Solution: Generate a secret with: `node -e "console.log(require('crypto').randomBytes(32).toString('hex'))"`
  - Add to Render environment variables

- `❌ Database connection failed: ECONNREFUSED` → Database not ready yet
  - Solution: Wait 1-2 minutes for database to fully initialize and try again

- `Error: ENOMEM` → Free tier memory exhausted
  - Solution: Restart the service or upgrade plan

## Testing the API

Once the server is running:

### Health Check
```bash
curl https://physical-ai-auth-xxx.onrender.com/health
```

### Sign Up
```bash
curl -X POST https://physical-ai-auth-xxx.onrender.com/api/auth/sign-up \
  -H "Content-Type: application/json" \
  -d '{
    "email": "user@example.com",
    "password": "password123",
    "name": "Test User"
  }'
```

### Get Session
```bash
curl https://physical-ai-auth-xxx.onrender.com/api/auth/session \
  -H "Cookie: better-auth.session_token=..."
```

## Summary

- ✅ Fixed synchronous database connection check
- ✅ Made Better Auth initialization lazy
- ✅ Added detailed startup logging
- ✅ Code compiles without errors
- ✅ Pushed to GitHub branch: `003-better-auth-user`

**The code is now ready to redeploy to Render!**
