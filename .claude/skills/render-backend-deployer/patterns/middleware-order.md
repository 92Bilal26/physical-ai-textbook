# Middleware Order: Critical CORS Issue

This guide explains why middleware order matters and how to fix CORS errors caused by incorrect ordering.

---

## The Problem

You've deployed your FastAPI backend to Render.com, your frontend is on GitHub Pages, but you get this error:

```
Access to fetch at 'https://backend.onrender.com/api/query'
from origin 'https://yourname.github.io'
has been blocked by CORS policy: Response to preflight request
doesn't pass access control check: No 'Access-Control-Allow-Origin'
header is present on the requested resource.
```

The issue: **Middleware order**.

---

## Why Middleware Order Matters

When your frontend makes a request to the backend:

1. Browser sends **preflight** OPTIONS request first
2. Middleware processes this request in order
3. CORS middleware must run FIRST to add headers
4. Other middleware must run after

### Incorrect Order
```python
# ❌ WRONG - Rate limiting runs first
app.add_middleware(RateLimitMiddleware)     # Runs 1st - may block
app.add_middleware(CORSMiddleware, ...)     # Runs 2nd - too late!
```

When preflight request comes in:
1. RateLimitMiddleware sees it → May rate-limit or return error
2. CORSMiddleware never gets to add CORS headers
3. Browser sees no CORS headers → Blocks request

### Correct Order
```python
# ✅ CORRECT - CORS runs first
app.add_middleware(CORSMiddleware, ...)    # Runs 1st - adds headers
app.add_middleware(RateLimitMiddleware)    # Runs 2nd - doesn't block preflight
```

When preflight request comes in:
1. CORSMiddleware runs first → Adds CORS headers
2. RateLimitMiddleware runs → May rate-limit, but headers already set
3. Browser sees CORS headers → Allows request

---

## FastAPI Middleware Stack

In FastAPI, middleware is added in **reverse order of execution**:

```python
# Execution order: 3 → 2 → 1 → App → 1 → 2 → 3

app.add_middleware(ErrorHandling)    # Runs 3rd
app.add_middleware(RateLimit)        # Runs 2nd
app.add_middleware(CORS)             # Runs 1st (added last!)
```

**Key Rule**: Add middleware in REVERSE order of execution. Since CORS must run first, add it LAST.

---

## Correct FastAPI CORS Setup

```python
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from src.middleware.rate_limit import RateLimitMiddleware

app = FastAPI()

# ✅ STEP 1: Add CORS middleware FIRST (or last in code)
app.add_middleware(
    CORSMiddleware,
    allow_origins=settings.allowed_origins,  # From environment variable
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# ✅ STEP 2: Add other middleware AFTER CORS
if settings.rate_limit_enabled:
    app.add_middleware(RateLimitMiddleware)

# ✅ STEP 3: Include routes (lowest priority)
app.include_router(chat_router)
```

---

## Preflight Request Flow

When your frontend calls a backend endpoint:

### Step 1: Browser Sends Preflight

```http
OPTIONS /api/v1/chat/query HTTP/1.1
Host: backend-XXXX.onrender.com
Origin: https://yourname.github.io
Access-Control-Request-Method: POST
Access-Control-Request-Headers: content-type
```

### Step 2: Backend Processes (Middleware Order)

1. **CORSMiddleware** sees Origin header
   - Checks if origin is in `allow_origins`
   - Adds response headers:
     ```http
     Access-Control-Allow-Origin: https://yourname.github.io
     Access-Control-Allow-Methods: POST
     Access-Control-Allow-Headers: content-type
     ```
   - Returns 200 OK

2. **RateLimitMiddleware** runs (too late - can't block preflight if CORS ran first)

3. Response reaches browser with CORS headers

### Step 3: Browser Allows Actual Request

```http
POST /api/v1/chat/query HTTP/1.1
Host: backend-XXXX.onrender.com
Origin: https://yourname.github.io
Content-Type: application/json

{ "query": "..." }
```

---

## Common Middleware Patterns

### Pattern 1: CORS + Rate Limiting

```python
# ✅ CORRECT ORDER
app.add_middleware(CORSMiddleware, ...)
app.add_middleware(RateLimitMiddleware)
```

### Pattern 2: CORS + Logging + Rate Limiting

```python
# ✅ CORRECT ORDER
app.add_middleware(CORSMiddleware, ...)          # Run 3rd
app.add_middleware(LoggingMiddleware)            # Run 2nd
app.add_middleware(RateLimitMiddleware)          # Run 1st
```

### Pattern 3: Multiple Middleware

```python
# ✅ CORRECT ORDER - Critical ones first
app.add_middleware(CORSMiddleware, ...)          # MOST CRITICAL (run last, so added first)
app.add_middleware(ErrorHandlingMiddleware)     # Important
app.add_middleware(RateLimitMiddleware)          # Less critical
app.add_middleware(LoggingMiddleware)            # Least critical
```

---

## Debugging Middleware Order

### Check Middleware Stack

```python
# Print middleware stack to verify order
print("Middleware stack:")
for middleware in app.user_middleware:
    print(f"  - {middleware.cls.__name__}")

# Should show CORS near the end (meaning it runs first)
```

### Test Preflight Request

```bash
# Send OPTIONS request with CORS headers
curl -X OPTIONS https://backend-XXXX.onrender.com/api/query \
     -H "Origin: https://yourname.github.io" \
     -H "Access-Control-Request-Method: POST" \
     -v

# Look for response headers like:
# < access-control-allow-origin: https://yourname.github.io
# < access-control-allow-methods: POST
```

### Browser DevTools

In your frontend:
1. Open Developer Tools (F12)
2. Go to Network tab
3. Make API call
4. Look for OPTIONS request (preflight)
5. Check Response Headers for `access-control-allow-origin`

---

## Environment Variable Configuration

Set allowed origins in your Render environment:

```
ALLOWED_ORIGINS=https://yourname.github.io,https://anotherdomain.com,http://localhost:3000
```

In your Pydantic Settings:

```python
from pydantic import field_validator
from typing import Union

class Settings(BaseSettings):
    allowed_origins: Union[str, list] = "http://localhost:3000"

    @field_validator("allowed_origins", mode="before")
    @classmethod
    def parse_origins(cls, v):
        if isinstance(v, str):
            return [o.strip() for o in v.split(",")]
        return v if isinstance(v, list) else []
```

Pass to CORS middleware:

```python
app.add_middleware(
    CORSMiddleware,
    allow_origins=settings.allowed_origins,  # Automatically converted to list
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)
```

---

## Summary

**The Golden Rule**: In FastAPI, middleware added LAST runs FIRST.

Since CORS must run first, add it last in your code:

```python
# ✅ CORRECT
app.add_middleware(CORSMiddleware, ...)         # Add LAST
app.add_middleware(RateLimitMiddleware)         # Add earlier
```

This ensures:
- ✅ Preflight requests get CORS headers
- ✅ Browser allows cross-origin requests
- ✅ Your frontend can communicate with backend
- ✅ Other middleware still works (rate limiting, logging, etc.)
