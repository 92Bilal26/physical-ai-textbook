# Workflow 4: CORS Configuration - Fix Cross-Origin Errors

This workflow helps you configure CORS correctly so your frontend can call your backend without browser blocking.

**Time estimate**: 10-15 minutes
**Prerequisites**: Completed Workflow 1 (service deployed)
**Success indicator**: Frontend makes API calls without CORS errors

---

## Understanding the Problem

When your frontend (on GitHub Pages or other domain) tries to call your backend (on Render), the browser may block it:

```
Access to fetch at 'https://backend-XXXX.onrender.com/api/query'
from origin 'https://yourname.github.io'
has been blocked by CORS policy
```

This happens because:

1. **Different domains**: Frontend and backend on different domains
2. **Browser protection**: Browser prevents cross-origin requests without permission
3. **Preflight request**: Browser sends OPTIONS request first to ask permission
4. **Missing headers**: Backend must respond with CORS headers

The solution is proper CORS middleware configuration.

---

## Step 1: Understand Middleware Order

### The Critical Rule

**In FastAPI, middleware is executed in REVERSE order of addition.**

If you add middleware like this:

```python
app.add_middleware(RateLimitMiddleware)     # ← Added first
app.add_middleware(CORSMiddleware)          # ← Added second
```

They execute like this:

```
Request → CORSMiddleware → RateLimitMiddleware → App
Response ← CORSMiddleware ← RateLimitMiddleware ← App
```

Wait... that's backwards!

Actually, in FastAPI it's:

```
Request → RateLimitMiddleware → CORSMiddleware → App  ❌ WRONG!
```

**The problem**: Rate limiter runs first, might block preflight OPTIONS request before CORS can add headers.

### The Solution

**Add CORS middleware LAST so it runs FIRST**:

```python
app.add_middleware(CORSMiddleware)          # ← Added last (runs first)
app.add_middleware(RateLimitMiddleware)     # ← Added earlier (runs second)
```

Now the execution is:

```
Request → CORSMiddleware → RateLimitMiddleware → App  ✅ CORRECT!
```

**Rule of thumb**: More critical middleware goes LAST in code.

See **Pattern: Middleware Order** for detailed explanation.

---

## Step 2: Configure CORS in Your Application

Update `src/main.py`:

```python
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from src.config import settings

app = FastAPI()

# ✅ STEP 1: Add CORS middleware LAST (runs first)
app.add_middleware(
    CORSMiddleware,
    allow_origins=settings.ALLOWED_ORIGINS,  # Load from environment
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
    max_age=600,  # Cache preflight for 10 minutes
)

# ✅ STEP 2: Add other middleware after CORS
# (If you have rate limiting, logging, etc., add them here)

@app.get("/api/v1/health")
async def health_check():
    return {"status": "ok"}

# ... rest of your endpoints ...
```

### 2.1 CORS Configuration Options

```python
app.add_middleware(
    CORSMiddleware,
    allow_origins=[
        "https://example.com",           # Specific domain
        "https://example.github.io",     # GitHub Pages
        "http://localhost:3000",         # Local development
    ],
    # OR load from environment:
    # allow_origins=settings.ALLOWED_ORIGINS,

    allow_credentials=True,              # Allow cookies/auth headers
    allow_methods=["*"],                 # Allow all HTTP methods
    allow_headers=["*"],                 # Allow all headers
    max_age=600,                         # Cache preflight for 10 min
)
```

### 2.2 Load ALLOWED_ORIGINS from Environment

**In render.yaml**:

```yaml
envVars:
  - key: ALLOWED_ORIGINS
    value: https://example.com,https://example.github.io,http://localhost:3000
```

**In src/config.py**:

```python
from typing import Union
from pydantic import field_validator

class Settings(BaseSettings):
    ALLOWED_ORIGINS: Union[str, list] = "http://localhost:3000"

    @field_validator("ALLOWED_ORIGINS", mode="before")
    @classmethod
    def parse_origins(cls, v):
        """Convert comma-separated string to list"""
        if isinstance(v, str):
            return [o.strip() for o in v.split(",")]
        return v if isinstance(v, list) else []
```

**In src/main.py**:

```python
from src.config import settings

app.add_middleware(
    CORSMiddleware,
    allow_origins=settings.ALLOWED_ORIGINS,  # Loads as list: ["url1", "url2"]
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)
```

See `examples/config.py.example` and `examples/main.py.example` for complete examples.

**Check**:
- [ ] CORS middleware added LAST in code
- [ ] `allow_origins` loaded from settings
- [ ] Settings load ALLOWED_ORIGINS from environment

---

## Step 3: Configure ALLOWED_ORIGINS in Render

Your frontend URLs must be in ALLOWED_ORIGINS:

### 3.1 Find Your Frontend URL

**GitHub Pages**:
```
https://yourusername.github.io
```

**Vercel/Netlify**:
```
https://yourproject-xxx.vercel.app
```

**Local development**:
```
http://localhost:3000
http://localhost:5173  # Vite
```

### 3.2 Add to Render Environment

In Render Dashboard → Service → **Environment** tab:

Click **Add Environment Variable**:

```
Key: ALLOWED_ORIGINS
Value: https://yourusername.github.io,https://example.com,http://localhost:3000
```

**Format Rules**:
- ✅ Comma-separated, no spaces: `url1,url2,url3`
- ✅ Include `https://` or `http://`
- ❌ No JSON brackets: `["url1"]`
- ❌ No extra quotes: `"url1"`

**Check**:
- [ ] All frontend URLs included
- [ ] Comma-separated format (no brackets)
- [ ] No spaces around commas
- [ ] Includes https:// or http://

---

## Step 4: Test CORS Configuration

### 4.1 Test from Browser

1. Open your frontend in browser
2. Open DevTools (F12)
3. Go to **Network** tab
4. Make an API call to your backend
5. Look for OPTIONS request (preflight)

If successful, you'll see:

```
OPTIONS /api/endpoint ... 200
```

Response headers should include:

```
access-control-allow-origin: https://yourname.github.io
access-control-allow-methods: GET, POST, PUT, DELETE, ...
access-control-allow-headers: content-type, ...
```

### 4.2 Test with curl

```bash
curl -X OPTIONS https://backend-XXXX.onrender.com/api/endpoint \
     -H "Origin: https://yourname.github.io" \
     -H "Access-Control-Request-Method: POST" \
     -v

# Look for response headers like:
# < access-control-allow-origin: https://yourname.github.io
# < access-control-allow-methods: POST
```

### 4.3 Common Test Results

✅ **Success** - CORS headers present:
```
< access-control-allow-origin: https://yourname.github.io
< access-control-allow-methods: POST, GET, PUT, DELETE
```

❌ **Failure** - CORS headers missing:
```
(no access-control headers in response)
```

**Check**:
- [ ] OPTIONS request returns 200
- [ ] Response includes `access-control-allow-origin` header
- [ ] Origin matches your frontend URL
- [ ] Methods include what you're using (POST, GET, etc.)

---

## Step 5: Debug CORS Errors

### 5.1 Error: "CORS policy blocked"

**Symptoms**:
- Browser shows CORS error
- OPTIONS request fails
- No `access-control-allow-origin` header in response

**Check these in order**:

1. **Middleware Order** (most common)
   ```python
   # ❌ WRONG
   app.add_middleware(RateLimitMiddleware)
   app.add_middleware(CORSMiddleware)

   # ✅ CORRECT
   app.add_middleware(CORSMiddleware)
   app.add_middleware(RateLimitMiddleware)
   ```

2. **ALLOWED_ORIGINS**
   - Check Render Environment tab
   - Verify frontend URL is in ALLOWED_ORIGINS
   - Check format: `https://yourname.github.io` (no trailing slash)

3. **Frontend URL**
   - Check Origin header matches ALLOWED_ORIGINS exactly
   - Example: If ALLOWED_ORIGINS has `https://example.com` and you access from `https://www.example.com`, they don't match

4. **Test Preflight**
   ```bash
   curl -X OPTIONS https://backend-XXXX.onrender.com/api/endpoint \
        -H "Origin: https://yourname.github.io" \
        -v
   ```

### 5.2 Error: "Preflight request timed out"

**Cause**: Service not responding to OPTIONS requests

**Solution**:
1. Check service is "Live" (Render dashboard)
2. Verify health endpoint works
3. Check service logs for errors
4. If service crashed, check error logs

### 5.3 Error: "Method not allowed"

**Symptoms**:
- OPTIONS request returns 405 Method Not Allowed
- Preflight fails

**Solution**:
1. CORS middleware should handle OPTIONS automatically
2. Check middleware is configured correctly
3. Ensure `allow_methods=["*"]` includes OPTIONS

---

## Step 6: Commit and Deploy

After updating `src/main.py`:

```bash
git add src/main.py
git commit -m "Configure CORS middleware with environment variables"
git push origin main
```

Render will redeploy. After deployment:

1. Check service is "Live"
2. Test CORS again from browser or curl
3. Verify no errors in logs

**Check**:
- [ ] Changes committed and pushed
- [ ] Service redeployed successfully
- [ ] Service shows "Live" status
- [ ] CORS test succeeds

---

## Complete CORS Configuration Checklist

- [ ] CORS middleware in src/main.py
- [ ] CORS middleware added LAST (runs first)
- [ ] ALLOWED_ORIGINS loaded from environment
- [ ] ALLOWED_ORIGINS set in Render Environment tab
- [ ] All frontend URLs in ALLOWED_ORIGINS
- [ ] Comma-separated format (no brackets)
- [ ] OPTIONS preflight request returns 200
- [ ] Response includes `access-control-allow-origin` header
- [ ] Frontend can make API calls without CORS error

---

## Success Checklist

- [ ] Browser DevTools shows OPTIONS request returning 200
- [ ] OPTIONS response includes CORS headers
- [ ] Frontend can fetch from backend without CORS error
- [ ] Multiple frontend URLs work (if configured)
- [ ] Local development works (http://localhost:3000)

**Perfect!** CORS is configured and your frontend can talk to your backend. 🎉

---

## Next Steps

1. **Debug other errors** → Workflow 5: Error Debugging
2. **Connect frontend** → Workflow 6: Frontend Integration
3. **Learn more** → Pattern: Middleware Order (detailed explanation)
