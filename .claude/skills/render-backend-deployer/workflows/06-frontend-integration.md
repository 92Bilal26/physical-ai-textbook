# Workflow 6: Frontend Integration - Connect Your Frontend to Backend

This workflow guides you through connecting your frontend application to your deployed FastAPI backend on Render.

**Time estimate**: 5-10 minutes
**Prerequisites**: Completed Workflow 1 (backend deployed and "Live")
**Success indicator**: Frontend successfully calls backend endpoints

---

## Overview

Your frontend (on GitHub Pages, Vercel, Netlify, etc.) needs to:

1. Know the correct backend URL (on Render)
2. Make HTTP requests to backend endpoints
3. Handle CORS properly (already configured in Workflow 4)
4. Manage API responses and errors

---

## Step 1: Find Your Backend URL

### 1.1 Get the Service URL

From Render Dashboard:

1. Click your backend service
2. At the top, find the URL:
   ```
   https://my-fastapi-backend-XXXX.onrender.com
   ```

**Important**:
- Include the random suffix (XXXX)
- It's unique to your service
- Changes each redeployment? No - stays the same unless you delete the service

### 1.2 Verify Backend is Live

Test the health endpoint:

```bash
curl https://my-fastapi-backend-XXXX.onrender.com/api/v1/health
# Should return: {"status":"ok",...}
```

**Check**:
- [ ] Service shows "Live" in Render dashboard
- [ ] Health endpoint returns 200
- [ ] Can copy the full URL

---

## Step 2: Configure Frontend Environment

### 2.1 Add Backend URL to Frontend

Store the backend URL in your frontend environment:

**Option A: React with .env**

Create `.env` in your frontend repository root:

```
VITE_BACKEND_URL=https://my-fastapi-backend-XXXX.onrender.com
```

Or for Create React App:

```
REACT_APP_BACKEND_URL=https://my-fastapi-backend-XXXX.onrender.com
```

Use in code:

```javascript
// In your React component
const BACKEND_URL = process.env.VITE_BACKEND_URL || process.env.REACT_APP_BACKEND_URL;

// Or hardcode if preferred
const BACKEND_URL = "https://my-fastapi-backend-XXXX.onrender.com";
```

**Option B: Plain JavaScript**

```javascript
const BACKEND_URL = "https://my-fastapi-backend-XXXX.onrender.com";
```

**Option C: Configuration File**

```javascript
// config.js
export const API_CONFIG = {
  BACKEND_URL: "https://my-fastapi-backend-XXXX.onrender.com",
  TIMEOUT: 30000,
};
```

### 2.2 Different URLs for Development and Production

If you need different URLs:

```javascript
const BACKEND_URL = process.env.NODE_ENV === 'production'
  ? 'https://my-fastapi-backend-XXXX.onrender.com'
  : 'http://localhost:8000';
```

**Check**:
- [ ] Backend URL configured in frontend
- [ ] Correct URL with random suffix (XXXX)
- [ ] URL accessible (no 404 when visiting in browser)

---

## Step 3: Create API Service

### 3.1 Basic Fetch Wrapper

```javascript
// services/api.js
const BACKEND_URL = "https://my-fastapi-backend-XXXX.onrender.com";

export async function fetchAPI(endpoint, options = {}) {
  const url = `${BACKEND_URL}${endpoint}`;

  const response = await fetch(url, {
    headers: {
      'Content-Type': 'application/json',
      ...options.headers,
    },
    ...options,
  });

  if (!response.ok) {
    throw new Error(`API error: ${response.status} ${response.statusText}`);
  }

  return response.json();
}

// Usage
const data = await fetchAPI('/api/v1/health');
```

### 3.2 TypeScript Version

```typescript
// services/api.ts
interface ApiOptions extends RequestInit {
  headers?: Record<string, string>;
}

const BACKEND_URL = "https://my-fastapi-backend-XXXX.onrender.com";

export async function fetchAPI<T>(
  endpoint: string,
  options: ApiOptions = {}
): Promise<T> {
  const url = `${BACKEND_URL}${endpoint}`;

  const response = await fetch(url, {
    headers: {
      'Content-Type': 'application/json',
      ...options.headers,
    },
    ...options,
  });

  if (!response.ok) {
    throw new Error(`API error: ${response.status}`);
  }

  return response.json() as Promise<T>;
}
```

### 3.3 With Error Handling

```javascript
// services/api.js
export async function fetchAPI(endpoint, options = {}) {
  const BACKEND_URL = "https://my-fastapi-backend-XXXX.onrender.com";
  const url = `${BACKEND_URL}${endpoint}`;

  try {
    const response = await fetch(url, {
      headers: {
        'Content-Type': 'application/json',
        ...options.headers,
      },
      ...options,
    });

    if (!response.ok) {
      const errorData = await response.json().catch(() => ({}));
      throw new Error(
        errorData.detail ||
        `${response.status} ${response.statusText}`
      );
    }

    return await response.json();
  } catch (error) {
    console.error(`API request failed: ${error.message}`);
    throw error;
  }
}
```

**Check**:
- [ ] API service wraps fetch calls
- [ ] Backend URL is configurable
- [ ] Error handling in place

---

## Step 4: Call Backend Endpoints

### 4.1 Test Health Endpoint

```javascript
// Test connectivity
async function testBackend() {
  try {
    const response = await fetchAPI('/api/v1/health');
    console.log('Backend healthy:', response);
  } catch (error) {
    console.error('Backend error:', error);
  }
}

// Call on app startup
testBackend();
```

### 4.2 Call Your API Endpoints

```javascript
// For a chat endpoint
async function sendQuery(message) {
  const response = await fetchAPI('/api/v1/chat/query', {
    method: 'POST',
    body: JSON.stringify({ message }),
  });
  return response;
}

// For a GET endpoint
async function fetchDocuments() {
  const response = await fetchAPI('/api/v1/documents');
  return response;
}
```

### 4.3 In React Components

```javascript
import { useState, useEffect } from 'react';
import { fetchAPI } from './services/api';

export function ChatComponent() {
  const [message, setMessage] = useState('');
  const [response, setResponse] = useState('');
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState('');

  const handleSubmit = async (e) => {
    e.preventDefault();
    setLoading(true);
    setError('');

    try {
      const result = await fetchAPI('/api/v1/chat/query', {
        method: 'POST',
        body: JSON.stringify({ message }),
      });
      setResponse(result.response);
    } catch (err) {
      setError(err.message);
    } finally {
      setLoading(false);
    }
  };

  return (
    <div>
      <form onSubmit={handleSubmit}>
        <input
          value={message}
          onChange={(e) => setMessage(e.target.value)}
          placeholder="Ask a question..."
        />
        <button type="submit" disabled={loading}>
          {loading ? 'Loading...' : 'Send'}
        </button>
      </form>
      {error && <div className="error">{error}</div>}
      {response && <div className="response">{response}</div>}
    </div>
  );
}
```

**Check**:
- [ ] Endpoints callable from frontend
- [ ] Responses parsed correctly
- [ ] Error messages display

---

## Step 5: Troubleshooting Frontend Errors

### 5.1 Error: "Failed to fetch" / "Network error"

**Causes**:
- Backend URL is wrong
- Backend not running (status not "Live")
- Network connectivity issue

**Solution**:
1. Verify backend URL (with XXXX suffix)
2. Test health endpoint: `curl https://backend-XXXX.onrender.com/api/v1/health`
3. Check Render service status is "Live"
4. Check browser console for exact error

### 5.2 Error: "CORS policy blocked"

**See Workflow 4: CORS Configuration** for detailed CORS debugging.

**Quick fix**:
1. Verify backend has CORS middleware
2. Check ALLOWED_ORIGINS includes frontend URL
3. Test preflight request:
   ```bash
   curl -X OPTIONS https://backend-XXXX.onrender.com/api/endpoint \
        -H "Origin: https://yourname.github.io" \
        -v
   ```

### 5.3 Error: "404 Not Found"

**Cause**: Endpoint path is wrong or doesn't exist

**Solution**:
1. Check endpoint path matches backend code
2. Verify full path: `/api/v1/endpoint` (not just `/endpoint`)
3. Check request method (GET vs POST)
4. Check backend logs for what endpoints exist

### 5.4 Error: "405 Method Not Allowed"

**Cause**: Using wrong HTTP method (POST instead of GET, etc.)

**Solution**:
1. Check endpoint definition: `@app.get()` or `@app.post()`
2. Match the method in frontend fetch:
   ```javascript
   // For @app.get endpoint
   fetchAPI('/api/v1/endpoint')  // Default is GET

   // For @app.post endpoint
   fetchAPI('/api/v1/endpoint', {
     method: 'POST',
     body: JSON.stringify(data),
   })
   ```

### 5.5 Error: "500 Internal Server Error"

**Cause**: Backend endpoint has a bug or runtime error

**Solution**:
1. Check backend logs (Render → Logs tab)
2. Look for Python exception/traceback
3. Fix the backend code
4. Redeploy

---

## Step 6: Testing

### 6.1 Test with Browser DevTools

1. Open browser DevTools (F12)
2. Go to **Network** tab
3. Make API call from frontend
4. Check request:
   - URL is correct
   - Method is correct (GET/POST)
   - Headers include `Content-Type: application/json`
5. Check response:
   - Status is 200 (or expected code)
   - Body contains expected data
   - No CORS error

### 6.2 Test with curl

```bash
# Test GET endpoint
curl https://backend-XXXX.onrender.com/api/v1/health

# Test POST endpoint
curl -X POST https://backend-XXXX.onrender.com/api/v1/chat/query \
  -H "Content-Type: application/json" \
  -H "Origin: https://yourname.github.io" \
  -d '{"message":"test"}'
```

### 6.3 Test Different Browsers

Test in multiple browsers to rule out browser-specific issues:
- Chrome
- Firefox
- Safari
- Mobile browser

**Check**:
- [ ] Works in at least 2 browsers
- [ ] Request headers look correct
- [ ] Response status is 200 or expected code
- [ ] Response body contains expected data

---

## Step 7: Deployment

### 7.1 Commit Frontend Changes

```bash
git add .
git commit -m "Connect frontend to Render backend"
git push origin main
```

### 7.2 Deploy Frontend

**If using GitHub Pages**:
```bash
npm run build  # Build for production
npm run deploy  # Deploy to gh-pages
```

**If using Vercel**:
- Commit to GitHub
- Vercel auto-deploys on push

**If using Netlify**:
- Commit to GitHub
- Netlify auto-deploys on push

### 7.3 Verify Deployment

After frontend deploys:

1. Visit your frontend URL
2. Test API calls
3. Check browser DevTools for errors
4. Verify responses display correctly

**Check**:
- [ ] Frontend deployed successfully
- [ ] API calls work from production URL
- [ ] No CORS errors
- [ ] Data displays correctly

---

## Complete Integration Checklist

- [ ] Backend service URL identified with random suffix
- [ ] Backend service status is "Live"
- [ ] Backend URL configured in frontend
- [ ] API service created with fetch wrapper
- [ ] At least one endpoint callable
- [ ] Error handling in place
- [ ] Tested with browser DevTools
- [ ] Tested with curl
- [ ] Works in multiple browsers
- [ ] Frontend deployed
- [ ] API calls work from production

---

## Success Checklist

- [ ] Frontend loads and displays
- [ ] API calls return expected data
- [ ] Responses display in UI
- [ ] Error messages show on errors
- [ ] Works on desktop and mobile
- [ ] No CORS errors
- [ ] No 404 errors

**Perfect!** Your frontend and backend are fully integrated. 🎉

---

## Next Steps

1. **Handle CORS errors** → Workflow 4: CORS Configuration
2. **Debug API errors** → Workflow 5: Error Debugging
3. **Learn more** → Pattern: Common Errors, Pattern: Log Interpretation

---

## Example: Complete Flow

```javascript
// 1. Configure backend URL
const BACKEND_URL = "https://my-fastapi-backend-XXXX.onrender.com";

// 2. Create API service
async function queryBackend(message) {
  const response = await fetch(
    `${BACKEND_URL}/api/v1/chat/query`,
    {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ message }),
    }
  );
  return response.json();
}

// 3. Use in React
export function App() {
  const [result, setResult] = useState('');

  const handleQuery = async (msg) => {
    try {
      const data = await queryBackend(msg);
      setResult(data.response);
    } catch (error) {
      console.error('API error:', error);
    }
  };

  return (
    <div>
      <input placeholder="Ask a question..." />
      <button onClick={(e) => handleQuery(e.target.value)}>
        Ask
      </button>
      {result && <p>{result}</p>}
    </div>
  );
}
```

That's it! Full integration complete.
