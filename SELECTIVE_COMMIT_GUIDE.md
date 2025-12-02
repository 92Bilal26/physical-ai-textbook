# Selective Commit Guide - Auth Fix Only

## Current Situation:
- You're on branch: `004-urdu-translation`
- You have auth fixes that need to go to: `003-better-auth-user`
- You have other work (Translation) that should stay uncommitted

## Files to Commit (Auth Fix):
1. ✅ `book/src/contexts/AuthContext.tsx` - Fixed backend URL
2. ✅ `book/src/components/AuthModal.tsx` - Fixed backend URL
3. ✅ `COMPLETE_AUTH_FIX_GUIDE.md` - Documentation
4. ✅ `QUICK_FIX_AUTH.md` - Quick reference
5. ✅ `auth-backend/FIX_RENDER_DEPLOYMENT.md` - Render guide

## Files to KEEP Uncommitted (Your Work):
- ❌ `book/src/components/TranslationPanel/` - Keep for later
- ❌ `book/src/contexts/TranslationContext.tsx` - Keep for later
- ❌ `auth-backend/.env.example` - Skip (not needed for production)

---

## Option 1: Commit to Current Branch (004-urdu-translation)

If you want to keep the auth fixes on your current branch:

```bash
# Stage only the auth fix files
git add book/src/contexts/AuthContext.tsx
git add book/src/components/AuthModal.tsx
git add COMPLETE_AUTH_FIX_GUIDE.md
git add QUICK_FIX_AUTH.md
git add auth-backend/FIX_RENDER_DEPLOYMENT.md

# Commit with descriptive message
git commit -m "fix: update auth backend URL to point to Render deployment

- Fixed AuthContext to use physical-ai-auth.onrender.com
- Fixed AuthModal to use correct backend URL
- Added comprehensive deployment fix guides
- Resolves 'Failed to fetch' error on GitHub Pages"

# Push to your current branch
git push origin 004-urdu-translation
```

---

## Option 2: Switch to 003-better-auth-user Branch (RECOMMENDED)

If you want the auth fix on the `003-better-auth-user` branch:

```bash
# Step 1: Stage only the auth fix files (don't stage Translation files!)
git add book/src/contexts/AuthContext.tsx
git add book/src/components/AuthModal.tsx
git add COMPLETE_AUTH_FIX_GUIDE.md
git add QUICK_FIX_AUTH.md
git add auth-backend/FIX_RENDER_DEPLOYMENT.md

# Step 2: Stash the staged changes with a name
git stash push -m "Auth backend URL fix for Render deployment"

# Step 3: Switch to the auth branch
git checkout 003-better-auth-user

# Step 4: Apply the stashed auth fix
git stash pop

# Step 5: Verify what's staged
git status

# Step 6: Commit the auth fix
git commit -m "fix: update auth backend URL to point to Render deployment

- Fixed AuthContext to use physical-ai-auth.onrender.com
- Fixed AuthModal to use correct backend URL  
- Added comprehensive deployment fix guides
- Resolves 'Failed to fetch' error on GitHub Pages
- Resolves Render exit code 128 (missing BETTER_AUTH_SECRET)"

# Step 7: Push to GitHub
git push origin 003-better-auth-user

# Step 8: Return to your translation work
git checkout 004-urdu-translation

# Your Translation files will still be there, uncommitted! ✅
```

---

## Option 3: Cherry-Pick After Commit (Advanced)

Commit on current branch, then cherry-pick to 003:

```bash
# Commit on current branch (004-urdu-translation)
git add book/src/contexts/AuthContext.tsx book/src/components/AuthModal.tsx COMPLETE_AUTH_FIX_GUIDE.md QUICK_FIX_AUTH.md auth-backend/FIX_RENDER_DEPLOYMENT.md
git commit -m "fix: update auth backend URL to Render deployment"

# Switch to auth branch
git checkout 003-better-auth-user

# Cherry-pick the commit you just made
git cherry-pick 004-urdu-translation

# Push
git push origin 003-better-auth-user

# Return to translation work
git checkout 004-urdu-translation
```

---

## 🎯 RECOMMENDED: Quick Commands (Option 2)

Copy and paste these commands one by one:

```bash
# Navigate to project root
cd f:\ai_projects\physical-ai-textbook

# Stage ONLY auth fix files
git add book/src/contexts/AuthContext.tsx book/src/components/AuthModal.tsx COMPLETE_AUTH_FIX_GUIDE.md QUICK_FIX_AUTH.md auth-backend/FIX_RENDER_DEPLOYMENT.md

# Create a stash for these changes
git stash push -m "Auth fix: backend URL to Render"

# Switch to auth branch
git checkout 003-better-auth-user

# Apply the auth fix
git stash pop

# Commit
git commit -m "fix: update auth backend URL to point to Render deployment"

# Push
git push origin 003-better-auth-user

# Return to your translation work
git checkout 004-urdu-translation
```

Now your translation files (`TranslationPanel/` and `TranslationContext.tsx`) will still be there, uncommitted! ✅

---

## Verification:

After running the commands above:

```bash
# Check your translation work is still there
git status
# Should show:
#   Untracked files:
#     book/src/components/TranslationPanel/
#     book/src/contexts/TranslationContext.tsx

# Check the auth branch got the fix
git log 003-better-auth-user --oneline -1
# Should show: "fix: update auth backend URL to point to Render deployment"
```

---

**Which option would you like to use?** I recommend **Option 2** (stash → switch → commit → return) ✅
