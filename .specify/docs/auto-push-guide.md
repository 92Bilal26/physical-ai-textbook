# Auto-Push to GitHub Guide

This guide explains how to automatically push changes to GitHub when you edit files in this project.

---

## ⚠️ **Important Considerations**

**Pros:**
- ✅ Never forget to push changes
- ✅ Automatic backups
- ✅ Changes immediately available on GitHub

**Cons:**
- ❌ Creates many small commits (can clutter history)
- ❌ Might push broken/incomplete code
- ❌ Less control over commit messages
- ❌ Can cause conflicts if working from multiple machines

**Recommendation:** Use auto-push for **personal projects** or **learning environments**. For professional/team projects, use manual commits.

---

## 🎯 **Option 1: Post-Commit Hook (Recommended)**

**What it does:** Automatically pushes to GitHub **after you create a commit**.

**Status:** ✅ **Already configured!**

**How it works:**
1. You create a commit manually (with meaningful message)
2. Git hook automatically pushes to GitHub
3. Your changes are on GitHub immediately

**Location:** `.git/hooks/post-commit`

**Test it:**
```bash
# Make a change
echo "test" >> test.txt

# Commit (will auto-push)
git add test.txt
git commit -m "test: testing auto-push"

# 🚀 Automatically pushed to GitHub!
```

---

## 🎯 **Option 2: File Watcher (Auto-Commit + Auto-Push)**

**What it does:** Watches for file changes and **automatically commits AND pushes** every 5 seconds.

**Status:** ✅ Created but not running (requires manual start)

**How to use:**

### **On Windows:**
```bash
# Open a new terminal/PowerShell
cd C:\WINDOWS\system32\physical-ai-textbook
.specify\scripts\auto-push.bat
```

### **On Linux/Mac/Git Bash:**
```bash
# Open a new terminal
cd /path/to/physical-ai-textbook
./.specify/scripts/auto-push.sh
```

**Keep the terminal open** while working. Press `Ctrl+C` to stop.

**Smart commit messages:**
- Constitution changes → `docs: update constitution`
- Skills/agents changes → `feat: update Claude Code skills/agents`
- History changes → `docs: add prompt history record`
- Other changes → `chore: auto-commit changes`

---

## 🎯 **Option 3: VSCode Extension (Manual Control)**

If you use VSCode, install **"GitDoc"** extension:

1. Open VSCode
2. Install extension: `arcsine.gitdoc`
3. Enable auto-commit in settings
4. Commits every X minutes automatically

**Advantage:** Works with any editor, visual control

---

## 🎯 **Option 4: GitHub Actions (CI/CD)**

**What it does:** GitHub automatically runs tasks when you push.

**Use case:** Auto-deploy Docusaurus site after push

**Setup:** Create `.github/workflows/deploy.yml` (can help with this later)

---

## 📋 **Current Setup Status**

| Feature | Status | Location |
|---------|--------|----------|
| Post-commit hook | ✅ Active | `.git/hooks/post-commit` |
| File watcher (Bash) | ⏸️ Ready | `.specify/scripts/auto-push.sh` |
| File watcher (Windows) | ⏸️ Ready | `.specify/scripts/auto-push.bat` |

---

## 🛠️ **Disable Auto-Push**

If you want to disable auto-push:

```bash
# Remove post-commit hook
rm .git/hooks/post-commit

# Or rename it to disable temporarily
mv .git/hooks/post-commit .git/hooks/post-commit.disabled
```

---

## 🧪 **Test Your Setup**

**Test Option 1 (Post-commit hook):**
```bash
echo "test" >> README.md
git add README.md
git commit -m "test: verify auto-push works"
# Should auto-push!
```

**Test Option 2 (File watcher):**
```bash
# Start watcher in one terminal
.specify/scripts/auto-push.sh  # or .bat on Windows

# In another terminal, make changes
echo "test" >> README.md

# Wait 5 seconds - should auto-commit and push!
```

---

## 🚨 **Troubleshooting**

**Push fails (authentication error):**
- Make sure SSH key is configured (you already did this!)
- Test: `ssh -T git@github.com`

**Hook doesn't run:**
- Check executable permission: `chmod +x .git/hooks/post-commit`
- Check file exists: `ls -la .git/hooks/post-commit`

**File watcher doesn't detect changes:**
- Make sure you're in project root directory
- Check if git is tracking the file: `git status`

---

## 💡 **Recommended Workflow**

For your Physical AI textbook project:

**Best approach:** Use **Option 1 (Post-commit hook)** ✅ Already active!

**Why:**
- You still write meaningful commit messages
- Changes push immediately after commit
- No clutter from auto-commits
- Works seamlessly with Claude Code

**When to use Option 2 (File watcher):**
- During intense development sessions
- When you want zero-friction backups
- When working alone on a personal branch

---

**Your current setup:** Option 1 is active! Just commit normally and it will auto-push. 🎉
