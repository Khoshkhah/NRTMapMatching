# Quick Start: Deploy to GitHub Pages 🚀

## ✅ What You Need to Do (3 Steps)

### 1️⃣ Push to GitHub

```bash
cd /home/kaveh/projects/NRTMapMatching
git add .
git commit -m "Add documentation with GitHub Pages setup"
git push origin main
```

### 2️⃣ Enable GitHub Pages

1. Go to: https://github.com/khoshkhah/NRTMapMatching
2. Click **Settings** → **Pages** (left sidebar)
3. Under **Source**:
   - Branch: `gh-pages`
   - Folder: `/ (root)`
4. Click **Save**

### 3️⃣ View Your Documentation

After 2-3 minutes, visit:
**https://khoshkhah.github.io/NRTMapMatching**

## 🎯 What Happens Automatically

Once you push:
- ✅ GitHub Actions workflow runs automatically
- ✅ Builds your documentation using MkDocs
- ✅ Deploys to `gh-pages` branch
- ✅ Makes it available on GitHub Pages

## 📍 Check Status

**Monitor the deployment:**
1. Go to your repository on GitHub
2. Click the **Actions** tab
3. Watch the "Deploy Documentation" workflow run

**First time?** It may take 5-10 minutes to appear.

## 🔄 Future Updates

Just push changes to `docs/` or `mkdocs.yml` and documentation updates automatically!

---

**Need help?** See [DEPLOY.md](DEPLOY.md) for detailed instructions.

