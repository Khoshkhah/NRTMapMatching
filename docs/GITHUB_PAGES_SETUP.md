# Quick Setup Guide for GitHub Pages

## What is GitHub Pages?

GitHub Pages is a free hosting service that turns your GitHub repository into a website. Your documentation will be accessible at:
```
https://YOUR_USERNAME.github.io/NRTMapMatching
```

## Quick Setup (5 minutes)

### 1. Update Repository URLs

Edit `mkdocs.yml` and replace `YOUR_USERNAME` with your GitHub username in two places:

```yaml
repo_url: https://github.com/YOUR_USERNAME/NRTMapMatching
```

And:

```yaml
extra:
  social:
    - icon: fontawesome/brands/github
      link: https://github.com/YOUR_USERNAME/NRTMapMatching
```

### 2. Commit and Push

```bash
git add .
git commit -m "Add GitHub Pages documentation setup"
git push origin main
```

### 3. Enable GitHub Pages

1. Go to your repository on GitHub
2. Click **Settings** → **Pages**
3. Under **Source**, select:
   - Branch: `gh-pages`
   - Folder: `/ (root)`
4. Click **Save**

### 4. Wait for Deployment

1. Go to the **Actions** tab in your repository
2. Wait for the "Deploy Documentation" workflow to complete (1-2 minutes)
3. Your documentation will be live at:
   ```
   https://YOUR_USERNAME.github.io/NRTMapMatching
   ```

## That's It! 🎉

Your documentation is now live on GitHub Pages. Every time you push changes to the `docs/` folder or `mkdocs.yml`, the documentation will automatically rebuild and deploy.

## Preview Locally

To preview changes before pushing:

```bash
pip install -r requirements-docs.txt
mkdocs serve
```

Then open http://127.0.0.1:8000

## Troubleshooting

- **Can't find Pages settings?** Make sure you're the repository owner or have admin access
- **Workflow not running?** Check the Actions tab - it should trigger automatically on push
- **Documentation not updating?** Wait a few minutes for GitHub Pages to rebuild

## Need Help?

See the full setup guide in [SETUP.md](SETUP.md) for detailed instructions and troubleshooting.

