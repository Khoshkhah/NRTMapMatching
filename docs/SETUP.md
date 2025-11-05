# Setting Up GitHub Pages Documentation

This guide will help you set up GitHub Pages for your documentation using MkDocs.

## Prerequisites

- A GitHub repository
- Python 3.7+ installed
- Git installed

## Step 1: Update Configuration

1. Open `mkdocs.yml` in the root directory
2. Replace `YOUR_USERNAME` with your GitHub username:
   ```yaml
   repo_url: https://github.com/YOUR_USERNAME/NRTMapMatching
   ```
   And in the social links section:
   ```yaml
   social:
     - icon: fontawesome/brands/github
       link: https://github.com/YOUR_USERNAME/NRTMapMatching
   ```

## Step 2: Install Dependencies Locally (Optional)

To preview the documentation locally:

```bash
pip install -r requirements-docs.txt
```

## Step 3: Test Locally

Start the development server:

```bash
mkdocs serve
```

Open http://127.0.0.1:8000 in your browser to preview your documentation.

## Step 4: Enable GitHub Pages

1. Go to your GitHub repository
2. Click on **Settings**
3. Scroll down to **Pages** in the left sidebar
4. Under **Source**, select:
   - Branch: `gh-pages`
   - Folder: `/ (root)`
5. Click **Save**

## Step 5: Push to GitHub

The GitHub Actions workflow will automatically build and deploy your documentation when you push to the main branch:

```bash
git add .
git commit -m "Add MkDocs documentation setup"
git push origin main
```

## Step 6: Verify Deployment

1. Go to the **Actions** tab in your GitHub repository
2. You should see a workflow running called "Deploy Documentation"
3. Wait for it to complete (usually takes 1-2 minutes)
4. Once complete, your documentation will be available at:
   ```
   https://YOUR_USERNAME.github.io/NRTMapMatching
   ```

## Manual Deployment (Alternative)

If you prefer to deploy manually instead of using GitHub Actions:

```bash
mkdocs gh-deploy
```

This will build the documentation and push it to the `gh-pages` branch.

## Troubleshooting

### Documentation not appearing

1. Check that GitHub Pages is enabled in repository settings
2. Verify the workflow completed successfully in the Actions tab
3. Wait a few minutes for GitHub Pages to update
4. Clear your browser cache

### Workflow fails

1. Check the Actions tab for error messages
2. Ensure `mkdocs.yml` is valid YAML
3. Verify all documentation files are in the `docs/` folder

### Local preview not working

1. Ensure all dependencies are installed: `pip install -r requirements-docs.txt`
2. Check that `mkdocs.yml` is in the root directory
3. Verify all markdown files exist in the `docs/` folder

## Customization

### Change Theme

Edit `mkdocs.yml` to use a different theme:

```yaml
theme:
  name: readthedocs  # or other themes
```

### Add Plugins

Add plugins in `mkdocs.yml`:

```yaml
plugins:
  - search
  - minify
```

### Custom Domain

To use a custom domain:

1. Create a file named `CNAME` in the `docs/` folder with your domain
2. Configure DNS settings for your domain
3. GitHub Pages will automatically detect and use the CNAME file

## Updating Documentation

Simply edit the markdown files in the `docs/` folder and push to GitHub. The workflow will automatically rebuild and deploy the documentation.

## Features

The documentation includes:

- **Search functionality** - Full-text search across all pages
- **Dark mode** - Toggle between light and dark themes
- **Responsive design** - Works on desktop and mobile
- **Code highlighting** - Syntax highlighting for code blocks
- **Navigation** - Easy navigation with tabs and sections
- **Mobile-friendly** - Optimized for mobile devices

