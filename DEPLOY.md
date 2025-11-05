# Deploy Documentation to GitHub Pages

Follow these steps to deploy your documentation to GitHub Pages.

## Step 1: Push Your Code to GitHub

If you haven't already, push your code to GitHub:

```bash
cd /home/kaveh/projects/NRTMapMatching

# Initialize git if not already done
git init

# Add all files
git add .

# Commit
git commit -m "Add documentation and GitHub Pages setup"

# Add your GitHub repository as remote (replace with your repo URL)
git remote add origin https://github.com/khoshkhah/NRTMapMatching.git

# Push to GitHub
git push -u origin main
```

> **Note**: If your default branch is `master` instead of `main`, use `master` in the commands above.

## Step 2: Enable GitHub Pages

1. Go to your GitHub repository: `https://github.com/khoshkhah/NRTMapMatching`
2. Click on **Settings** (top right of the repository)
3. Scroll down to **Pages** in the left sidebar
4. Under **Source**, select:
   - **Branch**: `gh-pages`
   - **Folder**: `/ (root)`
5. Click **Save**

## Step 3: Wait for Deployment

1. Go to the **Actions** tab in your repository
2. You should see a workflow called "Deploy Documentation" running
3. Wait for it to complete (usually 1-2 minutes)
4. The workflow will:
   - Build your documentation using MkDocs
   - Deploy it to the `gh-pages` branch
   - Make it available on GitHub Pages

## Step 4: View Your Documentation

Once the workflow completes, your documentation will be available at:

**https://khoshkhah.github.io/NRTMapMatching**

> ⚠️ **Note**: It may take a few minutes for GitHub Pages to update after the first deployment.

## Troubleshooting

### Workflow Not Running

- Check that you pushed the `.github/workflows/docs.yml` file
- Verify your branch name is `main` or `master`
- Check the Actions tab for any error messages

### Documentation Not Appearing

- Wait 5-10 minutes after enabling GitHub Pages (first deployment takes longer)
- Check the Actions tab to ensure the workflow completed successfully
- Verify GitHub Pages is enabled in Settings → Pages
- Clear your browser cache or try incognito mode

### Workflow Fails

- Check the Actions tab for error details
- Ensure `mkdocs.yml` is valid YAML
- Verify all documentation files are in the `docs/` folder

## Automatic Updates

After the initial setup, your documentation will automatically update whenever you:
- Push changes to `docs/` folder
- Update `mkdocs.yml`
- Push changes to `.github/workflows/docs.yml`

Just push your changes and the workflow will automatically rebuild and deploy!

## Preview Locally First

Before pushing, you can preview your documentation locally:

```bash
# Install dependencies
pip install -r requirements-docs.txt

# Start local server
mkdocs serve
```

Then open http://127.0.0.1:8000 in your browser.

