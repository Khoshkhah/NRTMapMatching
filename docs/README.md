# Documentation

This folder contains the documentation for NRTMapMatching.

## Local Development

To preview the documentation locally:

1. Install dependencies:
```bash
pip install -r requirements-docs.txt
```

2. Start the development server:
```bash
mkdocs serve
```

3. Open http://127.0.0.1:8000 in your browser

## Building Documentation

To build the documentation:

```bash
mkdocs build
```

The built files will be in the `site/` directory.

## Deploying to GitHub Pages

The documentation is automatically deployed to GitHub Pages when changes are pushed to the main branch. The GitHub Actions workflow (`.github/workflows/docs.yml`) handles the deployment automatically.

To manually deploy:

```bash
mkdocs gh-deploy
```

## Documentation Structure

- `index.md` - Documentation index/navigation
- `overview.md` - Overview and architecture
- `installation.md` - Installation guide
- `user-guide.md` - Usage examples and workflows
- `api-reference.md` - Complete API documentation
- `data-formats.md` - Data format specifications
- `algorithms.md` - Algorithm explanations
- `configuration.md` - Configuration parameters

