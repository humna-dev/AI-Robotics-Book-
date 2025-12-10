# Quickstart Guide: Physical AI & Humanoid Robotics Textbook

## Development Setup

### Prerequisites
- Node.js 18+
- npm or yarn
- Python 3.9+ (for backend services)
- Git

### Initial Setup

1. **Clone and navigate to project:**
```bash
git clone <repository-url>
cd <project-name>
```

2. **Install frontend dependencies:**
```bash
npm install
```

3. **Set up backend (for RAG chatbot):**
```bash
cd api/rag-chatbot
pip install -r requirements.txt
```

4. **Environment variables:**
Create `.env` file in the project root:
```env
OPENAI_API_KEY=your_openai_api_key
QDRANT_URL=your_qdrant_url
QDRANT_API_KEY=your_qdrant_api_key
NEON_DB_URL=your_neon_db_url
```

### Running the Application

1. **For development:**
```bash
# Terminal 1: Start the Docusaurus development server
npm run start

# Terminal 2: Start the backend API server
cd api/rag-chatbot
python main.py
```

2. **For production build:**
```bash
# Build the static site
npm run build

# Serve the build locally
npm run serve
```

## Content Creation

### Adding a New Chapter
1. Create a new directory in `docs/` (e.g., `docs/module-1/`)
2. Add an `index.md` file for the chapter overview
3. Add individual topic files (e.g., `topic-1.md`, `topic-2.md`)
4. Update `sidebars.js` to include the new chapter in the navigation

### Content Format
- Use MDX format for all content files
- Include proper frontmatter with title, description, etc.
- Use Docusaurus markdown features for enhanced documentation

Example frontmatter:
```md
---
title: ROS 2 Nodes & Topics
description: Understanding the fundamental concepts of ROS 2
---

# ROS 2 Nodes & Topics

Content goes here...
```

## RAG Chatbot Integration

### Text Selection and Querying
The chatbot is integrated directly into the textbook pages. To use:
1. Select text in any chapter
2. Click the chatbot icon that appears
3. Ask your question about the selected text

### Backend Services
The RAG system consists of:
- FastAPI backend for API endpoints
- Qdrant vector database for document storage and similarity search
- OpenAI API for natural language processing

## Deployment

### GitHub Pages Deployment
1. Configure your GitHub repository for GitHub Pages
2. Set the source to the `/docs` folder or use GitHub Actions
3. The static site will be automatically deployed

### GitHub Actions Workflow
Example workflow file `.github/workflows/deploy.yml`:
```yaml
name: Deploy to GitHub Pages
on:
  push:
    branches: [main]
jobs:
  deploy:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v3
      - uses: actions/setup-node@v3
        with:
          node-version: 18
      - name: Install dependencies
        run: npm install
      - name: Build
        run: npm run build
      - name: Deploy to GitHub Pages
        uses: peaceiris/actions-gh-pages@v3
        with:
          github_token: ${{ secrets.GITHUB_TOKEN }}
          publish_dir: ./build
```

## Bonus Features

### Urdu Translation
To enable Urdu translation:
1. Use the "Translate to Urdu" button on any page
2. The translation is generated using the backend translation API
3. Translated content is cached for performance

### Personalization
User preferences are stored in browser localStorage:
- Theme (light/dark mode)
- Reading progress
- Personal notes