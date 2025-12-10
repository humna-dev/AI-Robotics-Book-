# Physical AI & Humanoid Robotics Textbook Development Guidelines

Auto-generated from all feature plans. Last updated: 2025-12-09

## Active Technologies

- Docusaurus 3.0 (static site generator for documentation)
- React 18 (frontend component library)
- Node.js 18+ (runtime environment)
- JavaScript/TypeScript (primary languages)
- OpenAI SDK (for RAG chatbot)
- FastAPI (Python web framework for backend)
- Qdrant (vector database for RAG)
- Neon Postgres (PostgreSQL database service)
- GitHub Pages (deployment platform)

## Project Structure

```text
docusaurus.config.js
package.json
tsconfig.json
static/
├── img/
│   └── logo.svg
src/
├── components/
├── css/
├── pages/
└── theme/
docs/
├── module-1/
│   ├── index.md
│   ├── ros-nodes-topics.md
│   └── urdf-python-agent.md
├── module-2/
│   ├── index.md
│   ├── physics-simulation.md
│   └── sensors-environment-rendering.md
├── module-3/
│   ├── index.md
│   ├── isaac-ros-perception.md
│   └── path-planning-navigation.md
├── module-4/
│   ├── index.md
│   ├── voice-to-action.md
│   └── cognitive-planning-llms.md
├── module-5/
│   ├── index.md
│   ├── simulated-humanoid.md
│   └── obstacle-navigation.md
api/
├── rag-chatbot/
│   ├── main.py
│   ├── models.py
│   ├── database.py
│   └── requirements.txt
plugins/
└── docusaurus-plugin-urdu-translation/
    ├── src/
    └── index.js
public/
.babelrc
.gitignore
README.md
```

## Commands

### Frontend Development
```bash
# Install dependencies
npm install

# Start development server
npm run start

# Build for production
npm run build

# Serve production build locally
npm run serve
```

### Backend Development
```bash
# Navigate to backend
cd api/rag-chatbot

# Install Python dependencies
pip install -r requirements.txt

# Start backend server
python main.py
```

### Content Management
```bash
# Add new content to docs/ directory
# Update sidebars.js to include new content in navigation
```

## Code Style

### JavaScript/TypeScript
- Use functional components with React hooks
- Follow Docusaurus conventions for documentation
- Use TypeScript for type safety where possible
- Maintain consistent component structure

### Python (FastAPI)
- Use type hints for all function parameters and return values
- Follow PEP 8 style guide
- Use async/await for I/O operations
- Implement proper error handling

## Recent Changes

- Physical AI & Humanoid Robotics Textbook: Created comprehensive textbook with 5 modules, RAG chatbot, and multilingual support
- AI-Native Book Website: Implemented MDX-based book website with responsive design and dark mode

<!-- MANUAL ADDITIONS START -->
<!-- MANUAL ADDITIONS END -->