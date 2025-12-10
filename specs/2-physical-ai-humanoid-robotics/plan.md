# Implementation Plan: Physical AI & Humanoid Robotics Textbook

**Branch**: `2-physical-ai-humanoid-robotics` | **Date**: 2025-12-09 | **Spec**: [link to be updated]

**Input**: Feature specification for Physical AI & Humanoid Robotics textbook project

## Summary

Create a comprehensive AI-Native textbook for teaching "Physical AI & Humanoid Robotics" course using Docusaurus, deployable to GitHub Pages, with integrated RAG chatbot. The project will include 5+ modules/chapters, smooth UX with mobile-first design, dark mode, and progress tracking. Optional bonus features include user personalization and content translation.

## Technical Context

**Language/Version**: JavaScript/TypeScript (Node.js 18+)
**Primary Dependencies**: Docusaurus 3.0, React 18, OpenAI SDK, FastAPI (Python), Qdrant vector database, Neon Postgres
**Storage**: GitHub Pages (static hosting), with optional Neon Postgres for chatbot backend and Qdrant for RAG
**Testing**: Jest for frontend, pytest for backend APIs
**Target Platform**: Web browser (mobile-first responsive design), GitHub Pages deployment
**Project Type**: Web application with static site generation
**Performance Goals**: Fast page loads with Core Web Vitals scores >0.9, mobile-responsive design
**Constraints**: Static site deployable to GitHub Pages, bundle size optimized for fast loading, mobile-first approach
**Scale/Scope**: Educational textbook with 5+ chapters, each containing 2 topics; support for RAG chatbot queries

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

Based on the current constitution for the AI-Native Driven Development book website, I need to evaluate how this new project aligns:

**Potential Violations to Address:**
1. Technology stack: New project uses Docusaurus instead of Next.js/Tailwind
2. Hosting: GitHub Pages vs Vercel (acceptable as both are static hosting)
3. UI Framework: Docusaurus default vs shadcn/ui + Tailwind requirement
4. Content format: Docusaurus MDX vs MDX in specific directory structure
5. Page structure: Docusaurus standard pages vs 4-page constraint

**Justification for Differences:**
1. Docusaurus is specifically designed for documentation/textbook websites and is more appropriate for this use case than Next.js
2. GitHub Pages is explicitly requested in the requirements and is a valid static hosting solution
3. Docusaurus has its own styling system which is more appropriate for documentation than shadcn/ui
4. Docusaurus uses MDX content which aligns with the content management principle
5. Docusaurus provides standard documentation pages (home, docs, blog) which can accommodate the required functionality

## Project Structure

### Documentation (this feature)

```text
specs/2-physical-ai-humanoid-robotics/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
docusaurus.config.js
package.json
tsconfig.json
static/
├── img/
│   └── logo.svg
└── ...

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
└── ...

api/
├── rag-chatbot/
│   ├── main.py
│   ├── models.py
│   ├── database.py
│   └── requirements.txt
└── ...

plugins/
└── docusaurus-plugin-urdu-translation/
    ├── src/
    └── index.js

public/
└── ...

.babelrc
.gitignore
README.md
```

**Structure Decision**: Selected web application structure with Docusaurus for documentation generation and a separate API directory for the RAG chatbot backend. The docs/ directory will contain the textbook content organized by modules, with src/ containing custom React components for enhanced functionality like the RAG chatbot interface.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| Docusaurus instead of Next.js | Docusaurus is purpose-built for documentation/textbook sites with built-in features like versioning, search, and navigation | Next.js would require building documentation-specific features from scratch |
| Docusaurus styling instead of shadcn/ui | Docusaurus provides documentation-optimized UI components and themes | Custom UI framework would not integrate well with Docusaurus's built-in features |
| 5+ module pages instead of 4-page constraint | Textbook structure requires multiple chapter pages for educational content | 4-page constraint is specific to previous project and not applicable to documentation site |