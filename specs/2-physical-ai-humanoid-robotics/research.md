# Research: Physical AI & Humanoid Robotics Textbook

## Decision: Docusaurus as Documentation Framework
**Rationale**: Docusaurus is the optimal choice for this textbook project because it's specifically designed for documentation websites with built-in features like:
- Integrated search functionality
- Versioning support
- Responsive design
- Easy content organization
- Markdown/MDX support
- Built-in analytics
- Deployment to GitHub Pages

**Alternatives considered**:
- Next.js with custom documentation system: Would require building documentation-specific features from scratch
- GitBook: Less flexible and modern than Docusaurus
- Hugo: More complex setup and less JavaScript-focused
- Sphinx: Python-focused, not ideal for web deployment

## Decision: RAG Chatbot Architecture
**Rationale**: The RAG (Retrieval-Augmented Generation) chatbot will be implemented using:
- Frontend: Embedded in Docusaurus pages with text selection capability
- Backend: FastAPI for the API layer
- Vector Database: Qdrant for efficient similarity search
- Language Model: OpenAI API for natural language understanding
- Storage: Neon Postgres for metadata and user interactions

This architecture allows users to select text in the textbook and ask questions about it, with the system retrieving relevant context and generating appropriate responses.

**Alternatives considered**:
- Client-side only: Would be limited by browser constraints and security
- Different vector databases (Pinecone, Weaviate): Qdrant is open-source and well-performing
- Different backend frameworks: FastAPI offers excellent performance and OpenAPI integration

## Decision: Content Structure and Organization
**Rationale**: The textbook content will be organized into 5 modules with 2 topics each, following the specified course outline:
- Module 1: The Robotic Nervous System (ROS 2)
- Module 2: The Digital Twin (Gazebo & Unity)
- Module 3: The AI-Robot Brain (NVIDIA Isaac)
- Module 4: Vision-Language-Action (VLA)
- Module 5: Capstone Autonomous Humanoid Project

Each module will have its own directory with individual topic files, allowing for clear organization and easy navigation.

**Alternatives considered**:
- Different organizational structures: The specified module structure aligns with the course requirements
- Flatter content organization: Hierarchical structure better supports learning progression

## Decision: Deployment Strategy
**Rationale**: GitHub Pages deployment is chosen because:
- It's explicitly requested in the requirements
- It provides free, reliable hosting for static sites
- It integrates well with Docusaurus
- It offers custom domain support
- It provides HTTPS by default

**Alternatives considered**:
- Vercel: Would work but GitHub Pages is specifically requested
- Netlify: Would work but GitHub Pages is specifically requested
- Self-hosting: Unnecessary complexity for this project

## Decision: Hardware and Cloud Integration Notes
**Rationale**: The textbook will include practical notes about:
- RTX-enabled workstations for AI/robotics computation
- Jetson Orin Nano edge kits for embedded AI
- RealSense sensors for computer vision
- Cloud alternatives (AWS RoboMaker, NVIDIA Omniverse Cloud) for users without hardware access
- Latency management strategies (cloud training → local deployment)

These notes will be integrated into relevant chapters to provide practical context for students.

**Alternatives considered**:
- Omitting hardware notes: Would reduce educational value
- Different hardware platforms: Selected platforms are industry-standard for robotics

## Decision: Bonus Features Implementation
**Rationale**: Optional bonus features will be implemented as follows:
- User personalization: Cookie-based preferences for content display
- Urdu translation: Plugin-based translation system for accessibility
- Subagents: Reusable components for different textbook sections

These features enhance the educational value without complicating the core functionality.

**Alternatives considered**:
- Different personalization approaches: Cookie-based is simple and effective
- Different translation methods: Plugin approach allows for easy maintenance