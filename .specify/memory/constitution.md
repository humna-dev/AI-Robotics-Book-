<!--
Sync Impact Report:
Version change: 1.0.0 → 1.1.0
Modified principles: Completely redefined to match AI-Native Driven Development – Premium Book Website requirements
Added sections: All new principles for book website project
Removed sections: Previous generic principles
Templates requiring updates: ✅ plan-template.md, ✅ spec-template.md, ✅ tasks-template.md (templates are generic and adaptable)
Follow-up TODOs: None
-->
# AI-Native Driven Development – Premium Book Website Constitution

## Core Principles

### I. Minimal UI Dependencies
Zero external UI libraries except shadcn/ui & Tailwind for consistent, lightweight design; All UI components must be built from these approved libraries only; External UI dependencies require explicit architectural approval

### II. Mobile-First Glassmorphism Design
Design and develop for mobile devices first; Implement glassmorphism and gradient design aesthetic throughout the interface; All visual elements must maintain aesthetic quality across all device sizes

### III. MDX Content Management
All book content must be authored in MDX format for easy future updates; Chapter content stored in /content/chapters/ directory; Content structure enables easy maintenance and modification

### IV. Performance Optimization
Lightning fast page loads with First Contentful Paint under 1.5 seconds; Total bundle size must remain under 150KB (excluding images); All performance metrics must be measured and verified before deployment

### V. Accessibility First
100% accessibility compliance with ARIA attributes and keyboard navigation support; All interactive elements must meet WCAG standards; Screen reader compatibility mandatory for all content

### VI. Dark/Light Mode Support
Dynamic dark and light mode toggle available across all pages; Theme preferences persist across user sessions; Both themes maintain equal visual quality and readability

## Additional Constraints

Page structure limited to 4 pages only: Home, Book, About, Contact; Book content consists of 5 chapters with 2 topics each; Chapter content must be stored in /content/chapters/ directory; Deployment restricted to Vercel platform (free tier)

## Development Workflow

Code must follow TypeScript strict mode for type safety; All components must be designed for reusability; Technology stack: Next.js 15, Tailwind CSS, MDX; Strict bundle size monitoring to maintain <150KB requirement

## Governance

All PRs must verify compliance with performance requirements; Code reviews must confirm accessibility standards are met; Bundle size checks must pass before merging; Dark/light mode functionality must be tested on all UI changes

**Version**: 1.1.0 | **Ratified**: 2025-12-09 | **Last Amended**: 2025-12-09