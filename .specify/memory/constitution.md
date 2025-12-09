<!--
Sync Impact Report:
Version change: None -> 1.0.0
List of modified principles: (initial creation)
Added sections: Core Principles, Project Scope, Development Environment & Workflow, Governance
Removed sections: None
Templates requiring updates:
  - .specify/templates/plan-template.md: ✅ updated
  - .specify/templates/spec-template.md: ✅ updated
  - .specify/templates/tasks-template.md: ✅ updated
  - .claude/commands/sp.adr.md: ✅ updated
  - .claude/commands/sp.analyze.md: ✅ updated
  - .claude/commands/sp.checklist.md: ✅ updated
  - .claude/commands/sp.clarify.md: ✅ updated
  - .claude/commands/sp.constitution.md: ✅ updated
  - .claude/commands/sp.git.commit_pr.md: ✅ updated
  - .claude/commands/sp.implement.md: ✅ updated
  - .claude/commands/sp.phr.md: ✅ updated
  - .claude/commands/sp.plan.md: ✅ updated
  - .claude/commands/sp.specify.md: ✅ updated
  - .claude/commands/sp.tasks.md: ✅ updated
  - .specify/templates/phr-template.prompt.md: ✅ updated
  - .specify/scripts/powershell/check-prerequisites.ps1: ✅ updated
  - .specify/scripts/powershell/common.ps1: ✅ updated
  - .specify/scripts/powershell/create-new-feature.ps1: ✅ updated
  - .specify/scripts/powershell/setup-plan.ps1: ✅ updated
  - .specify/scripts/powershell/update-agent-context.ps1: ✅ updated
  - CLAUDE.md: ✅ updated
Follow-up TODOs: None
-->
# Physical AI & Humanoid Robotics Constitution

## Core Principles

### I. Educational Focus
The primary purpose of this book is to teach the Physical AI & Humanoid Robotics course. Content must be tailored for students and beginners with basic Python knowledge, focusing on embodied AI and humanoid robotics.

### II. Structured Content
The book must adhere to a strict structure of exactly 4 modules, with each module containing exactly 4 chapters, totaling 16 chapters. Content must cover Module 1 (ROS 2), Module 2 (Gazebo & Unity), Module 3 (NVIDIA Isaac), and Module 4 (VLA and Autonomous Humanoid capstone).

### III. Pedagogical Style
Writing must be student-friendly, with clear explanations and a practical focus. All concepts should be accompanied by code and simulation examples, suggestions for diagrams, and relevant exercises to reinforce learning.

### IV. Docusaurus & Markdown Compliance
The book will be built with Docusaurus and written using Spec-Kit Plus + Claude Code. Content must be formatted in Markdown, ensuring compatibility with Docusaurus rendering.

## Project Scope

**Purpose:** To teach the Physical AI & Humanoid Robotics course.
**Target Audience:** Students and beginners who know basic Python and want to learn embodied AI and humanoid robotics.
**In Scope Modules:**
- Module 1: The Robotic Nervous System (ROS 2)
- Module 2: The Digital Twin (Gazebo & Unity)
- Module 3: The AI-Robot Brain (NVIDIA Isaac)
- Module 4: Vision-Language-Action (VLA) and the Autonomous Humanoid capstone
**Structure:** Exactly 4 modules, each with exactly 4 chapters (16 chapters total).

## Development Environment & Workflow

**Tooling:** Docusaurus, Spec-Kit Plus, Claude Code.
**Writing Style:** Student-friendly, clear explanations, practical focus, code + simulation examples, diagrams suggestions, and exercises.
**Formatting Style:** Markdown compatible with Docusaurus.

## Governance

This Constitution supersedes all other project practices. Amendments require documentation, approval, and a migration plan. All contributions and reviews must verify compliance with these principles. Use CLAUDE.md for runtime development guidance.

**Version**: 1.0.0 | **Ratified**: 2025-12-04 | **Last Amended**: 2025-12-04
