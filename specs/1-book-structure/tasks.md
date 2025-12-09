# Tasks: Book Structure

**Input**: Design documents from `/specs/1-book-structure/`
**Prerequisites**: plan.md (required), spec.md (required for user stories)

**Tests**: Not explicitly requested for this structural task. Content tests will be part of a later phase.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- All paths assume the project root `D:/Ai robotics Book/physical-ai-humanoid-robotics-book`

## Phase 1: Setup (Project Initialization)

**Purpose**: Initialize the Docusaurus project and basic folder structure.

- [ ] T001 Create `docs/` directory for all book content
- [ ] T002 Create `static/img/` directory for image assets
- [ ] T003 Initialize Docusaurus project (requires `npx create-docusaurus@latest` - user to execute)
- [ ] T004 Review and update `docusaurus.config.js` for basic book metadata
- [ ] T005 Update `package.json` with required Docusaurus dependencies

---

## Phase 2: Foundational (Module Structure)

**Purpose**: Create module directories and their `_category_.json` files for navigation.

- [ ] T006 [P] Create `docs/module01-ros2/` directory for Module 1
- [ ] T007 [P] Create `docs/module01-ros2/_category_.json` for Module 1 navigation
- [ ] T008 [P] Create `docs/module02-digital-twin/` directory for Module 2
- [ ] T009 [P] Create `docs/module02-digital-twin/_category_.json` for Module 2 navigation
- [ ] T010 [P] Create `docs/module03-isaac-brain/` directory for Module 3
- [ ] T011 [P] Create `docs/module03-isaac-brain/_category_.json` for Module 3 navigation
- [ ] T012 [P] Create `docs/module04-vla-humanoid/` directory for Module 4
- [ ] T013 [P] Create `docs/module04-vla-humanoid/_category_.json` for Module 4 navigation

**Checkpoint**: Module directories and basic navigation categories are in place.

---

## Phase 3: User Story 1 - Structured Learning Path (Priority: P1) 🎯 MVP

**Goal**: Establish a clear, progressive learning path by creating all 16 chapter Markdown files with initial frontmatter.

**Independent Test**: Verify that all chapter Markdown files are created in the correct module directories and have appropriate filenames and initial frontmatter for Docusaurus.

### Implementation for User Story 1

#### Module 1: The Robotic Nervous System (ROS 2)
- [ ] T014 [P] [US1] Create chapter file `docs/module01-ros2/01-intro-robotics-ros2.md`
- [ ] T015 [P] [US1] Create chapter file `docs/module01-ros2/02-ros2-nodes-topics.md`
- [ ] T016 [P] [US1] Create chapter file `docs/module01-ros2/03-ros2-services-actions-params.md`
- [ ] T017 [P] [US1] Create chapter file `docs/module01-ros2/04-ros2-advanced-tools.md`

#### Module 2: The Digital Twin (Gazebo & Unity)
- [ ] T018 [P] [US1] Create chapter file `docs/module02-digital-twin/01-intro-simulation-urdf.md`
- [ ] T019 [P] [US1] Create chapter file `docs/module02-digital-twin/02-gazebo-simulation.md`
- [ ] T020 [P] [US1] Create chapter file `docs/module02-digital-twin/03-unity-robotics-simulation.md`
- [ ] T021 [P] [US1] Create chapter file `docs/module02-digital-twin/04-ros2-digital-twins.md`

#### Module 3: The AI-Robot Brain (NVIDIA Isaac)
- [ ] T022 [P] [US1] Create chapter file `docs/module03-isaac-brain/01-intro-nvidia-isaac.md`
- [ ] T023 [P] [US1] Create chapter file `docs/module03-isaac-brain/02-isaac-sim-high-fidelity.md`
- [ ] T024 [P] [US1] Create chapter file `docs/module03-isaac-brain/03-isaac-perception-navigation.md`
- [ ] T025 [P] [US1] Create chapter file `docs/module03-isaac-brain/04-isaac-manipulation-learning.md`

#### Module 4: Vision-Language-Action (VLA) and the Autonomous Humanoid Capstone
- [ ] T026 [P] [US1] Create chapter file `docs/module04-vla-humanoid/01-intro-vlm.md`
- [ ] T027 [P] [US1] Create chapter file `docs/module04-vla-humanoid/02-language-robotics-integration.md`
- [ ] T028 [P] [US1] Create chapter file `docs/module04-vla-humanoid/03-action-from-vla.md`
- [ ] T029 [P] [US1] Create chapter file `docs/module04-vla-humanoid/04-humanoid-capstone.md`

**Checkpoint**: All 16 chapter Markdown files are created, providing the complete book structure.

---

## Phase 4: Polish & Cross-Cutting Concerns

**Purpose**: Finalize Docusaurus configuration and validate the overall structure.

- [ ] T030 Update main `docs/_category_.json` for overall book navigation
- [ ] T031 Run Docusaurus build command (`npm run build` or `yarn build`) to validate structure (user to execute)
- [ ] T032 Review Docusaurus site locally to ensure all modules and chapters are navigable

---

## Dependencies & Execution Order

### Phase Dependencies

-   **Setup (Phase 1)**: No dependencies - can start immediately
-   **Foundational (Phase 2)**: Depends on Setup completion
-   **User Story 1 (Phase 3)**: Depends on Foundational phase completion
-   **Polish (Phase 4)**: Depends on User Story 1 phase completion

### User Story Dependencies

-   **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories for structural setup.
-   *User Story 2 (P1) and User Story 3 (P2)*: These stories are primarily content-focused and will be addressed during the actual content writing phase. The structural tasks generated here provide the foundation for their implementation but do not directly include file creation tasks for these stories.

### Within Each User Story

- Tasks within each phase should be completed sequentially where dependencies exist, or in parallel where indicated by `[P]`.

### Parallel Opportunities

- Tasks marked with `[P]` within each phase can be executed in parallel.
- Module directory and category file creation (Phase 2) can be done in parallel.
- All chapter Markdown file creations (Phase 3) can be done in parallel.

---

## Parallel Example: Chapter File Creation (Phase 3)

```bash
Task: "Create chapter file docs/module01-ros2/01-intro-robotics-ros2.md"
Task: "Create chapter file docs/module01-ros2/02-ros2-nodes-topics.md"
# ... (all 16 chapter files can be created in parallel)
```

---

## Implementation Strategy

### MVP First (Structural Foundation)

1.  Complete Phase 1: Setup
2.  Complete Phase 2: Foundational
3.  Complete Phase 3: User Story 1 (all chapter files created)
4.  Complete Phase 4: Polish & Cross-Cutting Concerns (Docusaurus build validation)
5.  **STOP and VALIDATE**: Ensure the Docusaurus site builds and displays the correct structure with empty chapters.

### Incremental Delivery

Subsequent phases would involve content creation for each chapter, adding code examples, diagrams, and exercises, building upon this established structure.

## Notes

-   Tasks are focused on creating the Docusaurus structure. Content writing, code examples, simulation details, and diagram generation will be separate, subsequent tasks.
-   The `[P]` marker indicates tasks that can run in parallel.
-   User Story 2 (Practical Skill Development) and User Story 3 (Visual Concept Reinforcement) will be fulfilled by content creation within the chapter Markdown files.
