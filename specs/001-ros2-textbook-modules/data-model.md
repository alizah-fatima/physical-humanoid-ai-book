# Data Model: ROS 2 Textbook Module

## Entities

### Textbook Module
- **Name**: String (e.g., "Module 1: The Robotic Nervous System")
- **Description**: String (overview of the module content)
- **Chapters**: Array of Chapter entities
- **LearningObjectives**: Array of String (educational goals)
- **TargetAudience**: String (e.g., "AI/robotics students and professionals")

### Chapter
- **Title**: String (chapter name)
- **Content**: String (Markdown content)
- **LearningObjectives**: Array of String (specific objectives for this chapter)
- **CodeExamples**: Array of CodeExample entities
- **Diagrams**: Array of Diagram entities
- **Tables**: Array of Table entities
- **Prerequisites**: Array of String (required knowledge)
- **Keywords**: Array of String (important terms)

### CodeExample
- **Language**: String (e.g., "python", "bash", "yaml")
- **Code**: String (the actual code snippet)
- **Description**: String (explanation of the example)
- **FileName**: String (optional file name for context)

### Diagram
- **Type**: String (e.g., "mermaid", "plantuml", "image")
- **Content**: String (diagram definition or image path)
- **Title**: String (caption for the diagram)
- **Description**: String (explanation of what the diagram shows)

### Table
- **Headers**: Array of String (column headers)
- **Rows**: Array of Array of String (table data)
- **Title**: String (caption for the table)
- **Description**: String (explanation of the table's purpose)

## Relationships
- Module 1:N Chapter (one module contains many chapters)
- Chapter 1:N CodeExample (one chapter contains many code examples)
- Chapter 1:N Diagram (one chapter contains many diagrams)
- Chapter 1:N Table (one chapter contains many tables)

## Validation Rules
- Each Chapter must have a unique title within its Module
- Each Chapter must have content (not empty)
- CodeExample language must be a recognized programming language
- Module must have at least one Chapter
- All entities must have non-empty titles/descriptions