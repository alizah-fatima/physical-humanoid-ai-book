# AI-driven Textbook on Physical AI & Humanoid Robotics

This repository contains an educational textbook focused on Physical AI and Humanoid Robotics, designed to provide comprehensive coverage of cutting-edge topics in artificial intelligence applied to physical systems and humanoid robots.

The repository also includes a content ingestion pipeline that crawls the deployed documentation site, extracts text content, generates embeddings using Cohere, and stores them in Qdrant with metadata for RAG (Retrieval Augmented Generation) functionality.

## Overview

This AI-driven textbook combines traditional educational content with modern technology to create an engaging learning experience. The content covers fundamental concepts to advanced applications in Physical AI and Humanoid Robotics, with interactive elements and AI-powered assistance.

## Features

- **Educational Content**: Comprehensive coverage of Physical AI and Humanoid Robotics topics
- **Modern Platform**: Built with Docusaurus for optimal user experience
- **Multilingual Support**: On-demand Urdu translation for broader accessibility
- **AI Integration**: Embedded RAG chatbot for intelligent Q&A based on book content
- **Responsive Design**: Clean, professional UI that works across all devices
- **GitHub Pages Deployment**: Static site deployed for easy access
- **Content Ingestion Pipeline**: Automated pipeline for crawling, embedding, and storing textbook content

## Content Ingestion Pipeline

The repository includes a complete implementation of an ingestion pipeline with the following capabilities:

- **Web Crawling**: Discovers all accessible pages from the target documentation site
- **Content Extraction**: Extracts clean text content and chapter titles from HTML pages
- **Text Chunking**: Splits content into context-preserving chunks with configurable size
- **Embedding Generation**: Creates vector embeddings using Cohere's API
- **Vector Storage**: Stores embeddings with metadata in Qdrant vector database
- **Error Handling**: Comprehensive error handling and logging for robust operation

### Pipeline Architecture

The implementation follows a single-file Python architecture with modular functions:

- `get_all_urls()`: Discovers all accessible URLs on the target site
- `extract_text_from_url()`: Extracts clean text and chapter titles from a URL
- `chunk_text()`: Splits text into chunks with context preservation
- `embed()`: Generates embeddings for text chunks using Cohere
- `create_collection()`: Creates Qdrant collection for vector storage
- `save_chunk_to_qdrant()`: Stores embeddings with metadata to Qdrant

### Setup

1. Navigate to the backend directory:
   ```bash
   cd backend
   ```

2. Install dependencies:
   ```bash
   pip install -r requirements.txt
   ```

3. Configure environment variables in `.env`:
   ```env
   COHERE_API_KEY=your_cohere_api_key
   QDRANT_URL=your_qdrant_cluster_url
   QDRANT_API_KEY=your_qdrant_api_key
   BASE_URL=https://alizah-fatima.github.io/physical-humanoid-ai-book/
   ```

### Usage

Run the ingestion pipeline:
```bash
python main.py
```

## Core Principles

This project follows the principles outlined in our [Constitution](./.specify/memory/constitution.md):

- Educational integrity with accurate, pedagogically sound content
- Technical excellence with minimal latency and reliable delivery
- High-quality content standards using proper Markdown formatting
- Multilingual accessibility with Urdu translation support
- Superior user experience with intuitive navigation
- Functional completeness with AI-powered assistance

## Structure

- `docs/` - Educational content organized by chapters/modules
- `backend/` - Content ingestion pipeline implementation
- `.docusaurus/` - Docusaurus build artifacts
- `.specify/` - Project specifications and governance documents
- `history/` - Prompt History Records and project evolution
- `specs/1-ingestion-pipeline/` - Implementation specifications and documentation
- `src/` - Custom source code for enhanced functionality

## Getting Started

1. Clone the repository
2. For the textbook: Install dependencies with `npm install` and start with `npm start`
3. For the ingestion pipeline: Navigate to `backend/` and follow the setup instructions above

## Contributing

Contributions are welcome! Please follow the guidelines outlined in our project constitution and ensure all content maintains educational integrity.

## License

This project is licensed under the terms specified in the LICENSE file.