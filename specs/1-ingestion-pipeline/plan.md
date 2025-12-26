# Implementation Plan: Content Ingestion Pipeline for RAG System

**Feature**: 1-ingestion-pipeline
**Created**: 2025-12-24
**Status**: Draft
**Plan Version**: 1.0

## Technical Context

This plan outlines the implementation of a content ingestion pipeline that will crawl the Physical AI & Humanoid Robotics textbook website, extract text content, generate embeddings, and store them in a vector database. The implementation will be contained in a single Python file (`main.py`) with specific functions for each step of the process.

### Technology Stack
- Python 3.9+
- UV package manager
- Cohere API for embeddings
- Qdrant Cloud for vector storage
- BeautifulSoup for HTML parsing
- Requests for HTTP operations
- Deployment URL: https://alizah-fatima.github.io/physical-humanoid-ai-book/
- SiteMap URL: https://alizah-fatima.github.io/physical-humanoid-ai-book/sitemap.xml

### Architecture Overview
- Single-file implementation in `main.py`
- Functions: `get_all_urls`, `extract_text_from_url`, `chunk_text`, `embed`, `create_collection`, `save_chunk_to_qdrant`
- Main execution function orchestrates the pipeline

## Constitution Check

### Code Quality Requirements
- Follow Python PEP 8 standards
- Include type hints for all functions
- Add comprehensive error handling
- Include logging for debugging and monitoring

### Security Requirements
- Secure API key handling (environment variables)
- Validate all external inputs
- Implement rate limiting for API calls

### Performance Requirements
- Efficient memory usage during crawling
- Resumable processing in case of failures
- Parallel processing where appropriate

### Architecture Requirements
- Modular function design
- Single responsibility principle
- Configurable parameters

## Gates

### ✅ Feasibility Gate
The proposed implementation is technically feasible using the specified technologies.

### ✅ Architecture Gate
The single-file architecture with well-defined functions aligns with the requirements.

### ✅ Security Gate
The plan includes secure handling of API keys and input validation.

### ✅ Performance Gate
The plan includes considerations for efficient processing and resumable operations.

---

## Phase 0: Research & Discovery

### Research Tasks

1. **Python Web Crawling Best Practices**
   - Identify optimal libraries for crawling static sites
   - Determine best approaches for link discovery and navigation
   - Research handling of JavaScript-rendered content

2. **Text Extraction from HTML**
   - Best practices for extracting clean text from HTML
   - Handling of different HTML structures and elements
   - Preservation of semantic meaning during extraction

3. **Text Chunking Strategies**
   - Optimal chunk sizes for embedding generation
   - Strategies for maintaining context across chunks
   - Overlap techniques to preserve context

4. **Cohere Embedding API Integration**
   - API rate limits and best practices
   - Optimal batch sizes for embedding requests
   - Error handling for API calls

5. **Qdrant Vector Database Integration**
   - Collection creation and management
   - Optimal vector storage strategies
   - Metadata storage best practices

### Dependencies to Research
- UV package manager usage and setup
- Environment variable management for API keys
- Error handling patterns for external API calls

---

## Phase 1: Design & Architecture

### Data Model

#### Text Chunk Entity
- **id**: Unique identifier for the chunk
- **content**: The actual text content of the chunk
- **source_url**: URL where the content was found
- **chapter_title**: Title of the chapter/section
- **position**: Position of the chunk within the document
- **embedding**: Vector representation of the content

#### Metadata Entity
- **source_url**: Original URL of the content
- **chapter_title**: Title of the chapter/section
- **created_at**: Timestamp of when the chunk was processed
- **chunk_index**: Index of the chunk within the document

### API Contracts

#### Internal Function Contracts

```python
def get_all_urls(base_url: str) -> List[str]:
    """
    Crawls the base URL and returns all accessible page URLs.

    Args:
        base_url: The root URL to start crawling from

    Returns:
        List of all discovered URLs within the site

    Raises:
        ConnectionError: If unable to connect to the base URL
        ValueError: If the base URL is invalid
    """
```

```python
def extract_text_from_url(url: str) -> Tuple[str, str]:
    """
    Extracts clean text content and chapter title from a URL.

    Args:
        url: The URL to extract content from

    Returns:
        Tuple of (chapter_title, clean_text_content)

    Raises:
        HTTPError: If unable to fetch the URL
        ValueError: If no content could be extracted
    """
```

```python
def chunk_text(text: str, chunk_size: int = 1000) -> List[Dict[str, str]]:
    """
    Splits text into chunks of specified size with context preservation.

    Args:
        text: The text to be chunked
        chunk_size: Maximum size of each chunk (default 1000 characters)

    Returns:
        List of dictionaries containing chunk information
    """
```

```python
def embed(text_chunks: List[str]) -> List[List[float]]:
    """
    Generates embeddings for a list of text chunks.

    Args:
        text_chunks: List of text chunks to embed

    Returns:
        List of embedding vectors (each a list of floats)

    Raises:
        APIError: If embedding API call fails
    """
```

```python
def create_collection(collection_name: str) -> bool:
    """
    Creates a vector collection in Qdrant if it doesn't exist.

    Args:
        collection_name: Name of the collection to create

    Returns:
        True if collection was created or already exists
    """
```

```python
def save_chunk_to_qdrant(chunk_data: Dict, embedding: List[float]) -> bool:
    """
    Saves a text chunk with its embedding to Qdrant.

    Args:
        chunk_data: Dictionary containing chunk content and metadata
        embedding: The embedding vector for the chunk

    Returns:
        True if successfully saved
    """
```

### System Architecture

#### Main Pipeline Flow
1. Initialize configuration and API clients
2. Create vector collection in Qdrant
3. Discover all URLs from the base site
4. For each URL:
   - Extract text content and metadata
   - Chunk the text appropriately
   - Generate embeddings for chunks
   - Save to Qdrant with metadata
5. Validate completion and generate summary

#### Error Handling Strategy
- Retry mechanisms for network failures
- Graceful degradation when individual pages fail
- Progress tracking and resumable processing
- Comprehensive logging for debugging

### Quickstart Guide

#### Prerequisites
- Python 3.9+
- UV package manager
- Cohere API key
- Qdrant Cloud API key and endpoint

#### Setup
1. Clone the repository
2. Install dependencies with UV
3. Set environment variables for API keys
4. Run the main script

#### Configuration
- Base URL: Set to the deployment URL
- Chunk size: Configurable parameter
- Batch size for embeddings: Configurable parameter

---

## Phase 2: Implementation Tasks

### Task 1: Project Setup
- [ ] Create backend directory
- [ ] Initialize Python project with UV
- [ ] Set up virtual environment
- [ ] Install required dependencies

### Task 2: API Client Setup
- [ ] Configure Cohere client
- [ ] Configure Qdrant client
- [ ] Implement API key validation
- [ ] Set up rate limiting

### Task 3: Web Crawling Implementation
- [ ] Implement `get_all_urls` function
- [ ] Handle relative and absolute URLs
- [ ] Implement proper URL filtering
- [ ] Add error handling for inaccessible pages

### Task 4: Text Extraction
- [ ] Implement `extract_text_from_url` function
- [ ] Use BeautifulSoup for HTML parsing
- [ ] Extract chapter titles and clean text
- [ ] Handle different HTML structures

### Task 5: Text Chunking
- [ ] Implement `chunk_text` function
- [ ] Implement context-preserving chunking
- [ ] Add configurable chunk size
- [ ] Handle edge cases for small documents

### Task 6: Embedding Generation
- [ ] Implement `embed` function
- [ ] Handle batch processing for efficiency
- [ ] Implement retry logic for API failures
- [ ] Add rate limiting compliance

### Task 7: Vector Storage
- [ ] Implement `create_collection` function
- [ ] Implement `save_chunk_to_qdrant` function
- [ ] Design metadata schema
- [ ] Implement error handling for storage operations

### Task 8: Main Pipeline
- [ ] Implement main execution function
- [ ] Add progress tracking
- [ ] Implement resumable processing
- [ ] Add comprehensive logging

### Task 9: Testing & Validation
- [ ] Test with sample URLs
- [ ] Validate embedding quality
- [ ] Verify storage in Qdrant
- [ ] Performance testing

---

## Risk Analysis

### Technical Risks
- **API Rate Limits**: Cohere and Qdrant have rate limits that could slow processing
- **Large Site Size**: The textbook site may have many pages, requiring efficient processing
- **HTML Variations**: Different page structures may affect text extraction quality

### Mitigation Strategies
- Implement proper rate limiting and retry mechanisms
- Use parallel processing where appropriate
- Create robust HTML parsing that handles various structures

## Success Criteria

### Implementation Criteria
- All functions implemented as specified
- Code follows PEP 8 standards
- Type hints included for all functions
- Comprehensive error handling implemented

### Functional Criteria
- Successfully crawl all pages from the deployment URL
- Extract clean text content with appropriate metadata
- Generate embeddings without exceeding API limits
- Store all chunks in Qdrant with proper metadata

### Performance Criteria
- Process the entire site within reasonable time
- Handle API rate limits gracefully
- Resume processing if interrupted