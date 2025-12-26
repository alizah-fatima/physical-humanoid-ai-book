# Research Document: Content Ingestion Pipeline

**Feature**: 1-ingestion-pipeline
**Created**: 2025-12-24

## Research Findings

### 1. Python Web Crawling Best Practices

**Decision**: Use requests and BeautifulSoup for crawling static sites
**Rationale**: The target site (GitHub Pages) is static HTML, so JavaScript rendering isn't required. requests + BeautifulSoup is lightweight and efficient for this use case.
**Alternatives considered**:
- Selenium (requires browser, heavy)
- Scrapy (overkill for simple site crawling)
- Playwright (also heavy for static content)

### 2. Text Extraction from HTML

**Decision**: Use BeautifulSoup with custom cleaning rules
**Rationale**: BeautifulSoup provides excellent HTML parsing capabilities and allows for custom extraction rules to remove navigation, headers, and other non-content elements.
**Alternatives considered**:
- newspaper3k (designed for news articles, may not work well with textbook structure)
- readability (good for articles, but requires more setup)

### 3. Text Chunking Strategies

**Decision**: RecursiveCharacterTextSplitter approach with 1000 character chunks and 100 character overlap
**Rationale**: This preserves context while keeping chunks small enough for embedding models. The overlap ensures semantic continuity.
**Alternatives considered**:
- Sentence-based splitting (might create very large chunks)
- Paragraph-based splitting (might break up related concepts)

### 4. Cohere Embedding API Integration

**Decision**: Use Cohere's embed-multilingual-v3.0 model with batch requests
**Rationale**: This model handles technical content well and supports batching for efficiency.
**Rate Limits**: Cohere has rate limits that vary by account type, typically requiring batch sizes of 96 or less.
**Alternatives considered**: OpenAI embeddings (different pricing and API structure)

### 5. Qdrant Vector Database Integration

**Decision**: Create collection with 1024-dimensional vectors (matching Cohere's output)
**Rationale**: Cohere embeddings are 1024-dimensional, so this matches the output format.
**Metadata**: Store source URL, chapter title, and chunk index as payload
**Alternatives considered**: Other vector databases like Pinecone, Weaviate (Qdrant was specified in requirements)

## Dependencies Research

### UV Package Manager
UV is a fast Python package installer and resolver. For this project, we'll use it to manage dependencies efficiently.

### Environment Variables for API Keys
Best practice is to use python-dotenv for local development and environment variables in production.

## Integration Patterns

### Error Handling
Implement retry mechanisms with exponential backoff for API calls and network requests.

### Progress Tracking
Use a simple file-based checkpoint system to track processed URLs and allow resumption.

## Technology-Specific Findings

### Python Libraries Required
- requests: for HTTP requests
- beautifulsoup4: for HTML parsing
- cohere: for embeddings
- qdrant-client: for vector storage
- python-dotenv: for environment management