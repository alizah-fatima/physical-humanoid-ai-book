# ADR: Content Ingestion Pipeline Architecture

**Date**: 2025-12-24
**Status**: Accepted
**Author**: Claude

## Context

We need to implement an ingestion pipeline that will crawl the Physical AI & Humanoid Robotics textbook website, extract text content, generate embeddings using Cohere, and store them in Qdrant with metadata. The system needs to handle the entire process from URL discovery to vector storage.

## Decision

We will implement a single-file Python application with the following architectural choices:

1. **Single-File Architecture**: All functionality in main.py for simplicity and ease of deployment
2. **Function-Based Design**: Separate functions for each step of the pipeline
3. **Cohere Embeddings**: Using Cohere's embed-multilingual-v3.0 model for text embeddings
4. **Qdrant Vector Database**: For storing embeddings with metadata
5. **Batch Processing**: To handle API rate limits efficiently
6. **Context-Preserving Chunking**: Overlapping chunks to maintain semantic context

## Alternatives Considered

1. **Multi-File Architecture**:
   - Pros: Better separation of concerns, more maintainable
   - Cons: More complex deployment, overkill for this specific use case

2. **Different Embedding Models**:
   - OpenAI embeddings: Higher cost, different API structure
   - Local embeddings: Higher computational requirements, more complex setup
   - Sentence Transformers: Self-hosted option but requires more infrastructure

3. **Different Vector Databases**:
   - Pinecone: Proprietary, different API
   - Weaviate: Alternative open-source option
   - Elasticsearch: More general-purpose, less optimized for vector search

## Consequences

### Positive
- Simple deployment with single file
- Clear separation of concerns within functions
- Efficient batch processing for API rate limits
- Proper error handling and logging
- Configurable via environment variables

### Negative
- Single file may become unwieldy as features grow
- Potential memory issues with very large sites (mitigated by batch processing)
- Dependency on external APIs (Cohere, Qdrant)

## Technical Details

- Embedding dimension: 1024 (Cohere multilingual model)
- Chunk size: Configurable (default 1000 characters)
- Chunk overlap: Configurable (default 100 characters)
- Batch size: Configurable (default 10) for API calls
- Distance metric: Cosine similarity in Qdrant