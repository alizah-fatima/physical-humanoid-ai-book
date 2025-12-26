# Data Model: Content Ingestion Pipeline

**Feature**: 1-ingestion-pipeline
**Created**: 2025-12-24

## Entity Definitions

### TextChunk
Represents a segment of extracted content from the textbook.

**Fields**:
- `id` (str): Unique identifier for the chunk (auto-generated)
- `content` (str): The actual text content of the chunk
- `source_url` (str): URL where the content was found
- `chapter_title` (str): Title of the chapter/section
- `chunk_index` (int): Position of the chunk within the document
- `created_at` (datetime): Timestamp of when the chunk was processed

**Validation Rules**:
- `content` must not be empty
- `source_url` must be a valid URL
- `chunk_index` must be non-negative

### ChunkMetadata
Metadata associated with each text chunk for storage in Qdrant.

**Fields**:
- `source_url` (str): Original URL of the content
- `chapter_title` (str): Title of the chapter/section
- `chunk_index` (int): Index of the chunk within the document
- `processed_at` (datetime): When the chunk was processed

**Validation Rules**:
- All fields are required
- `chunk_index` must be non-negative

### EmbeddingVector
High-dimensional representation of text content.

**Fields**:
- `vector` (List[float]): The embedding vector (1024-dimensional for Cohere)
- `chunk_id` (str): Reference to the associated TextChunk
- `model_version` (str): Version of the embedding model used

**Validation Rules**:
- `vector` must have exactly 1024 dimensions (for Cohere)
- `chunk_id` must reference an existing TextChunk

## Relationships

```
TextChunk (1) -> (1) EmbeddingVector
TextChunk (1) -> (1) ChunkMetadata
```

## Storage Schema (Qdrant)

### Collection: ai_rag_embedding
- **Vectors**: 1024-dimensional float vectors
- **Payload**:
  - source_url (keyword)
  - chapter_title (text)
  - chunk_index (integer)
  - content (text)
  - processed_at (datetime)

## State Transitions

### TextChunk Lifecycle
1. **Created**: When text is extracted from URL
2. **Processed**: When embedding is generated
3. **Stored**: When saved to Qdrant

## Constraints

- Each TextChunk must have exactly one associated EmbeddingVector
- Duplicate content should be detected and avoided
- Metadata must be preserved during storage and retrieval