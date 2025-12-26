# API Contracts: Retrieval Pipeline

## Overview
This document defines the API contracts for the retrieval pipeline that queries Qdrant using Cohere embeddings.

## Endpoints

### POST /api/retrieve
Retrieve relevant content chunks based on a natural language query.

#### Request
- **Method**: POST
- **Path**: /api/retrieve
- **Content-Type**: application/json

##### Request Body
```json
{
  "query_text": "string",
  "top_k": 5,
  "filters": {}
}
```

| Field | Type | Required | Default | Description |
|-------|-------|----------|---------|-------------|
| query_text | string | Yes | - | The natural language query text to search for |
| top_k | integer | No | 5 | Number of top results to retrieve |
| filters | object | No | {} | Optional filters to apply to the search |

##### Request Body Example
```json
{
  "query_text": "Explain ROS2 architecture components",
  "top_k": 5
}
```

#### Response
- **Content-Type**: application/json

##### Success Response (200 OK)
```json
{
  "query": "Explain ROS2 architecture components",
  "results": [
    {
      "text": "ROS 2 is designed with a layered architecture...",
      "source_url": "/chapters/ros2-architecture.md",
      "chapter_title": "ROS2 Architecture Overview",
      "score": 0.87,
      "chunk_id": "chunk-123"
    }
  ],
  "total_chunks_found": 1,
  "execution_time_ms": 125,
  "embedding_model": "embed-english-v3.0"
}
```

| Field | Type | Description |
|-------|------|-------------|
| query | string | The original query text |
| results | array | List of retrieved chunks ranked by relevance |
| total_chunks_found | integer | Total number of chunks found (before top_k filtering) |
| execution_time_ms | number | Time taken to execute the retrieval in milliseconds |
| embedding_model | string | Model used for embedding generation |

##### Error Responses
- **400 Bad Request**: Invalid request parameters
- **500 Internal Server Error**: Server error during retrieval

##### Error Response Example (400)
```json
{
  "error": "Invalid query parameters",
  "details": "query_text is required and must be between 1 and 1000 characters"
}
```

##### Error Response Example (500)
```json
{
  "error": "Retrieval failed",
  "details": "Failed to connect to Qdrant database"
}
```

### GET /api/health
Health check endpoint for the retrieval service.

#### Request
- **Method**: GET
- **Path**: /api/health

#### Response
- **Content-Type**: application/json

##### Success Response (200 OK)
```json
{
  "status": "healthy",
  "timestamp": "2025-12-24T10:30:00Z",
  "services": {
    "cohere": "connected",
    "qdrant": "connected"
  }
}
```

## Versioning Strategy
- API versioning through path: `/api/v1/retrieve`
- Backward compatibility maintained for minor version changes
- Breaking changes introduced in new major versions

## Idempotency, Timeouts, and Retries
- **Idempotency**: GET operations are idempotent; POST operations are not idempotent
- **Timeouts**:
  - Cohere API calls: 30 seconds
  - Qdrant database queries: 10 seconds
  - Overall request timeout: 45 seconds
- **Retries**:
  - Automatic retry for transient errors (connection timeouts, rate limits)
  - Maximum 3 retries with exponential backoff

## Error Taxonomy

| Error Code | HTTP Status | Description | Retry Strategy |
|------------|-------------|-------------|----------------|
| RETRIEVAL_001 | 400 | Invalid query parameters | No retry - fix parameters |
| RETRIEVAL_002 | 500 | Cohere API connection failed | Yes - exponential backoff |
| RETRIEVAL_003 | 500 | Qdrant database connection failed | Yes - exponential backoff |
| RETRIEVAL_004 | 500 | Internal processing error | Yes - immediate retry once |
| RETRIEVAL_005 | 200 | No relevant results found | No retry - valid response |