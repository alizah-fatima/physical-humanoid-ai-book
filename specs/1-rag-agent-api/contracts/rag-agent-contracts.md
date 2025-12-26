# API Contracts: RAG Agent Endpoints

## Endpoint: Query Processing

### POST /api/agent/query
**Description**: Process a user query using the RAG agent with book content retrieval

**Request**:
```json
{
  "query": "Explain ROS 2 architecture",
  "top_k": 5,
  "temperature": 0.3
}
```

**Request Schema**:
- `query` (string, required): The user's natural language query (1-1000 characters)
- `top_k` (integer, optional): Number of results to retrieve (1-20, default: 5)
- `temperature` (number, optional): Response creativity control (0.0-1.0, default: 0.3)

**Response (Success)**:
```json
{
  "query": "Explain ROS 2 architecture",
  "response": "ROS 2 (Robot Operating System 2) is a flexible framework for writing robot software. It provides a collection of libraries and tools that help developers create robot applications. Key features include improved security, better real-time support, and a more robust communication system based on DDS (Data Distribution Service)...",
  "context_used": [
    {
      "text": "ROS 2 is a flexible framework for writing robot software...",
      "source_url": "/chapters/ros2-intro.md",
      "chapter_title": "ROS 2 Introduction",
      "score": 0.85
    }
  ],
  "thread_id": "thread_abc123",
  "execution_time_ms": 1250,
  "timestamp": "2025-12-26T10:30:00.123Z",
  "model_used": "gpt-4-turbo"
}
```

**Response Schema**:
- `query` (string): Original query text
- `response` (string): Agent's response to the query
- `context_used` (array[object]): Retrieved chunks used to generate the response
  - `text` (string): Content text
  - `source_url` (string): Source URL
  - `chapter_title` (string): Chapter title
  - `score` (number): Relevance score
- `thread_id` (string): Thread identifier for conversation continuity
- `execution_time_ms` (number): Processing time in milliseconds
- `timestamp` (string): ISO 8601 timestamp
- `model_used` (string): LLM model used

**Response (Error)**:
```json
{
  "query": "Explain ROS 2 architecture",
  "response": "Error processing query: Missing required environment variables",
  "context_used": [],
  "thread_id": null,
  "execution_time_ms": 15,
  "timestamp": "2025-12-26T10:30:00.123Z",
  "model_used": null,
  "error": "Missing required environment variables"
}
```

**Status Codes**:
- `200`: Query processed successfully
- `400`: Invalid request parameters
- `500`: Internal server error during processing

## Endpoint: Agent Health Check

### GET /api/agent/health
**Description**: Check the health and availability of the RAG agent

**Request**: No request body required

**Response**:
```json
{
  "status": "healthy",
  "timestamp": "2025-12-26T10:30:00.123Z",
  "services": {
    "openai": "connected",
    "qdrant": "connected",
    "cohere": "connected"
  }
}
```

**Status Codes**:
- `200`: Agent is healthy and all services are connected
- `503`: Agent is unhealthy (one or more services unavailable)

## Endpoint: Agent Configuration

### GET /api/agent/config
**Description**: Get current agent configuration

**Request**: No request body required

**Response**:
```json
{
  "model": "gpt-4-turbo",
  "top_k": 5,
  "temperature": 0.3,
  "retrieval_threshold": 0.5,
  "timestamp": "2025-12-26T10:30:00.123Z"
}
```

**Status Codes**:
- `200`: Configuration retrieved successfully

## Error Response Format

All error responses follow this structure:
```json
{
  "error": {
    "type": "validation_error" | "retrieval_error" | "llm_error" | "internal_error",
    "message": "Descriptive error message",
    "details": { /* optional detailed error information */ },
    "timestamp": "2025-12-26T10:30:00.123Z"
  }
}
```

## Versioning Strategy

- API version is specified in the URL path: `/api/v1/agent/...`
- Backward-compatible changes are made within the same version
- Breaking changes result in a new version: `/api/v2/agent/...`

## Idempotency

- Query requests are not idempotent as they may produce different responses based on context
- Health and config endpoints are idempotent

## Timeouts and Retries

- Default timeout: 30 seconds for query processing
- Client should implement exponential backoff for retry logic
- Maximum retry attempts: 3

## Error Taxonomy

- `validation_error`: Invalid request parameters (400)
- `retrieval_error`: Issue with retrieving context from Qdrant (500)
- `llm_error`: Issue with OpenAI API (500)
- `internal_error`: General server error (500)
- `timeout_error`: Request exceeded timeout (408)