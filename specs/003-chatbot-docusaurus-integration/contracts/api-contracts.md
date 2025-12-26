# API Contracts: Chatbot Docusaurus Integration

## Overview

This document defines the API contracts for the RAG chatbot integration with the Docusaurus textbook site. It specifies the endpoints, request/response formats, and data contracts used for communication between the frontend chat interface and backend services.

## Endpoints

### Query Processing Endpoint

**POST** `/api/agent/query`

Process a user query using the RAG agent with book content retrieval.

#### Request

```json
{
  "query": "Explain ROS 2 architecture",
  "selectedText": "Optional selected text that provides additional context",
  "currentPageUrl": "/module1/chapter1-introduction/",
  "chapterTitle": "Chapter 1: Introduction to ROS 2",
  "topK": 5
}
```

#### Request Schema

- `query` (string, required): The user's natural language query (1-1000 characters)
- `selectedText` (string, optional): Additional context from selected text (0-5000 characters)
- `currentPageUrl` (string, optional): URL of the current page for context
- `chapterTitle` (string, optional): Title of the current chapter for context
- `topK` (integer, optional): Number of results to retrieve (1-20, default: 5)

#### Response (Success)

```json
{
  "query": "Explain ROS 2 architecture",
  "response": "ROS 2 (Robot Operating System 2) is a flexible framework for writing robot software...",
  "contextUsed": [
    {
      "text": "ROS 2 is a flexible framework for writing robot software...",
      "sourceUrl": "/chapters/ros2-intro.md",
      "chapterTitle": "ROS 2 Introduction",
      "score": 0.85
    }
  ],
  "threadId": "thread_abc123",
  "executionTimeMs": 1250,
  "timestamp": "2025-12-26T10:30:00.123Z",
  "modelUsed": "gpt-4-turbo"
}
```

#### Response Schema

- `query` (string): Original query text
- `response` (string): Agent's response to the query
- `contextUsed` (array[object]): Retrieved chunks used to generate the response
  - `text` (string): Content text
  - `sourceUrl` (string): Source URL
  - `chapterTitle` (string): Chapter title
  - `score` (number): Relevance score
- `threadId` (string, optional): Thread identifier for conversation continuity
- `executionTimeMs` (number): Processing time in milliseconds
- `timestamp` (string): ISO 8601 timestamp
- `modelUsed` (string): LLM model used

#### Response (Error)

```json
{
  "query": "Explain ROS 2 architecture",
  "response": "Error processing query: Missing required environment variables",
  "contextUsed": [],
  "threadId": null,
  "executionTimeMs": 15,
  "timestamp": "2025-12-26T10:30:00.123Z",
  "modelUsed": null,
  "error": "Missing required environment variables"
}
```

#### Status Codes

- `200`: Query processed successfully
- `400`: Invalid request parameters
- `500`: Internal server error during processing

## Health Check Endpoint

**GET** `/api/agent/health`

Check the health and availability of the RAG agent.

### Response

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

### Response Schema

- `status` (string): Health status ("healthy", "degraded", "unhealthy")
- `timestamp` (string): ISO 8601 timestamp
- `services` (object): Status of dependent services

### Status Codes

- `200`: Agent is healthy and all services are connected
- `503`: Agent is unhealthy (one or more services unavailable)

## Configuration Endpoint

**GET** `/api/agent/config`

Get current agent configuration.

### Response

```json
{
  "model": "gpt-4-turbo",
  "topK": 5,
  "temperature": 0.3,
  "retrievalThreshold": 0.5,
  "timestamp": "2025-12-26T10:30:00.123Z"
}
```

### Response Schema

- `model` (string): OpenAI model being used
- `topK` (integer): Number of results to retrieve
- `temperature` (number): Response creativity control
- `retrievalThreshold` (number, optional): Minimum relevance score threshold
- `timestamp` (string): ISO 8601 timestamp

### Status Codes

- `200`: Configuration retrieved successfully

## Error Response Format

All error responses follow this structure:

```json
{
  "error": {
    "type": "validation_error" | "retrieval_error" | "llm_error" | "internal_error" | "timeout_error",
    "message": "Descriptive error message",
    "details": {
      "field": "field_name",
      "value": "problematic_value",
      "reason": "why_it_failed"
    },
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