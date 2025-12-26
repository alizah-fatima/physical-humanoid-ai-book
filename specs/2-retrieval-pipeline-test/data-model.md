# Data Model: Retrieval Pipeline

## Overview
This document defines the data structures used in the retrieval pipeline for the RAG chatbot system.

## Core Entities

### QueryInput
Represents the input to the retrieval system.

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| query_text | string | Yes | The natural language query text to search for |
| top_k | integer | No | Number of top results to retrieve (default: 5) |
| filters | object | No | Optional filters to apply to the search |

### QueryEmbedding
Represents the embedded form of the input query.

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| text | string | Yes | Original query text |
| embedding | array[float] | Yes | Vector representation of the query text |
| model | string | Yes | Name of the embedding model used |

### RetrievedChunk
Represents a single chunk of content retrieved from the vector database.

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| text | string | Yes | The content text of the retrieved chunk |
| source_url | string | Yes | URL or identifier of the source document |
| chapter_title | string | Yes | Title of the chapter containing this chunk |
| score | float | Yes | Similarity score between query and chunk (0.0-1.0) |
| chunk_id | string | No | Unique identifier for the chunk |

### RetrievalResult
Represents the complete result of a retrieval operation.

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| query | string | Yes | The original query text |
| results | array[RetrievedChunk] | Yes | List of retrieved chunks ranked by relevance |
| total_chunks_found | integer | No | Total number of chunks found (before top_k filtering) |
| execution_time_ms | number | No | Time taken to execute the retrieval in milliseconds |
| embedding_model | string | No | Model used for embedding generation |

## Relationships
- One QueryInput produces one QueryEmbedding
- One QueryEmbedding is used to retrieve multiple RetrievedChunk objects
- Multiple RetrievedChunk objects form one RetrievalResult

## Validation Rules
- query_text must be between 1 and 1000 characters
- top_k must be between 1 and 20
- score must be between 0.0 and 1.0
- text in RetrievedChunk must not be empty
- source_url must be a valid URL or identifier format

## State Transitions
1. QueryInput is received by the system
2. QueryInput is transformed into QueryEmbedding using Cohere API
3. QueryEmbedding is used to search Qdrant database
4. RetrievedChunk objects are collected and ranked
5. RetrievalResult is assembled with metadata and returned