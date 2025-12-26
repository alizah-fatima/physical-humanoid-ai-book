"""
Response models for RAG Agent API
"""
from pydantic import BaseModel, Field
from typing import List, Optional, Dict, Any
from datetime import datetime
from .request import QueryRequest


class RetrievedChunk(BaseModel):
    """
    Model for a single retrieved chunk
    """
    text: str = Field(..., description="The actual content text")
    source_url: str = Field(..., description="URL where the content was sourced from")
    chapter_title: str = Field(..., description="Title of the chapter this content belongs to")
    score: float = Field(..., description="Similarity score from the vector search", ge=0.0)
    confidence: Optional[float] = Field(
        None,
        description="Confidence score for this chunk's relevance to the query",
        ge=0.0,
        le=1.0
    )


class AgentResponse(BaseModel):
    """
    Response model for query endpoint
    """
    query: str = Field(..., description="The original user query")
    response: str = Field(..., description="The agent's response to the query")
    context_used: List[RetrievedChunk] = Field(
        ...,
        description="List of chunks used to generate the response"
    )
    thread_id: Optional[str] = Field(
        None,
        description="Thread ID for conversation continuity"
    )
    execution_time_ms: float = Field(
        ...,
        description="Time taken to process the query in milliseconds",
        ge=0.0
    )
    timestamp: str = Field(
        ...,
        description="ISO 8601 timestamp of the response"
    )
    model_used: Optional[str] = Field(
        None,
        description="The LLM model used for generation"
    )
    error: Optional[str] = Field(
        None,
        description="Error message if the query failed"
    )

    class Config:
        schema_extra = {
            "example": {
                "query": "Explain ROS 2 architecture",
                "response": "ROS 2 (Robot Operating System 2) is a flexible framework...",
                "context_used": [
                    {
                        "text": "ROS 2 is a flexible framework for writing robot software...",
                        "source_url": "/chapters/ros2-intro.md",
                        "chapter_title": "ROS 2 Introduction",
                        "score": 0.85
                    }
                ],
                "thread_id": "thread_abc123",
                "execution_time_ms": 1250.5,
                "timestamp": "2025-12-26T10:30:00.123Z",
                "model_used": "gpt-4-turbo"
            }
        }


class HealthResponse(BaseModel):
    """
    Response model for health check endpoint
    """
    status: str = Field(..., description="Health status of the service")
    timestamp: str = Field(..., description="ISO 8601 timestamp")
    services: Dict[str, str] = Field(
        ...,
        description="Status of dependent services (openai, qdrant, cohere)"
    )


class ConfigResponse(BaseModel):
    """
    Response model for configuration endpoint
    """
    model: str = Field(..., description="OpenAI model being used")
    top_k: int = Field(..., description="Number of results to retrieve", ge=1, le=20)
    temperature: float = Field(
        ...,
        description="Response creativity control",
        ge=0.0,
        le=1.0
    )
    retrieval_threshold: Optional[float] = Field(
        None,
        description="Minimum relevance score threshold",
        ge=0.0,
        le=1.0
    )
    timestamp: str = Field(..., description="ISO 8601 timestamp")