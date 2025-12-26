"""
Request models for RAG Agent API
"""
from pydantic import BaseModel, Field
from typing import Optional


class QueryRequest(BaseModel):
    """
    Request model for query endpoint
    """
    query: str = Field(
        ...,
        description="The user's natural language query",
        min_length=1,
        max_length=1000
    )
    selected_text: Optional[str] = Field(
        None,
        description="Additional context from selected text",
        max_length=5000
    )
    top_k: Optional[int] = Field(
        5,
        description="Number of results to retrieve",
        ge=1,
        le=20
    )
    temperature: Optional[float] = Field(
        0.3,
        description="Response creativity control",
        ge=0.0,
        le=1.0
    )

    class Config:
        schema_extra = {
            "example": {
                "query": "Explain ROS 2 architecture",
                "selected_text": "This is additional context from selected text...",
                "top_k": 5,
                "temperature": 0.3
            }
        }