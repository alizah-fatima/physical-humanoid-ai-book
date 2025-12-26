"""
Error response models for RAG Agent API
"""
from pydantic import BaseModel, Field
from typing import Optional, Dict, Any
from enum import Enum


class ErrorType(str, Enum):
    """Enumeration of error types"""
    VALIDATION_ERROR = "validation_error"
    RETRIEVAL_ERROR = "retrieval_error"
    LLM_ERROR = "llm_error"
    INTERNAL_ERROR = "internal_error"
    TIMEOUT_ERROR = "timeout_error"


class ErrorResponse(BaseModel):
    """
    Standard error response model for the API
    """
    error: Dict[str, Any] = Field(
        ...,
        description="Error details including type, message, and optional details"
    )

    class Config:
        schema_extra = {
            "example": {
                "error": {
                    "type": "validation_error",
                    "message": "query_text must be between 1 and 1000 characters",
                    "details": {
                        "field": "query",
                        "value": ""
                    },
                    "timestamp": "2025-12-26T10:30:00.123Z"
                }
            }
        }