"""
Error handling utilities for RAG Agent
"""
from enum import Enum
from typing import Optional, Dict, Any
from datetime import datetime

class ErrorType(Enum):
    """Enumeration of error types for the RAG Agent"""
    VALIDATION_ERROR = "validation_error"
    RETRIEVAL_ERROR = "retrieval_error"
    LLM_ERROR = "llm_error"
    INTERNAL_ERROR = "internal_error"
    TIMEOUT_ERROR = "timeout_error"

class RAGAgentError(Exception):
    """Base exception class for RAG Agent errors"""

    def __init__(self,
                 message: str,
                 error_type: ErrorType,
                 details: Optional[Dict[str, Any]] = None,
                 status_code: int = 500):
        super().__init__(message)
        self.message = message
        self.error_type = error_type
        self.details = details or {}
        self.status_code = status_code
        self.timestamp = datetime.now().isoformat()

    def to_dict(self) -> Dict[str, Any]:
        """Convert error to dictionary format for API responses"""
        return {
            "error": {
                "type": self.error_type.value,
                "message": self.message,
                "details": self.details,
                "timestamp": self.timestamp
            }
        }

class ValidationError(RAGAgentError):
    """Exception raised for validation errors"""
    def __init__(self, message: str, details: Optional[Dict[str, Any]] = None):
        super().__init__(message, ErrorType.VALIDATION_ERROR, details, 400)

class RetrievalError(RAGAgentError):
    """Exception raised for retrieval errors"""
    def __init__(self, message: str, details: Optional[Dict[str, Any]] = None):
        super().__init__(message, ErrorType.RETRIEVAL_ERROR, details, 500)

class LLMError(RAGAgentError):
    """Exception raised for LLM (OpenAI) errors"""
    def __init__(self, message: str, details: Optional[Dict[str, Any]] = None):
        super().__init__(message, ErrorType.LLM_ERROR, details, 500)

class TimeoutError(RAGAgentError):
    """Exception raised for timeout errors"""
    def __init__(self, message: str, details: Optional[Dict[str, Any]] = None):
        super().__init__(message, ErrorType.TIMEOUT_ERROR, details, 408)

def handle_error(error: Exception, default_status_code: int = 500) -> RAGAgentError:
    """Handle an error and return an appropriate RAGAgentError"""
    if isinstance(error, RAGAgentError):
        return error
    else:
        return RAGAgentError(
            str(error),
            ErrorType.INTERNAL_ERROR,
            {"original_error_type": type(error).__name__},
            default_status_code
        )