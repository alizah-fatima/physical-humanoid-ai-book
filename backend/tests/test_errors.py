"""
Test cases for error handling
"""
import pytest
from ..utils.errors import (
    RAGAgentError, ValidationError, RetrievalError,
    LLMError, TimeoutError as RAGTimeoutError, handle_error
)


def test_validation_error():
    """Test ValidationError creation and properties"""
    error = ValidationError("Invalid query format", {"field": "query", "value": ""})

    assert error.error_type.value == "validation_error"
    assert error.status_code == 400
    assert "Invalid query format" in str(error)

    error_dict = error.to_dict()
    assert error_dict["error"]["type"] == "validation_error"
    assert error_dict["error"]["message"] == "Invalid query format"
    assert error_dict["error"]["details"]["field"] == "query"


def test_retrieval_error():
    """Test RetrievalError creation and properties"""
    error = RetrievalError("Failed to retrieve from Qdrant", {"collection": "ai_rag_embedding"})

    assert error.error_type.value == "retrieval_error"
    assert error.status_code == 500

    error_dict = error.to_dict()
    assert error_dict["error"]["type"] == "retrieval_error"
    assert error_dict["error"]["details"]["collection"] == "ai_rag_embedding"


def test_llm_error():
    """Test LLMError creation and properties"""
    error = LLMError("OpenAI API request failed", {"model": "gpt-4-turbo", "status_code": 429})

    assert error.error_type.value == "llm_error"
    assert error.status_code == 500

    error_dict = error.to_dict()
    assert error_dict["error"]["type"] == "llm_error"


def test_timeout_error():
    """Test TimeoutError creation and properties"""
    error = RAGTimeoutError("Request timed out", {"timeout": 30})

    assert error.error_type.value == "timeout_error"
    assert error.status_code == 408

    error_dict = error.to_dict()
    assert error_dict["error"]["type"] == "timeout_error"


def test_error_handling_utility():
    """Test the handle_error utility function"""
    # Test with RAGAgentError (should return as-is)
    original_error = ValidationError("Test error")
    handled = handle_error(original_error)
    assert handled is original_error

    # Test with regular exception (should convert to RAGAgentError)
    regular_exception = ValueError("Regular error")
    handled = handle_error(regular_exception)
    assert isinstance(handled, RAGAgentError)
    assert str(handled) == "Regular error"
    assert handled.error_type.value == "internal_error"

    # Test with custom status code
    handled = handle_error(regular_exception, 422)
    assert handled.status_code == 422


def test_error_types_enum():
    """Test ErrorType enum values"""
    from ..utils.errors import ErrorType

    assert ErrorType.VALIDATION_ERROR.value == "validation_error"
    assert ErrorType.RETRIEVAL_ERROR.value == "retrieval_error"
    assert ErrorType.LLM_ERROR.value == "llm_error"
    assert ErrorType.INTERNAL_ERROR.value == "internal_error"
    assert ErrorType.TIMEOUT_ERROR.value == "timeout_error"


def test_error_timestamp():
    """Test that errors have proper timestamps"""
    error = ValidationError("Test error")

    # Timestamp should be set
    assert hasattr(error, 'timestamp')
    assert isinstance(error.timestamp, str)

    # Should be in ISO format
    assert len(error.timestamp) > 0


def test_error_to_dict_structure():
    """Test the structure of error.to_dict() output"""
    error = RetrievalError("Test retrieval error", {"details": "extra info"})

    error_dict = error.to_dict()

    # Should have proper structure
    assert "error" in error_dict
    assert "type" in error_dict["error"]
    assert "message" in error_dict["error"]
    assert "details" in error_dict["error"]
    assert "timestamp" in error_dict["error"]

    # Values should be correct
    assert error_dict["error"]["type"] == "retrieval_error"
    assert error_dict["error"]["message"] == "Test retrieval error"
    assert error_dict["error"]["details"]["details"] == "extra info"


def test_error_hierarchy():
    """Test that all specific errors inherit from RAGAgentError"""
    errors = [
        ValidationError("test"),
        RetrievalError("test"),
        LLMError("test"),
        RAGTimeoutError("test")
    ]

    for error in errors:
        assert isinstance(error, RAGAgentError)
        assert isinstance(error, Exception)


if __name__ == "__main__":
    pytest.main([__file__])