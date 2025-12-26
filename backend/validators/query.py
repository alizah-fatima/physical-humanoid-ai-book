"""
Query validation utilities for RAG Agent
"""
import re
from typing import Dict, Any, Optional
from ..utils.errors import ValidationError


def validate_query_request(query: str, top_k: Optional[int] = None, temperature: Optional[float] = None) -> Dict[str, Any]:
    """
    Validate a query request according to the data model requirements.

    Args:
        query: The query text to validate
        top_k: Number of results to retrieve (optional)
        temperature: Temperature parameter (optional)

    Returns:
        Dict with validation results

    Raises:
        ValidationError: If validation fails
    """
    errors = []

    # Validate query text
    if not query:
        errors.append("Query text is required")
    elif len(query) < 1:
        errors.append("Query text must be at least 1 character")
    elif len(query) > 1000:
        errors.append("Query text must be no more than 1000 characters")

    # Validate top_k if provided
    if top_k is not None:
        if not isinstance(top_k, int):
            errors.append("top_k must be an integer")
        elif top_k < 1:
            errors.append("top_k must be at least 1")
        elif top_k > 20:
            errors.append("top_k must be no more than 20")

    # Validate temperature if provided
    if temperature is not None:
        if not isinstance(temperature, (int, float)):
            errors.append("temperature must be a number")
        elif temperature < 0.0:
            errors.append("temperature must be at least 0.0")
        elif temperature > 1.0:
            errors.append("temperature must be no more than 1.0")

    if errors:
        raise ValidationError("Query validation failed", {"errors": errors})

    return {
        "is_valid": True,
        "query_length": len(query),
        "top_k": top_k,
        "temperature": temperature
    }


def validate_retrieved_chunks(chunks: list) -> Dict[str, Any]:
    """
    Validate retrieved chunks to ensure they meet requirements.

    Args:
        chunks: List of retrieved chunks to validate

    Returns:
        Dict with validation results

    Raises:
        ValidationError: If validation fails
    """
    errors = []

    if not isinstance(chunks, list):
        errors.append("Retrieved chunks must be a list")

    for i, chunk in enumerate(chunks):
        if not isinstance(chunk, dict):
            errors.append(f"Chunk at index {i} must be a dictionary")
            continue

        required_fields = ['text', 'source_url', 'chapter_title', 'score']
        for field in required_fields:
            if field not in chunk:
                errors.append(f"Chunk at index {i} missing required field: {field}")

        if 'text' in chunk and not chunk['text'].strip():
            errors.append(f"Chunk at index {i} has empty text content")

        if 'score' in chunk and not isinstance(chunk['score'], (int, float)):
            errors.append(f"Chunk at index {i} has invalid score type")

    if errors:
        raise ValidationError("Retrieved chunks validation failed", {"errors": errors})

    return {
        "is_valid": True,
        "chunk_count": len(chunks)
    }