"""
Citation validation utilities for RAG Agent
"""
from typing import Dict, Any, List
import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from utils.errors import ValidationError


def validate_citation_format(citation: Dict[str, Any]) -> Dict[str, Any]:
    """
    Validate a single citation to ensure it has proper format.

    Args:
        citation: The citation to validate

    Returns:
        Dict with validation results

    Raises:
        ValidationError: If validation fails
    """
    errors = []

    required_fields = ['text', 'source_url', 'chapter_title', 'score']
    for field in required_fields:
        if field not in citation:
            errors.append(f"Missing required field: {field}")

    if 'text' in citation and not isinstance(citation['text'], str):
        errors.append("text field must be a string")

    if 'source_url' in citation and not isinstance(citation['source_url'], str):
        errors.append("source_url field must be a string")

    if 'chapter_title' in citation and not isinstance(citation['chapter_title'], str):
        errors.append("chapter_title field must be a string")

    if 'score' in citation and not isinstance(citation['score'], (int, float)):
        errors.append("score field must be a number")

    if 'text' in citation and not citation['text'].strip():
        errors.append("text field cannot be empty")

    if 'source_url' in citation and not citation['source_url'].strip():
        errors.append("source_url field cannot be empty")

    if 'chapter_title' in citation and not citation['chapter_title'].strip():
        errors.append("chapter_title field cannot be empty")

    if 'score' in citation and citation['score'] < 0:
        errors.append("score must be non-negative")

    if errors:
        raise ValidationError("Citation validation failed", {"errors": errors})

    return {
        "is_valid": True,
        "citation_id": citation.get('source_url', 'unknown')
    }


def validate_citation_list(citations: List[Dict[str, Any]]) -> Dict[str, Any]:
    """
    Validate a list of citations.

    Args:
        citations: List of citations to validate

    Returns:
        Dict with validation results

    Raises:
        ValidationError: If validation fails
    """
    errors = []

    if not isinstance(citations, list):
        errors.append("Citations must be provided as a list")

    for i, citation in enumerate(citations):
        try:
            validate_citation_format(citation)
        except ValidationError as e:
            errors.append(f"Citation at index {i} is invalid: {str(e)}")

    if errors:
        raise ValidationError("Citation list validation failed", {"errors": errors})

    return {
        "is_valid": True,
        "citation_count": len(citations)
    }


def validate_citation_quality(citations: List[Dict[str, Any]], min_score: float = 0.1) -> Dict[str, Any]:
    """
    Validate the quality of citations based on relevance scores.

    Args:
        citations: List of citations to validate
        min_score: Minimum acceptable relevance score (default: 0.1)

    Returns:
        Dict with validation results
    """
    low_quality_citations = []
    valid_citations = []

    for i, citation in enumerate(citations):
        if 'score' in citation and citation['score'] >= min_score:
            valid_citations.append(citation)
        else:
            low_quality_citations.append(i)

    return {
        "is_valid": True,
        "valid_count": len(valid_citations),
        "low_quality_count": len(low_quality_citations),
        "low_quality_indices": low_quality_citations,
        "quality_threshold": min_score
    }