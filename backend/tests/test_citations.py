"""
Test cases for citation functionality
"""
import pytest
from ..validators.citation import validate_citation_format, validate_citation_list, validate_citation_quality
from ..services.citation_service import CitationService
from ..models.response import RetrievedChunk


def test_citation_format_validation():
    """Test citation format validation"""
    # Valid citation
    valid_citation = {
        "text": "This is valid text",
        "source_url": "https://example.com",
        "chapter_title": "Test Chapter",
        "score": 0.8
    }

    result = validate_citation_format(valid_citation)
    assert result["is_valid"] is True

    # Invalid citation (missing required field)
    invalid_citation = {
        "text": "This is valid text",
        "source_url": "https://example.com",
        # Missing chapter_title
        "score": 0.8
    }

    try:
        validate_citation_format(invalid_citation)
        assert False, "Should have raised ValidationError"
    except Exception:
        pass  # Expected


def test_citation_list_validation():
    """Test citation list validation"""
    valid_citations = [
        {
            "text": "First citation text",
            "source_url": "https://example.com/1",
            "chapter_title": "Chapter 1",
            "score": 0.8
        },
        {
            "text": "Second citation text",
            "source_url": "https://example.com/2",
            "chapter_title": "Chapter 2",
            "score": 0.6
        }
    ]

    result = validate_citation_list(valid_citations)
    assert result["is_valid"] is True
    assert result["citation_count"] == 2

    # Invalid citation list (one invalid citation)
    invalid_citations = [
        {
            "text": "Valid citation",
            "source_url": "https://example.com/1",
            "chapter_title": "Chapter 1",
            "score": 0.8
        },
        {
            "text": "Invalid citation",  # Missing required fields
            "source_url": "https://example.com/2"
            # Missing chapter_title and score
        }
    ]

    try:
        validate_citation_list(invalid_citations)
        assert False, "Should have raised ValidationError"
    except Exception:
        pass  # Expected


def test_citation_quality_validation():
    """Test citation quality validation"""
    citations = [
        {
            "text": "High quality citation",
            "source_url": "https://example.com/1",
            "chapter_title": "Chapter 1",
            "score": 0.8
        },
        {
            "text": "Low quality citation",
            "source_url": "https://example.com/2",
            "chapter_title": "Chapter 2",
            "score": 0.2
        }
    ]

    result = validate_citation_quality(citations, min_score=0.5)
    assert result["valid_count"] == 1
    assert result["low_quality_count"] == 1
    assert result["low_quality_indices"] == [1]
    assert result["quality_threshold"] == 0.5


def test_confidence_score_calculation():
    """Test confidence score calculation"""
    citation_service = CitationService()

    chunk = {
        "text": "This is a test chunk with some content to calculate confidence on",
        "source_url": "https://example.com/test",
        "chapter_title": "Test Chapter",
        "score": 0.85
    }

    confidence = citation_service.calculate_confidence_score(chunk, query_similarity=0.9)
    assert 0.0 <= confidence <= 1.0

    # Test with low similarity
    low_confidence = citation_service.calculate_confidence_score(chunk, query_similarity=0.1)
    assert low_confidence < confidence


def test_confidence_scores_for_list():
    """Test confidence scores calculation for a list of chunks"""
    citation_service = CitationService()

    chunks = [
        {
            "text": "First chunk with some content",
            "source_url": "https://example.com/1",
            "chapter_title": "Chapter 1",
            "score": 0.9
        },
        {
            "text": "Second chunk with different content",
            "source_url": "https://example.com/2",
            "chapter_title": "Chapter 2",
            "score": 0.6
        }
    ]

    enhanced_chunks = citation_service.calculate_confidence_scores(chunks, query_similarity=0.8)

    assert len(enhanced_chunks) == 2
    for chunk in enhanced_chunks:
        assert isinstance(chunk, RetrievedChunk)
        assert chunk.confidence is not None
        assert 0.0 <= chunk.confidence <= 1.0


def test_citation_verification():
    """Test citation verification functionality"""
    citation_service = CitationService()

    original_chunk = {
        "text": "Artificial intelligence is a wonderful field with many applications in robotics",
        "source_url": "https://example.com/ai",
        "chapter_title": "AI Basics",
        "score": 0.7
    }

    # Response that references the original content
    generated_response = "Artificial intelligence is indeed a wonderful field that has many applications in robotics"

    verification = citation_service.verify_citation_accuracy(original_chunk, generated_response)

    assert "accuracy_score" in verification
    assert "overlap_count" in verification
    assert "chunk_word_count" in verification
    assert "is_accurate" in verification


def test_citation_enhancement():
    """Test citation enhancement functionality"""
    citation_service = CitationService()

    chunks = [
        RetrievedChunk(
            text="This is a high relevance chunk",
            source_url="https://example.com/1",
            chapter_title="Chapter 1",
            score=0.85
        ),
        RetrievedChunk(
            text="This is a medium relevance chunk",
            source_url="https://example.com/2",
            chapter_title="Chapter 2",
            score=0.45
        )
    ]

    enhanced = citation_service.enhance_citation_format(chunks)

    assert len(enhanced) == 2
    for enhanced_chunk in enhanced:
        assert "formatted_citation" in enhanced_chunk
        assert "relevance_label" in enhanced_chunk
        assert enhanced_chunk["relevance_label"] in ["High", "Medium", "Low"]


if __name__ == "__main__":
    pytest.main([__file__])