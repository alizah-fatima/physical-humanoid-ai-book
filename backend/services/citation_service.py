"""
Citation service for RAG Agent
"""
from typing import List, Dict, Any
import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from models.response import RetrievedChunk
from validators.citation import validate_citation_list, validate_citation_quality


class CitationService:
    """Service for handling citation-related operations"""

    @staticmethod
    def calculate_confidence_score(chunk: Dict[str, Any], query_similarity: float = 1.0) -> float:
        """
        Calculate confidence score for a retrieved chunk based on various factors.

        Args:
            chunk: The retrieved chunk
            query_similarity: Similarity score from the query (0.0-1.0)

        Returns:
            Confidence score (0.0-1.0)
        """
        # Base confidence on the original score from the retrieval
        base_score = min(1.0, max(0.0, chunk.get('score', 0.0)))

        # Factor in content length (longer, more detailed content might be more reliable)
        content_length = len(chunk.get('text', ''))
        length_factor = min(1.0, content_length / 1000.0)  # Normalize against 1000 chars

        # Combine factors (this is a simple model - can be enhanced with ML)
        confidence = (base_score * 0.7) + (length_factor * 0.3)
        confidence = min(1.0, max(0.0, confidence * query_similarity))

        return confidence

    @staticmethod
    def calculate_confidence_scores(chunks: List[Dict[str, Any]], query_similarity: float = 1.0) -> List[RetrievedChunk]:
        """
        Calculate confidence scores for a list of retrieved chunks.

        Args:
            chunks: List of retrieved chunks
            query_similarity: Overall similarity of query (0.0-1.0)

        Returns:
            List of RetrievedChunk objects with confidence scores
        """
        result_chunks = []
        for chunk in chunks:
            confidence = CitationService.calculate_confidence_score(chunk, query_similarity)
            # Create a new chunk dict with confidence score
            chunk_with_confidence = {**chunk, 'confidence': confidence}
            result_chunks.append(RetrievedChunk(**chunk_with_confidence))

        return result_chunks

    @staticmethod
    def verify_citation_accuracy(original_chunk: Dict[str, Any], generated_response: str) -> Dict[str, Any]:
        """
        Verify that the generated response accurately cites the provided chunk.

        Args:
            original_chunk: The original retrieved chunk
            generated_response: The generated response to verify

        Returns:
            Verification results with accuracy score and details
        """
        chunk_text = original_chunk.get('text', '').lower()
        response_text = generated_response.lower()

        # Simple check: see if key terms from chunk appear in response
        chunk_words = set(chunk_text.split()[:20])  # First 20 words as representative sample
        response_words = set(response_text.split())

        overlap = len(chunk_words.intersection(response_words))
        total_chunk_words = len(chunk_words)

        accuracy_score = overlap / total_chunk_words if total_chunk_words > 0 else 0.0

        return {
            "accuracy_score": accuracy_score,
            "overlap_count": overlap,
            "chunk_word_count": total_chunk_words,
            "is_accurate": accuracy_score > 0.1  # Threshold for accuracy
        }

    @staticmethod
    def enhance_citation_format(chunks: List[RetrievedChunk]) -> List[Dict[str, Any]]:
        """
        Enhance citation formatting for better presentation.

        Args:
            chunks: List of RetrievedChunk objects

        Returns:
            List of enhanced citation dictionaries
        """
        enhanced_citations = []
        for chunk in chunks:
            enhanced = chunk.dict()
            # Add formatted citation information
            enhanced['formatted_citation'] = f"[{chunk.chapter_title} - {chunk.source_url}]"
            enhanced['relevance_label'] = CitationService._get_relevance_label(chunk.score)
            enhanced_citations.append(enhanced)

        return enhanced_citations

    @staticmethod
    def _get_relevance_label(score: float) -> str:
        """
        Get a human-readable relevance label based on score.

        Args:
            score: The relevance score

        Returns:
            Relevance label (High, Medium, Low)
        """
        if score >= 0.7:
            return "High"
        elif score >= 0.4:
            return "Medium"
        else:
            return "Low"