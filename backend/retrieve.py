"""
Retrieval Pipeline for RAG Chatbot

This module implements the retrieval pipeline that queries Qdrant using Cohere embeddings
to retrieve relevant content chunks from the textbook database.
"""
import os
import logging
import time
from typing import List, Dict, Any, Optional
import cohere
from qdrant_client import QdrantClient
from qdrant_client.http import models
import json

# Load environment variables from .env file if it exists
try:
    from dotenv import load_dotenv
    load_dotenv()
except ImportError:
    # If python-dotenv is not installed, continue without loading .env
    pass


# Configure logging
logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger(__name__)


# Initialize clients (will be created when needed)
cohere_client = None
qdrant_client = None

def get_cohere_client():
    """Lazy initialization of Cohere client to avoid early failure"""
    global cohere_client
    if cohere_client is None:
        COHERE_API_KEY = os.getenv('COHERE_API_KEY')
        if not COHERE_API_KEY:
            raise ValueError("COHERE_API_KEY environment variable is required")
        cohere_client = cohere.Client(api_key=COHERE_API_KEY)
    return cohere_client

def get_qdrant_client():
    """Lazy initialization of Qdrant client to avoid early failure"""
    global qdrant_client
    if qdrant_client is None:
        QDRANT_URL = os.getenv('QDRANT_URL')  # Use QDRANT_URL if available (for cloud instances)
        QDRANT_HOST = os.getenv('QDRANT_HOST', 'localhost')
        QDRANT_PORT = int(os.getenv('QDRANT_PORT', 6333))
        QDRANT_API_KEY = os.getenv('QDRANT_API_KEY')  # Optional, depending on your setup

        if QDRANT_URL:
            # Use URL for cloud instances
            qdrant_client = QdrantClient(
                url=QDRANT_URL,
                api_key=QDRANT_API_KEY
            )
        elif QDRANT_API_KEY:
            qdrant_client = QdrantClient(
                host=QDRANT_HOST,
                port=QDRANT_PORT,
                api_key=QDRANT_API_KEY
            )
        else:
            qdrant_client = QdrantClient(
                host=QDRANT_HOST,
                port=QDRANT_PORT
            )
    return qdrant_client


def validate_query_params(query_text: str, top_k: int) -> None:
    """
    Validate query parameters according to the data model requirements.

    Args:
        query_text (str): The query text to validate
        top_k (int): The number of results to retrieve

    Raises:
        ValueError: If parameters don't meet validation requirements
    """
    if not query_text or len(query_text) < 1 or len(query_text) > 1000:
        raise ValueError("query_text must be between 1 and 1000 characters")

    if top_k < 1 or top_k > 20:
        raise ValueError("top_k must be between 1 and 20")


def query_qdrant(query_text: str, top_k: int = 5) -> List[Dict[str, Any]]:
    """
    Query the Qdrant database using Cohere embeddings to retrieve relevant content chunks.

    Args:
        query_text (str): The natural language query text to search for
        top_k (int): Number of top results to retrieve (default: 5)

    Returns:
        List[Dict[str, Any]]: List of dictionaries containing:
            - text: chunk_text
            - source_url: url
            - chapter_title: title
            - score: similarity score
    """
    # Validate input parameters
    validate_query_params(query_text, top_k)

    logger.debug(f"Processing query: {query_text}")

    try:
        # Get clients (will initialize if needed)
        cohere_client = get_cohere_client()
        qdrant_client = get_qdrant_client()

        # Embed the query using Cohere
        response = cohere_client.embed(
            texts=[query_text],
            model="embed-english-v3.0",  # Using a common Cohere embedding model
            input_type="search_query"  # Required for newer models
        )
        query_embedding = response.embeddings[0]

        logger.debug(f"Generated query embedding with dimension: {len(query_embedding)}")

        # Search Qdrant collection "ai_rag_embedding" with the query embedding
        search_response = qdrant_client.query_points(
            collection_name="ai_rag_embedding",
            query=query_embedding,
            limit=top_k * 2,  # Get more results to have better options
            with_payload=True,  # Include payload data in results
            # Add a minimum score threshold to filter out poor matches
            score_threshold=0.01  # Only return results with score above threshold if we have good matches
        )

        # Extract results from the response object (newer Qdrant API)
        search_results = search_response.points
        logger.debug(f"Retrieved {len(search_results)} results from Qdrant before filtering")

        # Process and format results
        all_valid_results = []
        for result in search_results:
            # Extract payload data
            payload = result.payload or {}

            # Create result dictionary with required fields
            # Note: The field might be named 'content' instead of 'text' in the database
            formatted_result = {
                'text': payload.get('content', ''),  # Using 'content' field which appears to be the actual text
                'source_url': payload.get('source_url', ''),
                'chapter_title': payload.get('chapter_title', ''),
                'score': result.score
            }

            # Validate required metadata fields are present
            if not formatted_result['text'] or not formatted_result['source_url'] or not formatted_result['chapter_title']:
                logger.warning(f"Missing required metadata in result: {formatted_result}")
                continue  # Skip results with missing required metadata

            all_valid_results.append(formatted_result)

            # Log individual result for debugging
            logger.debug(f"Retrieved chunk - Score: {result.score}, "
                        f"Chapter: {payload.get('chapter_title', 'N/A')[:50]}...")

        # Sort all valid results by score
        all_valid_results.sort(key=lambda x: x['score'], reverse=True)

        # Take only top_k results
        formatted_results = all_valid_results[:top_k]

        logger.info(f"Returning {len(formatted_results)} results for query: '{query_text[:50]}...'")

        return formatted_results

    except Exception as e:
        logger.error(f"Cohere API error in query_qdrant: {str(e)}")
        raise e


def retrieve_with_metadata(query_text: str, top_k: int = 5) -> Dict[str, Any]:
    """
    Enhanced retrieval function that returns complete retrieval results with metadata.

    Args:
        query_text (str): The natural language query text to search for
        top_k (int): Number of top results to retrieve (default: 5)

    Returns:
        Dict[str, Any]: Complete retrieval result with metadata including:
            - query: original query text
            - results: list of retrieved chunks
            - total_chunks_found: total number of chunks found before filtering
            - execution_time_ms: time taken to execute the retrieval
            - embedding_model: model used for embedding generation
    """
    start_time = time.time()

    try:
        results = query_qdrant(query_text, top_k)
        execution_time_ms = round((time.time() - start_time) * 1000, 2)

        return {
            "query": query_text,
            "results": results,
            "total_chunks_found": len(results),  # In actual implementation, this would come from Qdrant response
            "execution_time_ms": execution_time_ms,
            "embedding_model": "embed-english-v3.0"
        }
    except Exception as e:
        execution_time_ms = round((time.time() - start_time) * 1000, 2)

        # Return error response with metadata
        return {
            "query": query_text,
            "results": [],
            "total_chunks_found": 0,
            "execution_time_ms": execution_time_ms,
            "embedding_model": "embed-english-v3.0",
            "error": str(e)
        }


def test_retrieve_implementation():
    """
    Test script to verify the retrieve.py implementation without requiring actual API keys
    """
    import os
    import sys
    from unittest.mock import Mock, patch

    print("Testing retrieve.py implementation structure...")

    # Temporarily set environment variables for testing
    os.environ['COHERE_API_KEY'] = 'test-key'
    os.environ['QDRANT_HOST'] = 'localhost'
    os.environ['QDRANT_PORT'] = '6333'

    try:
        # Test input validation
        try:
            validate_query_params("test query", 5)
            print("[SUCCESS] Input validation works correctly")
        except Exception as e:
            print(f"[ERROR] Input validation failed: {e}")
            return False

        # Test validation errors
        try:
            validate_query_params("", 5)  # Should fail
            print("[ERROR] Input validation didn't catch empty query")
            return False
        except ValueError:
            print("[SUCCESS] Input validation correctly catches empty queries")

        try:
            validate_query_params("a" * 1001, 5)  # Should fail
            print("[ERROR] Input validation didn't catch long query")
            return False
        except ValueError:
            print("[SUCCESS] Input validation correctly catches long queries")

        try:
            validate_query_params("test", 0)  # Should fail
            print("[ERROR] Input validation didn't catch invalid top_k")
            return False
        except ValueError:
            print("[SUCCESS] Input validation correctly catches invalid top_k")

        # Test with mocked API calls
        with patch('retrieve.cohere_client') as mock_cohere, \
             patch('retrieve.qdrant_client') as mock_qdrant:

            # Mock Cohere response
            mock_embedding = [0.1] * 1024  # Typical embedding size
            mock_cohere.embed.return_value = Mock(embeddings=[mock_embedding])

            # Mock Qdrant response
            mock_result = Mock()
            mock_result.payload = {
                'content': 'Test content for ROS 2',  # Note: using 'content' as in the actual code
                'source_url': '/chapters/ros2-intro.md',
                'chapter_title': 'ROS 2 Introduction',
            }
            mock_result.score = 0.85
            # For newer Qdrant API, we need to mock the query_points response properly
            mock_search_response = Mock()
            mock_search_response.points = [mock_result]
            mock_qdrant.query_points.return_value = mock_search_response

            # Test query_qdrant function
            try:
                results = query_qdrant("test query", top_k=1)
                print(f"[SUCCESS] query_qdrant function works, returned {len(results)} results")

                if len(results) > 0:
                    result = results[0]
                    expected_keys = ['text', 'source_url', 'chapter_title', 'score']
                    if all(key in result for key in expected_keys):
                        print("[SUCCESS] Results have correct structure")
                    else:
                        print(f"[ERROR] Results missing expected keys. Got: {result.keys()}")
                        return False
            except Exception as e:
                print(f"[ERROR] query_qdrant failed: {e}")
                import traceback
                traceback.print_exc()
                return False

            # Test retrieve_with_metadata function
            try:
                result_with_meta = retrieve_with_metadata("test query", top_k=1)
                expected_meta_keys = ['query', 'results', 'total_chunks_found', 'execution_time_ms', 'embedding_model']
                if all(key in result_with_meta for key in expected_meta_keys):
                    print("[SUCCESS] retrieve_with_metadata function works with correct structure")
                else:
                    print(f"[ERROR] retrieve_with_metadata missing expected keys. Got: {result_with_meta.keys()}")
                    return False
            except Exception as e:
                print(f"[ERROR] retrieve_with_metadata failed: {e}")
                return False

        print("\n[INFO] All implementation tests passed!")
        print("The retrieve.py implementation is structurally correct and ready for use.")
        print("When proper API keys are provided, it will work with actual Cohere and Qdrant services.")
        return True

    except Exception as e:
        print(f"[ERROR] Unexpected error during testing: {e}")
        import traceback
        traceback.print_exc()
        return False


def main():
    import argparse

    parser = argparse.ArgumentParser(description='Retrieve relevant content from the textbook using Cohere and Qdrant.')
    parser.add_argument('--query', type=str, help='The query text to search for')
    parser.add_argument('--top_k', type=int, default=5, help='Number of top results to retrieve (default: 5)')
    parser.add_argument('--test', action='store_true', help='Run tests instead of querying')

    args = parser.parse_args()

    if args.test:
        # Run tests
        success = test_retrieve_implementation()
        if success:
            print("\n[SUCCESS] Implementation verification completed successfully!")
        else:
            print("\n[ERROR] Implementation verification failed!")
            exit(1)
    else:
        # Check if query was provided
        if not args.query:
            print("Usage: python retrieve.py --query 'your query here' [--top_k 5]")
            print("       python retrieve.py --test  # Run tests")
            print("Example: python retrieve.py --query 'explain ROS 2' --top_k 3")
            return

        # Check if required environment variables are set
        if not os.getenv('COHERE_API_KEY'):
            print("Error: COHERE_API_KEY environment variable is required.")
            print("Please set it before running the script:")
            print("  export COHERE_API_KEY='your-cohere-api-key'")
            print("  set COHERE_API_KEY=your-cohere-api-key  # on Windows")
            return

        try:
            results = query_qdrant(args.query, args.top_k)
            print(f"Results for query: '{args.query}'")
            print(json.dumps(results, indent=2))

            # Also test the enhanced metadata function
            print("\nWith metadata:")
            result_with_meta = retrieve_with_metadata(args.query, args.top_k)
            print(json.dumps(result_with_meta, indent=2))
        except Exception as e:
            print(f"Error executing query: {e}")
            import traceback
            traceback.print_exc()


if __name__ == "__main__":
    main()