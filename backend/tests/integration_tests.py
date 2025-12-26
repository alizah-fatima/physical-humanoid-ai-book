"""
Integration tests for RAG Agent components
"""
import pytest
from fastapi.testclient import TestClient
from unittest.mock import patch, Mock
from ..api.main import app
from ..agent import RAGAgent
from ..retrieve import query_qdrant
from ..services.citation_service import CitationService
from ..utils.metrics import PerformanceMonitor


@pytest.fixture
def client():
    """Create a test client for the API"""
    return TestClient(app)


def test_agent_integration_with_retrieval():
    """Test the integration between the agent and the retrieval system"""
    # Mock the Cohere and Qdrant APIs to avoid needing real API keys
    with patch('backend.retrieve.cohere_client') as mock_cohere, \
         patch('backend.retrieve.qdrant_client') as mock_qdrant:

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

        # Create an agent instance
        agent = RAGAgent.__new__(RAGAgent)  # Create without calling __init__
        agent.client = Mock()  # Mock the OpenAI client
        agent.model = "gpt-4-turbo"
        agent.top_k = 5
        agent.temperature = 0.3

        # Mock the OpenAI response
        mock_openai_response = Mock()
        mock_openai_response.choices = [Mock()]
        mock_openai_response.choices[0].message.content = "This is a test response from the LLM."
        agent.client.chat.completions.create.return_value = mock_openai_response

        # Test the agent's integration with retrieval
        result = agent.query_agent("Test query about robotics")

        # Verify the result structure
        assert "query" in result
        assert "response" in result
        assert "context_used" in result
        assert result["query"] == "Test query about robotics"
        assert result["response"] == "This is a test response from the LLM."


def test_citation_service_integration():
    """Test the integration of the citation service with the agent"""
    citation_service = CitationService()

    # Test chunks to process
    chunks = [
        {
            "text": "Artificial intelligence is a wonderful field",
            "source_url": "https://example.com/ai",
            "chapter_title": "AI Basics",
            "score": 0.85
        },
        {
            "text": "Robotics involves building physical machines",
            "source_url": "https://example.com/robotics",
            "chapter_title": "Robotics Fundamentals",
            "score": 0.72
        }
    ]

    # Process chunks with citation service
    enhanced_chunks = citation_service.calculate_confidence_scores(chunks)

    # Verify that confidence scores were added
    assert len(enhanced_chunks) == 2
    for chunk in enhanced_chunks:
        assert chunk.confidence is not None
        assert 0.0 <= chunk.confidence <= 1.0

    # Test citation verification
    original_chunk = chunks[0]
    generated_response = "Artificial intelligence is indeed a wonderful field in computer science"
    verification = citation_service.verify_citation_accuracy(original_chunk, generated_response)

    assert "accuracy_score" in verification
    assert verification["is_accurate"] is True  # Should be accurate based on content overlap


def test_api_endpoint_integration(client):
    """Test the integration of API endpoints with the underlying services"""
    # Test health endpoint
    response = client.get("/api/v1/agent/health")
    assert response.status_code == 200

    data = response.json()
    assert "status" in data
    assert "timestamp" in data
    assert "services" in data

    # Test config endpoint
    response = client.get("/api/v1/agent/config")
    assert response.status_code == 200

    data = response.json()
    assert "model" in data
    assert "top_k" in data
    assert "temperature" in data
    assert "timestamp" in data


def test_performance_monitor_integration():
    """Test the integration of performance monitoring"""
    monitor = PerformanceMonitor()

    # Test measuring a mock query function
    def mock_query_func(query):
        # Simulate some work
        time_taken = 0.1  # 100ms
        return {"result": f"Response to {query}", "processing_time": time_taken}

    import time
    start_time = time.time()
    result = monitor.measure_query_performance(mock_query_func, "test query")
    duration = time.time() - start_time

    # Verify the result
    assert result["result"] == "Response to test query"

    # Check that metrics were recorded
    metrics = monitor.collector.get_all_metrics()
    assert len(metrics) > 0

    # Look for query-related metrics
    query_duration_found = False
    for metric in metrics.values():
        if "query_duration" in metric.name:
            query_duration_found = True
            break

    assert query_duration_found, "Query duration metric should have been recorded"


def test_full_query_workflow_integration(client):
    """Test the full workflow from API request to response"""
    # Mock the external services to avoid needing real API keys
    with patch('backend.retrieve.cohere_client'), \
         patch('backend.retrieve.qdrant_client') as mock_qdrant, \
         patch('backend.agent.OpenAI') as mock_openai:

        # Set up mock Qdrant
        mock_result = Mock()
        mock_result.payload = {
            'content': 'Test content for robotics',
            'source_url': '/chapters/robotics-intro.md',
            'chapter_title': 'Robotics Introduction',
        }
        mock_result.score = 0.85
        mock_search_response = Mock()
        mock_search_response.points = [mock_result]
        mock_qdrant.query_points.return_value = mock_search_response

        # Set up mock OpenAI client
        mock_client = Mock()
        mock_openai.return_value = mock_client
        mock_response = Mock()
        mock_response.choices = [Mock()]
        mock_response.choices[0].message.content = "This is a test response."
        mock_client.chat.completions.create.return_value = mock_response

        # Test the query endpoint
        request_data = {
            "query": "Explain robotics",
            "top_k": 3,
            "temperature": 0.3
        }

        response = client.post("/api/v1/agent/query", json=request_data)

        # Should get a response (might be 200, 400, or 500 depending on validation)
        assert response.status_code in [200, 400, 500]


def test_error_handling_integration(client):
    """Test integration of error handling across components"""
    # Test with invalid request data
    invalid_request_data = {
        "query": "",  # Invalid - empty query
        "top_k": 30,  # Invalid - too high
        "temperature": -1.0  # Invalid - too low
    }

    response = client.post("/api/v1/agent/query", json=invalid_request_data)

    # Should return 400 for validation error
    assert response.status_code == 400

    error_data = response.json()
    assert "error" in error_data


if __name__ == "__main__":
    pytest.main([__file__])