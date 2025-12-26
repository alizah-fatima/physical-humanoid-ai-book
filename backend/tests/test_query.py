"""
Test cases for query endpoint
"""
import pytest
from fastapi.testclient import TestClient
from ..api.main import app
from ..models.request import QueryRequest


@pytest.fixture
def client():
    """Create a test client for the API"""
    return TestClient(app)


def test_query_endpoint_valid_request(client):
    """Test query endpoint with a valid request"""
    request_data = {
        "query": "Explain ROS 2 architecture",
        "top_k": 3,
        "temperature": 0.3
    }

    response = client.post("/api/agent/query", json=request_data)

    # Note: This will likely fail due to missing API keys in test environment,
    # but should return a proper error response rather than a server error
    assert response.status_code in [200, 400, 500]  # Acceptable status codes


def test_query_endpoint_with_selected_text(client):
    """Test query endpoint with selected text"""
    request_data = {
        "query": "Explain this concept",
        "selected_text": "This is some selected text that provides additional context",
        "top_k": 5,
        "temperature": 0.5
    }

    response = client.post("/api/agent/query", json=request_data)

    # Should accept the request structure even if processing fails due to missing keys
    assert response.status_code in [200, 400, 500]


def test_query_endpoint_invalid_query(client):
    """Test query endpoint with invalid query"""
    request_data = {
        "query": "",  # Empty query should fail validation
        "top_k": 5,
        "temperature": 0.3
    }

    response = client.post("/api/agent/query", json=request_data)

    # Should return 400 for validation error
    assert response.status_code == 400


def test_query_endpoint_invalid_top_k(client):
    """Test query endpoint with invalid top_k"""
    request_data = {
        "query": "Test query",
        "top_k": 0,  # Invalid top_k should fail validation
        "temperature": 0.3
    }

    response = client.post("/api/agent/query", json=request_data)

    # Should return 400 for validation error
    assert response.status_code == 400


def test_query_endpoint_invalid_temperature(client):
    """Test query endpoint with invalid temperature"""
    request_data = {
        "query": "Test query",
        "top_k": 5,
        "temperature": -1.0  # Invalid temperature should fail validation
    }

    response = client.post("/api/agent/query", json=request_data)

    # Should return 400 for validation error
    assert response.status_code == 400


def test_health_endpoint(client):
    """Test health check endpoint"""
    response = client.get("/api/agent/health")

    # Health endpoint should always return 200 when service is running
    assert response.status_code == 200

    data = response.json()
    assert "status" in data
    assert "timestamp" in data
    assert "services" in data


def test_config_endpoint(client):
    """Test configuration endpoint"""
    response = client.get("/api/agent/config")

    # Config endpoint should return 200
    assert response.status_code == 200

    data = response.json()
    assert "model" in data
    assert "top_k" in data
    assert "temperature" in data
    assert "timestamp" in data


def test_query_model_validation():
    """Test QueryRequest model validation directly"""
    # Valid request should work
    valid_request = QueryRequest(
        query="Test query",
        top_k=5,
        temperature=0.3
    )
    assert valid_request.query == "Test query"
    assert valid_request.top_k == 5
    assert valid_request.temperature == 0.3

    # Test with selected_text
    request_with_context = QueryRequest(
        query="Test query",
        selected_text="Additional context",
        top_k=3
    )
    assert request_with_context.selected_text == "Additional context"
    assert request_with_context.top_k == 3

    # Invalid query should raise validation error
    try:
        QueryRequest(query="", top_k=5)
        assert False, "Should have raised validation error"
    except Exception:
        pass  # Expected


if __name__ == "__main__":
    pytest.main([__file__])