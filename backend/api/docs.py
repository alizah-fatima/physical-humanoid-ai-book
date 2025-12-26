"""
Documentation for RAG Agent API endpoints
"""
from fastapi import FastAPI
from fastapi.openapi.utils import get_openapi


def customize_openapi(app: FastAPI):
    """
    Customize the OpenAPI schema for the RAG Agent API
    """
    if app.openapi_schema:
        return app.openapi_schema

    openapi_schema = get_openapi(
        title="RAG Agent API",
        version="1.0.0",
        description="API for RAG (Retrieval-Augmented Generation) agent that integrates with Qdrant retrieval system to provide responses based solely on book content.",
        routes=app.routes,
    )

    # Add custom information to the schema
    openapi_schema["info"] = {
        "title": "RAG Agent API",
        "version": "1.0.0",
        "description": "API for RAG (Retrieval-Augmented Generation) agent that integrates with Qdrant retrieval system to provide responses based solely on book content.",
        "contact": {
            "name": "API Support",
            "email": "support@example.com",
        },
        "license": {
            "name": "MIT License",
            "url": "https://opensource.org/licenses/MIT",
        },
    }

    # Add custom tags for better organization
    openapi_schema["tags"] = [
        {
            "name": "agent",
            "description": "RAG Agent operations - query processing, health checks, and configuration"
        }
    ]

    # Update paths to include more detailed information
    for path, methods in openapi_schema["paths"].items():
        for method, details in methods.items():
            # Add examples and more detailed descriptions
            if path == "/api/agent/query" and method == "post":
                details["description"] = "Process a user query using the RAG agent with book content retrieval"
                details["summary"] = "Query Processing"
            elif path == "/api/agent/health" and method == "get":
                details["description"] = "Check the health and availability of the RAG agent"
                details["summary"] = "Health Check"
            elif path == "/api/agent/config" and method == "get":
                details["description"] = "Get current agent configuration"
                details["summary"] = "Get Configuration"

    app.openapi_schema = openapi_schema
    return app.openapi_schema


# Example usage of request/response examples
query_request_example = {
    "query": "Explain ROS 2 architecture",
    "selected_text": "Additional context from selected text...",
    "top_k": 5,
    "temperature": 0.3
}

query_response_example = {
    "query": "Explain ROS 2 architecture",
    "response": "ROS 2 (Robot Operating System 2) is a flexible framework for writing robot software...",
    "context_used": [
        {
            "text": "ROS 2 is a flexible framework for writing robot software...",
            "source_url": "/chapters/ros2-intro.md",
            "chapter_title": "ROS 2 Introduction",
            "score": 0.85,
            "confidence": 0.82
        }
    ],
    "thread_id": None,
    "execution_time_ms": 1250.5,
    "timestamp": "2025-12-26T10:30:00.123456",
    "model_used": "gpt-4-turbo"
}

health_response_example = {
    "status": "healthy",
    "timestamp": "2025-12-26T10:30:00.123456",
    "services": {
        "openai": "connected",
        "qdrant": "connected",
        "cohere": "connected"
    }
}

config_response_example = {
    "model": "gpt-4-turbo",
    "top_k": 5,
    "temperature": 0.3,
    "retrieval_threshold": 0.5,
    "timestamp": "2025-12-26T10:30:00.123456"
}