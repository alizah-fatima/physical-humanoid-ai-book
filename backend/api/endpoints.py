"""
API endpoints for RAG Agent
"""
from fastapi import APIRouter, HTTPException, Depends
from typing import Optional
import time

from ..models.request import QueryRequest
from ..models.response import AgentResponse, HealthResponse, ConfigResponse
from ..models.error import ErrorResponse
from ..agent import RAGAgent
from ..config import AgentConfig
from ..validators.query import validate_query_request
from ..utils.errors import ValidationError, RetrievalError, LLMError, handle_error
from ..utils.logging import get_logger

router = APIRouter()
logger = get_logger()

# Initialize the RAG agent
rag_agent: Optional[RAGAgent] = None


def get_agent():
    """Get or create the RAG agent instance"""
    global rag_agent
    if rag_agent is None:
        try:
            rag_agent = RAGAgent()
        except Exception as e:
            logger.error(f"Failed to initialize RAG agent: {str(e)}")
            raise HTTPException(status_code=503, detail="Service unavailable: Failed to initialize RAG agent")
    return rag_agent


@router.post("/query", response_model=AgentResponse, responses={400: {"model": ErrorResponse}, 500: {"model": ErrorResponse}})
async def query_endpoint(request: QueryRequest):
    """
    Process a user query using the RAG agent with book content retrieval.
    """
    start_time = time.time()
    logger.info(f"Received query: '{request.query[:50]}...'")

    try:
        # Validate the request parameters
        validate_query_request(
            query=request.query,
            top_k=request.top_k,
            temperature=request.temperature
        )

        # Get the agent instance
        agent = get_agent()

        import subprocess
        import json
        import sys
        import os

        # Run the agent query as a separate process to avoid event loop conflicts
        cmd = [
            sys.executable,
            os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'run_agent_query.py'),
            '--query', request.query,
            '--selected_text', request.selected_text or "",
            '--top_k', str(request.top_k or 5),
            '--temperature', str(request.temperature or 0.3)
        ]

        process_result = subprocess.run(cmd, capture_output=True, text=True, timeout=60)

        if process_result.returncode != 0:
            raise Exception(f"Agent execution failed: {process_result.stderr}")

        # Extract JSON from output (logs may be mixed with JSON output)
        output = process_result.stdout.strip()

        # Find the first JSON object in the output
        start_idx = output.find('{')
        end_idx = output.rfind('}')

        if start_idx != -1 and end_idx != -1 and start_idx < end_idx:
            json_str = output[start_idx:end_idx+1]
            result = json.loads(json_str)
        else:
            raise Exception(f"Could not extract JSON from subprocess output: {output}")

        # Calculate response time
        response_time = (time.time() - start_time) * 1000

        # Log successful query
        logger.info(f"Query processed successfully in {response_time:.2f}ms")

        # Update execution time with actual API time
        result["execution_time_ms"] = response_time

        return AgentResponse(**result)

    except ValidationError as e:
        logger.warning(f"Validation error for query: {str(e)}")
        raise HTTPException(status_code=400, detail=e.to_dict())

    except RetrievalError as e:
        logger.error(f"Retrieval error for query: {str(e)}")
        raise HTTPException(status_code=500, detail=e.to_dict())

    except LLMError as e:
        logger.error(f"LLM error for query: {str(e)}")
        raise HTTPException(status_code=500, detail=e.to_dict())

    except Exception as e:
        logger.error(f"Unexpected error processing query: {str(e)}")
        handled_error = handle_error(e)
        raise HTTPException(status_code=handled_error.status_code, detail=handled_error.to_dict())


@router.get("/health", response_model=HealthResponse, responses={503: {"model": ErrorResponse}})
async def health_check():
    """
    Check the health and availability of the RAG agent.
    """
    try:
        # Check if the agent can be initialized
        agent = get_agent()

        # Perform a basic check to ensure services are available
        services_status = {
            "openai": "connected" if hasattr(agent, 'agent') else "disconnected",
            "qdrant": "checking",  # We could add actual Qdrant connectivity check
            "cohere": "checking"   # We could add actual Cohere connectivity check
        }

        return HealthResponse(
            status="healthy",
            timestamp=str(int(time.time())),
            services=services_status
        )

    except Exception as e:
        logger.error(f"Health check failed: {str(e)}")
        raise HTTPException(status_code=503, detail={"status": "unhealthy", "error": str(e)})


@router.get("/config", response_model=ConfigResponse, responses={200: {"model": ConfigResponse}})
async def get_config():
    """
    Get current agent configuration.
    """
    try:
        return ConfigResponse(
            model=AgentConfig.OPENAI_MODEL,
            top_k=AgentConfig.RETRIEVAL_TOP_K,
            temperature=AgentConfig.AGENT_TEMPERATURE,
            retrieval_threshold=0.5,  # Default threshold
            timestamp=str(int(time.time()))
        )
    except Exception as e:
        logger.error(f"Config retrieval failed: {str(e)}")
        raise HTTPException(status_code=500, detail=str(e))