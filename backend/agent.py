"""
RAG Agent Implementation using OpenAI Agents SDK

This module implements an AI agent that uses the OpenAI Agents SDK to process user queries
and integrates with the existing Qdrant retrieval system to provide responses based solely on book content.
"""
import os
import logging
import json
from typing import Dict, Any, List, Optional
from datetime import datetime
from agents import Agent, Runner, function_tool
from pydantic import BaseModel
from agents import OpenAIChatCompletionsModel
from openai import AsyncOpenAI
import sys
import os
# Add the backend directory to the path to resolve imports
current_dir = os.path.dirname(os.path.abspath(__file__))
backend_dir = os.path.dirname(current_dir)
if backend_dir not in sys.path:
    sys.path.insert(0, backend_dir)

from retrieve import query_qdrant, retrieve_with_metadata
from config import AgentConfig
from utils.logging import get_logger
from services.citation_service import CitationService

logger = get_logger()

ROUTER_API_KEY = "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"

client = AsyncOpenAI(
    api_key=ROUTER_API_KEY,
    base_url= "https://openrouter.ai/api/v1" 
)

router_model = OpenAIChatCompletionsModel(
    openai_client=client,
    model="mistralai/devstral-2512:free"
)

class RetrievedChunk(BaseModel):
    """
    Model for a single retrieved chunk
    """
    text: str
    source_url: str
    chapter_title: str
    score: float
    confidence: Optional[float] = None


@function_tool
def retrieve_book_content(query: str, top_k: int = 5) -> List[Dict[str, Any]]:
    """
    Retrieve relevant book content from the Qdrant database based on the query.

    Args:
        query: The natural language query to search for
        top_k: Number of results to retrieve (default: 5)

    Returns:
        List of retrieved content chunks with metadata
    """
    logger.info(f"Retrieving book content for query: '{query[:50]}...'")

    try:
        # Use the existing retrieval function
        results = query_qdrant(query, top_k=top_k)

        # Calculate confidence scores for the retrieved chunks
        citation_service = CitationService()
        enhanced_chunks = citation_service.calculate_confidence_scores(results)

        # Convert to dictionaries for return
        return [chunk.dict() for chunk in enhanced_chunks]

    except Exception as e:
        logger.error(f"Error retrieving book content: {str(e)}")
        raise


class RAGAgent:
    """
    RAG Agent that integrates OpenAI Agents SDK with Qdrant retrieval system.
    """

    def __init__(self):
        """
        Initialize the RAG Agent with OpenAI Agents SDK and configuration.
        """
        # Validate configuration
        AgentConfig.validate()

        # Configuration
        self.model = AgentConfig.OPENAI_MODEL
        self.top_k = AgentConfig.RETRIEVAL_TOP_K
        self.temperature = AgentConfig.AGENT_TEMPERATURE

        # Create the agent with retrieval tool
        self.agent = Agent(
            name="Physical AI & Humanoid Robotics Assistant",
            instructions="""You are an AI assistant for the Physical AI & Humanoid Robotics textbook. Your responses must be based ONLY on the provided book content. Do not use any external knowledge or make up information. If the provided context doesn't contain enough information to answer the question, clearly state that the information is not available in the provided content.

Guidelines:
1. Answer based solely on the provided context from the retrieve_book_content tool
2. Cite specific chapters or sections when possible
3. If uncertain, say you don't have enough information from the provided content
4. Maintain academic accuracy and clarity
5. Provide concise, helpful responses""",
            tools=[retrieve_book_content],
            model=router_model
        )

        logger.info("RAG Agent initialized successfully with OpenAI Agents SDK")

    def query_agent(self, user_query: str, selected_text: Optional[str] = None, top_k: Optional[int] = None, temperature: Optional[float] = None) -> Dict[str, Any]:
        """
        Process a user query using the RAG agent.

        Args:
            user_query (str): The user's query
            selected_text (Optional[str]): Additional context from selected text (optional)
            top_k (Optional[int]): Number of results to retrieve (uses default if None)
            temperature (Optional[float]): Creativity control (uses default if None)

        Returns:
            Dict[str, Any]: Response containing the answer and metadata
        """
        start_time = datetime.now()

        # Use provided parameters or defaults
        local_top_k = top_k if top_k is not None else self.top_k
        local_temperature = temperature if temperature is not None else self.temperature

        try:
            logger.info(f"Processing query: '{user_query[:50]}...'")

            # Add selected text to the query if provided
            if selected_text:
                full_query = f"{user_query}\n\nAdditional context: {selected_text}"
            else:
                full_query = user_query

            # Use the Agents SDK to run the agent
            result = Runner.run_sync(
                self.agent,
                full_query
            )

            # Extract the response
            agent_response = result.final_output

            # Extract context used from tool calls
            context_used = []
            # Note: In the Agents SDK, we'd need to extract the tool call results
            # For now, we'll simulate this by running the tool separately to get context
            try:
                temp_context = retrieve_book_content(user_query, local_top_k)
                context_used = temp_context
            except Exception as e:
                logger.warning(f"Could not retrieve context for response: {str(e)}")
                context_used = []

            # Calculate execution time
            execution_time_ms = (datetime.now() - start_time).total_seconds() * 1000

            # Prepare the result
            result_dict = {
                "query": user_query,
                "response": agent_response,
                "context_used": context_used,
                "thread_id": None,  # Not using threads in this implementation
                "execution_time_ms": execution_time_ms,
                "timestamp": datetime.now().isoformat(),
                "model_used": self.model
            }

            logger.info(f"Query processed successfully in {execution_time_ms:.2f}ms")
            return result_dict

        except Exception as e:
            logger.error(f"Error processing query: {str(e)}")
            execution_time_ms = (datetime.now() - start_time).total_seconds() * 1000

            return {
                "query": user_query,
                "response": f"Error processing query: {str(e)}",
                "context_used": [],
                "thread_id": None,
                "execution_time_ms": execution_time_ms,
                "timestamp": datetime.now().isoformat(),
                "model_used": self.model,
                "error": str(e)
            }

    def create_assistant(self, name: str = "Physical AI & Humanoid Robotics Assistant",
                        instructions: str = None) -> str:
        """
        Create an OpenAI assistant for the RAG system (one-time setup).

        Args:
            name (str): Name for the assistant
            instructions (str): Custom instructions for the assistant (optional)

        Returns:
            str: Assistant ID (though in Agents SDK this is handled differently)
        """
        # In the Agents SDK, assistants are created as Agent objects, not through the API
        # This method is kept for compatibility but returns a placeholder
        logger.info(f"Agent configured with name: {name}")
        return "agents-sdk-agent-placeholder-id"


def main():
    """
    Main function to demonstrate the RAG Agent usage.
    """
    import argparse

    parser = argparse.ArgumentParser(description='Agent for textbook understandings.')
    parser.add_argument('--query', type=str, help='The query text to process')
    parser.add_argument('--setup', action='store_true', help='Setup the assistant (one-time)')

    args = parser.parse_args()

    # Check if required environment variables are set
    if not os.getenv('OPENAI_API_KEY'):
        print("Error: OPENAI_API_KEY environment variable is required.")
        print("Please set it before running the script:")
        print("  export OPENAI_API_KEY='your-openai-api-key'")
        print("  set OPENAI_API_KEY=your-openai-api-key  # on Windows")
        return

    if not os.getenv('COHERE_API_KEY'):
        print("Error: COHERE_API_KEY environment variable is required for retrieval.")
        return

    try:
        # Initialize the RAG Agent
        agent = RAGAgent()

        if args.setup:
            # In the Agents SDK, setup is handled automatically when the agent is created
            print("Agent initialized successfully with OpenAI Agents SDK.")
            print("No additional setup required.")
        elif args.query:
            # Process a query
            result = agent.query_agent(args.query)
            print("Query Result:")
            print(json.dumps(result, indent=2))
        else:
            print("Usage:")
            print("  python agent.py --query 'your query here'     # Process a query")
            print("  python agent.py --setup                       # Initialize the agent")
            print("\nExample:")
            print("  python agent.py --query 'Explain ROS 2 architecture'")

    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()