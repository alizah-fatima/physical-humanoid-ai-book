import sys
import os
import json
import argparse
import logging
import io
from contextlib import redirect_stdout, redirect_stderr

# Add the project root to the path
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, project_root)

def run_query(query, selected_text="", top_k=5, temperature=0.3):
    """Run a query using the RAG agent."""
    # Completely suppress all logging during execution
    logging.getLogger().setLevel(logging.CRITICAL)

    # Import after setting logging level to suppress module-level logs
    from backend.agent import RAGAgent

    # Redirect all output during execution to suppress any print statements or logs
    with redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()):
        agent = RAGAgent()
        result = agent.query_agent(
            user_query=query,
            selected_text=selected_text,
            top_k=top_k,
            temperature=temperature
        )

    return result

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Run RAG agent query')
    parser.add_argument('--query', required=True, help='The query to process')
    parser.add_argument('--selected_text', default="", help='Selected text context')
    parser.add_argument('--top_k', type=int, default=5, help='Number of results to retrieve')
    parser.add_argument('--temperature', type=float, default=0.3, help='Temperature for response')

    args = parser.parse_args()

    result = run_query(args.query, args.selected_text, args.top_k, args.temperature)
    print(json.dumps(result))