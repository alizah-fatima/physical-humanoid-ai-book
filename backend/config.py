"""
Configuration module for RAG Agent
"""
import os
from typing import Optional
from dotenv import load_dotenv

# Load environment variables from the root directory
root_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
env_path = os.path.join(root_dir, '.env')
load_dotenv(dotenv_path=env_path)

class AgentConfig:
    """Configuration class for the RAG Agent"""

    # OpenAI Configuration
    OPENAI_API_KEY: str = os.getenv("OPENAI_API_KEY", "")
    OPENAI_ASSISTANT_ID: Optional[str] = os.getenv("OPENAI_ASSISTANT_ID")
    OPENAI_MODEL: str = os.getenv("OPENAI_MODEL", "gpt-4o")

    # OpenRouter Configuration
    OPENROUTER_API_KEY: str = os.getenv("OPENROUTER_API_KEY", "")
    OPENROUTER_MODEL: str = os.getenv("OPENROUTER_MODEL", "openai/gpt-4o")

    # Use OpenRouter flag
    USE_OPENROUTER: bool = os.getenv("USE_OPENROUTER", "false").lower() == "true"

    # Cohere Configuration
    COHERE_API_KEY: str = os.getenv("COHERE_API_KEY", "")

    # Qdrant Configuration
    QDRANT_URL: Optional[str] = os.getenv("QDRANT_URL")
    QDRANT_HOST: str = os.getenv("QDRANT_HOST", "localhost")
    QDRANT_PORT: int = int(os.getenv("QDRANT_PORT", "6333"))
    QDRANT_API_KEY: Optional[str] = os.getenv("QDRANT_API_KEY")

    # Agent Configuration
    RETRIEVAL_TOP_K: int = int(os.getenv("RETRIEVAL_TOP_K", "5"))
    AGENT_TEMPERATURE: float = float(os.getenv("AGENT_TEMPERATURE", "0.3"))

    # Validation
    @classmethod
    def validate(cls) -> bool:
        """Validate that required configuration is present"""
        if cls.USE_OPENROUTER:
            if not cls.OPENROUTER_API_KEY:
                raise ValueError("OPENROUTER_API_KEY environment variable is required when USE_OPENROUTER is true")
        else:
            if not cls.OPENAI_API_KEY:
                raise ValueError("OPENAI_API_KEY environment variable is required when USE_OPENROUTER is false")
        if not cls.COHERE_API_KEY:
            raise ValueError("COHERE_API_KEY environment variable is required")
        return True