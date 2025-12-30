"""
Hugging Face Space Application Entry Point for Backend
This file serves as the entry point for Hugging Face Spaces deployment.
"""
import sys
import os

# Add the project root to the Python path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from api.main import app

# For Hugging Face Spaces, we'll use the app object directly
# The space will handle the uvicorn server configuration

if __name__ == "__main__":
    # This is for local testing of the Hugging Face Space configuration
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=int(os.environ.get("PORT", 7860)))