"""
Hugging Face Space Application Entry Point
This file serves as the entry point for Hugging Face Spaces deployment.
"""
import sys
import os

# Add the project root to the Python path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# Add the backend directory to the Python path
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), 'backend'))

from backend.api.main import app

# For Hugging Face Spaces, we'll use the app object directly
# The space will handle the uvicorn server configuration

# You can add any Hugging Face Space specific initialization here if needed
def initialize_app():
    """Initialize the application for Hugging Face Spaces"""
    print("Initializing RAG Agent API for Hugging Face Spaces...")
    # Add any initialization logic here if needed
    pass

# Initialize the app
initialize_app()

# Export the app for Hugging Face Spaces
if __name__ == "__main__":
    # This is for local testing of the Hugging Face Space configuration
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=int(os.environ.get("PORT", 7860)))