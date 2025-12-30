"""
Hugging Face Space API for RAG Agent
This file serves as the application entry point for Hugging Face Spaces.
"""
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
import uvicorn
import sys
import os

# Add the project root directory to the Python path to allow proper imports
project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))  # Go up to project root
if project_root not in sys.path:
    sys.path.insert(0, project_root)

# Import based on execution context
try:
    # Try relative imports first (for when running as part of the package)
    from .api.endpoints import router as api_router
    from .api.docs import customize_openapi
except ImportError:
    # Fall back to absolute imports (for when running directly)
    from backend.api.endpoints import router as api_router
    from backend.api.docs import customize_openapi

# Create the FastAPI application
app = FastAPI(
    title="Physical AI & Humanoid Robotics RAG API",
    description="API for RAG (Retrieval-Augmented Generation) agent that integrates with Qdrant retrieval system to provide responses based solely on textbook content.",
    version="1.0.0"
)

# Customize OpenAPI schema
app.openapi = lambda: customize_openapi(app)

# Add CORS middleware to allow requests from GitHub Pages
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # Allow all origins for GitHub Pages compatibility
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Import and include API routes with versioning
app.include_router(api_router, prefix="/api/v1/agent", tags=["agent"])  # API versioning

# Root endpoint
@app.get("/")
def read_root():
    return {"message": "Physical AI & Humanoid Robotics RAG API", "status": "running"}

# Health check endpoint
@app.get("/health")
def health_check():
    return {"status": "healthy", "service": "RAG Agent API"}

# Configuration endpoint
@app.get("/config")
def config():
    return {"model": os.getenv("OPENAI_MODEL", "gpt-4o"), "status": "configured"}

if __name__ == "__main__":
    # For local testing
    uvicorn.run(app, host="0.0.0.0", port=int(os.environ.get("PORT", 7860)))