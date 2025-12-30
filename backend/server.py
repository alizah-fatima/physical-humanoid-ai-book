"""
Simple API server for RAG Agent
"""
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
import uvicorn
import sys
import os

# Add the parent directory to the path to import modules
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from backend.api.endpoints import router as api_router
from backend.api.docs import customize_openapi

# Create the FastAPI application
app = FastAPI(
    title="RAG Agent API",
    description="API for RAG (Retrieval-Augmented Generation) agent that integrates with Qdrant retrieval system to provide responses based solely on book content.",
    version="1.0.0"
)

# Customize OpenAPI schema
app.openapi = lambda: customize_openapi(app)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, replace with specific origins
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Import and include API routes with versioning
app.include_router(api_router, prefix="/api/v1/agent", tags=["agent"])  # API versioning

# Root endpoint
@app.get("/")
def read_root():
    return {"message": "RAG Agent API", "status": "running"}

# Health check endpoint
@app.get("/health")
def health_check():
    return {"status": "healthy", "service": "RAG Agent API"}

if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=7860)