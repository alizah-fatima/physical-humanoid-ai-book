# Quickstart Guide: Content Ingestion Pipeline

**Feature**: 1-ingestion-pipeline
**Created**: 2025-12-24

## Prerequisites

- Python 3.9 or higher
- UV package manager
- Cohere API key
- Qdrant Cloud API key and endpoint URL

## Setup Instructions

### 1. Clone and Navigate to Project
```bash
git clone <repository-url>
cd <repository-directory>
```

### 2. Create Backend Directory
```bash
mkdir backend
cd backend
```

### 3. Install UV Package Manager (if not already installed)
```bash
pip install uv
```

### 4. Initialize Python Project
```bash
uv init
```

### 5. Create Virtual Environment and Install Dependencies
```bash
uv venv
source .venv/bin/activate  # On Windows: .venv\Scripts\activate
uv pip install requests beautifulsoup4 cohere qdrant-client python-dotenv
```

## Environment Configuration

Create a `.env` file in the backend directory with the following content:

```env
COHERE_API_KEY=your_cohere_api_key_here
QDRANT_API_KEY=your_qdrant_api_key_here
QDRANT_URL=your_qdrant_cluster_url_here
BASE_URL=https://alizah-fatima.github.io/physical-humanoid-ai-book/
```

## Running the Pipeline

1. Make sure you're in the backend directory
2. Ensure your virtual environment is activated
3. Run the main script:

```bash
python main.py
```

## Expected Output

The pipeline will:
1. Discover all URLs from the base documentation site
2. Extract text content from each page
3. Chunk the text appropriately
4. Generate embeddings using Cohere
5. Store the embeddings in Qdrant with metadata
6. Provide a summary of the processing results

## Troubleshooting

### Common Issues

- **API Rate Limits**: The pipeline includes rate limiting, but if you encounter issues, reduce the concurrent processing.
- **Connection Errors**: Verify your API keys and network connection.
- **Memory Issues**: For large sites, the pipeline processes content in batches to manage memory usage.

## Verification

After running the pipeline, you can verify the results by:
1. Checking the Qdrant dashboard to confirm collections were created
2. Verifying the "ai_rag_embedding" collection contains the expected number of vectors
3. Confirming metadata includes proper source URLs and chapter titles