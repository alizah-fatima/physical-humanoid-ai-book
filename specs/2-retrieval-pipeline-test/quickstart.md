# Quickstart: Retrieval Pipeline

## Overview
This guide provides a quick setup and usage guide for the retrieval pipeline that queries Qdrant using Cohere embeddings.

## Prerequisites
- Python 3.8 or higher
- Cohere API key
- Qdrant database access credentials
- Environment with required Python packages installed

## Installation
1. Install required packages:
```bash
pip install cohere qdrant-client python-dotenv
```

2. Set up environment variables:
```bash
export COHERE_API_KEY="your-cohere-api-key"
export QDRANT_HOST="your-qdrant-host"
export QDRANT_PORT="6333"
export QDRANT_API_KEY="your-qdrant-api-key"  # if authentication required
```

## Usage Examples

### Basic Query
```python
from retrieve import query_qdrant

# Query the database for relevant content
results = query_qdrant("Explain ROS2 architecture", top_k=5)

# Process the results
for result in results:
    print(f"Score: {result['score']}")
    print(f"Text: {result['text'][:100]}...")
    print(f"Source: {result['source_url']}")
    print("---")
```

### Configuration
The system uses environment variables for configuration:
- `COHERE_API_KEY`: Your Cohere API key
- `QDRANT_HOST`: Host address for Qdrant database
- `QDRANT_PORT`: Port for Qdrant database (default: 6333)
- `QDRANT_API_KEY`: API key for Qdrant authentication (if required)

## Testing
Run the following to test the retrieval pipeline:
```bash
python -c "from retrieve import query_qdrant; print(query_qdrant('test query', top_k=1))"
```

## Troubleshooting
- **Connection errors**: Verify Qdrant host and port configuration
- **Authentication errors**: Check API keys are properly set
- **Empty results**: Verify the database has content and the collection name is correct
- **Slow performance**: Check network connectivity and database indexing