# Backend Services for RAG System

This project implements backend services for the Physical AI & Humanoid Robotics textbook RAG system, including both ingestion and retrieval pipelines.

## Ingestion Pipeline
The ingestion pipeline crawls the textbook website, extracts text content, generates embeddings using Cohere, and stores them in Qdrant with metadata.

## Retrieval Pipeline
The retrieval pipeline handles querying the vector database to find relevant textbook content based on user queries.

## Prerequisites

- Python 3.9+
- UV package manager
- Cohere API key
- Qdrant Cloud API key and endpoint URL

## Setup

1. Clone the repository
2. Navigate to the backend directory:
   ```bash
   cd backend
   ```
3. Install dependencies using UV:
   ```bash
   uv pip install -r requirements.txt
   ```
   Or using pip:
   ```bash
   pip install -r requirements.txt
   ```

4. Create a `.env` file with your API keys:
   ```env
   COHERE_API_KEY=your_cohere_api_key_here
   QDRANT_URL=your_qdrant_cluster_url_here
   QDRANT_API_KEY=your_qdrant_api_key_here
   BASE_URL=https://alizah-fatima.github.io/physical-humanoid-ai-book/
   ```

## Configuration

You can customize these environment variables:

- `CHUNK_SIZE`: Size of text chunks (default: 1000)
- `CHUNK_OVERLAP`: Overlap between chunks (default: 100)
- `BATCH_SIZE`: Number of chunks to process at once (default: 10)

## Usage

Run the ingestion pipeline:

```bash
python main.py
```

## Pipeline Overview

The pipeline performs the following steps:

1. **URL Discovery**: Crawls the target site to discover all accessible pages
2. **Content Extraction**: Extracts clean text content and chapter titles from each page
3. **Text Chunking**: Splits content into appropriately sized chunks with overlap to preserve context
4. **Embedding Generation**: Creates vector embeddings using the Cohere API
5. **Storage**: Saves embeddings with metadata to Qdrant vector database

## Functions

- `get_all_urls(base_url)`: Discovers all accessible URLs on the target site
- `extract_text_from_url(url)`: Extracts clean text and chapter title from a URL
- `chunk_text(text, chapter_title, source_url)`: Splits text into chunks with context preservation
- `embed(text_chunks, cohere_client)`: Generates embeddings for text chunks
- `create_collection(collection_name)`: Creates a Qdrant collection for storing vectors
- `save_chunk_to_qdrant(chunk_data, embedding)`: Saves a chunk with its embedding to Qdrant

## Data Stored in Qdrant

Each entry in the Qdrant collection contains:
- Vector embedding (1024 dimensions from Cohere)
- Metadata:
  - `source_url`: Original URL of the content
  - `chapter_title`: Title of the chapter/section
  - `chunk_index`: Position of the chunk within the document
  - `content`: The actual text content of the chunk

## Retrieval Pipeline Usage

The retrieval pipeline is available in `retrieve.py` and can be used to query the vector database:

```python
from retrieve import query_qdrant

# Query for relevant content
results = query_qdrant("Explain ROS2 architecture", top_k=5)

# Process the results
for result in results:
    print(f"Score: {result['score']}")
    print(f"Text: {result['text'][:100]}...")
    print(f"Source: {result['source_url']}")
    print(f"Chapter: {result['chapter_title']}")
    print("---")
```

### Configuration

The retrieval pipeline uses the following environment variables:

- `COHERE_API_KEY`: Your Cohere API key for embedding generation
- `QDRANT_HOST`: Host address for Qdrant database (default: localhost)
- `QDRANT_PORT`: Port for Qdrant database (default: 6333)
- `QDRANT_API_KEY`: API key for Qdrant authentication (optional)

### API

#### `query_qdrant(query_text: str, top_k: int = 5) -> List[Dict[str, Any]]`

Queries the Qdrant database for content relevant to the provided query text.

Parameters:
- `query_text`: The natural language query to search for
- `top_k`: Number of top results to retrieve (default: 5)

Returns:
- List of dictionaries containing:
  - `text`: The content text of the retrieved chunk
  - `source_url`: URL or identifier of the source document
  - `chapter_title`: Title of the chapter containing this chunk
  - `score`: Similarity score between query and chunk (0.0-1.0)