"""
Content Ingestion Pipeline for RAG System

This script implements a pipeline to:
1. Crawl the Physical AI & Humanoid Robotics textbook website
2. Extract text content from all pages
3. Generate embeddings using Cohere
4. Store embeddings in Qdrant with metadata
"""
import os
import logging
from typing import List, Tuple, Dict, Any
from urllib.parse import urljoin, urlparse
import requests
from bs4 import BeautifulSoup
import cohere
from qdrant_client import QdrantClient
from qdrant_client.http import models
from dotenv import load_dotenv


# Load environment variables
load_dotenv()

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

# Configuration constants
CHUNK_SIZE = int(os.getenv('CHUNK_SIZE', '1000'))
CHUNK_OVERLAP = int(os.getenv('CHUNK_OVERLAP', '100'))
BATCH_SIZE = int(os.getenv('BATCH_SIZE', '10'))
BASE_URL = os.getenv('BASE_URL', 'https://alizah-fatima.github.io/physical-humanoid-ai-book/')
QDRANT_URL = os.getenv('QDRANT_URL')
QDRANT_API_KEY = os.getenv('QDRANT_API_KEY')
COHERE_API_KEY = os.getenv('COHERE_API_KEY')
COLLECTION_NAME = "ai_rag_embedding"


def save_chunk_to_qdrant(chunk_data: Dict[str, str], embedding: List[float]) -> bool:
    """
    Saves a text chunk with its embedding to Qdrant.

    Args:
        chunk_data: Dictionary containing chunk content and metadata
        embedding: The embedding vector for the chunk

    Returns:
        True if successfully saved
    """
    from qdrant_client import QdrantClient
    from qdrant_client.http import models
    import uuid

    # Initialize Qdrant client
    qdrant_client = QdrantClient(
        url=QDRANT_URL,
        api_key=QDRANT_API_KEY,
    )

    try:
        # Prepare the payload with metadata
        payload = {
            "source_url": chunk_data['source_url'],
            "chapter_title": chunk_data['chapter_title'],
            "chunk_index": chunk_data['chunk_index'],
            "content": chunk_data['content']
        }

        # Generate a unique ID for this record
        record_id = str(uuid.uuid5(uuid.NAMESPACE_URL, chunk_data['id']))

        # Upsert the record to Qdrant
        qdrant_client.upsert(
            collection_name=COLLECTION_NAME,
            points=[
                models.PointStruct(
                    id=record_id,
                    vector=embedding,
                    payload=payload
                )
            ]
        )

        logger.info(f"Saved chunk to Qdrant: {chunk_data['source_url']}#{chunk_data['chunk_index']}")
        return True

    except Exception as e:
        logger.error(f"Error saving chunk to Qdrant: {str(e)}")
        return False


def embed(text_chunks: List[str], cohere_client: Any) -> List[List[float]]:
    """
    Generates embeddings for a list of text chunks.

    Args:
        text_chunks: List of text chunks to embed
        cohere_client: Initialized Cohere client

    Returns:
        List of embedding vectors (each a list of floats)
    """
    import time

    # Process in batches to respect API limits
    embeddings = []
    for i in range(0, len(text_chunks), BATCH_SIZE):
        batch = text_chunks[i:i + BATCH_SIZE]

        try:
            # Generate embeddings using Cohere
            response = cohere_client.embed(
                texts=batch,
                model='embed-multilingual-v3.0',  # Using multilingual model for broader support
                input_type="search_document"  # Specify the input type
            )

            # Extract embeddings from the response
            batch_embeddings = [embedding for embedding in response.embeddings]
            embeddings.extend(batch_embeddings)

            logger.info(f"Generated embeddings for batch {i//BATCH_SIZE + 1}/{(len(text_chunks)-1)//BATCH_SIZE + 1}")

            # Add a small delay to respect rate limits
            time.sleep(0.1)

        except Exception as e:
            logger.error(f"Error generating embeddings for batch {i//BATCH_SIZE + 1}: {str(e)}")
            # Return zeros for failed embeddings to maintain alignment
            failed_embeddings = [[0.0] * 1024 for _ in range(len(batch))]
            embeddings.extend(failed_embeddings)
            continue

    return embeddings


def chunk_text(text: str, chapter_title: str, source_url: str, chunk_size: int = 1000) -> List[Dict[str, str]]:
    """
    Splits text into chunks of specified size with context preservation.

    Args:
        text: The text to be chunked
        chapter_title: Title of the chapter this text belongs to
        source_url: URL where the text was found
        chunk_size: Maximum size of each chunk (default 1000 characters)

    Returns:
        List of dictionaries containing chunk information
    """
    import re

    # Split text into sentences to avoid breaking in the middle of sentences
    sentences = re.split(r'[.!?]+\s+', text)
    chunks = []
    current_chunk = ""
    chunk_index = 0

    for sentence in sentences:
        # If adding this sentence would exceed chunk size
        if len(current_chunk) + len(sentence) > chunk_size and current_chunk:
            # Add the current chunk to the list
            chunks.append({
                'id': f"{source_url}#{chunk_index}",
                'content': current_chunk.strip(),
                'source_url': source_url,
                'chapter_title': chapter_title,
                'chunk_index': chunk_index
            })
            chunk_index += 1

            # Start a new chunk with some overlap to preserve context
            # Take the last part of the current chunk as the beginning of the next
            overlap_size = min(CHUNK_OVERLAP, len(current_chunk))
            current_chunk = current_chunk[-overlap_size:] + " " + sentence + " "
        else:
            # Add the sentence to the current chunk
            current_chunk += sentence + ". "

    # Add the last chunk if it has content
    if current_chunk.strip():
        chunks.append({
            'id': f"{source_url}#{chunk_index}",
            'content': current_chunk.strip(),
            'source_url': source_url,
            'chapter_title': chapter_title,
            'chunk_index': chunk_index
        })

    return chunks


def extract_text_from_url(url: str) -> Tuple[str, str]:
    """
    Extracts clean text content and chapter title from a URL.

    Args:
        url: The URL to extract content from

    Returns:
        Tuple of (chapter_title, clean_text_content)
    """
    import requests
    from bs4 import BeautifulSoup

    try:
        response = requests.get(url)
        response.raise_for_status()

        soup = BeautifulSoup(response.content, 'html.parser')

        # Extract chapter title - try multiple common selectors
        chapter_title = ""
        title_selectors = ['title', 'h1', '.hero__title', '.docTitle', '.post-title']

        for selector in title_selectors:
            if selector.startswith('.'):
                element = soup.select_one(selector)
            else:
                element = soup.find(selector)

            if element:
                chapter_title = element.get_text().strip()
                break

        # If no title found, use the URL path as title
        if not chapter_title:
            from urllib.parse import urlparse
            path = urlparse(url).path
            chapter_title = path.strip('/').replace('/', ' - ') or "Untitled"

        # Extract main content - try multiple common selectors for main content
        main_content = ""
        content_selectors = [
            'main',
            '.main-content',
            '.container',
            '.post-content',
            '.markdown',
            '.doc-content',
            '.docs-content',
            '.content',
            'article',
            '.article'
        ]

        for selector in content_selectors:
            element = soup.select_one(selector)
            if element:
                main_content = element.get_text()
                break

        # If no specific content container found, get all text from body
        if not main_content:
            body = soup.find('body')
            if body:
                main_content = body.get_text()

        # Clean up the text content
        import re
        # Remove extra whitespace and newlines
        clean_content = re.sub(r'\n\s*\n', '\n\n', main_content)
        clean_content = re.sub(r'[ \t]+', ' ', clean_content)
        clean_content = clean_content.strip()

        # Remove navigation elements, headers, footers, and sidebars if they were included
        lines = clean_content.split('\n')
        filtered_lines = []
        for line in lines:
            line = line.strip()
            if line and len(line) > 10:  # Skip very short lines that might be navigation
                # Skip lines that look like navigation or menu items
                if not any(keyword in line.lower() for keyword in
                          ['menu', 'nav', 'footer', 'header', 'sidebar', 'navigation',
                           'previous', 'next', 'table of contents', 'toc']):
                    filtered_lines.append(line)

        clean_content = '\n'.join(filtered_lines)

        return chapter_title, clean_content

    except Exception as e:
        logger.error(f"Error extracting text from {url}: {str(e)}")
        raise ValueError(f"Could not extract content from {url}: {str(e)}")


def get_all_urls(base_url: str) -> List[str]:
    """
    Crawls the base URL and returns all accessible page URLs.

    Args:
        base_url: The root URL to start crawling from

    Returns:
        List of all discovered URLs within the site
    """
    from urllib.parse import urljoin, urlparse
    import requests
    from bs4 import BeautifulSoup
    import time

    # Parse the base URL to get the domain
    parsed_base = urlparse(base_url)
    base_domain = f"{parsed_base.scheme}://{parsed_base.netloc}"

    # Set to keep track of visited URLs
    visited_urls = set()
    # Queue for URLs to visit
    urls_to_visit = [base_url]
    # List to store all discovered URLs
    all_urls = []

    # Limit the number of pages to crawl to prevent infinite crawling
    max_pages = 100
    pages_crawled = 0

    while urls_to_visit and pages_crawled < max_pages:
        current_url = urls_to_visit.pop(0)

        # Skip if already visited
        if current_url in visited_urls:
            continue

        try:
            # Check if URL is within the same domain
            parsed_url = urlparse(current_url)
            current_domain = f"{parsed_url.scheme}://{parsed_url.netloc}"

            if current_domain != base_domain:
                continue

            # Mark as visited
            visited_urls.add(current_url)

            # Make request to the URL
            response = requests.get(current_url)
            response.raise_for_status()

            # Parse HTML content
            soup = BeautifulSoup(response.content, 'html.parser')

            # Add to the list of discovered URLs
            all_urls.append(current_url)
            logger.info(f"Crawled: {current_url}")

            # Find all links in the page
            for link in soup.find_all('a', href=True):
                href = link['href']
                absolute_url = urljoin(current_url, href)

                # Only add URLs that are within the same domain and not already visited
                parsed_link = urlparse(absolute_url)
                link_domain = f"{parsed_link.scheme}://{parsed_link.netloc}"

                if link_domain == base_domain and absolute_url not in visited_urls:
                    urls_to_visit.append(absolute_url)

            pages_crawled += 1
            # Add a small delay to be respectful to the server
            time.sleep(0.1)

        except Exception as e:
            logger.error(f"Error crawling {current_url}: {str(e)}")
            continue

    return list(set(all_urls))  # Remove any potential duplicates


def create_collection(collection_name: str) -> bool:
    """
    Creates a vector collection in Qdrant if it doesn't exist.

    Args:
        collection_name: Name of the collection to create

    Returns:
        True if collection was created or already exists
    """
    from qdrant_client import QdrantClient
    from qdrant_client.http import models

    # Initialize Qdrant client
    qdrant_client = QdrantClient(
        url=QDRANT_URL,
        api_key=QDRANT_API_KEY,
    )

    try:
        # Check if collection already exists
        collections = qdrant_client.get_collections()
        collection_exists = any(col.name == collection_name for col in collections.collections)

        if not collection_exists:
            # Create collection with 1024-dimensional vectors (for Cohere embeddings)
            qdrant_client.create_collection(
                collection_name=collection_name,
                vectors_config=models.VectorParams(size=1024, distance=models.Distance.COSINE)
            )
            logger.info(f"Created Qdrant collection: {collection_name}")
        else:
            logger.info(f"Qdrant collection already exists: {collection_name}")

        return True

    except Exception as e:
        logger.error(f"Error creating Qdrant collection {collection_name}: {str(e)}")
        return False


def main():
    """Main execution function that orchestrates the entire pipeline."""
    logger.info("Starting content ingestion pipeline...")

    # Initialize clients
    cohere_client = cohere.Client(COHERE_API_KEY)

    logger.info("Clients initialized successfully")

    # Create collection in Qdrant
    create_collection(COLLECTION_NAME)
    logger.info(f"Collection '{COLLECTION_NAME}' created/verified")

    # Get all URLs from the site
    logger.info("Discovering URLs...")
    urls = get_all_urls(BASE_URL)
    logger.info(f"Discovered {len(urls)} URLs")

    # Process each URL
    processed_count = 0
    for url in urls:
        try:
            logger.info(f"Processing URL: {url}")

            # Extract text content and chapter title
            chapter_title, text_content = extract_text_from_url(url)

            if not text_content.strip():
                logger.warning(f"No content extracted from {url}, skipping...")
                continue

            # Chunk the text
            chunks = chunk_text(text_content, chapter_title, url)

            # Generate embeddings for chunks
            embeddings = embed([chunk['content'] for chunk in chunks], cohere_client)

            # Save chunks to Qdrant
            for chunk, embedding in zip(chunks, embeddings):
                save_chunk_to_qdrant(chunk, embedding)

            processed_count += 1
            logger.info(f"Successfully processed {url}")

        except Exception as e:
            logger.error(f"Error processing {url}: {str(e)}")
            continue

    logger.info(f"Pipeline completed. Processed {processed_count}/{len(urls)} URLs successfully")


if __name__ == "__main__":
    main()