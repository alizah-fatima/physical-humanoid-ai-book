"""
Caching service for RAG Agent to optimize response times
"""
import hashlib
import json
import time
from typing import Any, Optional, Dict
from datetime import datetime, timedelta
from dataclasses import dataclass, field


@dataclass
class CacheEntry:
    """Represents a cached entry with its metadata"""
    data: Any
    created_at: datetime
    ttl_seconds: int
    query_hash: str

    def is_expired(self) -> bool:
        """Check if the cache entry has expired"""
        return datetime.now() > (self.created_at + timedelta(seconds=self.ttl_seconds))

    def is_valid(self) -> bool:
        """Check if the cache entry is still valid"""
        return not self.is_expired()


class SimpleCacheService:
    """Simple in-memory cache service for the RAG Agent"""

    def __init__(self, default_ttl: int = 300):  # 5 minutes default TTL
        self.cache: Dict[str, CacheEntry] = {}
        self.default_ttl = default_ttl

    def generate_query_hash(self, query: str, top_k: int, temperature: float) -> str:
        """
        Generate a hash for the query to use as cache key.

        Args:
            query: The query text
            top_k: Number of results to retrieve
            temperature: Temperature parameter

        Returns:
            Hash string representing the query
        """
        query_data = {
            "query": query,
            "top_k": top_k,
            "temperature": temperature
        }
        query_str = json.dumps(query_data, sort_keys=True)
        return hashlib.sha256(query_str.encode()).hexdigest()

    def get(self, query_hash: str) -> Optional[Any]:
        """
        Get a value from the cache by its hash.

        Args:
            query_hash: The hash key for the cached value

        Returns:
            Cached value if found and valid, None otherwise
        """
        if query_hash in self.cache:
            entry = self.cache[query_hash]
            if entry.is_valid():
                return entry.data
            else:
                # Remove expired entry
                del self.cache[query_hash]
        return None

    def set(self, query_hash: str, data: Any, ttl_seconds: Optional[int] = None) -> None:
        """
        Set a value in the cache.

        Args:
            query_hash: The hash key for the cached value
            data: The data to cache
            ttl_seconds: Time to live in seconds (uses default if not provided)
        """
        ttl = ttl_seconds if ttl_seconds is not None else self.default_ttl
        entry = CacheEntry(
            data=data,
            created_at=datetime.now(),
            ttl_seconds=ttl,
            query_hash=query_hash
        )
        self.cache[query_hash] = entry

    def delete(self, query_hash: str) -> bool:
        """
        Delete a value from the cache.

        Args:
            query_hash: The hash key for the cached value

        Returns:
            True if the value was found and deleted, False otherwise
        """
        if query_hash in self.cache:
            del self.cache[query_hash]
            return True
        return False

    def clear_expired(self) -> int:
        """
        Clear all expired entries from the cache.

        Returns:
            Number of entries removed
        """
        expired_keys = []
        for key, entry in self.cache.items():
            if entry.is_expired():
                expired_keys.append(key)

        for key in expired_keys:
            del self.cache[key]

        return len(expired_keys)

    def clear_all(self) -> None:
        """Clear all entries from the cache."""
        self.cache.clear()

    def get_stats(self) -> Dict[str, Any]:
        """
        Get cache statistics.

        Returns:
            Dictionary with cache statistics
        """
        total_entries = len(self.cache)
        valid_entries = sum(1 for entry in self.cache.values() if entry.is_valid())
        expired_entries = total_entries - valid_entries

        return {
            "total_entries": total_entries,
            "valid_entries": valid_entries,
            "expired_entries": expired_entries,
            "default_ttl": self.default_ttl
        }


class QueryResponseCache:
    """Cache service specifically for query responses"""

    def __init__(self, ttl_seconds: int = 600):  # 10 minutes for query responses
        self.cache_service = SimpleCacheService(ttl_seconds)

    def get_cached_response(self, query: str, top_k: int = 5, temperature: float = 0.3) -> Optional[Dict[str, Any]]:
        """
        Get a cached response for a query.

        Args:
            query: The query text
            top_k: Number of results to retrieve
            temperature: Temperature parameter

        Returns:
            Cached response if found, None otherwise
        """
        query_hash = self.cache_service.generate_query_hash(query, top_k, temperature)
        return self.cache_service.get(query_hash)

    def cache_response(self, query: str, response: Dict[str, Any], top_k: int = 5, temperature: float = 0.3) -> None:
        """
        Cache a response for a query.

        Args:
            query: The query text
            response: The response to cache
            top_k: Number of results to retrieve
            temperature: Temperature parameter
        """
        query_hash = self.cache_service.generate_query_hash(query, top_k, temperature)
        self.cache_service.set(query_hash, response)

    def invalidate_query(self, query: str, top_k: int = 5, temperature: float = 0.3) -> bool:
        """
        Invalidate the cache for a specific query.

        Args:
            query: The query text
            top_k: Number of results to retrieve
            temperature: Temperature parameter

        Returns:
            True if cache entry was found and deleted, False otherwise
        """
        query_hash = self.cache_service.generate_query_hash(query, top_k, temperature)
        return self.cache_service.delete(query_hash)


# Global cache instance
query_cache = QueryResponseCache()


def get_cache() -> QueryResponseCache:
    """Get the global cache instance"""
    return query_cache