"""Caching service for query results and embeddings."""

import logging
import json
import hashlib
from typing import Optional, Any
from datetime import timedelta

logger = logging.getLogger(__name__)


class CacheService:
    """Service for caching query results and embeddings."""

    def __init__(self, redis_client=None):
        """Initialize cache service with optional Redis client."""
        self.redis = redis_client
        self.cache_ttl = 3600  # 1 hour default

    def _generate_key(self, prefix: str, data: dict) -> str:
        """Generate a cache key from data."""
        data_str = json.dumps(data, sort_keys=True)
        data_hash = hashlib.md5(data_str.encode()).hexdigest()
        return f"{prefix}:{data_hash}"

    async def get_query_cache(
        self,
        question: str,
        session_id: str,
    ) -> Optional[dict]:
        """Get cached query result."""
        if not self.redis:
            return None

        try:
            key = self._generate_key("query", {
                "question": question,
                "session_id": str(session_id),
            })

            cached = await self.redis.get(key)
            if cached:
                logger.info(f"✓ Cache hit for query: {question[:50]}...")
                return json.loads(cached)

            return None
        except Exception as e:
            logger.warning(f"Cache retrieval error: {e}")
            return None

    async def set_query_cache(
        self,
        question: str,
        session_id: str,
        result: dict,
        ttl_seconds: int = 3600,
    ) -> bool:
        """Cache a query result."""
        if not self.redis:
            return False

        try:
            key = self._generate_key("query", {
                "question": question,
                "session_id": str(session_id),
            })

            await self.redis.setex(
                key,
                ttl_seconds,
                json.dumps(result),
            )
            logger.info(f"✓ Cached query result: {key}")
            return True
        except Exception as e:
            logger.warning(f"Cache set error: {e}")
            return False

    async def get_embedding_cache(self, text: str) -> Optional[list]:
        """Get cached embedding for text."""
        if not self.redis:
            return None

        try:
            key = self._generate_key("embedding", {"text": text})
            cached = await self.redis.get(key)

            if cached:
                logger.info(f"✓ Cache hit for embedding: {text[:30]}...")
                return json.loads(cached)

            return None
        except Exception as e:
            logger.warning(f"Embedding cache error: {e}")
            return None

    async def set_embedding_cache(
        self,
        text: str,
        embedding: list,
        ttl_seconds: int = 86400,  # 24 hours
    ) -> bool:
        """Cache an embedding."""
        if not self.redis:
            return False

        try:
            key = self._generate_key("embedding", {"text": text})
            await self.redis.setex(
                key,
                ttl_seconds,
                json.dumps(embedding),
            )
            logger.info(f"✓ Cached embedding: {key}")
            return True
        except Exception as e:
            logger.warning(f"Embedding cache set error: {e}")
            return False

    async def clear_session_cache(self, session_id: str) -> bool:
        """Clear all cache for a session."""
        if not self.redis:
            return False

        try:
            pattern = f"query:*{session_id}*"
            keys = await self.redis.keys(pattern)

            if keys:
                await self.redis.delete(*keys)
                logger.info(f"✓ Cleared {len(keys)} cache entries for session")

            return True
        except Exception as e:
            logger.warning(f"Cache clear error: {e}")
            return False

    async def get_cache_stats(self) -> dict:
        """Get cache statistics."""
        try:
            if not self.redis:
                return {"status": "disabled"}

            info = await self.redis.info()
            return {
                "status": "active",
                "used_memory": info.get("used_memory_human"),
                "connected_clients": info.get("connected_clients"),
                "total_commands": info.get("total_commands_processed"),
            }
        except Exception as e:
            logger.warning(f"Cache stats error: {e}")
            return {"status": "error", "details": str(e)}


# Global instance
_cache_service = None


def get_cache_service(redis_client=None) -> CacheService:
    """Get or create cache service instance."""
    global _cache_service
    if _cache_service is None:
        _cache_service = CacheService(redis_client)
    return _cache_service
