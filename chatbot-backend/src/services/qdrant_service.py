"""Qdrant vector database service for RAG retrieval."""

from typing import List, Dict, Any
from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams, PointStruct
import logging

from ..config import settings
from ..utils.errors import QdrantException

logger = logging.getLogger(__name__)


class QdrantService:
    """Service for Qdrant vector database operations."""

    def __init__(self):
        """Initialize Qdrant client."""
        try:
            self.client = QdrantClient(
                url=settings.qdrant_url,
                api_key=settings.qdrant_api_key,
            )
            logger.info(f"✓ Connected to Qdrant at {settings.qdrant_url}")
        except Exception as e:
            logger.error(f"✗ Failed to connect to Qdrant: {e}")
            raise QdrantException(f"Qdrant connection failed: {e}")

    async def search(
        self,
        query_vector: List[float],
        top_k: int = 5,
        score_threshold: float = 0.5
    ) -> List[Dict[str, Any]]:
        """Search for similar content chunks in Qdrant."""
        try:
            results = self.client.search(
                collection_name=settings.qdrant_collection_name,
                query_vector=query_vector,
                limit=top_k,
                score_threshold=score_threshold,
            )

            # Format results
            formatted_results = []
            for result in results:
                formatted_results.append({
                    "id": result.id,
                    "score": result.score,
                    "chapter": result.payload.get("chapter", ""),
                    "section": result.payload.get("section", ""),
                    "content": result.payload.get("content", ""),
                })

            logger.info(f"✓ Qdrant search returned {len(formatted_results)} results")
            return formatted_results

        except Exception as e:
            logger.error(f"✗ Qdrant search failed: {e}")
            raise QdrantException(f"Search failed: {e}")

    async def store_vector(
        self,
        vector_id: int,
        vector: List[float],
        payload: Dict[str, Any]
    ) -> bool:
        """Store vector in Qdrant."""
        try:
            point = PointStruct(
                id=vector_id,
                vector=vector,
                payload=payload,
            )

            self.client.upsert(
                collection_name=settings.qdrant_collection_name,
                points=[point],
            )

            logger.info(f"✓ Stored vector {vector_id} in Qdrant")
            return True

        except Exception as e:
            logger.error(f"✗ Failed to store vector: {e}")
            raise QdrantException(f"Store vector failed: {e}")

    async def health_check(self) -> bool:
        """Check Qdrant health."""
        try:
            self.client.get_collections()
            logger.info("✓ Qdrant health check passed")
            return True
        except Exception as e:
            logger.error(f"✗ Qdrant health check failed: {e}")
            return False


# Global instance
_qdrant_service = None


def get_qdrant_service() -> QdrantService:
    """Get or create Qdrant service instance."""
    global _qdrant_service
    if _qdrant_service is None:
        _qdrant_service = QdrantService()
    return _qdrant_service
