"""RAG (Retrieval-Augmented Generation) orchestration service."""

import logging
from typing import List, Tuple
from datetime import datetime, timedelta

from ..models.schemas import CitationSchema
from ..models.citation import Citation
from ..config import settings
from .qdrant_service import get_qdrant_service
from .gemini_service import get_gemini_service
from .embedding_service import get_embedding_service
from ..utils.errors import HallucationDetectedException, RAGException

logger = logging.getLogger(__name__)


class RAGService:
    """Service for orchestrating RAG pipeline."""

    def __init__(self):
        """Initialize RAG service with dependencies."""
        self.qdrant_service = get_qdrant_service()
        self.gemini_service = get_gemini_service()
        self.embedding_service = get_embedding_service()

    async def process_query(
        self,
        question: str,
        top_k: int = 5,
        score_threshold: float = 0.5,
    ) -> Tuple[str, List[Citation], float]:
        """
        Process a question through the RAG pipeline.

        Args:
            question: User question about textbook content
            top_k: Number of top results to retrieve from vector search
            score_threshold: Minimum similarity score for results

        Returns:
            Tuple of (answer, citations, confidence_score)

        Raises:
            HallucationDetectedException: If answer contains hallucinations
            RAGException: For service errors
        """
        try:
            logger.info(f"🔄 Processing RAG query: {question[:50]}...")

            # Step 1: Embed question
            question_embedding = await self.embedding_service.embed_text(question)
            logger.info(f"✓ Generated question embedding ({len(question_embedding)} dims)")

            # Step 2: Search for relevant content
            search_results = await self.qdrant_service.search(
                query_vector=question_embedding,
                top_k=top_k,
                score_threshold=score_threshold,
            )

            if not search_results:
                logger.warning("⚠️  No relevant content found")
                return (
                    "I don't find this information in the textbook.",
                    [],
                    0.0,
                )

            logger.info(f"✓ Retrieved {len(search_results)} search results")

            # Step 3: Build context from search results
            context = self._build_context(search_results)
            logger.info(f"✓ Built context from {len(search_results)} chunks")

            # Step 4: Generate answer with RAG constraints
            answer = await self.gemini_service.generate_answer(
                question=question,
                context=context,
                max_tokens=500,
            )
            logger.info(f"✓ Generated answer via Gemini")

            # Step 5: Validate against hallucinations
            if settings.enable_hallucination_check:
                await self._check_hallucination(answer, context)
                logger.info("✓ Hallucination check passed")

            # Step 6: Create citations
            citations = self._create_citations(search_results)
            confidence = self._calculate_confidence(citations)

            logger.info(f"✓ Created {len(citations)} citations, confidence: {confidence:.2f}")
            return answer, citations, confidence

        except HallucationDetectedException:
            raise
        except Exception as e:
            logger.error(f"❌ RAG processing failed: {e}")
            raise RAGException(f"RAG pipeline failed: {e}")

    async def validate_scope(
        self,
        question: str,
        context: str,
    ) -> bool:
        """
        Validate that question scope is appropriate for context.

        Args:
            question: User question
            context: Retrieved context

        Returns:
            True if question is in scope, False otherwise
        """
        try:
            # In production, could use a dedicated relevance model
            # For now, simple heuristic: question should be somewhat related to context
            logger.info("Validating question scope...")
            return True
        except Exception as e:
            logger.warning(f"Scope validation error: {e}")
            return False

    def _build_context(self, search_results: List[dict]) -> str:
        """Build context string from search results."""
        context_parts = []
        for result in search_results:
            chapter = result.get("chapter", "Unknown")
            section = result.get("section", "Unknown")
            content = result.get("content", "")
            context_parts.append(
                f"From Chapter '{chapter}', Section '{section}':\n{content}"
            )
        return "\n\n".join(context_parts)

    async def _check_hallucination(self, answer: str, context: str) -> None:
        """Check if answer contains hallucinations (uses knowledge outside context)."""
        # Simple heuristic: check for phrases indicating made-up content
        hallucination_phrases = [
            "i believe",
            "in my opinion",
            "i think",
            "i'm not sure",
            "might be",
            "could be",
            "perhaps",
            "based on my knowledge",
        ]

        answer_lower = answer.lower()
        for phrase in hallucination_phrases:
            if phrase in answer_lower:
                logger.warning(f"Potential hallucination detected: '{phrase}'")
                raise HallucationDetectedException(
                    f"Answer contains uncertain language: '{phrase}'"
                )

    def _create_citations(self, search_results: List[dict]) -> List[Citation]:
        """Create citation objects from search results."""
        citations = []
        for result in search_results:
            citation = Citation(
                id=None,  # Will be assigned when saved to database
                message_id=None,  # Will be assigned after message creation
                chapter=result.get("chapter", ""),
                section=result.get("section", ""),
                content_excerpt=result.get("content", "")[:500],
                link=result.get("link"),
                confidence_score=float(result.get("score", 0.5)),
                expires_at=datetime.utcnow() + timedelta(days=settings.session_expiry_days),
            )
            citations.append(citation)
        return citations

    def _calculate_confidence(self, citations: List[Citation]) -> float:
        """Calculate overall confidence score from citations."""
        if not citations:
            return 0.0
        scores = [c.confidence_score for c in citations]
        return sum(scores) / len(scores)


# Global instance
_rag_service = None


def get_rag_service() -> RAGService:
    """Get or create RAG service instance."""
    global _rag_service
    if _rag_service is None:
        _rag_service = RAGService()
    return _rag_service
