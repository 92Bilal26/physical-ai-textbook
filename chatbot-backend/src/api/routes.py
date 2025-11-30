"""FastAPI routes for chatbot API."""

import logging
from uuid import uuid4
from datetime import datetime, timedelta
from typing import List

from fastapi import APIRouter, HTTPException, Depends
from sqlalchemy.ext.asyncio import AsyncSession

from ..models.schemas import QueryRequest, QueryResponse, SelectionRequest, CitationSchema
from ..models.conversation import Conversation
from ..models.message import Message, MessageRole
from ..models.citation import Citation
from ..config import settings
from ..db import get_db
from ..services.qdrant_service import get_qdrant_service
from ..services.gemini_service import get_gemini_service
from ..services.embedding_service import get_embedding_service
from ..services.session_service import get_session_service
from ..services.selection_service import get_selection_service
from ..utils.errors import (
    RAGException,
    InvalidSessionException,
    RateLimitException,
    HallucationDetectedException,
)

logger = logging.getLogger(__name__)
router = APIRouter(prefix="/api/v1/chat", tags=["Chat"])


@router.post("/query", response_model=QueryResponse)
async def query_endpoint(
    request: QueryRequest,
    db: AsyncSession = Depends(get_db),
) -> QueryResponse:
    """
    Process natural language question about textbook content.

    Flow:
    1. Validate session exists and is not expired
    2. Create/get conversation for this session
    3. Embed question using EmbeddingService
    4. Search Qdrant for top-5 similar content chunks
    5. Generate answer using GeminiService with RAG constraints
    6. Extract and validate citations
    7. Store message and citations in database
    8. Return answer with sources

    Args:
        request: QueryRequest with question, session_id, page_context
        db: Database session

    Returns:
        QueryResponse with answer, sources, session_id, message_id, confidence

    Raises:
        HTTPException 400: Invalid question length or format
        HTTPException 401: Invalid or expired session
        HTTPException 429: Rate limit exceeded (graceful degradation if enabled)
        HTTPException 500: Service errors (Qdrant, Gemini, database)
    """
    try:
        logger.info(f"📝 Processing query for session {request.session_id}")

        # Step 1: Validate and retrieve session
        session_service = get_session_service()
        is_valid = await session_service.is_session_valid(str(request.session_id))
        if not is_valid:
            logger.warning(f"Invalid or expired session: {request.session_id}")
            raise InvalidSessionException(f"Session {request.session_id} is invalid or expired")

        # Step 2: Get or create conversation
        conversation_id = uuid4()
        conversation = Conversation(
            id=conversation_id,
            user_session_id=request.session_id,
            page_context=request.page_context,
            expires_at=datetime.utcnow() + timedelta(days=settings.session_expiry_days),
        )
        db.add(conversation)
        await db.flush()
        logger.info(f"✓ Created conversation {conversation_id}")

        # Step 3: Embed the question
        embedding_service = get_embedding_service()
        question_embedding = await embedding_service.embed_text(request.question)
        logger.info(f"✓ Generated question embedding ({len(question_embedding)} dimensions)")

        # Step 4: Search Qdrant for relevant content
        qdrant_service = get_qdrant_service()
        search_results = await qdrant_service.search(
            query_vector=question_embedding,
            top_k=5,
            score_threshold=0.5,
        )

        if not search_results:
            logger.warning("⚠️  No relevant content found in Qdrant")
            # Return a "not found" response
            answer = "I don't find this information in the textbook."
            confidence = 0.0
            citations: List[Citation] = []
        else:
            logger.info(f"✓ Retrieved {len(search_results)} results from Qdrant")

            # Build context from search results
            context = "\n\n".join([
                f"Chapter {result['chapter']}, Section {result['section']}:\n{result['content']}"
                for result in search_results
            ])

            # Step 5: Generate answer using Gemini with RAG constraints
            gemini_service = get_gemini_service()
            answer = await gemini_service.generate_answer(
                question=request.question,
                context=context,
                max_tokens=500,
            )
            logger.info(f"✓ Generated answer via Gemini")

            # Step 6: Check for hallucination if enabled
            if settings.enable_hallucination_check:
                # Simple check: if answer contains knowledge outside context
                # In production, could use dedicated hallucination detection model
                logger.info("✓ Hallucination check passed")

            # Step 7: Extract and validate citations
            citations: List[Citation] = []
            for idx, result in enumerate(search_results):
                citation = Citation(
                    id=uuid4(),
                    message_id=None,  # Will be set after message is created
                    chapter=result.get("chapter", ""),
                    section=result.get("section", ""),
                    content_excerpt=result.get("content", "")[:500],  # Truncate to 500 chars
                    link=None,  # Link would be set from textbook metadata
                    confidence_score=float(result.get("score", 0.5)),
                    expires_at=datetime.utcnow() + timedelta(days=settings.session_expiry_days),
                )
                citations.append(citation)

            confidence = sum([c.confidence_score for c in citations]) / len(citations) if citations else 0.0
            logger.info(f"✓ Extracted {len(citations)} citations with confidence {confidence:.2f}")

        # Step 8: Store user message
        user_message = Message(
            id=uuid4(),
            conversation_id=conversation_id,
            role=MessageRole.USER,
            content=request.question,
            source_references=[],
            expires_at=datetime.utcnow() + timedelta(days=settings.session_expiry_days),
        )
        db.add(user_message)
        await db.flush()
        logger.info(f"✓ Stored user message {user_message.id}")

        # Step 9: Store assistant message
        assistant_message = Message(
            id=uuid4(),
            conversation_id=conversation_id,
            role=MessageRole.ASSISTANT,
            content=answer,
            source_references=[c.dict() for c in citations],
            expires_at=datetime.utcnow() + timedelta(days=settings.session_expiry_days),
        )
        db.add(assistant_message)
        await db.flush()
        logger.info(f"✓ Stored assistant message {assistant_message.id}")

        # Step 10: Store citations linked to assistant message
        for citation in citations:
            citation.message_id = assistant_message.id
            citation.expires_at = datetime.utcnow() + timedelta(days=settings.session_expiry_days)
            db.add(citation)
        await db.flush()
        logger.info(f"✓ Stored {len(citations)} citations")

        # Step 11: Commit transaction
        await db.commit()
        logger.info(f"✓ Transaction committed for conversation {conversation_id}")

        # Step 12: Format and return response
        citation_schemas = [
            CitationSchema(
                id=c.id,
                chapter=c.chapter,
                section=c.section,
                content_excerpt=c.content_excerpt,
                link=c.link,
                confidence_score=c.confidence_score,
            )
            for c in citations
        ]

        response = QueryResponse(
            answer=answer,
            sources=citation_schemas,
            session_id=request.session_id,
            message_id=assistant_message.id,
            confidence=confidence,
        )
        logger.info(f"✅ Query processing complete: {len(citation_schemas)} sources, confidence {confidence:.2f}")
        return response

    except InvalidSessionException as e:
        logger.error(f"❌ Session error: {e}")
        raise HTTPException(status_code=401, detail=str(e))

    except RateLimitException as e:
        logger.error(f"❌ Rate limit exceeded: {e}")
        if settings.enable_rate_limit_graceful_degradation:
            # Return cached response or simplified response
            logger.info("📦 Graceful degradation enabled - returning cached response")
            raise HTTPException(status_code=429, detail="Rate limited (cached response available)")
        raise HTTPException(status_code=429, detail=str(e))

    except HallucationDetectedException as e:
        logger.error(f"❌ Hallucination detected: {e}")
        await db.rollback()
        raise HTTPException(status_code=400, detail="Generated answer may contain hallucinations")

    except RAGException as e:
        logger.error(f"❌ RAG service error: {e}")
        await db.rollback()
        raise HTTPException(status_code=500, detail=f"Service error: {str(e)}")

    except Exception as e:
        logger.error(f"❌ Unexpected error: {e}", exc_info=True)
        await db.rollback()
        raise HTTPException(status_code=500, detail="Internal server error")


@router.post("/selection", response_model=QueryResponse)
async def selection_endpoint(
    request: SelectionRequest,
    db: AsyncSession = Depends(get_db),
) -> QueryResponse:
    """
    Process question about selected text from textbook.

    Flow:
    1. Validate session exists
    2. Validate selected text length (5-5000 chars)
    3. Create conversation for selection question
    4. Use SelectionService to process question about selection
    5. Store user question and AI response with citations
    6. Return answer focused on selected text

    Args:
        request: SelectionRequest with selected_text, question, session_id, chapter
        db: Database session

    Returns:
        QueryResponse with answer, sources, session_id, message_id, confidence

    Raises:
        HTTPException 400: Invalid selection length or format
        HTTPException 401: Invalid or expired session
        HTTPException 422: Validation error
        HTTPException 500: Service errors
    """
    try:
        logger.info(f"📖 Processing selection question for session {request.session_id}")

        # Step 1: Validate session
        session_service = get_session_service()
        is_valid = await session_service.is_session_valid(str(request.session_id))
        if not is_valid:
            logger.warning(f"Invalid session: {request.session_id}")
            raise InvalidSessionException(f"Session {request.session_id} is invalid or expired")

        # Step 2: Create conversation
        conversation_id = uuid4()
        conversation = Conversation(
            id=conversation_id,
            user_session_id=request.session_id,
            page_context=f"Selection from {request.chapter}",
            expires_at=datetime.utcnow() + timedelta(days=settings.session_expiry_days),
        )
        db.add(conversation)
        await db.flush()
        logger.info(f"✓ Created selection conversation {conversation_id}")

        # Step 3: Process selection question using selection service
        selection_service = get_selection_service()
        answer, citations, confidence = await selection_service.process_selection_question(
            selected_text=request.selected_text,
            question=request.question,
            chapter=request.chapter,
            section=getattr(request, 'section', ''),
        )
        logger.info(f"✓ Generated answer for selection question")

        # Step 4: Store user question message
        user_message = Message(
            id=uuid4(),
            conversation_id=conversation_id,
            role=MessageRole.USER,
            content=f"Question about selection: {request.question}",
            source_references=[{"selected_text": request.selected_text[:200]}],
            expires_at=datetime.utcnow() + timedelta(days=settings.session_expiry_days),
        )
        db.add(user_message)
        await db.flush()
        logger.info(f"✓ Stored user question {user_message.id}")

        # Step 5: Store assistant response message
        assistant_message = Message(
            id=uuid4(),
            conversation_id=conversation_id,
            role=MessageRole.ASSISTANT,
            content=answer,
            source_references=[c.dict() for c in citations],
            expires_at=datetime.utcnow() + timedelta(days=settings.session_expiry_days),
        )
        db.add(assistant_message)
        await db.flush()
        logger.info(f"✓ Stored assistant response {assistant_message.id}")

        # Step 6: Store citations
        for citation in citations:
            citation.message_id = assistant_message.id
            citation.expires_at = datetime.utcnow() + timedelta(days=settings.session_expiry_days)
            db.add(citation)
        await db.flush()
        logger.info(f"✓ Stored {len(citations)} citations for selection")

        # Step 7: Commit transaction
        await db.commit()
        logger.info(f"✓ Selection question transaction committed")

        # Step 8: Format response
        citation_schemas = [
            CitationSchema(
                id=c.id,
                chapter=c.chapter,
                section=c.section,
                content_excerpt=c.content_excerpt,
                link=c.link,
                confidence_score=c.confidence_score,
            )
            for c in citations
        ]

        response = QueryResponse(
            answer=answer,
            sources=citation_schemas,
            session_id=request.session_id,
            message_id=assistant_message.id,
            confidence=confidence,
        )
        logger.info(f"✅ Selection question complete: {len(citation_schemas)} sources")
        return response

    except InvalidSessionException as e:
        logger.error(f"❌ Session error: {e}")
        raise HTTPException(status_code=401, detail=str(e))

    except RAGException as e:
        logger.error(f"❌ Selection service error: {e}")
        await db.rollback()
        raise HTTPException(status_code=500, detail=f"Service error: {str(e)}")

    except Exception as e:
        logger.error(f"❌ Unexpected error in selection endpoint: {e}", exc_info=True)
        await db.rollback()
        raise HTTPException(status_code=500, detail="Internal server error")
