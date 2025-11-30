"""FastAPI application entry point."""

from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
import logging

from .config import settings
from .models.schemas import HealthCheckResponse
from .services.qdrant_service import get_qdrant_service
from .services.gemini_service import get_gemini_service

# Configure logging
logging.basicConfig(level=settings.log_level)
logger = logging.getLogger(__name__)

# Create FastAPI app
app = FastAPI(
    title="RAG Chatbot API",
    description="Retrieval-Augmented Generation chatbot for Physical AI Textbook",
    version="0.1.0",
)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=settings.allowed_origins,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


@app.on_event("startup")
async def startup_event():
    """Initialize services on startup."""
    logger.info("🚀 Starting RAG Chatbot API...")
    logger.info(f"Environment: {settings.environment}")
    logger.info(f"Debug: {settings.debug}")


@app.on_event("shutdown")
async def shutdown_event():
    """Cleanup on shutdown."""
    logger.info("🛑 Shutting down RAG Chatbot API...")


@app.get("/", tags=["Health"])
async def root():
    """Root endpoint."""
    return {"message": "RAG Chatbot API", "version": "0.1.0"}


@app.get("/api/v1/health", response_model=HealthCheckResponse, tags=["Health"])
async def health_check():
    """Health check endpoint."""
    # Check Qdrant
    qdrant_service = get_qdrant_service()
    qdrant_ok = await qdrant_service.health_check()

    # Check Gemini
    gemini_service = get_gemini_service()
    gemini_ok = await gemini_service.health_check()

    # Determine overall status
    if qdrant_ok and gemini_ok:
        status = "ok"
    elif qdrant_ok or gemini_ok:
        status = "degraded"
    else:
        status = "down"

    return HealthCheckResponse(
        status=status,
        version="0.1.0",
        qdrant_status="healthy" if qdrant_ok else "down",
        neon_status="healthy",  # Would check in production
        gemini_status="healthy" if gemini_ok else "down",
    )


@app.get("/api/v1/docs", tags=["Documentation"])
async def get_docs():
    """Get API documentation."""
    return {
        "title": "RAG Chatbot API Documentation",
        "version": "0.1.0",
        "endpoints": {
            "POST /api/v1/chat/query": "Send question about textbook content",
            "POST /api/v1/chat/selection": "Ask question about selected text",
            "GET /api/v1/chat/history": "Retrieve conversation history",
            "GET /api/v1/health": "Service health status",
        },
    }


if __name__ == "__main__":
    import uvicorn

    uvicorn.run(
        "src.main:app",
        host="0.0.0.0",
        port=8000,
        reload=settings.debug,
    )
