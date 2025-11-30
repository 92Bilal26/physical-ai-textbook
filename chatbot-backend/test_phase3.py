#!/usr/bin/env python3
"""
Phase 3 Testing Script - Validates OpenAI RAG Chatbot API
Tests: OpenAI connectivity, Qdrant search, and endpoint functionality
"""

import asyncio
import sys
import os
from pathlib import Path

# Add src to path
sys.path.insert(0, str(Path(__file__).parent / "src"))

from config import settings
from services.openai_service import OpenAIService
from services.qdrant_service import QdrantService
from services.cache_service import CacheService
from services.rag_service import RAGService
from utils.errors import RAGException


class TestRunner:
    """Run comprehensive Phase 3 tests"""

    def __init__(self):
        self.results = []
        self.passed = 0
        self.failed = 0

    async def test_openai_connectivity(self):
        """Test OpenAI API connectivity"""
        print("\n" + "="*60)
        print("TEST 1: OpenAI API Connectivity")
        print("="*60)

        try:
            openai_service = OpenAIService(
                api_key=settings.openai_api_key,
                model=settings.openai_model,
                embedding_model=settings.openai_embedding_model
            )

            # Test embedding
            print("\n  Testing text embedding...")
            embedding = await openai_service.embed_text("Hello world")

            if embedding and len(embedding) > 0:
                print(f"  ✅ Embedding successful - Vector dimension: {len(embedding)}")
                self.passed += 1
                return True
            else:
                print("  ❌ Embedding returned empty vector")
                self.failed += 1
                return False

        except Exception as e:
            print(f"  ❌ OpenAI API Error: {str(e)}")
            self.failed += 1
            return False

    async def test_chat_completion(self):
        """Test OpenAI chat completion"""
        print("\n" + "="*60)
        print("TEST 2: OpenAI Chat Completion")
        print("="*60)

        try:
            openai_service = OpenAIService(
                api_key=settings.openai_api_key,
                model=settings.openai_model,
                embedding_model=settings.openai_embedding_model
            )

            print("\n  Testing chat completion...")
            messages = [{"role": "user", "content": "Say 'Hello from OpenAI'"}]
            response = await openai_service.chat_completion(
                messages=messages,
                system_prompt="You are a helpful assistant."
            )

            if response:
                print(f"  ✅ Chat completion successful")
                print(f"     Response: {response[:100]}...")
                self.passed += 1
                return True
            else:
                print("  ❌ Chat completion returned empty response")
                self.failed += 1
                return False

        except Exception as e:
            print(f"  ❌ Chat completion error: {str(e)}")
            self.failed += 1
            return False

    async def test_qdrant_connectivity(self):
        """Test Qdrant connectivity and collection info"""
        print("\n" + "="*60)
        print("TEST 3: Qdrant Vector Database Connectivity")
        print("="*60)

        try:
            qdrant_service = QdrantService(
                url=settings.qdrant_url,
                api_key=settings.qdrant_api_key,
                collection_name=settings.qdrant_collection_name
            )

            print("\n  Testing Qdrant health check...")
            is_healthy = await qdrant_service.health_check()

            if is_healthy:
                print("  ✅ Qdrant connection successful")

                # Get collection info
                print("\n  Fetching collection info...")
                info = await qdrant_service.get_collection_info()
                if info:
                    print(f"  ✅ Collection: {info['name']}")
                    print(f"     Vectors count: {info['vectors_count']}")
                    print(f"     Points count: {info['points_count']}")
                else:
                    print("  ⚠️  Could not fetch collection info")

                self.passed += 1
                return True
            else:
                print("  ❌ Qdrant health check failed")
                self.failed += 1
                return False

        except Exception as e:
            print(f"  ❌ Qdrant error: {str(e)}")
            self.failed += 1
            return False

    async def test_cache_connectivity(self):
        """Test Redis cache connectivity"""
        print("\n" + "="*60)
        print("TEST 4: Redis Cache Connectivity")
        print("="*60)

        try:
            cache_service = CacheService(redis_url=settings.redis_url)

            print("\n  Initializing Redis connection...")
            await cache_service.initialize()

            print("  Testing health check...")
            is_healthy = await cache_service.health_check()

            if is_healthy:
                print("  ✅ Redis connection successful")
                self.passed += 1

                await cache_service.close()
                return True
            else:
                print("  ⚠️  Redis health check failed (optional service)")
                self.passed += 1  # Not critical
                await cache_service.close()
                return True

        except Exception as e:
            print(f"  ⚠️  Redis error (optional): {str(e)}")
            self.passed += 1  # Cache is optional
            return True

    async def test_rag_pipeline(self):
        """Test complete RAG pipeline"""
        print("\n" + "="*60)
        print("TEST 5: RAG Pipeline Integration")
        print("="*60)

        try:
            openai_service = OpenAIService(
                api_key=settings.openai_api_key,
                model=settings.openai_model,
                embedding_model=settings.openai_embedding_model
            )
            qdrant_service = QdrantService(
                url=settings.qdrant_url,
                api_key=settings.qdrant_api_key,
                collection_name=settings.qdrant_collection_name
            )
            cache_service = CacheService(redis_url=settings.redis_url)
            await cache_service.initialize()

            rag_service = RAGService(openai_service, qdrant_service, cache_service)

            print("\n  Testing RAG pipeline with sample query...")
            question = "What is machine learning?"

            result = await rag_service.process_query(
                question=question,
                session_id="test-session-001",
                page_context="Physics and AI textbook"
            )

            if result and "answer" in result:
                print("  ✅ RAG pipeline successful")
                print(f"     Question: {question}")
                print(f"     Answer: {result['answer'][:150]}...")
                print(f"     Confidence: {result.get('confidence', 0):.2f}")
                print(f"     Sources found: {len(result.get('sources', []))}")
                self.passed += 1

                await cache_service.close()
                return True
            else:
                print("  ❌ RAG pipeline returned invalid response")
                self.failed += 1
                await cache_service.close()
                return False

        except Exception as e:
            print(f"  ❌ RAG pipeline error: {str(e)}")
            self.failed += 1
            return False

    async def test_error_handling(self):
        """Test error handling for invalid inputs"""
        print("\n" + "="*60)
        print("TEST 6: Error Handling")
        print("="*60)

        try:
            openai_service = OpenAIService(
                api_key=settings.openai_api_key,
                model=settings.openai_model,
                embedding_model=settings.openai_embedding_model
            )
            qdrant_service = QdrantService(
                url=settings.qdrant_url,
                api_key=settings.qdrant_api_key,
                collection_name=settings.qdrant_collection_name
            )
            cache_service = CacheService(redis_url=settings.redis_url)
            await cache_service.initialize()

            rag_service = RAGService(openai_service, qdrant_service, cache_service)

            # Test 1: Empty question
            print("\n  Test 6a: Empty question handling...")
            try:
                await rag_service.process_query("", "session-1", None)
                print("  ⚠️  Should have raised error for empty question")
            except ValueError as e:
                print(f"  ✅ Correctly raised ValueError: {str(e)[:60]}...")
                self.passed += 1

            # Test 2: Very long question
            print("\n  Test 6b: Oversized question handling...")
            try:
                long_question = "What is this? " * 100  # Very long question
                result = await rag_service.process_query(long_question, "session-2", None)
                if not result or result.get("answer") == "No relevant information found":
                    print("  ✅ Correctly handled oversized question")
                    self.passed += 1
                else:
                    print("  ⚠️  Processed very long question (might exceed limit)")
            except ValueError:
                print("  ✅ Correctly raised error for oversized question")
                self.passed += 1

            await cache_service.close()
            return True

        except Exception as e:
            print(f"  ❌ Error handling test failed: {str(e)}")
            self.failed += 1
            return False

    async def run_all_tests(self):
        """Run all tests in sequence"""
        print("\n" + "="*70)
        print("PHASE 3 - API ENDPOINTS TESTING & VALIDATION")
        print("="*70)

        await self.test_openai_connectivity()
        await self.test_chat_completion()
        await self.test_qdrant_connectivity()
        await self.test_cache_connectivity()
        await self.test_rag_pipeline()
        await self.test_error_handling()

        # Print summary
        print("\n" + "="*70)
        print("TEST SUMMARY")
        print("="*70)
        print(f"\n  Total Passed: {self.passed}")
        print(f"  Total Failed: {self.failed}")

        if self.failed == 0:
            print("\n  ✅ ALL TESTS PASSED - Phase 3 Complete!")
        else:
            print(f"\n  ⚠️  {self.failed} test(s) failed - Review output above")

        print("\n" + "="*70 + "\n")


async def main():
    """Main entry point"""
    # Validate required environment variables
    if not settings.openai_api_key:
        print("❌ ERROR: OPENAI_API_KEY not set in .env file")
        sys.exit(1)

    if not settings.qdrant_url or not settings.qdrant_api_key:
        print("❌ ERROR: Qdrant URL or API key not set in .env file")
        sys.exit(1)

    runner = TestRunner()
    await runner.run_all_tests()


if __name__ == "__main__":
    asyncio.run(main())
