"""
LLM API client with Adapter pattern for multiple providers

Supports:
- OpenAI-compatible endpoints (OpenAI, Ollama, vLLM, etc.)
- Mock client for testing and development
- Future: Anthropic native, Gemini, VertexAI, etc.
"""

import asyncio
import logging
import random
import time
from abc import ABC, abstractmethod
from typing import Dict, List, Optional

from openai import AsyncOpenAI

from vlaarena_shared.config import settings

from .model_service import ModelConfig


logger = logging.getLogger(__name__)


class LLMResponse:
    """
    LLM API response wrapper

    Contains response text and metadata
    """

    def __init__(self, content: str, latency_ms: int, model_id: str):
        self.content = content
        self.latency_ms = latency_ms
        self.model_id = model_id


class LLMClientInterface(ABC):
    """
    Abstract interface for LLM clients

    Adapter pattern: allows multiple provider implementations
    (OpenAI-compatible, Anthropic native, Gemini, etc.)
    """

    @abstractmethod
    async def chat_completion(
        self,
        model_config: ModelConfig,
        messages: List[Dict[str, str]],
    ) -> LLMResponse:
        """
        Call LLM API with chat completion format

        Args:
            model_config: Model configuration
            messages: Conversation history
                [{"role": "user", "content": "Hello"}, ...]

        Returns:
            LLMResponse with content and latency

        Raises:
            Exception: If API call fails after retries
        """
        pass


class OpenAILLMClient(LLMClientInterface):
    """
    OpenAI-compatible LLM client using official OpenAI SDK

    Supports:
    - OpenAI API (https://api.openai.com/v1)
    - Ollama (http://localhost:11434/v1)
    - vLLM (custom endpoint)
    - Any endpoint exposing OpenAI-compatible /v1/chat/completions
    """

    def __init__(self):
        """
        Initialize OpenAI client

        Note: Client instance is created per request to support
        different base_url and api_key per model
        """
        self.timeout = settings.llm_read_timeout
        self.max_retries = settings.llm_retry_attempts

    async def chat_completion(
        self,
        model_config: ModelConfig,
        messages: List[Dict[str, str]],
    ) -> LLMResponse:
        """
        Call OpenAI-compatible API using official SDK

        Args:
            model_config: Model configuration
            messages: Conversation history in OpenAI format

        Returns:
            LLMResponse with content and latency

        Raises:
            Exception: If API call fails after retries
        """
        # Create client with model-specific config
        client = AsyncOpenAI(
            base_url=model_config.base_url,
            api_key=model_config.api_key or "dummy",  # Ollama doesn't need API key
            timeout=self.timeout,
            max_retries=self.max_retries,
        )

        start_time = time.time()

        try:
            response = await client.chat.completions.create(
                model=model_config.model,
                messages=messages,  # type: ignore
                temperature=0.7,
                max_tokens=1024,
            )

            latency_ms = int((time.time() - start_time) * 1000)

            content = response.choices[0].message.content or ""

            logger.info(f"LLM API call successful: model={model_config.id}, latency={latency_ms}ms")

            return LLMResponse(content=content, latency_ms=latency_ms, model_id=model_config.id)

        except Exception as e:
            latency_ms = int((time.time() - start_time) * 1000)
            error_msg = (
                f"LLM API call failed: model={model_config.id}, "
                f"latency={latency_ms}ms, error={str(e)}"
            )
            logger.error(error_msg)
            raise Exception(error_msg)


class MockLLMClient(LLMClientInterface):
    """
    Mock LLM client for testing and development

    Returns deterministic responses without external API calls.
    Simulates realistic latency (100-300ms) for testing.
    """

    async def chat_completion(
        self,
        model_config: ModelConfig,
        messages: List[Dict[str, str]],
    ) -> LLMResponse:
        """
        Return mock response based on model and prompt

        Args:
            model_config: Model configuration
            messages: Conversation history in OpenAI format

        Returns:
            LLMResponse with mock content and simulated latency
        """
        start_time = time.time()

        # Simulate API latency (100-300ms)
        latency_ms = random.randint(100, 300)
        await asyncio.sleep(latency_ms / 1000)

        # Extract last user message
        last_user_msg = next((m["content"] for m in reversed(messages) if m["role"] == "user"), "")

        # Generate deterministic mock response
        mock_content = (
            f"This is a mock response from {model_config.id}.\n\n"
            f"You asked: {last_user_msg[:100]}{'...' if len(last_user_msg) > 100 else ''}\n\n"
            f"In a real scenario, this would be an actual LLM response."
        )

        actual_latency_ms = int((time.time() - start_time) * 1000)

        logger.info(
            f"Mock LLM call successful: model={model_config.id}, "
            f"latency={actual_latency_ms}ms, message_count={len(messages)}"
        )

        return LLMResponse(
            content=mock_content, latency_ms=actual_latency_ms, model_id=model_config.id
        )


# Singleton instance (mutable for dependency injection)
_llm_client: Optional[LLMClientInterface] = None


def get_llm_client() -> LLMClientInterface:
    """
    Get LLM client instance (singleton)

    Factory function that returns appropriate client based on configuration.
    Can be overridden for testing via set_llm_client()

    Returns:
        LLMClientInterface implementation

    Future:
        # Based on config, return different implementations
        if settings.llm_provider == "anthropic":
            return AnthropicLLMClient()
        elif settings.llm_provider == "gemini":
            return GeminiLLMClient()
        else:
            return OpenAILLMClient()
    """
    global _llm_client
    if _llm_client is None:
        # Default to OpenAI-compatible client
        _llm_client = OpenAILLMClient()
    return _llm_client


def set_llm_client(client: LLMClientInterface) -> None:
    """
    Override LLM client instance (dependency injection)

    Allows injecting mock or alternative implementations for testing/development.

    Args:
        client: LLMClientInterface implementation to use

    Example:
        # In tests or dev mode
        set_llm_client(MockLLMClient())

        # In production
        set_llm_client(OpenAILLMClient())
    """
    global _llm_client
    _llm_client = client
    logger.info(f"LLM client set to: {client.__class__.__name__}")


def reset_llm_client() -> None:
    """
    Reset LLM client to None (forces re-initialization)

    Useful for testing to ensure clean state between test runs.
    """
    global _llm_client
    _llm_client = None
    logger.debug("LLM client reset")
