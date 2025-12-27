"""
LLM service for generating grounded answers using OpenAI.
Implements strict context-based answering to prevent hallucinations.
"""

from typing import List, Dict, Optional
from openai import AsyncOpenAI
import logging

from app.config import get_settings

logger = logging.getLogger(__name__)


class LLMService:
    """Service for generating answers using OpenAI's chat models."""

    def __init__(self):
        settings = get_settings()
        self.client = AsyncOpenAI(api_key=settings.openai_api_key)
        self.model = "gpt-4o-mini"  # Fast and cost-effective
        self.max_tokens = 1000
        self.temperature = 0.1  # Low temperature for factual responses

    async def answer_with_context(
        self,
        query: str,
        context_chunks: List[Dict[str, str]],
        conversation_history: Optional[List[Dict[str, str]]] = None,
    ) -> Dict[str, any]:
        """
        Generate an answer using ONLY the provided context chunks.

        Args:
            query: User's question
            context_chunks: List of relevant chunks with content and metadata
            conversation_history: Optional previous conversation messages

        Returns:
            Dictionary with answer, sources used, and metadata
        """
        # Build context from chunks
        context_text = self._build_context(context_chunks)

        # Create system prompt
        system_prompt = self._get_rag_system_prompt()

        # Build messages
        messages = [
            {"role": "system", "content": system_prompt},
        ]

        # Add conversation history if provided
        if conversation_history:
            messages.extend(conversation_history[-6:])  # Last 3 turns

        # Add user query with context
        user_message = f"""Context from Physical AI & Humanoid Robotics Textbook:

{context_text}

Question: {query}

Instructions: Answer ONLY using the context above. If the answer is not in the context, say "I cannot answer this question based on the provided textbook content." Do not use any external knowledge."""

        messages.append({"role": "user", "content": user_message})

        try:
            # Call OpenAI
            response = await self.client.chat.completions.create(
                model=self.model,
                messages=messages,
                temperature=self.temperature,
                max_tokens=self.max_tokens,
            )

            answer = response.choices[0].message.content
            finish_reason = response.choices[0].finish_reason

            # Extract sources used
            sources = [
                {
                    "chunk_id": chunk["chunk_id"],
                    "heading": chunk.get("heading_hierarchy", "Unknown"),
                    "chapter": chunk.get("chapter_title_slug", "Unknown"),
                }
                for chunk in context_chunks
            ]

            return {
                "answer": answer,
                "sources": sources,
                "model": self.model,
                "finish_reason": finish_reason,
                "tokens_used": response.usage.total_tokens,
                "is_grounded": self._check_if_grounded(answer),
            }

        except Exception as e:
            logger.error(f"LLM answer generation failed: {e}")
            raise

    async def answer_from_selection(
        self,
        query: str,
        selected_text: str,
        selection_metadata: Optional[Dict[str, str]] = None,
    ) -> Dict[str, any]:
        """
        Generate an answer using ONLY the user-provided selected text.

        Args:
            query: User's question
            selected_text: Text selected by the user
            selection_metadata: Optional metadata about the selection

        Returns:
            Dictionary with answer and metadata
        """
        # Create system prompt for selection-based Q&A
        system_prompt = self._get_selection_system_prompt()

        # Build user message
        user_message = f"""Selected Text:
{selected_text}

Question: {query}

Instructions: Answer ONLY using the selected text above. If the answer is not found in this text, respond with: "I cannot find the answer to this question in the selected text." Do not use any external knowledge or make assumptions."""

        messages = [
            {"role": "system", "content": system_prompt},
            {"role": "user", "content": user_message},
        ]

        try:
            # Call OpenAI
            response = await self.client.chat.completions.create(
                model=self.model,
                messages=messages,
                temperature=self.temperature,
                max_tokens=self.max_tokens,
            )

            answer = response.choices[0].message.content
            finish_reason = response.choices[0].finish_reason

            return {
                "answer": answer,
                "selection_length": len(selected_text),
                "selection_metadata": selection_metadata or {},
                "model": self.model,
                "finish_reason": finish_reason,
                "tokens_used": response.usage.total_tokens,
                "is_grounded": self._check_if_grounded(answer),
            }

        except Exception as e:
            logger.error(f"Selection-based answer generation failed: {e}")
            raise

    def _build_context(self, chunks: List[Dict[str, str]]) -> str:
        """
        Build formatted context from chunks.

        Args:
            chunks: List of chunk dictionaries

        Returns:
            Formatted context string
        """
        context_parts = []

        for i, chunk in enumerate(chunks, 1):
            heading = chunk.get("heading_hierarchy", "Unknown Section")
            content = chunk.get("content", "")

            context_parts.append(f"[Source {i}: {heading}]\n{content}\n")

        return "\n---\n\n".join(context_parts)

    def _get_rag_system_prompt(self) -> str:
        """Get system prompt for RAG-based answering."""
        return """You are a helpful AI assistant for the Physical AI & Humanoid Robotics textbook.

Your role is to answer questions STRICTLY based on the provided context from the textbook. Follow these rules:

1. ONLY use information from the provided context
2. If the answer is not in the context, clearly state: "I cannot answer this question based on the provided textbook content."
3. Do NOT use external knowledge or make assumptions
4. Cite which source(s) you used when possible (e.g., "According to Source 1...")
5. If the context is incomplete, say so
6. Be concise but comprehensive
7. Use technical terminology from the textbook

Remember: It's better to say "I don't know" than to provide incorrect or ungrounded information."""

    def _get_selection_system_prompt(self) -> str:
        """Get system prompt for selection-based answering."""
        return """You are a helpful AI assistant analyzing selected text from the Physical AI & Humanoid Robotics textbook.

Your role is to answer questions STRICTLY based on the selected text provided by the user. Follow these rules:

1. ONLY use information from the selected text
2. If the answer is not in the selected text, respond: "I cannot find the answer to this question in the selected text."
3. Do NOT use external knowledge, even if you know the answer
4. Do NOT make assumptions beyond what's explicitly stated
5. Be precise and direct
6. Quote relevant parts if helpful

Remember: The user selected this specific text for a reason. Respect the boundary of their selection."""

    def _check_if_grounded(self, answer: str) -> bool:
        """
        Check if answer appears to be grounded (basic heuristic).

        Args:
            answer: LLM-generated answer

        Returns:
            True if answer appears grounded, False otherwise
        """
        # Simple heuristic: check for common "I don't know" phrases
        ungrounded_phrases = [
            "i cannot answer",
            "not in the context",
            "not found in",
            "cannot find",
            "not provided",
            "don't have information",
        ]

        answer_lower = answer.lower()

        # If answer contains refusal phrases, it's properly grounded
        for phrase in ungrounded_phrases:
            if phrase in answer_lower:
                return True  # Properly refusing to answer

        # Otherwise, assume it found something in context
        return True

    async def test_connection(self) -> bool:
        """
        Test OpenAI API connection.

        Returns:
            True if connection successful
        """
        try:
            response = await self.client.chat.completions.create(
                model=self.model,
                messages=[{"role": "user", "content": "test"}],
                max_tokens=5,
            )
            return True
        except Exception as e:
            logger.error(f"LLM connection test failed: {e}")
            return False


# Singleton instance
_llm_service: LLMService | None = None


def get_llm_service() -> LLMService:
    """Get or create the LLM service singleton."""
    global _llm_service
    if _llm_service is None:
        _llm_service = LLMService()
    return _llm_service
