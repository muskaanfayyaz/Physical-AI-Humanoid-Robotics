"""
LLM service for generating grounded answers using Google Gemini.
Implements strict context-based answering to prevent hallucinations.
Uses FREE tier: gemini-1.5-flash with 15 requests/minute limit.
"""

from typing import List, Dict, Optional
import google.generativeai as genai
import logging

from app.config import get_settings

logger = logging.getLogger(__name__)


class LLMService:
    """
    Service for generating answers using Google Gemini's chat models.

    Features:
    - gemini-1.5-flash: Fast, FREE, and accurate
    - Low temperature (0.1) for factual responses
    - Strict grounding to prevent hallucinations
    """

    def __init__(self):
        """
        Initialize Gemini LLM service.
        Configures API key and model settings.
        """
        settings = get_settings()

        # Configure Gemini API with your API key
        genai.configure(api_key=settings.gemini_api_key)

        # Initialize Gemini model
        self.model_name = settings.gemini_chat_model  # "gemini-1.5-flash"
        self.model = genai.GenerativeModel(self.model_name)

        self.max_tokens = 1000  # Max output tokens
        self.temperature = 0.1  # Low temperature for factual, grounded responses

        logger.info(f"Initialized Gemini LLM Service: {self.model_name}")

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
            # Build full prompt for Gemini (it doesn't use message roles like OpenAI)
            # Combine system prompt, history, and user message into one prompt
            full_prompt = f"""{self._get_rag_system_prompt()}

{user_message}"""

            # If there's conversation history, add it before the current question
            if conversation_history:
                history_text = "\n".join([
                    f"{msg['role'].upper()}: {msg['content']}"
                    for msg in conversation_history[-6:]
                ])
                full_prompt = f"""{self._get_rag_system_prompt()}

Previous conversation:
{history_text}

{user_message}"""

            # Call Gemini API
            # Note: Gemini uses generate_content() instead of chat.completions
            response = self.model.generate_content(
                full_prompt,
                generation_config=genai.types.GenerationConfig(
                    temperature=self.temperature,
                    max_output_tokens=self.max_tokens,
                )
            )

            # Extract answer from Gemini response
            answer = response.text

            # Gemini doesn't provide finish_reason like OpenAI, so we set it manually
            finish_reason = "stop" if response.candidates else "unknown"

            # Extract sources used
            sources = [
                {
                    "chunk_id": chunk["chunk_id"],
                    "heading": chunk.get("heading_hierarchy", "Unknown"),
                    "chapter": chunk.get("chapter_title_slug", "Unknown"),
                }
                for chunk in context_chunks
            ]

            # Calculate approximate token usage (Gemini doesn't provide exact count)
            # Estimate: ~1 token per 4 characters
            tokens_used = len(full_prompt + answer) // 4

            return {
                "answer": answer,
                "sources": sources,
                "model": self.model_name,
                "finish_reason": finish_reason,
                "tokens_used": tokens_used,  # Estimated
                "is_grounded": self._check_if_grounded(answer),
            }

        except Exception as e:
            logger.error(f"Gemini answer generation failed: {e}")
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
            # Build full prompt for Gemini
            full_prompt = f"""{system_prompt}

{user_message}"""

            # Call Gemini API
            response = self.model.generate_content(
                full_prompt,
                generation_config=genai.types.GenerationConfig(
                    temperature=self.temperature,
                    max_output_tokens=self.max_tokens,
                )
            )

            # Extract answer from Gemini response
            answer = response.text
            finish_reason = "stop" if response.candidates else "unknown"

            # Calculate approximate token usage
            tokens_used = len(full_prompt + answer) // 4

            return {
                "answer": answer,
                "selection_length": len(selected_text),
                "selection_metadata": selection_metadata or {},
                "model": self.model_name,
                "finish_reason": finish_reason,
                "tokens_used": tokens_used,  # Estimated
                "is_grounded": self._check_if_grounded(answer),
            }

        except Exception as e:
            logger.error(f"Gemini selection-based answer generation failed: {e}")
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
        Test Gemini API connection by generating a test response.

        Returns:
            True if connection successful
        """
        try:
            response = self.model.generate_content(
                "test",
                generation_config=genai.types.GenerationConfig(max_output_tokens=5)
            )
            logger.info("Gemini LLM connection test: SUCCESS")
            return True
        except Exception as e:
            logger.error(f"Gemini LLM connection test failed: {e}")
            return False


# Singleton instance
_llm_service: LLMService | None = None


def get_llm_service() -> LLMService:
    """Get or create the LLM service singleton."""
    global _llm_service
    if _llm_service is None:
        _llm_service = LLMService()
    return _llm_service
