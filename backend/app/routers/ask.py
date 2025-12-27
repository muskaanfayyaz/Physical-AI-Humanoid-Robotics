"""
Ask endpoints for LLM-powered question answering.
Implements grounded RAG and selection-based Q&A.
"""

import time
from typing import List
from fastapi import APIRouter, Depends, HTTPException
from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy import select

from app.database import get_db, ConversationHistory
from app.schemas import (
    AskRequest,
    AskResponse,
    AskSelectedRequest,
    AskSelectedResponse,
    SourceReference,
)
from app.services.embeddings import get_embedding_service
from app.services.qdrant_service import get_qdrant_service
from app.services.llm_service import get_llm_service

router = APIRouter(prefix="/ask", tags=["Question Answering"])


@router.post("/", response_model=AskResponse)
async def ask_question(
    request: AskRequest,
    session: AsyncSession = Depends(get_db),
    embedding_service=Depends(get_embedding_service),
    qdrant_service=Depends(get_qdrant_service),
    llm_service=Depends(get_llm_service),
):
    """
    Ask a question and get a grounded answer using RAG.

    This endpoint:
    1. Generates embedding for the query
    2. Retrieves top-k relevant chunks from Qdrant
    3. Sends ONLY retrieved chunks to the LLM
    4. Returns a grounded answer that refuses to hallucinate

    The LLM is strictly instructed to:
    - Answer ONLY using the provided context
    - Say "I cannot answer" if information is not in the context
    - NOT use external knowledge or assumptions

    Args:
        request: AskRequest with query, filters, and options

    Returns:
        AskResponse with grounded answer and source references
    """
    start_time = time.time()

    try:
        # Step 1: Generate query embedding
        query_vector = await embedding_service.generate_embedding(request.query)

        # Step 2: Retrieve relevant chunks from Qdrant
        qdrant_results = await qdrant_service.search(
            query_vector=query_vector,
            top_k=request.top_k,
            filters=request.filters,
            score_threshold=request.similarity_threshold,
        )

        if not qdrant_results:
            # No relevant chunks found
            processing_time = time.time() - start_time
            return AskResponse(
                query=request.query,
                answer="I cannot find any relevant information in the textbook to answer this question. Please try rephrasing your question or asking about a different topic covered in the Physical AI & Humanoid Robotics textbook.",
                sources=[],
                model=llm_service.model,
                tokens_used=0,
                is_grounded=True,
                processing_time_seconds=round(processing_time, 2),
                chunks_retrieved=0,
            )

        # Step 3: Get conversation history if requested
        conversation_history = None
        if request.include_history and request.session_id:
            history_result = await session.execute(
                select(ConversationHistory)
                .where(ConversationHistory.session_id == request.session_id)
                .order_by(ConversationHistory.created_at.desc())
                .limit(6)  # Last 3 turns
            )
            history_messages = history_result.scalars().all()
            conversation_history = [
                {"role": msg.role, "content": msg.message}
                for msg in reversed(history_messages)
            ]

        # Step 4: Prepare context chunks for LLM
        context_chunks = [
            {
                "chunk_id": result["chunk_id"],
                "content": result["payload"].get("content", ""),
                "heading_hierarchy": result["payload"].get("heading_hierarchy", "Unknown"),
                "chapter_title_slug": result["payload"].get("chapter_title_slug", "Unknown"),
            }
            for result in qdrant_results
        ]

        # Step 5: Generate grounded answer using LLM
        llm_result = await llm_service.answer_with_context(
            query=request.query,
            context_chunks=context_chunks,
            conversation_history=conversation_history,
        )

        # Step 6: Store conversation if session_id provided
        if request.session_id:
            # Store user message
            user_msg = ConversationHistory(
                session_id=request.session_id,
                role="user",
                message=request.query,
                chunk_references=[chunk["chunk_id"] for chunk in context_chunks],
            )
            session.add(user_msg)

            # Store assistant response
            assistant_msg = ConversationHistory(
                session_id=request.session_id,
                role="assistant",
                message=llm_result["answer"],
                chunk_references=[chunk["chunk_id"] for chunk in context_chunks],
            )
            session.add(assistant_msg)
            await session.commit()

        processing_time = time.time() - start_time

        return AskResponse(
            query=request.query,
            answer=llm_result["answer"],
            sources=[
                SourceReference(**source) for source in llm_result["sources"]
            ],
            model=llm_result["model"],
            tokens_used=llm_result["tokens_used"],
            is_grounded=llm_result["is_grounded"],
            processing_time_seconds=round(processing_time, 2),
            chunks_retrieved=len(context_chunks),
        )

    except Exception as e:
        raise HTTPException(
            status_code=500, detail=f"Question answering failed: {str(e)}"
        )


@router.post("/selected", response_model=AskSelectedResponse)
async def ask_from_selection(
    request: AskSelectedRequest,
    llm_service=Depends(get_llm_service),
):
    """
    Ask a question about user-selected text.

    This endpoint:
    1. Receives user's question and selected text
    2. Sends ONLY the selected text to the LLM
    3. Returns answer based ONLY on that text
    4. NO retrieval from database
    5. Says "Not found in selection" if answer not present

    The LLM is strictly instructed to:
    - Answer ONLY using the selected text
    - Say "I cannot find the answer in the selected text" if not found
    - NOT use external knowledge, even if it knows the answer
    - NOT make assumptions beyond the text

    This is useful for:
    - Deep dive into specific sections
    - Understanding particular paragraphs
    - Clarifying confusing passages
    - Quick fact-checking within a section

    Args:
        request: AskSelectedRequest with query and selected text

    Returns:
        AskSelectedResponse with grounded answer
    """
    start_time = time.time()

    try:
        # Generate answer using ONLY the selected text
        llm_result = await llm_service.answer_from_selection(
            query=request.query,
            selected_text=request.selected_text,
            selection_metadata=request.selection_metadata,
        )

        processing_time = time.time() - start_time

        return AskSelectedResponse(
            query=request.query,
            answer=llm_result["answer"],
            selection_length=llm_result["selection_length"],
            selection_metadata=llm_result["selection_metadata"],
            model=llm_result["model"],
            tokens_used=llm_result["tokens_used"],
            is_grounded=llm_result["is_grounded"],
            processing_time_seconds=round(processing_time, 2),
        )

    except Exception as e:
        raise HTTPException(
            status_code=500,
            detail=f"Selection-based question answering failed: {str(e)}",
        )
