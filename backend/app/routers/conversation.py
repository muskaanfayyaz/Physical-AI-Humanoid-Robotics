"""
Conversation endpoints for managing chat history.
"""

from typing import List
from fastapi import APIRouter, Depends, HTTPException
from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy import select

from app.database import get_db, ConversationHistory
from app.schemas import ConversationMessageCreate, ConversationMessageResponse

router = APIRouter(prefix="/conversation", tags=["Conversation"])


@router.post("/message", response_model=ConversationMessageResponse)
async def create_message(
    message: ConversationMessageCreate,
    session: AsyncSession = Depends(get_db),
):
    """
    Store a conversation message.

    Args:
        message: Message to store (user or assistant)

    Returns:
        Stored message with ID and timestamp
    """
    try:
        conversation = ConversationHistory(
            session_id=message.session_id,
            role=message.role,
            message=message.message,
            chunk_references=message.chunk_references,
        )

        session.add(conversation)
        await session.commit()
        await session.refresh(conversation)

        return ConversationMessageResponse.model_validate(conversation)

    except Exception as e:
        raise HTTPException(
            status_code=500, detail=f"Failed to store message: {str(e)}"
        )


@router.get("/session/{session_id}", response_model=List[ConversationMessageResponse])
async def get_conversation_history(
    session_id: str,
    limit: int = 50,
    session: AsyncSession = Depends(get_db),
):
    """
    Retrieve conversation history for a session.

    Args:
        session_id: Session identifier
        limit: Maximum number of messages to return (default: 50)

    Returns:
        List of conversation messages in chronological order
    """
    try:
        result = await session.execute(
            select(ConversationHistory)
            .where(ConversationHistory.session_id == session_id)
            .order_by(ConversationHistory.created_at)
            .limit(limit)
        )

        conversations = result.scalars().all()

        return [
            ConversationMessageResponse.model_validate(conv) for conv in conversations
        ]

    except Exception as e:
        raise HTTPException(
            status_code=500, detail=f"Failed to retrieve history: {str(e)}"
        )


@router.delete("/session/{session_id}")
async def delete_conversation_session(
    session_id: str,
    session: AsyncSession = Depends(get_db),
):
    """
    Delete all messages for a conversation session.

    Args:
        session_id: Session identifier

    Returns:
        Success message with count of deleted messages
    """
    try:
        result = await session.execute(
            select(ConversationHistory).where(
                ConversationHistory.session_id == session_id
            )
        )

        conversations = result.scalars().all()
        count = len(conversations)

        for conversation in conversations:
            await session.delete(conversation)

        await session.commit()

        return {"message": f"Deleted {count} messages for session {session_id}"}

    except Exception as e:
        raise HTTPException(
            status_code=500, detail=f"Failed to delete session: {str(e)}"
        )
