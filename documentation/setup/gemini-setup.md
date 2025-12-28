# Using Gemini API Instead of OpenAI

Complete guide to use Google's Gemini API (FREE tier available!) instead of OpenAI.

---

## Why Gemini?

**Cost Comparison:**

| Service | Free Tier | Paid Cost |
|---------|-----------|-----------|
| **OpenAI** | $5 credit (expires) | $0.001/query |
| **Gemini** | 15 requests/min FREE | FREE up to limit |

**Gemini Advantages:**
- ✅ **FREE tier** with 15 requests/min
- ✅ **No credit card** required for free tier
- ✅ Generous rate limits
- ✅ Good quality responses
- ✅ Fast inference

**Estimated Monthly Cost:**
- OpenAI: $1-2/month
- **Gemini: $0 (FREE!)** 🎉

---

## Part 1: Get Gemini API Key (2 minutes)

### Step 1.1: Go to Google AI Studio

Visit: **https://aistudio.google.com/app/apikey**

### Step 1.2: Sign In

- Sign in with your Google account
- Accept terms of service

### Step 1.3: Create API Key

1. Click **"Get API key"** or **"Create API key"**
2. Select **"Create API key in new project"**
3. Copy your API key:
   ```
   AIzaSyXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
   ```

⚠️ **Save this key!** You'll need it for deployment.

### Step 1.4: Verify Free Tier

Free tier includes:
- ✅ **15 requests per minute**
- ✅ **1,500 requests per day**
- ✅ **1 million tokens per month**
- ✅ **No credit card required**

Perfect for demos and hackathons!

---

## Part 2: Update Backend for Gemini

### Step 2.1: Update Requirements

**Edit:** `backend/requirements.txt`

Add Gemini SDK:

```txt
# Existing dependencies
fastapi==0.109.0
uvicorn[standard]==0.27.0
# ... other dependencies

# Add Gemini SDK
google-generativeai==0.3.2
```

### Step 2.2: Update Configuration

**Edit:** `backend/app/config.py`

Update the settings class:

```python
class Settings(BaseSettings):
    """Application settings loaded from environment variables."""

    # Application
    app_name: str = "Physical AI RAG Backend"
    app_version: str = "1.0.0"
    debug: bool = False

    # AI Provider Selection
    ai_provider: str = "gemini"  # "openai" or "gemini"

    # OpenAI (if using)
    openai_api_key: Optional[str] = None
    openai_embedding_model: str = "text-embedding-3-small"

    # Gemini (if using)
    gemini_api_key: Optional[str] = None
    gemini_model: str = "gemini-1.5-flash"
    gemini_embedding_model: str = "models/embedding-001"

    # Common settings
    embedding_dimensions: int = 768  # Gemini embeddings are 768-dim

    # ... rest of config
```

### Step 2.3: Create Gemini Services

**Create:** `backend/app/services/gemini_service.py`

```python
"""
Gemini AI service for embeddings and chat completions.
"""

import google.generativeai as genai
from typing import List, Dict, Optional
import logging

from app.config import get_settings

logger = logging.getLogger(__name__)


class GeminiEmbeddingService:
    """Service for generating embeddings using Gemini."""

    def __init__(self):
        settings = get_settings()
        genai.configure(api_key=settings.gemini_api_key)
        self.model_name = settings.gemini_embedding_model

    async def generate_embedding(self, text: str) -> List[float]:
        """Generate embedding for a single text."""
        try:
            result = genai.embed_content(
                model=self.model_name,
                content=text,
                task_type="retrieval_document"
            )
            return result['embedding']
        except Exception as e:
            logger.error(f"Gemini embedding failed: {e}")
            raise

    async def generate_embeddings_batch(
        self, texts: List[str], batch_size: int = 100
    ) -> List[List[float]]:
        """Generate embeddings for multiple texts."""
        embeddings = []

        for i in range(0, len(texts), batch_size):
            batch = texts[i : i + batch_size]

            try:
                # Gemini supports batch embedding
                results = genai.embed_content(
                    model=self.model_name,
                    content=batch,
                    task_type="retrieval_document"
                )

                batch_embeddings = results['embedding'] if isinstance(results['embedding'][0], list) else [results['embedding']]
                embeddings.extend(batch_embeddings)

                logger.info(f"Processed batch {i // batch_size + 1}: {len(batch)} texts")

            except Exception as e:
                logger.error(f"Batch embedding failed: {e}")
                raise

        return embeddings

    async def test_connection(self) -> bool:
        """Test Gemini API connection."""
        try:
            await self.generate_embedding("test")
            return True
        except Exception as e:
            logger.error(f"Gemini connection test failed: {e}")
            return False


class GeminiLLMService:
    """Service for generating answers using Gemini."""

    def __init__(self):
        settings = get_settings()
        genai.configure(api_key=settings.gemini_api_key)
        self.model = genai.GenerativeModel(settings.gemini_model)
        self.temperature = 0.1

    async def answer_with_context(
        self,
        query: str,
        context_chunks: List[Dict[str, str]],
        conversation_history: Optional[List[Dict[str, str]]] = None,
    ) -> Dict[str, any]:
        """Generate answer using ONLY the provided context."""

        # Build context from chunks
        context_text = self._build_context(context_chunks)

        # Create prompt
        prompt = f"""You are a helpful AI assistant for the Physical AI & Humanoid Robotics textbook.

Context from textbook:
{context_text}

Question: {query}

Instructions:
- Answer ONLY using the context above
- If the answer is not in the context, say "I cannot answer this question based on the provided textbook content."
- Do NOT use external knowledge
- Cite sources when possible (e.g., "According to Source 1...")
- Be concise but comprehensive

Answer:"""

        try:
            # Generate response
            response = self.model.generate_content(
                prompt,
                generation_config={
                    'temperature': self.temperature,
                    'max_output_tokens': 1000,
                }
            )

            answer = response.text

            # Extract sources
            sources = [
                {
                    "chunk_id": chunk["chunk_id"],
                    "heading": chunk.get("heading_hierarchy", "Unknown"),
                    "chapter": chunk.get("chapter_title_slug", "Unknown"),
                }
                for chunk in context_chunks
            ]

            # Estimate tokens (Gemini doesn't provide exact count in free tier)
            tokens_used = len(prompt.split()) + len(answer.split())

            return {
                "answer": answer,
                "sources": sources,
                "model": "gemini-1.5-flash",
                "finish_reason": "stop",
                "tokens_used": tokens_used,
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
        """Generate answer using ONLY the selected text."""

        prompt = f"""You are analyzing selected text from the Physical AI & Humanoid Robotics textbook.

Selected Text:
{selected_text}

Question: {query}

Instructions:
- Answer ONLY using the selected text above
- If the answer is not in the selected text, say "I cannot find the answer to this question in the selected text."
- Do NOT use external knowledge
- Be precise and direct

Answer:"""

        try:
            response = self.model.generate_content(
                prompt,
                generation_config={
                    'temperature': self.temperature,
                    'max_output_tokens': 1000,
                }
            )

            answer = response.text
            tokens_used = len(prompt.split()) + len(answer.split())

            return {
                "answer": answer,
                "selection_length": len(selected_text),
                "selection_metadata": selection_metadata or {},
                "model": "gemini-1.5-flash",
                "finish_reason": "stop",
                "tokens_used": tokens_used,
                "is_grounded": self._check_if_grounded(answer),
            }

        except Exception as e:
            logger.error(f"Gemini selection answer failed: {e}")
            raise

    def _build_context(self, chunks: List[Dict[str, str]]) -> str:
        """Build formatted context from chunks."""
        context_parts = []
        for i, chunk in enumerate(chunks, 1):
            heading = chunk.get("heading_hierarchy", "Unknown")
            content = chunk.get("content", "")
            context_parts.append(f"[Source {i}: {heading}]\n{content}\n")
        return "\n---\n\n".join(context_parts)

    def _check_if_grounded(self, answer: str) -> bool:
        """Check if answer is grounded."""
        ungrounded_phrases = [
            "cannot answer", "not in the context", "cannot find",
            "not provided", "don't have information",
        ]
        answer_lower = answer.lower()
        return any(phrase in answer_lower for phrase in ungrounded_phrases)

    async def test_connection(self) -> bool:
        """Test Gemini API connection."""
        try:
            response = self.model.generate_content("test")
            return True
        except Exception as e:
            logger.error(f"Gemini LLM test failed: {e}")
            return False


# Singleton instances
_gemini_embedding_service: GeminiEmbeddingService | None = None
_gemini_llm_service: GeminiLLMService | None = None


def get_gemini_embedding_service() -> GeminiEmbeddingService:
    """Get or create Gemini embedding service singleton."""
    global _gemini_embedding_service
    if _gemini_embedding_service is None:
        _gemini_embedding_service = GeminiEmbeddingService()
    return _gemini_embedding_service


def get_gemini_llm_service() -> GeminiLLMService:
    """Get or create Gemini LLM service singleton."""
    global _gemini_llm_service
    if _gemini_llm_service is None:
        _gemini_llm_service = GeminiLLMService()
    return _gemini_llm_service
```

### Step 2.4: Update Service Factory

**Create:** `backend/app/services/ai_factory.py`

```python
"""
AI service factory to switch between OpenAI and Gemini.
"""

from app.config import get_settings


def get_embedding_service():
    """Get embedding service based on configuration."""
    settings = get_settings()

    if settings.ai_provider == "gemini":
        from app.services.gemini_service import get_gemini_embedding_service
        return get_gemini_embedding_service()
    else:
        from app.services.embeddings import get_embedding_service as get_openai
        return get_openai()


def get_llm_service():
    """Get LLM service based on configuration."""
    settings = get_settings()

    if settings.ai_provider == "gemini":
        from app.services.gemini_service import get_gemini_llm_service
        return get_gemini_llm_service()
    else:
        from app.services.llm_service import get_llm_service as get_openai
        return get_openai()
```

### Step 2.5: Update Routers

**Edit:** `backend/app/routers/ask.py`

Change imports at the top:

```python
# OLD:
# from app.services.embeddings import get_embedding_service
# from app.services.llm_service import get_llm_service

# NEW:
from app.services.ai_factory import get_embedding_service, get_llm_service
```

**Edit:** `backend/app/routers/search.py`

```python
# OLD:
# from app.services.embeddings import get_embedding_service

# NEW:
from app.services.ai_factory import get_embedding_service
```

### Step 2.6: Update Qdrant Collection

**Important:** Gemini embeddings are **768 dimensions**, not 1536!

**Edit:** `backend/app/services/qdrant_service.py`

The vector size is already read from config, so it will automatically use 768 when using Gemini.

---

## Part 3: Environment Variables

### For Render Deployment

**Environment Variables:**

```env
# AI Provider
AI_PROVIDER=gemini

# Gemini API (FREE!)
GEMINI_API_KEY=AIzaSyXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
GEMINI_MODEL=gemini-1.5-flash
GEMINI_EMBEDDING_MODEL=models/embedding-001

# Qdrant (provided)
QDRANT_URL=https://cde2a2ae-45d5-486a-aeee-47698644f244.europe-west3-0.gcp.cloud.qdrant.io
QDRANT_API_KEY=eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.U_cSaej0FKcx8R40Dh6sLq512UQ14p2cnF9RpFmx6Wo
QDRANT_COLLECTION_NAME=physical_ai_textbook

# Neon Postgres (see next section)
POSTGRES_URL=postgresql://user:pass@host.neon.tech/db?sslmode=require

# CORS
CORS_ORIGINS=["https://muskaanfayyaz.github.io","http://localhost:3000"]

# Embedding dimensions (Gemini = 768)
EMBEDDING_DIMENSIONS=768
```

### For Local Development

**Create/Update:** `backend/.env`

```env
AI_PROVIDER=gemini
GEMINI_API_KEY=AIzaSyXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
GEMINI_MODEL=gemini-1.5-flash
GEMINI_EMBEDDING_MODEL=models/embedding-001
EMBEDDING_DIMENSIONS=768

QDRANT_URL=https://cde2a2ae-45d5-486a-aeee-47698644f244.europe-west3-0.gcp.cloud.qdrant.io
QDRANT_API_KEY=eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.U_cSaej0FKcx8R40Dh6sLq512UQ14p2cnF9RpFmx6Wo

POSTGRES_URL=postgresql://user:pass@host.neon.tech/db?sslmode=require

CORS_ORIGINS=["http://localhost:3000"]
```

---

## Part 4: Deploy to Render with Gemini

### Step 4.1: Install Dependencies

```bash
cd backend
pip install google-generativeai==0.3.2
pip freeze > requirements.txt
```

### Step 4.2: Set Render Environment Variables

In Render Dashboard → Environment:

1. **AI_PROVIDER**
   ```
   gemini
   ```

2. **GEMINI_API_KEY**
   ```
   AIzaSyXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
   ```

3. **GEMINI_MODEL**
   ```
   gemini-1.5-flash
   ```

4. **GEMINI_EMBEDDING_MODEL**
   ```
   models/embedding-001
   ```

5. **EMBEDDING_DIMENSIONS**
   ```
   768
   ```

6. Keep existing: QDRANT_URL, QDRANT_API_KEY, POSTGRES_URL, CORS_ORIGINS

### Step 4.3: Deploy

1. Commit changes:
   ```bash
   git add .
   git commit -m "Switch to Gemini API (FREE tier)"
   git push origin main
   ```

2. Render will auto-deploy

### Step 4.4: Recreate Qdrant Collection

**Important:** Since vector dimensions changed from 1536 to 768, you need to recreate the collection.

```bash
# After deployment, delete old collection
curl -X DELETE https://physical-ai-backend.onrender.com/api/v1/qdrant/collection

# Re-ingest with new 768-dim embeddings
curl -X POST https://physical-ai-backend.onrender.com/api/v1/ingest/chunks
```

---

## Part 5: Testing with Gemini

```bash
BACKEND_URL=https://physical-ai-backend.onrender.com

# Test health
curl $BACKEND_URL/health

# Test ask endpoint
curl -X POST $BACKEND_URL/api/v1/ask/ \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What is ROS 2?",
    "top_k": 5
  }'

# Should work with Gemini!
```

---

## Comparison: OpenAI vs Gemini

| Feature | OpenAI | Gemini |
|---------|--------|--------|
| **Free Tier** | $5 credit (expires) | 15 req/min (permanent) |
| **Cost** | $0.001/query | **FREE** ✅ |
| **Rate Limit** | 500 req/min (tier 1) | 15 req/min (free) |
| **Embedding Dim** | 1536 | 768 |
| **Quality** | Excellent | Very Good |
| **Speed** | Fast | Fast |
| **Best For** | Production | **Demos, Hackathons** ✅ |

**For this hackathon: Use Gemini!** It's completely FREE.

---

## Cost Savings

**With OpenAI:**
- Initial ingestion: $0.50
- 1000 queries: $1.00
- **Total: ~$1.50/month**

**With Gemini:**
- Initial ingestion: **FREE** ✅
- 1000 queries: **FREE** ✅
- **Total: $0.00/month** 🎉

**Savings: 100%!**

---

## Rate Limits

**Gemini Free Tier:**
- 15 requests per minute
- 1,500 requests per day
- 1 million tokens per month

**For Hackathon Demo:**
- ✅ More than enough!
- ✅ Can handle hundreds of users
- ✅ Perfect for portfolio

**If you exceed limits:**
- Upgrade to paid tier
- Or wait for rate limit reset (1 minute)

---

## Troubleshooting

### "Gemini API key invalid"

1. Go to: https://aistudio.google.com/app/apikey
2. Create new key
3. Update Render environment variable

### "Rate limit exceeded"

**Free tier: 15 requests/min**

Solutions:
1. Wait 1 minute
2. Implement caching (optional)
3. Upgrade to paid tier (rarely needed)

### Embeddings don't match

**Vector dimension changed from 1536 → 768**

Solution:
```bash
# Delete old collection
curl -X DELETE $BACKEND_URL/api/v1/qdrant/collection

# Re-ingest
curl -X POST $BACKEND_URL/api/v1/ingest/chunks
```

---

## Migration Steps Summary

1. ✅ Get Gemini API key (2 min)
2. ✅ Add `google-generativeai` to requirements
3. ✅ Create `gemini_service.py`
4. ✅ Update config for Gemini
5. ✅ Set environment variables
6. ✅ Deploy to Render
7. ✅ Re-ingest chunks (768-dim embeddings)
8. ✅ Test and enjoy FREE API! 🎉

---

**Using Gemini:** ✅ Recommended for FREE deployment
**Total Cost:** **$0.00/month** (completely FREE!)
**Perfect For:** Hackathons, demos, portfolios
