# RAG Content Preparation for Physical AI Textbook

This directory contains tools and data for preparing the Physical AI & Humanoid Robotics textbook content for Retrieval-Augmented Generation (RAG) systems.

## Overview

The textbook has been chunked into **632 semantic segments** optimized for vector embedding and retrieval. Each chunk:

- Contains **300-500 tokens** (target range)
- Preserves **chapter and section metadata**
- Maintains **semantic coherence**
- Includes **hierarchical context**

## Files

### Generated Data

- **`chunks.json`** - Main output file with all chunks and metadata (primary RAG data source)
- **`chunk_schema.json`** - JSON schema documenting chunk structure
- **`chunk_stats.md`** - Detailed statistics about chunking results

### Scripts

- **`chunk_textbook_v2.py`** - Optimized chunking script (recommended)
- **`chunk_textbook.py`** - Original chunking script (deprecated)

## Chunk Structure

Each chunk in `chunks.json` follows this schema:

```json
{
  "chunk_id": "chapter-01-introduction-to-physical-ai_chunk_0001",
  "content": "The actual text content of the chunk...",
  "metadata": {
    "chapter_type": "chapter",
    "chapter_number": 1,
    "chapter_title_slug": "introduction-to-physical-ai",
    "filename": "chapter-01-introduction-to-physical-ai",
    "section_level": 2,
    "section_title": "From Digital to Physical AI",
    "section_path": ["Chapter 1: Introduction to Physical AI", "From Digital to Physical AI"],
    "heading_hierarchy": "Chapter 1: Introduction to Physical AI > From Digital to Physical AI",
    "part_number": 1,
    "total_parts": 1,
    "token_count": 365,
    "char_count": 2145
  }
}
```

### Metadata Fields

| Field | Type | Description |
|-------|------|-------------|
| `chunk_id` | string | Unique identifier for the chunk |
| `content` | string | Actual text content (300-500 tokens) |
| `chapter_type` | string | Type: "chapter" or "appendix" |
| `chapter_number` | integer | Chapter/appendix number |
| `chapter_title_slug` | string | URL-friendly chapter title |
| `filename` | string | Source markdown filename |
| `section_level` | integer | Heading level (1-6) |
| `section_title` | string | Section title |
| `section_path` | array | Full heading hierarchy |
| `heading_hierarchy` | string | Human-readable path |
| `part_number` | integer | Part number if chunk was split |
| `total_parts` | integer | Total parts for this section |
| `token_count` | integer | Estimated token count |
| `char_count` | integer | Character count |

## Statistics

```
Total Chunks: 632
Average Tokens: 306.6
Min Tokens: 0
Max Tokens: 1030
In Target Range (300-500): 353 (55.9%)
```

### Chapter Distribution

| Chapter | Chunks |
|---------|--------|
| Chapter 1: Introduction to Physical AI | 12 |
| Chapter 2: Sensor Systems | 15 |
| Chapter 3: Introduction to ROS 2 | 15 |
| Chapter 4: Building with ROS 2 | 17 |
| Chapter 5: ROS 2 for Humanoid Robots | 24 |
| Chapter 6: Physics Simulation with Gazebo | 28 |
| Chapter 7: High-Fidelity Simulation with Unity | 40 |
| Chapter 8: NVIDIA Isaac Platform | 31 |
| Chapter 9: Isaac ROS | 33 |
| Chapter 10: Navigation and Path Planning | 41 |
| Chapter 11: Humanoid Robot Kinematics | 26 |
| Chapter 12: Bipedal Locomotion | 36 |
| Chapter 13: Manipulation and Grasping | 34 |
| Chapter 14: Human-Robot Interaction | 36 |
| Chapter 15: Conversational Robotics | 43 |
| Chapter 16: Sim-to-Real Transfer | 34 |
| Chapter 17: Edge Computing | 38 |
| Chapter 18: The Autonomous Humanoid | 27 |
| Appendix A: Hardware Setup | 20 |
| Appendix B: Software Installation | 27 |
| Appendix C: Reference Materials | 20 |
| Appendix D: Mathematical Foundations | 19 |
| Appendix E: Datasets and Resources | 16 |

## Usage with RAG Systems

### 1. Vector Database Integration

**Recommended:** Qdrant Cloud (Free Tier)

```python
from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams, PointStruct
import json

# Load chunks
with open('chunks.json', 'r') as f:
    data = json.load(f)
    chunks = data['chunks']

# Initialize Qdrant
client = QdrantClient(url="your-qdrant-url", api_key="your-api-key")

# Create collection
client.create_collection(
    collection_name="physical_ai_textbook",
    vectors_config=VectorParams(size=1536, distance=Distance.COSINE)  # OpenAI ada-002
)

# Upload chunks with embeddings
points = []
for i, chunk in enumerate(chunks):
    # Generate embedding (using OpenAI, for example)
    embedding = generate_embedding(chunk['content'])  # Your embedding function

    points.append(PointStruct(
        id=i,
        vector=embedding,
        payload={
            "chunk_id": chunk['chunk_id'],
            "content": chunk['content'],
            **chunk['metadata']
        }
    ))

client.upsert(collection_name="physical_ai_textbook", points=points)
```

### 2. Query Example

```python
# User query
query = "How does ROS 2 handle communication between nodes?"

# Generate query embedding
query_embedding = generate_embedding(query)

# Search
results = client.search(
    collection_name="physical_ai_textbook",
    query_vector=query_embedding,
    limit=5,
    with_payload=True
)

# Results include chunk content and metadata
for result in results:
    print(f"Score: {result.score}")
    print(f"Chapter: {result.payload['heading_hierarchy']}")
    print(f"Content: {result.payload['content'][:200]}...")
    print()
```

### 3. Contextual Retrieval

Use metadata for filtering and context:

```python
# Filter by chapter
results = client.search(
    collection_name="physical_ai_textbook",
    query_vector=query_embedding,
    query_filter={
        "must": [
            {"key": "chapter_number", "match": {"value": 3}}  # ROS 2 chapters
        ]
    },
    limit=5
)

# Filter by section level (only main sections)
results = client.search(
    collection_name="physical_ai_textbook",
    query_vector=query_embedding,
    query_filter={
        "must": [
            {"key": "section_level", "range": {"lte": 2}}  # H1 and H2 only
        ]
    },
    limit=5
)
```

## Regenerating Chunks

To regenerate chunks with different parameters:

```bash
cd rag
python3 chunk_textbook_v2.py
```

Modify parameters in the script:

```python
# In chunk_textbook_v2.py
chunker = OptimizedTextbookChunker(
    target_min=300,  # Minimum tokens
    target_max=500   # Maximum tokens
)
```

## Integration with Hackathon Requirements

### RAG Chatbot Requirements

This chunking supports all hackathon RAG requirements:

✅ **Text Selection Query Support**
- Each chunk has precise `chunk_id` for reference
- Metadata enables exact source location
- Hierarchical context preserved

✅ **FastAPI Integration**
```python
from fastapi import FastAPI
from pydantic import BaseModel

app = FastAPI()

class Query(BaseModel):
    text: str
    selected_text: str = None  # For text selection queries

@app.post("/query")
async def query_textbook(query: Query):
    if query.selected_text:
        # Search within selected text context
        results = search_with_context(query.text, query.selected_text)
    else:
        # General search
        results = search(query.text)
    return results
```

✅ **Neon Postgres for Conversation History**
```sql
CREATE TABLE conversations (
    id SERIAL PRIMARY KEY,
    user_id VARCHAR(255),
    query TEXT,
    selected_text TEXT,
    retrieved_chunks JSONB,
    response TEXT,
    created_at TIMESTAMP DEFAULT NOW()
);
```

✅ **OpenAI Agents SDK Integration**
```python
from openai import OpenAI

def generate_response(query: str, context_chunks: List[Dict]):
    # Combine retrieved chunks
    context = "\n\n".join([
        f"[{chunk['metadata']['heading_hierarchy']}]\n{chunk['content']}"
        for chunk in context_chunks
    ])

    # Generate response
    response = client.chat.completions.create(
        model="gpt-4",
        messages=[
            {"role": "system", "content": "You are a Physical AI textbook assistant."},
            {"role": "user", "content": f"Context:\n{context}\n\nQuestion: {query}"}
        ]
    )

    return response.choices[0].message.content
```

## Token Counting

Tokens are estimated using **word-based approximation**:

```
tokens ≈ words × 1.3
```

This provides conservative estimates for GPT-style tokenization (cl100k_base encoding).

For production use, consider:
- Installing `tiktoken` for exact counts
- Pre-computing exact embeddings
- Validating token counts during vector upload

## Best Practices

### Embedding Generation

1. **Use consistent embedding model**: OpenAI `text-embedding-ada-002` or similar
2. **Batch process**: Generate embeddings in batches of 100-500
3. **Cache embeddings**: Store in database to avoid regeneration
4. **Monitor costs**: Track API usage for embedding generation

### Retrieval Optimization

1. **Hybrid search**: Combine vector similarity with keyword matching
2. **Re-ranking**: Use cross-encoder for final ranking
3. **Context windows**: Retrieve adjacent chunks for continuity
4. **Metadata filtering**: Use chapter/section filters for precision

### Response Generation

1. **Cite sources**: Include chunk_id and heading_hierarchy in responses
2. **Handle multi-chunk answers**: Combine multiple relevant chunks
3. **Preserve context**: Show section hierarchy to user
4. **Link to full chapters**: Provide GitHub Pages URLs for deep reading

## Troubleshooting

### Issue: Chunks too small/large

**Solution:** Adjust `target_min` and `target_max` in script

### Issue: Poor semantic boundaries

**Solution:** Chunks split at section headings and paragraphs. For better boundaries, modify `merge_small_sections()` logic.

### Issue: Missing metadata

**Solution:** Ensure markdown files have proper heading structure (# ## ###)

## Future Improvements

- [ ] Add code block detection and special handling
- [ ] Implement table extraction and structuring
- [ ] Add image reference tracking
- [ ] Create embeddings pipeline
- [ ] Generate chunk summaries for hybrid search
- [ ] Add inter-chunk relationship mapping

## Support

For issues or questions about RAG integration:
- Check `chunks.json` schema
- Verify token counts are reasonable
- Test with small subset first
- Monitor embedding quality

---

**Generated:** December 27, 2025
**Script Version:** v2.0
**Total Chunks:** 632
**Ready for:** Vector embedding and RAG deployment
