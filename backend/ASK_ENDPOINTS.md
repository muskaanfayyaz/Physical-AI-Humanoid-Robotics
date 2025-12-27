# Ask Endpoints Documentation

Complete guide to the LLM-powered question answering endpoints.

---

## Overview

The `/ask` endpoints provide **grounded question answering** using GPT-4o-mini with strict anti-hallucination measures.

### Key Features

✅ **Grounded Responses** - Answers ONLY from provided context
✅ **No Hallucination** - Refuses to answer if information not present
✅ **Source Attribution** - References specific textbook sections
✅ **Conversation Context** - Maintains chat history (optional)
✅ **Selection-Based Q&A** - Direct answers from user-selected text

---

## Endpoint 1: `/api/v1/ask/` (RAG-Based)

Ask questions and get answers grounded in retrieved textbook chunks.

### How It Works

```
User Query
    ↓
Generate Embedding (OpenAI)
    ↓
Vector Search (Qdrant) → Retrieve Top 5 Chunks
    ↓
LLM (GPT-4o-mini) → Answer ONLY from Chunks
    ↓
Grounded Answer + Sources
```

### Request Schema

**POST** `/api/v1/ask/`

```json
{
  "query": "How does ROS 2 handle communication between nodes?",
  "top_k": 5,
  "similarity_threshold": 0.7,
  "filters": {
    "chapter_type": "chapter",
    "chapter_number": 3
  },
  "session_id": "user-123-session",
  "include_history": true
}
```

**Parameters:**

| Field | Type | Required | Default | Description |
|-------|------|----------|---------|-------------|
| `query` | string | ✅ | - | User's question (1-1000 chars) |
| `top_k` | integer | ❌ | 5 | Number of chunks to retrieve (1-10) |
| `similarity_threshold` | float | ❌ | 0.7 | Minimum similarity score (0.0-1.0) |
| `filters` | object | ❌ | null | Chapter/section filters |
| `session_id` | string | ❌ | null | Session ID for conversation tracking |
| `include_history` | boolean | ❌ | false | Include previous messages in context |

### Response Schema

```json
{
  "query": "How does ROS 2 handle communication between nodes?",
  "answer": "According to Source 1, ROS 2 uses DDS (Data Distribution Service) as its middleware layer for inter-process communication. DDS provides...",
  "sources": [
    {
      "chunk_id": "chapter-03-introduction-to-ros2_chunk_0004",
      "heading": "Chapter 3: ROS 2 > Architecture > DDS Middleware",
      "chapter": "introduction-to-ros2"
    }
  ],
  "model": "gpt-4o-mini",
  "tokens_used": 456,
  "is_grounded": true,
  "processing_time_seconds": 1.23,
  "chunks_retrieved": 5
}
```

**Response Fields:**

| Field | Type | Description |
|-------|------|-------------|
| `query` | string | Original question |
| `answer` | string | Grounded answer from LLM |
| `sources` | array | References to chunks used |
| `model` | string | LLM model used |
| `tokens_used` | integer | Total tokens consumed |
| `is_grounded` | boolean | Whether answer is properly grounded |
| `processing_time_seconds` | float | Total processing time |
| `chunks_retrieved` | integer | Number of chunks retrieved |

### Example Usage

#### Basic Question

```bash
curl -X POST http://localhost:8000/api/v1/ask/ \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What is physical AI?"
  }'
```

#### With Filters

```bash
curl -X POST http://localhost:8000/api/v1/ask/ \
  -H "Content-Type: application/json" \
  -d '{
    "query": "Explain bipedal locomotion algorithms",
    "top_k": 8,
    "filters": {
      "chapter_number": 12
    }
  }'
```

#### With Conversation History

```bash
curl -X POST http://localhost:8000/api/v1/ask/ \
  -H "Content-Type: application/json" \
  -d '{
    "query": "Can you elaborate on that?",
    "session_id": "user-abc-123",
    "include_history": true
  }'
```

### Anti-Hallucination Features

The LLM is instructed to:

1. **Only use provided context** - No external knowledge
2. **Refuse when uncertain** - Say "I cannot answer" if info not present
3. **Cite sources** - Reference which chunks were used
4. **No assumptions** - Don't fill in gaps with guesses
5. **Low temperature** - 0.1 for factual responses

### Example Responses

**When Answer Found:**
```json
{
  "answer": "According to Source 1, ROS 2 uses a pub-sub pattern where nodes communicate through topics. Publishers send messages to topics, and subscribers receive them asynchronously..."
}
```

**When Answer NOT Found:**
```json
{
  "answer": "I cannot answer this question based on the provided textbook content. The retrieved sections discuss [related topic], but don't contain information about [your specific question]."
}
```

**When No Relevant Chunks:**
```json
{
  "answer": "I cannot find any relevant information in the textbook to answer this question. Please try rephrasing your question or asking about a different topic covered in the Physical AI & Humanoid Robotics textbook.",
  "chunks_retrieved": 0
}
```

---

## Endpoint 2: `/api/v1/ask/selected` (Selection-Based)

Ask questions about specific user-selected text without database retrieval.

### How It Works

```
User Selects Text on Page
    ↓
Frontend Sends Text + Question
    ↓
LLM (GPT-4o-mini) → Answer ONLY from Selection
    ↓
Grounded Answer (No Database Lookup)
```

### Request Schema

**POST** `/api/v1/ask/selected`

```json
{
  "query": "What does this paragraph say about DDS?",
  "selected_text": "ROS 2 uses DDS (Data Distribution Service) as its underlying middleware. DDS is an OMG standard that provides real-time, scalable, and efficient data distribution...",
  "selection_metadata": {
    "chunk_id": "chapter-03-introduction-to-ros2_chunk_0004",
    "heading": "Chapter 3 > ROS 2 Architecture",
    "start_offset": 120,
    "end_offset": 450
  }
}
```

**Parameters:**

| Field | Type | Required | Default | Description |
|-------|------|----------|---------|-------------|
| `query` | string | ✅ | - | User's question (1-1000 chars) |
| `selected_text` | string | ✅ | - | Text selected by user (1-10000 chars) |
| `selection_metadata` | object | ❌ | {} | Optional metadata about selection |

### Response Schema

```json
{
  "query": "What does this paragraph say about DDS?",
  "answer": "The selected text explains that DDS (Data Distribution Service) is used as ROS 2's underlying middleware. It's described as an OMG standard that provides real-time, scalable, and efficient data distribution.",
  "selection_length": 330,
  "selection_metadata": {
    "chunk_id": "chapter-03-introduction-to-ros2_chunk_0004",
    "heading": "Chapter 3 > ROS 2 Architecture"
  },
  "model": "gpt-4o-mini",
  "tokens_used": 234,
  "is_grounded": true,
  "processing_time_seconds": 0.45
}
```

### Example Usage

#### Basic Selection Query

```bash
curl -X POST http://localhost:8000/api/v1/ask/selected \
  -H "Content-Type: application/json" \
  -d '{
    "query": "Summarize this section",
    "selected_text": "Humanoid robots are designed to mimic human form and function..."
  }'
```

#### With Metadata

```bash
curl -X POST http://localhost:8000/api/v1/ask/selected \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What are the key points here?",
    "selected_text": "NVIDIA Isaac Sim provides...",
    "selection_metadata": {
      "chunk_id": "chapter-08-nvidia-isaac_chunk_0012",
      "heading": "Chapter 8 > Isaac Sim Features"
    }
  }'
```

### Use Cases

1. **Deep Dive** - Understand specific paragraphs in detail
2. **Clarification** - Get explanations of confusing passages
3. **Quick Facts** - Extract specific information from a section
4. **Comparison** - Analyze selected text without context mixing
5. **Quote Analysis** - Understand what a specific quote means

### Anti-Hallucination Features

The LLM is instructed to:

1. **Only use selected text** - Ignore everything else
2. **Refuse when not present** - "I cannot find the answer in the selected text"
3. **No external knowledge** - Even if it knows the answer
4. **Respect boundaries** - Don't go beyond what user selected
5. **Be precise** - Quote directly when helpful

### Example Responses

**When Answer in Selection:**
```json
{
  "answer": "The selected text states that NVIDIA Isaac Sim uses RTX rendering for photorealistic simulation. It specifically mentions support for ray tracing and path tracing."
}
```

**When Answer NOT in Selection:**
```json
{
  "answer": "I cannot find the answer to this question in the selected text. The selection discusses [what it does discuss], but doesn't mention [your specific question]."
}
```

---

## Comparison: `/ask` vs `/ask/selected`

| Feature | `/ask` (RAG) | `/ask/selected` |
|---------|--------------|-----------------|
| **Retrieval** | ✅ Qdrant vector search | ❌ No database lookup |
| **Context Source** | Top-K chunks | User-selected text |
| **Use Case** | General questions | Specific passage analysis |
| **Processing Time** | ~1-2 seconds | ~0.5 seconds |
| **Context Size** | 5-10 chunks | Single selection |
| **Source Attribution** | Automatic | User provides metadata |
| **Conversation History** | ✅ Supported | ❌ Not applicable |

---

## Implementation Details

### LLM Configuration

```python
model = "gpt-4o-mini"  # Fast and cost-effective
temperature = 0.1      # Low for factual responses
max_tokens = 1000      # Sufficient for detailed answers
```

### System Prompts

#### RAG System Prompt
```
You are a helpful AI assistant for the Physical AI & Humanoid Robotics textbook.

Your role is to answer questions STRICTLY based on the provided context from the textbook. Follow these rules:

1. ONLY use information from the provided context
2. If the answer is not in the context, clearly state: "I cannot answer..."
3. Do NOT use external knowledge or make assumptions
4. Cite which source(s) you used when possible
5. If the context is incomplete, say so
6. Be concise but comprehensive
```

#### Selection System Prompt
```
You are a helpful AI assistant analyzing selected text from the Physical AI & Humanoid Robotics textbook.

Your role is to answer questions STRICTLY based on the selected text. Follow these rules:

1. ONLY use information from the selected text
2. If the answer is not in the selected text, respond: "I cannot find..."
3. Do NOT use external knowledge, even if you know the answer
4. Be precise and direct
```

### Conversation History

When `include_history: true`:
- Last 6 messages (3 turns) included
- Helps with follow-up questions
- Maintains context across multiple queries
- Stored in `conversation_history` table

---

## Error Handling

### Common Errors

**500 - LLM Service Error**
```json
{
  "detail": "Question answering failed: OpenAI API error"
}
```

**500 - Retrieval Error**
```json
{
  "detail": "Question answering failed: Qdrant connection timeout"
}
```

**422 - Validation Error**
```json
{
  "detail": [
    {
      "loc": ["body", "query"],
      "msg": "ensure this value has at least 1 characters",
      "type": "value_error.any_str.min_length"
    }
  ]
}
```

---

## Best Practices

### For `/ask` Endpoint

✅ **Do:**
- Use descriptive questions
- Apply filters to narrow results
- Use session_id for multi-turn conversations
- Check `chunks_retrieved` to verify context quality

❌ **Don't:**
- Ask questions outside textbook scope
- Expect answers beyond the content
- Use very generic queries without filters

### For `/ask/selected` Endpoint

✅ **Do:**
- Select complete thoughts/paragraphs
- Include context if selection is short
- Use for specific passage analysis
- Provide metadata for tracking

❌ **Don't:**
- Select partial sentences
- Expect answers beyond selection
- Use for general questions (use `/ask` instead)

---

## Performance Metrics

### `/ask` Endpoint
- **Latency**: 1-2 seconds
  - Embedding: ~100ms
  - Vector search: ~50ms
  - LLM generation: 500-1000ms
- **Cost**: ~$0.001 per query (GPT-4o-mini)
- **Token Usage**: 300-800 tokens avg

### `/ask/selected` Endpoint
- **Latency**: 0.4-0.8 seconds
  - LLM generation: 400-800ms
- **Cost**: ~$0.0005 per query
- **Token Usage**: 200-500 tokens avg

---

## Frontend Integration

### React Example - `/ask`

```javascript
async function askQuestion(query, filters = null) {
  const response = await fetch(`${API_URL}/api/v1/ask/`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({
      query,
      top_k: 5,
      filters,
      session_id: getSessionId(),
      include_history: true
    })
  });

  const data = await response.json();
  return data;
}
```

### React Example - `/ask/selected`

```javascript
async function askAboutSelection(query, selectedText, metadata) {
  const response = await fetch(`${API_URL}/api/v1/ask/selected`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({
      query,
      selected_text: selectedText,
      selection_metadata: metadata
    })
  });

  const data = await response.json();
  return data;
}
```

---

## Testing

### Test `/ask` Endpoint

```bash
# Test basic question
curl -X POST http://localhost:8000/api/v1/ask/ \
  -H "Content-Type: application/json" \
  -d '{"query": "What is ROS 2?"}'

# Test with no results
curl -X POST http://localhost:8000/api/v1/ask/ \
  -H "Content-Type: application/json" \
  -d '{"query": "How to bake a cake?"}'  # Outside textbook scope
```

### Test `/ask/selected` Endpoint

```bash
# Test with selection
curl -X POST http://localhost:8000/api/v1/ask/selected \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What is the main point?",
    "selected_text": "Physical AI combines artificial intelligence with physical embodiment..."
  }'
```

---

## Cost Estimation

**GPT-4o-mini Pricing** (as of Dec 2024):
- Input: $0.150 per 1M tokens
- Output: $0.600 per 1M tokens

**Estimated Costs:**
- `/ask`: ~$0.001 per query
- `/ask/selected`: ~$0.0005 per query
- 1000 queries: ~$1.00

**Free Tier Limit**: First $5 credit from OpenAI

---

## Security Considerations

✅ **Input Validation** - Query length limits
✅ **Rate Limiting** - Prevent abuse (optional)
✅ **Context Limits** - Max 10 chunks, 10000 chars selection
✅ **No PII Logging** - Queries logged without user data
✅ **CORS Configured** - Restrict origins

---

## Monitoring

Track these metrics:
- Average query latency
- Token usage per query
- Grounded vs ungrounded responses
- Failed queries (errors)
- Most common queries

---

**Status**: ✅ Production Ready
**Model**: GPT-4o-mini
**Anti-Hallucination**: Strict prompting + context limits
**Cost**: ~$0.001 per query
