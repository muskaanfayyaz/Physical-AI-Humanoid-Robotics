# RAG Chatbot Setup Guide

## 🎯 Overview

This guide explains how to set up and deploy the RAG (Retrieval-Augmented Generation) chatbot integrated into the Physical AI & Humanoid Robotics textbook.

## 📋 Prerequisites

### Backend Requirements
- ✅ Render account (for backend hosting)
- ✅ Google AI Studio API key (for Gemini)
- ✅ Qdrant Cloud account (for vector storage)
- ✅ Neon Postgres account (for metadata storage)

### Frontend Requirements
- ✅ GitHub Pages (for static site hosting)
- ✅ Node.js 18+ (for building Docusaurus)

## 🔧 Backend Setup (Render)

### Step 1: Environment Variables

In your Render dashboard, add these environment variables:

```bash
# Required - Google Gemini API
GOOGLE_API_KEY=your_google_api_key_here

# Required - Neon Postgres
POSTGRES_URL=postgresql://user:pass@host.neon.tech/database?sslmode=require

# Required - Qdrant Cloud
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=your_qdrant_api_key_here

# Optional - CORS (add your GitHub Pages URL)
CORS_ORIGINS=["https://muskaanfayyaz.github.io"]

# Optional - Debug mode
DEBUG=false
```

### Step 2: Verify Backend Health

After deploying, check these endpoints:

```bash
# Root endpoint
curl https://physical-ai-humanoid-robotics-kafl.onrender.com/

# Health check
curl https://physical-ai-humanoid-robotics-kafl.onrender.com/health
```

**Expected Response (Healthy):**
```json
{
  "status": "healthy",
  "app_name": "Physical AI RAG Backend",
  "version": "1.0.0",
  "database_connected": true,
  "qdrant_connected": true,
  "openai_configured": true
}
```

### Step 3: Test API Endpoints

```bash
# Test search endpoint
curl -X POST https://physical-ai-humanoid-robotics-kafl.onrender.com/api/v1/ask \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What is ROS 2?",
    "session_id": "test-session"
  }'
```

## 🎨 Frontend Setup (Docusaurus)

### Step 1: Verify Configuration

Check that `src/config/api.js` has the correct backend URL:

```javascript
export const API_CONFIG = {
  BACKEND_URL: process.env.REACT_APP_BACKEND_URL ||
    'https://physical-ai-humanoid-robotics-kafl.onrender.com',
  // ... rest of config
};
```

### Step 2: Build and Deploy

```bash
# Install dependencies
npm install

# Build for production
npm run build

# Deploy to GitHub Pages
npm run deploy
```

### Step 3: Verify Integration

1. Visit your deployed site: `https://muskaanfayyaz.github.io/Physical-AI-Humanoid-Robotics/`
2. Look for the floating chat button (bottom right)
3. Click to open the chatbot
4. Try asking a question

## 🧪 Testing the Chatbot

### Test 1: Basic Q&A
1. Open the chatbot
2. Type: "What is ROS 2?"
3. Verify you get a response with sources

### Test 2: Text Selection
1. Select any text from the textbook
2. Click "Ask about selection" button
3. Verify the chatbot opens and asks about that text

### Test 3: Conversation History
1. Ask multiple related questions
2. Verify context is maintained
3. Check that conversation flows naturally

### Test 4: Error Handling
1. Temporarily stop backend
2. Try asking a question
3. Verify error message displays properly

## 🔍 Troubleshooting

### Issue: Chat Button Not Appearing

**Possible Causes:**
- Build failed
- JavaScript error in browser console
- Component not imported correctly

**Solution:**
```bash
# Clear cache and rebuild
npm run clear
npm run build

# Check browser console for errors
# Verify src/theme/Root.js exists
```

### Issue: "API request failed" Error

**Possible Causes:**
- Backend not running
- CORS not configured
- Network connectivity

**Solution:**
```bash
# 1. Check backend health
curl https://physical-ai-humanoid-robotics-kafl.onrender.com/health

# 2. Verify CORS settings in backend
# Add your GitHub Pages URL to CORS_ORIGINS

# 3. Check browser network tab for errors
```

### Issue: "Google API key not configured"

**Possible Causes:**
- GOOGLE_API_KEY not set in Render
- API key invalid or expired

**Solution:**
1. Go to Render dashboard → Environment
2. Add/update `GOOGLE_API_KEY`
3. Redeploy backend service
4. Wait for service to restart

### Issue: Chatbot Works But No Sources Shown

**Possible Causes:**
- Chunks not ingested into database
- Qdrant not properly configured

**Solution:**
```bash
# 1. Check if chunks are ingested
curl https://physical-ai-humanoid-robotics-kafl.onrender.com/api/v1/ingest

# 2. Verify Qdrant connection in health check
curl https://physical-ai-humanoid-robotics-kafl.onrender.com/health

# 3. Check backend logs in Render dashboard
```

## 📊 Monitoring

### Backend Logs (Render)
- Go to Render dashboard → Your service → Logs
- Look for errors or warnings
- Monitor API request/response times

### Frontend Logs (Browser)
- Open browser DevTools (F12)
- Check Console tab for errors
- Check Network tab for API calls

### Health Checks
Set up a monitoring service to ping:
```
https://physical-ai-humanoid-robotics-kafl.onrender.com/health
```

Expected uptime: 99%+ (Render free tier may have cold starts)

## 🚀 Optimization Tips

### Backend Performance
1. **Upgrade to Paid Tier** - Eliminates cold starts
2. **Connection Pooling** - Already configured in database.py
3. **Caching** - Consider Redis for frequently asked questions

### Frontend Performance
1. **Code Splitting** - Lazy load chatbot component
2. **Message Limiting** - Paginate long conversations
3. **Debouncing** - Add input debouncing for search-as-you-type

### User Experience
1. **Suggested Questions** - Show popular queries
2. **Quick Actions** - Add predefined question buttons
3. **Keyboard Shortcuts** - Cmd/Ctrl+K to open chat

## 📈 Analytics (Optional)

Track chatbot usage with:

### Frontend Analytics
```javascript
// Add to src/components/RAGChatbot/index.js
useEffect(() => {
  if (typeof window.gtag !== 'undefined') {
    window.gtag('event', 'chatbot_message_sent', {
      'event_category': 'chatbot',
      'event_label': 'user_query'
    });
  }
}, [messages]);
```

### Backend Logging
Already implemented in `app/database.py`:
- SearchQuery table logs all queries
- ConversationHistory tracks all messages
- Can generate usage reports

## 🎓 User Guide for Students

### Getting Started
1. **Open the chatbot** - Click the purple button (bottom right)
2. **Ask a question** - Type any question about Physical AI
3. **Read the response** - AI answers using textbook content
4. **Check sources** - See which chapters were referenced

### Advanced Features
- **Select text** - Highlight any paragraph and click "Ask about selection"
- **Follow-up questions** - Continue the conversation naturally
- **Clear chat** - Click trash icon to start fresh

### Best Practices
- Be specific in your questions
- Ask about concepts you're reading
- Use text selection for detailed explanations
- Check sources to learn more

## 📝 API Documentation

Full API documentation available at:
```
https://physical-ai-humanoid-robotics-kafl.onrender.com/docs
```

Interactive API testing via Swagger UI.

## 🛡️ Security Considerations

### API Keys
- Never commit API keys to Git
- Use environment variables only
- Rotate keys periodically

### CORS
- Restrict to your domain only
- Don't use wildcard (*) in production

### Rate Limiting
Consider adding rate limiting for production:
```python
# In backend
from slowapi import Limiter

limiter = Limiter(key_func=get_remote_address)
app.state.limiter = limiter

@app.post("/api/v1/ask")
@limiter.limit("10/minute")
async def ask_question(...):
    ...
```

## 📞 Support

### Issues
Report issues at: https://github.com/muskaanfayyaz/Physical-AI-Humanoid-Robotics/issues

### Documentation
- Chatbot Component: `src/components/RAGChatbot/README.md`
- Backend Setup: `backend/README.md`
- Project Overview: `CLAUDE.md`

## ✅ Deployment Checklist

Before marking as complete:

### Backend
- [ ] GOOGLE_API_KEY configured
- [ ] Neon Postgres connected
- [ ] Qdrant Cloud connected
- [ ] Health endpoint returns "healthy"
- [ ] CORS includes GitHub Pages URL
- [ ] Chunks ingested successfully

### Frontend
- [ ] Chatbot button visible
- [ ] Chat opens/closes correctly
- [ ] Messages send and receive
- [ ] Text selection works
- [ ] Sources display properly
- [ ] Mobile responsive
- [ ] Dark mode works

### Testing
- [ ] Ask question - gets response
- [ ] Select text - explanation works
- [ ] Error handling works
- [ ] Multiple users can chat simultaneously
- [ ] Conversation history persists

## 🎉 Success Criteria

Your chatbot is fully operational when:

1. ✅ Backend health status is "healthy"
2. ✅ Questions receive accurate, grounded answers
3. ✅ Sources are cited for all responses
4. ✅ Text selection feature works
5. ✅ UI is professional and responsive
6. ✅ Error messages are helpful
7. ✅ Works on mobile devices

## 🌟 Bonus Features (Future)

Consider implementing:
- [ ] Voice input/output
- [ ] Multi-language support (Urdu translation)
- [ ] User authentication with better-auth
- [ ] Personalized responses based on user background
- [ ] Question suggestions based on current chapter
- [ ] Export chat history as PDF
- [ ] Code snippet execution for examples

---

**Last Updated:** December 29, 2024
**Version:** 1.0.0
**Created for:** GIAIC Hackathon I
