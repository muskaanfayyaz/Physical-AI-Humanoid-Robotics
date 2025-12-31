# Chatbot Integration Guide

Complete guide to integrating the Physical AI chatbot into your Docusaurus textbook.

---

## Overview

The chatbot provides two powerful modes:

1. **Normal Mode** - Ask questions with RAG retrieval from entire textbook
2. **Selected Text Mode** - Ask about specific text selections (auto-activated on text selection)

**Features:**
- 🤖 AI-powered answers using GPT-4o-mini
- 📚 Source attribution with links
- 💬 Conversation history
- 🌓 Dark mode support
- 📱 Fully responsive
- ⚡ Fast and lightweight

---

## Quick Start (5 Minutes)

### Step 1: Configure API URL

Edit `docusaurus.config.js`:

```javascript
module.exports = {
  // ... existing config

  customFields: {
    // For local development
    chatbotApiUrl: 'http://localhost:8000/api/v1',

    // For production (update with your deployed backend URL)
    // chatbotApiUrl: 'https://your-backend.railway.app/api/v1',
  },
};
```

### Step 2: Add to Each Chapter

At the bottom of each chapter MDX file, add:

```mdx
---
import ChatbotWrapper from '@site/src/components/ChatbotWrapper';

<ChatbotWrapper />
---
```

**Example: `docs/chapters/chapter-01-introduction-to-physical-ai.mdx`**

```mdx
# Chapter 1: Introduction to Physical AI

## What is Physical AI?

Physical AI represents the convergence...

[... rest of chapter content ...]

---

## Summary

In this chapter, we explored...

---

import ChatbotWrapper from '@site/src/components/ChatbotWrapper';

<ChatbotWrapper />
```

### Step 3: Test Locally

```bash
# Terminal 1: Start backend
cd backend
uvicorn app.main:app --reload

# Terminal 2: Start Docusaurus
cd book
npm start
```

Visit: http://localhost:3000

---

## Integration Options

### Option 1: Per-Chapter Integration (Recommended)

Add chatbot to specific chapters:

```mdx
import ChatbotWrapper from '@site/src/components/ChatbotWrapper';

<ChatbotWrapper />
```

**Pros:**
- Control which pages have chatbot
- Can customize per chapter
- Cleaner for non-chapter pages

**Cons:**
- Manual addition to each file

---

### Option 2: Global Integration

Add chatbot to all pages via theme component.

**Create: `src/theme/Root.js`**

```javascript
import React from 'react';
import ChatbotWrapper from '@site/src/components/ChatbotWrapper';

export default function Root({ children }) {
  return (
    <>
      {children}
      <ChatbotWrapper />
    </>
  );
}
```

**Pros:**
- Automatic on all pages
- Single configuration

**Cons:**
- Appears on ALL pages (including blog, docs home, etc.)

---

### Option 3: Conditional Global Integration

Show chatbot only on chapter pages.

**Create: `src/theme/Root.js`**

```javascript
import React from 'react';
import { useLocation } from '@docusaurus/router';
import ChatbotWrapper from '@site/src/components/ChatbotWrapper';

export default function Root({ children }) {
  const location = useLocation();

  // Only show on chapter pages
  const isChapterPage = location.pathname.includes('/chapters/');

  return (
    <>
      {children}
      {isChapterPage && <ChatbotWrapper />}
    </>
  );
}
```

---

## Configuration

### API URL Configuration

**Development:**
```javascript
customFields: {
  chatbotApiUrl: 'http://localhost:8000/api/v1',
}
```

**Production:**
```javascript
customFields: {
  chatbotApiUrl: process.env.NODE_ENV === 'production'
    ? 'https://your-backend.railway.app/api/v1'
    : 'http://localhost:8000/api/v1',
}
```

**With Environment Variables:**

```javascript
// docusaurus.config.js
customFields: {
  chatbotApiUrl: process.env.CHATBOT_API_URL || 'http://localhost:8000/api/v1',
}
```

Then in `.env`:
```
CHATBOT_API_URL=https://your-backend.railway.app/api/v1
```

---

## Usage Guide

### Normal Mode

1. Click the floating 💬 button
2. Type your question
3. Get AI-powered answer with sources

**Example Questions:**
- "What is ROS 2?"
- "Explain bipedal locomotion algorithms"
- "How does DDS work in ROS 2?"
- "What are the key features of NVIDIA Isaac Sim?"

### Selected Text Mode

1. **Select text** on the page (highlight with mouse)
2. Chatbot opens automatically
3. Type your question about the selection
4. Get answer based ONLY on selected text

**Example Workflow:**
1. Select paragraph about "DDS middleware"
2. Ask: "What does this say about real-time communication?"
3. Get answer specific to that paragraph

**Use Cases:**
- Clarify confusing paragraphs
- Summarize long sections
- Extract specific facts
- Understand technical terms in context

---

## Customization

### Custom Styling

Override styles in `src/css/custom.css`:

```css
/* Custom chatbot button color */
:root {
  --chatbot-primary: #667eea;
  --chatbot-secondary: #764ba2;
}

/* Custom button position */
.chatbot-floating-button {
  bottom: 80px; /* Move up to avoid footer */
}
```

### Custom API Configuration

Pass props directly to Chatbot component:

```jsx
import Chatbot from '@site/src/components/Chatbot';

<Chatbot
  apiBaseUrl="https://custom-api.com/api/v1"
/>
```

---

## Troubleshooting

### Issue 1: Chatbot Not Appearing

**Symptom:** No floating button visible

**Solutions:**
1. Check import path is correct
2. Verify component files exist in `src/components/`
3. Clear `.docusaurus` cache: `npm run clear`
4. Restart dev server

### Issue 2: "Backend connection failed"

**Symptom:** Error banner in chatbot

**Solutions:**
1. Verify backend is running: `curl http://localhost:8000/health`
2. Check `chatbotApiUrl` in `docusaurus.config.js`
3. Check CORS settings in backend
4. Check browser console for CORS errors

### Issue 3: CORS Errors

**Symptom:** Browser console shows CORS error

**Solution:** Update backend `.env`:
```env
CORS_ORIGINS=["http://localhost:3000","https://your-site.github.io"]
```

### Issue 4: Selection Mode Not Working

**Symptom:** Selecting text doesn't open chatbot

**Solutions:**
1. Select more than 10 characters
2. Select less than 5000 characters
3. Check browser console for errors
4. Try refreshing the page

---

## Production Deployment

### Step 1: Deploy Backend

```bash
# Example with Railway
cd backend
railway up

# Get deployment URL
railway domain
# Returns: https://your-app.railway.app
```

### Step 2: Update Docusaurus Config

```javascript
customFields: {
  chatbotApiUrl: 'https://your-app.railway.app/api/v1',
}
```

### Step 3: Deploy Frontend

```bash
cd book
npm run build
npm run serve  # Test production build locally
```

### Step 4: Verify

1. Visit deployed site
2. Click chatbot button
3. Ask a question
4. Verify sources link correctly

---

## Advanced Features

### Custom Welcome Message

Edit `Chatbot.jsx`:

```jsx
{messages.length === 0 && (
  <div className={styles.welcomeMessage}>
    <p>👋 <strong>Custom Welcome!</strong></p>
    <p>Your custom instructions here...</p>
  </div>
)}
```

### Add Analytics

Track chatbot usage:

```jsx
const sendMessage = async (question) => {
  // Track question
  if (window.gtag) {
    window.gtag('event', 'chatbot_question', {
      question: question.substring(0, 50), // First 50 chars
      mode: mode,
    });
  }

  // ... rest of sendMessage function
};
```

### Custom Session IDs

Use user IDs for session tracking:

```jsx
const [sessionId] = useState(() => {
  // Use actual user ID if available
  const userId = getUserId(); // Your auth function
  return userId || `session-${Date.now()}`;
});
```

---

## Performance Optimization

### Lazy Loading

Load chatbot only when needed:

```jsx
import React, { lazy, Suspense } from 'react';

const Chatbot = lazy(() => import('./Chatbot'));

function ChatbotWrapper() {
  return (
    <Suspense fallback={null}>
      <Chatbot apiBaseUrl="..." />
    </Suspense>
  );
}
```

### Debounce Text Selection

Reduce selection mode triggers:

```jsx
const [selectionTimeout, setSelectionTimeout] = useState(null);

const handleSelection = () => {
  clearTimeout(selectionTimeout);

  setSelectionTimeout(setTimeout(() => {
    const text = window.getSelection().toString().trim();
    if (text.length > 10) {
      setSelectedText(text);
      setMode('selected');
    }
  }, 500)); // 500ms debounce
};
```

---

## Testing

### Manual Testing Checklist

- [ ] Chatbot button appears
- [ ] Clicking button opens/closes chat
- [ ] Can type and send message
- [ ] Loading indicator shows while processing
- [ ] Answer appears with sources
- [ ] Source links work correctly
- [ ] Text selection triggers selected mode
- [ ] Selected text preview shows
- [ ] Can clear selected text
- [ ] Conversation history maintained
- [ ] Clear conversation works
- [ ] Error handling works (try with backend off)
- [ ] Dark mode styling looks good
- [ ] Mobile responsive (test on phone)
- [ ] Works in all major browsers

### Automated Testing

```bash
# Install dependencies
npm install --save-dev @testing-library/react @testing-library/jest-dom

# Run tests
npm test
```

---

## Browser Support

✅ **Supported:**
- Chrome 90+
- Firefox 88+
- Safari 14+
- Edge 90+

⚠️ **Limited Support:**
- IE 11 (requires polyfills)

---

## Security Considerations

### Input Sanitization

Already implemented in component:
- Max input length: 1000 characters
- HTML escaping via React
- No eval() or dangerous code

### API Security

Backend handles:
- CORS validation
- Rate limiting (optional)
- Input validation
- SQL injection protection

### User Privacy

- Session IDs are temporary
- No PII stored in localStorage
- Conversations stored server-side only
- Can be cleared anytime

---

## FAQ

**Q: Does the chatbot work offline?**
A: No, it requires backend API connection.

**Q: Can I use a different LLM?**
A: Yes, modify backend's `llm_service.py` to use different models.

**Q: How much does OpenAI API cost?**
A: ~$0.001 per question with GPT-4o-mini. 1000 questions ≈ $1.

**Q: Can I self-host the LLM?**
A: Yes, modify backend to use Ollama, vLLM, or other local models.

**Q: Does it remember context?**
A: Yes, with `include_history: true` in the API call.

**Q: Can I customize the appearance?**
A: Yes, edit `Chatbot.module.css` or override with custom CSS.

---

## Next Steps

1. ✅ Complete this integration guide
2. Add chatbot to all chapters
3. Test on mobile devices
4. Deploy backend to production
5. Update API URL in config
6. Deploy Docusaurus site
7. Monitor usage and costs
8. Gather user feedback
9. Iterate and improve

---

## Support

**Issues:**
- Check browser console for errors
- Verify backend is running
- Check CORS configuration
- Test with curl commands

**Resources:**
- Backend API docs: `/docs` endpoint
- Docusaurus docs: https://docusaurus.io/
- React docs: https://react.dev/

---

**Status**: ✅ Ready for Production
**Components**: 3 files (Chatbot.jsx, CSS, Wrapper)
**Size**: ~15KB total (minified + gzipped)
**Dependencies**: React (already in Docusaurus)
