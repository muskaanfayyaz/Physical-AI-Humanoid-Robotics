# RAG Chatbot Component

## Overview

The RAG (Retrieval-Augmented Generation) Chatbot is an intelligent assistant integrated into the Physical AI & Humanoid Robotics textbook. It provides context-aware answers using the actual textbook content.

## Features

### 🤖 Intelligent Q&A
- Ask questions about any Physical AI or Humanoid Robotics topic
- Receives grounded, accurate answers from the textbook content
- Maintains conversation history for contextual responses

### 💡 Text Selection Support
- Select any text on the page
- Click "Ask about selection" to get detailed explanations
- Uses `/api/v1/ask/selected` endpoint for focused responses

### 📚 Source Citations
- Shows which chapters/sections were used to answer
- Displays relevance scores for transparency
- Links back to original content

### 🎨 Professional UI
- Floating chat button for easy access
- Clean, modern interface
- Dark mode support
- Responsive design for mobile devices
- Smooth animations and transitions

## Architecture

### Component Structure

```
RAGChatbot/
├── index.js          # Main React component
├── styles.css        # Comprehensive styling
└── README.md         # Documentation (this file)
```

### Integration

The chatbot is integrated globally via the Docusaurus Root wrapper:
- `src/theme/Root.js` - Wraps entire app with chatbot
- Available on all pages automatically

### API Integration

Uses the configuration from `src/config/api.js`:

**Endpoints Used:**
- `GET /health` - Backend health check
- `POST /api/v1/ask` - General questions
- `POST /api/v1/ask/selected` - Selected text queries

**Request Format:**
```javascript
// General query
{
  "query": "What is ROS 2?",
  "session_id": "session-123456"
}

// Selected text query
{
  "selected_text": "Text from the page...",
  "session_id": "session-123456",
  "additional_context": "User selected this from the textbook"
}
```

**Response Format:**
```javascript
{
  "answer": "ROS 2 is...",
  "sources": [
    {
      "chapter_title": "Chapter 3: ROS 2 Fundamentals",
      "section_title": "Introduction to ROS 2",
      "similarity_score": 0.92
    }
  ]
}
```

## Session Management

- Generates unique session ID on component mount
- Format: `session-{timestamp}-{random}`
- Persists for conversation duration
- Backend uses this for conversation history

## User Interactions

### Opening the Chat
1. Click floating chat button (bottom right)
2. Chat window slides up
3. Welcome message displayed

### Asking Questions
1. Type question in input field
2. Press Enter or click send button
3. Message appears in chat
4. Response streams back with sources

### Using Text Selection
1. Select text anywhere on the page
2. "Ask about selection" button appears
3. Click button to open chat and ask
4. Or open chat manually and click "Ask about this"

### Clearing Chat
- Click trash icon in header
- Resets to welcome message
- Session ID remains the same

## Styling & Theming

### Color Scheme
- Primary gradient: Purple to violet (`#667eea` → `#764ba2`)
- Integrates with Docusaurus theme variables
- Automatic dark mode support

### Responsive Breakpoints
- Desktop: 400px wide, 600px tall
- Mobile: Full width minus margins, full height minus header

### Animations
- `slideUp` - Chat window entry
- `slideIn` - Selection prompt entry
- `fadeIn` - Message appearance
- `typing` - Loading indicator

## Error Handling

### Backend Unavailable
```
❌ Sorry, I encountered an error. Please make sure the backend
is running and the Google API key is configured.

Error: {error message}
```

### Empty Responses
```
I received your question but couldn't generate a response.
```

### Network Failures
- Displays error message in chat
- Does not crash component
- User can retry

## Performance Optimizations

### Auto-scroll
- Smooth scroll to latest message
- Only triggers on new messages

### Message Limiting
- Currently unlimited history
- Could add pagination for long conversations

### API Request Debouncing
- Input validation prevents empty requests
- Loading state prevents duplicate submissions

## Accessibility

### Keyboard Support
- Enter to send (Shift+Enter for new line)
- Tab navigation through buttons
- Focus management

### ARIA Labels
- `aria-label="Toggle chatbot"` on main button
- Semantic HTML structure
- Screen reader friendly

## Future Enhancements

### Potential Additions
1. **Voice Input** - Speech-to-text for questions
2. **Export Chat** - Download conversation history
3. **Suggested Questions** - Quick-start prompts
4. **Code Highlighting** - Syntax highlighting for code responses
5. **Bookmarking** - Save important Q&As
6. **Multi-language** - Translation support
7. **Analytics** - Track popular questions

### Backend Dependencies
- Requires backend at configured URL
- Needs Google API key configured
- Depends on Qdrant and Neon Postgres

## Testing

### Manual Testing Checklist
- [ ] Chat opens/closes correctly
- [ ] Messages send and receive
- [ ] Text selection works
- [ ] Sources display properly
- [ ] Error handling works
- [ ] Dark mode styling correct
- [ ] Mobile responsive
- [ ] Keyboard shortcuts work

### Integration Testing
```bash
# Start local development server
npm start

# Test features manually in browser
```

## Troubleshooting

### Chat Not Appearing
- Check `src/theme/Root.js` exists
- Verify component import path
- Check browser console for errors

### API Errors
- Verify backend URL in `src/config/api.js`
- Check backend is running
- Confirm GOOGLE_API_KEY is set in Render

### Styling Issues
- Clear browser cache
- Rebuild Docusaurus: `npm run build`
- Check for CSS conflicts

## Development

### Local Development
```bash
# Install dependencies
npm install

# Start dev server
npm start

# Component will hot-reload on changes
```

### Building for Production
```bash
# Build optimized bundle
npm run build

# Serve locally to test
npm run serve
```

## Credits

**Technology Stack:**
- React 18
- Docusaurus 3
- FastAPI (Backend)
- Google Gemini (LLM)
- Qdrant (Vector DB)
- Neon Postgres (Metadata)

**Created for:**
GIAIC Hackathon I - Physical AI & Humanoid Robotics Textbook

**License:**
Same as parent project
