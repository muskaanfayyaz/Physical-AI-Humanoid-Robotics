# Chatbot Configuration Examples

Quick reference for common configuration scenarios.

---

## 1. Basic Setup (Local Development)

**File: `docusaurus.config.js`**

```javascript
module.exports = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'From Simulation to Reality',
  // ... other config

  customFields: {
    // Point to local backend
    chatbotApiUrl: 'http://localhost:8000/api/v1',
  },
};
```

**File: `docs/chapters/chapter-01.mdx`**

```mdx
# Chapter 1: Introduction

Your chapter content here...

---

import ChatbotWrapper from '@site/src/components/ChatbotWrapper';

<ChatbotWrapper />
```

---

## 2. Production Setup

**File: `docusaurus.config.js`**

```javascript
const isProduction = process.env.NODE_ENV === 'production';

module.exports = {
  title: 'Physical AI & Humanoid Robotics',

  customFields: {
    chatbotApiUrl: isProduction
      ? 'https://your-backend.railway.app/api/v1'
      : 'http://localhost:8000/api/v1',
  },
};
```

---

## 3. Environment Variable Configuration

**File: `.env`**

```env
CHATBOT_API_URL=https://your-backend.railway.app/api/v1
```

**File: `docusaurus.config.js`**

```javascript
module.exports = {
  title: 'Physical AI & Humanoid Robotics',

  customFields: {
    chatbotApiUrl: process.env.CHATBOT_API_URL || 'http://localhost:8000/api/v1',
  },
};
```

**File: `.env.example`** (commit to git)

```env
# Chatbot Backend API URL
CHATBOT_API_URL=http://localhost:8000/api/v1
```

---

## 4. Global Integration (All Pages)

**File: `src/theme/Root.js`** (create if doesn't exist)

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

**No changes needed in MDX files!**

---

## 5. Chapter-Only Integration

**File: `src/theme/Root.js`**

```javascript
import React from 'react';
import { useLocation } from '@docusaurus/router';
import ChatbotWrapper from '@site/src/components/ChatbotWrapper';

export default function Root({ children }) {
  const location = useLocation();

  // Show chatbot only on chapter pages
  const showChatbot = location.pathname.startsWith('/chapters/');

  return (
    <>
      {children}
      {showChatbot && <ChatbotWrapper />}
    </>
  );
}
```

---

## 6. Custom Styling

**File: `src/css/custom.css`**

```css
/* Change chatbot button position */
.chatbot-floating-button {
  bottom: 100px !important;
  right: 30px !important;
}

/* Custom colors (light mode) */
:root {
  --chatbot-primary-gradient-start: #667eea;
  --chatbot-primary-gradient-end: #764ba2;
}

/* Custom colors (dark mode) */
[data-theme='dark'] {
  --chatbot-primary-gradient-start: #a78bfa;
  --chatbot-primary-gradient-end: #c084fc;
}
```

---

## 7. Multiple Deployment Targets

**File: `docusaurus.config.js`**

```javascript
const getApiUrl = () => {
  // GitHub Pages deployment
  if (process.env.DEPLOY_TARGET === 'github') {
    return 'https://your-backend.railway.app/api/v1';
  }

  // Netlify deployment
  if (process.env.DEPLOY_TARGET === 'netlify') {
    return 'https://your-backend.netlify.app/api/v1';
  }

  // Vercel deployment
  if (process.env.DEPLOY_TARGET === 'vercel') {
    return 'https://your-backend.vercel.app/api/v1';
  }

  // Local development
  return 'http://localhost:8000/api/v1';
};

module.exports = {
  customFields: {
    chatbotApiUrl: getApiUrl(),
  },
};
```

**GitHub Actions:**
```yaml
- name: Build
  env:
    DEPLOY_TARGET: github
  run: npm run build
```

---

## 8. Lazy Loading (Performance)

**File: `src/components/ChatbotWrapper.jsx`**

```javascript
import React, { lazy, Suspense } from 'react';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

const Chatbot = lazy(() => import('./Chatbot'));

const ChatbotWrapper = () => {
  const { siteConfig } = useDocusaurusContext();
  const apiBaseUrl = siteConfig.customFields?.chatbotApiUrl;

  return (
    <Suspense fallback={null}>
      <Chatbot apiBaseUrl={apiBaseUrl} />
    </Suspense>
  );
};

export default ChatbotWrapper;
```

---

## 9. Analytics Integration

**File: `src/components/Chatbot.jsx`** (add to sendMessage function)

```javascript
const sendMessage = async (question) => {
  // ... existing code

  // Google Analytics
  if (window.gtag) {
    window.gtag('event', 'chatbot_question', {
      event_category: 'Chatbot',
      event_label: mode,
      value: question.length,
    });
  }

  // Plausible Analytics
  if (window.plausible) {
    window.plausible('Chatbot Question', {
      props: { mode: mode }
    });
  }

  // ... rest of function
};
```

---

## 10. Custom Session Management

**File: `src/components/Chatbot.jsx`**

```javascript
// Use localStorage for persistent sessions
const [sessionId] = useState(() => {
  let id = localStorage.getItem('chatbot-session-id');

  if (!id) {
    id = `session-${Date.now()}-${Math.random().toString(36).substr(2, 9)}`;
    localStorage.setItem('chatbot-session-id', id);
  }

  return id;
});
```

---

## 11. Backend CORS Configuration

**Backend: `.env`**

```env
# Local development
CORS_ORIGINS=["http://localhost:3000"]

# Production (update with your frontend URLs)
CORS_ORIGINS=["https://muskaanfayyaz.github.io","https://your-custom-domain.com"]
```

**Backend: `app/config.py`**

```python
cors_origins: list[str] = [
    "http://localhost:3000",
    "https://muskaanfayyaz.github.io",
]
```

---

## 12. Testing Configuration

**File: `docusaurus.config.js`**

```javascript
const isTest = process.env.NODE_ENV === 'test';

module.exports = {
  customFields: {
    chatbotApiUrl: isTest
      ? 'http://localhost:8000/api/v1'  // Use real backend in tests
      : process.env.CHATBOT_API_URL || 'http://localhost:8000/api/v1',
  },
};
```

---

## Complete Example: Production-Ready Config

**File: `docusaurus.config.js`**

```javascript
const lightCodeTheme = require('prism-react-renderer').themes.github;
const darkCodeTheme = require('prism-react-renderer').themes.dracula;

const isProduction = process.env.NODE_ENV === 'production';

module.exports = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'From Simulation to Reality',
  favicon: 'img/favicon.ico',

  url: 'https://muskaanfayyaz.github.io',
  baseUrl: '/Physical-AI-Humanoid-Robotics/',

  organizationName: 'muskaanfayyaz',
  projectName: 'Physical-AI-Humanoid-Robotics',
  deploymentBranch: 'gh-pages',
  trailingSlash: false,

  onBrokenLinks: 'warn',
  onBrokenMarkdownLinks: 'warn',

  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  // Custom fields for chatbot
  customFields: {
    chatbotApiUrl: isProduction
      ? process.env.CHATBOT_API_URL || 'https://your-backend.railway.app/api/v1'
      : 'http://localhost:8000/api/v1',
  },

  presets: [
    [
      'classic',
      {
        docs: {
          routeBasePath: '/',
          sidebarPath: require.resolve('./sidebars.js'),
        },
        blog: false,
        theme: {
          customCss: require.resolve('./src/css/custom.css'),
        },
      },
    ],
  ],

  themeConfig: {
    navbar: {
      title: 'Physical AI & Humanoid Robotics',
      logo: {
        alt: 'Physical AI Logo',
        src: 'img/logo-transparent.png',
      },
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'textbookSidebar',
          position: 'left',
          label: 'Textbook',
        },
        {
          href: 'https://github.com/muskaanfayyaz/Physical-AI-Humanoid-Robotics',
          label: 'GitHub',
          position: 'right',
        },
      ],
    },
    footer: {
      style: 'dark',
      copyright: `Copyright © ${new Date().getFullYear()} Panaversity. Built with Docusaurus.`,
    },
    prism: {
      theme: lightCodeTheme,
      darkTheme: darkCodeTheme,
      additionalLanguages: ['python', 'bash', 'yaml', 'json'],
    },
    colorMode: {
      defaultMode: 'light',
      disableSwitch: false,
      respectPrefersColorScheme: true,
    },
  },
};
```

---

## Deployment Commands

### Local Development
```bash
# Terminal 1: Backend
cd backend
uvicorn app.main:app --reload

# Terminal 2: Frontend
cd book
npm start
```

### Production Build
```bash
# Build frontend
cd book
npm run build

# Test production build locally
npm run serve

# Deploy
npm run deploy  # For GitHub Pages
```

### Environment Setup
```bash
# Create .env file
cp .env.example .env

# Edit with your values
nano .env
```

---

**Quick Test:**

1. Update `docusaurus.config.js` with API URL
2. Add `<ChatbotWrapper />` to a chapter
3. Start backend: `uvicorn app.main:app --reload`
4. Start frontend: `npm start`
5. Open http://localhost:3000
6. Click chatbot button
7. Ask: "What is Physical AI?"

---

**Status**: ✅ Configuration Complete
**Files**: 3 (Chatbot.jsx, CSS, Wrapper)
**Setup Time**: 5-10 minutes
**Production Ready**: Yes
