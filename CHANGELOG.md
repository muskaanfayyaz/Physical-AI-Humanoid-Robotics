# Changelog

All notable changes to the Physical AI & Humanoid Robotics textbook will be documented in this file.

## [2.0.0] - 2024-12-29

### 🎉 Major Features

#### Interactive RAG Chatbot
- **NEW:** Integrated AI-powered chatbot assistant on every page
- **RAG Technology:** Retrieval-Augmented Generation ensures grounded, accurate responses
- **Text Selection:** Highlight any text and click "Ask about selection" for instant explanations
- **Source Citations:** All responses include references to relevant textbook chapters
- **Conversation History:** Maintains context across multiple questions
- **Powered by:** Google Gemini 2.0 Flash Lite + Qdrant Vector DB

### Technical Stack
- **Frontend:** React chatbot component integrated into Docusaurus
- **Backend:** FastAPI on Render.com
- **Vector Storage:** Qdrant Cloud
- **Database:** Neon Serverless Postgres
- **AI Model:** Google Gemini 2.0 Flash Lite

### 🔧 Fixes
- Fixed `process.env` issue causing "process is not defined" error in browser
- Updated Gemini model from deprecated `gemini-1.5-flash` to `gemini-2.0-flash-lite`
- Added trailing slashes to API endpoints to prevent 307 redirects
- Removed leaked API keys from repository for security

### 📚 Documentation
- Updated introduction page with v2.0.0 and chatbot features
- Added comprehensive chatbot setup guide (`CHATBOT_SETUP.md`)
- Created chatbot integration documentation (`RAG_CHATBOT_INTEGRATION_COMPLETE.md`)

### 🔒 Security
- Removed exposed Google API key from commit history
- Moved all API keys to environment variables only
- Updated security best practices documentation

---

## [1.0.0] - 2024-12-27

### Initial Release

#### Content
- 18 comprehensive chapters covering Physical AI fundamentals through advanced topics
- 5 appendices with reference materials, installation guides, and resources
- Complete course structure for 13-week university curriculum

#### Chapters
1. Introduction to Physical AI
2. Sensor Systems for Physical AI
3. Introduction to ROS 2
4. Building with ROS 2
5. ROS 2 for Humanoid Robots
6. Physics Simulation with Gazebo
7. Unity for Robotics Simulation
8. NVIDIA Isaac Platform
9. Isaac ROS: Hardware-Accelerated Perception
10. Isaac Sim for Digital Twins
11. Humanoid Robot Kinematics and Dynamics
12. Bipedal Locomotion and Balance
13. Manipulation and Grasping
14. Natural Human-Robot Interaction
15. Vision-Language-Action Models
16. Sim-to-Real Transfer
17. Edge Computing for Physical AI
18. The Autonomous Humanoid

#### Technology Stack
- **Framework:** Docusaurus 3.1.0
- **Deployment:** GitHub Pages
- **Development:** Spec-Kit Plus + Claude Code (AI-native development)

#### Features
- Responsive design with dark mode support
- Mobile-friendly navigation
- Searchable content
- Print-friendly layouts
- Progressive learning structure

---

**Created for GIAIC Hackathon I**
**Powered by:** Spec-Kit Plus & Claude Code
