# Project Constitution: Physical AI & Humanoid Robotics Textbook

## Document Overview

**Version:** 1.0
**Last Updated:** 2025-12-29
**Project:** Physical AI & Humanoid Robotics: From Simulation to Reality
**Hackathon:** GIAIC Hackathon I

This constitution establishes the core principles, standards, and guidelines for all development work on the Physical AI & Humanoid Robotics textbook project. All contributors must adhere to these principles to ensure consistency, quality, and maintainability.

---

## Table of Contents

1. [Core Principles](#core-principles)
2. [Content Standards](#content-standards)
3. [Code Standards](#code-standards)
4. [Development Workflow](#development-workflow)
5. [Documentation Standards](#documentation-standards)
6. [Quality Assurance](#quality-assurance)
7. [Testing Standards](#testing-standards)
8. [Deployment Standards](#deployment-standards)
9. [Security Standards](#security-standards)
10. [Performance Standards](#performance-standards)

---

## Core Principles

### 1. Educational Excellence

**Principle:** Every decision must prioritize the learning experience and educational outcomes.

- **Clarity First:** Technical accuracy is paramount, but never at the expense of student comprehension
- **Progressive Complexity:** Build knowledge incrementally from foundations to advanced topics
- **Practical Relevance:** Every concept must connect to real-world applications
- **Conceptual Understanding:** Focus on "why" and "how" things work, not just "what" they do

### 2. Technical Accuracy

**Principle:** All content, code, and examples must be technically correct and verifiable.

- **Verified Examples:** Every code example must be tested and runnable
- **Current Standards:** Use latest stable versions of ROS 2, NVIDIA Isaac, and related tools
- **Scientific Rigor:** Reference authoritative sources for algorithms and methodologies
- **Honest Limitations:** Acknowledge edge cases, limitations, and trade-offs

### 3. Consistency and Coherence

**Principle:** Maintain uniform style, terminology, and structure throughout the project.

- **Terminology:** Use consistent technical terms across all chapters
- **Structure:** Follow identical chapter format throughout the textbook
- **Notation:** Apply uniform mathematical and algorithmic notation
- **Voice:** Maintain consistent pedagogical tone and approach

### 4. Maintainability and Scalability

**Principle:** Build for long-term sustainability and future enhancements.

- **Modular Design:** Keep components loosely coupled and highly cohesive
- **Clean Code:** Write self-documenting code with clear intent
- **Version Control:** Maintain clear commit history and branching strategy
- **Documentation:** Document architecture, decisions, and complex logic

### 5. Spec-Driven Development

**Principle:** Follow the spec-driven AI-native development methodology.

- **Specification First:** Always start with clear specifications before implementation
- **Plan Before Code:** Create technical plans before writing code
- **Task Breakdown:** Decompose complex work into actionable tasks
- **Quality Gates:** Use specifications as acceptance criteria

---

## Content Standards

### Writing Standards

#### 1. Technical Writing

**Requirements:**
- Use clear, concise, active voice
- Define technical terms before using them
- Break complex concepts into digestible chunks
- Use consistent terminology throughout

**Example - Good:**
```markdown
ROS 2 uses DDS (Data Distribution Service) middleware for communication.
DDS provides real-time, peer-to-peer messaging between nodes.
```

**Example - Bad:**
```markdown
The middleware layer utilized by ROS 2, which is based on DDS technology,
facilitates communication patterns between various computational nodes.
```

#### 2. Chapter Structure

**Required Format:**

```markdown
# Chapter Title

## Introduction and Motivation (5%)
- Real-world context
- Why this topic matters
- Learning objectives

## Core Concepts (30%)
- Theoretical foundations
- Key principles
- Algorithm explanations

## Practical Understanding (50%)
- Step-by-step tutorials
- Code examples with explanations
- Configuration guides
- Troubleshooting tips

## Knowledge Checkpoint (10%)
- Review questions (multiple levels)
- Conceptual questions
- Application questions
- Analysis questions

## Chapter Summary (5%)
- Key takeaways
- Prerequisites for next chapter
- Further reading
```

#### 3. Learning Objectives

**Requirements:**
- 3-5 specific, measurable objectives per chapter
- Use Bloom's taxonomy verbs (understand, apply, analyze, evaluate)
- Focus on student capabilities, not content coverage
- Make objectives testable

**Example - Good:**
```markdown
**Learning Objectives:**
- Implement forward kinematics using DH parameters
- Calculate Jacobian matrices for velocity control
- Solve inverse kinematics for 7-DOF arms
- Handle kinematic singularities in control systems
```

**Example - Bad:**
```markdown
**Learning Objectives:**
- Learn about kinematics
- Know how robots move
- Understand mathematics
```

#### 4. Code Examples

**Requirements:**
- Complete, runnable examples (not fragments)
- Include all necessary imports
- Provide setup instructions
- Add comprehensive inline comments
- Show expected output

**Template:**
```python
"""
Description of what this example demonstrates.

Setup:
1. Install required packages: pip install ...
2. Configure environment: ...

Expected output:
- What you should see when running this code
"""

# Import statements
import rclpy
from rclpy.node import Node

# Well-commented implementation
class ExampleNode(Node):
    """
    Node that demonstrates [specific concept].

    This example shows how to:
    - Point 1
    - Point 2
    """

    def __init__(self):
        super().__init__('example_node')
        # Clear explanation of each step
        self.publisher = self.create_publisher(...)

# Main execution
def main():
    rclpy.init()
    node = ExampleNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### 5. Diagrams and Visualizations

**Requirements:**
- Include diagrams for complex concepts
- Use consistent visual style
- Label all components clearly
- Provide alt text for accessibility
- Reference diagrams in text

**Guidelines:**
- System architecture diagrams for multi-component systems
- Flowcharts for algorithms and decision processes
- 3D visualizations for spatial concepts
- Screenshots for UI/tool demonstrations

#### 6. Knowledge Checkpoints

**Requirements:**
- 10-15 questions per chapter
- Multiple cognitive levels (Bloom's taxonomy)
- Mix of question types (conceptual, application, analysis)
- No trivial questions
- Answers verify understanding, not memorization

**Question Levels:**

1. **Remember/Understand (30%):** Basic concept verification
2. **Apply (40%):** Use concepts in new situations
3. **Analyze/Evaluate (30%):** Critical thinking and problem-solving

---

### Content Quality Standards

#### 1. Accuracy Verification

**Required Checks:**
- [ ] All technical facts verified against authoritative sources
- [ ] All code examples tested and confirmed working
- [ ] All commands verified on target platform
- [ ] All references are current and accessible
- [ ] Mathematical equations verified for correctness

#### 2. Pedagogical Effectiveness

**Required Elements:**
- [ ] Concepts build incrementally on previous knowledge
- [ ] Multiple explanations provided for complex topics
- [ ] Real-world context provided for abstract concepts
- [ ] Common misconceptions addressed
- [ ] Practical applications demonstrated

#### 3. Accessibility

**Requirements:**
- Use inclusive language and diverse examples
- Provide alternative explanations for different learning styles
- Include visual, textual, and code-based explanations
- Avoid assumptions about prerequisite knowledge
- Define all acronyms on first use

---

## Code Standards

### Python Code Standards

#### 1. General Principles

**Follow:**
- PEP 8 style guide for Python code
- PEP 257 for docstring conventions
- Type hints for all function signatures (Python 3.10+)
- Async/await patterns for I/O operations
- Dependency injection for services

#### 2. File Organization

**Backend Structure:**
```
backend/
├── app/
│   ├── __init__.py
│   ├── main.py              # FastAPI application entry
│   ├── config.py            # Settings and configuration
│   ├── database.py          # Database models and connection
│   ├── routers/             # API endpoints
│   │   ├── __init__.py
│   │   ├── ask.py           # Q&A endpoints
│   │   ├── search.py        # Search endpoints
│   │   └── ...
│   ├── services/            # Business logic
│   │   ├── __init__.py
│   │   ├── llm_service.py   # LLM integration
│   │   ├── embeddings.py    # Embedding generation
│   │   └── ...
│   └── schemas/             # Pydantic models
│       ├── __init__.py
│       └── ...
├── tests/                   # Test suite
└── requirements.txt         # Dependencies
```

#### 3. Code Style

**Naming Conventions:**
- `snake_case` for functions, variables, modules
- `PascalCase` for classes
- `UPPER_CASE` for constants
- Descriptive names that reveal intent

**Example - Good:**
```python
def calculate_forward_kinematics(joint_angles: List[float]) -> np.ndarray:
    """
    Calculate end-effector pose from joint angles.

    Args:
        joint_angles: List of joint angles in radians

    Returns:
        4x4 transformation matrix representing end-effector pose
    """
    transformation_matrix = np.eye(4)
    # Implementation...
    return transformation_matrix
```

**Example - Bad:**
```python
def calc(ja):  # Unclear function name and parameter
    tm = np.eye(4)  # Unclear variable name
    return tm
```

#### 4. Type Hints

**Required:**
- All function parameters must have type hints
- All return types must be specified
- Use Union, Optional, List, Dict from typing module
- Use modern syntax (Python 3.10+): `list[str]` instead of `List[str]`

**Example:**
```python
from typing import Optional, Union

async def search_chunks(
    query_vector: list[float],
    top_k: int = 10,
    filters: Optional[dict[str, str]] = None,
    score_threshold: float = 0.7
) -> list[dict[str, Union[str, float]]]:
    """
    Search for similar chunks using vector similarity.

    Args:
        query_vector: Embedding vector for the query
        top_k: Number of results to return
        filters: Optional metadata filters
        score_threshold: Minimum similarity score

    Returns:
        List of matching chunks with metadata
    """
    # Implementation...
```

#### 5. Docstrings

**Required Format (Google Style):**

```python
def function_name(param1: type, param2: type) -> return_type:
    """
    One-line summary of function purpose.

    More detailed description if needed. Explain what the function does,
    why it exists, and any important context.

    Args:
        param1: Description of first parameter
        param2: Description of second parameter

    Returns:
        Description of return value

    Raises:
        ExceptionType: When this exception is raised

    Example:
        >>> result = function_name("value1", "value2")
        >>> print(result)
        Expected output
    """
    # Implementation
```

**Class Docstrings:**
```python
class ServiceName:
    """
    One-line description of the service.

    Detailed description of what this service does, its responsibilities,
    and how it fits into the overall architecture.

    Attributes:
        attribute1: Description of attribute
        attribute2: Description of attribute

    Example:
        >>> service = ServiceName(config)
        >>> result = service.method()
    """

    def __init__(self, config: Config):
        """Initialize the service with configuration."""
        self.config = config
```

#### 6. Error Handling

**Principles:**
- Catch specific exceptions, not bare `except:`
- Provide meaningful error messages
- Log errors appropriately
- Raise custom exceptions for domain-specific errors
- Clean up resources in `finally` blocks

**Example:**
```python
async def process_request(request: Request) -> Response:
    """Process incoming request with proper error handling."""
    try:
        # Validate input
        if not request.query:
            raise ValueError("Query cannot be empty")

        # Process request
        result = await service.process(request.query)

        return Response(success=True, data=result)

    except ValueError as e:
        logger.warning(f"Invalid request: {e}")
        raise HTTPException(status_code=400, detail=str(e))

    except ServiceError as e:
        logger.error(f"Service error: {e}", exc_info=True)
        raise HTTPException(status_code=500, detail="Processing failed")

    except Exception as e:
        logger.error(f"Unexpected error: {e}", exc_info=True)
        raise HTTPException(status_code=500, detail="Internal server error")
```

#### 7. Logging

**Standards:**
- Use Python `logging` module
- Configure logging in `main.py`
- Use appropriate log levels: DEBUG, INFO, WARNING, ERROR, CRITICAL
- Include context in log messages
- Never log sensitive information (API keys, passwords)

**Example:**
```python
import logging

logger = logging.getLogger(__name__)

async def retrieve_chunks(query: str) -> list[dict]:
    """Retrieve relevant chunks with logging."""
    logger.info(f"Retrieving chunks for query: '{query[:50]}...'")

    try:
        results = await vector_db.search(query)
        logger.info(f"Retrieved {len(results)} chunks")
        return results

    except Exception as e:
        logger.error(f"Failed to retrieve chunks: {e}", exc_info=True)
        raise
```

#### 8. Async/Await Patterns

**Requirements:**
- Use `async def` for I/O-bound operations
- Use `await` for async function calls
- Use `asyncio.gather()` for parallel operations
- Use async context managers (`async with`)
- Don't mix sync and async code inappropriately

**Example:**
```python
async def process_multiple_queries(queries: list[str]) -> list[Response]:
    """Process multiple queries in parallel."""
    # Use asyncio.gather for parallel processing
    tasks = [process_single_query(q) for q in queries]
    results = await asyncio.gather(*tasks)
    return results

async def process_single_query(query: str) -> Response:
    """Process a single query with async I/O."""
    # Use async context manager
    async with get_db() as session:
        # Async database operations
        result = await session.execute(query)
        return result
```

#### 9. Dependency Injection

**Pattern:**
- Use FastAPI's dependency injection system
- Create service factory functions
- Use Depends() for dependency declaration
- Singleton services for shared resources

**Example:**
```python
from functools import lru_cache
from fastapi import Depends

@lru_cache()
def get_embedding_service() -> EmbeddingService:
    """Get singleton embedding service instance."""
    settings = get_settings()
    return EmbeddingService(settings.openai_api_key)

@router.post("/search")
async def search(
    request: SearchRequest,
    embedding_service: EmbeddingService = Depends(get_embedding_service)
):
    """Search endpoint using dependency injection."""
    embedding = await embedding_service.generate(request.query)
    # Process search...
```

---

### JavaScript/TypeScript Standards (Docusaurus)

#### 1. Configuration Files

**Requirements:**
- Use JSDoc type annotations in JavaScript files
- Follow Docusaurus configuration standards
- Comment complex configuration sections
- Use constants for repeated values

**Example (docusaurus.config.js):**
```javascript
// @ts-check
/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'From Simulation to Reality',

  // Production deployment configuration
  url: 'https://muskaanfayyaz.github.io',
  baseUrl: '/Physical-AI-Humanoid-Robotics/',

  // Build behavior
  onBrokenLinks: 'warn',
  onBrokenMarkdownLinks: 'warn',

  // ...rest of configuration
};

module.exports = config;
```

#### 2. React Components (if custom components added)

**Requirements:**
- Use functional components with hooks
- PropTypes or TypeScript for type checking
- Clear component documentation
- Follow React best practices

---

## Development Workflow

### 1. Spec-Driven Workflow

**Process:**

```
1. Specification
   ↓
   - Define requirements in spec.md
   - Establish acceptance criteria
   - Review and approve specification

2. Planning
   ↓
   - Create technical plan (plan.md)
   - Choose technologies and approaches
   - Identify dependencies and risks

3. Task Breakdown
   ↓
   - Decompose into actionable tasks
   - Estimate complexity
   - Prioritize tasks

4. Implementation
   ↓
   - Follow code standards
   - Write tests alongside code
   - Document as you go

5. Review
   ↓
   - Self-review against specification
   - Code review by peers
   - Testing and validation

6. Integration
   ↓
   - Merge to main branch
   - Deploy to staging
   - Validate in production-like environment
```

### 2. Git Workflow

**Branch Strategy:**
- `main`: Production-ready code
- `develop`: Integration branch for features
- `feature/*`: Individual feature branches
- `fix/*`: Bug fix branches
- `docs/*`: Documentation updates

**Commit Standards:**
- Write clear, descriptive commit messages
- Use conventional commit format
- One logical change per commit
- Reference issues/tasks in commits

**Commit Format:**
```
type(scope): brief description

Longer explanation if needed. Explain what changed and why,
not how (the code shows how).

Closes #123
```

**Types:**
- `feat`: New feature
- `fix`: Bug fix
- `docs`: Documentation changes
- `style`: Code formatting (no logic change)
- `refactor`: Code restructuring (no behavior change)
- `test`: Adding or updating tests
- `chore`: Build process, dependencies, etc.

**Example:**
```
feat(rag): add selection-based question answering

Implement /ask/selected endpoint that answers questions
based on user-selected text without RAG retrieval.

This enables focused Q&A on specific passages without
hallucination risk.

Closes #42
```

### 3. Pull Request Process

**Requirements:**
- Descriptive title and description
- Link to related issues/tasks
- Self-review checklist completed
- Tests passing
- Documentation updated

**PR Template:**
```markdown
## Description
Brief description of changes

## Type of Change
- [ ] New feature
- [ ] Bug fix
- [ ] Documentation update
- [ ] Refactoring

## Checklist
- [ ] Code follows project standards
- [ ] Tests added/updated
- [ ] Documentation updated
- [ ] Self-reviewed
- [ ] No breaking changes (or documented)

## Testing
How were these changes tested?

## Screenshots (if applicable)
Visual proof of changes
```

---

## Documentation Standards

### 1. Code Documentation

**Required:**
- Module-level docstrings explaining purpose
- Class docstrings with usage examples
- Function docstrings with Args/Returns/Raises
- Inline comments for complex logic
- README files in each major directory

### 2. API Documentation

**Requirements:**
- OpenAPI/Swagger docs for all endpoints
- Request/response schemas documented
- Authentication requirements specified
- Example requests and responses
- Error codes and meanings

### 3. Setup Documentation

**Required Files:**
- `README.md`: Project overview and quick start
- `SETUP.md`: Detailed setup instructions
- `DEPLOYMENT_GUIDE.md`: Deployment procedures
- `ARCHITECTURE.md`: System architecture
- `CONTRIBUTING.md`: Contribution guidelines

**README Structure:**
```markdown
# Project Name

Brief description

## Quick Start

```bash
# Minimal commands to get running
```

## Features

- Feature 1
- Feature 2

## Documentation

- [Setup Guide](SETUP.md)
- [Architecture](ARCHITECTURE.md)
- [API Docs](http://localhost:8000/docs)

## Contributing

See [CONTRIBUTING.md](CONTRIBUTING.md)

## License

Project license
```

---

## Quality Assurance

### 1. Code Review Checklist

**Before Requesting Review:**
- [ ] Code follows all style standards
- [ ] All functions have docstrings
- [ ] Type hints are present
- [ ] No hardcoded values (use config)
- [ ] No commented-out code
- [ ] No debug print statements
- [ ] Error handling implemented
- [ ] Logging added appropriately
- [ ] Tests written and passing
- [ ] Documentation updated

**Reviewer Checklist:**
- [ ] Code is readable and maintainable
- [ ] Logic is correct and efficient
- [ ] Edge cases handled
- [ ] Security considerations addressed
- [ ] No code duplication
- [ ] Tests are comprehensive
- [ ] Documentation is clear

### 2. Content Review Checklist

**Before Publishing:**
- [ ] Technical accuracy verified
- [ ] All code examples tested
- [ ] Terminology consistent
- [ ] Structure follows template
- [ ] Learning objectives clear and measurable
- [ ] Knowledge checkpoints included
- [ ] Cross-references valid
- [ ] Diagrams properly labeled
- [ ] Alt text for images
- [ ] Grammar and spelling checked

---

## Testing Standards

### 1. Test Requirements

**Coverage:**
- Minimum 80% code coverage for backend
- All API endpoints tested
- Edge cases covered
- Error scenarios tested

**Test Types:**
- Unit tests for individual functions
- Integration tests for workflows
- API endpoint tests
- Database interaction tests

### 2. Test Structure

**Example:**
```python
import pytest
from app.services.embeddings import EmbeddingService

class TestEmbeddingService:
    """Test suite for EmbeddingService."""

    @pytest.fixture
    async def service(self):
        """Create service instance for testing."""
        return EmbeddingService(api_key="test-key")

    @pytest.mark.asyncio
    async def test_generate_embedding_success(self, service):
        """Test successful embedding generation."""
        text = "Test input text"
        embedding = await service.generate_embedding(text)

        assert isinstance(embedding, list)
        assert len(embedding) == 1536  # OpenAI embedding dimension
        assert all(isinstance(x, float) for x in embedding)

    @pytest.mark.asyncio
    async def test_generate_embedding_empty_text(self, service):
        """Test embedding generation with empty text."""
        with pytest.raises(ValueError, match="Text cannot be empty"):
            await service.generate_embedding("")
```

### 3. Testing Commands

```bash
# Run all tests
pytest

# Run with coverage
pytest --cov=app --cov-report=html

# Run specific test file
pytest tests/test_embeddings.py

# Run specific test
pytest tests/test_embeddings.py::TestEmbeddingService::test_generate_embedding_success
```

---

## Deployment Standards

### 1. Environment Management

**Environments:**
- **Development:** Local development machines
- **Staging:** Pre-production testing
- **Production:** Live deployment (GitHub Pages)

**Configuration:**
- Use environment variables for configuration
- Never commit secrets to repository
- Use `.env` files locally (gitignored)
- Use environment-specific configs

### 2. Deployment Process

**Frontend (Docusaurus):**
```bash
# Build
npm run build

# Test locally
npm run serve

# Deploy to GitHub Pages
npm run deploy
```

**Backend (FastAPI):**
```bash
# Install dependencies
pip install -r requirements.txt

# Run locally
uvicorn app.main:app --reload

# Deploy to production
# (Follow platform-specific instructions)
```

### 3. Pre-Deployment Checklist

- [ ] All tests passing
- [ ] Build successful
- [ ] Configuration validated
- [ ] Environment variables set
- [ ] Database migrations applied
- [ ] Monitoring configured
- [ ] Backup created
- [ ] Rollback plan ready

---

## Security Standards

### 1. Secrets Management

**Requirements:**
- Never commit API keys, passwords, or tokens
- Use environment variables for secrets
- Use `.env.example` for documentation
- Rotate secrets regularly
- Use different secrets per environment

**Example (.env.example):**
```bash
# OpenAI Configuration
OPENAI_API_KEY=your_api_key_here

# Database Configuration
DATABASE_URL=postgresql://user:password@host:port/database

# Qdrant Configuration
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=your_qdrant_api_key
```

### 2. Input Validation

**Requirements:**
- Validate all user input
- Sanitize data before database insertion
- Use Pydantic models for validation
- Limit request sizes
- Implement rate limiting

**Example:**
```python
from pydantic import BaseModel, Field, validator

class SearchRequest(BaseModel):
    """Search request with validation."""

    query: str = Field(..., min_length=1, max_length=500)
    top_k: int = Field(default=10, ge=1, le=100)

    @validator('query')
    def validate_query(cls, v):
        """Ensure query is not just whitespace."""
        if not v.strip():
            raise ValueError("Query cannot be empty")
        return v.strip()
```

### 3. API Security

**Requirements:**
- Use HTTPS in production
- Implement CORS properly
- Add rate limiting
- Validate authentication tokens
- Log security events
- Sanitize error messages (no stack traces to users)

---

## Performance Standards

### 1. Response Time Targets

**API Endpoints:**
- Health check: < 100ms
- Search: < 500ms
- Question answering: < 2s
- Ingestion: < 5s per document

### 2. Optimization Guidelines

**Backend:**
- Use async operations for I/O
- Implement caching where appropriate
- Optimize database queries
- Use connection pooling
- Batch operations when possible

**Frontend:**
- Optimize images and assets
- Lazy load components
- Minimize bundle size
- Use CDN for static assets

### 3. Monitoring

**Metrics to Track:**
- API response times
- Error rates
- Database query performance
- Memory usage
- Concurrent users

---

## Enforcement and Compliance

### 1. Automated Checks

**Pre-commit Hooks:**
- Code formatting (black, prettier)
- Linting (flake8, eslint)
- Type checking (mypy)
- Test execution

**CI/CD Pipeline:**
- Run all tests
- Check code coverage
- Build verification
- Security scanning

### 2. Manual Review

**Required for:**
- All pull requests
- Major architecture changes
- New features
- Security-related changes

### 3. Exceptions

**Process:**
- Document reason for exception
- Get approval from project lead
- Add TODO for future compliance
- Track in issues

---

## Continuous Improvement

This constitution is a living document and should evolve with the project.

**Review Process:**
- Quarterly review of standards
- Update based on lessons learned
- Incorporate community feedback
- Align with industry best practices

**Proposing Changes:**
1. Create issue describing proposed change
2. Discuss rationale and impact
3. Update constitution
4. Announce to team
5. Update related documentation

---

## Acknowledgments

This constitution was created following the **spec-driven AI-native development** methodology using:
- **Spec-Kit Plus** for structured development
- **Claude Code** for AI-assisted content generation
- **Industry best practices** for software engineering

---

**Document Version:** 1.0
**Effective Date:** 2025-12-29
**Next Review:** 2026-03-29
