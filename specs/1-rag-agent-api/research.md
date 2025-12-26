# Research: OpenAI Agents SDK Integration

## Decision: OpenAI Assistants API for RAG Agent Implementation

### Rationale:
The implementation uses the OpenAI Assistants API rather than a specific "Agents SDK" because:
1. OpenAI's current recommended approach for building AI agents is through the Assistants API
2. The Assistants API provides the necessary tools for creating conversational agents
3. It allows for dynamic context injection, which is essential for the RAG system
4. It supports various models and provides good integration with the OpenAI ecosystem

### Implementation Approach:
- Use the OpenAI Chat Completions API as the primary interface since the Assistants API requires pre-created assistants
- Integrate with the existing Qdrant retrieval system in `retrieve.py`
- Inject retrieved context as part of the prompt to ensure responses are based only on book content
- Implement proper error handling and logging

## Decision: Integration with Existing Retrieval System

### Rationale:
Rather than duplicating retrieval logic, the agent integrates directly with the existing `retrieve.py` module because:
1. Maintains consistency with the existing codebase
2. Reuses battle-tested retrieval logic
3. Reduces code duplication
4. Leverages existing configuration and error handling

## Decision: Environment Variables Structure

### Rationale:
The environment variables were updated to include OpenAI-specific configurations:
1. `OPENAI_API_KEY` - Required for accessing OpenAI services
2. `OPENAI_ASSISTANT_ID` - For future use with the Assistants API
3. `OPENAI_MODEL` - Allows model selection flexibility
4. `AGENT_TEMPERATURE` - Controls response creativity
5. `RETRIEVAL_TOP_K` - Configures number of results to retrieve

## Decision: Response Validation and Citations

### Rationale:
The agent ensures responses are based only on retrieved content by:
1. Including a system prompt that explicitly restricts responses to provided context
2. Formatting retrieved chunks in a clear, structured format for the LLM
3. Including source citations in responses
4. Implementing error handling when retrieval fails