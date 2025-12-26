# Data Model: Chatbot Docusaurus Integration

## Core Entities

### ChatMessage
**Description**: Represents a single message in the chat conversation

**Fields**:
- `id` (string, required): Unique identifier for the message
- `content` (string, required): The text content of the message
- `sender` (string, required): Who sent the message ("user" or "bot")
- `timestamp` (string, required): ISO 8601 timestamp of when message was created
- `sources` (array[SourceCitation], optional): Source citations for bot responses

**Validation Rules**:
- `id` must be unique within the conversation
- `content` must be 1-5000 characters
- `sender` must be either "user" or "bot"
- `timestamp` must be valid ISO 8601 format

### SourceCitation
**Description**: Represents a citation to source material in the textbook

**Fields**:
- `text` (string, required): The cited text content
- `sourceUrl` (string, required): URL to the source location
- `chapterTitle` (string, required): Title of the chapter containing the source
- `score` (number, required): Relevance score from the vector search (0.0-1.0)

**Validation Rules**:
- `text` must be 1-10000 characters
- `sourceUrl` must be a valid URL
- `score` must be between 0.0-1.0
- All fields must be present

### ChatSession
**Description**: Represents a user's chat session with conversation history

**Fields**:
- `sessionId` (string, required): Unique identifier for the session
- `messages` (array[ChatMessage], required): Array of messages in the conversation
- `createdAt` (string, required): ISO 8601 timestamp of session creation
- `lastActiveAt` (string, required): ISO 8601 timestamp of last activity
- `currentPageUrl` (string, optional): URL of the current page in the session context

**Validation Rules**:
- `sessionId` must be unique
- `messages` must be an array of valid ChatMessage objects
- `createdAt` and `lastActiveAt` must be valid ISO 8601 timestamps
- `currentPageUrl` must be a valid URL if provided

### QueryRequest
**Description**: Represents a user query request to the backend API

**Fields**:
- `query` (string, required): The user's natural language query
- `selectedText` (string, optional): Additional context from selected text on the page
- `currentPageUrl` (string, optional): URL of the current page for context
- `chapterTitle` (string, optional): Title of the current chapter for context
- `topK` (integer, optional): Number of results to retrieve (default: 5, min: 1, max: 20)

**Validation Rules**:
- `query` must be 1-1000 characters
- `selectedText` must be 0-5000 characters if provided
- `topK` must be between 1-20 if provided
- `currentPageUrl` must be a valid URL if provided

### QueryResponse
**Description**: Response from the backend API to a query request

**Fields**:
- `query` (string, required): Echo of the original user query
- `response` (string, required): The agent's response to the query
- `contextUsed` (array[SourceCitation], required): Retrieved chunks used to generate the response
- `threadId` (string, optional): Thread identifier for conversation continuity
- `executionTimeMs` (number, required): Time taken to process the query in milliseconds
- `timestamp` (string, required): ISO 8601 timestamp of the response
- `modelUsed` (string, required): The LLM model used for generation
- `error` (string, optional): Error message if the query failed

**Validation Rules**:
- `response` must be 1-10000 characters
- `contextUsed` must be an array of valid SourceCitation objects
- `executionTimeMs` must be non-negative
- If `error` is present, the response represents a failed query

### TextSelection
**Description**: Represents selected text on a page with positioning information

**Fields**:
- `text` (string, required): The selected text content
- `rect` (object, required): Rectangle coordinates of the selection
  - `top` (number): Top coordinate relative to viewport
  - `left` (number): Left coordinate relative to viewport
  - `width` (number): Width of the selection
  - `height` (number): Height of the selection
- `pageUrl` (string, required): URL of the page where text was selected
- `elementId` (string, optional): ID of the element containing the selection

**Validation Rules**:
- `text` must be 1-5000 characters
- `rect` must have valid numeric values for all coordinates
- `pageUrl` must be a valid URL
- `text` must not be empty

## State Transitions

### ChatMessage Lifecycle
1. **Created**: User types message and clicks send
2. **Processing**: Message appears with loading indicator while waiting for response
3. **Responded**: Bot response is received and added to conversation
4. **Cited**: Source citations are displayed with the bot response

### ChatSession Lifecycle
1. **Started**: User opens chat interface for first time on site
2. **Active**: User engages in conversation with chatbot
3. **Paused**: User closes chat interface but session remains in localStorage
4. **Resumed**: User reopens chat and continues previous conversation
5. **Expired**: Session data is cleared after inactivity period

## Relationships

- One `ChatSession` contains many `ChatMessage` objects
- One `ChatMessage` (from bot) may reference many `SourceCitation` objects
- One `QueryRequest` generates one `QueryResponse`
- One `TextSelection` triggers one `QueryRequest` (when using selected text)

## API Data Flow

### Request Flow
```
User Action (typed query or selected text)
→ QueryRequest object
→ Backend API
→ RAG Processing
→ QueryResponse object
→ ChatMessage objects (user + bot)
```

### Component State Flow
```
TextSelection detected
→ AskButton appears
→ User clicks button
→ ChatWindow opens with pre-filled text
→ User submits query
→ QueryRequest sent to API
→ QueryResponse received
→ ChatMessages updated in UI
```

## Storage Model

### Client-Side Session Storage
- `sessionId`: Randomly generated session identifier
- `messages`: Array of ChatMessage objects serialized as JSON
- `lastActive`: Timestamp of last user interaction
- `currentPageUrl`: Current page URL for context preservation

### Validation Constraints
- Session data must be encrypted before storage
- Session expiration after 24 hours of inactivity
- Maximum storage size of 10MB per session
- Automatic cleanup of expired sessions