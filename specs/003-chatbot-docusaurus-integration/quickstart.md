# Quickstart Guide: Chatbot Docusaurus Integration

## Overview
This guide provides instructions for getting started with the RAG chatbot integration in the Docusaurus textbook site.

## Implementation Overview

### 1. Component Structure
The chatbot is implemented as a single component in `src/components/AIChatbot/index.js` that includes:
- Floating chat button in bottom-right corner
- Expandable chat window with message history
- Text selection detection with floating "Ask AI" button
- Source citation display with clickable links
- Responsive design for all screen sizes

### 2. Docusaurus Integration
The chatbot is integrated globally using Docusaurus' theme system:
- Created custom Layout component at `src/theme/Layout.js`
- Component automatically appears on all pages
- Works with Docusaurus' dark/light mode

## Backend Setup

### 1. Install Backend Dependencies
```bash
cd backend
pip install -r requirements.txt
```

### 2. Configure Environment Variables
Create a `.env` file in the backend directory with the following variables:
```env
OPENROUTER_API_KEY=your_openrouter_api_key
QDRANT_URL=your_qdrant_url
QDRANT_API_KEY=your_qdrant_api_key
COHERE_API_KEY=your_cohere_api_key
OPENAI_MODEL=your_preferred_model
```

### 3. Start the Backend API
```bash
cd backend
python -m api.main
```
The API will be available at `http://localhost:8000`

## Frontend Setup

### 1. Install Frontend Dependencies
```bash
npm install
```

### 2. Start the Docusaurus Development Server
```bash
npm start
```

## Using the Chatbot

### 1. Accessing the Chat Interface
- Look for the floating chat button in the bottom-right corner of any page
- Click the button to open the chat interface
- Type your question about the textbook content and press Enter or click Send

### 2. Text Selection Feature
- Select any text on a page
- A floating "Ask AI" button will appear near your selection
- Click the button to open the chat with the selected text pre-filled
- Ask questions specifically about the selected content

### 3. Viewing Sources
- Responses include source citations with clickable links
- Click on source links to navigate to the referenced content
- Relevance scores are shown for each source

## API Endpoints

### Query Endpoint
- **POST** `/api/v1/agent/query`
- Sends a query to the RAG agent
- Returns response with source citations

### Health Check
- **GET** `/api/v1/agent/health`
- Checks the health of the RAG agent

### Configuration
- **GET** `/api/v1/agent/config`
- Gets current agent configuration

## Troubleshooting

### Common Issues
1. **Chatbot not appearing**: Make sure you're using the latest version of the code and have restarted the development server
2. **API connection errors**: Verify backend API is running and environment variables are properly configured
3. **Text selection not working**: Check browser compatibility and ensure JavaScript is enabled
4. **CORS issues**: Ensure backend API allows requests from frontend origin