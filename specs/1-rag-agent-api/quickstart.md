# Quickstart: RAG Agent API

## Setup

### Prerequisites
- Python 3.11+
- Access to OpenAI API
- Access to Cohere API
- Qdrant vector database running

### Installation

1. Clone the repository:
```bash
git clone <repository-url>
cd physical-humanoid-ai-book
```

2. Navigate to backend:
```bash
cd backend
```

3. Install dependencies:
```bash
pip install -r requirements.txt
```

### Environment Configuration

Create a `.env` file based on `.env.example`:

```bash
cp .env.example .env
```

Update the `.env` file with your API keys:

```env
OPENAI_API_KEY="your-openai-api-key-here"
COHERE_API_KEY="your-cohere-api-key-here"
QDRANT_HOST=localhost
QDRANT_PORT=6333
OPENAI_MODEL=gpt-4-turbo
```

## Usage

### 1. Setup the Agent (One-time)

Create the OpenAI assistant:

```bash
python agent.py --setup
```

This will output an assistant ID. Update your `.env` file with:

```env
OPENAI_ASSISTANT_ID="assistant-id-from-above"
```

### 2. Query the Agent

Process a query using the RAG agent:

```bash
python agent.py --query "Explain ROS 2 architecture"
```

### 3. Using the Agent Programmatically

```python
from agent import RAGAgent

# Initialize the agent
agent = RAGAgent()

# Process a query
result = agent.query_agent("Explain ROS 2 architecture")
print(result['response'])
```

## API Integration

To integrate with a web API, you can use the RAGAgent class:

```python
from flask import Flask, request, jsonify
from agent import RAGAgent

app = Flask(__name__)
rag_agent = RAGAgent()

@app.route('/api/agent/query', methods=['POST'])
def query_agent():
    data = request.json
    query = data.get('query')

    result = rag_agent.query_agent(query)
    return jsonify(result)
```

## Testing

Run the retrieval tests to ensure the system works:

```bash
python test_retrieve.py
```

## Troubleshooting

### Common Issues

1. **API Key Errors**: Ensure all required API keys are set in the environment
2. **Qdrant Connection**: Verify Qdrant is running and accessible
3. **Retrieval Issues**: Check that the ingestion pipeline has populated the Qdrant database

### Verification Steps

1. Test retrieval independently:
```bash
python retrieve.py --query "test query"
```

2. Check environment variables:
```bash
python -c "import os; print('OpenAI:', bool(os.getenv('OPENAI_API_KEY'))); print('Cohere:', bool(os.getenv('COHERE_API_KEY')))"
```

## Next Steps

- Integrate the agent with your frontend application
- Implement rate limiting for production use
- Add monitoring and logging for production deployments
- Consider implementing streaming responses for better UX