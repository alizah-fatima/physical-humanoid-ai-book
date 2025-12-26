"""
Prompt templates for RAG Agent
"""
from typing import List, Dict, Any


class PromptTemplate:
    """Base class for prompt templates"""

    def format(self, **kwargs) -> str:
        """Format the prompt with the given arguments"""
        raise NotImplementedError


class RAGPromptTemplate(PromptTemplate):
    """Template for RAG-specific prompts"""

    SYSTEM_PROMPT = """You are an AI assistant for the Physical AI & Humanoid Robotics textbook. Your responses must be based ONLY on the provided book content. Do not use any external knowledge or make up information. If the provided context doesn't contain enough information to answer the question, clearly state that the information is not available in the provided content.

Guidelines:
1. Answer based solely on the provided context
2. Cite specific chapters or sections when possible
3. If uncertain, say you don't have enough information from the provided content
4. Maintain academic accuracy and clarity
5. Provide concise, helpful responses"""

    USER_PROMPT_TEMPLATE = """USER QUERY: {query}

CONTEXT FROM BOOK:
{context}"""

    def format(self, query: str, context: str) -> str:
        """
        Format a RAG-specific prompt that constrains the LLM to use only provided context.

        Args:
            query: The original user query
            context: Formatted context from the retrieval system

        Returns:
            Complete prompt for the agent
        """
        user_prompt = self.USER_PROMPT_TEMPLATE.format(query=query, context=context)
        return f"{self.SYSTEM_PROMPT}\n\n{user_prompt}"


class CitationPromptTemplate(PromptTemplate):
    """Template for citation-related prompts"""

    CITATION_INSTRUCTION = """When referencing information from the provided context, clearly cite the source using the format: [Chapter: {chapter_title}, Source: {source_url}]."""

    def format(self, query: str, context: str) -> str:
        """
        Format a prompt with citation instructions.

        Args:
            query: The original user query
            context: Formatted context from the retrieval system

        Returns:
            Complete prompt with citation instructions
        """
        base_prompt = RAGPromptTemplate().format(query=query, context=context)
        return f"{base_prompt}\n\n{self.CITATION_INSTRUCTION}"


class ValidationPromptTemplate(PromptTemplate):
    """Template for validation-related prompts"""

    VALIDATION_INSTRUCTION = """Verify that your response is based solely on the provided context. Do not include any information not present in the provided context."""

    def format(self, query: str, context: str) -> str:
        """
        Format a prompt with validation instructions.

        Args:
            query: The original user query
            context: Formatted context from the retrieval system

        Returns:
            Complete prompt with validation instructions
        """
        base_prompt = RAGPromptTemplate().format(query=query, context=context)
        return f"{base_prompt}\n\n{self.VALIDATION_INSTRUCTION}"


# Default instance
rag_prompt_template = RAGPromptTemplate()