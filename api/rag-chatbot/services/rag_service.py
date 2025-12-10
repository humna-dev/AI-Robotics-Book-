import logging
from typing import List, Dict, Any, Optional
from services.openai_service import openai_service
from database.vector_db import vector_db_service
from models.chat_models import Document, SearchRequest, SearchResponse

logger = logging.getLogger(__name__)

class RAGService:
    def __init__(self):
        self.openai_service = openai_service
        self.vector_db = vector_db_service

    async def retrieve_context(self, query: str, top_k: int = 3) -> List[Dict[str, Any]]:
        """
        Retrieve relevant context from the vector database based on the query
        """
        try:
            # Create embedding for the query
            query_embedding = await self.openai_service.embed_text(query)

            # Search for similar documents
            search_results = await self.vector_db.search_similar(
                query_embedding=query_embedding,
                limit=top_k
            )

            logger.info(f"Retrieved {len(search_results)} context documents")
            return search_results

        except Exception as e:
            logger.error(f"Error retrieving context: {str(e)}")
            return []

    async def generate_response(self, query: str, context: List[Dict[str, Any]], selected_text: str = None) -> str:
        """
        Generate a response using the context and query
        """
        try:
            # Format context for the LLM
            context_str = ""
            sources = []

            for doc in context:
                content = doc.get("content", "")
                metadata = doc.get("metadata", {})
                source = metadata.get("source", "Unknown source")

                context_str += f"Source: {source}\nContent: {content}\n\n"
                sources.append(source)

            # Prepare messages for the LLM
            messages = [
                {
                    "role": "system",
                    "content": f"You are an AI assistant for the Physical AI & Humanoid Robotics textbook. Answer questions based on the provided context. If the context doesn't contain relevant information, say so. Be concise and helpful."
                }
            ]

            # Add selected text context if provided
            if selected_text:
                messages.append({
                    "role": "user",
                    "content": f"Selected text: {selected_text}\n\nQuestion: {query}"
                })
            else:
                messages.append({
                    "role": "user",
                    "content": f"Context: {context_str}\n\nQuestion: {query}"
                })

            # Get response from OpenAI
            response = await self.openai_service.get_completion(messages, context_str)

            logger.info(f"Generated response for query: {query[:50]}...")
            return response

        except Exception as e:
            logger.error(f"Error generating response: {str(e)}")
            return "I encountered an error while processing your request. Please try again."

    async def process_query(self, query: str, selected_text: str = None, top_k: int = 3) -> Dict[str, Any]:
        """
        Process a query through the full RAG pipeline
        """
        try:
            # Retrieve relevant context
            context = await self.retrieve_context(query, top_k)

            # Generate response based on context
            response = await self.generate_response(query, context, selected_text)

            return {
                "response": response,
                "context": context,
                "query": query,
                "selected_text": selected_text
            }

        except Exception as e:
            logger.error(f"Error processing query: {str(e)}")
            return {
                "response": "I encountered an error while processing your request. Please try again.",
                "context": [],
                "query": query,
                "selected_text": selected_text
            }

    async def add_document_to_kb(self, doc_id: str, content: str, metadata: Dict[str, Any] = None) -> bool:
        """
        Add a document to the knowledge base
        """
        try:
            success = await self.vector_db.add_document(doc_id, content, metadata)
            if success:
                logger.info(f"Successfully added document {doc_id} to knowledge base")
            else:
                logger.error(f"Failed to add document {doc_id} to knowledge base")
            return success

        except Exception as e:
            logger.error(f"Error adding document to knowledge base: {str(e)}")
            return False

rag_service = RAGService()