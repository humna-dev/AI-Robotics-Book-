from fastapi import FastAPI, HTTPException
from pydantic import BaseModel
from typing import List, Optional, Dict, Any
import os
import uvicorn
from models.chat_models import ChatRequest, ChatResponse, SearchRequest, SearchResponse
from services.rag_service import rag_service
from services.openai_service import openai_service
from database.vector_db import vector_db_service

app = FastAPI(title="RAG Chatbot API", version="1.0.0")

@app.get("/")
def read_root():
    return {"message": "RAG Chatbot API is running"}

@app.post("/chat", response_model=ChatResponse)
async def chat(request: ChatRequest):
    """
    Chat endpoint that processes user messages with RAG context
    """
    try:
        # Process the query through the RAG pipeline
        result = await rag_service.process_query(
            query=request.message,
            selected_text=request.selected_text
        )

        # Create response
        response = ChatResponse(
            session_id=request.session_id,
            response=result["response"],
            sources=[doc.get("metadata", {}).get("source", "Unknown") for doc in result["context"]],
            timestamp=result.get("timestamp")
        )

        return response
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error processing chat request: {str(e)}")

@app.post("/search")
async def search(request: SearchRequest):
    """
    Search endpoint for finding relevant content in the textbook
    """
    try:
        # Create embedding for the query
        query_embedding = await openai_service.embed_text(request.query)

        # Search for similar documents
        search_results = await vector_db_service.search_similar(
            query_embedding=query_embedding,
            limit=request.limit
        )

        response = SearchResponse(
            query=request.query,
            results=search_results,
            count=len(search_results)
        )

        return response
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error processing search request: {str(e)}")

@app.post("/translate")
async def translate(request: ChatRequest):
    """
    Translation endpoint for content localization
    """
    try:
        # In a real implementation, this would use a translation API like Google Translate, etc.
        # For now, returning the original text as a mock
        if request.context and 'ur' in request.context.lower():
            # Mock Urdu translation
            mock_urdu = "یہ اردو میں ترجمہ ہے" if request.message else request.message
            return {"translated_text": mock_urdu}
        else:
            return {"translated_text": request.message}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error translating text: {str(e)}")

@app.get("/health")
async def health_check():
    """
    Health check endpoint
    """
    return {"status": "healthy", "service": "rag-chatbot-api"}

@app.post("/ingest")
async def ingest_document(doc_id: str, content: str, metadata: Dict[str, Any] = None):
    """
    Ingest endpoint for adding documents to the knowledge base
    """
    try:
        success = await rag_service.add_document_to_kb(doc_id, content, metadata)
        if success:
            return {"status": "success", "message": f"Document {doc_id} added successfully"}
        else:
            raise HTTPException(status_code=500, detail=f"Failed to add document {doc_id}")
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error ingesting document: {str(e)}")

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)