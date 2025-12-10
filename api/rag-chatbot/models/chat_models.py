from pydantic import BaseModel, Field
from typing import List, Optional, Dict, Any
from datetime import datetime

class Message(BaseModel):
    """Represents a single message in a chat"""
    role: str = Field(..., description="Role of the message sender (user or assistant)")
    content: str = Field(..., description="Content of the message")
    timestamp: datetime = Field(default_factory=datetime.utcnow, description="Timestamp of the message")


class ChatSession(BaseModel):
    """Represents a chat session"""
    session_id: str = Field(..., description="Unique identifier for the session")
    user_id: Optional[str] = Field(None, description="ID of the user (optional for anonymous chats)")
    messages: List[Message] = Field(default_factory=list, description="List of messages in the session")
    created_at: datetime = Field(default_factory=datetime.utcnow, description="When the session was created")
    last_active: datetime = Field(default_factory=datetime.utcnow, description="When the session was last active")
    metadata: Dict[str, Any] = Field(default_factory=dict, description="Additional metadata")


class ChatRequest(BaseModel):
    """Request model for chat endpoint"""
    session_id: str = Field(..., description="Session identifier")
    message: str = Field(..., description="User message content")
    selected_text: Optional[str] = Field(None, description="Text selected by user in the textbook")
    context: Optional[str] = Field(None, description="Additional context from the textbook")
    user_id: Optional[str] = Field(None, description="User identifier")


class ChatResponse(BaseModel):
    """Response model for chat endpoint"""
    session_id: str = Field(..., description="Session identifier")
    response: str = Field(..., description="AI response content")
    sources: List[str] = Field(default_factory=list, description="Sources used in the response")
    timestamp: datetime = Field(default_factory=datetime.utcnow, description="Response timestamp")


class Document(BaseModel):
    """Model for documents stored in the vector database"""
    id: str = Field(..., description="Unique document identifier")
    content: str = Field(..., description="Document content")
    metadata: Dict[str, Any] = Field(default_factory=dict, description="Document metadata including source location")
    embedding: Optional[List[float]] = Field(None, description="Embedding vector")


class SearchRequest(BaseModel):
    """Request model for search endpoint"""
    query: str = Field(..., description="Search query")
    limit: int = Field(default=5, description="Maximum number of results to return")


class SearchResponse(BaseModel):
    """Response model for search endpoint"""
    query: str = Field(..., description="Original search query")
    results: List[Dict[str, Any]] = Field(..., description="Search results")
    count: int = Field(..., description="Number of results returned")