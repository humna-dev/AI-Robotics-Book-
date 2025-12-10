import os
from typing import List, Dict, Any, Optional
from qdrant_client import QdrantClient
from qdrant_client.http import models
from qdrant_client.http.models import Distance, VectorParams
import logging

logger = logging.getLogger(__name__)

class VectorDBService:
    def __init__(self):
        # Get Qdrant configuration from environment
        qdrant_url = os.getenv("QDRANT_URL", "http://localhost:6333")
        qdrant_api_key = os.getenv("QDRANT_API_KEY")

        # Initialize Qdrant client
        if qdrant_api_key:
            self.client = QdrantClient(url=qdrant_url, api_key=qdrant_api_key)
        else:
            self.client = QdrantClient(url=qdrant_url)

        # Collection name for textbook content
        self.collection_name = "textbook_content"

        # Initialize the collection
        self._initialize_collection()

    def _initialize_collection(self):
        """Initialize the Qdrant collection if it doesn't exist"""
        try:
            # Check if collection exists
            collections = self.client.get_collections()
            collection_names = [c.name for c in collections.collections]

            if self.collection_name not in collection_names:
                # Create collection with 1536 dimensions (for OpenAI embeddings)
                self.client.create_collection(
                    collection_name=self.collection_name,
                    vectors_config=VectorParams(size=1536, distance=Distance.COSINE),
                )
                logger.info(f"Created collection: {self.collection_name}")
            else:
                logger.info(f"Collection {self.collection_name} already exists")

        except Exception as e:
            logger.error(f"Error initializing collection: {str(e)}")
            raise

    async def add_document(self, doc_id: str, content: str, metadata: Dict[str, Any] = None) -> bool:
        """Add a document to the vector database"""
        try:
            # Create embedding for the content
            # In a real implementation, we'd call the OpenAI service to create embedding
            # For now, we'll simulate this with a placeholder
            # embedding = await openai_service.embed_text(content)
            # For this example, we'll use a mock embedding
            embedding = [0.1] * 1536  # Placeholder embedding

            # Prepare the record
            record = models.PointStruct(
                id=doc_id,
                vector=embedding,
                payload={
                    "content": content,
                    "metadata": metadata or {}
                }
            )

            # Upsert the record
            self.client.upsert(
                collection_name=self.collection_name,
                points=[record]
            )

            logger.info(f"Added document {doc_id} to vector database")
            return True

        except Exception as e:
            logger.error(f"Error adding document {doc_id}: {str(e)}")
            return False

    async def search_similar(self, query_embedding: List[float], limit: int = 5) -> List[Dict[str, Any]]:
        """Search for similar documents to the query"""
        try:
            # Search for similar vectors
            search_results = self.client.search(
                collection_name=self.collection_name,
                query_vector=query_embedding,
                limit=limit,
                with_payload=True
            )

            # Format results
            results = []
            for hit in search_results:
                results.append({
                    "id": hit.id,
                    "content": hit.payload.get("content", ""),
                    "metadata": hit.payload.get("metadata", {}),
                    "score": hit.score
                })

            logger.info(f"Found {len(results)} similar documents")
            return results

        except Exception as e:
            logger.error(f"Error searching vector database: {str(e)}")
            return []

    async def get_document(self, doc_id: str) -> Optional[Dict[str, Any]]:
        """Get a specific document by ID"""
        try:
            records = self.client.retrieve(
                collection_name=self.collection_name,
                ids=[doc_id],
                with_payload=True
            )

            if records:
                record = records[0]
                return {
                    "id": record.id,
                    "content": record.payload.get("content", ""),
                    "metadata": record.payload.get("metadata", {})
                }

            return None

        except Exception as e:
            logger.error(f"Error retrieving document {doc_id}: {str(e)}")
            return None

    async def delete_document(self, doc_id: str) -> bool:
        """Delete a document by ID"""
        try:
            self.client.delete(
                collection_name=self.collection_name,
                points_selector=models.PointIdsList(points=[doc_id])
            )
            logger.info(f"Deleted document {doc_id}")
            return True
        except Exception as e:
            logger.error(f"Error deleting document {doc_id}: {str(e)}")
            return False

vector_db_service = VectorDBService()