import openai
import os
from typing import List, Dict, Any
import logging

logger = logging.getLogger(__name__)

class OpenAIService:
    def __init__(self):
        # Set OpenAI API key from environment variable
        api_key = os.getenv("OPENAI_API_KEY")
        if not api_key:
            raise ValueError("OPENAI_API_KEY environment variable is not set")

        openai.api_key = api_key
        self.model = os.getenv("OPENAI_MODEL", "gpt-3.5-turbo")

    async def get_completion(self, messages: List[Dict[str, str]], context: str = "") -> str:
        """
        Get completion from OpenAI API with context
        """
        try:
            # Prepend context to the messages if provided
            enhanced_messages = []
            if context:
                enhanced_messages.append({
                    "role": "system",
                    "content": f"Context information: {context}\n\nAnswer the user's questions based on the provided context. If the context doesn't contain relevant information, say so."
                })

            enhanced_messages.extend(messages)

            response = await openai.ChatCompletion.acreate(
                model=self.model,
                messages=enhanced_messages,
                temperature=0.7,
                max_tokens=500,
                timeout=30
            )

            return response.choices[0].message['content'].strip()

        except openai.error.RateLimitError:
            logger.error("OpenAI rate limit exceeded")
            raise Exception("Rate limit exceeded. Please try again later.")
        except openai.error.AuthenticationError:
            logger.error("OpenAI authentication failed")
            raise Exception("Authentication failed. Check your API key.")
        except Exception as e:
            logger.error(f"Error calling OpenAI API: {str(e)}")
            raise Exception(f"Error calling OpenAI API: {str(e)}")

    async def embed_text(self, text: str) -> List[float]:
        """
        Create embedding for text using OpenAI
        """
        try:
            response = await openai.Embedding.acreate(
                input=text,
                model="text-embedding-ada-002"
            )

            return response.data[0].embedding

        except Exception as e:
            logger.error(f"Error creating embedding: {str(e)}")
            raise Exception(f"Error creating embedding: {str(e)}")

openai_service = OpenAIService()