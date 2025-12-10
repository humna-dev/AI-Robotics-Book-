# Data Model: Physical AI & Humanoid Robotics Textbook

## Core Entities

### Chapter/Module
**Description**: Represents a major section of the textbook
- **id**: Unique identifier for the chapter
- **title**: Title of the chapter (e.g., "The Robotic Nervous System")
- **moduleNumber**: Sequential number of the module (1-5)
- **description**: Brief overview of the chapter content
- **topics**: Array of topic IDs belonging to this chapter
- **order**: Display order in the textbook
- **metadata**: Additional information (author, last updated, etc.)

### Topic
**Description**: Represents a specific topic within a chapter
- **id**: Unique identifier for the topic
- **title**: Title of the topic (e.g., "ROS 2 Nodes & Topics")
- **chapterId**: Reference to the parent chapter
- **content**: The actual content in MDX format
- **order**: Display order within the chapter
- **prerequisites**: Array of prerequisite topic IDs
- **relatedTopics**: Array of related topic IDs

### User
**Description**: Represents a student or reader using the textbook
- **id**: Unique identifier for the user
- **email**: User's email address (optional for basic access)
- **preferences**: User preferences (theme, language, personalization settings)
- **progress**: Tracking of user's progress through the textbook
- **createdAt**: Timestamp when user account was created
- **lastAccessed**: Timestamp of last access

### UserProgress
**Description**: Tracks a user's progress through the textbook
- **id**: Unique identifier for the progress record
- **userId**: Reference to the user
- **chapterId**: Reference to the chapter
- **topicId**: Reference to the topic
- **completed**: Boolean indicating if the topic is completed
- **lastReadAt**: Timestamp of last reading session
- **timeSpent**: Time spent reading in seconds
- **notes**: User's personal notes on the topic

### ChatSession
**Description**: Represents a session with the RAG chatbot
- **id**: Unique identifier for the chat session
- **userId**: Reference to the user (optional for anonymous users)
- **sessionId**: Session identifier for tracking conversation
- **messages**: Array of messages in the conversation
- **createdAt**: Timestamp when session was created
- **lastActiveAt**: Timestamp of last message in session

### ChatMessage
**Description**: Represents a single message in a chat session
- **id**: Unique identifier for the message
- **sessionId**: Reference to the chat session
- **sender**: "user" or "bot"
- **content**: The actual message content
- **timestamp**: When the message was sent
- **context**: Relevant textbook content that was selected
- **sourceChunks**: Vector database chunks used to generate the response

### Translation
**Description**: Stores translated content for multilingual support
- **id**: Unique identifier for the translation
- **contentId**: Reference to the original content (chapter/topic)
- **language**: Language code (e.g., "ur", "en")
- **translatedContent**: The translated content
- **status**: "pending", "approved", "rejected"
- **createdAt**: When the translation was created
- **lastUpdated**: When the translation was last updated

## Relationships

### Chapter-Topic Relationship
- One Chapter contains many Topics
- Each Topic belongs to exactly one Chapter
- Relationship: Chapter (1) → (0..*) Topic

### User-UserProgress Relationship
- One User has many UserProgress records
- Each UserProgress record belongs to exactly one User
- Relationship: User (1) → (0..*) UserProgress

### ChatSession-ChatMessage Relationship
- One ChatSession contains many ChatMessages
- Each ChatMessage belongs to exactly one ChatSession
- Relationship: ChatSession (1) → (0..*) ChatMessage

## Validation Rules

### Chapter Validation
- Title must be between 5-100 characters
- Module number must be between 1-5
- Order must be a positive integer
- Topics array cannot be empty

### Topic Validation
- Title must be between 5-100 characters
- Content must be in valid MDX format
- Order must be a positive integer
- ChapterId must reference an existing chapter

### User Validation
- Email must be valid if provided
- Preferences must be a valid JSON object
- Cannot have duplicate email addresses

### UserProgress Validation
- Each user-topic combination must be unique
- TimeSpent must be a non-negative number
- Completed status must be boolean

### ChatMessage Validation
- Content must not exceed 10,000 characters
- Sender must be either "user" or "bot"
- Timestamp must be current or past

## State Transitions

### UserProgress States
- `not_started` → `in_progress` → `completed`
- Can transition back from `completed` to `in_progress` if user revisits content

### Translation States
- `pending` → `approved` or `rejected`
- `rejected` → `pending` (for revisions)
- `approved` → `pending` (for updates)

## Indexes for Performance

### Required Indexes
- Chapter: index on `moduleNumber` and `order`
- Topic: index on `chapterId` and `order`
- UserProgress: composite index on `userId` and `topicId`
- ChatSession: index on `userId` and `lastActiveAt`
- ChatMessage: index on `sessionId` and `timestamp`
- Translation: composite index on `contentId` and `language`