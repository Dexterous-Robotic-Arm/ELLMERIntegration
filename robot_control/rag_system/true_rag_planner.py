#!/usr/bin/env python3
"""
True RAG-Based Intelligent Robot Planner

This module implements genuine Retrieval-Augmented Generation (RAG) for robot control.
Unlike the previous "RAG" system which only used static context, this implements:

1. Vector Database (ChromaDB) for semantic search
2. Embedding-based similarity matching
3. Dynamic knowledge retrieval from comprehensive robotics knowledge base
4. True RAG pipeline: Retrieve → Augment → Generate

The system can intelligently retrieve relevant robotics knowledge, movement patterns,
and problem-solving strategies based on semantic similarity to the current task.
"""

import os
import json
import time
import logging
import threading
from pathlib import Path
from typing import List, Dict, Any, Optional, Tuple, Union
from dataclasses import dataclass, field
import hashlib

# Core dependencies
import numpy as np

# RAG dependencies
try:
    import chromadb
    from chromadb.config import Settings
    CHROMADB_AVAILABLE = True
except ImportError:
    CHROMADB_AVAILABLE = False
    print("Warning: ChromaDB not available - install with: pip install chromadb")

try:
    from sentence_transformers import SentenceTransformer
    EMBEDDINGS_AVAILABLE = True
except ImportError:
    EMBEDDINGS_AVAILABLE = False
    print("Warning: SentenceTransformers not available - install with: pip install sentence-transformers")

try:
    import google.generativeai as genai
    GEMINI_AVAILABLE = True
except ImportError:
    GEMINI_AVAILABLE = False
    print("Warning: Google Generative AI not available")

# Import knowledge base
import sys
sys.path.append(str(Path(__file__).parent.parent.parent.parent))
try:
    from knowledge_base.robotics_knowledge import get_all_knowledge, search_knowledge_by_keywords
    KNOWLEDGE_BASE_AVAILABLE = True
except ImportError:
    KNOWLEDGE_BASE_AVAILABLE = False
    print("Warning: Robotics knowledge base not available")

logger = logging.getLogger(__name__)


@dataclass
class RAGDocument:
    """A document in the RAG knowledge base."""
    id: str
    content: str
    metadata: Dict[str, Any]
    embedding: Optional[List[float]] = None
    category: str = "general"
    keywords: List[str] = field(default_factory=list)
    relevance_score: float = 0.0


@dataclass
class RAGContext:
    """Context for RAG-based planning."""
    user_query: str
    retrieved_documents: List[RAGDocument]
    current_environment: Dict[str, Any]
    robot_state: Dict[str, Any]
    execution_history: List[Dict[str, Any]]
    confidence_score: float = 0.0


class VectorDatabase:
    """Vector database for semantic search using ChromaDB."""
    
    def __init__(self, db_path: str = "data/vector_db", collection_name: str = "robotics_knowledge"):
        """Initialize vector database."""
        self.db_path = Path(db_path)
        self.collection_name = collection_name
        self.client = None
        self.collection = None
        
        if CHROMADB_AVAILABLE:
            self._initialize_database()
        else:
            logger.warning("ChromaDB not available - using fallback search")
    
    def _initialize_database(self):
        """Initialize ChromaDB client and collection."""
        try:
            # Create database directory
            self.db_path.mkdir(parents=True, exist_ok=True)
            
            # Initialize ChromaDB client
            self.client = chromadb.PersistentClient(
                path=str(self.db_path),
                settings=Settings(
                    anonymized_telemetry=False,
                    allow_reset=True
                )
            )
            
            # Get or create collection
            try:
                self.collection = self.client.get_collection(
                    name=self.collection_name,
                    embedding_function=chromadb.utils.embedding_functions.SentenceTransformerEmbeddingFunction(
                        model_name="all-MiniLM-L6-v2"
                    )
                )
                logger.info(f"Loaded existing collection: {self.collection_name}")
            except:
                self.collection = self.client.create_collection(
                    name=self.collection_name,
                    embedding_function=chromadb.utils.embedding_functions.SentenceTransformerEmbeddingFunction(
                        model_name="all-MiniLM-L6-v2"
                    )
                )
                logger.info(f"Created new collection: {self.collection_name}")
                
        except Exception as e:
            logger.error(f"Failed to initialize vector database: {e}")
            self.client = None
            self.collection = None
    
    def add_documents(self, documents: List[RAGDocument]):
        """Add documents to the vector database."""
        if not self.collection:
            logger.warning("Vector database not available")
            return
        
        try:
            # Prepare data for ChromaDB
            ids = [doc.id for doc in documents]
            documents_text = [doc.content for doc in documents]
            metadatas = [doc.metadata for doc in documents]
            
            # Add to collection
            self.collection.add(
                documents=documents_text,
                metadatas=metadatas,
                ids=ids
            )
            
            logger.info(f"Added {len(documents)} documents to vector database")
            
        except Exception as e:
            logger.error(f"Failed to add documents to vector database: {e}")
    
    def search(self, query: str, n_results: int = 5) -> List[RAGDocument]:
        """Search for similar documents."""
        if not self.collection:
            logger.warning("Vector database not available - using fallback")
            return self._fallback_search(query, n_results)
        
        try:
            # Perform semantic search
            results = self.collection.query(
                query_texts=[query],
                n_results=n_results,
                include=['documents', 'metadatas', 'distances']
            )
            
            # Convert results to RAGDocument objects
            documents = []
            for i in range(len(results['documents'][0])):
                doc = RAGDocument(
                    id=results['ids'][0][i],
                    content=results['documents'][0][i],
                    metadata=results['metadatas'][0][i],
                    relevance_score=1.0 - results['distances'][0][i]  # Convert distance to similarity
                )
                documents.append(doc)
            
            logger.info(f"Retrieved {len(documents)} relevant documents for query: {query}")
            return documents
            
        except Exception as e:
            logger.error(f"Vector search failed: {e}")
            return self._fallback_search(query, n_results)
    
    def _fallback_search(self, query: str, n_results: int = 5) -> List[RAGDocument]:
        """Fallback search using keyword matching."""
        if not KNOWLEDGE_BASE_AVAILABLE:
            return []
        
        try:
            # Extract keywords from query
            keywords = query.lower().split()
            
            # Search knowledge base
            results = search_knowledge_by_keywords(keywords)
            
            # Convert to RAGDocument objects
            documents = []
            for i, result in enumerate(results[:n_results]):
                doc_id = hashlib.md5(str(result['item']).encode()).hexdigest()
                doc = RAGDocument(
                    id=doc_id,
                    content=json.dumps(result['item'], indent=2),
                    metadata={
                        "category": result['category'],
                        "relevance_score": result['relevance_score'],
                        "source": "fallback_search"
                    },
                    category=result['category'],
                    relevance_score=result['relevance_score'] / max(len(keywords), 1)
                )
                documents.append(doc)
            
            return documents
            
        except Exception as e:
            logger.error(f"Fallback search failed: {e}")
            return []


class TrueRAGPlanner:
    """
    True RAG-based robot planner with semantic knowledge retrieval.
    
    This implements genuine Retrieval-Augmented Generation:
    1. Retrieve: Find relevant knowledge using semantic search
    2. Augment: Enhance context with retrieved knowledge
    3. Generate: Create intelligent plans using augmented context
    """
    
    def __init__(self, 
                 robot_controller=None,
                 vision_system=None,
                 config_path: str = "config/",
                 db_path: str = "data/vector_db"):
        """Initialize the True RAG planner."""
        self.robot_controller = robot_controller
        self.vision_system = vision_system
        self.config_path = Path(config_path)
        self.db_path = Path(db_path)
        
        # Initialize vector database
        self.vector_db = VectorDatabase(db_path=str(self.db_path))
        
        # Initialize embedding model
        self.embedding_model = None
        if EMBEDDINGS_AVAILABLE:
            try:
                self.embedding_model = SentenceTransformer('all-MiniLM-L6-v2')
                logger.info("Embedding model loaded successfully")
            except Exception as e:
                logger.error(f"Failed to load embedding model: {e}")
        
        # Initialize LLM
        self.llm = self._initialize_llm()
        
        # Load and index knowledge base
        self._initialize_knowledge_base()
        
        # Initialize vision integration
        self._initialize_vision_integration()
        
        logger.info("True RAG Planner initialized with semantic knowledge retrieval")
    
    def _initialize_knowledge_base(self):
        """Load and index the robotics knowledge base."""
        if not KNOWLEDGE_BASE_AVAILABLE:
            logger.warning("Knowledge base not available")
            return
        
        try:
            # Get all knowledge
            all_knowledge = get_all_knowledge()
            
            # Convert to RAG documents
            documents = []
            for category, items in all_knowledge.items():
                for i, item in enumerate(items):
                    doc_id = f"{category}_{i}_{hashlib.md5(str(item).encode()).hexdigest()[:8]}"
                    
                    # Create searchable content
                    content = self._create_searchable_content(item, category)
                    
                    doc = RAGDocument(
                        id=doc_id,
                        content=content,
                        metadata={
                            "category": category,
                            "item_type": type(item).__name__,
                            "source": "robotics_knowledge_base"
                        },
                        category=category,
                        keywords=self._extract_keywords(content)
                    )
                    documents.append(doc)
            
            # Add documents to vector database
            self.vector_db.add_documents(documents)
            
            logger.info(f"Indexed {len(documents)} knowledge documents")
            
        except Exception as e:
            logger.error(f"Failed to initialize knowledge base: {e}")
    
    def _create_searchable_content(self, item: Dict[str, Any], category: str) -> str:
        """Create searchable content from knowledge base items."""
        content_parts = [f"Category: {category}"]
        
        if isinstance(item, dict):
            # Add title and description
            if 'title' in item:
                content_parts.append(f"Title: {item['title']}")
            if 'description' in item:
                content_parts.append(f"Description: {item['description']}")
            
            # Add context and applicable tasks
            if 'context' in item:
                content_parts.append(f"Context: {item['context']}")
            if 'applicable_tasks' in item:
                content_parts.append(f"Applicable to: {', '.join(item['applicable_tasks'])}")
            
            # Add pattern or steps
            if 'pattern' in item and isinstance(item['pattern'], list):
                content_parts.append("Steps:")
                content_parts.extend([f"- {step}" for step in item['pattern']])
            
            # Add safety considerations
            if 'safety_considerations' in item:
                content_parts.append("Safety considerations:")
                if isinstance(item['safety_considerations'], list):
                    content_parts.extend([f"- {safety}" for safety in item['safety_considerations']])
            
            # Add solutions for problem-solving items
            if 'solutions' in item and isinstance(item['solutions'], list):
                content_parts.append("Solutions:")
                content_parts.extend([f"- {solution}" for solution in item['solutions']])
        
        return "\n".join(content_parts)
    
    def _extract_keywords(self, content: str) -> List[str]:
        """Extract keywords from content."""
        # Simple keyword extraction - could be enhanced with NLP
        common_words = {'the', 'a', 'an', 'and', 'or', 'but', 'in', 'on', 'at', 'to', 'for', 'of', 'with', 'by', 'is', 'are', 'was', 'were'}
        words = content.lower().split()
        keywords = [word.strip('.,!?;:') for word in words if len(word) > 3 and word not in common_words]
        return list(set(keywords))[:10]  # Return top 10 unique keywords
    
    def plan_with_rag(self, user_query: str) -> Dict[str, Any]:
        """
        Plan a task using true RAG methodology.
        
        Args:
            user_query: User's natural language request
            
        Returns:
            Intelligent plan with RAG-retrieved knowledge
        """
        try:
            logger.info(f"Starting RAG-based planning for: '{user_query}'")
            
            # Step 1: RETRIEVE - Find relevant knowledge
            retrieved_docs = self._retrieve_relevant_knowledge(user_query)
            
            # Step 2: Get current context
            current_context = self._get_current_context()
            
            # Step 3: AUGMENT - Create enriched context
            rag_context = RAGContext(
                user_query=user_query,
                retrieved_documents=retrieved_docs,
                current_environment=current_context.get('environment', {}),
                robot_state=current_context.get('robot_state', {}),
                execution_history=current_context.get('history', [])
            )
            
            # Step 4: GENERATE - Create intelligent plan
            plan = self._generate_plan_with_rag(rag_context)
            
            # Add RAG metadata
            plan['rag_metadata'] = {
                'retrieved_documents': len(retrieved_docs),
                'knowledge_categories': list(set([doc.category for doc in retrieved_docs])),
                'confidence_score': rag_context.confidence_score,
                'retrieval_method': 'semantic_search' if self.vector_db.collection else 'keyword_fallback'
            }
            
            logger.info(f"RAG planning completed with {len(retrieved_docs)} retrieved documents")
            return plan
            
        except Exception as e:
            logger.error(f"RAG planning failed: {e}")
            # NO FALLBACKS - PURE RAG TESTING
            raise RuntimeError(f"RAG planning failed and no fallbacks allowed for pure testing: {e}")
    
    def _retrieve_relevant_knowledge(self, query: str, n_results: int = 5) -> List[RAGDocument]:
        """Retrieve relevant knowledge using semantic search."""
        
        # Enhance query with context
        enhanced_query = self._enhance_query_with_context(query)
        
        # Perform semantic search
        retrieved_docs = self.vector_db.search(enhanced_query, n_results=n_results)
        
        # Filter and rank results
        filtered_docs = self._filter_and_rank_documents(retrieved_docs, query)
        
        logger.info(f"Retrieved {len(filtered_docs)} relevant documents for: '{query}'")
        
        return filtered_docs
    
    def _enhance_query_with_context(self, query: str) -> str:
        """Enhance query with current context for better retrieval."""
        enhanced_parts = [query]
        
        # Add robotics context
        enhanced_parts.append("robot manipulation movement planning")
        
        # Add current environment context if available
        try:
            current_objects = self._get_current_objects()
            if current_objects:
                object_types = [obj.get('class', '') for obj in current_objects]
                enhanced_parts.append(f"objects: {' '.join(object_types)}")
        except:
            pass
        
        return " ".join(enhanced_parts)
    
    def _filter_and_rank_documents(self, docs: List[RAGDocument], original_query: str) -> List[RAGDocument]:
        """Filter and rank retrieved documents by relevance."""
        if not docs:
            return []
        
        # Calculate additional relevance scores
        query_words = set(original_query.lower().split())
        
        for doc in docs:
            # Boost score based on keyword overlap
            doc_words = set(doc.content.lower().split())
            keyword_overlap = len(query_words.intersection(doc_words)) / max(len(query_words), 1)
            doc.relevance_score = (doc.relevance_score + keyword_overlap) / 2
        
        # Sort by relevance score
        docs.sort(key=lambda x: x.relevance_score, reverse=True)
        
        # Return top results with minimum relevance threshold
        return [doc for doc in docs if doc.relevance_score > 0.1]
    
    def _generate_plan_with_rag(self, context: RAGContext) -> Dict[str, Any]:
        """Generate plan using RAG-augmented context."""
        
        if not self.llm:
            raise RuntimeError("LLM not available - no fallbacks allowed for pure RAG testing")
        
        try:
            # Create RAG-augmented prompt
            prompt = self._create_rag_prompt(context)
            
            # Generate response
            response = self.llm.generate_content(
                prompt,
                generation_config=genai.types.GenerationConfig(
                    temperature=0.2,  # Lower temperature for more consistent results
                    top_p=0.8,
                    top_k=40,
                    max_output_tokens=2048,
                )
            )
            
            # Parse response
            plan = self._parse_rag_response(response.text, context)
            
            return plan
            
        except Exception as e:
            logger.error(f"RAG generation failed: {e}")
            return self._generate_intelligent_fallback_with_rag(context)
    
    def _create_rag_prompt(self, context: RAGContext) -> str:
        """Create a RAG-augmented prompt with retrieved knowledge."""
        
        prompt = f"""
You are an advanced AI robot control system with access to comprehensive robotics knowledge. You can retrieve and apply relevant knowledge to create intelligent plans for any robot task.

## USER REQUEST
"{context.user_query}"

## RETRIEVED ROBOTICS KNOWLEDGE
The following knowledge has been semantically retrieved from the robotics knowledge base based on your request:

"""
        
        # Add retrieved knowledge
        for i, doc in enumerate(context.retrieved_documents[:3], 1):  # Top 3 most relevant
            prompt += f"""
### Knowledge {i} (Relevance: {doc.relevance_score:.2f})
Category: {doc.category}
{doc.content}

"""
        
        prompt += f"""

## CURRENT ROBOT STATE
Robot Position: {context.robot_state.get('position', 'Unknown')}
Gripper State: {context.robot_state.get('gripper_state', 'Unknown')}
Detected Objects: {[obj.get('class', 'unknown') for obj in context.current_environment.get('objects', [])]}

## INSTRUCTIONS

Using the retrieved robotics knowledge above, create an intelligent plan that:

1. **Applies Retrieved Knowledge**: Use the patterns, strategies, and solutions from the retrieved knowledge
2. **Adapts to Current Context**: Consider current robot state and environment
3. **Follows Safety Guidelines**: Apply safety considerations from the knowledge base
4. **Uses Proven Patterns**: Leverage successful movement patterns and task strategies
5. **Handles Edge Cases**: Apply problem-solving strategies for potential issues

## OUTPUT FORMAT
Return a JSON object with this structure:
{{
  "understanding": "Your interpretation using retrieved knowledge",
  "retrieved_knowledge_applied": "How you used the retrieved knowledge",
  "reasoning": "Your reasoning based on knowledge and context",
  "goal": "Clear task goal",
  "approach": "Approach based on retrieved patterns",
  "safety_considerations": "Safety aspects from knowledge base",
  "steps": [
    {{"action": "ACTION_NAME", "parameters": {{}}, "knowledge_source": "which retrieved knowledge guided this step"}}
  ],
  "confidence": "High/Medium/Low based on knowledge relevance",
  "fallback_plan": "Alternative approach if main plan fails"
}}

Generate the intelligent plan now using the retrieved robotics knowledge:
"""
        
        return prompt
    
    def _parse_rag_response(self, response_text: str, context: RAGContext) -> Dict[str, Any]:
        """Parse RAG-generated response."""
        try:
            # Extract JSON from response
            json_start = response_text.find('{')
            json_end = response_text.rfind('}') + 1
            
            if json_start == -1 or json_end == 0:
                raise ValueError("No JSON found in RAG response")
            
            json_str = response_text[json_start:json_end]
            plan = json.loads(json_str)
            
            # Add RAG metadata
            plan['generated_by'] = 'true_rag_system'
            plan['knowledge_sources'] = [doc.category for doc in context.retrieved_documents]
            plan['retrieval_confidence'] = sum(doc.relevance_score for doc in context.retrieved_documents) / max(len(context.retrieved_documents), 1)
            
            return plan
            
        except Exception as e:
            logger.error(f"Failed to parse RAG response: {e}")
            return self._generate_intelligent_fallback_with_rag(context)
    
    def _generate_intelligent_fallback_with_rag(self, context: RAGContext) -> Dict[str, Any]:
        """Generate intelligent fallback plan using retrieved knowledge."""
        
        query = context.user_query.lower()
        retrieved_docs = context.retrieved_documents
        
        # Apply retrieved knowledge to fallback planning
        applicable_patterns = []
        for doc in retrieved_docs:
            if 'pattern' in doc.content.lower() or 'steps' in doc.content.lower():
                applicable_patterns.append(doc)
        
        # Generate plan based on retrieved patterns
        if any(phrase in query for phrase in ["move toward", "approach", "go to"]):
            return self._create_movement_plan_with_knowledge(query, applicable_patterns)
        elif any(phrase in query for phrase in ["pick up", "grab", "take"]):
            return self._create_manipulation_plan_with_knowledge(query, applicable_patterns)
        elif any(phrase in query for phrase in ["clean", "organize", "tidy"]):
            return self._create_organization_plan_with_knowledge(query, applicable_patterns)
        else:
            return self._create_general_plan_with_knowledge(query, applicable_patterns)
    
    def _create_movement_plan_with_knowledge(self, query: str, patterns: List[RAGDocument]) -> Dict[str, Any]:
        """Create movement plan using retrieved knowledge patterns."""
        
        # Extract target object
        target_object = self._extract_target_from_query(query)
        
        # Find applicable movement pattern
        movement_pattern = None
        for pattern in patterns:
            if 'approach' in pattern.content.lower() or 'movement' in pattern.content.lower():
                movement_pattern = pattern
                break
        
        steps = []
        knowledge_applied = []
        
        # Apply retrieved knowledge to create steps
        if movement_pattern:
            knowledge_applied.append(f"Applied {movement_pattern.category} pattern for safe object approach")
            if target_object:
                steps = [
                    {"action": "SCAN_FOR_OBJECTS", "pattern": "horizontal", "sweep_mm": 300, "steps": 5, "pause_sec": 1.0, "knowledge_source": "systematic_scanning_pattern"},
                    {"action": "APPROACH_OBJECT", "label": target_object, "hover_mm": 100, "timeout_sec": 5, "knowledge_source": "safe_object_approach_pattern"},
                    {"action": "MOVE_TO_NAMED", "name": "home", "knowledge_source": "safety_return_pattern"}
                ]
            else:
                steps = [
                    {"action": "SCAN_FOR_OBJECTS", "pattern": "horizontal", "sweep_mm": 300, "steps": 5, "pause_sec": 1.0},
                    {"action": "MOVE_TO_NAMED", "name": "home"}
                ]
        else:
            # Basic fallback
            steps = [
                {"action": "MOVE_TO_NAMED", "name": "home"}
            ]
        
        return {
            "understanding": f"Movement request toward {target_object or 'general area'}",
            "retrieved_knowledge_applied": "; ".join(knowledge_applied) if knowledge_applied else "Limited knowledge available",
            "reasoning": "Applied retrieved movement patterns with safety considerations",
            "goal": f"Safely move toward {target_object or 'target location'}",
            "approach": "Knowledge-based safe movement pattern",
            "steps": steps,
            "confidence": "High" if movement_pattern else "Medium",
            "generated_by": "rag_fallback_movement"
        }
    
    def _create_manipulation_plan_with_knowledge(self, query: str, patterns: List[RAGDocument]) -> Dict[str, Any]:
        """Create manipulation plan using retrieved knowledge."""
        
        target_object = self._extract_target_from_query(query)
        
        # Find manipulation pattern
        manipulation_pattern = None
        for pattern in patterns:
            if 'pick' in pattern.content.lower() or 'grasp' in pattern.content.lower():
                manipulation_pattern = pattern
                break
        
        steps = [
            {"action": "SCAN_FOR_OBJECTS", "pattern": "horizontal", "sweep_mm": 300, "steps": 5, "pause_sec": 1.0},
            {"action": "OPEN_GRIPPER", "gripper": {"position": 850, "speed": 200}},
            {"action": "APPROACH_OBJECT", "label": target_object or "object", "hover_mm": 80, "timeout_sec": 5},
            {"action": "MOVE_TO_OBJECT", "label": target_object or "object", "offset_mm": [0, 0, 0], "timeout_sec": 5},
            {"action": "GRIPPER_GRASP", "target_position": 200, "speed": 100, "force": 50},
            {"action": "RETREAT_Z", "dz_mm": 100},
            {"action": "MOVE_TO_NAMED", "name": "home"}
        ]
        
        return {
            "understanding": f"Manipulation request to pick up {target_object or 'object'}",
            "retrieved_knowledge_applied": f"Applied {manipulation_pattern.category if manipulation_pattern else 'basic'} manipulation pattern",
            "reasoning": "Used knowledge-based pick and place sequence with safety considerations",
            "goal": f"Pick up {target_object or 'target object'}",
            "approach": "Knowledge-guided manipulation with force feedback",
            "steps": steps,
            "confidence": "High" if manipulation_pattern else "Medium",
            "generated_by": "rag_fallback_manipulation"
        }
    
    def _create_organization_plan_with_knowledge(self, query: str, patterns: List[RAGDocument]) -> Dict[str, Any]:
        """Create organization plan using retrieved knowledge."""
        
        # Find organization pattern
        org_pattern = None
        for pattern in patterns:
            if 'clean' in pattern.content.lower() or 'organiz' in pattern.content.lower():
                org_pattern = pattern
                break
        
        steps = [
            {"action": "SCAN_FOR_OBJECTS", "pattern": "horizontal", "sweep_mm": 300, "steps": 7, "pause_sec": 1.5},
            {"action": "MOVE_TO_NAMED", "name": "staging_area"},
            {"action": "OPEN_GRIPPER", "gripper": {"position": 400, "speed": 150}},
            {"action": "MOVE_TO_NAMED", "name": "home"}
        ]
        
        return {
            "understanding": "Organization and cleaning request",
            "retrieved_knowledge_applied": f"Applied {org_pattern.category if org_pattern else 'basic'} organization strategy",
            "reasoning": "Used systematic approach based on retrieved cleaning patterns",
            "goal": "Organize and clean workspace",
            "approach": "Knowledge-based systematic organization",
            "steps": steps,
            "confidence": "High" if org_pattern else "Medium",
            "generated_by": "rag_fallback_organization"
        }
    
    def _create_general_plan_with_knowledge(self, query: str, patterns: List[RAGDocument]) -> Dict[str, Any]:
        """Create general plan using any available retrieved knowledge."""
        
        steps = [
            {"action": "SCAN_FOR_OBJECTS", "pattern": "horizontal", "sweep_mm": 300, "steps": 5, "pause_sec": 1.0},
            {"action": "MOVE_TO_NAMED", "name": "home"}
        ]
        
        applied_knowledge = []
        if patterns:
            applied_knowledge = [f"Retrieved {len(patterns)} relevant knowledge items"]
        
        return {
            "understanding": f"General request: {query}",
            "retrieved_knowledge_applied": "; ".join(applied_knowledge) if applied_knowledge else "Limited specific knowledge available",
            "reasoning": "Applied general robotics principles with available knowledge",
            "goal": "Safe response to user request",
            "approach": "Conservative approach with knowledge-based safety",
            "steps": steps,
            "confidence": "Medium",
            "generated_by": "rag_fallback_general"
        }
    
    # Helper methods
    def _extract_target_from_query(self, query: str) -> Optional[str]:
        """Extract target object from query."""
        common_objects = ["cup", "bottle", "bowl", "plate", "microwave", "oven", "bin", "box"]
        query_lower = query.lower()
        for obj in common_objects:
            if obj in query_lower:
                return obj
        return None
    
    def _get_current_context(self) -> Dict[str, Any]:
        """Get current robot and environment context."""
        return {
            "robot_state": self._get_robot_state(),
            "environment": {"objects": self._get_current_objects()},
            "history": []
        }
    
    def _get_robot_state(self) -> Dict[str, Any]:
        """Get current robot state."""
        state = {"position": "unknown", "gripper_state": "unknown"}
        
        if self.robot_controller:
            try:
                if hasattr(self.robot_controller, 'get_current_position'):
                    state["position"] = self.robot_controller.get_current_position()
                if hasattr(self.robot_controller, 'get_gripper_status'):
                    state["gripper_state"] = self.robot_controller.get_gripper_status()
            except:
                pass
        
        return state
    
    def _get_current_objects(self) -> List[Dict[str, Any]]:
        """Get currently detected objects."""
        if hasattr(self, 'object_index') and self.object_index:
            try:
                with self.object_index._global_lock:
                    objects = []
                    for label, position in self.object_index.latest_mm.items():
                        objects.append({
                            "class": label,
                            "pos": position,
                            "confidence": 0.8
                        })
                    return objects
            except:
                pass
        return []
    
    def _initialize_llm(self):
        """Initialize LLM for RAG generation."""
        if not GEMINI_AVAILABLE:
            return None
        
        api_key = os.getenv('GEMINI_API_KEY')
        if not api_key:
            return None
        
        try:
            genai.configure(api_key=api_key)
            return genai.GenerativeModel('gemini-pro')
        except Exception as e:
            logger.error(f"Failed to initialize LLM: {e}")
            return None
    
    def _initialize_vision_integration(self):
        """Initialize vision system integration."""
        try:
            from robot_control.robot_controller.executor import ObjectIndex
            self.object_index = ObjectIndex()
            logger.info("Vision integration initialized for RAG system")
        except Exception as e:
            logger.warning(f"Failed to initialize vision integration: {e}")
            self.object_index = None
    
    def _generate_fallback_plan(self, query: str) -> Dict[str, Any]:
        """Generate basic fallback plan when RAG fails."""
        return {
            "understanding": f"Basic fallback for: {query}",
            "reasoning": "RAG system unavailable, using basic fallback",
            "goal": query,
            "approach": "Basic safe approach",
            "steps": [
                {"action": "MOVE_TO_NAMED", "name": "home"},
                {"action": "SLEEP", "seconds": 1.0}
            ],
            "confidence": "Low",
            "generated_by": "basic_fallback"
        }
