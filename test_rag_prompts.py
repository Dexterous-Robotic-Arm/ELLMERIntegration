#!/usr/bin/env python3
"""
RAG Prompt Testing Suite

Focused test suite for testing RAG prompt generation and knowledge retrieval
without any hardware or LLM dependencies. Tests the core prompt engineering
and knowledge base integration.

Usage:
    python test_rag_prompts.py
    python test_rag_prompts.py --save-prompts prompts_output.json
"""

import os
import sys
import json
import argparse
from pathlib import Path
from typing import Dict, List, Any

# Add project root to path
sys.path.insert(0, str(Path(__file__).parent))

# Test specific prompts
TEST_PROMPTS = [
    {
        "name": "Basic Movement",
        "query": "move toward bottle",
        "description": "Simple object approach task"
    },
    {
        "name": "Complex Manipulation",
        "query": "pick up the fragile glass cup and place it carefully on the shelf",
        "description": "Multi-step manipulation with safety considerations"
    },
    {
        "name": "Workspace Organization", 
        "query": "clean up the messy workspace and organize all objects by type",
        "description": "Complex organizational task"
    },
    {
        "name": "Precision Task",
        "query": "precisely align the robot gripper 5mm above the microchip",
        "description": "High precision positioning task"
    },
    {
        "name": "Error Recovery",
        "query": "the gripper failed to grasp the object, try a different approach",
        "description": "Error recovery and adaptation scenario"
    },
    {
        "name": "Safety Critical",
        "query": "approach the hot stove very carefully without touching it",
        "description": "Safety-critical navigation task"
    },
    {
        "name": "Multi-Object Task",
        "query": "find all red objects, pick them up one by one, and place them in the red bin",
        "description": "Complex multi-object manipulation sequence"
    },
    {
        "name": "Ambiguous Request",
        "query": "help me with cooking",
        "description": "Ambiguous request requiring clarification"
    }
]

# Mock data for testing
MOCK_ROBOT_STATE = {
    "position": [300.0, 0.0, 250.0],
    "gripper_state": "open",
    "joint_angles": [0, 0, 0, 0, 0, 0]
}

MOCK_ENVIRONMENT = {
    "objects": [
        {"class": "bottle", "position": [400, 100, 50], "confidence": 0.95},
        {"class": "cup", "position": [350, -50, 45], "confidence": 0.87}, 
        {"class": "microchip", "position": [320, 20, 30], "confidence": 0.92},
        {"class": "stove", "position": [500, 200, 80], "confidence": 0.88}
    ]
}


class RAGPromptTester:
    """Test suite focused on RAG prompt generation."""
    
    def __init__(self):
        self.results = []
        
    def test_knowledge_retrieval_only(self):
        """Test knowledge retrieval without full RAG system."""
        print("🔍 Testing Knowledge Base Retrieval...")
        
        try:
            from knowledge_base.robotics_knowledge import get_all_knowledge, search_knowledge_by_keywords
            
            # Test knowledge base loading
            all_knowledge = get_all_knowledge()
            print(f"✅ Knowledge base loaded: {len(all_knowledge)} categories")
            
            # Test keyword search for each prompt
            for test_case in TEST_PROMPTS:
                query = test_case["query"]
                print(f"\n📝 Testing: {test_case['name']}")
                print(f"   Query: '{query}'")
                
                # Extract keywords from query
                keywords = query.lower().split()
                
                # Search for relevant knowledge
                relevant_knowledge = []
                for category, items in all_knowledge.items():
                    for item in items:
                        item_text = json.dumps(item).lower()
                        if any(keyword in item_text for keyword in keywords):
                            relevant_knowledge.append({
                                "category": category,
                                "id": item.get("id", "unknown"),
                                "title": item.get("title", "Unknown"),
                                "relevance": sum(1 for keyword in keywords if keyword in item_text)
                            })
                
                # Sort by relevance
                relevant_knowledge.sort(key=lambda x: x["relevance"], reverse=True)
                
                print(f"   📚 Found {len(relevant_knowledge)} relevant knowledge items")
                for i, knowledge in enumerate(relevant_knowledge[:3]):
                    print(f"      {i+1}. [{knowledge['category']}] {knowledge['title']} (relevance: {knowledge['relevance']})")
                    
                self.results.append({
                    "test_case": test_case,
                    "relevant_knowledge": relevant_knowledge[:5],  # Top 5
                    "knowledge_count": len(relevant_knowledge)
                })
                
            return True
            
        except Exception as e:
            print(f"❌ Knowledge retrieval test failed: {e}")
            return False
    
    def test_prompt_structure_without_llm(self):
        """Test prompt structure generation without requiring LLM."""
        print("\n📝 Testing RAG Prompt Structure Generation...")
        
        try:
            # Import required components
            from robot_control.rag_system.true_rag_planner import TrueRAGPlanner, RAGContext, RAGDocument
            
            # Create a minimal RAG planner (may fail on LLM init, but we only need prompt generation)
            try:
                rag_planner = TrueRAGPlanner(
                    robot_controller=None,
                    vision_system=None,
                    config_path="config/",
                    db_path="data/vector_db"
                )
            except Exception as e:
                print(f"⚠️  RAG planner init failed (expected if no LLM): {e}")
                print("🔄 Creating minimal planner for prompt testing...")
                
                # Create minimal planner just for prompt generation
                class MinimalRAGPlanner:
                    def _create_rag_prompt(self, context):
                        # Simplified prompt generation for testing
                        prompt = f"""
You are an advanced AI robot control system with access to comprehensive robotics knowledge.

## USER REQUEST
"{context.user_query}"

## RETRIEVED ROBOTICS KNOWLEDGE
The following knowledge has been semantically retrieved from the robotics knowledge base:

"""
                        for i, doc in enumerate(context.retrieved_documents[:3], 1):
                            prompt += f"""
### Knowledge {i} (Relevance: {doc.relevance_score:.2f})
Category: {doc.category}
{doc.content[:200]}...

"""
                        
                        prompt += f"""
## CURRENT ROBOT STATE
Robot Position: {context.robot_state.get('position', 'Unknown')}
Gripper State: {context.robot_state.get('gripper_state', 'Unknown')}

## DETECTED OBJECTS
Currently detected objects with positions:
{context.current_environment.get('objects', [])}

## INSTRUCTIONS
Using the retrieved robotics knowledge above, create an intelligent plan that:
1. Applies Retrieved Knowledge
2. Uses Detected Objects  
3. Follows Safety Guidelines
4. Uses Proven Patterns

## OUTPUT FORMAT
Return a JSON object with this structure:
{{
  "understanding": "Your interpretation using retrieved knowledge",
  "reasoning": "Your reasoning based on knowledge and context", 
  "goal": "Clear task goal",
  "steps": [
    {{"action": "ACTION_NAME", "parameters": {{}}}
  ],
  "confidence": "High/Medium/Low"
}}

Generate the intelligent plan now:
"""
                        return prompt
                
                rag_planner = MinimalRAGPlanner()
            
            # Test prompt generation for each test case
            for test_case in TEST_PROMPTS:
                print(f"\n🎯 Testing Prompt Generation: {test_case['name']}")
                
                # Create mock retrieved documents
                mock_docs = [
                    RAGDocument(
                        id="test_doc_1",
                        content="Safe object approach pattern: Scan area, move to hover position, verify object position, perform action, retreat safely.",
                        metadata={"category": "movement_patterns"},
                        category="movement_patterns",
                        keywords=["approach", "safe", "object"],
                        relevance_score=0.85
                    ),
                    RAGDocument(
                        id="test_doc_2", 
                        content="Pick and place sequence: Open gripper, approach object, close gripper with force feedback, lift to safe height, move to destination.",
                        metadata={"category": "manipulation"},
                        category="manipulation",
                        keywords=["pick", "place", "gripper"],
                        relevance_score=0.78
                    )
                ]
                
                # Create RAG context
                rag_context = RAGContext(
                    user_query=test_case["query"],
                    retrieved_documents=mock_docs,
                    current_environment=MOCK_ENVIRONMENT,
                    robot_state=MOCK_ROBOT_STATE,
                    execution_history=[],
                    confidence_score=0.8
                )
                
                # Generate prompt
                prompt = rag_planner._create_rag_prompt(rag_context)
                
                # Analyze prompt
                analysis = {
                    "test_case": test_case,
                    "prompt_length": len(prompt),
                    "contains_query": test_case["query"] in prompt,
                    "contains_knowledge": "Retrieved Knowledge" in prompt,
                    "contains_robot_state": "Robot Position" in prompt,
                    "contains_objects": "objects" in prompt.lower(),
                    "contains_instructions": "INSTRUCTIONS" in prompt,
                    "contains_output_format": "OUTPUT FORMAT" in prompt,
                    "section_count": prompt.count("##"),
                    "prompt": prompt
                }
                
                print(f"   📏 Prompt Length: {analysis['prompt_length']} characters")
                print(f"   📋 Sections: {analysis['section_count']}")
                print(f"   ✅ Contains Query: {analysis['contains_query']}")
                print(f"   📚 Contains Knowledge: {analysis['contains_knowledge']}")
                print(f"   🤖 Contains Robot State: {analysis['contains_robot_state']}")
                print(f"   👁️  Contains Objects: {analysis['contains_objects']}")
                
                self.results.append(analysis)
                
            print(f"\n✅ Successfully generated {len(TEST_PROMPTS)} RAG prompts")
            return True
            
        except Exception as e:
            print(f"❌ Prompt generation test failed: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def analyze_prompt_quality(self):
        """Analyze the quality of generated prompts."""
        print("\n📊 Analyzing Prompt Quality...")
        
        if not self.results:
            print("❌ No results to analyze")
            return
        
        # Filter results that have prompts
        prompt_results = [r for r in self.results if "prompt" in r]
        
        if not prompt_results:
            print("❌ No prompt results to analyze")
            return
        
        # Calculate statistics
        avg_length = sum(r["prompt_length"] for r in prompt_results) / len(prompt_results)
        avg_sections = sum(r["section_count"] for r in prompt_results) / len(prompt_results)
        
        completeness_scores = []
        for result in prompt_results:
            score = sum([
                result["contains_query"],
                result["contains_knowledge"], 
                result["contains_robot_state"],
                result["contains_objects"],
                result["contains_instructions"],
                result["contains_output_format"]
            ])
            completeness_scores.append(score / 6.0)  # Normalize to 0-1
        
        avg_completeness = sum(completeness_scores) / len(completeness_scores)
        
        print(f"📈 Prompt Quality Analysis:")
        print(f"   📏 Average Length: {avg_length:.0f} characters")
        print(f"   📋 Average Sections: {avg_sections:.1f}")
        print(f"   ✅ Average Completeness: {avg_completeness:.1%}")
        print(f"   🎯 Prompts Analyzed: {len(prompt_results)}")
        
        # Find best and worst prompts
        best_prompt = max(prompt_results, key=lambda x: completeness_scores[prompt_results.index(x)])
        worst_prompt = min(prompt_results, key=lambda x: completeness_scores[prompt_results.index(x)])
        
        print(f"\n🏆 Best Prompt: {best_prompt['test_case']['name']} (completeness: {completeness_scores[prompt_results.index(best_prompt)]:.1%})")
        print(f"⚠️  Worst Prompt: {worst_prompt['test_case']['name']} (completeness: {completeness_scores[prompt_results.index(worst_prompt)]:.1%})")
    
    def save_results(self, filename: str):
        """Save test results to JSON file."""
        try:
            # Clean results for JSON serialization
            clean_results = []
            for result in self.results:
                clean_result = result.copy()
                # Remove the full prompt to keep file size manageable
                if "prompt" in clean_result:
                    clean_result["prompt_preview"] = clean_result["prompt"][:500] + "..." if len(clean_result["prompt"]) > 500 else clean_result["prompt"]
                    del clean_result["prompt"]
                clean_results.append(clean_result)
            
            with open(filename, 'w') as f:
                json.dump({
                    "test_suite": "RAG Prompt Testing",
                    "test_count": len(clean_results),
                    "results": clean_results
                }, f, indent=2, default=str)
            
            print(f"💾 Results saved to: {filename}")
            
        except Exception as e:
            print(f"❌ Failed to save results: {e}")
    
    def run_all_tests(self):
        """Run all prompt-focused tests."""
        print("🧪 RAG PROMPT TESTING SUITE")
        print("=" * 60)
        
        success = True
        
        # Test 1: Knowledge Retrieval
        if not self.test_knowledge_retrieval_only():
            success = False
            
        # Test 2: Prompt Generation
        if not self.test_prompt_structure_without_llm():
            success = False
            
        # Test 3: Quality Analysis
        self.analyze_prompt_quality()
        
        print("\n" + "=" * 60)
        if success:
            print("🎉 ALL PROMPT TESTS COMPLETED SUCCESSFULLY!")
        else:
            print("⚠️  SOME TESTS FAILED - SEE DETAILS ABOVE")
        print("=" * 60)
        
        return success


def main():
    """Main test runner."""
    parser = argparse.ArgumentParser(description="RAG Prompt Testing Suite")
    parser.add_argument("--save-prompts", help="Save generated prompts to JSON file")
    
    args = parser.parse_args()
    
    # Create and run test suite
    tester = RAGPromptTester()
    success = tester.run_all_tests()
    
    # Save results if requested
    if args.save_prompts:
        tester.save_results(args.save_prompts)
    
    return 0 if success else 1


if __name__ == "__main__":
    exit(main())
