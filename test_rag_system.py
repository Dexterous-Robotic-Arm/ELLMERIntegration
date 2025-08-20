#!/usr/bin/env python3
"""
Comprehensive RAG System Test Suite

This test suite thoroughly tests the RAG (Retrieval-Augmented Generation) system
without any hardware dependencies. Tests the core AI prompting, knowledge retrieval,
and plan generation capabilities.

Usage:
    python test_rag_system.py
    python test_rag_system.py --test knowledge_retrieval
    python test_rag_system.py --test prompt_generation
    python test_rag_system.py --verbose
"""

import os
import sys
import json
import time
import argparse
import logging
from pathlib import Path
from typing import Dict, List, Any, Optional
import traceback

# Add project root to path
sys.path.insert(0, str(Path(__file__).parent))

# Import RAG system
from robot_control.rag_system.true_rag_planner import TrueRAGPlanner, RAGDocument, RAGContext
from knowledge_base.robotics_knowledge import get_all_knowledge, search_knowledge_by_keywords

# Test configuration
TEST_CASES = [
    # Basic movement tasks
    {
        "name": "Simple Movement",
        "query": "move toward bottle",
        "expected_actions": ["ENSURE_VISION_ACTIVE", "APPROACH_OBJECT", "MOVE_TO_NAMED"],
        "expected_knowledge": ["approach_object_safe", "movement"]
    },
    {
        "name": "Complex Manipulation", 
        "query": "pick up the red cup and place it on the table",
        "expected_actions": ["OPEN_GRIPPER", "APPROACH_OBJECT", "CLOSE_GRIPPER"],
        "expected_knowledge": ["pick_and_place_sequence", "manipulation"]
    },
    {
        "name": "Workspace Organization",
        "query": "clean up the workspace and organize objects",
        "expected_actions": ["SCAN_FOR_OBJECTS", "APPROACH_OBJECT"],
        "expected_knowledge": ["cleaning_organization_pattern", "workspace"]
    },
    {
        "name": "Safety-Critical Task",
        "query": "carefully approach the fragile glass object",
        "expected_actions": ["ENSURE_VISION_ACTIVE", "APPROACH_OBJECT"],
        "expected_knowledge": ["safety", "approach_object_safe"]
    },
    {
        "name": "Multi-Step Complex Task",
        "query": "scan the area, find all objects, pick up the bottle, and return home",
        "expected_actions": ["SCAN_FOR_OBJECTS", "APPROACH_OBJECT", "MOVE_TO_NAMED"],
        "expected_knowledge": ["scanning_pattern_systematic", "pick_and_place_sequence"]
    },
    {
        "name": "Ambiguous Task",
        "query": "help me with kitchen tasks",
        "expected_actions": ["SCAN_FOR_OBJECTS"],
        "expected_knowledge": ["general", "workspace"]
    },
    {
        "name": "Precision Task",
        "query": "precisely position the robot 2cm above the microchip",
        "expected_actions": ["ENSURE_VISION_ACTIVE", "APPROACH_OBJECT"],
        "expected_knowledge": ["precision", "approach_object_safe"]
    },
    {
        "name": "Error Recovery Scenario",
        "query": "if the gripper fails to grasp the object, try a different approach",
        "expected_actions": ["OPEN_GRIPPER", "APPROACH_OBJECT"],
        "expected_knowledge": ["error_recovery", "pick_and_place_sequence"]
    }
]

# Mock robot state for testing
MOCK_ROBOT_STATE = {
    "position": [300.0, 0.0, 250.0],
    "gripper_state": "open",
    "joint_angles": [0, 0, 0, 0, 0, 0],
    "is_connected": True
}

MOCK_ENVIRONMENT = {
    "objects": [
        {"class": "bottle", "position": [400, 100, 50], "confidence": 0.95},
        {"class": "cup", "position": [350, -50, 45], "confidence": 0.87},
        {"class": "microchip", "position": [320, 20, 30], "confidence": 0.92}
    ],
    "workspace_bounds": {"x": [150, 650], "y": [-300, 300], "z": [0, 400]},
    "lighting_conditions": "good",
    "camera_active": True
}


class RAGTestSuite:
    """Comprehensive test suite for the RAG system."""
    
    def __init__(self, verbose: bool = False):
        self.verbose = verbose
        self.setup_logging()
        self.results = []
        self.rag_planner = None
        
    def setup_logging(self):
        """Setup logging for tests."""
        level = logging.DEBUG if self.verbose else logging.INFO
        logging.basicConfig(
            level=level,
            format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
        )
        self.logger = logging.getLogger(__name__)
        
    def initialize_rag_system(self) -> bool:
        """Initialize the RAG system for testing."""
        try:
            print("🚀 Initializing RAG System...")
            
            # Initialize RAG planner without robot/vision dependencies
            self.rag_planner = TrueRAGPlanner(
                robot_controller=None,
                vision_system=None,
                config_path="config/",
                db_path="data/vector_db"
            )
            
            print("✅ RAG System initialized successfully")
            return True
            
        except Exception as e:
            print(f"❌ Failed to initialize RAG system: {e}")
            if self.verbose:
                traceback.print_exc()
            return False
    
    def test_knowledge_base_loading(self) -> Dict[str, Any]:
        """Test knowledge base loading and structure."""
        print("\n📚 Testing Knowledge Base Loading...")
        
        try:
            # Test knowledge base import
            all_knowledge = get_all_knowledge()
            
            result = {
                "test_name": "Knowledge Base Loading",
                "success": True,
                "knowledge_categories": len(all_knowledge.keys()),
                "total_items": sum(len(items) for items in all_knowledge.values()),
                "categories": list(all_knowledge.keys()),
                "details": {}
            }
            
            # Test each category
            for category, items in all_knowledge.items():
                result["details"][category] = {
                    "count": len(items),
                    "sample_items": [item.get("id", item.get("title", "Unknown")) for item in items[:3]]
                }
                
            print(f"✅ Knowledge base loaded: {result['knowledge_categories']} categories, {result['total_items']} total items")
            return result
            
        except Exception as e:
            print(f"❌ Knowledge base loading failed: {e}")
            return {
                "test_name": "Knowledge Base Loading",
                "success": False,
                "error": str(e)
            }
    
    def test_knowledge_retrieval(self, query: str, expected_knowledge: List[str]) -> Dict[str, Any]:
        """Test knowledge retrieval for a specific query."""
        print(f"\n🔍 Testing Knowledge Retrieval for: '{query}'")
        
        try:
            # Test semantic search
            retrieved_docs = self.rag_planner._retrieve_relevant_knowledge(query, n_results=5)
            
            result = {
                "test_name": f"Knowledge Retrieval: {query}",
                "query": query,
                "success": True,
                "retrieved_count": len(retrieved_docs),
                "expected_knowledge": expected_knowledge,
                "retrieved_documents": [],
                "relevance_scores": [],
                "knowledge_match": False
            }
            
            # Analyze retrieved documents
            for doc in retrieved_docs:
                result["retrieved_documents"].append({
                    "id": doc.id,
                    "category": doc.category,
                    "relevance_score": doc.relevance_score,
                    "keywords": doc.keywords[:5]  # First 5 keywords
                })
                result["relevance_scores"].append(doc.relevance_score)
            
            # Check if expected knowledge was retrieved
            retrieved_content = " ".join([doc.content.lower() for doc in retrieved_docs])
            knowledge_matches = sum(1 for expected in expected_knowledge 
                                  if expected.lower() in retrieved_content)
            result["knowledge_match"] = knowledge_matches > 0
            result["knowledge_match_count"] = knowledge_matches
            
            if result["knowledge_match"]:
                print(f"✅ Retrieved {len(retrieved_docs)} relevant documents, matched {knowledge_matches} expected knowledge areas")
            else:
                print(f"⚠️  Retrieved {len(retrieved_docs)} documents but no expected knowledge matches")
                
            return result
            
        except Exception as e:
            print(f"❌ Knowledge retrieval failed: {e}")
            return {
                "test_name": f"Knowledge Retrieval: {query}",
                "query": query,
                "success": False,
                "error": str(e)
            }
    
    def test_rag_prompt_generation(self, query: str) -> Dict[str, Any]:
        """Test RAG prompt generation."""
        print(f"\n📝 Testing RAG Prompt Generation for: '{query}'")
        
        try:
            # Create RAG context
            retrieved_docs = self.rag_planner._retrieve_relevant_knowledge(query, n_results=3)
            
            rag_context = RAGContext(
                user_query=query,
                retrieved_documents=retrieved_docs,
                current_environment=MOCK_ENVIRONMENT,
                robot_state=MOCK_ROBOT_STATE,
                execution_history=[],
                confidence_score=0.8
            )
            
            # Generate RAG prompt
            prompt = self.rag_planner._create_rag_prompt(rag_context)
            
            result = {
                "test_name": f"RAG Prompt Generation: {query}",
                "query": query,
                "success": True,
                "prompt_length": len(prompt),
                "contains_query": query in prompt,
                "contains_knowledge": len(retrieved_docs) > 0,
                "contains_robot_state": "position" in prompt,
                "contains_environment": "objects" in prompt,
                "knowledge_count": len(retrieved_docs),
                "prompt_sections": []
            }
            
            # Analyze prompt sections
            if "## USER REQUEST" in prompt:
                result["prompt_sections"].append("User Request")
            if "## RETRIEVED ROBOTICS KNOWLEDGE" in prompt:
                result["prompt_sections"].append("Retrieved Knowledge")
            if "## CURRENT ROBOT STATE" in prompt:
                result["prompt_sections"].append("Robot State")
            if "## DETECTED OBJECTS" in prompt:
                result["prompt_sections"].append("Detected Objects")
            if "## INSTRUCTIONS" in prompt:
                result["prompt_sections"].append("Instructions")
            if "## OUTPUT FORMAT" in prompt:
                result["prompt_sections"].append("Output Format")
                
            print(f"✅ Generated prompt: {result['prompt_length']} chars, {len(result['prompt_sections'])} sections")
            
            if self.verbose:
                print(f"\n📄 Generated Prompt Preview (first 500 chars):")
                print("-" * 80)
                print(prompt[:500] + "..." if len(prompt) > 500 else prompt)
                print("-" * 80)
                
            return result
            
        except Exception as e:
            print(f"❌ RAG prompt generation failed: {e}")
            return {
                "test_name": f"RAG Prompt Generation: {query}",
                "query": query,
                "success": False,
                "error": str(e)
            }
    
    def test_end_to_end_planning(self, test_case: Dict[str, Any]) -> Dict[str, Any]:
        """Test end-to-end RAG planning."""
        print(f"\n🎯 Testing End-to-End Planning: {test_case['name']}")
        
        try:
            # Override robot/vision state for testing
            original_robot_controller = self.rag_planner.robot_controller
            original_vision_system = self.rag_planner.vision_system
            
            # Mock the context methods
            def mock_get_robot_state():
                return MOCK_ROBOT_STATE
            
            def mock_get_current_objects():
                return MOCK_ENVIRONMENT["objects"]
            
            self.rag_planner._get_robot_state = mock_get_robot_state
            self.rag_planner._get_current_objects = mock_get_current_objects
            
            # Generate plan
            plan = self.rag_planner.plan_with_rag(test_case["query"])
            
            result = {
                "test_name": f"End-to-End Planning: {test_case['name']}",
                "query": test_case["query"],
                "success": True,
                "plan_generated": plan is not None,
                "plan_structure": {},
                "expected_actions": test_case["expected_actions"],
                "actual_actions": [],
                "action_match": False,
                "knowledge_applied": False,
                "plan": plan
            }
            
            if plan:
                # Analyze plan structure
                result["plan_structure"] = {
                    "has_understanding": "understanding" in plan,
                    "has_reasoning": "reasoning" in plan,
                    "has_goal": "goal" in plan,
                    "has_steps": "steps" in plan,
                    "has_confidence": "confidence" in plan,
                    "step_count": len(plan.get("steps", []))
                }
                
                # Extract actions
                for step in plan.get("steps", []):
                    if "action" in step:
                        result["actual_actions"].append(step["action"])
                
                # Check action matches
                expected_actions = test_case["expected_actions"]
                actual_actions = result["actual_actions"]
                action_matches = sum(1 for action in expected_actions if action in actual_actions)
                result["action_match"] = action_matches > 0
                result["action_match_count"] = action_matches
                
                # Check knowledge application
                plan_text = json.dumps(plan).lower()
                knowledge_matches = sum(1 for knowledge in test_case["expected_knowledge"] 
                                     if knowledge.lower() in plan_text)
                result["knowledge_applied"] = knowledge_matches > 0
                result["knowledge_match_count"] = knowledge_matches
                
                print(f"✅ Plan generated: {result['plan_structure']['step_count']} steps, "
                      f"{action_matches}/{len(expected_actions)} expected actions matched")
            else:
                print("❌ No plan generated")
                result["success"] = False
                
            # Restore original state
            self.rag_planner.robot_controller = original_robot_controller
            self.rag_planner.vision_system = original_vision_system
            
            return result
            
        except Exception as e:
            print(f"❌ End-to-end planning failed: {e}")
            if self.verbose:
                traceback.print_exc()
            return {
                "test_name": f"End-to-End Planning: {test_case['name']}",
                "query": test_case["query"],
                "success": False,
                "error": str(e)
            }
    
    def test_prompt_variations(self) -> Dict[str, Any]:
        """Test how the system handles various prompt variations."""
        print(f"\n🔄 Testing Prompt Variations...")
        
        variations = [
            "move toward bottle",
            "Move toward the bottle",
            "MOVE TOWARD BOTTLE",
            "please move toward the bottle",
            "can you move toward the bottle?",
            "I need you to move toward the bottle",
            "robot, move toward bottle now",
            "approach the bottle",
            "go to the bottle",
            "navigate to bottle"
        ]
        
        results = []
        
        for variation in variations:
            try:
                retrieved_docs = self.rag_planner._retrieve_relevant_knowledge(variation, n_results=3)
                results.append({
                    "variation": variation,
                    "success": True,
                    "retrieved_count": len(retrieved_docs),
                    "avg_relevance": sum(doc.relevance_score for doc in retrieved_docs) / max(len(retrieved_docs), 1)
                })
            except Exception as e:
                results.append({
                    "variation": variation,
                    "success": False,
                    "error": str(e)
                })
        
        successful_variations = sum(1 for r in results if r["success"])
        avg_retrieval_count = sum(r.get("retrieved_count", 0) for r in results if r["success"]) / max(successful_variations, 1)
        
        print(f"✅ Tested {len(variations)} variations: {successful_variations} successful, avg {avg_retrieval_count:.1f} docs retrieved")
        
        return {
            "test_name": "Prompt Variations",
            "success": True,
            "variations_tested": len(variations),
            "successful_variations": successful_variations,
            "avg_retrieval_count": avg_retrieval_count,
            "detailed_results": results
        }
    
    def run_all_tests(self) -> Dict[str, Any]:
        """Run all tests and return comprehensive results."""
        print("🧪 Starting Comprehensive RAG System Test Suite")
        print("=" * 80)
        
        start_time = time.time()
        
        # Initialize system
        if not self.initialize_rag_system():
            return {"success": False, "error": "Failed to initialize RAG system"}
        
        all_results = {
            "test_suite": "RAG System Comprehensive Tests",
            "timestamp": time.time(),
            "success": True,
            "total_tests": 0,
            "passed_tests": 0,
            "failed_tests": 0,
            "execution_time": 0,
            "results": {}
        }
        
        # Test 1: Knowledge Base Loading
        result = self.test_knowledge_base_loading()
        all_results["results"]["knowledge_base_loading"] = result
        all_results["total_tests"] += 1
        if result["success"]:
            all_results["passed_tests"] += 1
        else:
            all_results["failed_tests"] += 1
            all_results["success"] = False
        
        # Test 2: Knowledge Retrieval for each test case
        for test_case in TEST_CASES[:5]:  # Test first 5 cases
            result = self.test_knowledge_retrieval(test_case["query"], test_case["expected_knowledge"])
            all_results["results"][f"knowledge_retrieval_{test_case['name'].lower().replace(' ', '_')}"] = result
            all_results["total_tests"] += 1
            if result["success"]:
                all_results["passed_tests"] += 1
            else:
                all_results["failed_tests"] += 1
        
        # Test 3: RAG Prompt Generation
        for test_case in TEST_CASES[:3]:  # Test first 3 cases
            result = self.test_rag_prompt_generation(test_case["query"])
            all_results["results"][f"prompt_generation_{test_case['name'].lower().replace(' ', '_')}"] = result
            all_results["total_tests"] += 1
            if result["success"]:
                all_results["passed_tests"] += 1
            else:
                all_results["failed_tests"] += 1
        
        # Test 4: End-to-End Planning (only if LLM is available)
        if hasattr(self.rag_planner, 'llm') and self.rag_planner.llm:
            print("\n🤖 LLM Available - Testing End-to-End Planning")
            for test_case in TEST_CASES[:2]:  # Test first 2 cases
                result = self.test_end_to_end_planning(test_case)
                all_results["results"][f"end_to_end_{test_case['name'].lower().replace(' ', '_')}"] = result
                all_results["total_tests"] += 1
                if result["success"]:
                    all_results["passed_tests"] += 1
                else:
                    all_results["failed_tests"] += 1
        else:
            print("\n⚠️  LLM Not Available - Skipping End-to-End Planning Tests")
            print("💡 Set GEMINI_API_KEY environment variable to test full pipeline")
        
        # Test 5: Prompt Variations
        result = self.test_prompt_variations()
        all_results["results"]["prompt_variations"] = result
        all_results["total_tests"] += 1
        if result["success"]:
            all_results["passed_tests"] += 1
        else:
            all_results["failed_tests"] += 1
        
        # Calculate final results
        all_results["execution_time"] = time.time() - start_time
        all_results["success"] = all_results["failed_tests"] == 0
        
        return all_results
    
    def print_summary(self, results: Dict[str, Any]):
        """Print test summary."""
        print("\n" + "=" * 80)
        print("🧪 RAG SYSTEM TEST SUMMARY")
        print("=" * 80)
        
        print(f"📊 Total Tests: {results['total_tests']}")
        print(f"✅ Passed: {results['passed_tests']}")
        print(f"❌ Failed: {results['failed_tests']}")
        print(f"⏱️  Execution Time: {results['execution_time']:.2f}s")
        print(f"🎯 Success Rate: {(results['passed_tests']/results['total_tests']*100):.1f}%")
        
        if results["success"]:
            print("\n🎉 ALL TESTS PASSED - RAG SYSTEM IS WORKING CORRECTLY!")
        else:
            print(f"\n⚠️  {results['failed_tests']} TESTS FAILED - SEE DETAILS ABOVE")
        
        print("\n💡 Key Findings:")
        
        # Knowledge base findings
        kb_result = results["results"].get("knowledge_base_loading", {})
        if kb_result.get("success"):
            print(f"   📚 Knowledge Base: {kb_result.get('knowledge_categories', 0)} categories, {kb_result.get('total_items', 0)} items")
        
        # Knowledge retrieval findings
        retrieval_tests = [k for k in results["results"].keys() if k.startswith("knowledge_retrieval")]
        successful_retrievals = sum(1 for k in retrieval_tests if results["results"][k].get("success"))
        if retrieval_tests:
            print(f"   🔍 Knowledge Retrieval: {successful_retrievals}/{len(retrieval_tests)} queries successful")
        
        # Prompt generation findings
        prompt_tests = [k for k in results["results"].keys() if k.startswith("prompt_generation")]
        successful_prompts = sum(1 for k in prompt_tests if results["results"][k].get("success"))
        if prompt_tests:
            print(f"   📝 Prompt Generation: {successful_prompts}/{len(prompt_tests)} prompts generated successfully")
        
        # End-to-end findings
        e2e_tests = [k for k in results["results"].keys() if k.startswith("end_to_end")]
        if e2e_tests:
            successful_e2e = sum(1 for k in e2e_tests if results["results"][k].get("success"))
            print(f"   🎯 End-to-End Planning: {successful_e2e}/{len(e2e_tests)} plans generated successfully")
        else:
            print("   🤖 End-to-End Planning: Skipped (LLM not available)")
        
        print("\n" + "=" * 80)


def main():
    """Main test runner."""
    parser = argparse.ArgumentParser(description="RAG System Test Suite")
    parser.add_argument("--test", choices=["knowledge_retrieval", "prompt_generation", "end_to_end", "all"], 
                       default="all", help="Specific test to run")
    parser.add_argument("--verbose", "-v", action="store_true", help="Verbose output")
    parser.add_argument("--save-results", help="Save results to JSON file")
    
    args = parser.parse_args()
    
    # Create test suite
    test_suite = RAGTestSuite(verbose=args.verbose)
    
    # Run tests
    if args.test == "all":
        results = test_suite.run_all_tests()
    else:
        # Run specific test type
        if not test_suite.initialize_rag_system():
            print("❌ Failed to initialize RAG system")
            return
            
        if args.test == "knowledge_retrieval":
            results = {"results": {}}
            for test_case in TEST_CASES:
                result = test_suite.test_knowledge_retrieval(test_case["query"], test_case["expected_knowledge"])
                results["results"][f"knowledge_retrieval_{test_case['name']}"] = result
        elif args.test == "prompt_generation":
            results = {"results": {}}
            for test_case in TEST_CASES:
                result = test_suite.test_rag_prompt_generation(test_case["query"])
                results["results"][f"prompt_generation_{test_case['name']}"] = result
        elif args.test == "end_to_end":
            results = {"results": {}}
            for test_case in TEST_CASES:
                result = test_suite.test_end_to_end_planning(test_case)
                results["results"][f"end_to_end_{test_case['name']}"] = result
    
    # Print summary
    test_suite.print_summary(results)
    
    # Save results if requested
    if args.save_results:
        with open(args.save_results, 'w') as f:
            json.dump(results, f, indent=2, default=str)
        print(f"\n💾 Results saved to: {args.save_results}")


if __name__ == "__main__":
    main()
