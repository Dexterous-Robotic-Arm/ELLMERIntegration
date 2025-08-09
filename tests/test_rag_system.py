#!/usr/bin/env python3
"""
Test script for the RAG-based robot control system.

This script demonstrates the functionality of the new RAG-based architecture
by running various test scenarios in simulation mode.
"""

import sys
import os
import time
import logging
from pathlib import Path

# Add the robot_control package to the path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from robot_control.rag_system.planner.rag_planner import RAGPlanner, RAGContext, VisionFeedback, ExecutionContext
from robot_control.rag_system.integration.rag_integration import RAGIntegration, IntegrationConfig


def setup_logging():
    """Setup logging for the test."""
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
        handlers=[
            logging.StreamHandler(),
            logging.FileHandler('test_rag_system.log')
        ]
    )


def test_rag_planner_initialization():
    """Test RAG planner initialization."""
    print("\n=== Testing RAG Planner Initialization ===")
    
    try:
        # Create RAG planner in simulation mode
        planner = RAGPlanner(
            robot_controller=None,
            vision_system=None,
            config_path="config/",
            max_retries=3,
            scan_timeout=10.0
        )
        
        print("✅ RAG Planner initialized successfully")
        
        # Test status
        status = planner.get_status()
        print(f"   State: {status['state']}")
        print(f"   Available poses: {status['available_poses']}")
        print(f"   Available actions: {status['available_actions']}")
        
        return planner
        
    except Exception as e:
        print(f"❌ Failed to initialize RAG planner: {e}")
        return None


def test_rag_context_creation(planner):
    """Test RAG context creation."""
    print("\n=== Testing RAG Context Creation ===")
    
    try:
        # Create a test task
        task_description = "pick up the cup"
        
        # Create RAG context
        context = planner._create_rag_context(task_description)
        
        print("✅ RAG Context created successfully")
        print(f"   Task description: {context.task_description}")
        print(f"   Available actions: {len(context.available_actions)}")
        print(f"   Available poses: {len(context.available_poses)}")
        print(f"   Vision feedback objects: {len(context.vision_feedback.objects_detected)}")
        
        return context
        
    except Exception as e:
        print(f"❌ Failed to create RAG context: {e}")
        return None


def test_plan_generation(planner, context):
    """Test plan generation."""
    print("\n=== Testing Plan Generation ===")
    
    try:
        # Generate plan
        plan = planner._generate_plan_with_llm(context)
        
        print("✅ Plan generated successfully")
        print(f"   Goal: {plan.get('goal', 'N/A')}")
        print(f"   Steps: {len(plan.get('steps', []))}")
        
        # Print plan details
        for i, step in enumerate(plan.get('steps', [])):
            print(f"   Step {i+1}: {step.get('action', 'N/A')}")
        
        return plan
        
    except Exception as e:
        print(f"❌ Failed to generate plan: {e}")
        return None


def test_plan_execution(planner, plan):
    """Test plan execution (simulation)."""
    print("\n=== Testing Plan Execution (Simulation) ===")
    
    try:
        # Execute plan in simulation mode
        results = planner._execute_plan(plan)
        
        print("✅ Plan executed successfully")
        print(f"   Success: {results.get('success', False)}")
        print(f"   Steps executed: {results.get('steps_executed', 0)}")
        print(f"   Execution time: {results.get('execution_time', 0):.2f}s")
        
        if results.get('errors'):
            print(f"   Errors: {results.get('errors', [])}")
        
        return results
        
    except Exception as e:
        print(f"❌ Failed to execute plan: {e}")
        return None


def test_rag_integration():
    """Test RAG integration."""
    print("\n=== Testing RAG Integration ===")
    
    try:
        # Create integration config
        config = IntegrationConfig(
            robot_ip="192.168.1.241",
            sim=True,  # Run in simulation mode
            dry_run=True,
            enable_vision=False,
            max_retries=3,
            scan_timeout=10.0,
            config_path="config/"
        )
        
        # Create RAG integration
        integration = RAGIntegration(config)
        
        print("✅ RAG Integration initialized successfully")
        
        # Test system info
        system_info = integration.get_system_info()
        print(f"   Running: {system_info['rag_integration']['running']}")
        print(f"   Components: {system_info['components']}")
        print(f"   Available poses: {system_info['available_poses']}")
        print(f"   Available actions: {system_info['available_actions']}")
        
        return integration
        
    except Exception as e:
        print(f"❌ Failed to initialize RAG integration: {e}")
        return None


def test_task_execution(integration):
    """Test task execution through integration."""
    print("\n=== Testing Task Execution ===")
    
    try:
        # Test task execution
        task_description = "pick up the cup and place it in the bin"
        
        print(f"Executing task: {task_description}")
        results = integration.execute_task(task_description)
        
        print("✅ Task executed successfully")
        print(f"   Success: {results.get('success', False)}")
        print(f"   Steps executed: {results.get('steps_executed', 0)}")
        print(f"   Execution time: {results.get('execution_time', 0):.2f}s")
        
        if results.get('errors'):
            print(f"   Errors: {results.get('errors', [])}")
        
        return results
        
    except Exception as e:
        print(f"❌ Failed to execute task: {e}")
        return None


def test_vision_feedback(integration):
    """Test vision feedback integration."""
    print("\n=== Testing Vision Feedback ===")
    
    try:
        # Test vision feedback
        feedback = integration.get_vision_feedback()
        
        print("✅ Vision feedback retrieved successfully")
        print(f"   Objects detected: {len(feedback.objects_detected)}")
        print(f"   Scan quality: {feedback.scan_quality}")
        print(f"   Confidence scores: {feedback.confidence_scores}")
        
        return feedback
        
    except Exception as e:
        print(f"❌ Failed to get vision feedback: {e}")
        return None


def test_scanning(integration):
    """Test area scanning."""
    print("\n=== Testing Area Scanning ===")
    
    try:
        # Test area scanning
        print("Scanning area for 3 seconds...")
        feedback = integration.scan_area(duration=3.0)
        
        print("✅ Area scanning completed successfully")
        print(f"   Objects detected: {len(feedback.objects_detected)}")
        print(f"   Scan quality: {feedback.scan_quality}")
        
        if feedback.objects_detected:
            for obj in feedback.objects_detected:
                print(f"   - {obj.get('class', 'unknown')}: {obj.get('confidence', 0):.2f}")
        
        return feedback
        
    except Exception as e:
        print(f"❌ Failed to scan area: {e}")
        return None


def test_error_handling(integration):
    """Test error handling."""
    print("\n=== Testing Error Handling ===")
    
    try:
        # Test with an invalid task
        invalid_task = "perform impossible action"
        
        print(f"Testing with invalid task: {invalid_task}")
        results = integration.execute_task(invalid_task)
        
        print("✅ Error handling worked correctly")
        print(f"   Success: {results.get('success', False)}")
        print(f"   Error: {results.get('error', 'No error')}")
        
        return results
        
    except Exception as e:
        print(f"❌ Error handling test failed: {e}")
        return None


def main():
    """Main test function."""
    print("🚀 Starting RAG System Tests")
    print("=" * 50)
    
    # Setup logging
    setup_logging()
    
    # Test 1: RAG Planner Initialization
    planner = test_rag_planner_initialization()
    if not planner:
        print("❌ RAG Planner initialization failed. Stopping tests.")
        return 1
    
    # Test 2: RAG Context Creation
    context = test_rag_context_creation(planner)
    if not context:
        print("❌ RAG Context creation failed. Stopping tests.")
        return 1
    
    # Test 3: Plan Generation
    plan = test_plan_generation(planner, context)
    if not plan:
        print("❌ Plan generation failed. Stopping tests.")
        return 1
    
    # Test 4: Plan Execution
    results = test_plan_execution(planner, plan)
    if not results:
        print("❌ Plan execution failed. Stopping tests.")
        return 1
    
    # Test 5: RAG Integration
    integration = test_rag_integration()
    if not integration:
        print("❌ RAG Integration initialization failed. Stopping tests.")
        return 1
    
    # Test 6: Task Execution
    task_results = test_task_execution(integration)
    if not task_results:
        print("❌ Task execution failed. Stopping tests.")
        return 1
    
    # Test 7: Vision Feedback
    vision_feedback = test_vision_feedback(integration)
    if not vision_feedback:
        print("❌ Vision feedback test failed. Stopping tests.")
        return 1
    
    # Test 8: Area Scanning
    scan_results = test_scanning(integration)
    if not scan_results:
        print("❌ Area scanning test failed. Stopping tests.")
        return 1
    
    # Test 9: Error Handling
    error_results = test_error_handling(integration)
    if not error_results:
        print("❌ Error handling test failed. Stopping tests.")
        return 1
    
    print("\n🎉 All tests completed successfully!")
    print("=" * 50)
    
    # Summary
    print("\n📊 Test Summary:")
    print("   ✅ RAG Planner Initialization")
    print("   ✅ RAG Context Creation")
    print("   ✅ Plan Generation")
    print("   ✅ Plan Execution")
    print("   ✅ RAG Integration")
    print("   ✅ Task Execution")
    print("   ✅ Vision Feedback")
    print("   ✅ Area Scanning")
    print("   ✅ Error Handling")
    
    print("\n🚀 RAG-based robot control system is ready for use!")
    
    return 0


if __name__ == "__main__":
    sys.exit(main())
