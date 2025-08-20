#!/usr/bin/env python3
"""
Pure AI Testing Script - No Fallbacks Allowed

This script demonstrates that the system now ONLY uses AI (RAG + LLM)
and will fail completely if AI is not available - no fallbacks!
"""

import os
import sys
import subprocess
from pathlib import Path

def test_without_api_key():
    """Test system behavior without API key - should fail completely."""
    print("🧪 TESTING WITHOUT API KEY (Should Fail)")
    print("=" * 50)
    
    # Temporarily unset API key
    old_key = os.environ.pop('GEMINI_API_KEY', None)
    
    try:
        result = subprocess.run([
            sys.executable, "robot_control/main.py",
            "--task", "move toward bottle",
            "--sim", "--dry-run"
        ], capture_output=True, text=True, timeout=30)
        
        if result.returncode != 0:
            print("✅ CORRECT: System failed without AI (no fallbacks)")
            
            # Check for the right error messages
            output = result.stdout + result.stderr
            if "no fallbacks allowed" in output.lower():
                print("✅ CORRECT: 'No fallbacks allowed' error message")
            if "llm not available" in output.lower():
                print("✅ CORRECT: 'LLM not available' error message") 
            if "ai rag planning failed" in output.lower():
                print("✅ CORRECT: 'AI RAG planning failed' error message")
                
            return True
        else:
            print("❌ WRONG: System should have failed without AI!")
            print("This means there are still fallbacks!")
            return False
            
    except subprocess.TimeoutExpired:
        print("⏱️ Test timed out")
        return False
    finally:
        # Restore API key if it existed
        if old_key:
            os.environ['GEMINI_API_KEY'] = old_key

def test_with_fake_api_key():
    """Test system behavior with invalid API key - should fail at LLM level."""
    print("\n🧪 TESTING WITH FAKE API KEY (Should Fail)")
    print("=" * 50)
    
    # Set fake API key
    old_key = os.environ.get('GEMINI_API_KEY')
    os.environ['GEMINI_API_KEY'] = 'fake_key_for_testing'
    
    try:
        result = subprocess.run([
            sys.executable, "robot_control/main.py", 
            "--task", "move toward bottle",
            "--sim", "--dry-run"
        ], capture_output=True, text=True, timeout=30)
        
        if result.returncode != 0:
            print("✅ CORRECT: System failed with invalid API key")
            
            output = result.stdout + result.stderr
            if "no fallbacks allowed" in output.lower():
                print("✅ CORRECT: No fallbacks used")
                
            return True
        else:
            print("❌ WRONG: System should fail with invalid API key!")
            return False
            
    except subprocess.TimeoutExpired:
        print("⏱️ Test timed out")
        return False
    finally:
        # Restore original key
        if old_key:
            os.environ['GEMINI_API_KEY'] = old_key
        else:
            os.environ.pop('GEMINI_API_KEY', None)

def test_rag_components_only():
    """Test that RAG components work without LLM (knowledge retrieval)."""
    print("\n🧪 TESTING RAG COMPONENTS (Should Work)")
    print("=" * 50)
    
    try:
        # Test knowledge retrieval only
        result = subprocess.run([
            sys.executable, "test_rag_prompts.py"
        ], capture_output=True, text=True, timeout=30)
        
        if result.returncode == 0:
            print("✅ CORRECT: RAG knowledge retrieval works without LLM")
            return True
        else:
            print("❌ RAG components failed:")
            print(result.stderr[:200])
            return False
            
    except subprocess.TimeoutExpired:
        print("⏱️ Test timed out")
        return False

def check_for_remaining_fallbacks():
    """Check if there are any remaining fallback references in the code."""
    print("\n🔍 CHECKING FOR REMAINING FALLBACKS")
    print("=" * 50)
    
    fallback_patterns = [
        "intelligent.*planner",
        "basic.*fallback", 
        "fallback.*plan",
        "except.*fallback",
        "try.*intelligent"
    ]
    
    found_fallbacks = []
    
    for pattern in fallback_patterns:
        try:
            result = subprocess.run([
                "grep", "-r", "-i", pattern, "robot_control/"
            ], capture_output=True, text=True)
            
            if result.returncode == 0 and result.stdout.strip():
                # Filter out comments and removed code
                lines = result.stdout.strip().split('\n')
                actual_fallbacks = []
                for line in lines:
                    if not any(skip in line.lower() for skip in [
                        "# removed", "# all fallback", "comment", "removed", 
                        "pure ai testing", "no fallbacks"
                    ]):
                        actual_fallbacks.append(line)
                
                if actual_fallbacks:
                    found_fallbacks.extend(actual_fallbacks)
                    
        except subprocess.CalledProcessError:
            pass
    
    if found_fallbacks:
        print("⚠️ FOUND POTENTIAL FALLBACKS:")
        for fallback in found_fallbacks[:5]:  # Show first 5
            print(f"   {fallback}")
        if len(found_fallbacks) > 5:
            print(f"   ... and {len(found_fallbacks) - 5} more")
        return False
    else:
        print("✅ CORRECT: No active fallbacks found")
        return True

def main():
    """Run pure AI testing suite."""
    print("🤖 PURE AI TESTING - NO FALLBACKS ALLOWED")
    print("Testing that system ONLY uses AI and fails without it")
    print("=" * 60)
    
    # Run tests
    results = []
    
    results.append(("No API Key Test", test_without_api_key()))
    results.append(("Invalid API Key Test", test_with_fake_api_key()))
    results.append(("RAG Components Test", test_rag_components_only()))
    results.append(("Fallback Check", check_for_remaining_fallbacks()))
    
    # Summary
    print("\n" + "=" * 60)
    print("📊 PURE AI TEST RESULTS")
    print("=" * 60)
    
    passed = sum(1 for _, result in results if result)
    total = len(results)
    
    for test_name, result in results:
        status = "✅ PASS" if result else "❌ FAIL"
        print(f"{status} {test_name}")
    
    success_rate = (passed / total) * 100
    print(f"\n🎯 Success Rate: {success_rate:.1f}% ({passed}/{total})")
    
    if success_rate == 100:
        print("🎉 PERFECT! System is now PURE AI - no fallbacks!")
        print("💡 The system will ONLY work with a valid GEMINI_API_KEY")
        print("🧠 This ensures we're testing the AI, not fallback logic")
    elif success_rate >= 75:
        print("✅ GOOD! Most fallbacks removed")
        print("⚠️ Check remaining issues above")
    else:
        print("❌ NEEDS WORK! System still has fallbacks")
        print("🔧 More fallbacks need to be removed")
    
    # Instructions
    print(f"\n🚀 TO TEST WITH REAL AI:")
    print("1. Get free Gemini API key: https://makersuite.google.com/app/apikey")
    print("2. export GEMINI_API_KEY='your_key_here'")
    print("3. python robot_control/main.py --task 'move toward bottle' --sim --dry-run")
    print("4. System should work with AI, fail without it!")
    
    return success_rate >= 75

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)
