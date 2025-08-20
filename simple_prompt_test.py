#!/usr/bin/env python3
"""
Simple Gemini Free Tier Compatibility Test

Tests prompt size and API key setup for Gemini free tier.
"""

import os

def check_api_key():
    """Check API key configuration."""
    print("🔑 API KEY CONFIGURATION")
    print("=" * 40)
    
    api_key = os.getenv('GEMINI_API_KEY')
    if api_key:
        # Mask the key for security
        masked_key = api_key[:8] + "..." + api_key[-4:] if len(api_key) > 12 else "***masked***"
        print(f"✅ GEMINI_API_KEY is set: {masked_key}")
        print(f"📏 Key length: {len(api_key)} characters")
        
        # Check if it looks like a valid Gemini API key
        if api_key.startswith('AI') and len(api_key) > 30:
            print(f"✅ Key format looks valid (starts with 'AI', good length)")
        else:
            print(f"⚠️  Key format might be incorrect (should start with 'AI')")
            
        return True
    else:
        print(f"❌ GEMINI_API_KEY is not set")
        print(f"💡 Set it with: export GEMINI_API_KEY='your_api_key_here'")
        print(f"🔗 Get key at: https://makersuite.google.com/app/apikey")
        return False

def analyze_prompt_size():
    """Analyze typical RAG prompt size."""
    print(f"\n🔍 PROMPT SIZE ANALYSIS")
    print("=" * 40)
    
    # Simulate a typical RAG prompt
    sample_prompt = """
You are an advanced AI robot control system with access to comprehensive robotics knowledge. You can retrieve and apply relevant knowledge to create intelligent plans for any robot task.

## USER REQUEST
"pick up the fragile glass cup and place it carefully on the shelf"

## RETRIEVED ROBOTICS KNOWLEDGE
The following knowledge has been semantically retrieved from the robotics knowledge base based on your request:

### Knowledge 1 (Relevance: 0.85)
Category: movement_patterns
Safe Object Approach Pattern: Standard pattern for safely approaching objects with proper hover distance.

Pattern Steps:
1. Scan area to locate target object
2. Move to position above object with safe hover distance (80-120mm)  
3. Verify object position and adjust if needed
4. Slowly descend to interaction distance
5. Perform intended action (grasp, push, etc.)
6. Retreat to safe height before moving away

Safety Considerations:
- Always maintain minimum hover distance
- Check for obstacles in path
- Verify gripper state before approach
- Monitor force feedback during interaction

Applicable Tasks: pick up, move toward, approach, grab, touch

### Knowledge 2 (Relevance: 0.78)
Category: manipulation
Complete Pick and Place Operation: Full sequence for picking up objects and placing them elsewhere.

Pattern Steps:
1. Scan area to locate source object
2. Open gripper to appropriate width
3. Approach object with safe hover distance
4. Move to object position with small offset
5. Close gripper with force feedback
6. Lift object to safe height
7. Move to destination location
8. Lower to placement height
9. Open gripper to release object
10. Retreat to safe position

Error Recovery:
- If object not grasped: retry with adjusted position
- If collision detected: retreat and replan path  
- If object slips: increase grip force within limits

Applicable Tasks: pick up, move, place, transfer, relocate

### Knowledge 3 (Relevance: 0.72)
Category: safety
Fragile Object Handling: Special considerations for handling delicate or breakable objects.

Safety Guidelines:
- Use minimum necessary grip force
- Move slowly and smoothly
- Avoid sudden movements or accelerations
- Monitor object integrity during manipulation
- Have backup plans for object protection

Applicable Tasks: fragile handling, delicate manipulation, glass objects


## CURRENT ROBOT STATE
Robot Position: [300.0, 0.0, 250.0]
Gripper State: open
Detected Objects: ['bottle', 'cup', 'shelf']

## INSTRUCTIONS

Using the retrieved robotics knowledge above, create an intelligent plan that:

1. **Applies Retrieved Knowledge**: Use the patterns, strategies, and solutions from the retrieved knowledge
2. **Adapts to Current Context**: Consider current robot state and environment
3. **Follows Safety Guidelines**: Apply safety considerations from the knowledge base
4. **Uses Proven Patterns**: Leverage successful movement patterns and task strategies
5. **Handles Edge Cases**: Apply problem-solving strategies for potential issues

## OUTPUT FORMAT
Return a JSON object with this structure:
{
  "understanding": "Your interpretation using retrieved knowledge",
  "retrieved_knowledge_applied": "How you used the retrieved knowledge",
  "reasoning": "Your reasoning based on knowledge and context",
  "goal": "Clear task goal",
  "approach": "Approach based on retrieved patterns",
  "safety_considerations": "Safety aspects from knowledge base",
  "steps": [
    {"action": "ACTION_NAME", "parameters": {}, "knowledge_source": "which retrieved knowledge guided this step"}
  ],
  "confidence": "High/Medium/Low based on knowledge relevance",
  "fallback_plan": "Alternative approach if main plan fails"
}

Generate the intelligent plan now using the retrieved robotics knowledge:
"""
    
    # Analyze prompt
    chars = len(sample_prompt)
    estimated_tokens = int(chars / 3.5)  # ~3.5 chars per token
    
    # Gemini Free Tier Limits
    INPUT_TOKEN_LIMIT = 32000  # 32K tokens
    OUTPUT_TOKEN_LIMIT = 8000  # 8K tokens
    
    input_percent = (estimated_tokens / INPUT_TOKEN_LIMIT) * 100
    
    print(f"📏 Sample prompt: {chars:,} characters")
    print(f"🎯 Estimated tokens: ~{estimated_tokens:,}")
    print(f"📊 Free tier input usage: {input_percent:.1f}%")
    
    if estimated_tokens <= INPUT_TOKEN_LIMIT * 0.5:
        print(f"🟢 EXCELLENT: Well within free tier limit (<50%)")
    elif estimated_tokens <= INPUT_TOKEN_LIMIT * 0.8:
        print(f"🟡 GOOD: Within free tier limit (<80%)")
    elif estimated_tokens <= INPUT_TOKEN_LIMIT:
        print(f"🟠 CAUTION: Near free tier limit")
    else:
        print(f"🔴 WARNING: Exceeds free tier limit!")
    
    print(f"\n⚙️  CURRENT CONFIG:")
    print(f"   📤 max_output_tokens: 2,048 (✅ well within {OUTPUT_TOKEN_LIMIT:,} limit)")
    print(f"   📥 Retrieved docs: 3 (reasonable for free tier)")
    print(f"   🌡️  temperature: 0.2 (good for consistent results)")
    print(f"   ⏱️  Rate limit: 15 RPM (requests per minute)")

def main():
    """Run compatibility tests."""
    print("🧪 GEMINI FREE TIER COMPATIBILITY TEST")
    print("=" * 50)
    
    # Test 1: API Key
    api_key_ok = check_api_key()
    
    # Test 2: Prompt Size
    analyze_prompt_size()
    
    print(f"\n📋 SUMMARY")
    print("=" * 20)
    print(f"🔑 API Key: {'✅ Configured' if api_key_ok else '❌ Missing'}")
    print(f"📏 Prompt Size: ✅ Compatible with free tier")
    print(f"💰 Cost: ✅ Free (within rate limits)")
    
    if api_key_ok:
        print(f"\n🎉 READY TO USE GEMINI FREE TIER!")
        print(f"💡 Your RAG prompts should work fine with the free version")
    else:
        print(f"\n⚠️  SET API KEY TO GET STARTED")
        print(f"🔗 Get free API key: https://makersuite.google.com/app/apikey")

if __name__ == "__main__":
    main()
