# Haystack RAG Pipelines (Gemini)

This folder includes scripts and configurations to run retrieval-augmented generation (RAG) workflows using Gemini for question-answering or evaluation based on robotics domain data.

## Files

- `generate_questions.py`: Creates domain-specific questions.
- `evaluate_models.py`: Benchmarks Gemini's answers.
- `custom_knowledge.md`: Internal knowledge base.
- `generated_questions.json`: Stores the generated question set.
- `responses/`: Stores LLM responses and metrics.

## How to Use

1. Generate questions:
   ```bash
   python3 generate_questions.py
   ```

2. Evaluate responses:
   ```bash
   python3 evaluate_models.py
   ```

Make sure your `.env` includes valid Gemini credentials.
