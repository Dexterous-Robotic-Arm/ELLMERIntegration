# LLM Robot Planning (Gemini-Powered)

This repository is a modular, ROS-integrated framework for enabling robotic planning and execution using Google's Gemini large language model. It adapts the original ELLMER `llm_robot_planning` package to work with the UFactory xArm via Gemini API.

---

## 🌐 Project Overview

- **Goal**: Allow robots to understand and execute high-level instructions through LLM reasoning.
- **Backend**: Uses Gemini via Google AI SDK (https://ai.google.dev).
- **Hardware**: Designed for real-world deployment on UFactory xArm robotic platforms.
- **Architecture**: Gemini-invoked planning system with action execution through ROS nodes.

---

## 🗂 Structure

```bash
llm_robot_gemini/
│
├── .env.example
├── README.md
│
├── custom_gemini_examples/        # Task planning and schema prompting
│   ├── app.py
│   ├── custom_knowledge.md
│   ├── custom_gpt_instructions.md
│   ├── custom_gpt_action_schema.md
│   └── README.md
│
└── haystack_rag_pipelines/        # Retrieval-Augmented Generation (RAG)
    ├── evaluate_models.py
    ├── example1.py
    ├── example2.py
    ├── generate_questions.py
    ├── generated_questions.json
    ├── results.md
    ├── custom_knowledge.md
    ├── responses/
    └── README.md
```

---

## ⚙️ Installation

```bash
cd ~/catkin_ws/src
git clone https://github.com/your-org/llm_robot_gemini.git
cd ..
catkin_make
source devel/setup.bash
```

Install Python dependencies:
```bash
pip install -r requirements.txt
```

Set your environment:
```bash
cp .env.example .env
nano .env  # add your Gemini API key and robot connection details
```

---

## 🤖 Running the Example

```bash
cd custom_gemini_examples
python3 app.py
```

Ensure:
- You have Gemini API access.
- Robot gripper node is launched and reachable.
