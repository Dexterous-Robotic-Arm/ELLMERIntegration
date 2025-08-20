#!/usr/bin/env python3
"""
Pure RAG-based Robot Control System

This module provides a PURE Retrieval-Augmented Generation (RAG) based robot control system
with semantic knowledge retrieval and AI-only planning.

Key Components:
- TrueRAGPlanner: Pure RAG system with vector database and semantic search
- RAGDocument: Knowledge document structure
- RAGContext: Planning context with retrieved knowledge

NO FALLBACKS - System requires AI (Gemini API key) to function.

Usage:
    from robot_control.rag_system import TrueRAGPlanner, RAGDocument, RAGContext
"""

from .true_rag_planner import TrueRAGPlanner, RAGDocument, RAGContext

__all__ = [
    'TrueRAGPlanner',
    'RAGDocument', 
    'RAGContext'
]
