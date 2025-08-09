#!/usr/bin/env python3
"""
RAG-based Robot Control System

This module provides a Retrieval-Augmented Generation (RAG) based robot control system
that uses LLM as the central decision maker for robot tasks.

Key Components:
- RAGPlanner: Main planning component with enhanced movement logic
- MovementLogic: Enhanced movement logic from detect_and_move.py
- RAGIntegration: Integration layer for connecting components
- rag_main: Main entry point for RAG-based system

Usage:
    from robot_control.rag_system import RAGPlanner, MovementLogic
    from robot_control.rag_system.rag_main import main as rag_main
"""

from .planner.rag_planner import RAGPlanner
from .planner.movement_logic import MovementLogic, CameraConfig, ObjectDetection
from .integration.rag_integration import RAGIntegration, IntegrationConfig

__all__ = [
    'RAGPlanner',
    'MovementLogic', 
    'CameraConfig',
    'ObjectDetection',
    'RAGIntegration',
    'IntegrationConfig'
]
