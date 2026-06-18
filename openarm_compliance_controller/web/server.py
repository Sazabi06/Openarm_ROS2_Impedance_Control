#!/usr/bin/env python3
"""
Agent-L: Robotic Lecturer Dashboard — FastAPI Server
Serves static assets and provides a Gemini-powered API to answer questions
about the OpenArm system architecture, control laws, and vision pipelines.

Author: Antigravity AI
"""

import os
import sys
import json
import logging
from pathlib import Path
from typing import Optional

from fastapi import FastAPI, HTTPException, Request
from fastapi.staticfiles import StaticFiles
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
import uvicorn

try:
    import google.generativeai as genai
    HAS_GEMINI = True
except ImportError:
    HAS_GEMINI = False

# Setup logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("agent_l_server")

# Define paths
WEB_DIR = Path(__file__).parent.resolve()
STATIC_DIR = WEB_DIR / "static"
STATIC_DIR.mkdir(parents=True, exist_ok=True)

WORKSPACE_DIR = Path("/home/user/ros2_ws/src")
COMPLIANCE_DIR = WORKSPACE_DIR / "impedance_control" / "openarm_compliance_controller"
BRAIN_DIR = Path("/home/user/.gemini/antigravity/brain/a7d60202-aa60-4271-9c6b-0407bf12d883")

app = FastAPI(
    title="Agent-L: Robotic Lecturer Dashboard API",
    description="Backend API for the OpenArm V10 knowledge base.",
    version="1.0.0"
)

# Enable CORS for local development
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Request schema
class ChatRequest(BaseModel):
    message: str
    apiKey: Optional[str] = None
    history: Optional[list] = []

def load_system_context() -> str:
    """Reads core markdown files to build a rich first-principles context database."""
    context = []
    
    files_to_read = [
        ("ARCHITECTURE.md", WORKSPACE_DIR / "ARCHITECTURE.md"),
        ("AGENT_L_LECTURER.md", COMPLIANCE_DIR / "AGENT_L_LECTURER.md"),
        ("COMPLETE_NODE_GRAPH.md", BRAIN_DIR / "openarm_complete_node_graph.md"),
        ("PROPRIOCEPTIVE_FORCE.md", COMPLIANCE_DIR / "PROPRIOCEPTIVE_FORCE.md"),
        ("TEACH_MODE_GUIDE.md", COMPLIANCE_DIR / "Enable_Teaching_Biarm.md"),
    ]
    
    for label, path in files_to_read:
        if path.exists():
            try:
                content = path.read_text(encoding="utf-8")
                context.append(f"=== DATABASE: {label} ===\n{content}\n")
                logger.info(f"Loaded context file: {label}")
            except Exception as e:
                logger.error(f"Failed to read context file {label}: {e}")
        else:
            logger.warning(f"Context file not found: {path}")
            
    if not context:
        return "No system architecture files found. Answer general robotics questions."
        
    return "\n\n".join(context)

# Preload architecture context
SYSTEM_CONTEXT = load_system_context()

SYSTEM_INSTRUCTION = """
You are Dr. L (Lecturer), a world-class robotics professor and the official knowledge transfer expert for the OpenArm V10 bimanual robot project.
Your job is to explain this robot's architecture, control theory, vision systems, and dataset recording to the project leader (the Boss), co-workers from adjacent teams, and new developers.

STUDY AND ADHERE STRICTLY TO THE SYSTEM DATABASE PROVIDED. Ground every explanation in the actual files loaded in your context:
- C++ Variable Impedance Controller (VIC) code details
- The joint-space impedance law: τ_cmd = τ_ff + Kp(q_des - q) + Kd(dq_des - dq)
- TRAC-IK parallel solver vs KDL details
- YOLOv8 perception, depth back-projection, and SVD hand-eye calibration
- Conda / ROS 2 split and UDP bridge node

COMMUNICATION STYLE:
1. Explain from first-principles first using clear, physical analogies (like springs, dampers, weightless helpers).
2. Be accessible but highly precise. Quote exact files, line ranges, or parameters from the database.
3. Adopt a humble, professional, and mathematically rigorous academic tone. Avoid overconfident superlatives like 'perfectly' or 'flawlessly'.
4. Format your output in beautiful Github-style Markdown (using lists, code blocks, bold text, and math notation).
5. If the user asks about something outside the database, answer using general robotics and controls knowledge but clarify what is specific to the OpenArm design.
"""

@app.post("/api/chat")
async def chat_endpoint(payload: ChatRequest):
    if not HAS_GEMINI:
        raise HTTPException(
            status_code=500,
            detail="google-generativeai is not installed in the server environment."
        )
        
    # Determine API key (payload key overrides environment variable)
    api_key = payload.apiKey or os.environ.get("GEMINI_API_KEY")
    if not api_key:
        raise HTTPException(
            status_code=401,
            detail="No Gemini API Key provided. Please set the GEMINI_API_KEY environment variable or enter it in the UI settings panel."
        )
        
    try:
        genai.configure(api_key=api_key)
        
        # Configure the generation model
        model = genai.GenerativeModel(
            model_name="gemini-1.5-flash",
            system_instruction=SYSTEM_INSTRUCTION
        )
        
        # Build prompt incorporating user message and preloaded database context
        prompt = f"""
Here is the official OpenArm V10 repository database context for your reference:
{SYSTEM_CONTEXT}

User's Question: {payload.message}
"""
        # Convert simple payload history to Gemini API format if provided
        contents = []
        for h in payload.history:
            role = "user" if h.get("role") == "user" else "model"
            contents.append({"role": role, "parts": [h.get("content", "")]})
            
        contents.append({"role": "user", "parts": [prompt]})
        
        response = model.generate_content(contents)
        return {"response": response.text}
        
    except Exception as e:
        logger.error(f"Gemini API generation failed: {e}")
        raise HTTPException(
            status_code=500,
            detail=f"Gemini API Error: {str(e)}"
        )

@app.get("/api/status")
async def status_endpoint():
    """Returns the health status of the dashboard and database parsing."""
    api_key_configured = bool(os.environ.get("GEMINI_API_KEY"))
    return {
        "status": "online",
        "has_gemini_installed": HAS_GEMINI,
        "is_api_key_env_set": api_key_configured,
        "context_files_loaded": [
            "ARCHITECTURE.md",
            "AGENT_L_LECTURER.md",
            "COMPLETE_NODE_GRAPH.md",
            "PROPRIOCEPTIVE_FORCE.md",
            "TEACH_MODE_GUIDE.md"
        ]
    }

@app.get("/api/docs")
async def docs_endpoint(file: str):
    """Dynamically reads and returns a markdown file to the web frontend."""
    file_mapping = {
        "architecture": WORKSPACE_DIR / "ARCHITECTURE.md",
        "lecturer": COMPLIANCE_DIR / "AGENT_L_LECTURER.md",
        "complete_node_graph": BRAIN_DIR / "openarm_complete_node_graph.md",
        "proprioceptive_force": COMPLIANCE_DIR / "PROPRIOCEPTIVE_FORCE.md",
        "teach_mode_guide": COMPLIANCE_DIR / "Enable_Teaching_Biarm.md"
    }
    
    path = file_mapping.get(file)
    if not path or not path.exists():
        raise HTTPException(
            status_code=404,
            detail=f"Documentation file '{file}' not found in local workspace."
        )
        
    try:
        content = path.read_text(encoding="utf-8")
        return {"content": content}
    except Exception as e:
        logger.error(f"Failed to read file {file}: {e}")
        raise HTTPException(
            status_code=500,
            detail=f"Error reading document: {str(e)}"
        )

# Mount static files (HTML, CSS, JS) at the root
app.mount("/", StaticFiles(directory=STATIC_DIR, html=True), name="static")

def main():
    port = int(os.environ.get("PORT", 8000))
    # Expose to 0.0.0.0 so colleagues on the intranet/office Wi-Fi can connect directly
    logger.info(f"Starting Agent-L Lecturer Server on http://0.0.0.0:{port}")
    uvicorn.run(app, host="0.0.0.0", port=port)

if __name__ == "__main__":
    main()
