# multi_agent_system.py
# Advanced Multi-Agent Engineering System
# Includes: ANSYS tools, Topology Optimization, Vehicle Dynamics + Memory

import json
from datetime import datetime
from typing import Dict, List, Any

class Agent:
    def __init__(self, name: str, role: str, capabilities: List[str], tools: List[str] = None):
        self.name = name
        self.role = role
        self.capabilities = capabilities
        self.tools = tools or []
        self.memory = []  # Agent-specific short-term memory
    
    def process(self, query: str, context: Dict = None) -> Dict:
        context = context or {}
        response = {
            "agent": self.name,
            "timestamp": datetime.now().isoformat(),
            "query": query,
            "reasoning": f"Using capabilities: {self.capabilities[:3]}...",
            "output": f"[{self.name}] Analyzed: {query}",
            "suggestions": [],
            "tools_used": []
        }
        
        # Simulate tool usage
        if "fluent" in self.name.lower() and any(k in query.lower() for k in ["aero", "floor", "diffuser"]):
            response["tools_used"].append("ANSYS_Fluent_CFD")
            response["output"] += "\n→ Recommended: Steady-state RANS with k-omega SST turbulence model."
            response["suggestions"].append("Use pressure far-field boundaries and refine mesh underbody.")
        
        elif "mechanical" in self.name.lower() or "topology" in self.name.lower():
            response["tools_used"].append("ANSYS_Mechanical_Topology")
            response["output"] += "\n→ Mass target: 50% reduction. Preserve stiffness."
            response["suggestions"].append("Validate final design with static structural analysis.")
        
        elif "dynamics" in self.name.lower():
            response["tools_used"].append("Vehicle_Dynamics_Model")
            response["output"] += "\n→ Key factors: Ride height, diffuser angle, yaw balance."
            response["suggestions"].append("Couple with Fluent for aero-load transfer.")
        
        self.memory.append({"query": query, "response": response["output"]})
        return response

class MemoryManager:
    def __init__(self):
        self.project_context = {}
        self.conversation_history = []
    
    def update_context(self, key: str, value: Any):
        self.project_context[key] = value
    
    def get_context(self) -> Dict:
        return self.project_context
    
    def add_to_history(self, entry: Dict):
        self.conversation_history.append(entry)
    
    def get_relevant_history(self, query: str) -> List[Dict]:
        # Simple keyword-based relevance (can upgrade to embeddings)
        return [h for h in self.conversation_history[-10:] if any(word in query.lower() for word in ["aero", "floor", "optimize", "simulation"])]

class MultiAgentSystem:
    def __init__(self):
        self.agents = self._initialize_agents()
        self.memory = MemoryManager()
        self.history = []
    
    def _initialize_agents(self):
        return {
            "mechanical": Agent(
                name="ANSYS_Mechanical",
                role="Structural & Mechanical Analysis",
                capabilities=["static", "fatigue", "modal", "nonlinear", "thermal"],
                tools=["Workbench", "Topology Optimization"]
            ),
            "fluent": Agent(
                name="ANSYS_Fluent",
                role="CFD & Aerodynamics",
                capabilities=["external flow", "turbulence", "heat transfer", "vehicle aero", "diffuser"],
                tools=["Fluent Meshing", "RANS/LES"]
            ),
            "dynamics": Agent(
                name="Vehicle_Dynamics",
                role="Motorsport Vehicle Dynamics",
                capabilities=["flat floor aero", "yaw moment", "load transfer", "suspension", "understeer/oversteer"],
                tools=["Bicycle Model", "Friction Circle Analysis"]
            ),
            "topology": Agent(
                name="Topology_Optimizer",
                role="Generative Design & Lightweighting",
                capabilities=["mass reduction", "3D printing optimization"],
                tools=["ANSYS Topology", "Fusion Generative"]
            ),
            "coordinator": Agent(
                name="System_Coordinator",
                role="Orchestrator",
                capabilities=["routing", "multi-agent coordination"]
            )
        }
    
    def route_and_process(self, user_query: str) -> Dict:
        self.memory.add_to_history({"query": user_query, "time": datetime.now().isoformat()})
        
        context = self.memory.get_context()
        relevant_history = self.memory.get_relevant_history(user_query)
        
        # Intelligent routing
        query_lower = user_query.lower()
        active_agents = []
        
        if any(word in query_lower for word in ["aero", "floor", "diffuser", "downforce", "drag", "cfd"]):
            active_agents.append(self.agents["fluent"])
            active_agents.append(self.agents["dynamics"])
        
        if any(word in query_lower for word in ["bracket", "stress", "optimize", "mass", "structure"]):
            active_agents.append(self.agents["mechanical"])
            active_agents.append(self.agents["topology"])
        
        if any(word in query_lower for word in ["handling", "yaw", "suspension", "ride height"]):
            active_agents.append(self.agents["dynamics"])
        
        # Coordinator always involved
        active_agents.append(self.agents["coordinator"])
        
        results = {}
        for agent in active_agents:
            result = agent.process(user_query, context)
            results[agent.name] = result
        
        final_response = {
            "query": user_query,
            "active_agents": [a.name for a in active_agents],
            "results": results,
            "context_summary": context,
            "history_length": len(self.memory.conversation_history)
        }
        
        return final_response

# Initialize
system = MultiAgentSystem()

# Example usage / testing
if __name__ == "__main__":
    test_queries = [
        "Explain how a flat floor and diffuser work in race cars and how to simulate it",
        "Optimize a bracket for 50% mass reduction",
        "Analyze ride height sensitivity for underbody aerodynamics"
    ]
    
    for q in test_queries:
        print(f"\n{'='*60}")
        print(f"QUERY: {q}")
        response = system.route_and_process(q)
        print(f"Agents used: {response['active_agents']}")
        print(f"Context items: {len(response['context_summary'])}")
