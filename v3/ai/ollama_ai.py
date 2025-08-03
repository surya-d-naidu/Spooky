"""
Ollama-based AI module with llava support
"""

import requests
import json
import base64
import io
from typing import Dict, Any, List, Optional
from PIL import Image
import time

from ..core.interfaces import IAIInterface, SensorData, RobotState, RobotAction

class OllamaAI(IAIInterface):
    """AI module using Ollama with llava model"""
    
    def __init__(self, 
                 base_url: str = "http://localhost:11434",
                 model: str = "llava:7b",
                 timeout: int = 30):
        self.base_url = base_url
        self.model = model
        self.timeout = timeout
        self.api_url = f"{base_url}/api/generate"
        self.available = False
        
        # Decision history for context
        self.decision_history = []
        self.max_history = 10
        
    def initialize(self) -> bool:
        """Initialize the Ollama AI module"""
        try:
            # Check if Ollama is running
            response = requests.get(f"{self.base_url}/api/tags", timeout=5)
            response.raise_for_status()
            
            # Check if model is available
            models = response.json().get('models', [])
            model_names = [model['name'] for model in models]
            
            if self.model not in model_names:
                print(f"Model {self.model} not found. Available models: {model_names}")
                print(f"Please run: ollama pull {self.model}")
                return False
                
            self.available = True
            print(f"Ollama AI initialized with model: {self.model}")
            return True
            
        except Exception as e:
            print(f"Failed to initialize Ollama AI: {e}")
            return False
    
    def is_available(self) -> bool:
        """Check if AI service is available"""
        return self.available
    
    def make_decision(self, 
                     sensor_data: List[SensorData], 
                     current_state: RobotState,
                     context: Dict[str, Any]) -> RobotAction:
        """Make a decision based on sensor inputs"""
        
        if not self.available:
            return self._fallback_decision(current_state)
        
        # Find visual data
        visual_data = None
        other_sensors = []
        
        for data in sensor_data:
            if data.sensor_type == "camera" and "image" in data.data:
                visual_data = data
            else:
                other_sensors.append(data)
        
        # Create decision prompt
        prompt = self._create_decision_prompt(current_state, other_sensors, context)
        
        try:
            if visual_data:
                # Use vision-language model
                decision = self._make_visual_decision(visual_data, prompt)
            else:
                # Use text-only model
                decision = self._make_text_decision(prompt)
                
            # Add to history
            self.decision_history.append({
                'timestamp': time.time(),
                'state': current_state,
                'decision': decision,
                'sensors': len(sensor_data)
            })
            
            # Keep history size manageable
            if len(self.decision_history) > self.max_history:
                self.decision_history.pop(0)
                
            return decision
            
        except Exception as e:
            print(f"AI decision error: {e}")
            return self._fallback_decision(current_state)
    
    def analyze_scene(self, image_data: bytes, prompt: str) -> Dict[str, Any]:
        """Analyze visual scene with custom prompt"""
        
        if not self.available:
            return {"error": "AI not available", "analysis": "Unable to analyze"}
        
        try:
            # Encode image
            encoded_image = base64.b64encode(image_data).decode('utf-8')
            
            payload = {
                "model": self.model,
                "prompt": prompt,
                "images": [encoded_image],
                "stream": False
            }
            
            response = requests.post(self.api_url, json=payload, timeout=self.timeout)
            response.raise_for_status()
            
            result = response.json().get('response', '')
            
            # Try to parse as JSON first
            try:
                return json.loads(result)
            except:
                # Return as text analysis
                return {"analysis": result, "raw_response": result}
                
        except Exception as e:
            return {"error": str(e), "analysis": "Analysis failed"}
    
    def _create_decision_prompt(self, 
                               current_state: RobotState, 
                               sensor_data: List[SensorData],
                               context: Dict[str, Any]) -> str:
        """Create prompt for decision making"""
        
        personality = context.get('personality', {})
        robot_type = context.get('robot_type', 'quadruped_dog')
        
        prompt = f"""You are an autonomous {robot_type} robot with dog-like intelligence and personality.

Current State: {current_state.value}
Personality Traits: {personality}

Sensor Information:"""
        
        for data in sensor_data:
            prompt += f"\n- {data.sensor_type}: {data.data}"
        
        # Add recent history context
        if self.decision_history:
            recent = self.decision_history[-3:]  # Last 3 decisions
            prompt += f"\n\nRecent Actions: {[d['decision'].action_type for d in recent]}"
        
        prompt += f"""

Available Actions:
- explore: Move around and investigate environment
- investigate: Focus on specific object/area
- social_approach: Approach detected person/animal
- play: Playful behavior (spins, jumps)
- rest: Stay still and observe
- avoid: Move away from obstacles/threats
- follow: Follow detected entity
- alert: Heightened awareness mode

Respond with a JSON object:
{{
    "action": "action_name",
    "reasoning": "why this action",
    "duration": 3.5,
    "confidence": 0.8,
    "parameters": {{"direction": "forward", "speed": "normal"}},
    "priority": 1
}}

Be dog-like: curious about new things, friendly to people, playful, but cautious of dangers.
Consider your personality traits when making decisions."""

        return prompt
    
    def _make_visual_decision(self, visual_data: SensorData, prompt: str) -> RobotAction:
        """Make decision with visual input"""
        
        image_data = visual_data.data.get("image")
        if isinstance(image_data, str):
            # Already base64 encoded
            encoded_image = image_data
        else:
            # Encode image data
            encoded_image = base64.b64encode(image_data).decode('utf-8')
        
        payload = {
            "model": self.model,
            "prompt": prompt,
            "images": [encoded_image],
            "stream": False
        }
        
        response = requests.post(self.api_url, json=payload, timeout=self.timeout)
        response.raise_for_status()
        
        ai_response = response.json().get('response', '')
        return self._parse_ai_response(ai_response)
    
    def _make_text_decision(self, prompt: str) -> RobotAction:
        """Make decision with text only"""
        
        payload = {
            "model": self.model,
            "prompt": prompt,
            "stream": False
        }
        
        response = requests.post(self.api_url, json=payload, timeout=self.timeout)
        response.raise_for_status()
        
        ai_response = response.json().get('response', '')
        return self._parse_ai_response(ai_response)
    
    def _parse_ai_response(self, response: str) -> RobotAction:
        """Parse AI response into RobotAction"""
        
        try:
            # Try to extract JSON from response
            import re
            json_match = re.search(r'\{.*\}', response, re.DOTALL)
            
            if json_match:
                data = json.loads(json_match.group())
                
                return RobotAction(
                    action_type=data.get('action', 'rest'),
                    parameters=data.get('parameters', {}),
                    duration=float(data.get('duration', 3.0)),
                    priority=int(data.get('priority', 1)),
                    metadata={
                        'reasoning': data.get('reasoning', ''),
                        'confidence': float(data.get('confidence', 0.5)),
                        'ai_response': response
                    }
                )
            
        except Exception as e:
            print(f"Error parsing AI response: {e}")
        
        # Fallback parsing
        return self._fallback_parse(response)
    
    def _fallback_parse(self, response: str) -> RobotAction:
        """Fallback response parsing"""
        
        response_lower = response.lower()
        
        # Simple keyword matching
        if any(word in response_lower for word in ['explore', 'move', 'walk']):
            action = 'explore'
        elif any(word in response_lower for word in ['play', 'spin', 'jump']):
            action = 'play'
        elif any(word in response_lower for word in ['follow', 'approach']):
            action = 'social_approach'
        elif any(word in response_lower for word in ['investigate', 'check', 'look']):
            action = 'investigate'
        elif any(word in response_lower for word in ['avoid', 'back', 'retreat']):
            action = 'avoid'
        else:
            action = 'rest'
        
        return RobotAction(
            action_type=action,
            parameters={},
            duration=3.0,
            priority=1,
            metadata={'reasoning': 'fallback_parsing', 'confidence': 0.3}
        )
    
    def _fallback_decision(self, current_state: RobotState) -> RobotAction:
        """Fallback decision when AI is unavailable"""
        
        # Simple state-based fallback
        if current_state == RobotState.IDLE:
            action = 'explore'
        elif current_state == RobotState.MOVING:
            action = 'rest'
        else:
            action = 'rest'
        
        return RobotAction(
            action_type=action,
            parameters={},
            duration=2.0,
            priority=1,
            metadata={'reasoning': 'ai_unavailable_fallback', 'confidence': 0.2}
        )
    
    def get_conversation_summary(self) -> str:
        """Get summary of recent decisions for context"""
        if not self.decision_history:
            return "No previous decisions"
        
        recent = self.decision_history[-5:]
        summary = "Recent decisions: "
        summary += ", ".join([f"{d['decision'].action_type}({d['state'].value})" for d in recent])
        return summary
