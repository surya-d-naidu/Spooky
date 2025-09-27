import requests
import json
import logging
from typing import Optional, Dict, Any

class OllamaAI:
    def __init__(self, host="localhost", port=11434, model="llava"):
        self.base_url = f"http://{host}:{port}"
        self.model = model
        self.logger = logging.getLogger(__name__)
        
    def analyze_image(self, base64_image: str) -> Optional[Dict[str, Any]]:
        try:
            system_prompt = """You are the AI brain of a robotic dog. Analyze the image and respond with emotions and actions.

Available emotions: happy, excited, curious, calm, alert, playful, tired, scared, aggressive, confused
Available actions: walk, sit, stand, turn_left, turn_right, walk_backward, say_hi, end_walk

Respond in this exact JSON format:
{
    "emotion": "emotion_name",
    "action": "action_name", 
    "confidence": 0.8,
    "reason": "brief explanation",
    "duration": 3
}

Rules:
- Be reactive to what you see
- Show appropriate emotions for a dog
- Choose actions that match emotions
- Duration is in seconds (1-10)
- Higher confidence for clear situations"""

            user_prompt = "What do you see? How should the robotic dog react emotionally and physically?"

            payload = {
                "model": self.model,
                "prompt": user_prompt,
                "system": system_prompt,
                "images": [base64_image],
                "stream": False,
                "options": {
                    "temperature": 0.7,
                    "top_p": 0.9
                }
            }
            
            response = requests.post(
                f"{self.base_url}/api/generate",
                json=payload,
                timeout=30
            )
            
            if response.status_code == 200:
                result = response.json()
                response_text = result.get('response', '').strip()
                
                try:
                    if response_text.startswith('```json'):
                        response_text = response_text.split('```json')[1].split('```')[0].strip()
                    elif response_text.startswith('```'):
                        response_text = response_text.split('```')[1].split('```')[0].strip()
                    
                    parsed_response = json.loads(response_text)
                    
                    if all(key in parsed_response for key in ['emotion', 'action', 'confidence']):
                        return parsed_response
                    else:
                        self.logger.warning(f"Incomplete response from AI: {parsed_response}")
                        return self._get_default_response()
                        
                except json.JSONDecodeError as e:
                    self.logger.warning(f"Failed to parse AI response as JSON: {response_text}")
                    return self._extract_fallback_response(response_text)
                    
            else:
                self.logger.error(f"AI request failed with status {response.status_code}")
                return None
                
        except Exception as e:
            self.logger.error(f"Error communicating with AI: {str(e)}")
            return None
    
    def _extract_fallback_response(self, text: str) -> Dict[str, Any]:
        emotions = ["happy", "excited", "curious", "calm", "alert", "playful", "tired", "scared", "aggressive", "confused"]
        actions = ["walk", "sit", "stand", "turn_left", "turn_right", "walk_backward", "say_hi", "end_walk"]
        
        detected_emotion = "curious"
        detected_action = "stand"
        
        text_lower = text.lower()
        for emotion in emotions:
            if emotion in text_lower:
                detected_emotion = emotion
                break
                
        for action in actions:
            if action.replace('_', ' ') in text_lower or action in text_lower:
                detected_action = action
                break
        
        return {
            "emotion": detected_emotion,
            "action": detected_action,
            "confidence": 0.5,
            "reason": "Fallback parsing",
            "duration": 3
        }
    
    def _get_default_response(self) -> Dict[str, Any]:
        return {
            "emotion": "curious",
            "action": "stand",
            "confidence": 0.3,
            "reason": "Default response - no clear input",
            "duration": 2
        }
