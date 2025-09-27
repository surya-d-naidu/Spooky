import time
import random
from enum import Enum
from typing import Dict, Any, Optional

class Emotion(Enum):
    HAPPY = "happy"
    EXCITED = "excited"
    CURIOUS = "curious"
    CALM = "calm"
    ALERT = "alert"
    PLAYFUL = "playful"
    TIRED = "tired"
    SCARED = "scared"
    AGGRESSIVE = "aggressive"
    CONFUSED = "confused"

class DogPersonality:
    def __init__(self):
        self.current_emotion = Emotion.CALM
        self.emotion_intensity = 0.5
        self.last_emotion_change = time.time()
        self.personality_traits = {
            "playfulness": 0.8,
            "friendliness": 0.9,
            "energy_level": 0.7,
            "curiosity": 0.8,
            "anxiety_level": 0.3
        }
        self.emotion_history = []
        self.action_cooldowns = {}
        
    def process_ai_response(self, ai_response: Dict[str, Any]) -> Dict[str, Any]:
        if not ai_response:
            return self._get_idle_behavior()
            
        emotion = ai_response.get("emotion", "curious")
        action = ai_response.get("action", "stand")
        confidence = ai_response.get("confidence", 0.5)
        duration = ai_response.get("duration", 3)
        
        emotion_obj = self._string_to_emotion(emotion)
        modified_response = self._apply_personality_filter(emotion_obj, action, confidence, duration)
        
        self._update_emotional_state(emotion_obj, confidence)
        self._log_emotion_history(emotion_obj, confidence)
        
        return modified_response
    
    def _string_to_emotion(self, emotion_str: str) -> Emotion:
        try:
            return Emotion(emotion_str.lower())
        except ValueError:
            return Emotion.CURIOUS
    
    def _apply_personality_filter(self, emotion: Emotion, action: str, confidence: float, duration: int) -> Dict[str, Any]:
        if action in self.action_cooldowns:
            if time.time() - self.action_cooldowns[action] < 5:
                action = self._get_alternative_action(emotion)
        
        self.action_cooldowns[action] = time.time()
        
        if emotion == Emotion.PLAYFUL and self.personality_traits["playfulness"] > 0.7:
            if random.random() < 0.3:
                action = "say_hi" if action == "stand" else action
                duration = min(duration + 2, 8)
        
        if emotion == Emotion.SCARED and self.personality_traits["anxiety_level"] > 0.5:
            if action in ["walk", "turn_left", "turn_right"]:
                action = "walk_backward"
            duration = max(duration - 1, 1)
        
        if emotion == Emotion.EXCITED and self.personality_traits["energy_level"] > 0.6:
            if action == "stand":
                action = random.choice(["walk", "turn_left", "turn_right", "say_hi"])
            duration = min(duration + 1, 6)
        
        confidence = min(confidence * (1 + self.personality_traits["friendliness"] * 0.2), 1.0)
        
        return {
            "emotion": emotion.value,
            "action": action,
            "confidence": confidence,
            "duration": duration,
            "personality_adjusted": True
        }
    
    def _get_alternative_action(self, emotion: Emotion) -> str:
        emotion_action_map = {
            Emotion.HAPPY: ["say_hi", "walk", "stand"],
            Emotion.EXCITED: ["walk", "turn_left", "turn_right", "say_hi"],
            Emotion.CURIOUS: ["turn_left", "turn_right", "walk"],
            Emotion.CALM: ["stand", "sit"],
            Emotion.ALERT: ["stand", "turn_left", "turn_right"],
            Emotion.PLAYFUL: ["say_hi", "walk", "turn_left", "turn_right"],
            Emotion.TIRED: ["sit", "stand"],
            Emotion.SCARED: ["walk_backward", "sit"],
            Emotion.AGGRESSIVE: ["stand", "turn_left", "turn_right"],
            Emotion.CONFUSED: ["turn_left", "turn_right", "stand"]
        }
        
        actions = emotion_action_map.get(emotion, ["stand"])
        return random.choice(actions)
    
    def _update_emotional_state(self, new_emotion: Emotion, confidence: float):
        if new_emotion != self.current_emotion:
            self.current_emotion = new_emotion
            self.last_emotion_change = time.time()
        
        self.emotion_intensity = min(confidence * 1.2, 1.0)
    
    def _log_emotion_history(self, emotion: Emotion, confidence: float):
        self.emotion_history.append({
            "emotion": emotion.value,
            "confidence": confidence,
            "timestamp": time.time()
        })
        
        if len(self.emotion_history) > 50:
            self.emotion_history.pop(0)
    
    def _get_idle_behavior(self) -> Dict[str, Any]:
        time_since_last_change = time.time() - self.last_emotion_change
        
        if time_since_last_change > 30:
            idle_actions = ["stand", "sit", "turn_left", "turn_right"]
            if random.random() < self.personality_traits["playfulness"]:
                idle_actions.extend(["say_hi", "walk"])
            
            action = random.choice(idle_actions)
            emotion = Emotion.CALM if random.random() < 0.7 else Emotion.CURIOUS
            
            return {
                "emotion": emotion.value,
                "action": action,
                "confidence": 0.4,
                "duration": random.randint(2, 5),
                "idle_behavior": True
            }
        
        return {
            "emotion": self.current_emotion.value,
            "action": "stand",
            "confidence": 0.3,
            "duration": 2,
            "maintain_state": True
        }
    
    def get_current_state(self) -> Dict[str, Any]:
        return {
            "current_emotion": self.current_emotion.value,
            "emotion_intensity": self.emotion_intensity,
            "time_since_change": time.time() - self.last_emotion_change,
            "recent_emotions": self.emotion_history[-5:] if self.emotion_history else []
        }
