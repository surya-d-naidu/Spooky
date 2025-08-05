"""
Dynamic personality system for autonomous robots
"""

import time
import random
import math
from typing import Dict, Any

try:
    from ..core.interfaces import IPersonalityInterface, RobotAction
except ImportError:
    import sys
    import os
    sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    from core.interfaces import IPersonalityInterface, RobotAction

class DogPersonality(IPersonalityInterface):
    """Dog-like personality system"""
    
    def __init__(self, base_traits: Dict[str, float] = None):
        # Base personality traits (0.0 to 1.0)
        self.traits = base_traits or {
            'energy': 0.8,
            'curiosity': 0.7,
            'playfulness': 0.6,
            'sociability': 0.8,
            'courage': 0.5,
            'independence': 0.4,
            'loyalty': 0.9,
            'alertness': 0.6
        }
        
        # Temporary modifiers
        self.temp_modifiers = {}
        
        # Experience tracking
        self.experiences = []
        self.max_experiences = 50
        
        # Time-based factors
        self.last_update = time.time()
        self.circadian_phase = 0.0  # 0-1 representing daily cycle
        
    def get_personality_traits(self) -> Dict[str, float]:
        """Get current personality traits including temporary modifiers"""
        current_traits = self.traits.copy()
        
        # Apply temporary modifiers
        for trait, modifier in self.temp_modifiers.items():
            if trait in current_traits:
                current_traits[trait] = max(0.0, min(1.0, current_traits[trait] + modifier))
        
        # Apply circadian rhythm effects
        current_traits = self._apply_circadian_effects(current_traits)
        
        return current_traits
    
    def update_personality(self, experience: Dict[str, Any]):
        """Update personality based on experience"""
        current_time = time.time()
        
        # Add experience to history
        experience['timestamp'] = current_time
        self.experiences.append(experience)
        
        # Keep experience history manageable
        if len(self.experiences) > self.max_experiences:
            self.experiences.pop(0)
        
        # Update traits based on experience
        self._process_experience(experience)
        
        # Natural personality drift over time
        self._natural_personality_drift()
        
        # Update circadian phase
        self._update_circadian_phase()
        
        # Decay temporary modifiers
        self._decay_temp_modifiers()
        
        self.last_update = current_time
    
    def influence_decision(self, base_action: RobotAction) -> RobotAction:
        """Modify action based on personality"""
        traits = self.get_personality_traits()
        
        # Create modified action
        modified_action = RobotAction(
            action_type=base_action.action_type,
            parameters=base_action.parameters.copy(),
            duration=base_action.duration,
            priority=base_action.priority,
            metadata=base_action.metadata.copy() if base_action.metadata else {}
        )
        
        # Modify based on personality traits
        modified_action = self._apply_energy_influence(modified_action, traits['energy'])
        modified_action = self._apply_playfulness_influence(modified_action, traits['playfulness'])
        modified_action = self._apply_sociability_influence(modified_action, traits['sociability'])
        modified_action = self._apply_courage_influence(modified_action, traits['courage'])
        modified_action = self._apply_curiosity_influence(modified_action, traits['curiosity'])
        
        # Add personality metadata
        modified_action.metadata['personality_influence'] = traits
        
        return modified_action
    
    def _process_experience(self, experience: Dict[str, Any]):
        """Process experience and update traits"""
        exp_type = experience.get('type', 'unknown')
        outcome = experience.get('outcome', 'neutral')
        intensity = experience.get('intensity', 0.5)
        
        # Small trait adjustments based on experience
        adjustment = 0.02 * intensity  # Small incremental changes
        
        if exp_type == 'social_interaction':
            if outcome == 'positive':
                self.traits['sociability'] = min(1.0, self.traits['sociability'] + adjustment)
                self.traits['playfulness'] = min(1.0, self.traits['playfulness'] + adjustment * 0.5)
            elif outcome == 'negative':
                self.traits['sociability'] = max(0.0, self.traits['sociability'] - adjustment)
                
        elif exp_type == 'exploration':
            if outcome == 'interesting':
                self.traits['curiosity'] = min(1.0, self.traits['curiosity'] + adjustment)
            elif outcome == 'dangerous':
                self.traits['courage'] = max(0.0, self.traits['courage'] - adjustment)
                self.traits['independence'] = max(0.0, self.traits['independence'] - adjustment * 0.5)
                
        elif exp_type == 'play':
            self.traits['playfulness'] = min(1.0, self.traits['playfulness'] + adjustment * 0.5)
            self.traits['energy'] = min(1.0, self.traits['energy'] + adjustment * 0.3)
            
        elif exp_type == 'rest':
            self.traits['energy'] = min(1.0, self.traits['energy'] + adjustment)
    
    def _natural_personality_drift(self):
        """Simulate natural personality changes over time"""
        drift_amount = 0.001  # Very small changes
        
        for trait in self.traits:
            # Random walk with slight tendency toward middle values
            change = random.uniform(-drift_amount, drift_amount)
            
            # Slight regression to mean (0.5)
            if self.traits[trait] > 0.5:
                change -= drift_amount * 0.1
            else:
                change += drift_amount * 0.1
                
            self.traits[trait] = max(0.0, min(1.0, self.traits[trait] + change))
    
    def _update_circadian_phase(self):
        """Update circadian rhythm phase"""
        # Simulate daily cycle (24 hours = 2π radians)
        hours_per_day = 24
        current_hour = (time.time() / 3600) % hours_per_day
        self.circadian_phase = current_hour / hours_per_day
    
    def _apply_circadian_effects(self, traits: Dict[str, float]) -> Dict[str, float]:
        """Apply circadian rhythm effects to traits"""
        modified_traits = traits.copy()
        
        # Energy varies with time of day
        # Peak energy in morning (0.25) and evening (0.75), low at night
        energy_modifier = 0.3 * math.sin(2 * math.pi * self.circadian_phase + math.pi/2)
        modified_traits['energy'] = max(0.0, min(1.0, traits['energy'] + energy_modifier))
        
        # Alertness higher during day
        alertness_modifier = 0.2 * math.sin(2 * math.pi * self.circadian_phase + math.pi/2)
        modified_traits['alertness'] = max(0.0, min(1.0, traits['alertness'] + alertness_modifier))
        
        return modified_traits
    
    def _decay_temp_modifiers(self):
        """Decay temporary personality modifiers"""
        decay_rate = 0.1
        
        for trait in list(self.temp_modifiers.keys()):
            self.temp_modifiers[trait] *= (1 - decay_rate)
            
            # Remove very small modifiers
            if abs(self.temp_modifiers[trait]) < 0.01:
                del self.temp_modifiers[trait]
    
    def _apply_energy_influence(self, action: RobotAction, energy: float) -> RobotAction:
        """Apply energy level influence to action"""
        if energy > 0.7:
            # High energy - longer, more intense actions
            action.duration *= 1.2
            if action.action_type in ['explore', 'play']:
                action.parameters['intensity'] = 'high'
        elif energy < 0.3:
            # Low energy - shorter, calmer actions
            action.duration *= 0.7
            if action.action_type not in ['rest']:
                # Chance to change to rest
                if random.random() < 0.3:
                    action.action_type = 'rest'
        
        return action
    
    def _apply_playfulness_influence(self, action: RobotAction, playfulness: float) -> RobotAction:
        """Apply playfulness influence to action"""
        if playfulness > 0.7 and action.action_type in ['explore', 'social_approach']:
            # High playfulness - chance to turn into play
            if random.random() < 0.2:
                action.action_type = 'play'
                
        if action.action_type == 'play':
            action.parameters['intensity'] = 'high' if playfulness > 0.6 else 'normal'
            
        return action
    
    def _apply_sociability_influence(self, action: RobotAction, sociability: float) -> RobotAction:
        """Apply sociability influence to action"""
        if sociability > 0.7:
            # High sociability - prefer social actions
            if action.action_type == 'explore':
                if random.random() < 0.15:
                    action.action_type = 'social_approach'
        elif sociability < 0.3:
            # Low sociability - avoid social actions
            if action.action_type == 'social_approach':
                if random.random() < 0.3:
                    action.action_type = 'investigate'
                    
        return action
    
    def _apply_courage_influence(self, action: RobotAction, courage: float) -> RobotAction:
        """Apply courage influence to action"""
        if courage < 0.3:
            # Low courage - more likely to avoid
            if action.action_type in ['investigate', 'explore']:
                if random.random() < 0.2:
                    action.action_type = 'alert'
                    action.parameters['intensity'] = 'high'
        elif courage > 0.7:
            # High courage - more bold exploration
            if action.action_type == 'investigate':
                action.duration *= 1.3
                
        return action
    
    def _apply_curiosity_influence(self, action: RobotAction, curiosity: float) -> RobotAction:
        """Apply curiosity influence to action"""
        if curiosity > 0.7:
            # High curiosity - longer investigation
            if action.action_type in ['investigate', 'explore']:
                action.duration *= 1.2
        elif curiosity < 0.3:
            # Low curiosity - shorter exploration
            if action.action_type == 'explore':
                action.duration *= 0.8
                
        return action
    
    def add_temp_modifier(self, trait: str, modifier: float, decay_time: float = 30.0):
        """Add temporary personality modifier"""
        self.temp_modifiers[trait] = modifier
        # Note: In a full implementation, you'd want to track decay times individually
    
    def get_personality_summary(self) -> str:
        """Get human-readable personality summary"""
        traits = self.get_personality_traits()
        
        summary = "Personality: "
        if traits['energy'] > 0.7:
            summary += "energetic, "
        if traits['playfulness'] > 0.7:
            summary += "playful, "
        if traits['sociability'] > 0.7:
            summary += "social, "
        if traits['curiosity'] > 0.7:
            summary += "curious, "
        if traits['courage'] > 0.7:
            summary += "brave, "
        elif traits['courage'] < 0.3:
            summary += "cautious, "
            
        return summary.rstrip(", ")
