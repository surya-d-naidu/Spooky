# AI Vision Dog System

This system enables your robotic dog to see through a webcam, analyze what it sees using AI (Ollama), and react with appropriate emotions and behaviors like a real dog.

## Features

- **Real-time Vision**: Captures video from webcam and processes it continuously
- **AI Analysis**: Uses Ollama with LLaVA model to understand what the robot sees
- **Dog Personality**: Realistic emotional responses and behaviors
- **Smooth Integration**: Works with existing animation system
- **Personality Traits**: Configurable playfulness, friendliness, energy levels

## Setup

1. **Install Dependencies**:
   ```bash
   ./setup.sh
   ```

2. **Setup Ollama on your laptop**:
   ```bash
   curl -fsSL https://ollama.ai/install.sh | sh
   ollama serve
   ollama pull llava
   ```

3. **Configure Connection**:
   - If Ollama is on a different machine, set: `export OLLAMA_HOST=your_laptop_ip`
   - For different camera: `export CAMERA_INDEX=1`

## Usage

**Basic Usage**:
```bash
python3 run_ai_dog.py
```

**With custom Ollama host**:
```bash
OLLAMA_HOST=192.168.1.100 python3 run_ai_dog.py
```

## How It Works

1. **Camera Stream**: Continuously captures frames from webcam
2. **AI Analysis**: Every 3 seconds, sends frame to Ollama for analysis
3. **Personality Processing**: Filters AI responses through dog personality traits
4. **Action Execution**: Performs appropriate animations based on emotions
5. **Behavior Loop**: Maintains realistic dog-like behavior patterns

## Available Emotions & Actions

**Emotions**: happy, excited, curious, calm, alert, playful, tired, scared, aggressive, confused

**Actions**: walk, sit, stand, turn_left, turn_right, walk_backward, say_hi, end_walk

## Personality Traits

- **Playfulness**: How likely to engage in playful behaviors
- **Friendliness**: Response to people and positive stimuli  
- **Energy Level**: Activity and movement preferences
- **Curiosity**: Exploration and investigation behaviors
- **Anxiety Level**: Reaction to unknown or threatening situations

## System Architecture

```
Camera → AI Analysis → Personality Filter → Animation Controller → Robot
```

The system maintains emotional continuity, prevents action spam, and provides natural idle behaviors when nothing interesting is happening.

## Troubleshooting

- **Camera not found**: Try different CAMERA_INDEX values (0, 1, 2...)
- **Ollama connection failed**: Check if Ollama server is running and accessible
- **No emotions/actions**: Verify llava model is installed in Ollama
- **Jerky movements**: Adjust analysis_interval in ai_controller.py
