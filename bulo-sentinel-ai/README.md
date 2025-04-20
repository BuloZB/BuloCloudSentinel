# Bulo.Cloud Sentinel AI Integrations Microservice

This microservice provides integration with various AI tools for the Bulo.Cloud Sentinel platform. It supports modular adapters for different AI providers, enabling seamless and secure communication with external AI APIs.

## Features

- Modular plugin-like architecture for easy addition/removal of AI tools
- Unified REST API to trigger AI tasks and receive results
- Audit logging of AI tool usage and response times
- Support for ChatGPT, Claude, Gemini, DALL·E, Whisper adapters (expandable)
- Placeholder endpoints for vision and audio processing
- Containerized with Docker for easy deployment

## Setup

1. Clone the repository and navigate to the `bulo-sentinel-ai` directory.

2. Create a `.env` file with the following environment variables:

```
CHATGPT_API_KEY=your_openai_api_key
CLAUDE_API_KEY=your_anthropic_api_key
GEMINI_API_KEY=your_google_gemini_api_key
DALLE_API_KEY=your_openai_api_key_for_dalle
WHISPER_API_KEY=your_openai_api_key_for_whisper
AI_AUDIT_LOG_FILE=path_to_audit_log.json
```

3. Build and run the Docker container:

```bash
docker build -t bulo-sentinel-ai .
docker run -p 8002:8002 --env-file .env bulo-sentinel-ai
```

## API Endpoints

- `POST /ai/chat` - Proxy to LLMs with role/user context
- `POST /ai/vision/analyze` - Send image to multimodal AI (placeholder)
- `POST /ai/audio/transcribe` - Convert uploaded audio into text (placeholder)
- `GET /ai/status` - Show tool availability and health

## Adding New AI Tools

To add a new AI tool:

1. Create an adapter class implementing the common interface (`chat`, `analyze_image`, `transcribe_audio`, `status`).

2. Add the adapter to the `adapters` dictionary in `main.py`.

3. Implement any necessary API endpoints or frontend integration.

## Testing

Unit and integration tests should be added to ensure reliability.

## License

This project is open-source and licensed under the MIT License.
