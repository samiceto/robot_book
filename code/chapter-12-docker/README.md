# Chapter 12: Containerization & CI/CD

## Quick Start

```bash
# Build Docker image
docker build -t humanoid_perception:latest .

# Run container
docker run -it --rm --network host humanoid_perception:latest

# Deploy with Docker Compose
docker-compose up -d

# View logs
docker-compose logs -f perception
```

## Files

- `Dockerfile` - ROS 2 perception container
- `docker-compose.yml` - Multi-service orchestration
- `.github/workflows/ci.yml` - GitHub Actions CI/CD

## CI/CD Workflow

1. Push code to GitHub
2. GitHub Actions builds and tests
3. Docker image pushed to registry
4. Jetson auto-pulls and restarts

## Deployment

```bash
# On Jetson (one-command deploy)
docker-compose pull && docker-compose up -d
```
