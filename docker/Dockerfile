# NexusOS Production Multi-Stage Dockerfile
# Builds: nexusos-cloud (FastAPI server) and nexusos-edge (robot edge agent)

# ─────────────────────────────────────────────────────────────────────────────
# Stage 1: Build Base
# ─────────────────────────────────────────────────────────────────────────────

FROM python:3.11-slim AS base

LABEL maintainer="NexusOS Maintainers <hello@nexusos.io>"
LABEL org.opencontainers.image.title="NexusOS"
LABEL org.opencontainers.image.description="Universal Robotics OS & Fleet Control Middleware"
LABEL org.opencontainers.image.version="1.0.0"
LABEL org.opencontainers.image.licenses="NCL-1.0"

ENV PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1 \
    PYTHONFAULTHANDLER=1 \
    PIP_NO_CACHE_DIR=1 \
    PIP_DISABLE_PIP_VERSION_CHECK=1

# System dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    gcc \
    libffi-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /nexusos

# Install Python dependencies first (layer caching)
COPY requirements.txt ./
RUN pip install --upgrade pip && pip install -r requirements.txt

# Copy full source
COPY . .
RUN pip install -e . --no-deps

# ─────────────────────────────────────────────────────────────────────────────
# Stage 2: Cloud Server Image
# ─────────────────────────────────────────────────────────────────────────────

FROM base AS cloud

EXPOSE 8000

HEALTHCHECK --interval=15s --timeout=5s --start-period=10s --retries=3 \
    CMD python -c "import urllib.request; urllib.request.urlopen('http://localhost:8000/api/v1/health')"

CMD ["python", "-m", "uvicorn", "nexusos.server.main:app", \
     "--host", "0.0.0.0", "--port", "8000", \
     "--log-level", "info", "--workers", "1"]

# ─────────────────────────────────────────────────────────────────────────────
# Stage 3: Edge Agent Image
# ─────────────────────────────────────────────────────────────────────────────

FROM base AS edge

ENV NEXUSOS_ROBOT_ID="sim_robot_01" \
    NEXUSOS_CLOUD_URL="ws://nexusos-cloud:8000/ws/edge" \
    NEXUSOS_LICENSE_TOKEN="" \
    NEXUSOS_DRIVER_TYPE="DIFFERENTIAL_DRIVE"

CMD ["python", "-m", "nexusos.agent.run_edge"]

# ─────────────────────────────────────────────────────────────────────────────
# Stage 4: License Server Mock Image (for local development/testing)
# ─────────────────────────────────────────────────────────────────────────────

FROM base AS license-server

EXPOSE 9000

CMD ["python", "-m", "nexusos.licensing.server"]
