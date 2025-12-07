FROM python:3.12-slim

WORKDIR /app

# Install system dependencies required by some numerical libs (OpenBLAS, libgomp)
RUN apt-get update && apt-get install -y --no-install-recommends \
	build-essential \
	libgomp1 \
	libopenblas-dev \
	&& rm -rf /var/lib/apt/lists/*

# Pre-install a CPU PyTorch wheel so sentence-transformers can install
# (uses the official PyTorch CPU wheel index). Pin a recent stable CPU wheel.
RUN pip install --no-cache-dir torch==2.2.2+cpu -f https://download.pytorch.org/whl/cpu/torch_stable.html || true

# Copy requirements and install Python dependencies
COPY ragbot-api/requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# Copy application code
COPY ragbot-api/ .

# Expose port
EXPOSE 8000

# Run application
CMD ["python", "-m", "uvicorn", "main:app", "--host", "0.0.0.0", "--port", "8000"]
