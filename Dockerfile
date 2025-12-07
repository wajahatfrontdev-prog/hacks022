FROM python:3.12-slim

WORKDIR /app
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

COPY ragbot-api /app/ragbot-api

EXPOSE 8000
ENV PORT=8000

CMD ["uvicorn", "ragbot-api.main:app", "--host", "0.0.0.0", "--port", "8000"]
