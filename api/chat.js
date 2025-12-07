async function handler(req, res) {
  try {
    // Allow CORS
    res.setHeader('Access-Control-Allow-Origin', '*');
    res.setHeader('Access-Control-Allow-Methods', 'GET, POST, PUT, DELETE, OPTIONS');
    res.setHeader('Access-Control-Allow-Headers', 'Content-Type');

    if (req.method === 'OPTIONS') {
      return res.status(200).end();
    }

    const HF_BACKEND = process.env.HF_BACKEND_URL || 'https://wajahat1313-ragbot-backend.hf.space';
    // Build target path: forward the path after /api
    const path = (req.url || '').replace(/^\/api/, '') || '/';
    const target = new URL(path, HF_BACKEND).toString();

    const headers = {};
    // Copy relevant headers
    if (req.headers) {
      for (const k of Object.keys(req.headers)) {
        const lower = k.toLowerCase();
        if (lower === 'host' || lower === 'connection') continue;
        headers[k] = req.headers[k];
      }
    }

    // For POST/PUT, read the body and serialize
    let body;
    if (req.method !== 'GET' && req.method !== 'HEAD') {
      if (typeof req.body === 'string') {
        body = req.body;
      } else if (req.body) {
        body = JSON.stringify(req.body);
      } else {
        body = '';
      }
    }

    const fetchInit = {
      method: req.method,
      headers,
      body
    };

    const backendRes = await fetch(target, fetchInit);

    // Get response body as Buffer or text
    const contentType = backendRes.headers.get('content-type') || 'application/octet-stream';
    const arr = await backendRes.arrayBuffer();
    const buf = Buffer.from(arr);

    res.status(backendRes.status);
    res.setHeader('content-type', contentType);
    res.send(buf);
  } catch (err) {
    console.error('proxy error:', err);
    res.status(500).json({ error: 'proxy_error', detail: String(err) });
  }
}

module.exports = handler;
