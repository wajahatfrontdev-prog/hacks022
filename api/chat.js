async function handler(req, res) {
  try {
    const HF_BACKEND = process.env.HF_BACKEND_URL || 'https://wajahat1313-ragbot-backend.hf.space';
    // Build target path: forward the path after /api
    const path = (req.url || '').replace(/^\/api/, '') || '/';
    const target = new URL(path, HF_BACKEND).toString();

    const headers = {};
    // Copy relevant headers
    if (req.headers) {
      for (const k of Object.keys(req.headers)) {
        if (k.toLowerCase() === 'host') continue;
        headers[k] = req.headers[k];
      }
    }

    const fetchInit = {
      method: req.method,
      headers,
      body: req.method === 'GET' || req.method === 'HEAD' ? undefined : JSON.stringify(req.body || {})
    };

    const backendRes = await fetch(target, fetchInit);

    // Stream back response with correct content-type
    const contentType = backendRes.headers.get('content-type') || 'application/octet-stream';
    const arr = await backendRes.arrayBuffer();
    const buf = Buffer.from(arr);

    res.status(backendRes.status);
    res.setHeader('content-type', contentType);
    res.send(buf);
  } catch (err) {
    console.error('proxy error', err);
    res.status(500).json({ error: 'proxy_error', detail: String(err) });
  }
}

module.exports = handler;
