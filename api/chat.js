export default async function handler(req) {
  const HF_BACKEND = process.env.HF_BACKEND_URL || 'https://wajahat1313-ragbot-backend.hf.space';
  const path = req.url.replace('/api', '');
  const target = new URL(path, HF_BACKEND).toString();

  const init = {
    method: req.method,
    headers: {
      'content-type': req.headers['content-type'] || 'application/json'
    },
    body: req.method !== 'GET' && req.body ? JSON.stringify(req.body) : undefined
  };

  const res = await fetch(target, init);
  const text = await res.text();

  return new Response(text, {
    status: res.status,
    headers: { 'content-type': res.headers.get('content-type') || 'text/plain' }
  });
}
