import json

def parse_json_line(s: str):
    s = s.strip()
    if not s:
        return None
    try:
        return json.loads(s)
    except Exception:
        return {"_raw": s}  # fallback : on publie brut
