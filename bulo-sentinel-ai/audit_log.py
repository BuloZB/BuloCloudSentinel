import json
import os
from datetime import datetime, timezone
from typing import List, Dict, Any

AUDIT_LOG_FILE = os.getenv("AI_AUDIT_LOG_FILE", "ai_audit_log.json")

def log_audit_entry(entry: Dict[str, Any]) -> None:
    entry['timestamp'] = datetime.now(timezone.utc).isoformat()
    if os.path.exists(AUDIT_LOG_FILE):
        with open(AUDIT_LOG_FILE, "r+", encoding="utf-8") as f:
            try:
                data = json.load(f)
            except json.JSONDecodeError:
                data = []
            data.append(entry)
            f.seek(0)
            json.dump(data, f, indent=2)
    else:
        with open(AUDIT_LOG_FILE, "w", encoding="utf-8") as f:
            json.dump([entry], f, indent=2)

def read_audit_log() -> List[Dict[str, Any]]:
    if not os.path.exists(AUDIT_LOG_FILE):
        return []
    with open(AUDIT_LOG_FILE, "r", encoding="utf-8") as f:
        try:
            return json.load(f)
        except json.JSONDecodeError:
            return []
