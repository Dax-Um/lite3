"""Mongo-backed bge-m3 top-1 resolver for the Lite3 voice action allowlist."""
from __future__ import annotations
import argparse
import json
import logging
import os
import signal
import time
from dataclasses import asdict
import numpy as np
from pymongo import MongoClient
from sentence_transformers import SentenceTransformer
from .catalog import ACTIONS
from .policy import plan

LOG = logging.getLogger("lite3_voice")


def append_event(path: str, event: dict) -> None:
    encoded = (json.dumps(event, ensure_ascii=False, separators=(",", ":")) + "\n").encode("utf-8")
    fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_APPEND, 0o640)
    try:
        os.write(fd, encoded)
        os.fsync(fd)
    finally:
        os.close(fd)

class VoiceResolver:
    def __init__(self, uri: str, model_name: str = "BAAI/bge-m3"):
        self.coll = MongoClient(uri, serverSelectionTimeoutMS=3000)["lite3_voice"]["actions"]
        self.model = SentenceTransformer(model_name)

    def bootstrap(self) -> None:
        docs = []
        for action in ACTIONS:
            text = ", ".join(action.phrases)
            vec = self.model.encode([text], normalize_embeddings=True)[0].tolist()
            docs.append({"_id": action.id, **asdict(action), "embedding_text": text, "embedding": vec})
        for doc in docs:
            self.coll.replace_one({"_id": doc["_id"]}, doc, upsert=True)

    def resolve(self, text: str, minimum_score: float = 0.65, minimum_margin: float = 0.0) -> dict:
        docs = list(self.coll.find({"enabled": {"$ne": False}}, {"embedding": 1, "id": 1, "kind": 1}))
        if not docs:
            raise RuntimeError("catalog is empty; run bootstrap first")
        query = self.model.encode([text], normalize_embeddings=True)[0]
        scores = np.asarray([doc["embedding"] for doc in docs]) @ query
        order = np.argsort(-scores)
        first, second = order[0], order[1] if len(order) > 1 else order[0]
        score, margin = float(scores[first]), float(scores[first] - scores[second])
        return {"text": text, "action_id": docs[first]["id"], "score": score, "margin": margin,
                "accepted": score >= minimum_score and margin >= minimum_margin}


def follow_events(resolver: VoiceResolver, event_path: str, action_path: str, *, basic_state: int, minimum_score: float, minimum_margin: float, suppression_seconds: float) -> int:
    """Tail ASR events and emit deterministic, auditable dry-run action plans."""
    stopping = False
    def stop(_signum: int, _frame: object) -> None:
        nonlocal stopping
        stopping = True
    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    suppress_unknown_until = 0.0
    LOG.info("voice resolver listening: events=%s dry_run=true", event_path)
    with open(event_path, "a+", encoding="utf-8") as stream:
        stream.seek(0, os.SEEK_END)
        while not stopping:
            line = stream.readline()
            if not line:
                time.sleep(0.10)
                continue
            try:
                event = json.loads(line)
                if event.get("type") == "utterance_ended":
                    # Energy VAD alone cannot distinguish speech from noise.
                    # Acknowledge only after the transcript passes command
                    # resolution below.
                    continue
                if event.get("type") not in (None, "transcript"):
                    continue
                decision = resolver.resolve(event["text"], minimum_score, minimum_margin)
                request_id = str(event.get("id", ""))
                if decision["accepted"]:
                    suppress_unknown_until = time.monotonic() + suppression_seconds
                    decision["plan"] = [step.__dict__ for step in plan(decision["action_id"], basic_state)]
                    append_event(action_path, {"type": "voice_bark", "request_id": request_id})
                    append_event(action_path, {"type": "voice_action", "request_id": request_id, **decision})
                    append_event(action_path, {"type": "voice_tts", "request_id": request_id,
                                               "text": _accepted_phrase(decision["action_id"])})
                else:
                    if time.monotonic() >= suppress_unknown_until:
                        append_event(action_path, {"type": "voice_tts", "request_id": request_id,
                                                   "text": "I did not understand the command."})
                    else:
                        LOG.info("VOICE_IGNORED during post-command suppression text=%r", event["text"])
                LOG.info("VOICE_DECISION %s", json.dumps(decision, ensure_ascii=False, sort_keys=True))
            except (KeyError, ValueError, json.JSONDecodeError) as exc:
                LOG.warning("discarded malformed/unplannable ASR event: %s", exc)
    return 0


def _accepted_phrase(action_id: str) -> str:
    return {
        "stand_up": "I will stand up.",
        "sit_down": "I will sit down.",
        "move_forward": "I will move forward.",
        "move_backward": "I will move backward.",
        "stop": "I will stop.",
        "turn_left_full": "I will turn left.",
        "turn_right_full": "I will turn right.",
        "moonwalk": "I will do a moonwalk.",
        "hello": "Hello.",
    }.get(action_id, "I did not understand the command.")

def main() -> int:
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    p=argparse.ArgumentParser()
    p.add_argument("command", choices=("bootstrap", "resolve", "follow"))
    p.add_argument("text", nargs="?")
    p.add_argument("--uri", default="mongodb://127.0.0.1:27017")
    p.add_argument("--events", default="/home/ubuntu/iq9_coyote/outputs/voice_control/asr_events.jsonl")
    p.add_argument("--actions", default="/home/ubuntu/iq9_coyote/outputs/voice_control/action_events.jsonl")
    p.add_argument("--basic-state", type=int, default=6, help="Dry-run planning state; executor replaces this with live state.")
    p.add_argument("--minimum-score", type=float, default=0.65)
    p.add_argument("--minimum-margin", type=float, default=0.0, help="Logged for review; default preserves top-1 execution.")
    p.add_argument("--suppression-seconds", type=float, default=4.0, help="Ignore unknown residual speech after an accepted command.")
    a=p.parse_args(); r=VoiceResolver(a.uri)
    if a.command == "bootstrap":
        r.bootstrap(); print("catalog bootstrapped")
    elif a.command == "follow":
        return follow_events(r, a.events, a.actions, basic_state=a.basic_state, minimum_score=a.minimum_score, minimum_margin=a.minimum_margin, suppression_seconds=a.suppression_seconds)
    else:
        if not a.text: p.error("resolve requires text")
        print(r.resolve(a.text))
    return 0
if __name__ == "__main__": raise SystemExit(main())
