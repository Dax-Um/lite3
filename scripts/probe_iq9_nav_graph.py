#!/usr/bin/env python3
"""Evaluate a mocked IQ9 Nav2 graph snapshot from JSON."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from lite3_iq9.nav_graph_probe import NavGraphProbe, NavGraphSnapshot


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("snapshot_json", type=Path)
    args = parser.parse_args()

    data = json.loads(args.snapshot_json.read_text(encoding="utf-8"))
    snapshot = NavGraphSnapshot(
        nodes=set(data.get("nodes", [])),
        topics=set(data.get("topics", [])),
        actions=set(data.get("actions", [])),
        cmd_vel_publishers=set(data.get("cmd_vel_publishers", [])),
        cmd_vel_subscribers=set(data.get("cmd_vel_subscribers", [])),
    )
    report = NavGraphProbe().evaluate(snapshot)
    print(
        json.dumps(
            {
                "ok": report.ok,
                "missing_nodes": report.missing_nodes,
                "missing_topics": report.missing_topics,
                "missing_actions": report.missing_actions,
                "cmd_vel_ready": report.cmd_vel_ready,
                "reasons": report.reasons,
            },
            indent=2,
            sort_keys=True,
        )
    )
    return 0 if report.ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
