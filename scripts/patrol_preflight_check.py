#!/usr/bin/env python3
"""Interactive real-robot preflight checklist."""

CHECKS = [
    "App manual mode 전환 가능",
    "E-stop 가능",
    "배터리 충분",
    "테스트 공간 앞뒤 3m 확보",
    "motion host ping 성공",
    "테스트 속도 0.1 m/s 이하",
    "stop command 반복 송신 준비",
]


def main() -> None:
    for item in CHECKS:
        answer = input(f"[check] {item} (yes/no): ").strip().lower()
        if answer != "yes":
            raise SystemExit(f"preflight failed: {item}")
    print("preflight ok")


if __name__ == "__main__":
    main()
