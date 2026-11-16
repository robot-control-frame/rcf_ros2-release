#!/usr/bin/env python3
"""Drive `bloom-release` non-interactively.

bloom-release has no non-interactive mode: it calls safe_input(), which does a
termios tcflush() on stdin and therefore crashes with

    termios.error: (25, 'Inappropriate ioctl for device')

when stdin is a pipe (CI).  Running it under a pty via pexpect works, so this
script spawns bloom-release inside a pty and answers the prompts from the
environment.

Environment (all optional except the URLs):
    RELEASE_VERSION     version to release, e.g. 0.7.0      (required)
    RELEASE_REPO_URL    https://github.com/OWNER/REPO-release.git
    UPSTREAM_URL        https://github.com/OWNER/REPO
    REPO_NAME           rosdistro repository key           (default: rcf_ros2)
    DEVEL_BRANCH        default: main
    ROS_DISTRO          default: jazzy
    RELEASE_INC         default: 1  (only used on new tracks)
    BLOOM_NEW_TRACK     "0" to skip --new-track (same version, packaging only)

Usage:
    python3 scripts/bloom_release.py --version 0.7.0 [--no-new-track]
"""

from __future__ import annotations

import argparse
import os
import re
import sys

import pexpect

ANSI = re.compile(r"\x1b\[[0-9;]*m")
# A prompt looks like   "Version ['0.6.0']:"  or  "Continue [Y/n]?"
VALUE_PROMPT = re.compile(r"\[[^\]\n]*\]:\s*$")
YESNO_PROMPT = re.compile(r"\[[^\]\n]*\]\?\s*$")
# The line above a value prompt names the field being asked for.
HEADER = re.compile(r"^([A-Z][A-Za-z ()_/-]+):\s*$")


def build_answers(version: str) -> list[tuple[str, str]]:
    """(field name, answer), matched against the header line before the prompt."""
    return [
        ("Release Repository Push URL", os.environ["RELEASE_REPO_URL"]),
        ("Release Repository URI", os.environ["RELEASE_REPO_URL"]),
        ("Release Repository URL", os.environ["RELEASE_REPO_URL"]),
        ("Repository Name", os.environ.get("REPO_NAME", "rcf_ros2")),
        ("Upstream Repository URI", os.environ["UPSTREAM_URL"]),
        ("Upstream VCS Type", "git"),
        ("Version", version),
        # must stay verbatim: bloom expands {version} itself
        ("Release Tag", "v:{version}"),
        ("Upstream Devel Branch", os.environ.get("DEVEL_BRANCH", "main")),
        ("ROS Distro", os.environ.get("ROS_DISTRO", "jazzy")),
        ("Patches Directory", ""),
        ("Release Increment", os.environ.get("RELEASE_INC", "1")),
    ]


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--version", required=True)
    ap.add_argument("--no-new-track", action="store_true")
    ap.add_argument("--attempts", type=int, default=2, help="retries on failure")
    args = ap.parse_args()

    version = args.version.lstrip("v")
    os.environ["RELEASE_VERSION"] = version
    name = os.environ.get("REPO_NAME", "rcf_ros2")
    distro = os.environ.get("ROS_DISTRO", "jazzy")

    cmd = [
        "bloom-release",
        "--rosdistro",
        distro,
        "--track",
        distro,
        name,
    ]
    if not args.no_new_track:
        cmd.append("--new-track")
    cmd += ["--no-pull-request", "--no-web"]

    answers = build_answers(version)

    last = 1
    for attempt in range(1, args.attempts + 1):
        print(f"=== bloom-release attempt {attempt}/{args.attempts}: {' '.join(cmd)}", flush=True)
        last = run_once(cmd, answers)
        if last == 0:
            return 0
        print(f"=== attempt {attempt} failed with exit code {last}", flush=True)
    return last


def run_once(cmd: list[str], answers: list[tuple[str, str]]) -> int:
    child = pexpect.spawn(
        cmd[0],
        cmd[1:],
        timeout=300,
        encoding="utf-8",
        codec_errors="replace",
        env=os.environ,
    )
    header = ""
    buf = ""
    log: list[tuple[str, str]] = []

    while True:
        try:
            char = child.read_nonblocking(size=1, timeout=300)
        except pexpect.TIMEOUT:
            print("\n[bloom_release] timed out waiting for output", flush=True)
            child.close(force=True)
            return 124
        except pexpect.EOF:
            break

        sys.stdout.write(char)
        sys.stdout.flush()
        buf += char

        while "\n" in buf:
            line, buf = buf.split("\n", 1)
            match = HEADER.match(ANSI.sub("", line).strip())
            if match:
                header = match.group(1)

        tail = ANSI.sub("", buf).strip()
        if not (VALUE_PROMPT.search(tail) or YESNO_PROMPT.search(tail)):
            continue

        if YESNO_PROMPT.search(tail):
            answer = "y"
        else:
            answer = ""
            if header:
                # Header matching must win: "Release Tag [':{version}']:" also
                # contains "version", and answering it with the bare version
                # makes bloom look for an upstream tag named 0.7.0 instead of
                # v0.7.0.
                for field, value in answers:
                    if field in header:
                        answer = value
                        break
            elif "release repository url" in tail.lower():
                answer = os.environ["RELEASE_REPO_URL"]

        child.send(answer + "\n")
        log.append((header, tail[-60:], answer))
        buf = ""

    child.close()
    status = child.exitstatus if child.exitstatus is not None else 1
    print("\n=== prompts / answers ===", flush=True)
    for field, prompt, answer in log:
        print(f"  {field:<32} {prompt:<62} -> {answer!r}")
    print(f"=== bloom-release exit status: {status}", flush=True)
    return status


if __name__ == "__main__":
    sys.exit(main())
