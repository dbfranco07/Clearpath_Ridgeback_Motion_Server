#!/usr/bin/env python3
"""Generate a CycloneDDS unicast peer profile from deploy-time IPs.

Mirror of generate_fastrtps_profile.py for the Cyclone RMW. Reads
config/cyclonedds_peer.xml.in next to this script, substitutes the wired
NIC name and one or more peer IPs, and writes the result to --output.

Multiple --peer-ip flags add multiple <Peer> entries; duplicates are
de-duped in input order.
"""

from __future__ import annotations

import argparse
from pathlib import Path


def render_peers(peers: list[str], indent: int = 8) -> str:
    pad = " " * indent
    seen: list[str] = []
    for peer in peers:
        if peer and peer not in seen:
            seen.append(peer)
    return "\n".join(f'{pad}<Peer Address="{peer}"/>' for peer in seen)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--peer-ip", action="append", required=True, help="Peer IP. Repeat for multiple peers.")
    parser.add_argument("--output", required=True, help="Output XML path")
    parser.add_argument(
        "--template",
        default=None,
        help="Override template path (defaults to <repo>/config/cyclonedds_peer.xml.in)",
    )
    args = parser.parse_args()

    script_dir = Path(__file__).resolve().parent
    template_path = Path(args.template) if args.template else script_dir.parent / "config" / "cyclonedds_peer.xml.in"
    template = template_path.read_text()

    peers_block = render_peers(args.peer_ip)
    rendered = template.replace(
        '        <Peer Address="@PEER_IP@"/>',
        peers_block,
    )

    output = Path(args.output).expanduser()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(rendered)
    print(output)


if __name__ == "__main__":
    main()
