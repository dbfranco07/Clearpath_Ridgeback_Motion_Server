#!/usr/bin/env python3
"""Generate a minimal FastDDS unicast profile from deploy-time IPs."""

from __future__ import annotations

import argparse
from pathlib import Path


TEMPLATE = """<?xml version="1.0" encoding="UTF-8"?>
<!--
  defaultUnicastLocatorList is intentionally omitted so FastDDS binds user and
  metatraffic locators to *all* interfaces, including 127.0.0.1. Restricting
  the bind list to a single WiFi IP (as a previous revision did) breaks
  intra-machine service discovery: sibling Nav2 processes have to hairpin
  every SPDP packet through the WiFi NIC, and `lifecycle_manager` ends up
  stuck on "Waiting for service controller_server/get_state...".

  initialPeersList still suppresses default multicast and limits cross-host
  discovery to the listed peers, which is the whole point of the profile.
-->
<dds xmlns="http://www.eprosima.com/XMLSchemas/FastRTPS_Profiles">
    <profiles>
        <participant profile_name="participant_profile" is_default_profile="true">
            <rtps>
                <builtin>
                    <initialPeersList>
{peer_locators}
                    </initialPeersList>
                </builtin>
            </rtps>
        </participant>
    </profiles>
</dds>
"""


def locator(address: str, indent: int = 24) -> str:
    pad = " " * indent
    return (
        f"{pad}<locator>\n"
        f"{pad}    <udpv4>\n"
        f"{pad}        <address>{address}</address>\n"
        f"{pad}    </udpv4>\n"
        f"{pad}</locator>"
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--local-ip", required=True, help="IP address of the machine using the generated profile")
    parser.add_argument("--peer-ip", action="append", required=True, help="Peer IP. Repeat for multiple peers.")
    parser.add_argument("--output", required=True, help="Output XML path")
    args = parser.parse_args()

    initial_peers = [args.local_ip, "127.0.0.1", *args.peer_ip]
    peer_locators = "\n".join(locator(peer) for peer in dict.fromkeys(initial_peers))
    output = Path(args.output).expanduser()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(TEMPLATE.format(peer_locators=peer_locators))
    print(output)


if __name__ == "__main__":
    main()
