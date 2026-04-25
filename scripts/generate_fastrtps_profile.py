#!/usr/bin/env python3
"""Generate a minimal FastDDS unicast profile from deploy-time IPs."""

from __future__ import annotations

import argparse
from pathlib import Path


TEMPLATE = """<?xml version="1.0" encoding="UTF-8"?>
<dds xmlns="http://www.eprosima.com/XMLSchemas/FastRTPS_Profiles">
    <profiles>
        <participant profile_name="participant_profile" is_default_profile="true">
            <rtps>
                <defaultUnicastLocatorList>
{local_locators}
                </defaultUnicastLocatorList>
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

    local_locators = locator(args.local_ip)
    peer_locators = "\n".join(locator(peer) for peer in args.peer_ip)
    output = Path(args.output).expanduser()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(TEMPLATE.format(local_locators=local_locators, peer_locators=peer_locators))
    print(output)


if __name__ == "__main__":
    main()
