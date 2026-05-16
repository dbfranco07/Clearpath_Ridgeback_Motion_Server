#!/usr/bin/env python3
"""Generate a minimal FastDDS unicast profile from deploy-time IPs."""

from __future__ import annotations

import argparse
from pathlib import Path


TEMPLATE = """<?xml version="1.0" encoding="UTF-8"?>
<!--
  Why this profile:
    1. defaultUnicastLocatorList is intentionally omitted so FastDDS binds
       to all interfaces (including 127.0.0.1).
    2. SHM transport is declared explicitly. When FASTRTPS_DEFAULT_PROFILES_FILE
       is set, the env var RMW_FASTRTPS_USE_SHM=1 is ignored — the profile
       owns the transport stack. Without SHM declared here, every local
       participant talks UDP-only, the well-known port pool gets crowded
       with 15+ Jetson processes, and Nav2 lifecycle_manager hangs forever
       on "Waiting for service controller_server/get_state...".
    3. UDPv4 stays for cross-host comms with the Ridgeback. initialPeersList
       suppresses multicast and limits unicast SPDP to the listed peers.
-->
<dds xmlns="http://www.eprosima.com/XMLSchemas/FastRTPS_Profiles">
    <profiles>
        <transport_descriptors>
            <transport_descriptor>
                <transport_id>shm_transport</transport_id>
                <type>SHM</type>
            </transport_descriptor>
            <transport_descriptor>
                <transport_id>udp_transport</transport_id>
                <type>UDPv4</type>
            </transport_descriptor>
        </transport_descriptors>
        <participant profile_name="participant_profile" is_default_profile="true">
            <rtps>
                <userTransports>
                    <transport_id>shm_transport</transport_id>
                    <transport_id>udp_transport</transport_id>
                </userTransports>
                <useBuiltinTransports>false</useBuiltinTransports>
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
