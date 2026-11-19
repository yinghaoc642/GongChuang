#!/usr/bin/env python3
"""Query the vendor runtime version only after explicit user authorization.

This sends the current camera's /device_key to the same Sipeed HTTPS endpoint
observed in its official Settings installer. It never installs a runtime,
changes a device file, prints the identifier, or disables TLS verification.
"""

import argparse
import hashlib
import json
from pathlib import Path
import re
import subprocess
import urllib.error
import urllib.parse
import urllib.request


ENDPOINT = "https://maixvision.sipeed.com/api/v1/devices/encryption/version"
DOWNLOAD_ENDPOINT = "https://maixvision.sipeed.com/api/v1/devices/encryption"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--allow-device-id-transmission", action="store_true")
    parser.add_argument("--device-type", choices=("maixcam", "maixcam_pro"))
    parser.add_argument("--download-version")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if not args.allow_device_id_transmission:
        parser.error("Explicit user authorization is required before sending the device identifier")
    if args.download_version:
        if not re.fullmatch(r"\d+\.\d+\.\d+", args.download_version) or not args.output:
            parser.error("A numeric --download-version requires --output")
        if args.output.exists():
            parser.error("Output already exists; nothing will be overwritten")
    metadata_program = (
        "import json,re; "
        "board=dict(line.strip().split('=',1) for line in open('/boot/board') if '=' in line); "
        "v=open('/usr/lib/python3.11/site-packages/maix/version.py').read(); "
        "version='.'.join(re.search('version_'+part+r'\\s*=\\s*(\\d+)',v).group(1) "
        "for part in ('major','minor','patch')); "
        "print(json.dumps(dict(uid=open('/device_key').read().strip(), "
        "os=open('/boot/ver').read().strip(),maixpy=version,device='maixcam')))"
    )
    import shlex
    command = [
        "ssh", "-S", "/private/tmp/gc-maixcam-ssh-control",
        "-o", "UserKnownHostsFile=/private/tmp/gc-maixcam-known-hosts",
        "-o", "StrictHostKeyChecking=yes", "-o", "BatchMode=yes",
        "-o", "ConnectTimeout=8", "root@10.62.9.1",
        "python3 -c " + shlex.quote(metadata_program),
    ]
    result = subprocess.run(command, capture_output=True, text=True, check=True)
    metadata = json.loads(result.stdout)
    device_id = metadata["uid"]
    if args.device_type:
        metadata["device"] = args.device_type
    # This camera was measured to use 16 hexadecimal characters. The installed
    # API stub's example uses a different length, so validate the actual device.
    if not re.fullmatch(r"[0-9a-fA-F]{16}", device_id):
        raise SystemExit("Unexpected device identifier format; no request sent")
    # Verified from the installed Settings executable: the device parameter is
    # the literal 'maixcam'; its httplib request includes this fixed token header.
    # It is the vendor installer's client header, not a user account credential.
    if args.download_version:
        # Confirmed in installed Settings' download request: uid, version, device.
        metadata = {"uid": device_id, "version": args.download_version, "device": "maixcam"}
        endpoint = DOWNLOAD_ENDPOINT
    else:
        endpoint = ENDPOINT
    url = endpoint + "?" + urllib.parse.urlencode(metadata)
    request = urllib.request.Request(url, headers={"token": "MaixVision2024"})
    try:
        with urllib.request.urlopen(request, timeout=20) as response:
            payload = response.read(16 * 1024 * 1024 + 1)
            if args.download_version:
                if len(payload) > 16 * 1024 * 1024:
                    raise SystemExit("Candidate exceeds size limit; no file written")
                if payload[:6] != b"\x7fELF\x02\x01" or int.from_bytes(payload[18:20], "little") != 243:
                    message = payload[:1000].decode(errors="replace")
                    raise SystemExit("Not a RISC-V ELF library: " + message.replace(device_id, "[device id]"))
                with args.output.open("xb") as target:
                    target.write(payload)
                print(json.dumps({"http_status": response.status, "bytes": len(payload),
                                  "sha256": hashlib.sha256(payload).hexdigest(),
                                  "path": str(args.output), "endpoint": endpoint}))
                return
            data = json.loads(payload)
            output = json.dumps(data, ensure_ascii=False)
            output = re.sub(re.escape(device_id), "[device id]", output, flags=re.I)
            print(json.dumps({"http_status": response.status, "response_bytes": len(payload)}))
            print(output[:2200])
    except (urllib.error.URLError, ValueError) as error:
        message = re.sub(re.escape(device_id), "[device id]", str(error), flags=re.I)
        raise SystemExit(type(error).__name__ + ": " + message[:300])


if __name__ == "__main__":
    main()
