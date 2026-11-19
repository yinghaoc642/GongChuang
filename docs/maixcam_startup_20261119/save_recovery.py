#!/usr/bin/env python3
"""Read-only camera backup, with each archived file checked against the source."""
import hashlib
import json
from pathlib import Path
import re
import shlex
import subprocess
import tarfile

SSH = [
    "ssh", "-S", "/private/tmp/gc-maixcam-ssh-control",
    "-o", "UserKnownHostsFile=/private/tmp/gc-maixcam-known-hosts",
    "-o", "StrictHostKeyChecking=yes", "-o", "BatchMode=yes",
    "-o", "ConnectTimeout=8", "root@10.62.9.1",
]
ROOT = Path(__file__).resolve().parents[2]
DEST = ROOT / "vision/deploy/maixcam_recovery_20261119_runtime1240"
PATHS = [
    "usr/lib/libmaixcam_lib.so.1.24.0", "lib/libmaixcam_lib.so",
    "maixapp/maixcam_lib.version", "maixapp/auto_start.txt",
    "maixapp/sys_conf.ini", "maixapp/apps/app.info",
    "maixapp/apps/xg13", "maixapp/apps/gc_try_r2_331476_20261119",
    "root/models", "boot/ver", "boot/board",
    "usr/lib/python3.11/site-packages/maix/version.py",
]


def remote(command):
    return subprocess.check_output(SSH + [command], timeout=240)


def main():
    DEST.mkdir(mode=0o700, parents=True, exist_ok=False)
    program = '''import hashlib,json,os,stat
paths=PATHS
result={}
def record(p):
    s=os.lstat('/'+p)
    entry={'mode':stat.S_IMODE(s.st_mode),'uid':s.st_uid,'gid':s.st_gid}
    if stat.S_ISLNK(s.st_mode): entry.update(type='symlink',target=os.readlink('/'+p))
    elif stat.S_ISREG(s.st_mode):
        h=hashlib.sha256()
        with open('/'+p,'rb') as f:
            for b in iter(lambda:f.read(1048576),b''):h.update(b)
        entry.update(type='file',bytes=s.st_size,sha256=h.hexdigest())
    elif stat.S_ISDIR(s.st_mode):
        entry['type']='directory'
        for name in sorted(os.listdir('/'+p)):record(p+'/'+name)
    else: raise RuntimeError('Unexpected file type: '+p)
    result[p]=entry
for p in paths:record(p)
print(json.dumps(result,sort_keys=True))
'''.replace('PATHS', repr(PATHS))
    manifest = json.loads(remote("python3 -c " + shlex.quote(program)))
    (DEST / "before_manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    archive = DEST / "camera_before_runtime_refresh.tar"
    with archive.open("xb") as f:
        subprocess.run(SSH + ["tar -cf - -C / " + " ".join(map(shlex.quote, PATHS))],
                       stdout=f, check=True, timeout=240)
    observed = set()
    with tarfile.open(archive) as tar:
        for member in tar:
            p = member.name.rstrip('/')
            expected = manifest[p]
            observed.add(p)
            assert (member.mode, member.uid, member.gid) == (
                expected['mode'], expected['uid'], expected['gid']), p
            if member.isfile():
                assert hashlib.sha256(tar.extractfile(member).read()).hexdigest() == expected['sha256'], p
            elif member.issym():
                assert member.linkname == expected['target'], p
            else:
                assert member.isdir() and expected['type'] == 'directory', p
    assert observed == set(manifest)
    after = json.loads(remote("python3 -c " + shlex.quote(program)))
    assert after == manifest, 'Camera source changed during backup'
    digest = hashlib.sha256(archive.read_bytes()).hexdigest()
    result = dict(archive=archive.name, bytes=archive.stat().st_size,
                  sha256=digest, entries=len(manifest),
                  verification='Every archived entry matched camera metadata; every regular file SHA-256 matched; camera source manifest unchanged after transfer.')
    (DEST / "verification.json").write_text(json.dumps(result, indent=2) + "\n")
    logs = Path(__file__).parent / "logs"
    for name in ['gc_device_runtime_official_model.log', 'gc_candidate_try_inference.log']:
        data = remote('cat ' + shlex.quote('/tmp/' + name)).decode(errors='replace')
        data = re.sub(r'(?<![0-9A-Za-z])[0-9a-fA-F]{16,32}(?![0-9A-Za-z])',
                      '[redacted identifier]', data)
        with (logs / name).open('x') as f:
            f.write(data)
    print(json.dumps(dict(directory=str(DEST), **result), indent=2))


if __name__ == '__main__':
    main()
