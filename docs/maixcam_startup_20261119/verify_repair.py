#!/usr/bin/env python3
"""Verify protected camera files against the pre-repair backup; export logs."""
import hashlib
import json
from pathlib import Path
import re
import shlex
import subprocess
import tarfile

ROOT = Path(__file__).resolve().parents[2]
DOCS = Path(__file__).parent
RECOVERY = ROOT / 'vision/deploy/maixcam_recovery_20261119_runtime1240'
SSH = [
    'ssh', '-S', '/private/tmp/gc-maixcam-ssh-control',
    '-o', 'UserKnownHostsFile=/private/tmp/gc-maixcam-known-hosts',
    '-o', 'StrictHostKeyChecking=yes', '-o', 'BatchMode=yes',
    '-o', 'ConnectTimeout=8', 'root@10.62.9.1',
]


def remote(command):
    return subprocess.check_output(SSH + [command], timeout=90)


def main():
    before = json.loads((RECOVERY / 'before_manifest.json').read_text())
    paths = [p for p in before if not any(p.startswith(other + '/') for other in before)]
    program = '''import hashlib,json,os,stat
result={}
def record(p):
    s=os.lstat('/'+p)
    entry={'mode':stat.S_IMODE(s.st_mode),'uid':s.st_uid,'gid':s.st_gid}
    if stat.S_ISLNK(s.st_mode):entry.update(type='symlink',target=os.readlink('/'+p))
    elif stat.S_ISREG(s.st_mode):
        h=hashlib.sha256()
        with open('/'+p,'rb') as f:
            for b in iter(lambda:f.read(1048576),b''):h.update(b)
        entry.update(type='file',bytes=s.st_size,sha256=h.hexdigest())
    elif stat.S_ISDIR(s.st_mode):
        entry['type']='directory'
        for name in sorted(os.listdir('/'+p)):record(p+'/'+name)
    else:raise RuntimeError(p)
    result[p]=entry
for p in PATHS:record(p)
print(json.dumps(result,sort_keys=True))
'''.replace('PATHS', repr(paths))
    after = json.loads(remote('python3 -c ' + shlex.quote(program)))
    expected_changed = {
        'usr/lib/libmaixcam_lib.so.1.24.0': '33a6e0ca2f9fe59bd8b7a6f7bd28f03730fd8db58c8bf870fd2ad963615f83da',
        'maixapp/apps/gc_try_r2_331476_20261119/app.yaml': 'a92c57aceffcd843d585237b5efc7bd15a007996dcb6ce5a916c08a6e3b27e63',
        'maixapp/apps/app.info': 'c1bf66864b84f9672bb7ac99a95ca716d2c33f9f71ff24f06523f7a3ebfb4c3c',
    }
    differences = sorted(p for p in before.keys() | after.keys() if before.get(p) != after.get(p))
    assert set(differences) == set(expected_changed), differences
    for p, digest in expected_changed.items():
        assert after[p]['sha256'] == digest, p
        for key in ('mode', 'uid', 'gid'):
            assert before[p][key] == after[p][key], (p, key)
    current_info = remote('cat /maixapp/apps/app.info')
    with tarfile.open(RECOVERY / 'camera_before_runtime_refresh.tar') as tar:
        old_info = tar.extractfile('maixapp/apps/app.info').read()
    pattern = rb'(?ms)^\[gc_try_r2_331476_20261119\]\n.*?(?=^\[|\Z)'
    assert re.sub(pattern, b'', current_info) == re.sub(pattern, b'', old_info)
    saved_logs = {}
    for name in ['gc_normal_official_model.log', 'gc_normal_try_inference.log', 'gc_gc_try_full_startup.log']:
        data = remote('cat ' + shlex.quote('/tmp/' + name)).decode(errors='replace')
        data = re.sub(r'(?<![0-9A-Za-z])[0-9a-fA-F]{16,32}(?![0-9A-Za-z])',
                      '[redacted identifier]', data)
        with (DOCS / 'logs' / name).open('x') as f:
            f.write(data)
        saved_logs[name] = data
    assert 'NORMAL OFFICIAL MODEL OK 80' in saved_logs['gc_normal_official_model.log']
    assert 'NORMAL TRY 3 FRAMES OK' in saved_logs['gc_normal_try_inference.log']
    full = saved_logs['gc_gc_try_full_startup.log']
    assert 'FULL GC TRY STARTUP OK NORMAL EXIT' in full
    counts = json.loads(re.search(r'^FULL STARTUP COUNTS (.*)$', full, re.M).group(1))
    assert counts['camera_frames'] >= 30 and counts['display_frames'] >= 15
    assert counts['vision_uart_write_calls'] == 0 and counts['modes'] == [0]
    (RECOVERY / 'after_manifest.json').write_text(json.dumps(after, indent=2) + '\n')
    result = dict(result='PASS', unchanged_entries=len(before) - len(differences),
                  intentional_changes=differences,
                  protected='xg13, /root/models, autostart, system config, OS/MaixPy version files, our scripts/models and unrelated application registry sections all unchanged',
                  official_model_exit=0, try_three_frame_inference_exit=0,
                  full_app_exit=0, camera_frames=counts['camera_frames'], display_frames=counts['display_frames'],
                  vision_uart_write_calls=counts['vision_uart_write_calls'], request_modes=counts['modes'],
                  limitations='Single camera; short startup verification only. No recognition accuracy, real coarse alignment or STM32 integration validated.')
    (RECOVERY / 'repair_verification.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
