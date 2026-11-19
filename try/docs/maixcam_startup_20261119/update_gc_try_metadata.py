#!/usr/bin/env python3
"""Update only our installed app metadata after verifying all payload hashes.

MaixVision's native folder dialog did not accept its confirmed path. Since the
new package has identical scripts/models, preserve those installed files and
update only app.yaml plus this app's existing registry section over SSH.
"""
import hashlib
import json
from pathlib import Path
import shlex
import subprocess
import zipfile

ROOT = Path(__file__).resolve().parents[2]
DEPLOY = ROOT / 'vision/deploy/maixvision_gc_try_331476_v5_6_1_20261119'
RECOVERY = ROOT / 'vision/deploy/maixcam_recovery_20261119_runtime1240'
SSH = [
    'ssh', '-S', '/private/tmp/gc-maixcam-ssh-control',
    '-o', 'UserKnownHostsFile=/private/tmp/gc-maixcam-known-hosts',
    '-o', 'StrictHostKeyChecking=yes', '-o', 'BatchMode=yes',
    '-o', 'ConnectTimeout=8', 'root@10.62.9.1',
]


def main():
    manifest = json.loads(DEPLOY.with_name(DEPLOY.name + '_manifest.json').read_text())
    before = json.loads((RECOVERY / 'before_manifest.json').read_text())
    package = DEPLOY / 'dist/maix-gc_try_r2_331476_20261119-v5.6.1.zip'
    package.parent.mkdir(exist_ok=True)
    with zipfile.ZipFile(package, 'x', compression=zipfile.ZIP_DEFLATED) as z:
        for name, expected in manifest['files'].items():
            data = (DEPLOY / name).read_bytes()
            assert hashlib.sha256(data).hexdigest() == expected['sha256'], name
            z.write(DEPLOY / name, name)
    with zipfile.ZipFile(package) as z:
        assert set(z.namelist()) == set(manifest['files'])
        for name, expected in manifest['files'].items():
            assert hashlib.sha256(z.read(name)).hexdigest() == expected['sha256'], name
    payload = dict(files=manifest['files'], yaml=(DEPLOY / 'app.yaml').read_text(),
                   old_registry_hash=before['maixapp/apps/app.info']['sha256'],
                   old_yaml_hash=before['maixapp/apps/gc_try_r2_331476_20261119/app.yaml']['sha256'])
    program = '''import configparser,hashlib,json,os,re,stat
from pathlib import Path
p=PAYLOAD
app_id='gc_try_r2_331476_20261119'
root=Path('/maixapp/apps')/app_id
info=Path('/maixapp/apps/app.info')
app_yaml=root/'app.yaml'
h=lambda b:hashlib.sha256(b).hexdigest()
for name,item in p['files'].items():
    if name!='app.yaml':assert h((root/name).read_bytes())==item['sha256'],name
old=info.read_bytes()
old_yaml=app_yaml.read_bytes()
assert h(old)==p['old_registry_hash']
assert h(old_yaml)==p['old_yaml_hash']
pattern=r'(?ms)^\\['+re.escape(app_id)+r'\\]\\n.*?(?=^\\[|\\Z)'
match=re.search(pattern,old.decode())
assert match
section=match.group()
updated=section
for key,value in [('name','GC Try'),('name[zh]','GC Try'),('version','5.6.1')]:
    updated,n=re.subn(r'(?m)^'+re.escape(key)+r'=.*$',key+'='+value,updated)
    assert n==1,key
new=(old.decode()[:match.start()]+updated+old.decode()[match.end():]).encode()
parser=configparser.ConfigParser(interpolation=None);parser.read_string(new.decode())
assert parser[app_id]['name']=='GC Try' and parser[app_id]['version']=='5.6.1'
recovery=Path('/maixapp/gc_recovery_20261119_runtime1240');recovery.mkdir(mode=0o700)
(recovery/'app.info.before').write_bytes(old)
(recovery/'app.yaml.before').write_bytes(old_yaml)
def replace(path,data):
    s=path.stat();stage=path.with_name('.'+path.name+'.gc-stage-20261119')
    with stage.open('xb') as f:f.write(data);f.flush();os.fsync(f.fileno())
    os.chmod(stage,stat.S_IMODE(s.st_mode));os.chown(stage,s.st_uid,s.st_gid)
    os.replace(stage,path)
try:
    replace(app_yaml,p['yaml'].encode())
    replace(info,new)
except BaseException:
    replace(app_yaml,old_yaml);replace(info,old);raise
os.sync()
assert h(app_yaml.read_bytes())==p['files']['app.yaml']['sha256']
assert info.read_bytes()==new
print(json.dumps(dict(result='METADATA UPDATE OK',app_id=app_id,name='GC Try',version='5.6.1',registry_sha256=h(new),unrelated_registry_sections='byte-identical',payload='scripts and models unchanged')))
'''.replace('PAYLOAD', repr(payload))
    result = subprocess.check_output(SSH + ['python3 -c ' + shlex.quote(program)], timeout=60)
    status = json.loads(result)
    (RECOVERY / 'metadata_update.json').write_text(json.dumps(status, indent=2) + '\n')
    manifest.update(package=dict(path=str(package.relative_to(ROOT)), bytes=package.stat().st_size,
                                 sha256=hashlib.sha256(package.read_bytes()).hexdigest()),
                    device_installation=status,
                    installation_method='Existing identical scripts/models verified; app metadata and only its own registry section atomically updated over SSH after MaixVision native dialog failed.')
    DEPLOY.with_name(DEPLOY.name + '_manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    print(json.dumps(manifest['package'], indent=2))
    print(result.decode().strip())


if __name__ == '__main__':
    main()
