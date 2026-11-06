"""Freeze compact reports and exact hashes; raw traces remain in the original run directory."""
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
source=Path(sys.argv[1]).resolve();dest=Path(sys.argv[2]);dest.mkdir(parents=True,exist_ok=False)
run=next(source.glob('runs/*/source_manifest.json')).parent
for path in [source/'acceptance.json',source/'driver.json',source/'timing.json',run/'source_manifest.json',run/'psychology_demo.json',run/'runtime_manifest.txt']:
    if path.exists():shutil.copyfile(path,dest/path.name)
with (dest/'summary.json').open('x') as out:
    subprocess.run([sys.executable,str(Path(__file__).with_name('summarize_run.py')),str(source)],stdout=out,check=True)
paths=[*run.glob('psychology_*.jsonl'),source/'acceptance.actual.jsonl',source/'launch.log']
inv={str(p):dict(bytes=p.stat().st_size,sha256=hashlib.sha256(p.read_bytes()).hexdigest()) for p in paths if p.exists()}
(dest/'trace_inventory.json').write_text(json.dumps(inv,indent=2)+'\n')
