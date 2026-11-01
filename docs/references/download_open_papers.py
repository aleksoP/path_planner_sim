#!/usr/bin/env python3
"""Fetch verified/discovered open PDF copies to a local folder. No authentication or paywall bypass.
Run in a network-enabled terminal: python3 download_open_papers.py --all
"""
import argparse, json, os, time, urllib.request, urllib.error, tempfile
from pathlib import Path
p=argparse.ArgumentParser()
p.add_argument('--all',action='store_true',help='Include cooperative/fleet-focused papers (default downloads sensor-only papers)')
p.add_argument('--list',action='store_true',help='List URLs and exit')
p.add_argument('--destination',type=Path,default=Path(__file__).parent/'02_downloaded_open_pdfs')
p.add_argument('--timeout',type=int,default=50)
args=p.parse_args()
base=Path(__file__).resolve().parent
rows=json.loads((base/'paper_manifest.json').read_text())['open_pdf_links']
if not args.all:rows=[r for r in rows if r['mode']=='onboard']
if args.list:
 for r in rows:print(f"{r['id']}: {r['pdf_url']}")
 raise SystemExit(0)
args.destination.mkdir(parents=True,exist_ok=True)
success=0; skipped=0; failed=[]
for r in rows:
 name=''.join(c if (c.isascii() and (c.isalnum() or c in '-_')) else '_' for c in r['id'])+'.pdf'
 target=args.destination/name
 if target.exists() and target.stat().st_size>1000 and target.read_bytes()[:5]==b'%PDF-':
  print('SKIP',name);skipped+=1;continue
 try:
  req=urllib.request.Request(r['pdf_url'],headers={'User-Agent':'Mozilla/5.0 (research literature download; single user)', 'Accept':'application/pdf'})
  with urllib.request.urlopen(req, timeout=args.timeout) as resp, open(target.with_suffix('.part'),'wb') as f:
   while True:
    chunk=resp.read(1<<18)
    if not chunk:break
    f.write(chunk)
  if target.with_suffix('.part').stat().st_size<1000 or target.with_suffix('.part').open('rb').read(5)!=b'%PDF-':
   raise ValueError('source did not return a PDF')
  target.with_suffix('.part').replace(target);success+=1
  print('OK  ', name, f'{target.stat().st_size/1048576:.1f} MiB')
 except Exception as exc:
  target.with_suffix('.part').unlink(missing_ok=True)
  failed.append((r['id'],str(exc)))
  print('FAIL',r['id'],exc)
 time.sleep(0.7)
print(f"\nSuccess={success}, already present={skipped}, failed={len(failed)}")
if failed:
 for ident,err in failed: print(f' - {ident}: {err}')
 print('Some academic sites rate-limit automated downloads. Use PDF links in the index manually as needed.')
