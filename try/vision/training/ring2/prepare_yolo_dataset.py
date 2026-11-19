#!/usr/bin/env python3
"""Convert downloaded MaixHub VOC dataset; remove byte-identical duplicates.
Retains original split, aborts on cross-split overlap; never edits source.
"""
from pathlib import Path
import argparse, hashlib, json, shutil, xml.etree.ElementTree as ET

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source',type=Path,default=Path(__file__).parent/'dataset_original')
    parser.add_argument('--output',type=Path,default=Path(__file__).parent/'dataset_yolo_unique')
    args=parser.parse_args();src=args.source.resolve();out=args.output.resolve()
    if out.exists():raise SystemExit('Output exists; choose a new --output to preserve it.')
    records=[];seen={};counts={'train':0,'val':0}
    for split in ('train','val'):
        for name in (src/(split+'.txt')).read_text().splitlines():
            name=Path(name).name;photo=src/'images'/name
            digest=hashlib.sha256(photo.read_bytes()).hexdigest()
            if digest in seen:
                if seen[digest]!=split:raise ValueError('Duplicate across train/val: '+name)
                continue
            seen[digest]=split
            annotation=ET.parse(src/'annotations'/Path(name).with_suffix('.xml')).getroot()
            width=int(annotation.findtext('size/width'));height=int(annotation.findtext('size/height'))
            lines=[]
            for obj in annotation.findall('object'):
                if obj.findtext('name')!='2r':raise ValueError('Unexpected label')
                b=obj.find('bndbox');x0,y0,x1,y1=[float(b.findtext(k)) for k in ('xmin','ymin','xmax','ymax')]
                assert 0<=x0<x1<=width and 0<=y0<y1<=height,(name,x0,y0,x1,y1)
                lines.append('0 %.8f %.8f %.8f %.8f'%((x0+x1)/2/width,(y0+y1)/2/height,(x1-x0)/width,(y1-y0)/height))
            stem=digest[:20]
            for folder in ('images','labels'):(out/folder/split).mkdir(parents=True,exist_ok=True)
            shutil.copy2(photo,out/'images'/split/(stem+'.jpg'))
            (out/'labels'/split/(stem+'.txt')).write_text('\n'.join(lines)+'\n')
            records.append({'source':name,'sha256':digest,'split':split,'output':stem});counts[split]+=1
    (out/'data.yaml').write_text('path: "'+str(out)+'"\ntrain: images/train\nval: images/val\nnames:\n  0: 2r\n')
    (out/'manifest.json').write_text(json.dumps(records,ensure_ascii=False,indent=2)+'\n')
    print('Converted unique samples:',counts)
if __name__=='__main__':main()
