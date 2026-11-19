#!/usr/bin/env python3
"""Create an independent VOC augmentation set without modifying source data.

Only unique training images generate variants. Original train/val files are
copied byte-for-byte. Standard four-corner affine box bounds are conservative
for rings; rotations are limited to six degrees. No mirror or cut-off targets.
"""
import hashlib
import json
import math
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path
from zipfile import ZipFile, ZIP_DEFLATED

import cv2
import numpy as np
from PIL import Image, ImageDraw

ROOT = Path(__file__).resolve().parent
SOURCE = ROOT / 'dataset_original'
OUT = ROOT / 'augmentation_20261119_v1'
SEED = 20261119


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    if OUT.exists():
        raise SystemExit('Refusing to overwrite existing output: ' + str(OUT))
    before = {str(p.relative_to(SOURCE)): sha(p) for p in SOURCE.rglob('*') if p.is_file()}
    dataset = OUT / 'dataset'
    shutil.copytree(SOURCE, dataset)
    split_names = {s: (SOURCE / (s + '.txt')).read_text().splitlines() for s in ('train', 'val')}
    val_hashes = {sha(SOURCE / 'images' / n) for n in split_names['val']}
    unique = {}
    for name in split_names['train']:
        digest = sha(SOURCE / 'images' / name)
        assert digest not in val_hashes, 'Cross-split duplicate'
        unique.setdefault(digest, name)
    rng = np.random.default_rng(SEED)
    manifest = []
    for source_index, (digest, name) in enumerate(unique.items()):
        original = cv2.imread(str(SOURCE / 'images' / name))
        h, w = original.shape[:2]
        root = ET.parse(SOURCE / 'annotations' / Path(name).with_suffix('.xml')).getroot()
        assert len(root.findall('object')) == 1
        box = root.find('object/bndbox')
        x0, y0, x1, y1 = [float(box.findtext(k)) for k in ('xmin', 'ymin', 'xmax', 'ymax')]
        corners = np.array([[x0,y0,1],[x1,y0,1],[x0,y1,1],[x1,y1,1]], dtype=float)
        border = np.concatenate([original[:8].reshape(-1,3), original[-8:].reshape(-1,3)])
        fill = tuple(float(x) for x in np.median(border, axis=0))
        for variant in range(16):
            # Four photometric-only examples retain the original spatial distribution.
            if variant < 4:
                angle, scale = 0.0, 1.0
                matrix = np.array([[1.,0.,0.],[0.,1.,0.]])
            else:
                angle = float(rng.uniform(-6, 6))
                scale = float(rng.uniform(.62, 1.03))
                matrix = cv2.getRotationMatrix2D(((x0+x1)/2, (y0+y1)/2), angle, scale)
                moved = corners @ matrix.T
                lower, upper = moved.min(axis=0), moved.max(axis=0)
                matrix[:,2] += [rng.uniform(9-lower[0], w-9-upper[0]),
                                 rng.uniform(9-lower[1], h-9-upper[1])]
            moved = corners @ matrix.T
            lo, hi = moved.min(axis=0), moved.max(axis=0)
            bbox = [math.floor(lo[0]), math.floor(lo[1]), math.ceil(hi[0]), math.ceil(hi[1])]
            assert 0 <= bbox[0] < bbox[2] <= w and 0 <= bbox[1] < bbox[3] <= h
            transformed = cv2.warpAffine(original, matrix, (w,h), flags=cv2.INTER_LINEAR,
                                         borderMode=cv2.BORDER_CONSTANT, borderValue=fill)
            gamma = float(rng.uniform(.78, 1.25))
            contrast = float(rng.uniform(.85, 1.16))
            brightness = float(rng.uniform(-16, 16))
            gradient = float(rng.uniform(-.17, .17))
            arr = 255 * (transformed.astype(np.float32)/255) ** gamma
            arr = (arr-127.5)*contrast+127.5+brightness
            arr *= (1+np.linspace(-gradient, gradient, w))[None,:,None]
            noise = float(rng.uniform(0, 2.8))
            arr = np.clip(arr+rng.normal(0,noise,arr.shape),0,255).astype(np.uint8)
            blur = 'none'
            if variant % 4 == 1:
                sigma = float(rng.uniform(.35,.8))
                arr = cv2.GaussianBlur(arr,(3,3),sigma)
                blur = {'gaussian_sigma':sigma}
            elif variant % 8 == 3:
                kernel = np.zeros((3,3),np.float32)
                kernel[1,:] = 1/3
                arr = cv2.filter2D(arr,-1,kernel)
                blur = {'horizontal_motion_kernel':3}
            quality = int(rng.integers(83,98))
            new_name = f'aug_{source_index:02d}_{digest[:8]}_{variant:02d}.jpg'
            assert cv2.imwrite(str(dataset/'images'/new_name),arr,[cv2.IMWRITE_JPEG_QUALITY,quality])
            new_root = ET.fromstring(ET.tostring(root))
            new_root.find('filename').text = new_name
            for key, value in zip(('xmin','ymin','xmax','ymax'), bbox):
                new_root.find('object/bndbox/'+key).text = str(value)
            path_element = new_root.find('path')
            if path_element is not None:
                path_element.text = 'images/'+new_name
            ET.ElementTree(new_root).write(dataset/'annotations'/Path(new_name).with_suffix('.xml'),
                                           encoding='utf-8',xml_declaration=True)
            split_names['train'].append(new_name)
            manifest.append({'output':new_name,'source':name,'source_sha256':digest,
                             'seed':SEED,'angle_degrees':angle,'scale':scale,
                             'affine_matrix':matrix.tolist(),'bbox':bbox,
                             'gamma':gamma,'contrast':contrast,'brightness':brightness,
                             'horizontal_light_gradient':gradient,'noise_sigma':noise,
                             'blur':blur,'jpeg_quality':quality,
                             'output_sha256':sha(dataset/'images'/new_name)})
    (dataset/'train.txt').write_text('\n'.join(split_names['train'])+'\n')
    # Full source immutability and exact validation retention, including split bytes.
    assert before == {str(p.relative_to(SOURCE)):sha(p) for p in SOURCE.rglob('*') if p.is_file()}
    for name in split_names['val']:
        for relative in ('images/'+name, 'annotations/'+str(Path(name).with_suffix('.xml'))):
            assert sha(SOURCE/relative) == sha(dataset/relative)
    assert sha(SOURCE/'val.txt') == sha(dataset/'val.txt')
    for split,names in split_names.items():
        assert len(names) == len(set(names))
        for name in names:
            image = cv2.imread(str(dataset/'images'/name))
            h,w = image.shape[:2]
            tree = ET.parse(dataset/'annotations'/Path(name).with_suffix('.xml'))
            assert int(tree.findtext('size/width')) == w and int(tree.findtext('size/height')) == h
            for obj in tree.findall('object'):
                assert obj.findtext('name') == '2r'
                b = [int(obj.findtext('bndbox/'+k)) for k in ('xmin','ymin','xmax','ymax')]
                assert 0<=b[0]<b[2]<=w and 0<=b[1]<b[3]<=h
    assert not val_hashes.intersection(sha(dataset/'images'/n) for n in split_names['train'])
    (OUT/'manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2)+'\n')
    audit = {'seed':SEED,'original_train':42,'unique_train_sources':len(unique),
             'generated':len(manifest),'train':len(split_names['train']), 'val':len(split_names['val']),
             'source_files_unchanged':len(before),'validation_preserved_byte_for_byte':True,
             'cross_split_exact_duplicates':0,'all_boxes_valid_and_unclipped':True,
             'box_method':'Affine of four original box corners; conservative axis-aligned bounds.',
             'limitations':['Synthetic variants do not add independent scenes.',
                           'No real high-view camera images or negative scenes added.',
                           'Exact hash split audit does not rule out visually similar scenes.'],
             'versions':{'opencv':cv2.__version__,'numpy':np.__version__}}
    (OUT/'audit.json').write_text(json.dumps(audit,ensure_ascii=False,indent=2)+'\n')
    (OUT/'source_sha256.json').write_text(json.dumps(before,ensure_ascii=False,indent=2)+'\n')
    # Every generated image is included in a numbered visual QA sheet.
    for start in range(0,len(manifest),32):
        rows = manifest[start:start+32]
        sheet = Image.new('RGB',(416*4,258*math.ceil(len(rows)/4)),'white')
        draw = ImageDraw.Draw(sheet)
        for i,item in enumerate(rows):
            im = Image.open(dataset/'images'/item['output']).convert('RGB')
            ImageDraw.Draw(im).rectangle(item['bbox'],outline=(0,255,0),width=2)
            x,y = (i%4)*416,(i//4)*258
            sheet.paste(im,(x,y))
            draw.text((x+4,y+236),item['output'],fill='black')
        sheet.save(OUT/f'qa_{start//32+1:02d}.jpg',quality=90)
    archive = OUT/'ring2_aug_v1_voc.zip'
    with ZipFile(archive,'w',ZIP_DEFLATED) as z:
        for p in sorted(dataset.rglob('*')):
            if p.is_file():
                z.write(p,p.relative_to(dataset))
    with ZipFile(archive) as z:
        assert z.testzip() is None
    print(json.dumps(audit,ensure_ascii=False,indent=2))
    print('Archive:',archive,'SHA256:',sha(archive))


if __name__ == '__main__':
    main()
