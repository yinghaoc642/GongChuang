#!/usr/bin/env python3
"""Evaluate downloaded report samples and audit artifacts; no model inference.

Run with the existing MISC environment (Pillow is used for the preview).
This script only writes evaluation artifacts next to itself.
"""
import configparser
import hashlib
import importlib.util
import json
import statistics
import zipfile
from pathlib import Path
from types import SimpleNamespace

from PIL import Image, ImageDraw

HERE = Path(__file__).resolve().parent
RING = HERE.parent
VISION = RING.parent.parent
MODELS = VISION / "models/ring2"
SOURCE = RING / "dataset_original"
IDS = (331402, 331476, 331477)


def load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def save(name, value):
    (HERE / name).write_text(json.dumps(value, ensure_ascii=False, indent=2) + "\n")


def main():
    compare = load_module("compare_reports", RING / "tuning_20261119/compare_reports.py")
    runtime = load_module("ring2_coarse", VISION / "ring2_coarse.py")
    results, reports, replay, artifacts = [], {}, [], []
    preview = Image.new("RGB", (416 * 3, 286 * 2), "white")
    pen = ImageDraw.Draw(preview)
    for tid in IDS:
        folder = MODELS / f"trained_{tid}"
        report_path = folder / "report.json"
        report = reports[tid] = json.loads(report_path.read_text())
        result = compare.evaluate(report_path, 0.70)
        result["training_id"] = tid
        result["path"] = str(report_path.relative_to(VISION.parent))
        matches = [m for s in result["details"] for m in s["matches"]]
        result["summary"] = {
            "min_score": min(m["score"] for m in matches),
            "max_score": max(m["score"] for m in matches),
            "min_iou": min(m["iou"] for m in matches),
            "mean_iou": statistics.mean(m["iou"] for m in matches),
            "mean_center_error_px": statistics.mean(m["center_error_px"] for m in matches),
            "max_center_error_px": max(m["center_error_px"] for m in matches),
        }
        results.append(result)
        details = []
        for sample in report["best_eval"]["val_info"]:
            objects = [SimpleNamespace(x=p[0], y=p[1], w=p[2]-p[0], h=p[3]-p[1],
                                       score=p[4], class_id=int(p[5])) for p in sample["pred"]]
            candidate = runtime.box_candidate(objects, report["labels"])
            details.append({"name": sample["nickname"], "pass": candidate is not None,
                            "candidate": candidate})
        replay.append({"training_id": tid, "passed": sum(d["pass"] for d in details),
                       "total": len(details), "details": details})
        if tid == 331402:
            continue
        archive = MODELS / f"model-{tid}.maixcam.zip"
        with zipfile.ZipFile(archive) as z:
            assert z.testzip() is None, f"CRC failed: {tid}"
            for name in z.namelist():
                assert (folder / name).read_bytes() == z.read(name), name
        mud = configparser.ConfigParser()
        mud.read(folder / f"model_{tid}.mud")
        assert mud["basic"]["model"] == f"model_{tid}.cvimodel"
        assert mud["extra"]["model_type"] == "yolov5"
        assert mud["extra"]["labels"] == "2r"
        assert report["inputs"] == {"input0": [224, 224, 3]}
        artifacts.append({"training_id": tid, "archive_sha256": digest(archive),
                          "crc_ok": True, "extracted_files_match_archive": True,
                          "files": {p.name: {"size": p.stat().st_size, "sha256": digest(p)}
                                    for p in folder.iterdir() if p.is_file()}})
        for col, sample in enumerate(report["best_eval"]["val_info"]):
            photo = SOURCE / "images" / sample["nickname"]
            im = Image.open(photo).convert("RGB")
            assert im.size == (416, 234)
            draw = ImageDraw.Draw(im)
            for box in sample["gt"]:
                draw.rectangle(box[:4], outline=(20, 210, 30), width=2)
            for box in sample["pred"]:
                draw.rectangle(box[:4], outline=(240, 50, 30), width=2)
            x, y = col * 416, (tid - 331476) * 286
            pen.text((x+8, y+7), f"{tid}  {sample['nickname']}  score {sample['pred'][0][4]:.4f}", fill="black")
            pen.text((x+8, y+24), "green: report GT | red: prediction", fill="black")
            preview.paste(im, (x, y+48))
    save("comparison.json", results)
    save("artifact_integrity.json", artifacts)
    save("runtime_gate_replay.json", {
        "scope": "Static replay of report boxes in original416x234 coordinates through existing box_candidate. Camera uses320x240: not hardware inference, temporal stability, Hough refinement or full validation.",
        "runtime_source_sha256": digest(VISION / "ring2_coarse.py"), "results": replay})
    preview.save(HERE / "report_predictions.jpg", quality=95)

    source_hashes = json.loads((HERE / "source_sha256.json").read_text())
    changed = [name for name, sha in source_hashes.items() if digest(SOURCE / name) != sha]
    assert not changed, changed
    assert (SOURCE / "val.txt").read_bytes() == (HERE / "dataset/val.txt").read_bytes()
    validation = []
    for name in (SOURCE / "val.txt").read_text().splitlines():
        for relative in (Path("images") / name, Path("annotations") / Path(name).with_suffix(".xml")):
            assert (SOURCE / relative).read_bytes() == (HERE / "dataset" / relative).read_bytes()
        validation.append({"name": name, "sha256": digest(SOURCE / "images" / name)})
    save("dataset_integrity_after_training.json", {
        "source_files_checked": len(source_hashes), "source_files_changed": changed,
        "validation_images_and_annotations_byte_identical": True,
        "validation_list_byte_identical": True, "validation": validation})

    first = {s["name"]: s for s in reports[331476]["best_eval"]["val_info"]}
    second = {s["name"]: s for s in reports[331477]["best_eval"]["val_info"]}
    paired = []
    for name in first.keys() & second.keys():
        a, b = first[name], second[name]
        assert a["gt"] == b["gt"] and a["nickname"] == b["nickname"]
        paired.append({"server_image_id": name, "nickname": a["nickname"],
                       "source_sha256": digest(SOURCE / "images" / a["nickname"]),
                       "results": {str(tid): next(d for d in r["details"] if d["name"] == a["nickname"])
                                   for tid, r in zip(IDS, results) if tid != 331402}})
    save("paired_samples.json", {"count": len(paired), "pairs": paired,
                                 "scope": "Only common image IDs exposed by both new reports; other sampled images differ."})
    print(json.dumps([{ "id": r["training_id"], **r["summary"]} for r in results], indent=2))
    print(f"Verified {len(source_hashes)} unchanged source files; 12 unchanged validation images/XML; archives and metadata valid.")
    print("Static runtime box gate:", [(r["training_id"], r["passed"], r["total"]) for r in replay])


if __name__ == "__main__":
    main()
