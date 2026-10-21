"""Convert raw GPU PPM evidence to compact PNGs and labelled contact sheets.

Usage: python tools/raphtalia/convert_gpu_evidence.py build/raphtalia_tools/gpu_prepared docs/raphtalia
Raw PPM files remain in the build directory. Requires Pillow.
"""
from pathlib import Path
import csv
import shutil
import sys
from PIL import Image, ImageDraw


def sheet(rows, destination, columns, tile=300):
    if not rows:
        return
    label_height = 38
    canvas = Image.new("RGB", (columns * tile, len(rows) * (tile + label_height)), "white")
    draw = ImageDraw.Draw(canvas)
    for row_index, row in enumerate(rows):
        for column, (path, label) in enumerate(row):
            image = Image.open(path).convert("RGB")
            image.thumbnail((tile, tile))
            x, y = column * tile, row_index * (tile + label_height)
            draw.text((x + 5, y + 3), label, fill="black")
            canvas.paste(image, (x, y + label_height))
    canvas.save(destination, optimize=True)


def main():
    source = Path(sys.argv[1] if len(sys.argv) > 1 else "build/raphtalia_tools/gpu_prepared")
    output = Path(sys.argv[2] if len(sys.argv) > 2 else "docs/raphtalia")
    output.mkdir(parents=True, exist_ok=True)
    # Convert only the frames referenced by this run. A reused build folder may
    # still contain images from old sampling schemes or clip indices.
    with (source / 'gpu_samples.csv').open(newline='',encoding='utf-8') as f:
        frame_names={r['image'] for r in csv.DictReader(f)}
    if (source / 'gpu_mix.csv').exists():
        with (source / 'gpu_mix.csv').open(newline='',encoding='utf-8') as f:
            frame_names.update(r['image'] for r in csv.DictReader(f))
    view_names = ('front','back','face_front','face_back')
    for line in (source / 'gpu_audit.txt').read_text(encoding='utf-8').splitlines():
        if line.startswith('views='):
            view_names = tuple(line.partition('=')[2].split(','))
    frame_names.update('gpu_neutral_'+v+'.ppm' for v in view_names)
    for ppm in sorted(source / n for n in frame_names):
        with Image.open(ppm) as image:
            image.save(output / f"{ppm.stem}.png", optimize=True)
    for name in ("gpu_audit.txt", "gpu_samples.csv", "gpu_endpoints.csv"):
        shutil.copyfile(source / name, output / name)
    if (source / "gpu_mix.csv").exists():
        shutil.copyfile(source / "gpu_mix.csv", output / "gpu_mix.csv")
    neutral = [(output / f"gpu_neutral_{view}.png", f"Bind pose: {view}")
               for view in ("front", "back", "face_front", "face_back")]
    sheet([neutral[:2], neutral[2:]], output / "gpu_neutral_contact.png", 2, 384)
    with (output / "gpu_samples.csv").open(newline="", encoding="utf-8") as stream:
        samples = list(csv.DictReader(stream))
    clips = {}
    for sample in samples:
        clips.setdefault((int(sample["clip_index"]), sample["clip_name"]), []).append(sample)
    front_rows, back_rows, face_rows = [], [], []
    for (index, name), clip in sorted(clips.items()):
        title = name.split("|")[-1]
        for view, rows in (("front", front_rows), ("back", back_rows)):
            row = []
            for s in clip:
                if s["view"] == view:
                    path = output / (Path(s["image"]).stem + ".png")
                    row.append((path, f"{index}: {title[:34]}\n{float(s['fraction']):.0%} | delta bytes {s['changed_rgb_bytes_vs_bind']}"))
            rows.append(row)
        for fraction in dict.fromkeys(s['fraction'] for s in clip):
            row = []
            for s in clip:
                if s["view"].startswith("face_") and s['fraction'] == fraction:
                    row.append((output / (Path(s["image"]).stem + ".png"),
                                f"{index}: {title[:34]}\n{float(s['fraction']):.1%} | {s['view']} | bytes {s['changed_rgb_bytes_vs_bind']}"))
            if row:
                face_rows.append(row)
    sheet(front_rows, output / "gpu_front_contact.png", max(map(len,front_rows),default=3))
    sheet(back_rows, output / "gpu_back_contact.png", max(map(len,back_rows),default=3))
    sheet(face_rows, output / "gpu_face_contact.png", 2)
    detail_rows = []
    for (index, name), clip in sorted(clips.items()):
        for view in view_names[4:]:
            row = [(output / (Path(s['image']).stem + '.png'),
                    f"{index}: {view}\n{float(s['fraction']):.1%} | {name.split('|')[-1][:28]}")
                   for s in clip if s['view'] == view]
            if row:
                detail_rows.append(row)
    sheet(detail_rows, output / 'gpu_body_cloth_contact.png',
          max(map(len, detail_rows), default=1), 320)
    if (output / "gpu_mix.csv").exists():
        with (output / "gpu_mix.csv").open(newline="", encoding="utf-8") as stream:
            mixed = list(csv.DictReader(stream))
        rows = []
        for name in dict.fromkeys(s["pose"] for s in mixed):
            row = [(output / (Path(s["image"]).stem + ".png"),
                    f"{s['pose']} | {s['view']}\nchanged bytes {s['changed_rgb_bytes_vs_bind']}")
                   for s in mixed if s["pose"] == name and s["view"] in ("front", "back")]
            rows.append(row)
        sheet(rows, output / "gpu_mix_contact.png", 2)
    print(f"Converted {len(frame_names)} GPU frames; {len(clips)} clips; output {output}")


if __name__ == "__main__":
    main()
