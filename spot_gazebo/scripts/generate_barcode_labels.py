#!/usr/bin/env python3
"""
generate_barcode_labels.py — migrate server_rack_1..8 from RFID tags to
printed Code 128 barcode labels for the Zebra Symbol LS2208.

WHAT THIS DOES
  1. Assigns each asset a sequential 7-hex-digit ID (031FB00, 031FB01, ...),
     matching the physical tags.
  2. Renders one label texture per asset: a Code 128 Subset B symbol plus the
     human-readable code underneath, rasterized at an EXACT integer number of
     pixels per module so the bars stay crisp and a camera-based decoder could
     read them later.
  3. Rewrites models/server_rack_{1..8}/model.sdf, replacing every
     <model name="rfid_tag_*"> block (and its CommsEndpoint plugin) with a
     <model name="barcode_label_<HEX>"> block carrying that texture.
  4. Rewrites rack_tag_manifest.csv with the new names and the barcode column.

  The RFID plugin sources (RfidReader.cc, RfidPdReader.cc) and their build
  targets are NOT touched. Only the world content changes.

WHY THE NUMBERS ARE WHAT THEY ARE
  Physical tag  38.1 x 12.7 x 6.35 mm  (1.5" x 0.5" x 0.25")
  Print area    28 x 3 mm, 7-character code beneath

  A 7-character hex payload cannot be packed by Code 128 Subset C, which
  encodes digit PAIRS and has no A-F. Budget it as 7 Subset B codewords:

      start 11 + 7*11 data + 11 check + 13 stop      = 112 modules
      + 10 modules quiet zone on each side           = 132 modules

  python-barcode actually emits a mixed encoding — it starts in Subset C, pairs
  the leading "03", then spends a TO_B switch for the rest — but that comes to
  the same 112 modules, because the pair it saves costs exactly one switch
  codeword. The assertion in render_label() checks the real output against this
  budget and hard-fails if a future prefix breaks the coincidence.

  At 7.5 mil (0.1905 mm/module) that is 25.15 mm — fits the 28 mm print area
  with 1.4 mm of margin each side. At 10 mil the bars alone would be 28.4 mm,
  already over budget before quiet zones. So 7.5 mil is forced, which fixes
  the LS2208 depth of field at 1.50"-10.00" = 0.0381-0.2540 m (Motorola
  SS-LS2208 12/08, Code 39 7.5 mil row; the sheet lists no Code 128 row, but
  depth of field follows the narrow element width, not the symbology). That
  number is the input to <dof_near>/<dof_far> in the BarcodeScanner plugin
  block, and it is a big reduction from the 1.0 m the RFID reader had.

USAGE
  pip install python-barcode pillow pyzbar     # pyzbar also needs libzbar0
  python3 generate_barcode_labels.py --dry-run
  python3 generate_barcode_labels.py
  python3 generate_barcode_labels.py --verify  # round-trip decode every PNG
"""

from __future__ import annotations

import argparse
import csv
import math
import re
import shutil
import sys
from dataclasses import dataclass
from pathlib import Path

# --------------------------------------------------------------------------
# Geometry. Every dimension in metres unless the name says otherwise.
# --------------------------------------------------------------------------

TAG_W = 0.0381        # 1.5"  substrate width,  label local +X
TAG_H = 0.0127        # 0.5"  substrate height, label local +Z
TAG_T = 0.00635       # 0.25" substrate thickness, label local Y

PRINT_W = 0.028       # print area width
BAR_H = 0.003         # bar height

MIL = 7.5
MODULE_M = MIL * 0.0254 / 1000.0        # 0.0001905 m
MODULES_QUIET = 10                      # per side, Code 128 minimum

# Faceplate is the plane y = -0.50 in the rack frame, and the equipment boxes'
# front faces land exactly there. Seat the label 1 mm proud of it — the same
# clearance the 4 mm RFID tags had at y = -0.503. Do NOT just subtract half the
# new thickness from the old centre: 6.35 mm of substrate at y = -0.5032 leaves
# 25 micrometres, which z-fights against the equipment.
LABEL_Y = -0.5042                       # spans -0.501025 .. -0.507375

BEAM_SPOT = 0.0002                      # LS2208 spot diameter; must stay <=
                                        # one narrow module to resolve bars

# Ignition maps a box face's UV rotated 90 deg clockwise relative to the way
# you would expect from the box's own X/Y extents, so an upright label texture
# renders with the bars running along the SHORT axis of the tag and the
# human-readable line turned on its side. Pre-rotating the image by the same
# amount counter-clockwise cancels it exactly.
#
# This is a renderer convention, not something derivable from the SDF, so it is
# a constant rather than a computed value: if a label comes out upside down
# rather than correct, negate it; if a future Ignition release fixes the UV
# mapping, set it to 0.
TEXTURE_ROTATE_DEG = 90

PX_PER_MODULE = 4                       # integer => no resampling artefacts
PX_PER_M = PX_PER_MODULE / MODULE_M     # ~21000 px/m == 21 px/mm

FIRST_ID = 0x031FB00
LABEL_PREFIX = "barcode_label_"

# --------------------------------------------------------------------------

HERE = Path(__file__).resolve().parent
MODELS = (HERE / ".." / "models").resolve()
TEXTURE_MODEL = MODELS / "barcode_labels"
TEXTURE_DIR = TEXTURE_MODEL / "materials" / "textures"
MANIFEST = MODELS / "rack_tag_manifest.csv"

TAG_BLOCK_RE = re.compile(
    r'[ \t]*<model name="(?P<name>rfid_tag_[^"]+)">.*?</model>\n',
    re.DOTALL,
)


@dataclass
class Asset:
    rack_model: str
    tag_name: str
    epc: str
    equipment: str
    u_start: str
    u_end: str
    u_size: str
    depth_m: str
    z_local_m: str
    x_local_m: str
    code: str = ""          # 031FB0D

    @property
    def label_name(self) -> str:
        return LABEL_PREFIX + self.code


def symbol_modules(payload: str) -> int:
    """Total Code 128 Subset B module count including quiet zones."""
    return 11 + 11 * len(payload) + 11 + 13 + 2 * MODULES_QUIET


def roll_limit_deg(payload: str = "0000000") -> float:
    """The scanner's real roll limit, in degrees.

    A scan line rolled by theta drifts symbol_width*tan(theta) vertically
    across the symbol and must still land on the bars. Mirrors the
    <auto_roll_limit> computation in BarcodeScanner.cc — keep the two in step.
    """
    return math.degrees(
        math.atan2(BAR_H + BEAM_SPOT, symbol_modules(payload) * MODULE_M)
    )


def check_fit(payload: str) -> None:
    n = symbol_modules(payload)
    width = n * MODULE_M
    if width > PRINT_W:
        raise SystemExit(
            f"Payload {payload!r} needs {n} modules = {width*1000:.2f} mm at "
            f"{MIL} mil, but the print area is only {PRINT_W*1000:.1f} mm. "
            f"Shorten the payload or drop to a finer mil (and shorten the "
            f"depth of field accordingly)."
        )


# --------------------------------------------------------------------------
# Texture rendering
# --------------------------------------------------------------------------

def render_label(payload: str, out: Path) -> None:
    """Render one 38.1 x 12.7 mm label face to PNG.

    Layout, top to bottom: white margin, the 3 mm-tall Code 128 symbol, a thin
    gap, then the human-readable code. Bars are drawn at exactly
    PX_PER_MODULE pixels wide, so no interpolation ever splits a module.
    """
    from barcode import Code128
    from PIL import Image, ImageDraw, ImageFont

    # python-barcode hands back one string of '1'/'0', one char per module,
    # WITHOUT quiet zones. We add those ourselves so the count is auditable.
    pattern = Code128(payload).build()[0]
    expected = symbol_modules(payload) - 2 * MODULES_QUIET
    if len(pattern) != expected:
        raise SystemExit(
            f"Code 128 module count mismatch for {payload!r}: encoder produced "
            f"{len(pattern)}, the width budget assumed {expected}. The label "
            f"geometry in this script is no longer trustworthy — stop and "
            f"recheck before generating."
        )
    pattern = "0" * MODULES_QUIET + pattern + "0" * MODULES_QUIET

    img_w = int(round(TAG_W * PX_PER_M))
    img_h = int(round(TAG_H * PX_PER_M))
    img = Image.new("RGB", (img_w, img_h), "white")
    draw = ImageDraw.Draw(img)

    sym_w = len(pattern) * PX_PER_MODULE
    sym_h = int(round(BAR_H * PX_PER_M))
    x0 = (img_w - sym_w) // 2
    # The bars MUST be centred on the substrate, because BarcodeScanner gates
    # the scan line against the label model ORIGIN, which is the substrate
    # centre. Its vertical tolerance is only atan(0.5*(3 mm + spot)/range),
    # i.e. +/-1.6 mm, so printing the symbol even 3 mm high — as a
    # visually-balanced "barcode on top, text underneath" layout would —
    # puts every bar outside the band the plugin will ever accept and nothing
    # decodes anywhere. The human-readable text goes BELOW the centred symbol,
    # in the space that leaves.
    y0 = (img_h - sym_h) // 2

    for i, bit in enumerate(pattern):
        if bit == "1":
            x = x0 + i * PX_PER_MODULE
            draw.rectangle([x, y0, x + PX_PER_MODULE - 1, y0 + sym_h - 1],
                           fill="black")

    # Human-readable interpretation, as printed on the physical tag. Sized to
    # fit between the centred symbol and the bottom edge.
    text_h = int(round(0.26 * img_h))
    font = None
    for candidate in (
        "/usr/share/fonts/truetype/dejavu/DejaVuSansMono-Bold.ttf",
        "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
    ):
        if Path(candidate).exists():
            font = ImageFont.truetype(candidate, text_h)
            break
    if font is None:
        font = ImageFont.load_default()

    bbox = draw.textbbox((0, 0), payload, font=font)
    draw.text(
        ((img_w - (bbox[2] - bbox[0])) // 2, y0 + sym_h + int(0.04 * img_h)),
        payload, fill="black", font=font,
    )

    # Composed upright — barcode centred, code beneath, reading left to right —
    # then turned to suit the renderer's box UV. Everything above this line
    # reasons in label coordinates; only the save is in texture coordinates.
    if TEXTURE_ROTATE_DEG:
        img = img.rotate(TEXTURE_ROTATE_DEG, expand=True)

    out.parent.mkdir(parents=True, exist_ok=True)
    img.save(out)


def verify(assets: list[Asset]) -> int:
    """Decode every generated PNG back and confirm the payload survives."""
    from PIL import Image
    from pyzbar.pyzbar import decode

    bad = 0
    for a in assets:
        png = TEXTURE_DIR / f"{a.code}.png"
        img = Image.open(png)
        # Undo TEXTURE_ROTATE_DEG so this checks the label as a scanner sees
        # it, not as the file happens to be stored. zbar would probably cope
        # with the rotation on its own, but then a wrong TEXTURE_ROTATE_DEG
        # would still pass and the whole point is to catch that.
        if TEXTURE_ROTATE_DEG:
            img = img.rotate(-TEXTURE_ROTATE_DEG, expand=True)
        results = decode(img)
        got = [r.data.decode() for r in results
               if r.type in ("CODE128", "CODE-128")]
        if got != [a.code]:
            print(f"  FAIL {a.code}: decoded {got!r}")
            bad += 1
    print(f"verify: {len(assets) - bad}/{len(assets)} labels round-tripped")
    return bad


# --------------------------------------------------------------------------
# SDF rewriting
# --------------------------------------------------------------------------

def label_sdf(a: Asset, indent: str = "    ") -> str:
    """One barcode label model, replacing one rfid_tag model.

    Frame convention required by BarcodeScanner: outward normal is the label's
    local -Y, symbol long axis is local +X, bar height is local +Z. With the
    rack faceplate normal already pointing along -Y, an unrotated label
    satisfies this for free.
    """
    tex = f"model://barcode_labels/materials/textures/{a.code}.png"
    i = indent
    return (
        f'{i}<!-- {a.equipment}, U{a.u_start}-{a.u_end} -->\n'
        f'{i}<model name="{a.label_name}">\n'
        f'{i}  <static>true</static>\n'
        f'{i}  <pose>{a.x_local_m} {LABEL_Y} {a.z_local_m} 0 0 0</pose>\n'
        f'{i}  <link name="label_link">\n'
        f'{i}    <visual name="substrate">\n'
        f'{i}      <geometry><box><size>'
        f'{TAG_W:.5f} {TAG_T:.5f} {TAG_H:.5f}'
        f'</size></box></geometry>\n'
        f'{i}      <material>\n'
        f'{i}        <ambient>0.9 0.9 0.9 1</ambient>\n'
        f'{i}        <diffuse>0.9 0.9 0.9 1</diffuse>\n'
        f'{i}      </material>\n'
        f'{i}    </visual>\n'
        f'{i}    <!-- Printed face, 0.1 mm proud of the substrate so it never\n'
        f'{i}         z-fights. Rotating +90 deg about X sends the quad\'s +Z\n'
        f'{i}         face to -Y (the aisle) and its +Y to world up, so the\n'
        f'{i}         texture reads the right way round. -90, which is what\n'
        f'{i}         calibration_plate_model uses, points the BACK face out\n'
        f'{i}         and flips the image. Code 128 is bi-directional so a\n'
        f'{i}         mirrored symbol still decodes, but the human-readable\n'
        f'{i}         line would be reversed. roughness 1 / metalness 0 keeps\n'
        f'{i}         it matte: a specular highlight across the bars is\n'
        f'{i}         exactly what stops a real scanner decoding. -->\n'
        f'{i}    <visual name="print">\n'
        f'{i}      <pose>0 {-(TAG_T / 2 + 0.0001):.5f} 0 1.5708 0 0</pose>\n'
        f'{i}      <geometry><box><size>'
        f'{TAG_W:.5f} {TAG_H:.5f} 0.0002'
        f'</size></box></geometry>\n'
        f'{i}      <material>\n'
        f'{i}        <ambient>1 1 1 1</ambient>\n'
        f'{i}        <diffuse>1 1 1 1</diffuse>\n'
        f'{i}        <pbr><metal>\n'
        f'{i}          <albedo_map>{tex}</albedo_map>\n'
        f'{i}          <metalness>0.0</metalness>\n'
        f'{i}          <roughness>1.0</roughness>\n'
        f'{i}        </metal></pbr>\n'
        f'{i}      </material>\n'
        f'{i}    </visual>\n'
        f'{i}  </link>\n'
        f'{i}</model>\n'
    )


def patch_header(text: str) -> str:
    """Bring each rack's leading XML comment in line with the barcode models.

    These blocks document the RFID contract in detail — tag naming, the 3 mm
    protrusion, and two hard requirements on the including world. Every one of
    those statements is false once the tags become printed labels, and a stale
    header that confidently describes the wrong thing is worse than no header.
    Whole paragraphs are replaced rather than single sentences: patching only
    the first line of a three-line item leaves a dangling remainder that still
    asserts the CommsEndpoint dependency.

    Written to be a no-op on files that don't carry the text, and on files
    already migrated, so re-running is safe.
    """
    text = re.sub(
        r"(\d+)U empty, (\d+) tags\.",
        r"\1U empty, \2 barcode labels.",
        text,
    )

    text = re.sub(
        r"===== Tags\n.*?the name here is final\.",
        (
            "===== Barcode labels\n"
            "  Nested models named " + LABEL_PREFIX + "<7 hex digits>, on the "
            "front faceplate\n"
            "  (front of rack = -Y, local y = -0.50; labels sit 1 mm proud at "
            f"y = {LABEL_Y}).\n"
            "  VISUAL-ONLY (no <collision>) so lidar / depth sensors and "
            "costmaps are not\n"
            f"  affected by the {TAG_T * 1000:.2f} mm protrusions.\n"
            "\n"
            "  BarcodeScanner matches the PLAIN, UNSCOPED components::Name "
            "with no world-parent\n"
            "  filter, so nested labels are discovered exactly like top-level "
            "ones. Unlike the\n"
            "  rfid_tag_r1_* names these replaced, the model name is the "
            "barcode PAYLOAD and\n"
            "  carries no rack or equipment hint — that join lives in\n"
            "  rack_tag_manifest.csv, which is the point: the scanner reports "
            "a string and\n"
            "  nothing else, exactly as the hardware does."
        ),
        text,
        flags=re.DOTALL,
    )

    text = re.sub(
        r"  2\. The RFComms world plugin must be present.*?never the EPC "
        r"address\.",
        (
            "  2. Nothing else. A printed label is passive geometry with no "
            "plugins, so\n"
            "     this model no longer needs the RFComms world plugin the "
            "CommsEndpoint\n"
            "     tags required."
        ),
        text,
        flags=re.DOTALL,
    )

    return text


def rewrite_rack(path: Path, by_tag: dict[str, Asset], dry: bool) -> int:
    text = path.read_text()
    n = 0

    def sub(m: re.Match) -> str:
        nonlocal n
        name = m.group("name")
        a = by_tag.get(name)
        if a is None:
            print(f"  ! {path.name}: {name} is not in the manifest, left as-is")
            return m.group(0)
        n += 1
        return label_sdf(a)

    out = TAG_BLOCK_RE.sub(sub, text)

    out = patch_header(out)

    if not dry and out != text:
        shutil.copy2(path, path.with_suffix(".sdf.rfid.bak"))
        path.write_text(out)
    return n


def write_manifest(assets: list[Asset], dry: bool) -> None:
    header = f"""\
# GROUND-TRUTH ASSET MANIFEST for server_rack_1..8
# Read with: pandas.read_csv(path, comment='#')
# Generated by scripts/generate_barcode_labels.py. If you edit a rack, re-run it.
#
# barcode      : the Code 128 payload printed on the label, and the ONLY thing
#                the LS2208 reports. 7 hex digits at {MIL} mil,
#                {symbol_modules('0000000')} modules = {symbol_modules('0000000')*MODULE_M*1000:.2f} mm wide inside a {PRINT_W*1000:.0f} mm print area.
#                Budgeted as 7 Subset B codewords; the encoder actually emits a
#                mixed C-then-B sequence that comes to the same module count.
# label_name   : the world model name, always {LABEL_PREFIX}<barcode>. The
#                scanner derives the payload from this suffix, so the model
#                name no longer leaks equipment type to the estimator the way
#                rfid_tag_r1_ups did.
# tag_name     : the old RFID model name. Retained ONLY to join against
#                historical bags and the RFID worlds, which are unchanged.
# epc          : the old CommsEndpoint address. Dead for the barcode path.
# z_local_m    : label z in the RACK frame. World z = rack pose z + this.
# x_local_m    : label x in the RACK frame, the label COLUMN.
#                Label y is always {LABEL_Y} (substrate is {TAG_T*1000:.2f} mm thick, so it
#                spans y = {LABEL_Y - TAG_T/2:.6f}..{LABEL_Y + TAG_T/2:.6f}, entirely in front of
#                the -0.50 faceplate plane).
#
# Substrate {TAG_W*1000:.1f} x {TAG_H*1000:.1f} x {TAG_T*1000:.2f} mm. Bar height {BAR_H*1000:.0f} mm, which is what forces the
# scanner's roll limit down to atan(({BAR_H*1000:.1f} + {BEAM_SPOT*1000:.1f}) / {symbol_modules('0000000')*MODULE_M*1000:.2f}) = {roll_limit_deg():.1f} deg,
# far tighter than the LS2208's published +/-30 deg. The denominator is the
# SYMBOL width incl. quiet zones, not the {PRINT_W*1000:.0f} mm print area, and the numerator
# includes the beam spot; quoting a bare atan({BAR_H*1000:.0f}/{PRINT_W*1000:.0f}) = 6.1 deg understates it.
#
# NB the codes are unique per ASSET, not per physical label. server_room.sdf
# includes racks 1-4 on rows 0 and 2 and racks 5-8 on rows 1 and 3, so all
# {len(assets)} codes appear TWICE in that world, {2*len(assets)} labels in total. A decoded
# string does not identify a location on its own. This is inherited from the
# RFID setup, whose EPCs duplicated the same way.
"""
    rows = [
        [
            a.rack_model, a.label_name, a.code, a.tag_name, a.epc,
            a.equipment, a.u_start, a.u_end, a.u_size, a.depth_m,
            a.z_local_m, a.x_local_m,
        ]
        for a in assets
    ]
    cols = [
        "rack_model", "label_name", "barcode", "tag_name", "epc",
        "equipment", "u_start", "u_end", "u_size", "depth_m",
        "z_local_m", "x_local_m",
    ]
    if dry:
        print(f"  would write {MANIFEST} ({len(rows)} rows)")
        return
    # Guard the backup: rewrite_rack only backs up when it changed something,
    # but this runs unconditionally, so an unguarded copy2 would overwrite the
    # pristine RFID manifest with the already-migrated one on a second run.
    backup = MANIFEST.with_suffix(".csv.rfid.bak")
    if not backup.exists():
        shutil.copy2(MANIFEST, backup)
    with MANIFEST.open("w", newline="") as fh:
        fh.write(header)
        w = csv.writer(fh)
        w.writerow(cols)
        w.writerows(rows)


def write_model_config(dry: bool) -> None:
    cfg = TEXTURE_MODEL / "model.config"
    body = """<?xml version="1.0"?>
<model>
  <name>barcode_labels</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <description>
    Texture-only model. Holds the generated Code 128 label PNGs so the rack
    models can reference them as model://barcode_labels/materials/textures/*.
    Never included in a world directly.
  </description>
</model>
"""
    stub = """<?xml version="1.0"?>
<sdf version="1.9">
  <!-- Intentionally empty. This model exists solely as a texture resource
       path for model://barcode_labels/materials/textures/*.png -->
  <model name="barcode_labels"><static>true</static>
    <link name="link"/>
  </model>
</sdf>
"""
    if dry:
        print(f"  would write {cfg} and model.sdf stub")
        return
    TEXTURE_MODEL.mkdir(parents=True, exist_ok=True)
    cfg.write_text(body)
    (TEXTURE_MODEL / "model.sdf").write_text(stub)


# --------------------------------------------------------------------------

def load_assets() -> list[Asset]:
    with MANIFEST.open() as fh:
        rows = list(csv.DictReader(r for r in fh if not r.startswith("#")))
    assets = []
    for i, r in enumerate(rows):
        a = Asset(
            rack_model=r["rack_model"], tag_name=r["tag_name"],
            epc=r["epc"], equipment=r["equipment"],
            u_start=r["u_start"], u_end=r["u_end"], u_size=r["u_size"],
            depth_m=r["depth_m"], z_local_m=r["z_local_m"],
            x_local_m=r["x_local_m"],
        )
        a.code = f"{FIRST_ID + i:07X}"
        assets.append(a)
    return assets


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--verify", action="store_true",
                    help="decode the generated PNGs and exit")
    ap.add_argument("--textures-only", action="store_true",
                    help="re-render the PNGs and leave the SDFs and manifest "
                         "alone. Use this when iterating on label appearance "
                         "after the racks have already been migrated — a full "
                         "run would match no rfid_tag_ blocks and report '0 "
                         "labels' for every rack, which looks like a failure "
                         "but is not.")
    args = ap.parse_args()

    assets = load_assets()
    if not assets:
        print("No assets in the manifest.", file=sys.stderr)
        return 1

    check_fit(assets[0].code)
    n_mod = symbol_modules(assets[0].code)
    print(
        f"{len(assets)} assets, codes {assets[0].code}..{assets[-1].code}\n"
        f"Code 128 Subset B, {n_mod} modules incl. quiet zones, {MIL} mil\n"
        f"  symbol {n_mod * MODULE_M * 1000:.2f} mm wide in a "
        f"{PRINT_W * 1000:.0f} mm print area\n"
        f"  raster {PX_PER_MODULE} px/module -> "
        f"{int(round(TAG_W * PX_PER_M))}x{int(round(TAG_H * PX_PER_M))} px "
        f"per label, saved rotated {TEXTURE_ROTATE_DEG} deg for the box UV\n"
        f"  LS2208 depth of field at {MIL} mil: 0.0381-0.2540 m\n"
        f"  scanner roll limit from this label: {roll_limit_deg():.1f} deg\n"
    )

    if args.verify:
        return 1 if verify(assets) else 0

    if not args.dry_run:
        for a in assets:
            render_label(a.code, TEXTURE_DIR / f"{a.code}.png")
        print(f"  wrote {len(assets)} textures to {TEXTURE_DIR}")
    else:
        print(f"  would write {len(assets)} textures to {TEXTURE_DIR}")

    write_model_config(args.dry_run)

    if args.textures_only:
        print("  --textures-only: SDFs and manifest left untouched")
        return 0

    by_tag = {a.tag_name: a for a in assets}
    total = 0
    for n in range(1, 9):
        path = MODELS / f"server_rack_{n}" / "model.sdf"
        if not path.exists():
            print(f"  ! missing {path}")
            continue
        c = rewrite_rack(path, by_tag, args.dry_run)
        total += c
        print(f"  {'would rewrite' if args.dry_run else 'rewrote'} "
              f"{path.parent.name}: {c} labels")
    print(f"  {total} tag blocks replaced (manifest has {len(assets)})")

    write_manifest(assets, args.dry_run)

    if not args.dry_run:
        print(
            "\nOriginals kept as *.sdf.rfid.bak / *.csv.rfid.bak.\n"
            "Next: re-run with --verify, then colcon build and update the\n"
            "world plugin blocks and spot_bridge.yaml."
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
