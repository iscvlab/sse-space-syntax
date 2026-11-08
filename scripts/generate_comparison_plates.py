# -*- coding: utf-8 -*-
"""
Academic 3-Panel Minimalist Collage Generator for SSE v1.1.0.
Produces clean, white-background journal-style comparison figures (depthmapX | SSE | 1:1 Parity)
with high-legibility typography tuned for 2-column GitHub Markdown tables.
"""
import csv
import os
import shutil
import numpy as np
from PIL import Image, ImageDraw, ImageFont

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(SCRIPT_DIR, ".."))
WORKSPACE_ROOT = os.path.abspath(os.path.join(REPO, "..", ".."))
QGIS_LAYERS = os.environ.get(
    "SSE_QGIS_LAYERS_DIR",
    os.path.join(WORKSPACE_ROOT, "QGIS-workspace", "layers"),
)
THESIS_READY = os.environ.get(
    "SSE_THESIS_READY_DIR",
    os.path.join(
        WORKSPACE_ROOT,
        "Thesis-workspace",
        "01_site_analysis",
        "01_macro_scale",
        "05_space_syntax",
        "depthmapx_ready",
    ),
)
SCRATCH = os.environ.get("SSE_BENCH_CSV_DIR", SCRIPT_DIR)
ARTIFACT_DIR = os.environ.get("SSE_ARTIFACT_DIR", "")
BENCH_DOCS = os.path.join(REPO, "docs", "benchmarks")
os.makedirs(BENCH_DOCS, exist_ok=True)

WIN_DIR = os.environ.get("WINDIR", "")
FONT_REG_PATH = os.path.join(WIN_DIR, "Fonts", "segoeui.ttf") if WIN_DIR else ""
FONT_BOLD_PATH = os.path.join(WIN_DIR, "Fonts", "segoeuib.ttf") if WIN_DIR else ""


def get_font(size: int, bold: bool = False):
    p = FONT_BOLD_PATH if bold and os.path.exists(FONT_BOLD_PATH) else FONT_REG_PATH
    if p and os.path.exists(p):
        return ImageFont.truetype(p, size)
    return ImageFont.load_default()


def depthmap_classic_rgb(t: float):
    """Standard UCL depthmapX Blue -> Cyan -> Green -> Yellow -> Red ramp."""
    t = max(0.0, min(1.0, float(t)))
    stops = [
        (0.00, (43, 92, 230)),
        (0.25, (34, 184, 207)),
        (0.50, (64, 192, 87)),
        (0.75, (250, 176, 5)),
        (1.00, (224, 49, 49)),
    ]
    for i in range(len(stops) - 1):
        t0, c0 = stops[i]
        t1, c1 = stops[i + 1]
        if t <= t1:
            u = (t - t0) / (t1 - t0) if t1 > t0 else 0.0
            return (
                int(round(c0[0] + u * (c1[0] - c0[0]))),
                int(round(c0[1] + u * (c1[1] - c0[1]))),
                int(round(c0[2] + u * (c1[2] - c0[2]))),
            )
    return stops[-1][1]


def fmt_num(val: float) -> str:
    av = abs(val)
    if av >= 1_000_000:
        return f"{val / 1_000_000:.2f}M"
    if av >= 10_000:
        return f"{val / 1_000:.1f}k"
    if av >= 100:
        return f"{val:,.0f}"
    return f"{val:,.2f}"


def read_mif_mid_geom_and_df(mif_path: str):
    mid_path = mif_path[:-4] + ".mid"
    cols = []
    segs = []
    pts = []
    in_cols = False
    in_data = False
    with open(mif_path, "r", encoding="utf-8", errors="ignore") as f:
        lines = [ln.strip() for ln in f]
    i = 0
    while i < len(lines):
        ln = lines[i]
        u = ln.upper()
        if u.startswith("COLUMNS"):
            in_cols = True
            i += 1
            continue
        if u == "DATA":
            in_cols = False
            in_data = True
            i += 1
            continue
        if in_cols and ln:
            cols.append(ln.split()[0].strip('"'))
        elif in_data and ln:
            parts = ln.split()
            gu = parts[0].upper()
            if gu == "LINE" and len(parts) >= 5:
                segs.append(
                    (
                        (float(parts[1]), float(parts[2])),
                        (float(parts[3]), float(parts[4])),
                    )
                )
            elif gu == "PLINE":
                cnt = int(parts[1]) if len(parts) > 1 else 0
                c_pts = []
                for _ in range(cnt):
                    i += 1
                    p2 = lines[i].split()
                    c_pts.append((float(p2[0]), float(p2[1])))
                if len(c_pts) >= 2:
                    segs.append((c_pts[0], c_pts[-1]))
            elif gu == "POINT" and len(parts) >= 3:
                pts.append((float(parts[1]), float(parts[2])))
            elif gu == "REGION":
                num_polys = int(parts[1]) if len(parts) > 1 else 1
                c_pts = []
                for _ in range(num_polys):
                    i += 1
                    nv = int(lines[i].strip())
                    for _ in range(nv):
                        i += 1
                        p2 = lines[i].split()
                        c_pts.append((float(p2[0]), float(p2[1])))
                if c_pts:
                    xs = [p[0] for p in c_pts[:-1]] if len(c_pts) > 1 else [c_pts[0][0]]
                    ys = [p[1] for p in c_pts[:-1]] if len(c_pts) > 1 else [c_pts[0][1]]
                    pts.append((sum(xs) / len(xs), sum(ys) / len(ys)))
        i += 1

    data = {c: [] for c in cols}
    with open(mid_path, "r", encoding="utf-8", errors="ignore") as f:
        reader = csv.reader(f)
        for row in reader:
            if not row:
                continue
            for ci, c in enumerate(cols):
                if ci < len(row):
                    v_str = row[ci].strip().strip('"')
                    try:
                        v = float(v_str)
                    except ValueError:
                        v = -1.0
                    data[c].append(v)
    for c in cols:
        data[c] = np.array(data[c], dtype=np.float64)
    return segs, pts, data


def read_csv_df(csv_path: str):
    with open(csv_path, "r", encoding="utf-8") as f:
        reader = csv.reader(f)
        headers = [h.strip() for h in next(reader)]
        cols = {h: [] for h in headers}
        for row in reader:
            if not row:
                continue
            for idx, h in enumerate(headers):
                if idx < len(row):
                    try:
                        cols[h].append(float(row[idx]))
                    except ValueError:
                        cols[h].append(-1.0)
    for h in headers:
        cols[h] = np.array(cols[h], dtype=np.float64)
    return cols


def draw_colorbar_strip(
    img: Image.Image,
    draw: ImageDraw.ImageDraw,
    x0: int,
    y0: int,
    w: int,
    h: int,
    vmin: float,
    vmax: float,
    metric_label: str,
):
    bar_x0 = x0 + 95
    bar_x1 = x0 + w - 95
    bar_w = max(80, bar_x1 - bar_x0)
    for px in range(bar_w):
        rgb = depthmap_classic_rgb(px / max(1, bar_w - 1))
        draw.line((bar_x0 + px, y0, bar_x0 + px, y0 + h), fill=rgb)
    draw.rectangle((bar_x0, y0, bar_x1, y0 + h), outline=(148, 163, 184), width=1)
    f_lbl = get_font(16, False)
    f_bold = get_font(17, True)
    draw.text(
        (bar_x0 - 8, y0 - 4),
        fmt_num(vmin),
        fill=(51, 65, 85),
        font=f_lbl,
        anchor="ra",
    )
    draw.text(
        (bar_x1 + 8, y0 - 4),
        fmt_num(vmax),
        fill=(51, 65, 85),
        font=f_lbl,
        anchor="la",
    )
    draw.text(
        ((bar_x0 + bar_x1) // 2, y0 + h + 5),
        metric_label,
        fill=(15, 23, 42),
        font=f_bold,
        anchor="ma",
    )


def draw_academic_line_panel(
    img: Image.Image,
    draw: ImageDraw.ImageDraw,
    rect,
    segs,
    vals,
    sub_label: str,
    metric_label: str,
    lw_px: int = 2,
):
    x0, y0, x1, y1 = rect
    draw.rectangle(rect, fill=(255, 255, 255), outline=(148, 163, 184), width=1)
    draw.text((x0 + 14, y0 + 9), sub_label, fill=(15, 23, 42), font=get_font(21, True))
    draw.line((x0, y0 + 44, x1, y0 + 44), fill=(203, 213, 225), width=1)

    pad = 18
    mx0, my0, mx1, my1 = x0 + pad, y0 + 54, x1 - pad, y1 - 54
    mw, mh = mx1 - mx0, my1 - my0

    xs = [p[0] for s in segs for p in s]
    ys = [p[1] for s in segs for p in s]
    min_x, max_x = min(xs), max(xs)
    min_y, max_y = min(ys), max(ys)
    dx = max(max_x - min_x, 1e-6)
    dy = max(max_y - min_y, 1e-6)
    scale = min(mw / dx, mh / dy)
    ox = mx0 + (mw - dx * scale) * 0.5
    oy = my0 + (mh - dy * scale) * 0.5

    valid_mask = np.isfinite(vals) & (vals > -0.5)
    v_valid = vals[valid_mask] if np.any(valid_mask) else np.array([0.0, 1.0])
    vmin = float(np.percentile(v_valid, 2))
    vmax = float(np.percentile(v_valid, 98))
    if vmax <= vmin:
        vmax = vmin + 1.0

    order = np.argsort(vals)
    for idx in order:
        if idx >= len(segs):
            continue
        p1, p2 = segs[idx]
        v = vals[idx]
        px1 = int(round(ox + (p1[0] - min_x) * scale))
        py1 = int(round(oy + (max_y - p1[1]) * scale))
        px2 = int(round(ox + (p2[0] - min_x) * scale))
        py2 = int(round(oy + (max_y - p2[1]) * scale))
        if not np.isfinite(v) or v <= -0.5:
            col = (203, 213, 225)
        else:
            t = (float(v) - vmin) / (vmax - vmin)
            col = depthmap_classic_rgb(t)
        draw.line((px1, py1, px2, py2), fill=col, width=lw_px)

    draw_colorbar_strip(
        img,
        draw,
        x0 + 10,
        y1 - 40,
        (x1 - x0) - 20,
        11,
        vmin,
        vmax,
        metric_label,
    )


def draw_academic_vga_panel(
    img: Image.Image,
    draw: ImageDraw.ImageDraw,
    rect,
    pts_x,
    pts_y,
    vals,
    wall_segs,
    sub_label: str,
    metric_label: str,
):
    x0, y0, x1, y1 = rect
    draw.rectangle(rect, fill=(255, 255, 255), outline=(148, 163, 184), width=1)
    draw.text((x0 + 14, y0 + 9), sub_label, fill=(15, 23, 42), font=get_font(21, True))
    draw.line((x0, y0 + 44, x1, y0 + 44), fill=(203, 213, 225), width=1)

    pad = 20
    mx0, my0, mx1, my1 = x0 + pad, y0 + 54, x1 - pad, y1 - 54
    mw, mh = mx1 - mx0, my1 - my0

    min_x, max_x = float(np.min(pts_x)) - 1.0, float(np.max(pts_x)) + 1.0
    min_y, max_y = float(np.min(pts_y)) - 1.0, float(np.max(pts_y)) + 1.0
    dx = max(max_x - min_x, 1e-6)
    dy = max(max_y - min_y, 1e-6)
    scale = min(mw / dx, mh / dy)
    ox = mx0 + (mw - dx * scale) * 0.5
    oy = my0 + (mh - dy * scale) * 0.5

    valid_mask = np.isfinite(vals) & (vals > -0.5)
    v_valid = vals[valid_mask] if np.any(valid_mask) else np.array([0.0, 1.0])
    vmin = float(np.percentile(v_valid, 2))
    vmax = float(np.percentile(v_valid, 98))
    if vmax <= vmin:
        vmax = vmin + 1.0

    r_cell = max(2, int(round(scale * 0.46)))
    for i in range(len(pts_x)):
        v = vals[i]
        cx = int(round(ox + (pts_x[i] - min_x) * scale))
        cy = int(round(oy + (max_y - pts_y[i]) * scale))
        col = (
            depthmap_classic_rgb((float(v) - vmin) / (vmax - vmin))
            if (np.isfinite(v) and v > -0.5)
            else (226, 232, 240)
        )
        draw.rectangle(
            (cx - r_cell, cy - r_cell, cx + r_cell, cy + r_cell),
            fill=col,
        )

    for p1, p2 in wall_segs:
        px1 = int(round(ox + (p1[0] - min_x) * scale))
        py1 = int(round(oy + (max_y - p1[1]) * scale))
        px2 = int(round(ox + (p2[0] - min_x) * scale))
        py2 = int(round(oy + (max_y - p2[1]) * scale))
        draw.line((px1, py1, px2, py2), fill=(15, 23, 42), width=2)

    draw_colorbar_strip(
        img,
        draw,
        x0 + 10,
        y1 - 40,
        (x1 - x0) - 20,
        11,
        vmin,
        vmax,
        metric_label,
    )


def draw_academic_scatter_panel(
    img: Image.Image,
    draw: ImageDraw.ImageDraw,
    rect,
    x_vals,
    y_vals,
    sub_label: str,
    x_axis_label: str,
    y_axis_label: str,
):
    x0, y0, x1, y1 = rect
    draw.rectangle(rect, fill=(255, 255, 255), outline=(148, 163, 184), width=1)
    draw.text((x0 + 14, y0 + 9), sub_label, fill=(15, 23, 42), font=get_font(21, True))
    draw.line((x0, y0 + 44, x1, y0 + 44), fill=(203, 213, 225), width=1)

    mask = np.isfinite(x_vals) & np.isfinite(y_vals) & (x_vals > -0.5) & (y_vals > -0.5)
    xv = x_vals[mask]
    yv = y_vals[mask]
    r_val = float(np.corrcoef(xv, yv)[0, 1]) if len(xv) > 2 else 1.0
    r2_val = r_val * r_val
    max_err = float(np.max(np.abs(xv - yv))) if len(xv) > 0 else 0.0

    px0, py0, px1, py1 = x0 + 78, y0 + 62, x1 - 24, y1 - 54
    pw, ph = px1 - px0, py1 - py0

    for k in range(5):
        gy = int(round(py0 + k * ph / 4.0))
        gx = int(round(px0 + k * pw / 4.0))
        draw.line((px0, gy, px1, gy), fill=(241, 245, 249), width=1)
        draw.line((gx, py0, gx, py1), fill=(241, 245, 249), width=1)

    draw.rectangle((px0, py0, px1, py1), outline=(100, 116, 139), width=1)
    draw.line((px0, py1, px1, py0), fill=(185, 28, 28), width=3)

    vmin = float(min(np.min(xv), np.min(yv)))
    vmax = float(max(np.max(xv), np.max(yv)))
    span = max(vmax - vmin, 1e-9)

    step = max(1, len(xv) // 2500)
    for i in range(0, len(xv), step):
        ux = (float(xv[i]) - vmin) / span
        uy = (float(yv[i]) - vmin) / span
        sx = int(round(px0 + ux * pw))
        sy = int(round(py1 - uy * ph))
        draw.ellipse((sx - 3, sy - 3, sx + 3, sy + 3), fill=(30, 64, 175))

    ib_x0, ib_y0 = px0 + 14, py0 + 14
    draw.rectangle(
        (ib_x0, ib_y0, ib_x0 + 276, ib_y0 + 82),
        fill=(255, 255, 255),
        outline=(148, 163, 184),
        width=1,
    )
    draw.text(
        (ib_x0 + 10, ib_y0 + 7),
        f"R² = {r2_val:.6f} (n = {len(xv):,})",
        fill=(15, 23, 42),
        font=get_font(17, True),
    )
    draw.text(
        (ib_x0 + 10, ib_y0 + 31),
        f"Pearson r = {r_val:.6f}",
        fill=(51, 65, 85),
        font=get_font(16, False),
    )
    draw.text(
        (ib_x0 + 10, ib_y0 + 54),
        f"y = x (max|Δ| = {max_err:.4f})",
        fill=(185, 28, 28),
        font=get_font(16, True),
    )

    f_tick = get_font(15, False)
    draw.text((px0, py1 + 6), fmt_num(vmin), fill=(51, 65, 85), font=f_tick)
    draw.text(
        (px1, py1 + 6), fmt_num(vmax), fill=(51, 65, 85), font=f_tick, anchor="ra"
    )
    draw.text(
        ((px0 + px1) // 2, y1 - 27),
        x_axis_label,
        fill=(15, 23, 42),
        font=get_font(17, True),
        anchor="ma",
    )
    draw.text(
        (px0 - 8, py0 - 4),
        fmt_num(vmax),
        fill=(51, 65, 85),
        font=f_tick,
        anchor="ra",
    )
    draw.text(
        (px0 - 8, py1 - 16),
        fmt_num(vmin),
        fill=(51, 65, 85),
        font=f_tick,
        anchor="ra",
    )
    draw.text(
        (x0 + 12, (py0 + py1) // 2 - 10),
        y_axis_label,
        fill=(15, 23, 42),
        font=get_font(17, True),
    )


def save_both(img: Image.Image, filename: str):
    p_doc = os.path.join(BENCH_DOCS, filename)
    img.save(p_doc, optimize=True)
    if ARTIFACT_DIR and os.path.exists(ARTIFACT_DIR):
        shutil.copy2(p_doc, os.path.join(ARTIFACT_DIR, filename))
    print(f"Saved: {p_doc} ({img.size[0]}x{img.size[1]})")


def main():
    W, H = 1860, 620
    p1_rect = (10, 10, 620, 610)
    p2_rect = (630, 10, 1240, 610)
    p3_rect = (1250, 10, 1850, 610)

    # ==============================================================================
    # FIGURE 1: UCL BARNSBURY SEGMENT ASA (Tulip-1024, N = 178)
    # ==============================================================================
    barns_mif = os.path.join(
        REPO, "examples", "ucl_depthmapx", "barnsbury_segment_lines.mif"
    )
    barns_segs, _, ref_barns = read_mif_mid_geom_and_df(barns_mif)
    barns_csv = os.path.join(SCRATCH, "sse_barnsbury_asa.csv")
    if not os.path.exists(barns_csv):
        barns_csv = os.path.join(SCRATCH, "sse_barnsbury.csv")
    sse_barns = read_csv_df(barns_csv)

    img1 = Image.new("RGB", (W, H), (255, 255, 255))
    d1 = ImageDraw.Draw(img1)
    draw_academic_line_panel(
        img1,
        d1,
        p1_rect,
        barns_segs,
        ref_barns["T1024_Choice_Segment_Length_Wgt_"],
        "(a) UCL depthmapX v0.8.0 (48.0 ms)",
        "T1024 Choice [Seg Length Wgt] Rn",
        lw_px=3,
    )
    draw_academic_line_panel(
        img1,
        d1,
        p2_rect,
        barns_segs,
        sse_barns["Choice_Rn"],
        "(b) SSE v1.1.0 (0.83 ms | 57.8× Faster)",
        "T1024 Choice [Seg Length Wgt] Rn",
        lw_px=3,
    )
    draw_academic_scatter_panel(
        img1,
        d1,
        p3_rect,
        ref_barns["T1024_Choice_Segment_Length_Wgt_"],
        sse_barns["Choice_Rn"],
        "(c) 1:1 Parity (Barnsbury N = 178)",
        "UCL depthmapX v0.8.0",
        "SSE",
    )
    save_both(img1, "comparison_01_asa_tulip1024_barnsbury_onnut.png")

    # ==============================================================================
    # FIGURE 2: BANGKOK ON NUT MULTI-RADIUS SEGMENT ASA (Tulip-1024, N = 30,493)
    # ==============================================================================
    onnut_asa_mif = os.path.join(
        THESIS_READY, "03_onnut_walkable_meso", "onnut_walkable_meso_analyzed.mif"
    )
    onnut_segs, _, ref_onnut_asa = read_mif_mid_geom_and_df(onnut_asa_mif)
    sse_onnut_asa = read_csv_df(os.path.join(SCRATCH, "sse_onnut_asa.csv"))

    img2 = Image.new("RGB", (W, H), (255, 255, 255))
    d2 = ImageDraw.Draw(img2)
    draw_academic_line_panel(
        img2,
        d2,
        p1_rect,
        onnut_segs,
        ref_onnut_asa["T1024_Integration"],
        "(a) UCL depthmapX v0.8.0 (870.0 s)",
        "T1024 Integration Rn (N = 30,493)",
        lw_px=1,
    )
    draw_academic_line_panel(
        img2,
        d2,
        p2_rect,
        onnut_segs,
        sse_onnut_asa["T1024_Integration_Rn"],
        "(b) SSE v1.1.0 (15.98 s | 54.4× Faster)",
        "T1024 Integration Rn (N = 30,493)",
        lw_px=1,
    )
    draw_academic_scatter_panel(
        img2,
        d2,
        p3_rect,
        ref_onnut_asa["T1024_Integration"],
        sse_onnut_asa["T1024_Integration_Rn"],
        "(c) 1:1 Parity (On Nut ASA N = 30,493)",
        "UCL depthmapX v0.8.0",
        "SSE",
    )
    save_both(img2, "comparison_02_axial_onnut_30493.png")

    # ==============================================================================
    # FIGURE 3: BANGKOK ON NUT TOPOLOGICAL AXIAL ANALYSIS (Hillier D_k, N = 30,493)
    # ==============================================================================
    ax_mif = os.path.join(QGIS_LAYERS, "depthmapx_onnut_meso_axial_baseline.mif")
    ax_segs, _, ref_ax = read_mif_mid_geom_and_df(ax_mif)
    sse_ax = read_csv_df(os.path.join(SCRATCH, "sse_onnut_axial.csv"))

    img3 = Image.new("RGB", (W, H), (255, 255, 255))
    d3 = ImageDraw.Draw(img3)
    draw_academic_line_panel(
        img3,
        d3,
        p1_rect,
        ax_segs,
        ref_ax["Integration_HH_"],
        "(a) UCL depthmapX v0.8.0 (48.50 s)",
        "Axial Integration [HH] Rn (N = 30,493)",
        lw_px=1,
    )
    draw_academic_line_panel(
        img3,
        d3,
        p2_rect,
        ax_segs,
        sse_ax["Integration_HH_n"],
        "(b) SSE v1.1.0 (1.39 s | 34.9× Faster)",
        "Axial Integration [HH] Rn (N = 30,493)",
        lw_px=1,
    )
    draw_academic_scatter_panel(
        img3,
        d3,
        p3_rect,
        ref_ax["Integration_HH_"],
        sse_ax["Integration_HH_n"],
        "(c) 1:1 Parity (On Nut Axial N = 30,493)",
        "UCL depthmapX v0.8.0",
        "SSE",
    )
    save_both(img3, "comparison_03_vga_isovists_onnut_gallery.png")

    # ==============================================================================
    # FIGURE 4: ARCHITECTURAL FLOORPLAN VGA & 2D ISOVISTS (On Nut VGA, N = 2,101)
    # ==============================================================================
    vga_in_mif = os.path.join(QGIS_LAYERS, "onnut_vga_layout_input.mif")
    vga_walls, _, _ = read_mif_mid_geom_and_df(vga_in_mif)
    vga_ref_mif = os.path.join(QGIS_LAYERS, "depthmapx_onnut_vga_baseline.mif")
    _, ref_vga_pts, ref_vga = read_mif_mid_geom_and_df(vga_ref_mif)
    sse_vga = read_csv_df(os.path.join(SCRATCH, "sse_onnut_vga.csv"))

    ref_pt_map = {
        (round(pt[0], 2), round(pt[1], 2)): i for i, pt in enumerate(ref_vga_pts)
    }
    sse_vga_aligned = {k: np.zeros(len(ref_vga_pts)) for k in sse_vga.keys()}
    for row_i in range(len(sse_vga["x"])):
        key = (round(sse_vga["x"][row_i], 2), round(sse_vga["y"][row_i], 2))
        if key in ref_pt_map:
            idx = ref_pt_map[key]
            for k in sse_vga.keys():
                sse_vga_aligned[k][idx] = sse_vga[k][row_i]

    pts_x = np.array([p[0] for p in ref_vga_pts])
    pts_y = np.array([p[1] for p in ref_vga_pts])

    img4 = Image.new("RGB", (W, H), (255, 255, 255))
    d4 = ImageDraw.Draw(img4)
    draw_academic_vga_panel(
        img4,
        d4,
        p1_rect,
        pts_x,
        pts_y,
        ref_vga["Visual_Integration_HH_"],
        vga_walls,
        "(a) UCL depthmapX v0.8.0 (4,800 ms)",
        "Visual Integration [HH] (N = 2,101)",
    )
    draw_academic_vga_panel(
        img4,
        d4,
        p2_rect,
        pts_x,
        pts_y,
        sse_vga_aligned["Visual_Integration_HH"],
        vga_walls,
        "(b) SSE v1.1.0 (134.1 ms | 35.8× Faster)",
        "Visual Integration [HH] (N = 2,101)",
    )
    draw_academic_scatter_panel(
        img4,
        d4,
        p3_rect,
        ref_vga["Visual_Integration_HH_"],
        sse_vga_aligned["Visual_Integration_HH"],
        "(c) 1:1 Parity (On Nut VGA N = 2,101)",
        "UCL depthmapX v0.8.0",
        "SSE",
    )
    save_both(img4, "comparison_04_benchmark_summary_and_agents.png")
    print("ALL 4 HIGH-LEGIBILITY ACADEMIC COLLAGE PLATES GENERATED SUCCESSFULLY.")


if __name__ == "__main__":
    main()
