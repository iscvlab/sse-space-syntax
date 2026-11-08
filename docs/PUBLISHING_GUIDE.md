# SSE — Space Syntax Engine (`v1.1.0`) Publishing & Release Guide

> **SSE — Space Syntax Engine (`v1.1.0`)**  
> Author: Napat (Tan) Phasundhiae (ณภัทร ฝาสันเทียะ) | Computational Design Technologist  
> Target Platforms: GitHub (`napattan/sse-space-syntax`), Official QGIS Python Plugins Repository (`plugins.qgis.org`), PyPI (`pip install osse-space-syntax`), Food4Rhino (Grasshopper)

---

## 📋 Pre-Flight Checklist (`v1.1.0` Verified Green)

Before publishing to any public registry, verify all 5 automated release gates:

- [x] **Rust Core & CLI Verification**: `cargo test --release` (`15/15` passing) and `.\sse-cli.exe benchmark` (`9/9` suites `PASS`).
- [x] **Python & QGIS Engine Unit Tests**: `python -m pytest tests/test_osse_engine.py -v` (`19/19` passing, including `test_depthmapx_tulip_1024_barnsbury_exact_parity` with `R² = 1.00000000` and `max|Δ| = 0.0000`).
- [x] **QGIS 3.22–4.x (`Qt6 Ready`) Scanner**: `check_qt6.py` (`100% COMPLIANT`, `supportsQt6=True`, `qgisMaximumVersion=4.99`).
- [x] **Deterministic Single-Root Zip Packaging**: `package_qgis.py` producing `dist/sse_v1.1.0.zip` (single root `sse/`, `icon.png` `256×256` RGBA, 5 street network algorithms).
- [x] **Pre-Flight Security & Path Audit**: `audit_engine.py --platform qgis` and `audit_engine.py --platform github` (`0 Blocking / 0 Advisory`, zero hardcoded machine paths).

---

## 1. 🐙 GitHub Repository Setup, Push & `v1.1.0` Binary Release

Follow these steps to publish the repository and standalone Windows binaries (`sse-desktop.exe`, `sse-cli.exe`, `sse-mcp.exe`) to your GitHub profile (`https://github.com/napattan/sse-space-syntax`):

### Step 1.1: Create Remote Repository on GitHub
1. Open your browser and navigate to: `https://github.com/new` (or use `gh repo create`).
2. Repository name: `sse-space-syntax`
3. Description: `SSE — Space Syntax Engine (v1.1.0): Dual-Engine Native Rust Workstation (sse-desktop.exe / sse-cli.exe / sse-mcp.exe) & QGIS 3.22-4.x (Qt6 Ready) Processing Plugin with Exact UCL depthmapX Tulip-1024 Parity`
4. Visibility: **Public**
5. Do **NOT** initialize with README, `.gitignore`, or license (these already exist locally).

### Step 1.2: Initialize, Commit, Tag `v1.1.0` & Push from Local Directory

Open PowerShell in the `osse-space-syntax` repository root:

```powershell
# Stage all source, documentation, and benchmark plate files
git add .

# Create the v1.1.0 release commit
git commit -m "feat(v1.1.0): release SSE Space Syntax Engine dual-engine Rust workstation & QGIS Qt6 plugin"

# Create annotated v1.1.0 release tag
git tag -a v1.1.0 -m "SSE — Space Syntax Engine v1.1.0 (Exact depthmapX Tulip-1024 Parity)"

# Link remote GitHub origin (if not already linked) and push main + tags
git branch -M main
git remote add origin https://github.com/napattan/sse-space-syntax.git
git push -u origin main --tags
```

### Step 1.3: Publish GitHub Release `v1.1.0` with Standalone Binaries & QGIS Zip
Using GitHub CLI (`gh`) or the GitHub web UI (`https://github.com/napattan/sse-space-syntax/releases/new`):

```powershell
gh release create v1.1.0 `
  sse-desktop.exe `
  sse-cli.exe `
  sse-mcp.exe `
  dist/sse_v1.1.0.zip `
  --title "SSE — Space Syntax Engine v1.1.0 (Rust Workstation + CLI + MCP + QGIS Qt6 Plugin)" `
  --notes "Dual-engine Space Syntax release featuring 100% bit-exact UCL depthmapX v0.8.0 Directed-Segment Tulip-1024 parity (R² = 1.00000000, max|Δ| = 0.0000), 34.9×–62.3× multi-core speedup, 5-Step P90 Adaptive Watertight OSM Cleaner, 16-Attribute Hillier Axial Analysis, Analytical 2D Benedikt Isovists, Gibson Ecological Visual Agents, and 9-tool JSON-RPC 2.0 MCP server."
```

---

## 2. 🗺️ Official QGIS Plugin Repository Release (`plugins.qgis.org`)

The 5-algorithm QGIS 3.22–4.x (`Qt5`/`Qt6 Ready`) Street Network Processing plugin is located at `osse/plugins/qgis_plugin/` and packaged into `dist/sse_v1.1.0.zip`.

### Step 2.1: Plugin Archive Structure (`sse/`)
The official QGIS Plugin Repository requires a single-root `.zip` archive (`sse/`) containing:
```text
sse/
├── __init__.py
├── metadata.txt
├── plugin.py
├── provider.py
├── dialog_cleaner.py
├── dialog_scatter.py
├── icon.png
├── LICENSE
├── README.md
├── algorithms/
├── core/
├── io_handlers/
└── styles/
```

### Step 2.2: Build & Audit Deterministic Single-Root Release Archive
Run the `/xpublish` packaging and audit scripts from the repository root:
```powershell
python "$env:USERPROFILE\.gemini\config\skills\xpublish\scripts\check_qt6.py" "osse/plugins/qgis_plugin"
python "$env:USERPROFILE\.gemini\config\skills\xpublish\scripts\package_qgis.py" "osse/plugins/qgis_plugin" --output-dir "dist" --package-name "sse"
python "$env:USERPROFILE\.gemini\config\skills\xpublish\scripts\audit_engine.py" "osse/plugins/qgis_plugin" --platform qgis
```
This outputs `dist/sse_v1.1.0.zip` (single-root `sse/`), verified against all Qt6, Flake8, Security, and Single-Root gates.

### Step 2.3: Upload to `plugins.qgis.org`
1. Navigate to: [https://plugins.qgis.org/plugins/add/](https://plugins.qgis.org/plugins/add/)
2. Sign in with your OSGeo ID account.
3. Click **Upload a plugin** and select `dist/sse_v1.1.0.zip`.
4. All required fields are automatically populated from `sse/metadata.txt`:
   - **Name**: `SSE — Space Syntax Engine`
   - **Version**: `1.1.0`
   - **Minimum QGIS Version**: `3.22`
   - **Maximum QGIS Version**: `4.99` (`supportsQt6=True`)
   - **Experimental**: `False`
   - **Tracker / Repository**: `https://github.com/napattan/sse-space-syntax`

---

## 3. 🐍 PyPI Package Distribution (`pip install osse-space-syntax`)

Publishing to PyPI allows researchers and practitioners to install the Python API and CLI via `pip install osse-space-syntax[fast]`.

```powershell
# Install build & upload tools
python -m pip install --upgrade build twine

# Build sdist and wheel into dist/
python -m build

# Upload to official PyPI
python -m twine upload dist/osse_space_syntax*
```

---

## 4. 🦏 Food4Rhino & Yak Package Release (Grasshopper C#)

The multi-threaded Grasshopper C# solver is located at `osse/plugins/grasshopper_csharp/SpaceSyntaxComponent.cs`.
1. Compile `SpaceSyntaxComponent.cs` against `RhinoCommon.dll` and `Grasshopper.dll` (`.NET 7.0` for Rhino 8 / `.NET Framework 4.8` for Rhino 7).
2. Package via `yak build` and publish to Food4Rhino / McNeel Yak Package Server.
