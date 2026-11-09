use std::env;
use std::path::{Path, PathBuf};
use std::process::Command;

fn find_windows_rc() -> Option<PathBuf> {
    if let Ok(output) = Command::new("where").arg("rc.exe").output() {
        if output.status.success() {
            if let Some(first_line) = String::from_utf8_lossy(&output.stdout).lines().next() {
                let p = PathBuf::from(first_line.trim());
                if p.exists() {
                    return Some(p);
                }
            }
        }
    }

    let kits_root = Path::new(r"C:\Program Files (x86)\Windows Kits\10\bin");
    if let Ok(entries) = std::fs::read_dir(kits_root) {
        let mut versions: Vec<PathBuf> = entries.filter_map(|e| e.ok().map(|e| e.path())).collect();
        versions.sort();
        versions.reverse();
        for ver_dir in versions {
            let candidate = ver_dir.join("x64").join("rc.exe");
            if candidate.exists() {
                return Some(candidate);
            }
        }
    }
    None
}

fn main() {
    println!("cargo:rerun-if-changed=assets/sse_icon.ico");
    println!("cargo:rerun-if-changed=assets/sse_icon.rc");
    println!("cargo:rerun-if-changed=sse/plugins/qgis_plugin/icon.png");

    if env::var("CARGO_CFG_TARGET_OS").unwrap_or_default() != "windows" {
        return;
    }

    let manifest_dir = PathBuf::from(env::var("CARGO_MANIFEST_DIR").unwrap_or_default());
    let rc_file = manifest_dir.join("assets").join("sse_icon.rc");
    let ico_file = manifest_dir.join("assets").join("sse_icon.ico");
    if !rc_file.exists() || !ico_file.exists() {
        return;
    }

    let out_dir = PathBuf::from(env::var("OUT_DIR").unwrap_or_default());
    let res_file = out_dir.join("sse_icon.res");

    if let Some(rc_bin) = find_windows_rc() {
        let status = Command::new(rc_bin)
            .arg("/nologo")
            .arg("/fo")
            .arg(&res_file)
            .arg(&rc_file)
            .status();
        if let Ok(st) = status {
            if st.success() && res_file.exists() {
                println!("cargo:rustc-link-arg-bins={}", res_file.display());
            }
        }
    }
}
