#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
build_armcc.py — 用本机 Keil(ARMCC 5.06u5 + armlink + fromelf) 直接编译本工程。
不依赖 uVision GUI，只把 .uvprojx 当作"文件清单 + 宏 + 包含路径"的来源。

用法:
    python tools/build_armcc.py           # 编译 + 链接 + 出 hex
    python tools/build_armcc.py --clean   # 先清 OBJ_armcc

产物: OBJ_armcc/Template.axf / Template.hex
退出码: 0 成功; 1 有错误
"""
import os
import re
import subprocess
import sys
import xml.etree.ElementTree as ET

try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
except Exception:
    pass

KEIL = r"C:\stm32&c51\KeiluVision MDK"
BIN = os.path.join(KEIL, r"ARM\ARMCC\bin")
ARMCC = os.path.join(BIN, "armcc.exe")
ARMASM = os.path.join(BIN, "armasm.exe")
ARMLINK = os.path.join(BIN, "armlink.exe")
FROMELF = os.path.join(BIN, "fromelf.exe")

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PROJ = os.path.join(ROOT, "USER", "Template.uvprojx")
OUT = os.path.join(ROOT, "OBJ_armcc")

SCATTER = """LR_IROM1 0x08000000 0x00040000  {
  ER_IROM1 0x08000000 0x00040000  {
   *.o (RESET, +First)
   *(InRoot$$Sections)
   .ANY (+RO)
   .ANY (+XO)
  }
  RW_IRAM1 0x20000000 0x0000C000  {
   .ANY (+RW +ZI)
  }
}
"""


def parse_project():
    text = open(PROJ, "rb").read().decode("utf-8-sig")

    def grab(tag):
        m = re.search(r"<%s>(.*?)</%s>" % (tag, tag), text, re.S)
        return m.group(1).strip() if m else ""

    defines = [d.strip() for d in grab("Define").split(",") if d.strip()]

    raw_paths = ""
    for m in re.finditer(r"<IncludePath>(.*?)</IncludePath>", text, re.S):
        if m.group(1).strip():          # 文档里有多个 IncludePath，取第一个非空的
            raw_paths = m.group(1).strip()
            break
    includes = [p.strip() for p in raw_paths.split(";") if p.strip()]

    outname = grab("OutputName") or "Template"
    cpu = "Cortex-M3"

    files = []
    for node in ET.fromstring(text.encode("utf-8")).iter("File"):
        path = node.findtext("FilePath")
        ftype = (node.findtext("FileType") or "").strip()
        if not path or ftype not in ("1", "2"):
            continue
        full = os.path.normpath(os.path.join(ROOT, "USER", path.replace("\\", os.sep)))
        files.append((full, ftype))
    return defines, includes, outname, cpu, files


def run(cmd, what, show_output=False):
    p = subprocess.run(cmd, cwd=ROOT, capture_output=True, text=True,
                       encoding="utf-8", errors="replace")
    out = (p.stdout or "") + (p.stderr or "")
    ok = p.returncode == 0
    if not ok:
        print("FAIL %s" % what)
        print(out.rstrip())
    elif show_output:
        print(out.rstrip())
    return ok


def main():
    if "--clean" in sys.argv and os.path.isdir(OUT):
        for n in os.listdir(OUT):
            os.remove(os.path.join(OUT, n))
    os.makedirs(OUT, exist_ok=True)

    defines, includes, outname, cpu, files = parse_project()
    if not files:
        print("no source files parsed")
        return 1

    base = ["--cpu", cpu, "--apcs=interwork", "-O0", "-g", "--c90"]
    base += ["-D" + d for d in defines]
    base += ["-I" + os.path.normpath(os.path.join(ROOT, "USER", inc.replace("\\", os.sep)))
             for inc in includes]

    objs, errors, warnings = [], 0, 0
    for full, ftype in files:
        if not os.path.exists(full):
            print("MISSING", full)
            errors += 1
            continue
        obj = os.path.join(OUT, os.path.splitext(os.path.basename(full))[0] + ".o")
        rel = os.path.relpath(full, ROOT)
        cmd = ([ARMASM, "--cpu", cpu, "-g", "-o", obj, full] if ftype == "2"
               else [ARMCC] + base + ["-c", "-o", obj, full])
        p = subprocess.run(cmd, cwd=ROOT, capture_output=True, text=True,
                           encoding="utf-8", errors="replace")
        out = (p.stdout or "") + (p.stderr or "")
        if p.returncode != 0:
            print("FAIL %s" % rel)
            print(out.rstrip())
            errors += 1
        else:
            objs.append(obj)
            n = out.lower().count("warning")
            if n:
                warnings += n
                print("WARN %s: %d" % (rel, n))

    if errors:
        print("\n%d file(s) failed" % errors)
        return 1
    print("compiled %d object(s), %d warning(s)" % (len(objs), warnings))

    axf = os.path.join(OUT, outname + ".axf")
    sct = os.path.join(OUT, outname + ".sct")
    with open(sct, "w", encoding="ascii", newline="\n") as fh:
        fh.write(SCATTER)

    if not run([ARMLINK, "--cpu", cpu, "--scatter", sct, "--strict", "-o", axf] + objs, "link"):
        return 1

    hexf = os.path.join(OUT, outname + ".hex")
    run([FROMELF, "--i32", "-o", hexf, axf], "hex")
    run([FROMELF, "--text", "-z", axf], "size", show_output=True)

    with open(hexf, "r", encoding="ascii") as fh:
        first = fh.readline().strip()
        second = fh.readline().strip()
    print("\nhex head:", first, "|", second[:24])
    print("BUILD OK ->", axf)
    print("HEX      ->", hexf)
    return 0


if __name__ == "__main__":
    sys.exit(main())
