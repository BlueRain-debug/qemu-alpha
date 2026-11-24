#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-or-later
"""Select native SDM845 PEP/ABD/SMMU ASL for opt-in local diagnostics.

Requires an already extracted, disassembled native DSDT and ACPICA iasl.
No vendor firmware or AML is included in the source tree. This table is
incomplete: GPU, CPU power namespaces and firmware shared memory are not
provided by the extraction step.
"""

import argparse
import hashlib
import json
from pathlib import Path
import re
import subprocess


def extract(text):
    """Preserve byte positions while ignoring comments and quoted braces."""
    mask = re.sub(r'//[^\n]*|/\*[\s\S]*?\*/|"(?:\\.|[^"\\])*"',
                  lambda match: " " * len(match[0]), text)

    def block(at):
        start = mask.index("{", at)
        depth = 0
        for pos in range(start, len(mask)):
            if mask[pos] == "{":
                depth += 1
            elif mask[pos] == "}":
                depth -= 1
                if depth == 0:
                    return text[at:pos + 1]
        raise ValueError(f"Unterminated ASL block at {at}")

    devices = []
    for name in ("ABD", "PEP0", "MMU0", "MMU1"):
        matches = list(re.finditer(r"\bDevice\s*\(" + name + r"\)", mask))
        if len(matches) != 1:
            raise ValueError(f"Expected exactly one Device ({name})")
        devices.append(block(matches[0].start()))
    scopes = [block(match.start()) for match in
              re.finditer(r"\bScope\s*\(\\_SB\.PEP0\)", mask)]
    if not scopes:
        raise ValueError("Native PEP0 scopes were not found")
    header = r'''// Generated for local diagnostic use; not a standalone native DSDT.
DefinitionBlock ("", "SSDT", 2, "QEMU", "OSK845PW", 1)
{
    External (\_SB.PSUB, StrObj)
    External (\_SB.SIDV, IntObj)
    External (\_SB.AGR0, DeviceObj)
    External (\_SB.AGR0._PUR, PkgObj)
    Scope (\_SB)
    {
'''
    return header + "\n".join(devices) + "\n}\n" + "\n".join(scopes) + "\n}\n", len(scopes)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("dsdt", type=Path, help="Disassembled native dsdt.dsl")
    parser.add_argument("--output-prefix", type=Path, required=True)
    parser.add_argument("--iasl", default="iasl")
    args = parser.parse_args()
    source = args.dsdt.resolve(strict=True)
    content, scopes = extract(source.read_text(encoding="utf-8-sig"))
    prefix = args.output_prefix.resolve()
    prefix.parent.mkdir(parents=True, exist_ok=True)
    asl = Path(str(prefix) + ".asl")
    asl.write_text(content, encoding="utf-8", newline="\n")
    result = subprocess.run([args.iasl, "-p", str(prefix), str(asl)],
                            capture_output=True, text=True, check=False)
    Path(str(prefix) + "-compile.log").write_text(
        result.stdout + result.stderr, encoding="utf-8")
    print(result.stdout + result.stderr)
    if result.returncode:
        raise SystemExit(result.returncode)
    Path(str(prefix) + "-provenance.json").write_text(json.dumps({
        "source": str(source),
        "source_sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
        "power_scopes": scopes,
        "diagnostic_only": True,
        "native_gpu_exposed": False,
    }, indent=2), encoding="utf-8")


if __name__ == "__main__":
    main()
