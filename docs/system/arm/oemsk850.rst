.. SPDX-License-Identifier: GPL-2.0-or-later

OEMSK SDM845 board (``oemsk850``)
===============================

This independent, local experimental board uses the abstract ArmVirt base
for virtual firmware and boot I/O. It instantiates only ``qcom845-*`` native
devices. It does not instantiate MSM8998's display, RPM, SPMI or SCM model.
Use TCG, an AArch64 CPU, GICv2, ACPI and disabled EL2/EL3. Up to eight CPUs
and a 16 MiB variable flash are supported. Migration is not supported.

This is an experimental SDM845 bring-up board. With the opt-in native power,
SMMU, PIL and GPU SSDT plus Command DB, it supports unpatched ``qcdxkm845.sys``
display takeover, DPU 4.0 dual-panel ``1350x1800`` (and merged ``2700x1800``)
scanout, and bounded Adreno 630 direct-render IR3 rasterization. ``ramfb``
remains the early boot framebuffer.

Resource layout
---------------

Native INTIDs through 541 are reserved; synthetic interrupts start at 576.
The GIC exposes 768 INTIDs. Allocatable platform-bus space is
``0x1b000000..0x1bffffff``; low PCI MMIO is
``0x20000000..0x2effffff``. Neither includes the native SDM845 MMIO range.
Code flash resides at ``0x30000000..0x33ffffff`` and variable flash at
``0x34000000..0x34ffffff``. CPU RVBAR points to ``0x30000000``. This board
requires the separately built ``ArmVirtFakeOemsk850`` firmware: the old
zero-based FakeOemsk image is incompatible with this layout. In particular,
GCC and identity/fuse reads can no longer read bytes from the boot ROM.
Fixed virtual GIC, UART/RTC/fw_cfg and virtio transports use gaps at
``0x0b000000``, ``0x0b100000`` and ``0x0b200000`` respectively, compatible
with the current ArmVirt firmware's early device mapping.
PEP's native distributor address ``0x17a00000..0x17a00fff`` aliases the
actual GICv2 register state for compatible enable/pending operations.
This is not a native GICv3 implementation.
The ID words at ``0x17a0ffd0..0x17a0ffff`` also alias the real GICv2's
identification registers and report architecture version 2.

Implemented native frontends are:

* SCM/QSEE transport with bounded volatile UEFI-variable services.
* SPMI v5 at ``0x0c400000`` with three PMIC revision peripherals. PMIC
  writes fail explicitly; physical regulator and GPIO behavior is absent.
* RPMh RSC at ``0x179c0000``, ``0x179d0000``, ``0x179e0000`` and
  ``0x0af20000``. Bounded software resource votes complete TCS requests and
  signal native IRQs. No AOP firmware, rail settling or sleep solver runs.
  APSS TCS blocks start at ``+0xd00``; display TCS blocks at ``+0x1c00``.
  Capabilities report distinct six-bit NUM_TCS fields for DRV0/1/2.
  The read-only BCM revision word at ``0x0bbf1708`` advertises virtual
  1.5 compatibility for ICB's zero-bandwidth-vote workaround selection.
  This is not a physical revision readback; other BCM status/counter
  registers remain unmapped.
* Two SMMUv2 frontends at ``0x15000000`` and ``0x05040000``. AArch64
  stage-1 4 KiB page walks (both TTBR0 and ``TCR2.SEP`` sign-extended TTBR1
  upper ranges), ATOS, fault registers, per-process ``CP_SMMU_TABLE_UPDATE``
  switches and IRQ aggregation are implemented.
* 256 KiB SDM845 IMEM RAM at ``0x14680000`` for PIL/subsystem handoff.
* Isolated ``qcom845-display`` frontend (``hw/display/qcom-845-display.c`` and
  ``hw/display/qcom-845-render.c``) mapping MDSS/DPU 4.0 (``0x0ae00000``),
  DP PHY (``0x088e0000``), Adreno 630 (``0x05000000``), GMU_CX (``0x05060000``),
  CX_MISC (``0x05099000``), GPU PDC sequencer/regs (``0x0b280000``/``0x0b480000``),
  and Venus 4.4 wrapper ID (``0x0aa00000``). It implements DPU 4.0 CTL/DSI/MDP
  interrupts, 60 Hz VSYNC, three graphic consoles (``head=0``/``head=1`` at
  ``1350x1800`` and merged ``head=2`` at ``2700x1800``) scanned out via APPS
  SMMU SID ``0x880``, Adreno 630 CP ring/IB/fence/IRQ execution, and a bounded
  direct-render IR3 software rasterizer (``cat0``-``cat6`` including A6xx
  bindless ``cat5``/``cat6``, VFD, ``DI_PT_RECTLIST``, ``DI_PT_TRILIST``/
  ``TRIFAN``/``TRISTRIP``, alpha blending, and linear A8/R8/RG8/RGBA8/BGRA8
  SYSMEM targets). GMEM binning, UBWC tiling, Venus video execution, and real
  GMU firmware execution are not implemented.
* TSENS v2 SROT/TM pairs at ``0x0c222000``/``0x0c263000`` and
  ``0x0c223000``/``0x0c265000``. Thirteen/eight sensors report constant
  25 degrees C. Signed thresholds, latched alarms, masks, clear registers
  and native INTIDs 538/540/539/541 are implemented.
* AOSS QMP message RAM at ``0x0c300000``, doorbell at ``0x1799000c`` and
  edge INTID 421. Link/channel handshakes and the QDSS clock software vote
  are supported. Unknown messages remain pending rather than completing.
* SDM845 chip ID 321 / revision 2.1 at native HW_VERSION/JTAG windows.
  QFPROM reads
  zero: physical per-device calibration fuses are not available.
* CPRh APC0/APC1 register frontends at ``0x17dc0000``/``0x17db0000``.
  A configured OSM firmware handoff uses the native table's 400 mV base
  and 4 mV steps. This is register state, not a physical CPR feedback loop.
* An external read-only Command DB supplied with ``-M oemsk850,cmd-db=FILE``.
  Size, header, entry and auxiliary-data bounds are checked before boot.
  The native AOP directory at ``0x0c3f000c`` points into the model's SRAM
  window at ``0x0c3d0000``, outside guest RAM. With no file, the directory
  reports no table. The model does not invent missing resource entries.
* GCC at ``0x00100000`` with software branch-vote CLOCK_OFF status, immediate
  RCG UPDATE completion, GDSC collapse status, and bounded Fabia PLL lock
  state. Register indexes follow the public SDM845 GCC driver. QUP DFS is
  disabled, not populated with fictitious frequency tables. No physical
  clock timing, voltage feedback or TCG speed scaling is implemented.
  GPLL0/6/4 FSM ACTIVE status follows their own vote bits 0/6/4 at
  ``+0x52000`` and valid enable/OPMODE state. ACTIVE/LOCK flags are read-only.
* DISP_CC at ``0x0af00000`` stores display-clock configuration, implements
  the software GDSC collapse status at ``+0x3000``, 22 branch status words,
  14 RCG update strobes and Fabia PLL lock state. Hardware-controlled sleep
  transitions, DSI PHY clocks and native DPU output are not implemented.
* VIDEO_CC/CAM_CC/GPU_CC at ``0x0ab00000``, ``0x0ad00000`` and
  ``0x05090000`` expose software GDSC, branch, RCG and Fabia PLL controls.
  Register addresses follow the public SDM845 clock drivers. This does not
  implement video, camera, Adreno or GMU execution or hardware sleep control.
* OSM L3/Silver/Gold frontends at ``0x17d41000``, ``0x17d43000`` and
  ``0x17d45800``. Each exposes eighteen explicitly virtual operating points,
  a read-only LUT with a repeated final row, and bounded per-core performance
  selection. These tables are not physical-device readback. LMh throttling,
  physical voltage feedback and changes to TCG execution speed are absent.
* Five NoC QoS generator frontends for AGGRE1/AGGRE2/MEM/MMSS/SYSTEM fabrics.
  Sixteen virtual ports per fabric expose masked MAINCTL, LIMITBW, REGUL0CTL
  and REGUL0BW words. Other addresses remain unassigned. Configuration is
  retained and reset, but there is no transaction arbitration, bandwidth
  enforcement, performance measurement or NoC error reporting.
* TLMM's 150 GPIOs use their native EAST/NORTH/SOUTH tile assignments at
  ``0x03500000``, ``0x03900000`` and ``0x03d00000``. Pin ID, mux/pull/drive,
  output latch and interrupt configuration are retained. Wrong-tile probes
  return absent. There are no external inputs, GPIO interrupts or wake events;
  an unconnected input reads low, and an enabled output reads its own latch.

Optional native power-table diagnostics
--------------------------------------

Default ACPI exposes SCM0, TREE, SPMI and PMIC only. To investigate the
original PEP/ABD/SMMU drivers, extract those devices and all PEP scopes from
an already disassembled native SDM845 DSDT::

  python scripts/oemsk850-extract-power-ssdt.py path/to/dsdt.dsl \
      --output-prefix path/to/power-smmu --iasl path/to/iasl

Pass the resulting AML with ``-M oemsk850,board-ssdt=path/to/power-smmu.aml``.
The extraction preserves native configuration data, not a complete board
namespace. CPU power objects, SMEM firmware data and many clock/power
devices remain unimplemented. The original SMEM address lies inside guest
RAM: do not initialize it without a matching firmware reservation. Opt-in
power-table boots can still fail and are not a native display launch path.
Vendor ACPI, firmware, drivers and images are not distributed in this tree.

For the original 17047 AOP build, the static ARC/BCM CmdDB groups can be
reconstructed with ``scripts/oemsk850-extract-cmddb.py AOP.img
--output-prefix path/to/cmddb-arc-bcm``. The script checks the source hash,
extracts the 56 real resource entries and auxiliary data, and emits a
2080-byte table plus provenance. Dynamic VRM entries are absent. Other
firmware builds are rejected rather than using unverified table offsets.

Local Windows launcher
----------------------

``scripts/run-oemsk850.ps1`` uses the separate FakeOemsk850 AArch64 firmware,
a small per-run VARS/RPMB copy, GTK with scaling, and ``snapshot=on`` for
the OS disk. It creates no persistent OS differential disk and never opens
the base image writable. Default forwarded ports are Telnet 5925 to guest
23, QMP 5970 and GDB 5971. Telnet reachability does not establish login.
Paths can be supplied as parameters; ``-BoardSsdt`` opts into the diagnostic
table above, and ``-CmdDb`` supplies the corresponding firmware resource
table. ``-WaitForDebugger`` starts the guest paused.

The unit checks are in ``tests/qtest/oemsk850-test.c``. They cover machine
isolation, PMIC revision/failure/reset, RSC votes/IRQs/bounds, SMMU ATOS and
fault IRQs, TSENS temperatures/alarms, AOSS handshake/messages and SoC/CPRh
firmware identity, GCC control and OSM selection. Passing these checks does
not establish native Windows
display takeover or successful initialization of every 845 driver.

Local 17047 checkpoint (2026-11-25)
---------------------------------

The AArch64 build passes all 24 OEMSK850 qtests (including SMMUv2 TTBR1,
``CP_SMMU_TABLE_UPDATE``, ``CP_COND_EXEC``, DPU 4.0 VSYNC/DSI IRQs, and
``/oemsk850/a630-direct-render``) and all 20 existing MSM8998 ``oemsk`` qtests
with strict machine isolation.

A live ``10.0.17047.1034`` FactoryOS boot via ``scripts/run-oemsk850.ps1``
with ``snapshot=on`` and the opt-in ``power-smmu-gpu.aml`` (exposing PEP, ABD,
SMMU, PILC and GPU0) plus ``cmddb-repro.bin`` passed unpatched
``qcdxkm845.sys`` ``DxgkDdiStartDevice`` and ``DxgkDdiQueryChildRelations``
with zero bugchecks, zero SMMU faults, and zero GPU SW resets. All 41 initial
``dwm.exe`` / ``ComposableShell`` direct-render draws completed with zero
rejects, lighting up both native ``1350x1800`` panels (and merged
``2700x1800`` view) with the dual-screen wallpaper and anti-aliased
DirectWrite text overlays.
