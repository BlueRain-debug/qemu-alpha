.. SPDX-License-Identifier: GPL-2.0-or-later

OEMSK MSM8998 board (``oemsk``)
==============================

This local experimental board runs an AArch64 Windows OEMSK image with
synthetic UEFI firmware and selected native MSM8998 devices. It shares the
generic CPU, firmware, PCI and ACPI implementation with ``virt`` through an
abstract base machine. ``oemsk-machine`` and ``virt-machine`` are sibling
types; MSM8998 devices are built and instantiated by the OEMSK board.

Use ``-machine oemsk`` in place of the former
``-machine virt,gic-version=2,qcom8998-display=on`` experiment. GICv2 and ACPI
are enabled by default. The board requires TCG, an AArch64 CPU, and disabled
EL2 and EL3. The default CPU is ``max``; up to eight CPUs can be instantiated.
Legacy AArch32 WinRT board options and the machine-wide SMMUv3 option are
rejected. Native SMMUv2 translation is separate from the synthetic PCI bus.

The ``board-ssdt`` property accepts a single external SSDT with a valid
header, size and checksum. Supply the extracted board panel/PEP data from
the local experiment. The inherited ``winrt-qcdx-ssdt`` spelling also works.
Vendor firmware, AML, Windows drivers and disk images are not distributed
with the model.

Memory and interrupt layout
---------------------------

The synthetic boot ABI retains code flash at zero, a 16 MiB variable-flash
bank at ``0x04000000``, and RAM starting at ``0x40000000``. Native chip-info,
QFPROM and RPM register windows intentionally override small portions of
code flash, matching the earlier firmware experiment.

Native devices retain their physical addresses. Synthetic resources are
placed outside their windows:

* GICv2 distributor/CPU interface: ``0x0b000000`` / ``0x0b010000``.
* UART, RTC and fw_cfg: ``0x0b100000``, ``0x0b110000``, ``0x0b120000``.
* Virtio MMIO transports: ``0x0b300000``.
* Allocatable platform bus: ``0x0b400000..0x0bffffff``.
* Low PCI MMIO: ``0x20000000..0x3efeffff``.

The platform bus excludes native multimedia/SMMU MMIO. The PCI window
excludes native RPM IPC and CPU CPRh registers. FDT and ACPI generation use
these same board memory-map entries. The GIC has 704 INTIDs. Native IRQs
retain their INTIDs; synthetic devices use INTIDs 512 through 703.

CPU-subsystem MMIO at ``0x17800000..0x179fffff`` has an explicit
read-as-zero/write-ignore compatibility placeholder. This contains PEP's
SPM/ACC, CPU PLL and OSM accesses that previously fell through to the
unassigned PCI aperture. Implemented CPRh and RPM IPC regions override the
placeholder. The remaining registers stay unimplemented and log accesses
with ``-d unimp``; this does not model power sequencing, PLL lock or OSM.
The native MADT GICv3 distributor, redistributor and MSI-frame windows also
retain explicit RAZ/WI placeholders. Interrupt delivery uses the synthetic
GICv2 described above; native GICv3 operation is not implemented.
Shared IMEM and the overlapping ACPI diagnostic-buffer ranges at
``0x146bf000..0x146c1fff`` are backed by zero-initialized SRAM. Writes persist
within the instance. Secure firmware does not populate these buffers.

Adreno SMMUv2 fault handling
---------------------------

The CP DMA stream uses the guest's stream-0 mapping and AArch64 stage-1,
4 KiB translation tables. Translation, access-flag, permission, address-size,
unsupported-format and table-walk external faults record FSR, FAR and
FSYNR0. FAR preserves the input address before SEP sign extension. The first
syndrome is retained; additional faults set MULTI when HUPCF permits further
transactions. With HUPCF clear, outstanding context faults block subsequent
transactions until software acknowledges them.

Missing/conflicting stream mappings and invalid context selections record
global fault status and syndrome. The global interrupt uses INTID 366;
context banks 0 and 1 use INTIDs 361 and 362. GFIE and CFIE mask interrupts
without discarding fault state. FSR/GFSR support write-one-to-clear and fault
restore. Reset clears fault state and lowers the fault IRQs. CFRE does not
disable recording or CFIE.

This implementation supports terminating faults only: SCTLR.CFCFG is RAZ/WI,
there is no stalled transaction, and RESUME has no effect. After fixing and
acknowledging a fault, software must resubmit the device operation. Bypass
selection and disabled context translation use physical addresses; DMA
accesses remain restricted to ordinary writable guest RAM.

Bounded Adreno execution
------------------------

The software command processor executes supported commands synchronously.
WAIT_REG_MEM supports masked register/RAM comparisons and retries an unmet
dependency after 1 ms of virtual time. This is a software polling interval,
not an emulated Adreno clock. Halt and reset cancel the retry; terminating
DMA faults still require explicit resubmission. CP writes to the two SMMU
contexts' TLBIALL/TLBSYNC registers complete immediately because the model
does not cache translations or have outstanding page walks.

RBBM raw interrupt status and masked status are distinct. CP completion
interrupts remain latched until acknowledged with write-one-to-clear, and
the output level is the Boolean intersection of status and mask. A later
fence or interrupt cannot pass an unsupported earlier draw.

A bounded IR3 interpreter executes moves, relative constant reads, integer
shift, full-precision float add/multiply, unfused MAD and END. Constant
uploads can also read the graphics stages' shared UBO address table. The
supported source-3 form packs a slot and a vec4-aligned byte offset. Block-6
auxiliary words remain opaque; shader UBO instructions and preemption are
not implemented. A source fault cannot partially publish a constant upload.

Dirty draw-state groups execute in group order before a draw. Unchanged
bindings retain cached state unless DIRTY is set. An internal state stream
can run inside IB2 without creating another guest-visible IB register set.
Its supported commands are register/state uploads, NOP and WAIT_FOR_IDLE;
recursive work and immediate-state execution remain unsupported. Faults
retain the state-stream position and keep the caller's draw pending.

The nonindexed RECTLIST path supports two pretransformed integer screen
vertices and a constant RGBA8 fragment output into a linear attachment.
It also supports QCDX's observed single texture-prefetch configuration with
an END-only fragment shader, a float2 varying and a linear RGBA8/R8 texture.
Sampling uses unnormalized XY, nearest filtering and clamp-to-edge; source
and destination byte permutations are decoded independently. Source pages
are cached only when sampled, and a later source fault cannot publish
earlier pixels. Other prefetch encodings remain unsupported. Inactive
prefetch command words may retain their previous contents.
R8 targets also permit the native single-pixel initialization at the origin
of an uncompressed TILE5_3 allocation. Only its first byte is accessed;
this does not implement the remaining tiled address layout or UBWC flags.
The indexed TRIANGLELIST path supports 16-bit indices, one instance and a
single vec4 color varying. Vertex fetch handles float2/float3 and RGBA8 UNORM.
Positions require W=1 and a symmetric origin-zero viewport; D3D11's 16.8
nearest-even snapping and a top-left coverage rule determine covered pixels.
Affine color interpolation supplies bary.f, and the supported linear RGBA8
blend mode is premultiplied ONE + ONE_MINUS_SRC_ALPHA. Each draw stages its
pixels before publication, so a later vertex or shader failure cannot
partially update its attachment. Work and shader steps have explicit limits.

This remains a restricted pipeline: shader texture instructions, other
texture formats/filtering, perspective division, primitive restart,
instancing, other blend modes, depth/stencil and MSAA are not implemented.
Denormal and nonfinite arithmetic remain unsupported. Interpolation,
sampling and blend precision have been checked with software fixtures,
not compared bit-for-bit with an A540. Linear draws preflight and snapshot
the complete clipped rectangle, which must be resident in ordinary RAM.

The A540's 1 MiB GMEM is represented as private logical RGBA8 texels. The
model supports fast clear and indexed color draws in single-sample bins,
including RB_WINDOW_OFFSET, and resolve to linear system memory. Fast clear
can follow the previous bin's store without resetting its direction bits.
Linear BYPASS draws ignore that bin offset. This does not expose a hardware
tile swizzle or support externally tiled/UBWC surfaces. All destination
rows and an optional
timestamp are validated before changing pixels. A later-row SMMU fault
therefore leaves earlier destination rows intact. GMEM persists between
commands and is cleared on device reset.

MDSS still displays transferred border colors; source-pipe DMA and native
scanout are not implemented. Successful GPU clears and the existence of a
ComposableShell process do not establish that Composer has displayed a
frame. The two panel consoles retain their 1350 by 1800 portrait dimensions.

Limitations
-----------

This is not a complete native MSM8998 SoC or a native UFS/XBL/TZ boot path.
The LUN4 partition images are reference inputs, not emulated UFS LUNs. Boot
still uses virtual firmware and a synthetic storage controller. MDSS SMMU
DMA, stage 2, TLB caching, secure translation, full GPU rendering, PMIC power
behavior and migration are not implemented. Splitting the board does not
establish native desktop or dual-panel rendering correctness.

Register semantics are based on the `Arm SMMUv2 architecture specification
<https://documentation-service.arm.com/static/5f900d34f86e16515cdc08fb>`_
and the `Linux arm-smmu driver
<https://github.com/torvalds/linux/tree/v6.12/drivers/iommu/arm/arm-smmu>`_.

Run the diskless regression tests with::

  QTEST_QEMU_BINARY=build/qemu-system-aarch64 build/tests/qtest/oemsk-test

For a firmware boot, replace the paths with local copies and use snapshots::

  qemu-system-aarch64 -machine oemsk,board-ssdt=board.aml \
    -cpu max,pauth-impdef=on -accel tcg,thread=multi -smp 4 -m 4G \
    -nodefaults -snapshot \
    -drive if=pflash,unit=0,format=raw,readonly=on,file=code.fd \
    -drive if=pflash,unit=1,format=raw,file=vars-16m.fd \
    -drive if=none,id=os,format=qcow2,file=os.qcow2 \
    -device ramfb -device qemu-xhci,id=usb \
    -device usb-kbd,bus=usb.0 -device usb-tablet,bus=usb.0 \
    -device usb-storage,bus=usb.0,drive=os,logical_block_size=4096,physical_block_size=4096 \
    -display gtk,zoom-to-fit=on,show-cursor=on -nic none
