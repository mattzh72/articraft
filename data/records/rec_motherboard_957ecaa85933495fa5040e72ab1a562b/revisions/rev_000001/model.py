from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


BOARD_X = 0.305
BOARD_Y = 0.244
BOARD_T = 0.002

CPU_POS = (0.0, 0.040)
DIMM_BANK_POS = (0.094, 0.040)
EXPANSION_POS = (-0.010, -0.085)
M2_POS = (0.030, -0.050)
CHIPSET_POS = (0.074, -0.095)

DIMM_SLOT_XS = (-0.021, -0.007, 0.007, 0.021)


def _add_heatsink_fins(
    part,
    *,
    count: int,
    pitch: float,
    size: tuple[float, float, float],
    start: tuple[float, float, float],
    axis: str,
    material,
    prefix: str,
) -> None:
    sx, sy, sz = size
    x0, y0, z0 = start
    for index in range(count):
        dx = pitch * index if axis == "x" else 0.0
        dy = pitch * index if axis == "y" else 0.0
        part.visual(
            Box(size),
            origin=Origin(xyz=(x0 + dx, y0 + dy, z0)),
            material=material,
            name=f"{prefix}_{index + 1}",
        )


def _build_dimm_latch(part, material) -> None:
    part.visual(
        Cylinder(radius=0.0021, length=0.010),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=material,
        name="pivot_barrel",
    )
    part.visual(
        Box((0.0034, 0.004, 0.013)),
        origin=Origin(xyz=(0.0, 0.0006, 0.0065)),
        material=material,
        name="latch_arm",
    )
    part.visual(
        Box((0.0075, 0.004, 0.0035)),
        origin=Origin(xyz=(0.0, -0.0028, 0.0128)),
        material=material,
        name="finger_hook",
    )
    part.visual(
        Box((0.006, 0.004, 0.003)),
        origin=Origin(xyz=(0.0, 0.0026, 0.0026)),
        material=material,
        name="outer_tab",
    )


def _build_pcie_latch(part, material) -> None:
    part.visual(
        Cylinder(radius=0.0024, length=0.012),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=material,
        name="pivot_barrel",
    )
    part.visual(
        Box((0.018, 0.012, 0.003)),
        origin=Origin(xyz=(-0.009, 0.0, 0.0015)),
        material=material,
        name="latch_arm",
    )
    part.visual(
        Box((0.005, 0.012, 0.006)),
        origin=Origin(xyz=(-0.016, 0.0, 0.0045)),
        material=material,
        name="catch_tip",
    )


def _build_chipset_fan(part, material) -> None:
    part.visual(
        Cylinder(radius=0.004, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=material,
        name="hub",
    )
    blade_angles = (0.0, 72.0, 144.0, 216.0, 288.0)
    for index, degrees in enumerate(blade_angles, start=1):
        angle = math.radians(degrees)
        part.visual(
            Box((0.012, 0.0042, 0.0016)),
            origin=Origin(
                xyz=(0.0082 * math.cos(angle), 0.0082 * math.sin(angle), 0.0018),
                rpy=(0.0, 0.22, angle),
            ),
            material=material,
            name=f"blade_{index}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="atx_gaming_motherboard")

    pcb_black = model.material("pcb_black", rgba=(0.08, 0.09, 0.10, 1.0))
    dark_plastic = model.material("dark_plastic", rgba=(0.13, 0.14, 0.15, 1.0))
    slot_gray = model.material("slot_gray", rgba=(0.28, 0.29, 0.31, 1.0))
    steel = model.material("steel", rgba=(0.72, 0.74, 0.77, 1.0))
    gunmetal = model.material("gunmetal", rgba=(0.24, 0.25, 0.27, 1.0))
    matte_black = model.material("matte_black", rgba=(0.04, 0.04, 0.05, 1.0))
    charcoal = model.material("charcoal", rgba=(0.16, 0.17, 0.19, 1.0))
    chipset_gray = model.material("chipset_gray", rgba=(0.20, 0.21, 0.23, 1.0))

    pcb = model.part("pcb")
    pcb.visual(
        Box((BOARD_X, BOARD_Y, BOARD_T)),
        origin=Origin(xyz=(0.0, 0.0, BOARD_T * 0.5)),
        material=pcb_black,
        name="board",
    )
    pcb.visual(
        Box((0.016, 0.058, 0.015)),
        origin=Origin(xyz=(0.141, 0.040, 0.0095)),
        material=dark_plastic,
        name="atx_24pin",
    )
    pcb.visual(
        Box((0.012, 0.050, 0.003)),
        origin=Origin(xyz=(0.141, 0.040, 0.0185)),
        material=slot_gray,
        name="atx_24pin_latch",
    )
    pcb.visual(
        Box((0.026, 0.016, 0.014)),
        origin=Origin(xyz=(-0.121, 0.110, 0.008)),
        material=dark_plastic,
        name="eps_power",
    )
    for index, y_pos in enumerate((0.084, 0.058, 0.030, 0.000), start=1):
        pcb.visual(
            Box((0.012, 0.020, 0.014)),
            origin=Origin(xyz=(-0.145, y_pos, 0.008)),
            material=steel,
            name=f"rear_io_port_{index}",
        )
    for index, y_pos in enumerate((-0.040, -0.058), start=1):
        pcb.visual(
            Box((0.018, 0.014, 0.013)),
            origin=Origin(xyz=(0.136, y_pos, 0.0075)),
            material=dark_plastic,
            name=f"sata_port_{index}",
        )
    for index, x_pos in enumerate((-0.050, -0.032, -0.014, 0.004), start=1):
        pcb.visual(
            Box((0.012, 0.012, 0.007)),
            origin=Origin(xyz=(x_pos, 0.052, 0.0055)),
            material=charcoal,
            name=f"vrm_choke_{index}",
        )

    cpu_socket = model.part("cpu_socket")
    cpu_socket.visual(
        Box((0.060, 0.060, 0.003)),
        origin=Origin(xyz=(0.0, 0.0, 0.0015)),
        material=dark_plastic,
        name="socket_base",
    )
    cpu_socket.visual(
        Box((0.056, 0.056, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=steel,
        name="retention_frame",
    )
    cpu_socket.visual(
        Box((0.042, 0.042, 0.001)),
        origin=Origin(xyz=(0.0, 0.0, 0.0075)),
        material=steel,
        name="lid_plate",
    )

    vrm_shroud = model.part("vrm_shroud")
    vrm_shroud.visual(
        Box((0.048, 0.048, 0.010)),
        origin=Origin(xyz=(-0.012, 0.0, 0.005)),
        material=gunmetal,
        name="corner_bridge",
    )
    vrm_shroud.visual(
        Box((0.108, 0.028, 0.010)),
        origin=Origin(xyz=(0.038, 0.022, 0.005)),
        material=gunmetal,
        name="top_sink",
    )
    vrm_shroud.visual(
        Box((0.032, 0.104, 0.010)),
        origin=Origin(xyz=(-0.041, -0.021, 0.005)),
        material=gunmetal,
        name="left_sink",
    )
    vrm_shroud.visual(
        Box((0.020, 0.060, 0.018)),
        origin=Origin(xyz=(-0.054, -0.006, 0.009)),
        material=charcoal,
        name="rear_io_cover",
    )
    _add_heatsink_fins(
        vrm_shroud,
        count=6,
        pitch=0.016,
        size=(0.010, 0.020, 0.008),
        start=(-0.002, 0.022, 0.014),
        axis="x",
        material=charcoal,
        prefix="top_fin",
    )
    _add_heatsink_fins(
        vrm_shroud,
        count=5,
        pitch=0.018,
        size=(0.020, 0.010, 0.008),
        start=(-0.041, -0.052, 0.014),
        axis="y",
        material=charcoal,
        prefix="left_fin",
    )

    dimm_bank = model.part("dimm_bank")
    dimm_bank.visual(
        Box((0.058, 0.150, 0.002)),
        origin=Origin(xyz=(0.0, 0.0, 0.001)),
        material=matte_black,
        name="slot_base",
    )
    for index, slot_x in enumerate(DIMM_SLOT_XS, start=1):
        slot_material = slot_gray if index % 2 else dark_plastic
        dimm_bank.visual(
            Box((0.008, 0.142, 0.010)),
            origin=Origin(xyz=(slot_x, 0.0, 0.007)),
            material=slot_material,
            name=f"dimm_slot_{index}",
        )
        dimm_bank.visual(
            Box((0.002, 0.006, 0.010)),
            origin=Origin(xyz=(slot_x - 0.005, 0.074, 0.007)),
            material=slot_material,
            name=f"latch_cheek_{index}_left",
        )
        dimm_bank.visual(
            Box((0.002, 0.006, 0.010)),
            origin=Origin(xyz=(slot_x + 0.005, 0.074, 0.007)),
            material=slot_material,
            name=f"latch_cheek_{index}_right",
        )

    expansion_slots = model.part("expansion_slots")
    expansion_slots.visual(
        Box((0.150, 0.036, 0.002)),
        origin=Origin(xyz=(0.0, 0.0, 0.001)),
        material=matte_black,
        name="slot_base",
    )
    expansion_slots.visual(
        Box((0.132, 0.012, 0.010)),
        origin=Origin(xyz=(-0.004, 0.011, 0.007)),
        material=slot_gray,
        name="pcie_x16",
    )
    expansion_slots.visual(
        Box((0.050, 0.010, 0.009)),
        origin=Origin(xyz=(-0.040, -0.004, 0.0065)),
        material=dark_plastic,
        name="pcie_x1",
    )
    expansion_slots.visual(
        Box((0.090, 0.010, 0.009)),
        origin=Origin(xyz=(0.018, -0.019, 0.0065)),
        material=dark_plastic,
        name="pcie_x4",
    )
    expansion_slots.visual(
        Box((0.006, 0.012, 0.011)),
        origin=Origin(xyz=(-0.069, 0.011, 0.0075)),
        material=slot_gray,
        name="pcie_rear_stop",
    )
    expansion_slots.visual(
        Box((0.008, 0.002, 0.010)),
        origin=Origin(xyz=(0.066, 0.016, 0.007)),
        material=slot_gray,
        name="pcie_latch_cheek_upper",
    )
    expansion_slots.visual(
        Box((0.008, 0.002, 0.010)),
        origin=Origin(xyz=(0.066, 0.006, 0.007)),
        material=slot_gray,
        name="pcie_latch_cheek_lower",
    )

    m2_heatsink = model.part("m2_heatsink")
    m2_heatsink.visual(
        Box((0.095, 0.025, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=gunmetal,
        name="m2_cover",
    )
    m2_heatsink.visual(
        Box((0.083, 0.015, 0.002)),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=charcoal,
        name="m2_accent",
    )
    _add_heatsink_fins(
        m2_heatsink,
        count=5,
        pitch=0.018,
        size=(0.010, 0.019, 0.006),
        start=(-0.036, 0.0, 0.007),
        axis="x",
        material=charcoal,
        prefix="m2_rib",
    )

    chipset_cooler = model.part("chipset_cooler")
    chipset_cooler.visual(
        Box((0.040, 0.040, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=chipset_gray,
        name="base",
    )
    for offset in (-0.012, -0.004, 0.004, 0.012):
        chipset_cooler.visual(
            Box((0.003, 0.032, 0.006)),
            origin=Origin(xyz=(offset, 0.0, 0.007)),
            material=gunmetal,
            name=f"fin_x_{offset:+.3f}",
        )
    chipset_cooler.visual(
        Box((0.040, 0.004, 0.010)),
        origin=Origin(xyz=(0.0, 0.018, 0.009)),
        material=gunmetal,
        name="fan_frame_top",
    )
    chipset_cooler.visual(
        Box((0.040, 0.004, 0.010)),
        origin=Origin(xyz=(0.0, -0.018, 0.009)),
        material=gunmetal,
        name="fan_frame_bottom",
    )
    chipset_cooler.visual(
        Box((0.004, 0.032, 0.010)),
        origin=Origin(xyz=(0.018, 0.0, 0.009)),
        material=gunmetal,
        name="fan_frame_right",
    )
    chipset_cooler.visual(
        Box((0.004, 0.032, 0.010)),
        origin=Origin(xyz=(-0.018, 0.0, 0.009)),
        material=gunmetal,
        name="fan_frame_left",
    )
    chipset_cooler.visual(
        Cylinder(radius=0.0018, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=steel,
        name="fan_spindle",
    )

    chipset_fan = model.part("chipset_fan")
    _build_chipset_fan(chipset_fan, matte_black)

    dimm_latches = []
    for index in range(4):
        latch = model.part(f"dimm_latch_{index + 1}")
        _build_dimm_latch(latch, slot_gray)
        dimm_latches.append(latch)

    pcie_latch = model.part("pcie_latch")
    _build_pcie_latch(pcie_latch, slot_gray)

    model.articulation(
        "pcb_to_cpu_socket",
        ArticulationType.FIXED,
        parent=pcb,
        child=cpu_socket,
        origin=Origin(xyz=(CPU_POS[0], CPU_POS[1], BOARD_T)),
    )
    model.articulation(
        "pcb_to_vrm_shroud",
        ArticulationType.FIXED,
        parent=pcb,
        child=vrm_shroud,
        origin=Origin(xyz=(-0.040, 0.086, BOARD_T)),
    )
    model.articulation(
        "pcb_to_dimm_bank",
        ArticulationType.FIXED,
        parent=pcb,
        child=dimm_bank,
        origin=Origin(xyz=(DIMM_BANK_POS[0], DIMM_BANK_POS[1], BOARD_T)),
    )
    model.articulation(
        "pcb_to_expansion_slots",
        ArticulationType.FIXED,
        parent=pcb,
        child=expansion_slots,
        origin=Origin(xyz=(EXPANSION_POS[0], EXPANSION_POS[1], BOARD_T)),
    )
    model.articulation(
        "pcb_to_m2_heatsink",
        ArticulationType.FIXED,
        parent=pcb,
        child=m2_heatsink,
        origin=Origin(xyz=(M2_POS[0], M2_POS[1], BOARD_T)),
    )
    model.articulation(
        "pcb_to_chipset_cooler",
        ArticulationType.FIXED,
        parent=pcb,
        child=chipset_cooler,
        origin=Origin(xyz=(CHIPSET_POS[0], CHIPSET_POS[1], BOARD_T)),
    )
    model.articulation(
        "chipset_fan_spin",
        ArticulationType.CONTINUOUS,
        parent=chipset_cooler,
        child=chipset_fan,
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.3, velocity=300.0),
    )
    for index, (slot_x, latch) in enumerate(zip(DIMM_SLOT_XS, dimm_latches), start=1):
        model.articulation(
            f"dimm_latch_{index}_hinge",
            ArticulationType.REVOLUTE,
            parent=dimm_bank,
            child=latch,
            origin=Origin(xyz=(slot_x, 0.0732, 0.012)),
            axis=(-1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=0.15,
                velocity=4.0,
                lower=0.0,
                upper=math.radians(82.0),
            ),
        )
    model.articulation(
        "pcie_latch_hinge",
        ArticulationType.REVOLUTE,
        parent=expansion_slots,
        child=pcie_latch,
        origin=Origin(xyz=(0.0684, 0.011, 0.012)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.15,
            velocity=4.0,
            lower=0.0,
            upper=math.radians(68.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pcb = object_model.get_part("pcb")
    board_visual = pcb.get_visual("board")
    cpu_socket = object_model.get_part("cpu_socket")
    vrm_shroud = object_model.get_part("vrm_shroud")
    dimm_bank = object_model.get_part("dimm_bank")
    expansion_slots = object_model.get_part("expansion_slots")
    m2_heatsink = object_model.get_part("m2_heatsink")
    chipset_cooler = object_model.get_part("chipset_cooler")
    chipset_fan = object_model.get_part("chipset_fan")
    pcie_latch = object_model.get_part("pcie_latch")
    dimm_latches = [object_model.get_part(f"dimm_latch_{index}") for index in range(1, 5)]

    fan_joint = object_model.get_articulation("chipset_fan_spin")
    pcie_joint = object_model.get_articulation("pcie_latch_hinge")
    dimm_joints = [
        object_model.get_articulation(f"dimm_latch_{index}_hinge") for index in range(1, 5)
    ]

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    for part_name in (
        "pcb",
        "cpu_socket",
        "vrm_shroud",
        "dimm_bank",
        "dimm_latch_1",
        "dimm_latch_2",
        "dimm_latch_3",
        "dimm_latch_4",
        "expansion_slots",
        "pcie_latch",
        "m2_heatsink",
        "chipset_cooler",
        "chipset_fan",
    ):
        ctx.check(f"part present: {part_name}", object_model.get_part(part_name) is not None)

    pcb_aabb = ctx.part_world_aabb(pcb)
    assert pcb_aabb is not None
    board_dx = pcb_aabb[1][0] - pcb_aabb[0][0]
    board_dy = pcb_aabb[1][1] - pcb_aabb[0][1]
    ctx.check(
        "board has ATX proportions",
        abs(board_dx - BOARD_X) < 0.001 and abs(board_dy - BOARD_Y) < 0.001,
        details=f"measured ({board_dx:.4f}, {board_dy:.4f}) expected ({BOARD_X:.4f}, {BOARD_Y:.4f})",
    )

    for part, label in (
        (cpu_socket, "cpu socket"),
        (vrm_shroud, "vrm shroud"),
        (dimm_bank, "dimm bank"),
        (expansion_slots, "expansion slots"),
        (m2_heatsink, "m2 heatsink"),
        (chipset_cooler, "chipset cooler"),
    ):
        ctx.expect_contact(part, pcb, contact_tol=0.0005, name=f"{label} mounted to pcb")
        ctx.expect_within(part, pcb, axes="xy", margin=0.001, name=f"{label} footprint stays on pcb")

    ctx.expect_origin_distance(cpu_socket, pcb, axes="x", max_dist=0.020, name="cpu socket sits near board centerline")
    ctx.expect_origin_gap(cpu_socket, pcb, axis="y", min_gap=0.020, max_gap=0.065, name="cpu socket sits in upper half")
    ctx.expect_origin_gap(dimm_bank, cpu_socket, axis="x", min_gap=0.075, max_gap=0.110, name="dimm bank sits beside cpu socket")
    ctx.expect_origin_distance(dimm_bank, cpu_socket, axes="y", max_dist=0.020, name="dimm bank aligns with cpu socket")
    ctx.expect_origin_gap(cpu_socket, expansion_slots, axis="y", min_gap=0.105, max_gap=0.145, name="expansion slots sit below cpu area")
    ctx.expect_origin_gap(m2_heatsink, chipset_cooler, axis="y", min_gap=0.030, max_gap=0.055, name="chipset cooler sits below m.2 heatsink")

    ctx.check(
        "chipset fan axis is local z",
        fan_joint.axis == (0.0, 0.0, 1.0),
        details=f"axis={fan_joint.axis}",
    )
    ctx.check(
        "pcie latch axis is local y",
        pcie_joint.axis == (0.0, 1.0, 0.0),
        details=f"axis={pcie_joint.axis}",
    )
    for index, joint in enumerate(dimm_joints, start=1):
        ctx.check(
            f"dimm latch {index} axis uses local x",
            joint.axis in ((1.0, 0.0, 0.0), (-1.0, 0.0, 0.0)),
            details=f"axis={joint.axis}",
        )

    ctx.expect_contact(chipset_fan, chipset_cooler, contact_tol=0.0005, name="chipset fan remains seated on spindle")
    ctx.expect_within(chipset_fan, chipset_cooler, axes="xy", margin=0.003, name="chipset fan stays inside cooler frame")

    dimm_rest_top_z: list[float] = []
    dimm_rest_max_y: list[float] = []
    for index, latch in enumerate(dimm_latches, start=1):
        ctx.expect_contact(latch, dimm_bank, contact_tol=0.0005, name=f"dimm latch {index} contacts slot end")
        ctx.expect_overlap(latch, dimm_bank, axes="xy", min_overlap=0.004, name=f"dimm latch {index} sits on dimm slot end")
        latch_aabb = ctx.part_world_aabb(latch)
        assert latch_aabb is not None
        dimm_rest_top_z.append(latch_aabb[1][2])
        dimm_rest_max_y.append(latch_aabb[1][1])
        ctx.expect_gap(
            latch,
            pcb,
            axis="z",
            min_gap=0.002,
            negative_elem=board_visual,
            name=f"dimm latch {index} clears board in rest pose",
        )

    ctx.expect_contact(pcie_latch, expansion_slots, contact_tol=0.0005, name="pcie latch contacts slot body")
    ctx.expect_overlap(pcie_latch, expansion_slots, axes="xy", min_overlap=0.006, name="pcie latch sits on slot end")

    pcie_rest_aabb = ctx.part_world_aabb(pcie_latch)
    assert pcie_rest_aabb is not None

    for index, (joint, latch) in enumerate(zip(dimm_joints, dimm_latches), start=1):
        with ctx.pose({joint: math.radians(76.0)}):
            latch_aabb = ctx.part_world_aabb(latch)
            assert latch_aabb is not None
            ctx.check(
                f"dimm latch {index} swings outward",
                latch_aabb[1][1] > dimm_rest_max_y[index - 1] + 0.006,
                details=f"rest_y={dimm_rest_max_y[index - 1]:.4f}, open_y={latch_aabb[1][1]:.4f}",
            )
            ctx.expect_within(
                latch,
                dimm_bank,
                axes="x",
                margin=0.002,
                name=f"dimm latch {index} stays clipped to slot width",
            )
            ctx.check(
                f"dimm latch {index} stays above board when open",
                latch_aabb[1][2] > dimm_rest_top_z[index - 1] - 0.008,
                details=f"rest_top={dimm_rest_top_z[index - 1]:.4f}, open_top={latch_aabb[1][2]:.4f}",
            )
            ctx.expect_gap(
                latch,
                pcb,
                axis="z",
                min_gap=0.002,
                negative_elem=board_visual,
                name=f"dimm latch {index} clears board when open",
            )

    with ctx.pose({pcie_joint: math.radians(62.0)}):
        pcie_open_aabb = ctx.part_world_aabb(pcie_latch)
        assert pcie_open_aabb is not None
        ctx.check(
            "pcie latch opens upward",
            pcie_open_aabb[1][2] > pcie_rest_aabb[1][2] + 0.008,
            details=f"rest_top={pcie_rest_aabb[1][2]:.4f}, open_top={pcie_open_aabb[1][2]:.4f}",
        )
        ctx.expect_within(
            pcie_latch,
            expansion_slots,
            axes="y",
            margin=0.002,
            name="pcie latch stays captured by hinge cheeks",
        )

    with ctx.pose({fan_joint: 2.1}):
        ctx.expect_contact(
            chipset_fan,
            chipset_cooler,
            contact_tol=0.0005,
            name="chipset fan stays seated while spinning",
        )
        ctx.expect_within(
            chipset_fan,
            chipset_cooler,
            axes="xy",
            margin=0.003,
            name="chipset fan clears frame while spinning",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
