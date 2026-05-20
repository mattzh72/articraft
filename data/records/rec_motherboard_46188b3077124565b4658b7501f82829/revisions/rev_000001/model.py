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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


BOARD_LENGTH = 0.405
BOARD_WIDTH = 0.205
BOARD_THICKNESS = 0.002
BOARD_TOP_Z = BOARD_THICKNESS
SLOT_TOP_Z = BOARD_TOP_Z + 0.010

PCIE_SLOT_SIZE = (0.010, 0.032, 0.010)
DIMM_SLOT_SIZE = (0.140, 0.010, 0.010)

PCIE_SLOT_X_POSITIONS = (-0.065, -0.030, 0.005, 0.040, 0.075, 0.110, 0.145, 0.180)
PCIE_SLOT_CENTER_Y = -0.012
DIMM_SLOT_Y_POSITIONS = (0.086, 0.068)
DIMM_SLOT_CENTER_X = -0.020


def _add_box_visual(
    part,
    *,
    size: tuple[float, float, float],
    center: tuple[float, float, float],
    material,
    name: str,
):
    part.visual(
        Box(size),
        origin=Origin(xyz=center),
        material=material,
        name=name,
    )


def _slot_top_center(center_z: float, height: float) -> float:
    return BOARD_TOP_Z + (height * 0.5)


def _make_pcie_latch(
    model: ArticulatedObject,
    *,
    index: int,
    slot_center_x: float,
    slot_center_y: float,
    latch_material,
):
    slot_end_y = slot_center_y + (PCIE_SLOT_SIZE[1] * 0.5)
    pivot_radius = 0.0015
    latch = model.part(f"pcie_latch_{index}")
    latch.visual(
        Cylinder(radius=pivot_radius, length=PCIE_SLOT_SIZE[0]),
        origin=Origin(rpy=(0.0, math.pi * 0.5, 0.0)),
        material=latch_material,
        name="pcie_pivot",
    )
    latch.visual(
        Box((0.007, 0.006, 0.0024)),
        origin=Origin(xyz=(0.0, -0.0030, 0.0023)),
        material=latch_material,
        name="pcie_latch_body",
    )
    latch.visual(
        Box((0.006, 0.003, 0.005)),
        origin=Origin(xyz=(0.0, -0.0063, 0.0046)),
        material=latch_material,
        name="pcie_latch_tab",
    )
    latch.inertial = Inertial.from_geometry(
        Box((0.008, 0.010, 0.007)),
        mass=0.008,
        origin=Origin(xyz=(0.0, -0.0030, 0.0035)),
    )

    model.articulation(
        f"pcie_latch_joint_{index}",
        ArticulationType.REVOLUTE,
        parent="motherboard",
        child=latch,
        origin=Origin(
            xyz=(
                slot_center_x,
                slot_end_y,
                SLOT_TOP_Z + pivot_radius,
            )
        ),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.15,
            velocity=3.0,
            lower=0.0,
            upper=0.70,
        ),
    )


def _make_dimm_latch(
    model: ArticulatedObject,
    *,
    slot_index: int,
    side: str,
    slot_center_x: float,
    slot_center_y: float,
    latch_material,
):
    pivot_radius = 0.0016
    if side == "left":
        direction = 1.0
        axis = (0.0, -1.0, 0.0)
        pivot_x = slot_center_x - (DIMM_SLOT_SIZE[0] * 0.5)
    else:
        direction = -1.0
        axis = (0.0, 1.0, 0.0)
        pivot_x = slot_center_x + (DIMM_SLOT_SIZE[0] * 0.5)

    latch = model.part(f"dimm_{slot_index}_{side}_latch")
    latch.visual(
        Cylinder(radius=pivot_radius, length=DIMM_SLOT_SIZE[1]),
        origin=Origin(rpy=(math.pi * 0.5, 0.0, 0.0)),
        material=latch_material,
        name="dimm_pivot",
    )
    latch.visual(
        Box((0.0045, 0.007, 0.0024)),
        origin=Origin(xyz=(direction * 0.0022, 0.0, 0.0022)),
        material=latch_material,
        name="dimm_latch_body",
    )
    latch.visual(
        Box((0.0060, 0.007, 0.0048)),
        origin=Origin(xyz=(direction * 0.0050, 0.0, 0.0048)),
        material=latch_material,
        name="dimm_latch_tab",
    )
    latch.visual(
        Box((0.0025, 0.007, 0.0035)),
        origin=Origin(xyz=(direction * 0.0078, 0.0, 0.0066)),
        material=latch_material,
        name="dimm_latch_tip",
    )
    latch.inertial = Inertial.from_geometry(
        Box((0.012, 0.007, 0.010)),
        mass=0.006,
        origin=Origin(xyz=(direction * 0.0040, 0.0, 0.0045)),
    )

    model.articulation(
        f"dimm_{slot_index}_{side}_joint",
        ArticulationType.REVOLUTE,
        parent="motherboard",
        child=latch,
        origin=Origin(
            xyz=(
                pivot_x,
                slot_center_y,
                SLOT_TOP_Z + pivot_radius,
            )
        ),
        axis=axis,
        motion_limits=MotionLimits(
            effort=0.12,
            velocity=2.5,
            lower=0.0,
            upper=1.05,
        ),
    )


def _add_rotated_blade(part, *, angle: float, material, name: str):
    part.visual(
        Box((0.0032, 0.0120, 0.0016)),
        origin=Origin(
            xyz=(0.0, 0.0092, 0.0022),
            rpy=(0.22, 0.0, angle),
        ),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mining_motherboard")

    pcb = model.material("pcb", rgba=(0.11, 0.26, 0.12, 1.0))
    solder_mask = model.material("solder_mask", rgba=(0.07, 0.16, 0.08, 1.0))
    slot_plastic = model.material("slot_plastic", rgba=(0.86, 0.86, 0.84, 1.0))
    dark_slot = model.material("dark_slot", rgba=(0.12, 0.12, 0.13, 1.0))
    matte_black = model.material("matte_black", rgba=(0.10, 0.10, 0.11, 1.0))
    chipset_black = model.material("chipset_black", rgba=(0.08, 0.08, 0.09, 1.0))
    metal = model.material("metal", rgba=(0.73, 0.74, 0.77, 1.0))
    steel = model.material("steel", rgba=(0.56, 0.58, 0.61, 1.0))
    connector = model.material("connector", rgba=(0.27, 0.27, 0.29, 1.0))
    gold = model.material("gold", rgba=(0.78, 0.63, 0.24, 1.0))

    board = model.part("motherboard")
    _add_box_visual(
        board,
        size=(BOARD_LENGTH, BOARD_WIDTH, BOARD_THICKNESS),
        center=(0.0, 0.0, BOARD_THICKNESS * 0.5),
        material=pcb,
        name="pcb_main",
    )
    _add_box_visual(
        board,
        size=(0.012, 0.102, 0.036),
        center=(-0.1965, 0.036, BOARD_TOP_Z + 0.018),
        material=steel,
        name="rear_io_block",
    )
    _add_box_visual(
        board,
        size=(0.055, 0.055, 0.004),
        center=(-0.148, 0.054, BOARD_TOP_Z + 0.002),
        material=dark_slot,
        name="cpu_socket_frame",
    )
    _add_box_visual(
        board,
        size=(0.046, 0.046, 0.0014),
        center=(-0.148, 0.054, BOARD_TOP_Z + 0.0047),
        material=metal,
        name="cpu_socket_plate",
    )
    _add_box_visual(
        board,
        size=(0.070, 0.016, 0.014),
        center=(-0.128, 0.097, BOARD_TOP_Z + 0.007),
        material=connector,
        name="vrm_heatsink_top",
    )
    _add_box_visual(
        board,
        size=(0.016, 0.072, 0.014),
        center=(-0.104, 0.074, BOARD_TOP_Z + 0.007),
        material=connector,
        name="vrm_heatsink_side",
    )
    _add_box_visual(
        board,
        size=(0.022, 0.022, 0.012),
        center=(-0.176, 0.095, BOARD_TOP_Z + 0.006),
        material=connector,
        name="eps_power",
    )
    _add_box_visual(
        board,
        size=(0.009, 0.060, 0.012),
        center=(0.185, 0.071, BOARD_TOP_Z + 0.006),
        material=connector,
        name="atx_power",
    )
    _add_box_visual(
        board,
        size=(0.036, 0.012, 0.010),
        center=(0.170, -0.082, BOARD_TOP_Z + 0.005),
        material=connector,
        name="sata_block",
    )

    for slot_index, slot_center_y in enumerate(DIMM_SLOT_Y_POSITIONS, start=1):
        _add_box_visual(
            board,
            size=DIMM_SLOT_SIZE,
            center=(DIMM_SLOT_CENTER_X, slot_center_y, BOARD_TOP_Z + 0.005),
            material=dark_slot,
            name=f"dimm_slot_body_{slot_index}",
        )
        _add_box_visual(
            board,
            size=(0.126, 0.0030, 0.0016),
            center=(DIMM_SLOT_CENTER_X, slot_center_y, BOARD_TOP_Z + 0.0092),
            material=slot_plastic,
            name=f"dimm_slot_ridge_{slot_index}",
        )
        _add_box_visual(
            board,
            size=(0.126, 0.0020, 0.0008),
            center=(DIMM_SLOT_CENTER_X, slot_center_y, BOARD_TOP_Z + 0.0100),
            material=gold,
            name=f"dimm_slot_contacts_{slot_index}",
        )

    for slot_index, slot_center_x in enumerate(PCIE_SLOT_X_POSITIONS, start=1):
        _add_box_visual(
            board,
            size=PCIE_SLOT_SIZE,
            center=(slot_center_x, PCIE_SLOT_CENTER_Y, BOARD_TOP_Z + 0.005),
            material=slot_plastic,
            name=f"pcie_slot_body_{slot_index}",
        )
        _add_box_visual(
            board,
            size=(0.0065, 0.026, 0.0016),
            center=(slot_center_x, PCIE_SLOT_CENTER_Y - 0.001, BOARD_TOP_Z + 0.0092),
            material=dark_slot,
            name=f"pcie_slot_channel_{slot_index}",
        )

    _add_box_visual(
        board,
        size=(0.050, 0.050, 0.008),
        center=(0.022, 0.040, BOARD_TOP_Z + 0.004),
        material=chipset_black,
        name="chipset_heatsink",
    )
    _add_box_visual(
        board,
        size=(0.040, 0.004, 0.006),
        center=(0.022, 0.058, BOARD_TOP_Z + 0.011),
        material=matte_black,
        name="fan_frame_top",
    )
    _add_box_visual(
        board,
        size=(0.040, 0.004, 0.006),
        center=(0.022, 0.022, BOARD_TOP_Z + 0.011),
        material=matte_black,
        name="fan_frame_bottom",
    )
    _add_box_visual(
        board,
        size=(0.004, 0.032, 0.006),
        center=(0.040, 0.040, BOARD_TOP_Z + 0.011),
        material=matte_black,
        name="fan_frame_right",
    )
    _add_box_visual(
        board,
        size=(0.004, 0.032, 0.006),
        center=(0.004, 0.040, BOARD_TOP_Z + 0.011),
        material=matte_black,
        name="fan_frame_left",
    )
    board.visual(
        Cylinder(radius=0.0055, length=0.004),
        origin=Origin(xyz=(0.022, 0.040, BOARD_TOP_Z + 0.010), rpy=(0.0, 0.0, 0.0)),
        material=matte_black,
        name="fan_bearing_cap",
    )
    _add_box_visual(
        board,
        size=(0.060, 0.010, 0.008),
        center=(0.110, 0.086, BOARD_TOP_Z + 0.004),
        material=solder_mask,
        name="aux_header_bank",
    )
    _add_box_visual(
        board,
        size=(0.120, 0.010, 0.003),
        center=(0.055, -0.090, BOARD_TOP_Z + 0.0015),
        material=gold,
        name="rear_trace_strip",
    )

    board.inertial = Inertial.from_geometry(
        Box((BOARD_LENGTH, BOARD_WIDTH, 0.016)),
        mass=1.7,
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
    )

    fan = model.part("chipset_fan_rotor")
    fan.visual(
        Cylinder(radius=0.008, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=matte_black,
        name="rotor_hub",
    )
    for blade_index, angle in enumerate((0.0, 0.72 * math.pi, 1.44 * math.pi, 2.16 * math.pi, 2.88 * math.pi), start=1):
        _add_rotated_blade(
            fan,
            angle=angle,
            material=matte_black,
            name=f"rotor_blade_{blade_index}",
        )
    fan.inertial = Inertial.from_geometry(
        Cylinder(radius=0.016, length=0.006),
        mass=0.015,
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
    )

    model.articulation(
        "chipset_fan_spin",
        ArticulationType.CONTINUOUS,
        parent=board,
        child=fan,
        origin=Origin(xyz=(0.022, 0.040, BOARD_TOP_Z + 0.012)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.08, velocity=40.0),
    )

    for slot_index, slot_center_x in enumerate(PCIE_SLOT_X_POSITIONS, start=1):
        _make_pcie_latch(
            model,
            index=slot_index,
            slot_center_x=slot_center_x,
            slot_center_y=PCIE_SLOT_CENTER_Y,
            latch_material=matte_black,
        )

    for slot_index, slot_center_y in enumerate(DIMM_SLOT_Y_POSITIONS, start=1):
        _make_dimm_latch(
            model,
            slot_index=slot_index,
            side="left",
            slot_center_x=DIMM_SLOT_CENTER_X,
            slot_center_y=slot_center_y,
            latch_material=slot_plastic,
        )
        _make_dimm_latch(
            model,
            slot_index=slot_index,
            side="right",
            slot_center_x=DIMM_SLOT_CENTER_X,
            slot_center_y=slot_center_y,
            latch_material=slot_plastic,
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    board = object_model.get_part("motherboard")
    fan = object_model.get_part("chipset_fan_rotor")
    fan_joint = object_model.get_articulation("chipset_fan_spin")

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

    required_parts = {
        "motherboard",
        "chipset_fan_rotor",
        *{f"pcie_latch_{index}" for index in range(1, len(PCIE_SLOT_X_POSITIONS) + 1)},
        *{
            f"dimm_{slot_index}_{side}_latch"
            for slot_index in range(1, len(DIMM_SLOT_Y_POSITIONS) + 1)
            for side in ("left", "right")
        },
    }
    authored_part_names = {part.name for part in object_model.parts}
    for part_name in sorted(required_parts):
        ctx.check(
            f"part_present_{part_name}",
            part_name in authored_part_names,
            details=f"Missing required part: {part_name}",
        )

    required_joints = {
        "chipset_fan_spin",
        *{f"pcie_latch_joint_{index}" for index in range(1, len(PCIE_SLOT_X_POSITIONS) + 1)},
        *{
            f"dimm_{slot_index}_{side}_joint"
            for slot_index in range(1, len(DIMM_SLOT_Y_POSITIONS) + 1)
            for side in ("left", "right")
        },
    }
    authored_joint_names = {joint.name for joint in object_model.articulations}
    for joint_name in sorted(required_joints):
        ctx.check(
            f"joint_present_{joint_name}",
            joint_name in authored_joint_names,
            details=f"Missing required articulation: {joint_name}",
        )

    ctx.check(
        "chipset_fan_is_continuous",
        fan_joint.articulation_type == ArticulationType.CONTINUOUS
        and tuple(fan_joint.axis) == (0.0, 0.0, 1.0),
        details="Chipset fan should spin continuously about its local vertical axis.",
    )

    with ctx.pose({fan_joint: 1.4}):
        ctx.expect_contact(
            fan,
            board,
            elem_a="rotor_hub",
            elem_b="fan_bearing_cap",
            name="chipset_fan_hub_stays_on_bearing",
        )
        ctx.expect_overlap(
            fan,
            board,
            axes="xy",
            elem_a="rotor_hub",
            elem_b="chipset_heatsink",
            min_overlap=0.012,
            name="chipset_fan_sits_over_heatsink",
        )

    for slot_index in range(1, len(PCIE_SLOT_X_POSITIONS) + 1):
        latch = object_model.get_part(f"pcie_latch_{slot_index}")
        joint = object_model.get_articulation(f"pcie_latch_joint_{slot_index}")
        limits = joint.motion_limits
        ctx.check(
            f"pcie_latch_joint_axis_{slot_index}",
            tuple(joint.axis) == (-1.0, 0.0, 0.0)
            and limits is not None
            and limits.lower == 0.0
            and limits.upper is not None
            and limits.upper >= 0.60,
            details=f"PCIe latch {slot_index} needs a small clip rotation about its x-axis pivot.",
        )
        with ctx.pose({joint: 0.48}):
            ctx.expect_contact(
                latch,
                board,
                elem_a="pcie_pivot",
                elem_b=f"pcie_slot_body_{slot_index}",
                name=f"pcie_latch_{slot_index}_pivot_remains_clipped",
            )
            ctx.expect_gap(
                latch,
                board,
                axis="z",
                positive_elem="pcie_latch_body",
                negative_elem=f"pcie_slot_body_{slot_index}",
                min_gap=0.0003,
                max_gap=0.012,
                name=f"pcie_latch_{slot_index}_body_lifts_clear_when_open",
            )

    for slot_index in range(1, len(DIMM_SLOT_Y_POSITIONS) + 1):
        for side, expected_axis in (("left", (0.0, -1.0, 0.0)), ("right", (0.0, 1.0, 0.0))):
            latch = object_model.get_part(f"dimm_{slot_index}_{side}_latch")
            joint = object_model.get_articulation(f"dimm_{slot_index}_{side}_joint")
            limits = joint.motion_limits
            ctx.check(
                f"dimm_latch_joint_axis_{slot_index}_{side}",
                tuple(joint.axis) == expected_axis
                and limits is not None
                and limits.lower == 0.0
                and limits.upper is not None
                and limits.upper >= 1.0,
                details=f"DIMM latch {slot_index} {side} needs a hinged swing axis at the slot end.",
            )
            with ctx.pose({joint: 0.75}):
                ctx.expect_contact(
                    latch,
                    board,
                    elem_a="dimm_pivot",
                    elem_b=f"dimm_slot_body_{slot_index}",
                    name=f"dimm_latch_{slot_index}_{side}_pivot_remains_clipped",
                )
                ctx.expect_gap(
                    latch,
                    board,
                    axis="z",
                    positive_elem="dimm_latch_body",
                    negative_elem=f"dimm_slot_body_{slot_index}",
                    min_gap=0.0003,
                    max_gap=0.020,
                    name=f"dimm_latch_{slot_index}_{side}_body_lifts_clear_when_open",
                )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
