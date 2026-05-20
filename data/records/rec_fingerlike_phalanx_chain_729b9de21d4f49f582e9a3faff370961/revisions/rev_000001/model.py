from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
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


OUTER_WIDTH = 0.024
GAP_WIDTH = 0.012
CHEEK_THICKNESS = (OUTER_WIDTH - GAP_WIDTH) / 2.0
CORE_WIDTH = 0.008
JOINT_RADIUS = 0.005

ROOT_BLOCK_REAR = 0.030
ROOT_BLOCK_HEIGHT = 0.024

PROXIMAL_LENGTH = 0.043
MIDDLE_LENGTH = 0.030
DISTAL_LENGTH = 0.022

PROXIMAL_HEIGHT = 0.020
MIDDLE_HEIGHT = 0.016
DISTAL_HEIGHT = 0.013
CHEEK_CENTER_Y = GAP_WIDTH / 2.0 + CHEEK_THICKNESS / 2.0

def _add_box(part, size, xyz, material, name: str) -> None:
    part.visual(
        Box(size),
        origin=Origin(xyz=xyz),
        material=material,
        name=name,
    )


def _add_y_cylinder(part, radius, length, xyz, material, name: str) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(-1.5707963267948966, 0.0, 0.0)),
        material=material,
        name=name,
    )


def _build_knuckle_block(part, material) -> None:
    _add_box(
        part,
        size=(0.020, OUTER_WIDTH, ROOT_BLOCK_HEIGHT),
        xyz=(-0.020, 0.0, 0.0),
        material=material,
        name="base_body",
    )
    _add_box(
        part,
        size=(0.018, CHEEK_THICKNESS, PROXIMAL_HEIGHT + 0.006),
        xyz=(-0.001, CHEEK_CENTER_Y, 0.0),
        material=material,
        name="left_cheek",
    )
    _add_box(
        part,
        size=(0.018, CHEEK_THICKNESS, PROXIMAL_HEIGHT + 0.006),
        xyz=(-0.001, -CHEEK_CENTER_Y, 0.0),
        material=material,
        name="right_cheek",
    )
    _add_y_cylinder(
        part,
        radius=0.0018,
        length=OUTER_WIDTH,
        xyz=(0.0, 0.0, 0.0),
        material=material,
        name="root_pin",
    )


def _build_forked_link(
    part,
    *,
    length: float,
    start_height: float,
    mid_height: float,
    child_height: float,
    material,
) -> None:
    _add_y_cylinder(
        part,
        radius=JOINT_RADIUS,
        length=GAP_WIDTH,
        xyz=(0.0, 0.0, 0.0),
        material=material,
        name="root_barrel",
    )
    _add_box(
        part,
        size=(0.010, CORE_WIDTH, start_height),
        xyz=(0.005, 0.0, 0.0),
        material=material,
        name="root_gusset",
    )

    spine_end = length - 0.018
    spine_length = max(spine_end - 0.010, 0.008)
    _add_box(
        part,
        size=(spine_length, CORE_WIDTH, mid_height),
        xyz=(0.010 + spine_length / 2.0, 0.0, 0.0),
        material=material,
        name="main_spine",
    )
    _add_box(
        part,
        size=(0.006, CORE_WIDTH, child_height),
        xyz=(length - 0.015, 0.0, 0.0),
        material=material,
        name="distal_taper",
    )
    _add_box(
        part,
        size=(0.004, OUTER_WIDTH, child_height + 0.004),
        xyz=(length - 0.012, 0.0, 0.0),
        material=material,
        name="fork_bridge",
    )
    _add_box(
        part,
        size=(0.016, CHEEK_THICKNESS, child_height + 0.008),
        xyz=(length - 0.002, CHEEK_CENTER_Y, 0.0),
        material=material,
        name="left_cheek",
    )
    _add_box(
        part,
        size=(0.016, CHEEK_THICKNESS, child_height + 0.008),
        xyz=(length - 0.002, -CHEEK_CENTER_Y, 0.0),
        material=material,
        name="right_cheek",
    )
    _add_y_cylinder(
        part,
        radius=0.0016,
        length=OUTER_WIDTH,
        xyz=(length, 0.0, 0.0),
        material=material,
        name="distal_pin",
    )


def _build_distal_link(part, material, pad_material) -> None:
    _add_y_cylinder(
        part,
        radius=JOINT_RADIUS,
        length=GAP_WIDTH,
        xyz=(0.0, 0.0, 0.0),
        material=material,
        name="root_barrel",
    )
    _add_box(
        part,
        size=(0.010, CORE_WIDTH, DISTAL_HEIGHT),
        xyz=(0.005, 0.0, 0.0),
        material=material,
        name="root_gusset",
    )
    _add_box(
        part,
        size=(0.010, CORE_WIDTH, 0.011),
        xyz=(0.015, 0.0, 0.0),
        material=material,
        name="mid_spine",
    )
    _add_box(
        part,
        size=(0.010, CORE_WIDTH, 0.010),
        xyz=(0.025, 0.0, -0.0005),
        material=material,
        name="tip_core",
    )
    _add_box(
        part,
        size=(0.014, 0.016, 0.010),
        xyz=(DISTAL_LENGTH + 0.010, 0.0, -0.001),
        material=pad_material,
        name="fingertip_pad",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="phalanx_chain")

    metal = model.material("metal", rgba=(0.73, 0.74, 0.76, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.24, 0.25, 0.28, 1.0))
    pad_rubber = model.material("pad_rubber", rgba=(0.14, 0.15, 0.17, 1.0))

    knuckle_block = model.part("knuckle_block")
    _build_knuckle_block(knuckle_block, dark_metal)

    proximal = model.part("proximal_phalanx")
    _build_forked_link(
        proximal,
        length=PROXIMAL_LENGTH,
        start_height=PROXIMAL_HEIGHT,
        mid_height=0.018,
        child_height=MIDDLE_HEIGHT,
        material=metal,
    )

    middle = model.part("middle_phalanx")
    _build_forked_link(
        middle,
        length=MIDDLE_LENGTH,
        start_height=MIDDLE_HEIGHT,
        mid_height=0.014,
        child_height=DISTAL_HEIGHT,
        material=metal,
    )

    distal = model.part("distal_phalanx")
    _build_distal_link(distal, metal, pad_rubber)

    model.articulation(
        "knuckle_flex",
        ArticulationType.REVOLUTE,
        parent=knuckle_block,
        child=proximal,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=2.5,
            lower=0.0,
            upper=1.35,
        ),
    )
    model.articulation(
        "proximal_flex",
        ArticulationType.REVOLUTE,
        parent=proximal,
        child=middle,
        origin=Origin(xyz=(PROXIMAL_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=5.0,
            velocity=2.5,
            lower=0.0,
            upper=1.55,
        ),
    )
    model.articulation(
        "distal_flex",
        ArticulationType.REVOLUTE,
        parent=middle,
        child=distal,
        origin=Origin(xyz=(MIDDLE_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.5,
            lower=0.0,
            upper=1.35,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    knuckle_block = object_model.get_part("knuckle_block")
    proximal = object_model.get_part("proximal_phalanx")
    middle = object_model.get_part("middle_phalanx")
    distal = object_model.get_part("distal_phalanx")

    knuckle_flex = object_model.get_articulation("knuckle_flex")
    proximal_flex = object_model.get_articulation("proximal_flex")
    distal_flex = object_model.get_articulation("distal_flex")

    fingertip_pad = distal.get_visual("fingertip_pad")

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

    ctx.check(
        "all_joint_axes_bend_in_one_plane",
        knuckle_flex.axis == (0.0, -1.0, 0.0)
        and proximal_flex.axis == (0.0, -1.0, 0.0)
        and distal_flex.axis == (0.0, -1.0, 0.0),
        "Expected all three revolute joints to share the same Y-axis flexion direction.",
    )
    ctx.check(
        "fingertip_pad_is_forward_mounted",
        fingertip_pad.origin.xyz[0] > DISTAL_LENGTH,
        f"Expected fingertip pad origin x to sit forward of the distal joint chain end; got {fingertip_pad.origin.xyz!r}.",
    )

    ctx.expect_contact(
        proximal,
        knuckle_block,
        contact_tol=5e-5,
        name="proximal_barrel_seats_in_grounded_knuckle",
    )
    ctx.expect_contact(
        middle,
        proximal,
        contact_tol=5e-5,
        name="middle_barrel_seats_in_proximal_cheeks",
    )
    ctx.expect_contact(
        distal,
        middle,
        contact_tol=5e-5,
        name="distal_barrel_seats_in_middle_cheeks",
    )

    with ctx.pose({knuckle_flex: 0.85, proximal_flex: 0.95, distal_flex: 0.80}):
        ctx.expect_origin_gap(
            distal,
            knuckle_block,
            axis="z",
            min_gap=0.020,
            name="flexion_lifts_the_fingertip",
        )
        ctx.expect_origin_gap(
            distal,
            knuckle_block,
            axis="x",
            min_gap=0.010,
            name="flexion_keeps_the_chain_reaching_forward",
        )
        ctx.expect_origin_gap(
            distal,
            knuckle_block,
            axis="y",
            min_gap=-0.001,
            max_gap=0.001,
            name="flexion_stays_in_a_single_bending_plane",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
