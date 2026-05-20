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

from math import pi


BOSS_RADIUS = 0.014
PIN_HEAD_RADIUS = 0.017
BASE_BOSS_LENGTH = 0.010
ROOT_BOSS_LENGTH = 0.006
DISTAL_BOSS_LENGTH = 0.010
PIN_HEAD_THICKNESS = 0.004

LINK_SECTION_DEPTH = 0.018
LINK_SHOULDER_DEPTH = 0.028
CENTER_BEAM_WIDTH = 0.016
ROOT_LUG_LENGTH = 0.030
ROOT_LUG_WIDTH = 0.006
DISTAL_BLOCK_LENGTH = 0.026
DISTAL_BLOCK_WIDTH = 0.008
BEAM_START_X = 0.024
BEAM_END_BACKOFF = 0.022
BEAM_Y_CENTER = 0.004
RIB_THICKNESS = 0.005
RIB_WIDTH = 0.010
END_NECK_LENGTH = 0.018
END_TAB_LENGTH = 0.026

LINK_1_LENGTH = 0.115
LINK_2_LENGTH = 0.102
LINK_3_LENGTH = 0.086

HINGE_AXIS = (0.0, -1.0, 0.0)

CYLINDER_Y = Origin(rpy=(pi / 2.0, 0.0, 0.0))


def _beam_dims(length: float) -> tuple[float, float]:
    beam_end = length - BEAM_END_BACKOFF
    beam_length = beam_end - BEAM_START_X
    beam_center_x = BEAM_START_X + beam_length / 2.0
    return beam_length, beam_center_x


def _add_base_visuals(model: ArticulatedObject, base) -> None:
    base.visual(
        Box((0.120, 0.070, 0.018)),
        origin=Origin(xyz=(-0.040, 0.0, -0.060)),
        material="base_steel",
        name="base_plate",
    )
    base.visual(
        Box((0.050, 0.026, 0.044)),
        origin=Origin(xyz=(-0.025, -0.002, -0.035)),
        material="base_steel",
        name="base_pedestal",
    )
    base.visual(
        Box((0.040, BASE_BOSS_LENGTH, 0.048)),
        origin=Origin(xyz=(-0.020, -0.005, 0.0)),
        material="base_steel",
        name="base_cheek",
    )
    base.visual(
        Box((0.024, BASE_BOSS_LENGTH, 0.034)),
        origin=Origin(xyz=(-0.014, -0.005, -0.020)),
        material="base_steel",
        name="base_gusset",
    )
    base.visual(
        Cylinder(radius=BOSS_RADIUS, length=BASE_BOSS_LENGTH),
        origin=Origin(xyz=(0.0, -0.005, 0.0), rpy=CYLINDER_Y.rpy),
        material="base_steel",
        name="base_pin_boss",
    )
    base.visual(
        Cylinder(radius=PIN_HEAD_RADIUS, length=PIN_HEAD_THICKNESS),
        origin=Origin(xyz=(0.0, -0.012, 0.0), rpy=CYLINDER_Y.rpy),
        material="base_steel",
        name="base_pin_head",
    )


def _add_link_visuals(link, *, length: float, material: str, prefix: str) -> None:
    beam_length, beam_center_x = _beam_dims(length)

    link.visual(
        Box((ROOT_LUG_LENGTH, ROOT_LUG_WIDTH, LINK_SHOULDER_DEPTH)),
        origin=Origin(xyz=(ROOT_LUG_LENGTH / 2.0, ROOT_LUG_WIDTH / 2.0, 0.0)),
        material=material,
        name=f"{prefix}_root_lug",
    )
    link.visual(
        Cylinder(radius=BOSS_RADIUS, length=ROOT_BOSS_LENGTH),
        origin=Origin(xyz=(0.0, ROOT_LUG_WIDTH + ROOT_BOSS_LENGTH / 2.0, 0.0), rpy=CYLINDER_Y.rpy),
        material=material,
        name=f"{prefix}_root_boss",
    )
    link.visual(
        Cylinder(radius=PIN_HEAD_RADIUS, length=PIN_HEAD_THICKNESS),
        origin=Origin(
            xyz=(0.0, ROOT_LUG_WIDTH + ROOT_BOSS_LENGTH + PIN_HEAD_THICKNESS / 2.0, 0.0),
            rpy=CYLINDER_Y.rpy,
        ),
        material=material,
        name=f"{prefix}_root_head",
    )
    link.visual(
        Box((beam_length, CENTER_BEAM_WIDTH, LINK_SECTION_DEPTH)),
        origin=Origin(xyz=(beam_center_x, BEAM_Y_CENTER, 0.0)),
        material=material,
        name=f"{prefix}_beam",
    )
    link.visual(
        Box((beam_length - 0.010, RIB_WIDTH, RIB_THICKNESS)),
        origin=Origin(xyz=(beam_center_x, BEAM_Y_CENTER, 0.0115)),
        material=material,
        name=f"{prefix}_top_rib",
    )
    link.visual(
        Box((beam_length - 0.010, RIB_WIDTH, RIB_THICKNESS)),
        origin=Origin(xyz=(beam_center_x, BEAM_Y_CENTER, -0.0115)),
        material=material,
        name=f"{prefix}_bottom_rib",
    )
    link.visual(
        Box((DISTAL_BLOCK_LENGTH, DISTAL_BLOCK_WIDTH, LINK_SHOULDER_DEPTH)),
        origin=Origin(xyz=(length - DISTAL_BLOCK_LENGTH / 2.0, -DISTAL_BLOCK_WIDTH / 2.0, 0.0)),
        material=material,
        name=f"{prefix}_distal_block",
    )
    link.visual(
        Cylinder(radius=BOSS_RADIUS, length=DISTAL_BOSS_LENGTH),
        origin=Origin(xyz=(length, -DISTAL_BOSS_LENGTH / 2.0, 0.0), rpy=CYLINDER_Y.rpy),
        material=material,
        name=f"{prefix}_distal_boss",
    )
    link.visual(
        Cylinder(radius=PIN_HEAD_RADIUS, length=PIN_HEAD_THICKNESS),
        origin=Origin(xyz=(length, -DISTAL_BOSS_LENGTH - PIN_HEAD_THICKNESS / 2.0, 0.0), rpy=CYLINDER_Y.rpy),
        material=material,
        name=f"{prefix}_distal_head",
    )


def _add_end_link_visuals(link, *, length: float, material: str, prefix: str) -> None:
    beam_length, beam_center_x = _beam_dims(length)

    link.visual(
        Box((ROOT_LUG_LENGTH, ROOT_LUG_WIDTH, LINK_SHOULDER_DEPTH)),
        origin=Origin(xyz=(ROOT_LUG_LENGTH / 2.0, ROOT_LUG_WIDTH / 2.0, 0.0)),
        material=material,
        name=f"{prefix}_root_lug",
    )
    link.visual(
        Cylinder(radius=BOSS_RADIUS, length=ROOT_BOSS_LENGTH),
        origin=Origin(xyz=(0.0, ROOT_LUG_WIDTH + ROOT_BOSS_LENGTH / 2.0, 0.0), rpy=CYLINDER_Y.rpy),
        material=material,
        name=f"{prefix}_root_boss",
    )
    link.visual(
        Cylinder(radius=PIN_HEAD_RADIUS, length=PIN_HEAD_THICKNESS),
        origin=Origin(
            xyz=(0.0, ROOT_LUG_WIDTH + ROOT_BOSS_LENGTH + PIN_HEAD_THICKNESS / 2.0, 0.0),
            rpy=CYLINDER_Y.rpy,
        ),
        material=material,
        name=f"{prefix}_root_head",
    )
    link.visual(
        Box((beam_length, CENTER_BEAM_WIDTH, LINK_SECTION_DEPTH)),
        origin=Origin(xyz=(beam_center_x, BEAM_Y_CENTER, 0.0)),
        material=material,
        name=f"{prefix}_beam",
    )
    link.visual(
        Box((beam_length - 0.010, RIB_WIDTH, RIB_THICKNESS)),
        origin=Origin(xyz=(beam_center_x, BEAM_Y_CENTER, 0.0115)),
        material=material,
        name=f"{prefix}_top_rib",
    )
    link.visual(
        Box((beam_length - 0.010, RIB_WIDTH, RIB_THICKNESS)),
        origin=Origin(xyz=(beam_center_x, BEAM_Y_CENTER, -0.0115)),
        material=material,
        name=f"{prefix}_bottom_rib",
    )
    link.visual(
        Box((END_NECK_LENGTH, 0.010, 0.016)),
        origin=Origin(xyz=(length - 0.024, 0.004, 0.0)),
        material=material,
        name=f"{prefix}_terminal_neck",
    )
    link.visual(
        Box((END_TAB_LENGTH, 0.008, 0.010)),
        origin=Origin(xyz=(length - END_TAB_LENGTH / 2.0, 0.004, 0.0)),
        material=material,
        name=f"{prefix}_end_tab",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="industrial_lever_chain")

    model.material("base_steel", rgba=(0.24, 0.26, 0.29, 1.0))
    model.material("link_steel", rgba=(0.66, 0.69, 0.72, 1.0))

    base = model.part("base")
    _add_base_visuals(model, base)

    primary_link = model.part("primary_link")
    _add_link_visuals(primary_link, length=LINK_1_LENGTH, material="link_steel", prefix="primary")

    secondary_link = model.part("secondary_link")
    _add_link_visuals(secondary_link, length=LINK_2_LENGTH, material="link_steel", prefix="secondary")

    end_link = model.part("end_link")
    _add_end_link_visuals(end_link, length=LINK_3_LENGTH, material="link_steel", prefix="end")

    model.articulation(
        "base_to_primary",
        ArticulationType.REVOLUTE,
        parent=base,
        child=primary_link,
        origin=Origin(),
        axis=HINGE_AXIS,
        motion_limits=MotionLimits(lower=-0.35, upper=1.15, effort=28.0, velocity=1.4),
    )
    model.articulation(
        "primary_to_secondary",
        ArticulationType.REVOLUTE,
        parent=primary_link,
        child=secondary_link,
        origin=Origin(xyz=(LINK_1_LENGTH, 0.0, 0.0)),
        axis=HINGE_AXIS,
        motion_limits=MotionLimits(lower=-0.45, upper=1.20, effort=22.0, velocity=1.6),
    )
    model.articulation(
        "secondary_to_end",
        ArticulationType.REVOLUTE,
        parent=secondary_link,
        child=end_link,
        origin=Origin(xyz=(LINK_2_LENGTH, 0.0, 0.0)),
        axis=HINGE_AXIS,
        motion_limits=MotionLimits(lower=-0.55, upper=1.20, effort=16.0, velocity=1.8),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    primary_link = object_model.get_part("primary_link")
    secondary_link = object_model.get_part("secondary_link")
    end_link = object_model.get_part("end_link")

    base_to_primary = object_model.get_articulation("base_to_primary")
    primary_to_secondary = object_model.get_articulation("primary_to_secondary")
    secondary_to_end = object_model.get_articulation("secondary_to_end")

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
        "all_hinge_axes_parallel",
        base_to_primary.axis == primary_to_secondary.axis == secondary_to_end.axis == HINGE_AXIS,
        details=(
            f"axes were {base_to_primary.axis}, {primary_to_secondary.axis}, "
            f"{secondary_to_end.axis}; expected {HINGE_AXIS}"
        ),
    )

    ctx.expect_contact(base, primary_link, name="base_contacts_primary_link")
    ctx.expect_contact(primary_link, secondary_link, name="primary_contacts_secondary_link")
    ctx.expect_contact(secondary_link, end_link, name="secondary_contacts_end_link")

    closed_end_pos = ctx.part_world_position(end_link)
    with ctx.pose(
        {
            base_to_primary: 0.55,
            primary_to_secondary: 0.45,
            secondary_to_end: 0.30,
        }
    ):
        opened_end_pos = ctx.part_world_position(end_link)
        ctx.check(
            "positive_rotation_lifts_end_link",
            closed_end_pos is not None
            and opened_end_pos is not None
            and opened_end_pos[2] > closed_end_pos[2] + 0.080,
            details=f"closed={closed_end_pos}, opened={opened_end_pos}",
        )
        ctx.fail_if_parts_overlap_in_current_pose(name="raised_pose_has_no_overlaps")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
