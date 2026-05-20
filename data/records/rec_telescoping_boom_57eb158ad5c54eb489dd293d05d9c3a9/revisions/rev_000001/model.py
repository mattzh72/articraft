from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


OUTER_LENGTH = 0.95
OUTER_WIDTH = 0.09
OUTER_HEIGHT = 0.10
OUTER_WALL = 0.006

MIDDLE_LENGTH = 0.78
MIDDLE_WIDTH = 0.066
MIDDLE_HEIGHT = 0.074
MIDDLE_WALL = 0.005

INNER_LENGTH = 0.61
INNER_WIDTH = 0.048
INNER_HEIGHT = 0.040
INNER_WALL = 0.004

REAR_CAP = 0.012
FRONT_CAP = 0.012

OUTER_TO_MIDDLE_HOME = 0.14
OUTER_TO_MIDDLE_TRAVEL = 0.42
MIDDLE_TO_INNER_HOME = 0.12
MIDDLE_TO_INNER_TRAVEL = 0.35

SUPPORT_BEAM_LENGTH = 0.60
SUPPORT_BEAM_WIDTH = 0.18
SUPPORT_BEAM_HEIGHT = 0.07
SUPPORT_BEAM_X = -0.02
SUPPORT_BEAM_Z = 0.215

SADDLE_LENGTH = 0.30
SADDLE_WIDTH = 0.106
SADDLE_THICKNESS = 0.02
SADDLE_X = 0.07
SADDLE_Z = OUTER_HEIGHT / 2.0 + SADDLE_THICKNESS / 2.0

CRADLE_PLATE_LENGTH = 0.32
CRADLE_PLATE_THICKNESS = 0.012
CRADLE_PLATE_Z = 0.1275
CRADLE_PLATE_HEIGHT = 0.115
CRADLE_SIDE_Y = 0.059
CRADLE_END_THICKNESS = 0.012
CRADLE_END_FRONT_X = 0.06
CRADLE_END_REAR_X = 0.36

def _add_box_visual(
    part,
    *,
    size: tuple[float, float, float],
    xyz: tuple[float, float, float],
    material: str,
    name: str,
) -> None:
    part.visual(Box(size), origin=Origin(xyz=xyz), material=material, name=name)


def _add_support_visuals(part, *, material: str) -> None:
    _add_box_visual(
        part,
        size=(SUPPORT_BEAM_LENGTH, SUPPORT_BEAM_WIDTH, SUPPORT_BEAM_HEIGHT),
        xyz=(SUPPORT_BEAM_X + SUPPORT_BEAM_LENGTH / 2.0, 0.0, SUPPORT_BEAM_Z),
        material=material,
        name="support_beam",
    )
    _add_box_visual(
        part,
        size=(SADDLE_LENGTH, SADDLE_WIDTH, SADDLE_THICKNESS),
        xyz=(SADDLE_X + SADDLE_LENGTH / 2.0, 0.0, SADDLE_Z),
        material=material,
        name="support_saddle",
    )
    _add_box_visual(
        part,
        size=(CRADLE_PLATE_LENGTH, CRADLE_PLATE_THICKNESS, CRADLE_PLATE_HEIGHT),
        xyz=(SADDLE_X + CRADLE_PLATE_LENGTH / 2.0, CRADLE_SIDE_Y, CRADLE_PLATE_Z),
        material=material,
        name="left_cradle_plate",
    )
    _add_box_visual(
        part,
        size=(CRADLE_PLATE_LENGTH, CRADLE_PLATE_THICKNESS, CRADLE_PLATE_HEIGHT),
        xyz=(SADDLE_X + CRADLE_PLATE_LENGTH / 2.0, -CRADLE_SIDE_Y, CRADLE_PLATE_Z),
        material=material,
        name="right_cradle_plate",
    )
    _add_box_visual(
        part,
        size=(CRADLE_END_THICKNESS, SADDLE_WIDTH, CRADLE_PLATE_HEIGHT),
        xyz=(CRADLE_END_FRONT_X + CRADLE_END_THICKNESS / 2.0, 0.0, CRADLE_PLATE_Z),
        material=material,
        name="front_cradle_web",
    )
    _add_box_visual(
        part,
        size=(CRADLE_END_THICKNESS, SADDLE_WIDTH, CRADLE_PLATE_HEIGHT),
        xyz=(CRADLE_END_REAR_X + CRADLE_END_THICKNESS / 2.0, 0.0, CRADLE_PLATE_Z),
        material=material,
        name="rear_cradle_web",
    )


def _add_channel_visuals(
    part,
    *,
    length: float,
    width: float,
    height: float,
    wall: float,
    material: str,
    prefix: str,
    rear_cap: bool = False,
    front_cap: bool = False,
) -> None:
    web_height = height - wall
    _add_box_visual(
        part,
        size=(length, width, wall),
        xyz=(length / 2.0, 0.0, height / 2.0 - wall / 2.0),
        material=material,
        name=f"{prefix}_top_plate",
    )
    _add_box_visual(
        part,
        size=(length, wall, web_height),
        xyz=(length / 2.0, width / 2.0 - wall / 2.0, -wall / 2.0),
        material=material,
        name=f"{prefix}_left_web",
    )
    _add_box_visual(
        part,
        size=(length, wall, web_height),
        xyz=(length / 2.0, -width / 2.0 + wall / 2.0, -wall / 2.0),
        material=material,
        name=f"{prefix}_right_web",
    )
    if rear_cap:
        _add_box_visual(
            part,
            size=(REAR_CAP, width, height),
            xyz=(REAR_CAP / 2.0, 0.0, 0.0),
            material=material,
            name=f"{prefix}_rear_cap",
        )
    if front_cap:
        _add_box_visual(
            part,
            size=(FRONT_CAP, width, height),
            xyz=(length - FRONT_CAP / 2.0, 0.0, 0.0),
            material=material,
            name=f"{prefix}_front_cap",
        )


def _add_top_wear_pads(
    part,
    *,
    stage_length: float,
    stage_width: float,
    stage_top_z: float,
    pad_thickness: float,
    material: str,
    prefix: str,
) -> None:
    if pad_thickness <= 0.0:
        return
    pad_length = min(0.11, stage_length * 0.18)
    pad_width = stage_width * 0.18
    y_offset = stage_width * 0.24
    for idx, x_center in enumerate((0.11 * stage_length, 0.31 * stage_length), start=1):
        _add_box_visual(
            part,
            size=(pad_length, pad_width, pad_thickness),
            xyz=(x_center, y_offset, stage_top_z + pad_thickness / 2.0),
            material=material,
            name=f"{prefix}_pad_left_{idx}",
        )
        _add_box_visual(
            part,
            size=(pad_length, pad_width, pad_thickness),
            xyz=(x_center, -y_offset, stage_top_z + pad_thickness / 2.0),
            material=material,
            name=f"{prefix}_pad_right_{idx}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="underslung_telescoping_beam")

    model.material("support_steel", rgba=(0.22, 0.24, 0.27, 1.0))
    model.material("outer_paint", rgba=(0.77, 0.66, 0.19, 1.0))
    model.material("middle_paint", rgba=(0.83, 0.73, 0.28, 1.0))
    model.material("inner_paint", rgba=(0.72, 0.74, 0.76, 1.0))

    support = model.part("top_support")
    _add_support_visuals(support, material="support_steel")
    support.inertial = Inertial.from_geometry(
        Box((0.60, 0.18, 0.20)),
        mass=22.0,
        origin=Origin(xyz=(0.28, 0.0, 0.15)),
    )

    outer = model.part("outer_stage")
    _add_channel_visuals(
        outer,
        length=OUTER_LENGTH,
        width=OUTER_WIDTH,
        height=OUTER_HEIGHT,
        wall=OUTER_WALL,
        material="outer_paint",
        prefix="outer_stage",
        rear_cap=True,
    )
    outer.inertial = Inertial.from_geometry(
        Box((OUTER_LENGTH, OUTER_WIDTH, OUTER_HEIGHT)),
        mass=14.0,
        origin=Origin(xyz=(OUTER_LENGTH / 2.0, 0.0, 0.0)),
    )

    middle = model.part("middle_stage")
    _add_channel_visuals(
        middle,
        length=MIDDLE_LENGTH,
        width=MIDDLE_WIDTH,
        height=MIDDLE_HEIGHT,
        wall=MIDDLE_WALL,
        material="middle_paint",
        prefix="middle_stage",
        rear_cap=True,
    )
    middle.inertial = Inertial.from_geometry(
        Box((MIDDLE_LENGTH, MIDDLE_WIDTH, MIDDLE_HEIGHT)),
        mass=10.0,
        origin=Origin(xyz=(MIDDLE_LENGTH / 2.0, 0.0, 0.0)),
    )

    inner = model.part("inner_stage")
    _add_channel_visuals(
        inner,
        length=INNER_LENGTH,
        width=INNER_WIDTH,
        height=INNER_HEIGHT,
        wall=INNER_WALL,
        material="inner_paint",
        prefix="inner_stage",
        front_cap=True,
    )
    inner.inertial = Inertial.from_geometry(
        Box((INNER_LENGTH, INNER_WIDTH, INNER_HEIGHT)),
        mass=6.5,
        origin=Origin(xyz=(INNER_LENGTH / 2.0, 0.0, 0.0)),
    )

    model.articulation(
        "support_to_outer",
        ArticulationType.FIXED,
        parent=support,
        child=outer,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )
    model.articulation(
        "outer_to_middle",
        ArticulationType.PRISMATIC,
        parent=outer,
        child=middle,
        origin=Origin(xyz=(OUTER_TO_MIDDLE_HOME, 0.0, 0.007)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1800.0,
            velocity=0.35,
            lower=0.0,
            upper=OUTER_TO_MIDDLE_TRAVEL,
        ),
    )
    model.articulation(
        "middle_to_inner",
        ArticulationType.PRISMATIC,
        parent=middle,
        child=inner,
        origin=Origin(xyz=(MIDDLE_TO_INNER_HOME, 0.0, 0.012)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1200.0,
            velocity=0.35,
            lower=0.0,
            upper=MIDDLE_TO_INNER_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support = object_model.get_part("top_support")
    outer = object_model.get_part("outer_stage")
    middle = object_model.get_part("middle_stage")
    inner = object_model.get_part("inner_stage")
    outer_to_middle = object_model.get_articulation("outer_to_middle")
    middle_to_inner = object_model.get_articulation("middle_to_inner")

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
        "prismatic_axes_are_aligned_with_beam",
        tuple(outer_to_middle.axis) == (1.0, 0.0, 0.0)
        and tuple(middle_to_inner.axis) == (1.0, 0.0, 0.0),
        f"Expected both stage joints to slide along +X, got {outer_to_middle.axis} and {middle_to_inner.axis}.",
    )
    ctx.check(
        "prismatic_limits_are_positive_extension_only",
        outer_to_middle.motion_limits is not None
        and middle_to_inner.motion_limits is not None
        and outer_to_middle.motion_limits.lower == 0.0
        and outer_to_middle.motion_limits.upper == OUTER_TO_MIDDLE_TRAVEL
        and middle_to_inner.motion_limits.lower == 0.0
        and middle_to_inner.motion_limits.upper == MIDDLE_TO_INNER_TRAVEL,
        "The telescoping stages should retract at q=0 and extend only in the positive beam direction.",
    )
    ctx.expect_gap(
        support,
        outer,
        axis="z",
        min_gap=0.0,
        max_gap=0.0001,
        name="top_support_carries_outer_from_above",
    )
    ctx.expect_contact(support, outer, name="support_cradle_contacts_outer_stage")
    ctx.expect_contact(outer, middle, name="outer_stage_supports_middle_stage")
    ctx.expect_contact(middle, inner, name="middle_stage_supports_inner_stage")
    ctx.expect_within(
        middle,
        outer,
        axes="yz",
        margin=0.0,
        name="middle_stage_stays_nested_within_outer_profile",
    )
    ctx.expect_within(
        inner,
        middle,
        axes="yz",
        margin=0.0,
        name="inner_stage_stays_nested_within_middle_profile",
    )

    with ctx.pose({outer_to_middle: OUTER_TO_MIDDLE_TRAVEL, middle_to_inner: MIDDLE_TO_INNER_TRAVEL}):
        ctx.expect_contact(outer, middle, name="outer_stage_keeps_middle_supported_at_full_extension")
        ctx.expect_contact(middle, inner, name="middle_stage_keeps_inner_supported_at_full_extension")
        ctx.expect_within(
            middle,
            outer,
            axes="yz",
            margin=0.0,
            name="middle_stage_remains_guided_when_extended",
        )
        ctx.expect_within(
            inner,
            middle,
            axes="yz",
            margin=0.0,
            name="inner_stage_remains_guided_when_extended",
        )

    with ctx.pose({outer_to_middle: 0.0, middle_to_inner: 0.0}):
        middle_x_retracted = ctx.part_world_position(middle)[0]
        inner_x_retracted = ctx.part_world_position(inner)[0]
    with ctx.pose({outer_to_middle: OUTER_TO_MIDDLE_TRAVEL, middle_to_inner: MIDDLE_TO_INNER_TRAVEL}):
        middle_x_extended = ctx.part_world_position(middle)[0]
        inner_x_extended = ctx.part_world_position(inner)[0]

    ctx.check(
        "middle_stage_extends_in_positive_x",
        middle_x_extended > middle_x_retracted + OUTER_TO_MIDDLE_TRAVEL - 1e-6,
        f"Middle stage should advance by {OUTER_TO_MIDDLE_TRAVEL:.3f} m along +X, got {middle_x_extended - middle_x_retracted:.6f} m.",
    )
    ctx.check(
        "inner_stage_advances_through_both_prismatic_stages",
        inner_x_extended
        > inner_x_retracted + OUTER_TO_MIDDLE_TRAVEL + MIDDLE_TO_INNER_TRAVEL - 1e-6,
        (
            "Inner stage should inherit the outer-stage travel and add its own travel; "
            f"got {inner_x_extended - inner_x_retracted:.6f} m total."
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
