from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


OUTER_LENGTH = 0.400
OUTER_DEPTH = 0.015
OUTER_HEIGHT = 0.048
OUTER_WALL = 0.0016
OUTER_REAR_STOP = 0.016
OUTER_FRONT_STOP = 0.012

MIDDLE_LENGTH = 0.310
MIDDLE_DEPTH = 0.010
MIDDLE_HEIGHT = 0.036
MIDDLE_WALL = 0.0014
MIDDLE_REAR_STOP = 0.014
MIDDLE_FRONT_STOP = 0.010

INNER_LENGTH = 0.240
INNER_DEPTH = 0.0066
INNER_HEIGHT = 0.025
INNER_WALL = 0.0012
INNER_REAR_STOP = 0.010
INNER_FRONT_STOP = 0.008

OUTER_TO_MIDDLE_HOME = 0.052
OUTER_TO_MIDDLE_TRAVEL = 0.155
MIDDLE_TO_INNER_HOME = 0.047
MIDDLE_TO_INNER_TRAVEL = 0.165

OUTER_TO_MIDDLE_Z = (OUTER_HEIGHT - MIDDLE_HEIGHT) / 2.0
MIDDLE_TO_INNER_Z = (MIDDLE_HEIGHT - INNER_HEIGHT) / 2.0

SUPPORT_PLATE_THICKNESS = 0.005
SUPPORT_PLATE_DEPTH = 0.030
SUPPORT_PLATE_HEIGHT = 0.100
SUPPORT_PLATE_Z0 = -0.018
SUPPORT_STRAP_LENGTH = 0.075
SUPPORT_STRAP_DEPTH = 0.022
SUPPORT_STRAP_THICKNESS = 0.006

RETURN_LIP_DEPTH = 0.0011
RETURN_LIP_HEIGHT = 0.008
OUTER_FLANGE_DEPTH = 0.0120
MIDDLE_FLANGE_DEPTH = 0.0086
INNER_FLANGE_DEPTH = 0.0050
MIDDLE_RETURN_REACH = 0.0023
INNER_RETURN_REACH = 0.0020

MIDDLE_PAD_START = 0.030
MIDDLE_PAD_LENGTH = 0.150
MIDDLE_PAD_HEIGHT = 0.0045

INNER_PAD_START = 0.025
INNER_PAD_LENGTH = 0.070
INNER_PAD_HEIGHT = 0.0035


def _box_at(size: tuple[float, float, float], xyz_min: tuple[float, float, float]) -> cq.Workplane:
    sx, sy, sz = size
    x0, y0, z0 = xyz_min
    return cq.Workplane("XY").box(sx, sy, sz, centered=(False, False, False)).translate((x0, y0, z0))


def _cylinder_x(radius: float, length: float, x0: float, y: float, z: float) -> cq.Workplane:
    return cq.Workplane("YZ").circle(radius).extrude(length).translate((x0, y, z))


def _profile_x(length: float, yz_points: list[tuple[float, float]]) -> cq.Workplane:
    return cq.Workplane("YZ").polyline(yz_points).close().extrude(length)


def _rear_support_shape() -> cq.Workplane:
    plate = _box_at(
        (SUPPORT_PLATE_THICKNESS, SUPPORT_PLATE_DEPTH, SUPPORT_PLATE_HEIGHT),
        (-SUPPORT_PLATE_THICKNESS, -SUPPORT_PLATE_DEPTH / 2.0, SUPPORT_PLATE_Z0),
    )
    upper_strap = _box_at(
        (SUPPORT_STRAP_LENGTH, SUPPORT_STRAP_DEPTH, SUPPORT_STRAP_THICKNESS),
        (-SUPPORT_STRAP_LENGTH, -SUPPORT_STRAP_DEPTH / 2.0, OUTER_HEIGHT),
    )
    lower_strap = _box_at(
        (SUPPORT_STRAP_LENGTH, SUPPORT_STRAP_DEPTH, SUPPORT_STRAP_THICKNESS),
        (-SUPPORT_STRAP_LENGTH, -SUPPORT_STRAP_DEPTH / 2.0, -SUPPORT_STRAP_THICKNESS),
    )
    upper_side_gusset = _box_at((0.030, 0.003, 0.036), (-0.030, 0.008, 0.006))
    lower_side_gusset = _box_at((0.030, 0.003, 0.036), (-0.030, -0.011, 0.006))

    support = (
        plate.union(upper_strap)
        .union(lower_strap)
        .union(upper_side_gusset)
        .union(lower_side_gusset)
    )

    top_hole = _cylinder_x(0.004, 0.008, -0.006, 0.0, 0.056)
    bottom_hole = _cylinder_x(0.004, 0.008, -0.006, 0.0, 0.012)
    return support.cut(top_hole).cut(bottom_hole)


def _outer_body_shape() -> cq.Workplane:
    profile = [
        (-0.0075, 0.0),
        (-0.0030, 0.0),
        (-0.0030, OUTER_WALL),
        (-0.0060, OUTER_WALL),
        (-0.0060, OUTER_HEIGHT - OUTER_WALL),
        (-0.0030, OUTER_HEIGHT - OUTER_WALL),
        (-0.0030, OUTER_HEIGHT),
        (-0.0075, OUTER_HEIGHT),
    ]
    body = _profile_x(OUTER_LENGTH, profile)
    bridge_mount = _box_at((0.018, 0.0035, 0.020), (0.0, -0.0065, 0.014))
    return body.union(bridge_mount)


def _middle_stage_shape() -> cq.Workplane:
    profile = [
        (-0.0030, 0.0),
        (0.0016, 0.0),
        (0.0016, MIDDLE_WALL),
        (-0.0018, MIDDLE_WALL),
        (-0.0018, MIDDLE_HEIGHT - MIDDLE_WALL),
        (0.0016, MIDDLE_HEIGHT - MIDDLE_WALL),
        (0.0016, MIDDLE_HEIGHT),
        (-0.0030, MIDDLE_HEIGHT),
    ]
    body = _profile_x(MIDDLE_LENGTH, profile)
    lower_runner = _box_at((MIDDLE_LENGTH - 0.050, 0.0030, 0.0030), (0.025, -0.0060, 0.0100))
    upper_runner = _box_at((MIDDLE_LENGTH - 0.050, 0.0030, 0.0030), (0.025, -0.0060, 0.0230))
    return body.union(lower_runner).union(upper_runner)


def _inner_stage_shape() -> cq.Workplane:
    profile = [
        (0.0016, 0.0),
        (0.0050, 0.0),
        (0.0050, INNER_WALL),
        (0.0028, INNER_WALL),
        (0.0028, INNER_HEIGHT - INNER_WALL),
        (0.0050, INNER_HEIGHT - INNER_WALL),
        (0.0050, INNER_HEIGHT),
        (0.0016, INNER_HEIGHT),
    ]
    nose_plate = _box_at((0.014, 0.0034, INNER_HEIGHT), (INNER_LENGTH - 0.004, 0.0016, 0.0))
    body = _profile_x(INNER_LENGTH, profile)
    lower_runner = _box_at((INNER_LENGTH - 0.040, 0.0034, 0.0026), (0.020, -0.0018, 0.0132))
    upper_runner = _box_at((INNER_LENGTH - 0.040, 0.0034, 0.0026), (0.020, -0.0018, 0.0192))
    return body.union(nose_plate).union(lower_runner).union(upper_runner)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bridge_backed_three_stage_extension_slide")

    model.material("support_zinc", rgba=(0.82, 0.84, 0.86, 1.0))
    model.material("outer_steel", rgba=(0.30, 0.32, 0.35, 1.0))
    model.material("middle_steel", rgba=(0.55, 0.58, 0.61, 1.0))
    model.material("inner_steel", rgba=(0.73, 0.75, 0.78, 1.0))

    rear_support = model.part("rear_support")
    rear_support.visual(
        mesh_from_cadquery(_rear_support_shape(), "rear_support"),
        material="support_zinc",
        name="support_bracket",
    )

    outer_body = model.part("outer_body")
    outer_body.visual(
        mesh_from_cadquery(_outer_body_shape(), "outer_body"),
        material="outer_steel",
        name="outer_channel",
    )

    middle_stage = model.part("middle_stage")
    middle_stage.visual(
        mesh_from_cadquery(_middle_stage_shape(), "middle_stage"),
        material="middle_steel",
        name="middle_channel",
    )

    inner_stage = model.part("inner_stage")
    inner_stage.visual(
        mesh_from_cadquery(_inner_stage_shape(), "inner_stage"),
        material="inner_steel",
        name="inner_channel",
    )

    model.articulation(
        "support_to_outer",
        ArticulationType.FIXED,
        parent=rear_support,
        child=outer_body,
        origin=Origin(),
    )
    model.articulation(
        "outer_to_middle",
        ArticulationType.PRISMATIC,
        parent=outer_body,
        child=middle_stage,
        origin=Origin(xyz=(OUTER_TO_MIDDLE_HOME, 0.0, OUTER_TO_MIDDLE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.45,
            lower=0.0,
            upper=OUTER_TO_MIDDLE_TRAVEL,
        ),
    )
    model.articulation(
        "middle_to_inner",
        ArticulationType.PRISMATIC,
        parent=middle_stage,
        child=inner_stage,
        origin=Origin(xyz=(MIDDLE_TO_INNER_HOME, 0.0, MIDDLE_TO_INNER_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=90.0,
            velocity=0.50,
            lower=0.0,
            upper=MIDDLE_TO_INNER_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    rear_support = object_model.get_part("rear_support")
    outer_body = object_model.get_part("outer_body")
    middle_stage = object_model.get_part("middle_stage")
    inner_stage = object_model.get_part("inner_stage")
    support_to_outer = object_model.get_articulation("support_to_outer")
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
        "serial_joint_types_are_correct",
        support_to_outer.articulation_type == ArticulationType.FIXED
        and outer_to_middle.articulation_type == ArticulationType.PRISMATIC
        and middle_to_inner.articulation_type == ArticulationType.PRISMATIC,
        "Rear support should fix the outer body and the two nested stages should be prismatic.",
    )
    ctx.check(
        "prismatic_stages_share_axis",
        tuple(outer_to_middle.axis) == (1.0, 0.0, 0.0) and tuple(middle_to_inner.axis) == (1.0, 0.0, 0.0),
        "Both nested stages should extend along the same +X slide axis.",
    )

    ctx.expect_contact(
        rear_support,
        outer_body,
        contact_tol=6e-4,
        name="rear_support_physically_carries_outer_body",
    )
    ctx.expect_contact(
        outer_body,
        middle_stage,
        contact_tol=6e-4,
        name="outer_body_guides_middle_stage",
    )
    ctx.expect_contact(
        middle_stage,
        inner_stage,
        contact_tol=6e-4,
        name="middle_stage_guides_inner_stage",
    )

    with ctx.pose({outer_to_middle: 0.0, middle_to_inner: 0.0}):
        ctx.expect_within(
            middle_stage,
            outer_body,
            axes="z",
            margin=0.0,
            name="middle_stage_stays_vertically_nested_in_outer_body_at_home",
        )
        ctx.expect_within(
            inner_stage,
            middle_stage,
            axes="z",
            margin=0.0,
            name="inner_stage_stays_vertically_nested_in_middle_stage_at_home",
        )
        ctx.expect_overlap(
            middle_stage,
            outer_body,
            axes="x",
            min_overlap=0.28,
            name="outer_and_middle_have_clear_home_overlap",
        )
        ctx.expect_overlap(
            inner_stage,
            middle_stage,
            axes="x",
            min_overlap=0.20,
            name="middle_and_inner_have_clear_home_overlap",
        )
        outer_aabb = ctx.part_world_aabb(outer_body)
        middle_aabb = ctx.part_world_aabb(middle_stage)
        inner_aabb = ctx.part_world_aabb(inner_stage)
        ctx.check(
            "nested_channels_step_toward_open_side_at_home",
            outer_aabb is not None
            and middle_aabb is not None
            and inner_aabb is not None
            and outer_aabb[1][1] < middle_aabb[1][1] < inner_aabb[1][1],
            "The three channels should stair-step toward the open side, reading as nested opposing rails.",
        )

    home_middle_x = ctx.part_world_position(middle_stage)[0]
    home_inner_x = ctx.part_world_position(inner_stage)[0]

    with ctx.pose({outer_to_middle: OUTER_TO_MIDDLE_TRAVEL}):
        extended_middle_x = ctx.part_world_position(middle_stage)[0]
        ctx.expect_within(
            middle_stage,
            outer_body,
            axes="z",
            margin=0.0,
            name="middle_stage_remains_vertically_captured_when_extended",
        )
        ctx.expect_overlap(
            middle_stage,
            outer_body,
            axes="x",
            min_overlap=0.18,
            name="outer_and_middle_keep_running_overlap_when_extended",
        )

    with ctx.pose({outer_to_middle: OUTER_TO_MIDDLE_TRAVEL, middle_to_inner: MIDDLE_TO_INNER_TRAVEL}):
        extended_inner_x = ctx.part_world_position(inner_stage)[0]
        ctx.expect_within(
            inner_stage,
            middle_stage,
            axes="z",
            margin=0.0,
            name="inner_stage_remains_vertically_captured_when_fully_extended",
        )
        ctx.expect_overlap(
            inner_stage,
            middle_stage,
            axes="x",
            min_overlap=0.08,
            name="middle_and_inner_keep_running_overlap_when_fully_extended",
        )

    ctx.check(
        "middle_stage_extends_forward",
        extended_middle_x > home_middle_x + 0.14,
        "Positive outer_to_middle travel should move the middle stage forward along +X.",
    )
    ctx.check(
        "inner_stage_extends_forward",
        extended_inner_x > home_inner_x + 0.30,
        "Combined extension should move the terminal inner stage forward along +X.",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
