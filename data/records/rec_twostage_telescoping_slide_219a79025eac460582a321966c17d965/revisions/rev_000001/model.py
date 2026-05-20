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


OUTER_LENGTH = 0.340
OUTER_WIDTH = 0.016
OUTER_HEIGHT = 0.045
OUTER_WALL = 0.0018

INNER_LENGTH = 0.320
INNER_WIDTH = 0.010
INNER_HEIGHT = 0.032
INNER_WALL = 0.0016

STAGE_HOME_OFFSET = 0.028
STAGE_TRAVEL = 0.190

OUTER_REAR_X = 0.056

SUPPORT_PLATE_T = 0.006
SUPPORT_PLATE_W = 0.028
SUPPORT_PLATE_H = 0.090
SUPPORT_FRONT_PAD_T = 0.008
SUPPORT_RIB_LEN = OUTER_REAR_X - SUPPORT_FRONT_PAD_T
SUPPORT_RIB_W = 0.020
SUPPORT_RIB_H = 0.010
SUPPORT_RIB_Z = 0.024
SUPPORT_SHELF_LEN = 0.018
SUPPORT_SHELF_W = 0.012
SUPPORT_SHELF_T = 0.004


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _rear_support_shape() -> cq.Workplane:
    plate = _box(
        (SUPPORT_PLATE_T, SUPPORT_PLATE_W, SUPPORT_PLATE_H),
        (-SUPPORT_PLATE_T / 2.0, 0.0, 0.0),
    )
    upper_rib = _box(
        (SUPPORT_RIB_LEN, SUPPORT_RIB_W, SUPPORT_RIB_H),
        (SUPPORT_RIB_LEN / 2.0, 0.0, SUPPORT_RIB_Z),
    )
    lower_rib = _box(
        (SUPPORT_RIB_LEN, SUPPORT_RIB_W, SUPPORT_RIB_H),
        (SUPPORT_RIB_LEN / 2.0, 0.0, -SUPPORT_RIB_Z),
    )
    front_pad = _box(
        (SUPPORT_FRONT_PAD_T, OUTER_WIDTH, OUTER_HEIGHT),
        (OUTER_REAR_X - SUPPORT_FRONT_PAD_T / 2.0, 0.0, 0.0),
    )
    shelf = _box(
        (SUPPORT_SHELF_LEN, SUPPORT_SHELF_W, SUPPORT_SHELF_T),
        (
            OUTER_REAR_X + SUPPORT_SHELF_LEN / 2.0,
            0.0,
            -OUTER_HEIGHT / 2.0 - SUPPORT_SHELF_T / 2.0,
        ),
    )
    support = plate.union(upper_rib).union(lower_rib).union(front_pad).union(shelf)

    slot_size = (SUPPORT_PLATE_T * 2.2, 0.007, 0.020)
    for slot_z in (-0.026, 0.026):
        support = support.cut(_box(slot_size, (-SUPPORT_PLATE_T / 2.0, 0.0, slot_z)))

    return support


def _channel_shape(
    *,
    length: float,
    width: float,
    height: float,
    wall: float,
    web_side: str,
) -> cq.Workplane:
    web_y = -width / 2.0 + wall / 2.0 if web_side == "negative" else width / 2.0 - wall / 2.0
    web = _box((length, wall, height), (length / 2.0, web_y, 0.0))
    top = _box((length, width, wall), (length / 2.0, 0.0, height / 2.0 - wall / 2.0))
    bottom = _box((length, width, wall), (length / 2.0, 0.0, -height / 2.0 + wall / 2.0))
    return web.union(top).union(bottom)


def _outer_slide_shape() -> cq.Workplane:
    outer = _channel_shape(
        length=OUTER_LENGTH,
        width=OUTER_WIDTH,
        height=OUTER_HEIGHT,
        wall=OUTER_WALL,
        web_side="negative",
    )
    rear_doubler = _box(
        (0.030, OUTER_WALL * 1.6, 0.026),
        (0.015, -OUTER_WIDTH / 2.0 - OUTER_WALL * 0.3, 0.0),
    )
    nose = _box(
        (0.010, OUTER_WIDTH * 0.72, OUTER_WALL * 1.8),
        (OUTER_LENGTH - 0.005, 0.0, -OUTER_HEIGHT / 2.0 + OUTER_WALL * 0.9),
    )
    outer = outer.union(rear_doubler).union(nose)

    slot_y = -OUTER_WIDTH / 2.0 + OUTER_WALL / 2.0
    for slot_x in (0.100, 0.245):
        outer = outer.cut(_box((0.058, OUTER_WALL * 2.6, 0.010), (slot_x, slot_y, 0.0)))

    return outer


def _inner_stage_shape() -> cq.Workplane:
    inner = _channel_shape(
        length=INNER_LENGTH,
        width=INNER_WIDTH,
        height=INNER_HEIGHT,
        wall=INNER_WALL,
        web_side="positive",
    )
    front_plate = _box((0.004, 0.012, 0.034), (INNER_LENGTH - 0.002, 0.0, 0.0))
    front_runner = _box(
        (0.032, INNER_WIDTH * 0.72, INNER_WALL * 1.8),
        (INNER_LENGTH - 0.016, 0.0, -INNER_HEIGHT / 2.0 + INNER_WALL * 0.9),
    )
    inner = inner.union(front_plate).union(front_runner)

    for guide_x in (0.038, 0.092):
        guide_arm = _box((0.010, 0.0008, 0.007), (guide_x, -0.0048, -0.0115))
        guide_ball = (
            cq.Workplane("XY")
            .sphere(0.001)
            .translate((guide_x, -0.0052, -0.0102))
        )
        inner = inner.union(guide_arm).union(guide_ball)

    slot_y = INNER_WIDTH / 2.0 - INNER_WALL / 2.0
    for slot_x in (0.095, 0.205):
        inner = inner.cut(_box((0.048, INNER_WALL * 2.4, 0.009), (slot_x, slot_y, 0.0)))

    return inner


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bridge_backed_extension_slide")

    model.material("support_zinc", rgba=(0.80, 0.82, 0.85, 1.0))
    model.material("outer_steel", rgba=(0.46, 0.49, 0.53, 1.0))
    model.material("inner_steel", rgba=(0.72, 0.74, 0.77, 1.0))

    rear_support = model.part("rear_support")
    rear_support.visual(
        mesh_from_cadquery(_rear_support_shape(), "rear_support"),
        material="support_zinc",
        name="support_shell",
    )

    outer_slide = model.part("outer_slide")
    outer_slide.visual(
        mesh_from_cadquery(_outer_slide_shape(), "outer_slide"),
        material="outer_steel",
        name="outer_shell",
    )

    inner_stage = model.part("inner_stage")
    inner_stage.visual(
        mesh_from_cadquery(_inner_stage_shape(), "inner_stage"),
        material="inner_steel",
        name="stage_shell",
    )

    model.articulation(
        "support_to_outer",
        ArticulationType.FIXED,
        parent=rear_support,
        child=outer_slide,
        origin=Origin(xyz=(OUTER_REAR_X, 0.0, 0.0)),
    )
    model.articulation(
        "outer_to_inner_stage",
        ArticulationType.PRISMATIC,
        parent=outer_slide,
        child=inner_stage,
        origin=Origin(xyz=(STAGE_HOME_OFFSET, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=STAGE_TRAVEL,
            effort=120.0,
            velocity=0.35,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    rear_support = object_model.get_part("rear_support")
    outer_slide = object_model.get_part("outer_slide")
    inner_stage = object_model.get_part("inner_stage")
    stage_slide = object_model.get_articulation("outer_to_inner_stage")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts(contact_tol=5e-06)
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

    limits = stage_slide.motion_limits
    ctx.check(
        "stage_joint_is_prismatic",
        stage_slide.articulation_type == ArticulationType.PRISMATIC,
        details=f"expected PRISMATIC, got {stage_slide.articulation_type}",
    )
    ctx.check(
        "stage_joint_axis_is_shared_x",
        tuple(float(v) for v in stage_slide.axis) == (1.0, 0.0, 0.0),
        details=f"expected axis (1, 0, 0), got {stage_slide.axis}",
    )
    ctx.check(
        "stage_joint_limits_match_extension",
        limits is not None and limits.lower == 0.0 and limits.upper == STAGE_TRAVEL,
        details=f"unexpected limits: {limits}",
    )

    ctx.expect_contact(rear_support, outer_slide, name="rear_support_carries_outer_slide")
    ctx.expect_contact(
        outer_slide,
        inner_stage,
        contact_tol=5e-06,
        name="inner_stage_supported_by_outer_slide",
    )
    ctx.expect_gap(
        inner_stage,
        rear_support,
        axis="x",
        min_gap=0.008,
        name="inner_stage_clears_rear_support_at_home",
    )

    with ctx.pose({stage_slide: 0.0}):
        ctx.expect_within(
            inner_stage,
            outer_slide,
            axes="yz",
            margin=0.0015,
            name="inner_stage_nested_in_outer_at_home",
        )
        ctx.expect_origin_gap(
            inner_stage,
            outer_slide,
            axis="x",
            min_gap=STAGE_HOME_OFFSET - 0.001,
            max_gap=STAGE_HOME_OFFSET + 0.001,
            name="stage_home_offset",
        )

    with ctx.pose({stage_slide: STAGE_TRAVEL}):
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_at_full_extension")
        ctx.expect_contact(
            outer_slide,
            inner_stage,
            contact_tol=5e-06,
            name="inner_stage_remains_supported_when_extended",
        )
        ctx.expect_within(
            inner_stage,
            outer_slide,
            axes="yz",
            margin=0.0015,
            name="inner_stage_stays_on_shared_axis_when_extended",
        )
        ctx.expect_origin_gap(
            inner_stage,
            outer_slide,
            axis="x",
            min_gap=STAGE_HOME_OFFSET + STAGE_TRAVEL - 0.001,
            max_gap=STAGE_HOME_OFFSET + STAGE_TRAVEL + 0.001,
            name="stage_full_extension_offset",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
