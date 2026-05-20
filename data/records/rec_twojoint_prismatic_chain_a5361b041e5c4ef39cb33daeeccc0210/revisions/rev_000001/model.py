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


THICKNESS = 0.003

SUPPORT_LENGTH = 0.86
SUPPORT_WIDTH = 0.090
SUPPORT_TOP_THICKNESS = 0.008
SUPPORT_SIDE_WIDTH = 0.008
SUPPORT_SIDE_DROP = 0.016
SUPPORT_END_BRIDGE_LENGTH = 0.024
SUPPORT_END_BRIDGE_HEIGHT = 0.012

OUTER_BODY_LENGTH = 0.46
OUTER_TOP_WIDTH = 0.074
OUTER_TOP_THICKNESS = 0.006
OUTER_SIDE_WIDTH = 0.008
OUTER_SIDE_DROP = 0.018
OUTER_BOTTOM_WIDTH = 0.032
OUTER_BOTTOM_THICKNESS = 0.006
OUTER_BULKHEAD_LENGTH = 0.016

INNER_BODY_LENGTH = 0.32
INNER_TOP_WIDTH = 0.028
INNER_TOP_THICKNESS = 0.006
INNER_WEB_WIDTH = 0.010
INNER_WEB_DROP = 0.018
INNER_BOTTOM_WIDTH = 0.046
INNER_BOTTOM_THICKNESS = 0.006

SUPPORT_TO_OUTER_X = 0.06
SUPPORT_TO_OUTER_Z = -(SUPPORT_TOP_THICKNESS + SUPPORT_SIDE_DROP)
OUTER_TO_INNER_X = 0.04
OUTER_TO_INNER_Z = -(OUTER_SIDE_DROP + OUTER_BOTTOM_THICKNESS)
OUTER_TRAVEL = 0.24
INNER_TRAVEL = 0.16


def _x_box(
    length: float,
    width: float,
    height: float,
    *,
    x_start: float = 0.0,
    y_center: float = 0.0,
    z_center: float = 0.0,
) -> cq.Workplane:
    return cq.Workplane("XY").box(length, width, height).translate(
        (x_start + length / 2.0, y_center, z_center)
    )


def _make_support_shape() -> cq.Workplane:
    top_plate = _x_box(
        SUPPORT_LENGTH,
        SUPPORT_WIDTH,
        SUPPORT_TOP_THICKNESS,
        z_center=-(SUPPORT_TOP_THICKNESS / 2.0),
    )
    side_y = SUPPORT_WIDTH / 2.0 - SUPPORT_SIDE_WIDTH / 2.0
    side_z = -(SUPPORT_TOP_THICKNESS + SUPPORT_SIDE_DROP / 2.0)
    left_side = _x_box(
        SUPPORT_LENGTH,
        SUPPORT_SIDE_WIDTH,
        SUPPORT_SIDE_DROP,
        y_center=side_y,
        z_center=side_z,
    )
    right_side = _x_box(
        SUPPORT_LENGTH,
        SUPPORT_SIDE_WIDTH,
        SUPPORT_SIDE_DROP,
        y_center=-side_y,
        z_center=side_z,
    )
    bridge_z = -(SUPPORT_TOP_THICKNESS + SUPPORT_END_BRIDGE_HEIGHT / 2.0)
    front_bridge = _x_box(
        SUPPORT_END_BRIDGE_LENGTH,
        SUPPORT_WIDTH - 2.0 * SUPPORT_SIDE_WIDTH,
        SUPPORT_END_BRIDGE_HEIGHT,
        x_start=0.0,
        z_center=bridge_z,
    )
    rear_bridge = _x_box(
        SUPPORT_END_BRIDGE_LENGTH,
        SUPPORT_WIDTH - 2.0 * SUPPORT_SIDE_WIDTH,
        SUPPORT_END_BRIDGE_HEIGHT,
        x_start=SUPPORT_LENGTH - SUPPORT_END_BRIDGE_LENGTH,
        z_center=bridge_z,
    )
    return top_plate.union(left_side).union(right_side).union(front_bridge).union(rear_bridge)


def _make_outer_slider_shape() -> cq.Workplane:
    top_plate = _x_box(
        OUTER_BODY_LENGTH,
        OUTER_TOP_WIDTH,
        OUTER_TOP_THICKNESS,
        z_center=-(OUTER_TOP_THICKNESS / 2.0),
    )
    side_y = OUTER_TOP_WIDTH / 2.0 - OUTER_SIDE_WIDTH / 2.0
    side_height = OUTER_SIDE_DROP - OUTER_TOP_THICKNESS
    side_z = -(OUTER_TOP_THICKNESS + side_height / 2.0)
    left_side = _x_box(
        OUTER_BODY_LENGTH,
        OUTER_SIDE_WIDTH,
        side_height,
        y_center=side_y,
        z_center=side_z,
    )
    right_side = _x_box(
        OUTER_BODY_LENGTH,
        OUTER_SIDE_WIDTH,
        side_height,
        y_center=-side_y,
        z_center=side_z,
    )
    bottom_z = -(OUTER_SIDE_DROP + OUTER_BOTTOM_THICKNESS / 2.0)
    bottom_runner = _x_box(
        OUTER_BODY_LENGTH * 0.92,
        OUTER_BOTTOM_WIDTH,
        OUTER_BOTTOM_THICKNESS,
        x_start=OUTER_BODY_LENGTH * 0.04,
        z_center=bottom_z,
    )
    bulkhead_height = OUTER_SIDE_DROP - OUTER_TOP_THICKNESS
    bulkhead_z = -(OUTER_TOP_THICKNESS + bulkhead_height / 2.0)
    front_bulkhead = _x_box(
        OUTER_BULKHEAD_LENGTH,
        OUTER_BOTTOM_WIDTH,
        bulkhead_height,
        x_start=OUTER_BODY_LENGTH * 0.06,
        z_center=bulkhead_z,
    )
    rear_bulkhead = _x_box(
        OUTER_BULKHEAD_LENGTH,
        OUTER_BOTTOM_WIDTH,
        bulkhead_height,
        x_start=OUTER_BODY_LENGTH - OUTER_BODY_LENGTH * 0.06 - OUTER_BULKHEAD_LENGTH,
        z_center=bulkhead_z,
    )
    return (
        top_plate.union(left_side)
        .union(right_side)
        .union(bottom_runner)
        .union(front_bulkhead)
        .union(rear_bulkhead)
    )


def _make_inner_slider_shape() -> cq.Workplane:
    top_plate = _x_box(
        INNER_BODY_LENGTH,
        INNER_TOP_WIDTH,
        INNER_TOP_THICKNESS,
        z_center=-(INNER_TOP_THICKNESS / 2.0),
    )
    web_height = INNER_WEB_DROP - INNER_TOP_THICKNESS
    web = _x_box(
        INNER_BODY_LENGTH * 0.40,
        INNER_WEB_WIDTH,
        web_height,
        x_start=INNER_BODY_LENGTH * 0.30,
        z_center=-(INNER_TOP_THICKNESS + web_height / 2.0),
    )
    bottom_plate = _x_box(
        INNER_BODY_LENGTH * 0.72,
        INNER_BOTTOM_WIDTH,
        INNER_BOTTOM_THICKNESS,
        x_start=INNER_BODY_LENGTH * 0.14,
        z_center=-(INNER_WEB_DROP + INNER_BOTTOM_THICKNESS / 2.0),
    )
    return top_plate.union(web).union(bottom_plate)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="under_slung_two_stage_slide")

    support_mat = model.material("support_steel", rgba=(0.20, 0.22, 0.25, 1.0))
    outer_mat = model.material("outer_slider_steel", rgba=(0.67, 0.69, 0.72, 1.0))
    inner_mat = model.material("inner_slider_steel", rgba=(0.53, 0.55, 0.58, 1.0))

    top_support = model.part("top_support")
    top_support.visual(
        mesh_from_cadquery(_make_support_shape(), "top_support"),
        material=support_mat,
        name="support_shell",
    )

    outer_slider = model.part("outer_slider")
    outer_slider.visual(
        mesh_from_cadquery(_make_outer_slider_shape(), "outer_slider"),
        material=outer_mat,
        name="outer_shell",
    )

    inner_slider = model.part("inner_slider")
    inner_slider.visual(
        mesh_from_cadquery(_make_inner_slider_shape(), "inner_slider"),
        material=inner_mat,
        name="inner_shell",
    )

    model.articulation(
        "support_to_outer",
        ArticulationType.PRISMATIC,
        parent=top_support,
        child=outer_slider,
        origin=Origin(xyz=(SUPPORT_TO_OUTER_X, 0.0, SUPPORT_TO_OUTER_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=0.35,
            lower=0.0,
            upper=OUTER_TRAVEL,
        ),
    )

    model.articulation(
        "outer_to_inner",
        ArticulationType.PRISMATIC,
        parent=outer_slider,
        child=inner_slider,
        origin=Origin(xyz=(OUTER_TO_INNER_X, 0.0, OUTER_TO_INNER_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=140.0,
            velocity=0.35,
            lower=0.0,
            upper=INNER_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    top_support = object_model.get_part("top_support")
    outer_slider = object_model.get_part("outer_slider")
    inner_slider = object_model.get_part("inner_slider")
    support_to_outer = object_model.get_articulation("support_to_outer")
    outer_to_inner = object_model.get_articulation("outer_to_inner")

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
        "both joints are serial +X prismatic slides",
        (
            support_to_outer.articulation_type == ArticulationType.PRISMATIC
            and outer_to_inner.articulation_type == ArticulationType.PRISMATIC
            and support_to_outer.axis == (1.0, 0.0, 0.0)
            and outer_to_inner.axis == (1.0, 0.0, 0.0)
        ),
        details=(
            f"support_to_outer={support_to_outer.articulation_type}/{support_to_outer.axis}, "
            f"outer_to_inner={outer_to_inner.articulation_type}/{outer_to_inner.axis}"
        ),
    )
    ctx.check(
        "slide travels are realistic",
        (
            support_to_outer.motion_limits is not None
            and outer_to_inner.motion_limits is not None
            and support_to_outer.motion_limits.lower == 0.0
            and outer_to_inner.motion_limits.lower == 0.0
            and abs((support_to_outer.motion_limits.upper or 0.0) - OUTER_TRAVEL) < 1e-9
            and abs((outer_to_inner.motion_limits.upper or 0.0) - INNER_TRAVEL) < 1e-9
        ),
        details="Expected two positive-only telescoping stages with retained overlap.",
    )

    ctx.expect_contact(
        outer_slider,
        top_support,
        contact_tol=0.0005,
        name="outer slider is supported by the top support rails",
    )
    ctx.expect_contact(
        inner_slider,
        outer_slider,
        contact_tol=0.0005,
        name="inner slider is supported by the outer slider rails",
    )
    ctx.expect_within(
        outer_slider,
        top_support,
        axes="y",
        margin=0.0,
        name="outer slider stays laterally nested beneath the support",
    )
    ctx.expect_within(
        inner_slider,
        outer_slider,
        axes="y",
        margin=0.0,
        name="inner slider stays laterally nested beneath the outer stage",
    )
    ctx.expect_origin_gap(
        top_support,
        outer_slider,
        axis="z",
        min_gap=0.0235,
        max_gap=0.0245,
        name="outer slider hangs below the top support",
    )
    ctx.expect_origin_gap(
        outer_slider,
        inner_slider,
        axis="z",
        min_gap=0.0235,
        max_gap=0.0245,
        name="inner slider hangs below the outer slider",
    )

    with ctx.pose({support_to_outer: OUTER_TRAVEL, outer_to_inner: INNER_TRAVEL}):
        ctx.expect_contact(
            outer_slider,
            top_support,
            contact_tol=0.0005,
            name="outer slider remains guided at full extension",
        )
        ctx.expect_contact(
            inner_slider,
            outer_slider,
            contact_tol=0.0005,
            name="inner slider remains guided at full extension",
        )
        ctx.expect_origin_gap(
            inner_slider,
            top_support,
            axis="x",
            min_gap=0.35,
            name="serial stages produce clear forward extension",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
