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


PLATE_LENGTH = 0.260
PLATE_WIDTH = 0.090
PLATE_THICKNESS = 0.012

BODY_LENGTH = 0.082
BODY_WIDTH = 0.068
BODY_HEIGHT = 0.062

GUIDE_LENGTH = 0.220
GUIDE_WIDTH = 0.028
GUIDE_HEIGHT = 0.018

PLATE_CENTER_Z = GUIDE_HEIGHT / 2.0 + BODY_HEIGHT + PLATE_THICKNESS / 2.0
BODY_CENTER_Z = GUIDE_HEIGHT / 2.0 + BODY_HEIGHT / 2.0

CARRIAGE_LENGTH = 0.052
CARRIAGE_WIDTH = 0.040
CARRIAGE_HEIGHT = 0.034
CARRIAGE_CENTER_Z = -0.026

OPEN_OFFSET = 0.080
TRAVEL = 0.050


def _build_support_shape() -> cq.Workplane:
    plate = (
        cq.Workplane("XY")
        .box(PLATE_LENGTH, PLATE_WIDTH, PLATE_THICKNESS)
        .translate((0.0, 0.0, PLATE_CENTER_Z))
    )
    body = (
        cq.Workplane("XY")
        .box(BODY_LENGTH, BODY_WIDTH, BODY_HEIGHT)
        .translate((0.0, 0.0, BODY_CENTER_Z))
    )
    guide = (
        cq.Workplane("XY")
        .box(GUIDE_LENGTH, GUIDE_WIDTH, GUIDE_HEIGHT)
    )
    support = plate.union(body).union(guide)

    mounting_holes = (
        cq.Workplane("XY")
        .pushPoints([(-0.080, 0.0), (0.080, 0.0)])
        .circle(0.0045)
        .extrude(0.020)
        .translate((0.0, 0.0, PLATE_CENTER_Z - PLATE_THICKNESS / 2.0 - 0.002))
    )
    return support.cut(mounting_holes)


def _build_carriage_shape(mirror_x: bool = False) -> cq.Workplane:
    shoe = (
        cq.Workplane("XY")
        .box(CARRIAGE_LENGTH, CARRIAGE_WIDTH, CARRIAGE_HEIGHT)
        .translate((0.0, 0.0, CARRIAGE_CENTER_Z))
    )

    left_profile = [
        (-0.020, -0.020),
        (-0.020, -0.090),
        (-0.014, -0.104),
        (0.026, -0.104),
        (0.026, -0.092),
        (0.010, -0.092),
        (0.004, -0.056),
        (-0.004, -0.020),
    ]
    profile = (
        [(-x, z) for x, z in reversed(left_profile)]
        if mirror_x
        else left_profile
    )
    hanger = (
        cq.Workplane("XZ")
        .polyline(profile)
        .close()
        .extrude(0.022)
        .translate((0.0, -0.011, 0.0))
    )

    carriage = shoe.union(hanger)
    return carriage


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="under_slung_twin_jaw_gripper")

    dark_anodized = model.material(
        "dark_anodized",
        rgba=(0.15, 0.16, 0.18, 1.0),
    )
    steel = model.material(
        "steel",
        rgba=(0.67, 0.69, 0.72, 1.0),
    )

    support_body = model.part("support_body")
    support_body.visual(
        mesh_from_cadquery(_build_support_shape(), "support_body"),
        origin=Origin(),
        material=dark_anodized,
        name="support_shell",
    )

    left_carriage = model.part("left_carriage")
    left_carriage.visual(
        mesh_from_cadquery(_build_carriage_shape(mirror_x=False), "left_carriage"),
        origin=Origin(),
        material=steel,
        name="left_carriage_shell",
    )

    right_carriage = model.part("right_carriage")
    right_carriage.visual(
        mesh_from_cadquery(_build_carriage_shape(mirror_x=True), "right_carriage"),
        origin=Origin(),
        material=steel,
        name="right_carriage_shell",
    )

    model.articulation(
        "left_slide",
        ArticulationType.PRISMATIC,
        parent=support_body,
        child=left_carriage,
        origin=Origin(xyz=(-OPEN_OFFSET, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=200.0,
            velocity=0.300,
            lower=0.0,
            upper=TRAVEL,
        ),
    )
    model.articulation(
        "right_slide",
        ArticulationType.PRISMATIC,
        parent=support_body,
        child=right_carriage,
        origin=Origin(xyz=(OPEN_OFFSET, 0.0, 0.0)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=200.0,
            velocity=0.300,
            lower=0.0,
            upper=TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support_body = object_model.get_part("support_body")
    left_carriage = object_model.get_part("left_carriage")
    right_carriage = object_model.get_part("right_carriage")
    left_slide = object_model.get_articulation("left_slide")
    right_slide = object_model.get_articulation("right_slide")

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
        "left_slide_is_prismatic_on_x",
        left_slide.articulation_type == ArticulationType.PRISMATIC
        and abs(left_slide.axis[0] - 1.0) < 1e-9
        and abs(left_slide.axis[1]) < 1e-9
        and abs(left_slide.axis[2]) < 1e-9,
        details=f"Unexpected left slide definition: type={left_slide.articulation_type}, axis={left_slide.axis}",
    )
    ctx.check(
        "right_slide_is_prismatic_on_same_axis",
        right_slide.articulation_type == ArticulationType.PRISMATIC
        and abs(right_slide.axis[0] + 1.0) < 1e-9
        and abs(right_slide.axis[1]) < 1e-9
        and abs(right_slide.axis[2]) < 1e-9,
        details=f"Unexpected right slide definition: type={right_slide.articulation_type}, axis={right_slide.axis}",
    )

    ctx.expect_contact(
        left_carriage,
        support_body,
        name="left_carriage_is_supported_by_guide",
    )
    ctx.expect_contact(
        right_carriage,
        support_body,
        name="right_carriage_is_supported_by_guide",
    )

    support_aabb = ctx.part_world_aabb(support_body)
    left_aabb = ctx.part_world_aabb(left_carriage)
    right_aabb = ctx.part_world_aabb(right_carriage)
    if support_aabb is not None and left_aabb is not None:
        ctx.check(
            "left_jaw_hangs_below_support",
            left_aabb[0][2] < support_aabb[0][2] - 0.050,
            details=f"left bottom={left_aabb[0][2]:.4f}, support bottom={support_aabb[0][2]:.4f}",
        )
    if support_aabb is not None and right_aabb is not None:
        ctx.check(
            "right_jaw_hangs_below_support",
            right_aabb[0][2] < support_aabb[0][2] - 0.050,
            details=f"right bottom={right_aabb[0][2]:.4f}, support bottom={support_aabb[0][2]:.4f}",
        )

    with ctx.pose({left_slide: 0.0, right_slide: 0.0}):
        ctx.expect_gap(
            right_carriage,
            left_carriage,
            axis="x",
            min_gap=0.100,
            max_gap=0.115,
            name="jaws_have_open_clear_span",
        )

    with ctx.pose({left_slide: TRAVEL, right_slide: TRAVEL}):
        ctx.expect_gap(
            right_carriage,
            left_carriage,
            axis="x",
            min_gap=0.006,
            max_gap=0.012,
            name="jaws_close_without_overlap",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
