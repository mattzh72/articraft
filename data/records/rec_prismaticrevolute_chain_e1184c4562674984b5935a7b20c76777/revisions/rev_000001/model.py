from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_LENGTH = 0.42
BASE_WIDTH = 0.14
BASE_THICKNESS = 0.018

RAIL_LENGTH = 0.34
RAIL_WIDTH = 0.04
RAIL_HEIGHT = 0.032

CARRIAGE_LENGTH = 0.11
CARRIAGE_WIDTH = 0.09
CARRIAGE_TOP_HEIGHT = 0.024
CARRIAGE_TOP_Z = 0.008 + (CARRIAGE_TOP_HEIGHT / 2.0)
CARRIAGE_CENTER_Z = BASE_THICKNESS + RAIL_HEIGHT

SLIDE_HOME_X = -0.11
SLIDE_TRAVEL = 0.22

BEARING_LENGTH = 0.06
BEARING_WIDTH = 0.012
BEARING_HEIGHT = 0.008
BEARING_Y = 0.014

SKIRT_LENGTH = 0.085
SKIRT_WIDTH = 0.016
SKIRT_HEIGHT = 0.03
SKIRT_Y = 0.037
SKIRT_Z = SKIRT_HEIGHT / 2.0

PIN_X = 0.062
PIN_Z = 0.018
PIN_RADIUS = 0.0055
PIN_HOLE_RADIUS = 0.0065

EAR_LENGTH = 0.028
EAR_THICKNESS = 0.01
EAR_GAP = 0.022
EAR_HEIGHT = 0.032
EAR_Y_OFFSET = (EAR_GAP / 2.0) + (EAR_THICKNESS / 2.0)
PIN_LENGTH = EAR_GAP + (2.0 * EAR_THICKNESS)
LINK_WIDTH = 0.018
SPACER_RADIUS = 0.011
SPACER_THICKNESS = 0.004
SPACER_Y_OFFSET = (LINK_WIDTH / 2.0) + (SPACER_THICKNESS / 2.0)

LINK_EYE_RADIUS = 0.016
LINK_ARM_LENGTH = 0.17
LINK_ARM_HEIGHT = 0.022
LINK_ARM_CENTER_X = 0.095
LINK_ROOT_LENGTH = 0.036
LINK_ROOT_HEIGHT = 0.028
LINK_ROOT_CENTER_X = 0.022
LINK_TIP_RADIUS = 0.011
LINK_TIP_CENTER_X = 0.2

def _extrude_xz(profile: cq.Workplane, depth: float) -> cq.Workplane:
    return profile.extrude(depth).translate((0.0, depth / 2.0, 0.0))


def _make_output_link() -> cq.Workplane:
    eye = _extrude_xz(cq.Workplane("XZ").circle(LINK_EYE_RADIUS), LINK_WIDTH)
    root = _extrude_xz(
        cq.Workplane("XZ").rect(LINK_ROOT_LENGTH, LINK_ROOT_HEIGHT),
        LINK_WIDTH,
    ).translate((LINK_ROOT_CENTER_X, 0.0, 0.0))
    arm = _extrude_xz(
        cq.Workplane("XZ").rect(LINK_ARM_LENGTH, LINK_ARM_HEIGHT),
        LINK_WIDTH,
    ).translate((LINK_ARM_CENTER_X, 0.0, 0.0))
    tip = _extrude_xz(cq.Workplane("XZ").circle(LINK_TIP_RADIUS), LINK_WIDTH).translate(
        (LINK_TIP_CENTER_X, 0.0, 0.0)
    )
    pin_hole = _extrude_xz(
        cq.Workplane("XZ").circle(PIN_HOLE_RADIUS),
        LINK_WIDTH + 0.004,
    )

    return eye.union(root).union(arm).union(tip).cut(pin_hole)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="two_joint_actuator_chain")

    steel = model.material("steel", color=(0.38, 0.4, 0.43))
    anodized = model.material("anodized", color=(0.7, 0.72, 0.75))
    signal_orange = model.material("signal_orange", color=(0.89, 0.46, 0.12))

    base = model.part("base_guide")
    base.visual(
        Box((BASE_LENGTH, BASE_WIDTH, BASE_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS / 2.0)),
        material=steel,
        name="base_plate",
    )
    base.visual(
        Box((RAIL_LENGTH, RAIL_WIDTH, RAIL_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS + RAIL_HEIGHT / 2.0)),
        material=anodized,
        name="guide_rail",
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.05, CARRIAGE_WIDTH, CARRIAGE_TOP_HEIGHT)),
        origin=Origin(xyz=(-0.025, 0.0, CARRIAGE_TOP_Z)),
        material=anodized,
        name="rear_plate",
    )
    carriage.visual(
        Box((0.092, 0.016, 0.02)),
        origin=Origin(xyz=(0.016, 0.026, 0.018)),
        material=anodized,
        name="left_rail",
    )
    carriage.visual(
        Box((0.092, 0.016, 0.02)),
        origin=Origin(xyz=(0.016, -0.026, 0.018)),
        material=anodized,
        name="right_rail",
    )
    carriage.visual(
        Box((0.07, BEARING_WIDTH, BEARING_HEIGHT)),
        origin=Origin(xyz=(-0.012, BEARING_Y, BEARING_HEIGHT / 2.0)),
        material=steel,
        name="left_bearing",
    )
    carriage.visual(
        Box((0.07, BEARING_WIDTH, BEARING_HEIGHT)),
        origin=Origin(xyz=(-0.012, -BEARING_Y, BEARING_HEIGHT / 2.0)),
        material=steel,
        name="right_bearing",
    )
    carriage.visual(
        Box((0.075, SKIRT_WIDTH, SKIRT_HEIGHT)),
        origin=Origin(xyz=(-0.01, SKIRT_Y, SKIRT_Z)),
        material=anodized,
        name="left_skirt",
    )
    carriage.visual(
        Box((0.075, SKIRT_WIDTH, SKIRT_HEIGHT)),
        origin=Origin(xyz=(-0.01, -SKIRT_Y, SKIRT_Z)),
        material=anodized,
        name="right_skirt",
    )
    carriage.visual(
        Box((EAR_LENGTH, EAR_THICKNESS, EAR_HEIGHT)),
        origin=Origin(xyz=(PIN_X, EAR_Y_OFFSET, PIN_Z)),
        material=anodized,
        name="left_ear",
    )
    carriage.visual(
        Box((EAR_LENGTH, EAR_THICKNESS, EAR_HEIGHT)),
        origin=Origin(xyz=(PIN_X, -EAR_Y_OFFSET, PIN_Z)),
        material=anodized,
        name="right_ear",
    )
    carriage.visual(
        Cylinder(radius=SPACER_RADIUS, length=SPACER_THICKNESS),
        origin=Origin(xyz=(PIN_X, SPACER_Y_OFFSET, PIN_Z), rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="left_spacer",
    )
    carriage.visual(
        Cylinder(radius=SPACER_RADIUS, length=SPACER_THICKNESS),
        origin=Origin(xyz=(PIN_X, -SPACER_Y_OFFSET, PIN_Z), rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="right_spacer",
    )
    carriage.visual(
        Cylinder(radius=PIN_RADIUS, length=PIN_LENGTH),
        origin=Origin(xyz=(PIN_X, 0.0, PIN_Z), rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="pivot_pin",
    )

    output_link = model.part("output_link")
    output_link.visual(
        mesh_from_cadquery(_make_output_link(), "output_link_mesh"),
        material=signal_orange,
        name="link_body",
    )

    model.articulation(
        "guide_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=carriage,
        origin=Origin(xyz=(SLIDE_HOME_X, 0.0, CARRIAGE_CENTER_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=0.4,
            lower=0.0,
            upper=SLIDE_TRAVEL,
        ),
    )
    model.articulation(
        "carriage_hinge",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=output_link,
        origin=Origin(xyz=(PIN_X, 0.0, PIN_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=1.4,
            lower=-1.1,
            upper=0.15,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_guide")
    carriage = object_model.get_part("carriage")
    output_link = object_model.get_part("output_link")
    slide = object_model.get_articulation("guide_slide")
    hinge = object_model.get_articulation("carriage_hinge")

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
    ctx.fail_if_articulation_overlaps(
        max_pose_samples=24,
        name="joint_sweep_no_overlaps",
    )
    ctx.fail_if_isolated_parts(
        max_pose_samples=24,
        name="joint_sweep_no_floating",
    )

    ctx.check(
        "guide_slide_axis",
        slide.axis == (1.0, 0.0, 0.0),
        f"expected prismatic axis (1, 0, 0), got {slide.axis}",
    )
    ctx.check(
        "carriage_hinge_axis",
        hinge.axis == (0.0, 1.0, 0.0),
        f"expected revolute axis (0, 1, 0), got {hinge.axis}",
    )
    ctx.check(
        "guide_slide_limits",
        slide.motion_limits is not None
        and slide.motion_limits.lower == 0.0
        and slide.motion_limits.upper == SLIDE_TRAVEL,
        f"unexpected slide limits: {slide.motion_limits}",
    )
    ctx.check(
        "carriage_hinge_limits",
        hinge.motion_limits is not None
        and hinge.motion_limits.lower == -1.1
        and hinge.motion_limits.upper == 0.15,
        f"unexpected hinge limits: {hinge.motion_limits}",
    )

    ctx.expect_origin_gap(
        output_link,
        carriage,
        axis="x",
        min_gap=PIN_X - 0.001,
        max_gap=PIN_X + 0.001,
        name="hinge_sits_at_carriage_front",
    )
    ctx.expect_origin_gap(
        output_link,
        carriage,
        axis="z",
        min_gap=PIN_Z - 0.001,
        max_gap=PIN_Z + 0.001,
        name="hinge_pin_height_on_carriage",
    )

    with ctx.pose({slide: 0.0, hinge: 0.0}):
        ctx.expect_contact(
            carriage,
            base,
            elem_a="left_bearing",
            elem_b="guide_rail",
            name="rest_left_bearing_supported_by_guide",
        )
        ctx.expect_contact(
            carriage,
            base,
            elem_a="right_bearing",
            elem_b="guide_rail",
            name="rest_right_bearing_supported_by_guide",
        )
        ctx.expect_overlap(
            carriage,
            base,
            axes="x",
            min_overlap=0.05,
            elem_a="left_bearing",
            elem_b="guide_rail",
            name="rest_bearing_overlap_on_rail",
        )
        ctx.expect_contact(
            carriage,
            output_link,
            elem_a="left_spacer",
            elem_b="link_body",
            name="rest_link_supported_by_left_spacer",
        )
        ctx.expect_contact(
            carriage,
            output_link,
            elem_a="right_spacer",
            elem_b="link_body",
            name="rest_link_supported_by_right_spacer",
        )

    slide_limits = slide.motion_limits
    if slide_limits is not None and slide_limits.lower is not None and slide_limits.upper is not None:
        with ctx.pose({slide: slide_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="guide_slide_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="guide_slide_lower_no_floating")
            ctx.expect_contact(
                carriage,
                base,
                elem_a="left_bearing",
                elem_b="guide_rail",
                name="guide_slide_lower_left_contact",
            )
            ctx.expect_contact(
                carriage,
                base,
                elem_a="right_bearing",
                elem_b="guide_rail",
                name="guide_slide_lower_right_contact",
            )
        with ctx.pose({slide: slide_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="guide_slide_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="guide_slide_upper_no_floating")
            ctx.expect_contact(
                carriage,
                base,
                elem_a="left_bearing",
                elem_b="guide_rail",
                name="guide_slide_upper_left_contact",
            )
            ctx.expect_contact(
                carriage,
                base,
                elem_a="right_bearing",
                elem_b="guide_rail",
                name="guide_slide_upper_right_contact",
            )
            ctx.expect_overlap(
                carriage,
                base,
                axes="x",
                min_overlap=0.05,
                elem_a="left_bearing",
                elem_b="guide_rail",
                name="guide_slide_upper_overlap",
            )

    hinge_limits = hinge.motion_limits
    if hinge_limits is not None and hinge_limits.lower is not None and hinge_limits.upper is not None:
        with ctx.pose({hinge: hinge_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="carriage_hinge_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="carriage_hinge_lower_no_floating")
            ctx.expect_contact(
                carriage,
                output_link,
                elem_a="left_spacer",
                elem_b="link_body",
                name="carriage_hinge_lower_left_support_contact",
            )
            ctx.expect_contact(
                carriage,
                output_link,
                elem_a="right_spacer",
                elem_b="link_body",
                name="carriage_hinge_lower_right_support_contact",
            )
        with ctx.pose({slide: slide_limits.upper if slide_limits is not None else 0.0, hinge: hinge_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="carriage_hinge_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="carriage_hinge_upper_no_floating")
            ctx.expect_contact(
                carriage,
                output_link,
                elem_a="left_spacer",
                elem_b="link_body",
                name="carriage_hinge_upper_left_support_contact",
            )
            ctx.expect_contact(
                carriage,
                output_link,
                elem_a="right_spacer",
                elem_b="link_body",
                name="carriage_hinge_upper_right_support_contact",
            )

    # For bounded REVOLUTE/PRISMATIC joints, also check at least the lower/upper
    # motion-limit poses for both no overlap and no floating. Example:
    # hinge = object_model.get_articulation("lid_hinge")
    # limits = hinge.motion_limits
    # if limits is not None and limits.lower is not None and limits.upper is not None:
    #     with ctx.pose({hinge: limits.lower}):
    #         ctx.fail_if_parts_overlap_in_current_pose(name="lid_hinge_lower_no_overlap")
    #         ctx.fail_if_isolated_parts(name="lid_hinge_lower_no_floating")
    #     with ctx.pose({hinge: limits.upper}):
    #         ctx.fail_if_parts_overlap_in_current_pose(name="lid_hinge_upper_no_overlap")
    #         ctx.fail_if_isolated_parts(name="lid_hinge_upper_no_floating")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
