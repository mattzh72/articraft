from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

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


GUIDE_LENGTH = 0.62
GUIDE_WIDTH = 0.14
BASE_THICKNESS = 0.016
RAIL_LENGTH = 0.54
RAIL_WIDTH = 0.074
RAIL_HEIGHT = 0.036
FOOT_LENGTH = 0.10
FOOT_THICKNESS = 0.008
END_STOP_LENGTH = 0.026
END_STOP_WIDTH = 0.09
END_STOP_HEIGHT = 0.03
RAIL_TOP_Z = BASE_THICKNESS + RAIL_HEIGHT

CARRIAGE_LENGTH = 0.12
CARRIAGE_WIDTH = 0.112
CARRIAGE_TOP_THICKNESS = 0.024
CARRIAGE_SKIRT_THICKNESS = 0.019
CARRIAGE_SKIRT_DEPTH = 0.036
CARRIAGE_PEDESTAL_LENGTH = 0.06
CARRIAGE_PEDESTAL_WIDTH = 0.06
CARRIAGE_PEDESTAL_HEIGHT = 0.064
SHOULDER_PIVOT_Z = 0.085
CARRIAGE_EAR_LENGTH = 0.034
CARRIAGE_EAR_THICKNESS = 0.01
CARRIAGE_EAR_HEIGHT = 0.05
CARRIAGE_EAR_CENTER_Y = 0.035

SHOULDER_REACH = 0.168
SHOULDER_HUB_RADIUS = 0.018
SHOULDER_HUB_LENGTH = 0.06
SHOULDER_BEAM_LENGTH = 0.14
SHOULDER_BEAM_WIDTH = 0.04
SHOULDER_BEAM_HEIGHT = 0.02
SHOULDER_BRACE_LENGTH = 0.09
SHOULDER_BRACE_WIDTH = 0.026
SHOULDER_BRACE_HEIGHT = 0.016
SHOULDER_EAR_LENGTH = 0.028
SHOULDER_EAR_THICKNESS = 0.008
SHOULDER_EAR_HEIGHT = 0.034
SHOULDER_EAR_CENTER_X = SHOULDER_REACH - SHOULDER_EAR_LENGTH / 2.0
SHOULDER_EAR_CENTER_Y = 0.024

OUTER_HUB_RADIUS = 0.014
OUTER_HUB_LENGTH = 0.04
OUTER_BEAM_LENGTH = 0.078
OUTER_BEAM_WIDTH = 0.028
OUTER_BEAM_HEIGHT = 0.018
OUTER_BRACE_LENGTH = 0.052
OUTER_BRACE_WIDTH = 0.02
OUTER_BRACE_HEIGHT = 0.014
OUTER_BRACKET_LENGTH = 0.028
OUTER_BRACKET_WIDTH = 0.032
OUTER_BRACKET_HEIGHT = 0.024
OUTER_BRACKET_CENTER_X = 0.088
SERVICE_PAD_LENGTH = 0.04
SERVICE_PAD_WIDTH = 0.036
SERVICE_PAD_HEIGHT = 0.012
SERVICE_PAD_CENTER_X = 0.12

Y_AXIS_CYLINDER_RPY = (pi / 2.0, 0.0, 0.0)


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None):
    if aabb is None:
        return None
    low, high = aabb
    return tuple((lo + hi) * 0.5 for lo, hi in zip(low, high))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="service_carriage_elbow_linkage")

    base_metal = model.material("base_metal", color=(0.23, 0.24, 0.26))
    rail_metal = model.material("rail_metal", color=(0.42, 0.44, 0.47))
    carriage_paint = model.material("carriage_paint", color=(0.16, 0.32, 0.45))
    link_alloy = model.material("link_alloy", color=(0.74, 0.76, 0.79))
    tool_polymer = model.material("tool_polymer", color=(0.79, 0.57, 0.14))

    guide = model.part("guide")
    guide.visual(
        Box((GUIDE_LENGTH, GUIDE_WIDTH, BASE_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS / 2.0)),
        material=base_metal,
        name="base_plate",
    )
    guide.visual(
        Box((RAIL_LENGTH, RAIL_WIDTH, RAIL_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS + RAIL_HEIGHT / 2.0)),
        material=rail_metal,
        name="rail_body",
    )
    guide.visual(
        Box((FOOT_LENGTH, GUIDE_WIDTH, FOOT_THICKNESS)),
        origin=Origin(xyz=(-0.21, 0.0, FOOT_THICKNESS / 2.0)),
        material=base_metal,
        name="left_foot",
    )
    guide.visual(
        Box((FOOT_LENGTH, GUIDE_WIDTH, FOOT_THICKNESS)),
        origin=Origin(xyz=(0.21, 0.0, FOOT_THICKNESS / 2.0)),
        material=base_metal,
        name="right_foot",
    )
    guide.visual(
        Box((END_STOP_LENGTH, END_STOP_WIDTH, END_STOP_HEIGHT)),
        origin=Origin(
            xyz=(
                -(RAIL_LENGTH / 2.0 - END_STOP_LENGTH / 2.0),
                0.0,
                BASE_THICKNESS + END_STOP_HEIGHT / 2.0,
            )
        ),
        material=base_metal,
        name="left_stop",
    )
    guide.visual(
        Box((END_STOP_LENGTH, END_STOP_WIDTH, END_STOP_HEIGHT)),
        origin=Origin(
            xyz=(
                RAIL_LENGTH / 2.0 - END_STOP_LENGTH / 2.0,
                0.0,
                BASE_THICKNESS + END_STOP_HEIGHT / 2.0,
            )
        ),
        material=base_metal,
        name="right_stop",
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((CARRIAGE_LENGTH, CARRIAGE_WIDTH, CARRIAGE_TOP_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, CARRIAGE_TOP_THICKNESS / 2.0)),
        material=carriage_paint,
        name="saddle_roof",
    )
    carriage.visual(
        Box((CARRIAGE_LENGTH, CARRIAGE_SKIRT_THICKNESS, CARRIAGE_SKIRT_DEPTH)),
        origin=Origin(
            xyz=(
                0.0,
                CARRIAGE_WIDTH / 2.0 - CARRIAGE_SKIRT_THICKNESS / 2.0,
                -CARRIAGE_SKIRT_DEPTH / 2.0,
            )
        ),
        material=carriage_paint,
        name="left_skirt",
    )
    carriage.visual(
        Box((CARRIAGE_LENGTH, CARRIAGE_SKIRT_THICKNESS, CARRIAGE_SKIRT_DEPTH)),
        origin=Origin(
            xyz=(
                0.0,
                -(CARRIAGE_WIDTH / 2.0 - CARRIAGE_SKIRT_THICKNESS / 2.0),
                -CARRIAGE_SKIRT_DEPTH / 2.0,
            )
        ),
        material=carriage_paint,
        name="right_skirt",
    )
    carriage.visual(
        Box((CARRIAGE_PEDESTAL_LENGTH, CARRIAGE_PEDESTAL_WIDTH, CARRIAGE_PEDESTAL_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                CARRIAGE_PEDESTAL_HEIGHT / 2.0,
            )
        ),
        material=carriage_paint,
        name="pedestal",
    )
    carriage.visual(
        Box((CARRIAGE_EAR_LENGTH, CARRIAGE_EAR_THICKNESS, CARRIAGE_EAR_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                CARRIAGE_EAR_CENTER_Y,
                SHOULDER_PIVOT_Z,
            )
        ),
        material=carriage_paint,
        name="left_shoulder_ear",
    )
    carriage.visual(
        Box((CARRIAGE_EAR_LENGTH, CARRIAGE_EAR_THICKNESS, CARRIAGE_EAR_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                -CARRIAGE_EAR_CENTER_Y,
                SHOULDER_PIVOT_Z,
            )
        ),
        material=carriage_paint,
        name="right_shoulder_ear",
    )

    shoulder = model.part("shoulder_link")
    shoulder.visual(
        Cylinder(radius=SHOULDER_HUB_RADIUS, length=SHOULDER_HUB_LENGTH),
        origin=Origin(rpy=Y_AXIS_CYLINDER_RPY),
        material=link_alloy,
        name="shoulder_hub",
    )
    shoulder.visual(
        Box((SHOULDER_BEAM_LENGTH, SHOULDER_BEAM_WIDTH, SHOULDER_BEAM_HEIGHT)),
        origin=Origin(xyz=(SHOULDER_BEAM_LENGTH / 2.0, 0.0, 0.0)),
        material=link_alloy,
        name="shoulder_beam",
    )
    shoulder.visual(
        Box((SHOULDER_BRACE_LENGTH, SHOULDER_BRACE_WIDTH, SHOULDER_BRACE_HEIGHT)),
        origin=Origin(
            xyz=(
                SHOULDER_BRACE_LENGTH / 2.0,
                0.0,
                -(SHOULDER_BEAM_HEIGHT / 2.0 + SHOULDER_BRACE_HEIGHT / 2.0 - 0.002),
            )
        ),
        material=link_alloy,
        name="shoulder_brace",
    )
    shoulder.visual(
        Box((SHOULDER_EAR_LENGTH, SHOULDER_EAR_THICKNESS, SHOULDER_EAR_HEIGHT)),
        origin=Origin(
            xyz=(
                SHOULDER_EAR_CENTER_X,
                SHOULDER_EAR_CENTER_Y,
                0.0,
            )
        ),
        material=link_alloy,
        name="left_elbow_ear",
    )
    shoulder.visual(
        Box((SHOULDER_EAR_LENGTH, SHOULDER_EAR_THICKNESS, SHOULDER_EAR_HEIGHT)),
        origin=Origin(
            xyz=(
                SHOULDER_EAR_CENTER_X,
                -SHOULDER_EAR_CENTER_Y,
                0.0,
            )
        ),
        material=link_alloy,
        name="right_elbow_ear",
    )

    outer = model.part("outer_link")
    outer.visual(
        Cylinder(radius=OUTER_HUB_RADIUS, length=OUTER_HUB_LENGTH),
        origin=Origin(rpy=Y_AXIS_CYLINDER_RPY),
        material=link_alloy,
        name="outer_hub",
    )
    outer.visual(
        Box((OUTER_BEAM_LENGTH, OUTER_BEAM_WIDTH, OUTER_BEAM_HEIGHT)),
        origin=Origin(xyz=(OUTER_BEAM_LENGTH / 2.0, 0.0, 0.0)),
        material=link_alloy,
        name="outer_beam",
    )
    outer.visual(
        Box((OUTER_BRACE_LENGTH, OUTER_BRACE_WIDTH, OUTER_BRACE_HEIGHT)),
        origin=Origin(
            xyz=(
                OUTER_BRACE_LENGTH / 2.0,
                0.0,
                -(OUTER_BEAM_HEIGHT / 2.0 + OUTER_BRACE_HEIGHT / 2.0 - 0.002),
            )
        ),
        material=link_alloy,
        name="outer_brace",
    )
    outer.visual(
        Box((OUTER_BRACKET_LENGTH, OUTER_BRACKET_WIDTH, OUTER_BRACKET_HEIGHT)),
        origin=Origin(xyz=(OUTER_BRACKET_CENTER_X, 0.0, 0.0)),
        material=link_alloy,
        name="tool_bracket",
    )
    outer.visual(
        Box((SERVICE_PAD_LENGTH, SERVICE_PAD_WIDTH, SERVICE_PAD_HEIGHT)),
        origin=Origin(
            xyz=(
                SERVICE_PAD_CENTER_X,
                0.0,
                0.004,
            )
        ),
        material=tool_polymer,
        name="service_pad",
    )

    model.articulation(
        "guide_to_carriage",
        ArticulationType.PRISMATIC,
        parent=guide,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, RAIL_TOP_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=0.35,
            lower=-0.18,
            upper=0.18,
        ),
    )
    model.articulation(
        "carriage_to_shoulder",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=shoulder,
        origin=Origin(xyz=(0.0, 0.0, SHOULDER_PIVOT_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=45.0,
            velocity=1.6,
            lower=-0.45,
            upper=1.25,
        ),
    )
    model.articulation(
        "shoulder_to_outer",
        ArticulationType.REVOLUTE,
        parent=shoulder,
        child=outer,
        origin=Origin(xyz=(SHOULDER_REACH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=2.0,
            lower=0.0,
            upper=2.1,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    guide = object_model.get_part("guide")
    carriage = object_model.get_part("carriage")
    shoulder = object_model.get_part("shoulder_link")
    outer = object_model.get_part("outer_link")

    slide = object_model.get_articulation("guide_to_carriage")
    shoulder_joint = object_model.get_articulation("carriage_to_shoulder")
    elbow_joint = object_model.get_articulation("shoulder_to_outer")

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

    ctx.expect_contact(carriage, guide, name="carriage_supported_by_guide")
    ctx.expect_contact(shoulder, carriage, name="shoulder_mounted_in_carriage_yoke")
    ctx.expect_contact(outer, shoulder, name="outer_link_mounted_in_elbow_yoke")
    ctx.expect_overlap(
        carriage,
        guide,
        axes="xy",
        min_overlap=0.07,
        name="carriage_stays_captured_over_guide",
    )

    slide_limits = slide.motion_limits
    shoulder_limits = shoulder_joint.motion_limits
    elbow_limits = elbow_joint.motion_limits

    slide_lower = slide_limits.lower if slide_limits and slide_limits.lower is not None else -0.18
    slide_upper = slide_limits.upper if slide_limits and slide_limits.upper is not None else 0.18
    shoulder_raise = min(
        0.75,
        shoulder_limits.upper if shoulder_limits and shoulder_limits.upper is not None else 0.75,
    )
    elbow_raise = min(
        1.05,
        elbow_limits.upper if elbow_limits and elbow_limits.upper is not None else 1.05,
    )

    with ctx.pose({slide: slide_lower}):
        carriage_at_min = ctx.part_world_position(carriage)
    with ctx.pose({slide: slide_upper}):
        carriage_at_max = ctx.part_world_position(carriage)
    slide_ok = (
        carriage_at_min is not None
        and carriage_at_max is not None
        and carriage_at_max[0] - carriage_at_min[0] > 0.34
        and abs(carriage_at_max[1] - carriage_at_min[1]) < 1e-4
        and abs(carriage_at_max[2] - carriage_at_min[2]) < 1e-4
    )
    ctx.check(
        "prismatic_joint_translates_carriage_along_x",
        slide_ok,
        details=f"min pose={carriage_at_min}, max pose={carriage_at_max}",
    )

    with ctx.pose({shoulder_joint: 0.0, elbow_joint: 0.0}):
        elbow_neutral = ctx.part_world_position(outer)
    with ctx.pose({shoulder_joint: shoulder_raise, elbow_joint: 0.0}):
        elbow_raised = ctx.part_world_position(outer)
    shoulder_ok = (
        elbow_neutral is not None
        and elbow_raised is not None
        and elbow_raised[2] > elbow_neutral[2] + 0.07
        and abs(elbow_raised[1] - elbow_neutral[1]) < 0.01
    )
    ctx.check(
        "shoulder_positive_rotation_lifts_elbow",
        shoulder_ok,
        details=f"neutral elbow={elbow_neutral}, raised elbow={elbow_raised}",
    )

    with ctx.pose({shoulder_joint: 0.45, elbow_joint: 0.0}):
        pad_neutral = _aabb_center(ctx.part_element_world_aabb(outer, elem="service_pad"))
    with ctx.pose({shoulder_joint: 0.45, elbow_joint: elbow_raise}):
        pad_curled = _aabb_center(ctx.part_element_world_aabb(outer, elem="service_pad"))
    elbow_ok = (
        pad_neutral is not None
        and pad_curled is not None
        and pad_curled[2] > pad_neutral[2] + 0.025
    )
    ctx.check(
        "elbow_positive_rotation_curls_service_pad_upward",
        elbow_ok,
        details=f"neutral pad={pad_neutral}, curled pad={pad_curled}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
