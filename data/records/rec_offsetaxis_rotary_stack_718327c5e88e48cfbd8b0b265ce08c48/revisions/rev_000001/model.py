from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

import cadquery as cq

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
    mesh_from_cadquery,
)


BASE_RADIUS = 0.210
BASE_FOOT_HEIGHT = 0.040
BASE_SHOULDER_RADIUS = 0.170
BASE_SHOULDER_HEIGHT = 0.011
BASE_COLLAR_RADIUS = 0.092
BASE_COLLAR_HEIGHT = 0.004
BASE_HEIGHT = BASE_FOOT_HEIGHT + BASE_SHOULDER_HEIGHT + BASE_COLLAR_HEIGHT

DRUM_RADIUS = 0.165
DRUM_BODY_HEIGHT = 0.148
DRUM_LIP_RADIUS = 0.145
DRUM_LIP_HEIGHT = 0.010

BRIDGE_START_Z = 0.165
BRIDGE_LENGTH = 0.285
BRIDGE_WIDTH = 0.120
BRIDGE_THICKNESS = 0.028
BRIDGE_CENTER_X = 0.160

ROOT_BLOCK_START_Z = 0.140
ROOT_BLOCK_LENGTH = 0.090
ROOT_BLOCK_WIDTH = 0.045
ROOT_BLOCK_HEIGHT = 0.035
ROOT_BLOCK_CENTER_X = 0.060
ROOT_BLOCK_CENTER_Y = -0.028

SUPPORT_WEB_WIDTH = 0.032
SUPPORT_WEB_Y = -0.050

OUTER_STRUT_START_Z = 0.118
OUTER_STRUT_LENGTH = 0.100
OUTER_STRUT_WIDTH = 0.038
OUTER_STRUT_HEIGHT = BRIDGE_START_Z - OUTER_STRUT_START_Z + 0.004
OUTER_STRUT_CENTER_X = 0.235
OUTER_STRUT_CENTER_Y = 0.028

UPPER_AXIS_X = 0.270
UPPER_AXIS_Y = 0.041
HOUSING_BASE_RADIUS = 0.047
HOUSING_LIP_RADIUS = 0.055
HOUSING_START_Z = 0.185
HOUSING_BODY_HEIGHT = 0.030
HOUSING_LIP_HEIGHT = 0.008
UPPER_JOINT_Z = HOUSING_START_Z + HOUSING_BODY_HEIGHT + HOUSING_LIP_HEIGHT

UPPER_HUB_RADIUS = 0.038
UPPER_HUB_HEIGHT = 0.040
UPPER_FLANGE_RADIUS = 0.078
UPPER_FLANGE_START_Z = 0.010
UPPER_FLANGE_THICKNESS = 0.014
UPPER_CAP_RADIUS = 0.026
UPPER_CAP_START_Z = UPPER_HUB_HEIGHT
UPPER_CAP_HEIGHT = 0.010
UPPER_TOTAL_HEIGHT = UPPER_CAP_START_Z + UPPER_CAP_HEIGHT


def _make_base_shape() -> cq.Workplane:
    foot = cq.Workplane("XY").circle(BASE_RADIUS).extrude(BASE_FOOT_HEIGHT)
    shoulder = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, BASE_FOOT_HEIGHT))
        .circle(BASE_SHOULDER_RADIUS)
        .extrude(BASE_SHOULDER_HEIGHT)
    )
    base = foot.union(shoulder)
    base = (
        base.faces(">Z")
        .workplane()
        .polarArray(0.138, 0.0, 360.0, 6)
        .hole(0.012, depth=0.010)
    )
    collar = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, BASE_FOOT_HEIGHT + BASE_SHOULDER_HEIGHT))
        .circle(BASE_COLLAR_RADIUS)
        .extrude(BASE_COLLAR_HEIGHT)
    )
    base = base.union(collar)
    return base


def _make_lower_stage_shape() -> cq.Workplane:
    drum = cq.Workplane("XY").circle(DRUM_RADIUS).extrude(DRUM_BODY_HEIGHT)
    drum_lip = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, DRUM_BODY_HEIGHT))
        .circle(DRUM_LIP_RADIUS)
        .extrude(DRUM_LIP_HEIGHT)
    )
    drum = drum.union(drum_lip)
    drum = drum.faces(">Z").workplane().circle(0.104).cutBlind(-0.008)
    drum = (
        drum.faces(">Z")
        .workplane()
        .polarArray(0.120, 15.0, 360.0, 5)
        .hole(0.010, depth=0.008)
    )

    bridge = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, BRIDGE_START_Z))
        .center(BRIDGE_CENTER_X, 0.0)
        .rect(BRIDGE_LENGTH, BRIDGE_WIDTH)
        .extrude(BRIDGE_THICKNESS)
    )

    root_block = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, ROOT_BLOCK_START_Z))
        .center(ROOT_BLOCK_CENTER_X, ROOT_BLOCK_CENTER_Y)
        .rect(ROOT_BLOCK_LENGTH, ROOT_BLOCK_WIDTH)
        .extrude(ROOT_BLOCK_HEIGHT)
    )

    support_web = (
        cq.Workplane("XZ")
        .polyline(
            [
                (-0.015, 0.132),
                (0.090, 0.132),
                (0.205, 0.173),
                (0.040, 0.173),
            ]
        )
        .close()
        .extrude(SUPPORT_WEB_WIDTH)
        .translate((0.0, SUPPORT_WEB_Y, 0.0))
    )

    outer_strut = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, OUTER_STRUT_START_Z))
        .center(OUTER_STRUT_CENTER_X, OUTER_STRUT_CENTER_Y)
        .rect(OUTER_STRUT_LENGTH, OUTER_STRUT_WIDTH)
        .extrude(OUTER_STRUT_HEIGHT)
    )

    housing_body = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, HOUSING_START_Z))
        .center(UPPER_AXIS_X, UPPER_AXIS_Y)
        .circle(HOUSING_BASE_RADIUS)
        .extrude(HOUSING_BODY_HEIGHT)
    )
    housing_lip = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, HOUSING_START_Z + HOUSING_BODY_HEIGHT))
        .center(UPPER_AXIS_X, UPPER_AXIS_Y)
        .circle(HOUSING_LIP_RADIUS)
        .extrude(HOUSING_LIP_HEIGHT)
    )
    housing = housing_body.union(housing_lip)
    housing = housing.faces(">Z").workplane().circle(0.026).cutBlind(-0.006)

    return drum.union(bridge).union(root_block).union(support_web).union(outer_strut).union(housing)


def _make_upper_flange_shape() -> cq.Workplane:
    flange_plate = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, UPPER_FLANGE_START_Z))
        .circle(UPPER_FLANGE_RADIUS)
        .extrude(UPPER_FLANGE_THICKNESS)
    )
    flange_plate = (
        flange_plate.faces(">Z")
        .workplane()
        .polarArray(0.050, 0.0, 360.0, 4)
        .hole(0.008)
    )

    hub = cq.Workplane("XY").circle(UPPER_HUB_RADIUS).extrude(UPPER_HUB_HEIGHT)
    cap = (
        cq.Workplane("XY")
        .transformed(offset=(0.0, 0.0, UPPER_CAP_START_Z))
        .circle(UPPER_CAP_RADIUS)
        .extrude(UPPER_CAP_HEIGHT)
    )
    return hub.union(flange_plate).union(cap)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="side_bridge_rotary_stack")

    base_material = model.material("base_paint", rgba=(0.19, 0.20, 0.22, 1.0))
    housing_material = model.material("housing_metal", rgba=(0.58, 0.60, 0.63, 1.0))
    flange_material = model.material("flange_metal", rgba=(0.80, 0.81, 0.84, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_make_base_shape(), "side_bridge_base"),
        material=base_material,
        name="base_shell",
    )
    base.inertial = Inertial.from_geometry(
        Cylinder(radius=BASE_RADIUS, length=BASE_HEIGHT),
        mass=24.0,
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT / 2.0)),
    )

    lower_stage = model.part("lower_stage")
    lower_stage.visual(
        mesh_from_cadquery(_make_lower_stage_shape(), "side_bridge_lower_stage"),
        material=housing_material,
        name="lower_stage_shell",
    )
    lower_stage.inertial = Inertial.from_geometry(
        Box((0.500, 0.240, UPPER_JOINT_Z)),
        mass=18.0,
        origin=Origin(xyz=(0.080, 0.0, UPPER_JOINT_Z / 2.0)),
    )

    upper_flange = model.part("upper_flange")
    upper_flange.visual(
        mesh_from_cadquery(_make_upper_flange_shape(), "side_bridge_upper_flange"),
        material=flange_material,
        name="upper_flange_shell",
    )
    upper_flange.inertial = Inertial.from_geometry(
        Cylinder(radius=UPPER_FLANGE_RADIUS, length=UPPER_TOTAL_HEIGHT),
        mass=3.2,
        origin=Origin(xyz=(0.0, 0.0, UPPER_TOTAL_HEIGHT / 2.0)),
    )

    model.articulation(
        "base_to_lower_stage",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lower_stage,
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=1.5,
            lower=-2.6,
            upper=2.6,
        ),
    )

    model.articulation(
        "lower_stage_to_upper_flange",
        ArticulationType.REVOLUTE,
        parent=lower_stage,
        child=upper_flange,
        origin=Origin(xyz=(UPPER_AXIS_X, UPPER_AXIS_Y, UPPER_JOINT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=2.2,
            lower=-2.1,
            upper=2.1,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    lower_stage = object_model.get_part("lower_stage")
    upper_flange = object_model.get_part("upper_flange")
    lower_joint = object_model.get_articulation("base_to_lower_stage")
    upper_joint = object_model.get_articulation("lower_stage_to_upper_flange")

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
        "two_vertical_revolutes",
        lower_joint.articulation_type == ArticulationType.REVOLUTE
        and upper_joint.articulation_type == ArticulationType.REVOLUTE
        and tuple(lower_joint.axis) == (0.0, 0.0, 1.0)
        and tuple(upper_joint.axis) == (0.0, 0.0, 1.0),
        details=(
            f"lower={lower_joint.articulation_type}/{lower_joint.axis}, "
            f"upper={upper_joint.articulation_type}/{upper_joint.axis}"
        ),
    )

    axis_offset = math.hypot(upper_joint.origin.xyz[0], upper_joint.origin.xyz[1])
    ctx.check(
        "upper_axis_deliberately_offset",
        0.24 <= axis_offset <= 0.31,
        details=f"upper axis offset was {axis_offset:.3f} m",
    )
    ctx.check(
        "upper_axis_carried_on_bridge_edge",
        upper_joint.origin.xyz[0] >= 0.24 and abs(upper_joint.origin.xyz[1]) >= 0.03,
        details=(
            f"upper axis position was "
            f"({upper_joint.origin.xyz[0]:.3f}, {upper_joint.origin.xyz[1]:.3f}) m"
        ),
    )

    ctx.expect_gap(
        lower_stage,
        base,
        axis="z",
        max_gap=0.002,
        max_penetration=0.0,
        name="lower_stage_seated_on_base",
    )
    ctx.expect_overlap(
        lower_stage,
        base,
        axes="xy",
        min_overlap=0.200,
        name="lower_stage_footprint_over_base",
    )
    ctx.expect_contact(
        upper_flange,
        lower_stage,
        contact_tol=0.001,
        name="upper_flange_carried_by_bridge_housing",
    )
    ctx.expect_origin_distance(
        upper_flange,
        lower_stage,
        axes="xy",
        min_dist=0.250,
        max_dist=0.300,
        name="upper_stage_xy_displacement",
    )
    ctx.expect_origin_gap(
        upper_flange,
        lower_stage,
        axis="z",
        min_gap=0.210,
        max_gap=0.235,
        name="upper_stage_bridge_elevation",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
