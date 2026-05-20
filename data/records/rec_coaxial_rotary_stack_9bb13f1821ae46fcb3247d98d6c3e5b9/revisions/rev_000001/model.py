from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import cos, pi, sin

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


PEDESTAL_FOOT_RADIUS = 0.125
PEDESTAL_FOOT_HEIGHT = 0.016
PEDESTAL_BODY_RADIUS = 0.095
PEDESTAL_BODY_HEIGHT = 0.050
PEDESTAL_COLLAR_RADIUS = 0.058
PEDESTAL_COLLAR_HEIGHT = 0.010
SPINDLE_RADIUS = 0.018
SPINDLE_HEIGHT = 0.255

LOWER_JOINT_Z = PEDESTAL_FOOT_HEIGHT + PEDESTAL_BODY_HEIGHT + PEDESTAL_COLLAR_HEIGHT

LOWER_TABLE_RADIUS = 0.165
LOWER_TABLE_BORE = 0.024
LOWER_TABLE_THICKNESS = 0.014
LOWER_HUB_RADIUS = 0.050
LOWER_HUB_HEIGHT = 0.072
LOWER_SUPPORT_RADIUS = 0.052
LOWER_SUPPORT_HEIGHT = 0.008
MIDDLE_JOINT_Z = LOWER_HUB_HEIGHT + LOWER_SUPPORT_HEIGHT

MIDDLE_BORE = 0.0215
MIDDLE_BOTTOM_RADIUS = 0.048
MIDDLE_BOTTOM_HEIGHT = 0.008
MIDDLE_HUB_RADIUS = 0.034
MIDDLE_HUB_HEIGHT = 0.060
MIDDLE_TOP_SUPPORT_RADIUS = 0.040
MIDDLE_TOP_SUPPORT_HEIGHT = 0.008
MIDDLE_RING_OUTER_RADIUS = 0.122
MIDDLE_RING_INNER_RADIUS = 0.090
MIDDLE_RING_Z = 0.034
MIDDLE_RING_HEIGHT = 0.018
MIDDLE_ARM_LENGTH = 0.076
MIDDLE_ARM_WIDTH = 0.018
MIDDLE_ARM_HEIGHT = 0.012
MIDDLE_ARM_CENTER = 0.060
MIDDLE_ARM_Z = 0.037
TOP_JOINT_Z = MIDDLE_HUB_HEIGHT + MIDDLE_TOP_SUPPORT_HEIGHT

TOP_FLANGE_RADIUS = 0.073
TOP_FLANGE_BORE = 0.0215
TOP_FLANGE_THICKNESS = 0.012
TOP_HUB_RADIUS = 0.038
TOP_HUB_HEIGHT = 0.024


def _annulus(outer_radius: float, inner_radius: float, height: float, *, z: float = 0.0) -> cq.Workplane:
    shape = cq.Workplane("XY").circle(outer_radius).extrude(height)
    shape = shape.cut(_cylinder(inner_radius, height + 0.002, z=-0.001))
    return shape.translate((0.0, 0.0, z)) if z else shape


def _cylinder(radius: float, height: float, *, z: float = 0.0) -> cq.Workplane:
    shape = cq.Workplane("XY").circle(radius).extrude(height)
    return shape.translate((0.0, 0.0, z)) if z else shape


def _bolt_cutters(
    bolt_circle_radius: float,
    hole_radius: float,
    count: int,
    height: float,
    *,
    z: float = 0.0,
) -> cq.Workplane:
    points = [
        (
            bolt_circle_radius * cos(2.0 * pi * idx / count),
            bolt_circle_radius * sin(2.0 * pi * idx / count),
        )
        for idx in range(count)
    ]
    return cq.Workplane("XY").workplane(offset=z).pushPoints(points).circle(hole_radius).extrude(height)


def _build_pedestal_spindle_shape() -> cq.Workplane:
    pedestal = (
        _cylinder(PEDESTAL_FOOT_RADIUS, PEDESTAL_FOOT_HEIGHT)
        .union(_cylinder(PEDESTAL_BODY_RADIUS, PEDESTAL_BODY_HEIGHT, z=PEDESTAL_FOOT_HEIGHT))
        .union(_cylinder(PEDESTAL_COLLAR_RADIUS, PEDESTAL_COLLAR_HEIGHT, z=LOWER_JOINT_Z - PEDESTAL_COLLAR_HEIGHT))
        .union(_cylinder(0.030, LOWER_JOINT_Z))
    )
    foot_holes = _bolt_cutters(0.085, 0.008, 4, PEDESTAL_FOOT_HEIGHT + 0.002, z=-0.001)
    return pedestal.cut(foot_holes)


def _build_lower_table_shape() -> cq.Workplane:
    table = (
        _cylinder(LOWER_TABLE_RADIUS, LOWER_TABLE_THICKNESS)
        .union(_cylinder(LOWER_HUB_RADIUS, LOWER_HUB_HEIGHT))
        .union(
            _cylinder(
                LOWER_SUPPORT_RADIUS,
                LOWER_SUPPORT_HEIGHT,
                z=LOWER_HUB_HEIGHT,
            )
        )
    )
    bolt_holes = _bolt_cutters(0.118, 0.005, 6, LOWER_TABLE_THICKNESS + 0.002, z=-0.001)
    return table.cut(bolt_holes)


def _build_middle_ring_shape() -> cq.Workplane:
    middle = (
        _cylinder(MIDDLE_BOTTOM_RADIUS, MIDDLE_BOTTOM_HEIGHT)
        .union(_cylinder(MIDDLE_HUB_RADIUS, MIDDLE_HUB_HEIGHT))
        .union(
            _cylinder(
                MIDDLE_TOP_SUPPORT_RADIUS,
                MIDDLE_TOP_SUPPORT_HEIGHT,
                z=MIDDLE_HUB_HEIGHT,
            )
        )
        .union(
            _annulus(
                MIDDLE_RING_OUTER_RADIUS,
                MIDDLE_RING_INNER_RADIUS,
                MIDDLE_RING_HEIGHT,
                z=MIDDLE_RING_Z,
            )
        )
    )
    arm = (
        cq.Workplane("XY")
        .box(
            MIDDLE_ARM_LENGTH,
            MIDDLE_ARM_WIDTH,
            MIDDLE_ARM_HEIGHT,
            centered=(True, True, False),
        )
        .translate((MIDDLE_ARM_CENTER, 0.0, MIDDLE_ARM_Z))
    )
    for angle_deg in (0.0, 120.0, 240.0):
        middle = middle.union(arm.rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), angle_deg))
    return middle


def _build_top_flange_shape() -> cq.Workplane:
    flange = (
        _cylinder(TOP_FLANGE_RADIUS, TOP_FLANGE_THICKNESS)
        .union(_cylinder(TOP_HUB_RADIUS, TOP_HUB_HEIGHT))
    )
    bolt_holes = _bolt_cutters(0.046, 0.004, 4, TOP_FLANGE_THICKNESS + 0.002, z=-0.001)
    return flange.cut(bolt_holes)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pedestal_coaxial_rotary_column")

    model.material("pedestal_paint", rgba=(0.22, 0.24, 0.27, 1.0))
    model.material("machined_table", rgba=(0.76, 0.78, 0.80, 1.0))
    model.material("anodized_ring", rgba=(0.57, 0.61, 0.66, 1.0))
    model.material("flange_steel", rgba=(0.83, 0.84, 0.86, 1.0))

    pedestal_spindle = model.part("pedestal_spindle")
    pedestal_spindle.visual(
        mesh_from_cadquery(_build_pedestal_spindle_shape(), "pedestal_spindle"),
        material="pedestal_paint",
        name="pedestal_shell",
    )
    pedestal_spindle.inertial = Inertial.from_geometry(
        Cylinder(radius=PEDESTAL_FOOT_RADIUS, length=SPINDLE_HEIGHT),
        mass=6.2,
        origin=Origin(xyz=(0.0, 0.0, SPINDLE_HEIGHT / 2.0)),
    )

    lower_table = model.part("lower_table")
    lower_table.visual(
        mesh_from_cadquery(_build_lower_table_shape(), "lower_table"),
        material="machined_table",
        name="lower_table_shell",
    )
    lower_table.inertial = Inertial.from_geometry(
        Cylinder(radius=LOWER_TABLE_RADIUS, length=MIDDLE_JOINT_Z),
        mass=2.0,
        origin=Origin(xyz=(0.0, 0.0, MIDDLE_JOINT_Z / 2.0)),
    )

    middle_ring = model.part("middle_ring")
    middle_ring.visual(
        mesh_from_cadquery(_build_middle_ring_shape(), "middle_ring"),
        material="anodized_ring",
        name="middle_ring_shell",
    )
    middle_ring.inertial = Inertial.from_geometry(
        Cylinder(radius=MIDDLE_RING_OUTER_RADIUS, length=TOP_JOINT_Z),
        mass=1.1,
        origin=Origin(xyz=(0.0, 0.0, TOP_JOINT_Z / 2.0)),
    )

    top_flange = model.part("top_flange")
    top_flange.visual(
        mesh_from_cadquery(_build_top_flange_shape(), "top_flange"),
        material="flange_steel",
        name="top_flange_shell",
    )
    top_flange.inertial = Inertial.from_geometry(
        Cylinder(radius=TOP_FLANGE_RADIUS, length=TOP_HUB_HEIGHT),
        mass=0.55,
        origin=Origin(xyz=(0.0, 0.0, TOP_HUB_HEIGHT / 2.0)),
    )

    model.articulation(
        "pedestal_to_lower_table",
        ArticulationType.REVOLUTE,
        parent=pedestal_spindle,
        child=lower_table,
        origin=Origin(xyz=(0.0, 0.0, LOWER_JOINT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=-pi, upper=pi, effort=40.0, velocity=1.8),
    )
    model.articulation(
        "lower_table_to_middle_ring",
        ArticulationType.REVOLUTE,
        parent=lower_table,
        child=middle_ring,
        origin=Origin(xyz=(0.0, 0.0, MIDDLE_JOINT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=-2.6, upper=2.6, effort=24.0, velocity=2.0),
    )
    model.articulation(
        "middle_ring_to_top_flange",
        ArticulationType.REVOLUTE,
        parent=middle_ring,
        child=top_flange,
        origin=Origin(xyz=(0.0, 0.0, TOP_JOINT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=-pi, upper=pi, effort=16.0, velocity=2.4),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    pedestal_spindle = object_model.get_part("pedestal_spindle")
    lower_table = object_model.get_part("lower_table")
    middle_ring = object_model.get_part("middle_ring")
    top_flange = object_model.get_part("top_flange")

    lower_joint = object_model.get_articulation("pedestal_to_lower_table")
    middle_joint = object_model.get_articulation("lower_table_to_middle_ring")
    top_joint = object_model.get_articulation("middle_ring_to_top_flange")

    joints = (lower_joint, middle_joint, top_joint)
    ctx.check(
        "all rotary stages use the common vertical axis",
        all(joint.axis == (0.0, 0.0, 1.0) for joint in joints),
        details=str([joint.axis for joint in joints]),
    )
    ctx.check(
        "all rotary stages have bidirectional travel",
        all(
            joint.motion_limits is not None
            and joint.motion_limits.lower is not None
            and joint.motion_limits.upper is not None
            and joint.motion_limits.lower < 0.0
            and joint.motion_limits.upper > 0.0
            for joint in joints
        ),
        details=str(
            [
                (
                    joint.name,
                    None if joint.motion_limits is None else joint.motion_limits.lower,
                    None if joint.motion_limits is None else joint.motion_limits.upper,
                )
                for joint in joints
            ]
        ),
    )

    ctx.expect_contact(
        lower_table,
        pedestal_spindle,
        name="lower table is seated on the pedestal collar",
    )
    ctx.expect_contact(
        middle_ring,
        lower_table,
        name="middle ring is seated on the lower table support",
    )
    ctx.expect_contact(
        top_flange,
        middle_ring,
        name="top flange is seated on the middle ring support",
    )

    for part_name, part_obj in (
        ("lower table", lower_table),
        ("middle ring", middle_ring),
        ("top flange", top_flange),
    ):
        ctx.expect_origin_distance(
            part_obj,
            pedestal_spindle,
            axes="xy",
            max_dist=1e-6,
            name=f"{part_name} stays centered on the spindle axis",
        )

    ctx.expect_overlap(
        lower_table,
        pedestal_spindle,
        axes="xy",
        min_overlap=0.11,
        name="lower table plan view remains coaxial with pedestal",
    )
    ctx.expect_overlap(
        middle_ring,
        pedestal_spindle,
        axes="xy",
        min_overlap=0.16,
        name="middle ring plan view remains coaxial with pedestal",
    )
    ctx.expect_overlap(
        top_flange,
        pedestal_spindle,
        axes="xy",
        min_overlap=0.12,
        name="top flange plan view remains coaxial with pedestal",
    )

    top_rest = ctx.part_world_position(top_flange)
    middle_rest = ctx.part_world_position(middle_ring)
    with ctx.pose(
        {
            lower_joint: 1.15,
            middle_joint: -0.85,
            top_joint: 1.70,
        }
    ):
        ctx.expect_contact(
            lower_table,
            pedestal_spindle,
            name="lower table remains supported while rotating",
        )
        ctx.expect_contact(
            middle_ring,
            lower_table,
            name="middle ring remains supported while rotating",
        )
        ctx.expect_contact(
            top_flange,
            middle_ring,
            name="top flange remains supported while rotating",
        )
        top_posed = ctx.part_world_position(top_flange)
        middle_posed = ctx.part_world_position(middle_ring)

    ctx.check(
        "top flange origin does not orbit away from the spindle axis",
        top_rest is not None
        and top_posed is not None
        and abs(top_rest[0] - top_posed[0]) < 1e-6
        and abs(top_rest[1] - top_posed[1]) < 1e-6,
        details=f"rest={top_rest}, posed={top_posed}",
    )
    ctx.check(
        "middle ring origin does not orbit away from the spindle axis",
        middle_rest is not None
        and middle_posed is not None
        and abs(middle_rest[0] - middle_posed[0]) < 1e-6
        and abs(middle_rest[1] - middle_posed[1]) < 1e-6,
        details=f"rest={middle_rest}, posed={middle_posed}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
