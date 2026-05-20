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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_WIDTH = 0.34
BASE_DEPTH = 0.24
BASE_THICKNESS = 0.03
BASE_PLATE_BOTTOM_Z = -0.07

LOWER_HOUSING_RADIUS = 0.082
LOWER_HOUSING_HEIGHT = 0.075
LOWER_HOUSING_BOTTOM_Z = -0.045
LOWER_BORE_RADIUS = 0.055

LOWER_SPINDLE_RADIUS = 0.024
LOWER_SPINDLE_BOTTOM_Z = LOWER_HOUSING_BOTTOM_Z + LOWER_HOUSING_HEIGHT
LOWER_SPINDLE_HEIGHT = 0.038
LOWER_THRUST_COLLAR_RADIUS = 0.071
LOWER_THRUST_COLLAR_Z = LOWER_HOUSING_BOTTOM_Z + LOWER_HOUSING_HEIGHT
LOWER_THRUST_COLLAR_THICKNESS = 0.008
LOWER_TABLE_RADIUS = 0.095
LOWER_TABLE_Z = 0.038
LOWER_TABLE_THICKNESS = 0.022

UPPER_AXIS_OFFSET_X = 0.17
UPPER_AXIS_HEIGHT = 0.24
UPPER_HOUSING_RADIUS = 0.055
UPPER_HOUSING_HEIGHT = 0.065
UPPER_BORE_RADIUS = 0.038

UPPER_SPINDLE_RADIUS = 0.017
UPPER_SPINDLE_BOTTOM_Z = UPPER_HOUSING_HEIGHT / 2.0
UPPER_SPINDLE_HEIGHT = 0.04
UPPER_THRUST_COLLAR_RADIUS = 0.047
UPPER_THRUST_COLLAR_Z = UPPER_HOUSING_HEIGHT / 2.0
UPPER_THRUST_COLLAR_THICKNESS = 0.008
UPPER_HUB_RADIUS = 0.045
UPPER_HUB_Z = 0.0455
UPPER_HUB_THICKNESS = 0.0145
FACEPLATE_RADIUS = 0.075
FACEPLATE_Z = 0.06
FACEPLATE_THICKNESS = 0.014


def _cylinder(radius: float, height: float, *, x: float = 0.0, y: float = 0.0, z0: float = 0.0):
    return cq.Workplane("XY").circle(radius).extrude(height).translate((x, y, z0))


def _box(size_x: float, size_y: float, size_z: float, *, x: float = 0.0, y: float = 0.0, z0: float = 0.0):
    return (
        cq.Workplane("XY")
        .rect(size_x, size_y)
        .extrude(size_z)
        .translate((x, y, z0))
    )


def _base_frame_shape():
    plate = (
        cq.Workplane("XY")
        .rect(BASE_WIDTH, BASE_DEPTH)
        .extrude(BASE_THICKNESS)
        .faces(">Z")
        .workplane()
        .pushPoints(
            [
                (-0.115, -0.075),
                (-0.115, 0.075),
                (0.115, -0.075),
                (0.115, 0.075),
            ]
        )
        .hole(0.016)
        .edges("|Z")
        .fillet(0.018)
        .translate((0.0, 0.0, BASE_PLATE_BOTTOM_Z))
    )

    pedestal = _cylinder(
        LOWER_HOUSING_RADIUS,
        LOWER_HOUSING_HEIGHT,
        z0=LOWER_HOUSING_BOTTOM_Z,
    )
    bore = _cylinder(LOWER_BORE_RADIUS, 0.09, z0=-0.03)

    return plate.union(pedestal).cut(bore)


def _carrier_shape():
    spindle = _cylinder(
        LOWER_SPINDLE_RADIUS,
        LOWER_SPINDLE_HEIGHT,
        z0=LOWER_SPINDLE_BOTTOM_Z,
    )
    thrust_collar = _cylinder(
        LOWER_THRUST_COLLAR_RADIUS,
        LOWER_THRUST_COLLAR_THICKNESS,
        z0=LOWER_THRUST_COLLAR_Z,
    )
    table = _cylinder(
        LOWER_TABLE_RADIUS,
        LOWER_TABLE_THICKNESS,
        z0=LOWER_TABLE_Z,
    )

    main_web = _box(0.046, 0.036, 0.165, x=0.098, z0=0.06)
    bridge = _box(0.09, 0.036, 0.03, x=0.125, z0=0.205)
    cheek = _box(0.034, 0.06, 0.058, x=0.151, z0=0.19)
    diagonal_rib = (
        cq.Workplane("XZ")
        .polyline(
            [
                (0.048, 0.06),
                (0.098, 0.06),
                (0.164, 0.205),
                (0.128, 0.205),
            ]
        )
        .close()
        .extrude(0.018)
        .translate((0.0, -0.009, 0.0))
    )

    upper_support_outer = _cylinder(
        UPPER_HOUSING_RADIUS,
        UPPER_HOUSING_HEIGHT,
        x=UPPER_AXIS_OFFSET_X,
        z0=UPPER_AXIS_HEIGHT - UPPER_HOUSING_HEIGHT / 2.0,
    )
    upper_support_bore = _cylinder(
        UPPER_BORE_RADIUS,
        UPPER_HOUSING_HEIGHT + 0.01,
        x=UPPER_AXIS_OFFSET_X,
        z0=UPPER_AXIS_HEIGHT - UPPER_HOUSING_HEIGHT / 2.0 - 0.005,
    )
    upper_support = upper_support_outer.cut(upper_support_bore)

    return (
        spindle.union(thrust_collar)
        .union(table)
        .union(main_web)
        .union(bridge)
        .union(cheek)
        .union(diagonal_rib)
        .union(upper_support)
    )


def _upper_stage_shape():
    spindle = _cylinder(
        UPPER_SPINDLE_RADIUS,
        UPPER_SPINDLE_HEIGHT,
        z0=UPPER_SPINDLE_BOTTOM_Z,
    )
    thrust_collar = _cylinder(
        UPPER_THRUST_COLLAR_RADIUS,
        UPPER_THRUST_COLLAR_THICKNESS,
        z0=UPPER_THRUST_COLLAR_Z,
    )
    hub = _cylinder(UPPER_HUB_RADIUS, UPPER_HUB_THICKNESS, z0=UPPER_HUB_Z)
    neck = _cylinder(0.029, 0.02, z0=0.04)

    faceplate = (
        cq.Workplane("XY")
        .circle(FACEPLATE_RADIUS)
        .extrude(FACEPLATE_THICKNESS)
        .faces(">Z")
        .workplane()
        .pushPoints(
            [
                (0.048, 0.0),
                (-0.048, 0.0),
                (0.0, 0.048),
                (0.0, -0.048),
            ]
        )
        .hole(0.012)
        .faces(">Z")
        .workplane()
        .hole(0.022)
        .translate((0.0, 0.0, FACEPLATE_Z))
    )

    return spindle.union(thrust_collar).union(neck).union(hub).union(faceplate)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="offset_axis_rotary_stack")

    model.material("base_finish", rgba=(0.22, 0.24, 0.27, 1.0))
    model.material("carrier_finish", rgba=(0.47, 0.50, 0.54, 1.0))
    model.material("faceplate_finish", rgba=(0.76, 0.79, 0.82, 1.0))

    base_frame = model.part("base_frame")
    base_frame.visual(
        mesh_from_cadquery(_base_frame_shape(), "base_frame"),
        material="base_finish",
        name="base_shell",
    )
    base_frame.inertial = Inertial.from_geometry(
        Box((BASE_WIDTH, BASE_DEPTH, 0.11)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, -0.02)),
    )

    carrier = model.part("carrier")
    carrier.visual(
        mesh_from_cadquery(_carrier_shape(), "carrier"),
        material="carrier_finish",
        name="carrier_shell",
    )
    carrier.inertial = Inertial.from_geometry(
        Box((0.25, 0.12, 0.30)),
        mass=7.5,
        origin=Origin(xyz=(0.10, 0.0, 0.12)),
    )

    upper_stage = model.part("upper_stage")
    upper_stage.visual(
        mesh_from_cadquery(_upper_stage_shape(), "upper_stage"),
        material="faceplate_finish",
        name="upper_shell",
    )
    upper_stage.inertial = Inertial.from_geometry(
        Cylinder(radius=0.075, length=0.14),
        mass=2.8,
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
    )

    model.articulation(
        "base_rotation",
        ArticulationType.REVOLUTE,
        parent=base_frame,
        child=carrier,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=45.0,
            velocity=1.4,
            lower=-2.7,
            upper=2.7,
        ),
    )
    model.articulation(
        "upper_rotation",
        ArticulationType.REVOLUTE,
        parent=carrier,
        child=upper_stage,
        origin=Origin(xyz=(UPPER_AXIS_OFFSET_X, 0.0, UPPER_AXIS_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=22.0,
            velocity=2.0,
            lower=-pi,
            upper=pi,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base_frame = object_model.get_part("base_frame")
    carrier = object_model.get_part("carrier")
    upper_stage = object_model.get_part("upper_stage")
    base_rotation = object_model.get_articulation("base_rotation")
    upper_rotation = object_model.get_articulation("upper_rotation")

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

    ctx.expect_contact(
        carrier,
        base_frame,
        contact_tol=5e-4,
        name="lower rotary stage is physically supported by the base",
    )
    ctx.expect_contact(
        upper_stage,
        carrier,
        contact_tol=5e-4,
        name="upper rotary stage is physically supported by the side arm",
    )
    ctx.expect_origin_gap(
        upper_stage,
        base_frame,
        axis="x",
        min_gap=0.16,
        max_gap=0.18,
        name="upper axis is laterally offset from the base axis in the rest pose",
    )
    ctx.expect_origin_gap(
        upper_stage,
        base_frame,
        axis="z",
        min_gap=0.23,
        max_gap=0.25,
        name="upper axis sits above the lower turntable axis",
    )

    def _max_abs_delta(a, b) -> float:
        return max(abs(a[i] - b[i]) for i in range(3))

    parallel_vertical_axes = (
        abs(base_rotation.axis[0]) < 1e-9
        and abs(base_rotation.axis[1]) < 1e-9
        and abs(base_rotation.axis[2] - 1.0) < 1e-9
        and abs(upper_rotation.axis[0]) < 1e-9
        and abs(upper_rotation.axis[1]) < 1e-9
        and abs(upper_rotation.axis[2] - 1.0) < 1e-9
    )
    ctx.check(
        "both articulations use supported parallel vertical axes",
        parallel_vertical_axes,
        details=f"base axis={base_rotation.axis}, upper axis={upper_rotation.axis}",
    )

    upper_home = ctx.part_world_position(upper_stage)
    with ctx.pose(base_rotation=1.0, upper_rotation=0.0):
        upper_swung = ctx.part_world_position(upper_stage)
    lower_stage_orbits_offset_axis = (
        upper_home is not None
        and upper_swung is not None
        and upper_swung[1] > 0.12
        and upper_swung[0] < upper_home[0]
        and abs(upper_swung[2] - upper_home[2]) < 1e-6
    )
    ctx.check(
        "base stage carries the offset upper axis around the lower axis",
        lower_stage_orbits_offset_axis,
        details=f"home={upper_home}, swung={upper_swung}",
    )

    with ctx.pose(base_rotation=0.9, upper_rotation=0.0):
        upper_pose_a = ctx.part_world_position(upper_stage)
    with ctx.pose(base_rotation=0.9, upper_rotation=1.3):
        upper_pose_b = ctx.part_world_position(upper_stage)
    upper_spins_in_place = (
        upper_pose_a is not None
        and upper_pose_b is not None
        and _max_abs_delta(upper_pose_a, upper_pose_b) < 1e-9
    )
    ctx.check(
        "upper stage rotates about its own supported axis without translating",
        upper_spins_in_place,
        details=f"pose_a={upper_pose_a}, pose_b={upper_pose_b}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
