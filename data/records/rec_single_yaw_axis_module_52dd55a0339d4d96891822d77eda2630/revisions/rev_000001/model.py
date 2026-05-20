from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import isclose

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


FOOT_LENGTH = 0.300
FOOT_WIDTH = 0.220
FOOT_HEIGHT = 0.030
FOOT_CORNER_RADIUS = 0.018
FOOT_MOUNT_HOLE_D = 0.012

PEDESTAL_LENGTH = 0.094
PEDESTAL_WIDTH = 0.074
PEDESTAL_HEIGHT = 0.105
PEDESTAL_CORNER_RADIUS = 0.010

HOUSING_BASE_RADIUS = 0.062
HOUSING_BASE_HEIGHT = 0.014
HOUSING_DRUM_RADIUS = 0.057
HOUSING_DRUM_HEIGHT = 0.040
HOUSING_CAVITY_RADIUS = 0.038
HOUSING_CAVITY_DEPTH = 0.032

ROTOR_SKIRT_RADIUS = 0.034
ROTOR_SKIRT_HEIGHT = 0.028
OUTPUT_PLATE_LENGTH = 0.080
OUTPUT_PLATE_WIDTH = 0.066
OUTPUT_PLATE_THICKNESS = 0.014
OUTPUT_PLATE_CORNER_RADIUS = 0.008
OUTPUT_BOSS_RADIUS = 0.017
OUTPUT_BOSS_HEIGHT = 0.008
OUTPUT_HOLE_D = 0.006

YAW_ORIGIN_Z = FOOT_HEIGHT + PEDESTAL_HEIGHT + HOUSING_BASE_HEIGHT + HOUSING_DRUM_HEIGHT


def _rounded_plate(length: float, width: float, corner_radius: float, thickness: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .rect(length, width)
        .extrude(thickness)
        .edges("|Z")
        .fillet(corner_radius)
    )


def _make_support_geometry() -> tuple[cq.Workplane, cq.Workplane, cq.Workplane]:
    mount_points = [
        (0.095, 0.065),
        (0.095, -0.065),
        (-0.095, 0.065),
        (-0.095, -0.065),
    ]

    foot = (
        cq.Workplane("XY")
        .rect(FOOT_LENGTH, FOOT_WIDTH)
        .extrude(FOOT_HEIGHT)
        .edges("|Z")
        .fillet(FOOT_CORNER_RADIUS)
    )
    foot = (
        foot.faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .pushPoints(mount_points)
        .hole(FOOT_MOUNT_HOLE_D)
    )

    pedestal = (
        cq.Workplane("XY")
        .rect(PEDESTAL_LENGTH, PEDESTAL_WIDTH)
        .extrude(PEDESTAL_HEIGHT)
        .edges("|Z")
        .fillet(PEDESTAL_CORNER_RADIUS)
        .faces(">Z")
        .edges()
        .chamfer(0.004)
        .translate((0.0, 0.0, FOOT_HEIGHT))
    )

    housing = (
        cq.Workplane("XY")
        .circle(HOUSING_BASE_RADIUS)
        .extrude(HOUSING_BASE_HEIGHT)
        .faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .circle(HOUSING_DRUM_RADIUS)
        .extrude(HOUSING_DRUM_HEIGHT)
    )
    housing = housing.faces(">Z").edges().chamfer(0.003)
    housing = (
        housing.faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .circle(HOUSING_CAVITY_RADIUS)
        .cutBlind(-HOUSING_CAVITY_DEPTH)
        .translate((0.0, 0.0, FOOT_HEIGHT + PEDESTAL_HEIGHT))
    )

    return foot, pedestal, housing


def _make_rotor_geometry() -> tuple[cq.Workplane, cq.Workplane, cq.Workplane]:
    rotor_skirt = (
        cq.Workplane("XY")
        .workplane(offset=-ROTOR_SKIRT_HEIGHT)
        .circle(ROTOR_SKIRT_RADIUS)
        .extrude(ROTOR_SKIRT_HEIGHT)
    )
    rotor_skirt = rotor_skirt.faces("<Z").edges().chamfer(0.002)

    output_plate = _rounded_plate(
        OUTPUT_PLATE_LENGTH,
        OUTPUT_PLATE_WIDTH,
        OUTPUT_PLATE_CORNER_RADIUS,
        OUTPUT_PLATE_THICKNESS,
    )
    output_plate = (
        output_plate.faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .pushPoints(
            [
                (0.024, 0.018),
                (0.024, -0.018),
                (-0.024, 0.018),
                (-0.024, -0.018),
            ]
        )
        .hole(OUTPUT_HOLE_D)
    )

    output_boss = (
        cq.Workplane("XY")
        .workplane(offset=OUTPUT_PLATE_THICKNESS)
        .circle(OUTPUT_BOSS_RADIUS)
        .extrude(OUTPUT_BOSS_HEIGHT)
        .faces(">Z")
        .edges()
        .chamfer(0.0015)
    )

    return rotor_skirt, output_plate, output_boss


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tower_yaw_rotary_module")

    base_paint = model.material("base_paint", rgba=(0.24, 0.25, 0.27, 1.0))
    output_metal = model.material("output_metal", rgba=(0.70, 0.72, 0.74, 1.0))
    spindle_metal = model.material("spindle_metal", rgba=(0.50, 0.52, 0.56, 1.0))

    support = model.part("support")
    foot, pedestal, housing = _make_support_geometry()
    support.visual(mesh_from_cadquery(foot, "support_foot"), material=base_paint, name="foot_shell")
    support.visual(
        mesh_from_cadquery(pedestal, "support_pedestal"),
        material=base_paint,
        name="pedestal_shell",
    )
    support.visual(
        mesh_from_cadquery(housing, "support_housing"),
        material=base_paint,
        name="housing_shell",
    )
    support.inertial = Inertial.from_geometry(
        Box((FOOT_LENGTH, FOOT_WIDTH, YAW_ORIGIN_Z)),
        mass=8.5,
        origin=Origin(xyz=(0.0, 0.0, YAW_ORIGIN_Z * 0.5)),
    )

    rotor = model.part("rotor")
    rotor_skirt, output_plate, output_boss = _make_rotor_geometry()
    rotor.visual(
        mesh_from_cadquery(rotor_skirt, "rotor_skirt"),
        material=spindle_metal,
        name="rotor_skirt",
    )
    rotor.visual(
        mesh_from_cadquery(output_plate, "rotor_output_plate"),
        material=output_metal,
        name="output_face",
    )
    rotor.visual(
        mesh_from_cadquery(output_boss, "rotor_output_boss"),
        material=output_metal,
        name="output_boss",
    )
    rotor.inertial = Inertial.from_geometry(
        Cylinder(radius=0.045, length=0.055),
        mass=1.6,
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
    )

    model.articulation(
        "support_to_rotor_yaw",
        ArticulationType.REVOLUTE,
        parent=support,
        child=rotor,
        origin=Origin(xyz=(0.0, 0.0, YAW_ORIGIN_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=45.0,
            velocity=2.0,
            lower=-3.0,
            upper=3.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support = object_model.get_part("support")
    rotor = object_model.get_part("rotor")
    yaw = object_model.get_articulation("support_to_rotor_yaw")
    housing_shell = support.get_visual("housing_shell")
    output_face = rotor.get_visual("output_face")

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

    limits = yaw.motion_limits
    axis_is_vertical = yaw.axis == (0.0, 0.0, 1.0)
    has_reasonable_span = (
        limits is not None
        and limits.lower is not None
        and limits.upper is not None
        and limits.lower < 0.0 < limits.upper
        and (limits.upper - limits.lower) >= 5.5
    )
    origin_is_on_support_top = all(
        isclose(value, target, abs_tol=1e-9)
        for value, target in zip(yaw.origin.xyz, (0.0, 0.0, YAW_ORIGIN_Z))
    )
    ctx.check(
        "yaw_joint_is_vertical_revolute_module",
        yaw.articulation_type == ArticulationType.REVOLUTE
        and axis_is_vertical
        and has_reasonable_span
        and origin_is_on_support_top,
        details=(
            f"type={yaw.articulation_type}, axis={yaw.axis}, "
            f"origin={yaw.origin.xyz}, limits={limits}"
        ),
    )

    ctx.expect_contact(
        rotor,
        support,
        elem_a=output_face,
        elem_b=housing_shell,
        name="output_face_bears_on_housing",
    )
    ctx.expect_gap(
        rotor,
        support,
        axis="z",
        positive_elem=output_face,
        negative_elem=housing_shell,
        min_gap=0.0,
        max_gap=0.001,
        name="output_face_seated_on_axis_package",
    )
    ctx.expect_within(
        rotor,
        support,
        axes="xy",
        inner_elem=output_face,
        outer_elem=housing_shell,
        margin=0.0,
        name="moving_member_smaller_than_support",
    )

    with ctx.pose({yaw: 1.0}):
        ctx.expect_gap(
            rotor,
            support,
            axis="z",
            positive_elem=output_face,
            negative_elem=housing_shell,
            min_gap=0.0,
            max_gap=0.001,
            name="output_face_keeps_bearing_height_when_yawed",
        )
        ctx.expect_within(
            rotor,
            support,
            axes="xy",
            inner_elem=output_face,
            outer_elem=housing_shell,
            margin=0.001,
            name="output_face_stays_inside_support_envelope_when_yawed",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
