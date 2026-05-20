from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_thermostat")

    plate_width = 0.240
    plate_height = 0.170
    plate_radius = 0.018
    plate_thickness = 0.006

    plate_geom = ExtrudeGeometry.from_z0(
        rounded_rect_profile(plate_width, plate_height, plate_radius),
        plate_thickness,
        cap=True,
        closed=True,
    )
    plate_geom.rotate_x(-math.pi / 2.0)

    body_profile = [
        (0.000, 0.000),
        (0.044, 0.000),
        (0.050, 0.003),
        (0.050, 0.012),
        (0.042, 0.018),
        (0.000, 0.018),
    ]
    body_geom = LatheGeometry(body_profile, segments=72, closed=True)
    body_geom.rotate_x(-math.pi / 2.0)

    dial_profile = [
        (0.058, 0.001),
        (0.070, 0.001),
        (0.078, 0.005),
        (0.078, 0.024),
        (0.070, 0.029),
        (0.058, 0.029),
        (0.054, 0.023),
        (0.054, 0.007),
    ]
    dial_geom = LatheGeometry(dial_profile, segments=96, closed=True)
    dial_geom.rotate_x(-math.pi / 2.0)

    off_white = model.material("off_white", rgba=(0.93, 0.93, 0.91, 1.0))
    warm_grey = model.material("warm_grey", rgba=(0.82, 0.82, 0.80, 1.0))
    charcoal = model.material("charcoal", rgba=(0.22, 0.23, 0.24, 1.0))
    smoked_glass = model.material("smoked_glass", rgba=(0.14, 0.16, 0.17, 0.92))
    satin_silver = model.material("satin_silver", rgba=(0.77, 0.79, 0.81, 1.0))

    body_assembly = model.part("body_assembly")
    body_assembly.visual(
        mesh_from_geometry(plate_geom, "thermostat_wall_plate"),
        material=off_white,
        name="wall_plate",
    )
    body_assembly.visual(
        mesh_from_geometry(body_geom, "thermostat_body_shell"),
        origin=Origin(xyz=(0.0, plate_thickness, 0.0)),
        material=warm_grey,
        name="body_shell",
    )
    body_assembly.visual(
        Cylinder(radius=0.031, length=0.010),
        origin=Origin(
            xyz=(0.0, plate_thickness + 0.018 + 0.005, 0.0),
            rpy=(-math.pi / 2.0, 0.0, 0.0),
        ),
        material=smoked_glass,
        name="center_cap",
    )
    body_assembly.visual(
        Box((0.022, 0.002, 0.010)),
        origin=Origin(xyz=(0.0, plate_thickness + 0.001, 0.073)),
        material=charcoal,
        name="index_marker",
    )
    body_assembly.inertial = Inertial.from_geometry(
        Box((plate_width, 0.034, plate_height)),
        mass=0.42,
        origin=Origin(xyz=(0.0, 0.017, 0.0)),
    )

    dial_ring = model.part("dial_ring")
    dial_ring.visual(
        mesh_from_geometry(dial_geom, "thermostat_dial_ring"),
        origin=Origin(xyz=(0.0, plate_thickness, 0.0)),
        material=satin_silver,
        name="outer_ring",
    )
    dial_ring.inertial = Inertial.from_geometry(
        Box((0.156, 0.029, 0.156)),
        mass=0.11,
        origin=Origin(xyz=(0.0, plate_thickness + 0.015, 0.0)),
    )

    model.articulation(
        "body_to_dial",
        ArticulationType.CONTINUOUS,
        parent=body_assembly,
        child=dial_ring,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.25, velocity=5.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body_assembly")
    dial = object_model.get_part("dial_ring")
    dial_joint = object_model.get_articulation("body_to_dial")

    ctx.check(
        "dial articulation is continuous",
        dial_joint.joint_type == ArticulationType.CONTINUOUS
        and dial_joint.motion_limits is not None
        and dial_joint.motion_limits.lower is None
        and dial_joint.motion_limits.upper is None,
        details=(
            f"type={dial_joint.joint_type}, "
            f"limits={dial_joint.motion_limits}"
        ),
    )
    ctx.expect_origin_distance(
        dial,
        body,
        axes="xz",
        max_dist=0.0001,
        name="dial stays centered on the thermostat face",
    )
    ctx.expect_within(
        dial,
        body,
        axes="xz",
        margin=0.0,
        name="dial ring remains inside the wall plate outline",
    )
    ctx.expect_overlap(
        dial,
        body,
        axes="xz",
        min_overlap=0.145,
        name="dial ring dominates the front face",
    )

    rest_pos = ctx.part_world_position(dial)
    with ctx.pose({dial_joint: math.pi * 0.75}):
        ctx.expect_origin_distance(
            dial,
            body,
            axes="xz",
            max_dist=0.0001,
            name="turned dial remains concentric",
        )
        turned_pos = ctx.part_world_position(dial)

    center_locked = (
        rest_pos is not None
        and turned_pos is not None
        and abs(rest_pos[0] - turned_pos[0]) <= 1e-6
        and abs(rest_pos[1] - turned_pos[1]) <= 1e-6
        and abs(rest_pos[2] - turned_pos[2]) <= 1e-6
    )
    ctx.check(
        "dial rotation does not translate the part",
        center_locked,
        details=f"rest={rest_pos}, turned={turned_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
