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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_thermostat_with_large_dial")

    wall_plate_white = model.material(
        "wall_plate_white",
        rgba=(0.95, 0.95, 0.94, 1.0),
    )
    housing_soft_white = model.material(
        "housing_soft_white",
        rgba=(0.88, 0.89, 0.87, 1.0),
    )
    trim_grey = model.material(
        "trim_grey",
        rgba=(0.73, 0.75, 0.76, 1.0),
    )
    dial_silver = model.material(
        "dial_silver",
        rgba=(0.76, 0.79, 0.81, 1.0),
    )
    lens_black = model.material(
        "lens_black",
        rgba=(0.13, 0.14, 0.15, 1.0),
    )
    marker_amber = model.material(
        "marker_amber",
        rgba=(0.82, 0.62, 0.33, 1.0),
    )

    wall_body = model.part("wall_body")

    wall_plate_mesh = mesh_from_geometry(
        ExtrudeGeometry.from_z0(
            rounded_rect_profile(0.154, 0.154, 0.014),
            0.003,
            cap=True,
            closed=True,
        ),
        "thermostat_wall_plate",
    )
    wall_body.visual(
        wall_plate_mesh,
        material=wall_plate_white,
        name="wall_plate",
    )

    body_shell_mesh = mesh_from_geometry(
        ExtrudeGeometry.from_z0(
            rounded_rect_profile(0.106, 0.106, 0.022),
            0.023,
            cap=True,
            closed=True,
        ),
        "thermostat_body_shell",
    )
    wall_body.visual(
        body_shell_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=housing_soft_white,
        name="body_shell",
    )
    wall_body.visual(
        Cylinder(radius=0.035, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.022)),
        material=trim_grey,
        name="center_pad",
    )
    wall_body.visual(
        Box((0.013, 0.026, 0.018)),
        origin=Origin(xyz=(-0.046, 0.0, 0.019)),
        material=housing_soft_white,
        name="left_cheek",
    )
    wall_body.visual(
        Box((0.013, 0.026, 0.018)),
        origin=Origin(xyz=(0.046, 0.0, 0.019)),
        material=housing_soft_white,
        name="right_cheek",
    )
    wall_body.visual(
        Box((0.008, 0.014, 0.003)),
        origin=Origin(xyz=(0.0, 0.042, 0.0245)),
        material=marker_amber,
        name="index_marker",
    )
    wall_body.inertial = Inertial.from_geometry(
        Box((0.154, 0.154, 0.028)),
        mass=0.34,
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
    )

    dial = model.part("dial")
    dial.visual(
        Cylinder(radius=0.056, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=dial_silver,
        name="dial_ring",
    )
    dial.visual(
        Cylinder(radius=0.044, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=trim_grey,
        name="dial_face",
    )
    dial.visual(
        Cylinder(radius=0.028, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
        material=lens_black,
        name="display_lens",
    )
    dial.visual(
        Box((0.003, 0.020, 0.0015)),
        origin=Origin(xyz=(0.0, 0.035, 0.01875)),
        material=marker_amber,
        name="dial_pointer",
    )
    dial.inertial = Inertial.from_geometry(
        Cylinder(radius=0.056, length=0.018),
        mass=0.12,
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
    )

    model.articulation(
        "wall_body_to_dial",
        ArticulationType.CONTINUOUS,
        parent=wall_body,
        child=dial,
        origin=Origin(xyz=(0.0, 0.0, 0.028)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.25,
            velocity=4.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    wall_body = object_model.get_part("wall_body")
    dial = object_model.get_part("dial")
    dial_joint = object_model.get_articulation("wall_body_to_dial")

    ctx.expect_origin_distance(
        dial,
        wall_body,
        axes="xy",
        max_dist=1e-6,
        name="dial remains centered on the housing",
    )
    ctx.expect_overlap(
        dial,
        wall_body,
        axes="xy",
        min_overlap=0.10,
        name="dial stays within the thermostat face footprint",
    )
    ctx.expect_gap(
        dial,
        wall_body,
        axis="z",
        min_gap=0.0,
        max_gap=0.001,
        name="dial seats flush against the support zone without penetration",
    )

    rest_pos = ctx.part_world_position(dial)
    with ctx.pose({dial_joint: math.pi * 0.5}):
        ctx.expect_origin_distance(
            dial,
            wall_body,
            axes="xy",
            max_dist=1e-6,
            name="dial rotation keeps the center axis fixed",
        )
        ctx.expect_overlap(
            dial,
            wall_body,
            axes="xy",
            min_overlap=0.10,
            name="turned dial remains captured by the body footprint",
        )
        turned_pos = ctx.part_world_position(dial)

    ctx.check(
        "dial turns in place",
        rest_pos is not None
        and turned_pos is not None
        and abs(rest_pos[0] - turned_pos[0]) < 1e-6
        and abs(rest_pos[1] - turned_pos[1]) < 1e-6
        and abs(rest_pos[2] - turned_pos[2]) < 1e-6,
        details=f"rest={rest_pos}, turned={turned_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
