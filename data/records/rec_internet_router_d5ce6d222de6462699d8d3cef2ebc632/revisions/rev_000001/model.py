from __future__ import annotations

import math

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


BODY_X = 0.095
BODY_Y = 0.060
BODY_Z = 0.038


def _rounded_box(size: tuple[float, float, float], radius: float) -> cq.Workplane:
    """Small filleted consumer-electronics block, authored in meters."""
    return cq.Workplane("XY").box(*size).edges().fillet(radius)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_travel_router")

    matte_white = model.material("matte_white", rgba=(0.86, 0.88, 0.87, 1.0))
    dark_plastic = model.material("dark_plastic", rgba=(0.015, 0.017, 0.020, 1.0))
    soft_black = model.material("soft_black", rgba=(0.035, 0.037, 0.040, 1.0))
    brushed_metal = model.material("brushed_metal", rgba=(0.82, 0.78, 0.68, 1.0))
    led_green = model.material("led_green", rgba=(0.1, 0.95, 0.28, 1.0))
    led_blue = model.material("led_blue", rgba=(0.15, 0.42, 1.0, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_cadquery(_rounded_box((BODY_X, BODY_Y, BODY_Z), 0.0045), "router_rounded_shell"),
        material=matte_white,
        name="rounded_shell",
    )

    # Router face details: dark status strip, an Ethernet-style jack, and two LEDs.
    body.visual(
        Box((0.065, 0.0012, 0.010)),
        origin=Origin(xyz=(-0.006, BODY_Y / 2 + 0.00015, -0.001)),
        material=dark_plastic,
        name="front_status_strip",
    )
    body.visual(
        Box((0.022, 0.0016, 0.011)),
        origin=Origin(xyz=(0.023, BODY_Y / 2 + 0.00045, -0.0015)),
        material=soft_black,
        name="ethernet_socket",
    )
    for i, (x_pos, mat) in enumerate(((-0.028, led_green), (-0.021, led_blue))):
        body.visual(
            Cylinder(radius=0.00145, length=0.0010),
            origin=Origin(
                xyz=(x_pos, BODY_Y / 2 + 0.00075, 0.0018),
                rpy=(-math.pi / 2, 0.0, 0.0),
            ),
            material=mat,
            name=f"status_led_{i}",
        )

    # The plug lives in a shallow dark tray on the -X wall.
    body.visual(
        Box((0.0012, 0.034, 0.036)),
        origin=Origin(xyz=(-BODY_X / 2 - 0.0006, 0.0, -0.002)),
        material=dark_plastic,
        name="plug_tray",
    )

    plug_hinge_x = -BODY_X / 2 - 0.0027
    plug_hinge_z = 0.014
    plug_offsets_y = (-0.008, 0.008)
    for i, y_pos in enumerate(plug_offsets_y):
        body.visual(
            Cylinder(radius=0.0018, length=0.0075),
            origin=Origin(
                xyz=(plug_hinge_x, y_pos, plug_hinge_z),
                rpy=(math.pi / 2, 0.0, 0.0),
            ),
            material=brushed_metal,
            name=f"plug_hinge_pin_{i}",
        )

    # A small forked hinge mount on the opposite +X top edge carries the antenna.
    ant_joint_x = BODY_X / 2 + 0.0040
    ant_joint_y = -0.020
    ant_joint_z = BODY_Z / 2 + 0.0060
    body.visual(
        Box((0.010, 0.018, 0.0030)),
        origin=Origin(xyz=(BODY_X / 2 - 0.0010, ant_joint_y, BODY_Z / 2 + 0.0010)),
        material=dark_plastic,
        name="antenna_pedestal",
    )
    for i, y_pos in enumerate((ant_joint_y - 0.0050, ant_joint_y + 0.0050)):
        body.visual(
            Box((0.006, 0.0020, 0.0080)),
            origin=Origin(xyz=(BODY_X / 2 + 0.0020, y_pos, ant_joint_z - 0.0002)),
            material=dark_plastic,
            name=f"antenna_fork_{i}",
        )

    plug_blade_mesh = _rounded_box((0.00145, 0.0052, 0.030), 0.00045)
    for i, y_pos in enumerate(plug_offsets_y):
        blade = model.part(f"plug_blade_{i}")
        blade.visual(
            mesh_from_cadquery(plug_blade_mesh, f"plug_blade_{i}_leaf"),
            origin=Origin(xyz=(0.0, 0.0, -0.0168)),
            material=brushed_metal,
            name="blade_leaf",
        )
        model.articulation(
            f"body_to_plug_blade_{i}",
            ArticulationType.REVOLUTE,
            parent=body,
            child=blade,
            origin=Origin(xyz=(plug_hinge_x, y_pos, plug_hinge_z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=0.35, velocity=3.0, lower=0.0, upper=math.radians(92.0)),
        )

    antenna = model.part("antenna")
    antenna.visual(
        Cylinder(radius=0.0030, length=0.0080),
        origin=Origin(rpy=(math.pi / 2, 0.0, 0.0)),
        material=soft_black,
        name="antenna_barrel",
    )
    antenna.visual(
        mesh_from_cadquery(_rounded_box((0.056, 0.009, 0.005), 0.0017), "antenna_bar"),
        origin=Origin(xyz=(-0.028, 0.0, 0.0)),
        material=soft_black,
        name="antenna_bar",
    )
    model.articulation(
        "body_to_antenna",
        ArticulationType.REVOLUTE,
        parent=body,
        child=antenna,
        origin=Origin(xyz=(ant_joint_x, ant_joint_y, ant_joint_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.55, velocity=2.5, lower=0.0, upper=math.radians(105.0)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    antenna = object_model.get_part("antenna")
    antenna_joint = object_model.get_articulation("body_to_antenna")

    for i in (0, 1):
        blade = object_model.get_part(f"plug_blade_{i}")
        ctx.expect_overlap(
            blade,
            body,
            axes="yz",
            elem_a="blade_leaf",
            elem_b="plug_tray",
            min_overlap=0.004,
            name=f"folded plug blade {i} sits in the side tray",
        )
        ctx.expect_gap(
            body,
            blade,
            axis="x",
            positive_elem="plug_tray",
            negative_elem="blade_leaf",
            min_gap=0.0005,
            max_gap=0.004,
            name=f"folded plug blade {i} clears the body wall",
        )

    ctx.expect_gap(
        antenna,
        body,
        axis="z",
        positive_elem="antenna_bar",
        negative_elem="rounded_shell",
        min_gap=0.001,
        max_gap=0.006,
        name="folded antenna rides just above the top shell",
    )
    ctx.expect_overlap(
        antenna,
        body,
        axes="xy",
        elem_a="antenna_bar",
        elem_b="rounded_shell",
        min_overlap=0.006,
        name="folded antenna lies along the opposite top edge",
    )

    for i in (0, 1):
        blade = object_model.get_part(f"plug_blade_{i}")
        joint = object_model.get_articulation(f"body_to_plug_blade_{i}")
        rest_aabb = ctx.part_element_world_aabb(blade, elem="blade_leaf")
        with ctx.pose({joint: math.radians(88.0)}):
            deployed_aabb = ctx.part_element_world_aabb(blade, elem="blade_leaf")
        ctx.check(
            f"plug blade {i} rotates outward",
            rest_aabb is not None
            and deployed_aabb is not None
            and deployed_aabb[0][0] < rest_aabb[0][0] - 0.020,
            details=f"rest_aabb={rest_aabb}, deployed_aabb={deployed_aabb}",
        )

    folded_aabb = ctx.part_element_world_aabb(antenna, elem="antenna_bar")
    with ctx.pose({antenna_joint: math.radians(95.0)}):
        raised_aabb = ctx.part_element_world_aabb(antenna, elem="antenna_bar")
    ctx.check(
        "antenna hinge raises the short antenna",
        folded_aabb is not None
        and raised_aabb is not None
        and raised_aabb[1][2] > folded_aabb[1][2] + 0.035,
        details=f"folded_aabb={folded_aabb}, raised_aabb={raised_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
