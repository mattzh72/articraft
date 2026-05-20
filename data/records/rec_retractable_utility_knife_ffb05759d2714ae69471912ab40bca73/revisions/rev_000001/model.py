from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _snap_blade_geometry() -> cq.Workplane:
    """Thin side-profile snap-off blade with an angled breakaway tip."""
    # The blade is authored in the child part frame.  X is the sliding axis,
    # Z is blade height, and the small extrusion thickness is along Y.
    points_xz = [
        (0.020, 0.0075),
        (0.126, 0.0075),
        (0.137, -0.0075),
        (0.020, -0.0075),
    ]
    return cq.Workplane("XZ").polyline(points_xz).close().extrude(0.0012, both=True)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="chunky_snap_off_utility_knife")

    orange = model.material("safety_orange_plastic", rgba=(0.95, 0.33, 0.08, 1.0))
    dark = model.material("dark_rubber", rgba=(0.04, 0.045, 0.05, 1.0))
    graphite = model.material("graphite_plastic", rgba=(0.18, 0.18, 0.19, 1.0))
    metal = model.material("brushed_steel", rgba=(0.78, 0.80, 0.80, 1.0))
    groove = model.material("engraved_shadow", rgba=(0.10, 0.10, 0.11, 1.0))

    # Root: a straight, chunky open channel shell.  The central Y gap and top
    # slot leave clearance for the blade, carrier neck, and thumb pad.
    shell = model.part("shell")
    shell.visual(
        Box((0.180, 0.007, 0.030)),
        origin=Origin(xyz=(0.0, -0.0155, 0.0)),
        material=orange,
        name="side_wall_near",
    )
    shell.visual(
        Box((0.180, 0.007, 0.030)),
        origin=Origin(xyz=(0.0, 0.0155, 0.0)),
        material=orange,
        name="side_wall_far",
    )
    shell.visual(
        Box((0.180, 0.038, 0.005)),
        origin=Origin(xyz=(0.0, 0.0, -0.0125)),
        material=orange,
        name="bottom_spine",
    )
    shell.visual(
        Box((0.145, 0.010, 0.005)),
        origin=Origin(xyz=(-0.012, -0.014, 0.0125)),
        material=orange,
        name="top_rail_near",
    )
    shell.visual(
        Box((0.145, 0.010, 0.005)),
        origin=Origin(xyz=(-0.012, 0.014, 0.0125)),
        material=orange,
        name="top_rail_far",
    )
    shell.visual(
        Box((0.008, 0.038, 0.030)),
        origin=Origin(xyz=(-0.086, 0.0, 0.0)),
        material=orange,
        name="rear_cap",
    )
    shell.visual(
        Box((0.074, 0.0012, 0.012)),
        origin=Origin(xyz=(-0.032, -0.01935, -0.001)),
        material=dark,
        name="side_grip_inset",
    )
    shell.visual(
        Cylinder(radius=0.0032, length=0.0015),
        origin=Origin(xyz=(-0.060, -0.0188, 0.006), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=graphite,
        name="rear_screw_head",
    )
    shell.visual(
        Cylinder(radius=0.0032, length=0.0015),
        origin=Origin(xyz=(0.060, -0.0188, 0.006), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=graphite,
        name="front_screw_head",
    )

    blade_carrier = model.part("blade_carrier")
    blade_carrier.visual(
        mesh_from_cadquery(_snap_blade_geometry(), "segmented_snap_blade", tolerance=0.0004),
        material=metal,
        name="blade_plate",
    )
    # Dark diagonal score marks on the visible face make the blade read as a
    # snap-off segmented blade rather than a plain strip.
    for i, x in enumerate((0.045, 0.065, 0.085, 0.105)):
        blade_carrier.visual(
            Box((0.020, 0.0009, 0.00075)),
            origin=Origin(
                xyz=(x, -0.0012, 0.0),
                rpy=(0.0, math.radians(36.0), 0.0),
            ),
            material=groove,
            name=f"score_line_{i}",
        )
    blade_carrier.visual(
        Box((0.050, 0.012, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=graphite,
        name="internal_carrier",
    )
    blade_carrier.visual(
        Box((0.017, 0.009, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=graphite,
        name="slider_neck",
    )
    blade_carrier.visual(
        Box((0.034, 0.026, 0.007)),
        origin=Origin(xyz=(0.0, 0.0, 0.0185)),
        material=dark,
        name="thumb_pad",
    )
    for i, x in enumerate((-0.011, -0.0035, 0.004, 0.0115)):
        blade_carrier.visual(
            Box((0.0032, 0.014, 0.0018)),
            origin=Origin(xyz=(x, 0.0, 0.0228)),
            material=graphite,
            name=f"thumb_rib_{i}",
        )

    model.articulation(
        "shell_to_carrier",
        ArticulationType.PRISMATIC,
        parent=shell,
        child=blade_carrier,
        origin=Origin(xyz=(-0.012, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=0.18, lower=0.0, upper=0.045),
    )

    lock_wheel = model.part("lock_wheel")
    wheel_geometry = KnobGeometry(
        0.020,
        0.006,
        body_style="faceted",
        edge_radius=0.0005,
        grip=KnobGrip(style="ribbed", count=18, depth=0.0006, width=0.0012),
    )
    lock_wheel.visual(
        mesh_from_geometry(wheel_geometry, "ribbed_lock_wheel"),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=graphite,
        name="wheel_disc",
    )
    lock_wheel.visual(
        Cylinder(radius=0.003, length=0.018),
        origin=Origin(xyz=(0.0, 0.006, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="wheel_axle",
    )

    model.articulation(
        "shell_to_lock_wheel",
        ArticulationType.REVOLUTE,
        parent=shell,
        child=lock_wheel,
        origin=Origin(xyz=(0.030, -0.022, 0.003)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=4.0, lower=-math.pi, upper=math.pi),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    shell = object_model.get_part("shell")
    carrier = object_model.get_part("blade_carrier")
    wheel = object_model.get_part("lock_wheel")
    slide = object_model.get_articulation("shell_to_carrier")
    wheel_joint = object_model.get_articulation("shell_to_lock_wheel")

    ctx.allow_overlap(
        shell,
        wheel,
        elem_a="side_wall_near",
        elem_b="wheel_axle",
        reason="The lock-wheel axle is intentionally seated through the side wall as a captured shaft.",
    )
    ctx.expect_overlap(
        shell,
        wheel,
        axes="xz",
        elem_a="side_wall_near",
        elem_b="wheel_axle",
        min_overlap=0.003,
        name="lock axle passes through the side wall footprint",
    )
    ctx.expect_contact(
        shell,
        wheel,
        elem_a="side_wall_near",
        elem_b="wheel_disc",
        contact_tol=0.0008,
        name="lock wheel disc bears on the shell side",
    )

    ctx.expect_contact(
        carrier,
        shell,
        elem_a="thumb_pad",
        elem_b="top_rail_near",
        contact_tol=0.0006,
        name="thumb slider rides on the top rail",
    )

    rest_pos = ctx.part_world_position(carrier)
    rest_blade = ctx.part_element_world_aabb(carrier, elem="blade_plate")
    shell_box = ctx.part_world_aabb(shell)
    if rest_blade is not None and shell_box is not None:
        ctx.check(
            "blade protrudes through the nose opening at rest",
            rest_blade[1][0] > shell_box[1][0] + 0.025,
            details=f"blade_aabb={rest_blade}, shell_aabb={shell_box}",
        )

    with ctx.pose({slide: 0.045, wheel_joint: math.pi / 2.0}):
        extended_pos = ctx.part_world_position(carrier)
        extended_blade = ctx.part_element_world_aabb(carrier, elem="blade_plate")
        if rest_pos is not None and extended_pos is not None:
            ctx.check(
                "blade carrier slides forward along the body axis",
                extended_pos[0] > rest_pos[0] + 0.040,
                details=f"rest={rest_pos}, extended={extended_pos}",
            )
        if extended_blade is not None and shell_box is not None:
            ctx.check(
                "extended blade remains visibly longer than the nose",
                extended_blade[1][0] > shell_box[1][0] + 0.065,
                details=f"extended_blade={extended_blade}, shell_aabb={shell_box}",
            )
        ctx.expect_overlap(
            carrier,
            shell,
            axes="x",
            elem_a="blade_plate",
            elem_b="side_wall_near",
            min_overlap=0.025,
            name="extended blade keeps retained insertion in the channel",
        )

    return ctx.report()


object_model = build_object_model()
