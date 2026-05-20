from __future__ import annotations

import math

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
import cadquery as cq


def _handle_shell() -> cq.Workplane:
    """Rounded utility-knife handle with a real open blade track."""
    length = 0.180
    width = 0.038
    height = 0.026

    body = cq.Workplane("XY").box(length, width, height)
    body = body.edges("|X").fillet(0.0055)
    body = body.edges("|Z").fillet(0.0030)

    # A long open trough surrounds the blade carriage instead of hiding it in a
    # solid block.  The cut opens through the top and the front nose.
    track_cut = (
        cq.Workplane("XY")
        .box(0.162, 0.012, 0.026)
        .translate((0.016, 0.0, 0.004))
    )
    return body.cut(track_cut)


def _blade_mesh() -> cq.Workplane:
    """Thin trapezoidal utility blade, authored in the X-Z profile plane."""
    profile = [
        (-0.002, -0.0055),
        (0.090, -0.0055),
        (0.112, -0.0005),
        (0.088, 0.0045),
        (0.005, 0.0045),
        (-0.004, 0.0015),
    ]
    blade = cq.Workplane("XZ").polyline(profile).close().extrude(0.003, both=True)
    bevel_cut = cq.Workplane("XZ").polyline(
        [(0.083, -0.0055), (0.112, -0.0005), (0.086, 0.0012)]
    ).close().extrude(0.0032, both=True)
    return blade.union(bevel_cut)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="broad_utility_knife")

    handle_mat = model.material("safety_orange_plastic", rgba=(1.0, 0.48, 0.08, 1.0))
    rubber_mat = model.material("matte_black_rubber", rgba=(0.015, 0.014, 0.013, 1.0))
    dark_mat = model.material("dark_anodized_metal", rgba=(0.06, 0.065, 0.07, 1.0))
    steel_mat = model.material("brushed_steel", rgba=(0.72, 0.72, 0.68, 1.0))
    lever_mat = model.material("red_lock_lever", rgba=(0.75, 0.06, 0.035, 1.0))

    handle = model.part("handle")
    handle.visual(
        mesh_from_cadquery(_handle_shell(), "handle_shell", tolerance=0.0007),
        material=handle_mat,
        name="handle_shell",
    )
    handle.visual(
        Box((0.146, 0.009, 0.0012)),
        origin=Origin(xyz=(0.014, 0.0, -0.0084)),
        material=dark_mat,
        name="track_floor",
    )
    handle.visual(
        Box((0.058, 0.0012, 0.012)),
        origin=Origin(xyz=(-0.046, 0.0196, -0.002)),
        material=rubber_mat,
        name="side_grip",
    )
    handle.visual(
        Box((0.052, 0.0012, 0.010)),
        origin=Origin(xyz=(-0.042, -0.0196, -0.002)),
        material=rubber_mat,
        name="opposite_grip",
    )
    for x, z, name in ((-0.072, 0.004, "rear_screw"), (0.066, -0.004, "front_screw")):
        handle.visual(
            Cylinder(radius=0.0035, length=0.0014),
            origin=Origin(xyz=(x, 0.0197, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_mat,
            name=name,
        )
    handle.visual(
        Cylinder(radius=0.0046, length=0.0020),
        origin=Origin(xyz=(0.006, 0.0200, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_mat,
        name="pivot_boss",
    )

    carriage = model.part("blade_carriage")
    carriage.visual(
        Box((0.070, 0.0060, 0.0065)),
        origin=Origin(xyz=(-0.018, 0.0, -0.00455)),
        material=dark_mat,
        name="carriage_rail",
    )
    carriage.visual(
        mesh_from_cadquery(_blade_mesh(), "blade", tolerance=0.0004),
        material=steel_mat,
        name="blade",
    )
    carriage.visual(
        Box((0.013, 0.0060, 0.010)),
        origin=Origin(xyz=(-0.030, 0.0, 0.0013)),
        material=dark_mat,
        name="slider_stem",
    )
    carriage.visual(
        Box((0.022, 0.0074, 0.009)),
        origin=Origin(xyz=(-0.030, 0.0, 0.0100)),
        material=dark_mat,
        name="slider_neck",
    )
    carriage.visual(
        Box((0.030, 0.019, 0.0040)),
        origin=Origin(xyz=(-0.030, 0.0, 0.0165)),
        material=rubber_mat,
        name="thumb_pad",
    )
    for dx in (-0.010, 0.0, 0.010):
        carriage.visual(
            Box((0.0020, 0.0185, 0.0012)),
            origin=Origin(xyz=(-0.030 + dx, 0.0, 0.0191)),
            material=dark_mat,
            name=f"thumb_ridge_{int((dx + 0.010) * 1000):02d}",
        )

    model.articulation(
        "handle_to_blade_carriage",
        ArticulationType.PRISMATIC,
        parent=handle,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=0.18, lower=0.0, upper=0.035),
    )

    lever = model.part("lock_lever")
    lever.visual(
        Cylinder(radius=0.0056, length=0.0030),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=lever_mat,
        name="lever_collar",
    )
    lever.visual(
        Box((0.047, 0.0030, 0.0070)),
        origin=Origin(xyz=(0.022, 0.0, 0.000)),
        material=lever_mat,
        name="lever_bar",
    )
    lever.visual(
        Box((0.012, 0.0034, 0.010)),
        origin=Origin(xyz=(0.045, 0.0, 0.000)),
        material=lever_mat,
        name="lever_tip",
    )

    model.articulation(
        "handle_to_lock_lever",
        ArticulationType.REVOLUTE,
        parent=handle,
        child=lever,
        origin=Origin(xyz=(0.006, 0.0225, 0.000)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.5, lower=-0.45, upper=0.45),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    handle = object_model.get_part("handle")
    carriage = object_model.get_part("blade_carriage")
    lever = object_model.get_part("lock_lever")
    slide = object_model.get_articulation("handle_to_blade_carriage")
    lever_joint = object_model.get_articulation("handle_to_lock_lever")

    ctx.expect_within(
        carriage,
        handle,
        axes="yz",
        inner_elem="carriage_rail",
        outer_elem="handle_shell",
        margin=0.001,
        name="carriage rail stays inside the handle track envelope",
    )
    ctx.expect_overlap(
        carriage,
        handle,
        axes="x",
        elem_a="carriage_rail",
        elem_b="handle_shell",
        min_overlap=0.040,
        name="retracted carriage remains retained in the track",
    )
    rest_pos = ctx.part_world_position(carriage)
    with ctx.pose({slide: 0.035}):
        ctx.expect_overlap(
            carriage,
            handle,
            axes="x",
            elem_a="carriage_rail",
            elem_b="handle_shell",
            min_overlap=0.030,
            name="extended carriage remains retained in the track",
        )
        extended_pos = ctx.part_world_position(carriage)
    ctx.check(
        "blade carriage slides forward along the handle axis",
        rest_pos is not None and extended_pos is not None and extended_pos[0] > rest_pos[0] + 0.030,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    ctx.expect_gap(
        lever,
        handle,
        axis="y",
        positive_elem="lever_collar",
        negative_elem="pivot_boss",
        max_gap=0.0008,
        max_penetration=0.000001,
        name="lock lever is seated on the side pivot boss",
    )
    rest_bar = ctx.part_element_world_aabb(lever, elem="lever_bar")
    with ctx.pose({lever_joint: 0.35}):
        rotated_bar = ctx.part_element_world_aabb(lever, elem="lever_bar")
    if rest_bar is not None and rotated_bar is not None:
        rest_z = (rest_bar[0][2] + rest_bar[1][2]) * 0.5
        rotated_z = (rotated_bar[0][2] + rotated_bar[1][2]) * 0.5
        lever_moves = rotated_z < rest_z - 0.006
    else:
        lever_moves = False
    ctx.check(
        "lock lever rotates about its side pivot",
        lever_moves,
        details=f"rest_bar={rest_bar}, rotated_bar={rotated_bar}",
    )

    return ctx.report()


object_model = build_object_model()
