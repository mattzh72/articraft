from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


HANDLE_LENGTH = 0.170
HANDLE_WIDTH = 0.034
HANDLE_HEIGHT = 0.026
HINGE_X = -0.028
HINGE_Y = 0.0196
HINGE_Z = -0.0090
SLIDE_ORIGIN_X = -0.055
SLIDE_TRAVEL = 0.040


def _handle_shell_geometry() -> cq.Workplane:
    """Rounded handle shell with a front blade channel and a side storage pocket."""

    body = cq.Workplane("XY").box(HANDLE_LENGTH, HANDLE_WIDTH, HANDLE_HEIGHT)
    body = body.edges("|X").fillet(0.0045)
    body = body.edges(">X or <X").fillet(0.0030)

    # Straight internal blade-carrier channel, blind at the rear and open at the nose.
    channel = (
        cq.Workplane("XY")
        .box(0.154, 0.014, 0.014)
        .translate((0.015, 0.0, 0.000))
    )
    body = body.cut(channel)

    # Narrow top slot for the thumb slider post.
    top_slot = (
        cq.Workplane("XY")
        .box(0.074, 0.007, 0.014)
        .translate((0.014, 0.0, 0.010))
    )
    body = body.cut(top_slot)

    # Shallow side cavity behind the hinged spare-blade cover.
    side_pocket = (
        cq.Workplane("XY")
        .box(0.074, 0.010, 0.018)
        .translate((HINGE_X, 0.0160, 0.0010))
    )
    body = body.cut(side_pocket)

    return body


def _blade_geometry() -> cq.Workplane:
    """Thin trapezoidal utility blade in the sliding carrier's local frame."""

    blade_profile = [
        (0.080, -0.0045),
        (0.124, -0.0045),
        (0.137, 0.0010),
        (0.130, 0.0060),
        (0.080, 0.0060),
    ]
    blade = cq.Workplane("XZ").polyline(blade_profile).close().extrude(0.0020, both=True)
    notch = cq.Workplane("XZ").center(0.101, 0.0010).circle(0.0022).extrude(
        0.0030, both=True
    )
    return blade.cut(notch)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="retractable_utility_knife")

    shell_mat = model.material("safety_orange_shell", rgba=(0.95, 0.30, 0.04, 1.0))
    dark_mat = model.material("matte_black_plastic", rgba=(0.015, 0.015, 0.014, 1.0))
    rubber_mat = model.material("dark_rubber", rgba=(0.03, 0.03, 0.032, 1.0))
    steel_mat = model.material("brushed_steel", rgba=(0.72, 0.72, 0.68, 1.0))
    door_mat = model.material("dark_gray_cover", rgba=(0.16, 0.17, 0.18, 1.0))

    handle = model.part("handle_shell")
    handle.visual(
        mesh_from_cadquery(_handle_shell_geometry(), "handle_body", tolerance=0.0008),
        material=shell_mat,
        name="handle_body",
    )
    # Dark recess floor makes the spare-blade cavity read as hollow behind the door.
    handle.visual(
        Box((0.066, 0.0010, 0.014)),
        origin=Origin(xyz=(HINGE_X, 0.0112, 0.0010)),
        material=dark_mat,
        name="storage_cavity_back",
    )
    # Two fixed hinge mounts bridge the side shell to the exposed hinge barrels.
    for idx, x in enumerate((HINGE_X - 0.043, HINGE_X + 0.043)):
        handle.visual(
            Box((0.012, 0.0060, 0.0050)),
            origin=Origin(xyz=(x, 0.0176, HINGE_Z)),
            material=shell_mat,
            name=f"hinge_mount_{idx}",
        )
        handle.visual(
            Cylinder(radius=0.0018, length=0.010),
            origin=Origin(xyz=(x, HINGE_Y, HINGE_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=steel_mat,
            name=f"shell_hinge_barrel_{idx}",
        )

    blade_carrier = model.part("blade_carrier")
    blade_carrier.visual(
        Box((0.100, 0.010, 0.006)),
        origin=Origin(xyz=(0.050, 0.0, 0.000)),
        material=dark_mat,
        name="carrier_rail",
    )
    blade_carrier.visual(
        mesh_from_cadquery(_blade_geometry(), "utility_blade", tolerance=0.00035),
        material=steel_mat,
        name="utility_blade",
    )
    blade_carrier.visual(
        Box((0.010, 0.0042, 0.014)),
        origin=Origin(xyz=(0.045, 0.0, 0.010)),
        material=dark_mat,
        name="slider_stem",
    )
    blade_carrier.visual(
        Box((0.028, 0.015, 0.005)),
        origin=Origin(xyz=(0.045, 0.0, 0.0195)),
        material=rubber_mat,
        name="thumb_slider",
    )
    for idx, y in enumerate((-0.0060, 0.0060)):
        blade_carrier.visual(
            Box((0.026, 0.0030, 0.0060)),
            origin=Origin(xyz=(0.045, y, 0.0160)),
            material=rubber_mat,
            name=f"slider_foot_{idx}",
        )
    for idx, dx in enumerate((-0.009, -0.003, 0.003, 0.009)):
        blade_carrier.visual(
            Box((0.0020, 0.013, 0.0016)),
            origin=Origin(xyz=(0.045 + dx, 0.0, 0.0226)),
            material=dark_mat,
            name=f"slider_rib_{idx}",
        )

    storage_door = model.part("storage_door")
    storage_door.visual(
        Box((0.074, 0.0026, 0.020)),
        origin=Origin(xyz=(0.0, 0.00135, 0.010)),
        material=door_mat,
        name="door_panel",
    )
    storage_door.visual(
        Cylinder(radius=0.00175, length=0.032),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel_mat,
        name="door_hinge_barrel",
    )
    for idx, x in enumerate((-0.020, 0.020)):
        storage_door.visual(
            Cylinder(radius=0.0020, length=0.0009),
            origin=Origin(xyz=(x, 0.0031, 0.010), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=steel_mat,
            name=f"door_screw_{idx}",
        )

    model.articulation(
        "handle_to_blade_carrier",
        ArticulationType.PRISMATIC,
        parent=handle,
        child=blade_carrier,
        origin=Origin(xyz=(SLIDE_ORIGIN_X, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=0.35, lower=0.0, upper=SLIDE_TRAVEL),
    )
    model.articulation(
        "handle_to_storage_door",
        ArticulationType.REVOLUTE,
        parent=handle,
        child=storage_door,
        origin=Origin(xyz=(HINGE_X, HINGE_Y, HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.5, lower=0.0, upper=1.25),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    handle = object_model.get_part("handle_shell")
    carrier = object_model.get_part("blade_carrier")
    door = object_model.get_part("storage_door")
    slide = object_model.get_articulation("handle_to_blade_carrier")
    hinge = object_model.get_articulation("handle_to_storage_door")

    ctx.expect_within(
        carrier,
        handle,
        axes="yz",
        inner_elem="carrier_rail",
        outer_elem="handle_body",
        margin=0.001,
        name="carrier rail is retained in the straight handle channel",
    )
    ctx.expect_gap(
        carrier,
        handle,
        axis="z",
        positive_elem="thumb_slider",
        negative_elem="handle_body",
        min_gap=0.001,
        max_gap=0.006,
        name="top thumb slider rides above the shell slot",
    )
    ctx.expect_gap(
        door,
        handle,
        axis="y",
        positive_elem="door_panel",
        negative_elem="handle_body",
        min_gap=0.0,
        max_gap=0.006,
        name="side storage door sits just proud of the handle side",
    )
    ctx.expect_overlap(
        door,
        handle,
        axes="xz",
        elem_a="door_panel",
        elem_b="handle_body",
        min_overlap=0.012,
        name="side door covers the storage cavity footprint",
    )

    blade_rest = ctx.part_element_world_aabb(carrier, elem="utility_blade")
    handle_aabb = ctx.part_element_world_aabb(handle, elem="handle_body")
    rest_pos = ctx.part_world_position(carrier)
    with ctx.pose({slide: SLIDE_TRAVEL}):
        ctx.expect_overlap(
            carrier,
            handle,
            axes="x",
            elem_a="carrier_rail",
            elem_b="handle_body",
            min_overlap=0.070,
            name="extended blade carrier remains captured in the channel",
        )
        blade_extended = ctx.part_element_world_aabb(carrier, elem="utility_blade")
        extended_pos = ctx.part_world_position(carrier)

    ctx.check(
        "blade carrier extends toward the knife nose",
        rest_pos is not None
        and extended_pos is not None
        and extended_pos[0] > rest_pos[0] + 0.030,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )
    ctx.check(
        "blade is retracted at rest and protrudes when slid forward",
        blade_rest is not None
        and blade_extended is not None
        and handle_aabb is not None
        and blade_rest[1][0] <= handle_aabb[1][0] + 0.002
        and blade_extended[1][0] >= handle_aabb[1][0] + 0.030,
        details=f"blade_rest={blade_rest}, blade_extended={blade_extended}, handle={handle_aabb}",
    )

    door_closed = ctx.part_element_world_aabb(door, elem="door_panel")
    with ctx.pose({hinge: 1.10}):
        door_open = ctx.part_element_world_aabb(door, elem="door_panel")
    ctx.check(
        "storage door swings outward on its side hinge",
        door_closed is not None
        and door_open is not None
        and door_open[1][1] > door_closed[1][1] + 0.010,
        details=f"closed={door_closed}, open={door_open}",
    )

    return ctx.report()


object_model = build_object_model()
