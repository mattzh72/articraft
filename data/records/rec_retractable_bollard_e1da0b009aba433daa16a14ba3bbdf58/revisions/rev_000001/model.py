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


def _socket_shell() -> cq.Workplane:
    """One connected, hollow, above/below-grade reinforced ground socket."""
    base_hole = cq.Workplane("XY").circle(0.195).extrude(0.12).translate((0, 0, -0.03))
    base_plate = (
        cq.Workplane("XY")
        .box(0.82, 0.82, 0.06)
        .translate((0, 0, 0.03))
        .cut(base_hole)
    )
    buried_sleeve = (
        cq.Workplane("XY")
        .circle(0.30)
        .circle(0.18)
        .extrude(1.20)
        .translate((0, 0, -1.02))
    )
    rim_collar = (
        cq.Workplane("XY")
        .circle(0.42)
        .circle(0.18)
        .extrude(0.10)
        .translate((0, 0, 0.13))
    )
    return base_plate.union(buried_sleeve).union(rim_collar)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="heavy_duty_anti_vehicle_bollard")

    galvanized = Material("galvanized_steel", rgba=(0.43, 0.46, 0.45, 1.0))
    dark_steel = Material("dark_phosphate_steel", rgba=(0.04, 0.045, 0.045, 1.0))
    safety_yellow = Material("safety_yellow_powdercoat", rgba=(1.0, 0.72, 0.04, 1.0))
    black_rubber = Material("black_keyway_shadow", rgba=(0.0, 0.0, 0.0, 1.0))
    worn_edge = Material("worn_bright_steel", rgba=(0.74, 0.72, 0.66, 1.0))

    socket = model.part("socket")
    socket.visual(
        mesh_from_cadquery(_socket_shell(), "reinforced_ground_socket", tolerance=0.0015),
        material=galvanized,
        name="socket_shell",
    )

    # Four welded radial gussets and oversized anchor bolts make the collar read
    # as a perimeter-security ground installation rather than a light post base.
    for idx, angle in enumerate((0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0)):
        x = 0.325 * math.cos(angle)
        y = 0.325 * math.sin(angle)
        socket.visual(
            Box((0.22, 0.045, 0.22)),
            origin=Origin(xyz=(x, y, 0.12), rpy=(0.0, 0.0, angle)),
            material=galvanized,
            name=f"gusset_{idx}",
        )
        px = 0.167 * math.cos(angle)
        py = 0.167 * math.sin(angle)
        socket.visual(
            Box((0.030, 0.055, 0.140)),
            origin=Origin(xyz=(px, py, 0.065), rpy=(0.0, 0.0, angle)),
            material=worn_edge,
            name=f"guide_pad_{idx}",
        )

    for idx, (x, y) in enumerate(((0.31, 0.31), (-0.31, 0.31), (-0.31, -0.31), (0.31, -0.31))):
        socket.visual(
            Cylinder(radius=0.043, length=0.010),
            origin=Origin(xyz=(x, y, 0.064)),
            material=worn_edge,
            name=f"washer_{idx}",
        )
        socket.visual(
            Cylinder(radius=0.027, length=0.030),
            origin=Origin(xyz=(x, y, 0.080)),
            material=dark_steel,
            name=f"anchor_bolt_{idx}",
        )

    # Socket-side hinge hardware for the small debris flap at the rim.
    socket.visual(
        Cylinder(radius=0.008, length=0.34),
        origin=Origin(xyz=(0.0, 0.445, 0.255), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="cover_pin",
    )
    for idx, x in enumerate((-0.155, 0.155)):
        socket.visual(
            Box((0.036, 0.135, 0.070)),
            origin=Origin(xyz=(x, 0.430, 0.251)),
            material=galvanized,
            name=f"hinge_ear_{idx}",
        )

    ram = model.part("ram")
    ram.visual(
        Cylinder(radius=0.155, length=1.32),
        origin=Origin(xyz=(0.0, 0.0, 0.23)),
        material=safety_yellow,
        name="ram_body",
    )
    for idx, z in enumerate((0.04, 0.36, 0.68)):
        ram.visual(
            Cylinder(radius=0.158, length=0.042),
            origin=Origin(xyz=(0.0, 0.0, z)),
            material=dark_steel,
            name=f"wear_band_{idx}",
        )
    ram.visual(
        Cylinder(radius=0.28, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 0.925)),
        material=dark_steel,
        name="top_crown",
    )
    ram.visual(
        Cylinder(radius=0.168, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.890)),
        material=worn_edge,
        name="crown_weld",
    )

    debris_cover = model.part("debris_cover")
    debris_cover.visual(
        Box((0.24, 0.16, 0.018)),
        origin=Origin(xyz=(0.0, -0.080, -0.016)),
        material=dark_steel,
        name="cover_plate",
    )
    debris_cover.visual(
        Cylinder(radius=0.018, length=0.25),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="hinge_barrel",
    )
    debris_cover.visual(
        Box((0.16, 0.025, 0.010)),
        origin=Origin(xyz=(0.0, -0.080, -0.002)),
        material=worn_edge,
        name="raised_lip",
    )

    key_insert = model.part("key_insert")
    key_insert.visual(
        Cylinder(radius=0.066, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=worn_edge,
        name="key_cap",
    )
    key_insert.visual(
        Cylinder(radius=0.025, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, -0.0275)),
        material=worn_edge,
        name="key_stem",
    )
    key_insert.visual(
        Box((0.018, 0.085, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.022)),
        material=black_rubber,
        name="key_slot",
    )
    key_insert.visual(
        Cylinder(radius=0.023, length=0.004),
        origin=Origin(xyz=(0.0, 0.034, 0.023)),
        material=black_rubber,
        name="key_bow",
    )

    model.articulation(
        "socket_to_ram",
        ArticulationType.PRISMATIC,
        parent=socket,
        child=ram,
        origin=Origin(xyz=(0.0, 0.0, 0.18)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18000.0, velocity=0.28, lower=-0.55, upper=0.0),
    )
    model.articulation(
        "rim_to_debris_cover",
        ArticulationType.REVOLUTE,
        parent=socket,
        child=debris_cover,
        origin=Origin(xyz=(0.0, 0.445, 0.255)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.0, lower=0.0, upper=1.65),
    )
    model.articulation(
        "ram_to_key_insert",
        ArticulationType.REVOLUTE,
        parent=ram,
        child=key_insert,
        origin=Origin(xyz=(0.0, 0.0, 0.960)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=4.0, lower=-1.57, upper=1.57),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    socket = object_model.get_part("socket")
    ram = object_model.get_part("ram")
    cover = object_model.get_part("debris_cover")
    key = object_model.get_part("key_insert")
    slide = object_model.get_articulation("socket_to_ram")
    cover_hinge = object_model.get_articulation("rim_to_debris_cover")
    key_turn = object_model.get_articulation("ram_to_key_insert")

    ctx.allow_overlap(
        socket,
        cover,
        elem_a="cover_pin",
        elem_b="hinge_barrel",
        reason="The debris-cover hinge pin is intentionally captured inside the cover barrel.",
    )
    ctx.expect_within(
        socket,
        cover,
        axes="yz",
        inner_elem="cover_pin",
        outer_elem="hinge_barrel",
        margin=0.002,
        name="hinge pin sits inside the debris-cover barrel",
    )
    ctx.expect_overlap(
        socket,
        cover,
        axes="x",
        elem_a="cover_pin",
        elem_b="hinge_barrel",
        min_overlap=0.22,
        name="debris-cover hinge has captured pin length",
    )

    for idx in range(4):
        ctx.allow_overlap(
            socket,
            ram,
            elem_a=f"guide_pad_{idx}",
            elem_b="ram_body",
            reason="Preloaded socket guide pads lightly bear on the sliding ram to keep it grounded and guided.",
        )
        ctx.expect_overlap(
            socket,
            ram,
            axes="z",
            elem_a=f"guide_pad_{idx}",
            elem_b="ram_body",
            min_overlap=0.12,
            name=f"guide pad {idx} engages the ram over useful height",
        )

    ctx.allow_overlap(
        ram,
        key,
        elem_a="top_crown",
        elem_b="key_stem",
        reason="The key insert has a short spindle seated in the crown plate bore.",
    )
    ctx.expect_within(
        key,
        ram,
        axes="xy",
        inner_elem="key_stem",
        outer_elem="top_crown",
        margin=0.0,
        name="key spindle is centered in the crown plate",
    )
    ctx.expect_overlap(
        key,
        ram,
        axes="z",
        elem_a="key_stem",
        elem_b="top_crown",
        min_overlap=0.045,
        name="key spindle remains inserted in the crown plate",
    )

    ctx.expect_overlap(
        ram,
        socket,
        axes="z",
        elem_a="ram_body",
        elem_b="socket_shell",
        min_overlap=0.35,
        name="deployed ram retains deep insertion in the socket",
    )

    raised_position = ctx.part_world_position(ram)
    with ctx.pose({slide: -0.55}):
        ctx.expect_overlap(
            ram,
            socket,
            axes="z",
            elem_a="ram_body",
            elem_b="socket_shell",
            min_overlap=0.75,
            name="lowered ram remains guided inside the socket",
        )
        lowered_position = ctx.part_world_position(ram)
    ctx.check(
        "ram slides down vertically into the ground socket",
        raised_position is not None
        and lowered_position is not None
        and lowered_position[2] < raised_position[2] - 0.50,
        details=f"raised={raised_position}, lowered={lowered_position}",
    )

    closed_cover_aabb = ctx.part_world_aabb(cover)
    with ctx.pose({cover_hinge: 1.20}):
        open_cover_aabb = ctx.part_world_aabb(cover)
    ctx.check(
        "debris cover rotates upward from the socket rim",
        closed_cover_aabb is not None
        and open_cover_aabb is not None
        and open_cover_aabb[1][2] > closed_cover_aabb[1][2] + 0.08,
        details=f"closed={closed_cover_aabb}, open={open_cover_aabb}",
    )

    with ctx.pose({key_turn: math.pi / 2.0}):
        ctx.expect_within(
            key,
            ram,
            axes="xy",
            inner_elem="key_stem",
            outer_elem="top_crown",
            margin=0.0,
            name="rotated key insert stays in the crown bore",
        )

    return ctx.report()


object_model = build_object_model()
