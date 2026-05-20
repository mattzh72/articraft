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


def _tube_mesh(outer_radius: float, inner_radius: float, height: float, name: str):
    """Open-ended round tube, authored in meters."""
    tube = (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(height)
    )
    return mesh_from_cadquery(tube, name, tolerance=0.001, angular_tolerance=0.08)


def _plate_with_round_hole_mesh(
    width: float,
    depth: float,
    thickness: float,
    hole_diameter: float,
    name: str,
    hole_offset: tuple[float, float] = (0.0, 0.0),
):
    """Flat rectangular plate with a round through-hole for the mast."""
    plate = (
        cq.Workplane("XY")
        .box(width, depth, thickness)
        .faces(">Z")
        .workplane()
        .center(hole_offset[0], hole_offset[1])
        .hole(hole_diameter)
    )
    return mesh_from_cadquery(plate, name, tolerance=0.001, angular_tolerance=0.08)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="trailer_generator_floodlight")

    safety_yellow = model.material("safety_yellow", rgba=(1.0, 0.74, 0.05, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.05, 0.055, 0.055, 1.0))
    tire_rubber = model.material("tire_rubber", rgba=(0.012, 0.012, 0.011, 1.0))
    galvanized = model.material("galvanized_steel", rgba=(0.58, 0.60, 0.58, 1.0))
    generator_gray = model.material("generator_gray", rgba=(0.72, 0.73, 0.70, 1.0))
    vent_black = model.material("vent_black", rgba=(0.0, 0.0, 0.0, 1.0))
    warm_glass = model.material("warm_lamp_glass", rgba=(1.0, 0.88, 0.48, 0.72))

    # Trailer frame and fixed generator/sleeve assembly.  The part frame is on
    # the ground plane at the trailer centerline.
    frame = model.part("trailer_frame")

    rail_z = 0.54
    rail_h = 0.10
    rail_w = 0.09
    frame.visual(
        Box((2.60, rail_w, rail_h)),
        origin=Origin(xyz=(0.0, 0.64, rail_z)),
        material=safety_yellow,
        name="side_rail_0",
    )
    frame.visual(
        Box((2.60, rail_w, rail_h)),
        origin=Origin(xyz=(0.0, -0.64, rail_z)),
        material=safety_yellow,
        name="side_rail_1",
    )
    frame.visual(
        Box((rail_w, 1.36, rail_h)),
        origin=Origin(xyz=(1.25, 0.0, rail_z)),
        material=safety_yellow,
        name="front_rail",
    )
    frame.visual(
        Box((rail_w, 1.36, rail_h)),
        origin=Origin(xyz=(-1.25, 0.0, rail_z)),
        material=safety_yellow,
        name="rear_rail",
    )
    for i, x in enumerate((-0.45, 0.35)):
        frame.visual(
            Box((0.08, 1.18, 0.08)),
            origin=Origin(xyz=(x, 0.0, rail_z)),
            material=safety_yellow,
            name=f"crossmember_{i}",
        )
    frame.visual(
        _plate_with_round_hole_mesh(1.78, 1.34, 0.045, 0.16, "deck_plate", hole_offset=(0.63, 0.0)),
        origin=Origin(xyz=(-0.05, 0.0, 0.600)),
        material=galvanized,
        name="deck_plate",
    )
    frame.visual(
        Box((1.10, 0.16, 0.08)),
        origin=Origin(xyz=(1.78, 0.0, 0.48)),
        material=safety_yellow,
        name="tow_tongue",
    )
    frame.visual(
        Box((0.16, 0.22, 0.08)),
        origin=Origin(xyz=(2.37, 0.0, 0.48)),
        material=dark_steel,
        name="hitch_coupler",
    )

    # Fixed wheelset with an axle and drop brackets so the tires read mounted.
    frame.visual(
        Cylinder(radius=0.035, length=1.82),
        origin=Origin(xyz=(-0.72, 0.0, 0.30), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="wheel_axle",
    )
    for i, y in enumerate((-0.82, 0.82)):
        frame.visual(
            Cylinder(radius=0.285, length=0.16),
            origin=Origin(xyz=(-0.72, y, 0.30), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=tire_rubber,
            name=f"tire_{i}",
        )
        frame.visual(
            Cylinder(radius=0.145, length=0.175),
            origin=Origin(xyz=(-0.72, y, 0.30), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=galvanized,
            name=f"wheel_hub_{i}",
        )
        frame.visual(
            Box((0.13, 0.055, 0.34)),
            origin=Origin(xyz=(-0.72, 0.50 if y > 0 else -0.50, 0.43)),
            material=dark_steel,
            name=f"axle_hanger_{i}",
        )

    # Generator set mounted low on the trailer deck.
    frame.visual(
        Box((0.96, 0.70, 0.62)),
        origin=Origin(xyz=(-0.54, 0.0, 0.930)),
        material=generator_gray,
        name="generator_housing",
    )
    frame.visual(
        Box((0.42, 0.018, 0.28)),
        origin=Origin(xyz=(-0.54, -0.356, 0.97)),
        material=vent_black,
        name="side_vent",
    )
    for i, z in enumerate((0.86, 0.94, 1.02, 1.10)):
        frame.visual(
            Box((0.45, 0.022, 0.018)),
            origin=Origin(xyz=(-0.54, -0.365, z)),
            material=generator_gray,
            name=f"vent_louver_{i}",
        )
    frame.visual(
        Cylinder(radius=0.055, length=0.24),
        origin=Origin(xyz=(-0.82, 0.18, 1.295), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="exhaust_muffler",
    )
    frame.visual(
        Cylinder(radius=0.075, length=0.025),
        origin=Origin(xyz=(-0.36, 0.0, 1.252)),
        material=dark_steel,
        name="fuel_cap",
    )

    # Round mast socket: a true hollow sleeve, not a solid proxy.
    mast_x = 0.58
    socket_bottom = 0.56
    socket_height = 1.10
    socket_top = socket_bottom + socket_height
    frame.visual(
        _tube_mesh(0.105, 0.075, socket_height, "round_mast_socket"),
        origin=Origin(xyz=(mast_x, 0.0, socket_bottom)),
        material=dark_steel,
        name="mast_socket",
    )
    for i, (dx, dy, sx, sy) in enumerate(
        (
            (0.080, 0.0, 0.050, 0.045),
            (-0.080, 0.0, 0.050, 0.045),
            (0.0, 0.080, 0.045, 0.050),
            (0.0, -0.080, 0.045, 0.050),
        )
    ):
        frame.visual(
            Box((sx, sy, 0.10)),
            origin=Origin(xyz=(mast_x + dx, dy, socket_top - 0.05)),
            material=galvanized,
            name=f"guide_pad_{i}",
        )
    frame.visual(
        _plate_with_round_hole_mesh(0.46, 0.46, 0.055, 0.16, "socket_base_plate"),
        origin=Origin(xyz=(mast_x, 0.0, 0.632)),
        material=dark_steel,
        name="socket_base_plate",
    )
    for i, (dx, dy, yaw) in enumerate(
        ((0.215, 0.0, 0.0), (-0.215, 0.0, 0.0), (0.0, 0.215, math.pi / 2.0), (0.0, -0.215, math.pi / 2.0))
    ):
        frame.visual(
            Box((0.22, 0.045, 0.43)),
            origin=Origin(xyz=(mast_x + dx, dy, 0.84), rpy=(0.0, 0.0, yaw)),
            material=dark_steel,
            name=f"socket_gusset_{i}",
        )

    # Prismatic inner round mast.  The hidden lower length remains inserted in
    # the socket at full extension.
    mast = model.part("mast")
    mast.visual(
        Cylinder(radius=0.055, length=3.20),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        material=galvanized,
        name="mast_tube",
    )
    mast.visual(
        Cylinder(radius=0.088, length=0.16),
        origin=Origin(xyz=(0.0, 0.0, 1.66)),
        material=dark_steel,
        name="top_collar",
    )
    mast.visual(
        Cylinder(radius=0.045, length=0.60),
        origin=Origin(xyz=(0.0, 0.0, 1.66), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="yoke_cross_tube",
    )
    mast.visual(
        Box((0.12, 0.04, 0.38)),
        origin=Origin(xyz=(0.0, -0.30, 1.83)),
        material=dark_steel,
        name="yoke_arm_0",
    )
    mast.visual(
        Box((0.12, 0.04, 0.38)),
        origin=Origin(xyz=(0.0, 0.30, 1.83)),
        material=dark_steel,
        name="yoke_arm_1",
    )

    mast_slide = model.articulation(
        "socket_to_mast",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=mast,
        origin=Origin(xyz=(mast_x, 0.0, socket_top)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=350.0, velocity=0.25, lower=0.0, upper=1.20),
    )

    # Tilting lamp cluster.  The child frame is the hinge axis through the yoke.
    lamp = model.part("lamp_cluster")
    lamp.visual(
        Cylinder(radius=0.035, length=0.56),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="hinge_pin",
    )
    lamp.visual(
        Box((0.20, 0.08, 0.08)),
        origin=Origin(xyz=(0.10, 0.0, 0.0)),
        material=dark_steel,
        name="tilt_bracket",
    )
    lamp.visual(
        Box((0.055, 0.98, 0.70)),
        origin=Origin(xyz=(0.225, 0.0, 0.0)),
        material=dark_steel,
        name="back_frame",
    )
    lamp_positions = [(-0.24, -0.17), (0.24, -0.17), (-0.24, 0.17), (0.24, 0.17)]
    for i, (y, z) in enumerate(lamp_positions):
        row = 0 if z < 0.0 else 1
        col = 0 if y < 0.0 else 1
        lamp.visual(
            Box((0.18, 0.38, 0.25)),
            origin=Origin(xyz=(0.340, y, z)),
            material=safety_yellow,
            name=f"lamp_housing_{row}_{col}",
        )
        lamp.visual(
            Box((0.014, 0.30, 0.19)),
            origin=Origin(xyz=(0.437, y, z)),
            material=warm_glass,
            name=f"lens_{row}_{col}",
        )
        lamp.visual(
            Box((0.020, 0.34, 0.020)),
            origin=Origin(xyz=(0.438, y, z)),
            material=dark_steel,
            name=f"lens_crossbar_h_{row}_{col}",
        )
        lamp.visual(
            Box((0.020, 0.020, 0.22)),
            origin=Origin(xyz=(0.440, y, z)),
            material=dark_steel,
            name=f"lens_crossbar_v_{row}_{col}",
        )

    model.articulation(
        "mast_to_lamp",
        ArticulationType.REVOLUTE,
        parent=mast,
        child=lamp,
        origin=Origin(xyz=(0.0, 0.0, 1.83)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=1.0, lower=-0.55, upper=0.75),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("trailer_frame")
    mast = object_model.get_part("mast")
    lamp = object_model.get_part("lamp_cluster")
    mast_slide = object_model.get_articulation("socket_to_mast")
    lamp_tilt = object_model.get_articulation("mast_to_lamp")

    # The mast is a retained telescoping member in a hollow socket.
    ctx.expect_within(
        mast,
        frame,
        axes="xy",
        inner_elem="mast_tube",
        outer_elem="mast_socket",
        margin=0.0,
        name="round mast centered in socket",
    )
    ctx.expect_overlap(
        mast,
        frame,
        axes="z",
        elem_a="mast_tube",
        elem_b="mast_socket",
        min_overlap=0.50,
        name="collapsed mast remains deeply inserted",
    )

    rest_mast_pos = ctx.part_world_position(mast)
    with ctx.pose({mast_slide: 1.20}):
        ctx.expect_within(
            mast,
            frame,
            axes="xy",
            inner_elem="mast_tube",
            outer_elem="mast_socket",
            margin=0.0,
            name="extended mast stays centered in socket",
        )
        ctx.expect_overlap(
            mast,
            frame,
            axes="z",
            elem_a="mast_tube",
            elem_b="mast_socket",
            min_overlap=0.25,
            name="extended mast retains socket insertion",
        )
        extended_mast_pos = ctx.part_world_position(mast)
    ctx.check(
        "mast extends upward prismatically",
        rest_mast_pos is not None
        and extended_mast_pos is not None
        and extended_mast_pos[2] > rest_mast_pos[2] + 1.15,
        details=f"rest={rest_mast_pos}, extended={extended_mast_pos}",
    )

    # Hinge pin ends sit in the yoke arms, and positive tilt aims the lamp noses
    # downward about the horizontal yoke axis.
    ctx.expect_contact(
        lamp,
        mast,
        elem_a="hinge_pin",
        elem_b="yoke_arm_0",
        contact_tol=0.003,
        name="lamp hinge pin seated in yoke arm",
    )
    rest_lens = ctx.part_element_world_aabb(lamp, elem="lens_1_1")
    with ctx.pose({lamp_tilt: 0.60}):
        tilted_lens = ctx.part_element_world_aabb(lamp, elem="lens_1_1")
    rest_lens_z = None if rest_lens is None else (rest_lens[0][2] + rest_lens[1][2]) * 0.5
    tilted_lens_z = None if tilted_lens is None else (tilted_lens[0][2] + tilted_lens[1][2]) * 0.5
    ctx.check(
        "positive lamp tilt aims cluster downward",
        rest_lens_z is not None
        and tilted_lens_z is not None
        and tilted_lens_z < rest_lens_z - 0.18,
        details=f"rest_lens_z={rest_lens_z}, tilted_lens_z={tilted_lens_z}",
    )

    return ctx.report()


object_model = build_object_model()
