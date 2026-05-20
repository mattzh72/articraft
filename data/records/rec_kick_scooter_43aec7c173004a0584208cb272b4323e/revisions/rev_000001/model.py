from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TireGeometry,
    WheelGeometry,
    WheelBore,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_cadquery,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _rear_fender_geometry(
    *,
    center_x: float,
    center_z: float,
    inner_radius: float,
    thickness: float,
    width: float,
    start_deg: float = 20.0,
    end_deg: float = 160.0,
    segments: int = 28,
) -> MeshGeometry:
    """A closed, thin curved fender shell over the rear wheel."""

    geom = MeshGeometry()
    rows: list[tuple[int, int, int, int]] = []
    for i in range(segments + 1):
        theta = math.radians(start_deg + (end_deg - start_deg) * i / segments)
        c = math.cos(theta)
        s = math.sin(theta)
        inner_x = center_x + inner_radius * c
        inner_z = center_z + inner_radius * s
        outer_x = center_x + (inner_radius + thickness) * c
        outer_z = center_z + (inner_radius + thickness) * s
        y0 = -width / 2.0
        y1 = width / 2.0
        rows.append(
            (
                geom.add_vertex(inner_x, y0, inner_z),
                geom.add_vertex(inner_x, y1, inner_z),
                geom.add_vertex(outer_x, y0, outer_z),
                geom.add_vertex(outer_x, y1, outer_z),
            )
        )

    for i in range(segments):
        a0, a1, a2, a3 = rows[i]
        b0, b1, b2, b3 = rows[i + 1]
        # Inner and outer curved faces.
        geom.add_face(a0, b0, b1)
        geom.add_face(a0, b1, a1)
        geom.add_face(a2, a3, b3)
        geom.add_face(a2, b3, b2)
        # Side walls.
        geom.add_face(a0, a2, b2)
        geom.add_face(a0, b2, b0)
        geom.add_face(a1, b1, b3)
        geom.add_face(a1, b3, a3)

    # End caps.
    for row in (rows[0], rows[-1]):
        i0, i1, i2, i3 = row
        geom.add_face(i0, i1, i3)
        geom.add_face(i0, i3, i2)
    return geom


def _head_tube_mesh(name: str):
    # Hollow bearing sleeve: outer wall is part of the fixed frame, with a
    # clearance bore so the steering tube can pass through without collision.
    head_tube = cq.Workplane("XY").circle(0.026).circle(0.016).extrude(0.110)
    return mesh_from_cadquery(head_tube, name, tolerance=0.0008, angular_tolerance=0.08)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rigid_commuter_kick_scooter")

    matte_black = Material("matte_black", rgba=(0.01, 0.011, 0.012, 1.0))
    grip_black = Material("grip_tape_black", rgba=(0.0, 0.0, 0.0, 1.0))
    graphite = Material("graphite_deck", rgba=(0.08, 0.085, 0.09, 1.0))
    brushed = Material("brushed_aluminum", rgba=(0.72, 0.73, 0.70, 1.0))
    dark_metal = Material("dark_metal", rgba=(0.13, 0.13, 0.14, 1.0))
    tire_mat = Material("charcoal_urethane", rgba=(0.015, 0.016, 0.017, 1.0))
    rim_mat = Material("silver_wheel_core", rgba=(0.82, 0.82, 0.78, 1.0))

    # Main dimensions in meters, using a realistic adult commuter-scooter scale.
    deck_length = 0.82
    deck_width = 0.145
    deck_thickness = 0.034
    deck_z = 0.116
    wheel_radius = 0.095
    wheel_width = 0.034
    rear_axle_x = -0.535
    front_axle_x = 0.610
    axle_z = wheel_radius
    steer_pivot_z = 0.265
    front_wheel_rel_z = axle_z - steer_pivot_z

    deck_mesh = mesh_from_geometry(
        ExtrudeGeometry(
            rounded_rect_profile(deck_length, deck_width, 0.030, corner_segments=10),
            deck_thickness,
            center=True,
        ),
        "rounded_deck",
    )
    grip_mesh = mesh_from_geometry(
        ExtrudeGeometry(
            rounded_rect_profile(deck_length * 0.86, deck_width * 0.74, 0.018, corner_segments=8),
            0.004,
            center=True,
        ),
        "deck_grip_tape",
    )

    fender_mesh = mesh_from_geometry(
        _rear_fender_geometry(
            center_x=rear_axle_x,
            center_z=axle_z,
            inner_radius=wheel_radius + 0.010,
            thickness=0.011,
            width=0.078,
        ),
        "rear_curved_fender",
    )

    wheel_core = WheelGeometry(
        0.060,
        0.030,
        rim=WheelRim(
            inner_radius=0.043,
            flange_height=0.004,
            flange_thickness=0.003,
            bead_seat_depth=0.002,
        ),
        hub=WheelHub(radius=0.020, width=0.026, cap_style="domed"),
        spokes=WheelSpokes(style="split_y", count=6, thickness=0.003, window_radius=0.007),
        bore=WheelBore(style="round", diameter=0.012),
    )
    tire = TireGeometry(wheel_radius, wheel_width, inner_radius=0.061)

    frame = model.part("frame")
    frame.visual(deck_mesh, origin=Origin(xyz=(0.0, 0.0, deck_z)), material=graphite, name="deck")
    frame.visual(
        grip_mesh,
        origin=Origin(xyz=(0.0, 0.0, deck_z + deck_thickness / 2.0 + 0.001)),
        material=grip_black,
        name="grip_tape",
    )
    frame.visual(
        Box((0.200, 0.018, 0.018)),
        origin=Origin(xyz=(-0.455, 0.054, 0.106)),
        material=dark_metal,
        name="rear_side_rail_0",
    )
    frame.visual(
        Box((0.200, 0.018, 0.018)),
        origin=Origin(xyz=(-0.455, -0.054, 0.106)),
        material=dark_metal,
        name="rear_side_rail_1",
    )
    frame.visual(
        Cylinder(radius=0.006, length=0.128),
        origin=Origin(xyz=(rear_axle_x, 0.0, axle_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brushed,
        name="rear_axle",
    )
    frame.visual(fender_mesh, material=matte_black, name="rear_fender")
    frame.visual(
        Box((0.024, 0.072, 0.046)),
        origin=Origin(xyz=(-0.416, 0.0, 0.150)),
        material=matte_black,
        name="fender_mount",
    )
    frame.visual(
        Box((0.070, 0.056, 0.074)),
        origin=Origin(xyz=(0.486, 0.0, 0.185)),
        material=dark_metal,
        name="front_neck",
    )
    frame.visual(
        Box((0.100, 0.052, 0.026)),
        origin=Origin(xyz=(0.455, 0.0, 0.139)),
        material=dark_metal,
        name="front_bridge",
    )
    frame.visual(
        Box((0.116, 0.014, 0.018)),
        origin=Origin(xyz=(0.565, 0.025, 0.220)),
        material=dark_metal,
        name="upper_neck_0",
    )
    frame.visual(
        Box((0.116, 0.014, 0.018)),
        origin=Origin(xyz=(0.565, -0.025, 0.220)),
        material=dark_metal,
        name="upper_neck_1",
    )
    frame.visual(
        _head_tube_mesh("hollow_head_tube"),
        origin=Origin(xyz=(front_axle_x, 0.0, 0.218)),
        material=brushed,
        name="head_tube",
    )

    steering = model.part("steering")
    steering.visual(
        Cylinder(radius=0.012, length=0.740),
        origin=Origin(xyz=(0.0, 0.0, 0.315)),
        material=brushed,
        name="steering_column",
    )
    steering.visual(
        Cylinder(radius=0.015, length=0.430),
        origin=Origin(xyz=(0.0, 0.0, 0.690), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brushed,
        name="handle_bar",
    )
    steering.visual(
        Cylinder(radius=0.018, length=0.105),
        origin=Origin(xyz=(0.0, 0.212, 0.690), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=matte_black,
        name="grip_0",
    )
    steering.visual(
        Cylinder(radius=0.018, length=0.105),
        origin=Origin(xyz=(0.0, -0.212, 0.690), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=matte_black,
        name="grip_1",
    )
    steering.visual(
        Box((0.042, 0.126, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, -0.060)),
        material=dark_metal,
        name="fork_crown",
    )
    steering.visual(
        Box((0.019, 0.014, 0.112)),
        origin=Origin(xyz=(0.0, 0.052, -0.112)),
        material=dark_metal,
        name="fork_leg_0",
    )
    steering.visual(
        Box((0.019, 0.014, 0.112)),
        origin=Origin(xyz=(0.0, -0.052, -0.112)),
        material=dark_metal,
        name="fork_leg_1",
    )
    steering.visual(
        Cylinder(radius=0.006, length=0.128),
        origin=Origin(xyz=(0.0, 0.0, front_wheel_rel_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brushed,
        name="front_axle",
    )

    front_wheel = model.part("front_wheel")
    front_wheel.visual(
        mesh_from_geometry(wheel_core, "front_wheel_core"),
        material=rim_mat,
        name="front_rim",
    )
    front_wheel.visual(
        mesh_from_geometry(tire, "front_tire"),
        material=tire_mat,
        name="front_tire",
    )

    rear_wheel = model.part("rear_wheel")
    rear_wheel.visual(
        mesh_from_geometry(wheel_core, "rear_wheel_core"),
        material=rim_mat,
        name="rear_rim",
    )
    rear_wheel.visual(
        mesh_from_geometry(tire, "rear_tire"),
        material=tire_mat,
        name="rear_tire",
    )

    model.articulation(
        "frame_to_steering",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=steering,
        origin=Origin(xyz=(front_axle_x, 0.0, steer_pivot_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=3.5,
            lower=-math.pi / 4.0,
            upper=math.pi / 4.0,
        ),
    )
    model.articulation(
        "steering_to_front_wheel",
        ArticulationType.CONTINUOUS,
        parent=steering,
        child=front_wheel,
        origin=Origin(xyz=(0.0, 0.0, front_wheel_rel_z), rpy=(0.0, 0.0, math.pi / 2.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=40.0),
    )
    model.articulation(
        "frame_to_rear_wheel",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=rear_wheel,
        origin=Origin(xyz=(rear_axle_x, 0.0, axle_z), rpy=(0.0, 0.0, math.pi / 2.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=40.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    steering = object_model.get_part("steering")
    front_wheel = object_model.get_part("front_wheel")
    rear_wheel = object_model.get_part("rear_wheel")
    steer_joint = object_model.get_articulation("frame_to_steering")
    front_spin = object_model.get_articulation("steering_to_front_wheel")
    rear_spin = object_model.get_articulation("frame_to_rear_wheel")

    ctx.allow_overlap(
        steering,
        front_wheel,
        elem_a="front_axle",
        elem_b="front_rim",
        reason="The front wheel hub is intentionally captured on the fork axle so it can spin continuously.",
    )
    ctx.allow_overlap(
        frame,
        rear_wheel,
        elem_a="rear_axle",
        elem_b="rear_rim",
        reason="The rear wheel hub is intentionally captured on the tail axle so it can spin continuously.",
    )

    ctx.check(
        "front fork steers about +/-45 degrees",
        steer_joint.motion_limits is not None
        and abs((steer_joint.motion_limits.lower or 0.0) + math.pi / 4.0) < 1e-6
        and abs((steer_joint.motion_limits.upper or 0.0) - math.pi / 4.0) < 1e-6,
        details=f"limits={steer_joint.motion_limits}",
    )
    ctx.check(
        "both wheels use continuous spin joints",
        front_spin.articulation_type == ArticulationType.CONTINUOUS
        and rear_spin.articulation_type == ArticulationType.CONTINUOUS,
        details=f"front={front_spin.articulation_type}, rear={rear_spin.articulation_type}",
    )

    ctx.expect_within(
        steering,
        front_wheel,
        axes="xz",
        inner_elem="front_axle",
        outer_elem="front_rim",
        margin=0.002,
        name="front axle is centered in the wheel hub",
    )
    ctx.expect_overlap(
        steering,
        front_wheel,
        axes="y",
        elem_a="front_axle",
        elem_b="front_rim",
        min_overlap=0.025,
        name="front axle passes through the hub",
    )
    ctx.expect_within(
        frame,
        rear_wheel,
        axes="xz",
        inner_elem="rear_axle",
        outer_elem="rear_rim",
        margin=0.002,
        name="rear axle is centered in the wheel hub",
    )
    ctx.expect_overlap(
        frame,
        rear_wheel,
        axes="y",
        elem_a="rear_axle",
        elem_b="rear_rim",
        min_overlap=0.025,
        name="rear axle passes through the hub",
    )

    ctx.expect_origin_gap(
        steering,
        front_wheel,
        axis="z",
        min_gap=0.150,
        max_gap=0.190,
        name="front wheel hangs below the steering head",
    )

    rest_bar = ctx.part_element_world_aabb(steering, elem="handle_bar")
    with ctx.pose({steer_joint: math.pi / 4.0}):
        steered_bar = ctx.part_element_world_aabb(steering, elem="handle_bar")
    ctx.check(
        "T-bar rotates with the steering fork",
        rest_bar is not None
        and steered_bar is not None
        and (rest_bar[1][1] - rest_bar[0][1]) > 0.38
        and (rest_bar[1][0] - rest_bar[0][0]) < 0.06
        and (steered_bar[1][0] - steered_bar[0][0]) > 0.25
        and (steered_bar[1][1] - steered_bar[0][1]) > 0.25,
        details=f"rest_bar={rest_bar}, steered_bar={steered_bar}",
    )

    return ctx.report()


object_model = build_object_model()
