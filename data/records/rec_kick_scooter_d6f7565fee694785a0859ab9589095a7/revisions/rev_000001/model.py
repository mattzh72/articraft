from __future__ import annotations

from math import pi

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TireGeometry,
    TireGroove,
    TireSidewall,
    TireTread,
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_cadquery,
    mesh_from_geometry,
    tube_from_spline_points,
)


FRONT_X = 0.48
STEER_Z = 0.17
FRONT_AXLE_LOCAL = (0.125, 0.0, -0.105)
REAR_X = -0.39
REAR_Y = 0.145
REAR_Z = 0.055


def _deck_shell() -> cq.Workplane:
    """Widened kick-scooter deck, extruded upward from z=0."""
    outline = [
        (-0.47, -0.165),
        (-0.35, -0.178),
        (-0.22, -0.120),
        (0.33, -0.088),
        (0.48, -0.062),
        (0.505, 0.000),
        (0.48, 0.062),
        (0.33, 0.088),
        (-0.22, 0.120),
        (-0.35, 0.178),
        (-0.47, 0.165),
    ]
    return (
        cq.Workplane("XY")
        .polyline(outline)
        .close()
        .extrude(0.044)
        .edges("|Z")
        .fillet(0.016)
    )


def _grip_tape() -> cq.Workplane:
    outline = [
        (-0.415, -0.126),
        (-0.315, -0.136),
        (-0.190, -0.091),
        (0.300, -0.064),
        (0.405, -0.045),
        (0.425, 0.000),
        (0.405, 0.045),
        (0.300, 0.064),
        (-0.190, 0.091),
        (-0.315, 0.136),
        (-0.415, 0.126),
    ]
    return (
        cq.Workplane("XY")
        .polyline(outline)
        .close()
        .extrude(0.003)
        .edges("|Z")
        .fillet(0.010)
    )


def _bearing_ring() -> cq.Workplane:
    outer = cq.Workplane("XY").circle(0.043).extrude(0.080)
    inner = cq.Workplane("XY").circle(0.024).extrude(0.090).translate((0.0, 0.0, -0.005))
    return outer.cut(inner)


def _add_scooter_wheel(part, mesh_prefix: str, *, radius: float, width: float, wheel_mat, tire_mat) -> None:
    """Add a small scooter wheel whose axle/spin axis is the part local Y axis."""
    wheel_to_y = Origin(rpy=(0.0, 0.0, pi / 2.0))
    part.visual(
        mesh_from_geometry(
            WheelGeometry(
                radius * 0.72,
                width * 0.82,
                rim=WheelRim(
                    inner_radius=radius * 0.48,
                    flange_height=radius * 0.055,
                    flange_thickness=0.0025,
                    bead_seat_depth=0.002,
                ),
                hub=WheelHub(
                    radius=radius * 0.24,
                    width=width * 0.70,
                    cap_style="domed",
                    bolt_pattern=BoltPattern(
                        count=5,
                        circle_diameter=radius * 0.34,
                        hole_diameter=0.0028,
                    ),
                ),
                face=WheelFace(dish_depth=0.003, front_inset=0.0015, rear_inset=0.0015),
                spokes=WheelSpokes(style="straight", count=6, thickness=0.0022, window_radius=radius * 0.105),
                bore=WheelBore(style="round", diameter=0.018),
            ),
            f"{mesh_prefix}_rim",
        ),
        origin=wheel_to_y,
        material=wheel_mat,
        name="rim",
    )
    part.visual(
        mesh_from_geometry(
            TireGeometry(
                radius,
                width,
                inner_radius=radius * 0.70,
                tread=TireTread(style="circumferential", depth=0.0022, count=3),
                grooves=(TireGroove(center_offset=0.0, width=0.003, depth=0.0015),),
                sidewall=TireSidewall(style="rounded", bulge=0.045),
            ),
            f"{mesh_prefix}_tire",
        ),
        origin=wheel_to_y,
        material=tire_mat,
        name="tire",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="stability_kick_scooter")

    deck_blue = model.material("satin_blue", rgba=(0.07, 0.20, 0.42, 1.0))
    grip_black = model.material("grip_black", rgba=(0.015, 0.016, 0.017, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.12, 0.13, 0.14, 1.0))
    brushed_aluminum = model.material("brushed_aluminum", rgba=(0.68, 0.70, 0.72, 1.0))
    rubber = model.material("rubber_black", rgba=(0.035, 0.035, 0.038, 1.0))
    wheel_poly = model.material("wheel_poly", rgba=(0.92, 0.91, 0.86, 1.0))
    accent_red = model.material("tail_reflector_red", rgba=(0.9, 0.05, 0.03, 1.0))

    deck = model.part("deck")
    deck.visual(
        mesh_from_cadquery(_deck_shell(), "widened_deck_shell", tolerance=0.0008),
        origin=Origin(xyz=(0.0, 0.0, 0.106)),
        material=deck_blue,
        name="deck_shell",
    )
    deck.visual(
        mesh_from_cadquery(_grip_tape(), "deck_grip_tape", tolerance=0.0008),
        origin=Origin(xyz=(0.0, 0.0, 0.148)),
        material=grip_black,
        name="grip_tape",
    )
    deck.visual(
        Box((0.030, 0.320, 0.020)),
        origin=Origin(xyz=(-0.462, 0.0, 0.137)),
        material=accent_red,
        name="tail_reflector",
    )

    # Rigid rear hardware: a fixed axle tucked beneath a widened tail plus hangers
    # that physically bridge up into the deck.
    deck.visual(
        Cylinder(radius=0.0075, length=0.355),
        origin=Origin(xyz=(REAR_X, 0.0, REAR_Z), rpy=(pi / 2.0, 0.0, 0.0)),
        material=brushed_aluminum,
        name="rear_axle",
    )
    for y, name in [(-0.086, "rear_hanger_0"), (0.086, "rear_hanger_1")]:
        deck.visual(
            Box((0.052, 0.020, 0.058)),
            origin=Origin(xyz=(REAR_X, y, 0.086)),
            material=dark_metal,
            name=name,
        )
    deck.visual(
        Box((0.116, 0.214, 0.015)),
        origin=Origin(xyz=(REAR_X, 0.0, 0.109)),
        material=dark_metal,
        name="tail_axle_plate",
    )

    # Stationary nose bearing with a real center opening for the steering stem.
    deck.visual(
        mesh_from_cadquery(_bearing_ring(), "nose_bearing_ring", tolerance=0.0007),
        origin=Origin(xyz=(FRONT_X, 0.0, STEER_Z - 0.040)),
        material=brushed_aluminum,
        name="nose_bearing",
    )
    deck.visual(
        Box((0.092, 0.064, 0.030)),
        origin=Origin(xyz=(FRONT_X - 0.030, 0.0, 0.136)),
        material=dark_metal,
        name="nose_mount_block",
    )

    front_unit = model.part("front_unit")
    front_unit.visual(
        Cylinder(radius=0.018, length=0.170),
        origin=Origin(xyz=(0.0, 0.0, 0.065)),
        material=brushed_aluminum,
        name="steering_pin",
    )
    front_unit.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                [(0.0, 0.0, 0.065), (-0.050, 0.0, 0.310), (-0.115, 0.0, 0.735)],
                radius=0.017,
                samples_per_segment=18,
                radial_segments=20,
            ),
            "slanted_handlebar_column",
        ),
        material=brushed_aluminum,
        name="handlebar_column",
    )
    front_unit.visual(
        Cylinder(radius=0.014, length=0.500),
        origin=Origin(xyz=(-0.115, 0.0, 0.735), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="handlebar_crossbar",
    )
    for y, name in [(-0.270, "grip_0"), (0.270, "grip_1")]:
        front_unit.visual(
            Cylinder(radius=0.019, length=0.105),
            origin=Origin(xyz=(-0.115, y, 0.735), rpy=(pi / 2.0, 0.0, 0.0)),
            material=rubber,
            name=name,
        )
    front_unit.visual(
        Box((0.090, 0.064, 0.042)),
        origin=Origin(xyz=(0.025, 0.0, 0.064)),
        material=dark_metal,
        name="fork_crown",
    )
    front_unit.visual(
        Cylinder(radius=0.031, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.044)),
        material=dark_metal,
        name="upper_bearing_cap",
    )
    front_unit.visual(
        Box((0.050, 0.056, 0.052)),
        origin=Origin(xyz=(0.090, 0.0, 0.034)),
        material=dark_metal,
        name="fork_neck",
    )
    front_unit.visual(
        Box((0.060, 0.088, 0.022)),
        origin=Origin(xyz=(0.110, 0.0, 0.014)),
        material=dark_metal,
        name="fork_bridge",
    )
    for y, name in [(-0.037, "fork_leg_0"), (0.037, "fork_leg_1")]:
        front_unit.visual(
            Box((0.018, 0.012, 0.145)),
            origin=Origin(xyz=(FRONT_AXLE_LOCAL[0], y, -0.050)),
            material=dark_metal,
            name=name,
        )
        front_unit.visual(
            Box((0.026, 0.020, 0.030)),
            origin=Origin(xyz=(FRONT_AXLE_LOCAL[0], y, FRONT_AXLE_LOCAL[2])),
            material=dark_metal,
            name=f"dropout_{name[-1]}",
        )
    front_unit.visual(
        Cylinder(radius=0.0065, length=0.104),
        origin=Origin(xyz=FRONT_AXLE_LOCAL, rpy=(pi / 2.0, 0.0, 0.0)),
        material=brushed_aluminum,
        name="front_axle",
    )

    front_wheel = model.part("front_wheel")
    _add_scooter_wheel(front_wheel, "front_wheel", radius=0.066, width=0.038, wheel_mat=wheel_poly, tire_mat=rubber)
    front_wheel.visual(
        Cylinder(radius=0.010, length=0.032),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=brushed_aluminum,
        name="bearing_sleeve",
    )

    rear_wheel_0 = model.part("rear_wheel_0")
    _add_scooter_wheel(rear_wheel_0, "rear_wheel_0", radius=0.052, width=0.034, wheel_mat=wheel_poly, tire_mat=rubber)

    rear_wheel_1 = model.part("rear_wheel_1")
    _add_scooter_wheel(rear_wheel_1, "rear_wheel_1", radius=0.052, width=0.034, wheel_mat=wheel_poly, tire_mat=rubber)

    model.articulation(
        "front_steer",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=front_unit,
        origin=Origin(xyz=(FRONT_X, 0.0, STEER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=16.0, velocity=2.8, lower=-0.70, upper=0.70),
    )
    model.articulation(
        "front_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=front_unit,
        child=front_wheel,
        origin=Origin(xyz=FRONT_AXLE_LOCAL),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=35.0),
    )
    model.articulation(
        "rear_wheel_0_spin",
        ArticulationType.CONTINUOUS,
        parent=deck,
        child=rear_wheel_0,
        origin=Origin(xyz=(REAR_X, -REAR_Y, REAR_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=35.0),
    )
    model.articulation(
        "rear_wheel_1_spin",
        ArticulationType.CONTINUOUS,
        parent=deck,
        child=rear_wheel_1,
        origin=Origin(xyz=(REAR_X, REAR_Y, REAR_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=35.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    deck = object_model.get_part("deck")
    front_unit = object_model.get_part("front_unit")
    front_wheel = object_model.get_part("front_wheel")
    rear_wheel_0 = object_model.get_part("rear_wheel_0")
    rear_wheel_1 = object_model.get_part("rear_wheel_1")
    steer = object_model.get_articulation("front_steer")
    front_spin = object_model.get_articulation("front_wheel_spin")
    rear_spin_0 = object_model.get_articulation("rear_wheel_0_spin")
    rear_spin_1 = object_model.get_articulation("rear_wheel_1_spin")

    spin_joints = [front_spin, rear_spin_0, rear_spin_1]
    ctx.check(
        "three continuous wheel spin joints",
        all(j is not None and j.articulation_type == ArticulationType.CONTINUOUS for j in spin_joints),
        details=f"spin_joints={spin_joints}",
    )
    ctx.check(
        "front wheel spins under steerable fork",
        front_spin is not None and front_spin.parent == "front_unit" and front_spin.child == "front_wheel",
        details=f"front_spin={front_spin}",
    )
    ctx.check(
        "rear wheels spin on rigid deck axle",
        rear_spin_0 is not None
        and rear_spin_1 is not None
        and rear_spin_0.parent == "deck"
        and rear_spin_1.parent == "deck",
        details=f"rear_spin_0={rear_spin_0}, rear_spin_1={rear_spin_1}",
    )
    ctx.check(
        "steering joint has scooter yaw range",
        steer is not None
        and steer.articulation_type == ArticulationType.REVOLUTE
        and steer.motion_limits is not None
        and steer.motion_limits.lower <= -0.65
        and steer.motion_limits.upper >= 0.65,
        details=f"steer={steer}",
    )

    ctx.allow_overlap(
        front_unit,
        front_wheel,
        elem_a="front_axle",
        elem_b="bearing_sleeve",
        reason="The front wheel bearing sleeve is intentionally captured on the fork axle so the wheel remains retained while spinning.",
    )
    ctx.expect_overlap(
        front_unit,
        front_wheel,
        axes="y",
        min_overlap=0.025,
        elem_a="front_axle",
        elem_b="bearing_sleeve",
        name="front axle runs through bearing sleeve",
    )
    ctx.expect_overlap(
        front_unit,
        front_wheel,
        axes="xz",
        min_overlap=0.006,
        elem_a="front_axle",
        elem_b="bearing_sleeve",
        name="front bearing sleeve centered on axle",
    )

    ctx.expect_overlap(deck, rear_wheel_0, axes="x", min_overlap=0.015, elem_a="rear_axle", elem_b="rim", name="rear wheel 0 is on rear axle station")
    ctx.expect_overlap(deck, rear_wheel_1, axes="x", min_overlap=0.015, elem_a="rear_axle", elem_b="rim", name="rear wheel 1 is on rear axle station")

    if front_unit is not None and front_wheel is not None:
        tire_box = ctx.part_element_world_aabb(front_wheel, elem="tire")
        leg_0_box = ctx.part_element_world_aabb(front_unit, elem="fork_leg_0")
        leg_1_box = ctx.part_element_world_aabb(front_unit, elem="fork_leg_1")
        if tire_box is not None and leg_0_box is not None and leg_1_box is not None:
            tire_min, tire_max = tire_box
            leg_0_min, leg_0_max = leg_0_box
            leg_1_min, leg_1_max = leg_1_box
            lower_inner = min(leg_0_max[1], leg_1_max[1])
            upper_inner = max(leg_0_min[1], leg_1_min[1])
            clearance_low = tire_min[1] - lower_inner
            clearance_high = upper_inner - tire_max[1]
            ctx.check(
                "front tire clipped between fork legs",
                0.002 <= clearance_low <= 0.020 and 0.002 <= clearance_high <= 0.020,
                details=f"clearance_low={clearance_low}, clearance_high={clearance_high}",
            )
        else:
            ctx.fail("front fork clip aabbs available", "Expected front tire and fork leg AABBs.")

    if steer is not None and front_wheel is not None:
        rest_pos = ctx.part_world_position(front_wheel)
        with ctx.pose({steer: 0.55}):
            steered_pos = ctx.part_world_position(front_wheel)
        ctx.check(
            "front wheel follows steering yaw",
            rest_pos is not None
            and steered_pos is not None
            and abs(steered_pos[1] - rest_pos[1]) > 0.025
            and abs(steered_pos[2] - rest_pos[2]) < 0.005,
            details=f"rest={rest_pos}, steered={steered_pos}",
        )

    return ctx.report()


object_model = build_object_model()
