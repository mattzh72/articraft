from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TireCarcass,
    TireGeometry,
    TireGroove,
    TireShoulder,
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
    rounded_rect_profile,
)


def _tube_mesh(outer_radius: float, inner_radius: float, length: float, name: str):
    tube = (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(length)
        .translate((0.0, 0.0, -length / 2.0))
    )
    return mesh_from_cadquery(tube, name, tolerance=0.0007, angular_tolerance=0.08)


def _wheel_meshes(prefix: str):
    rim = WheelGeometry(
        0.059,
        0.032,
        rim=WheelRim(
            inner_radius=0.038,
            flange_height=0.004,
            flange_thickness=0.003,
            bead_seat_depth=0.002,
        ),
        hub=WheelHub(radius=0.018, width=0.026, cap_style="domed"),
        face=WheelFace(dish_depth=0.004, front_inset=0.002, rear_inset=0.002),
        spokes=WheelSpokes(style="straight", count=6, thickness=0.003, window_radius=0.006),
        bore=WheelBore(style="round", diameter=0.010),
    )
    tire = TireGeometry(
        0.085,
        0.040,
        inner_radius=0.055,
        carcass=TireCarcass(belt_width_ratio=0.70, sidewall_bulge=0.035),
        tread=TireTread(style="ribbed", depth=0.003, count=28, land_ratio=0.62),
        grooves=(TireGroove(center_offset=0.0, width=0.004, depth=0.0018),),
        sidewall=TireSidewall(style="rounded", bulge=0.035),
        shoulder=TireShoulder(width=0.004, radius=0.002),
    )
    return (
        mesh_from_geometry(rim, f"{prefix}_rim"),
        mesh_from_geometry(tire, f"{prefix}_tire"),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="commuting_kick_scooter")

    deck_mat = model.material("charcoal_deck", rgba=(0.08, 0.085, 0.09, 1.0))
    grip_mat = model.material("black_grip_tape", rgba=(0.005, 0.005, 0.004, 1.0))
    aluminum = model.material("brushed_aluminum", rgba=(0.72, 0.74, 0.75, 1.0))
    dark_metal = model.material("dark_hardware", rgba=(0.025, 0.027, 0.03, 1.0))
    rubber = model.material("matte_black_rubber", rgba=(0.01, 0.01, 0.011, 1.0))
    accent = model.material("blue_accent", rgba=(0.08, 0.28, 0.62, 1.0))

    deck = model.part("deck")
    deck.visual(
        mesh_from_geometry(
            ExtrudeGeometry.centered(
                rounded_rect_profile(0.720, 0.150, 0.060, corner_segments=10),
                0.045,
            ),
            "rounded_deck",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.1325)),
        material=deck_mat,
        name="deck_shell",
    )
    deck.visual(
        Box((0.560, 0.116, 0.005)),
        origin=Origin(xyz=(-0.030, 0.0, 0.1575)),
        material=grip_mat,
        name="grip_pad",
    )

    # Fixed head bearing and side gussets mounted to the front of the deck.
    deck.visual(
        _tube_mesh(0.026, 0.0135, 0.060, "head_bearing_ring"),
        origin=Origin(xyz=(0.380, 0.0, 0.260)),
        material=dark_metal,
        name="head_bearing",
    )
    for i, y in enumerate((-0.030, 0.030)):
        deck.visual(
            Box((0.070, 0.012, 0.115)),
            origin=Origin(xyz=(0.360, y, 0.2075)),
            material=aluminum,
            name=f"head_gusset_{i}",
        )

    # Tail dropouts carry the rear wheel axle outside the narrow deck.
    for i, y in enumerate((-0.038, 0.038)):
        deck.visual(
            Box((0.105, 0.014, 0.018)),
            origin=Origin(xyz=(-0.405, y, 0.106)),
            material=aluminum,
            name=f"rear_stay_{i}",
        )
        deck.visual(
            Cylinder(radius=0.017, length=0.012),
            origin=Origin(xyz=(-0.455, y * 0.67, 0.085), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_metal,
            name=f"rear_axle_washer_{i}",
        )

    # Small side hinge base under the deck for the parking kickstand.
    deck.visual(
        Box((0.082, 0.031, 0.029)),
        origin=Origin(xyz=(-0.120, -0.0875, 0.1115)),
        material=dark_metal,
        name="kickstand_mount",
    )

    front_fork = model.part("front_fork")
    front_fork.visual(
        Cylinder(radius=0.014, length=0.840),
        origin=Origin(xyz=(0.0, 0.0, 0.360)),
        material=aluminum,
        name="steering_tube",
    )
    front_fork.visual(
        Cylinder(radius=0.012, length=0.132),
        origin=Origin(
            xyz=(0.065, 0.0, -0.0625),
            rpy=(0.0, math.atan2(0.130, -0.015), 0.0),
        ),
        material=aluminum,
        name="fork_neck",
    )
    front_fork.visual(
        Box((0.110, 0.105, 0.024)),
        origin=Origin(xyz=(0.130, 0.0, -0.070)),
        material=aluminum,
        name="lower_crown",
    )
    for i, y in enumerate((-0.027, 0.027)):
        front_fork.visual(
            Cylinder(radius=0.009, length=0.095),
            origin=Origin(
                xyz=(0.130, y, -0.1285),
            ),
            material=aluminum,
            name=f"fork_leg_{i}",
        )
        front_fork.visual(
            Box((0.036, 0.014, 0.026)),
            origin=Origin(xyz=(0.130, y, -0.175)),
            material=dark_metal,
            name=f"front_dropout_{i}",
        )
    front_fork.visual(
        Cylinder(radius=0.011, length=0.460),
        origin=Origin(xyz=(0.0, 0.0, 0.735), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=aluminum,
        name="handlebar",
    )
    for i, y in enumerate((-0.225, 0.225)):
        front_fork.visual(
            Cylinder(radius=0.015, length=0.090),
            origin=Origin(xyz=(0.0, y, 0.735), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=rubber,
            name=f"grip_{i}",
        )

    front_wheel = model.part("front_wheel")
    front_rim, front_tire = _wheel_meshes("front_wheel")
    front_wheel.visual(front_rim, material=accent, name="rim")
    front_wheel.visual(front_tire, material=rubber, name="tire")

    rear_wheel = model.part("rear_wheel")
    rear_rim, rear_tire = _wheel_meshes("rear_wheel")
    rear_wheel.visual(rear_rim, material=accent, name="rim")
    rear_wheel.visual(rear_tire, material=rubber, name="tire")

    kickstand = model.part("kickstand")
    kickstand.visual(
        Cylinder(radius=0.008, length=0.065),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="hinge_barrel",
    )
    kickstand.visual(
        Cylinder(radius=0.0065, length=0.128),
        origin=Origin(
            xyz=(0.0, -0.050, -0.040),
            rpy=(math.atan2(0.100, -0.080), 0.0, 0.0),
        ),
        material=dark_metal,
        name="leg",
    )
    kickstand.visual(
        Box((0.062, 0.026, 0.008)),
        origin=Origin(xyz=(0.0, -0.100, -0.080)),
        material=rubber,
        name="foot_pad",
    )

    model.articulation(
        "deck_to_front_fork",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=front_fork,
        origin=Origin(xyz=(0.380, 0.0, 0.260)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=16.0, velocity=4.0, lower=-0.85, upper=0.85),
    )
    model.articulation(
        "fork_to_front_wheel",
        ArticulationType.CONTINUOUS,
        parent=front_fork,
        child=front_wheel,
        origin=Origin(xyz=(0.130, 0.0, -0.175), rpy=(0.0, 0.0, math.pi / 2.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=40.0),
    )
    model.articulation(
        "deck_to_rear_wheel",
        ArticulationType.CONTINUOUS,
        parent=deck,
        child=rear_wheel,
        origin=Origin(xyz=(-0.455, 0.0, 0.085), rpy=(0.0, 0.0, math.pi / 2.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=40.0),
    )
    model.articulation(
        "deck_to_kickstand",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=kickstand,
        origin=Origin(xyz=(-0.120, -0.108, 0.092)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.5, lower=-1.25, upper=0.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    deck = object_model.get_part("deck")
    front_fork = object_model.get_part("front_fork")
    front_wheel = object_model.get_part("front_wheel")
    rear_wheel = object_model.get_part("rear_wheel")
    kickstand = object_model.get_part("kickstand")

    steering = object_model.get_articulation("deck_to_front_fork")
    front_spin = object_model.get_articulation("fork_to_front_wheel")
    rear_spin = object_model.get_articulation("deck_to_rear_wheel")
    stand_hinge = object_model.get_articulation("deck_to_kickstand")

    ctx.allow_overlap(
        deck,
        kickstand,
        elem_a="kickstand_mount",
        elem_b="hinge_barrel",
        reason="The kickstand hinge barrel is intentionally captured a few millimeters inside the side-mounted deck hinge block.",
    )
    ctx.allow_overlap(
        deck,
        front_fork,
        elem_a="head_bearing",
        elem_b="steering_tube",
        reason="The steerer tube is intentionally represented as a close captured fit through the front head bearing.",
    )

    ctx.check(
        "front fork uses vertical steering yaw",
        steering.articulation_type == ArticulationType.REVOLUTE
        and tuple(round(v, 3) for v in steering.axis) == (0.0, 0.0, 1.0),
        details=f"type={steering.articulation_type}, axis={steering.axis}",
    )
    ctx.check(
        "both wheels have continuous axle spin joints",
        front_spin.articulation_type == ArticulationType.CONTINUOUS
        and rear_spin.articulation_type == ArticulationType.CONTINUOUS
        and tuple(round(v, 3) for v in front_spin.axis) == (1.0, 0.0, 0.0)
        and tuple(round(v, 3) for v in rear_spin.axis) == (1.0, 0.0, 0.0),
        details=f"front={front_spin.articulation_type}/{front_spin.axis}, rear={rear_spin.articulation_type}/{rear_spin.axis}",
    )
    ctx.check(
        "kickstand has a side hinge with a folding range",
        stand_hinge.articulation_type == ArticulationType.REVOLUTE
        and stand_hinge.motion_limits is not None
        and stand_hinge.motion_limits.lower < -1.0
        and stand_hinge.motion_limits.upper == 0.0,
        details=f"type={stand_hinge.articulation_type}, limits={stand_hinge.motion_limits}",
    )

    ctx.expect_gap(
        deck,
        rear_wheel,
        axis="x",
        min_gap=0.003,
        positive_elem="deck_shell",
        negative_elem="tire",
        name="rear wheel sits just behind tail without deck collision",
    )
    ctx.expect_gap(
        front_wheel,
        deck,
        axis="x",
        min_gap=0.015,
        positive_elem="tire",
        negative_elem="deck_shell",
        name="front wheel clears the front of the deck",
    )
    ctx.expect_overlap(
        front_wheel,
        front_fork,
        axes="z",
        min_overlap=0.02,
        name="front fork straddles wheel height at axle",
    )
    ctx.expect_within(
        front_fork,
        deck,
        axes="xy",
        inner_elem="steering_tube",
        outer_elem="head_bearing",
        margin=0.002,
        name="steering tube stays centered in head bearing",
    )
    ctx.expect_overlap(
        front_fork,
        deck,
        axes="z",
        elem_a="steering_tube",
        elem_b="head_bearing",
        min_overlap=0.045,
        name="head bearing captures steering tube vertically",
    )
    ctx.expect_gap(
        deck,
        kickstand,
        axis="y",
        max_penetration=0.004,
        positive_elem="kickstand_mount",
        negative_elem="hinge_barrel",
        name="kickstand hinge barrel is locally captured in mount",
    )
    ctx.expect_overlap(
        deck,
        kickstand,
        axes="x",
        min_overlap=0.040,
        elem_a="kickstand_mount",
        elem_b="hinge_barrel",
        name="kickstand hinge pin spans the side mount",
    )

    rest_front = ctx.part_world_position(front_wheel)
    with ctx.pose({steering: 0.65}):
        turned_front = ctx.part_world_position(front_wheel)
    ctx.check(
        "front wheel yaws around steering column",
        rest_front is not None
        and turned_front is not None
        and abs(turned_front[1] - rest_front[1]) > 0.045,
        details=f"rest={rest_front}, turned={turned_front}",
    )

    rest_foot = ctx.part_element_world_aabb(kickstand, elem="foot_pad")
    with ctx.pose({stand_hinge: -1.15}):
        folded_foot = ctx.part_element_world_aabb(kickstand, elem="foot_pad")
    ctx.check(
        "kickstand folds upward from deployed parking pose",
        rest_foot is not None
        and folded_foot is not None
        and folded_foot[0][2] > rest_foot[0][2] + 0.045,
        details=f"rest_foot={rest_foot}, folded_foot={folded_foot}",
    )

    return ctx.report()


object_model = build_object_model()
