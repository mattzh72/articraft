from __future__ import annotations

from math import pi

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
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hinged_knee_scooter")

    aluminum = model.material("brushed_aluminum", rgba=(0.72, 0.74, 0.74, 1.0))
    dark_metal = model.material("dark_hardware", rgba=(0.08, 0.09, 0.10, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.01, 0.01, 0.012, 1.0))
    blue_pad = model.material("blue_vinyl_pad", rgba=(0.04, 0.15, 0.36, 1.0))
    tray_plastic = model.material("matte_storage_tray", rgba=(0.18, 0.20, 0.22, 1.0))
    rim_silver = model.material("silver_rims", rgba=(0.82, 0.84, 0.82, 1.0))

    frame = model.part("frame")

    # Low side rails and cross-tubes: a scooter-like aluminum chassis.
    rail_r = 0.018
    for i, y in enumerate((-0.16, 0.16)):
        frame.visual(
            Cylinder(radius=rail_r, length=0.86),
            origin=Origin(xyz=(-0.06, y, 0.25), rpy=(0.0, pi / 2.0, 0.0)),
            material=aluminum,
            name=f"side_rail_{i}",
        )

    for name, x, radius in (
        ("rear_crossbar", -0.47, 0.015),
        ("center_crossbar", -0.08, 0.016),
        ("front_crossbar", 0.32, 0.015),
    ):
        frame.visual(
            Cylinder(radius=radius, length=0.38),
            origin=Origin(xyz=(x, 0.0, 0.25), rpy=(-pi / 2.0, 0.0, 0.0)),
            material=aluminum,
            name=name,
        )

    frame.visual(
        Cylinder(radius=0.008, length=0.54),
        origin=Origin(xyz=(-0.48, 0.0, 0.115), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="rear_axle",
    )
    for i, y in enumerate((-0.16, 0.16)):
        frame.visual(
            Cylinder(radius=0.012, length=0.145),
            origin=Origin(xyz=(-0.48, y, 0.182), rpy=(0.0, 0.0, 0.0)),
            material=aluminum,
            name=f"rear_dropout_{i}",
        )

    # Raised central storage tray that the hinged knee-pad closes over.
    frame.visual(
        Box((0.52, 0.27, 0.018)),
        origin=Origin(xyz=(-0.08, 0.0, 0.379)),
        material=tray_plastic,
        name="storage_floor",
    )
    for side_name, y in (("storage_side_0", -0.126), ("storage_side_1", 0.126)):
        frame.visual(
            Box((0.52, 0.040, 0.092)),
            origin=Origin(xyz=(-0.08, y, 0.425)),
            material=tray_plastic,
            name=side_name,
        )
    for name, x in (("storage_end_0", -0.34), ("storage_end_1", 0.18)):
        frame.visual(
            Box((0.020, 0.27, 0.092)),
            origin=Origin(xyz=(x, 0.0, 0.425)),
            material=tray_plastic,
            name=name,
        )
    for ix, x in enumerate((-0.27, 0.12)):
        for iy, y in enumerate((-0.145, 0.145)):
            frame.visual(
                Cylinder(radius=0.011, length=0.145),
                origin=Origin(xyz=(x, y, 0.313)),
                material=aluminum,
                name=f"tray_post_{ix}_{iy}",
            )

    # Exposed hinge brackets just outside the pad hinge tube.
    for lug_name, y in (("deck_hinge_lug_0", -0.162), ("deck_hinge_lug_1", 0.162)):
        frame.visual(
            Box((0.048, 0.036, 0.046)),
            origin=Origin(xyz=(-0.34, y, 0.49)),
            material=dark_metal,
            name=lug_name,
        )

    # Front head tube and braces for the bicycle-style steering fork.
    frame.visual(
        Cylinder(radius=0.034, length=0.20),
        origin=Origin(xyz=(0.40, 0.0, 0.37)),
        material=dark_metal,
        name="head_tube",
    )
    frame.visual(
        Cylinder(radius=0.014, length=0.18),
        origin=Origin(xyz=(0.33, 0.043, 0.305), rpy=(0.0, pi / 2.0, 0.0)),
        material=aluminum,
        name="head_brace",
    )
    frame.visual(
        Cylinder(radius=0.012, length=0.12),
        origin=Origin(xyz=(0.36, -0.043, 0.315), rpy=(0.0, pi / 2.0, 0.0)),
        material=aluminum,
        name="front_lower_brace",
    )
    for i, y in enumerate((-0.032, 0.032)):
        frame.visual(
            Box((0.14, 0.020, 0.035)),
            origin=Origin(xyz=(0.345, y, 0.274)),
            material=aluminum,
            name=f"neck_brace_{i}",
        )

    deck = model.part("knee_deck")
    deck.visual(
        Cylinder(radius=0.015, length=0.36),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="hinge_barrel",
    )
    deck.visual(
        Box((0.52, 0.27, 0.012)),
        origin=Origin(xyz=(0.26, 0.0, 0.016)),
        material=dark_metal,
        name="deck_board",
    )
    deck.visual(
        Box((0.49, 0.245, 0.052)),
        origin=Origin(xyz=(0.265, 0.0, 0.048)),
        material=blue_pad,
        name="knee_pad",
    )
    deck.visual(
        Box((0.055, 0.040, 0.020)),
        origin=Origin(xyz=(0.515, 0.0, 0.015)),
        material=dark_metal,
        name="front_latch_tab",
    )

    steer_fork = model.part("steering_fork")
    steer_fork.visual(
        Cylinder(radius=0.018, length=0.794),
        origin=Origin(xyz=(0.0, 0.0, 0.283)),
        material=aluminum,
        name="steering_stem",
    )
    steer_fork.visual(
        Cylinder(radius=0.015, length=0.46),
        origin=Origin(xyz=(0.0, 0.0, 0.68), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=aluminum,
        name="handlebar",
    )
    for i, y in enumerate((-0.26, 0.26)):
        steer_fork.visual(
            Cylinder(radius=0.022, length=0.10),
            origin=Origin(xyz=(0.0, y, 0.68), rpy=(-pi / 2.0, 0.0, 0.0)),
            material=black_rubber,
            name=f"grip_{i}",
        )
    steer_fork.visual(
        Cylinder(radius=0.016, length=0.17),
        origin=Origin(xyz=(0.0, 0.0, -0.130), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="fork_crown",
    )
    for i, y in enumerate((-0.060, 0.060)):
        steer_fork.visual(
            Cylinder(radius=0.012, length=0.235),
            origin=Origin(xyz=(0.0, y, -0.192)),
            material=aluminum,
            name=f"fork_blade_{i}",
        )
    steer_fork.visual(
        Cylinder(radius=0.008, length=0.17),
        origin=Origin(xyz=(0.0, 0.0, -0.265), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="front_axle",
    )

    wheel_mesh = mesh_from_geometry(
        WheelGeometry(
            0.085,
            0.050,
            rim=WheelRim(inner_radius=0.058, flange_height=0.006, flange_thickness=0.003),
            hub=WheelHub(
                radius=0.026,
                width=0.034,
                cap_style="domed",
                bolt_pattern=BoltPattern(count=5, circle_diameter=0.035, hole_diameter=0.004),
            ),
            face=WheelFace(dish_depth=0.005, front_inset=0.0025, rear_inset=0.002),
            spokes=WheelSpokes(style="split_y", count=5, thickness=0.0035, window_radius=0.010),
            bore=WheelBore(style="round", diameter=0.028),
        ),
        "scooter_rim",
    )
    tire_mesh = mesh_from_geometry(
        TireGeometry(
            0.115,
            0.052,
            inner_radius=0.085,
            carcass=TireCarcass(belt_width_ratio=0.70, sidewall_bulge=0.04),
            tread=TireTread(style="block", depth=0.006, count=24, land_ratio=0.58),
            grooves=(TireGroove(center_offset=0.0, width=0.006, depth=0.002),),
            sidewall=TireSidewall(style="rounded", bulge=0.035),
            shoulder=TireShoulder(width=0.006, radius=0.003),
        ),
        "scooter_tire",
    )

    def add_wheel(name: str) -> object:
        wheel = model.part(name)
        wheel.visual(
            tire_mesh,
            origin=Origin(rpy=(0.0, 0.0, pi / 2.0)),
            material=black_rubber,
            name="tire",
        )
        wheel.visual(
            wheel_mesh,
            origin=Origin(rpy=(0.0, 0.0, pi / 2.0)),
            material=rim_silver,
            name="rim",
        )
        wheel.visual(
            Cylinder(radius=0.014, length=0.062),
            origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
            material=dark_metal,
            name="hub_sleeve",
        )
        return wheel

    rear_wheel_0 = add_wheel("rear_wheel_0")
    rear_wheel_1 = add_wheel("rear_wheel_1")
    front_wheel = add_wheel("front_wheel")

    model.articulation(
        "frame_to_knee_deck",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=deck,
        origin=Origin(xyz=(-0.34, 0.0, 0.49)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.2, lower=0.0, upper=1.30),
    )
    model.articulation(
        "frame_to_steering_fork",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=steer_fork,
        origin=Origin(xyz=(0.40, 0.0, 0.37)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.5, lower=-0.80, upper=0.80),
    )
    model.articulation(
        "fork_to_front_wheel",
        ArticulationType.CONTINUOUS,
        parent=steer_fork,
        child=front_wheel,
        origin=Origin(xyz=(0.0, 0.0, -0.265)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=20.0),
    )
    model.articulation(
        "frame_to_rear_wheel_0",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=rear_wheel_0,
        origin=Origin(xyz=(-0.48, -0.245, 0.115)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=20.0),
    )
    model.articulation(
        "frame_to_rear_wheel_1",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=rear_wheel_1,
        origin=Origin(xyz=(-0.48, 0.245, 0.115)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=20.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    deck = object_model.get_part("knee_deck")
    fork = object_model.get_part("steering_fork")
    front_wheel = object_model.get_part("front_wheel")
    rear_wheel_0 = object_model.get_part("rear_wheel_0")
    rear_wheel_1 = object_model.get_part("rear_wheel_1")
    deck_hinge = object_model.get_articulation("frame_to_knee_deck")
    steering = object_model.get_articulation("frame_to_steering_fork")

    ctx.allow_overlap(
        frame,
        fork,
        elem_a="head_tube",
        elem_b="steering_stem",
        reason="The steering stem is intentionally captured inside the scooter head-tube bearing.",
    )
    ctx.expect_within(
        fork,
        frame,
        axes="xy",
        inner_elem="steering_stem",
        outer_elem="head_tube",
        margin=0.002,
        name="steering stem is concentric in head tube",
    )
    ctx.expect_overlap(
        fork,
        frame,
        axes="z",
        elem_a="steering_stem",
        elem_b="head_tube",
        min_overlap=0.16,
        name="steering stem remains captured through head tube",
    )

    for lug_name in ("deck_hinge_lug_0", "deck_hinge_lug_1"):
        ctx.allow_overlap(
            frame,
            deck,
            elem_a=lug_name,
            elem_b="hinge_barrel",
            reason="The knee-deck hinge barrel is intentionally captured inside the frame hinge lug.",
        )
        ctx.expect_overlap(
            deck,
            frame,
            axes="y",
            elem_a="hinge_barrel",
            elem_b=lug_name,
            min_overlap=0.020,
            name=f"deck hinge barrel passes through {lug_name}",
        )

    ctx.expect_gap(
        deck,
        frame,
        axis="z",
        positive_elem="deck_board",
        negative_elem="storage_side_0",
        min_gap=0.005,
        max_gap=0.040,
        name="closed deck clears storage tray rim",
    )
    ctx.expect_overlap(
        deck,
        frame,
        axes="xy",
        elem_a="knee_pad",
        elem_b="storage_floor",
        min_overlap=0.20,
        name="knee pad covers the central storage tray",
    )
    ctx.allow_overlap(
        fork,
        front_wheel,
        elem_a="front_axle",
        elem_b="hub_sleeve",
        reason="The wheel hub sleeve is intentionally modeled around the fork axle.",
    )
    ctx.expect_overlap(
        front_wheel,
        fork,
        axes="y",
        elem_a="hub_sleeve",
        elem_b="front_axle",
        min_overlap=0.050,
        name="front hub sleeve surrounds the fork axle length",
    )
    ctx.expect_overlap(
        front_wheel,
        fork,
        axes="xz",
        elem_a="hub_sleeve",
        elem_b="front_axle",
        min_overlap=0.014,
        name="front wheel is centered on fork axle",
    )
    for wheel in (rear_wheel_0, rear_wheel_1):
        ctx.allow_overlap(
            frame,
            wheel,
            elem_a="rear_axle",
            elem_b="hub_sleeve",
            reason="Each rear hub sleeve is intentionally modeled around the fixed rear axle.",
        )
        ctx.expect_overlap(
            wheel,
            frame,
            axes="y",
            elem_a="hub_sleeve",
            elem_b="rear_axle",
            min_overlap=0.050,
            name=f"{wheel.name} hub sleeve surrounds rear axle",
        )

    closed_aabb = ctx.part_world_aabb(deck)
    with ctx.pose({deck_hinge: 1.10}):
        opened_aabb = ctx.part_world_aabb(deck)
    ctx.check(
        "knee deck hinges upward for storage access",
        closed_aabb is not None
        and opened_aabb is not None
        and opened_aabb[1][2] > closed_aabb[1][2] + 0.20,
        details=f"closed={closed_aabb}, opened={opened_aabb}",
    )

    ctx.check(
        "steering fork has vertical limited rotation",
        tuple(round(v, 3) for v in steering.axis) == (0.0, 0.0, 1.0)
        and steering.motion_limits is not None
        and steering.motion_limits.lower < 0.0
        and steering.motion_limits.upper > 0.0,
    )
    continuous_wheels = [
        object_model.get_articulation("fork_to_front_wheel"),
        object_model.get_articulation("frame_to_rear_wheel_0"),
        object_model.get_articulation("frame_to_rear_wheel_1"),
    ]
    ctx.check(
        "all three wheels use continuous axle joints",
        all(j.articulation_type == ArticulationType.CONTINUOUS for j in continuous_wheels),
    )

    return ctx.report()


object_model = build_object_model()
