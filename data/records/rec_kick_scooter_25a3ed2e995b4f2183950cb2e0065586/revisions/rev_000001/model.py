from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
    Material,
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
    model = ArticulatedObject(name="commuter_kick_scooter")

    aluminum = model.material("brushed_aluminum", rgba=(0.70, 0.72, 0.72, 1.0))
    dark_metal = model.material("dark_graphite", rgba=(0.05, 0.055, 0.06, 1.0))
    black = model.material("matte_black", rgba=(0.01, 0.01, 0.012, 1.0))
    rubber = model.material("dark_rubber", rgba=(0.006, 0.006, 0.006, 1.0))
    blue = model.material("muted_blue", rgba=(0.02, 0.12, 0.32, 1.0))

    deck = model.part("deck")
    deck.visual(
        Box((0.72, 0.17, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, 0.160)),
        material=blue,
        name="deck_shell",
    )
    deck.visual(
        Box((0.63, 0.135, 0.006)),
        origin=Origin(xyz=(-0.025, 0.0, 0.1855)),
        material=black,
        name="grip_tape",
    )
    deck.visual(
        Box((0.055, 0.17, 0.060)),
        origin=Origin(xyz=(0.320, 0.0, 0.178)),
        material=blue,
        name="nose_riser",
    )
    deck.visual(
        Cylinder(radius=0.035, length=0.260),
        origin=Origin(xyz=(0.390, 0.0, 0.310)),
        material=aluminum,
        name="steer_tube",
    )
    deck.visual(
        Cylinder(radius=0.044, length=0.030),
        origin=Origin(xyz=(0.390, 0.0, 0.200)),
        material=dark_metal,
        name="lower_headset_collar",
    )
    deck.visual(
        Cylinder(radius=0.043, length=0.026),
        origin=Origin(xyz=(0.390, 0.0, 0.425)),
        material=dark_metal,
        name="upper_headset_collar",
    )
    deck.visual(
        Box((0.070, 0.018, 0.065)),
        origin=Origin(xyz=(-0.305, 0.112, 0.110)),
        material=dark_metal,
        name="pivot_clevis_0",
    )
    deck.visual(
        Box((0.070, 0.018, 0.065)),
        origin=Origin(xyz=(-0.305, -0.112, 0.110)),
        material=dark_metal,
        name="pivot_clevis_1",
    )
    deck.visual(
        Box((0.070, 0.020, 0.012)),
        origin=Origin(xyz=(-0.305, 0.094, 0.148)),
        material=dark_metal,
        name="pivot_gusset_0",
    )
    deck.visual(
        Box((0.070, 0.020, 0.012)),
        origin=Origin(xyz=(-0.305, -0.094, 0.148)),
        material=dark_metal,
        name="pivot_gusset_1",
    )
    deck.visual(
        Cylinder(radius=0.012, length=0.255),
        origin=Origin(xyz=(-0.305, 0.0, 0.110), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=aluminum,
        name="pivot_pin",
    )
    deck.visual(
        Box((0.070, 0.028, 0.026)),
        origin=Origin(xyz=(-0.050, -0.095, 0.129)),
        material=dark_metal,
        name="stand_hinge_mount",
    )
    deck.visual(
        Cylinder(radius=0.011, length=0.076),
        origin=Origin(xyz=(-0.050, -0.120, 0.118), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=aluminum,
        name="stand_hinge_pin",
    )

    front_fork = model.part("front_fork")
    front_fork.visual(
        Cylinder(radius=0.018, length=0.880),
        origin=Origin(xyz=(0.0, 0.0, 0.345)),
        material=aluminum,
        name="steerer",
    )
    front_fork.visual(
        Box((0.026, 0.120, 0.025)),
        origin=Origin(xyz=(0.006, 0.0, -0.035)),
        material=dark_metal,
        name="fork_crown",
    )
    front_fork.visual(
        Box((0.130, 0.020, 0.130)),
        origin=Origin(xyz=(0.065, 0.055, -0.050)),
        material=aluminum,
        name="fork_blade_0",
    )
    front_fork.visual(
        Box((0.130, 0.020, 0.130)),
        origin=Origin(xyz=(0.065, -0.055, -0.050)),
        material=aluminum,
        name="fork_blade_1",
    )
    front_fork.visual(
        Cylinder(radius=0.011, length=0.170),
        origin=Origin(xyz=(0.120, 0.0, -0.095), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=aluminum,
        name="front_axle",
    )
    front_fork.visual(
        Cylinder(radius=0.017, length=0.420),
        origin=Origin(xyz=(0.0, 0.0, 0.770), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=aluminum,
        name="handlebar",
    )
    front_fork.visual(
        Cylinder(radius=0.020, length=0.090),
        origin=Origin(xyz=(0.0, 0.245, 0.770), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="grip_0",
    )
    front_fork.visual(
        Cylinder(radius=0.020, length=0.090),
        origin=Origin(xyz=(0.0, -0.245, 0.770), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="grip_1",
    )

    small_wheel = WheelGeometry(
        0.070,
        0.040,
        rim=WheelRim(inner_radius=0.045, flange_height=0.005, flange_thickness=0.003),
        hub=WheelHub(
            radius=0.020,
            width=0.050,
            cap_style="domed",
            bolt_pattern=BoltPattern(count=4, circle_diameter=0.026, hole_diameter=0.003),
        ),
        face=WheelFace(dish_depth=0.004, front_inset=0.002, rear_inset=0.002),
        spokes=WheelSpokes(style="straight", count=6, thickness=0.003, window_radius=0.006),
        bore=WheelBore(style="round", diameter=0.010),
    )
    tire = TireGeometry(
        0.098,
        0.050,
        inner_radius=0.071,
        carcass=TireCarcass(belt_width_ratio=0.72, sidewall_bulge=0.05),
        tread=TireTread(style="ribbed", depth=0.003, count=20, land_ratio=0.62),
        grooves=(TireGroove(center_offset=0.0, width=0.006, depth=0.002),),
        sidewall=TireSidewall(style="rounded", bulge=0.045),
        shoulder=TireShoulder(width=0.005, radius=0.003),
    )
    wheel_mesh = mesh_from_geometry(small_wheel, "commuter_wheel_rim")
    tire_mesh = mesh_from_geometry(tire, "commuter_wheel_tire")

    front_wheel = model.part("front_wheel")
    front_wheel.visual(wheel_mesh, material=aluminum, name="rim")
    front_wheel.visual(tire_mesh, material=rubber, name="tire")

    swingarm = model.part("rear_swingarm")
    swingarm.visual(
        Cylinder(radius=0.026, length=0.198),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="pivot_boss",
    )
    swingarm.visual(
        Box((0.165, 0.020, 0.025)),
        origin=Origin(xyz=(-0.105, 0.070, -0.006), rpy=(0.0, -0.08, 0.0)),
        material=aluminum,
        name="arm_0",
    )
    swingarm.visual(
        Box((0.165, 0.020, 0.025)),
        origin=Origin(xyz=(-0.105, -0.070, -0.006), rpy=(0.0, -0.08, 0.0)),
        material=aluminum,
        name="arm_1",
    )
    swingarm.visual(
        Cylinder(radius=0.013, length=0.170),
        origin=Origin(xyz=(-0.184, 0.0, -0.012), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=aluminum,
        name="rear_axle",
    )
    swingarm.visual(
        Box((0.055, 0.030, 0.018)),
        origin=Origin(xyz=(-0.175, 0.070, -0.012)),
        material=dark_metal,
        name="dropout_0",
    )
    swingarm.visual(
        Box((0.055, 0.030, 0.018)),
        origin=Origin(xyz=(-0.175, -0.070, -0.012)),
        material=dark_metal,
        name="dropout_1",
    )

    rear_wheel = model.part("rear_wheel")
    rear_wheel.visual(wheel_mesh, material=aluminum, name="rim")
    rear_wheel.visual(tire_mesh, material=rubber, name="tire")

    kickstand = model.part("kickstand")
    kickstand.visual(
        Cylinder(radius=0.012, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="stand_knuckle",
    )
    kickstand.visual(
        Box((0.018, 0.018, 0.205)),
        origin=Origin(xyz=(0.0, -0.041, -0.097), rpy=(2.74, 0.0, 0.0)),
        material=aluminum,
        name="stand_leg",
    )
    kickstand.visual(
        Box((0.075, 0.033, 0.012)),
        origin=Origin(xyz=(0.0, -0.082, -0.194), rpy=(0.0, 0.0, 0.0)),
        material=rubber,
        name="stand_foot",
    )

    model.articulation(
        "deck_to_front_fork",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=front_fork,
        origin=Origin(xyz=(0.390, 0.0, 0.200)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=3.0, lower=-0.85, upper=0.85),
    )
    model.articulation(
        "front_fork_to_front_wheel",
        ArticulationType.CONTINUOUS,
        parent=front_fork,
        child=front_wheel,
        origin=Origin(xyz=(0.120, 0.0, -0.095), rpy=(0.0, 0.0, math.pi / 2.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=50.0),
    )
    model.articulation(
        "deck_to_rear_swingarm",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=swingarm,
        origin=Origin(xyz=(-0.305, 0.0, 0.110)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=4.0, lower=-0.28, upper=0.34),
    )
    model.articulation(
        "rear_swingarm_to_rear_wheel",
        ArticulationType.CONTINUOUS,
        parent=swingarm,
        child=rear_wheel,
        origin=Origin(xyz=(-0.184, 0.0, -0.012), rpy=(0.0, 0.0, math.pi / 2.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=50.0),
    )
    model.articulation(
        "deck_to_kickstand",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=kickstand,
        origin=Origin(xyz=(-0.050, -0.120, 0.118)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=4.0, lower=0.0, upper=0.90),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    deck = object_model.get_part("deck")
    front_fork = object_model.get_part("front_fork")
    front_wheel = object_model.get_part("front_wheel")
    swingarm = object_model.get_part("rear_swingarm")
    rear_wheel = object_model.get_part("rear_wheel")
    kickstand = object_model.get_part("kickstand")

    steering = object_model.get_articulation("deck_to_front_fork")
    front_spin = object_model.get_articulation("front_fork_to_front_wheel")
    rear_suspension = object_model.get_articulation("deck_to_rear_swingarm")
    rear_spin = object_model.get_articulation("rear_swingarm_to_rear_wheel")
    stand_hinge = object_model.get_articulation("deck_to_kickstand")

    ctx.allow_overlap(
        deck,
        front_fork,
        elem_a="steer_tube",
        elem_b="steerer",
        reason="The rotating steerer shaft is intentionally captured inside the fixed nose steer tube.",
    )
    ctx.expect_within(
        front_fork,
        deck,
        axes="xy",
        inner_elem="steerer",
        outer_elem="steer_tube",
        margin=0.001,
        name="steerer is centered inside steer tube",
    )
    ctx.expect_overlap(
        front_fork,
        deck,
        axes="z",
        elem_a="steerer",
        elem_b="steer_tube",
        min_overlap=0.20,
        name="steerer remains captured through steer tube",
    )
    for collar_name in ("lower_headset_collar", "upper_headset_collar"):
        ctx.allow_overlap(
            deck,
            front_fork,
            elem_a=collar_name,
            elem_b="steerer",
            reason="The headset collar is a simplified bearing ring with the steerer shaft passing through its center.",
        )
        ctx.expect_within(
            front_fork,
            deck,
            axes="xy",
            inner_elem="steerer",
            outer_elem=collar_name,
            margin=0.001,
            name=f"steerer is centered in {collar_name}",
        )
        ctx.expect_overlap(
            front_fork,
            deck,
            axes="z",
            elem_a="steerer",
            elem_b=collar_name,
            min_overlap=0.020,
            name=f"steerer passes through {collar_name}",
        )

    ctx.allow_overlap(
        deck,
        swingarm,
        elem_a="pivot_pin",
        elem_b="pivot_boss",
        reason="The rear swingarm pivot boss rotates around a captured transverse pivot pin under the deck.",
    )
    ctx.expect_within(
        deck,
        swingarm,
        axes="xz",
        inner_elem="pivot_pin",
        outer_elem="pivot_boss",
        margin=0.002,
        name="swingarm pivot pin sits inside pivot boss",
    )
    ctx.expect_overlap(
        deck,
        swingarm,
        axes="y",
        elem_a="pivot_pin",
        elem_b="pivot_boss",
        min_overlap=0.18,
        name="swingarm boss stays clipped onto transverse pivot",
    )

    ctx.allow_overlap(
        deck,
        kickstand,
        elem_a="stand_hinge_pin",
        elem_b="stand_knuckle",
        reason="The kickstand hinge knuckle is retained on a small pin below the side of the deck.",
    )
    ctx.expect_within(
        deck,
        kickstand,
        axes="yz",
        inner_elem="stand_hinge_pin",
        outer_elem="stand_knuckle",
        margin=0.002,
        name="kickstand hinge pin is concentric with knuckle",
    )
    ctx.expect_overlap(
        deck,
        kickstand,
        axes="x",
        elem_a="stand_hinge_pin",
        elem_b="stand_knuckle",
        min_overlap=0.050,
        name="kickstand knuckle is retained on hinge pin",
    )
    ctx.allow_overlap(
        deck,
        kickstand,
        elem_a="stand_hinge_pin",
        elem_b="stand_leg",
        reason="The simplified upper end of the kickstand leg wraps around the same hinge pin as the knuckle.",
    )
    ctx.expect_overlap(
        deck,
        kickstand,
        axes="x",
        elem_a="stand_hinge_pin",
        elem_b="stand_leg",
        min_overlap=0.015,
        name="kickstand leg wraps around hinge pin",
    )

    ctx.allow_overlap(
        front_fork,
        front_wheel,
        elem_a="front_axle",
        elem_b="rim",
        reason="The front wheel rim/hub spins around the fixed front axle.",
    )
    ctx.expect_within(
        front_fork,
        front_wheel,
        axes="xz",
        inner_elem="front_axle",
        outer_elem="rim",
        margin=0.002,
        name="front axle passes through wheel hub",
    )
    ctx.expect_overlap(
        front_fork,
        front_wheel,
        axes="y",
        elem_a="front_axle",
        elem_b="rim",
        min_overlap=0.040,
        name="front axle spans wheel hub",
    )

    ctx.allow_overlap(
        swingarm,
        rear_wheel,
        elem_a="rear_axle",
        elem_b="rim",
        reason="The rear wheel hub spins around the axle carried by the short swingarm.",
    )
    ctx.expect_within(
        swingarm,
        rear_wheel,
        axes="xz",
        inner_elem="rear_axle",
        outer_elem="rim",
        margin=0.002,
        name="rear axle passes through wheel hub",
    )
    ctx.expect_overlap(
        swingarm,
        rear_wheel,
        axes="y",
        elem_a="rear_axle",
        elem_b="rim",
        min_overlap=0.040,
        name="rear axle spans wheel hub",
    )

    ctx.check(
        "front fork is yaw articulated",
        steering.motion_limits is not None
        and steering.motion_limits.lower < 0.0
        and steering.motion_limits.upper > 0.0,
        details="Steering joint should yaw both directions.",
    )
    ctx.check(
        "both wheels use spin joints",
        front_spin.articulation_type == ArticulationType.CONTINUOUS
        and rear_spin.articulation_type == ArticulationType.CONTINUOUS,
        details="Front and rear wheels should each have a continuous axle spin joint.",
    )

    rest_front_pos = ctx.part_world_position(front_wheel)
    with ctx.pose({steering: 0.55}):
        steered_front_pos = ctx.part_world_position(front_wheel)
    ctx.check(
        "steering yaw moves front wheel laterally",
        rest_front_pos is not None
        and steered_front_pos is not None
        and abs(steered_front_pos[1] - rest_front_pos[1]) > 0.050,
        details=f"rest={rest_front_pos}, steered={steered_front_pos}",
    )

    rest_rear_pos = ctx.part_world_position(rear_wheel)
    with ctx.pose({rear_suspension: rear_suspension.motion_limits.upper}):
        raised_rear_pos = ctx.part_world_position(rear_wheel)
        ctx.expect_overlap(
            deck,
            swingarm,
            axes="y",
            elem_a="pivot_pin",
            elem_b="pivot_boss",
            min_overlap=0.18,
            name="swingarm remains clipped at suspension upper travel",
        )
    ctx.check(
        "rear swingarm lifts wheel through suspension travel",
        rest_rear_pos is not None
        and raised_rear_pos is not None
        and raised_rear_pos[2] > rest_rear_pos[2] + 0.040,
        details=f"rest={rest_rear_pos}, raised={raised_rear_pos}",
    )

    rest_stand_aabb = ctx.part_world_aabb(kickstand)
    with ctx.pose({stand_hinge: stand_hinge.motion_limits.upper}):
        folded_stand_aabb = ctx.part_world_aabb(kickstand)
    ctx.check(
        "kickstand folds upward below deck",
        rest_stand_aabb is not None
        and folded_stand_aabb is not None
        and folded_stand_aabb[0][2] > rest_stand_aabb[0][2] + 0.050,
        details=f"rest={rest_stand_aabb}, folded={folded_stand_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
