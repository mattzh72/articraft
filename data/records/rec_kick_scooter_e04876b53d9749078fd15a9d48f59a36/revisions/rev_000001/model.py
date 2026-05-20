from __future__ import annotations

from math import pi

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
    mesh_from_geometry,
    rounded_rect_profile,
    tube_from_spline_points,
)


WHEEL_RADIUS = 0.075
WHEEL_WIDTH = 0.042


def _wheel_visuals(part, prefix: str, *, rim_material, tire_material) -> None:
    """Add a small urethane scooter wheel whose spin axis is the part +Y axis."""
    spin_to_part_y = Origin(rpy=(0.0, 0.0, pi / 2.0))
    part.visual(
        mesh_from_geometry(
            WheelGeometry(
                0.056,
                0.034,
                rim=WheelRim(inner_radius=0.037, flange_height=0.004, flange_thickness=0.002),
                hub=WheelHub(radius=0.017, width=0.030, cap_style="domed"),
                face=WheelFace(dish_depth=0.003, front_inset=0.0015, rear_inset=0.0015),
                spokes=WheelSpokes(style="straight", count=6, thickness=0.0025, window_radius=0.005),
                bore=WheelBore(style="round", diameter=0.016),
            ),
            f"{prefix}_rim",
        ),
        origin=spin_to_part_y,
        material=rim_material,
        name="rim",
    )
    part.visual(
        mesh_from_geometry(
            TireGeometry(
                WHEEL_RADIUS,
                WHEEL_WIDTH,
                inner_radius=0.054,
                tread=TireTread(style="circumferential", depth=0.0025, count=3),
                grooves=(TireGroove(center_offset=0.0, width=0.004, depth=0.0015),),
                sidewall=TireSidewall(style="rounded", bulge=0.045),
            ),
            f"{prefix}_tire",
        ),
        origin=spin_to_part_y,
        material=tire_material,
        name="tire",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="three_wheel_childrens_kick_scooter")

    deck_blue = model.material("deck_blue", rgba=(0.04, 0.35, 0.78, 1.0))
    grip_black = model.material("grip_black", rgba=(0.02, 0.02, 0.025, 1.0))
    safety_red = model.material("safety_red", rgba=(0.86, 0.08, 0.06, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.74, 0.76, 0.78, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.23, 0.24, 0.25, 1.0))
    rubber = model.material("soft_black_rubber", rgba=(0.025, 0.025, 0.025, 1.0))
    urethane = model.material("translucent_urethane", rgba=(0.20, 0.72, 0.95, 0.92))
    white_plastic = model.material("white_wheel_core", rgba=(0.92, 0.92, 0.88, 1.0))

    deck = model.part("deck")
    deck.visual(
        mesh_from_geometry(
            ExtrudeGeometry(
                rounded_rect_profile(0.68, 0.175, 0.075, corner_segments=14),
                0.045,
                center=True,
            ),
            "rounded_deck_shell",
        ),
        origin=Origin(xyz=(-0.01, 0.0, 0.1325)),
        material=deck_blue,
        name="deck_shell",
    )
    deck.visual(
        mesh_from_geometry(
            ExtrudeGeometry(
                rounded_rect_profile(0.54, 0.125, 0.045, corner_segments=10),
                0.006,
                center=True,
            ),
            "textured_grip_pad",
        ),
        origin=Origin(xyz=(-0.04, 0.0, 0.158)),
        material=grip_black,
        name="grip_pad",
    )
    deck.visual(
        Cylinder(radius=0.031, length=0.050),
        origin=Origin(xyz=(0.355, 0.0, 0.1325), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="nose_bearing",
    )
    deck.visual(
        Box((0.030, 0.160, 0.048)),
        origin=Origin(xyz=(0.318, 0.0, 0.132)),
        material=safety_red,
        name="nose_cap",
    )
    deck.visual(
        Box((0.175, 0.012, 0.055)),
        origin=Origin(xyz=(-0.390, 0.037, 0.108)),
        material=dark_steel,
        name="rear_fork_0",
    )
    deck.visual(
        Box((0.175, 0.012, 0.055)),
        origin=Origin(xyz=(-0.390, -0.037, 0.108)),
        material=dark_steel,
        name="rear_fork_1",
    )
    deck.visual(
        Cylinder(radius=0.0082, length=0.105),
        origin=Origin(xyz=(-0.430, 0.0, WHEEL_RADIUS), rpy=(pi / 2.0, 0.0, 0.0)),
        material=satin_steel,
        name="rear_axle",
    )

    front_assembly = model.part("front_assembly")
    front_assembly.visual(
        Cylinder(radius=0.029, length=0.080),
        origin=Origin(xyz=(0.040, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="lean_collar",
    )
    front_assembly.visual(
        Cylinder(radius=0.0082, length=0.315),
        origin=Origin(xyz=(0.120, 0.0, -0.0575), rpy=(pi / 2.0, 0.0, 0.0)),
        material=satin_steel,
        name="front_axle",
    )
    front_assembly.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                [(0.020, 0.0, -0.004), (0.075, 0.0, -0.034), (0.120, 0.0, -0.0575)],
                radius=0.014,
                samples_per_segment=12,
                radial_segments=16,
            ),
            "front_center_fork",
        ),
        material=dark_steel,
        name="center_fork",
    )
    front_assembly.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                [(0.055, 0.0, 0.022), (-0.040, 0.0, 0.300), (-0.150, 0.0, 0.705)],
                radius=0.016,
                samples_per_segment=18,
                radial_segments=18,
            ),
            "leaning_handlebar_stem",
        ),
        material=satin_steel,
        name="steering_stem",
    )
    front_assembly.visual(
        Box((0.080, 0.060, 0.044)),
        origin=Origin(xyz=(-0.153, 0.0, 0.710)),
        material=safety_red,
        name="bar_clamp",
    )
    front_assembly.visual(
        Cylinder(radius=0.014, length=0.460),
        origin=Origin(xyz=(-0.165, 0.0, 0.742), rpy=(pi / 2.0, 0.0, 0.0)),
        material=satin_steel,
        name="handlebar_bar",
    )
    front_assembly.visual(
        Cylinder(radius=0.020, length=0.105),
        origin=Origin(xyz=(-0.165, 0.235, 0.742), rpy=(pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="grip_0",
    )
    front_assembly.visual(
        Cylinder(radius=0.020, length=0.105),
        origin=Origin(xyz=(-0.165, -0.235, 0.742), rpy=(pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="grip_1",
    )

    front_wheel_0 = model.part("front_wheel_0")
    _wheel_visuals(front_wheel_0, "front_wheel_0", rim_material=white_plastic, tire_material=urethane)

    front_wheel_1 = model.part("front_wheel_1")
    _wheel_visuals(front_wheel_1, "front_wheel_1", rim_material=white_plastic, tire_material=urethane)

    rear_wheel = model.part("rear_wheel")
    _wheel_visuals(rear_wheel, "rear_wheel", rim_material=white_plastic, tire_material=urethane)

    model.articulation(
        "deck_to_front",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=front_assembly,
        origin=Origin(xyz=(0.380, 0.0, 0.1325)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.5, lower=-0.45, upper=0.45),
    )
    model.articulation(
        "front_wheel_0_spin",
        ArticulationType.CONTINUOUS,
        parent=front_assembly,
        child=front_wheel_0,
        origin=Origin(xyz=(0.120, 0.110, -0.0575)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=24.0),
    )
    model.articulation(
        "front_wheel_1_spin",
        ArticulationType.CONTINUOUS,
        parent=front_assembly,
        child=front_wheel_1,
        origin=Origin(xyz=(0.120, -0.110, -0.0575)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=24.0),
    )
    model.articulation(
        "rear_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=deck,
        child=rear_wheel,
        origin=Origin(xyz=(-0.430, 0.0, WHEEL_RADIUS)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=24.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    deck = object_model.get_part("deck")
    front = object_model.get_part("front_assembly")
    front_wheel_0 = object_model.get_part("front_wheel_0")
    front_wheel_1 = object_model.get_part("front_wheel_1")
    rear_wheel = object_model.get_part("rear_wheel")
    lean = object_model.get_articulation("deck_to_front")

    ctx.allow_overlap(
        front,
        front_wheel_0,
        elem_a="front_axle",
        elem_b="rim",
        reason="The front axle is intentionally captured through the simplified solid wheel hub.",
    )
    ctx.allow_overlap(
        front,
        front_wheel_1,
        elem_a="front_axle",
        elem_b="rim",
        reason="The front axle is intentionally captured through the simplified solid wheel hub.",
    )
    ctx.allow_overlap(
        deck,
        rear_wheel,
        elem_a="rear_axle",
        elem_b="rim",
        reason="The rear axle is intentionally captured through the simplified solid wheel hub.",
    )

    ctx.expect_gap(
        front,
        deck,
        axis="x",
        positive_elem="lean_collar",
        negative_elem="nose_bearing",
        max_gap=0.0015,
        max_penetration=0.0,
        name="front bogie collar seats against deck nose bearing",
    )
    ctx.expect_overlap(
        front,
        deck,
        axes="yz",
        elem_a="lean_collar",
        elem_b="nose_bearing",
        min_overlap=0.045,
        name="lean pivot shares the deck nose bearing axis",
    )
    ctx.expect_origin_distance(
        front_wheel_0,
        front_wheel_1,
        axes="y",
        min_dist=0.20,
        max_dist=0.24,
        name="front wheels are the laterally separated twin pair",
    )
    ctx.expect_origin_gap(
        front_wheel_0,
        rear_wheel,
        axis="x",
        min_gap=0.75,
        max_gap=1.05,
        name="single rear wheel trails the front pair",
    )
    for wheel, centered_part, axle_name in (
        (front_wheel_0, front, "front_axle"),
        (front_wheel_1, front, "front_axle"),
        (rear_wheel, deck, "rear_axle"),
    ):
        ctx.expect_within(
            centered_part,
            wheel,
            axes="xz",
            inner_elem=axle_name,
            outer_elem="rim",
            margin=0.0,
            name=f"{wheel.name} axle is centered through the hub",
        )
        ctx.expect_overlap(
            centered_part,
            wheel,
            axes="y",
            elem_a=axle_name,
            elem_b="rim",
            min_overlap=0.028,
            name=f"{wheel.name} hub remains retained on the axle",
        )

    rest_bar = ctx.part_element_world_aabb(front, elem="handlebar_bar")
    with ctx.pose({lean: 0.35}):
        leaned_bar = ctx.part_element_world_aabb(front, elem="handlebar_bar")
    if rest_bar is not None and leaned_bar is not None:
        rest_y = (float(rest_bar[0][1]) + float(rest_bar[1][1])) * 0.5
        leaned_y = (float(leaned_bar[0][1]) + float(leaned_bar[1][1])) * 0.5
        ctx.check(
            "front assembly lean rolls handlebar sideways",
            abs(leaned_y - rest_y) > 0.11,
            details=f"rest_y={rest_y:.3f}, leaned_y={leaned_y:.3f}",
        )
    else:
        ctx.fail("front assembly lean rolls handlebar sideways", "Could not measure handlebar bar AABB.")

    for wheel_name, joint_name in (
        ("front_wheel_0", "front_wheel_0_spin"),
        ("front_wheel_1", "front_wheel_1_spin"),
        ("rear_wheel", "rear_wheel_spin"),
    ):
        wheel = object_model.get_part(wheel_name)
        joint = object_model.get_articulation(joint_name)
        rest_pos = ctx.part_world_position(wheel)
        with ctx.pose({joint: 1.2}):
            spun_pos = ctx.part_world_position(wheel)
        ctx.check(
            f"{wheel_name} spins about its axle without translating",
            rest_pos is not None
            and spun_pos is not None
            and max(abs(float(rest_pos[i]) - float(spun_pos[i])) for i in range(3)) < 1e-6,
            details=f"rest={rest_pos}, spun={spun_pos}",
        )

    return ctx.report()


object_model = build_object_model()
