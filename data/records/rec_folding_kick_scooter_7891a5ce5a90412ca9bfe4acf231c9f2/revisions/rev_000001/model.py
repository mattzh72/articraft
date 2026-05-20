from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Material,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    TireCarcass,
    TireGeometry,
    TireShoulder,
    TireSidewall,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _cylinder_between(part, start, end, radius, material, name):
    """Attach a cylinder whose local Z axis runs from start to end."""
    sx, sy, sz = start
    ex, ey, ez = end
    dx, dy, dz = ex - sx, ey - sy, ez - sz
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if length <= 0.0:
        raise ValueError("zero-length cylinder")

    # All angled struts in this model live in an XZ plane.  Rotate local +Z to
    # the desired vector with a pitch about local Y.
    if abs(dy) > 1e-9:
        raise ValueError("helper is intended for XZ-plane cylinders")
    pitch = math.atan2(dx, dz)
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(
            xyz=((sx + ex) * 0.5, (sy + ey) * 0.5, (sz + ez) * 0.5),
            rpy=(0.0, pitch, 0.0),
        ),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_stunt_scooter")

    blue = model.material("anodized_blue", rgba=(0.02, 0.20, 0.82, 1.0))
    black = model.material("matte_black", rgba=(0.005, 0.005, 0.006, 1.0))
    grip = model.material("sandpaper_grip", rgba=(0.015, 0.014, 0.012, 1.0))
    steel = model.material("brushed_steel", rgba=(0.64, 0.66, 0.68, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.18, 0.19, 0.20, 1.0))
    urethane = model.material("smoked_urethane", rgba=(0.82, 0.78, 0.64, 1.0))
    red = model.material("red_end_caps", rgba=(0.85, 0.04, 0.03, 1.0))

    deck_profile = rounded_rect_profile(0.72, 0.14, 0.045, corner_segments=10)
    deck_mesh = mesh_from_geometry(
        ExtrudeGeometry(deck_profile, 0.046, center=True),
        "solid_rounded_deck",
    )
    grip_profile = rounded_rect_profile(0.62, 0.116, 0.030, corner_segments=8)
    grip_mesh = mesh_from_geometry(
        ExtrudeGeometry(grip_profile, 0.004, center=True),
        "recessed_grip_tape",
    )

    wheel_core_mesh = mesh_from_geometry(
        WheelGeometry(
            0.038,
            0.028,
            rim=WheelRim(
                inner_radius=0.024,
                flange_height=0.003,
                flange_thickness=0.0025,
                bead_seat_depth=0.0015,
            ),
            hub=WheelHub(radius=0.016, width=0.030, cap_style="domed"),
            face=WheelFace(dish_depth=0.004, front_inset=0.0015, rear_inset=0.0015),
            spokes=WheelSpokes(
                style="split_y",
                count=5,
                thickness=0.003,
                window_radius=0.006,
            ),
        ),
        "spoked_wheel_core",
    )
    tire_mesh = mesh_from_geometry(
        TireGeometry(
            0.055,
            0.026,
            inner_radius=0.039,
            carcass=TireCarcass(belt_width_ratio=0.74, sidewall_bulge=0.035),
            sidewall=TireSidewall(style="rounded", bulge=0.045),
            shoulder=TireShoulder(width=0.004, radius=0.003),
        ),
        "smooth_urethane_tire",
    )

    deck = model.part("deck")
    deck.visual(
        deck_mesh,
        origin=Origin(xyz=(-0.040, 0.0, 0.145)),
        material=blue,
        name="deck_shell",
    )
    deck.visual(
        grip_mesh,
        origin=Origin(xyz=(-0.060, 0.0, 0.171)),
        material=grip,
        name="grip_tape",
    )
    deck.visual(
        Cylinder(radius=0.032, length=0.205),
        origin=Origin(xyz=(0.335, 0.0, 0.245)),
        material=dark_steel,
        name="headtube_shell",
    )
    deck.visual(
        Box((0.115, 0.012, 0.044)),
        origin=Origin(xyz=(0.280, 0.058, 0.151)),
        material=blue,
        name="neck_weld_0",
    )
    deck.visual(
        Box((0.115, 0.012, 0.044)),
        origin=Origin(xyz=(0.280, -0.058, 0.151)),
        material=blue,
        name="neck_weld_1",
    )
    deck.visual(
        Box((0.110, 0.012, 0.074)),
        origin=Origin(xyz=(-0.385, 0.053, 0.087)),
        material=blue,
        name="rear_dropout_0",
    )
    deck.visual(
        Box((0.110, 0.012, 0.074)),
        origin=Origin(xyz=(-0.385, -0.053, 0.087)),
        material=blue,
        name="rear_dropout_1",
    )
    deck.visual(
        Cylinder(radius=0.0065, length=0.138),
        origin=Origin(xyz=(-0.385, 0.0, 0.055), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="rear_axle",
    )
    deck.visual(
        Box((0.105, 0.088, 0.010)),
        origin=Origin(xyz=(-0.340, 0.0, 0.125), rpy=(0.0, -0.16, 0.0)),
        material=black,
        name="rear_brake",
    )

    front_fork = model.part("front_fork")
    front_fork.visual(
        Cylinder(radius=0.017, length=0.285),
        origin=Origin(xyz=(0.0, 0.0, 0.000)),
        material=steel,
        name="steerer",
    )
    front_fork.visual(
        Box((0.070, 0.086, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, 0.118)),
        material=dark_steel,
        name="folding_clamp",
    )
    front_fork.visual(
        Box((0.045, 0.014, 0.054)),
        origin=Origin(xyz=(-0.043, 0.038, 0.147)),
        material=dark_steel,
        name="hinge_cheek_0",
    )
    front_fork.visual(
        Box((0.045, 0.014, 0.054)),
        origin=Origin(xyz=(-0.043, -0.038, 0.147)),
        material=dark_steel,
        name="hinge_cheek_1",
    )
    front_fork.visual(
        Cylinder(radius=0.006, length=0.104),
        origin=Origin(xyz=(-0.043, 0.0, 0.147), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="folding_pin",
    )
    front_fork.visual(
        Box((0.095, 0.102, 0.026)),
        origin=Origin(xyz=(0.040, 0.0, -0.112)),
        material=dark_steel,
        name="fork_crown",
    )
    _cylinder_between(
        front_fork,
        (0.035, 0.043, -0.110),
        (0.070, 0.043, -0.180),
        0.010,
        dark_steel,
        "fork_leg_0",
    )
    _cylinder_between(
        front_fork,
        (0.035, -0.043, -0.110),
        (0.070, -0.043, -0.180),
        0.010,
        dark_steel,
        "fork_leg_1",
    )
    front_fork.visual(
        Box((0.030, 0.018, 0.030)),
        origin=Origin(xyz=(0.070, 0.043, -0.180)),
        material=dark_steel,
        name="front_dropout_0",
    )
    front_fork.visual(
        Box((0.030, 0.018, 0.030)),
        origin=Origin(xyz=(0.070, -0.043, -0.180)),
        material=dark_steel,
        name="front_dropout_1",
    )
    front_fork.visual(
        Cylinder(radius=0.0065, length=0.138),
        origin=Origin(xyz=(0.070, 0.0, -0.180), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="front_axle",
    )

    stem = model.part("stem")
    stem.visual(
        Cylinder(radius=0.018, length=0.056),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="hinge_barrel",
    )
    stem.visual(
        Cylinder(radius=0.018, length=0.740),
        origin=Origin(xyz=(0.0, 0.0, 0.410)),
        material=steel,
        name="upright_tube",
    )
    stem.visual(
        Box((0.056, 0.058, 0.070)),
        origin=Origin(xyz=(0.0, 0.0, 0.047)),
        material=dark_steel,
        name="stem_clamp",
    )
    stem.visual(
        Cylinder(radius=0.0155, length=0.520),
        origin=Origin(xyz=(0.0, 0.0, 0.775), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="handlebar_crossbar",
    )
    stem.visual(
        Cylinder(radius=0.019, length=0.105),
        origin=Origin(xyz=(0.0, 0.262, 0.775), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="grip_0",
    )
    stem.visual(
        Cylinder(radius=0.019, length=0.105),
        origin=Origin(xyz=(0.0, -0.262, 0.775), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="grip_1",
    )
    stem.visual(
        Sphere(radius=0.021),
        origin=Origin(xyz=(0.0, 0.320, 0.775)),
        material=red,
        name="bar_end_0",
    )
    stem.visual(
        Sphere(radius=0.021),
        origin=Origin(xyz=(0.0, -0.320, 0.775)),
        material=red,
        name="bar_end_1",
    )

    front_wheel = model.part("front_wheel")
    front_wheel.visual(
        tire_mesh,
        origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
        material=urethane,
        name="front_tire",
    )
    front_wheel.visual(
        wheel_core_mesh,
        origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
        material=steel,
        name="front_core",
    )

    rear_wheel = model.part("rear_wheel")
    rear_wheel.visual(
        tire_mesh,
        origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
        material=urethane,
        name="rear_tire",
    )
    rear_wheel.visual(
        wheel_core_mesh,
        origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
        material=steel,
        name="rear_core",
    )

    model.articulation(
        "steering",
        ArticulationType.CONTINUOUS,
        parent=deck,
        child=front_fork,
        origin=Origin(xyz=(0.335, 0.0, 0.245)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=5.0),
    )
    model.articulation(
        "folding_hinge",
        ArticulationType.REVOLUTE,
        parent=front_fork,
        child=stem,
        origin=Origin(xyz=(-0.043, 0.0, 0.147)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=1.8, lower=0.0, upper=1.62),
    )
    model.articulation(
        "front_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=front_fork,
        child=front_wheel,
        origin=Origin(xyz=(0.070, 0.0, -0.180)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=40.0),
    )
    model.articulation(
        "rear_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=deck,
        child=rear_wheel,
        origin=Origin(xyz=(-0.385, 0.0, 0.055)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=40.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    deck = object_model.get_part("deck")
    front_fork = object_model.get_part("front_fork")
    stem = object_model.get_part("stem")
    front_wheel = object_model.get_part("front_wheel")
    rear_wheel = object_model.get_part("rear_wheel")
    steering = object_model.get_articulation("steering")
    folding = object_model.get_articulation("folding_hinge")
    front_spin = object_model.get_articulation("front_wheel_spin")
    rear_spin = object_model.get_articulation("rear_wheel_spin")

    ctx.allow_overlap(
        deck,
        front_fork,
        elem_a="headtube_shell",
        elem_b="steerer",
        reason="The steerer is intentionally represented as a captured shaft running inside the headtube bearing.",
    )
    ctx.allow_overlap(
        front_fork,
        stem,
        elem_a="folding_pin",
        elem_b="hinge_barrel",
        reason="The folding hinge pin is intentionally nested inside the stem barrel.",
    )
    ctx.allow_overlap(
        front_fork,
        front_wheel,
        elem_a="front_axle",
        elem_b="front_core",
        reason="The wheel axle is intentionally represented as a captured shaft through the hub core.",
    )
    ctx.allow_overlap(
        deck,
        rear_wheel,
        elem_a="rear_axle",
        elem_b="rear_core",
        reason="The rear axle is intentionally represented as a captured shaft through the hub core.",
    )

    ctx.check(
        "primary joints are present",
        steering.articulation_type == ArticulationType.CONTINUOUS
        and folding.articulation_type == ArticulationType.REVOLUTE
        and front_spin.articulation_type == ArticulationType.CONTINUOUS
        and rear_spin.articulation_type == ArticulationType.CONTINUOUS,
        details="Scooter must steer, fold at the stem clamp, and roll both wheels.",
    )
    ctx.check(
        "folding hinge has realistic one-way travel",
        folding.motion_limits is not None
        and folding.motion_limits.lower == 0.0
        and folding.motion_limits.upper is not None
        and 1.4 <= folding.motion_limits.upper <= 1.8,
        details=f"limits={folding.motion_limits}",
    )

    ctx.expect_within(
        front_fork,
        deck,
        axes="xy",
        inner_elem="steerer",
        outer_elem="headtube_shell",
        margin=0.006,
        name="steerer stays centered in the headtube bearing",
    )
    ctx.expect_overlap(
        front_fork,
        deck,
        axes="z",
        elem_a="steerer",
        elem_b="headtube_shell",
        min_overlap=0.150,
        name="steerer remains captured through the headtube",
    )
    ctx.expect_overlap(
        front_fork,
        stem,
        axes="y",
        elem_a="folding_pin",
        elem_b="hinge_barrel",
        min_overlap=0.040,
        name="folding hinge pin crosses the stem barrel",
    )
    ctx.expect_overlap(
        front_fork,
        front_wheel,
        axes="y",
        elem_a="front_axle",
        elem_b="front_core",
        min_overlap=0.020,
        name="front axle spans the wheel hub",
    )
    ctx.expect_overlap(
        deck,
        rear_wheel,
        axes="y",
        elem_a="rear_axle",
        elem_b="rear_core",
        min_overlap=0.020,
        name="rear axle spans the wheel hub",
    )
    ctx.expect_gap(
        deck,
        rear_wheel,
        axis="z",
        positive_elem="deck_shell",
        negative_elem="rear_tire",
        min_gap=0.004,
        name="solid deck clears the rear tire",
    )

    closed_aabb = ctx.part_world_aabb(stem)
    with ctx.pose({folding: folding.motion_limits.upper}):
        folded_aabb = ctx.part_world_aabb(stem)
    ctx.check(
        "folding stem moves rearward and lower",
        closed_aabb is not None
        and folded_aabb is not None
        and folded_aabb[0][0] < closed_aabb[0][0] - 0.55
        and folded_aabb[1][2] < closed_aabb[1][2] - 0.45,
        details=f"closed={closed_aabb}, folded={folded_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
