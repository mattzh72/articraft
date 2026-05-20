from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
    Inertial,
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
    ExtrudeGeometry,
    tube_from_spline_points,
)


WHEEL_RADIUS = 0.095
WHEEL_WIDTH = 0.036
FRONT_AXLE_X = 0.50
REAR_AXLE_X = -0.46
AXLE_Z = WHEEL_RADIUS
STEER_ORIGIN = (0.355, 0.0, 0.225)
FRONT_AXLE_IN_YOKE = (
    FRONT_AXLE_X - STEER_ORIGIN[0],
    0.0,
    AXLE_Z - STEER_ORIGIN[2],
)
FOLD_HINGE_Z = 0.070
FOLD_UPPER = 1.50


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _wheel_meshes(prefix: str):
    wheel = _mesh(
        f"{prefix}_rim",
        WheelGeometry(
            0.074,
            WHEEL_WIDTH * 0.82,
            rim=WheelRim(
                inner_radius=0.050,
                flange_height=0.006,
                flange_thickness=0.0022,
                bead_seat_depth=0.002,
            ),
            hub=WheelHub(
                radius=0.020,
                width=WHEEL_WIDTH * 0.74,
                cap_style="domed",
                bolt_pattern=BoltPattern(count=5, circle_diameter=0.026, hole_diameter=0.003),
            ),
            face=WheelFace(dish_depth=0.0035, front_inset=0.0018, rear_inset=0.0015),
            spokes=WheelSpokes(style="split_y", count=5, thickness=0.0025, window_radius=0.007),
            bore=WheelBore(style="round", diameter=0.010),
        ),
    )
    tire = _mesh(
        f"{prefix}_tire",
        TireGeometry(
            WHEEL_RADIUS,
            WHEEL_WIDTH,
            inner_radius=0.075,
            tread=TireTread(style="circumferential", depth=0.0024, count=4),
            grooves=(
                TireGroove(center_offset=-0.010, width=0.0025, depth=0.0016),
                TireGroove(center_offset=0.010, width=0.0025, depth=0.0016),
            ),
            sidewall=TireSidewall(style="rounded", bulge=0.045),
        ),
    )
    return wheel, tire


def _add_scooter_wheel(part, prefix: str, *, wheel_material, tire_material, hub_material) -> None:
    wheel_mesh, tire_mesh = _wheel_meshes(prefix)
    part.visual(tire_mesh, material=tire_material, name="tire")
    part.visual(wheel_mesh, material=wheel_material, name="rim")
    part.visual(
        Cylinder(radius=0.013, length=WHEEL_WIDTH + 0.012),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=hub_material,
        name="hub_cap",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_kick_scooter")

    anodized = model.material("champagne_anodized_aluminum", rgba=(0.78, 0.72, 0.62, 1.0))
    satin_black = model.material("satin_black_metal", rgba=(0.035, 0.037, 0.040, 1.0))
    black_plastic = model.material("black_textured_plastic", rgba=(0.015, 0.015, 0.016, 1.0))
    dark_rubber = model.material("dark_rubber", rgba=(0.025, 0.024, 0.023, 1.0))
    silver = model.material("brushed_silver", rgba=(0.72, 0.74, 0.76, 1.0))
    charcoal = model.material("charcoal_detail", rgba=(0.16, 0.17, 0.18, 1.0))
    accent = model.material("muted_blue_release", rgba=(0.10, 0.25, 0.46, 1.0))

    deck = model.part("deck")
    deck.inertial = Inertial.from_geometry(
        Box((0.72, 0.17, 0.055)),
        mass=2.4,
        origin=Origin(xyz=(0.0, 0.0, 0.145)),
    )
    deck.visual(
        _mesh(
            "rounded_low_deck",
            ExtrudeGeometry(rounded_rect_profile(0.72, 0.155, 0.045, corner_segments=10), 0.046, center=True),
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.145)),
        material=anodized,
        name="rounded_deck_shell",
    )
    deck.visual(
        Box((0.62, 0.108, 0.006)),
        origin=Origin(xyz=(-0.015, 0.0, 0.171)),
        material=black_plastic,
        name="single_grip_pad",
    )
    for i, x in enumerate((-0.245, -0.145, -0.045, 0.055, 0.155, 0.255)):
        deck.visual(
            Box((0.050, 0.101, 0.0018)),
            origin=Origin(xyz=(x, 0.0, 0.1749)),
            material=charcoal,
            name=f"grip_groove_{i}",
        )
    deck.visual(
        Box((0.66, 0.024, 0.045)),
        origin=Origin(xyz=(-0.015, 0.080, 0.144)),
        material=silver,
        name="side_rail_0",
    )
    deck.visual(
        Box((0.66, 0.024, 0.045)),
        origin=Origin(xyz=(-0.015, -0.080, 0.144)),
        material=silver,
        name="side_rail_1",
    )
    deck.visual(
        Box((0.065, 0.145, 0.050)),
        origin=Origin(xyz=(0.312, 0.0, 0.198)),
        material=satin_black,
        name="front_socket_block",
    )
    deck.visual(
        Cylinder(radius=0.030, length=0.120),
        origin=Origin(xyz=(0.355, 0.0, 0.245), rpy=(pi / 2.0, 0.0, 0.0)),
        material=silver,
        name="headset_bearing_ring",
    )
    deck.visual(
        Box((0.070, 0.118, 0.018)),
        origin=Origin(xyz=(REAR_AXLE_X + 0.085, 0.0, 0.207)),
        material=satin_black,
        name="rear_fender_mount",
    )
    deck.visual(
        Box((0.040, 0.018, 0.112)),
        origin=Origin(xyz=(REAR_AXLE_X, 0.036, AXLE_Z + 0.012)),
        material=satin_black,
        name="rear_fork_0",
    )
    deck.visual(
        Box((0.040, 0.018, 0.112)),
        origin=Origin(xyz=(REAR_AXLE_X, -0.036, AXLE_Z + 0.012)),
        material=satin_black,
        name="rear_fork_1",
    )
    deck.visual(
        Box((0.155, 0.018, 0.025)),
        origin=Origin(xyz=(REAR_AXLE_X + 0.075, 0.036, AXLE_Z + 0.035)),
        material=satin_black,
        name="rear_stay_0",
    )
    deck.visual(
        Box((0.155, 0.018, 0.025)),
        origin=Origin(xyz=(REAR_AXLE_X + 0.075, -0.036, AXLE_Z + 0.035)),
        material=satin_black,
        name="rear_stay_1",
    )
    deck.visual(
        Cylinder(radius=0.010, length=0.100),
        origin=Origin(xyz=(REAR_AXLE_X, 0.0, AXLE_Z), rpy=(pi / 2.0, 0.0, 0.0)),
        material=silver,
        name="rear_axle_pin",
    )
    deck.visual(
        _mesh(
            "rear_fender_arch",
            tube_from_spline_points(
                [
                    (REAR_AXLE_X - 0.105, 0.0, AXLE_Z + 0.050),
                    (REAR_AXLE_X - 0.055, 0.0, AXLE_Z + 0.120),
                    (REAR_AXLE_X + 0.045, 0.0, AXLE_Z + 0.125),
                    (REAR_AXLE_X + 0.120, 0.0, AXLE_Z + 0.060),
                ],
                radius=0.013,
                samples_per_segment=16,
                radial_segments=18,
                cap_ends=True,
            ),
        ),
        material=satin_black,
        name="rear_brake_fender",
    )

    steering_yoke = model.part("steering_yoke")
    steering_yoke.inertial = Inertial.from_geometry(
        Box((0.13, 0.13, 0.23)),
        mass=0.95,
        origin=Origin(xyz=(0.035, 0.0, -0.045)),
    )
    steering_yoke.visual(
        Cylinder(radius=0.024, length=0.085),
        origin=Origin(xyz=(0.0, 0.0, 0.000)),
        material=satin_black,
        name="steerer_tube",
    )
    steering_yoke.visual(
        Box((0.050, 0.070, 0.034)),
        origin=Origin(xyz=(0.015, 0.0, -0.030)),
        material=satin_black,
        name="fork_crown",
    )
    for idx, y in enumerate((0.034, -0.034)):
        steering_yoke.visual(
            _mesh(
                f"front_fork_blade_{idx}",
                tube_from_spline_points(
                    [
                        (0.026, y, -0.038),
                        (0.058, y, -0.108),
                        (FRONT_AXLE_IN_YOKE[0], y, FRONT_AXLE_IN_YOKE[2]),
                    ],
                    radius=0.012,
                    samples_per_segment=12,
                    radial_segments=16,
                    cap_ends=True,
                ),
            ),
            material=satin_black,
            name=f"front_fork_blade_{idx}",
        )
        steering_yoke.visual(
            Box((0.038, 0.018, 0.050)),
            origin=Origin(xyz=(FRONT_AXLE_IN_YOKE[0], y, FRONT_AXLE_IN_YOKE[2] + 0.005)),
            material=satin_black,
            name=f"front_dropout_{idx}",
        )
    steering_yoke.visual(
        Cylinder(radius=0.010, length=0.104),
        origin=Origin(xyz=FRONT_AXLE_IN_YOKE, rpy=(pi / 2.0, 0.0, 0.0)),
        material=silver,
        name="front_axle_pin",
    )
    steering_yoke.visual(
        Box((0.052, 0.090, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, FOLD_HINGE_Z - 0.035)),
        material=satin_black,
        name="hinge_web",
    )
    steering_yoke.visual(
        Cylinder(radius=0.024, length=0.026),
        origin=Origin(xyz=(0.0, 0.035, FOLD_HINGE_Z), rpy=(pi / 2.0, 0.0, 0.0)),
        material=silver,
        name="fold_knuckle_0",
    )
    steering_yoke.visual(
        Cylinder(radius=0.024, length=0.026),
        origin=Origin(xyz=(0.0, -0.035, FOLD_HINGE_Z), rpy=(pi / 2.0, 0.0, 0.0)),
        material=silver,
        name="fold_knuckle_1",
    )
    steering_yoke.visual(
        Box((0.052, 0.028, 0.020)),
        origin=Origin(xyz=(-0.023, 0.054, FOLD_HINGE_Z + 0.032)),
        material=accent,
        name="release_latch",
    )

    folding_stem = model.part("folding_stem")
    folding_stem.inertial = Inertial.from_geometry(
        Box((0.11, 0.52, 0.76)),
        mass=1.35,
        origin=Origin(xyz=(-0.020, 0.0, 0.360)),
    )
    folding_stem.visual(
        Cylinder(radius=0.021, length=0.034),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=satin_black,
        name="center_hinge_barrel",
    )
    folding_stem.visual(
        Cylinder(radius=0.0105, length=0.098),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=silver,
        name="hinge_pin",
    )
    folding_stem.visual(
        _mesh(
            "brushed_stem_tube",
            tube_from_spline_points(
                [
                    (0.000, 0.0, 0.020),
                    (-0.010, 0.0, 0.210),
                    (-0.026, 0.0, 0.460),
                    (-0.038, 0.0, 0.665),
                ],
                radius=0.0185,
                samples_per_segment=20,
                radial_segments=24,
                cap_ends=True,
            ),
        ),
        material=silver,
        name="main_stem_tube",
    )
    folding_stem.visual(
        Cylinder(radius=0.026, length=0.075),
        origin=Origin(xyz=(-0.003, 0.0, 0.105)),
        material=satin_black,
        name="lower_clamp_collar",
    )
    folding_stem.visual(
        Cylinder(radius=0.024, length=0.055),
        origin=Origin(xyz=(-0.031, 0.0, 0.505)),
        material=satin_black,
        name="height_clamp_collar",
    )
    folding_stem.visual(
        Box((0.045, 0.030, 0.050)),
        origin=Origin(xyz=(-0.052, -0.023, 0.507)),
        material=accent,
        name="height_lock_lever",
    )
    folding_stem.visual(
        Box((0.085, 0.070, 0.045)),
        origin=Origin(xyz=(-0.040, 0.0, 0.675)),
        material=satin_black,
        name="bar_clamp_block",
    )
    folding_stem.visual(
        Cylinder(radius=0.014, length=0.500),
        origin=Origin(xyz=(-0.040, 0.0, 0.690), rpy=(pi / 2.0, 0.0, 0.0)),
        material=silver,
        name="handlebar_cross_tube",
    )
    folding_stem.visual(
        Cylinder(radius=0.020, length=0.110),
        origin=Origin(xyz=(-0.040, 0.295, 0.690), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_rubber,
        name="grip_0",
    )
    folding_stem.visual(
        Cylinder(radius=0.020, length=0.110),
        origin=Origin(xyz=(-0.040, -0.295, 0.690), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_rubber,
        name="grip_1",
    )
    folding_stem.visual(
        Box((0.030, 0.018, 0.030)),
        origin=Origin(xyz=(-0.040, 0.185, 0.672)),
        material=black_plastic,
        name="bell_button",
    )

    front_wheel = model.part("front_wheel")
    front_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=WHEEL_RADIUS, length=WHEEL_WIDTH),
        mass=0.34,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    _add_scooter_wheel(front_wheel, "front", wheel_material=silver, tire_material=dark_rubber, hub_material=charcoal)

    rear_wheel = model.part("rear_wheel")
    rear_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=WHEEL_RADIUS, length=WHEEL_WIDTH),
        mass=0.34,
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
    )
    _add_scooter_wheel(rear_wheel, "rear", wheel_material=silver, tire_material=dark_rubber, hub_material=charcoal)

    model.articulation(
        "steering",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=steering_yoke,
        origin=Origin(xyz=STEER_ORIGIN),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.2, lower=-0.70, upper=0.70),
    )
    model.articulation(
        "fold_hinge",
        ArticulationType.REVOLUTE,
        parent=steering_yoke,
        child=folding_stem,
        origin=Origin(xyz=(0.0, 0.0, FOLD_HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=1.2, lower=0.0, upper=FOLD_UPPER),
    )
    model.articulation(
        "front_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=steering_yoke,
        child=front_wheel,
        origin=Origin(xyz=FRONT_AXLE_IN_YOKE, rpy=(0.0, 0.0, pi / 2.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=30.0),
    )
    model.articulation(
        "rear_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=deck,
        child=rear_wheel,
        origin=Origin(xyz=(REAR_AXLE_X, 0.0, AXLE_Z), rpy=(0.0, 0.0, pi / 2.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=30.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    deck = object_model.get_part("deck")
    yoke = object_model.get_part("steering_yoke")
    stem = object_model.get_part("folding_stem")
    front_wheel = object_model.get_part("front_wheel")
    rear_wheel = object_model.get_part("rear_wheel")
    steering = object_model.get_articulation("steering")
    fold = object_model.get_articulation("fold_hinge")
    front_spin = object_model.get_articulation("front_wheel_spin")
    rear_spin = object_model.get_articulation("rear_wheel_spin")

    for joint, label in (
        (steering, "steering joint"),
        (fold, "folding hinge"),
        (front_spin, "front rolling joint"),
        (rear_spin, "rear rolling joint"),
    ):
        ctx.check(f"{label} present", joint is not None, f"Missing {label}.")

    ctx.allow_overlap(
        "deck",
        "steering_yoke",
        elem_a="headset_bearing_ring",
        elem_b="steerer_tube",
        reason="The steerer tube is intentionally captured inside the headset bearing ring.",
    )
    ctx.allow_overlap(
        "deck",
        "steering_yoke",
        elem_a="front_socket_block",
        elem_b="steerer_tube",
        reason="The simplified front socket block surrounds the lower steerer as an integrated bearing housing.",
    )
    ctx.allow_overlap(
        "deck",
        "steering_yoke",
        elem_a="headset_bearing_ring",
        elem_b="hinge_web",
        reason="The rotating yoke collar is intentionally seated inside the headset bearing stack.",
    )
    ctx.expect_within(
        yoke,
        deck,
        axes="xy",
        inner_elem="steerer_tube",
        outer_elem="headset_bearing_ring",
        margin=0.004,
        name="steerer is centered in headset",
    )
    ctx.expect_overlap(
        yoke,
        deck,
        axes="z",
        elem_a="steerer_tube",
        elem_b="headset_bearing_ring",
        min_overlap=0.015,
        name="steerer passes through headset bearing",
    )
    ctx.expect_overlap(
        yoke,
        deck,
        axes="z",
        elem_a="hinge_web",
        elem_b="headset_bearing_ring",
        min_overlap=0.005,
        name="yoke collar is seated in headset stack",
    )
    ctx.allow_overlap(
        "steering_yoke",
        "folding_stem",
        elem_a="fold_knuckle_0",
        elem_b="hinge_pin",
        reason="The visible hinge pin intentionally runs through the outer folding knuckle.",
    )
    ctx.allow_overlap(
        "steering_yoke",
        "folding_stem",
        elem_a="fold_knuckle_1",
        elem_b="hinge_pin",
        reason="The visible hinge pin intentionally runs through the outer folding knuckle.",
    )
    ctx.allow_overlap(
        "folding_stem",
        "steering_yoke",
        elem_a="center_hinge_barrel",
        elem_b="hinge_pin",
        reason="The hinge pin is captured through the center barrel of the folding stem.",
    )
    ctx.expect_overlap(
        stem,
        yoke,
        axes="xz",
        elem_a="center_hinge_barrel",
        elem_b="fold_knuckle_0",
        min_overlap=0.002,
        name="fold hinge barrels share a coaxial side profile",
    )
    ctx.allow_overlap(
        "steering_yoke",
        "front_wheel",
        elem_a="front_axle_pin",
        elem_b="hub_cap",
        reason="The front axle pin is intentionally nested through the wheel hub.",
    )
    ctx.allow_overlap(
        "steering_yoke",
        "front_wheel",
        elem_a="front_axle_pin",
        elem_b="rim",
        reason="The front axle passes through the wheel bore represented inside the rim mesh.",
    )
    ctx.allow_overlap(
        "deck",
        "rear_wheel",
        elem_a="rear_axle_pin",
        elem_b="hub_cap",
        reason="The rear axle pin is intentionally nested through the wheel hub.",
    )
    ctx.allow_overlap(
        "deck",
        "rear_wheel",
        elem_a="rear_axle_pin",
        elem_b="rim",
        reason="The rear axle passes through the wheel bore represented inside the rim mesh.",
    )
    ctx.expect_overlap(
        front_wheel,
        yoke,
        axes="xy",
        elem_a="hub_cap",
        elem_b="front_axle_pin",
        min_overlap=0.008,
        name="front wheel hub is retained on axle",
    )
    ctx.expect_overlap(
        rear_wheel,
        deck,
        axes="xy",
        elem_a="hub_cap",
        elem_b="rear_axle_pin",
        min_overlap=0.008,
        name="rear wheel hub is retained on axle",
    )

    upright_aabb = ctx.part_world_aabb(stem)
    with ctx.pose({fold: FOLD_UPPER}):
        folded_aabb = ctx.part_world_aabb(stem)
    ctx.check(
        "folding stem lowers toward deck",
        upright_aabb is not None
        and folded_aabb is not None
        and folded_aabb[1][2] < upright_aabb[1][2] - 0.45
        and folded_aabb[0][0] < upright_aabb[0][0] - 0.30,
        details=f"upright={upright_aabb}, folded={folded_aabb}",
    )

    rest_front = ctx.part_world_position(front_wheel)
    with ctx.pose({steering: 0.55}):
        steered_front = ctx.part_world_position(front_wheel)
    ctx.check(
        "steering yaws front wheel",
        rest_front is not None and steered_front is not None and steered_front[1] > rest_front[1] + 0.025,
        details=f"rest={rest_front}, steered={steered_front}",
    )

    rest_rear = ctx.part_world_position(rear_wheel)
    with ctx.pose({rear_spin: pi}):
        spun_rear = ctx.part_world_position(rear_wheel)
    ctx.check(
        "rear wheel spins about a fixed axle",
        rest_rear is not None
        and spun_rear is not None
        and max(abs(rest_rear[i] - spun_rear[i]) for i in range(3)) < 1e-6,
        details=f"rest={rest_rear}, spun={spun_rear}",
    )

    return ctx.report()


object_model = build_object_model()
