from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    ExtrudeGeometry,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TireGeometry,
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
    rounded_rect_profile,
)
import cadquery as cq


def _tube_xz(part, start, end, radius, material, name):
    """Add a cylinder whose endpoints differ in X/Z only."""
    sx, sy, sz = start
    ex, ey, ez = end
    length = math.sqrt((ex - sx) ** 2 + (ez - sz) ** 2)
    angle_y = math.atan2(ex - sx, ez - sz)
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(
            xyz=((sx + ex) / 2.0, sy, (sz + ez) / 2.0),
            rpy=(0.0, angle_y, 0.0),
        ),
        material=material,
        name=name,
    )


def _tube_y(part, center, length, radius, material, name):
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=center, rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def _box(part, size, center, material, name):
    part.visual(Box(size), origin=Origin(xyz=center), material=material, name=name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cargo_delivery_kick_scooter")

    aluminum = model.material("brushed_aluminum", rgba=(0.72, 0.74, 0.72, 1.0))
    dark = model.material("matte_black", rgba=(0.015, 0.016, 0.018, 1.0))
    deck_blue = model.material("delivery_blue", rgba=(0.05, 0.22, 0.52, 1.0))
    rubber = model.material("soft_black_rubber", rgba=(0.005, 0.005, 0.006, 1.0))
    rim_silver = model.material("machined_silver", rgba=(0.82, 0.84, 0.82, 1.0))
    reflector_red = model.material("red_reflector", rgba=(0.9, 0.05, 0.03, 1.0))

    # Shared wheel/tire proportions: 11.5-inch class cargo scooter wheels.
    tire = TireGeometry(
        0.145,
        0.066,
        inner_radius=0.103,
        tread=TireTread(style="block", depth=0.006, count=24, land_ratio=0.58),
        sidewall=TireSidewall(style="rounded", bulge=0.045),
        shoulder=TireShoulder(width=0.006, radius=0.003),
    )
    rim = WheelGeometry(
        0.103,
        0.054,
        rim=WheelRim(inner_radius=0.072, flange_height=0.006, flange_thickness=0.004),
        hub=WheelHub(
            radius=0.027,
            width=0.050,
            cap_style="domed",
            bolt_pattern=BoltPattern(count=5, circle_diameter=0.034, hole_diameter=0.004),
        ),
        face=WheelFace(dish_depth=0.005, front_inset=0.003, rear_inset=0.003),
        spokes=WheelSpokes(style="split_y", count=5, thickness=0.004, window_radius=0.014),
        bore=WheelBore(style="round", diameter=0.012),
    )

    chassis = model.part("chassis")
    deck_profile = rounded_rect_profile(0.86, 0.34, 0.075, corner_segments=10)
    chassis.visual(
        mesh_from_geometry(ExtrudeGeometry(deck_profile, 0.055, center=True), "rounded_deck"),
        origin=Origin(xyz=(-0.05, 0.0, 0.205)),
        material=deck_blue,
        name="deck_shell",
    )
    _box(chassis, (0.75, 0.255, 0.008), (-0.06, 0.0, 0.237), dark, "grip_pad")
    _box(chassis, (0.81, 0.026, 0.035), (-0.05, 0.178, 0.213), aluminum, "side_rail_0")
    _box(chassis, (0.81, 0.026, 0.035), (-0.05, -0.178, 0.213), aluminum, "side_rail_1")
    _box(chassis, (0.14, 0.22, 0.085), (0.37, 0.0, 0.258), aluminum, "neck_block")
    chassis.visual(
        Cylinder(radius=0.055, length=0.12),
        origin=Origin(xyz=(0.43, 0.0, 0.300)),
        material=aluminum,
        name="steering_cup",
    )
    # Rear dropouts sit outside the spinning wheel, leaving visible running clearance.
    _box(chassis, (0.26, 0.024, 0.026), (-0.56, 0.105, 0.178), aluminum, "rear_stay_0")
    _box(chassis, (0.26, 0.024, 0.026), (-0.56, -0.105, 0.178), aluminum, "rear_stay_1")
    _box(chassis, (0.045, 0.046, 0.080), (-0.66, 0.095, 0.148), aluminum, "rear_dropout_0")
    _box(chassis, (0.045, 0.046, 0.080), (-0.66, -0.095, 0.148), aluminum, "rear_dropout_1")
    _box(chassis, (0.25, 0.30, 0.030), (-0.66, 0.0, 0.330), dark, "rear_fender")
    _box(chassis, (0.020, 0.030, 0.185), (-0.66, 0.105, 0.243), dark, "fender_strut_0")
    _box(chassis, (0.020, 0.030, 0.185), (-0.66, -0.105, 0.243), dark, "fender_strut_1")
    _box(chassis, (0.018, 0.20, 0.030), (-0.76, 0.0, 0.352), reflector_red, "tail_reflector")

    front_fork = model.part("front_fork")
    front_fork.visual(
        Cylinder(radius=0.030, length=0.205),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=aluminum,
        name="steerer_tube",
    )
    _box(front_fork, (0.16, 0.18, 0.034), (0.025, 0.0, 0.022), aluminum, "fork_crown")
    _tube_xz(front_fork, (0.055, 0.075, 0.018), (0.250, 0.075, -0.155), 0.018, aluminum, "fork_blade_0")
    _tube_xz(front_fork, (0.055, -0.075, 0.018), (0.250, -0.075, -0.155), 0.018, aluminum, "fork_blade_1")
    _box(front_fork, (0.045, 0.035, 0.078), (0.250, 0.075, -0.155), aluminum, "front_dropout_0")
    _box(front_fork, (0.045, 0.035, 0.078), (0.250, -0.075, -0.155), aluminum, "front_dropout_1")
    _box(front_fork, (0.13, 0.14, 0.030), (0.000, 0.0, 0.045), aluminum, "hinge_clamp")
    _box(front_fork, (0.043, 0.026, 0.082), (0.000, 0.058, 0.090), aluminum, "hinge_cheek_0")
    _box(front_fork, (0.043, 0.026, 0.082), (0.000, -0.058, 0.090), aluminum, "hinge_cheek_1")
    _tube_y(front_fork, (0.000, 0.0, 0.090), 0.132, 0.011, dark, "hinge_pin")
    _box(front_fork, (0.15, 0.030, 0.030), (0.075, 0.0, 0.082), aluminum, "basket_spine")
    _box(front_fork, (0.045, 0.370, 0.030), (0.120, 0.0, 0.082), aluminum, "basket_crossbar")

    basket = model.part("basket")
    # Open wire basket, sized for a small delivery parcel and tied back to fork lugs.
    y_half = 0.255
    x_back = 0.080
    x_front = 0.450
    z_floor = 0.165
    z_top = 0.435
    rod = 0.013
    # Top and bottom rectangular rails.
    _box(basket, (rod, 2 * y_half + rod, rod), (x_back, 0.0, z_top), dark, "top_rear_rail")
    _box(basket, (rod, 2 * y_half + rod, rod), (x_front, 0.0, z_top), dark, "top_front_rail")
    _box(basket, (x_front - x_back + rod, rod, rod), ((x_back + x_front) / 2, y_half, z_top), dark, "top_side_rail_0")
    _box(basket, (x_front - x_back + rod, rod, rod), ((x_back + x_front) / 2, -y_half, z_top), dark, "top_side_rail_1")
    _box(basket, (rod, 2 * y_half + rod, rod), (x_back, 0.0, z_floor), dark, "floor_rear_rail")
    _box(basket, (rod, 2 * y_half + rod, rod), (x_front, 0.0, z_floor), dark, "floor_front_rail")
    _box(basket, (x_front - x_back + rod, rod, rod), ((x_back + x_front) / 2, y_half, z_floor), dark, "floor_side_rail_0")
    _box(basket, (x_front - x_back + rod, rod, rod), ((x_back + x_front) / 2, -y_half, z_floor), dark, "floor_side_rail_1")
    # Corner and panel verticals.
    for x in (x_back, x_front):
        for y in (-y_half, y_half):
            _box(basket, (rod, rod, z_top - z_floor + rod), (x, y, (z_top + z_floor) / 2), dark, "basket_corner")
    for y in (-0.155, -0.055, 0.055, 0.155):
        _box(basket, (rod, rod, z_top - z_floor), (x_front, y, (z_top + z_floor) / 2), dark, "front_upright")
        _box(basket, (rod, rod, z_top - z_floor), (x_back, y, (z_top + z_floor) / 2), dark, "rear_upright")
    for x in (0.175, 0.265, 0.355):
        _box(basket, (rod, rod, z_top - z_floor), (x, y_half, (z_top + z_floor) / 2), dark, "side_upright_0")
        _box(basket, (rod, rod, z_top - z_floor), (x, -y_half, (z_top + z_floor) / 2), dark, "side_upright_1")
    # Floor grid.
    for y in (-0.18, -0.09, 0.0, 0.09, 0.18):
        _box(basket, (x_front - x_back, rod, rod), ((x_back + x_front) / 2, y, z_floor), dark, "floor_longitudinal")
    for x in (0.165, 0.250, 0.335, 0.420):
        _box(basket, (rod, 2 * y_half, rod), (x, 0.0, z_floor), dark, "floor_crossbar")
    _tube_xz(basket, (0.080, 0.155, 0.185), (0.120, 0.155, 0.112), 0.010, dark, "basket_strut_0")
    _tube_xz(basket, (0.080, -0.155, 0.185), (0.120, -0.155, 0.112), 0.010, dark, "basket_strut_1")
    _box(basket, (0.034, 0.075, 0.030), (0.120, 0.155, 0.112), dark, "mount_foot_0")
    _box(basket, (0.034, 0.075, 0.030), (0.120, -0.155, 0.112), dark, "mount_foot_1")

    stem = model.part("stem")
    _tube_y(stem, (0.0, 0.0, 0.0), 0.078, 0.033, aluminum, "stem_hinge_barrel")
    _tube_xz(stem, (0.0, 0.0, 0.030), (-0.205, 0.0, 0.885), 0.022, aluminum, "main_stem")
    _tube_xz(stem, (-0.205, 0.0, 0.615), (-0.205, 0.0, 0.785), 0.027, dark, "height_clamp")
    _tube_y(stem, (-0.220, 0.0, 0.900), 0.620, 0.019, aluminum, "handlebar")
    _tube_y(stem, (-0.220, 0.255, 0.900), 0.120, 0.023, rubber, "grip_0")
    _tube_y(stem, (-0.220, -0.255, 0.900), 0.120, 0.023, rubber, "grip_1")
    _box(stem, (0.050, 0.026, 0.050), (-0.205, 0.0, 0.865), dark, "bar_clamp")

    front_wheel = model.part("front_wheel")
    front_wheel.visual(
        mesh_from_geometry(tire, "front_tire"),
        origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
        material=rubber,
        name="tire",
    )
    front_wheel.visual(
        mesh_from_geometry(rim, "front_rim"),
        origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
        material=rim_silver,
        name="rim",
    )
    _tube_y(front_wheel, (0.0, 0.0, 0.0), 0.170, 0.012, rim_silver, "front_axle_stub")

    rear_wheel = model.part("rear_wheel")
    rear_wheel.visual(
        mesh_from_geometry(tire, "rear_tire"),
        origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
        material=rubber,
        name="tire",
    )
    rear_wheel.visual(
        mesh_from_geometry(rim, "rear_rim"),
        origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
        material=rim_silver,
        name="rim",
    )
    _tube_y(rear_wheel, (0.0, 0.0, 0.0), 0.170, 0.012, rim_silver, "rear_axle_stub")

    model.articulation(
        "steering_bearing",
        ArticulationType.CONTINUOUS,
        parent=chassis,
        child=front_fork,
        origin=Origin(xyz=(0.43, 0.0, 0.300)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=4.0),
    )
    model.articulation(
        "basket_mount",
        ArticulationType.FIXED,
        parent=front_fork,
        child=basket,
        origin=Origin(),
    )
    model.articulation(
        "stem_hinge",
        ArticulationType.REVOLUTE,
        parent=front_fork,
        child=stem,
        origin=Origin(xyz=(0.0, 0.0, 0.090)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=55.0, velocity=1.7, lower=-1.45, upper=0.0),
    )
    model.articulation(
        "front_axle",
        ArticulationType.CONTINUOUS,
        parent=front_fork,
        child=front_wheel,
        origin=Origin(xyz=(0.250, 0.0, -0.155)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=25.0),
    )
    model.articulation(
        "rear_axle",
        ArticulationType.CONTINUOUS,
        parent=chassis,
        child=rear_wheel,
        origin=Origin(xyz=(-0.660, 0.0, 0.145)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=25.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    front_fork = object_model.get_part("front_fork")
    basket = object_model.get_part("basket")
    stem = object_model.get_part("stem")
    front_wheel = object_model.get_part("front_wheel")
    rear_wheel = object_model.get_part("rear_wheel")
    steering = object_model.get_articulation("steering_bearing")
    front_axle = object_model.get_articulation("front_axle")
    rear_axle = object_model.get_articulation("rear_axle")
    hinge = object_model.get_articulation("stem_hinge")

    ctx.allow_overlap(
        "chassis",
        front_fork,
        reason="The fork steerer is intentionally captured inside the steering bearing cup.",
    )
    ctx.allow_overlap(
        front_fork,
        stem,
        reason="The folding hinge pin intentionally passes through the stem hinge barrel.",
    )
    ctx.allow_overlap(
        front_fork,
        front_wheel,
        reason="The front hub axle stub is intentionally seated in the fork dropout slots.",
    )
    ctx.allow_overlap(
        "chassis",
        rear_wheel,
        reason="The rear hub axle stub is intentionally seated in the rear dropout slots.",
    )

    ctx.check(
        "continuous steering bearing",
        steering.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={steering.articulation_type}",
    )
    ctx.check(
        "both wheels spin continuously",
        front_axle.articulation_type == ArticulationType.CONTINUOUS
        and rear_axle.articulation_type == ArticulationType.CONTINUOUS,
        details=f"front={front_axle.articulation_type}, rear={rear_axle.articulation_type}",
    )
    limits = hinge.motion_limits
    ctx.check(
        "stem has rear folding range",
        hinge.articulation_type == ArticulationType.REVOLUTE
        and limits is not None
        and limits.lower is not None
        and limits.upper == 0.0
        and limits.lower < -1.2,
        details=f"type={hinge.articulation_type}, limits={limits}",
    )
    ctx.expect_within(
        front_wheel,
        front_fork,
        axes="y",
        margin=0.005,
        name="front wheel captured between fork blades",
    )
    ctx.expect_within(
        rear_wheel,
        "chassis",
        axes="y",
        margin=0.005,
        name="rear wheel centered in dropouts",
    )
    ctx.expect_overlap(
        front_fork,
        "chassis",
        axes="xy",
        min_overlap=0.050,
        name="steerer overlaps bearing cup footprint",
    )
    ctx.expect_overlap(
        front_fork,
        "chassis",
        axes="z",
        min_overlap=0.100,
        name="steerer retained through bearing cup",
    )
    ctx.expect_overlap(
        front_fork,
        stem,
        axes="y",
        min_overlap=0.070,
        name="hinge pin spans the stem barrel",
    )
    ctx.expect_overlap(
        front_fork,
        front_wheel,
        axes="y",
        min_overlap=0.120,
        name="front axle spans fork dropouts",
    )
    ctx.expect_overlap(
        "chassis",
        rear_wheel,
        axes="y",
        min_overlap=0.120,
        name="rear axle spans dropouts",
    )
    ctx.expect_gap(
        basket,
        front_wheel,
        axis="z",
        min_gap=0.080,
        name="basket floor clears front wheel",
    )
    ctx.expect_overlap(
        basket,
        front_wheel,
        axes="xy",
        min_overlap=0.050,
        name="basket projects over front wheel",
    )

    upright_bar = ctx.part_world_aabb(stem)
    with ctx.pose({hinge: -1.35}):
        folded_bar = ctx.part_world_aabb(stem)
    ctx.check(
        "folding hinge lowers handlebar",
        upright_bar is not None
        and folded_bar is not None
        and folded_bar[1][2] < upright_bar[1][2] - 0.35,
        details=f"upright={upright_bar}, folded={folded_bar}",
    )

    return ctx.report()


object_model = build_object_model()
