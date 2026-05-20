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


def _cylinder_between(
    part,
    name: str,
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    radius: float,
    material: Material | str,
) -> None:
    """Add a round tube with its local cylinder axis running from start to end."""
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    dz = end[2] - start[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if length <= 0.0:
        raise ValueError(f"zero-length tube {name}")

    center = (
        (start[0] + end[0]) * 0.5,
        (start[1] + end[1]) * 0.5,
        (start[2] + end[2]) * 0.5,
    )
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(math.sqrt(dx * dx + dy * dy), dz)
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=center, rpy=(0.0, pitch, yaw)),
        material=material,
        name=name,
    )


def _add_wheel_visuals(
    wheel_part,
    *,
    radius: float,
    width: float,
    tire_mesh,
    wheel_mesh,
    rubber: Material | str,
    rim: Material | str,
) -> None:
    """Add a tire and rim aligned so the authored wheel spins around local Y."""
    spin_alignment = Origin(rpy=(0.0, 0.0, math.pi / 2.0))
    wheel_part.visual(tire_mesh, origin=spin_alignment, material=rubber, name="tire")
    wheel_part.visual(wheel_mesh, origin=spin_alignment, material=rim, name="rim")
    # Small center cap gives the side face a visible bearing target for the fork/axle.
    wheel_part.visual(
        Cylinder(radius=radius * 0.22, length=width * 1.08),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rim,
        name="hub_cap",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="heavy_duty_knee_scooter")

    frame_blue = model.material("powder_coated_blue", rgba=(0.05, 0.18, 0.55, 1.0))
    satin_metal = model.material("satin_metal", rgba=(0.62, 0.65, 0.67, 1.0))
    dark_metal = model.material("dark_hardware", rgba=(0.05, 0.055, 0.06, 1.0))
    rubber = model.material("black_rubber", rgba=(0.005, 0.005, 0.004, 1.0))
    vinyl = model.material("padded_vinyl", rgba=(0.035, 0.035, 0.045, 1.0))
    warning = model.material("stand_safety_orange", rgba=(0.95, 0.43, 0.06, 1.0))

    utility_tire = TireGeometry(
        0.115,
        0.055,
        inner_radius=0.078,
        carcass=TireCarcass(belt_width_ratio=0.72, sidewall_bulge=0.05),
        tread=TireTread(style="block", depth=0.006, count=18, land_ratio=0.58),
        grooves=(TireGroove(center_offset=0.0, width=0.006, depth=0.0025),),
        sidewall=TireSidewall(style="rounded", bulge=0.05),
        shoulder=TireShoulder(width=0.006, radius=0.003),
    )
    utility_rim = WheelGeometry(
        0.081,
        0.050,
        rim=WheelRim(inner_radius=0.050, flange_height=0.006, flange_thickness=0.004),
        hub=WheelHub(
            radius=0.028,
            width=0.040,
            cap_style="domed",
            bolt_pattern=BoltPattern(count=5, circle_diameter=0.034, hole_diameter=0.004),
        ),
        face=WheelFace(dish_depth=0.006, front_inset=0.002, rear_inset=0.002),
        spokes=WheelSpokes(style="split_y", count=5, thickness=0.004, window_radius=0.010),
        bore=WheelBore(style="round", diameter=0.014),
    )
    tire_mesh = mesh_from_geometry(utility_tire, "utility_tire")
    rim_mesh = mesh_from_geometry(utility_rim, "utility_rim")

    frame = model.part("frame")
    # Rectangular lower frame: two long side tubes with cross tubes and dropouts.
    for y, suffix in ((0.19, "0"), (-0.19, "1")):
        _cylinder_between(frame, f"side_rail_{suffix}", (-0.55, y, 0.245), (0.43, y, 0.245), 0.018, frame_blue)
        _cylinder_between(frame, f"rear_dropout_{suffix}", (-0.46, y, 0.245), (-0.46, y * 1.16, 0.118), 0.013, satin_metal)
        _cylinder_between(frame, f"front_downbrace_{suffix}", (0.36, y, 0.245), (0.47, y * 0.21, 0.285), 0.014, frame_blue)

    for x, suffix in ((-0.50, "rear"), (-0.10, "mid"), (0.34, "front")):
        _cylinder_between(frame, f"{suffix}_cross_tube", (x, -0.19, 0.245), (x, 0.19, 0.245), 0.016, frame_blue)
    for x, suffix in ((-0.32, "rear_pad"), (0.05, "front_pad")):
        _cylinder_between(frame, f"{suffix}_cross_tube", (x, -0.19, 0.245), (x, 0.19, 0.245), 0.014, frame_blue)

    # Rear axle stubs stop just shy of the wheel side faces, leaving the wheels free to spin.
    _cylinder_between(frame, "rear_axle_stub_0", (-0.46, 0.218, 0.115), (-0.46, 0.262, 0.115), 0.012, satin_metal)
    _cylinder_between(frame, "rear_axle_stub_1", (-0.46, -0.218, 0.115), (-0.46, -0.262, 0.115), 0.012, satin_metal)

    # Head socket / bearing sleeve for the yawing steering fork.
    frame.visual(
        Cylinder(radius=0.035, length=0.110),
        origin=Origin(xyz=(0.47, 0.0, 0.275)),
        material=satin_metal,
        name="head_socket",
    )
    _cylinder_between(frame, "head_tube_left_gusset", (0.43, 0.19, 0.245), (0.47, 0.041, 0.315), 0.012, frame_blue)
    _cylinder_between(frame, "head_tube_right_gusset", (0.43, -0.19, 0.245), (0.47, -0.041, 0.315), 0.012, frame_blue)

    # Knee pad support posts and a broad cushioned rectangular knee rest.
    for x, xs in ((-0.32, "rear"), (0.05, "front")):
        for y, ys in ((0.115, "0"), (-0.115, "1")):
            _cylinder_between(frame, f"{xs}_pad_post_{ys}", (x, y, 0.245), (x, y, 0.515), 0.014, satin_metal)
    frame.visual(
        Box((0.50, 0.30, 0.030)),
        origin=Origin(xyz=(-0.135, 0.0, 0.522)),
        material=dark_metal,
        name="pad_support_plate",
    )
    frame.visual(
        Box((0.52, 0.32, 0.070)),
        origin=Origin(xyz=(-0.135, 0.0, 0.572)),
        material=vinyl,
        name="knee_pad",
    )

    # Three-knuckle side-stand hinge under one side rail: two frame leaves, center knuckle on the stand.
    for x, suffix in ((0.045, "rear"), (0.115, "front")):
        frame.visual(
            Cylinder(radius=0.017, length=0.026),
            origin=Origin(xyz=(x, -0.230, 0.205), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=satin_metal,
            name=f"stand_hinge_leaf_{suffix}",
        )
    frame.visual(
        Box((0.090, 0.024, 0.045)),
        origin=Origin(xyz=(0.080, -0.205, 0.218)),
        material=satin_metal,
        name="stand_hinge_bracket",
    )

    steering_fork = model.part("steering_fork")
    # The child frame origin is the vertical steering axis inside the head socket.
    steering_fork.visual(
        Cylinder(radius=0.018, length=0.790),
        origin=Origin(xyz=(0.0, 0.0, 0.340)),
        material=satin_metal,
        name="steerer_tube",
    )
    steering_fork.visual(
        Cylinder(radius=0.032, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, -0.080)),
        material=dark_metal,
        name="lower_bearing_collar",
    )
    _cylinder_between(steering_fork, "handlebar", (0.0, -0.300, 0.720), (0.0, 0.300, 0.720), 0.017, satin_metal)
    _cylinder_between(steering_fork, "grip_0", (0.0, 0.300, 0.720), (0.0, 0.390, 0.720), 0.020, rubber)
    _cylinder_between(steering_fork, "grip_1", (0.0, -0.300, 0.720), (0.0, -0.390, 0.720), 0.020, rubber)
    _cylinder_between(steering_fork, "crown_stem_bridge", (0.0, 0.0, -0.082), (0.090, 0.0, -0.082), 0.018, frame_blue)
    _cylinder_between(steering_fork, "fork_crown", (0.090, -0.145, -0.055), (0.090, 0.145, -0.055), 0.018, frame_blue)
    for y, suffix in ((0.135, "0"), (-0.135, "1")):
        _cylinder_between(steering_fork, f"fork_leg_{suffix}", (0.090, y, -0.055), (0.115, y, -0.160), 0.016, frame_blue)
        steering_fork.visual(
            Box((0.040, 0.020, 0.042)),
            origin=Origin(xyz=(0.115, y, -0.160)),
            material=frame_blue,
            name=f"axle_block_{suffix}",
        )
    steering_fork.visual(
        Cylinder(radius=0.011, length=0.057),
        origin=Origin(xyz=(0.115, 0.1635, -0.160), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=satin_metal,
        name="fork_axle_stub_0",
    )
    steering_fork.visual(
        Cylinder(radius=0.011, length=0.057),
        origin=Origin(xyz=(0.115, -0.1635, -0.160), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_metal,
        name="fork_axle_stub_1",
    )

    front_wheel_0 = model.part("front_wheel_0")
    front_wheel_1 = model.part("front_wheel_1")
    rear_wheel_0 = model.part("rear_wheel_0")
    rear_wheel_1 = model.part("rear_wheel_1")
    for wheel in (front_wheel_0, front_wheel_1, rear_wheel_0, rear_wheel_1):
        _add_wheel_visuals(
            wheel,
            radius=0.115,
            width=0.055,
            tire_mesh=tire_mesh,
            wheel_mesh=rim_mesh,
            rubber=rubber,
            rim=satin_metal,
        )

    side_stand = model.part("side_stand")
    side_stand.visual(
        Cylinder(radius=0.015, length=0.032),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_metal,
        name="hinge_knuckle",
    )
    stand_foot = (0.020, -0.125, -0.200)
    _cylinder_between(side_stand, "stand_leg", (0.0, 0.0, 0.0), stand_foot, 0.011, warning)
    side_stand.visual(
        Box((0.120, 0.048, 0.012)),
        origin=Origin(xyz=stand_foot, rpy=(0.0, 0.0, 0.12)),
        material=dark_metal,
        name="stand_foot",
    )

    model.articulation(
        "frame_to_fork",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=steering_fork,
        origin=Origin(xyz=(0.47, 0.0, 0.275)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=25.0, velocity=2.5, lower=-0.75, upper=0.75),
    )
    model.articulation(
        "fork_to_front_wheel_0",
        ArticulationType.CONTINUOUS,
        parent=steering_fork,
        child=front_wheel_0,
        origin=Origin(xyz=(0.115, 0.205, -0.160)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=18.0),
    )
    model.articulation(
        "fork_to_front_wheel_1",
        ArticulationType.CONTINUOUS,
        parent=steering_fork,
        child=front_wheel_1,
        origin=Origin(xyz=(0.115, -0.205, -0.160)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=18.0),
    )
    model.articulation(
        "frame_to_rear_wheel_0",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=rear_wheel_0,
        origin=Origin(xyz=(-0.46, 0.290, 0.115)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=18.0),
    )
    model.articulation(
        "frame_to_rear_wheel_1",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=rear_wheel_1,
        origin=Origin(xyz=(-0.46, -0.290, 0.115)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=18.0),
    )
    model.articulation(
        "frame_to_side_stand",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=side_stand,
        origin=Origin(xyz=(0.080, -0.230, 0.205)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=-1.25, upper=0.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    steering_fork = object_model.get_part("steering_fork")
    front_wheel_0 = object_model.get_part("front_wheel_0")
    front_wheel_1 = object_model.get_part("front_wheel_1")
    rear_wheel_0 = object_model.get_part("rear_wheel_0")
    side_stand = object_model.get_part("side_stand")

    yaw = object_model.get_articulation("frame_to_fork")
    stand_hinge = object_model.get_articulation("frame_to_side_stand")

    ctx.allow_overlap(
        frame,
        steering_fork,
        elem_a="head_socket",
        elem_b="steerer_tube",
        reason="The vertical steerer tube is intentionally captured inside the simplified solid head bearing sleeve.",
    )
    ctx.allow_overlap(
        front_wheel_0,
        steering_fork,
        elem_a="rim",
        elem_b="fork_axle_stub_0",
        reason="The fork axle stub is intentionally seated a few millimeters into the wheel rim bore.",
    )
    ctx.allow_overlap(
        front_wheel_0,
        steering_fork,
        elem_a="hub_cap",
        elem_b="fork_axle_stub_0",
        reason="The front axle passes through the wheel's central hub cap/bearing region.",
    )
    ctx.allow_overlap(
        front_wheel_1,
        steering_fork,
        elem_a="rim",
        elem_b="fork_axle_stub_1",
        reason="The fork axle stub is intentionally seated a few millimeters into the wheel rim bore.",
    )
    ctx.allow_overlap(
        front_wheel_1,
        steering_fork,
        elem_a="hub_cap",
        elem_b="fork_axle_stub_1",
        reason="The front axle passes through the wheel's central hub cap/bearing region.",
    )
    ctx.expect_within(
        steering_fork,
        frame,
        axes="xy",
        inner_elem="steerer_tube",
        outer_elem="head_socket",
        margin=0.002,
        name="steerer centered in head socket",
    )
    ctx.expect_overlap(
        steering_fork,
        frame,
        axes="z",
        elem_a="steerer_tube",
        elem_b="head_socket",
        min_overlap=0.080,
        name="steerer retained through head socket",
    )
    ctx.expect_overlap(
        front_wheel_0,
        steering_fork,
        axes="y",
        elem_a="rim",
        elem_b="fork_axle_stub_0",
        min_overlap=0.002,
        name="front wheel 0 axle stub seats in rim",
    )
    ctx.expect_overlap(
        front_wheel_1,
        steering_fork,
        axes="y",
        elem_a="rim",
        elem_b="fork_axle_stub_1",
        min_overlap=0.002,
        name="front wheel 1 axle stub seats in rim",
    )
    ctx.expect_overlap(
        front_wheel_0,
        steering_fork,
        axes="y",
        elem_a="hub_cap",
        elem_b="fork_axle_stub_0",
        min_overlap=0.004,
        name="front wheel 0 axle passes through hub",
    )
    ctx.expect_overlap(
        front_wheel_1,
        steering_fork,
        axes="y",
        elem_a="hub_cap",
        elem_b="fork_axle_stub_1",
        min_overlap=0.004,
        name="front wheel 1 axle passes through hub",
    )

    ctx.expect_origin_gap(
        front_wheel_0,
        front_wheel_1,
        axis="y",
        min_gap=0.35,
        max_gap=0.43,
        name="front wheels are paired across the fork",
    )
    ctx.expect_origin_gap(
        front_wheel_0,
        rear_wheel_0,
        axis="x",
        min_gap=0.48,
        max_gap=1.12,
        name="front axle is ahead of rear axle",
    )

    rest_front_pos = ctx.part_world_position(front_wheel_0)
    with ctx.pose({yaw: 0.55}):
        turned_front_pos = ctx.part_world_position(front_wheel_0)
    ctx.check(
        "steering fork yaws front axle",
        rest_front_pos is not None
        and turned_front_pos is not None
        and turned_front_pos[1] > rest_front_pos[1] + 0.020,
        details=f"rest={rest_front_pos}, turned={turned_front_pos}",
    )

    deployed_foot = ctx.part_element_world_aabb(side_stand, elem="stand_foot")
    with ctx.pose({stand_hinge: -1.10}):
        stowed_foot = ctx.part_element_world_aabb(side_stand, elem="stand_foot")
    ctx.check(
        "parking stand folds up under frame",
        deployed_foot is not None
        and stowed_foot is not None
        and stowed_foot[0][2] > deployed_foot[0][2] + 0.10,
        details=f"deployed={deployed_foot}, stowed={stowed_foot}",
    )

    return ctx.report()


object_model = build_object_model()
