from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def _leg_plus_z(yaw: float, spread: float) -> tuple[float, float, float]:
    return (
        -math.cos(yaw) * math.sin(spread),
        -math.sin(yaw) * math.sin(spread),
        math.cos(spread),
    )


def _add_box_tube(
    part,
    *,
    outer_width: float,
    outer_depth: float,
    wall: float,
    length: float,
    material,
    base_name: str,
) -> None:
    inner_width = outer_width - 2.0 * wall
    center_z = -length / 2.0

    part.visual(
        Box((wall, outer_depth, length)),
        origin=Origin(xyz=(-outer_width / 2.0 + wall / 2.0, 0.0, center_z)),
        material=material,
        name=f"{base_name}_left",
    )
    part.visual(
        Box((wall, outer_depth, length)),
        origin=Origin(xyz=(outer_width / 2.0 - wall / 2.0, 0.0, center_z)),
        material=material,
        name=f"{base_name}_right",
    )
    part.visual(
        Box((inner_width, wall, length)),
        origin=Origin(xyz=(0.0, -outer_depth / 2.0 + wall / 2.0, center_z)),
        material=material,
        name=f"{base_name}_back",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="camera_tripod_support")

    black_anodized = model.material("black_anodized", rgba=(0.15, 0.16, 0.18, 1.0))
    graphite = model.material("graphite", rgba=(0.26, 0.28, 0.31, 1.0))
    matte_gray = model.material("matte_gray", rgba=(0.48, 0.50, 0.53, 1.0))
    body_black = model.material("body_black", rgba=(0.09, 0.09, 0.10, 1.0))
    rubber = model.material("rubber", rgba=(0.06, 0.06, 0.07, 1.0))

    tripod_body = model.part("tripod_body")
    tripod_body.visual(
        Cylinder(radius=0.078, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.790)),
        material=black_anodized,
        name="spider",
    )
    tripod_body.visual(
        Cylinder(radius=0.058, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 0.848)),
        material=graphite,
        name="shoulder",
    )
    tripod_body.visual(
        Cylinder(radius=0.028, length=0.250),
        origin=Origin(xyz=(0.0, 0.0, 0.970)),
        material=black_anodized,
        name="center_column",
    )
    tripod_body.visual(
        Cylinder(radius=0.045, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 1.107)),
        material=graphite,
        name="top_plate",
    )

    spread = math.radians(22.0)
    socket_radius = 0.092
    socket_height = 0.780
    leg_angles = (0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)

    outer_width = 0.036
    outer_depth = 0.024
    outer_wall = 0.003
    outer_length = 0.360

    middle_width = 0.026
    middle_depth = 0.016
    middle_wall = 0.0025
    middle_length = 0.590

    inner_width = 0.018
    inner_depth = 0.010
    inner_length = 0.800

    for index, yaw in enumerate(leg_angles):
        socket_xyz = (
            socket_radius * math.cos(yaw),
            socket_radius * math.sin(yaw),
            socket_height,
        )
        outer = model.part(f"leg_{index}_outer")
        _add_box_tube(
            outer,
            outer_width=outer_width,
            outer_depth=outer_depth,
            wall=outer_wall,
            length=outer_length,
            material=graphite,
            base_name="tube",
        )

        middle = model.part(f"leg_{index}_middle")
        _add_box_tube(
            middle,
            outer_width=middle_width,
            outer_depth=middle_depth,
            wall=middle_wall,
            length=middle_length,
            material=graphite,
            base_name="tube",
        )
        middle.visual(
            Box((middle_width - 0.002, 0.006, 0.012)),
            origin=Origin(xyz=(0.006, -(middle_depth / 2.0 + 0.003), 0.006)),
            material=matte_gray,
            name="stop",
        )

        inner = model.part(f"leg_{index}_inner")
        inner.visual(
            Box((inner_width, inner_depth, inner_length)),
            origin=Origin(xyz=(0.0, 0.0, -inner_length / 2.0)),
            material=graphite,
            name="tube",
        )
        inner.visual(
            Box((inner_width - 0.004, 0.004, 0.016)),
            origin=Origin(xyz=(0.003, -(inner_depth / 2.0 + 0.0015), 0.004)),
            material=matte_gray,
            name="stop",
        )
        inner.visual(
            Cylinder(radius=0.012, length=0.022),
            origin=Origin(xyz=(0.0, 0.0, -(inner_length + 0.011))),
            material=rubber,
            name="foot",
        )

        model.articulation(
            f"tripod_body_to_leg_{index}_outer",
            ArticulationType.FIXED,
            parent=tripod_body,
            child=outer,
            origin=Origin(xyz=socket_xyz, rpy=(0.0, -spread, yaw)),
        )
        model.articulation(
            f"leg_{index}_outer_to_middle",
            ArticulationType.PRISMATIC,
            parent=outer,
            child=middle,
            origin=Origin(),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=120.0,
                velocity=0.18,
                lower=0.0,
                upper=0.120,
            ),
        )
        model.articulation(
            f"leg_{index}_middle_to_inner",
            ArticulationType.PRISMATIC,
            parent=middle,
            child=inner,
            origin=Origin(),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=100.0,
                velocity=0.18,
                lower=0.0,
                upper=0.120,
            ),
        )

    pan_head = model.part("pan_head")
    pan_head.visual(
        Cylinder(radius=0.050, length=0.036),
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material=black_anodized,
        name="pan_base",
    )
    pan_head.visual(
        Cylinder(radius=0.038, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.050)),
        material=graphite,
        name="pan_collar",
    )
    pan_head.visual(
        Box((0.032, 0.012, 0.112)),
        origin=Origin(xyz=(0.0, 0.046, 0.056)),
        material=graphite,
        name="yoke_0",
    )
    pan_head.visual(
        Box((0.032, 0.012, 0.112)),
        origin=Origin(xyz=(0.0, -0.046, 0.056)),
        material=graphite,
        name="yoke_1",
    )
    pan_head.visual(
        Box((0.018, 0.082, 0.016)),
        origin=Origin(xyz=(-0.020, 0.0, 0.101)),
        material=graphite,
        name="bridge",
    )

    model.articulation(
        "tripod_body_to_pan_head",
        ArticulationType.CONTINUOUS,
        parent=tripod_body,
        child=pan_head,
        origin=Origin(xyz=(0.0, 0.0, 1.119)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=6.0),
    )

    tilt_cradle = model.part("tilt_cradle")
    tilt_cradle.visual(
        Cylinder(radius=0.011, length=0.080),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=matte_gray,
        name="pivot_barrel",
    )
    tilt_cradle.visual(
        Box((0.100, 0.032, 0.024)),
        origin=Origin(xyz=(0.050, 0.0, 0.0)),
        material=graphite,
        name="support_arm",
    )
    tilt_cradle.visual(
        Box((0.044, 0.050, 0.036)),
        origin=Origin(xyz=(0.090, 0.0, 0.018)),
        material=graphite,
        name="pedestal",
    )
    tilt_cradle.visual(
        Box((0.092, 0.042, 0.012)),
        origin=Origin(xyz=(0.100, 0.0, 0.042)),
        material=matte_gray,
        name="deck",
    )

    model.articulation(
        "pan_head_to_tilt_cradle",
        ArticulationType.REVOLUTE,
        parent=pan_head,
        child=tilt_cradle,
        origin=Origin(xyz=(0.0, 0.0, 0.078)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=14.0,
            velocity=1.4,
            lower=-0.35,
            upper=0.90,
        ),
    )

    plate = model.part("plate")
    plate.visual(
        Box((0.100, 0.055, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=matte_gray,
        name="plate_body",
    )
    plate.visual(
        Box((0.014, 0.016, 0.018)),
        origin=Origin(xyz=(0.000, 0.035, 0.009)),
        material=black_anodized,
        name="lever_block",
    )

    model.articulation(
        "tilt_cradle_to_plate",
        ArticulationType.FIXED,
        parent=tilt_cradle,
        child=plate,
        origin=Origin(xyz=(0.100, 0.0, 0.048)),
    )

    plate_lever = model.part("plate_lever")
    plate_lever.visual(
        Cylinder(radius=0.0045, length=0.016),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black_anodized,
        name="pivot",
    )
    plate_lever.visual(
        Box((0.040, 0.014, 0.010)),
        origin=Origin(xyz=(0.020, 0.006, 0.000)),
        material=matte_gray,
        name="handle",
    )
    plate_lever.visual(
        Cylinder(radius=0.006, length=0.012),
        origin=Origin(xyz=(0.043, 0.018, -0.001), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=matte_gray,
        name="grip",
    )

    model.articulation(
        "plate_to_plate_lever",
        ArticulationType.REVOLUTE,
        parent=plate,
        child=plate_lever,
        origin=Origin(xyz=(0.015, 0.035, 0.014)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=2.0,
            lower=0.0,
            upper=1.05,
        ),
    )

    camera = model.part("camera")
    camera.visual(
        Box((0.040, 0.030, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=body_black,
        name="mount",
    )
    camera.visual(
        Box((0.130, 0.060, 0.080)),
        origin=Origin(xyz=(0.010, 0.0, 0.050)),
        material=body_black,
        name="body",
    )
    camera.visual(
        Box((0.042, 0.048, 0.018)),
        origin=Origin(xyz=(-0.022, 0.0, 0.099)),
        material=body_black,
        name="viewfinder",
    )
    camera.visual(
        Cylinder(radius=0.028, length=0.092),
        origin=Origin(xyz=(0.102, 0.0, 0.050), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=graphite,
        name="lens_barrel",
    )
    camera.visual(
        Cylinder(radius=0.032, length=0.020),
        origin=Origin(xyz=(0.156, 0.0, 0.050), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=graphite,
        name="lens_hood",
    )

    model.articulation(
        "plate_to_camera",
        ArticulationType.FIXED,
        parent=plate,
        child=camera,
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    for index in range(3):
        ctx.allow_overlap(
            "tripod_body",
            f"leg_{index}_outer",
            reason="Each outer leg root is intentionally represented as seated into the cast tripod spider socket.",
        )
        ctx.allow_overlap(
            f"leg_{index}_outer",
            f"leg_{index}_middle",
            elem_a="tube_back",
            elem_b="stop",
            reason="The middle stage is represented with a retained shoulder stop resting against the outer sleeve lip.",
        )
        ctx.allow_overlap(
            f"leg_{index}_middle",
            f"leg_{index}_inner",
            reason="The inner stage uses a simplified rear stop block retained against the middle sleeve proxy.",
        )

    camera = object_model.get_part("camera")
    plate = object_model.get_part("plate")
    leg_inner = object_model.get_part("leg_0_inner")
    plate_lever = object_model.get_part("plate_lever")

    pan_head = object_model.get_articulation("tripod_body_to_pan_head")
    tilt_cradle = object_model.get_articulation("pan_head_to_tilt_cradle")
    leg_0_outer_to_middle = object_model.get_articulation("leg_0_outer_to_middle")
    leg_0_middle_to_inner = object_model.get_articulation("leg_0_middle_to_inner")
    plate_lever_joint = object_model.get_articulation("plate_to_plate_lever")

    ctx.expect_gap(
        camera,
        plate,
        axis="z",
        positive_elem="mount",
        negative_elem="plate_body",
        max_gap=0.001,
        max_penetration=0.0,
        name="camera mount sits flush on the quick-release plate",
    )

    pan_limits = pan_head.motion_limits
    ctx.check(
        "pan head uses continuous rotation",
        pan_limits is not None and pan_limits.lower is None and pan_limits.upper is None,
        details=f"motion_limits={pan_limits}",
    )

    rest_camera_pos = ctx.part_world_position(camera)
    with ctx.pose({pan_head: math.pi / 2.0}):
        panned_camera_pos = ctx.part_world_position(camera)
    ctx.check(
        "pan motion swings the camera around the vertical axis",
        rest_camera_pos is not None
        and panned_camera_pos is not None
        and rest_camera_pos[0] > 0.08
        and panned_camera_pos[1] > 0.08
        and abs(panned_camera_pos[0]) < abs(rest_camera_pos[0]) * 0.5,
        details=f"rest={rest_camera_pos}, panned={panned_camera_pos}",
    )

    with ctx.pose({tilt_cradle: 0.70}):
        tilted_camera_pos = ctx.part_world_position(camera)
    ctx.check(
        "tilt motion raises the camera body",
        rest_camera_pos is not None
        and tilted_camera_pos is not None
        and tilted_camera_pos[2] > rest_camera_pos[2] + 0.05,
        details=f"rest={rest_camera_pos}, tilted={tilted_camera_pos}",
    )

    rest_leg_pos = ctx.part_world_position(leg_inner)
    with ctx.pose({leg_0_outer_to_middle: 0.10, leg_0_middle_to_inner: 0.10}):
        extended_leg_pos = ctx.part_world_position(leg_inner)
    ctx.check(
        "representative telescoping leg extends down and outward",
        rest_leg_pos is not None
        and extended_leg_pos is not None
        and extended_leg_pos[2] < rest_leg_pos[2] - 0.12
        and math.hypot(extended_leg_pos[0], extended_leg_pos[1])
        > math.hypot(rest_leg_pos[0], rest_leg_pos[1]) + 0.03,
        details=f"rest={rest_leg_pos}, extended={extended_leg_pos}",
    )

    closed_handle_aabb = ctx.part_element_world_aabb(plate_lever, elem="handle")
    with ctx.pose({plate_lever_joint: 0.90}):
        open_handle_aabb = ctx.part_element_world_aabb(plate_lever, elem="handle")
    ctx.check(
        "locking lever rotates up from the plate edge",
        closed_handle_aabb is not None
        and open_handle_aabb is not None
        and open_handle_aabb[1][2] > closed_handle_aabb[1][2] + 0.007,
        details=f"closed={closed_handle_aabb}, open={open_handle_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
