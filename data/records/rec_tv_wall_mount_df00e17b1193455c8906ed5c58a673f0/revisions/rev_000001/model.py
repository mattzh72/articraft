from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


BLACK_STEEL = Material("matte black powder coated steel", rgba=(0.015, 0.014, 0.013, 1.0))
DARK_STEEL = Material("dark satin stamped steel", rgba=(0.055, 0.055, 0.052, 1.0))
RUBBER = Material("black rubber bumpers", rgba=(0.006, 0.006, 0.006, 1.0))
ZINC = Material("zinc plated bolt heads", rgba=(0.62, 0.60, 0.55, 1.0))


def _box(part, name: str, size, xyz, material=BLACK_STEEL) -> None:
    part.visual(Box(size), origin=Origin(xyz=xyz), material=material, name=name)


def _cylinder_z(part, name: str, radius: float, length: float, xyz, material=BLACK_STEEL) -> None:
    part.visual(Cylinder(radius=radius, length=length), origin=Origin(xyz=xyz), material=material, name=name)


def _cylinder_x(part, name: str, radius: float, length: float, xyz, material=BLACK_STEEL) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(0.0, math.pi / 2.0, 0.0)),
        material=material,
        name=name,
    )


def _cylinder_y(part, name: str, radius: float, length: float, xyz, material=BLACK_STEEL) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="long_reach_tv_wall_mount")

    model.materials.extend([BLACK_STEEL, DARK_STEEL, RUBBER, ZINC])

    wall_plate = model.part("wall_plate")
    _box(wall_plate, "back_plate", (0.035, 0.30, 0.48), (0.0, 0.0, 0.0), DARK_STEEL)
    _box(wall_plate, "raised_spine", (0.060, 0.095, 0.45), (0.030, 0.0, 0.0), BLACK_STEEL)
    _box(wall_plate, "upper_wall_clevis", (0.170, 0.150, 0.026), (0.120, 0.0, 0.112), BLACK_STEEL)
    _box(wall_plate, "lower_wall_clevis", (0.170, 0.150, 0.026), (0.120, 0.0, -0.112), BLACK_STEEL)
    _box(wall_plate, "clevis_back_web", (0.050, 0.110, 0.250), (0.030, 0.0, 0.0), BLACK_STEEL)
    for iy, y in enumerate((-0.095, 0.095)):
        for iz, z in enumerate((-0.180, 0.180)):
            _cylinder_x(wall_plate, f"wall_lag_{iy}_{iz}", 0.015, 0.012, (0.018, y, z), ZINC)
    _cylinder_z(wall_plate, "wall_pivot_pin_top", 0.026, 0.028, (0.120, 0.0, 0.112), ZINC)
    _cylinder_z(wall_plate, "wall_pivot_pin_bottom", 0.026, 0.028, (0.120, 0.0, -0.112), ZINC)
    _cylinder_z(wall_plate, "wall_pivot_pin", 0.018, 0.250, (0.120, 0.0, 0.0), ZINC)

    first_arm_len = 0.78
    first_arm = model.part("first_arm")
    _cylinder_z(first_arm, "wall_bearing_sleeve", 0.049, 0.136, (0.0, 0.0, 0.0), BLACK_STEEL)
    _box(first_arm, "proximal_hub_web_0", (0.090, 0.012, 0.120), (0.035, -0.036, 0.0), BLACK_STEEL)
    _box(first_arm, "proximal_hub_web_1", (0.090, 0.012, 0.120), (0.035, 0.036, 0.0), BLACK_STEEL)
    _box(first_arm, "long_rectangular_tube", (0.650, 0.060, 0.055), (0.350, 0.0, 0.0), BLACK_STEEL)
    _box(first_arm, "upper_stiffener", (0.500, 0.067, 0.012), (0.350, 0.0, 0.031), DARK_STEEL)
    _box(first_arm, "lower_stiffener", (0.500, 0.067, 0.012), (0.350, 0.0, -0.031), DARK_STEEL)
    _box(first_arm, "distal_back_web", (0.070, 0.125, 0.205), (first_arm_len - 0.080, 0.0, 0.0), BLACK_STEEL)
    _box(first_arm, "distal_upper_fork", (0.130, 0.135, 0.026), (first_arm_len, 0.0, 0.096), BLACK_STEEL)
    _box(first_arm, "distal_lower_fork", (0.130, 0.135, 0.026), (first_arm_len, 0.0, -0.096), BLACK_STEEL)
    _box(first_arm, "distal_side_cheek_0", (0.105, 0.014, 0.175), (first_arm_len, -0.060, 0.0), BLACK_STEEL)
    _box(first_arm, "distal_side_cheek_1", (0.105, 0.014, 0.175), (first_arm_len, 0.060, 0.0), BLACK_STEEL)
    _cylinder_z(first_arm, "elbow_pivot_pin", 0.016, 0.220, (first_arm_len, 0.0, 0.0), ZINC)
    _cylinder_z(first_arm, "distal_pin_cap_top", 0.024, 0.018, (first_arm_len, 0.0, 0.112), ZINC)
    _cylinder_z(first_arm, "distal_pin_cap_bottom", 0.024, 0.018, (first_arm_len, 0.0, -0.112), ZINC)

    second_arm_len = 0.46
    second_arm = model.part("second_arm")
    _cylinder_z(second_arm, "first_bearing_sleeve", 0.043, 0.126, (0.0, 0.0, 0.0), BLACK_STEEL)
    _box(second_arm, "first_hub_web_0", (0.078, 0.012, 0.106), (0.032, -0.034, 0.0), BLACK_STEEL)
    _box(second_arm, "first_hub_web_1", (0.078, 0.012, 0.106), (0.032, 0.034, 0.0), BLACK_STEEL)
    _box(second_arm, "short_rectangular_tube", (0.400, 0.056, 0.052), (0.230, 0.0, 0.0), BLACK_STEEL)
    _box(second_arm, "short_top_stiffener", (0.300, 0.063, 0.011), (0.230, 0.0, 0.034), DARK_STEEL)
    _box(second_arm, "short_bottom_stiffener", (0.300, 0.063, 0.011), (0.230, 0.0, -0.034), DARK_STEEL)
    _box(second_arm, "head_back_web", (0.060, 0.112, 0.185), (second_arm_len - 0.060, 0.0, 0.0), BLACK_STEEL)
    _box(second_arm, "head_upper_fork", (0.115, 0.120, 0.024), (second_arm_len, 0.0, 0.087), BLACK_STEEL)
    _box(second_arm, "head_lower_fork", (0.115, 0.120, 0.024), (second_arm_len, 0.0, -0.087), BLACK_STEEL)
    _box(second_arm, "head_side_cheek_0", (0.095, 0.013, 0.155), (second_arm_len, -0.053, 0.0), BLACK_STEEL)
    _box(second_arm, "head_side_cheek_1", (0.095, 0.013, 0.155), (second_arm_len, 0.053, 0.0), BLACK_STEEL)
    _cylinder_z(second_arm, "swivel_pivot_pin", 0.014, 0.196, (second_arm_len, 0.0, 0.0), ZINC)
    _cylinder_z(second_arm, "head_pin_cap_top", 0.021, 0.016, (second_arm_len, 0.0, 0.101), ZINC)
    _cylinder_z(second_arm, "head_pin_cap_bottom", 0.021, 0.016, (second_arm_len, 0.0, -0.101), ZINC)

    head_frame = model.part("head_frame")
    _cylinder_z(head_frame, "swivel_bearing_sleeve", 0.035, 0.110, (0.0, 0.0, 0.0), BLACK_STEEL)
    _box(head_frame, "swivel_neck_0", (0.170, 0.018, 0.046), (0.085, -0.035, 0.0), BLACK_STEEL)
    _box(head_frame, "swivel_neck_1", (0.170, 0.018, 0.046), (0.085, 0.035, 0.0), BLACK_STEEL)
    _box(head_frame, "outer_top_rail", (0.035, 0.365, 0.030), (0.180, 0.0, 0.175), BLACK_STEEL)
    _box(head_frame, "outer_bottom_rail", (0.035, 0.365, 0.030), (0.180, 0.0, -0.175), BLACK_STEEL)
    _box(head_frame, "outer_side_rail_0", (0.035, 0.030, 0.365), (0.180, -0.175, 0.0), BLACK_STEEL)
    _box(head_frame, "outer_side_rail_1", (0.035, 0.030, 0.365), (0.180, 0.175, 0.0), BLACK_STEEL)
    _box(head_frame, "center_bridge_0", (0.035, 0.160, 0.022), (0.180, -0.082, 0.0), BLACK_STEEL)
    _box(head_frame, "center_bridge_1", (0.035, 0.160, 0.022), (0.180, 0.082, 0.0), BLACK_STEEL)
    _box(head_frame, "tilt_bearing_0", (0.050, 0.030, 0.070), (0.180, -0.138, 0.0), DARK_STEEL)
    _box(head_frame, "tilt_bearing_1", (0.050, 0.030, 0.070), (0.180, 0.138, 0.0), DARK_STEEL)

    tilt_cradle = model.part("tilt_cradle")
    _cylinder_y(tilt_cradle, "tilt_axle", 0.016, 0.230, (0.0, 0.0, 0.0), ZINC)
    _box(tilt_cradle, "axle_to_plate_web", (0.052, 0.050, 0.040), (0.042, 0.0, 0.0), BLACK_STEEL)
    _box(tilt_cradle, "vesa_mount_plate", (0.012, 0.205, 0.205), (0.050, 0.0, 0.0), DARK_STEEL)
    _box(tilt_cradle, "inner_top_rail", (0.026, 0.245, 0.026), (0.048, 0.0, 0.120), BLACK_STEEL)
    _box(tilt_cradle, "inner_bottom_rail", (0.026, 0.245, 0.026), (0.048, 0.0, -0.120), BLACK_STEEL)
    _box(tilt_cradle, "inner_side_rail_0", (0.026, 0.026, 0.245), (0.048, -0.120, 0.0), BLACK_STEEL)
    _box(tilt_cradle, "inner_side_rail_1", (0.026, 0.026, 0.245), (0.048, 0.120, 0.0), BLACK_STEEL)
    _box(tilt_cradle, "central_crossbar", (0.024, 0.245, 0.024), (0.048, 0.0, 0.0), DARK_STEEL)
    _box(tilt_cradle, "vertical_crossbar", (0.024, 0.024, 0.245), (0.048, 0.0, 0.0), DARK_STEEL)
    for iy, y in enumerate((-0.080, 0.080)):
        for iz, z in enumerate((-0.080, 0.080)):
            _cylinder_x(tilt_cradle, f"vesa_bolt_{iy}_{iz}", 0.010, 0.014, (0.064, y, z), ZINC)
            _box(tilt_cradle, f"rubber_pad_{iy}_{iz}", (0.008, 0.034, 0.034), (0.058, y, z), RUBBER)

    model.articulation(
        "wall_swing",
        ArticulationType.REVOLUTE,
        parent=wall_plate,
        child=first_arm,
        origin=Origin(xyz=(0.120, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.2, lower=-1.35, upper=1.35),
    )
    model.articulation(
        "elbow_swing",
        ArticulationType.REVOLUTE,
        parent=first_arm,
        child=second_arm,
        origin=Origin(xyz=(first_arm_len, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=100.0, velocity=1.2, lower=-1.70, upper=1.70),
    )
    model.articulation(
        "head_swivel",
        ArticulationType.REVOLUTE,
        parent=second_arm,
        child=head_frame,
        origin=Origin(xyz=(second_arm_len, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=45.0, velocity=1.4, lower=-1.05, upper=1.05),
    )
    model.articulation(
        "head_tilt",
        ArticulationType.REVOLUTE,
        parent=head_frame,
        child=tilt_cradle,
        origin=Origin(xyz=(0.180, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=0.8, lower=-0.35, upper=0.28),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    wall_swing = object_model.get_articulation("wall_swing")
    elbow_swing = object_model.get_articulation("elbow_swing")
    head_swivel = object_model.get_articulation("head_swivel")
    head_tilt = object_model.get_articulation("head_tilt")

    ctx.check(
        "three vertical revolute joints",
        all(
            joint.articulation_type == ArticulationType.REVOLUTE and joint.axis == (0.0, 0.0, 1.0)
            for joint in (wall_swing, elbow_swing, head_swivel)
        ),
        details="Wall swing, elbow swing, and head swivel must rotate about vertical axes.",
    )
    ctx.check(
        "separate horizontal tilt joint",
        head_tilt.articulation_type == ArticulationType.REVOLUTE and head_tilt.axis == (0.0, 1.0, 0.0),
        details=f"type={head_tilt.articulation_type}, axis={head_tilt.axis}",
    )

    first_arm = object_model.get_part("first_arm")
    second_arm = object_model.get_part("second_arm")
    head_frame = object_model.get_part("head_frame")
    tilt_cradle = object_model.get_part("tilt_cradle")

    ctx.allow_overlap(
        "wall_plate",
        "first_arm",
        elem_a="wall_pivot_pin",
        elem_b="wall_bearing_sleeve",
        reason="The wall-side vertical hinge pin is intentionally captured inside the first-arm bearing sleeve proxy.",
    )
    ctx.allow_overlap(
        "first_arm",
        "second_arm",
        elem_a="elbow_pivot_pin",
        elem_b="first_bearing_sleeve",
        reason="The elbow hinge pin is intentionally captured inside the second-arm bearing sleeve proxy.",
    )
    ctx.allow_overlap(
        "second_arm",
        "head_frame",
        elem_a="swivel_pivot_pin",
        elem_b="swivel_bearing_sleeve",
        reason="The head swivel pin is intentionally captured inside the head-frame bearing sleeve proxy.",
    )
    for bridge_name in ("center_bridge_0", "center_bridge_1", "tilt_bearing_0", "tilt_bearing_1"):
        ctx.allow_overlap(
            "head_frame",
            "tilt_cradle",
            elem_a=bridge_name,
            elem_b="tilt_axle",
            reason="The horizontal tilt axle is represented as a solid shaft captured inside bearing-block proxies.",
        )
    ctx.allow_overlap(
        "head_frame",
        "tilt_cradle",
        elem_a="swivel_neck_0",
        elem_b="tilt_axle",
        reason="The simplified solid head yoke includes a hidden clearance bore where the tilt shaft passes through the neck plate.",
    )
    ctx.allow_overlap(
        "head_frame",
        "tilt_cradle",
        elem_a="swivel_neck_1",
        elem_b="tilt_axle",
        reason="The simplified solid head yoke includes a hidden clearance bore where the tilt shaft passes through the neck plate.",
    )

    ctx.expect_within(
        "wall_plate",
        first_arm,
        axes="xy",
        inner_elem="wall_pivot_pin",
        outer_elem="wall_bearing_sleeve",
        margin=0.002,
        name="wall pivot pin sits inside first-arm sleeve",
    )
    ctx.expect_overlap(
        "wall_plate",
        first_arm,
        axes="z",
        elem_a="wall_pivot_pin",
        elem_b="wall_bearing_sleeve",
        min_overlap=0.120,
        name="wall hinge pin spans first-arm sleeve",
    )
    ctx.expect_within(
        first_arm,
        second_arm,
        axes="xy",
        inner_elem="elbow_pivot_pin",
        outer_elem="first_bearing_sleeve",
        margin=0.002,
        name="elbow pin sits inside second-arm sleeve",
    )
    ctx.expect_overlap(
        first_arm,
        second_arm,
        axes="z",
        elem_a="elbow_pivot_pin",
        elem_b="first_bearing_sleeve",
        min_overlap=0.110,
        name="elbow hinge pin spans second-arm sleeve",
    )
    ctx.expect_within(
        second_arm,
        head_frame,
        axes="xy",
        inner_elem="swivel_pivot_pin",
        outer_elem="swivel_bearing_sleeve",
        margin=0.002,
        name="head swivel pin sits inside sleeve",
    )
    ctx.expect_overlap(
        second_arm,
        head_frame,
        axes="z",
        elem_a="swivel_pivot_pin",
        elem_b="swivel_bearing_sleeve",
        min_overlap=0.100,
        name="head swivel pin spans sleeve",
    )
    ctx.expect_overlap(
        head_frame,
        tilt_cradle,
        axes="y",
        elem_a="center_bridge_0",
        elem_b="tilt_axle",
        min_overlap=0.060,
        name="tilt axle is captured by frame bearings",
    )
    ctx.expect_overlap(
        head_frame,
        tilt_cradle,
        axes="y",
        elem_a="swivel_neck_0",
        elem_b="tilt_axle",
        min_overlap=0.015,
        name="tilt shaft passes through neck clearance bore",
    )
    ctx.expect_overlap(
        head_frame,
        tilt_cradle,
        axes="y",
        elem_a="swivel_neck_1",
        elem_b="tilt_axle",
        min_overlap=0.015,
        name="tilt shaft passes through opposite neck bore",
    )

    ctx.expect_origin_distance(
        first_arm,
        second_arm,
        axes="xy",
        min_dist=0.77,
        max_dist=0.79,
        name="long first arm spans to elbow pivot",
    )
    ctx.expect_origin_distance(
        second_arm,
        head_frame,
        axes="xy",
        min_dist=0.45,
        max_dist=0.47,
        name="shorter second arm spans to head pivot",
    )
    ctx.expect_within(
        tilt_cradle,
        head_frame,
        axes="yz",
        margin=0.001,
        name="tilt cradle nests inside square head frame",
    )

    closed_head = ctx.part_world_aabb(tilt_cradle)
    with ctx.pose({head_tilt: head_tilt.motion_limits.upper}):
        tilted_head = ctx.part_world_aabb(tilt_cradle)

    ctx.check(
        "tilt changes cradle pitch envelope",
        closed_head is not None
        and tilted_head is not None
        and tilted_head[1][0] > closed_head[1][0] + 0.018
        and tilted_head[0][2] < closed_head[0][2] - 0.008,
        details=f"closed={closed_head}, tilted={tilted_head}",
    )

    return ctx.report()


object_model = build_object_model()
