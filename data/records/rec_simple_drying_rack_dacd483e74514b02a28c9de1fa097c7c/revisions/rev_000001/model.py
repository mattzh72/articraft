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


Vec3 = tuple[float, float, float]


def _segment_origin(start: Vec3, end: Vec3) -> tuple[Origin, float]:
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    dz = end[2] - start[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if length <= 0.0:
        raise ValueError("segment length must be positive")

    yaw = math.atan2(dy, dx)
    pitch = math.acos(max(-1.0, min(1.0, dz / length)))
    center = (
        0.5 * (start[0] + end[0]),
        0.5 * (start[1] + end[1]),
        0.5 * (start[2] + end[2]),
    )
    return Origin(xyz=center, rpy=(0.0, pitch, yaw)), length


def _add_rod(part, start: Vec3, end: Vec3, *, radius: float, material, name: str) -> None:
    origin, length = _segment_origin(start, end)
    part.visual(Cylinder(radius=radius, length=length), origin=origin, material=material, name=name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fold_out_drying_rack")

    metal = model.material("powder_coat", rgba=(0.83, 0.84, 0.86, 1.0))
    joint_metal = model.material("hinge_metal", rgba=(0.70, 0.72, 0.75, 1.0))
    foot_black = model.material("foot_black", rgba=(0.14, 0.14, 0.15, 1.0))

    tube_r = 0.009
    hinge_r = 0.012

    top_z = 0.88
    center_half_x = 0.30
    center_half_y = 0.17
    foot_x = 0.24
    foot_y = 0.24
    floor_z = 0.03

    wing_span = 0.26
    wing_half_y = center_half_y
    wing_hinge_z = 0.845

    lower_hinge_y = -0.12
    lower_hinge_z = 0.628
    lower_half_x = 0.23
    lower_depth = 0.25
    lower_plane_z = -0.03

    central_frame = model.part("central_frame")

    # Main upper drying surface.
    origin, length = _segment_origin((-center_half_x, -center_half_y, top_z), (center_half_x, -center_half_y, top_z))
    central_frame.visual(Cylinder(radius=tube_r, length=length), origin=origin, material=metal, name="rear_rail")
    origin, length = _segment_origin((-center_half_x, center_half_y, top_z), (center_half_x, center_half_y, top_z))
    central_frame.visual(Cylinder(radius=tube_r, length=length), origin=origin, material=metal, name="front_rail")
    origin, length = _segment_origin((-center_half_x, -center_half_y, top_z), (-center_half_x, center_half_y, top_z))
    central_frame.visual(Cylinder(radius=tube_r, length=length), origin=origin, material=metal, name="left_end_rail")
    origin, length = _segment_origin((center_half_x, -center_half_y, top_z), (center_half_x, center_half_y, top_z))
    central_frame.visual(Cylinder(radius=tube_r, length=length), origin=origin, material=metal, name="right_end_rail")
    for index, rod_x in enumerate((-0.20, -0.10, 0.00, 0.10, 0.20)):
        _add_rod(
            central_frame,
            (rod_x, -center_half_y, top_z),
            (rod_x, center_half_y, top_z),
            radius=tube_r * 0.9,
            material=metal,
            name=f"drying_rod_{index}",
        )

    # Freestanding end frames.
    _add_rod(
        central_frame,
        (-center_half_x, center_half_y, top_z),
        (-foot_x, foot_y, floor_z),
        radius=tube_r,
        material=metal,
        name="leg_0_front",
    )
    _add_rod(
        central_frame,
        (-center_half_x, -center_half_y, top_z),
        (-foot_x, -foot_y, floor_z),
        radius=tube_r,
        material=metal,
        name="leg_0_rear",
    )
    _add_rod(
        central_frame,
        (center_half_x, center_half_y, top_z),
        (foot_x, foot_y, floor_z),
        radius=tube_r,
        material=metal,
        name="leg_1_front",
    )
    _add_rod(
        central_frame,
        (center_half_x, -center_half_y, top_z),
        (foot_x, -foot_y, floor_z),
        radius=tube_r,
        material=metal,
        name="leg_1_rear",
    )
    _add_rod(
        central_frame,
        (-foot_x, -foot_y, floor_z),
        (-foot_x, foot_y, floor_z),
        radius=tube_r,
        material=metal,
        name="foot_0",
    )
    _add_rod(
        central_frame,
        (foot_x, -foot_y, floor_z),
        (foot_x, foot_y, floor_z),
        radius=tube_r,
        material=metal,
        name="foot_1",
    )

    for foot_index, foot_center in enumerate(
        (
            (-foot_x, -foot_y, floor_z),
            (-foot_x, foot_y, floor_z),
            (foot_x, -foot_y, floor_z),
            (foot_x, foot_y, floor_z),
        )
    ):
        central_frame.visual(
            Box((0.030, 0.020, 0.010)),
            origin=Origin(xyz=(foot_center[0], foot_center[1], foot_center[2] - 0.010)),
            material=foot_black,
            name=f"foot_cap_{foot_index}",
        )

    # Under-rack brace that visibly carries the lower folding frame.
    origin, length = _segment_origin((-0.22, lower_hinge_y, 0.66), (0.22, lower_hinge_y, 0.66))
    central_frame.visual(Cylinder(radius=tube_r, length=length), origin=origin, material=metal, name="lower_mount_rail")
    _add_rod(
        central_frame,
        (-0.22, -center_half_y, top_z),
        (-0.22, lower_hinge_y, 0.66),
        radius=tube_r * 0.8,
        material=metal,
        name="lower_mount_strut_0",
    )
    _add_rod(
        central_frame,
        (0.22, -center_half_y, top_z),
        (0.22, lower_hinge_y, 0.66),
        radius=tube_r * 0.8,
        material=metal,
        name="lower_mount_strut_1",
    )

    # Parent-side wing hinge hardware.
    for wing_index, side_sign in enumerate((-1.0, 1.0)):
        hinge_x = side_sign * center_half_x
        parent_leaf_center_x = hinge_x - side_sign * 0.011
        leaf_shift = -0.07 if side_sign < 0.0 else 0.07
        for barrel_index, barrel_y in enumerate((-0.115, 0.115)):
            _add_rod(
                central_frame,
                (hinge_x, barrel_y - 0.045, wing_hinge_z),
                (hinge_x, barrel_y + 0.045, wing_hinge_z),
                radius=hinge_r,
                material=joint_metal,
                name=f"wing_{wing_index}_hinge_barrel_{barrel_index}",
            )
            central_frame.visual(
                Box((0.022, 0.090, 0.032)),
                origin=Origin(xyz=(parent_leaf_center_x, barrel_y, wing_hinge_z + 0.016)),
                material=joint_metal,
                name=f"wing_{wing_index}_hinge_leaf_{barrel_index}",
            )

        central_frame.visual(
            Box((0.024, 0.050, 0.020)),
            origin=Origin(xyz=(hinge_x - side_sign * 0.012, leaf_shift, top_z - 0.006)),
            material=joint_metal,
            name=f"wing_{wing_index}_hinge_link",
        )

    # Parent-side lower hinge hardware.
    for index, barrel_x in enumerate((-0.17, 0.17)):
        _add_rod(
            central_frame,
            (barrel_x - 0.055, lower_hinge_y, lower_hinge_z),
            (barrel_x + 0.055, lower_hinge_y, lower_hinge_z),
            radius=hinge_r,
            material=joint_metal,
            name=f"lower_hinge_barrel_{index}",
        )
        central_frame.visual(
            Box((0.110, 0.022, 0.032)),
            origin=Origin(xyz=(barrel_x, lower_hinge_y + 0.011, lower_hinge_z + 0.016)),
            material=joint_metal,
            name=f"lower_hinge_leaf_{index}",
        )

    for wing_index, side_sign in enumerate((-1.0, 1.0)):
        wing = model.part(f"wing_{wing_index}")

        inner_x = side_sign * 0.018
        outer_x = side_sign * (wing_span - 0.018)
        rail_center_x = side_sign * (0.5 * wing_span - 0.018)
        wing_rail_size_x = wing_span - 0.036

        origin, length = _segment_origin((inner_x, -wing_half_y, 0.035), (inner_x, wing_half_y, 0.035))
        wing.visual(Cylinder(radius=tube_r, length=length), origin=origin, material=metal, name="inner_rail")
        _add_rod(
            wing,
            (outer_x, -wing_half_y, 0.035),
            (outer_x, wing_half_y, 0.035),
            radius=tube_r,
            material=metal,
            name="outer_rail",
        )
        origin, length = _segment_origin((inner_x, -wing_half_y, 0.035), (outer_x, -wing_half_y, 0.035))
        wing.visual(Cylinder(radius=tube_r, length=length), origin=origin, material=metal, name="rear_rail")
        origin, length = _segment_origin((inner_x, wing_half_y, 0.035), (outer_x, wing_half_y, 0.035))
        wing.visual(Cylinder(radius=tube_r, length=length), origin=origin, material=metal, name="front_rail")
        for bar_index, bar_x in enumerate((0.08, 0.145, 0.21)):
            x_pos = side_sign * bar_x
            _add_rod(
                wing,
                (x_pos, -wing_half_y, 0.035),
                (x_pos, wing_half_y, 0.035),
                radius=tube_r * 0.85,
                material=metal,
                name=f"drying_rod_{bar_index}",
            )

        _add_rod(
            wing,
            (0.0, -0.07, 0.0),
            (0.0, 0.07, 0.0),
            radius=hinge_r * 0.96,
            material=joint_metal,
            name="hinge_barrel",
        )
        _add_rod(
            wing,
            (0.0, -0.020, 0.0),
            (inner_x, -0.045, 0.035),
            radius=tube_r * 0.72,
            material=joint_metal,
            name="hinge_strut_0",
        )
        _add_rod(
            wing,
            (0.0, 0.020, 0.0),
            (inner_x, 0.045, 0.035),
            radius=tube_r * 0.72,
            material=joint_metal,
            name="hinge_strut_1",
        )
        wing.visual(
            Box((0.010, 0.180, 0.032)),
            origin=Origin(xyz=(side_sign * 0.024, 0.0, 0.016)),
            material=joint_metal,
            name="hinge_leaf",
        )

        model.articulation(
            f"central_to_wing_{wing_index}",
            ArticulationType.REVOLUTE,
            parent=central_frame,
            child=wing,
            origin=Origin(xyz=(side_sign * center_half_x, 0.0, wing_hinge_z)),
            axis=(0.0, 1.0, 0.0) if side_sign < 0.0 else (0.0, -1.0, 0.0),
            motion_limits=MotionLimits(effort=12.0, velocity=1.8, lower=0.0, upper=1.35),
        )

    lower_frame = model.part("lower_frame")
    rear_rail_y = 0.018
    front_rail_y = lower_depth

    _add_rod(
        lower_frame,
        (-0.09, 0.0, 0.0),
        (0.09, 0.0, 0.0),
        radius=hinge_r * 0.96,
        material=joint_metal,
        name="hinge_barrel",
    )
    lower_frame.visual(
        Box((0.230, 0.022, 0.032)),
        origin=Origin(xyz=(0.0, 0.011, -0.016)),
        material=joint_metal,
        name="hinge_leaf",
    )

    origin, length = _segment_origin((-lower_half_x, rear_rail_y, lower_plane_z), (lower_half_x, rear_rail_y, lower_plane_z))
    lower_frame.visual(Cylinder(radius=tube_r, length=length), origin=origin, material=metal, name="rear_rail")
    origin, length = _segment_origin((-lower_half_x, front_rail_y, lower_plane_z), (lower_half_x, front_rail_y, lower_plane_z))
    lower_frame.visual(Cylinder(radius=tube_r, length=length), origin=origin, material=metal, name="front_rail")
    _add_rod(
        lower_frame,
        (-lower_half_x, rear_rail_y, lower_plane_z),
        (-lower_half_x, front_rail_y, lower_plane_z),
        radius=tube_r,
        material=metal,
        name="side_rail_0",
    )
    _add_rod(
        lower_frame,
        (lower_half_x, rear_rail_y, lower_plane_z),
        (lower_half_x, front_rail_y, lower_plane_z),
        radius=tube_r,
        material=metal,
        name="side_rail_1",
    )
    for rail_index, rail_y in enumerate((0.07, 0.12, 0.17, 0.22)):
        _add_rod(
            lower_frame,
            (-lower_half_x, rail_y, lower_plane_z),
            (lower_half_x, rail_y, lower_plane_z),
            radius=tube_r * 0.85,
            material=metal,
            name=f"hanging_rail_{rail_index}",
        )

    model.articulation(
        "central_to_lower_frame",
        ArticulationType.REVOLUTE,
        parent=central_frame,
        child=lower_frame,
        origin=Origin(xyz=(0.0, lower_hinge_y, lower_hinge_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=14.0, velocity=1.6, lower=0.0, upper=1.15),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    central_frame = object_model.get_part("central_frame")
    wing_0 = object_model.get_part("wing_0")
    wing_1 = object_model.get_part("wing_1")
    lower_frame = object_model.get_part("lower_frame")

    wing_joint_0 = object_model.get_articulation("central_to_wing_0")
    wing_joint_1 = object_model.get_articulation("central_to_wing_1")
    lower_joint = object_model.get_articulation("central_to_lower_frame")

    ctx.expect_contact(
        central_frame,
        wing_0,
        elem_a="left_end_rail",
        elem_b="inner_rail",
        contact_tol=0.002,
        name="wing_0 nests against the central frame",
    )
    ctx.expect_contact(
        central_frame,
        wing_1,
        elem_a="right_end_rail",
        elem_b="inner_rail",
        contact_tol=0.002,
        name="wing_1 nests against the central frame",
    )
    ctx.expect_gap(
        central_frame,
        lower_frame,
        axis="z",
        positive_elem="lower_mount_rail",
        negative_elem="rear_rail",
        min_gap=0.01,
        max_gap=0.06,
        name="lower frame hangs beneath its mount rail",
    )
    ctx.expect_overlap(
        lower_frame,
        central_frame,
        axes="x",
        elem_a="front_rail",
        elem_b="lower_mount_rail",
        min_overlap=0.30,
        name="lower frame stays centered beneath the rack width",
    )

    def visual_center_z(part, elem: str) -> float | None:
        aabb = ctx.part_element_world_aabb(part, elem=elem)
        if aabb is None:
            return None
        return 0.5 * (aabb[0][2] + aabb[1][2])

    wing_0_rest_z = visual_center_z(wing_0, "outer_rail")
    wing_1_rest_z = visual_center_z(wing_1, "outer_rail")
    lower_rest_z = visual_center_z(lower_frame, "front_rail")

    wing_upper = wing_joint_0.motion_limits.upper if wing_joint_0.motion_limits is not None else None
    lower_upper = lower_joint.motion_limits.upper if lower_joint.motion_limits is not None else None

    if wing_upper is not None:
        with ctx.pose({wing_joint_0: wing_upper}):
            wing_0_open_z = visual_center_z(wing_0, "outer_rail")
        ctx.check(
            "wing_0 opens upward",
            wing_0_rest_z is not None and wing_0_open_z is not None and wing_0_open_z > wing_0_rest_z + 0.14,
            details=f"rest_z={wing_0_rest_z}, open_z={wing_0_open_z}",
        )

        with ctx.pose({wing_joint_1: wing_upper}):
            wing_1_open_z = visual_center_z(wing_1, "outer_rail")
        ctx.check(
            "wing_1 opens upward",
            wing_1_rest_z is not None and wing_1_open_z is not None and wing_1_open_z > wing_1_rest_z + 0.14,
            details=f"rest_z={wing_1_rest_z}, open_z={wing_1_open_z}",
        )

    if lower_upper is not None:
        with ctx.pose({lower_joint: lower_upper}):
            lower_folded_z = visual_center_z(lower_frame, "front_rail")
        ctx.check(
            "lower frame folds upward beneath the rack",
            lower_rest_z is not None and lower_folded_z is not None and lower_folded_z > lower_rest_z + 0.12,
            details=f"rest_z={lower_rest_z}, folded_z={lower_folded_z}",
        )

    return ctx.report()


object_model = build_object_model()
