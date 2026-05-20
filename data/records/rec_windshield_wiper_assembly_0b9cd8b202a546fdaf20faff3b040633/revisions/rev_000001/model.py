from __future__ import annotations

from math import atan2, cos, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    superellipse_profile,
    sweep_profile_along_spline,
)


LEFT_SPINDLE_POS = (-0.36, 0.047, 0.011)
RIGHT_SPINDLE_POS = (0.34, 0.047, 0.011)
MOTOR_SHAFT_POS = (-0.12, -0.028, -0.042)

LEFT_MOTOR_PIN_LOCAL = (0.085, -0.002, -0.054)
LEFT_CROSS_PIN_LOCAL = (0.085, 0.018, -0.054)
RIGHT_CROSS_PIN_LOCAL = (-0.085, 0.010, -0.054)

LEFT_ARM_YAW = 0.42
RIGHT_ARM_YAW = -0.34
BLADE_ROLL = -0.88
LEFT_BLADE_YAW = LEFT_ARM_YAW + 1.42
RIGHT_BLADE_YAW = RIGHT_ARM_YAW + 1.42


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _rotate_xy(x: float, y: float, yaw: float) -> tuple[float, float]:
    c = cos(yaw)
    s = sin(yaw)
    return (c * x - s * y, s * x + c * y)


def _rotate_points(points: list[tuple[float, float, float]], yaw: float) -> list[tuple[float, float, float]]:
    return [(*_rotate_xy(x, y, yaw), z) for x, y, z in points]


def _offset_profile(
    profile: list[tuple[float, float]],
    *,
    dx: float = 0.0,
    dy: float = 0.0,
) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def _rect_profile(width: float, height: float) -> list[tuple[float, float]]:
    half_w = width * 0.5
    half_h = height * 0.5
    return [
        (-half_w, -half_h),
        (half_w, -half_h),
        (half_w, half_h),
        (-half_w, half_h),
    ]


def _link_plate_mesh(name: str, *, length: float, width: float, thickness: float, hole_diameter: float):
    outer = rounded_rect_profile(length + width, width, width * 0.45, corner_segments=8)
    hole = superellipse_profile(hole_diameter, hole_diameter, exponent=2.0, segments=24)
    geometry = ExtrudeWithHolesGeometry(
        outer,
        [
            _offset_profile(hole, dx=-(length * 0.5)),
            _offset_profile(hole, dx=length * 0.5),
        ],
        thickness,
        center=True,
    )
    return _save_mesh(name, geometry)


def _arm_mesh(name: str, points: list[tuple[float, float, float]], *, width: float, thickness: float):
    profile = rounded_rect_profile(width, thickness, thickness * 0.45, corner_segments=4)
    geometry = sweep_profile_along_spline(
        points,
        profile=profile,
        samples_per_segment=12,
        cap_profile=True,
        up_hint=(0.0, 0.0, 1.0),
    )
    return _save_mesh(name, geometry)


def _ring_mesh(name: str, *, outer_diameter: float, inner_diameter: float, thickness: float):
    outer = superellipse_profile(outer_diameter, outer_diameter, exponent=2.0, segments=32)
    inner = superellipse_profile(inner_diameter, inner_diameter, exponent=2.0, segments=24)
    geometry = ExtrudeWithHolesGeometry(outer, [inner], thickness, center=True)
    return _save_mesh(name, geometry)


def _panel_with_holes_mesh(
    name: str,
    *,
    width: float,
    height: float,
    thickness: float,
    holes: list[tuple[float, float, float]],
):
    hole_profiles = [
        _offset_profile(
            superellipse_profile(diameter, diameter, exponent=2.0, segments=24),
            dx=x,
            dy=y,
        )
        for x, y, diameter in holes
    ]
    geometry = ExtrudeWithHolesGeometry(_rect_profile(width, height), hole_profiles, thickness, center=True)
    return _save_mesh(name, geometry)


def _aabb_center(aabb):
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple((mins[i] + maxs[i]) * 0.5 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="windshield_wiper_assembly")

    cowl_paint = model.material("cowl_paint", rgba=(0.13, 0.14, 0.15, 1.0))
    trim_black = model.material("trim_black", rgba=(0.07, 0.07, 0.08, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.28, 0.29, 0.31, 1.0))
    zinc_linkage = model.material("zinc_linkage", rgba=(0.64, 0.66, 0.68, 1.0))
    glass_tint = model.material("glass_tint", rgba=(0.58, 0.71, 0.78, 0.38))
    rubber = model.material("rubber", rgba=(0.04, 0.04, 0.04, 1.0))
    satin_black = model.material("satin_black", rgba=(0.11, 0.12, 0.13, 1.0))
    washer_steel = model.material("washer_steel", rgba=(0.74, 0.75, 0.77, 1.0))

    left_arm_points = _rotate_points(
        [
            (0.0, 0.000, 0.048),
            (0.0, 0.070, 0.067),
            (0.0, 0.205, 0.124),
            (0.0, 0.375, 0.212),
            (0.0, 0.468, 0.248),
        ],
        LEFT_ARM_YAW,
    )
    right_arm_points = _rotate_points(
        [
            (0.0, 0.000, 0.048),
            (0.0, 0.065, 0.064),
            (0.0, 0.190, 0.118),
            (0.0, 0.340, 0.198),
            (0.0, 0.432, 0.231),
        ],
        RIGHT_ARM_YAW,
    )
    left_arm_tip = left_arm_points[-1]
    right_arm_tip = right_arm_points[-1]
    left_tip_vector = tuple(left_arm_tip[i] - left_arm_points[-2][i] for i in range(3))
    right_tip_vector = tuple(right_arm_tip[i] - right_arm_points[-2][i] for i in range(3))
    left_tip_length = (left_tip_vector[0] ** 2 + left_tip_vector[1] ** 2 + left_tip_vector[2] ** 2) ** 0.5
    right_tip_length = (right_tip_vector[0] ** 2 + right_tip_vector[1] ** 2 + right_tip_vector[2] ** 2) ** 0.5
    left_tip_yaw = atan2(left_tip_vector[1], left_tip_vector[0])
    right_tip_yaw = atan2(right_tip_vector[1], right_tip_vector[0])
    left_tip_pitch = atan2(left_tip_vector[2], (left_tip_vector[0] ** 2 + left_tip_vector[1] ** 2) ** 0.5)
    right_tip_pitch = atan2(right_tip_vector[2], (right_tip_vector[0] ** 2 + right_tip_vector[1] ** 2) ** 0.5)
    left_tip_mid = tuple(left_arm_points[-2][i] + left_tip_vector[i] * 0.32 for i in range(3))
    right_tip_mid = tuple(right_arm_points[-2][i] + right_tip_vector[i] * 0.32 for i in range(3))
    left_tip_link_length = max(left_tip_length * 0.58, 0.026)
    right_tip_link_length = max(right_tip_length * 0.58, 0.026)

    drive_vector = (
        LEFT_SPINDLE_POS[0] + LEFT_MOTOR_PIN_LOCAL[0] - (MOTOR_SHAFT_POS[0] - 0.045),
        LEFT_SPINDLE_POS[1] + LEFT_MOTOR_PIN_LOCAL[1] - MOTOR_SHAFT_POS[1],
    )
    drive_length = (drive_vector[0] ** 2 + drive_vector[1] ** 2) ** 0.5
    drive_yaw = atan2(drive_vector[1], drive_vector[0])

    cross_vector = (
        RIGHT_SPINDLE_POS[0] + RIGHT_CROSS_PIN_LOCAL[0] - (LEFT_SPINDLE_POS[0] + LEFT_CROSS_PIN_LOCAL[0]),
        RIGHT_SPINDLE_POS[1] + RIGHT_CROSS_PIN_LOCAL[1] - (LEFT_SPINDLE_POS[1] + LEFT_CROSS_PIN_LOCAL[1]),
    )
    cross_length = (cross_vector[0] ** 2 + cross_vector[1] ** 2) ** 0.5
    cross_yaw = atan2(cross_vector[1], cross_vector[0])

    cowl = model.part("cowl")
    cowl.visual(
        _panel_with_holes_mesh(
            "cowl_panel_plate",
            width=1.42,
            height=0.19,
            thickness=0.022,
            holes=[
                (LEFT_SPINDLE_POS[0], LEFT_SPINDLE_POS[1], 0.018),
                (RIGHT_SPINDLE_POS[0], RIGHT_SPINDLE_POS[1], 0.018),
                (MOTOR_SHAFT_POS[0], MOTOR_SHAFT_POS[1], 0.040),
            ],
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
        material=cowl_paint,
        name="cowl_panel",
    )
    cowl.visual(
        Box((1.42, 0.026, 0.045)),
        origin=Origin(xyz=(0.0, -0.082, -0.0005)),
        material=cowl_paint,
        name="front_lip",
    )
    cowl.visual(
        _panel_with_holes_mesh(
            "motor_bracket_plate",
            width=0.42,
            height=0.072,
            thickness=0.048,
            holes=[(0.0, 0.002, 0.040)],
        ),
        origin=Origin(xyz=(-0.12, -0.030, -0.024)),
        material=trim_black,
        name="motor_bracket",
    )
    cowl.visual(
        Box((0.18, 0.092, 0.050)),
        origin=Origin(xyz=(-0.12, -0.036, -0.073)),
        material=dark_steel,
        name="motor_housing",
    )
    cowl.visual(
        Cylinder(radius=0.028, length=0.030),
        origin=Origin(xyz=(-0.12, -0.028, -0.026)),
        material=dark_steel,
        name="motor_output_boss",
    )
    cowl.visual(
        Cylinder(radius=0.030, length=0.040),
        origin=Origin(xyz=(LEFT_SPINDLE_POS[0], LEFT_SPINDLE_POS[1], -0.028)),
        material=dark_steel,
        name="left_housing",
    )
    cowl.visual(
        _ring_mesh(
            "left_cap_ring",
            outer_diameter=0.052,
            inner_diameter=0.018,
            thickness=0.010,
        ),
        origin=Origin(xyz=(LEFT_SPINDLE_POS[0], LEFT_SPINDLE_POS[1], 0.027)),
        material=trim_black,
        name="left_cap",
    )
    cowl.visual(
        Box((0.100, 0.090, 0.030)),
        origin=Origin(xyz=(LEFT_SPINDLE_POS[0], -0.010, -0.015)),
        material=dark_steel,
        name="left_brace",
    )
    cowl.visual(
        Cylinder(radius=0.030, length=0.040),
        origin=Origin(xyz=(RIGHT_SPINDLE_POS[0], RIGHT_SPINDLE_POS[1], -0.028)),
        material=dark_steel,
        name="right_housing",
    )
    cowl.visual(
        _ring_mesh(
            "right_cap_ring",
            outer_diameter=0.052,
            inner_diameter=0.018,
            thickness=0.010,
        ),
        origin=Origin(xyz=(RIGHT_SPINDLE_POS[0], RIGHT_SPINDLE_POS[1], 0.027)),
        material=trim_black,
        name="right_cap",
    )
    cowl.visual(
        Box((0.100, 0.090, 0.030)),
        origin=Origin(xyz=(RIGHT_SPINDLE_POS[0], -0.010, -0.015)),
        material=dark_steel,
        name="right_brace",
    )

    cowl.visual(
        Box((1.28, 0.010, 0.64)),
        origin=Origin(xyz=(0.0, 0.193, 0.300), rpy=(-1.02, 0.0, 0.0)),
        material=glass_tint,
        name="glass",
    )
    cowl.visual(
        Box((1.30, 0.030, 0.022)),
        origin=Origin(xyz=(0.0, 0.110, 0.011)),
        material=trim_black,
        name="base_seal",
    )
    motor_crank = model.part("motor_crank")
    motor_crank.visual(
        Cylinder(radius=0.018, length=0.012),
        material=dark_steel,
        name="hub",
    )
    motor_crank.visual(
        Box((0.066, 0.016, 0.005)),
        origin=Origin(xyz=(-0.022, 0.0, -0.060)),
        material=zinc_linkage,
        name="crank_arm",
    )
    motor_crank.visual(
        Box((0.026, 0.024, 0.005)),
        origin=Origin(xyz=(0.009, -0.018, -0.060)),
        material=zinc_linkage,
        name="counterweight",
    )
    motor_crank.visual(
        Cylinder(radius=0.0055, length=0.014),
        origin=Origin(xyz=(-0.045, 0.0, 0.0)),
        material=washer_steel,
        name="drive_pin",
    )

    drive_link = model.part("drive_link")
    drive_link.visual(
        _link_plate_mesh(
            "drive_link_plate",
            length=drive_length,
            width=0.024,
            thickness=0.0035,
            hole_diameter=0.0125,
        ),
        origin=Origin(xyz=(drive_length * 0.5, 0.0, 0.0)),
        material=zinc_linkage,
        name="drive_plate",
    )

    left_spindle = model.part("left_spindle")
    left_spindle.visual(
        Cylinder(radius=0.0075, length=0.165),
        origin=Origin(xyz=(0.0, 0.0, -0.010)),
        material=dark_steel,
        name="shaft_core",
    )
    left_spindle.visual(
        Cylinder(radius=0.013, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.078)),
        material=washer_steel,
        name="spindle_nut",
    )
    left_spindle.visual(
        Box((0.088, 0.024, 0.006)),
        origin=Origin(xyz=(0.043, 0.004, -0.062)),
        material=zinc_linkage,
        name="bellcrank",
    )
    left_spindle.visual(
        Box((0.046, 0.020, 0.006)),
        origin=Origin(xyz=(0.067, 0.022, -0.062)),
        material=zinc_linkage,
        name="cross_ear",
    )
    left_spindle.visual(
        Cylinder(radius=0.0055, length=0.016),
        origin=Origin(xyz=LEFT_MOTOR_PIN_LOCAL),
        material=washer_steel,
        name="motor_link_pin",
    )
    left_spindle.visual(
        Cylinder(radius=0.0055, length=0.016),
        origin=Origin(xyz=LEFT_CROSS_PIN_LOCAL),
        material=washer_steel,
        name="cross_link_pin",
    )
    left_spindle.visual(
        _arm_mesh("left_arm_beam", left_arm_points[:-1], width=0.022, thickness=0.006),
        material=satin_black,
        name="arm_beam",
    )
    left_spindle.visual(
        Box((left_tip_link_length, 0.010, 0.008)),
        origin=Origin(xyz=left_tip_mid, rpy=(0.0, -left_tip_pitch, left_tip_yaw)),
        material=satin_black,
        name="arm_tip",
    )
    left_spindle.visual(
        Box((0.040, 0.020, 0.012)),
        origin=Origin(xyz=(left_arm_points[1][0], left_arm_points[1][1], left_arm_points[1][2] + 0.004)),
        material=satin_black,
        name="arm_head",
    )

    right_spindle = model.part("right_spindle")
    right_spindle.visual(
        Cylinder(radius=0.0075, length=0.165),
        origin=Origin(xyz=(0.0, 0.0, -0.010)),
        material=dark_steel,
        name="shaft_core",
    )
    right_spindle.visual(
        Cylinder(radius=0.013, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.076)),
        material=washer_steel,
        name="spindle_nut",
    )
    right_spindle.visual(
        Box((0.088, 0.024, 0.006)),
        origin=Origin(xyz=(-0.043, 0.004, -0.062)),
        material=zinc_linkage,
        name="bellcrank",
    )
    right_spindle.visual(
        Cylinder(radius=0.0055, length=0.016),
        origin=Origin(xyz=RIGHT_CROSS_PIN_LOCAL),
        material=washer_steel,
        name="cross_link_pin",
    )
    right_spindle.visual(
        _arm_mesh("right_arm_beam", right_arm_points[:-1], width=0.021, thickness=0.006),
        material=satin_black,
        name="arm_beam",
    )
    right_spindle.visual(
        Box((right_tip_link_length, 0.010, 0.008)),
        origin=Origin(xyz=right_tip_mid, rpy=(0.0, -right_tip_pitch, right_tip_yaw)),
        material=satin_black,
        name="arm_tip",
    )
    right_spindle.visual(
        Box((0.036, 0.020, 0.012)),
        origin=Origin(xyz=(right_arm_points[1][0], right_arm_points[1][1], right_arm_points[1][2] + 0.004)),
        material=satin_black,
        name="arm_head",
    )

    cross_link = model.part("cross_link")
    cross_link.visual(
        _link_plate_mesh(
            "cross_link_plate",
            length=cross_length,
            width=0.026,
            thickness=0.0035,
            hole_diameter=0.0125,
        ),
        origin=Origin(xyz=(cross_length * 0.5, 0.0, 0.0)),
        material=zinc_linkage,
        name="cross_plate",
    )

    left_blade = model.part("left_blade")
    left_blade.visual(
        Box((0.510, 0.014, 0.006)),
        origin=Origin(xyz=(0.060, 0.0, -0.028)),
        material=satin_black,
        name="frame_rail",
    )
    left_blade.visual(
        Box((0.472, 0.004, 0.010)),
        origin=Origin(xyz=(0.060, 0.0, -0.036)),
        material=rubber,
        name="rubber_strip",
    )
    left_blade.visual(
        Box((0.026, 0.014, 0.026)),
        origin=Origin(xyz=(0.006, 0.0, -0.013)),
        material=satin_black,
        name="support_bridge",
    )
    left_blade.visual(
        Box((0.180, 0.010, 0.005)),
        origin=Origin(xyz=(0.060, 0.0, -0.0225)),
        material=satin_black,
        name="yoke_center",
    )
    left_blade.visual(
        Box((0.135, 0.008, 0.005)),
        origin=Origin(xyz=(-0.058, 0.0, -0.0235)),
        material=satin_black,
        name="yoke_inner",
    )
    left_blade.visual(
        Box((0.135, 0.008, 0.005)),
        origin=Origin(xyz=(0.178, 0.0, -0.0235)),
        material=satin_black,
        name="yoke_outer",
    )

    right_blade = model.part("right_blade")
    right_blade.visual(
        Box((0.465, 0.014, 0.006)),
        origin=Origin(xyz=(-0.055, 0.0, -0.028)),
        material=satin_black,
        name="frame_rail",
    )
    right_blade.visual(
        Box((0.432, 0.004, 0.010)),
        origin=Origin(xyz=(-0.055, 0.0, -0.036)),
        material=rubber,
        name="rubber_strip",
    )
    right_blade.visual(
        Box((0.026, 0.014, 0.026)),
        origin=Origin(xyz=(-0.006, 0.0, -0.013)),
        material=satin_black,
        name="support_bridge",
    )
    right_blade.visual(
        Box((0.166, 0.010, 0.005)),
        origin=Origin(xyz=(-0.055, 0.0, -0.0225)),
        material=satin_black,
        name="yoke_center",
    )
    right_blade.visual(
        Box((0.120, 0.008, 0.005)),
        origin=Origin(xyz=(-0.163, 0.0, -0.0235)),
        material=satin_black,
        name="yoke_inner",
    )
    right_blade.visual(
        Box((0.120, 0.008, 0.005)),
        origin=Origin(xyz=(0.053, 0.0, -0.0235)),
        material=satin_black,
        name="yoke_outer",
    )

    model.articulation(
        "crank_spin",
        ArticulationType.CONTINUOUS,
        parent=cowl,
        child=motor_crank,
        origin=Origin(xyz=MOTOR_SHAFT_POS),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=5.0, velocity=20.0),
    )
    model.articulation(
        "drive_link_pin",
        ArticulationType.REVOLUTE,
        parent=motor_crank,
        child=drive_link,
        origin=Origin(xyz=(-0.045, 0.0, 0.003), rpy=(0.0, 0.0, drive_yaw)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=8.0, lower=-0.7, upper=0.7),
    )
    model.articulation(
        "left_spindle_sweep",
        ArticulationType.REVOLUTE,
        parent=cowl,
        child=left_spindle,
        origin=Origin(xyz=LEFT_SPINDLE_POS),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=3.0, lower=-0.55, upper=0.85),
    )
    model.articulation(
        "right_spindle_sweep",
        ArticulationType.REVOLUTE,
        parent=cowl,
        child=right_spindle,
        origin=Origin(xyz=RIGHT_SPINDLE_POS),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=3.0, lower=-0.70, upper=0.70),
    )
    model.articulation(
        "cross_link_pin",
        ArticulationType.REVOLUTE,
        parent=left_spindle,
        child=cross_link,
        origin=Origin(xyz=LEFT_CROSS_PIN_LOCAL, rpy=(0.0, 0.0, cross_yaw)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=8.0, lower=-0.5, upper=0.5),
    )
    model.articulation(
        "left_blade_pitch",
        ArticulationType.REVOLUTE,
        parent=left_spindle,
        child=left_blade,
        origin=Origin(xyz=left_arm_tip, rpy=(BLADE_ROLL, 0.0, LEFT_BLADE_YAW)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=4.0, lower=-0.32, upper=0.32),
    )
    model.articulation(
        "right_blade_pitch",
        ArticulationType.REVOLUTE,
        parent=right_spindle,
        child=right_blade,
        origin=Origin(xyz=right_arm_tip, rpy=(BLADE_ROLL, 0.0, RIGHT_BLADE_YAW)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=4.0, lower=-0.32, upper=0.32),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    cowl = object_model.get_part("cowl")
    drive_link = object_model.get_part("drive_link")
    cross_link = object_model.get_part("cross_link")
    left_spindle = object_model.get_part("left_spindle")
    right_spindle = object_model.get_part("right_spindle")
    left_blade = object_model.get_part("left_blade")
    right_blade = object_model.get_part("right_blade")
    motor_crank = object_model.get_part("motor_crank")

    crank_spin = object_model.get_articulation("crank_spin")
    left_spindle_sweep = object_model.get_articulation("left_spindle_sweep")
    left_blade_pitch = object_model.get_articulation("left_blade_pitch")
    right_blade_pitch = object_model.get_articulation("right_blade_pitch")

    ctx.allow_overlap(
        cowl,
        left_spindle,
        elem_a="left_housing",
        elem_b="shaft_core",
        reason="The left spindle shaft intentionally journals through the left cowl housing.",
    )
    ctx.allow_overlap(
        cowl,
        left_spindle,
        elem_a="left_cap",
        elem_b="shaft_core",
        reason="The left spindle cap is represented as a simplified trim ring around the shaft.",
    )
    ctx.allow_overlap(
        cowl,
        left_spindle,
        elem_a="cowl_panel",
        elem_b="shaft_core",
        reason="The cowl panel is simplified around the left spindle pass-through hole.",
    )
    ctx.allow_overlap(
        cowl,
        right_spindle,
        elem_a="right_housing",
        elem_b="shaft_core",
        reason="The right spindle shaft intentionally journals through the right cowl housing.",
    )
    ctx.allow_overlap(
        cowl,
        right_spindle,
        elem_a="right_cap",
        elem_b="shaft_core",
        reason="The right spindle cap is represented as a simplified trim ring around the shaft.",
    )
    ctx.allow_overlap(
        cowl,
        right_spindle,
        elem_a="cowl_panel",
        elem_b="shaft_core",
        reason="The cowl panel is simplified around the right spindle pass-through hole.",
    )
    ctx.allow_overlap(
        cowl,
        motor_crank,
        elem_a="motor_bracket",
        elem_b="hub",
        reason="The motor output hub is intentionally represented passing through the stamped motor bracket.",
    )
    ctx.allow_overlap(
        cowl,
        motor_crank,
        elem_a="motor_bracket",
        elem_b="drive_pin",
        reason="The crank pin is simplified very close to the bracket plane at the motor output.",
    )
    ctx.allow_overlap(
        left_blade,
        left_spindle,
        elem_a="frame_rail",
        elem_b="arm_tip",
        reason="The left blade saddle is simplified closely around the narrow arm-tip adapter.",
    )
    ctx.allow_overlap(
        right_blade,
        right_spindle,
        elem_a="frame_rail",
        elem_b="arm_tip",
        reason="The right blade saddle is simplified closely around the narrow arm-tip adapter.",
    )
    ctx.allow_overlap(
        right_blade,
        right_spindle,
        elem_a="yoke_center",
        elem_b="arm_tip",
        reason="The right blade's center yoke is simplified tightly around the arm-tip saddle.",
    )

    ctx.expect_gap(
        cowl,
        drive_link,
        axis="z",
        positive_elem="cowl_panel",
        negative_elem="drive_plate",
        min_gap=0.030,
        max_gap=0.070,
        name="drive link stays tucked under the cowl",
    )
    ctx.expect_gap(
        cowl,
        cross_link,
        axis="z",
        positive_elem="cowl_panel",
        negative_elem="cross_plate",
        min_gap=0.030,
        max_gap=0.070,
        name="cross link stays tucked under the cowl",
    )
    ctx.expect_overlap(
        drive_link,
        left_spindle,
        axes="xy",
        elem_a="drive_plate",
        elem_b="motor_link_pin",
        min_overlap=0.009,
        name="drive link reaches the left spindle bellcrank pin",
    )
    ctx.expect_overlap(
        cross_link,
        right_spindle,
        axes="xy",
        elem_a="cross_plate",
        elem_b="cross_link_pin",
        min_overlap=0.009,
        name="cross-car link reaches the right spindle pin",
    )

    motor_rest = _aabb_center(ctx.part_world_aabb(motor_crank))
    with ctx.pose({crank_spin: 1.1}):
        motor_turned = _aabb_center(ctx.part_world_aabb(drive_link))
    drive_rest = _aabb_center(ctx.part_world_aabb(drive_link))
    ctx.check(
        "motor crank rotates the drive link around the output axis",
        drive_rest is not None
        and motor_turned is not None
        and ((motor_turned[0] - drive_rest[0]) ** 2 + (motor_turned[1] - drive_rest[1]) ** 2) ** 0.5 > 0.020,
        details=f"rest={drive_rest}, turned={motor_turned}, crank={motor_rest}",
    )

    left_rest = _aabb_center(ctx.part_world_aabb(left_blade))
    with ctx.pose({left_spindle_sweep: 0.40}):
        left_swept = _aabb_center(ctx.part_world_aabb(left_blade))
    ctx.check(
        "left spindle sweeps the left blade across the windshield",
        left_rest is not None
        and left_swept is not None
        and ((left_swept[0] - left_rest[0]) ** 2 + (left_swept[1] - left_rest[1]) ** 2) ** 0.5 > 0.070,
        details=f"rest={left_rest}, swept={left_swept}",
    )

    left_pitch_rest = ctx.part_world_aabb(left_blade)
    with ctx.pose({left_blade_pitch: 0.24}):
        left_pitch_up = ctx.part_world_aabb(left_blade)
    left_pitch_rest_center = _aabb_center(left_pitch_rest)
    left_pitch_up_center = _aabb_center(left_pitch_up)
    ctx.check(
        "left blade can pitch on its support joint",
        left_pitch_rest_center is not None
        and left_pitch_up_center is not None
        and abs(left_pitch_up_center[2] - left_pitch_rest_center[2]) > 0.003,
        details=f"rest={left_pitch_rest_center}, pitched={left_pitch_up_center}",
    )

    right_pitch_rest = ctx.part_world_aabb(right_blade)
    with ctx.pose({right_blade_pitch: -0.24}):
        right_pitch_up = ctx.part_world_aabb(right_blade)
    right_pitch_rest_center = _aabb_center(right_pitch_rest)
    right_pitch_up_center = _aabb_center(right_pitch_up)
    ctx.check(
        "right blade can pitch on its support joint",
        right_pitch_rest_center is not None
        and right_pitch_up_center is not None
        and abs(right_pitch_up_center[2] - right_pitch_rest_center[2]) > 0.003,
        details=f"rest={right_pitch_rest_center}, pitched={right_pitch_up_center}",
    )

    return ctx.report()
object_model = build_object_model()
