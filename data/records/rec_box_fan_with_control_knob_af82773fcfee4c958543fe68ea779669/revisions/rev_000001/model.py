from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


LOWER_ARM_LENGTH = 0.112
MID_ARM_LENGTH = 0.096
TIP_LINK_PIVOT_X = 0.086
SHOULDER_REST_ANGLE = math.radians(-54.0)
ELBOW_REST_ANGLE = math.radians(20.0)


def _circle_points_x(radius: float, x: float, samples: int = 20) -> list[tuple[float, float, float]]:
    return [
        (
            x,
            radius * math.cos((2.0 * math.pi * i) / samples),
            radius * math.sin((2.0 * math.pi * i) / samples),
        )
        for i in range(samples)
    ]


def _circle_profile_2d(
    radius: float,
    *,
    samples: int = 24,
    center: tuple[float, float] = (0.0, 0.0),
) -> list[tuple[float, float]]:
    cx, cy = center
    return [
        (
            cx + radius * math.cos((2.0 * math.pi * i) / samples),
            cy + radius * math.sin((2.0 * math.pi * i) / samples),
        )
        for i in range(samples)
    ]


def _build_guard_mesh():
    outer_profile = _circle_profile_2d(0.052, samples=48)
    hole_profiles: list[list[tuple[float, float]]] = []
    for ring_radius, hole_radius, hole_count, start_angle in (
        (0.017, 0.0058, 6, math.pi / 6.0),
        (0.034, 0.0062, 10, 0.0),
    ):
        for hole_index in range(hole_count):
            angle = start_angle + (2.0 * math.pi * hole_index) / hole_count
            hole_profiles.append(
                _circle_profile_2d(
                    hole_radius,
                    samples=18,
                    center=(ring_radius * math.cos(angle), ring_radius * math.sin(angle)),
                )
            )
    return ExtrudeWithHolesGeometry(outer_profile, hole_profiles, 0.003, center=True, closed=True)


def _build_usb_cable_mesh():
    return tube_from_spline_points(
        [
            (-0.058, 0.0, -0.001),
            (-0.072, 0.0, -0.010),
            (-0.086, 0.0, -0.021),
            (-0.097, 0.0, -0.029),
        ],
        radius=0.0022,
        samples_per_segment=8,
        radial_segments=10,
        cap_ends=True,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desk_gooseneck_usb_fan")

    body_dark = model.material("body_dark", rgba=(0.16, 0.17, 0.19, 1.0))
    arm_dark = model.material("arm_dark", rgba=(0.20, 0.21, 0.23, 1.0))
    grille_silver = model.material("grille_silver", rgba=(0.78, 0.80, 0.82, 1.0))
    blade_grey = model.material("blade_grey", rgba=(0.63, 0.66, 0.69, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.09, 0.09, 0.10, 1.0))
    accent_grey = model.material("accent_grey", rgba=(0.45, 0.47, 0.50, 1.0))

    clamp_base = model.part("clamp_base")
    clamp_base.visual(
        Box((0.100, 0.058, 0.010)),
        origin=Origin(xyz=(0.000, 0.000, 0.005)),
        material=body_dark,
        name="upper_jaw",
    )
    clamp_base.visual(
        Box((0.088, 0.046, 0.0015)),
        origin=Origin(xyz=(0.003, 0.000, 0.00075)),
        material=rubber_black,
        name="upper_pad",
    )
    clamp_base.visual(
        Box((0.090, 0.050, 0.008)),
        origin=Origin(xyz=(0.003, 0.000, -0.023)),
        material=body_dark,
        name="lower_jaw",
    )
    clamp_base.visual(
        Box((0.076, 0.042, 0.0015)),
        origin=Origin(xyz=(0.008, 0.000, -0.01875)),
        material=rubber_black,
        name="lower_pad",
    )
    clamp_base.visual(
        Box((0.018, 0.058, 0.041)),
        origin=Origin(xyz=(-0.041, 0.000, -0.0105)),
        material=body_dark,
        name="rear_spine",
    )
    clamp_base.visual(
        Box((0.020, 0.050, 0.014)),
        origin=Origin(xyz=(0.038, 0.000, -0.016)),
        material=body_dark,
        name="front_lip",
    )
    clamp_base.visual(
        Box((0.020, 0.026, 0.010)),
        origin=Origin(xyz=(0.001, 0.000, 0.010)),
        material=body_dark,
        name="shoulder_block",
    )
    clamp_base.visual(
        Box((0.012, 0.004, 0.024)),
        origin=Origin(xyz=(0.001, 0.012, 0.025)),
        material=arm_dark,
        name="left_shoulder_cheek",
    )
    clamp_base.visual(
        Box((0.012, 0.004, 0.024)),
        origin=Origin(xyz=(0.001, -0.012, 0.025)),
        material=arm_dark,
        name="right_shoulder_cheek",
    )
    clamp_base.visual(
        Cylinder(radius=0.0032, length=0.010),
        origin=Origin(xyz=(-0.053, 0.000, -0.001), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber_black,
        name="cable_relief",
    )
    clamp_base.visual(
        mesh_from_geometry(_build_usb_cable_mesh(), "usb_fan_cable"),
        material=rubber_black,
        name="usb_cable",
    )
    clamp_base.inertial = Inertial.from_geometry(
        Box((0.100, 0.058, 0.048)),
        mass=0.48,
        origin=Origin(xyz=(0.000, 0.000, -0.006)),
    )

    lower_arm = model.part("lower_arm")
    lower_arm.visual(
        Cylinder(radius=0.0095, length=0.020),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=arm_dark,
        name="shoulder_barrel",
    )
    lower_arm.visual(
        Box((0.088, 0.016, 0.008)),
        origin=Origin(xyz=(0.044, 0.000, 0.000)),
        material=arm_dark,
        name="lower_bar",
    )
    lower_arm.visual(
        Box((0.018, 0.022, 0.010)),
        origin=Origin(xyz=(0.093, 0.000, 0.000)),
        material=arm_dark,
        name="elbow_neck",
    )
    lower_arm.visual(
        Box((0.018, 0.004, 0.020)),
        origin=Origin(xyz=(0.103, 0.011, 0.000)),
        material=arm_dark,
        name="left_elbow_tab",
    )
    lower_arm.visual(
        Box((0.018, 0.004, 0.020)),
        origin=Origin(xyz=(0.103, -0.011, 0.000)),
        material=arm_dark,
        name="right_elbow_tab",
    )
    lower_arm.inertial = Inertial.from_geometry(
        Box((LOWER_ARM_LENGTH + 0.010, 0.026, 0.024)),
        mass=0.18,
        origin=Origin(xyz=(LOWER_ARM_LENGTH / 2.0, 0.000, 0.000)),
    )

    mid_arm = model.part("mid_arm")
    mid_arm.visual(
        Cylinder(radius=0.0085, length=0.018),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=arm_dark,
        name="elbow_barrel",
    )
    mid_arm.visual(
        Box((0.080, 0.015, 0.007)),
        origin=Origin(xyz=(0.040, 0.000, 0.000)),
        material=arm_dark,
        name="mid_bar",
    )
    mid_arm.visual(
        Box((0.014, 0.018, 0.014)),
        origin=Origin(xyz=(0.089, 0.000, 0.000)),
        material=arm_dark,
        name="distal_cap",
    )
    mid_arm.visual(
        Box((0.012, 0.012, 0.008)),
        origin=Origin(xyz=(0.083, 0.000, -0.007)),
        material=arm_dark,
        name="distal_gusset",
    )
    mid_arm.inertial = Inertial.from_geometry(
        Box((MID_ARM_LENGTH + 0.010, 0.024, 0.020)),
        mass=0.14,
        origin=Origin(xyz=(MID_ARM_LENGTH / 2.0, 0.000, 0.000)),
    )

    tip_link = model.part("tip_link")
    tip_link.visual(
        Box((0.008, 0.018, 0.012)),
        origin=Origin(xyz=(0.004, 0.000, -0.004)),
        material=arm_dark,
        name="root_block",
    )
    tip_link.visual(
        Box((0.042, 0.012, 0.008)),
        origin=Origin(xyz=(0.029, 0.000, -0.012)),
        material=arm_dark,
        name="tip_stem",
    )
    tip_link.visual(
        Box((0.010, 0.012, 0.028)),
        origin=Origin(xyz=(0.053, 0.000, -0.022)),
        material=arm_dark,
        name="drop_brace",
    )
    tip_link.visual(
        Box((0.040, 0.072, 0.008)),
        origin=Origin(xyz=(0.072, 0.000, -0.036)),
        material=arm_dark,
        name="yoke_bridge",
    )
    tip_link.visual(
        Box((0.010, 0.006, 0.040)),
        origin=Origin(xyz=(TIP_LINK_PIVOT_X, 0.033, -0.016)),
        material=arm_dark,
        name="left_yoke_tab",
    )
    tip_link.visual(
        Box((0.010, 0.006, 0.040)),
        origin=Origin(xyz=(TIP_LINK_PIVOT_X, -0.033, -0.016)),
        material=arm_dark,
        name="right_yoke_tab",
    )
    tip_link.inertial = Inertial.from_geometry(
        Box((0.100, 0.074, 0.052)),
        mass=0.11,
        origin=Origin(xyz=(0.052, 0.000, -0.018)),
    )

    fan_head = model.part("fan_head")
    fan_head.visual(
        mesh_from_geometry(_build_guard_mesh(), "usb_fan_guard"),
        origin=Origin(xyz=(0.030, 0.000, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=grille_silver,
        name="guard",
    )
    fan_head.visual(
        Cylinder(radius=0.024, length=0.022),
        origin=Origin(xyz=(0.003, 0.000, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=body_dark,
        name="motor_housing",
    )
    fan_head.visual(
        Cylinder(radius=0.030, length=0.010),
        origin=Origin(xyz=(-0.011, 0.000, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=body_dark,
        name="rear_cap",
    )
    fan_head.visual(
        Cylinder(radius=0.008, length=0.018),
        origin=Origin(xyz=(0.020, 0.000, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=body_dark,
        name="nose_cap",
    )
    fan_head.visual(
        Cylinder(radius=0.0055, length=0.060),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=arm_dark,
        name="pivot_barrel",
    )
    for blade_index, roll_angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        fan_head.visual(
            Box((0.006, 0.038, 0.018)),
            origin=Origin(
                xyz=(0.016, 0.022 * math.cos(roll_angle), 0.022 * math.sin(roll_angle)),
                rpy=(roll_angle, 0.35, 0.0),
            ),
            material=blade_grey,
            name=f"blade_{blade_index}",
        )
    fan_head.inertial = Inertial.from_geometry(
        Box((0.085, 0.110, 0.110)),
        mass=0.20,
        origin=Origin(xyz=(0.010, 0.000, 0.000)),
    )

    speed_dial = model.part("speed_dial")
    speed_dial.visual(
        Cylinder(radius=0.013, length=0.009),
        origin=Origin(xyz=(-0.0045, 0.000, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=accent_grey,
        name="dial_knob",
    )
    speed_dial.visual(
        Box((0.003, 0.010, 0.004)),
        origin=Origin(xyz=(-0.0075, 0.000, 0.011)),
        material=rubber_black,
        name="indicator",
    )
    speed_dial.inertial = Inertial.from_geometry(
        Box((0.010, 0.028, 0.028)),
        mass=0.02,
        origin=Origin(xyz=(-0.005, 0.000, 0.000)),
    )

    model.articulation(
        "clamp_to_lower_arm",
        ArticulationType.REVOLUTE,
        parent=clamp_base,
        child=lower_arm,
        origin=Origin(xyz=(0.001, 0.000, 0.025), rpy=(0.0, SHOULDER_REST_ANGLE, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=1.2,
            lower=-0.45,
            upper=0.55,
        ),
    )
    model.articulation(
        "lower_to_mid_arm",
        ArticulationType.REVOLUTE,
        parent=lower_arm,
        child=mid_arm,
        origin=Origin(xyz=(LOWER_ARM_LENGTH, 0.000, 0.000), rpy=(0.0, ELBOW_REST_ANGLE, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=1.4,
            lower=-1.05,
            upper=0.70,
        ),
    )
    model.articulation(
        "mid_to_tip_link",
        ArticulationType.FIXED,
        parent=mid_arm,
        child=tip_link,
        origin=Origin(xyz=(MID_ARM_LENGTH, 0.000, 0.000)),
    )
    model.articulation(
        "tip_to_fan_head",
        ArticulationType.REVOLUTE,
        parent=tip_link,
        child=fan_head,
        origin=Origin(xyz=(TIP_LINK_PIVOT_X, 0.000, 0.000)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.8,
            velocity=2.0,
            lower=-0.55,
            upper=0.95,
        ),
    )
    model.articulation(
        "fan_head_to_speed_dial",
        ArticulationType.REVOLUTE,
        parent=fan_head,
        child=speed_dial,
        origin=Origin(xyz=(-0.016, 0.000, 0.000)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.2,
            velocity=4.0,
            lower=0.0,
            upper=2.6,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    clamp_base = object_model.get_part("clamp_base")
    lower_arm = object_model.get_part("lower_arm")
    mid_arm = object_model.get_part("mid_arm")
    tip_link = object_model.get_part("tip_link")
    fan_head = object_model.get_part("fan_head")
    speed_dial = object_model.get_part("speed_dial")
    shoulder = object_model.get_articulation("clamp_to_lower_arm")
    elbow = object_model.get_articulation("lower_to_mid_arm")
    head_tilt = object_model.get_articulation("tip_to_fan_head")
    dial_joint = object_model.get_articulation("fan_head_to_speed_dial")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(
        lower_arm,
        clamp_base,
        elem_a="shoulder_barrel",
        elem_b="left_shoulder_cheek",
        name="lower arm barrel is seated in the clamp shoulder fork",
    )
    ctx.expect_contact(
        mid_arm,
        lower_arm,
        elem_a="elbow_barrel",
        elem_b="left_elbow_tab",
        name="mid arm barrel is seated in the lower arm elbow fork",
    )
    ctx.expect_contact(
        fan_head,
        tip_link,
        elem_a="pivot_barrel",
        elem_b="left_yoke_tab",
        name="fan head trunnion is seated in the tip yoke",
    )
    ctx.expect_contact(
        mid_arm,
        tip_link,
        elem_a="distal_cap",
        elem_b="root_block",
        name="tip link is mounted to the arm's distal cap",
    )
    ctx.expect_contact(
        speed_dial,
        fan_head,
        elem_a="dial_knob",
        elem_b="rear_cap",
        name="speed dial sits flush to the rear cap",
    )
    ctx.expect_overlap(
        speed_dial,
        fan_head,
        axes="yz",
        elem_a="dial_knob",
        elem_b="rear_cap",
        min_overlap=0.018,
        name="speed dial stays centered on the rear cap footprint",
    )

    rest_head_pos = ctx.part_world_position(fan_head)
    with ctx.pose({shoulder: 0.35}):
        shoulder_head_pos = ctx.part_world_position(fan_head)
    ctx.check(
        "shoulder lifts the fan head",
        rest_head_pos is not None
        and shoulder_head_pos is not None
        and shoulder_head_pos[2] > rest_head_pos[2] + 0.02,
        details=f"rest={rest_head_pos}, shoulder_pose={shoulder_head_pos}",
    )

    with ctx.pose({elbow: 0.45}):
        elbow_head_pos = ctx.part_world_position(fan_head)
    ctx.check(
        "elbow lifts the forearm and head",
        rest_head_pos is not None
        and elbow_head_pos is not None
        and elbow_head_pos[2] > rest_head_pos[2] + 0.015,
        details=f"rest={rest_head_pos}, elbow_pose={elbow_head_pos}",
    )

    guard_rest_aabb = ctx.part_element_world_aabb(fan_head, elem="guard")
    with ctx.pose({head_tilt: 0.55}):
        guard_tilted_aabb = ctx.part_element_world_aabb(fan_head, elem="guard")
    guard_rest_center_z = None
    guard_tilted_center_z = None
    if guard_rest_aabb is not None:
        guard_rest_center_z = (guard_rest_aabb[0][2] + guard_rest_aabb[1][2]) * 0.5
    if guard_tilted_aabb is not None:
        guard_tilted_center_z = (guard_tilted_aabb[0][2] + guard_tilted_aabb[1][2]) * 0.5
    ctx.check(
        "head tilt raises the fan face",
        guard_rest_center_z is not None
        and guard_tilted_center_z is not None
        and guard_tilted_center_z > guard_rest_center_z + 0.006,
        details=f"guard_rest_center_z={guard_rest_center_z}, guard_tilted_center_z={guard_tilted_center_z}",
    )
    indicator_rest_aabb = ctx.part_element_world_aabb(speed_dial, elem="indicator")
    with ctx.pose({dial_joint: 1.0}):
        indicator_turned_aabb = ctx.part_element_world_aabb(speed_dial, elem="indicator")
    indicator_rest_center = None
    indicator_turned_center = None
    if indicator_rest_aabb is not None:
        indicator_rest_center = (
            (indicator_rest_aabb[0][0] + indicator_rest_aabb[1][0]) * 0.5,
            (indicator_rest_aabb[0][1] + indicator_rest_aabb[1][1]) * 0.5,
            (indicator_rest_aabb[0][2] + indicator_rest_aabb[1][2]) * 0.5,
        )
    if indicator_turned_aabb is not None:
        indicator_turned_center = (
            (indicator_turned_aabb[0][0] + indicator_turned_aabb[1][0]) * 0.5,
            (indicator_turned_aabb[0][1] + indicator_turned_aabb[1][1]) * 0.5,
            (indicator_turned_aabb[0][2] + indicator_turned_aabb[1][2]) * 0.5,
        )
    ctx.check(
        "speed dial indicator rotates around the knob axis",
        indicator_rest_center is not None
        and indicator_turned_center is not None
        and (
            abs(indicator_turned_center[1] - indicator_rest_center[1]) > 0.004
            or abs(indicator_turned_center[2] - indicator_rest_center[2]) > 0.004
        ),
        details=f"rest={indicator_rest_center}, turned={indicator_turned_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
