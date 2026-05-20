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
    ExtrudeGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


MOTOR_LAYOUT: tuple[tuple[str, float], ...] = (
    ("front", 0.0),
    ("front_left", math.pi / 3.0),
    ("rear_left", 2.0 * math.pi / 3.0),
    ("rear", math.pi),
    ("rear_right", 4.0 * math.pi / 3.0),
    ("front_right", 5.0 * math.pi / 3.0),
)

ARM_CENTER_RADIUS = 0.42
ARM_LENGTH = 0.62
MOTOR_RADIUS = 0.76
MOTOR_TOP_Z = 0.140


def _tank_shell_mesh():
    outer_profile = [
        (0.050, -0.020),
        (0.128, -0.008),
        (0.205, -0.032),
        (0.248, -0.108),
        (0.252, -0.220),
        (0.212, -0.305),
        (0.112, -0.346),
        (0.042, -0.338),
    ]
    inner_profile = [
        (0.038, -0.032),
        (0.112, -0.022),
        (0.188, -0.046),
        (0.230, -0.116),
        (0.234, -0.212),
        (0.198, -0.292),
        (0.100, -0.329),
        (0.032, -0.320),
    ]
    return LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=64,
        start_cap="flat",
        end_cap="flat",
        lip_samples=8,
    )


def _side_skid_mesh(y_sign: float):
    return tube_from_spline_points(
        [
            (0.24, 0.21 * y_sign, -0.018),
            (0.34, 0.31 * y_sign, -0.215),
            (0.42, 0.34 * y_sign, -0.455),
            (-0.42, 0.34 * y_sign, -0.455),
            (-0.34, 0.31 * y_sign, -0.215),
            (-0.24, 0.21 * y_sign, -0.018),
        ],
        radius=0.018,
        samples_per_segment=12,
        radial_segments=18,
        cap_ends=True,
    )


def _motor_position(angle: float) -> tuple[float, float]:
    return (MOTOR_RADIUS * math.cos(angle), MOTOR_RADIUS * math.sin(angle))


def _prop_blade_mesh():
    return mesh_from_geometry(
        ExtrudeGeometry.centered(
            [
                (0.022, -0.018),
                (0.070, -0.024),
                (0.160, -0.018),
                (0.236, -0.008),
                (0.236, 0.008),
                (0.160, 0.018),
                (0.070, 0.024),
                (0.022, 0.018),
            ],
            0.008,
        ),
        "sprayer_prop_blade",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="agricultural_sprayer_hexacopter")

    frame_gray = model.material("frame_gray", rgba=(0.19, 0.20, 0.22, 1.0))
    graphite = model.material("graphite", rgba=(0.12, 0.13, 0.14, 1.0))
    motor_black = model.material("motor_black", rgba=(0.08, 0.08, 0.09, 1.0))
    tank_poly = model.material("tank_poly", rgba=(0.92, 0.95, 0.93, 0.82))
    spray_gray = model.material("spray_gray", rgba=(0.62, 0.66, 0.68, 1.0))
    nozzle_black = model.material("nozzle_black", rgba=(0.10, 0.10, 0.11, 1.0))
    prop_blade_mesh = _prop_blade_mesh()

    airframe = model.part("airframe")
    airframe.visual(
        Box((0.52, 0.30, 0.05)),
        material=frame_gray,
        name="center_deck",
    )
    airframe.visual(
        Box((0.84, 0.06, 0.03)),
        origin=Origin(xyz=(0.0, 0.18, -0.010)),
        material=frame_gray,
        name="left_main_rail",
    )
    airframe.visual(
        Box((0.84, 0.06, 0.03)),
        origin=Origin(xyz=(0.0, -0.18, -0.010)),
        material=frame_gray,
        name="right_main_rail",
    )
    airframe.visual(
        Box((0.12, 0.44, 0.028)),
        origin=Origin(xyz=(0.12, 0.0, -0.010)),
        material=frame_gray,
        name="front_crossmember",
    )
    airframe.visual(
        Box((0.12, 0.44, 0.028)),
        origin=Origin(xyz=(-0.12, 0.0, -0.010)),
        material=frame_gray,
        name="rear_crossmember",
    )
    airframe.visual(
        Box((0.34, 0.20, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.065)),
        material=graphite,
        name="battery_pack",
    )

    for arm_name, angle in MOTOR_LAYOUT:
        arm_x = ARM_CENTER_RADIUS * math.cos(angle)
        arm_y = ARM_CENTER_RADIUS * math.sin(angle)
        motor_x, motor_y = _motor_position(angle)
        airframe.visual(
            Box((ARM_LENGTH, 0.060, 0.024)),
            origin=Origin(xyz=(arm_x, arm_y, 0.016), rpy=(0.0, 0.0, angle)),
            material=frame_gray,
            name=f"{arm_name}_arm",
        )
        airframe.visual(
            Cylinder(radius=0.045, length=0.100),
            origin=Origin(xyz=(motor_x, motor_y, 0.072)),
            material=motor_black,
            name=f"{arm_name}_motor",
        )
        airframe.visual(
            Cylinder(radius=0.032, length=0.018),
            origin=Origin(xyz=(motor_x, motor_y, 0.131)),
            material=graphite,
            name=f"{arm_name}_motor_cap",
        )

    airframe.visual(
        mesh_from_geometry(_side_skid_mesh(1.0), "left_skid_gear"),
        material=frame_gray,
        name="left_skid_gear",
    )
    airframe.visual(
        mesh_from_geometry(_side_skid_mesh(-1.0), "right_skid_gear"),
        material=frame_gray,
        name="right_skid_gear",
    )
    airframe.inertial = Inertial.from_geometry(
        Box((1.80, 1.80, 0.70)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, -0.140)),
    )

    tank = model.part("tank")
    tank.visual(
        Box((0.22, 0.16, 0.02)),
        origin=Origin(xyz=(0.0, 0.0, -0.010)),
        material=frame_gray,
        name="tank_mount_plate",
    )
    tank.visual(
        Box((0.05, 0.03, 0.060)),
        origin=Origin(xyz=(0.095, 0.0, -0.042)),
        material=frame_gray,
        name="right_mount_lug",
    )
    tank.visual(
        Box((0.05, 0.03, 0.060)),
        origin=Origin(xyz=(-0.095, 0.0, -0.042)),
        material=frame_gray,
        name="left_mount_lug",
    )
    tank.visual(
        mesh_from_geometry(_tank_shell_mesh(), "sprayer_tank_shell"),
        material=tank_poly,
        name="tank_shell",
    )
    tank.visual(
        Cylinder(radius=0.040, length=0.032),
        origin=Origin(xyz=(0.0, 0.0, -0.328)),
        material=frame_gray,
        name="drain_sump",
    )
    tank.visual(
        Box((0.14, 0.05, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, -0.328)),
        material=frame_gray,
        name="boom_bracket_body",
    )
    tank.visual(
        Box((0.022, 0.040, 0.192)),
        origin=Origin(xyz=(0.056, 0.0, -0.244)),
        material=frame_gray,
        name="right_bracket_strut",
    )
    tank.visual(
        Box((0.022, 0.040, 0.192)),
        origin=Origin(xyz=(-0.056, 0.0, -0.244)),
        material=frame_gray,
        name="left_bracket_strut",
    )
    tank.visual(
        Box((0.026, 0.018, 0.048)),
        origin=Origin(xyz=(0.0, 0.038, -0.364)),
        material=frame_gray,
        name="left_hinge_ear",
    )
    tank.visual(
        Box((0.026, 0.018, 0.048)),
        origin=Origin(xyz=(0.0, -0.038, -0.364)),
        material=frame_gray,
        name="right_hinge_ear",
    )
    tank.inertial = Inertial.from_geometry(
        Cylinder(radius=0.26, length=0.36),
        mass=9.0,
        origin=Origin(xyz=(0.0, 0.0, -0.180)),
    )

    model.articulation(
        "airframe_to_tank",
        ArticulationType.FIXED,
        parent=airframe,
        child=tank,
        origin=Origin(xyz=(0.0, 0.0, -0.025)),
    )

    spray_boom = model.part("spray_boom")
    spray_boom.visual(
        Cylinder(radius=0.012, length=0.058),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=graphite,
        name="hinge_barrel",
    )
    spray_boom.visual(
        Box((0.036, 0.028, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, -0.016)),
        material=graphite,
        name="hinge_collar",
    )
    spray_boom.visual(
        Box((0.110, 0.028, 0.020)),
        origin=Origin(xyz=(0.055, 0.0, -0.028)),
        material=graphite,
        name="boom_offset_link",
    )
    spray_boom.visual(
        Box((0.024, 0.028, 0.174)),
        origin=Origin(xyz=(0.110, 0.0, -0.115)),
        material=graphite,
        name="boom_stem",
    )
    spray_boom.visual(
        Box((0.028, 1.18, 0.020)),
        origin=Origin(xyz=(0.110, 0.0, -0.212)),
        material=spray_gray,
        name="spray_bar",
    )
    spray_boom.visual(
        Cylinder(radius=0.016, length=0.040),
        origin=Origin(xyz=(0.110, 0.585, -0.212), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=spray_gray,
        name="left_bar_endcap",
    )
    spray_boom.visual(
        Cylinder(radius=0.016, length=0.040),
        origin=Origin(xyz=(0.110, -0.585, -0.212), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=spray_gray,
        name="right_bar_endcap",
    )
    for index, y_pos in enumerate((-0.46, -0.23, 0.0, 0.23, 0.46)):
        spray_boom.visual(
            Cylinder(radius=0.0045, length=0.036),
            origin=Origin(xyz=(0.110, y_pos, -0.231)),
            material=graphite,
            name=f"nozzle_stalk_{index}",
        )
        spray_boom.visual(
            Box((0.014, 0.020, 0.006)),
            origin=Origin(xyz=(0.110, y_pos, -0.249)),
            material=nozzle_black,
            name=f"nozzle_tip_{index}",
        )
    spray_boom.inertial = Inertial.from_geometry(
        Box((0.08, 1.20, 0.40)),
        mass=1.8,
        origin=Origin(xyz=(0.0, 0.0, -0.180)),
    )

    model.articulation(
        "tank_to_spray_boom",
        ArticulationType.REVOLUTE,
        parent=tank,
        child=spray_boom,
        origin=Origin(xyz=(0.0, 0.0, -0.364)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.5,
            lower=0.0,
            upper=0.65,
        ),
    )

    for index, (arm_name, angle) in enumerate(MOTOR_LAYOUT):
        motor_x, motor_y = _motor_position(angle)
        propeller = model.part(f"propeller_{index + 1}")
        propeller.visual(
            Cylinder(radius=0.042, length=0.016),
            origin=Origin(xyz=(0.0, 0.0, 0.008)),
            material=motor_black,
            name="hub",
        )
        propeller.visual(
            Cylinder(radius=0.016, length=0.024),
            origin=Origin(xyz=(0.0, 0.0, 0.020)),
            material=graphite,
            name="hub_cap",
        )
        propeller.visual(
            prop_blade_mesh,
            origin=Origin(xyz=(0.0, 0.0, 0.018), rpy=(0.0, 0.13, 0.0)),
            material=graphite,
            name="blade_right",
        )
        propeller.visual(
            prop_blade_mesh,
            origin=Origin(xyz=(0.0, 0.0, 0.018), rpy=(0.0, 0.13, math.pi)),
            material=graphite,
            name="blade_left",
        )
        propeller.inertial = Inertial.from_geometry(
            Cylinder(radius=0.23, length=0.030),
            mass=0.15,
            origin=Origin(xyz=(0.0, 0.0, 0.015)),
        )

        model.articulation(
            f"{arm_name}_prop_spin",
            ArticulationType.CONTINUOUS,
            parent=airframe,
            child=propeller,
            origin=Origin(xyz=(motor_x, motor_y, MOTOR_TOP_Z)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=1.5,
                velocity=90.0,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    airframe = object_model.get_part("airframe")
    tank = object_model.get_part("tank")
    spray_boom = object_model.get_part("spray_boom")
    boom_joint = object_model.get_articulation("tank_to_spray_boom")
    propeller_parts = [object_model.get_part(f"propeller_{index + 1}") for index in range(6)]
    propeller_joints = [
        object_model.get_articulation(f"{arm_name}_prop_spin")
        for arm_name, _ in MOTOR_LAYOUT
    ]

    ctx.check(
        "all six propellers are present",
        len(propeller_parts) == 6 and all(part is not None for part in propeller_parts),
        details=str([part.name for part in propeller_parts]),
    )
    ctx.expect_contact(
        airframe,
        tank,
        elem_a="center_deck",
        elem_b="tank_mount_plate",
        name="tank mount plate seats on the frame deck",
    )
    ctx.expect_overlap(
        tank,
        airframe,
        axes="xy",
        min_overlap=0.18,
        name="tank stays centered under the flat frame",
    )
    ctx.expect_origin_gap(
        airframe,
        tank,
        axis="z",
        min_gap=0.02,
        name="tank hangs below the airframe",
    )
    ctx.expect_origin_gap(
        tank,
        spray_boom,
        axis="z",
        min_gap=0.06,
        name="boom hinge sits below the tank bracket",
    )

    rest_bar = ctx.part_element_world_aabb(spray_boom, elem="spray_bar")
    upper_limit = boom_joint.motion_limits.upper if boom_joint.motion_limits is not None else 1.0
    with ctx.pose({boom_joint: upper_limit}):
        folded_bar = ctx.part_element_world_aabb(spray_boom, elem="spray_bar")

    ctx.check(
        "spray boom folds upward from the deployed position",
        rest_bar is not None
        and folded_bar is not None
        and folded_bar[1][2] > rest_bar[1][2] + 0.04,
        details=f"rest={rest_bar}, folded={folded_bar}",
    )

    for index, joint in enumerate(propeller_joints, start=1):
        limits = joint.motion_limits
        ctx.check(
            f"propeller {index} uses a vertical continuous spin joint",
            joint.joint_type == ArticulationType.CONTINUOUS
            and tuple(round(v, 6) for v in joint.axis) == (0.0, 0.0, 1.0)
            and limits is not None
            and limits.lower is None
            and limits.upper is None
            and limits.velocity >= 20.0,
            details=f"type={joint.joint_type}, axis={joint.axis}, limits={limits}",
        )

    for index, propeller in enumerate(propeller_parts, start=1):
        ctx.expect_origin_gap(
            propeller,
            airframe,
            axis="z",
            min_gap=0.12,
            max_gap=0.16,
            name=f"propeller {index} sits above its motor arm",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
