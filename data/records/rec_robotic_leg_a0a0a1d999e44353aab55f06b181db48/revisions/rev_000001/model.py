from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="retrofit_robotic_leg")

    shell_paint = model.material("shell_paint", rgba=(0.39, 0.44, 0.36, 1.0))
    service_panel = model.material("service_panel", rgba=(0.58, 0.57, 0.53, 1.0))
    structure = model.material("structure", rgba=(0.36, 0.37, 0.40, 1.0))
    fastener = model.material("fastener", rgba=(0.72, 0.73, 0.75, 1.0))
    rubber = model.material("rubber", rgba=(0.10, 0.10, 0.11, 1.0))

    def add_box(
        part,
        size,
        xyz,
        *,
        rpy=(0.0, 0.0, 0.0),
        material=None,
        name=None,
    ):
        part.visual(
            Box(size),
            origin=Origin(xyz=xyz, rpy=rpy),
            material=material,
            name=name,
        )

    def add_cylinder(
        part,
        radius,
        length,
        xyz,
        *,
        rpy=(0.0, 0.0, 0.0),
        material=None,
        name=None,
    ):
        part.visual(
            Cylinder(radius=radius, length=length),
            origin=Origin(xyz=xyz, rpy=rpy),
            material=material,
            name=name,
        )

    def add_bolts(part, *, axis, positions, radius, length, material, prefix):
        if axis == "x":
            rpy = (0.0, pi / 2.0, 0.0)
        elif axis == "y":
            rpy = (pi / 2.0, 0.0, 0.0)
        else:
            rpy = (0.0, 0.0, 0.0)
        for index, xyz in enumerate(positions, start=1):
            add_cylinder(
                part,
                radius,
                length,
                xyz,
                rpy=rpy,
                material=material,
                name=f"{prefix}_{index}",
            )

    hip_adapter = model.part("hip_adapter")
    add_box(
        hip_adapter,
        (0.16, 0.14, 0.022),
        (0.0, 0.0, 0.083),
        material=structure,
        name="top_adapter_plate",
    )
    add_box(
        hip_adapter,
        (0.082, 0.060, 0.050),
        (0.0, 0.0, 0.059),
        material=structure,
        name="adapter_spine",
    )
    add_box(
        hip_adapter,
        (0.060, 0.014, 0.120),
        (0.0, 0.053, -0.010),
        material=structure,
        name="left_hip_yoke",
    )
    add_box(
        hip_adapter,
        (0.060, 0.014, 0.120),
        (0.0, -0.053, -0.010),
        material=structure,
        name="right_hip_yoke",
    )
    add_box(
        hip_adapter,
        (0.028, 0.100, 0.016),
        (0.036, 0.0, 0.058),
        material=structure,
        name="front_crossmember",
    )
    add_box(
        hip_adapter,
        (0.026, 0.092, 0.016),
        (-0.036, 0.0, 0.056),
        material=structure,
        name="rear_crossmember",
    )
    add_box(
        hip_adapter,
        (0.026, 0.012, 0.088),
        (0.034, 0.060, 0.004),
        rpy=(0.0, 0.42, 0.0),
        material=structure,
        name="left_front_brace",
    )
    add_box(
        hip_adapter,
        (0.026, 0.012, 0.088),
        (0.034, -0.060, 0.004),
        rpy=(0.0, 0.42, 0.0),
        material=structure,
        name="right_front_brace",
    )
    add_box(
        hip_adapter,
        (0.024, 0.012, 0.080),
        (-0.031, 0.060, 0.006),
        rpy=(0.0, -0.36, 0.0),
        material=structure,
        name="left_rear_brace",
    )
    add_box(
        hip_adapter,
        (0.024, 0.012, 0.080),
        (-0.031, -0.060, 0.006),
        rpy=(0.0, -0.36, 0.0),
        material=structure,
        name="right_rear_brace",
    )
    add_bolts(
        hip_adapter,
        axis="z",
        positions=[
            (0.050, 0.048, 0.095),
            (0.050, -0.048, 0.095),
            (-0.050, 0.048, 0.095),
            (-0.050, -0.048, 0.095),
            (0.0, 0.048, 0.095),
            (0.0, -0.048, 0.095),
        ],
        radius=0.005,
        length=0.016,
        material=fastener,
        prefix="mount_bolt",
    )
    hip_adapter.inertial = Inertial.from_geometry(
        Box((0.16, 0.14, 0.16)),
        mass=4.8,
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
    )

    thigh = model.part("thigh")
    add_cylinder(
        thigh,
        0.024,
        0.018,
        (0.0, 0.037, 0.0),
        rpy=(pi / 2.0, 0.0, 0.0),
        material=structure,
        name="left_hip_hub",
    )
    add_cylinder(
        thigh,
        0.024,
        0.018,
        (0.0, -0.037, 0.0),
        rpy=(pi / 2.0, 0.0, 0.0),
        material=structure,
        name="right_hip_hub",
    )
    add_box(
        thigh,
        (0.092, 0.074, 0.070),
        (0.0, 0.0, -0.055),
        material=structure,
        name="upper_shoulder_block",
    )
    add_box(
        thigh,
        (0.050, 0.010, 0.210),
        (0.0, 0.031, -0.195),
        material=shell_paint,
        name="left_shell_rail",
    )
    add_box(
        thigh,
        (0.050, 0.010, 0.210),
        (0.0, -0.031, -0.195),
        material=shell_paint,
        name="right_shell_rail",
    )
    add_box(
        thigh,
        (0.014, 0.054, 0.210),
        (0.034, 0.0, -0.195),
        material=shell_paint,
        name="front_spine",
    )
    add_box(
        thigh,
        (0.010, 0.062, 0.220),
        (-0.036, 0.0, -0.205),
        material=service_panel,
        name="rear_service_hatch",
    )
    add_cylinder(
        thigh,
        0.008,
        0.230,
        (0.033, 0.018, -0.225),
        material=structure,
        name="left_actuator_bay",
    )
    add_cylinder(
        thigh,
        0.008,
        0.230,
        (0.033, -0.018, -0.225),
        material=structure,
        name="right_actuator_bay",
    )
    add_box(
        thigh,
        (0.010, 0.046, 0.070),
        (0.030, 0.0, -0.120),
        material=service_panel,
        name="actuator_service_band",
    )
    add_box(
        thigh,
        (0.018, 0.010, 0.120),
        (0.017, 0.027, -0.125),
        rpy=(0.0, 0.42, 0.0),
        material=structure,
        name="left_front_gusset",
    )
    add_box(
        thigh,
        (0.018, 0.010, 0.120),
        (0.017, -0.027, -0.125),
        rpy=(0.0, 0.42, 0.0),
        material=structure,
        name="right_front_gusset",
    )
    add_box(
        thigh,
        (0.016, 0.010, 0.110),
        (-0.014, 0.027, -0.140),
        rpy=(0.0, -0.34, 0.0),
        material=structure,
        name="left_rear_gusset",
    )
    add_box(
        thigh,
        (0.016, 0.010, 0.110),
        (-0.014, -0.027, -0.140),
        rpy=(0.0, -0.34, 0.0),
        material=structure,
        name="right_rear_gusset",
    )
    add_box(
        thigh,
        (0.040, 0.010, 0.118),
        (0.0, 0.050, -0.332),
        material=structure,
        name="left_knee_clevis",
    )
    add_box(
        thigh,
        (0.040, 0.010, 0.118),
        (0.0, -0.050, -0.332),
        material=structure,
        name="right_knee_clevis",
    )
    add_box(
        thigh,
        (0.024, 0.010, 0.150),
        (0.0, 0.050, -0.255),
        material=structure,
        name="left_knee_side_web",
    )
    add_box(
        thigh,
        (0.024, 0.010, 0.150),
        (0.0, -0.050, -0.255),
        material=structure,
        name="right_knee_side_web",
    )
    add_box(
        thigh,
        (0.010, 0.020, 0.080),
        (0.008, 0.040, -0.220),
        material=structure,
        name="left_knee_bridge",
    )
    add_box(
        thigh,
        (0.010, 0.020, 0.080),
        (0.008, -0.040, -0.220),
        material=structure,
        name="right_knee_bridge",
    )
    add_box(
        thigh,
        (0.016, 0.088, 0.022),
        (0.032, 0.0, -0.302),
        material=structure,
        name="knee_front_adapter",
    )
    add_box(
        thigh,
        (0.010, 0.072, 0.024),
        (-0.032, 0.0, -0.286),
        material=structure,
        name="knee_rear_adapter",
    )
    add_box(
        thigh,
        (0.014, 0.012, 0.040),
        (0.024, 0.053, -0.332),
        material=structure,
        name="left_front_adapter_tab",
    )
    add_box(
        thigh,
        (0.014, 0.012, 0.040),
        (0.024, -0.053, -0.332),
        material=structure,
        name="right_front_adapter_tab",
    )
    add_bolts(
        thigh,
        axis="x",
        positions=[
            (-0.041, 0.022, -0.150),
            (-0.041, -0.022, -0.150),
            (-0.041, 0.022, -0.215),
            (-0.041, -0.022, -0.215),
            (-0.041, 0.022, -0.280),
            (-0.041, -0.022, -0.280),
        ],
        radius=0.0038,
        length=0.010,
        material=fastener,
        prefix="thigh_hatch_bolt",
    )
    add_bolts(
        thigh,
        axis="y",
        positions=[
            (0.032, 0.048, -0.312),
            (0.032, -0.048, -0.312),
            (0.032, 0.048, -0.318),
            (0.032, -0.048, -0.318),
        ],
        radius=0.0032,
        length=0.014,
        material=fastener,
        prefix="knee_adapter_bolt",
    )
    thigh.inertial = Inertial.from_geometry(
        Box((0.12, 0.10, 0.40)),
        mass=6.2,
        origin=Origin(xyz=(0.0, 0.0, -0.200)),
    )

    shin = model.part("shin")
    add_cylinder(
        shin,
        0.020,
        0.092,
        (0.0, 0.0, 0.0),
        rpy=(pi / 2.0, 0.0, 0.0),
        material=structure,
        name="knee_barrel",
    )
    add_box(
        shin,
        (0.036, 0.040, 0.072),
        (0.0, 0.0, -0.054),
        material=structure,
        name="upper_knee_block",
    )
    add_box(
        shin,
        (0.032, 0.022, 0.220),
        (0.0, 0.020, -0.190),
        material=shell_paint,
        name="left_shin_rail",
    )
    add_box(
        shin,
        (0.032, 0.022, 0.220),
        (0.0, -0.020, -0.190),
        material=shell_paint,
        name="right_shin_rail",
    )
    add_box(
        shin,
        (0.014, 0.052, 0.220),
        (0.030, 0.0, -0.190),
        material=shell_paint,
        name="shin_front_spine",
    )
    add_box(
        shin,
        (0.014, 0.054, 0.180),
        (-0.012, 0.0, -0.170),
        material=structure,
        name="shin_rear_spine",
    )
    add_box(
        shin,
        (0.016, 0.056, 0.150),
        (-0.018, 0.0, -0.225),
        material=service_panel,
        name="side_service_hatch",
    )
    add_box(
        shin,
        (0.016, 0.048, 0.230),
        (0.0, 0.0, -0.250),
        material=structure,
        name="lower_keel",
    )
    add_box(
        shin,
        (0.042, 0.010, 0.072),
        (0.0, 0.045, -0.330),
        material=structure,
        name="left_ankle_cradle",
    )
    add_box(
        shin,
        (0.042, 0.010, 0.072),
        (0.0, -0.045, -0.330),
        material=structure,
        name="right_ankle_cradle",
    )
    add_box(
        shin,
        (0.024, 0.022, 0.048),
        (0.0, 0.034, -0.270),
        material=structure,
        name="left_ankle_side_web",
    )
    add_box(
        shin,
        (0.024, 0.022, 0.048),
        (0.0, -0.034, -0.270),
        material=structure,
        name="right_ankle_side_web",
    )
    add_box(
        shin,
        (0.010, 0.080, 0.020),
        (0.030, 0.0, -0.350),
        material=structure,
        name="ankle_front_adapter",
    )
    add_box(
        shin,
        (0.010, 0.082, 0.020),
        (-0.030, 0.0, -0.346),
        material=structure,
        name="ankle_rear_adapter",
    )
    add_box(
        shin,
        (0.014, 0.010, 0.038),
        (0.022, 0.045, -0.338),
        material=structure,
        name="left_front_ankle_tab",
    )
    add_box(
        shin,
        (0.014, 0.010, 0.038),
        (0.022, -0.045, -0.338),
        material=structure,
        name="right_front_ankle_tab",
    )
    add_box(
        shin,
        (0.014, 0.010, 0.034),
        (-0.022, 0.045, -0.334),
        material=structure,
        name="left_rear_ankle_tab",
    )
    add_box(
        shin,
        (0.014, 0.010, 0.034),
        (-0.022, -0.045, -0.334),
        material=structure,
        name="right_rear_ankle_tab",
    )
    add_box(
        shin,
        (0.014, 0.018, 0.090),
        (0.014, 0.024, -0.122),
        rpy=(0.0, 0.22, 0.0),
        material=structure,
        name="shin_left_front_brace",
    )
    add_box(
        shin,
        (0.014, 0.018, 0.090),
        (0.014, -0.024, -0.122),
        rpy=(0.0, 0.22, 0.0),
        material=structure,
        name="shin_right_front_brace",
    )
    add_box(
        shin,
        (0.012, 0.018, 0.088),
        (-0.012, 0.024, -0.132),
        rpy=(0.0, -0.20, 0.0),
        material=structure,
        name="shin_left_rear_brace",
    )
    add_box(
        shin,
        (0.012, 0.018, 0.088),
        (-0.012, -0.024, -0.132),
        rpy=(0.0, -0.20, 0.0),
        material=structure,
        name="shin_right_rear_brace",
    )
    add_bolts(
        shin,
        axis="x",
        positions=[
            (-0.030, 0.022, -0.180),
            (-0.030, -0.022, -0.180),
            (-0.030, 0.022, -0.225),
            (-0.030, -0.022, -0.225),
        ],
        radius=0.0035,
        length=0.010,
        material=fastener,
        prefix="shin_hatch_bolt",
    )
    add_bolts(
        shin,
        axis="y",
        positions=[
            (0.030, 0.048, -0.350),
            (0.030, -0.048, -0.350),
            (-0.030, 0.048, -0.346),
            (-0.030, -0.048, -0.346),
        ],
        radius=0.0032,
        length=0.014,
        material=fastener,
        prefix="ankle_adapter_bolt",
    )
    shin.inertial = Inertial.from_geometry(
        Box((0.10, 0.09, 0.37)),
        mass=5.1,
        origin=Origin(xyz=(0.0, 0.0, -0.190)),
    )

    foot = model.part("foot")
    add_cylinder(
        foot,
        0.022,
        0.012,
        (0.0, 0.034, 0.0),
        rpy=(pi / 2.0, 0.0, 0.0),
        material=structure,
        name="left_ankle_hub",
    )
    add_cylinder(
        foot,
        0.022,
        0.012,
        (0.0, -0.034, 0.0),
        rpy=(pi / 2.0, 0.0, 0.0),
        material=structure,
        name="right_ankle_hub",
    )
    add_box(
        foot,
        (0.012, 0.020, 0.030),
        (0.0, 0.034, -0.025),
        material=structure,
        name="left_ankle_hub_support",
    )
    add_box(
        foot,
        (0.012, 0.020, 0.030),
        (0.0, -0.034, -0.025),
        material=structure,
        name="right_ankle_hub_support",
    )
    add_box(
        foot,
        (0.012, 0.060, 0.100),
        (0.016, 0.0, -0.060),
        material=structure,
        name="front_ankle_web",
    )
    add_box(
        foot,
        (0.012, 0.052, 0.085),
        (-0.028, 0.0, -0.066),
        material=structure,
        name="rear_ankle_web",
    )
    add_box(
        foot,
        (0.180, 0.100, 0.040),
        (0.040, 0.0, -0.120),
        material=shell_paint,
        name="midfoot_shell",
    )
    add_box(
        foot,
        (0.070, 0.090, 0.050),
        (-0.065, 0.0, -0.118),
        material=structure,
        name="heel_block",
    )
    add_box(
        foot,
        (0.090, 0.090, 0.032),
        (0.135, 0.0, -0.102),
        material=shell_paint,
        name="toe_shell",
    )
    add_box(
        foot,
        (0.260, 0.110, 0.016),
        (0.035, 0.0, -0.148),
        material=rubber,
        name="sole_pad",
    )
    add_box(
        foot,
        (0.060, 0.010, 0.050),
        (0.000, 0.031, -0.086),
        material=structure,
        name="left_foot_bracket",
    )
    add_box(
        foot,
        (0.060, 0.010, 0.050),
        (0.000, -0.031, -0.086),
        material=structure,
        name="right_foot_bracket",
    )
    add_bolts(
        foot,
        axis="z",
        positions=[
            (-0.020, 0.026, -0.096),
            (-0.020, -0.026, -0.096),
            (0.028, 0.026, -0.096),
            (0.028, -0.026, -0.096),
        ],
        radius=0.0032,
        length=0.010,
        material=fastener,
        prefix="foot_bracket_bolt",
    )
    foot.inertial = Inertial.from_geometry(
        Box((0.26, 0.11, 0.13)),
        mass=3.7,
        origin=Origin(xyz=(0.030, 0.0, -0.100)),
    )

    model.articulation(
        "hip_pitch",
        ArticulationType.REVOLUTE,
        parent=hip_adapter,
        child=thigh,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=240.0,
            velocity=2.0,
            lower=-0.80,
            upper=1.10,
        ),
    )
    model.articulation(
        "knee_pitch",
        ArticulationType.REVOLUTE,
        parent=thigh,
        child=shin,
        origin=Origin(xyz=(0.0, 0.0, -0.340)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=260.0,
            velocity=2.2,
            lower=0.0,
            upper=2.15,
        ),
    )
    model.articulation(
        "ankle_pitch",
        ArticulationType.REVOLUTE,
        parent=shin,
        child=foot,
        origin=Origin(xyz=(0.0, 0.0, -0.338)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=140.0,
            velocity=2.5,
            lower=-0.55,
            upper=0.45,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    hip_adapter = object_model.get_part("hip_adapter")
    thigh = object_model.get_part("thigh")
    shin = object_model.get_part("shin")
    foot = object_model.get_part("foot")
    hip_joint = object_model.get_articulation("hip_pitch")
    knee_joint = object_model.get_articulation("knee_pitch")
    ankle_joint = object_model.get_articulation("ankle_pitch")

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

    ctx.check(
        "single_root_hip_adapter",
        [part.name for part in object_model.root_parts()] == ["hip_adapter"],
        details="The leg should resolve to one rooted serial mechanism.",
    )
    ctx.check(
        "joint_limits_read_as_leg_ranges",
        (
            hip_joint.motion_limits is not None
            and knee_joint.motion_limits is not None
            and ankle_joint.motion_limits is not None
            and hip_joint.motion_limits.lower is not None
            and hip_joint.motion_limits.upper is not None
            and knee_joint.motion_limits.lower == 0.0
            and knee_joint.motion_limits.upper is not None
            and knee_joint.motion_limits.upper >= 2.0
            and ankle_joint.motion_limits.lower is not None
            and ankle_joint.motion_limits.upper is not None
            and hip_joint.motion_limits.lower < 0.0 < hip_joint.motion_limits.upper
            and ankle_joint.motion_limits.lower < 0.0 < ankle_joint.motion_limits.upper
        ),
        details="Hip, knee, and ankle limits should resemble plausible serial leg motion.",
    )

    ctx.expect_contact(hip_adapter, thigh, name="hip_housing_is_supported")
    ctx.expect_contact(thigh, shin, name="knee_housing_is_supported")
    ctx.expect_contact(shin, foot, name="ankle_housing_is_supported")
    ctx.expect_origin_gap(
        hip_adapter,
        foot,
        axis="z",
        min_gap=0.65,
        name="rest_pose_has_full_leg_length",
    )

    with ctx.pose({hip_joint: 0.60}):
        ctx.expect_origin_gap(
            shin,
            hip_adapter,
            axis="x",
            min_gap=0.17,
            name="positive_hip_pitch_swings_knee_forward",
        )

    with ctx.pose({knee_joint: 1.00}):
        ctx.expect_origin_gap(
            hip_adapter,
            foot,
            axis="x",
            min_gap=0.18,
            name="positive_knee_pitch_tucks_foot_rearward",
        )

    neutral_toe = ctx.part_element_world_aabb(foot, elem="toe_shell")
    with ctx.pose({ankle_joint: 0.35}):
        raised_toe = ctx.part_element_world_aabb(foot, elem="toe_shell")
    toe_raises = (
        neutral_toe is not None
        and raised_toe is not None
        and raised_toe[1][2] > neutral_toe[1][2] + 0.03
    )
    ctx.check(
        "positive_ankle_pitch_raises_toe",
        toe_raises,
        details="Positive ankle motion should dorsiflex the foot and lift the toe shell.",
    )

    with ctx.pose({hip_joint: 0.35, knee_joint: 0.85, ankle_joint: 0.10}):
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_in_flexed_pose")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
