from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    section_loft,
    superellipse_profile,
)

ASSETS = AssetContext.from_script(__file__)


def _axis_rpy(axis: str) -> tuple[float, float, float]:
    if axis == "x":
        return (0.0, math.pi / 2.0, 0.0)
    if axis == "y":
        return (-math.pi / 2.0, 0.0, 0.0)
    return (0.0, 0.0, 0.0)


def _xy_section(
    width: float,
    depth: float,
    z: float,
    *,
    x_shift: float = 0.0,
    y_shift: float = 0.0,
    exponent: float = 2.7,
    segments: int = 36,
) -> list[tuple[float, float, float]]:
    return [
        (x + x_shift, y + y_shift, z)
        for x, y in superellipse_profile(
            width,
            depth,
            exponent=exponent,
            segments=segments,
        )
    ]


def _save_mesh(name: str, geometry) -> object:
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _add_side_fasteners(
    part,
    *,
    prefix: str,
    face_x: float,
    outward: float,
    y_positions: tuple[float, ...],
    z_positions: tuple[float, ...],
    material,
    radius: float = 0.0045,
    length: float = 0.004,
    embed: float = 0.005,
) -> None:
    for z_index, z in enumerate(z_positions):
        for y_index, y in enumerate(y_positions):
            part.visual(
                Cylinder(radius=radius, length=length),
                origin=Origin(
                    xyz=(face_x + outward * (length / 2.0 - embed), y, z),
                    rpy=_axis_rpy("x"),
                ),
                material=material,
                name=f"{prefix}_{z_index}_{y_index}",
            )


def _build_thigh_shell():
    geometry = section_loft(
        [
            _xy_section(0.094, 0.084, -0.010, x_shift=0.000, exponent=2.9),
            _xy_section(0.074, 0.066, -0.110, x_shift=-0.004, exponent=2.6),
            _xy_section(0.080, 0.070, -0.185, x_shift=0.004, exponent=2.6),
            _xy_section(0.086, 0.078, -0.215, x_shift=0.010, exponent=2.8),
        ]
    )
    return _save_mesh("thigh_shell.obj", geometry)


def _build_shank_shell():
    geometry = section_loft(
        [
            _xy_section(0.084, 0.076, -0.012, x_shift=0.002, exponent=2.8),
            _xy_section(0.068, 0.062, -0.105, x_shift=0.014, exponent=2.5),
            _xy_section(0.074, 0.068, -0.175, x_shift=0.022, exponent=2.6),
            _xy_section(0.080, 0.074, -0.196, x_shift=0.018, exponent=2.8),
        ]
    )
    return _save_mesh("shank_shell.obj", geometry)


def _build_foot_shell():
    geometry = section_loft(
        [
            _xy_section(0.045, 0.065, -0.040, x_shift=0.048, exponent=2.6),
            _xy_section(0.065, 0.095, 0.010, x_shift=0.048, exponent=2.8),
            _xy_section(0.052, 0.090, 0.080, x_shift=0.055, exponent=2.5),
            _xy_section(0.028, 0.060, 0.125, x_shift=0.060, exponent=2.2),
        ]
    )
    geometry.rotate_y(math.pi / 2.0)
    return _save_mesh("foot_shell.obj", geometry)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="utility_robotic_leg", assets=ASSETS)

    painted_steel = model.material("painted_steel", rgba=(0.27, 0.31, 0.35, 1.0))
    raw_steel = model.material("raw_steel", rgba=(0.63, 0.65, 0.67, 1.0))
    molded_polymer = model.material("molded_polymer", rgba=(0.11, 0.12, 0.14, 1.0))
    rubber = model.material("rubber", rgba=(0.05, 0.05, 0.06, 1.0))

    thigh_shell = _build_thigh_shell()
    shank_shell = _build_shank_shell()
    foot_shell = _build_foot_shell()

    hip_mount = model.part("hip_mount")
    hip_mount.visual(
        Box((0.140, 0.115, 0.080)),
        origin=Origin(xyz=(0.000, 0.000, 0.070)),
        material=painted_steel,
        name="hip_service_block",
    )
    hip_mount.visual(
        Box((0.060, 0.040, 0.016)),
        origin=Origin(xyz=(0.040, 0.000, 0.044)),
        material=painted_steel,
        name="hip_front_gusset",
    )
    hip_mount.visual(
        Box((0.068, 0.044, 0.020)),
        origin=Origin(xyz=(-0.024, 0.000, 0.040)),
        material=painted_steel,
        name="hip_rear_gusset",
    )
    hip_mount.visual(
        Box((0.042, 0.016, 0.095)),
        origin=Origin(xyz=(0.000, 0.059, -0.004)),
        material=painted_steel,
        name="hip_lug_right",
    )
    hip_mount.visual(
        Box((0.042, 0.016, 0.095)),
        origin=Origin(xyz=(0.000, -0.059, -0.004)),
        material=painted_steel,
        name="hip_lug_left",
    )
    hip_mount.visual(
        Cylinder(radius=0.020, length=0.010),
        origin=Origin(xyz=(0.000, 0.066, 0.000), rpy=_axis_rpy("y")),
        material=raw_steel,
        name="hip_axis_cap_right",
    )
    hip_mount.visual(
        Cylinder(radius=0.020, length=0.010),
        origin=Origin(xyz=(0.000, -0.066, 0.000), rpy=_axis_rpy("y")),
        material=raw_steel,
        name="hip_axis_cap_left",
    )
    for index, xy in enumerate(
        (
            (0.046, 0.034),
            (0.046, -0.034),
            (-0.046, 0.034),
            (-0.046, -0.034),
        )
    ):
        hip_mount.visual(
            Cylinder(radius=0.005, length=0.004),
            origin=Origin(xyz=(xy[0], xy[1], 0.112)),
            material=raw_steel,
            name=f"hip_top_fastener_{index}",
        )
    hip_mount.inertial = Inertial.from_geometry(
        Box((0.160, 0.120, 0.140)),
        mass=5.0,
        origin=Origin(xyz=(0.000, 0.000, 0.040)),
    )

    thigh = model.part("thigh")
    thigh.visual(thigh_shell, material=painted_steel, name="thigh_shell")
    thigh.visual(
        Cylinder(radius=0.023, length=0.018),
        origin=Origin(xyz=(0.000, 0.042, 0.000), rpy=_axis_rpy("y")),
        material=raw_steel,
        name="thigh_hip_hub_right",
    )
    thigh.visual(
        Cylinder(radius=0.023, length=0.018),
        origin=Origin(xyz=(0.000, -0.042, 0.000), rpy=_axis_rpy("y")),
        material=raw_steel,
        name="thigh_hip_hub_left",
    )
    thigh.visual(
        Box((0.020, 0.014, 0.030)),
        origin=Origin(xyz=(0.000, 0.040, -0.010)),
        material=painted_steel,
        name="thigh_hip_web_right",
    )
    thigh.visual(
        Box((0.020, 0.014, 0.030)),
        origin=Origin(xyz=(0.000, -0.040, -0.010)),
        material=painted_steel,
        name="thigh_hip_web_left",
    )
    thigh.visual(
        Box((0.044, 0.068, 0.024)),
        origin=Origin(xyz=(0.008, 0.000, -0.018)),
        material=painted_steel,
        name="hip_joint_saddle",
    )
    thigh.visual(
        Box((0.020, 0.046, 0.118)),
        origin=Origin(xyz=(0.043, 0.000, -0.115)),
        material=molded_polymer,
        name="thigh_actuator_bay_right",
    )
    thigh.visual(
        Box((0.020, 0.046, 0.118)),
        origin=Origin(xyz=(-0.043, 0.000, -0.115)),
        material=molded_polymer,
        name="thigh_actuator_bay_left",
    )
    thigh.visual(
        Box((0.018, 0.058, 0.168)),
        origin=Origin(xyz=(0.036, 0.000, -0.132)),
        material=raw_steel,
        name="thigh_front_rib",
    )
    thigh.visual(
        Box((0.038, 0.016, 0.085)),
        origin=Origin(xyz=(0.004, 0.059, -0.255)),
        material=painted_steel,
        name="thigh_knee_lug_right",
    )
    thigh.visual(
        Box((0.038, 0.016, 0.085)),
        origin=Origin(xyz=(0.004, -0.059, -0.255)),
        material=painted_steel,
        name="thigh_knee_lug_left",
    )
    thigh.visual(
        Cylinder(radius=0.019, length=0.010),
        origin=Origin(xyz=(0.004, 0.066, -0.270), rpy=_axis_rpy("y")),
        material=raw_steel,
        name="knee_axis_cap_right",
    )
    thigh.visual(
        Cylinder(radius=0.019, length=0.010),
        origin=Origin(xyz=(0.004, -0.066, -0.270), rpy=_axis_rpy("y")),
        material=raw_steel,
        name="knee_axis_cap_left",
    )
    thigh.visual(
        Box((0.020, 0.014, 0.040)),
        origin=Origin(xyz=(0.004, 0.056, -0.248)),
        material=painted_steel,
        name="thigh_knee_web_right",
    )
    thigh.visual(
        Box((0.020, 0.014, 0.040)),
        origin=Origin(xyz=(0.004, -0.056, -0.248)),
        material=painted_steel,
        name="thigh_knee_web_left",
    )
    thigh.visual(
        Box((0.038, 0.034, 0.028)),
        origin=Origin(xyz=(0.006, 0.000, -0.218)),
        material=painted_steel,
        name="knee_joint_saddle",
    )
    _add_side_fasteners(
        thigh,
        prefix="thigh_right_fastener",
        face_x=0.053,
        outward=1.0,
        y_positions=(-0.014, 0.014),
        z_positions=(-0.065, -0.115, -0.165),
        material=raw_steel,
    )
    _add_side_fasteners(
        thigh,
        prefix="thigh_left_fastener",
        face_x=-0.053,
        outward=-1.0,
        y_positions=(-0.014, 0.014),
        z_positions=(-0.065, -0.115, -0.165),
        material=raw_steel,
    )
    thigh.inertial = Inertial.from_geometry(
        Box((0.110, 0.110, 0.300)),
        mass=6.2,
        origin=Origin(xyz=(0.004, 0.000, -0.145)),
    )

    shank = model.part("shank")
    shank.visual(shank_shell, material=painted_steel, name="shank_shell")
    shank.visual(
        Cylinder(radius=0.021, length=0.018),
        origin=Origin(xyz=(0.000, 0.042, 0.000), rpy=_axis_rpy("y")),
        material=raw_steel,
        name="shank_knee_hub_right",
    )
    shank.visual(
        Cylinder(radius=0.021, length=0.018),
        origin=Origin(xyz=(0.000, -0.042, 0.000), rpy=_axis_rpy("y")),
        material=raw_steel,
        name="shank_knee_hub_left",
    )
    shank.visual(
        Box((0.018, 0.014, 0.030)),
        origin=Origin(xyz=(0.000, 0.040, -0.010)),
        material=painted_steel,
        name="shank_knee_web_right",
    )
    shank.visual(
        Box((0.018, 0.014, 0.030)),
        origin=Origin(xyz=(0.000, -0.040, -0.010)),
        material=painted_steel,
        name="shank_knee_web_left",
    )
    shank.visual(
        Box((0.040, 0.068, 0.030)),
        origin=Origin(xyz=(0.010, 0.000, -0.018)),
        material=painted_steel,
        name="knee_receiver_saddle",
    )
    shank.visual(
        Box((0.018, 0.040, 0.102)),
        origin=Origin(xyz=(0.039, 0.000, -0.102)),
        material=molded_polymer,
        name="shank_actuator_bay_right",
    )
    shank.visual(
        Box((0.018, 0.040, 0.102)),
        origin=Origin(xyz=(-0.039, 0.000, -0.102)),
        material=molded_polymer,
        name="shank_actuator_bay_left",
    )
    shank.visual(
        Box((0.016, 0.052, 0.138)),
        origin=Origin(xyz=(0.034, 0.000, -0.112)),
        material=raw_steel,
        name="shank_front_rib",
    )
    shank.visual(
        Box((0.034, 0.014, 0.078)),
        origin=Origin(xyz=(0.000, 0.052, -0.230)),
        material=painted_steel,
        name="shank_ankle_lug_right",
    )
    shank.visual(
        Box((0.034, 0.014, 0.078)),
        origin=Origin(xyz=(0.000, -0.052, -0.230)),
        material=painted_steel,
        name="shank_ankle_lug_left",
    )
    shank.visual(
        Cylinder(radius=0.017, length=0.010),
        origin=Origin(xyz=(0.000, 0.056, -0.230), rpy=_axis_rpy("y")),
        material=raw_steel,
        name="ankle_axis_cap_right",
    )
    shank.visual(
        Cylinder(radius=0.017, length=0.010),
        origin=Origin(xyz=(0.000, -0.056, -0.230), rpy=_axis_rpy("y")),
        material=raw_steel,
        name="ankle_axis_cap_left",
    )
    shank.visual(
        Box((0.018, 0.012, 0.034)),
        origin=Origin(xyz=(0.000, 0.049, -0.212)),
        material=painted_steel,
        name="shank_ankle_web_right",
    )
    shank.visual(
        Box((0.018, 0.012, 0.034)),
        origin=Origin(xyz=(0.000, -0.049, -0.212)),
        material=painted_steel,
        name="shank_ankle_web_left",
    )
    shank.visual(
        Box((0.032, 0.032, 0.026)),
        origin=Origin(xyz=(0.010, 0.000, -0.188)),
        material=painted_steel,
        name="ankle_joint_saddle",
    )
    _add_side_fasteners(
        shank,
        prefix="shank_right_fastener",
        face_x=0.048,
        outward=1.0,
        y_positions=(-0.012, 0.012),
        z_positions=(-0.065, -0.105, -0.145),
        material=raw_steel,
        radius=0.004,
    )
    _add_side_fasteners(
        shank,
        prefix="shank_left_fastener",
        face_x=-0.048,
        outward=-1.0,
        y_positions=(-0.012, 0.012),
        z_positions=(-0.065, -0.105, -0.145),
        material=raw_steel,
        radius=0.004,
    )
    shank.inertial = Inertial.from_geometry(
        Box((0.100, 0.100, 0.270)),
        mass=5.0,
        origin=Origin(xyz=(0.012, 0.000, -0.125)),
    )

    foot = model.part("foot")
    foot.visual(foot_shell, material=painted_steel, name="foot_shell")
    foot.visual(
        Cylinder(radius=0.018, length=0.018),
        origin=Origin(xyz=(0.000, 0.036, 0.000), rpy=_axis_rpy("y")),
        material=raw_steel,
        name="foot_ankle_hub_right",
    )
    foot.visual(
        Cylinder(radius=0.018, length=0.018),
        origin=Origin(xyz=(0.000, -0.036, 0.000), rpy=_axis_rpy("y")),
        material=raw_steel,
        name="foot_ankle_hub_left",
    )
    foot.visual(
        Box((0.032, 0.050, 0.022)),
        origin=Origin(xyz=(0.000, 0.000, -0.007)),
        material=painted_steel,
        name="foot_ankle_bridge",
    )
    foot.visual(
        Box((0.052, 0.060, 0.040)),
        origin=Origin(xyz=(0.006, 0.000, -0.027)),
        material=painted_steel,
        name="foot_dorsal_housing",
    )
    foot.visual(
        Box((0.180, 0.092, 0.016)),
        origin=Origin(xyz=(0.040, 0.000, -0.086)),
        material=rubber,
        name="foot_sole",
    )
    foot.visual(
        Box((0.026, 0.082, 0.030)),
        origin=Origin(xyz=(0.124, 0.000, -0.067)),
        material=rubber,
        name="toe_bumper",
    )
    foot.visual(
        Box((0.022, 0.070, 0.028)),
        origin=Origin(xyz=(-0.045, 0.000, -0.068)),
        material=rubber,
        name="heel_pad",
    )
    _add_side_fasteners(
        foot,
        prefix="foot_right_fastener",
        face_x=0.032,
        outward=1.0,
        y_positions=(-0.015, 0.015),
        z_positions=(-0.018, -0.044),
        material=raw_steel,
        radius=0.0038,
        length=0.0035,
    )
    _add_side_fasteners(
        foot,
        prefix="foot_left_fastener",
        face_x=-0.020,
        outward=-1.0,
        y_positions=(-0.015, 0.015),
        z_positions=(-0.018, -0.044),
        material=raw_steel,
        radius=0.0038,
        length=0.0035,
    )
    foot.inertial = Inertial.from_geometry(
        Box((0.190, 0.100, 0.080)),
        mass=2.8,
        origin=Origin(xyz=(0.040, 0.000, -0.055)),
    )

    model.articulation(
        "hip_pitch",
        ArticulationType.REVOLUTE,
        parent="hip_mount",
        child="thigh",
        origin=Origin(xyz=(0.000, 0.000, 0.000)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=320.0,
            velocity=2.0,
            lower=-0.55,
            upper=0.85,
        ),
    )
    model.articulation(
        "knee_pitch",
        ArticulationType.REVOLUTE,
        parent="thigh",
        child="shank",
        origin=Origin(xyz=(0.000, 0.000, -0.270)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=280.0,
            velocity=2.4,
            lower=0.00,
            upper=1.20,
        ),
    )
    model.articulation(
        "ankle_pitch",
        ArticulationType.REVOLUTE,
        parent="shank",
        child="foot",
        origin=Origin(xyz=(0.000, 0.000, -0.230)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=160.0,
            velocity=2.6,
            lower=-0.40,
            upper=0.35,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Default broad sensor; do not remove. Tune params only if warranted.
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    # Default broad sensor; do not remove. Tune params only if warranted.
    ctx.warn_if_part_geometry_disconnected()
    # Default articulated-joint clearance gate; adapt only if the model is not articulated.
    ctx.check_articulation_overlaps(max_pose_samples=160)
    # Default broad sensor; do not remove. Tune params only if warranted.
    ctx.warn_if_overlaps(max_pose_samples=160, ignore_adjacent=True, ignore_fixed=True)

    # Add narrow allowances here when conservative QC reports acceptable cases.
    # Add prompt-specific expect_* semantic checks below; they are the main regressions.
    ctx.expect_aabb_contact("hip_mount", "thigh")
    ctx.expect_aabb_contact("thigh", "shank")
    ctx.expect_aabb_contact("shank", "foot")

    ctx.expect_aabb_gap(
        "hip_mount",
        "thigh",
        axis="y",
        max_gap=0.001,
        max_penetration=1e-5,
        positive_elem="hip_lug_right",
        negative_elem="thigh_hip_hub_right",
        name="hip_right_bearing_seat",
    )
    ctx.expect_aabb_gap(
        "thigh",
        "hip_mount",
        axis="y",
        max_gap=0.001,
        max_penetration=1e-5,
        positive_elem="thigh_hip_hub_left",
        negative_elem="hip_lug_left",
        name="hip_left_bearing_seat",
    )
    ctx.expect_aabb_gap(
        "thigh",
        "shank",
        axis="y",
        max_gap=0.001,
        max_penetration=1e-5,
        positive_elem="thigh_knee_lug_right",
        negative_elem="shank_knee_hub_right",
        name="knee_right_bearing_seat",
    )
    ctx.expect_aabb_gap(
        "shank",
        "thigh",
        axis="y",
        max_gap=0.001,
        max_penetration=1e-5,
        positive_elem="shank_knee_hub_left",
        negative_elem="thigh_knee_lug_left",
        name="knee_left_bearing_seat",
    )
    ctx.expect_aabb_gap(
        "shank",
        "foot",
        axis="y",
        max_gap=0.001,
        max_penetration=1e-5,
        positive_elem="shank_ankle_lug_right",
        negative_elem="foot_ankle_hub_right",
        name="ankle_right_bearing_seat",
    )
    ctx.expect_aabb_gap(
        "foot",
        "shank",
        axis="y",
        max_gap=0.001,
        max_penetration=1e-5,
        positive_elem="foot_ankle_hub_left",
        negative_elem="shank_ankle_lug_left",
        name="ankle_left_bearing_seat",
    )

    ctx.expect_origin_distance("thigh", "hip_mount", axes="xy", max_dist=0.015)
    ctx.expect_origin_distance("shank", "thigh", axes="xy", max_dist=0.015)
    ctx.expect_origin_distance("foot", "shank", axes="y", max_dist=0.015)

    ctx.expect_aabb_overlap("hip_mount", "thigh", axes="y", min_overlap=0.070)
    ctx.expect_aabb_overlap("thigh", "shank", axes="xy", min_overlap=0.040)
    ctx.expect_aabb_overlap("shank", "foot", axes="y", min_overlap=0.060)

    ctx.expect_joint_motion_axis(
        "hip_pitch",
        "thigh",
        world_axis="x",
        direction="negative",
        min_delta=0.025,
    )
    ctx.expect_joint_motion_axis(
        "knee_pitch",
        "shank",
        world_axis="x",
        direction="negative",
        min_delta=0.040,
    )
    ctx.expect_joint_motion_axis(
        "ankle_pitch",
        "foot",
        world_axis="x",
        direction="negative",
        min_delta=0.012,
    )

    with ctx.pose(hip_pitch=0.60):
        ctx.expect_aabb_gap(
            "hip_mount",
            "thigh",
            axis="y",
            max_gap=0.001,
            max_penetration=1e-5,
            positive_elem="hip_lug_right",
            negative_elem="thigh_hip_hub_right",
            name="hip_right_bearing_seat_flexed",
        )
        ctx.expect_aabb_gap(
            "thigh",
            "hip_mount",
            axis="y",
            max_gap=0.001,
            max_penetration=1e-5,
            positive_elem="thigh_hip_hub_left",
            negative_elem="hip_lug_left",
            name="hip_left_bearing_seat_flexed",
        )

    with ctx.pose(knee_pitch=1.00):
        ctx.expect_aabb_gap(
            "thigh",
            "shank",
            axis="y",
            max_gap=0.001,
            max_penetration=1e-5,
            positive_elem="thigh_knee_lug_right",
            negative_elem="shank_knee_hub_right",
            name="knee_right_bearing_seat_flexed",
        )
        ctx.expect_aabb_gap(
            "shank",
            "thigh",
            axis="y",
            max_gap=0.001,
            max_penetration=1e-5,
            positive_elem="shank_knee_hub_left",
            negative_elem="thigh_knee_lug_left",
            name="knee_left_bearing_seat_flexed",
        )

    with ctx.pose(ankle_pitch=0.30):
        ctx.expect_aabb_gap(
            "shank",
            "foot",
            axis="y",
            max_gap=0.001,
            max_penetration=1e-5,
            positive_elem="shank_ankle_lug_right",
            negative_elem="foot_ankle_hub_right",
            name="ankle_right_bearing_seat_flexed",
        )
        ctx.expect_aabb_gap(
            "foot",
            "shank",
            axis="y",
            max_gap=0.001,
            max_penetration=1e-5,
            positive_elem="foot_ankle_hub_left",
            negative_elem="shank_ankle_lug_left",
            name="ankle_left_bearing_seat_flexed",
        )

    with ctx.pose(hip_pitch=0.45, knee_pitch=0.95, ankle_pitch=-0.20):
        ctx.expect_aabb_contact("shank", "foot")
        ctx.expect_aabb_overlap("shank", "foot", axes="y", min_overlap=0.055)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
