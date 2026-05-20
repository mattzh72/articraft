from __future__ import annotations

# The harness only exposes the editable block to the model.
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
    ExtrudeGeometry,
    Inertial,
    LatheGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root

Y_CYLINDER_RPY = (-math.pi / 2.0, 0.0, 0.0)
X_CYLINDER_RPY = (0.0, math.pi / 2.0, 0.0)


def _make_material(name: str, rgba: tuple[float, float, float, float]) -> Material:
    try:
        return Material(name=name, color=rgba)
    except TypeError:
        return Material(name=name, rgba=rgba)


def _arm_cylinder_rpy(angle: float) -> tuple[float, float, float]:
    return (0.0, math.pi / 2.0 - angle, 0.0)


def _arm_box_rpy(angle: float) -> tuple[float, float, float]:
    return (0.0, -angle, 0.0)


def _add_twin_arm_segment(
    part,
    *,
    length: float,
    angle: float,
    rail_radius: float,
    rail_offset_y: float,
    hub_radius: float,
    hub_length: float,
    body_material: Material,
    joint_material: Material,
) -> tuple[float, float]:
    end_x = length * math.cos(angle)
    end_z = length * math.sin(angle)
    arm_cylinder_rpy = _arm_cylinder_rpy(angle)
    arm_box_rpy = _arm_box_rpy(angle)

    part.visual(
        Cylinder(radius=hub_radius, length=hub_length),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=Y_CYLINDER_RPY),
        material=joint_material,
        name="rear_hub",
    )
    part.visual(
        Cylinder(radius=hub_radius + 0.0025, length=0.010),
        origin=Origin(xyz=(0.0, rail_offset_y + 0.004, 0.0), rpy=Y_CYLINDER_RPY),
        material=joint_material,
        name="rear_cap_pos",
    )
    part.visual(
        Cylinder(radius=hub_radius + 0.0025, length=0.010),
        origin=Origin(xyz=(0.0, -(rail_offset_y + 0.004), 0.0), rpy=Y_CYLINDER_RPY),
        material=joint_material,
        name="rear_cap_neg",
    )

    for rail_sign, rail_name in ((1.0, "left"), (-1.0, "right")):
        y = rail_sign * rail_offset_y
        part.visual(
            Cylinder(radius=rail_radius, length=length),
            origin=Origin(
                xyz=(0.5 * end_x, y, 0.5 * end_z),
                rpy=arm_cylinder_rpy,
            ),
            material=body_material,
            name=f"{rail_name}_rail",
        )
        for sleeve_pos, sleeve_name in ((0.028, "rear"), (length - 0.028, "front")):
            part.visual(
                Cylinder(radius=rail_radius + 0.0012, length=0.015),
                origin=Origin(
                    xyz=(
                        sleeve_pos * math.cos(angle),
                        y,
                        sleeve_pos * math.sin(angle),
                    ),
                    rpy=arm_cylinder_rpy,
                ),
                material=joint_material,
                name=f"{rail_name}_{sleeve_name}_sleeve",
            )

    rod_length = max(length - 0.020, 0.040)
    rod_mid = 0.5 * rod_length
    part.visual(
        Cylinder(radius=0.0026, length=rod_length),
        origin=Origin(
            xyz=(rod_mid * math.cos(angle), 0.0, rod_mid * math.sin(angle) - 0.008),
            rpy=arm_cylinder_rpy,
        ),
        material=joint_material,
        name="tension_rod",
    )

    for bridge_pos, bridge_name in ((0.34 * length, "rear"), (0.68 * length, "front")):
        part.visual(
            Box((0.014, 2.0 * rail_offset_y + 0.016, 0.010)),
            origin=Origin(
                xyz=(
                    bridge_pos * math.cos(angle),
                    0.0,
                    bridge_pos * math.sin(angle) - 0.002,
                ),
                rpy=arm_box_rpy,
            ),
            material=joint_material,
            name=f"{bridge_name}_bridge",
        )

    part.visual(
        Cylinder(radius=hub_radius, length=hub_length + 0.006),
        origin=Origin(xyz=(end_x, 0.0, end_z), rpy=Y_CYLINDER_RPY),
        material=joint_material,
        name="front_hub",
    )
    part.visual(
        Cylinder(radius=hub_radius + 0.0022, length=0.010),
        origin=Origin(
            xyz=(end_x, rail_offset_y + 0.004, end_z),
            rpy=Y_CYLINDER_RPY,
        ),
        material=joint_material,
        name="front_cap_pos",
    )
    part.visual(
        Cylinder(radius=hub_radius + 0.0022, length=0.010),
        origin=Origin(
            xyz=(end_x, -(rail_offset_y + 0.004), end_z),
            rpy=Y_CYLINDER_RPY,
        ),
        material=joint_material,
        name="front_cap_neg",
    )

    return end_x, end_z


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="refined_task_lamp", assets=ASSETS)

    graphite = _make_material("graphite_paint", (0.18, 0.19, 0.21, 1.0))
    satin_aluminum = _make_material("satin_aluminum", (0.74, 0.76, 0.79, 1.0))
    brushed_steel = _make_material("brushed_steel", (0.59, 0.61, 0.64, 1.0))
    rubber_black = _make_material("rubber_black", (0.08, 0.08, 0.09, 1.0))
    diffuser = _make_material("diffuser", (0.97, 0.96, 0.90, 0.38))
    warm_led = _make_material("warm_led", (1.00, 0.93, 0.78, 0.55))
    model.materials.extend(
        [graphite, satin_aluminum, brushed_steel, rubber_black, diffuser, warm_led]
    )

    base_radius = 0.105
    base_thickness = 0.022
    shoulder_x = -0.034
    shoulder_z = 0.068
    lower_length = 0.165
    upper_length = 0.145
    lower_style_angle = math.radians(62.0)
    upper_style_angle = math.radians(54.0)

    base_profile = [
        (0.0, -0.0095),
        (0.082, -0.0110),
        (0.098, -0.0100),
        (0.105, -0.0045),
        (0.105, 0.0065),
        (0.094, 0.0100),
        (0.070, 0.0110),
        (0.0, 0.0110),
    ]
    base_mesh = mesh_from_geometry(
        LatheGeometry(base_profile, segments=64),
        ASSETS.mesh_path("task_lamp_base_shell.obj"),
    )

    head_profile = rounded_rect_profile(0.030, 0.050, radius=0.009, corner_segments=10)
    head_shell_geom = ExtrudeGeometry(
        head_profile,
        0.086,
        cap=True,
        center=True,
        closed=True,
    )
    head_shell_geom.rotate_y(math.pi / 2.0)
    head_mesh = mesh_from_geometry(
        head_shell_geom,
        ASSETS.mesh_path("task_lamp_head_shell.obj"),
    )

    base = model.part("base")
    base.visual(
        base_mesh,
        origin=Origin(xyz=(0.0, 0.0, base_thickness / 2.0)),
        material=graphite,
        name="base_shell",
    )
    base.visual(
        Cylinder(radius=0.088, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=rubber_black,
        name="base_pad",
    )
    base.visual(
        Cylinder(radius=0.060, length=0.004),
        origin=Origin(xyz=(-0.010, 0.0, base_thickness - 0.0005)),
        material=brushed_steel,
        name="trim_disc",
    )
    base.visual(
        Box((0.050, 0.052, 0.046)),
        origin=Origin(xyz=(shoulder_x - 0.016, 0.0, 0.045)),
        material=graphite,
        name="pedestal_block",
    )
    base.visual(
        Box((0.024, 0.070, 0.012)),
        origin=Origin(xyz=(shoulder_x - 0.014, 0.0, 0.028)),
        material=brushed_steel,
        name="pedestal_gusset",
    )
    base.visual(
        Cylinder(radius=0.014, length=0.054),
        origin=Origin(xyz=(shoulder_x, 0.0, shoulder_z), rpy=Y_CYLINDER_RPY),
        material=brushed_steel,
        name="shoulder_barrel",
    )
    base.visual(
        Cylinder(radius=0.017, length=0.012),
        origin=Origin(xyz=(shoulder_x - 0.012, 0.0, shoulder_z), rpy=Y_CYLINDER_RPY),
        material=brushed_steel,
        name="shoulder_cap",
    )
    base.inertial = Inertial.from_geometry(
        Cylinder(radius=0.095, length=base_thickness),
        mass=2.8,
        origin=Origin(xyz=(0.0, 0.0, base_thickness / 2.0)),
    )

    lower_arm = model.part("lower_arm")
    lower_end_x, lower_end_z = _add_twin_arm_segment(
        lower_arm,
        length=lower_length,
        angle=lower_style_angle,
        rail_radius=0.0062,
        rail_offset_y=0.016,
        hub_radius=0.0105,
        hub_length=0.040,
        body_material=satin_aluminum,
        joint_material=brushed_steel,
    )
    lower_arm.visual(
        Box((0.020, 0.048, 0.014)),
        origin=Origin(
            xyz=(
                0.52 * lower_end_x,
                0.0,
                0.52 * lower_end_z - 0.006,
            ),
            rpy=_arm_box_rpy(lower_style_angle),
        ),
        material=graphite,
        name="center_stiffener",
    )
    lower_arm.inertial = Inertial.from_geometry(
        Box((0.145, 0.050, 0.024)),
        mass=0.45,
        origin=Origin(xyz=(0.5 * lower_end_x, 0.0, 0.5 * lower_end_z)),
    )

    upper_arm = model.part("upper_arm")
    upper_end_x, upper_end_z = _add_twin_arm_segment(
        upper_arm,
        length=upper_length,
        angle=upper_style_angle,
        rail_radius=0.0054,
        rail_offset_y=0.014,
        hub_radius=0.0095,
        hub_length=0.044,
        body_material=satin_aluminum,
        joint_material=brushed_steel,
    )
    upper_arm.visual(
        Box((0.018, 0.058, 0.018)),
        origin=Origin(
            xyz=(upper_end_x - 0.010, 0.0, upper_end_z - 0.002),
            rpy=_arm_box_rpy(upper_style_angle),
        ),
        material=graphite,
        name="head_yoke_block",
    )
    upper_arm.inertial = Inertial.from_geometry(
        Box((0.125, 0.046, 0.022)),
        mass=0.35,
        origin=Origin(xyz=(0.5 * upper_end_x, 0.0, 0.5 * upper_end_z)),
    )

    head = model.part("head")
    head.visual(
        Cylinder(radius=0.009, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=Y_CYLINDER_RPY),
        material=brushed_steel,
        name="tilt_trunnion",
    )
    head.visual(
        Cylinder(radius=0.013, length=0.006),
        origin=Origin(xyz=(0.0, 0.021, 0.0), rpy=Y_CYLINDER_RPY),
        material=brushed_steel,
        name="tilt_cap_pos",
    )
    head.visual(
        Cylinder(radius=0.013, length=0.006),
        origin=Origin(xyz=(0.0, -0.021, 0.0), rpy=Y_CYLINDER_RPY),
        material=brushed_steel,
        name="tilt_cap_neg",
    )
    head.visual(
        head_mesh,
        origin=Origin(xyz=(0.040, 0.0, 0.0)),
        material=graphite,
        name="head_shell",
    )
    head.visual(
        Cylinder(radius=0.017, length=0.014),
        origin=Origin(xyz=(0.006, 0.0, 0.0), rpy=X_CYLINDER_RPY),
        material=graphite,
        name="rear_neck",
    )
    head.visual(
        Cylinder(radius=0.027, length=0.010),
        origin=Origin(xyz=(0.084, 0.0, 0.0), rpy=X_CYLINDER_RPY),
        material=brushed_steel,
        name="bezel",
    )
    head.visual(
        Cylinder(radius=0.022, length=0.0035),
        origin=Origin(xyz=(0.0875, 0.0, 0.0), rpy=X_CYLINDER_RPY),
        material=diffuser,
        name="lens",
    )
    head.visual(
        Cylinder(radius=0.018, length=0.0015),
        origin=Origin(xyz=(0.0885, 0.0, 0.0), rpy=X_CYLINDER_RPY),
        material=warm_led,
        name="emitter",
    )
    for fin_x in (0.022, 0.036, 0.050, 0.064):
        head.visual(
            Box((0.008, 0.036, 0.005)),
            origin=Origin(xyz=(fin_x, 0.0, 0.0165)),
            material=brushed_steel,
            name=f"cooling_fin_{int(fin_x * 1000)}",
        )
    head.visual(
        Box((0.012, 0.020, 0.007)),
        origin=Origin(xyz=(0.020, 0.0, 0.016)),
        material=rubber_black,
        name="switch",
    )
    head.inertial = Inertial.from_geometry(
        Box((0.094, 0.052, 0.040)),
        mass=0.40,
        origin=Origin(xyz=(0.045, 0.0, 0.0)),
    )

    model.articulation(
        "shoulder_pitch",
        ArticulationType.REVOLUTE,
        parent="base",
        child="lower_arm",
        origin=Origin(xyz=(shoulder_x, 0.0, shoulder_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=2.0,
            lower=-0.55,
            upper=0.70,
        ),
    )
    model.articulation(
        "elbow_pitch",
        ArticulationType.REVOLUTE,
        parent="lower_arm",
        child="upper_arm",
        origin=Origin(xyz=(lower_end_x, 0.0, lower_end_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=14.0,
            velocity=2.2,
            lower=-0.30,
            upper=0.95,
        ),
    )
    model.articulation(
        "head_tilt",
        ArticulationType.REVOLUTE,
        parent="upper_arm",
        child="head",
        origin=Origin(xyz=(upper_end_x, 0.0, upper_end_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=3.0,
            lower=-1.05,
            upper=0.55,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.01)
    ctx.fail_if_part_contains_disconnected_geometry_islands(use="visual")

    ctx.allow_overlap(
        "base",
        "lower_arm",
        reason="shoulder barrel and inner arm hub share a concentric hinge envelope",
    )
    ctx.allow_overlap(
        "lower_arm",
        "upper_arm",
        reason="elbow sleeves and pivot hardware nest tightly at the hinge",
    )
    ctx.allow_overlap(
        "upper_arm",
        "head",
        reason="lamp head trunnion sits inside the upper-arm tilt yoke",
    )
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=192,
        overlap_tol=0.004,
        overlap_volume_tol=0.0,
    )

    ctx.expect_aabb_overlap("lower_arm", "base", axes="xy", min_overlap=0.03)
    ctx.expect_origin_distance("lower_arm", "base", axes="xy", max_dist=0.09)
    ctx.expect_origin_distance("upper_arm", "base", axes="xy", max_dist=0.14)
    ctx.expect_origin_distance("head", "base", axes="xy", max_dist=0.18)
    ctx.expect_joint_motion_axis(
        "shoulder_pitch",
        "lower_arm",
        world_axis="z",
        direction="positive",
        min_delta=0.02,
    )
    ctx.expect_joint_motion_axis(
        "elbow_pitch",
        "upper_arm",
        world_axis="z",
        direction="positive",
        min_delta=0.02,
    )

    with ctx.pose(shoulder_pitch=0.18, elbow_pitch=0.72, head_tilt=-0.90):
        ctx.expect_aabb_overlap("head", "base", axes="xy", min_overlap=0.02)
        ctx.expect_origin_distance("head", "base", axes="xy", max_dist=0.08)
        ctx.expect_origin_distance("head", "upper_arm", axes="xy", max_dist=0.10)

    with ctx.pose(shoulder_pitch=-0.20, elbow_pitch=0.20, head_tilt=-0.55):
        ctx.expect_origin_distance("head", "base", axes="xy", max_dist=0.23)
        ctx.expect_origin_distance("head", "upper_arm", axes="xy", max_dist=0.11)

    with ctx.pose(shoulder_pitch=0.55, elbow_pitch=0.60, head_tilt=0.20):
        ctx.expect_aabb_overlap("head", "base", axes="xy", min_overlap=0.01)
        ctx.expect_origin_distance("head", "base", axes="xy", max_dist=0.12)
        ctx.expect_origin_distance("head", "upper_arm", axes="xy", max_dist=0.11)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
