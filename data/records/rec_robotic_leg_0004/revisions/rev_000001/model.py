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
    rounded_rect_profile,
    section_loft,
)

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    def save_mesh(name: str, geometry):
        return mesh_from_geometry(geometry, ASSETS.mesh_path(name))

    def xy_section(width: float, depth: float, z: float) -> list[tuple[float, float, float]]:
        profile = rounded_rect_profile(
            width,
            depth,
            radius=min(width, depth) * 0.18,
            corner_segments=8,
        )
        return [(x, y, z) for x, y in profile]

    def yz_section(
        width_y: float,
        height_z: float,
        x: float,
        *,
        z_center: float,
    ) -> list[tuple[float, float, float]]:
        profile = rounded_rect_profile(
            width_y,
            height_z,
            radius=min(width_y, height_z) * 0.18,
            corner_segments=8,
        )
        return [(x, y, z + z_center) for y, z in profile]

    def add_x_bolts(
        part,
        *,
        prefix: str,
        x: float,
        y_positions: tuple[float, ...],
        z_positions: tuple[float, ...],
        material,
        radius: float = 0.004,
        length: float = 0.010,
    ) -> None:
        for row, y in enumerate(y_positions):
            for column, z in enumerate(z_positions):
                part.visual(
                    Cylinder(radius=radius, length=length),
                    origin=Origin(xyz=(x, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
                    material=material,
                    name=f"{prefix}_{row}_{column}",
                )

    def add_z_bolts(
        part,
        *,
        prefix: str,
        z: float,
        x_positions: tuple[float, ...],
        y_positions: tuple[float, ...],
        material,
        radius: float = 0.004,
        length: float = 0.008,
    ) -> None:
        for row, x in enumerate(x_positions):
            for column, y in enumerate(y_positions):
                part.visual(
                    Cylinder(radius=radius, length=length),
                    origin=Origin(xyz=(x, y, z)),
                    material=material,
                    name=f"{prefix}_{row}_{column}",
                )

    model = ArticulatedObject(name="utility_robotic_leg", assets=ASSETS)

    utility_paint = model.material("utility_paint", rgba=(0.39, 0.44, 0.34, 1.0))
    dark_housing = model.material("dark_housing", rgba=(0.18, 0.19, 0.21, 1.0))
    molded_polymer = model.material("molded_polymer", rgba=(0.24, 0.25, 0.27, 1.0))
    service_gray = model.material("service_gray", rgba=(0.53, 0.55, 0.58, 1.0))
    fastener_black = model.material("fastener_black", rgba=(0.09, 0.09, 0.10, 1.0))
    wear_steel = model.material("wear_steel", rgba=(0.67, 0.69, 0.71, 1.0))
    sole_rubber = model.material("sole_rubber", rgba=(0.10, 0.11, 0.12, 1.0))

    upper_leg_shell = save_mesh(
        "upper_leg_shell.obj",
        section_loft(
            [
                xy_section(0.112, 0.084, -0.045),
                xy_section(0.106, 0.078, -0.170),
                xy_section(0.094, 0.068, -0.275),
            ]
        ),
    )
    lower_leg_shell = save_mesh(
        "lower_leg_shell.obj",
        section_loft(
            [
                xy_section(0.102, 0.074, -0.040),
                xy_section(0.094, 0.068, -0.160),
                xy_section(0.086, 0.060, -0.260),
            ]
        ),
    )
    foot_shell = save_mesh(
        "foot_shell.obj",
        section_loft(
            [
                yz_section(0.072, 0.040, -0.070, z_center=-0.087),
                yz_section(0.090, 0.050, 0.030, z_center=-0.090),
                yz_section(0.070, 0.036, 0.180, z_center=-0.077),
            ]
        ),
    )

    hip_mount = model.part("hip_mount")
    hip_mount.visual(
        Box((0.160, 0.120, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, 0.075)),
        material=utility_paint,
        name="mount_block",
    )
    hip_mount.visual(
        Box((0.110, 0.100, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.065)),
        material=dark_housing,
        name="service_collar",
    )
    hip_mount.visual(
        Box((0.034, 0.086, 0.060)),
        origin=Origin(xyz=(0.086, 0.0, 0.055)),
        material=dark_housing,
        name="hip_drive_pack_right",
    )
    hip_mount.visual(
        Box((0.034, 0.086, 0.060)),
        origin=Origin(xyz=(-0.086, 0.0, 0.055)),
        material=dark_housing,
        name="hip_drive_pack_left",
    )
    hip_mount.visual(
        Box((0.008, 0.082, 0.052)),
        origin=Origin(xyz=(0.107, 0.0, 0.055)),
        material=service_gray,
        name="hip_drive_cover_right",
    )
    hip_mount.visual(
        Box((0.008, 0.082, 0.052)),
        origin=Origin(xyz=(-0.107, 0.0, 0.055)),
        material=service_gray,
        name="hip_drive_cover_left",
    )
    hip_mount.visual(
        Box((0.100, 0.018, 0.150)),
        origin=Origin(xyz=(0.0, 0.056, -0.020)),
        material=utility_paint,
        name="hip_yoke_left",
    )
    hip_mount.visual(
        Box((0.100, 0.018, 0.150)),
        origin=Origin(xyz=(0.0, -0.056, -0.020)),
        material=utility_paint,
        name="hip_yoke_right",
    )
    hip_mount.visual(
        Cylinder(radius=0.022, length=0.014),
        origin=Origin(xyz=(0.0, 0.041, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=wear_steel,
        name="hip_bearing_left",
    )
    hip_mount.visual(
        Cylinder(radius=0.022, length=0.014),
        origin=Origin(xyz=(0.0, -0.041, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=wear_steel,
        name="hip_bearing_right",
    )
    hip_mount.visual(
        Cylinder(radius=0.031, length=0.012),
        origin=Origin(xyz=(0.0, 0.071, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=fastener_black,
        name="hip_cap_left",
    )
    hip_mount.visual(
        Cylinder(radius=0.031, length=0.012),
        origin=Origin(xyz=(0.0, -0.071, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=fastener_black,
        name="hip_cap_right",
    )
    add_x_bolts(
        hip_mount,
        prefix="hip_cover_bolt_right",
        x=0.111,
        y_positions=(-0.026, 0.026),
        z_positions=(0.037, 0.075),
        material=fastener_black,
    )
    add_x_bolts(
        hip_mount,
        prefix="hip_cover_bolt_left",
        x=-0.111,
        y_positions=(-0.026, 0.026),
        z_positions=(0.037, 0.075),
        material=fastener_black,
    )
    add_z_bolts(
        hip_mount,
        prefix="top_mount_bolt",
        z=0.101,
        x_positions=(-0.050, 0.050),
        y_positions=(-0.034, 0.034),
        material=fastener_black,
    )
    hip_mount.inertial = Inertial.from_geometry(
        Box((0.220, 0.160, 0.200)),
        mass=5.6,
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
    )

    upper_leg = model.part("upper_leg")
    upper_leg.visual(
        Box((0.090, 0.064, 0.070)),
        origin=Origin(xyz=(0.0, 0.0, -0.015)),
        material=dark_housing,
        name="hip_core",
    )
    upper_leg.visual(
        Cylinder(radius=0.024, length=0.068),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=wear_steel,
        name="hip_barrel",
    )
    upper_leg.visual(upper_leg_shell, material=utility_paint, name="thigh_shell")
    upper_leg.visual(
        Box((0.036, 0.068, 0.160)),
        origin=Origin(xyz=(0.055, 0.0, -0.170)),
        material=dark_housing,
        name="upper_actuator_bay",
    )
    upper_leg.visual(
        Box((0.008, 0.062, 0.140)),
        origin=Origin(xyz=(0.074, 0.0, -0.170)),
        material=service_gray,
        name="upper_actuator_cover",
    )
    upper_leg.visual(
        Box((0.018, 0.078, 0.220)),
        origin=Origin(xyz=(-0.049, 0.0, -0.180)),
        material=molded_polymer,
        name="upper_rear_rib",
    )
    upper_leg.visual(
        Box((0.094, 0.024, 0.095)),
        origin=Origin(xyz=(0.0, 0.049, -0.317)),
        material=utility_paint,
        name="knee_clevis_left",
    )
    upper_leg.visual(
        Box((0.094, 0.024, 0.095)),
        origin=Origin(xyz=(0.0, -0.049, -0.317)),
        material=utility_paint,
        name="knee_clevis_right",
    )
    upper_leg.visual(
        Cylinder(radius=0.020, length=0.018),
        origin=Origin(xyz=(0.0, 0.036, -0.340), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=wear_steel,
        name="knee_bearing_left",
    )
    upper_leg.visual(
        Cylinder(radius=0.020, length=0.018),
        origin=Origin(xyz=(0.0, -0.036, -0.340), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=wear_steel,
        name="knee_bearing_right",
    )
    upper_leg.visual(
        Cylinder(radius=0.028, length=0.012),
        origin=Origin(xyz=(0.0, 0.064, -0.340), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=fastener_black,
        name="knee_cap_left",
    )
    upper_leg.visual(
        Cylinder(radius=0.028, length=0.012),
        origin=Origin(xyz=(0.0, -0.064, -0.340), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=fastener_black,
        name="knee_cap_right",
    )
    add_x_bolts(
        upper_leg,
        prefix="upper_cover_bolt",
        x=0.078,
        y_positions=(-0.020, 0.020),
        z_positions=(-0.230, -0.110),
        material=fastener_black,
    )
    upper_leg.inertial = Inertial.from_geometry(
        Box((0.160, 0.120, 0.380)),
        mass=4.2,
        origin=Origin(xyz=(0.0, 0.0, -0.180)),
    )

    lower_leg = model.part("lower_leg")
    lower_leg.visual(
        Box((0.084, 0.058, 0.060)),
        origin=Origin(xyz=(0.0, 0.0, -0.020)),
        material=dark_housing,
        name="knee_core",
    )
    lower_leg.visual(
        Cylinder(radius=0.0215, length=0.054),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=wear_steel,
        name="knee_barrel",
    )
    lower_leg.visual(lower_leg_shell, material=utility_paint, name="shin_shell")
    lower_leg.visual(
        Box((0.034, 0.060, 0.140)),
        origin=Origin(xyz=(-0.053, 0.0, -0.155)),
        material=dark_housing,
        name="lower_actuator_bay",
    )
    lower_leg.visual(
        Box((0.008, 0.054, 0.120)),
        origin=Origin(xyz=(-0.072, 0.0, -0.155)),
        material=service_gray,
        name="lower_actuator_cover",
    )
    lower_leg.visual(
        Box((0.016, 0.072, 0.180)),
        origin=Origin(xyz=(0.046, 0.0, -0.160)),
        material=molded_polymer,
        name="shin_wear_strip",
    )
    lower_leg.visual(
        Box((0.088, 0.024, 0.100)),
        origin=Origin(xyz=(0.0, 0.047, -0.268)),
        material=utility_paint,
        name="ankle_clevis_left",
    )
    lower_leg.visual(
        Box((0.088, 0.024, 0.100)),
        origin=Origin(xyz=(0.0, -0.047, -0.268)),
        material=utility_paint,
        name="ankle_clevis_right",
    )
    lower_leg.visual(
        Cylinder(radius=0.018, length=0.016),
        origin=Origin(xyz=(0.0, 0.0325, -0.300), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=wear_steel,
        name="ankle_bearing_left",
    )
    lower_leg.visual(
        Cylinder(radius=0.018, length=0.016),
        origin=Origin(xyz=(0.0, -0.0325, -0.300), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=wear_steel,
        name="ankle_bearing_right",
    )
    lower_leg.visual(
        Cylinder(radius=0.025, length=0.012),
        origin=Origin(xyz=(0.0, 0.060, -0.300), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=fastener_black,
        name="ankle_cap_left",
    )
    lower_leg.visual(
        Cylinder(radius=0.025, length=0.012),
        origin=Origin(xyz=(0.0, -0.060, -0.300), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=fastener_black,
        name="ankle_cap_right",
    )
    add_x_bolts(
        lower_leg,
        prefix="lower_cover_bolt",
        x=-0.076,
        y_positions=(-0.018, 0.018),
        z_positions=(-0.210, -0.100),
        material=fastener_black,
    )
    lower_leg.inertial = Inertial.from_geometry(
        Box((0.150, 0.110, 0.340)),
        mass=3.5,
        origin=Origin(xyz=(0.0, 0.0, -0.160)),
    )

    foot = model.part("foot")
    foot.visual(
        Cylinder(radius=0.020, length=0.049),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=wear_steel,
        name="ankle_barrel",
    )
    foot.visual(
        Box((0.074, 0.052, 0.052)),
        origin=Origin(xyz=(0.0, 0.0, -0.046)),
        material=dark_housing,
        name="ankle_block",
    )
    foot.visual(
        Box((0.050, 0.050, 0.085)),
        origin=Origin(xyz=(0.010, 0.0, -0.072)),
        material=molded_polymer,
        name="rocker_block",
    )
    foot.visual(foot_shell, material=utility_paint, name="foot_shell")
    foot.visual(
        Box((0.120, 0.080, 0.008)),
        origin=Origin(xyz=(0.055, 0.0, -0.060)),
        material=service_gray,
        name="foot_deck_plate",
    )
    foot.visual(
        Box((0.090, 0.070, 0.020)),
        origin=Origin(xyz=(-0.025, 0.0, -0.114)),
        material=sole_rubber,
        name="heel_pad",
    )
    foot.visual(
        Box((0.155, 0.068, 0.016)),
        origin=Origin(xyz=(0.107, 0.0, -0.114)),
        material=sole_rubber,
        name="toe_pad",
    )
    foot.visual(
        Box((0.018, 0.062, 0.026)),
        origin=Origin(xyz=(0.189, 0.0, -0.083)),
        material=wear_steel,
        name="toe_bumper",
    )
    foot.visual(
        Box((0.030, 0.052, 0.020)),
        origin=Origin(xyz=(-0.083, 0.0, -0.087)),
        material=wear_steel,
        name="heel_spur",
    )
    add_z_bolts(
        foot,
        prefix="foot_deck_bolt",
        z=-0.056,
        x_positions=(0.010, 0.100),
        y_positions=(-0.022, 0.022),
        material=fastener_black,
    )
    foot.inertial = Inertial.from_geometry(
        Box((0.280, 0.120, 0.140)),
        mass=1.8,
        origin=Origin(xyz=(0.050, 0.0, -0.080)),
    )

    model.articulation(
        "hip_pitch",
        ArticulationType.REVOLUTE,
        parent=hip_mount,
        child=upper_leg,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=420.0,
            velocity=1.4,
            lower=-0.70,
            upper=0.90,
        ),
    )
    model.articulation(
        "knee_pitch",
        ArticulationType.REVOLUTE,
        parent=upper_leg,
        child=lower_leg,
        origin=Origin(xyz=(0.0, 0.0, -0.340)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=390.0,
            velocity=1.7,
            lower=0.00,
            upper=1.35,
        ),
    )
    model.articulation(
        "ankle_pitch",
        ArticulationType.REVOLUTE,
        parent=lower_leg,
        child=foot,
        origin=Origin(xyz=(0.0, 0.0, -0.300)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=220.0,
            velocity=1.8,
            lower=-0.40,
            upper=0.35,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    hip_mount = object_model.get_part("hip_mount")
    upper_leg = object_model.get_part("upper_leg")
    lower_leg = object_model.get_part("lower_leg")
    foot = object_model.get_part("foot")

    hip_pitch = object_model.get_articulation("hip_pitch")
    knee_pitch = object_model.get_articulation("knee_pitch")
    ankle_pitch = object_model.get_articulation("ankle_pitch")

    hip_barrel = upper_leg.get_visual("hip_barrel")
    knee_barrel = lower_leg.get_visual("knee_barrel")
    ankle_barrel = foot.get_visual("ankle_barrel")

    hip_bearing_left = hip_mount.get_visual("hip_bearing_left")
    hip_bearing_right = hip_mount.get_visual("hip_bearing_right")
    knee_bearing_left = upper_leg.get_visual("knee_bearing_left")
    knee_bearing_right = upper_leg.get_visual("knee_bearing_right")
    ankle_bearing_left = lower_leg.get_visual("ankle_bearing_left")
    ankle_bearing_right = lower_leg.get_visual("ankle_bearing_right")

    thigh_shell = upper_leg.get_visual("thigh_shell")
    upper_actuator_cover = upper_leg.get_visual("upper_actuator_cover")
    shin_shell = lower_leg.get_visual("shin_shell")
    lower_actuator_cover = lower_leg.get_visual("lower_actuator_cover")
    foot_shell = foot.get_visual("foot_shell")
    toe_pad = foot.get_visual("toe_pad")

    def dims_from_aabb(aabb) -> tuple[float, float, float]:
        return tuple(aabb[1][index] - aabb[0][index] for index in range(3))

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
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
    ctx.fail_if_articulation_overlaps(max_pose_samples=24)

    ctx.expect_contact(
        upper_leg,
        hip_mount,
        elem_a=hip_barrel,
        elem_b=hip_bearing_left,
        contact_tol=0.001,
        name="hip_left_bearing_contact",
    )
    ctx.expect_contact(
        upper_leg,
        hip_mount,
        elem_a=hip_barrel,
        elem_b=hip_bearing_right,
        contact_tol=0.001,
        name="hip_right_bearing_contact",
    )
    ctx.expect_contact(
        lower_leg,
        upper_leg,
        elem_a=knee_barrel,
        elem_b=knee_bearing_left,
        contact_tol=0.001,
        name="knee_left_bearing_contact",
    )
    ctx.expect_contact(
        lower_leg,
        upper_leg,
        elem_a=knee_barrel,
        elem_b=knee_bearing_right,
        contact_tol=0.001,
        name="knee_right_bearing_contact",
    )
    ctx.expect_contact(
        foot,
        lower_leg,
        elem_a=ankle_barrel,
        elem_b=ankle_bearing_left,
        contact_tol=0.001,
        name="ankle_left_bearing_contact",
    )
    ctx.expect_contact(
        foot,
        lower_leg,
        elem_a=ankle_barrel,
        elem_b=ankle_bearing_right,
        contact_tol=0.001,
        name="ankle_right_bearing_contact",
    )

    ctx.expect_origin_distance(upper_leg, hip_mount, axes="xy", max_dist=0.001, name="hip_axis_centered")
    ctx.expect_origin_distance(lower_leg, upper_leg, axes="xy", max_dist=0.001, name="knee_axis_centered")
    ctx.expect_origin_distance(foot, lower_leg, axes="xy", max_dist=0.001, name="ankle_axis_centered")

    hip_aabb = ctx.part_world_aabb(hip_mount)
    upper_aabb = ctx.part_world_aabb(upper_leg)
    lower_aabb = ctx.part_world_aabb(lower_leg)
    foot_aabb = ctx.part_world_aabb(foot)
    upper_pos = ctx.part_world_position(upper_leg)
    lower_pos = ctx.part_world_position(lower_leg)
    foot_pos = ctx.part_world_position(foot)

    if hip_aabb is not None:
        hip_dims = dims_from_aabb(hip_aabb)
        ctx.check(
            "hip_module_is_rugged",
            hip_dims[0] > 0.17 and hip_dims[1] > 0.14 and hip_dims[2] > 0.18,
            f"hip dims={hip_dims}",
        )
    if upper_aabb is not None:
        upper_dims = dims_from_aabb(upper_aabb)
        ctx.check(
            "upper_leg_has_load_bearing_depth",
            0.10 < upper_dims[0] < 0.18 and 0.34 < upper_dims[2] < 0.42,
            f"upper dims={upper_dims}",
        )
    if lower_aabb is not None:
        lower_dims = dims_from_aabb(lower_aabb)
        ctx.check(
            "lower_leg_has_reinforced_span",
            0.09 < lower_dims[0] < 0.16 and 0.28 < lower_dims[2] < 0.36,
            f"lower dims={lower_dims}",
        )
    if foot_aabb is not None:
        foot_dims = dims_from_aabb(foot_aabb)
        ctx.check(
            "foot_is_practical_support_size",
            0.24 < foot_dims[0] < 0.32 and 0.07 < foot_dims[1] < 0.12,
            f"foot dims={foot_dims}",
        )

    if upper_pos is not None and lower_pos is not None and foot_pos is not None:
        ctx.check(
            "serial_links_stack_below_hip",
            upper_pos[2] > lower_pos[2] > foot_pos[2] and abs(lower_pos[1]) < 0.001 and abs(foot_pos[1]) < 0.001,
            f"upper={upper_pos}, lower={lower_pos}, foot={foot_pos}",
        )

    upper_shell_aabb = ctx.part_element_world_aabb(upper_leg, elem=thigh_shell)
    upper_cover_aabb = ctx.part_element_world_aabb(upper_leg, elem=upper_actuator_cover)
    if upper_shell_aabb is not None and upper_cover_aabb is not None:
        ctx.check(
            "upper_actuator_bay_is_exposed",
            upper_cover_aabb[1][0] > upper_shell_aabb[1][0] + 0.015,
            f"cover={upper_cover_aabb}, shell={upper_shell_aabb}",
        )

    shin_shell_aabb = ctx.part_element_world_aabb(lower_leg, elem=shin_shell)
    lower_cover_aabb = ctx.part_element_world_aabb(lower_leg, elem=lower_actuator_cover)
    if shin_shell_aabb is not None and lower_cover_aabb is not None:
        ctx.check(
            "lower_actuator_bay_is_serviceable",
            lower_cover_aabb[0][0] < shin_shell_aabb[0][0] - 0.015,
            f"cover={lower_cover_aabb}, shell={shin_shell_aabb}",
        )

    foot_shell_aabb = ctx.part_element_world_aabb(foot, elem=foot_shell)
    toe_pad_aabb = ctx.part_element_world_aabb(foot, elem=toe_pad)
    if foot_shell_aabb is not None and toe_pad_aabb is not None:
        ctx.check(
            "toe_pad_sits_below_shell",
            toe_pad_aabb[0][2] < foot_shell_aabb[0][2] - 0.004,
            f"toe_pad={toe_pad_aabb}, shell={foot_shell_aabb}",
        )

    with ctx.pose({hip_pitch: 0.60}):
        lower_flexed = ctx.part_world_position(lower_leg)
        foot_flexed = ctx.part_world_position(foot)
        if lower_flexed is not None and foot_flexed is not None:
            ctx.check(
                "hip_pitch_moves_leg_in_sagittal_plane",
                lower_flexed[0] > 0.18 and foot_flexed[0] > 0.18 and abs(lower_flexed[1]) < 0.01 and abs(foot_flexed[1]) < 0.01,
                f"lower={lower_flexed}, foot={foot_flexed}",
            )
        ctx.expect_contact(upper_leg, hip_mount, elem_a=hip_barrel, elem_b=hip_bearing_left, contact_tol=0.001)
        ctx.expect_contact(upper_leg, hip_mount, elem_a=hip_barrel, elem_b=hip_bearing_right, contact_tol=0.001)

    with ctx.pose({knee_pitch: 1.00}):
        foot_tucked = ctx.part_world_position(foot)
        if foot_tucked is not None:
            ctx.check(
                "knee_pitch_tucks_foot_forward",
                foot_tucked[0] > 0.20 and foot_tucked[2] > -0.52 and abs(foot_tucked[1]) < 0.01,
                f"foot={foot_tucked}",
            )
        ctx.expect_contact(lower_leg, upper_leg, elem_a=knee_barrel, elem_b=knee_bearing_left, contact_tol=0.001)
        ctx.expect_contact(lower_leg, upper_leg, elem_a=knee_barrel, elem_b=knee_bearing_right, contact_tol=0.001)
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_knee_bent_pose")

    if foot_aabb is not None:
        with ctx.pose({ankle_pitch: 0.30}):
            toe_down_aabb = ctx.part_world_aabb(foot)
            if toe_down_aabb is not None:
                ctx.check(
                    "ankle_pitch_changes_foot_attitude",
                    toe_down_aabb[0][2] < foot_aabb[0][2] - 0.010,
                    f"rest={foot_aabb}, toe_down={toe_down_aabb}",
                )
            ctx.expect_contact(foot, lower_leg, elem_a=ankle_barrel, elem_b=ankle_bearing_left, contact_tol=0.001)
            ctx.expect_contact(foot, lower_leg, elem_a=ankle_barrel, elem_b=ankle_bearing_right, contact_tol=0.001)

    with ctx.pose({hip_pitch: 0.35, knee_pitch: 0.85, ankle_pitch: -0.18}):
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_loaded_stride_pose")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
