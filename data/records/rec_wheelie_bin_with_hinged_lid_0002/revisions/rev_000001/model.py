from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import cos, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
)

ASSETS = AssetContext.from_script(__file__)


def _axis_rpy(axis: str) -> tuple[float, float, float]:
    if axis == "x":
        return (0.0, pi / 2.0, 0.0)
    if axis == "y":
        return (pi / 2.0, 0.0, 0.0)
    return (0.0, 0.0, 0.0)


def _add_cylinder_visual(
    part,
    *,
    name: str,
    radius: float,
    length: float,
    xyz: tuple[float, float, float],
    axis: str = "z",
    material=None,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=_axis_rpy(axis)),
        material=material,
        name=name,
    )


def _add_bolt_head(
    part,
    *,
    name: str,
    xyz: tuple[float, float, float],
    axis: str,
    radius: float = 0.006,
    length: float = 0.008,
    material=None,
) -> None:
    _add_cylinder_visual(
        part,
        name=name,
        radius=radius,
        length=length,
        xyz=xyz,
        axis=axis,
        material=material,
    )


def _add_wheel_lugs(part, *, prefix: str, face_x: float, material) -> None:
    lug_ring_radius = 0.031
    for index in range(5):
        angle = 2.0 * pi * index / 5.0
        y = lug_ring_radius * cos(angle)
        z = lug_ring_radius * sin(angle)
        _add_bolt_head(
            part,
            name=f"{prefix}_{index}",
            xyz=(face_x, y, z),
            axis="x",
            radius=0.005,
            length=0.008,
            material=material,
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rugged_wheelie_bin", assets=ASSETS)

    body_plastic = model.material("body_plastic", rgba=(0.18, 0.26, 0.19, 1.0))
    lid_plastic = model.material("lid_plastic", rgba=(0.22, 0.31, 0.22, 1.0))
    rubber = model.material("rubber", rgba=(0.07, 0.07, 0.07, 1.0))
    wheel_hub = model.material("wheel_hub", rgba=(0.19, 0.19, 0.20, 1.0))
    painted_steel = model.material("painted_steel", rgba=(0.30, 0.31, 0.33, 1.0))
    fastener_metal = model.material("fastener_metal", rgba=(0.70, 0.72, 0.74, 1.0))

    body_outer_width = 0.575
    body_outer_depth = 0.730
    body_wall = 0.012
    shell_height = 0.875
    rim_height = 0.030
    hinge_axis_y = -0.386
    hinge_axis_z = 0.900
    wheel_radius = 0.110
    wheel_width = 0.040
    wheel_center_x = 0.308
    axle_y = -0.395
    axle_z = wheel_radius

    body = model.part("body")
    body.visual(
        Box((0.551, 0.665, 0.024)),
        origin=Origin(xyz=(0.0, -0.010, 0.012)),
        material=body_plastic,
        name="floor",
    )
    body.visual(
        Box((body_wall, 0.700, shell_height)),
        origin=Origin(xyz=(-(body_outer_width / 2.0) + (body_wall / 2.0), 0.0, shell_height / 2.0)),
        material=body_plastic,
        name="left_wall",
    )
    body.visual(
        Box((body_wall, 0.700, shell_height)),
        origin=Origin(xyz=((body_outer_width / 2.0) - (body_wall / 2.0), 0.0, shell_height / 2.0)),
        material=body_plastic,
        name="right_wall",
    )
    body.visual(
        Box((0.560, body_wall, shell_height)),
        origin=Origin(xyz=(0.0, (body_outer_depth / 2.0) - (body_wall / 2.0), shell_height / 2.0)),
        material=body_plastic,
        name="front_wall",
    )
    body.visual(
        Box((0.560, body_wall, shell_height)),
        origin=Origin(xyz=(0.0, -(body_outer_depth / 2.0) + (body_wall / 2.0), shell_height / 2.0)),
        material=body_plastic,
        name="back_wall",
    )
    body.visual(
        Box((0.560, 0.050, rim_height)),
        origin=Origin(xyz=(0.0, 0.343, 0.890)),
        material=body_plastic,
        name="front_rim",
    )
    body.visual(
        Box((0.545, 0.032, rim_height)),
        origin=Origin(xyz=(0.0, -0.352, 0.890)),
        material=body_plastic,
        name="rear_rim",
    )
    body.visual(
        Box((0.050, 0.686, rim_height)),
        origin=Origin(xyz=(-0.263, 0.0, 0.890)),
        material=body_plastic,
        name="left_rim",
    )
    body.visual(
        Box((0.050, 0.686, rim_height)),
        origin=Origin(xyz=(0.263, 0.0, 0.890)),
        material=body_plastic,
        name="right_rim",
    )
    body.visual(
        Box((0.080, 0.125, 0.180)),
        origin=Origin(xyz=(-0.247, -0.2975, 0.090)),
        material=body_plastic,
        name="left_rear_pod",
    )
    body.visual(
        Box((0.080, 0.125, 0.180)),
        origin=Origin(xyz=(0.247, -0.2975, 0.090)),
        material=body_plastic,
        name="right_rear_pod",
    )
    body.visual(
        Box((0.070, 0.030, 0.080)),
        origin=Origin(xyz=(-0.228, -0.2825, 0.130)),
        material=painted_steel,
        name="left_mount_pad",
    )
    body.visual(
        Box((0.070, 0.030, 0.080)),
        origin=Origin(xyz=(0.228, -0.2825, 0.130)),
        material=painted_steel,
        name="right_mount_pad",
    )
    body.visual(
        Box((0.050, 0.035, 0.028)),
        origin=Origin(xyz=(-0.228, -0.3585, axle_z + 0.027)),
        material=painted_steel,
        name="left_axle_clamp",
    )
    body.visual(
        Box((0.050, 0.035, 0.028)),
        origin=Origin(xyz=(0.228, -0.3585, axle_z + 0.027)),
        material=painted_steel,
        name="right_axle_clamp",
    )
    body.visual(
        Box((0.460, 0.040, 0.070)),
        origin=Origin(xyz=(0.0, 0.365, 0.080)),
        material=body_plastic,
        name="front_toe_bar",
    )
    body.visual(
        Box((0.380, 0.030, 0.090)),
        origin=Origin(xyz=(0.0, -0.347, 0.780)),
        material=body_plastic,
        name="rear_handle_bridge",
    )
    for index, x_pos in enumerate((-0.145, 0.0, 0.145), start=1):
        body.visual(
            Box((0.034, 0.022, 0.620)),
            origin=Origin(xyz=(x_pos, 0.371, 0.380)),
            material=body_plastic,
            name=f"front_rib_{index}",
        )
    for index, y_pos in enumerate((0.210, 0.070), start=1):
        body.visual(
            Box((0.020, 0.090, 0.520)),
            origin=Origin(xyz=(-0.269, y_pos, 0.330)),
            material=body_plastic,
            name=f"left_side_rib_{index}",
        )
        body.visual(
            Box((0.020, 0.090, 0.520)),
            origin=Origin(xyz=(0.269, y_pos, 0.330)),
            material=body_plastic,
            name=f"right_side_rib_{index}",
        )
    body.visual(
        Box((0.120, 0.020, 0.028)),
        origin=Origin(xyz=(-0.215, hinge_axis_y + 0.0195, 0.892)),
        material=painted_steel,
        name="left_hinge_saddle",
    )
    body.visual(
        Box((0.090, 0.020, 0.028)),
        origin=Origin(xyz=(0.0, hinge_axis_y + 0.0195, 0.892)),
        material=painted_steel,
        name="center_hinge_saddle",
    )
    body.visual(
        Box((0.120, 0.020, 0.028)),
        origin=Origin(xyz=(0.215, hinge_axis_y + 0.0195, 0.892)),
        material=painted_steel,
        name="right_hinge_saddle",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.610, 0.760, 0.930)),
        mass=14.5,
        origin=Origin(xyz=(0.0, 0.0, 0.465)),
    )
    for side, x_pos in (("left", -0.228), ("right", 0.228)):
        _add_bolt_head(
            body,
            name=f"{side}_mount_upper_bolt",
            xyz=(x_pos, -0.3015, 0.155),
            axis="y",
            radius=0.0065,
            length=0.008,
            material=fastener_metal,
        )
        _add_bolt_head(
            body,
            name=f"{side}_mount_lower_bolt",
            xyz=(x_pos, -0.3015, 0.105),
            axis="y",
            radius=0.0065,
            length=0.008,
            material=fastener_metal,
        )

    axle = model.part("axle")
    _add_cylinder_visual(
        axle,
        name="shaft",
        radius=0.0115,
        length=0.456,
        xyz=(0.0, 0.0, 0.0),
        axis="x",
        material=painted_steel,
    )
    _add_cylinder_visual(
        axle,
        name="left_spindle",
        radius=0.011,
        length=0.060,
        xyz=(-0.258, 0.0, 0.0),
        axis="x",
        material=painted_steel,
    )
    _add_cylinder_visual(
        axle,
        name="right_spindle",
        radius=0.011,
        length=0.060,
        xyz=(0.258, 0.0, 0.0),
        axis="x",
        material=painted_steel,
    )
    _add_cylinder_visual(
        axle,
        name="left_bearing_collar",
        radius=0.020,
        length=0.018,
        xyz=(-0.228, 0.0, 0.0),
        axis="x",
        material=wheel_hub,
    )
    _add_cylinder_visual(
        axle,
        name="right_bearing_collar",
        radius=0.020,
        length=0.018,
        xyz=(0.228, 0.0, 0.0),
        axis="x",
        material=wheel_hub,
    )
    axle.visual(
        Box((0.048, 0.010, 0.031)),
        origin=Origin(xyz=(-0.228, 0.014, 0.027)),
        material=painted_steel,
        name="left_mount_tab",
    )
    axle.visual(
        Box((0.048, 0.010, 0.031)),
        origin=Origin(xyz=(0.228, 0.014, 0.027)),
        material=painted_steel,
        name="right_mount_tab",
    )
    axle.inertial = Inertial.from_geometry(
        Box((0.620, 0.040, 0.070)),
        mass=2.7,
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
    )

    hinge_pin = model.part("hinge_pin")
    _add_cylinder_visual(
        hinge_pin,
        name="hinge_rod",
        radius=0.0095,
        length=0.565,
        xyz=(0.0, 0.0, 0.0),
        axis="x",
        material=painted_steel,
    )
    _add_cylinder_visual(
        hinge_pin,
        name="left_pin_end_cap",
        radius=0.015,
        length=0.008,
        xyz=(-0.278, 0.0, 0.0),
        axis="x",
        material=fastener_metal,
    )
    _add_cylinder_visual(
        hinge_pin,
        name="right_pin_end_cap",
        radius=0.015,
        length=0.008,
        xyz=(0.278, 0.0, 0.0),
        axis="x",
        material=fastener_metal,
    )
    hinge_pin.inertial = Inertial.from_geometry(
        Box((0.570, 0.030, 0.030)),
        mass=0.5,
        origin=Origin(),
    )

    lid = model.part("lid")
    lid.visual(
        Box((0.602, 0.770, 0.018)),
        origin=Origin(xyz=(0.0, 0.385, 0.014)),
        material=lid_plastic,
        name="lid_shell",
    )
    lid.visual(
        Box((0.014, 0.720, 0.055)),
        origin=Origin(xyz=(-0.296, 0.360, -0.0225)),
        material=lid_plastic,
        name="left_skirt",
    )
    lid.visual(
        Box((0.014, 0.720, 0.055)),
        origin=Origin(xyz=(0.296, 0.360, -0.0225)),
        material=lid_plastic,
        name="right_skirt",
    )
    lid.visual(
        Box((0.560, 0.020, 0.060)),
        origin=Origin(xyz=(0.0, 0.760, -0.025)),
        material=lid_plastic,
        name="front_skirt",
    )
    lid.visual(
        Box((0.460, 0.040, 0.035)),
        origin=Origin(xyz=(0.0, 0.332, -0.0125)),
        material=lid_plastic,
        name="center_stiffener",
    )
    lid.visual(
        Box((0.420, 0.050, 0.040)),
        origin=Origin(xyz=(0.0, 0.570, -0.015)),
        material=lid_plastic,
        name="front_stiffener",
    )
    lid.visual(
        Box((0.180, 0.055, 0.018)),
        origin=Origin(xyz=(0.0, 0.712, 0.023)),
        material=lid_plastic,
        name="grip_ridge",
    )
    lid.visual(
        Box((0.090, 0.025, 0.055)),
        origin=Origin(xyz=(-0.100, 0.020, 0.028)),
        material=painted_steel,
        name="left_hinge_plate",
    )
    lid.visual(
        Box((0.090, 0.025, 0.055)),
        origin=Origin(xyz=(0.100, 0.020, 0.028)),
        material=painted_steel,
        name="right_hinge_plate",
    )
    _add_cylinder_visual(
        lid,
        name="left_lid_knuckle",
        radius=0.018,
        length=0.100,
        xyz=(-0.100, 0.0, 0.0),
        axis="x",
        material=painted_steel,
    )
    _add_cylinder_visual(
        lid,
        name="right_lid_knuckle",
        radius=0.018,
        length=0.100,
        xyz=(0.100, 0.0, 0.0),
        axis="x",
        material=painted_steel,
    )
    for side, x_pos in (("left", -0.100), ("right", 0.100)):
        _add_bolt_head(
            lid,
            name=f"{side}_hinge_plate_front_bolt",
            xyz=(x_pos - 0.018, 0.046, 0.026),
            axis="z",
            radius=0.0055,
            length=0.006,
            material=fastener_metal,
        )
        _add_bolt_head(
            lid,
            name=f"{side}_hinge_plate_rear_bolt",
            xyz=(x_pos + 0.018, 0.046, 0.026),
            axis="z",
            radius=0.0055,
            length=0.006,
            material=fastener_metal,
        )
    lid.inertial = Inertial.from_geometry(
        Box((0.610, 0.760, 0.070)),
        mass=3.1,
        origin=Origin(xyz=(0.0, 0.365, 0.005)),
    )

    left_wheel = model.part("left_wheel")
    _add_cylinder_visual(
        left_wheel,
        name="tire",
        radius=wheel_radius,
        length=wheel_width,
        xyz=(0.0, 0.0, 0.0),
        axis="x",
        material=rubber,
    )
    _add_cylinder_visual(
        left_wheel,
        name="hub_disc",
        radius=0.067,
        length=0.026,
        xyz=(0.0, 0.0, 0.0),
        axis="x",
        material=wheel_hub,
    )
    _add_cylinder_visual(
        left_wheel,
        name="hub_cap",
        radius=0.034,
        length=0.032,
        xyz=(0.0, 0.0, 0.0),
        axis="x",
        material=wheel_hub,
    )
    _add_cylinder_visual(
        left_wheel,
        name="inner_boss",
        radius=0.030,
        length=0.016,
        xyz=(0.012, 0.0, 0.0),
        axis="x",
        material=wheel_hub,
    )
    _add_wheel_lugs(left_wheel, prefix="outer_lug", face_x=-0.015, material=fastener_metal)
    left_wheel.inertial = Inertial.from_geometry(
        Box((wheel_width, 0.220, 0.220)),
        mass=1.8,
    )

    right_wheel = model.part("right_wheel")
    _add_cylinder_visual(
        right_wheel,
        name="tire",
        radius=wheel_radius,
        length=wheel_width,
        xyz=(0.0, 0.0, 0.0),
        axis="x",
        material=rubber,
    )
    _add_cylinder_visual(
        right_wheel,
        name="hub_disc",
        radius=0.067,
        length=0.026,
        xyz=(0.0, 0.0, 0.0),
        axis="x",
        material=wheel_hub,
    )
    _add_cylinder_visual(
        right_wheel,
        name="hub_cap",
        radius=0.034,
        length=0.032,
        xyz=(0.0, 0.0, 0.0),
        axis="x",
        material=wheel_hub,
    )
    _add_cylinder_visual(
        right_wheel,
        name="inner_boss",
        radius=0.030,
        length=0.016,
        xyz=(-0.012, 0.0, 0.0),
        axis="x",
        material=wheel_hub,
    )
    _add_wheel_lugs(right_wheel, prefix="outer_lug", face_x=0.015, material=fastener_metal)
    right_wheel.inertial = Inertial.from_geometry(
        Box((wheel_width, 0.220, 0.220)),
        mass=1.8,
    )

    model.articulation(
        "body_to_axle",
        ArticulationType.FIXED,
        parent=body,
        child=axle,
        origin=Origin(xyz=(0.0, axle_y, axle_z)),
    )
    model.articulation(
        "body_to_hinge_pin",
        ArticulationType.FIXED,
        parent=body,
        child=hinge_pin,
        origin=Origin(xyz=(0.0, hinge_axis_y, hinge_axis_z)),
    )
    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, hinge_axis_y, hinge_axis_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=2.4, lower=0.0, upper=2.2),
        motion_properties=MotionProperties(damping=0.45, friction=0.20),
    )
    model.articulation(
        "axle_to_left_wheel",
        ArticulationType.CONTINUOUS,
        parent=axle,
        child=left_wheel,
        origin=Origin(xyz=(-wheel_center_x, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=18.0),
        motion_properties=MotionProperties(damping=0.02, friction=0.02),
    )
    model.articulation(
        "axle_to_right_wheel",
        ArticulationType.CONTINUOUS,
        parent=axle,
        child=right_wheel,
        origin=Origin(xyz=(wheel_center_x, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=18.0),
        motion_properties=MotionProperties(damping=0.02, friction=0.02),
    )

    return model


def _safe_get_part(ctx: TestContext, name: str):
    try:
        return object_model.get_part(name)
    except Exception as exc:  # pragma: no cover - defensive test plumbing
        ctx.fail(f"{name}_present", str(exc))
        return None


def _safe_get_joint(ctx: TestContext, name: str):
    try:
        return object_model.get_articulation(name)
    except Exception as exc:  # pragma: no cover - defensive test plumbing
        ctx.fail(f"{name}_present", str(exc))
        return None


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    body = _safe_get_part(ctx, "body")
    axle = _safe_get_part(ctx, "axle")
    hinge_pin = _safe_get_part(ctx, "hinge_pin")
    lid = _safe_get_part(ctx, "lid")
    left_wheel = _safe_get_part(ctx, "left_wheel")
    right_wheel = _safe_get_part(ctx, "right_wheel")

    lid_hinge = _safe_get_joint(ctx, "body_to_lid")
    left_wheel_joint = _safe_get_joint(ctx, "axle_to_left_wheel")
    right_wheel_joint = _safe_get_joint(ctx, "axle_to_right_wheel")

    if hinge_pin is not None and lid is not None:
        ctx.allow_overlap(
            hinge_pin,
            lid,
            reason="The fixed steel hinge rod intentionally runs inside the lid knuckles as the bearing surface for the full-width lid pivot.",
        )

    # Preferred default QC stack:
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    if (
        body is None
        or axle is None
        or hinge_pin is None
        or lid is None
        or left_wheel is None
        or right_wheel is None
        or lid_hinge is None
        or left_wheel_joint is None
        or right_wheel_joint is None
    ):
        return ctx.report()

    left_axle_clamp = body.get_visual("left_axle_clamp")
    right_axle_clamp = body.get_visual("right_axle_clamp")
    left_mount_tab = axle.get_visual("left_mount_tab")
    right_mount_tab = axle.get_visual("right_mount_tab")
    hinge_rod = hinge_pin.get_visual("hinge_rod")
    left_hinge_saddle = body.get_visual("left_hinge_saddle")
    center_hinge_saddle = body.get_visual("center_hinge_saddle")
    lid_shell = lid.get_visual("lid_shell")
    grip_ridge = lid.get_visual("grip_ridge")
    front_rim = body.get_visual("front_rim")
    rear_rim = body.get_visual("rear_rim")

    body_aabb = ctx.part_world_aabb(body)
    if body_aabb is not None:
        body_min, body_max = body_aabb
        body_width = body_max[0] - body_min[0]
        body_depth = body_max[1] - body_min[1]
        body_height = body_max[2] - body_min[2]
        ctx.check(
            "body_dimensions_realistic",
            0.56 <= body_width <= 0.62 and 0.74 <= body_depth <= 0.79 and 0.89 <= body_height <= 0.94,
            f"body dims were {(body_width, body_depth, body_height)}",
        )
    else:
        ctx.fail("body_dimensions_realistic", "body world AABB was unavailable")

    left_pos = ctx.part_world_position(left_wheel)
    right_pos = ctx.part_world_position(right_wheel)
    if left_pos is not None and right_pos is not None:
        ctx.check(
            "wheel_centers_symmetric",
            abs(left_pos[0] + right_pos[0]) <= 1e-6
            and abs(left_pos[1] - right_pos[1]) <= 1e-6
            and abs(left_pos[2] - right_pos[2]) <= 1e-6
            and 0.29 <= abs(left_pos[0]) <= 0.32,
            f"wheel positions were left={left_pos} right={right_pos}",
        )
    else:
        ctx.fail("wheel_centers_symmetric", "wheel center positions were unavailable")

    def _axis_is_x(joint_obj) -> bool:
        axis = tuple(round(v, 6) for v in joint_obj.axis)
        return axis == (1.0, 0.0, 0.0)

    ctx.check("lid_hinge_axis_x", _axis_is_x(lid_hinge), f"lid hinge axis was {lid_hinge.axis}")
    ctx.check(
        "left_wheel_axis_x",
        _axis_is_x(left_wheel_joint),
        f"left wheel axis was {left_wheel_joint.axis}",
    )
    ctx.check(
        "right_wheel_axis_x",
        _axis_is_x(right_wheel_joint),
        f"right wheel axis was {right_wheel_joint.axis}",
    )

    lid_limits = lid_hinge.motion_limits
    ctx.check(
        "lid_hinge_limits_realistic",
        lid_limits is not None
        and lid_limits.lower is not None
        and lid_limits.upper is not None
        and abs(lid_limits.lower) <= 1e-6
        and 1.9 <= lid_limits.upper <= 2.3,
        f"lid limits were {lid_limits}",
    )

    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_contact(
            axle,
            body,
            elem_a=left_mount_tab,
            elem_b=left_axle_clamp,
            name="left_axle_clamp_contact",
        )
        ctx.expect_contact(
            axle,
            body,
            elem_a=right_mount_tab,
            elem_b=right_axle_clamp,
            name="right_axle_clamp_contact",
        )
        ctx.expect_contact(
            hinge_pin,
            body,
            elem_a=hinge_rod,
            elem_b=left_hinge_saddle,
            name="hinge_rod_left_saddle_contact",
        )
        ctx.expect_contact(
            hinge_pin,
            body,
            elem_a=hinge_rod,
            elem_b=center_hinge_saddle,
            name="hinge_rod_center_saddle_contact",
        )
        ctx.expect_contact(left_wheel, axle, name="left_wheel_supported_on_axle")
        ctx.expect_contact(right_wheel, axle, name="right_wheel_supported_on_axle")
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            elem_a=lid_shell,
            min_overlap=0.54,
            name="lid_covers_bin_opening",
        )
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem=lid_shell,
            negative_elem=front_rim,
            max_gap=0.002,
            max_penetration=0.0,
            name="lid_seats_on_front_rim",
        )

    if lid_limits is not None and lid_limits.upper is not None:
        with ctx.pose({lid_hinge: lid_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="lid_open_no_unintended_overlap")
            ctx.fail_if_isolated_parts(name="lid_open_still_supported")
            ctx.expect_gap(
                lid,
                body,
                axis="z",
                positive_elem=grip_ridge,
                negative_elem=rear_rim,
                min_gap=0.40,
                name="lid_open_front_edge_lifts_clear",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
