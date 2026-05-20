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
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _tray_mesh(
    name: str,
    *,
    tray_radius: float,
    hub_inner_radius: float,
    hub_outer_radius: float,
    floor_thickness: float,
    rim_height: float,
    rim_thickness: float,
    hub_height: float,
):
    platter_outer = [
        (hub_outer_radius - 0.002, 0.0),
        (tray_radius - rim_thickness * 1.35, 0.0),
        (tray_radius - rim_thickness * 0.40, 0.0015),
        (tray_radius, 0.003),
        (tray_radius, rim_height),
        (tray_radius - rim_thickness, rim_height),
    ]
    platter_inner = [
        (hub_outer_radius + 0.0015, floor_thickness),
        (tray_radius - rim_thickness, floor_thickness + 0.001),
        (tray_radius - rim_thickness, rim_height - 0.0015),
    ]
    platter = LatheGeometry.from_shell_profiles(
        platter_outer,
        platter_inner,
        segments=72,
    )

    hub_outer = [
        (hub_outer_radius, 0.0),
        (hub_outer_radius, hub_height),
    ]
    hub_inner = [
        (hub_inner_radius, 0.0015),
        (hub_inner_radius, hub_height - 0.0015),
    ]
    hub = LatheGeometry.from_shell_profiles(hub_outer, hub_inner, segments=72)
    platter.merge(hub)
    return mesh_from_geometry(platter, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="two_tier_pantry_lazy_susan")

    steel = model.material("steel", rgba=(0.73, 0.75, 0.78, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.32, 0.33, 0.36, 1.0))
    smoke = model.material("smoke", rgba=(0.36, 0.39, 0.41, 1.0))
    warm_gray = model.material("warm_gray", rgba=(0.64, 0.63, 0.60, 1.0))

    lower_hub_height = 0.084
    upper_hub_height = 0.082
    lower_joint_z = 0.128
    upper_joint_z = 0.299

    center_post = model.part("center_post")
    center_post.visual(
        Cylinder(radius=0.09, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=warm_gray,
        name="base_foot",
    )
    center_post.visual(
        Cylinder(radius=0.012, length=0.350),
        origin=Origin(xyz=(0.0, 0.0, 0.191)),
        material=steel,
        name="main_spindle",
    )
    center_post.visual(
        Cylinder(radius=0.022, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.081)),
        material=dark_steel,
        name="lower_support_collar",
    )
    center_post.visual(
        Cylinder(radius=0.0225, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.176)),
        material=dark_steel,
        name="lower_keeper_collar",
    )
    center_post.visual(
        Cylinder(radius=0.0225, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.253)),
        material=dark_steel,
        name="upper_support_collar",
    )
    center_post.visual(
        Cylinder(radius=0.026, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.348)),
        material=dark_steel,
        name="top_capture_cap",
    )
    center_post.visual(
        Sphere(radius=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.372)),
        material=steel,
        name="top_finial",
    )
    center_post.inertial = Inertial.from_geometry(
        Box((0.18, 0.18, 0.388)),
        mass=2.8,
        origin=Origin(xyz=(0.0, 0.0, 0.194)),
    )

    lower_tray = model.part("lower_tray")
    lower_tray.visual(
        _tray_mesh(
            "lower_tray_shell",
            tray_radius=0.165,
            hub_inner_radius=0.0145,
            hub_outer_radius=0.031,
            floor_thickness=0.0045,
            rim_height=0.022,
            rim_thickness=0.010,
            hub_height=lower_hub_height,
        ),
        origin=Origin(xyz=(0.0, 0.0, -lower_hub_height / 2.0)),
        material=smoke,
        name="lower_tier",
    )
    lower_tray.inertial = Inertial.from_geometry(
        Cylinder(radius=0.165, length=lower_hub_height),
        mass=1.1,
    )
    for clip_index in range(3):
        angle = (2.0 * math.pi * clip_index) / 3.0
        clip_x = math.cos(angle) * 0.020
        clip_y = math.sin(angle) * 0.020
        lower_tray.visual(
            Box((0.010, 0.014, 0.010)),
            origin=Origin(xyz=(clip_x, clip_y, -0.037)),
            material=smoke,
            name=f"lower_support_clip_{clip_index}",
        )
        lower_tray.visual(
            Box((0.010, 0.014, 0.004)),
            origin=Origin(xyz=(clip_x, clip_y, 0.041)),
            material=smoke,
            name=f"lower_capture_clip_{clip_index}",
        )

    upper_tray = model.part("upper_tray")
    upper_tray.visual(
        _tray_mesh(
            "upper_tray_shell",
            tray_radius=0.136,
            hub_inner_radius=0.0145,
            hub_outer_radius=0.029,
            floor_thickness=0.004,
            rim_height=0.019,
            rim_thickness=0.009,
            hub_height=upper_hub_height,
        ),
        origin=Origin(xyz=(0.0, 0.0, -upper_hub_height / 2.0)),
        material=smoke,
        name="upper_tier",
    )
    upper_tray.inertial = Inertial.from_geometry(
        Cylinder(radius=0.136, length=upper_hub_height),
        mass=0.9,
    )
    for clip_index in range(3):
        angle = (2.0 * math.pi * clip_index) / 3.0 + (math.pi / 3.0)
        clip_x = math.cos(angle) * 0.0205
        clip_y = math.sin(angle) * 0.0205
        upper_tray.visual(
            Box((0.010, 0.014, 0.008)),
            origin=Origin(xyz=(clip_x, clip_y, -0.037)),
            material=smoke,
            name=f"upper_support_clip_{clip_index}",
        )
        upper_tray.visual(
            Box((0.010, 0.014, 0.004)),
            origin=Origin(xyz=(clip_x, clip_y, 0.041)),
            material=smoke,
            name=f"upper_capture_clip_{clip_index}",
        )

    model.articulation(
        "lower_tray_spin",
        ArticulationType.CONTINUOUS,
        parent=center_post,
        child=lower_tray,
        origin=Origin(xyz=(0.0, 0.0, lower_joint_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=6.0, velocity=8.0),
    )
    model.articulation(
        "upper_tray_spin",
        ArticulationType.CONTINUOUS,
        parent=center_post,
        child=upper_tray,
        origin=Origin(xyz=(0.0, 0.0, upper_joint_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=5.0, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    center_post = object_model.get_part("center_post")
    lower_tray = object_model.get_part("lower_tray")
    upper_tray = object_model.get_part("upper_tray")
    lower_tray_spin = object_model.get_articulation("lower_tray_spin")
    upper_tray_spin = object_model.get_articulation("upper_tray_spin")

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
        "lower tray spins on vertical axis",
        lower_tray_spin.axis == (0.0, 0.0, 1.0),
        f"unexpected lower tray axis: {lower_tray_spin.axis}",
    )
    ctx.check(
        "upper tray spins on vertical axis",
        upper_tray_spin.axis == (0.0, 0.0, 1.0),
        f"unexpected upper tray axis: {upper_tray_spin.axis}",
    )
    ctx.check(
        "both trays use continuous rotation",
        lower_tray_spin.motion_limits is not None
        and upper_tray_spin.motion_limits is not None
        and lower_tray_spin.motion_limits.lower is None
        and lower_tray_spin.motion_limits.upper is None
        and upper_tray_spin.motion_limits.lower is None
        and upper_tray_spin.motion_limits.upper is None,
        "one or both tray joints are not configured as continuous spins",
    )

    ctx.expect_contact(lower_tray, center_post)
    ctx.expect_contact(upper_tray, center_post)
    ctx.expect_origin_gap(upper_tray, lower_tray, axis="z", min_gap=0.16, max_gap=0.19)
    ctx.expect_gap(upper_tray, lower_tray, axis="z", min_gap=0.08, max_gap=0.10)
    ctx.expect_overlap(lower_tray, upper_tray, axes="xy", min_overlap=0.26)

    with ctx.pose({lower_tray_spin: 1.7, upper_tray_spin: -1.1}):
        ctx.expect_contact(lower_tray, center_post)
        ctx.expect_contact(upper_tray, center_post)
        ctx.expect_gap(upper_tray, lower_tray, axis="z", min_gap=0.08, max_gap=0.10)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
