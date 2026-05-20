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
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _shell_mesh(
    name: str,
    outer_profile: list[tuple[float, float]],
    inner_profile: list[tuple[float, float]],
):
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer_profile,
            inner_profile,
            segments=72,
            start_cap="flat",
            end_cap="flat",
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="macro_lens")

    barrel_black = model.material("barrel_black", rgba=(0.10, 0.10, 0.11, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.14, 0.14, 0.15, 1.0))
    mount_metal = model.material("mount_metal", rgba=(0.71, 0.73, 0.75, 1.0))
    glass = model.material("glass", rgba=(0.42, 0.56, 0.67, 0.38))
    pin_metal = model.material("pin_metal", rgba=(0.88, 0.89, 0.90, 1.0))

    outer_shell = _shell_mesh(
        "outer_barrel_shell_v2",
        [
            (0.0310, 0.010),
            (0.0325, 0.016),
            (0.0356, 0.022),
            (0.0356, 0.064),
            (0.0343, 0.070),
            (0.0336, 0.082),
        ],
        [
            (0.0265, 0.010),
            (0.0265, 0.018),
            (0.0322, 0.022),
            (0.0322, 0.064),
            (0.0312, 0.070),
            (0.0312, 0.082),
        ],
    )
    inner_shell = _shell_mesh(
        "inner_barrel_shell_v2",
        [
            (0.0288, 0.000),
            (0.0288, 0.040),
            (0.0294, 0.044),
            (0.0300, 0.052),
        ],
        [
            (0.0245, 0.002),
            (0.0245, 0.046),
            (0.0252, 0.050),
        ],
    )
    outer_barrel = model.part("outer_barrel")
    outer_barrel.visual(outer_shell, material=barrel_black, name="outer_shell")
    outer_barrel.visual(
        Cylinder(radius=0.0363, length=0.0040),
        origin=Origin(xyz=(0.0, 0.0, 0.0330)),
        material=barrel_black,
        name="focus_support_collar",
    )
    outer_barrel.visual(
        Cylinder(radius=0.0318, length=0.0060),
        origin=Origin(xyz=(0.0, 0.0, 0.0170)),
        material=barrel_black,
        name="rear_coupling_sleeve",
    )
    outer_barrel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0356, length=0.082),
        mass=0.32,
        origin=Origin(xyz=(0.0, 0.0, 0.041)),
    )

    rear_mount = model.part("rear_mount")
    rear_mount.visual(
        Cylinder(radius=0.0305, length=0.0040),
        origin=Origin(xyz=(0.0, 0.0, 0.0020)),
        material=mount_metal,
        name="mount_shell",
    )
    rear_mount.visual(
        Cylinder(radius=0.0250, length=0.0100),
        origin=Origin(xyz=(0.0, 0.0, 0.0090)),
        material=mount_metal,
        name="mount_throat",
    )
    for index, angle in enumerate((0.20, 2.30, 4.35)):
        rear_mount.visual(
            Box((0.0040, 0.0110, 0.0020)),
            origin=Origin(
                xyz=(0.0270 * math.cos(angle), 0.0270 * math.sin(angle), 0.0010),
                rpy=(0.0, 0.0, angle),
            ),
            material=mount_metal,
            name=f"bayonet_lug_{index}",
        )
    rear_mount.visual(
        Cylinder(radius=0.0015, length=0.0020),
        origin=Origin(xyz=(0.0, 0.0260, 0.0010)),
        material=pin_metal,
        name="locking_pin",
    )
    rear_mount.visual(
        Cylinder(radius=0.0183, length=0.0035),
        origin=Origin(xyz=(0.0, 0.0, 0.0095)),
        material=glass,
        name="rear_element",
    )
    rear_mount.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0305, length=0.014),
        mass=0.07,
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
    )

    focus_ring = model.part("focus_ring")
    for index in range(24):
        angle = (2.0 * math.pi * index) / 24.0
        focus_ring.visual(
            Box((0.0046, 0.0108, 0.0260)),
            origin=Origin(
                xyz=(0.0385 * math.cos(angle), 0.0385 * math.sin(angle), 0.0),
                rpy=(0.0, 0.0, angle),
            ),
            material=rubber_black,
            name=f"focus_grip_block_{index:02d}",
        )
    focus_ring.visual(
        Cylinder(radius=0.0402, length=0.0010),
        origin=Origin(xyz=(0.0, 0.0, -0.0125)),
        material=rubber_black,
        name="focus_grip",
    )
    focus_ring.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0408, length=0.026),
        mass=0.08,
        origin=Origin(),
    )

    inner_barrel = model.part("inner_barrel")
    inner_barrel.visual(inner_shell, material=barrel_black, name="inner_shell")
    inner_barrel.visual(
        Cylinder(radius=0.0248, length=0.0030),
        origin=Origin(xyz=(0.0, 0.0, 0.0465)),
        material=glass,
        name="front_element",
    )
    inner_barrel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0300, length=0.052),
        mass=0.18,
        origin=Origin(xyz=(0.0, 0.0, 0.026)),
    )

    model.articulation(
        "outer_to_rear_mount",
        ArticulationType.FIXED,
        parent=outer_barrel,
        child=rear_mount,
        origin=Origin(),
    )
    model.articulation(
        "outer_to_focus_ring",
        ArticulationType.REVOLUTE,
        parent=outer_barrel,
        child=focus_ring,
        origin=Origin(xyz=(0.0, 0.0, 0.046)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.6,
            velocity=4.0,
            lower=-1.5,
            upper=1.5,
        ),
    )
    model.articulation(
        "outer_to_inner_barrel",
        ArticulationType.PRISMATIC,
        parent=outer_barrel,
        child=inner_barrel,
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=0.05,
            lower=0.0,
            upper=0.028,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    outer_barrel = object_model.get_part("outer_barrel")
    rear_mount = object_model.get_part("rear_mount")
    focus_ring = object_model.get_part("focus_ring")
    inner_barrel = object_model.get_part("inner_barrel")

    focus_joint = object_model.get_articulation("outer_to_focus_ring")
    extension_joint = object_model.get_articulation("outer_to_inner_barrel")

    outer_barrel.get_visual("outer_shell")
    rear_mount.get_visual("mount_shell")
    rear_mount.get_visual("mount_throat")
    rear_mount.get_visual("bayonet_lug_0")
    rear_mount.get_visual("locking_pin")
    focus_ring.get_visual("focus_grip")
    inner_barrel.get_visual("front_element")

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
        "focus_joint_axis_is_optical",
        tuple(focus_joint.axis) == (0.0, 0.0, 1.0),
        details=f"axis={focus_joint.axis}",
    )
    ctx.check(
        "extension_joint_axis_is_optical",
        tuple(extension_joint.axis) == (0.0, 0.0, 1.0),
        details=f"axis={extension_joint.axis}",
    )
    ctx.check(
        "focus_joint_throw_is_realistic",
        focus_joint.motion_limits is not None
        and focus_joint.motion_limits.lower is not None
        and focus_joint.motion_limits.upper is not None
        and 2.5 <= focus_joint.motion_limits.upper - focus_joint.motion_limits.lower <= 3.5,
        details="Focus ring should turn through a realistic manual-focus arc.",
    )
    ctx.check(
        "extension_travel_is_macro_scale",
        extension_joint.motion_limits is not None
        and extension_joint.motion_limits.upper is not None
        and 0.020 <= extension_joint.motion_limits.upper <= 0.040,
        details="Inner barrel should extend a few centimeters for close focus.",
    )

    ctx.expect_contact(
        rear_mount,
        outer_barrel,
        elem_a="mount_throat",
        elem_b="rear_coupling_sleeve",
        name="rear_mount_seats_in_outer_barrel",
    )
    ctx.expect_origin_distance(
        focus_ring,
        outer_barrel,
        axes="xy",
        max_dist=1e-6,
        name="focus_ring_concentric_with_outer_barrel",
    )
    ctx.expect_origin_distance(
        inner_barrel,
        outer_barrel,
        axes="xy",
        max_dist=1e-6,
        name="inner_barrel_concentric_with_outer_barrel",
    )
    ctx.expect_overlap(focus_ring, outer_barrel, axes="xy", min_overlap=0.060)
    ctx.expect_within(outer_barrel, focus_ring, axes="xy", margin=0.006)
    ctx.expect_overlap(rear_mount, outer_barrel, axes="xy", min_overlap=0.045)
    ctx.expect_within(rear_mount, outer_barrel, axes="xy", margin=0.006)
    ctx.expect_within(inner_barrel, outer_barrel, axes="xy", margin=0.004)

    outer_aabb = ctx.part_world_aabb(outer_barrel)
    mount_aabb = ctx.part_world_aabb(rear_mount)
    inner_aabb = ctx.part_world_aabb(inner_barrel)
    assert outer_aabb is not None
    assert mount_aabb is not None
    assert inner_aabb is not None

    compact_length = inner_aabb[1][2] - mount_aabb[0][2]
    outer_diameter = outer_aabb[1][0] - outer_aabb[0][0]
    ctx.check(
        "compact_macro_lens_length",
        0.078 <= compact_length <= 0.090,
        details=f"compact_length={compact_length:.4f}",
    )
    ctx.check(
        "compact_barrel_diameter",
        0.066 <= outer_diameter <= 0.074,
        details=f"outer_diameter={outer_diameter:.4f}",
    )

    extension_limits = extension_joint.motion_limits
    assert extension_limits is not None
    assert extension_limits.lower is not None
    assert extension_limits.upper is not None

    with ctx.pose({extension_joint: extension_limits.lower}):
        ctx.fail_if_parts_overlap_in_current_pose(name="inner_barrel_lower_no_overlap")
        ctx.fail_if_isolated_parts(name="inner_barrel_lower_no_floating")
        ctx.expect_within(inner_barrel, outer_barrel, axes="xy", margin=0.004, name="inner_barrel_lower_within")
        collapsed_aabb = ctx.part_world_aabb(inner_barrel)
        assert collapsed_aabb is not None

    with ctx.pose({extension_joint: extension_limits.upper}):
        ctx.fail_if_parts_overlap_in_current_pose(name="inner_barrel_upper_no_overlap")
        ctx.fail_if_isolated_parts(name="inner_barrel_upper_no_floating")
        ctx.expect_within(inner_barrel, outer_barrel, axes="xy", margin=0.004, name="inner_barrel_upper_within")
        extended_aabb = ctx.part_world_aabb(inner_barrel)
        assert extended_aabb is not None
        ctx.check(
            "inner_barrel_extends_forward",
            extended_aabb[1][2] >= collapsed_aabb[1][2] + 0.024,
            details=f"collapsed_front={collapsed_aabb[1][2]:.4f}, extended_front={extended_aabb[1][2]:.4f}",
        )

    focus_limits = focus_joint.motion_limits
    assert focus_limits is not None
    assert focus_limits.lower is not None
    assert focus_limits.upper is not None
    for pose_name, angle in (
        ("focus_ring_lower_pose", focus_limits.lower),
        ("focus_ring_upper_pose", focus_limits.upper),
    ):
        with ctx.pose({focus_joint: angle}):
            ctx.fail_if_parts_overlap_in_current_pose(name=f"{pose_name}_no_overlap")
            ctx.fail_if_isolated_parts(name=f"{pose_name}_no_floating")
            ctx.expect_within(
                outer_barrel,
                focus_ring,
                axes="xy",
                margin=0.006,
                name=f"{pose_name}_surrounds_outer_barrel",
            )

    with ctx.pose({focus_joint: focus_limits.upper, extension_joint: extension_limits.upper}):
        ctx.fail_if_parts_overlap_in_current_pose(name="combined_pose_no_overlap")
        ctx.fail_if_isolated_parts(name="combined_pose_no_floating")
        combined_inner_aabb = ctx.part_world_aabb(inner_barrel)
        assert combined_inner_aabb is not None
        ctx.check(
            "extended_macro_length",
            0.104 <= combined_inner_aabb[1][2] - mount_aabb[0][2] <= 0.118,
            details=f"extended_length={combined_inner_aabb[1][2] - mount_aabb[0][2]:.4f}",
        )

    ctx.fail_if_articulation_overlaps(max_pose_samples=24)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
