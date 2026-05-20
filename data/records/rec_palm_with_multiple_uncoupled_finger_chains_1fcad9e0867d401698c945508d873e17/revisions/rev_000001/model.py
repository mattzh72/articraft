from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import cos, pi, sin

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def _rotate_xy(x: float, y: float, yaw: float) -> tuple[float, float]:
    return (x * cos(yaw) - y * sin(yaw), x * sin(yaw) + y * cos(yaw))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="radial_gripper_palm")

    hub_material = model.material("hub_graphite", rgba=(0.23, 0.25, 0.28, 1.0))
    link_material = model.material("link_metal", rgba=(0.73, 0.76, 0.79, 1.0))
    pad_material = model.material("pad_rubber", rgba=(0.10, 0.10, 0.11, 1.0))

    hub_size = 0.120
    hub_thickness = 0.028
    hub_half = hub_size / 2.0

    finger_width = 0.024
    proximal_width = 0.022
    tab_thickness = 0.007
    ear_thickness = 0.0035
    distal_thickness = 0.016

    pivot_radius = 0.012
    ear_z = (tab_thickness + ear_thickness) / 2.0
    root_bridge_len = pivot_radius
    root_pivot_offset = hub_half + pivot_radius

    proximal_length = 0.070
    distal_bridge_len = 0.018
    distal_body_start = 0.015
    distal_body_len = 0.047
    distal_tip_center = 0.060
    fingertip_pad_len = 0.008

    side_configs = (
        ("east", 0.0),
        ("north", pi / 2.0),
        ("west", pi),
        ("south", -pi / 2.0),
    )

    hub = model.part("hub")
    hub.visual(
        Box((hub_size, hub_size, hub_thickness)),
        material=hub_material,
        name="hub_block",
    )

    for side_name, yaw in side_configs:
        bridge_x, bridge_y = _rotate_xy(hub_half + root_bridge_len / 2.0, 0.0, yaw)
        pivot_x, pivot_y = _rotate_xy(root_pivot_offset, 0.0, yaw)
        for label, z_sign in (("upper", 1.0), ("lower", -1.0)):
            z_pos = z_sign * ear_z
            hub.visual(
                Box((root_bridge_len, finger_width, ear_thickness)),
                origin=Origin(xyz=(bridge_x, bridge_y, z_pos), rpy=(0.0, 0.0, yaw)),
                material=hub_material,
                name=f"{side_name}_root_bridge_{label}",
            )
            hub.visual(
                Cylinder(radius=pivot_radius, length=ear_thickness),
                origin=Origin(xyz=(pivot_x, pivot_y, z_pos)),
                material=hub_material,
                name=f"{side_name}_root_ear_{label}",
            )

    for side_name, yaw in side_configs:
        proximal = model.part(f"{side_name}_proximal")
        distal = model.part(f"{side_name}_distal")

        proximal.visual(
            Box((proximal_length, proximal_width, tab_thickness)),
            origin=Origin(xyz=(proximal_length / 2.0, 0.0, 0.0)),
            material=link_material,
            name="body",
        )
        proximal.visual(
            Cylinder(radius=pivot_radius, length=tab_thickness),
            material=link_material,
            name="root_tab",
        )
        proximal.visual(
            Cylinder(radius=pivot_radius, length=tab_thickness),
            origin=Origin(xyz=(proximal_length, 0.0, 0.0)),
            material=link_material,
            name="hinge_tab",
        )

        for label, z_sign in (("upper", 1.0), ("lower", -1.0)):
            z_pos = z_sign * ear_z
            distal.visual(
                Box((distal_bridge_len, finger_width, ear_thickness)),
                origin=Origin(xyz=(distal_bridge_len / 2.0, 0.0, z_pos)),
                material=link_material,
                name=f"hinge_bridge_{label}",
            )
            distal.visual(
                Cylinder(radius=pivot_radius, length=ear_thickness),
                origin=Origin(xyz=(0.0, 0.0, z_pos)),
                material=link_material,
                name=f"hinge_ear_{label}",
            )

        distal.visual(
            Box((distal_body_len, finger_width, distal_thickness)),
            origin=Origin(
                xyz=(distal_body_start + distal_body_len / 2.0, 0.0, 0.0)
            ),
            material=link_material,
            name="body",
        )
        distal.visual(
            Cylinder(radius=finger_width / 2.0, length=distal_thickness),
            origin=Origin(xyz=(distal_tip_center, 0.0, 0.0)),
            material=link_material,
            name="tip_round",
        )
        distal.visual(
            Box((fingertip_pad_len, finger_width * 0.88, distal_thickness * 0.75)),
            origin=Origin(
                xyz=(distal_tip_center + fingertip_pad_len / 2.0 + pivot_radius, 0.0, 0.0)
            ),
            material=pad_material,
            name="grip_pad",
        )

        root_x, root_y = _rotate_xy(root_pivot_offset, 0.0, yaw)
        model.articulation(
            f"{side_name}_root_joint",
            ArticulationType.REVOLUTE,
            parent=hub,
            child=proximal,
            origin=Origin(xyz=(root_x, root_y, 0.0), rpy=(0.0, 0.0, yaw)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=20.0,
                velocity=2.5,
                lower=-0.85,
                upper=0.85,
            ),
        )
        model.articulation(
            f"{side_name}_mid_joint",
            ArticulationType.REVOLUTE,
            parent=proximal,
            child=distal,
            origin=Origin(xyz=(proximal_length, 0.0, 0.0)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=15.0,
                velocity=3.0,
                lower=-0.15,
                upper=1.35,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
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

    hub = object_model.get_part("hub")
    side_names = ("east", "north", "west", "south")

    root_parts = {part.name for part in object_model.root_parts()}
    ctx.check(
        "single_hub_root",
        root_parts == {"hub"},
        f"expected only 'hub' as root part, got {sorted(root_parts)}",
    )

    representative_root_axes = []
    representative_mid_axes = []

    for side_name in side_names:
        proximal = object_model.get_part(f"{side_name}_proximal")
        distal = object_model.get_part(f"{side_name}_distal")
        root_joint = object_model.get_articulation(f"{side_name}_root_joint")
        mid_joint = object_model.get_articulation(f"{side_name}_mid_joint")

        representative_root_axes.append(tuple(root_joint.axis))
        representative_mid_axes.append(tuple(mid_joint.axis))

        ctx.expect_contact(hub, proximal, name=f"{side_name}_root_pivot_captured")
        ctx.expect_contact(
            proximal,
            distal,
            name=f"{side_name}_mid_pivot_captured",
        )

    ctx.check(
        "all_root_axes_vertical",
        all(axis == (0.0, 0.0, 1.0) for axis in representative_root_axes),
        f"unexpected root axes: {representative_root_axes}",
    )
    ctx.check(
        "all_mid_axes_vertical",
        all(axis == (0.0, 0.0, 1.0) for axis in representative_mid_axes),
        f"unexpected mid axes: {representative_mid_axes}",
    )

    east_root_joint = object_model.get_articulation("east_root_joint")
    east_mid_joint = object_model.get_articulation("east_mid_joint")
    east_proximal = object_model.get_part("east_proximal")
    east_distal = object_model.get_part("east_distal")

    with ctx.pose({east_root_joint: 0.55, east_mid_joint: 0.95}):
        ctx.expect_contact(
            hub,
            east_proximal,
            name="east_root_captured_in_bent_pose",
        )
        ctx.expect_contact(
            east_proximal,
            east_distal,
            name="east_mid_captured_in_bent_pose",
        )
        ctx.fail_if_parts_overlap_in_current_pose(name="east_finger_bent_pose_clear")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
