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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)


def _xz_section(
    width: float,
    height: float,
    corner_radius: float,
    y: float,
    *,
    corner_segments: int = 8,
) -> list[tuple[float, float, float]]:
    return [(x, y, z) for x, z in rounded_rect_profile(width, height, corner_radius, corner_segments=corner_segments)]


def _beveled_slab_mesh(
    width: float,
    height: float,
    thickness: float,
    corner_radius: float,
    *,
    bevel: float,
    name: str,
) -> object:
    mid = _xz_section(width, height, corner_radius, 0.0)
    back = _xz_section(
        width - bevel * 2.0,
        height - bevel * 2.0,
        max(corner_radius - bevel, 0.001),
        -thickness * 0.5,
    )
    front = _xz_section(
        width - bevel,
        height - bevel,
        max(corner_radius - bevel * 0.5, 0.001),
        thickness * 0.5,
    )
    return mesh_from_geometry(section_loft([back, mid, front]), name)


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]]) -> tuple[float, float, float]:
    return tuple((aabb[0][axis] + aabb[1][axis]) * 0.5 for axis in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="flip_phone_rotating_camera")

    body_graphite = model.material("body_graphite", rgba=(0.20, 0.22, 0.24, 1.0))
    hinge_metal = model.material("hinge_metal", rgba=(0.54, 0.56, 0.60, 1.0))
    keypad_black = model.material("keypad_black", rgba=(0.08, 0.09, 0.10, 1.0))
    glass_black = model.material("glass_black", rgba=(0.10, 0.12, 0.14, 1.0))
    screen_glow = model.material("screen_glow", rgba=(0.26, 0.49, 0.66, 0.78))
    lens_glass = model.material("lens_glass", rgba=(0.06, 0.08, 0.11, 0.95))
    accent_silver = model.material("accent_silver", rgba=(0.72, 0.73, 0.76, 1.0))
    flash_white = model.material("flash_white", rgba=(0.90, 0.92, 0.85, 0.95))

    phone_width = 0.058
    lower_height = 0.087
    upper_height = 0.083
    lower_thickness = 0.0142
    upper_thickness = 0.0112
    corner_radius = 0.008
    side_bevel = 0.0011
    hinge_radius = 0.0046
    shell_seam = 0.0008
    shell_forward_offset = 0.0023

    lower_shell_mesh = _beveled_slab_mesh(
        phone_width,
        lower_height,
        lower_thickness,
        corner_radius,
        bevel=side_bevel,
        name="lower_body_shell",
    )
    upper_shell_mesh = _beveled_slab_mesh(
        phone_width,
        upper_height,
        upper_thickness,
        corner_radius,
        bevel=side_bevel,
        name="upper_body_shell",
    )
    camera_pod_mesh = _beveled_slab_mesh(
        0.013,
        0.022,
        0.0076,
        0.0032,
        bevel=0.0007,
        name="camera_pod_shell",
    )

    lower_body = model.part("lower_body")
    lower_body.visual(
        lower_shell_mesh,
        origin=Origin(xyz=(0.0, shell_forward_offset, -(lower_height * 0.5 + shell_seam * 0.5))),
        material=body_graphite,
        name="lower_shell",
    )
    lower_body.visual(
        Box((0.049, 0.0012, 0.056)),
        origin=Origin(xyz=(0.0, shell_forward_offset + lower_thickness * 0.5 + 0.0006, -0.051)),
        material=keypad_black,
        name="keypad_deck",
    )
    lower_body.visual(
        Box((0.020, 0.0018, 0.010)),
        origin=Origin(xyz=(0.0, shell_forward_offset + lower_thickness * 0.5 + 0.0009, -0.019)),
        material=keypad_black,
        name="navigation_pad",
    )
    lower_body.visual(
        Box((0.007, 0.0020, 0.007)),
        origin=Origin(xyz=(-0.014, shell_forward_offset + lower_thickness * 0.5 + 0.0010, -0.019)),
        material=body_graphite,
        name="soft_key_left",
    )
    lower_body.visual(
        Box((0.007, 0.0020, 0.007)),
        origin=Origin(xyz=(0.014, shell_forward_offset + lower_thickness * 0.5 + 0.0010, -0.019)),
        material=body_graphite,
        name="soft_key_right",
    )
    key_pitch_x = 0.014
    key_pitch_z = 0.011
    key_base_z = -0.037
    for row in range(4):
        for col in range(3):
            lower_body.visual(
                Box((0.010, 0.0019, 0.0074)),
                origin=Origin(
                    xyz=(
                        (col - 1) * key_pitch_x,
                        shell_forward_offset + lower_thickness * 0.5 + 0.00095,
                        key_base_z - row * key_pitch_z,
                    )
                ),
                material=body_graphite,
                name=f"key_{row}_{col}",
            )
    lower_body.visual(
        Cylinder(radius=hinge_radius, length=0.022),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hinge_metal,
        name="center_barrel",
    )
    lower_body.inertial = Inertial.from_geometry(
        Box((phone_width, lower_thickness + 0.004, lower_height + 0.010)),
        mass=0.112,
        origin=Origin(xyz=(0.0, shell_forward_offset, -(lower_height * 0.5 + shell_seam * 0.5))),
    )

    upper_body = model.part("upper_body")
    upper_body.visual(
        upper_shell_mesh,
        origin=Origin(xyz=(0.0, shell_forward_offset, upper_height * 0.5 + shell_seam * 0.5)),
        material=body_graphite,
        name="upper_shell",
    )
    upper_body.visual(
        Box((0.051, 0.0010, 0.071)),
        origin=Origin(xyz=(0.0, shell_forward_offset + upper_thickness * 0.5 + 0.0005, 0.046)),
        material=glass_black,
        name="front_frame",
    )
    upper_body.visual(
        Box((0.043, 0.0012, 0.061)),
        origin=Origin(xyz=(0.0, shell_forward_offset + upper_thickness * 0.5 + 0.0006, 0.046)),
        material=screen_glow,
        name="screen_glass",
    )
    upper_body.visual(
        Box((0.014, 0.0012, 0.0022)),
        origin=Origin(xyz=(0.0, shell_forward_offset + upper_thickness * 0.5 + 0.0006, 0.076)),
        material=keypad_black,
        name="earpiece_slot",
    )
    upper_body.visual(
        Cylinder(radius=hinge_radius, length=0.014),
        origin=Origin(xyz=(-0.018, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hinge_metal,
        name="left_barrel",
    )
    upper_body.visual(
        Cylinder(radius=hinge_radius, length=0.014),
        origin=Origin(xyz=(0.018, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hinge_metal,
        name="right_barrel",
    )

    camera_axis_x = phone_width * 0.5 - 0.007
    camera_axis_y = -0.0028
    camera_axis_z = upper_height - 0.018
    upper_body.visual(
        Cylinder(radius=0.0028, length=0.009),
        origin=Origin(
            xyz=(camera_axis_x, camera_axis_y, camera_axis_z),
        ),
        material=hinge_metal,
        name="camera_mount_center",
    )
    upper_body.visual(
        Box((0.0045, 0.004, 0.024)),
        origin=Origin(xyz=(camera_axis_x - 0.002, camera_axis_y, camera_axis_z)),
        material=accent_silver,
        name="camera_mount_bridge",
    )
    upper_body.inertial = Inertial.from_geometry(
        Box((phone_width, upper_thickness + 0.004, upper_height + 0.010)),
        mass=0.088,
        origin=Origin(xyz=(0.0, shell_forward_offset, upper_height * 0.5 + shell_seam * 0.5)),
    )

    camera_pod = model.part("camera_pod")
    camera_pod.visual(
        Cylinder(radius=0.0028, length=0.0045),
        origin=Origin(xyz=(0.0, 0.0, -0.00675)),
        material=hinge_metal,
        name="camera_knuckle_bottom",
    )
    camera_pod.visual(
        Cylinder(radius=0.0028, length=0.0045),
        origin=Origin(xyz=(0.0, 0.0, 0.00675)),
        material=hinge_metal,
        name="camera_knuckle_top",
    )
    camera_pod.visual(
        camera_pod_mesh,
        origin=Origin(xyz=(-0.0032, -0.0049, 0.0)),
        material=body_graphite,
        name="camera_housing",
    )
    camera_pod.visual(
        Box((0.0095, 0.0016, 0.019)),
        origin=Origin(xyz=(-0.0032, -0.0086, 0.0)),
        material=glass_black,
        name="camera_glass_strip",
    )
    camera_pod.visual(
        Cylinder(radius=0.0031, length=0.0018),
        origin=Origin(xyz=(-0.0032, -0.0094, 0.0058), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=lens_glass,
        name="lens_upper",
    )
    camera_pod.visual(
        Cylinder(radius=0.0031, length=0.0018),
        origin=Origin(xyz=(-0.0032, -0.0094, -0.0058), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=lens_glass,
        name="lens_lower",
    )
    camera_pod.visual(
        Cylinder(radius=0.0016, length=0.0014),
        origin=Origin(xyz=(0.0012, -0.0092, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=flash_white,
        name="flash",
    )
    camera_pod.inertial = Inertial.from_geometry(
        Box((0.015, 0.012, 0.024)),
        mass=0.018,
        origin=Origin(xyz=(-0.0032, -0.0049, 0.0)),
    )

    model.articulation(
        "phone_hinge",
        ArticulationType.REVOLUTE,
        parent=lower_body,
        child=upper_body,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=3.5,
            lower=-2.90,
            upper=0.0,
        ),
    )
    model.articulation(
        "camera_swivel",
        ArticulationType.REVOLUTE,
        parent=upper_body,
        child=camera_pod,
        origin=Origin(xyz=(camera_axis_x, camera_axis_y, camera_axis_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.4,
            velocity=4.0,
            lower=0.0,
            upper=math.pi,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    lower_body = object_model.get_part("lower_body")
    upper_body = object_model.get_part("upper_body")
    camera_pod = object_model.get_part("camera_pod")
    phone_hinge = object_model.get_articulation("phone_hinge")
    camera_swivel = object_model.get_articulation("camera_swivel")

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
        lower_body,
        upper_body,
        elem_a="center_barrel",
        elem_b="left_barrel",
        name="left hinge knuckle contacts center barrel",
    )
    ctx.expect_contact(
        lower_body,
        upper_body,
        elem_a="center_barrel",
        elem_b="right_barrel",
        name="right hinge knuckle contacts center barrel",
    )
    ctx.expect_contact(
        upper_body,
        camera_pod,
        elem_a="camera_mount_center",
        elem_b="camera_knuckle_bottom",
        name="camera lower knuckle contacts upper mount",
    )
    ctx.expect_contact(
        upper_body,
        camera_pod,
        elem_a="camera_mount_center",
        elem_b="camera_knuckle_top",
        name="camera upper knuckle contacts upper mount",
    )
    ctx.expect_gap(
        upper_body,
        lower_body,
        axis="z",
        positive_elem="upper_shell",
        negative_elem="lower_shell",
        min_gap=0.0004,
        max_gap=0.0013,
        name="shell seam gap is narrow but open",
    )
    ctx.check(
        "phone hinge axis runs across phone width",
        tuple(phone_hinge.axis) == (1.0, 0.0, 0.0),
        details=f"unexpected phone hinge axis: {phone_hinge.axis}",
    )
    ctx.check(
        "camera swivel axis is vertical",
        tuple(camera_swivel.axis) == (0.0, 0.0, 1.0),
        details=f"unexpected camera swivel axis: {camera_swivel.axis}",
    )

    screen_open_aabb = ctx.part_element_world_aabb(upper_body, elem="screen_glass")
    lens_rear_aabb = ctx.part_element_world_aabb(camera_pod, elem="lens_upper")
    if screen_open_aabb is not None:
        screen_open_center = _aabb_center(screen_open_aabb)
        with ctx.pose({phone_hinge: -1.45}):
            screen_folded_aabb = ctx.part_element_world_aabb(upper_body, elem="screen_glass")
            if screen_folded_aabb is not None:
                screen_folded_center = _aabb_center(screen_folded_aabb)
                ctx.check(
                    "upper display body folds toward keypad",
                    (
                        screen_folded_center[1] > screen_open_center[1] + 0.02
                        and screen_folded_center[2] < screen_open_center[2] - 0.03
                    ),
                    details=(
                        f"screen center did not rotate inward/downward enough: "
                        f"open=({screen_open_center[1]:.4f}, {screen_open_center[2]:.4f}), "
                        f"folded=({screen_folded_center[1]:.4f}, {screen_folded_center[2]:.4f})"
                    ),
                )
    if lens_rear_aabb is not None:
        lens_rear_center = _aabb_center(lens_rear_aabb)
        with ctx.pose({camera_swivel: math.pi}):
            lens_front_aabb = ctx.part_element_world_aabb(camera_pod, elem="lens_upper")
            if lens_front_aabb is not None:
                lens_front_center = _aabb_center(lens_front_aabb)
                ctx.check(
                    "camera pod flips from rear to front",
                    lens_rear_center[1] < -0.004 and lens_front_center[1] > 0.004,
                    details=(
                        f"lens center did not cross phone thickness: "
                        f"rear={lens_rear_center[1]:.4f}, front={lens_front_center[1]:.4f}"
                    ),
                )
                ctx.check(
                    "camera pod keeps the same vertical elevation while swiveling",
                    abs(lens_front_center[2] - lens_rear_center[2]) < 0.001,
                    details=(
                        f"lens z drifted during swivel: "
                        f"rear={lens_rear_center[2]:.4f}, front={lens_front_center[2]:.4f}"
                    ),
                )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
