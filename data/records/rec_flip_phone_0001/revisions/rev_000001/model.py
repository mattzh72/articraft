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
    Inertial,
    LoftGeometry,
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


def _named_material(name: str, rgba: tuple[float, float, float, float]) -> Material:
    for kwargs in ({"name": name, "color": rgba}, {"name": name, "rgba": rgba}):
        try:
            return Material(**kwargs)
        except TypeError:
            pass
    return Material(name, rgba)


def _rounded_profile(
    width: float, depth: float, radius: float, z: float
) -> list[tuple[float, float, float]]:
    clamped_radius = max(0.0005, min(radius, width * 0.5 - 1e-4, depth * 0.5 - 1e-4))
    return [
        (x, y, z) for x, y in rounded_rect_profile(width, depth, clamped_radius, corner_segments=10)
    ]


def _shell_mesh(
    filename: str,
    width: float,
    depth: float,
    thickness: float,
    radius: float,
    top_scale: tuple[float, float],
) -> object:
    profiles = [
        _rounded_profile(width, depth, radius, -thickness * 0.5),
        _rounded_profile(width * 0.988, depth * 0.992, radius * 0.98, -thickness * 0.18),
        _rounded_profile(width * 0.972, depth * 0.978, radius * 0.94, thickness * 0.18),
        _rounded_profile(
            width * top_scale[0], depth * top_scale[1], radius * 0.88, thickness * 0.5
        ),
    ]
    return mesh_from_geometry(
        LoftGeometry(profiles, cap=True, closed=True), ASSETS.mesh_path(filename)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="flip_phone", assets=ASSETS)

    body_black = _named_material("body_black", (0.10, 0.11, 0.12, 1.0))
    side_metal = _named_material("side_metal", (0.62, 0.65, 0.69, 1.0))
    key_gray = _named_material("key_gray", (0.36, 0.38, 0.41, 1.0))
    dark_glass = _named_material("dark_glass", (0.08, 0.11, 0.14, 0.58))
    display_black = _named_material("display_black", (0.03, 0.05, 0.07, 1.0))
    accent_silver = _named_material("accent_silver", (0.80, 0.82, 0.85, 1.0))
    lens_black = _named_material("lens_black", (0.02, 0.02, 0.03, 1.0))
    model.materials.extend(
        [
            body_black,
            side_metal,
            key_gray,
            dark_glass,
            display_black,
            accent_silver,
            lens_black,
        ]
    )

    lower_w = 0.055
    lower_d = 0.096
    lower_t = 0.0175

    upper_w = 0.0525
    upper_d = 0.083
    upper_t = 0.011
    upper_shell_z = -0.0015

    hinge_y = 0.043
    hinge_z = 0.0248
    hinge_outer_radius = 0.0043
    hinge_inner_radius = 0.0041

    lower_shell = _shell_mesh(
        "lower_body_shell.obj",
        width=lower_w,
        depth=lower_d,
        thickness=lower_t,
        radius=0.010,
        top_scale=(0.944, 0.952),
    )
    upper_shell = _shell_mesh(
        "upper_display_shell.obj",
        width=upper_w,
        depth=upper_d,
        thickness=upper_t,
        radius=0.009,
        top_scale=(0.948, 0.956),
    )

    lower = model.part("lower_body")
    lower.visual(
        lower_shell,
        origin=Origin(xyz=(0.0, 0.0, lower_t * 0.5)),
        material=body_black,
        name="lower_shell",
    )
    lower.visual(
        Box((0.044, 0.070, 0.0018)),
        origin=Origin(xyz=(0.0, -0.007, 0.0158)),
        material=display_black,
        name="keypad_deck",
    )
    lower.visual(
        Box((0.0018, 0.075, 0.0085)),
        origin=Origin(xyz=(lower_w * 0.5 - 0.0010, -0.004, 0.0084)),
        material=side_metal,
        name="right_lower_trim",
    )
    lower.visual(
        Box((0.0018, 0.075, 0.0085)),
        origin=Origin(xyz=(-lower_w * 0.5 + 0.0010, -0.004, 0.0084)),
        material=side_metal,
        name="left_lower_trim",
    )
    lower.visual(
        Box((0.030, 0.004, 0.0030)),
        origin=Origin(xyz=(0.0, -0.0465, 0.0047)),
        material=side_metal,
        name="front_lip",
    )
    lower.visual(
        Box((0.010, 0.0022, 0.0018)),
        origin=Origin(xyz=(0.0, -0.0474, 0.0023)),
        material=lens_black,
        name="microphone_slot",
    )
    lower.visual(
        Cylinder(radius=0.0110, length=0.0017),
        origin=Origin(xyz=(0.0, 0.0215, 0.0160)),
        material=key_gray,
        name="nav_ring",
    )
    lower.visual(
        Cylinder(radius=0.0072, length=0.0019),
        origin=Origin(xyz=(0.0, 0.0215, 0.0161)),
        material=display_black,
        name="nav_ring_inner",
    )
    lower.visual(
        Cylinder(radius=0.0038, length=0.0017),
        origin=Origin(xyz=(0.0, 0.0215, 0.0162)),
        material=accent_silver,
        name="nav_center",
    )
    lower.visual(
        Box((0.013, 0.0052, 0.0016)),
        origin=Origin(xyz=(-0.0138, 0.0345, 0.0161)),
        material=key_gray,
        name="softkey_left",
    )
    lower.visual(
        Box((0.013, 0.0052, 0.0016)),
        origin=Origin(xyz=(0.0138, 0.0345, 0.0161)),
        material=key_gray,
        name="softkey_right",
    )
    lower.visual(
        Box((0.009, 0.0045, 0.0016)),
        origin=Origin(xyz=(-0.0168, 0.0110, 0.0160)),
        material=key_gray,
        name="send_key",
    )
    lower.visual(
        Box((0.009, 0.0045, 0.0016)),
        origin=Origin(xyz=(0.0168, 0.0110, 0.0160)),
        material=key_gray,
        name="end_key",
    )

    button_xs = (-0.0135, 0.0, 0.0135)
    button_ys = (0.001, -0.012, -0.025, -0.038)
    for row, y in enumerate(button_ys):
        for col, x in enumerate(button_xs):
            lower.visual(
                Box((0.0106, 0.0076, 0.0015)),
                origin=Origin(
                    xyz=(
                        x,
                        y,
                        0.01605 + 0.00010 * ((row + col) % 2),
                    )
                ),
                material=key_gray,
                name=f"key_{row}_{col}",
            )

    lower.inertial = Inertial.from_geometry(
        Box((lower_w, lower_d, lower_t)),
        mass=0.085,
        origin=Origin(xyz=(0.0, 0.0, lower_t * 0.5)),
    )

    hinge = model.part("hinge_spine")
    hinge.visual(
        Box((0.049, 0.004, 0.004)),
        origin=Origin(xyz=(0.0, 0.0065, -0.0008)),
        material=side_metal,
        name="rear_spine_bridge",
    )
    hinge.visual(
        Box((0.014, 0.010, 0.0085)),
        origin=Origin(xyz=(-0.017, 0.0010, -0.0040)),
        material=side_metal,
        name="left_mount_cheek",
    )
    hinge.visual(
        Box((0.014, 0.010, 0.0085)),
        origin=Origin(xyz=(0.017, 0.0010, -0.0040)),
        material=side_metal,
        name="right_mount_cheek",
    )
    hinge.visual(
        Cylinder(radius=hinge_outer_radius, length=0.012),
        origin=Origin(
            xyz=(-0.017, 0.0, 0.0),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=side_metal,
        name="left_hinge_barrel",
    )
    hinge.visual(
        Cylinder(radius=hinge_outer_radius, length=0.012),
        origin=Origin(
            xyz=(0.017, 0.0, 0.0),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=side_metal,
        name="right_hinge_barrel",
    )
    hinge.inertial = Inertial.from_geometry(
        Box((0.049, 0.012, 0.012)),
        mass=0.014,
        origin=Origin(xyz=(0.0, 0.0020, -0.0010)),
    )

    upper = model.part("upper_display")
    upper.visual(
        upper_shell,
        origin=Origin(xyz=(0.0, -upper_d * 0.5, upper_shell_z)),
        material=body_black,
        name="upper_shell",
    )
    upper.visual(
        Cylinder(radius=hinge_inner_radius, length=0.020),
        origin=Origin(
            xyz=(0.0, 0.0, 0.0),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=body_black,
        name="center_hinge_barrel",
    )
    upper.visual(
        Box((0.044, 0.064, 0.0016)),
        origin=Origin(xyz=(0.0, -0.0415, -0.0059)),
        material=display_black,
        name="display_bezel",
    )
    upper.visual(
        Box((0.0368, 0.0485, 0.0010)),
        origin=Origin(xyz=(0.0, -0.0430, -0.0065)),
        material=dark_glass,
        name="display_glass",
    )
    upper.visual(
        Box((0.014, 0.0030, 0.0011)),
        origin=Origin(xyz=(0.0, -0.0145, -0.0062)),
        material=lens_black,
        name="earpiece_slot",
    )
    upper.visual(
        Cylinder(radius=0.0024, length=0.0011),
        origin=Origin(xyz=(0.0162, -0.0148, -0.0062)),
        material=lens_black,
        name="camera_lens",
    )
    upper.visual(
        Box((0.0015, 0.063, 0.0065)),
        origin=Origin(xyz=(upper_w * 0.5 - 0.0008, -0.0420, -0.0005)),
        material=side_metal,
        name="right_upper_trim",
    )
    upper.visual(
        Box((0.0015, 0.063, 0.0065)),
        origin=Origin(xyz=(-upper_w * 0.5 + 0.0008, -0.0420, -0.0005)),
        material=side_metal,
        name="left_upper_trim",
    )
    upper.visual(
        Box((0.022, 0.0040, 0.0012)),
        origin=Origin(xyz=(0.0, -0.0772, -0.0058)),
        material=accent_silver,
        name="lower_display_chin",
    )
    upper.inertial = Inertial.from_geometry(
        Box((upper_w, upper_d, upper_t)),
        mass=0.055,
        origin=Origin(xyz=(0.0, -upper_d * 0.5, upper_shell_z)),
    )

    model.articulation(
        "body_to_hinge_spine",
        ArticulationType.FIXED,
        parent="lower_body",
        child="hinge_spine",
        origin=Origin(xyz=(0.0, hinge_y, hinge_z)),
    )
    model.articulation(
        "clamshell_hinge",
        ArticulationType.REVOLUTE,
        parent="hinge_spine",
        child="upper_display",
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=3.2,
            lower=0.0,
            upper=2.35,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=HERE)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.01)
    ctx.fail_if_part_contains_disconnected_geometry_islands(use="visual")
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=160,
        overlap_tol=0.003,
        overlap_volume_tol=0.0,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    ctx.expect_aabb_overlap("hinge_spine", "lower_body", axes="xy", min_overlap=0.008)
    ctx.expect_aabb_gap("hinge_spine", "lower_body", axis="z", max_gap=0.0015, max_penetration=0.0015)
    ctx.expect_aabb_overlap("upper_display", "lower_body", axes="xy", min_overlap=0.03)
    ctx.expect_aabb_gap("upper_display", "lower_body", axis="z", max_gap=0.0010, max_penetration=0.0)
    ctx.expect_joint_motion_axis(
        "clamshell_hinge",
        "upper_display",
        world_axis="z",
        direction="positive",
        min_delta=0.02,
    )

    with ctx.pose(clamshell_hinge=1.15):
        ctx.expect_aabb_overlap("upper_display", "hinge_spine", axes="xy", min_overlap=0.005)
        ctx.expect_origin_distance("upper_display", "lower_body", axes="xy", max_dist=0.05)

    with ctx.pose(clamshell_hinge=2.25):
        ctx.expect_aabb_overlap("upper_display", "hinge_spine", axes="xy", min_overlap=0.005)
        ctx.expect_origin_distance("upper_display", "lower_body", axes="xy", max_dist=0.08)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
