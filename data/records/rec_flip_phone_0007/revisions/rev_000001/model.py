from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math
from pathlib import Path

from sdk import (
    AssetContext,
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)

ASSETS = AssetContext.from_script(__file__)
HERE = Path(__file__).resolve().parent
PHONE_WIDTH = 0.052
HALF_LENGTH = 0.094
HALF_THICKNESS = 0.011
SEAM_GAP = 0.0025
EDGE_RADIUS = 0.0045
HINGE_RADIUS = 0.004
SHELL_CENTER_Y = -HALF_LENGTH / 2.0
LOWER_SHELL_Z = -(HALF_THICKNESS + SEAM_GAP) / 2.0
UPPER_SHELL_Z = (HALF_THICKNESS + SEAM_GAP) / 2.0
LOWER_INTERIOR_Z = LOWER_SHELL_Z + HALF_THICKNESS / 2.0
UPPER_INTERIOR_Z = UPPER_SHELL_Z - HALF_THICKNESS / 2.0
LOWER_OUTER_Z = LOWER_SHELL_Z - HALF_THICKNESS / 2.0
UPPER_OUTER_Z = UPPER_SHELL_Z + HALF_THICKNESS / 2.0
BARREL_Y = -0.0015


def _add_half_shell(part, *, z_center, material) -> None:
    shell_mesh = mesh_from_geometry(
        ExtrudeGeometry.centered(
            rounded_rect_profile(
                PHONE_WIDTH,
                HALF_LENGTH,
                EDGE_RADIUS,
                corner_segments=10,
            ),
            HALF_THICKNESS,
        ),
        HERE / "assets" / "meshes" / f"{part.name}_shell.obj",
    )
    part.visual(
        shell_mesh,
        origin=Origin(xyz=(0.0, SHELL_CENTER_Y, z_center)),
        material=material,
        name="shell",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="retro_flip_phone", assets=ASSETS)

    silver = model.material("matte_silver", rgba=(0.71, 0.73, 0.76, 1.0))
    charcoal = model.material("charcoal", rgba=(0.12, 0.12, 0.13, 1.0))
    dark_glass = model.material("dark_glass", rgba=(0.08, 0.10, 0.13, 1.0))
    screen_glow = model.material("screen_glow", rgba=(0.42, 0.70, 0.88, 1.0))
    soft_key = model.material("soft_key", rgba=(0.88, 0.89, 0.90, 1.0))

    lower = model.part("lower_half")
    _add_half_shell(lower, z_center=LOWER_SHELL_Z, material=silver)
    lower.visual(
        Cylinder(radius=HINGE_RADIUS, length=0.020),
        origin=Origin(
            xyz=(0.0, BARREL_Y, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=silver,
        name="spine_barrel_center",
    )
    lower.visual(
        Box((0.020, 0.006, 0.00125)),
        origin=Origin(xyz=(0.0, -0.0025, LOWER_INTERIOR_Z + 0.000625)),
        material=charcoal,
        name="hinge_stop",
    )
    lower.visual(
        Box((0.010, 0.006, 0.0007)),
        origin=Origin(xyz=(-0.0125, -0.014, LOWER_INTERIOR_Z + 0.00035)),
        material=soft_key,
        name="soft_key_left",
    )
    lower.visual(
        Box((0.010, 0.006, 0.0007)),
        origin=Origin(xyz=(0.0125, -0.014, LOWER_INTERIOR_Z + 0.00035)),
        material=soft_key,
        name="soft_key_right",
    )
    lower.visual(
        Box((0.030, 0.022, 0.0010)),
        origin=Origin(xyz=(0.0, -0.028, LOWER_INTERIOR_Z + 0.0005)),
        material=charcoal,
        name="nav_pad",
    )
    lower.visual(
        Cylinder(radius=0.0065, length=0.0008),
        origin=Origin(xyz=(0.0, -0.028, LOWER_INTERIOR_Z + 0.0014)),
        material=soft_key,
        name="center_select",
    )
    lower.visual(
        Box((0.040, 0.046, 0.0006)),
        origin=Origin(xyz=(0.0, -0.064, LOWER_INTERIOR_Z + 0.0003)),
        material=charcoal,
        name="keypad_field",
    )
    key_x = (-0.012, 0.0, 0.012)
    key_y = (-0.046, -0.058, -0.070, -0.082)
    for row, y in enumerate(key_y):
        for col, x in enumerate(key_x):
            lower.visual(
                Box((0.009, 0.007, 0.0006)),
                origin=Origin(xyz=(x, y, LOWER_INTERIOR_Z + 0.0009)),
                material=soft_key,
                name=f"key_{row}_{col}",
            )

    lower.inertial = Inertial.from_geometry(
        Box((PHONE_WIDTH, HALF_LENGTH, HALF_THICKNESS)),
        mass=0.075,
        origin=Origin(xyz=(0.0, SHELL_CENTER_Y, LOWER_SHELL_Z)),
    )

    upper = model.part("upper_half")
    _add_half_shell(upper, z_center=UPPER_SHELL_Z, material=silver)
    upper.visual(
        Cylinder(radius=HINGE_RADIUS, length=0.016),
        origin=Origin(
            xyz=(-0.018, BARREL_Y, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=silver,
        name="spine_barrel_left",
    )
    upper.visual(
        Cylinder(radius=HINGE_RADIUS, length=0.016),
        origin=Origin(
            xyz=(0.018, BARREL_Y, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=silver,
        name="spine_barrel_right",
    )
    upper.visual(
        Box((0.020, 0.006, 0.00125)),
        origin=Origin(xyz=(0.0, -0.0025, UPPER_INTERIOR_Z - 0.000625)),
        material=charcoal,
        name="hinge_stop",
    )
    upper.visual(
        Box((0.034, 0.050, 0.0008)),
        origin=Origin(xyz=(0.0, -0.053, UPPER_INTERIOR_Z - 0.0004)),
        material=screen_glow,
        name="inner_screen",
    )
    upper.visual(
        Box((0.028, 0.018, 0.0008)),
        origin=Origin(xyz=(0.0, -0.048, UPPER_OUTER_Z + 0.0004)),
        material=dark_glass,
        name="outer_display",
    )
    for slot_x in (-0.010, -0.005, 0.0, 0.005, 0.010):
        upper.visual(
            Box((0.0022, 0.010, 0.0006)),
            origin=Origin(xyz=(slot_x, -0.081, UPPER_INTERIOR_Z - 0.0003)),
            material=charcoal,
            name=f"ear_slot_{slot_x:+.3f}",
        )

    upper.inertial = Inertial.from_geometry(
        Box((PHONE_WIDTH, HALF_LENGTH, HALF_THICKNESS)),
        mass=0.065,
        origin=Origin(xyz=(0.0, SHELL_CENTER_Y, UPPER_SHELL_Z)),
    )

    model.articulation(
        "spine_hinge",
        ArticulationType.REVOLUTE,
        parent=lower,
        child=upper,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=3.0,
            lower=-2.45,
            upper=0.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    lower = object_model.get_part("lower_half")
    upper = object_model.get_part("upper_half")
    hinge = object_model.get_articulation("spine_hinge")

    lower_shell = lower.get_visual("shell")
    upper_shell = upper.get_visual("shell")
    lower_stop = lower.get_visual("hinge_stop")
    upper_stop = upper.get_visual("hinge_stop")
    lower_barrel = lower.get_visual("spine_barrel_center")
    upper_barrel_left = upper.get_visual("spine_barrel_left")
    upper_barrel_right = upper.get_visual("spine_barrel_right")
    nav_pad = lower.get_visual("nav_pad")
    keypad_field = lower.get_visual("keypad_field")
    center_select = lower.get_visual("center_select")
    inner_screen = upper.get_visual("inner_screen")
    outer_display = upper.get_visual("outer_display")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_overlaps(max_pose_samples=64)

    def _aabb_size(aabb):
        return tuple(aabb[1][i] - aabb[0][i] for i in range(3))

    with ctx.pose({hinge: 0.0}):
        lower_shell_aabb = ctx.part_element_world_aabb(lower, elem=lower_shell)
        upper_shell_aabb = ctx.part_element_world_aabb(upper, elem=upper_shell)
        outer_display_aabb = ctx.part_element_world_aabb(upper, elem=outer_display)
        nav_pad_aabb = ctx.part_element_world_aabb(lower, elem=nav_pad)
        keypad_aabb = ctx.part_element_world_aabb(lower, elem=keypad_field)
        center_select_aabb = ctx.part_element_world_aabb(lower, elem=center_select)

        ctx.expect_overlap(upper, lower, axes="xy", min_overlap=0.040)
        ctx.expect_gap(
            upper,
            lower,
            axis="z",
            min_gap=SEAM_GAP - 1e-4,
            max_gap=SEAM_GAP + 1e-4,
            positive_elem=upper_shell,
            negative_elem=lower_shell,
            name="closed_shell_seam_gap",
        )
        ctx.expect_contact(upper, lower, elem_a=upper_stop, elem_b=lower_stop, name="closed_hinge_stop_contact")
        ctx.expect_origin_distance(upper, lower, axes="xy", max_dist=0.001)
        ctx.expect_within(upper, upper, axes="xy", inner_elem=outer_display, outer_elem=upper_shell)
        ctx.expect_within(upper, upper, axes="xy", inner_elem=inner_screen, outer_elem=upper_shell)
        ctx.expect_within(lower, lower, axes="xy", inner_elem=nav_pad, outer_elem=lower_shell)
        ctx.expect_within(lower, lower, axes="xy", inner_elem=keypad_field, outer_elem=lower_shell)
        ctx.check(
            "matching_upper_lower_shell_sizes",
            lower_shell_aabb is not None
            and upper_shell_aabb is not None
            and all(
                abs(a - b) <= 1e-6
                for a, b in zip(_aabb_size(lower_shell_aabb), _aabb_size(upper_shell_aabb))
            ),
            details=f"lower={_aabb_size(lower_shell_aabb) if lower_shell_aabb else None}, upper={_aabb_size(upper_shell_aabb) if upper_shell_aabb else None}",
        )
        ctx.check(
            "outer_display_on_upper_rear_face",
            outer_display_aabb is not None
            and upper_shell_aabb is not None
            and abs(outer_display_aabb[0][2] - upper_shell_aabb[1][2]) <= 1e-6,
            details=f"display_min_z={outer_display_aabb[0][2] if outer_display_aabb else None}, shell_top_z={upper_shell_aabb[1][2] if upper_shell_aabb else None}",
        )
        ctx.check(
            "navigation_cluster_is_surface_mounted",
            nav_pad_aabb is not None
            and lower_shell_aabb is not None
            and abs(nav_pad_aabb[0][2] - lower_shell_aabb[1][2]) <= 1e-6,
            details=f"nav_pad_min_z={nav_pad_aabb[0][2] if nav_pad_aabb else None}, shell_top_z={lower_shell_aabb[1][2] if lower_shell_aabb else None}",
        )
        ctx.check(
            "keypad_is_below_navigation_cluster",
            nav_pad_aabb is not None
            and keypad_aabb is not None
            and nav_pad_aabb[0][1] > keypad_aabb[1][1],
            details=f"nav_pad_y={nav_pad_aabb if nav_pad_aabb else None}, keypad_y={keypad_aabb if keypad_aabb else None}",
        )
        ctx.check(
            "select_key_sits_on_navigation_cluster",
            nav_pad_aabb is not None
            and center_select_aabb is not None
            and abs(center_select_aabb[0][2] - nav_pad_aabb[1][2]) <= 1e-6,
            details=f"select_min_z={center_select_aabb[0][2] if center_select_aabb else None}, nav_top_z={nav_pad_aabb[1][2] if nav_pad_aabb else None}",
        )

    limits = hinge.motion_limits
    if limits is not None and limits.lower is not None and limits.upper is not None:
        with ctx.pose({hinge: limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="spine_hinge_open_limit_no_overlap")
            ctx.fail_if_isolated_parts(name="spine_hinge_open_limit_no_floating")
        with ctx.pose({hinge: limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="spine_hinge_closed_limit_no_overlap")
            ctx.fail_if_isolated_parts(name="spine_hinge_closed_limit_no_floating")

    with ctx.pose({hinge: -2.2}):
        ctx.expect_contact(upper, lower, elem_a=upper_barrel_left, elem_b=lower_barrel, name="left_barrel_contact_open")
        ctx.expect_contact(upper, lower, elem_a=upper_barrel_right, elem_b=lower_barrel, name="right_barrel_contact_open")
        ctx.expect_gap(
            upper,
            lower,
            axis="z",
            min_gap=0.020,
            positive_elem=inner_screen,
            negative_elem=nav_pad,
            name="open_inner_screen_clears_keypad",
        )
        ctx.expect_overlap(upper, lower, axes="x", min_overlap=0.030)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
