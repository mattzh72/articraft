from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

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
    wire_from_points,
)


def _x_axis_cylinder(radius: float, length: float) -> tuple[Cylinder, Origin]:
    return Cylinder(radius=radius, length=length), Origin(rpy=(0.0, pi / 2.0, 0.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="guitar_effects_carry_case")

    case_black = model.material("case_black", rgba=(0.13, 0.13, 0.14, 1.0))
    tray_black = model.material("tray_black", rgba=(0.18, 0.19, 0.20, 1.0))
    hardware = model.material("hardware", rgba=(0.72, 0.73, 0.76, 1.0))
    dark_hardware = model.material("dark_hardware", rgba=(0.34, 0.35, 0.37, 1.0))
    grip_rubber = model.material("grip_rubber", rgba=(0.08, 0.08, 0.09, 1.0))

    width = 0.72
    depth = 0.36
    shell_height = 0.085
    wall = 0.012
    floor = 0.010
    tray_slope = 0.21

    cover_width = 0.748
    cover_depth = 0.392
    cover_thickness = 0.010
    cover_height = 0.135
    cover_lip = 0.022
    hinge_axis_y = -0.190
    hinge_axis_z = shell_height

    lower_shell = model.part("lower_shell")
    lower_shell.inertial = Inertial.from_geometry(
        Box((width, depth, shell_height)),
        mass=5.8,
        origin=Origin(xyz=(0.0, 0.0, shell_height / 2.0)),
    )
    lower_shell.visual(
        Box((width, depth, floor)),
        origin=Origin(xyz=(0.0, 0.0, floor / 2.0)),
        material=case_black,
        name="base_floor",
    )
    lower_shell.visual(
        Box((wall, depth, shell_height)),
        origin=Origin(xyz=(-(width / 2.0 - wall / 2.0), 0.0, shell_height / 2.0)),
        material=case_black,
        name="base_left_wall",
    )
    lower_shell.visual(
        Box((wall, depth, shell_height)),
        origin=Origin(xyz=((width / 2.0 - wall / 2.0), 0.0, shell_height / 2.0)),
        material=case_black,
        name="base_right_wall",
    )
    lower_shell.visual(
        Box((width - 2.0 * wall, wall, shell_height)),
        origin=Origin(xyz=(0.0, depth / 2.0 - wall / 2.0, shell_height / 2.0)),
        material=case_black,
        name="base_front_wall",
    )
    lower_shell.visual(
        Box((width - 2.0 * wall, wall, shell_height)),
        origin=Origin(xyz=(0.0, -(depth / 2.0 - wall / 2.0), shell_height / 2.0)),
        material=case_black,
        name="base_rear_wall",
    )

    lower_shell.visual(
        Box((0.656, 0.290, 0.009)),
        origin=Origin(xyz=(0.0, -0.015, 0.041), rpy=(-tray_slope, 0.0, 0.0)),
        material=tray_black,
        name="tray_deck",
    )
    lower_shell.visual(
        Box((0.024, 0.270, 0.024)),
        origin=Origin(xyz=(-0.338, -0.015, 0.027), rpy=(-tray_slope, 0.0, 0.0)),
        material=dark_hardware,
        name="tray_left_support",
    )
    lower_shell.visual(
        Box((0.024, 0.270, 0.024)),
        origin=Origin(xyz=(0.338, -0.015, 0.027), rpy=(-tray_slope, 0.0, 0.0)),
        material=dark_hardware,
        name="tray_right_support",
    )
    lower_shell.visual(
        Box((0.620, 0.028, 0.030)),
        origin=Origin(xyz=(0.0, -0.158, 0.049), rpy=(-tray_slope, 0.0, 0.0)),
        material=dark_hardware,
        name="tray_rear_support",
    )
    lower_shell.visual(
        Box((0.050, 0.015, 0.016)),
        origin=Origin(xyz=(-0.240, 0.181, 0.073)),
        material=hardware,
        name="latch_catch_left",
    )
    lower_shell.visual(
        Box((0.050, 0.015, 0.016)),
        origin=Origin(xyz=(0.240, 0.181, 0.073)),
        material=hardware,
        name="latch_catch_right",
    )

    hinge_geom, hinge_rpy = _x_axis_cylinder(radius=0.009, length=0.205)
    lower_shell.visual(
        hinge_geom,
        origin=Origin(xyz=(-0.254, hinge_axis_y, hinge_axis_z), rpy=hinge_rpy.rpy),
        material=hardware,
        name="base_hinge_left",
    )
    lower_shell.visual(
        hinge_geom,
        origin=Origin(xyz=(0.254, hinge_axis_y, hinge_axis_z), rpy=hinge_rpy.rpy),
        material=hardware,
        name="base_hinge_right",
    )
    lower_shell.visual(
        Box((0.205, 0.018, 0.018)),
        origin=Origin(xyz=(-0.254, -0.185, 0.076)),
        material=hardware,
        name="base_hinge_leaf_left",
    )
    lower_shell.visual(
        Box((0.205, 0.018, 0.018)),
        origin=Origin(xyz=(0.254, -0.185, 0.076)),
        material=hardware,
        name="base_hinge_leaf_right",
    )

    dust_cover = model.part("dust_cover")
    dust_cover.inertial = Inertial.from_geometry(
        Box((cover_width, cover_depth, cover_height + cover_lip)),
        mass=3.9,
        origin=Origin(xyz=(0.0, cover_depth / 2.0, (cover_height - cover_lip) / 2.0)),
    )
    cover_panel_height = cover_height + cover_lip
    cover_panel_center_z = (cover_height - cover_lip) / 2.0

    dust_cover.visual(
        Box((cover_width, cover_depth, cover_thickness)),
        origin=Origin(xyz=(0.0, cover_depth / 2.0, cover_height - cover_thickness / 2.0)),
        material=case_black,
        name="cover_top_panel",
    )
    dust_cover.visual(
        Box((cover_thickness, cover_depth, cover_panel_height)),
        origin=Origin(
            xyz=(-(cover_width / 2.0 - cover_thickness / 2.0), cover_depth / 2.0, cover_panel_center_z)
        ),
        material=case_black,
        name="cover_left_side",
    )
    dust_cover.visual(
        Box((cover_thickness, cover_depth, cover_panel_height)),
        origin=Origin(
            xyz=((cover_width / 2.0 - cover_thickness / 2.0), cover_depth / 2.0, cover_panel_center_z)
        ),
        material=case_black,
        name="cover_right_side",
    )
    dust_cover.visual(
        Box((cover_width - 2.0 * cover_thickness, cover_thickness, cover_panel_height)),
        origin=Origin(xyz=(0.0, cover_depth - cover_thickness / 2.0, cover_panel_center_z)),
        material=case_black,
        name="cover_front_panel",
    )
    dust_cover.visual(
        Box((cover_width, cover_thickness, 0.026)),
        origin=Origin(xyz=(0.0, cover_thickness / 2.0, 0.122)),
        material=case_black,
        name="cover_rear_strip",
    )
    dust_cover.visual(
        Box((0.032, 0.022, 0.024)),
        origin=Origin(xyz=(-0.205, 0.382, 0.050)),
        material=hardware,
        name="handle_bracket_left",
    )
    dust_cover.visual(
        Box((0.032, 0.022, 0.024)),
        origin=Origin(xyz=(0.205, 0.382, 0.050)),
        material=hardware,
        name="handle_bracket_right",
    )
    dust_cover.visual(
        Box((cover_width - 0.040, 0.028, 0.008)),
        origin=Origin(xyz=(0.0, 0.372, 0.004)),
        material=dark_hardware,
        name="cover_front_inner_lip",
    )
    dust_cover.visual(
        Box((0.038, 0.016, 0.056)),
        origin=Origin(xyz=(-0.240, 0.395, 0.006)),
        material=hardware,
        name="cover_latch_left",
    )
    dust_cover.visual(
        Box((0.038, 0.016, 0.056)),
        origin=Origin(xyz=(0.240, 0.395, 0.006)),
        material=hardware,
        name="cover_latch_right",
    )

    cover_hinge_geom, cover_hinge_rpy = _x_axis_cylinder(radius=0.008, length=0.285)
    dust_cover.visual(
        cover_hinge_geom,
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=cover_hinge_rpy.rpy),
        material=hardware,
        name="cover_hinge_barrel",
    )
    dust_cover.visual(
        Box((0.300, 0.010, 0.106)),
        origin=Origin(xyz=(0.0, 0.005, 0.057)),
        material=hardware,
        name="cover_rear_flange",
    )

    carry_handle = model.part("carry_handle")
    carry_handle.inertial = Inertial.from_geometry(
        Box((0.46, 0.09, 0.13)),
        mass=0.55,
        origin=Origin(xyz=(0.0, 0.020, -0.040)),
    )
    handle_loop = wire_from_points(
        [
            (-0.205, 0.0, 0.0),
            (-0.205, 0.014, -0.016),
            (-0.165, 0.034, -0.084),
            (0.165, 0.034, -0.084),
            (0.205, 0.014, -0.016),
            (0.205, 0.0, 0.0),
        ],
        radius=0.007,
        radial_segments=18,
        cap_ends=True,
        corner_mode="fillet",
        corner_radius=0.024,
        corner_segments=10,
    )
    carry_handle.visual(
        mesh_from_geometry(handle_loop, "carry_handle_loop"),
        material=dark_hardware,
        name="handle_loop",
    )
    pivot_geom, pivot_rpy = _x_axis_cylinder(radius=0.009, length=0.026)
    carry_handle.visual(
        pivot_geom,
        origin=Origin(xyz=(-0.205, 0.0, 0.0), rpy=pivot_rpy.rpy),
        material=hardware,
        name="handle_left_pivot",
    )
    carry_handle.visual(
        pivot_geom,
        origin=Origin(xyz=(0.205, 0.0, 0.0), rpy=pivot_rpy.rpy),
        material=hardware,
        name="handle_right_pivot",
    )
    grip_geom, grip_rpy = _x_axis_cylinder(radius=0.011, length=0.240)
    carry_handle.visual(
        grip_geom,
        origin=Origin(xyz=(0.0, 0.034, -0.084), rpy=grip_rpy.rpy),
        material=grip_rubber,
        name="handle_grip",
    )

    cover_joint = model.articulation(
        "lower_shell_to_cover",
        ArticulationType.REVOLUTE,
        parent=lower_shell,
        child=dust_cover,
        origin=Origin(xyz=(0.0, hinge_axis_y, hinge_axis_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.8, lower=0.0, upper=1.35),
    )
    model.articulation(
        "cover_to_handle",
        ArticulationType.REVOLUTE,
        parent=dust_cover,
        child=carry_handle,
        origin=Origin(xyz=(0.0, 0.402, 0.050)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.5, lower=0.0, upper=1.45),
    )

    lower_shell.meta["primary_joint"] = cover_joint.name
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    lower_shell = object_model.get_part("lower_shell")
    dust_cover = object_model.get_part("dust_cover")
    carry_handle = object_model.get_part("carry_handle")
    cover_joint = object_model.get_articulation("lower_shell_to_cover")
    handle_joint = object_model.get_articulation("cover_to_handle")

    with ctx.pose({cover_joint: 0.0, handle_joint: 0.0}):
        ctx.expect_overlap(
            dust_cover,
            lower_shell,
            axes="x",
            elem_a="cover_front_panel",
            elem_b="base_front_wall",
            min_overlap=0.62,
            name="cover front aligns across the case width",
        )
        ctx.expect_gap(
            dust_cover,
            lower_shell,
            axis="y",
            positive_elem="cover_front_panel",
            negative_elem="base_front_wall",
            min_gap=0.008,
            max_gap=0.020,
            name="closed cover front sits just ahead of the lower shell front wall",
        )
        ctx.expect_gap(
            dust_cover,
            lower_shell,
            axis="z",
            positive_elem="cover_top_panel",
            negative_elem="tray_deck",
            min_gap=0.050,
            name="dust cover clears the angled lower tray",
        )
        ctx.expect_gap(
            carry_handle,
            dust_cover,
            axis="y",
            positive_elem="handle_grip",
            negative_elem="cover_front_panel",
            min_gap=0.003,
            max_gap=0.060,
            name="stowed handle sits just proud of the cover front face",
        )

        closed_cover_front = ctx.part_element_world_aabb(dust_cover, elem="cover_front_panel")
        closed_handle = ctx.part_element_world_aabb(carry_handle, elem="handle_grip")

    with ctx.pose({cover_joint: cover_joint.motion_limits.upper, handle_joint: 0.0}):
        open_cover_front = ctx.part_element_world_aabb(dust_cover, elem="cover_front_panel")
        ctx.check(
            "cover rotates upward on the rear hinge axis",
            closed_cover_front is not None
            and open_cover_front is not None
            and open_cover_front[0][2] > closed_cover_front[1][2] + 0.10,
            details=f"closed={closed_cover_front}, open={open_cover_front}",
        )

    with ctx.pose({cover_joint: 0.0, handle_joint: handle_joint.motion_limits.upper}):
        raised_handle = ctx.part_element_world_aabb(carry_handle, elem="handle_grip")
        ctx.check(
            "handle pivots upward from the cover front face",
            closed_handle is not None
            and raised_handle is not None
            and raised_handle[1][2] > closed_handle[1][2] + 0.07
            and raised_handle[1][1] > closed_handle[1][1] + 0.030,
            details=f"closed={closed_handle}, raised={raised_handle}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
