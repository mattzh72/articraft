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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def _add_drawer(
    model: ArticulatedObject,
    *,
    name: str,
    joint_name: str,
    pedestal_x: float,
    drawer_bottom_z: float,
    rail_z: float,
    front_color,
    carcass_color,
    metal_color,
) -> None:
    drawer = model.part(name)

    shell_width = 0.280
    shell_depth = 0.410
    shell_height = 0.180
    side_thickness = 0.012
    bottom_thickness = 0.012
    back_thickness = 0.012
    front_width = 0.300
    front_height = 0.220
    front_thickness = 0.018
    runner_width = 0.006
    runner_length = 0.340
    runner_height = 0.028

    drawer.visual(
        Box((shell_width - 2.0 * side_thickness, shell_depth, bottom_thickness)),
        origin=Origin(xyz=(0.0, 0.0, bottom_thickness / 2.0)),
        material=carcass_color,
        name="drawer_bottom",
    )
    drawer.visual(
        Box((side_thickness, shell_depth, shell_height)),
        origin=Origin(
            xyz=(
                -(shell_width / 2.0 - side_thickness / 2.0),
                0.0,
                bottom_thickness + shell_height / 2.0,
            )
        ),
        material=carcass_color,
        name="left_side",
    )
    drawer.visual(
        Box((side_thickness, shell_depth, shell_height)),
        origin=Origin(
            xyz=(
                shell_width / 2.0 - side_thickness / 2.0,
                0.0,
                bottom_thickness + shell_height / 2.0,
            )
        ),
        material=carcass_color,
        name="right_side",
    )
    drawer.visual(
        Box((shell_width - 2.0 * side_thickness, back_thickness, shell_height)),
        origin=Origin(
            xyz=(
                0.0,
                -(shell_depth / 2.0 - back_thickness / 2.0),
                bottom_thickness + shell_height / 2.0,
            )
        ),
        material=carcass_color,
        name="drawer_back",
    )
    drawer.visual(
        Box((front_width, front_thickness, front_height)),
        origin=Origin(
            xyz=(
                0.0,
                shell_depth / 2.0 + front_thickness / 2.0,
                front_height / 2.0,
            )
        ),
        material=front_color,
        name="drawer_front",
    )
    drawer.visual(
        Box((0.012, 0.032, 0.012)),
        origin=Origin(xyz=(-0.055, 0.239, 0.110)),
        material=metal_color,
        name="left_handle_post",
    )
    drawer.visual(
        Box((0.012, 0.032, 0.012)),
        origin=Origin(xyz=(0.055, 0.239, 0.110)),
        material=metal_color,
        name="right_handle_post",
    )
    drawer.visual(
        Cylinder(radius=0.008, length=0.140),
        origin=Origin(xyz=(0.0, 0.263, 0.110), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal_color,
        name="bar_handle",
    )
    drawer.visual(
        Box((runner_width, runner_length, runner_height)),
        origin=Origin(xyz=(-0.143, -0.010, 0.110)),
        material=metal_color,
        name="left_runner",
    )
    drawer.visual(
        Box((runner_width, runner_length, runner_height)),
        origin=Origin(xyz=(0.143, -0.010, 0.110)),
        material=metal_color,
        name="right_runner",
    )

    model.articulation(
        joint_name,
        ArticulationType.PRISMATIC,
        parent="desk_base",
        child=drawer,
        origin=Origin(xyz=(pedestal_x, 0.039, drawer_bottom_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.30,
            lower=0.0,
            upper=0.240,
        ),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="student_pedestal_desk")

    wood = model.material("oak_laminate", rgba=(0.77, 0.65, 0.49, 1.0))
    shell = model.material("warm_white", rgba=(0.94, 0.94, 0.93, 1.0))
    drawer_face = model.material("soft_gray", rgba=(0.84, 0.85, 0.87, 1.0))
    metal = model.material("charcoal_metal", rgba=(0.24, 0.25, 0.28, 1.0))

    desk_base = model.part("desk_base")

    top_width = 1.00
    top_depth = 0.56
    top_thickness = 0.03
    desk_height = 0.75
    pedestal_x = -0.31
    pedestal_width = 0.34
    pedestal_depth = 0.52
    pedestal_height = desk_height - top_thickness
    side_thickness = 0.018
    back_thickness = 0.012

    desk_base.visual(
        Box((top_width, top_depth, top_thickness)),
        origin=Origin(xyz=(0.0, 0.0, desk_height - top_thickness / 2.0)),
        material=wood,
        name="worktop",
    )

    desk_base.visual(
        Box((side_thickness, pedestal_depth, pedestal_height)),
        origin=Origin(
            xyz=(
                pedestal_x - pedestal_width / 2.0 + side_thickness / 2.0,
                0.0,
                pedestal_height / 2.0,
            )
        ),
        material=shell,
        name="pedestal_left_side",
    )
    desk_base.visual(
        Box((side_thickness, pedestal_depth, pedestal_height)),
        origin=Origin(
            xyz=(
                pedestal_x + pedestal_width / 2.0 - side_thickness / 2.0,
                0.0,
                pedestal_height / 2.0,
            )
        ),
        material=shell,
        name="pedestal_right_side",
    )
    desk_base.visual(
        Box((pedestal_width - 2.0 * side_thickness, back_thickness, pedestal_height)),
        origin=Origin(
            xyz=(
                pedestal_x,
                -(pedestal_depth / 2.0 - back_thickness / 2.0),
                pedestal_height / 2.0,
            )
        ),
        material=shell,
        name="pedestal_back",
    )
    desk_base.visual(
        Box((pedestal_width - 2.0 * side_thickness, 0.48, 0.018)),
        origin=Origin(xyz=(pedestal_x, -0.014, 0.045)),
        material=shell,
        name="pedestal_bottom",
    )
    desk_base.visual(
        Box((pedestal_width - 2.0 * side_thickness, 0.48, 0.018)),
        origin=Origin(xyz=(pedestal_x, -0.014, 0.355)),
        material=shell,
        name="pedestal_divider",
    )
    desk_base.visual(
        Box((pedestal_width - 2.0 * side_thickness, 0.060, 0.080)),
        origin=Origin(xyz=(pedestal_x, 0.190, 0.040)),
        material=shell,
        name="pedestal_plinth",
    )

    for prefix, rail_z in (("lower", 0.220), ("upper", 0.495)):
        desk_base.visual(
            Box((0.006, 0.400, 0.028)),
            origin=Origin(xyz=(pedestal_x - 0.149, 0.0, rail_z)),
            material=metal,
            name=f"{prefix}_left_outer_rail",
        )
        desk_base.visual(
            Box((0.006, 0.400, 0.028)),
            origin=Origin(xyz=(pedestal_x + 0.149, 0.0, rail_z)),
            material=metal,
            name=f"{prefix}_right_outer_rail",
        )

    desk_base.visual(
        Box((0.022, 0.050, 0.030)),
        origin=Origin(xyz=(0.500, -0.070, desk_height - 0.004)),
        material=metal,
        name="rear_hinge_leaf",
    )
    desk_base.visual(
        Box((0.022, 0.050, 0.030)),
        origin=Origin(xyz=(0.500, 0.070, desk_height - 0.004)),
        material=metal,
        name="front_hinge_leaf",
    )
    desk_base.visual(
        Cylinder(radius=0.015, length=pedestal_height),
        origin=Origin(xyz=(0.420, 0.220, pedestal_height / 2.0), rpy=(0.0, 0.0, 0.0)),
        material=metal,
        name="front_right_leg",
    )
    desk_base.visual(
        Cylinder(radius=0.015, length=pedestal_height),
        origin=Origin(xyz=(0.420, -0.220, pedestal_height / 2.0), rpy=(0.0, 0.0, 0.0)),
        material=metal,
        name="rear_right_leg",
    )
    desk_base.visual(
        Box((0.600, 0.050, 0.080)),
        origin=Origin(xyz=(0.130, 0.245, 0.680)),
        material=shell,
        name="front_apron",
    )
    desk_base.visual(
        Box((0.600, 0.040, 0.070)),
        origin=Origin(xyz=(0.130, -0.245, 0.685)),
        material=shell,
        name="rear_apron",
    )
    desk_base.visual(
        Box((0.580, 0.012, 0.240)),
        origin=Origin(xyz=(0.140, -0.200, 0.470)),
        material=shell,
        name="modesty_panel",
    )

    desk_base.visual(
        Cylinder(radius=0.011, length=0.080),
        origin=Origin(
            xyz=(0.511, -0.070, desk_height + 0.011),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=metal,
        name="rear_hinge_knuckle",
    )
    desk_base.visual(
        Cylinder(radius=0.011, length=0.080),
        origin=Origin(
            xyz=(0.511, 0.070, desk_height + 0.011),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=metal,
        name="front_hinge_knuckle",
    )

    _add_drawer(
        model,
        name="lower_drawer",
        joint_name="base_to_lower_drawer",
        pedestal_x=pedestal_x,
        drawer_bottom_z=0.110,
        rail_z=0.220,
        front_color=drawer_face,
        carcass_color=shell,
        metal_color=metal,
    )
    _add_drawer(
        model,
        name="upper_drawer",
        joint_name="base_to_upper_drawer",
        pedestal_x=pedestal_x,
        drawer_bottom_z=0.385,
        rail_z=0.495,
        front_color=drawer_face,
        carcass_color=shell,
        metal_color=metal,
    )

    laptop_shelf = model.part("laptop_shelf")
    laptop_shelf.visual(
        Box((0.270, 0.220, 0.018)),
        origin=Origin(xyz=(0.135, 0.0, -0.020)),
        material=wood,
        name="shelf_panel",
    )
    laptop_shelf.visual(
        Box((0.012, 0.220, 0.016)),
        origin=Origin(xyz=(0.264, 0.0, -0.003)),
        material=metal,
        name="keeper_lip",
    )
    laptop_shelf.visual(
        Cylinder(radius=0.011, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="center_knuckle",
    )

    model.articulation(
        "base_to_laptop_shelf",
        ArticulationType.REVOLUTE,
        parent=desk_base,
        child=laptop_shelf,
        origin=Origin(xyz=(0.511, 0.0, desk_height + 0.011)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=2.0,
            lower=0.0,
            upper=1.25,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    desk_base = object_model.get_part("desk_base")
    lower_drawer = object_model.get_part("lower_drawer")
    upper_drawer = object_model.get_part("upper_drawer")
    laptop_shelf = object_model.get_part("laptop_shelf")
    lower_slide = object_model.get_articulation("base_to_lower_drawer")
    upper_slide = object_model.get_articulation("base_to_upper_drawer")
    shelf_hinge = object_model.get_articulation("base_to_laptop_shelf")

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
        upper_drawer,
        desk_base,
        elem_a="left_runner",
        elem_b="upper_left_outer_rail",
        name="upper drawer is carried by its slide rail",
    )
    ctx.expect_contact(
        lower_drawer,
        desk_base,
        elem_a="right_runner",
        elem_b="lower_right_outer_rail",
        name="lower drawer is carried by its slide rail",
    )
    ctx.expect_contact(
        laptop_shelf,
        desk_base,
        elem_a="center_knuckle",
        elem_b="front_hinge_knuckle",
        name="laptop shelf hinge knuckles meet cleanly",
    )

    lower_rest = ctx.part_world_position(lower_drawer)
    upper_rest = ctx.part_world_position(upper_drawer)
    shelf_rest = ctx.part_element_world_aabb(laptop_shelf, elem="keeper_lip")

    with ctx.pose({lower_slide: 0.240}):
        ctx.expect_overlap(
            lower_drawer,
            desk_base,
            axes="y",
            elem_a="left_runner",
            elem_b="lower_left_outer_rail",
            min_overlap=0.090,
            name="lower drawer retains rail insertion at full extension",
        )
        lower_extended = ctx.part_world_position(lower_drawer)

    with ctx.pose({upper_slide: 0.240}):
        ctx.expect_overlap(
            upper_drawer,
            desk_base,
            axes="y",
            elem_a="right_runner",
            elem_b="upper_right_outer_rail",
            min_overlap=0.090,
            name="upper drawer retains rail insertion at full extension",
        )
        upper_extended = ctx.part_world_position(upper_drawer)

    with ctx.pose({shelf_hinge: 1.10}):
        shelf_raised = ctx.part_element_world_aabb(laptop_shelf, elem="keeper_lip")

    ctx.check(
        "lower drawer slides outward",
        lower_rest is not None
        and lower_extended is not None
        and lower_extended[1] > lower_rest[1] + 0.18,
        details=f"rest={lower_rest}, extended={lower_extended}",
    )
    ctx.check(
        "upper drawer slides outward",
        upper_rest is not None
        and upper_extended is not None
        and upper_extended[1] > upper_rest[1] + 0.18,
        details=f"rest={upper_rest}, extended={upper_extended}",
    )
    ctx.check(
        "laptop shelf folds upward from the right edge",
        shelf_rest is not None
        and shelf_raised is not None
        and shelf_raised[1][2] > shelf_rest[1][2] + 0.18,
        details=f"rest={shelf_rest}, raised={shelf_raised}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
