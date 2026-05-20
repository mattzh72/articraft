from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
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


def _door_part(
    model: ArticulatedObject,
    *,
    name: str,
    panel_name: str,
    width: float,
    height: float,
    thickness: float,
    exterior_material,
    liner_material,
    handle_material,
    accent_material=None,
    with_control_panel: bool = False,
):
    door = model.part(name)

    door.visual(
        Box((width, thickness, height)),
        origin=Origin(xyz=(width / 2.0, thickness / 2.0, height / 2.0)),
        material=exterior_material,
        name=panel_name,
    )

    liner_margin_x = 0.075
    liner_margin_z = 0.10
    liner_thickness = 0.012
    door.visual(
        Box((width - liner_margin_x, liner_thickness, height - liner_margin_z)),
        origin=Origin(
            xyz=(
                width / 2.0,
                liner_thickness / 2.0,
                height / 2.0,
            )
        ),
        material=liner_material,
        name=f"{name}_inner_liner",
    )

    gasket_width = width - 0.035
    gasket_height = height - 0.05
    gasket_band = 0.02
    gasket_depth = 0.006
    gasket_y = gasket_depth / 2.0
    door.visual(
        Box((gasket_width, gasket_depth, gasket_band)),
        origin=Origin(xyz=(width / 2.0, gasket_y, gasket_band / 2.0)),
        material="gasket_gray",
        name=f"{name}_gasket_bottom",
    )
    door.visual(
        Box((gasket_width, gasket_depth, gasket_band)),
        origin=Origin(
            xyz=(width / 2.0, gasket_y, height - gasket_band / 2.0)
        ),
        material="gasket_gray",
        name=f"{name}_gasket_top",
    )
    side_gasket_height = gasket_height - 2.0 * gasket_band
    door.visual(
        Box((gasket_band, gasket_depth, side_gasket_height)),
        origin=Origin(
            xyz=(
                gasket_band / 2.0,
                gasket_y,
                height / 2.0,
            )
        ),
        material="gasket_gray",
        name=f"{name}_gasket_hinge_side",
    )
    door.visual(
        Box((gasket_band, gasket_depth, side_gasket_height)),
        origin=Origin(
            xyz=(
                width - gasket_band / 2.0,
                gasket_y,
                height / 2.0,
            )
        ),
        material="gasket_gray",
        name=f"{name}_gasket_latch_side",
    )

    handle_width = 0.026
    handle_depth = 0.018
    standoff_width = 0.018
    standoff_depth = 0.018
    standoff_height = 0.07
    handle_height = max(0.34, min(0.70, height * 0.58))
    handle_center_x = width - 0.055
    handle_bottom_z = (height - handle_height) / 2.0
    handle_center_z = handle_bottom_z + handle_height / 2.0
    upper_standoff_center_z = handle_bottom_z + handle_height * 0.82
    lower_standoff_center_z = handle_bottom_z + handle_height * 0.18
    standoff_center_y = thickness + standoff_depth / 2.0
    handle_center_y = thickness + standoff_depth + handle_depth / 2.0

    door.visual(
        Box((standoff_width, standoff_depth, standoff_height)),
        origin=Origin(
            xyz=(
                handle_center_x,
                standoff_center_y,
                lower_standoff_center_z,
            )
        ),
        material=handle_material,
        name=f"{name}_handle_lower_mount",
    )
    door.visual(
        Box((standoff_width, standoff_depth, standoff_height)),
        origin=Origin(
            xyz=(
                handle_center_x,
                standoff_center_y,
                upper_standoff_center_z,
            )
        ),
        material=handle_material,
        name=f"{name}_handle_upper_mount",
    )
    door.visual(
        Box((handle_width, handle_depth, handle_height)),
        origin=Origin(
            xyz=(
                handle_center_x,
                handle_center_y,
                handle_center_z,
            )
        ),
        material=handle_material,
        name=f"{name}_handle_bar",
    )

    if with_control_panel and accent_material is not None:
        display_width = 0.14
        display_height = 0.075
        display_depth = 0.014
        door.visual(
            Box((display_width, display_depth, display_height)),
            origin=Origin(
                xyz=(
                    width * 0.50,
                    thickness + display_depth / 2.0,
                    height - 0.14,
                )
            ),
            material=accent_material,
            name=f"{name}_control_panel",
        )

    return door


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="laboratory_refrigerator_split_doors")

    exterior_white = model.material("exterior_white", rgba=(0.92, 0.94, 0.95, 1.0))
    liner_white = model.material("liner_white", rgba=(0.86, 0.88, 0.89, 1.0))
    gasket_gray = model.material("gasket_gray", rgba=(0.57, 0.60, 0.62, 1.0))
    stainless = model.material("stainless", rgba=(0.74, 0.77, 0.80, 1.0))
    feet_black = model.material("feet_black", rgba=(0.18, 0.18, 0.19, 1.0))
    display_black = model.material("display_black", rgba=(0.12, 0.13, 0.14, 1.0))

    width = 0.72
    depth = 0.82
    cabinet_height = 1.89
    foot_height = 0.03
    wall_thickness = 0.04
    top_thickness = 0.06
    bottom_thickness = 0.06
    back_thickness = 0.03
    divider_thickness = 0.04
    shelf_thickness = 0.018

    lower_door_height = 1.02
    upper_door_height = 0.73
    door_width = 0.655
    door_thickness = 0.055
    door_front_gap = 0.0
    inter_door_gap = 0.012

    body = model.part("cabinet_body")

    side_height = cabinet_height - top_thickness - bottom_thickness
    side_center_z = foot_height + bottom_thickness + side_height / 2.0
    body.visual(
        Box((wall_thickness, depth, side_height)),
        origin=Origin(
            xyz=(-width / 2.0 + wall_thickness / 2.0, 0.0, side_center_z)
        ),
        material=exterior_white,
        name="left_wall",
    )
    body.visual(
        Box((wall_thickness, depth, side_height)),
        origin=Origin(
            xyz=(width / 2.0 - wall_thickness / 2.0, 0.0, side_center_z)
        ),
        material=exterior_white,
        name="right_wall",
    )
    body.visual(
        Box((width, depth, bottom_thickness)),
        origin=Origin(xyz=(0.0, 0.0, foot_height + bottom_thickness / 2.0)),
        material=exterior_white,
        name="bottom_deck",
    )
    body.visual(
        Box((width, depth, top_thickness)),
        origin=Origin(
            xyz=(0.0, 0.0, foot_height + cabinet_height - top_thickness / 2.0)
        ),
        material=exterior_white,
        name="top_cap",
    )
    body.visual(
        Box((width - 2.0 * wall_thickness, back_thickness, side_height)),
        origin=Origin(
            xyz=(
                0.0,
                -depth / 2.0 + back_thickness / 2.0,
                side_center_z,
            )
        ),
        material=liner_white,
        name="back_liner",
    )

    shelf_depth = depth - back_thickness - 0.03
    shelf_center_y = 0.005
    divider_z = foot_height + bottom_thickness + lower_door_height + inter_door_gap / 2.0
    body.visual(
        Box((width - 2.0 * wall_thickness, shelf_depth, divider_thickness)),
        origin=Origin(xyz=(0.0, shelf_center_y, divider_z)),
        material=liner_white,
        name="center_divider",
    )

    lower_shelf_z = foot_height + bottom_thickness + 0.47
    upper_shelf_z = divider_z + divider_thickness / 2.0 + 0.28
    shelf_width = width - 2.0 * wall_thickness
    body.visual(
        Box((shelf_width, shelf_depth - 0.03, shelf_thickness)),
        origin=Origin(xyz=(0.0, shelf_center_y, lower_shelf_z)),
        material=liner_white,
        name="lower_storage_shelf",
    )
    body.visual(
        Box((shelf_width, shelf_depth - 0.05, shelf_thickness)),
        origin=Origin(xyz=(0.0, shelf_center_y, upper_shelf_z)),
        material=liner_white,
        name="upper_storage_shelf",
    )

    mullion_height = cabinet_height - top_thickness - bottom_thickness
    front_return_depth = 0.02
    body.visual(
        Box((wall_thickness, front_return_depth, mullion_height)),
        origin=Origin(
            xyz=(
                -width / 2.0 + wall_thickness / 2.0,
                depth / 2.0 - front_return_depth / 2.0,
                side_center_z,
            )
        ),
        material=exterior_white,
        name="left_front_return",
    )
    body.visual(
        Box((wall_thickness, front_return_depth, mullion_height)),
        origin=Origin(
            xyz=(
                width / 2.0 - wall_thickness / 2.0,
                depth / 2.0 - front_return_depth / 2.0,
                side_center_z,
            )
        ),
        material=exterior_white,
        name="right_front_return",
    )
    body.visual(
        Box((width - 2.0 * wall_thickness, front_return_depth, divider_thickness)),
        origin=Origin(
            xyz=(0.0, depth / 2.0 - front_return_depth / 2.0, divider_z)
        ),
        material=exterior_white,
        name="front_divider_trim",
    )

    foot_radius = 0.018
    foot_offset_x = width / 2.0 - 0.08
    foot_offset_y = depth / 2.0 - 0.08
    for index, (fx, fy) in enumerate(
        (
            (-foot_offset_x, -foot_offset_y),
            (-foot_offset_x, foot_offset_y),
            (foot_offset_x, -foot_offset_y),
            (foot_offset_x, foot_offset_y),
        ),
        start=1,
    ):
        body.visual(
            Cylinder(radius=foot_radius, length=foot_height),
            origin=Origin(xyz=(fx, fy, foot_height / 2.0)),
            material=feet_black,
            name=f"leveling_foot_{index}",
        )

    upper_door = _door_part(
        model,
        name="upper_door",
        panel_name="upper_door_panel",
        width=door_width,
        height=upper_door_height,
        thickness=door_thickness,
        exterior_material=exterior_white,
        liner_material=liner_white,
        handle_material=stainless,
        accent_material=display_black,
        with_control_panel=True,
    )
    lower_door = _door_part(
        model,
        name="lower_door",
        panel_name="lower_door_panel",
        width=door_width,
        height=lower_door_height,
        thickness=door_thickness,
        exterior_material=exterior_white,
        liner_material=liner_white,
        handle_material=stainless,
        with_control_panel=False,
    )

    hinge_x = -door_width / 2.0
    hinge_y = depth / 2.0 + door_front_gap
    lower_hinge_z = foot_height + 0.05
    upper_hinge_z = lower_hinge_z + lower_door_height + inter_door_gap

    model.articulation(
        "cabinet_to_upper_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=upper_door,
        origin=Origin(xyz=(hinge_x, hinge_y, upper_hinge_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=1.6,
            lower=0.0,
            upper=2.05,
        ),
    )
    model.articulation(
        "cabinet_to_lower_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lower_door,
        origin=Origin(xyz=(hinge_x, hinge_y, lower_hinge_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=1.6,
            lower=0.0,
            upper=2.05,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("cabinet_body")
    upper_door = object_model.get_part("upper_door")
    lower_door = object_model.get_part("lower_door")
    upper_hinge = object_model.get_articulation("cabinet_to_upper_door")
    lower_hinge = object_model.get_articulation("cabinet_to_lower_door")

    upper_panel = upper_door.get_visual("upper_door_panel")
    lower_panel = lower_door.get_visual("lower_door_panel")

    ctx.expect_gap(
        upper_door,
        body,
        axis="y",
        min_gap=0.0,
        max_gap=0.001,
        positive_elem=upper_panel,
        name="upper door sits just proud of the cabinet face",
    )
    ctx.expect_gap(
        lower_door,
        body,
        axis="y",
        min_gap=0.0,
        max_gap=0.001,
        positive_elem=lower_panel,
        name="lower door sits just proud of the cabinet face",
    )
    ctx.expect_overlap(
        upper_door,
        body,
        axes="xz",
        elem_a=upper_panel,
        min_overlap=0.20,
        name="upper door covers the upper opening footprint",
    )
    ctx.expect_overlap(
        lower_door,
        body,
        axes="xz",
        elem_a=lower_panel,
        min_overlap=0.20,
        name="lower door covers the lower opening footprint",
    )
    ctx.expect_gap(
        upper_door,
        lower_door,
        axis="z",
        min_gap=0.008,
        max_gap=0.02,
        positive_elem=upper_panel,
        negative_elem=lower_panel,
        name="upper and lower doors remain visibly split",
    )

    upper_axis_ok = tuple(round(v, 6) for v in upper_hinge.axis) == (0.0, 0.0, 1.0)
    lower_axis_ok = tuple(round(v, 6) for v in lower_hinge.axis) == (0.0, 0.0, 1.0)
    same_side_ok = (
        abs(upper_hinge.origin.xyz[0] - lower_hinge.origin.xyz[0]) < 1e-6
        and abs(upper_hinge.origin.xyz[1] - lower_hinge.origin.xyz[1]) < 1e-6
    )
    ctx.check(
        "both door hinges are vertical on the same cabinet side",
        upper_axis_ok and lower_axis_ok and same_side_ok,
        details=(
            f"upper_axis={upper_hinge.axis}, lower_axis={lower_hinge.axis}, "
            f"upper_origin={upper_hinge.origin.xyz}, lower_origin={lower_hinge.origin.xyz}"
        ),
    )

    closed_upper = ctx.part_element_world_aabb(upper_door, elem=upper_panel)
    closed_lower = ctx.part_element_world_aabb(lower_door, elem=lower_panel)

    with ctx.pose({upper_hinge: 1.35}):
        upper_open = ctx.part_element_world_aabb(upper_door, elem=upper_panel)
        lower_still_closed = ctx.part_element_world_aabb(lower_door, elem=lower_panel)

    upper_opens_independently = (
        closed_upper is not None
        and upper_open is not None
        and closed_lower is not None
        and lower_still_closed is not None
        and upper_open[1][1] > closed_upper[1][1] + 0.18
        and abs(lower_still_closed[1][1] - closed_lower[1][1]) < 1e-4
    )
    ctx.check(
        "upper door opens without moving the lower door",
        upper_opens_independently,
        details=(
            f"closed_upper={closed_upper}, upper_open={upper_open}, "
            f"closed_lower={closed_lower}, lower_still_closed={lower_still_closed}"
        ),
    )

    with ctx.pose({lower_hinge: 1.35}):
        lower_open = ctx.part_element_world_aabb(lower_door, elem=lower_panel)
        upper_still_closed = ctx.part_element_world_aabb(upper_door, elem=upper_panel)

    lower_opens_independently = (
        closed_lower is not None
        and lower_open is not None
        and closed_upper is not None
        and upper_still_closed is not None
        and lower_open[1][1] > closed_lower[1][1] + 0.18
        and abs(upper_still_closed[1][1] - closed_upper[1][1]) < 1e-4
    )
    ctx.check(
        "lower door opens without moving the upper door",
        lower_opens_independently,
        details=(
            f"closed_lower={closed_lower}, lower_open={lower_open}, "
            f"closed_upper={closed_upper}, upper_still_closed={upper_still_closed}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
