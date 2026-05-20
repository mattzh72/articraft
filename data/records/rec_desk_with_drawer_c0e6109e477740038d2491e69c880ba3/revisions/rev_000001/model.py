from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="parsons_desk_with_pedestal")

    walnut = model.material("walnut", rgba=(0.37, 0.26, 0.18, 1.0))
    walnut_dark = model.material("walnut_dark", rgba=(0.28, 0.19, 0.13, 1.0))
    graphite = model.material("graphite", rgba=(0.18, 0.19, 0.21, 1.0))
    brass = model.material("brass", rgba=(0.67, 0.56, 0.33, 1.0))

    desk_width = 1.48
    desk_depth = 0.68
    top_thickness = 0.065
    support_height = 0.695
    front_inset = 0.030

    left_panel_thickness = 0.060
    support_depth = 0.620

    pedestal_width = 0.430
    pedestal_total_depth = 0.620
    pedestal_height = support_height
    front_setback = 0.024
    side_thickness = 0.018
    top_panel_thickness = 0.022
    bottom_panel_thickness = 0.022
    back_thickness = 0.012
    rail_thickness = 0.012
    rail_height = 0.030
    plinth_height = 0.053
    carcass_depth = pedestal_total_depth - front_setback - back_thickness
    internal_width = pedestal_width - 2.0 * side_thickness

    drawer_front_thickness = 0.020
    drawer_shell_depth = 0.560
    drawer_side_thickness = 0.012
    drawer_bottom_thickness = 0.012
    drawer_side_clearance = 0.0
    drawer_shell_width = internal_width - 2.0 * (rail_thickness + drawer_side_clearance)
    drawer_inner_width = drawer_shell_width - 2.0 * drawer_side_thickness
    drawer_pull_depth = 0.014
    drawer_pull_height = 0.012
    drawer_pull_post_length = 0.018
    drawer_pull_post_width = 0.012
    drawer_pull_width = 0.140
    drawer_travel = 0.285

    top = model.part("top")
    top.visual(
        Box((desk_depth, desk_width, top_thickness)),
        origin=Origin(xyz=(0.0, 0.0, top_thickness / 2.0)),
        material=walnut,
        name="top_slab",
    )

    knee_left_inner = -desk_width / 2.0 + left_panel_thickness
    knee_right_inner = desk_width / 2.0 - pedestal_width
    modesty_width = knee_right_inner - knee_left_inner - 0.040
    modesty_center_y = (knee_left_inner + knee_right_inner) / 2.0
    modesty_thickness = 0.018
    modesty_height = 0.390
    top.visual(
        Box((modesty_thickness, modesty_width, modesty_height)),
        origin=Origin(
            xyz=(
                desk_depth / 2.0 - 0.035 - modesty_thickness / 2.0,
                modesty_center_y,
                -modesty_height / 2.0,
            )
        ),
        material=walnut_dark,
        name="modesty_panel",
    )
    top.inertial = Inertial.from_geometry(
        Box((desk_depth, desk_width, top_thickness + modesty_height)),
        mass=24.0,
        origin=Origin(xyz=(0.0, 0.0, (top_thickness - modesty_height) / 2.0)),
    )

    left_panel = model.part("left_panel")
    left_panel.visual(
        Box((support_depth, left_panel_thickness, support_height)),
        origin=Origin(xyz=(support_depth / 2.0, 0.0, -support_height / 2.0)),
        material=walnut_dark,
        name="left_panel_slab",
    )
    left_panel.inertial = Inertial.from_geometry(
        Box((support_depth, left_panel_thickness, support_height)),
        mass=10.5,
        origin=Origin(xyz=(support_depth / 2.0, 0.0, -support_height / 2.0)),
    )

    pedestal = model.part("pedestal_body")
    side_center_y = pedestal_width / 2.0 - side_thickness / 2.0
    back_height = pedestal_height - top_panel_thickness - bottom_panel_thickness
    back_center_z = -(top_panel_thickness + back_height / 2.0)

    pedestal.visual(
        Box((carcass_depth, side_thickness, pedestal_height)),
        origin=Origin(xyz=(carcass_depth / 2.0, -side_center_y, -pedestal_height / 2.0)),
        material=walnut_dark,
        name="left_side",
    )
    pedestal.visual(
        Box((carcass_depth, side_thickness, pedestal_height)),
        origin=Origin(xyz=(carcass_depth / 2.0, side_center_y, -pedestal_height / 2.0)),
        material=walnut_dark,
        name="right_side",
    )
    pedestal.visual(
        Box((carcass_depth, internal_width, top_panel_thickness)),
        origin=Origin(xyz=(carcass_depth / 2.0, 0.0, -top_panel_thickness / 2.0)),
        material=walnut_dark,
        name="top_panel",
    )
    pedestal.visual(
        Box((carcass_depth, internal_width, bottom_panel_thickness)),
        origin=Origin(
            xyz=(carcass_depth / 2.0, 0.0, -pedestal_height + bottom_panel_thickness / 2.0)
        ),
        material=walnut_dark,
        name="bottom_panel",
    )
    pedestal.visual(
        Box((back_thickness, internal_width, back_height)),
        origin=Origin(xyz=(carcass_depth + back_thickness / 2.0, 0.0, back_center_z)),
        material=walnut_dark,
        name="back_panel",
    )
    pedestal.visual(
        Box((front_setback, internal_width, plinth_height)),
        origin=Origin(
            xyz=(
                -front_setback / 2.0,
                0.0,
                -pedestal_height + bottom_panel_thickness + plinth_height / 2.0,
            )
        ),
        material=walnut_dark,
        name="front_plinth",
    )

    drawer_front_heights = (0.145, 0.185, 0.245)
    drawer_names = ("top_drawer", "middle_drawer", "bottom_drawer")
    top_margin = 0.026
    reveal = 0.006
    rail_length = 0.470
    rail_start = 0.042
    rail_center_x = rail_start + rail_length / 2.0
    rail_center_y = internal_width / 2.0 - rail_thickness / 2.0

    drawer_center_zs: list[float] = []
    current_top = top_margin
    for drawer_name, front_height in zip(drawer_names, drawer_front_heights):
        center_z = -(current_top + front_height / 2.0)
        drawer_center_zs.append(center_z)
        rail_name_prefix = drawer_name.replace("_drawer", "")
        pedestal.visual(
            Box((rail_length, rail_thickness, rail_height)),
            origin=Origin(xyz=(rail_center_x, -rail_center_y, center_z)),
            material=graphite,
            name=f"{rail_name_prefix}_left_rail",
        )
        pedestal.visual(
            Box((rail_length, rail_thickness, rail_height)),
            origin=Origin(xyz=(rail_center_x, rail_center_y, center_z)),
            material=graphite,
            name=f"{rail_name_prefix}_right_rail",
        )
        current_top += front_height + reveal

    pedestal.inertial = Inertial.from_geometry(
        Box((pedestal_total_depth, pedestal_width, pedestal_height)),
        mass=22.0,
        origin=Origin(
            xyz=((pedestal_total_depth - front_setback) / 2.0, 0.0, -pedestal_height / 2.0)
        ),
    )

    drawer_front_width = pedestal_width - 0.012

    current_top = top_margin
    for index, (drawer_name, front_height, center_z) in enumerate(
        zip(drawer_names, drawer_front_heights, drawer_center_zs)
    ):
        drawer = model.part(drawer_name)
        front_material = walnut if index == 0 else walnut_dark
        shell_height = front_height - 0.028
        side_height = shell_height - drawer_bottom_thickness
        shell_top = front_height / 2.0 - 0.014
        shell_bottom = -front_height / 2.0 + 0.014
        side_center_z = shell_bottom + drawer_bottom_thickness + side_height / 2.0
        bottom_center_z = shell_bottom + drawer_bottom_thickness / 2.0
        back_panel_width = drawer_shell_width - 2.0 * drawer_side_thickness
        bottom_depth = drawer_shell_depth - drawer_side_thickness
        pull_offset = 0.045

        drawer.visual(
            Box((drawer_front_thickness, drawer_front_width, front_height)),
            origin=Origin(xyz=(drawer_front_thickness / 2.0, 0.0, 0.0)),
            material=front_material,
            name="drawer_front",
        )
        drawer.visual(
            Box((drawer_shell_depth, drawer_side_thickness, side_height)),
            origin=Origin(
                xyz=(
                    drawer_front_thickness + drawer_shell_depth / 2.0,
                    -(drawer_shell_width / 2.0 - drawer_side_thickness / 2.0),
                    side_center_z,
                )
            ),
            material=walnut,
            name="left_side",
        )
        drawer.visual(
            Box((drawer_shell_depth, drawer_side_thickness, side_height)),
            origin=Origin(
                xyz=(
                    drawer_front_thickness + drawer_shell_depth / 2.0,
                    drawer_shell_width / 2.0 - drawer_side_thickness / 2.0,
                    side_center_z,
                )
            ),
            material=walnut,
            name="right_side",
        )
        drawer.visual(
            Box((bottom_depth, drawer_inner_width, drawer_bottom_thickness)),
            origin=Origin(
                xyz=(
                    drawer_front_thickness + bottom_depth / 2.0,
                    0.0,
                    bottom_center_z,
                )
            ),
            material=walnut,
            name="bottom_panel",
        )
        drawer.visual(
            Box((drawer_side_thickness, back_panel_width, side_height)),
            origin=Origin(
                xyz=(
                    drawer_front_thickness + drawer_shell_depth - drawer_side_thickness / 2.0,
                    0.0,
                    side_center_z,
                )
            ),
            material=walnut,
            name="back_panel",
        )
        drawer.visual(
            Box((drawer_pull_post_length, drawer_pull_post_width, drawer_pull_post_width)),
            origin=Origin(xyz=(-drawer_pull_post_length / 2.0, -pull_offset, 0.0)),
            material=brass,
            name="left_pull_post",
        )
        drawer.visual(
            Box((drawer_pull_post_length, drawer_pull_post_width, drawer_pull_post_width)),
            origin=Origin(xyz=(-drawer_pull_post_length / 2.0, pull_offset, 0.0)),
            material=brass,
            name="right_pull_post",
        )
        drawer.visual(
            Box((drawer_pull_depth, drawer_pull_width, drawer_pull_height)),
            origin=Origin(
                xyz=(
                    -drawer_pull_post_length - drawer_pull_depth / 2.0,
                    0.0,
                    0.0,
                )
            ),
            material=brass,
            name="pull_bar",
        )
        drawer.inertial = Inertial.from_geometry(
            Box((drawer_shell_depth + drawer_front_thickness, drawer_front_width, front_height)),
            mass=4.0 + index * 0.5,
            origin=Origin(
                xyz=((drawer_shell_depth + drawer_front_thickness) / 2.0, 0.0, 0.0)
            ),
        )

        model.articulation(
            f"pedestal_to_{drawer_name}",
            ArticulationType.PRISMATIC,
            parent=pedestal,
            child=drawer,
            origin=Origin(xyz=(-front_setback, 0.0, center_z)),
            axis=(-1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=80.0,
                velocity=0.30,
                lower=0.0,
                upper=drawer_travel,
            ),
        )
        current_top += front_height + reveal

    model.articulation(
        "top_to_left_panel",
        ArticulationType.FIXED,
        parent=top,
        child=left_panel,
        origin=Origin(
            xyz=(
                -desk_depth / 2.0 + front_inset,
                -desk_width / 2.0 + left_panel_thickness / 2.0,
                0.0,
            )
        ),
    )
    model.articulation(
        "top_to_pedestal_body",
        ArticulationType.FIXED,
        parent=top,
        child=pedestal,
        origin=Origin(
            xyz=(
                -desk_depth / 2.0 + front_inset + front_setback,
                desk_width / 2.0 - pedestal_width / 2.0,
                0.0,
            )
        ),
    )

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

    top = object_model.get_part("top")
    left_panel = object_model.get_part("left_panel")
    pedestal = object_model.get_part("pedestal_body")
    top_slab = top.get_visual("top_slab")

    ctx.expect_gap(
        top,
        left_panel,
        axis="z",
        positive_elem=top_slab,
        max_gap=0.001,
        max_penetration=0.0,
        name="left support bears directly under the top",
    )
    ctx.expect_gap(
        top,
        pedestal,
        axis="z",
        positive_elem=top_slab,
        max_gap=0.001,
        max_penetration=0.0,
        name="pedestal bears directly under the top",
    )

    for drawer_name in ("top_drawer", "middle_drawer", "bottom_drawer"):
        drawer = object_model.get_part(drawer_name)
        slide = object_model.get_articulation(f"pedestal_to_{drawer_name}")
        slide_limits = slide.motion_limits
        upper = 0.0 if slide_limits is None or slide_limits.upper is None else slide_limits.upper

        ctx.expect_within(
            drawer,
            pedestal,
            axes="yz",
            margin=0.0035,
            name=f"{drawer_name} stays laterally captured in the pedestal",
        )
        ctx.expect_overlap(
            drawer,
            pedestal,
            axes="x",
            min_overlap=0.22,
            name=f"{drawer_name} is deeply inserted when closed",
        )

        rest_pos = ctx.part_world_position(drawer)
        with ctx.pose({slide: upper}):
            ctx.expect_within(
                drawer,
                pedestal,
                axes="yz",
                margin=0.0035,
                name=f"{drawer_name} stays aligned on its rails when open",
            )
            ctx.expect_overlap(
                drawer,
                pedestal,
                axes="x",
                min_overlap=0.12,
                name=f"{drawer_name} retains insertion when open",
            )
            open_pos = ctx.part_world_position(drawer)

        ctx.check(
            f"{drawer_name} opens forward along the desk depth",
            rest_pos is not None
            and open_pos is not None
            and open_pos[0] < rest_pos[0] - 0.12,
            details=f"rest={rest_pos}, open={open_pos}, axis={slide.axis}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
