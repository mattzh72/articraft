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
    model = ArticulatedObject(name="l_shaped_executive_desk")

    walnut = model.material("walnut", rgba=(0.44, 0.29, 0.18, 1.0))
    charcoal = model.material("charcoal", rgba=(0.23, 0.24, 0.26, 1.0))
    graphite = model.material("graphite", rgba=(0.31, 0.32, 0.35, 1.0))
    steel = model.material("steel", rgba=(0.72, 0.73, 0.75, 1.0))

    desk_height = 0.76
    top_thickness = 0.04
    support_height = desk_height - top_thickness

    main_top_size = (1.80, 0.90, top_thickness)
    return_top_size = (0.60, 1.10, top_thickness)

    desk_base = model.part("desk_base")
    desk_base.visual(
        Box(main_top_size),
        origin=Origin(xyz=(0.0, 0.0, desk_height - top_thickness / 2.0)),
        material=walnut,
        name="main_top",
    )
    desk_base.visual(
        Box(return_top_size),
        origin=Origin(xyz=(0.70, 1.00, desk_height - top_thickness / 2.0)),
        material=walnut,
        name="return_top",
    )
    desk_base.visual(
        Box((0.42, 0.12, 0.03)),
        origin=Origin(xyz=(0.69, 0.45, support_height - 0.015)),
        material=walnut,
        name="corner_connector",
    )
    desk_base.visual(
        Box((0.03, 0.82, support_height)),
        origin=Origin(xyz=(-0.86, 0.00, support_height / 2.0)),
        material=charcoal,
        name="left_gable",
    )
    desk_base.visual(
        Box((1.27, 0.03, 0.34)),
        origin=Origin(xyz=(-0.21, 0.28, 0.28)),
        material=charcoal,
        name="main_modesty_panel",
    )
    desk_base.visual(
        Box((0.03, 0.74, support_height)),
        origin=Origin(xyz=(0.41, 0.63, support_height / 2.0)),
        material=charcoal,
        name="inner_corner_gable",
    )
    desk_base.visual(
        Box((0.03, 0.75, support_height)),
        origin=Origin(xyz=(0.985, 1.12, support_height / 2.0)),
        material=charcoal,
        name="return_outer_gable",
    )
    desk_base.visual(
        Box((0.57, 0.03, 0.34)),
        origin=Origin(xyz=(0.70, 1.36, 0.28)),
        material=charcoal,
        name="return_modesty_panel",
    )
    desk_base.inertial = Inertial.from_geometry(
        Box((2.00, 1.60, desk_height)),
        mass=52.0,
        origin=Origin(xyz=(0.10, 0.55, desk_height / 2.0)),
    )

    pedestal_width = 0.44
    pedestal_depth = 0.60
    pedestal_height = support_height
    side_thickness = 0.018
    back_thickness = 0.012
    panel_thickness = 0.018
    rail_thickness = 0.010
    rail_length = 0.46
    rail_height = 0.022
    front_frame_thickness = 0.018

    pedestal = model.part("pedestal_body")
    pedestal.visual(
        Box((side_thickness, pedestal_depth, pedestal_height)),
        origin=Origin(
            xyz=(
                -(pedestal_width / 2.0 - side_thickness / 2.0),
                0.0,
                pedestal_height / 2.0,
            )
        ),
        material=charcoal,
        name="left_wall",
    )
    pedestal.visual(
        Box((side_thickness, pedestal_depth, pedestal_height)),
        origin=Origin(
            xyz=(
                pedestal_width / 2.0 - side_thickness / 2.0,
                0.0,
                pedestal_height / 2.0,
            )
        ),
        material=charcoal,
        name="right_wall",
    )
    pedestal.visual(
        Box((pedestal_width - 2.0 * side_thickness, pedestal_depth - back_thickness, panel_thickness)),
        origin=Origin(
            xyz=(
                0.0,
                -(back_thickness / 2.0),
                pedestal_height - panel_thickness / 2.0,
            )
        ),
        material=charcoal,
        name="carcass_top",
    )
    pedestal.visual(
        Box((pedestal_width - 2.0 * side_thickness, pedestal_depth - back_thickness, panel_thickness)),
        origin=Origin(
            xyz=(
                0.0,
                -(back_thickness / 2.0),
                0.04 + panel_thickness / 2.0,
            )
        ),
        material=charcoal,
        name="carcass_bottom",
    )
    pedestal.visual(
        Box((pedestal_width - 2.0 * side_thickness, back_thickness, 0.62)),
        origin=Origin(
            xyz=(
                0.0,
                pedestal_depth / 2.0 - back_thickness / 2.0,
                0.36,
            )
        ),
        material=graphite,
        name="back_panel",
    )
    pedestal.visual(
        Box((pedestal_width - 2.0 * side_thickness, front_frame_thickness, 0.04)),
        origin=Origin(xyz=(0.0, -(pedestal_depth / 2.0 - front_frame_thickness / 2.0), 0.02)),
        material=graphite,
        name="toe_kick",
    )
    pedestal.visual(
        Box((pedestal_width - 2.0 * side_thickness, front_frame_thickness, 0.05)),
        origin=Origin(xyz=(0.0, -(pedestal_depth / 2.0 - front_frame_thickness / 2.0), 0.659)),
        material=graphite,
        name="top_face_rail",
    )
    pedestal.visual(
        Box((pedestal_width - 2.0 * side_thickness, front_frame_thickness, 0.012)),
        origin=Origin(xyz=(0.0, -(pedestal_depth / 2.0 - front_frame_thickness / 2.0), 0.326)),
        material=graphite,
        name="middle_face_rail",
    )
    pedestal.visual(
        Box((pedestal_width - 2.0 * side_thickness, front_frame_thickness, 0.012)),
        origin=Origin(xyz=(0.0, -(pedestal_depth / 2.0 - front_frame_thickness / 2.0), 0.518)),
        material=graphite,
        name="upper_face_rail",
    )

    drawer_specs = (
        ("bottom_drawer", "pedestal_to_bottom_drawer", 0.195, 0.25, 0.244),
        ("middle_drawer", "pedestal_to_middle_drawer", 0.422, 0.18, 0.174),
        ("top_drawer", "pedestal_to_top_drawer", 0.579, 0.11, 0.104),
    )

    rail_center_x = pedestal_width / 2.0 - side_thickness - rail_thickness / 2.0
    rail_center_y = -0.05
    for drawer_name, _, drawer_z, _, _ in drawer_specs:
        pedestal.visual(
            Box((rail_thickness, rail_length, rail_height)),
            origin=Origin(xyz=(-rail_center_x, rail_center_y, drawer_z)),
            material=steel,
            name=f"{drawer_name}_left_rail",
        )
        pedestal.visual(
            Box((rail_thickness, rail_length, rail_height)),
            origin=Origin(xyz=(rail_center_x, rail_center_y, drawer_z)),
            material=steel,
            name=f"{drawer_name}_right_rail",
        )

    pedestal.inertial = Inertial.from_geometry(
        Box((pedestal_width, pedestal_depth, pedestal_height)),
        mass=24.0,
        origin=Origin(xyz=(0.0, 0.0, pedestal_height / 2.0)),
    )

    model.articulation(
        "desk_to_pedestal",
        ArticulationType.FIXED,
        parent=desk_base,
        child=pedestal,
        origin=Origin(xyz=(0.68, -0.10, 0.0)),
    )

    drawer_front_width = pedestal_width - 2.0 * side_thickness - 0.012
    drawer_body_width = 0.352
    drawer_side_thickness = 0.012
    drawer_front_thickness = 0.018
    drawer_depth = 0.518
    runner_width = 0.016
    runner_length = 0.44
    runner_height = 0.018
    drawer_side_center_x = drawer_body_width / 2.0 - drawer_side_thickness / 2.0
    drawer_runner_center_x = drawer_body_width / 2.0 + runner_width / 2.0

    for drawer_name, joint_name, drawer_z, opening_height, front_height in drawer_specs:
        drawer = model.part(drawer_name)
        drawer.visual(
            Box((drawer_front_width, drawer_front_thickness, front_height)),
            origin=Origin(xyz=(0.0, drawer_front_thickness / 2.0, 0.0)),
            material=walnut,
            name="front_panel",
        )
        drawer.visual(
            Box((drawer_side_thickness, 0.50, max(front_height - 0.030, 0.06))),
            origin=Origin(xyz=(-drawer_side_center_x, 0.268, 0.0)),
            material=graphite,
            name="left_side",
        )
        drawer.visual(
            Box((drawer_side_thickness, 0.50, max(front_height - 0.030, 0.06))),
            origin=Origin(xyz=(drawer_side_center_x, 0.268, 0.0)),
            material=graphite,
            name="right_side",
        )
        drawer.visual(
            Box((drawer_body_width, 0.494, 0.012)),
            origin=Origin(xyz=(0.0, 0.265, -(front_height / 2.0) + 0.006)),
            material=graphite,
            name="bottom_panel",
        )
        drawer.visual(
            Box((drawer_body_width, 0.012, max(front_height - 0.030, 0.06))),
            origin=Origin(xyz=(0.0, 0.512, 0.0)),
            material=graphite,
            name="back_panel",
        )
        drawer.visual(
            Box((runner_width, runner_length, runner_height)),
            origin=Origin(xyz=(-drawer_runner_center_x, 0.248, 0.0)),
            material=steel,
            name="left_runner",
        )
        drawer.visual(
            Box((runner_width, runner_length, runner_height)),
            origin=Origin(xyz=(drawer_runner_center_x, 0.248, 0.0)),
            material=steel,
            name="right_runner",
        )
        drawer.visual(
            Box((0.010, 0.016, 0.010)),
            origin=Origin(xyz=(-0.065, -0.008, 0.0)),
            material=steel,
            name="left_handle_post",
        )
        drawer.visual(
            Box((0.010, 0.016, 0.010)),
            origin=Origin(xyz=(0.065, -0.008, 0.0)),
            material=steel,
            name="right_handle_post",
        )
        drawer.visual(
            Box((0.140, 0.014, 0.010)),
            origin=Origin(xyz=(0.0, -0.023, 0.0)),
            material=steel,
            name="handle_bar",
        )
        drawer.inertial = Inertial.from_geometry(
            Box((drawer_front_width, drawer_depth, opening_height)),
            mass=4.0 if drawer_name == "bottom_drawer" else 3.0,
            origin=Origin(xyz=(0.0, drawer_depth / 2.0 - 0.03, 0.0)),
        )

        model.articulation(
            joint_name,
            ArticulationType.PRISMATIC,
            parent=pedestal,
            child=drawer,
            origin=Origin(xyz=(0.0, -(pedestal_depth / 2.0), drawer_z)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=80.0,
                velocity=0.25,
                lower=0.0,
                upper=0.26,
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

    desk_base = object_model.get_part("desk_base")
    pedestal = object_model.get_part("pedestal_body")
    top_drawer = object_model.get_part("top_drawer")
    middle_drawer = object_model.get_part("middle_drawer")
    bottom_drawer = object_model.get_part("bottom_drawer")

    top_joint = object_model.get_articulation("pedestal_to_top_drawer")
    middle_joint = object_model.get_articulation("pedestal_to_middle_drawer")
    bottom_joint = object_model.get_articulation("pedestal_to_bottom_drawer")

    ctx.expect_contact(
        pedestal,
        desk_base,
        elem_a="carcass_top",
        elem_b="main_top",
        name="pedestal supports the main top",
    )

    for drawer, joint, drawer_name in (
        (top_drawer, top_joint, "top_drawer"),
        (middle_drawer, middle_joint, "middle_drawer"),
        (bottom_drawer, bottom_joint, "bottom_drawer"),
    ):
        ctx.expect_contact(
            drawer,
            pedestal,
            elem_a="left_runner",
            elem_b=f"{drawer_name}_left_rail",
            name=f"{drawer_name} left runner rides on left guide rail",
        )
        ctx.expect_contact(
            drawer,
            pedestal,
            elem_a="right_runner",
            elem_b=f"{drawer_name}_right_rail",
            name=f"{drawer_name} right runner rides on right guide rail",
        )
        ctx.expect_within(
            drawer,
            pedestal,
            axes="xz",
            margin=0.02,
            name=f"{drawer_name} stays laterally contained by pedestal footprint",
        )

        closed_pos = ctx.part_world_position(drawer)
        upper = joint.motion_limits.upper if joint.motion_limits is not None else None
        with ctx.pose({joint: upper if upper is not None else 0.26}):
            ctx.expect_overlap(
                drawer,
                pedestal,
                axes="y",
                elem_a="left_runner",
                elem_b=f"{drawer_name}_left_rail",
                min_overlap=0.17,
                name=f"{drawer_name} retains insertion on its left guide rail when open",
            )
            open_pos = ctx.part_world_position(drawer)

        ctx.check(
            f"{drawer_name} opens forward along the pedestal rails",
            closed_pos is not None
            and open_pos is not None
            and open_pos[1] < closed_pos[1] - 0.20,
            details=f"closed={closed_pos}, open={open_pos}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
