from __future__ import annotations

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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="smart_production_slate")

    matte_black = model.material("matte_black", color=(0.08, 0.08, 0.09, 1.0))
    housing_black = model.material("housing_black", color=(0.11, 0.12, 0.13, 1.0))
    marker_white = model.material("marker_white", color=(0.95, 0.96, 0.95, 1.0))
    ivory = model.material("ivory", color=(0.92, 0.92, 0.88, 1.0))
    aluminum = model.material("aluminum", color=(0.68, 0.70, 0.73, 1.0))
    port_gray = model.material("port_gray", color=(0.28, 0.30, 0.33, 1.0))
    display_red = model.material("display_red", color=(0.73, 0.10, 0.08, 1.0))
    button_gray = model.material("button_gray", color=(0.22, 0.23, 0.24, 1.0))

    board_width = 0.285
    board_depth = 0.008
    board_height = 0.188

    mount_width = 0.060
    mount_depth = 0.014
    mount_height = 0.018
    mount_x = 0.094
    mount_y = 0.007
    mount_z = board_height / 2 + 0.009

    hinge_axis_y = 0.016
    hinge_axis_z = board_height / 2 + 0.012
    hinge_radius = 0.0045

    housing_width = 0.152
    housing_depth = 0.022
    housing_height = 0.036
    housing_center_y = -0.002
    housing_center_z = 0.123

    board = model.part("board")
    board.visual(
        Box((board_width, board_depth, board_height)),
        material=matte_black,
        name="board_backing",
    )
    board.visual(
        Box((0.252, 0.0022, 0.148)),
        origin=Origin(xyz=(0.0, 0.0029, -0.016)),
        material=marker_white,
        name="writing_field",
    )
    board.visual(
        Box((0.252, 0.0023, 0.020)),
        origin=Origin(xyz=(0.0, 0.0030, 0.067)),
        material=ivory,
        name="title_field",
    )
    for z_pos in (-0.008, -0.040, -0.072):
        board.visual(
            Box((0.232, 0.0015, 0.0022)),
            origin=Origin(xyz=(0.0, 0.0037, z_pos)),
            material=matte_black,
            name=f"marker_line_{int((z_pos + 0.1) * 1000)}",
        )

    board.visual(
        Box((mount_width, mount_depth, mount_height)),
        origin=Origin(xyz=(-mount_x, mount_y, mount_z)),
        material=housing_black,
        name="hinge_mount_0",
    )
    board.visual(
        Box((mount_width, mount_depth, mount_height)),
        origin=Origin(xyz=(mount_x, mount_y, mount_z)),
        material=housing_black,
        name="hinge_mount_1",
    )

    board.visual(
        Cylinder(radius=hinge_radius, length=0.052),
        origin=Origin(xyz=(-mount_x, hinge_axis_y, hinge_axis_z), rpy=(0.0, 1.5708, 0.0)),
        material=aluminum,
        name="hinge_barrel_0",
    )
    board.visual(
        Cylinder(radius=hinge_radius, length=0.052),
        origin=Origin(xyz=(mount_x, hinge_axis_y, hinge_axis_z), rpy=(0.0, 1.5708, 0.0)),
        material=aluminum,
        name="hinge_barrel_1",
    )

    board.visual(
        Box((housing_width, housing_depth, housing_height)),
        origin=Origin(xyz=(0.0, housing_center_y, housing_center_z)),
        material=housing_black,
        name="housing_body",
    )
    board.visual(
        Box((0.116, 0.003, 0.022)),
        origin=Origin(
            xyz=(0.0, housing_center_y + housing_depth / 2 - 0.0015, housing_center_z + 0.001),
        ),
        material=display_red,
        name="display_window",
    )
    board.visual(
        Box((0.020, 0.006, 0.004)),
        origin=Origin(xyz=(-0.048, 0.006, housing_center_z - 0.010)),
        material=button_gray,
        name="button_0",
    )
    board.visual(
        Box((0.020, 0.006, 0.004)),
        origin=Origin(xyz=(-0.020, 0.006, housing_center_z - 0.010)),
        material=button_gray,
        name="button_1",
    )
    board.visual(
        Box((0.020, 0.006, 0.004)),
        origin=Origin(xyz=(0.008, 0.006, housing_center_z - 0.010)),
        material=button_gray,
        name="button_2",
    )

    port_hinge_x = housing_width / 2 + 0.0017
    port_hinge_y = housing_center_y + housing_depth / 2 - 0.001
    port_center_z = housing_center_z - 0.001

    board.visual(
        Box((0.0012, 0.014, 0.009)),
        origin=Origin(xyz=(housing_width / 2 - 0.0006, port_hinge_y - 0.009, port_center_z)),
        material=port_gray,
        name="port_recess",
    )
    board.visual(
        Cylinder(radius=0.0017, length=0.004),
        origin=Origin(xyz=(port_hinge_x, port_hinge_y, port_center_z - 0.005)),
        material=aluminum,
        name="port_barrel_0",
    )
    board.visual(
        Cylinder(radius=0.0017, length=0.004),
        origin=Origin(xyz=(port_hinge_x, port_hinge_y, port_center_z + 0.005)),
        material=aluminum,
        name="port_barrel_1",
    )

    clapstick = model.part("clapstick")
    clapstick.visual(
        Box((0.294, 0.014, 0.012)),
        origin=Origin(xyz=(0.0, 0.027, -0.006)),
        material=matte_black,
        name="clapstick_body",
    )
    clapstick.visual(
        Box((0.120, 0.025, 0.006)),
        origin=Origin(xyz=(0.0, 0.0125, -0.005)),
        material=matte_black,
        name="clapstick_web",
    )
    clapstick.visual(
        Cylinder(radius=hinge_radius, length=0.128),
        origin=Origin(rpy=(0.0, 1.5708, 0.0)),
        material=aluminum,
        name="clapstick_barrel",
    )

    stripe_x_positions = (-0.116, -0.072, -0.028, 0.016, 0.060, 0.104)
    for index, x_pos in enumerate(stripe_x_positions):
        clapstick.visual(
            Box((0.040, 0.004, 0.004)),
            origin=Origin(xyz=(x_pos, 0.031, -0.006), rpy=(0.0, 0.60, 0.0)),
            material=ivory,
            name=f"stripe_{index}",
        )

    port_cover = model.part("port_cover")
    port_cover.visual(
        Box((0.0025, 0.018, 0.012)),
        origin=Origin(xyz=(-0.00045, -0.009, 0.0)),
        material=housing_black,
        name="cover_panel",
    )
    port_cover.visual(
        Cylinder(radius=0.0017, length=0.004),
        origin=Origin(),
        material=aluminum,
        name="cover_barrel",
    )

    model.articulation(
        "board_to_clapstick",
        ArticulationType.REVOLUTE,
        parent=board,
        child=clapstick,
        origin=Origin(xyz=(0.0, hinge_axis_y, hinge_axis_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=4.0, lower=0.0, upper=1.20),
    )
    model.articulation(
        "board_to_port_cover",
        ArticulationType.REVOLUTE,
        parent=board,
        child=port_cover,
        origin=Origin(xyz=(port_hinge_x, port_hinge_y, port_center_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.5, velocity=4.0, lower=0.0, upper=1.45),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    board = object_model.get_part("board")
    clapstick = object_model.get_part("clapstick")
    port_cover = object_model.get_part("port_cover")
    clap_hinge = object_model.get_articulation("board_to_clapstick")
    port_hinge = object_model.get_articulation("board_to_port_cover")

    ctx.expect_overlap(
        clapstick,
        board,
        axes="x",
        elem_a="clapstick_body",
        elem_b="board_backing",
        min_overlap=0.24,
        name="clapstick spans the width of the slate",
    )
    ctx.expect_gap(
        clapstick,
        board,
        axis="z",
        positive_elem="clapstick_body",
        negative_elem="board_backing",
        max_gap=0.0045,
        max_penetration=1e-6,
        name="closed clapstick sits at the top of the slate",
    )
    ctx.expect_gap(
        port_cover,
        board,
        axis="x",
        positive_elem="cover_panel",
        negative_elem="housing_body",
        max_gap=0.0015,
        max_penetration=1e-6,
        name="port cover sits flush on the housing side",
    )
    ctx.expect_overlap(
        port_cover,
        board,
        axes="yz",
        elem_a="cover_panel",
        elem_b="port_recess",
        min_overlap=0.008,
        name="port cover actually covers the sync port opening",
    )

    clap_closed = ctx.part_element_world_aabb(clapstick, elem="clapstick_body")
    if clap_hinge.motion_limits is not None and clap_hinge.motion_limits.upper is not None:
        with ctx.pose({clap_hinge: clap_hinge.motion_limits.upper}):
            clap_open = ctx.part_element_world_aabb(clapstick, elem="clapstick_body")
        ctx.check(
            "clapstick opens upward",
            clap_closed is not None
            and clap_open is not None
            and clap_open[1][2] > clap_closed[1][2] + 0.025,
            details=f"closed={clap_closed}, open={clap_open}",
        )

    cover_closed = ctx.part_element_world_aabb(port_cover, elem="cover_panel")
    if port_hinge.motion_limits is not None and port_hinge.motion_limits.upper is not None:
        with ctx.pose({port_hinge: port_hinge.motion_limits.upper}):
            cover_open = ctx.part_element_world_aabb(port_cover, elem="cover_panel")
        ctx.check(
            "port cover swings outward from the housing",
            cover_closed is not None
            and cover_open is not None
            and cover_open[1][0] > cover_closed[1][0] + 0.006,
            details=f"closed={cover_closed}, open={cover_open}",
        )

    return ctx.report()


object_model = build_object_model()
