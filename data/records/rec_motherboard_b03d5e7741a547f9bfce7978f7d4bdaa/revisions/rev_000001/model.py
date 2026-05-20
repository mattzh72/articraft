from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_motherboard")

    board_green = model.material("board_green", rgba=(0.15, 0.39, 0.20, 1.0))
    solder_mask = model.material("solder_mask_dark", rgba=(0.08, 0.10, 0.09, 1.0))
    socket_black = model.material("socket_black", rgba=(0.12, 0.12, 0.12, 1.0))
    metal = model.material("metal", rgba=(0.73, 0.75, 0.78, 1.0))
    steel_dark = model.material("steel_dark", rgba=(0.52, 0.55, 0.58, 1.0))
    connector = model.material("connector", rgba=(0.82, 0.82, 0.79, 1.0))

    board_x = 0.170
    board_y = 0.140
    board_t = 0.002

    socket_cx = -0.012
    socket_cy = -0.004
    socket_outer_x = 0.058
    socket_outer_y = 0.050
    socket_wall = 0.005
    socket_h = 0.008

    frame_outer_x = 0.064
    frame_outer_y = 0.056
    frame_bar = 0.004
    frame_t = 0.0016
    frame_hinge_x = socket_cx - frame_outer_x / 2.0
    frame_axis_z = board_t + socket_h + frame_t / 2.0 + 0.0002

    wifi_cx = 0.050
    wifi_hinge_y = board_y / 2.0 - 0.006
    wifi_axis_z = 0.0062
    wifi_bed_y = 0.043

    motherboard = model.part("motherboard")
    motherboard.visual(
        Box((board_x, board_y, board_t)),
        origin=Origin(xyz=(0.0, 0.0, board_t / 2.0)),
        material=board_green,
        name="board_pcb",
    )

    motherboard.visual(
        Box((socket_wall, socket_outer_y, socket_h)),
        origin=Origin(
            xyz=(
                socket_cx - (socket_outer_x / 2.0 - socket_wall / 2.0),
                socket_cy,
                board_t + socket_h / 2.0,
            )
        ),
        material=socket_black,
        name="socket_wall_left",
    )
    motherboard.visual(
        Box((socket_wall, socket_outer_y, socket_h)),
        origin=Origin(
            xyz=(
                socket_cx + (socket_outer_x / 2.0 - socket_wall / 2.0),
                socket_cy,
                board_t + socket_h / 2.0,
            )
        ),
        material=socket_black,
        name="socket_wall_right",
    )
    motherboard.visual(
        Box((socket_outer_x - 2.0 * socket_wall, socket_wall, socket_h)),
        origin=Origin(
            xyz=(
                socket_cx,
                socket_cy + (socket_outer_y / 2.0 - socket_wall / 2.0),
                board_t + socket_h / 2.0,
            )
        ),
        material=socket_black,
        name="socket_wall_top",
    )
    motherboard.visual(
        Box((socket_outer_x - 2.0 * socket_wall, socket_wall, socket_h)),
        origin=Origin(
            xyz=(
                socket_cx,
                socket_cy - (socket_outer_y / 2.0 - socket_wall / 2.0),
                board_t + socket_h / 2.0,
            )
        ),
        material=socket_black,
        name="socket_wall_bottom",
    )
    motherboard.visual(
        Box((0.043, 0.035, 0.001)),
        origin=Origin(xyz=(socket_cx, socket_cy, board_t + 0.0005)),
        material=solder_mask,
        name="socket_contact_field",
    )

    motherboard.visual(
        Box((0.004, 0.010, 0.009)),
        origin=Origin(
            xyz=(
                frame_hinge_x - 0.004,
                socket_cy + 0.017,
                board_t + 0.009 / 2.0,
            )
        ),
        material=steel_dark,
        name="socket_hinge_block_top",
    )
    motherboard.visual(
        Box((0.004, 0.010, 0.009)),
        origin=Origin(
            xyz=(
                frame_hinge_x - 0.004,
                socket_cy - 0.017,
                board_t + 0.009 / 2.0,
            )
        ),
        material=steel_dark,
        name="socket_hinge_block_bottom",
    )

    motherboard.visual(
        Box((0.018, 0.034, 0.018)),
        origin=Origin(xyz=(-0.073, 0.050, board_t + 0.018 / 2.0)),
        material=steel_dark,
        name="rear_io_shroud",
    )
    motherboard.visual(
        Box((0.018, 0.050, 0.014)),
        origin=Origin(xyz=(-0.045, 0.037, board_t + 0.014 / 2.0)),
        material=steel_dark,
        name="vrm_heatsink",
    )
    motherboard.visual(
        Box((0.026, 0.026, 0.010)),
        origin=Origin(xyz=(0.030, -0.036, board_t + 0.010 / 2.0)),
        material=steel_dark,
        name="chipset_heatsink",
    )
    motherboard.visual(
        Box((0.032, 0.038, 0.003)),
        origin=Origin(xyz=(wifi_cx, wifi_bed_y, board_t + 0.003 / 2.0)),
        material=solder_mask,
        name="wifi_module_bed",
    )
    motherboard.visual(
        Box((0.036, 0.004, wifi_axis_z - board_t)),
        origin=Origin(
            xyz=(
                wifi_cx,
                wifi_hinge_y + 0.002,
                board_t + (wifi_axis_z - board_t) / 2.0,
            )
        ),
        material=metal,
        name="wifi_hinge_bracket",
    )
    motherboard.visual(
        Box((0.008, 0.040, 0.012)),
        origin=Origin(xyz=(0.075, -0.012, board_t + 0.012 / 2.0)),
        material=connector,
        name="power_connector",
    )

    socket_frame = model.part("socket_frame")
    socket_frame.visual(
        Box((frame_bar, frame_outer_y, frame_t)),
        origin=Origin(xyz=(frame_bar / 2.0, 0.0, 0.0)),
        material=metal,
        name="frame_hinge_bar",
    )
    socket_frame.visual(
        Box((frame_bar, frame_outer_y, frame_t)),
        origin=Origin(xyz=(frame_outer_x - frame_bar / 2.0, 0.0, 0.0)),
        material=metal,
        name="frame_free_bar",
    )
    socket_frame.visual(
        Box((frame_outer_x - 2.0 * frame_bar, frame_bar, frame_t)),
        origin=Origin(xyz=(frame_outer_x / 2.0, frame_outer_y / 2.0 - frame_bar / 2.0, 0.0)),
        material=metal,
        name="frame_top_bar",
    )
    socket_frame.visual(
        Box((frame_outer_x - 2.0 * frame_bar, frame_bar, frame_t)),
        origin=Origin(xyz=(frame_outer_x / 2.0, -frame_outer_y / 2.0 + frame_bar / 2.0, 0.0)),
        material=metal,
        name="frame_bottom_bar",
    )

    wifi_cover = model.part("wifi_cover")
    wifi_cover.visual(
        Box((0.034, 0.004, 0.0024)),
        origin=Origin(xyz=(0.0, -0.002, 0.0)),
        material=metal,
        name="cover_leaf",
    )
    wifi_cover.visual(
        Box((0.034, 0.036, 0.0012)),
        origin=Origin(xyz=(0.0, -0.022, 0.0)),
        material=metal,
        name="cover_panel",
    )

    model.articulation(
        "socket_frame_hinge",
        ArticulationType.REVOLUTE,
        parent=motherboard,
        child=socket_frame,
        origin=Origin(xyz=(frame_hinge_x, socket_cy, frame_axis_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=3.0,
            lower=0.0,
            upper=1.25,
        ),
    )
    model.articulation(
        "wifi_cover_hinge",
        ArticulationType.REVOLUTE,
        parent=motherboard,
        child=wifi_cover,
        origin=Origin(xyz=(wifi_cx, wifi_hinge_y, wifi_axis_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=3.0,
            lower=0.0,
            upper=1.20,
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

    motherboard = object_model.get_part("motherboard")
    socket_frame = object_model.get_part("socket_frame")
    wifi_cover = object_model.get_part("wifi_cover")
    socket_hinge = object_model.get_articulation("socket_frame_hinge")
    wifi_hinge = object_model.get_articulation("wifi_cover_hinge")

    with ctx.pose({socket_hinge: 0.0, wifi_hinge: 0.0}):
        ctx.expect_overlap(
            socket_frame,
            motherboard,
            axes="xy",
            elem_a="frame_top_bar",
            elem_b="socket_wall_top",
            min_overlap=0.0005,
            name="socket frame aligns over the processor socket",
        )
        ctx.expect_gap(
            socket_frame,
            motherboard,
            axis="z",
            positive_elem="frame_top_bar",
            negative_elem="socket_wall_top",
            max_gap=0.003,
            max_penetration=0.0,
            name="socket frame closes just above the socket housing",
        )
        ctx.expect_overlap(
            wifi_cover,
            motherboard,
            axes="xy",
            elem_a="cover_panel",
            elem_b="wifi_module_bed",
            min_overlap=0.028,
            name="wi-fi cover sits over the wireless module zone",
        )
        ctx.expect_gap(
            wifi_cover,
            motherboard,
            axis="z",
            positive_elem="cover_panel",
            negative_elem="wifi_module_bed",
            max_gap=0.002,
            max_penetration=0.0,
            name="wi-fi cover closes close to the module bed",
        )
        socket_closed = ctx.part_element_world_aabb(socket_frame, elem="frame_free_bar")
        cover_closed = ctx.part_element_world_aabb(wifi_cover, elem="cover_panel")

    with ctx.pose({socket_hinge: 1.15}):
        socket_open = ctx.part_element_world_aabb(socket_frame, elem="frame_free_bar")

    with ctx.pose({wifi_hinge: 1.10}):
        cover_open = ctx.part_element_world_aabb(wifi_cover, elem="cover_panel")

    ctx.check(
        "socket frame opens upward",
        socket_closed is not None
        and socket_open is not None
        and socket_open[1][2] > socket_closed[1][2] + 0.030,
        details=f"closed={socket_closed}, open={socket_open}",
    )
    ctx.check(
        "wi-fi cover swings upward",
        cover_closed is not None
        and cover_open is not None
        and cover_open[1][2] > cover_closed[1][2] + 0.020,
        details=f"closed={cover_closed}, open={cover_open}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
