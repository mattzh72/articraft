from __future__ import annotations

from math import pi

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


BOARD_WIDTH = 0.300
BOARD_HEIGHT = 0.240
BOARD_THICKNESS = 0.008
STRIP_HEIGHT = 0.052
STRIP_DEPTH = 0.006
STICK_LENGTH = 0.315
STICK_DEPTH = 0.016
STICK_HEIGHT = 0.048
HINGE_RADIUS = 0.0045
WINDOW_COVER_WIDTH = 0.074
WINDOW_COVER_HEIGHT = 0.030
WINDOW_COVER_DEPTH = 0.004


def _aabb_axis_bounds(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None, axis: str) -> tuple[float, float] | None:
    if aabb is None:
        return None
    axis_index = {"x": 0, "y": 1, "z": 2}[axis]
    return aabb[0][axis_index], aabb[1][axis_index]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="production_clapperboard")

    board_white = model.material("board_white", rgba=(0.96, 0.96, 0.94, 1.0))
    frame_black = model.material("frame_black", rgba=(0.08, 0.08, 0.09, 1.0))
    strip_black = model.material("strip_black", rgba=(0.03, 0.03, 0.03, 1.0))
    hinge_metal = model.material("hinge_metal", rgba=(0.72, 0.74, 0.76, 1.0))
    display_dark = model.material("display_dark", rgba=(0.08, 0.12, 0.08, 1.0))
    display_red = model.material("display_red", rgba=(0.74, 0.16, 0.10, 1.0))
    plate_gray = model.material("plate_gray", rgba=(0.43, 0.44, 0.47, 1.0))
    stick_white = model.material("stick_white", rgba=(0.97, 0.97, 0.97, 1.0))
    smoked_cover = model.material("smoked_cover", rgba=(0.17, 0.17, 0.18, 0.98))

    body = model.part("body")
    body.visual(
        Box((BOARD_WIDTH, BOARD_THICKNESS, BOARD_HEIGHT)),
        material=board_white,
        name="writing_board",
    )
    body.visual(
        Box((BOARD_WIDTH, 0.004, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, BOARD_HEIGHT / 2.0 - 0.006)),
        material=frame_black,
        name="top_frame",
    )
    body.visual(
        Box((BOARD_WIDTH, 0.004, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, -BOARD_HEIGHT / 2.0 + 0.006)),
        material=frame_black,
        name="bottom_frame",
    )
    body.visual(
        Box((0.012, 0.004, BOARD_HEIGHT - 0.024)),
        origin=Origin(xyz=(-BOARD_WIDTH / 2.0 + 0.006, 0.0, 0.0)),
        material=frame_black,
        name="side_frame_0",
    )
    body.visual(
        Box((0.012, 0.004, BOARD_HEIGHT - 0.024)),
        origin=Origin(xyz=(BOARD_WIDTH / 2.0 - 0.006, 0.0, 0.0)),
        material=frame_black,
        name="side_frame_1",
    )
    body.visual(
        Box((BOARD_WIDTH, STRIP_DEPTH, STRIP_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.007, BOARD_HEIGHT / 2.0 - STRIP_HEIGHT / 2.0)),
        material=strip_black,
        name="electronics_strip",
    )
    body.visual(
        Box((0.096, 0.001, 0.026)),
        origin=Origin(xyz=(-0.070, 0.0095, 0.094)),
        material=display_dark,
        name="display_window",
    )
    body.visual(
        Box((0.056, 0.0014, 0.016)),
        origin=Origin(xyz=(-0.070, 0.0098, 0.094)),
        material=display_red,
        name="timecode_glow",
    )
    body.visual(
        Box((0.074, 0.0015, 0.030)),
        origin=Origin(xyz=(0.090, 0.00925, 0.094)),
        material=plate_gray,
        name="id_window_frame",
    )
    body.visual(
        Box((0.066, 0.001, 0.024)),
        origin=Origin(xyz=(0.090, 0.0098, 0.094)),
        material=display_dark,
        name="id_window_glass",
    )
    for index, z_pos in enumerate((0.084, 0.104)):
        body.visual(
            Cylinder(radius=0.0018, length=0.008),
            origin=Origin(xyz=(0.053, 0.0112, z_pos)),
            material=hinge_metal,
            name=f"cover_hinge_barrel_{index}",
        )
    body.visual(
        Box((BOARD_WIDTH, 0.004, 0.008)),
        origin=Origin(xyz=(0.0, 0.0085, BOARD_HEIGHT / 2.0 - 0.004)),
        material=frame_black,
        name="clap_hinge_mount",
    )
    for index, x_pos in enumerate((-0.108, 0.108)):
        body.visual(
            Cylinder(radius=HINGE_RADIUS, length=0.050),
            origin=Origin(
                xyz=(x_pos, 0.012, BOARD_HEIGHT / 2.0),
                rpy=(0.0, pi / 2.0, 0.0),
            ),
            material=hinge_metal,
            name=f"clap_hinge_barrel_{index}",
        )

    clapstick = model.part("clapstick")
    clapstick.visual(
        Box((STICK_LENGTH, STICK_DEPTH, STICK_HEIGHT)),
        origin=Origin(xyz=(0.0, STICK_DEPTH / 2.0, -0.026)),
        material=frame_black,
        name="stick_body",
    )
    for index, x_pos in enumerate((-0.125, -0.065, -0.005, 0.055, 0.115)):
        clapstick.visual(
            Box((0.078, 0.002, 0.012)),
            origin=Origin(
                xyz=(x_pos, STICK_DEPTH - 0.001, -0.023),
                rpy=(0.0, 0.90, 0.0),
            ),
            material=stick_white,
            name=f"stripe_{index}",
        )
    for index, x_pos in enumerate((-0.056, 0.0, 0.056)):
        clapstick.visual(
            Cylinder(radius=HINGE_RADIUS - 0.0003, length=0.034),
            origin=Origin(xyz=(x_pos, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=hinge_metal,
            name=f"stick_hinge_barrel_{index}",
        )

    window_cover = model.part("window_cover")
    window_cover.visual(
        Box((WINDOW_COVER_WIDTH, WINDOW_COVER_DEPTH, WINDOW_COVER_HEIGHT)),
        origin=Origin(
            xyz=(WINDOW_COVER_WIDTH / 2.0, WINDOW_COVER_DEPTH / 2.0, 0.0),
        ),
        material=smoked_cover,
        name="cover_panel",
    )
    window_cover.visual(
        Box((0.052, 0.0015, 0.009)),
        origin=Origin(xyz=(0.042, WINDOW_COVER_DEPTH - 0.00075, -0.007)),
        material=stick_white,
        name="cover_label",
    )
    window_cover.visual(
        Box((0.008, 0.002, 0.014)),
        origin=Origin(xyz=(WINDOW_COVER_WIDTH - 0.002, WINDOW_COVER_DEPTH - 0.001, 0.0)),
        material=plate_gray,
        name="cover_pull",
    )
    window_cover.visual(
        Cylinder(radius=0.0015, length=0.010),
        origin=Origin(),
        material=hinge_metal,
        name="cover_leaf_barrel",
    )

    model.articulation(
        "body_to_clapstick",
        ArticulationType.REVOLUTE,
        parent=body,
        child=clapstick,
        origin=Origin(xyz=(0.0, 0.012, BOARD_HEIGHT / 2.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=5.0,
            lower=0.0,
            upper=1.25,
        ),
    )
    model.articulation(
        "body_to_window_cover",
        ArticulationType.REVOLUTE,
        parent=body,
        child=window_cover,
        origin=Origin(xyz=(0.053, 0.0112, 0.094)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=3.0,
            lower=0.0,
            upper=1.45,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    clapstick = object_model.get_part("clapstick")
    window_cover = object_model.get_part("window_cover")
    clap_hinge = object_model.get_articulation("body_to_clapstick")
    cover_hinge = object_model.get_articulation("body_to_window_cover")

    clap_limits = clap_hinge.motion_limits
    cover_limits = cover_hinge.motion_limits
    assert clap_limits is not None
    assert cover_limits is not None
    assert clap_limits.upper is not None
    assert cover_limits.upper is not None

    with ctx.pose({clap_hinge: 0.0, cover_hinge: 0.0}):
        ctx.expect_gap(
            clapstick,
            body,
            axis="y",
            positive_elem="stick_body",
            negative_elem="electronics_strip",
            min_gap=0.0015,
            max_gap=0.0035,
            name="closed clapstick sits just proud of the electronics strip",
        )
        ctx.expect_overlap(
            clapstick,
            body,
            axes="x",
            elem_a="stick_body",
            elem_b="electronics_strip",
            min_overlap=0.295,
            name="clapstick spans the slate width",
        )
        ctx.expect_gap(
            window_cover,
            body,
            axis="y",
            positive_elem="cover_panel",
            negative_elem="id_window_glass",
            min_gap=0.0004,
            max_gap=0.0015,
            name="closed ID cover sits just over the window",
        )
        ctx.expect_overlap(
            window_cover,
            body,
            axes="xz",
            elem_a="cover_panel",
            elem_b="id_window_frame",
            min_overlap=0.024,
            name="ID cover masks the full window frame",
        )

    with ctx.pose({clap_hinge: 0.0}):
        closed_aabb = ctx.part_element_world_aabb(clapstick, elem="stick_body")
    with ctx.pose({clap_hinge: clap_limits.upper}):
        open_aabb = ctx.part_element_world_aabb(clapstick, elem="stick_body")
    with ctx.pose({cover_hinge: 0.0}):
        cover_closed_aabb = ctx.part_element_world_aabb(window_cover, elem="cover_panel")
    with ctx.pose({cover_hinge: cover_limits.upper}):
        cover_open_aabb = ctx.part_element_world_aabb(window_cover, elem="cover_panel")

    closed_y = _aabb_axis_bounds(closed_aabb, "y")
    open_y = _aabb_axis_bounds(open_aabb, "y")
    ctx.check(
        "clapstick opens away from the slate face",
        closed_y is not None and open_y is not None and open_y[1] > closed_y[1] + 0.012,
        details=f"closed_y={closed_y}, open_y={open_y}",
    )
    ctx.check(
        "opened clapstick remains above the board top edge",
        open_aabb is not None and open_aabb[0][2] > 0.035,
        details=f"open_aabb={open_aabb}",
    )
    cover_closed_y = _aabb_axis_bounds(cover_closed_aabb, "y")
    cover_open_y = _aabb_axis_bounds(cover_open_aabb, "y")
    ctx.check(
        "ID window cover swings outward from the strip",
        cover_closed_y is not None and cover_open_y is not None and cover_open_y[1] > cover_closed_y[1] + 0.030,
        details=f"cover_closed_y={cover_closed_y}, cover_open_y={cover_open_y}",
    )
    ctx.check(
        "opened ID cover clears the display strip",
        cover_open_aabb is not None and cover_open_aabb[0][0] > 0.048,
        details=f"cover_open_aabb={cover_open_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
