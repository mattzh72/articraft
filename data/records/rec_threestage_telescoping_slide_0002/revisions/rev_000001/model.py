from __future__ import annotations

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

ASSETS = AssetContext.from_script(__file__)
HERE = ASSETS.asset_root
MESH_DIR = ASSETS.ensure_mesh_dir()


# >>> USER_CODE_START
OUTER_LEN = 0.340
OUTER_W = 0.038
OUTER_H = 0.030
OUTER_WALL = 0.003

MIDDLE_LEN = 0.270
MIDDLE_W = 0.029
MIDDLE_H = 0.021
MIDDLE_WALL = 0.0025

INNER_LEN = 0.200
INNER_W = 0.020
INNER_H = 0.012
INNER_WALL = 0.002

OUTER_BODY_ORIGIN = (0.150, 0.0, 0.0)
MIDDLE_BODY_ORIGIN = (0.120, 0.0, 0.0)
INNER_BODY_ORIGIN = (0.085, 0.0, 0.0)

OUTER_BRACKET_SIZE = (0.010, 0.050, 0.052)
INNER_BRACKET_SIZE = (0.010, 0.042, 0.048)
OUTER_BRACKET_ORIGIN = (-0.025, 0.0, 0.0)
INNER_BRACKET_ORIGIN = (0.190, 0.0, 0.0)

OUTER_TO_MIDDLE_MAX = 0.220
MIDDLE_TO_INNER_HOME = 0.138
MIDDLE_TO_INNER_MAX = 0.095


def _origin(xyz: tuple[float, float, float]) -> Origin:
    return Origin(xyz=xyz)


def _make_hollow_box_rail(
    length: float,
    outer_width: float,
    outer_height: float,
    wall: float,
) -> cq.Workplane:
    outer = cq.Workplane("XY").box(length, outer_width, outer_height)
    inner = cq.Workplane("XY").box(
        length + 0.004,
        outer_width - 2.0 * wall,
        outer_height - 2.0 * wall,
    )
    return outer.cut(inner)


def _make_end_bracket(
    thickness: float,
    width: float,
    height: float,
    hole_pitch_y: float,
    hole_pitch_z: float,
) -> cq.Workplane:
    bracket = cq.Workplane("XY").box(thickness, width, height)
    bracket = bracket.edges("|X").fillet(min(0.0035, width * 0.08, height * 0.08))
    bracket = (
        bracket.faces(">X")
        .workplane(centerOption="CenterOfMass")
        .pushPoints(
            [
                (-hole_pitch_y / 2.0, -hole_pitch_z / 2.0),
                (-hole_pitch_y / 2.0, hole_pitch_z / 2.0),
                (hole_pitch_y / 2.0, -hole_pitch_z / 2.0),
                (hole_pitch_y / 2.0, hole_pitch_z / 2.0),
            ]
        )
        .hole(0.0055)
    )
    return bracket


def _make_top_tabs(
    x_positions: list[float],
    width: float,
    top_z: float,
    length: float,
    height: float,
) -> cq.Workplane:
    tabs: cq.Workplane | None = None
    for x_pos in x_positions:
        tab = (
            cq.Workplane("XY")
            .box(length, width, height)
            .translate((x_pos, 0.0, top_z + height / 2.0))
        )
        tabs = tab if tabs is None else tabs.union(tab)
    if tabs is None:
        raise ValueError("expected at least one stop-tab position")
    return tabs


def _add_mesh_visual(
    part,
    shape: cq.Workplane,
    filename: str,
    material: str,
) -> None:
    part.visual(mesh_from_cadquery(shape, filename, assets=ASSETS), material=material)


def _add_tube_collisions(
    part,
    name_prefix: str,
    length: float,
    outer_width: float,
    outer_height: float,
    wall: float,
    center: tuple[float, float, float],
) -> None:
    cx, cy, cz = center
    collision_length = length - 0.006
    side_height = outer_height - 2.0 * wall
    cap_width = outer_width - 2.0 * wall







def _pose_ctx(ctx: TestContext, positions: dict[str, float]):
    try:
        return ctx.pose(positions)
    except TypeError:
        return ctx.pose(**positions)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="three_stage_telescoping_slide", assets=ASSETS)

    model.material("outer_steel", rgba=(0.28, 0.30, 0.33, 1.0))
    model.material("middle_steel", rgba=(0.56, 0.58, 0.60, 1.0))
    model.material("inner_steel", rgba=(0.73, 0.74, 0.76, 1.0))
    model.material("machined_bracket", rgba=(0.82, 0.84, 0.86, 1.0))
    model.material("stop_feature", rgba=(0.16, 0.18, 0.20, 1.0))

    outer = model.part("outer_rail")
    middle = model.part("middle_rail")
    inner = model.part("inner_rail")

    outer_rail_shape = _make_hollow_box_rail(OUTER_LEN, OUTER_W, OUTER_H, OUTER_WALL).translate(
        OUTER_BODY_ORIGIN
    )
    outer_bracket_shape = _make_end_bracket(
        *OUTER_BRACKET_SIZE,
        hole_pitch_y=0.028,
        hole_pitch_z=0.032,
    ).translate(OUTER_BRACKET_ORIGIN)

    middle_rail_shape = _make_hollow_box_rail(
        MIDDLE_LEN, MIDDLE_W, MIDDLE_H, MIDDLE_WALL
    ).translate(MIDDLE_BODY_ORIGIN)
    middle_stop_shape = _make_top_tabs(
        x_positions=[0.042, 0.248],
        width=0.017,
        top_z=MIDDLE_H / 2.0,
        length=0.008,
        height=0.001,
    )

    inner_rail_shape = _make_hollow_box_rail(INNER_LEN, INNER_W, INNER_H, INNER_WALL).translate(
        INNER_BODY_ORIGIN
    )
    inner_bracket_shape = _make_end_bracket(
        *INNER_BRACKET_SIZE,
        hole_pitch_y=0.024,
        hole_pitch_z=0.028,
    ).translate(INNER_BRACKET_ORIGIN)
    inner_stop_shape = _make_top_tabs(
        x_positions=[0.020],
        width=0.013,
        top_z=INNER_H / 2.0,
        length=0.008,
        height=0.0012,
    )

    _add_mesh_visual(outer, outer_rail_shape, "outer_rail_body.obj", "outer_steel")
    _add_mesh_visual(outer, outer_bracket_shape, "outer_end_bracket.obj", "machined_bracket")

    _add_mesh_visual(middle, middle_rail_shape, "middle_rail_body.obj", "middle_steel")
    _add_mesh_visual(middle, middle_stop_shape, "middle_stop_tabs.obj", "stop_feature")

    _add_mesh_visual(inner, inner_rail_shape, "inner_rail_body.obj", "inner_steel")
    _add_mesh_visual(inner, inner_bracket_shape, "inner_end_bracket.obj", "machined_bracket")
    _add_mesh_visual(inner, inner_stop_shape, "inner_stop_tab.obj", "stop_feature")

    _add_tube_collisions(
        outer,
        "outer",
        OUTER_LEN,
        OUTER_W,
        OUTER_H,
        OUTER_WALL,
        OUTER_BODY_ORIGIN,
    )


    _add_tube_collisions(
        middle,
        "middle",
        MIDDLE_LEN,
        MIDDLE_W,
        MIDDLE_H,
        MIDDLE_WALL,
        MIDDLE_BODY_ORIGIN,
    )

    _add_tube_collisions(
        inner,
        "inner",
        INNER_LEN,
        INNER_W,
        INNER_H,
        INNER_WALL,
        INNER_BODY_ORIGIN,
    )


    outer.inertial = Inertial.from_geometry(
        Box((0.350, 0.050, 0.052)),
        mass=1.30,
        origin=_origin((0.145, 0.0, 0.0)),
    )
    middle.inertial = Inertial.from_geometry(
        Box((0.270, 0.029, 0.022)),
        mass=0.82,
        origin=_origin((0.120, 0.0, 0.0005)),
    )
    inner.inertial = Inertial.from_geometry(
        Box((0.210, 0.042, 0.048)),
        mass=0.62,
        origin=_origin((0.090, 0.0, 0.0)),
    )

    model.articulation(
        "outer_to_middle",
        ArticulationType.PRISMATIC,
        parent=outer,
        child=middle,
        origin=_origin((0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=OUTER_TO_MIDDLE_MAX,
            effort=80.0,
            velocity=0.60,
        ),
    )
    model.articulation(
        "middle_to_inner",
        ArticulationType.PRISMATIC,
        parent=middle,
        child=inner,
        origin=_origin((MIDDLE_TO_INNER_HOME, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=MIDDLE_TO_INNER_MAX,
            effort=65.0,
            velocity=0.60,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)
    ctx.fail_if_articulation_origin_far_from_geometry(tol=0.02)
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=96,
        overlap_tol=0.001,
        overlap_volume_tol=0.0,
    )

    ctx.expect_joint_motion_axis(
        "outer_to_middle",
        "middle_rail",
        world_axis="x",
        direction="positive",
        min_delta=0.03,
    )
    ctx.expect_joint_motion_axis(
        "middle_to_inner",
        "inner_rail",
        world_axis="x",
        direction="positive",
        min_delta=0.02,
    )

    with _pose_ctx(ctx, {"outer_to_middle": 0.0, "middle_to_inner": 0.0}):
        ctx.expect_aabb_overlap("outer_rail", "middle_rail", axes="xy", min_overlap=0.01)
        ctx.expect_aabb_overlap("middle_rail", "inner_rail", axes="xy", min_overlap=0.01)
        ctx.expect_aabb_overlap("outer_rail", "inner_rail", axes="xy", min_overlap=0.01)

    with _pose_ctx(ctx, {"outer_to_middle": OUTER_TO_MIDDLE_MAX, "middle_to_inner": 0.0}):
        ctx.expect_aabb_overlap("outer_rail", "middle_rail", axes="xy", min_overlap=0.01)
        ctx.expect_aabb_overlap("middle_rail", "inner_rail", axes="xy", min_overlap=0.01)

    with _pose_ctx(
        ctx,
        {
            "outer_to_middle": OUTER_TO_MIDDLE_MAX,
            "middle_to_inner": MIDDLE_TO_INNER_MAX,
        },
    ):
        ctx.expect_aabb_overlap("outer_rail", "middle_rail", axes="xy", min_overlap=0.01)
        ctx.expect_aabb_overlap("middle_rail", "inner_rail", axes="xy", min_overlap=0.01)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
