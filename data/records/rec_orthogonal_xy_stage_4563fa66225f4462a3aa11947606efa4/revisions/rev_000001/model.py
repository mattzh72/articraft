from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_L = 0.38
BASE_W = 0.24
BASE_T = 0.018
BASE_END_BLOCK_L = 0.028
BASE_END_BLOCK_W = 0.155
BASE_END_BLOCK_H = 0.020

X_GUIDE_L = 0.32
X_GUIDE_SPACING = 0.146
X_PED_W = 0.032
X_PED_H = 0.015
X_RAIL_W = 0.014
X_RAIL_H = 0.010

X_STAGE_L = 0.22
X_STAGE_W = 0.22
X_STAGE_T = 0.016
X_BEARING_BLOCK_L = 0.046
X_BEARING_BLOCK_W = 0.028
X_BEARING_BLOCK_H = 0.020
X_BEARING_SPAN_L = 0.17
X_BEARING_STRAP_W = 0.020
X_BEARING_STRAP_H = 0.002
X_TRAVEL = 0.075

Y_GUIDE_L = 0.20
Y_GUIDE_SPACING = 0.096
Y_PED_W = 0.028
Y_PED_H = 0.010
Y_RAIL_W = 0.012
Y_RAIL_H = 0.008

TABLE_L = 0.18
TABLE_W = 0.16
TABLE_T = 0.016
Y_BEARING_BLOCK_X = 0.026
Y_BEARING_BLOCK_Y = 0.034
Y_BEARING_BLOCK_H = 0.015
Y_BEARING_SPAN_Y = 0.12
Y_BEARING_STRAP_X = 0.020
Y_BEARING_STRAP_H = 0.002
Y_TRAVEL = 0.03

GUIDE_SINK = 0.0008
BEARING_TOP_OVERLAP = 0.0008
X_STAGE_FRAME_Z = 0.0622
Y_STAGE_FRAME_Z = 0.0482


def _box_at(size: tuple[float, float, float], xyz: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size, centered=(True, True, False)).translate(xyz)


def _base_body_shape() -> cq.Workplane:
    body = cq.Workplane("XY").box(BASE_L, BASE_W, BASE_T, centered=(True, True, False))

    for y_pos in (-X_GUIDE_SPACING / 2.0, X_GUIDE_SPACING / 2.0):
        body = body.union(_box_at((X_GUIDE_L, X_PED_W, X_PED_H), (0.0, y_pos, BASE_T)))

    for x_pos in (-BASE_L / 2.0 + 0.030, BASE_L / 2.0 - 0.030):
        body = body.union(
            _box_at((BASE_END_BLOCK_L, BASE_END_BLOCK_W, BASE_END_BLOCK_H), (x_pos, 0.0, BASE_T))
        )

    body = (
        body.faces(">Z")
        .workplane()
        .rect(0.16, 0.072)
        .cutBlind(-0.006)
        .faces(">Z")
        .workplane()
        .pushPoints(
            [
                (-0.145, -0.095),
                (-0.145, 0.095),
                (0.145, -0.095),
                (0.145, 0.095),
            ]
        )
        .hole(0.008)
    )
    return body


def _base_x_guides_shape() -> cq.Workplane:
    guides = None
    rail_bottom = BASE_T + X_PED_H - GUIDE_SINK
    for y_pos in (-X_GUIDE_SPACING / 2.0, X_GUIDE_SPACING / 2.0):
        rail = _box_at((X_GUIDE_L, X_RAIL_W, X_RAIL_H + GUIDE_SINK), (0.0, y_pos, rail_bottom))
        guides = rail if guides is None else guides.union(rail)

    bridge_w = X_GUIDE_SPACING + X_RAIL_W + 0.008
    for x_pos in (-X_GUIDE_L / 2.0 + 0.008, X_GUIDE_L / 2.0 - 0.008):
        guides = guides.union(
            _box_at((0.016, bridge_w, X_RAIL_H + GUIDE_SINK), (x_pos, 0.0, rail_bottom))
        )
    return guides


def _x_stage_body_shape() -> cq.Workplane:
    body = cq.Workplane("XY").box(X_STAGE_L, X_STAGE_W, X_STAGE_T, centered=(True, True, False))

    for x_pos in (-Y_GUIDE_SPACING / 2.0, Y_GUIDE_SPACING / 2.0):
        body = body.union(_box_at((Y_PED_W, Y_GUIDE_L, Y_PED_H), (x_pos, 0.0, X_STAGE_T)))

    for y_pos in (-0.075, 0.075):
        body = body.union(_box_at((0.11, 0.018, 0.010), (0.0, y_pos, X_STAGE_T)))

    body = (
        body.faces(">Z")
        .workplane()
        .rect(0.11, 0.085)
        .cutBlind(-0.004)
        .faces(">Z")
        .workplane()
        .pushPoints([(-0.060, -0.060), (-0.060, 0.060), (0.060, -0.060), (0.060, 0.060)])
        .hole(0.006)
    )
    return body


def _x_stage_x_bearings_shape() -> cq.Workplane:
    bearings = None
    strap_bottom = -0.001
    block_bottom = -X_BEARING_BLOCK_H + BEARING_TOP_OVERLAP

    for y_pos in (-X_GUIDE_SPACING / 2.0, X_GUIDE_SPACING / 2.0):
        strap = _box_at(
            (X_BEARING_SPAN_L, X_BEARING_STRAP_W, X_BEARING_STRAP_H),
            (0.0, y_pos, strap_bottom),
        )
        bearings = strap if bearings is None else bearings.union(strap)

        for x_pos in (-0.055, 0.055):
            bearings = bearings.union(
                _box_at(
                    (X_BEARING_BLOCK_L, X_BEARING_BLOCK_W, X_BEARING_BLOCK_H),
                    (x_pos, y_pos, block_bottom),
                )
            )
    return bearings


def _x_stage_y_guides_shape() -> cq.Workplane:
    guides = None
    rail_bottom = X_STAGE_T + Y_PED_H - GUIDE_SINK

    for x_pos in (-Y_GUIDE_SPACING / 2.0, Y_GUIDE_SPACING / 2.0):
        rail = _box_at((Y_RAIL_W, Y_GUIDE_L, Y_RAIL_H + GUIDE_SINK), (x_pos, 0.0, rail_bottom))
        guides = rail if guides is None else guides.union(rail)

    bridge_l = Y_GUIDE_SPACING + Y_RAIL_W + 0.008
    for y_pos in (-Y_GUIDE_L / 2.0 + 0.008, Y_GUIDE_L / 2.0 - 0.008):
        guides = guides.union(
            _box_at((bridge_l, 0.016, Y_RAIL_H + GUIDE_SINK), (0.0, y_pos, rail_bottom))
        )
    return guides


def _y_stage_body_shape() -> cq.Workplane:
    body = cq.Workplane("XY").box(TABLE_L, TABLE_W, TABLE_T, centered=(True, True, False))
    body = (
        body.faces(">Z")
        .workplane()
        .rect(0.10, 0.08)
        .cutBlind(-0.0035)
        .faces(">Z")
        .workplane()
        .pushPoints([(-0.050, -0.045), (-0.050, 0.045), (0.050, -0.045), (0.050, 0.045)])
        .hole(0.005)
    )
    return body


def _y_stage_bearings_shape() -> cq.Workplane:
    bearings = None
    strap_bottom = -0.001
    block_bottom = -Y_BEARING_BLOCK_H + BEARING_TOP_OVERLAP

    for x_pos in (-Y_GUIDE_SPACING / 2.0, Y_GUIDE_SPACING / 2.0):
        strap = _box_at(
            (Y_BEARING_STRAP_X, Y_BEARING_SPAN_Y, Y_BEARING_STRAP_H),
            (x_pos, 0.0, strap_bottom),
        )
        bearings = strap if bearings is None else bearings.union(strap)

        for y_pos in (-0.042, 0.042):
            bearings = bearings.union(
                _box_at(
                    (Y_BEARING_BLOCK_X, Y_BEARING_BLOCK_Y, Y_BEARING_BLOCK_H),
                    (x_pos, y_pos, block_bottom),
                )
            )
    return bearings


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="xy_positioning_stage")

    model.material("base_black", rgba=(0.18, 0.19, 0.21, 1.0))
    model.material("carriage_gray", rgba=(0.58, 0.61, 0.66, 1.0))
    model.material("table_blue", rgba=(0.48, 0.57, 0.72, 1.0))
    model.material("rail_steel", rgba=(0.73, 0.75, 0.79, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_base_body_shape(), "base_body"),
        material="base_black",
        name="elem_body",
    )
    base.visual(
        mesh_from_cadquery(_base_x_guides_shape(), "base_x_guides"),
        material="rail_steel",
        name="elem_x_guides",
    )
    base.inertial = Inertial.from_geometry(
        Box((BASE_L, BASE_W, BASE_T + X_PED_H + X_RAIL_H)),
        mass=7.0,
        origin=Origin(xyz=(0.0, 0.0, (BASE_T + X_PED_H + X_RAIL_H) / 2.0)),
    )

    x_stage = model.part("x_stage")
    x_stage.visual(
        mesh_from_cadquery(_x_stage_body_shape(), "x_stage_body"),
        material="carriage_gray",
        name="elem_body",
    )
    x_stage.visual(
        mesh_from_cadquery(_x_stage_x_bearings_shape(), "x_stage_x_bearings"),
        material="carriage_gray",
        name="elem_x_bearings",
    )
    x_stage.visual(
        mesh_from_cadquery(_x_stage_y_guides_shape(), "x_stage_y_guides"),
        material="rail_steel",
        name="elem_y_guides",
    )
    x_stage.inertial = Inertial.from_geometry(
        Box((X_STAGE_L, X_STAGE_W, X_STAGE_T + Y_PED_H + Y_RAIL_H + X_BEARING_BLOCK_H)),
        mass=3.2,
        origin=Origin(
            xyz=(0.0, 0.0, (X_STAGE_T + Y_PED_H + Y_RAIL_H - X_BEARING_BLOCK_H) / 2.0)
        ),
    )

    y_stage = model.part("y_stage")
    y_stage.visual(
        mesh_from_cadquery(_y_stage_body_shape(), "y_stage_body"),
        material="table_blue",
        name="elem_body",
    )
    y_stage.visual(
        mesh_from_cadquery(_y_stage_bearings_shape(), "y_stage_bearings"),
        material="carriage_gray",
        name="elem_y_bearings",
    )
    y_stage.inertial = Inertial.from_geometry(
        Box((TABLE_L, TABLE_W, TABLE_T + Y_BEARING_BLOCK_H)),
        mass=1.6,
        origin=Origin(xyz=(0.0, 0.0, (TABLE_T - Y_BEARING_BLOCK_H) / 2.0)),
    )

    model.articulation(
        "base_to_x",
        ArticulationType.PRISMATIC,
        parent=base,
        child=x_stage,
        origin=Origin(xyz=(0.0, 0.0, X_STAGE_FRAME_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=-X_TRAVEL,
            upper=X_TRAVEL,
            effort=180.0,
            velocity=0.25,
        ),
    )
    model.articulation(
        "x_to_y",
        ArticulationType.PRISMATIC,
        parent=x_stage,
        child=y_stage,
        origin=Origin(xyz=(0.0, 0.0, Y_STAGE_FRAME_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            lower=-Y_TRAVEL,
            upper=Y_TRAVEL,
            effort=120.0,
            velocity=0.20,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    x_stage = object_model.get_part("x_stage")
    y_stage = object_model.get_part("y_stage")
    base_to_x = object_model.get_articulation("base_to_x")
    x_to_y = object_model.get_articulation("x_to_y")

    base_x_guides = base.get_visual("elem_x_guides")
    x_bearings = x_stage.get_visual("elem_x_bearings")
    x_y_guides = x_stage.get_visual("elem_y_guides")
    y_bearings = y_stage.get_visual("elem_y_bearings")

    ctx.expect_contact(
        x_stage,
        base,
        contact_tol=0.0003,
        elem_a=x_bearings,
        elem_b=base_x_guides,
        name="x carriage bearings stay in contact with x guides",
    )
    ctx.expect_within(
        x_stage,
        base,
        axes="y",
        margin=0.010,
        inner_elem=x_bearings,
        outer_elem=base_x_guides,
        name="x carriage remains centered over x guide pair",
    )
    ctx.expect_overlap(
        x_stage,
        base,
        axes="x",
        min_overlap=0.14,
        elem_a=x_bearings,
        elem_b=base_x_guides,
        name="x carriage retains engagement on x guides",
    )

    ctx.expect_contact(
        y_stage,
        x_stage,
        contact_tol=0.0003,
        elem_a=y_bearings,
        elem_b=x_y_guides,
        name="y table bearings stay in contact with y guides",
    )
    ctx.expect_within(
        y_stage,
        x_stage,
        axes="x",
        margin=0.008,
        inner_elem=y_bearings,
        outer_elem=x_y_guides,
        name="y table remains centered over y guide pair",
    )
    ctx.expect_overlap(
        y_stage,
        x_stage,
        axes="y",
        min_overlap=0.10,
        elem_a=y_bearings,
        elem_b=x_y_guides,
        name="y table retains engagement on y guides",
    )

    rest_x_pos = ctx.part_world_position(x_stage)
    rest_y_pos = ctx.part_world_position(y_stage)
    with ctx.pose({base_to_x: X_TRAVEL}):
        extended_x_pos = ctx.part_world_position(x_stage)
        follower_y_pos = ctx.part_world_position(y_stage)
        ctx.expect_contact(
            x_stage,
            base,
            contact_tol=0.0003,
            elem_a=x_bearings,
            elem_b=base_x_guides,
            name="x carriage stays in contact at full x travel",
        )
        ctx.expect_overlap(
            x_stage,
            base,
            axes="x",
            min_overlap=0.14,
            elem_a=x_bearings,
            elem_b=base_x_guides,
            name="x carriage stays engaged at full x travel",
        )

    with ctx.pose({x_to_y: Y_TRAVEL}):
        extended_y_pos = ctx.part_world_position(y_stage)

    with ctx.pose({base_to_x: X_TRAVEL, x_to_y: Y_TRAVEL}):
        ctx.expect_contact(
            y_stage,
            x_stage,
            contact_tol=0.0003,
            elem_a=y_bearings,
            elem_b=x_y_guides,
            name="y table stays in contact at combined travel",
        )
        ctx.expect_overlap(
            y_stage,
            x_stage,
            axes="y",
            min_overlap=0.10,
            elem_a=y_bearings,
            elem_b=x_y_guides,
            name="y table stays engaged at combined travel",
        )

    ctx.check(
        "x stage extends along +X only",
        rest_x_pos is not None
        and extended_x_pos is not None
        and follower_y_pos is not None
        and rest_y_pos is not None
        and extended_x_pos[0] > rest_x_pos[0] + 0.05
        and abs(extended_x_pos[1] - rest_x_pos[1]) < 0.002
        and abs(follower_y_pos[1] - rest_y_pos[1]) < 0.002,
        details=f"rest_x={rest_x_pos}, extended_x={extended_x_pos}, y_follow={follower_y_pos}",
    )
    ctx.check(
        "y stage extends along +Y only",
        rest_y_pos is not None
        and extended_y_pos is not None
        and extended_y_pos[1] > rest_y_pos[1] + 0.02
        and abs(extended_y_pos[0] - rest_y_pos[0]) < 0.002,
        details=f"rest_y={rest_y_pos}, extended_y={extended_y_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
