from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_X = 0.280
BASE_Y = 0.220
BASE_Z = 0.018

X_RAIL_L = 0.214
X_RAIL_W = 0.022
X_RAIL_H = 0.010
X_RAIL_Y = 0.060

X_CARRIAGE_X = 0.120
X_CARRIAGE_Y = 0.196
X_CARRIAGE_Z = 0.030

Y_RAIL_X = 0.036
Y_RAIL_L = 0.172
Y_RAIL_W = 0.018
Y_RAIL_H = 0.010

Y_CARRIAGE_X = 0.156
Y_CARRIAGE_Y = 0.094
Y_CARRIAGE_Z = 0.028

TOOL_DECK_X = 0.092
TOOL_DECK_Y = 0.092
TOOL_DECK_Z = 0.012

X_TRAVEL = 0.046
Y_TRAVEL = 0.040


def _block(size_x: float, size_y: float, size_z: float) -> cq.Workplane:
    return cq.Workplane("XY").box(size_x, size_y, size_z, centered=(True, True, False))


def _base_body() -> cq.Workplane:
    body = _block(BASE_X, BASE_Y, BASE_Z)
    body = body.faces(">Z").edges().chamfer(0.002)

    underside_relief = _block(BASE_X - 0.060, BASE_Y - 0.060, 0.010).translate((0.0, 0.0, 0.004))
    body = body.cut(underside_relief)

    for y_off in (-X_RAIL_Y, X_RAIL_Y):
        rail = _block(X_RAIL_L, X_RAIL_W, X_RAIL_H).translate((0.0, y_off, BASE_Z))
        body = body.union(rail)

    return body


def _x_carriage_body() -> cq.Workplane:
    body = _block(X_CARRIAGE_X, X_CARRIAGE_Y, X_CARRIAGE_Z)
    body = body.faces(">Z").edges().chamfer(0.0015)

    underside_relief = _block(X_CARRIAGE_X - 0.028, 0.074, 0.009)
    body = body.cut(underside_relief)

    top_rebate = _block(0.032, 0.076, 0.0015).translate((0.0, 0.0, X_CARRIAGE_Z - 0.0015))
    body = body.cut(top_rebate)

    for x_off in (-Y_RAIL_X, Y_RAIL_X):
        rail = _block(Y_RAIL_W, Y_RAIL_L, Y_RAIL_H).translate((x_off, 0.0, X_CARRIAGE_Z))
        body = body.union(rail)

    return body


def _y_carriage_body() -> cq.Workplane:
    body = _block(Y_CARRIAGE_X, Y_CARRIAGE_Y, Y_CARRIAGE_Z)
    body = body.faces(">Z").edges().chamfer(0.0015)

    underside_relief = _block(0.040, Y_CARRIAGE_Y - 0.022, 0.009)
    body = body.cut(underside_relief)

    deck_rebate = _block(0.060, 0.060, 0.0015).translate((0.0, 0.0, Y_CARRIAGE_Z - 0.0015))
    body = body.cut(deck_rebate)

    return body


def _tool_deck_body() -> cq.Workplane:
    deck = _block(TOOL_DECK_X, TOOL_DECK_Y, TOOL_DECK_Z)
    deck = deck.faces(">Z").edges().chamfer(0.0015)
    deck = deck.faces(">Z").workplane(centerOption="CenterOfMass").circle(0.015).cutBlind(-0.0025)
    return deck


def _x_stop_bracket(sign: float) -> cq.Workplane:
    mount = _block(0.006, 0.038, 0.016).translate((sign * 0.003, 0.0, 0.0))
    post = _block(0.010, 0.018, 0.014).translate((sign * 0.001, 0.0, 0.016))
    arm = _block(0.036, 0.012, 0.008).translate((-sign * 0.014, 0.0, 0.030))
    body = mount.union(post).union(arm)
    return body.faces(">Z").edges().chamfer(0.0008)


def _y_stop_bracket(sign: float) -> cq.Workplane:
    mount = _block(0.028, 0.006, 0.016).translate((0.0, sign * 0.003, 0.0))
    post = _block(0.016, 0.010, 0.014).translate((0.0, sign * 0.001, 0.016))
    arm = _block(0.014, 0.016, 0.008).translate((0.0, -sign * 0.002, 0.030))
    body = mount.union(post).union(arm)
    return body.faces(">Z").edges().chamfer(0.0008)


def _add_screw_heads(
    part,
    *,
    coords: list[tuple[float, float]],
    top_z: float,
    material,
    prefix: str,
    radius: float = 0.0045,
    height: float = 0.003,
) -> None:
    for index, (x_pos, y_pos) in enumerate(coords, start=1):
        part.visual(
            Cylinder(radius=radius, length=height),
            origin=Origin(xyz=(x_pos, y_pos, top_z + height / 2.0)),
            material=material,
            name=f"{prefix}_{index}",
        )


def _add_rail_caps(
    part,
    *,
    coords: list[tuple[float, float, float]],
    size: tuple[float, float, float],
    material,
    prefix: str,
) -> None:
    sx, sy, sz = size
    for index, (x_pos, y_pos, z_pos) in enumerate(coords, start=1):
        part.visual(
            Box((sx, sy, sz)),
            origin=Origin(xyz=(x_pos, y_pos, z_pos)),
            material=material,
            name=f"{prefix}_{index}",
        )


def _add_x_wipers(part, *, material) -> None:
    for index, (x_pos, y_pos) in enumerate(
        (
            (-X_CARRIAGE_X / 2.0 + 0.0015, -X_RAIL_Y),
            (-X_CARRIAGE_X / 2.0 + 0.0015, X_RAIL_Y),
            (X_CARRIAGE_X / 2.0 - 0.0015, -X_RAIL_Y),
            (X_CARRIAGE_X / 2.0 - 0.0015, X_RAIL_Y),
        ),
        start=1,
    ):
        part.visual(
            Box((0.003, 0.020, 0.010)),
            origin=Origin(xyz=(x_pos, y_pos, 0.005)),
            material=material,
            name=f"x_wiper_{index}",
        )


def _add_y_wipers(part, *, material) -> None:
    for index, (x_pos, y_pos) in enumerate(
        (
            (-Y_RAIL_X, -Y_CARRIAGE_Y / 2.0 + 0.0015),
            (Y_RAIL_X, -Y_CARRIAGE_Y / 2.0 + 0.0015),
            (-Y_RAIL_X, Y_CARRIAGE_Y / 2.0 - 0.0015),
            (Y_RAIL_X, Y_CARRIAGE_Y / 2.0 - 0.0015),
        ),
        start=1,
    ):
        part.visual(
            Box((0.020, 0.003, 0.010)),
            origin=Origin(xyz=(x_pos, y_pos, 0.005)),
            material=material,
            name=f"y_wiper_{index}",
        )


def _add_stop_visuals(base, x_carriage, *, cap_black, rubber) -> None:
    base.visual(
        Box((0.008, 0.038, 0.016)),
        origin=Origin(xyz=(-0.144, 0.0, 0.008)),
        material=cap_black,
        name="x_stop_neg_mount",
    )
    base.visual(
        Box((0.010, 0.018, 0.016)),
        origin=Origin(xyz=(-0.149, 0.0, 0.024)),
        material=cap_black,
        name="x_stop_neg_post",
    )
    base.visual(
        Box((0.036, 0.012, 0.008)),
        origin=Origin(xyz=(-0.1265, 0.0, 0.034)),
        material=cap_black,
        name="x_stop_neg_arm",
    )
    base.visual(
        Box((0.002, 0.016, 0.008)),
        origin=Origin(xyz=(-0.108, 0.0, 0.034)),
        material=rubber,
        name="x_stop_neg_bumper",
    )
    base.visual(
        Box((0.008, 0.038, 0.016)),
        origin=Origin(xyz=(0.144, 0.0, 0.008)),
        material=cap_black,
        name="x_stop_pos_mount",
    )
    base.visual(
        Box((0.010, 0.018, 0.016)),
        origin=Origin(xyz=(0.149, 0.0, 0.024)),
        material=cap_black,
        name="x_stop_pos_post",
    )
    base.visual(
        Box((0.036, 0.012, 0.008)),
        origin=Origin(xyz=(0.1265, 0.0, 0.034)),
        material=cap_black,
        name="x_stop_pos_arm",
    )
    base.visual(
        Box((0.002, 0.016, 0.008)),
        origin=Origin(xyz=(0.108, 0.0, 0.034)),
        material=rubber,
        name="x_stop_pos_bumper",
    )

    x_carriage.visual(
        Box((0.028, 0.008, 0.016)),
        origin=Origin(xyz=(0.0, -0.102, 0.008)),
        material=cap_black,
        name="y_stop_neg_mount",
    )
    x_carriage.visual(
        Box((0.016, 0.010, 0.024)),
        origin=Origin(xyz=(0.0, -0.107, 0.028)),
        material=cap_black,
        name="y_stop_neg_post",
    )
    x_carriage.visual(
        Box((0.014, 0.036, 0.008)),
        origin=Origin(xyz=(0.0, -0.106, 0.044)),
        material=cap_black,
        name="y_stop_neg_arm",
    )
    x_carriage.visual(
        Box((0.012, 0.002, 0.008)),
        origin=Origin(xyz=(0.0, -0.089, 0.044)),
        material=rubber,
        name="y_stop_neg_bumper",
    )
    x_carriage.visual(
        Box((0.028, 0.008, 0.016)),
        origin=Origin(xyz=(0.0, 0.102, 0.008)),
        material=cap_black,
        name="y_stop_pos_mount",
    )
    x_carriage.visual(
        Box((0.016, 0.010, 0.024)),
        origin=Origin(xyz=(0.0, 0.107, 0.028)),
        material=cap_black,
        name="y_stop_pos_post",
    )
    x_carriage.visual(
        Box((0.014, 0.036, 0.008)),
        origin=Origin(xyz=(0.0, 0.106, 0.044)),
        material=cap_black,
        name="y_stop_pos_arm",
    )
    x_carriage.visual(
        Box((0.012, 0.002, 0.008)),
        origin=Origin(xyz=(0.0, 0.089, 0.044)),
        material=rubber,
        name="y_stop_pos_bumper",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_pick_and_place_stage")

    anodized_dark = model.material("anodized_dark", rgba=(0.28, 0.30, 0.33, 1.0))
    anodized_light = model.material("anodized_light", rgba=(0.60, 0.63, 0.67, 1.0))
    steel = model.material("steel", rgba=(0.78, 0.80, 0.83, 1.0))
    cover_metal = model.material("cover_metal", rgba=(0.70, 0.72, 0.75, 1.0))
    cap_black = model.material("cap_black", rgba=(0.12, 0.13, 0.14, 1.0))
    rubber = model.material("rubber", rgba=(0.07, 0.07, 0.08, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_base_body(), "base_body"),
        material=anodized_dark,
        name="base_body",
    )
    base.visual(
        Box((0.050, 0.082, 0.0016)),
        origin=Origin(xyz=(0.0, 0.0, BASE_Z + 0.0008)),
        material=cover_metal,
        name="x_stage_cover_strip",
    )
    _add_rail_caps(
        base,
        coords=[
            (-X_RAIL_L / 2.0, -X_RAIL_Y, BASE_Z + X_RAIL_H / 2.0),
            (X_RAIL_L / 2.0, -X_RAIL_Y, BASE_Z + X_RAIL_H / 2.0),
            (-X_RAIL_L / 2.0, X_RAIL_Y, BASE_Z + X_RAIL_H / 2.0),
            (X_RAIL_L / 2.0, X_RAIL_Y, BASE_Z + X_RAIL_H / 2.0),
        ],
        size=(0.004, 0.024, 0.010),
        material=cap_black,
        prefix="x_rail_cap",
    )
    x_carriage = model.part("x_carriage")
    x_carriage.visual(
        mesh_from_cadquery(_x_carriage_body(), "x_carriage_body"),
        material=anodized_light,
        name="x_carriage_body",
    )
    x_carriage.visual(
        Box((0.030, 0.076, 0.0016)),
        origin=Origin(xyz=(0.0, 0.0, X_CARRIAGE_Z + 0.0008)),
        material=cover_metal,
        name="y_stage_cover_strip",
    )
    _add_rail_caps(
        x_carriage,
        coords=[
            (-Y_RAIL_X, -Y_RAIL_L / 2.0, X_CARRIAGE_Z + Y_RAIL_H / 2.0),
            (-Y_RAIL_X, Y_RAIL_L / 2.0, X_CARRIAGE_Z + Y_RAIL_H / 2.0),
            (Y_RAIL_X, -Y_RAIL_L / 2.0, X_CARRIAGE_Z + Y_RAIL_H / 2.0),
            (Y_RAIL_X, Y_RAIL_L / 2.0, X_CARRIAGE_Z + Y_RAIL_H / 2.0),
        ],
        size=(0.020, 0.004, 0.010),
        material=cap_black,
        prefix="y_rail_cap",
    )
    _add_x_wipers(x_carriage, material=rubber)
    _add_screw_heads(
        x_carriage,
        coords=[(-0.050, -0.076), (-0.050, 0.076), (0.050, -0.076), (0.050, 0.076)],
        top_z=X_CARRIAGE_Z,
        material=steel,
        prefix="x_carriage_screw",
    )
    _add_stop_visuals(base, x_carriage, cap_black=cap_black, rubber=rubber)

    y_carriage = model.part("y_carriage")
    y_carriage.visual(
        mesh_from_cadquery(_y_carriage_body(), "y_carriage_body"),
        material=anodized_light,
        name="y_carriage_body",
    )
    _add_y_wipers(y_carriage, material=rubber)
    _add_screw_heads(
        y_carriage,
        coords=[(-0.058, -0.024), (-0.058, 0.024), (0.058, -0.024), (0.058, 0.024)],
        top_z=Y_CARRIAGE_Z,
        material=steel,
        prefix="y_carriage_screw",
    )

    tool_deck = model.part("tool_deck")
    tool_deck.visual(
        mesh_from_cadquery(_tool_deck_body(), "tool_deck_body"),
        material=cover_metal,
        name="tool_deck_body",
    )
    _add_screw_heads(
        tool_deck,
        coords=[(-0.028, -0.028), (-0.028, 0.028), (0.028, -0.028), (0.028, 0.028)],
        top_z=TOOL_DECK_Z,
        material=steel,
        prefix="tool_deck_screw",
        radius=0.004,
        height=0.0025,
    )

    model.articulation(
        "base_to_x_carriage",
        ArticulationType.PRISMATIC,
        parent=base,
        child=x_carriage,
        origin=Origin(xyz=(0.0, 0.0, BASE_Z + X_RAIL_H)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=0.35,
            lower=-X_TRAVEL,
            upper=X_TRAVEL,
        ),
    )
    model.articulation(
        "x_carriage_to_y_carriage",
        ArticulationType.PRISMATIC,
        parent=x_carriage,
        child=y_carriage,
        origin=Origin(xyz=(0.0, 0.0, X_CARRIAGE_Z + Y_RAIL_H)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=140.0,
            velocity=0.35,
            lower=-Y_TRAVEL,
            upper=Y_TRAVEL,
        ),
    )
    model.articulation(
        "y_carriage_to_tool_deck",
        ArticulationType.FIXED,
        parent=y_carriage,
        child=tool_deck,
        origin=Origin(xyz=(0.0, 0.0, Y_CARRIAGE_Z)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
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

    base = object_model.get_part("base")
    x_carriage = object_model.get_part("x_carriage")
    y_carriage = object_model.get_part("y_carriage")
    tool_deck = object_model.get_part("tool_deck")
    x_slide = object_model.get_articulation("base_to_x_carriage")
    y_slide = object_model.get_articulation("x_carriage_to_y_carriage")

    x_limits = x_slide.motion_limits
    y_limits = y_slide.motion_limits

    ctx.check(
        "x_slide_axis",
        tuple(x_slide.axis) == (1.0, 0.0, 0.0),
        details=f"expected x-axis prismatic motion, got {x_slide.axis}",
    )
    ctx.check(
        "y_slide_axis",
        tuple(y_slide.axis) == (0.0, 1.0, 0.0),
        details=f"expected y-axis prismatic motion, got {y_slide.axis}",
    )
    ctx.check(
        "x_slide_limits",
        x_limits is not None and x_limits.lower == -X_TRAVEL and x_limits.upper == X_TRAVEL,
        details=f"unexpected x limits: {x_limits}",
    )
    ctx.check(
        "y_slide_limits",
        y_limits is not None and y_limits.lower == -Y_TRAVEL and y_limits.upper == Y_TRAVEL,
        details=f"unexpected y limits: {y_limits}",
    )

    ctx.expect_contact(base, x_carriage, name="x_carriage_guides_contact_base")
    ctx.expect_contact(x_carriage, y_carriage, name="y_carriage_guides_contact_x_carriage")
    ctx.expect_contact(y_carriage, tool_deck, name="tool_deck_seated_on_y_carriage")

    ctx.expect_overlap(x_carriage, base, axes="xy", min_overlap=0.100, name="x_stage_has_guide_overlap")
    ctx.expect_overlap(y_carriage, x_carriage, axes="xy", min_overlap=0.085, name="y_stage_has_guide_overlap")
    ctx.expect_within(tool_deck, y_carriage, axes="xy", margin=0.0, name="tool_deck_within_y_carriage")

    with ctx.pose({x_slide: -X_TRAVEL}):
        ctx.expect_gap(
            x_carriage,
            base,
            axis="x",
            positive_elem="x_carriage_body",
            negative_elem="x_stop_neg_bumper",
            min_gap=0.0005,
            max_gap=0.0015,
            name="x_negative_stop_clearance",
        )
        ctx.expect_contact(base, x_carriage, name="x_carriage_stays_supported_at_negative_limit")
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_at_x_negative_limit")

    with ctx.pose({x_slide: X_TRAVEL}):
        ctx.expect_gap(
            base,
            x_carriage,
            axis="x",
            positive_elem="x_stop_pos_bumper",
            negative_elem="x_carriage_body",
            min_gap=0.0005,
            max_gap=0.0015,
            name="x_positive_stop_clearance",
        )
        ctx.expect_contact(base, x_carriage, name="x_carriage_stays_supported_at_positive_limit")
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_at_x_positive_limit")

    with ctx.pose({y_slide: -Y_TRAVEL}):
        ctx.expect_gap(
            y_carriage,
            x_carriage,
            axis="y",
            positive_elem="y_carriage_body",
            negative_elem="y_stop_neg_bumper",
            min_gap=0.0005,
            max_gap=0.0015,
            name="y_negative_stop_clearance",
        )
        ctx.expect_contact(x_carriage, y_carriage, name="y_carriage_stays_supported_at_negative_limit")
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_at_y_negative_limit")

    with ctx.pose({y_slide: Y_TRAVEL}):
        ctx.expect_gap(
            x_carriage,
            y_carriage,
            axis="y",
            positive_elem="y_stop_pos_bumper",
            negative_elem="y_carriage_body",
            min_gap=0.0005,
            max_gap=0.0015,
            name="y_positive_stop_clearance",
        )
        ctx.expect_contact(x_carriage, y_carriage, name="y_carriage_stays_supported_at_positive_limit")
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_at_y_positive_limit")

    with ctx.pose({x_slide: X_TRAVEL, y_slide: Y_TRAVEL}):
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_at_upper_corner_pose")

    with ctx.pose({x_slide: -X_TRAVEL, y_slide: -Y_TRAVEL}):
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_at_lower_corner_pose")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
