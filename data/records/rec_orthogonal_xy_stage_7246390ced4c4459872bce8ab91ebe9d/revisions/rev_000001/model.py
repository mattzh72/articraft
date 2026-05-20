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


BASE_L = 0.36
BASE_W = 0.26
BASE_H = 0.032
BASE_POCKET_L = 0.336
BASE_POCKET_W = 0.236
BASE_POCKET_D = 0.008
BASE_POCKET_FLOOR_Z = BASE_H - BASE_POCKET_D
BASE_RAIL_L = 0.304
BASE_RAIL_W = 0.024
BASE_RAIL_H = 0.016
BASE_RAIL_Y = 0.076
BASE_GUIDE_TOP_Z = BASE_POCKET_FLOOR_Z + BASE_RAIL_H

FIRST_CARRIAGE_L = 0.19
FIRST_CARRIAGE_W = 0.22
FIRST_CARRIAGE_H = 0.05
FIRST_GROOVE_L = 0.194
FIRST_GROOVE_W = 0.03
FIRST_GROOVE_D = 0.018
FIRST_BEARING_PAD_L = 0.154
FIRST_BEARING_PAD_W = 0.018
FIRST_BEARING_PAD_H = 0.004
FIRST_TOP_POCKET_L = 0.158
FIRST_TOP_POCKET_W = 0.204
FIRST_TOP_POCKET_D = 0.006
FIRST_TOP_POCKET_FLOOR_Z = FIRST_CARRIAGE_H - FIRST_TOP_POCKET_D
FIRST_TOP_RAIL_L = 0.188
FIRST_TOP_RAIL_W = 0.02
FIRST_TOP_RAIL_H = 0.014
FIRST_TOP_RAIL_X = 0.04

SECOND_CARRIAGE_L = 0.14
SECOND_CARRIAGE_W = 0.14
SECOND_CARRIAGE_H = 0.042
SECOND_GROOVE_L = 0.144
SECOND_GROOVE_W = 0.026
SECOND_GROOVE_D = 0.016
SECOND_GROOVE_X = 0.04

DECK_S = 0.10
DECK_H = 0.016

X_TRAVEL = 0.065
Y_TRAVEL = 0.03


def make_base_shape() -> cq.Workplane:
    base = cq.Workplane("XY").box(BASE_L, BASE_W, BASE_H, centered=(True, True, False))
    base = (
        base.faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .rect(BASE_POCKET_L, BASE_POCKET_W)
        .cutBlind(-BASE_POCKET_D)
    )

    left_rail = (
        cq.Workplane("XY")
        .center(0.0, BASE_RAIL_Y)
        .box(BASE_RAIL_L, BASE_RAIL_W, BASE_RAIL_H, centered=(True, True, False))
        .translate((0.0, 0.0, BASE_POCKET_FLOOR_Z))
    )
    right_rail = (
        cq.Workplane("XY")
        .center(0.0, -BASE_RAIL_Y)
        .box(BASE_RAIL_L, BASE_RAIL_W, BASE_RAIL_H, centered=(True, True, False))
        .translate((0.0, 0.0, BASE_POCKET_FLOOR_Z))
    )

    base = base.union(left_rail).union(right_rail)
    return base


def make_first_carriage_shape() -> cq.Workplane:
    body = cq.Workplane("XY").box(
        FIRST_CARRIAGE_L,
        FIRST_CARRIAGE_W,
        FIRST_CARRIAGE_H,
        centered=(True, True, False),
    )

    for y_pos in (-BASE_RAIL_Y, BASE_RAIL_Y):
        groove = (
            cq.Workplane("XY")
            .center(0.0, y_pos)
            .box(FIRST_GROOVE_L, FIRST_GROOVE_W, FIRST_GROOVE_D, centered=(True, True, False))
        )
        body = body.cut(groove)

    for y_pos in (-BASE_RAIL_Y, BASE_RAIL_Y):
        bearing_pad = (
            cq.Workplane("XY")
            .center(0.0, y_pos)
            .box(
                FIRST_BEARING_PAD_L,
                FIRST_BEARING_PAD_W,
                FIRST_BEARING_PAD_H,
                centered=(True, True, False),
            )
        )
        body = body.union(bearing_pad)

    body = (
        body.faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .rect(FIRST_TOP_POCKET_L, FIRST_TOP_POCKET_W)
        .cutBlind(-FIRST_TOP_POCKET_D)
    )

    front_rail = (
        cq.Workplane("XY")
        .center(FIRST_TOP_RAIL_X, 0.0)
        .box(FIRST_TOP_RAIL_W, FIRST_TOP_RAIL_L, FIRST_TOP_RAIL_H, centered=(True, True, False))
        .translate((0.0, 0.0, FIRST_TOP_POCKET_FLOOR_Z))
    )
    rear_rail = (
        cq.Workplane("XY")
        .center(-FIRST_TOP_RAIL_X, 0.0)
        .box(FIRST_TOP_RAIL_W, FIRST_TOP_RAIL_L, FIRST_TOP_RAIL_H, centered=(True, True, False))
        .translate((0.0, 0.0, FIRST_TOP_POCKET_FLOOR_Z))
    )

    body = body.union(front_rail).union(rear_rail)

    body = (
        body.faces(">X")
        .workplane(centerOption="CenterOfMass")
        .rect(0.13, 0.02)
        .cutBlind(-0.01)
    )
    body = (
        body.faces("<X")
        .workplane(centerOption="CenterOfMass")
        .rect(0.13, 0.02)
        .cutBlind(-0.01)
    )

    return body


def make_second_carriage_shape() -> cq.Workplane:
    body = cq.Workplane("XY").box(
        SECOND_CARRIAGE_L,
        SECOND_CARRIAGE_W,
        SECOND_CARRIAGE_H,
        centered=(True, True, False),
    )

    for x_pos in (-SECOND_GROOVE_X, SECOND_GROOVE_X):
        groove = (
            cq.Workplane("XY")
            .center(x_pos, 0.0)
            .box(SECOND_GROOVE_W, SECOND_GROOVE_L, SECOND_GROOVE_D, centered=(True, True, False))
        )
        body = body.cut(groove)

    body = (
        body.faces(">Y")
        .workplane(centerOption="CenterOfMass")
        .rect(0.11, 0.018)
        .cutBlind(-0.008)
    )
    body = (
        body.faces("<Y")
        .workplane(centerOption="CenterOfMass")
        .rect(0.11, 0.018)
        .cutBlind(-0.008)
    )

    return body


def make_tooling_deck_shape() -> cq.Workplane:
    deck = cq.Workplane("XY").box(DECK_S, DECK_S, DECK_H, centered=(True, True, False))
    deck = (
        deck.faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .circle(0.012)
        .cutThruAll()
    )
    deck = (
        deck.faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .pushPoints(
            [
                (-0.03, -0.03),
                (-0.03, 0.03),
                (0.03, -0.03),
                (0.03, 0.03),
            ]
        )
        .hole(0.007)
    )
    return deck


def _axis_tuple(axis: tuple[float, float, float] | list[float]) -> tuple[float, float, float]:
    return tuple(float(value) for value in axis)


def _delta(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pick_and_place_stage")

    base_mat = model.material("base_blackened_steel", rgba=(0.20, 0.22, 0.24, 1.0))
    carriage_mat = model.material("carriage_aluminum", rgba=(0.74, 0.76, 0.78, 1.0))
    deck_mat = model.material("tool_deck_gunmetal", rgba=(0.40, 0.43, 0.47, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(make_base_shape(), "base"),
        material=base_mat,
        name="base_shell",
    )
    base.inertial = Inertial.from_geometry(
        Box((BASE_L, BASE_W, BASE_H + BASE_RAIL_H * 0.5)),
        mass=8.5,
        origin=Origin(xyz=(0.0, 0.0, (BASE_H + BASE_RAIL_H * 0.5) * 0.5)),
    )

    first = model.part("first_carriage")
    first.visual(
        mesh_from_cadquery(make_first_carriage_shape(), "first_carriage"),
        material=carriage_mat,
        name="first_carriage_shell",
    )
    first.inertial = Inertial.from_geometry(
        Box((FIRST_CARRIAGE_L, FIRST_CARRIAGE_W, FIRST_CARRIAGE_H + FIRST_TOP_RAIL_H * 0.5)),
        mass=3.2,
        origin=Origin(xyz=(0.0, 0.0, (FIRST_CARRIAGE_H + FIRST_TOP_RAIL_H * 0.5) * 0.5)),
    )

    second = model.part("second_carriage")
    second.visual(
        mesh_from_cadquery(make_second_carriage_shape(), "second_carriage"),
        material=carriage_mat,
        name="second_carriage_shell",
    )
    second.inertial = Inertial.from_geometry(
        Box((SECOND_CARRIAGE_L, SECOND_CARRIAGE_W, SECOND_CARRIAGE_H)),
        mass=1.9,
        origin=Origin(xyz=(0.0, 0.0, SECOND_CARRIAGE_H * 0.5)),
    )

    tooling_deck = model.part("tooling_deck")
    tooling_deck.visual(
        mesh_from_cadquery(make_tooling_deck_shape(), "tooling_deck"),
        material=deck_mat,
        name="tooling_deck_shell",
    )
    tooling_deck.inertial = Inertial.from_geometry(
        Box((DECK_S, DECK_S, DECK_H)),
        mass=0.8,
        origin=Origin(xyz=(0.0, 0.0, DECK_H * 0.5)),
    )

    model.articulation(
        "base_to_first_carriage",
        ArticulationType.PRISMATIC,
        parent=base,
        child=first,
        origin=Origin(xyz=(0.0, 0.0, BASE_GUIDE_TOP_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=0.30,
            lower=-X_TRAVEL,
            upper=X_TRAVEL,
        ),
    )

    model.articulation(
        "first_to_second_carriage",
        ArticulationType.PRISMATIC,
        parent=first,
        child=second,
        origin=Origin(xyz=(0.0, 0.0, FIRST_TOP_POCKET_FLOOR_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.25,
            lower=-Y_TRAVEL,
            upper=Y_TRAVEL,
        ),
    )

    model.articulation(
        "second_to_tooling_deck",
        ArticulationType.FIXED,
        parent=second,
        child=tooling_deck,
        origin=Origin(xyz=(0.0, 0.0, SECOND_CARRIAGE_H)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    first = object_model.get_part("first_carriage")
    second = object_model.get_part("second_carriage")
    tooling_deck = object_model.get_part("tooling_deck")
    x_slide = object_model.get_articulation("base_to_first_carriage")
    y_slide = object_model.get_articulation("first_to_second_carriage")

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

    ctx.expect_contact(first, base, name="first_carriage_contacts_base")
    ctx.expect_contact(second, first, name="second_carriage_contacts_first")
    ctx.expect_contact(tooling_deck, second, name="tooling_deck_contacts_second")
    ctx.expect_overlap(first, base, axes="xy", min_overlap=0.15, name="first_carriage_supported_footprint")
    ctx.expect_overlap(second, first, axes="xy", min_overlap=0.12, name="second_carriage_supported_footprint")
    ctx.expect_within(tooling_deck, second, axes="xy", margin=0.0, name="tooling_deck_within_second_carriage")

    x_axis = _axis_tuple(x_slide.axis)
    y_axis = _axis_tuple(y_slide.axis)
    x_limits = x_slide.motion_limits
    y_limits = y_slide.motion_limits

    ctx.check(
        "slides_are_prismatic",
        x_slide.articulation_type == ArticulationType.PRISMATIC
        and y_slide.articulation_type == ArticulationType.PRISMATIC,
        "Both user-facing mechanisms should be prismatic slides.",
    )
    ctx.check(
        "slide_axes_are_orthogonal_and_horizontal",
        x_axis == (1.0, 0.0, 0.0) and y_axis == (0.0, 1.0, 0.0),
        f"Expected horizontal orthogonal axes, got x={x_axis}, y={y_axis}.",
    )
    ctx.check(
        "slide_limits_span_both_directions",
        x_limits is not None
        and y_limits is not None
        and x_limits.lower is not None
        and x_limits.upper is not None
        and y_limits.lower is not None
        and y_limits.upper is not None
        and x_limits.lower < 0.0 < x_limits.upper
        and y_limits.lower < 0.0 < y_limits.upper,
        "Both prismatic slides should have realistic bidirectional travel about the centered pose.",
    )

    rest_first = ctx.part_world_position(first)
    with ctx.pose({x_slide: 0.04}):
        shifted_first = ctx.part_world_position(first)
    if rest_first is not None and shifted_first is not None:
        dx, dy, dz = _delta(shifted_first, rest_first)
        ctx.check(
            "x_slide_translates_only_along_x",
            abs(dx - 0.04) < 1e-9 and abs(dy) < 1e-9 and abs(dz) < 1e-9,
            f"Expected +0.04 m x-only motion, got delta {(dx, dy, dz)}.",
        )
    else:
        ctx.fail("x_slide_translates_only_along_x", "Could not query first carriage world positions.")

    rest_second = ctx.part_world_position(second)
    with ctx.pose({y_slide: -0.02}):
        shifted_second = ctx.part_world_position(second)
    if rest_second is not None and shifted_second is not None:
        dx, dy, dz = _delta(shifted_second, rest_second)
        ctx.check(
            "y_slide_translates_only_along_y",
            abs(dx) < 1e-9 and abs(dy + 0.02) < 1e-9 and abs(dz) < 1e-9,
            f"Expected -0.02 m y-only motion, got delta {(dx, dy, dz)}.",
        )
    else:
        ctx.fail("y_slide_translates_only_along_y", "Could not query second carriage world positions.")

    with ctx.pose({x_slide: X_TRAVEL}):
        ctx.expect_overlap(first, base, axes="xy", min_overlap=0.12, name="first_carriage_supported_at_x_max")

    with ctx.pose({y_slide: -Y_TRAVEL}):
        ctx.expect_overlap(second, first, axes="xy", min_overlap=0.10, name="second_carriage_supported_at_y_min")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
