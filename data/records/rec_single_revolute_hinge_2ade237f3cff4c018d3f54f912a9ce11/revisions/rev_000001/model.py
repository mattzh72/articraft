from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BARREL_RADIUS = 0.008
CLEARANCE_RADIAL = 0.0
BRACKET_BORE_RADIUS = 0.0064
COVER_KNUCKLE_RADIUS = 0.0072
PIN_RADIUS = 0.0064
SIDE_KNUCKLE_LEN = 0.042
CENTER_KNUCKLE_LEN = 0.064
KNUCKLE_GAP = 0.003
BARREL_SPAN = SIDE_KNUCKLE_LEN * 2.0 + CENTER_KNUCKLE_LEN + KNUCKLE_GAP * 2.0

BRACKET_WIDTH = 0.140
BRACKET_HEIGHT = 0.075
BRACKET_THICKNESS = 0.006

SUPPORT_DEPTH = 0.014
SUPPORT_HEIGHT = 0.010
SUPPORT_Y = -0.011
SUPPORT_Z = -0.006

COVER_WIDTH = 0.180
COVER_HEIGHT = 0.140
COVER_THICKNESS = 0.0045
COVER_BACK_Y = 0.013
COVER_CENTER_Y = COVER_BACK_Y + COVER_THICKNESS / 2.0
COVER_TOP_DROP = 0.018
COVER_CENTER_Z = -(COVER_HEIGHT / 2.0 + COVER_TOP_DROP)

CONNECTOR_WIDTH = 0.064
LOWER_CONNECTOR_DEPTH = 0.008
LOWER_CONNECTOR_HEIGHT = 0.008
LOWER_CONNECTOR_CENTER_Y = 0.009
LOWER_CONNECTOR_CENTER_Z = -0.009
UPPER_CONNECTOR_DEPTH = 0.0045
UPPER_CONNECTOR_HEIGHT = 0.011
UPPER_CONNECTOR_CENTER_Y = 0.01525
UPPER_CONNECTOR_CENTER_Z = -0.0135

PLATE_FRONT_Y = -0.018
PLATE_CENTER_Y = PLATE_FRONT_Y - BRACKET_THICKNESS / 2.0
PLATE_CENTER_Z = -0.028


def _leaf_plate(
    width: float,
    height: float,
    thickness: float,
    corner_radius: float,
    hole_points: list[tuple[float, float]],
    hole_diameter: float,
) -> cq.Workplane:
    plate = cq.Workplane("XY").box(width, thickness, height)
    plate = plate.edges("|Y").fillet(corner_radius)
    if hole_points:
        plate = (
            plate.faces(">Y")
            .workplane(centerOption="CenterOfMass")
            .pushPoints(hole_points)
            .hole(hole_diameter)
        )
    return plate


def _knuckle_segment(start_x: float, length: float, radius: float) -> cq.Workplane:
    return cq.Workplane("YZ").circle(radius).extrude(length).translate((start_x, 0.0, 0.0))


def _knuckle_tube(start_x: float, length: float, outer_radius: float, inner_radius: float) -> cq.Workplane:
    return (
        cq.Workplane("YZ")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(length)
        .translate((start_x, 0.0, 0.0))
    )


def _pin_shape() -> cq.Workplane:
    return cq.Workplane("YZ").circle(PIN_RADIUS).extrude(BARREL_SPAN).translate((-BARREL_SPAN / 2.0, 0.0, 0.0))


def _rib(center_x: float, rib_width: float) -> cq.Workplane:
    profile = (
        cq.Workplane("YZ")
        .polyline(
            [
                (PLATE_FRONT_Y, -0.002),
                (PLATE_FRONT_Y, -0.052),
                (-0.004, -0.011),
            ]
        )
        .close()
    )
    return profile.extrude(rib_width).translate((center_x - rib_width / 2.0, 0.0, 0.0))


def _build_bracket_shape() -> cq.Workplane:
    plate = _leaf_plate(
        width=BRACKET_WIDTH,
        height=BRACKET_HEIGHT,
        thickness=BRACKET_THICKNESS,
        corner_radius=0.007,
        hole_points=[(-0.038, 0.015), (0.038, 0.015), (-0.038, -0.020), (0.038, -0.020)],
        hole_diameter=0.0065,
    ).translate((0.0, PLATE_CENTER_Y, PLATE_CENTER_Z))

    left_start = -(CENTER_KNUCKLE_LEN / 2.0 + KNUCKLE_GAP + SIDE_KNUCKLE_LEN)
    right_start = CENTER_KNUCKLE_LEN / 2.0 + KNUCKLE_GAP
    left_center = left_start + SIDE_KNUCKLE_LEN / 2.0
    right_center = right_start + SIDE_KNUCKLE_LEN / 2.0

    left_knuckle = _knuckle_tube(left_start, SIDE_KNUCKLE_LEN, BARREL_RADIUS, BRACKET_BORE_RADIUS)
    right_knuckle = _knuckle_tube(right_start, SIDE_KNUCKLE_LEN, BARREL_RADIUS, BRACKET_BORE_RADIUS)

    left_support = cq.Workplane("XY").box(SIDE_KNUCKLE_LEN, SUPPORT_DEPTH, SUPPORT_HEIGHT).translate(
        (left_center, SUPPORT_Y, SUPPORT_Z)
    )
    right_support = cq.Workplane("XY").box(SIDE_KNUCKLE_LEN, SUPPORT_DEPTH, SUPPORT_HEIGHT).translate(
        (right_center, SUPPORT_Y, SUPPORT_Z)
    )

    left_rib = _rib(left_center, 0.014)
    right_rib = _rib(right_center, 0.014)

    core = plate.union(left_support).union(right_support).union(left_rib).union(right_rib)
    knuckles = left_knuckle.union(right_knuckle)
    return core, knuckles


def _build_cover_shape() -> cq.Workplane:
    knuckle = _knuckle_tube(
        -CENTER_KNUCKLE_LEN / 2.0,
        CENTER_KNUCKLE_LEN,
        COVER_KNUCKLE_RADIUS,
        BRACKET_BORE_RADIUS,
    )

    lower_connector = cq.Workplane("XY").box(
        CONNECTOR_WIDTH,
        LOWER_CONNECTOR_DEPTH,
        LOWER_CONNECTOR_HEIGHT,
    ).translate(
        (0.0, LOWER_CONNECTOR_CENTER_Y, LOWER_CONNECTOR_CENTER_Z)
    )
    upper_connector = cq.Workplane("XY").box(
        CONNECTOR_WIDTH,
        UPPER_CONNECTOR_DEPTH,
        UPPER_CONNECTOR_HEIGHT,
    ).translate(
        (0.0, UPPER_CONNECTOR_CENTER_Y, UPPER_CONNECTOR_CENTER_Z)
    )

    panel = _leaf_plate(
        width=COVER_WIDTH,
        height=COVER_HEIGHT,
        thickness=COVER_THICKNESS,
        corner_radius=0.010,
        hole_points=[(-0.055, 0.030), (0.055, 0.030), (-0.055, -0.030), (0.055, -0.030)],
        hole_diameter=0.0055,
    ).translate((0.0, COVER_CENTER_Y, COVER_CENTER_Z))

    leaf = lower_connector.union(upper_connector).union(panel)
    return leaf, knuckle


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="gusseted_service_cover_hinge")

    bracket_mat = model.material("bracket_steel", rgba=(0.42, 0.44, 0.47, 1.0))
    cover_mat = model.material("cover_painted_steel", rgba=(0.73, 0.75, 0.78, 1.0))
    pin_mat = model.material("hinge_pin_dark", rgba=(0.26, 0.27, 0.29, 1.0))

    bracket_core_shape, bracket_knuckle_shape = _build_bracket_shape()
    cover_leaf_shape, cover_knuckle_shape = _build_cover_shape()
    pin_shape = _pin_shape()

    bracket = model.part("bracket")
    bracket.visual(
        mesh_from_cadquery(bracket_core_shape, "bracket_core"),
        origin=Origin(),
        material=bracket_mat,
        name="bracket_core",
    )
    bracket.visual(
        mesh_from_cadquery(bracket_knuckle_shape, "bracket_knuckles"),
        origin=Origin(),
        material=pin_mat,
        name="bracket_knuckles",
    )

    cover = model.part("cover")
    cover.visual(
        mesh_from_cadquery(cover_leaf_shape, "cover_leaf"),
        origin=Origin(),
        material=cover_mat,
        name="cover_leaf",
    )
    cover.visual(
        mesh_from_cadquery(cover_knuckle_shape, "cover_knuckle"),
        origin=Origin(),
        material=pin_mat,
        name="cover_knuckle",
    )

    pin = model.part("pin")
    pin.visual(
        mesh_from_cadquery(pin_shape, "hinge_pin"),
        origin=Origin(),
        material=pin_mat,
        name="hinge_pin",
    )

    model.articulation(
        "bracket_to_cover",
        ArticulationType.REVOLUTE,
        parent=bracket,
        child=cover,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=2.5,
            lower=0.0,
            upper=1.35,
        ),
    )
    model.articulation(
        "bracket_to_pin",
        ArticulationType.FIXED,
        parent=bracket,
        child=pin,
        origin=Origin(),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bracket = object_model.get_part("bracket")
    cover = object_model.get_part("cover")
    pin = object_model.get_part("pin")
    hinge = object_model.get_articulation("bracket_to_cover")
    bracket_core = bracket.get_visual("bracket_core")
    bracket_knuckles = bracket.get_visual("bracket_knuckles")
    cover_leaf = cover.get_visual("cover_leaf")
    cover_knuckle = cover.get_visual("cover_knuckle")
    hinge_pin = pin.get_visual("hinge_pin")

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
    ctx.allow_overlap(
        bracket,
        pin,
        elem_a=bracket_knuckles,
        elem_b=hinge_pin,
        reason="bracket leaf captures the fixed hinge pin inside its rolled barrel segments",
    )
    ctx.allow_overlap(
        cover,
        pin,
        elem_a=cover_knuckle,
        elem_b=hinge_pin,
        reason="cover leaf rotates around the hinge pin running through the center knuckle",
    )
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.expect_gap(
        cover,
        bracket,
        axis="y",
        min_gap=0.003,
        positive_elem=cover_leaf,
        negative_elem=bracket_core,
        name="cover_leaf_clears_bracket_core_when_closed",
    )
    ctx.expect_overlap(
        cover,
        bracket,
        axes="x",
        min_overlap=0.050,
        elem_a=cover_knuckle,
        elem_b=bracket_knuckles,
        name="knuckles_share_hinge_axis_span",
    )
    ctx.check(
        "hinge_axis_is_barrel_axis",
        tuple(round(v, 6) for v in hinge.axis) == (1.0, 0.0, 0.0),
        f"axis={hinge.axis}",
    )
    limits = hinge.motion_limits
    ctx.check(
        "hinge_limits_match_service_cover_motion",
        limits is not None and limits.lower == 0.0 and limits.upper is not None and 1.1 <= limits.upper <= 1.5,
        (
            "missing limits"
            if limits is None
            else f"lower={limits.lower}, upper={limits.upper}"
        ),
    )

    closed_aabb = ctx.part_world_aabb(cover)
    with ctx.pose({hinge: 1.15}):
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_with_cover_open")
        ctx.expect_gap(
            cover,
            bracket,
            axis="y",
            min_gap=0.002,
            positive_elem=cover_leaf,
            negative_elem=bracket_core,
            name="cover_leaf_stays_clear_of_bracket_core_when_open",
        )
        open_aabb = ctx.part_world_aabb(cover)

    moved_upward = False
    moved_forward = False
    details = "cover AABB unavailable"
    if closed_aabb is not None and open_aabb is not None:
        moved_upward = open_aabb[0][2] > closed_aabb[0][2] + 0.040
        moved_forward = open_aabb[1][1] > closed_aabb[1][1] + 0.040
        details = (
            f"closed min_z={closed_aabb[0][2]:.4f}, open min_z={open_aabb[0][2]:.4f}; "
            f"closed max_y={closed_aabb[1][1]:.4f}, open max_y={open_aabb[1][1]:.4f}"
        )
    ctx.check(
        "cover_opens_forward_and_upward",
        moved_upward and moved_forward,
        details,
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
