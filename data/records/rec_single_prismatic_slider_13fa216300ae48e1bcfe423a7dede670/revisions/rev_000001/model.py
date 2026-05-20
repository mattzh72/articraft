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


BACKPLATE_W = 0.240
BACKPLATE_H = 0.720
BACKPLATE_T = 0.012
BACKPLATE_CORNER_R = 0.018
BACKPLATE_HOLE_R = 0.0065
BACKPLATE_HOLE_X = (BACKPLATE_W / 2.0) - 0.035
BACKPLATE_HOLE_Z = (BACKPLATE_H / 2.0) - 0.075

GUIDEWAY_L = 0.620
GUIDEWAY_BODY_W = 0.055
GUIDEWAY_BODY_D = 0.014
GUIDEWAY_RIB_W = 0.028
GUIDEWAY_RIB_D = 0.008
GUIDEWAY_STEM_W = 0.020
GUIDEWAY_STEM_D = 0.007
GUIDEWAY_HOLE_R = 0.006
GUIDEWAY_HOLE_COUNT = 6
GUIDEWAY_TOTAL_D = GUIDEWAY_STEM_D + GUIDEWAY_BODY_D + GUIDEWAY_RIB_D
GUIDEWAY_CENTER_Y = (BACKPLATE_T / 2.0) + (GUIDEWAY_TOTAL_D / 2.0)
GUIDEWAY_STEM_CENTER_Y_LOCAL = (-GUIDEWAY_TOTAL_D / 2.0) + (GUIDEWAY_STEM_D / 2.0)
GUIDEWAY_BODY_CENTER_Y_LOCAL = (
    (-GUIDEWAY_TOTAL_D / 2.0) + GUIDEWAY_STEM_D + (GUIDEWAY_BODY_D / 2.0)
)
GUIDEWAY_RIB_CENTER_Y_LOCAL = (GUIDEWAY_TOTAL_D / 2.0) - (GUIDEWAY_RIB_D / 2.0)

CARRIAGE_W = 0.110
CARRIAGE_D = 0.042
CARRIAGE_L = 0.125
CARRIAGE_REAR_Y = (BACKPLATE_T / 2.0) + GUIDEWAY_STEM_D
CARRIAGE_CENTER_Y = CARRIAGE_REAR_Y + (CARRIAGE_D / 2.0)
CARRIAGE_CHANNEL_CLEAR_W = GUIDEWAY_BODY_W + 0.012
CARRIAGE_SIDE_SHOE_W = (CARRIAGE_W - CARRIAGE_CHANNEL_CLEAR_W) / 2.0
CARRIAGE_SIDE_SHOE_L = 0.105
CARRIAGE_FRONT_BRIDGE_D = 0.014
CARRIAGE_FRONT_BRIDGE_CENTER_Y_LOCAL = (CARRIAGE_D / 2.0) - (CARRIAGE_FRONT_BRIDGE_D / 2.0)
CARRIAGE_SIDE_SHOE_CENTER_X = (CARRIAGE_CHANNEL_CLEAR_W / 2.0) + (CARRIAGE_SIDE_SHOE_W / 2.0)
CARRIAGE_PAD_W = (CARRIAGE_CHANNEL_CLEAR_W - GUIDEWAY_BODY_W) / 2.0
CARRIAGE_PAD_D = 0.014
CARRIAGE_PAD_L = 0.092
CARRIAGE_PAD_CENTER_Y_LOCAL = (-CARRIAGE_D / 2.0) + (CARRIAGE_PAD_D / 2.0) + 0.007
CARRIAGE_PAD_CENTER_X = (GUIDEWAY_BODY_W / 2.0) + (CARRIAGE_PAD_W / 2.0)
CARRIAGE_FRONT_FILLET = 0.004
CARRIAGE_FRONT_POCKET_R = 0.0065
CARRIAGE_FRONT_POCKET_D = 0.004

SLIDE_TRAVEL = 0.380
SLIDE_LOWER = -SLIDE_TRAVEL / 2.0
SLIDE_UPPER = SLIDE_TRAVEL / 2.0


def _evenly_spaced_positions(count: int, half_span: float) -> list[float]:
    if count <= 1:
        return [0.0]
    step = (2.0 * half_span) / float(count - 1)
    return [(-half_span + (i * step)) for i in range(count)]


def _build_backplate_shape() -> cq.Workplane:
    plate = cq.Workplane("XY").box(BACKPLATE_W, BACKPLATE_T, BACKPLATE_H)
    plate = plate.edges("|Y").fillet(BACKPLATE_CORNER_R)

    wall_holes = cq.Workplane("XZ").pushPoints(
        [
            (-BACKPLATE_HOLE_X, -BACKPLATE_HOLE_Z),
            (-BACKPLATE_HOLE_X, BACKPLATE_HOLE_Z),
            (BACKPLATE_HOLE_X, -BACKPLATE_HOLE_Z),
            (BACKPLATE_HOLE_X, BACKPLATE_HOLE_Z),
        ]
    ).circle(BACKPLATE_HOLE_R).extrude(BACKPLATE_T, both=True)
    plate = plate.cut(wall_holes)

    return plate


def _build_guideway_shape() -> cq.Workplane:
    stem = cq.Workplane("XY").box(GUIDEWAY_STEM_W, GUIDEWAY_STEM_D, GUIDEWAY_L)
    stem = stem.translate((0.0, GUIDEWAY_STEM_CENTER_Y_LOCAL, 0.0))

    body = cq.Workplane("XY").box(GUIDEWAY_BODY_W, GUIDEWAY_BODY_D, GUIDEWAY_L)
    body = body.translate((0.0, GUIDEWAY_BODY_CENTER_Y_LOCAL, 0.0))

    rib = cq.Workplane("XY").box(GUIDEWAY_RIB_W, GUIDEWAY_RIB_D, GUIDEWAY_L)
    rib = rib.translate((0.0, GUIDEWAY_RIB_CENTER_Y_LOCAL, 0.0))

    rail = stem.union(body).union(rib)
    rail = rail.edges("|Z").fillet(0.0015)

    rail_hole_half_span = (GUIDEWAY_L / 2.0) - 0.085
    rail_holes = cq.Workplane("XZ").pushPoints(
        [(0.0, z_pos) for z_pos in _evenly_spaced_positions(GUIDEWAY_HOLE_COUNT, rail_hole_half_span)]
    ).circle(GUIDEWAY_HOLE_R).extrude(GUIDEWAY_TOTAL_D, both=True)
    rail = rail.cut(rail_holes)

    return rail


def _build_carriage_shape() -> cq.Workplane:
    front_bridge = cq.Workplane("XY").box(CARRIAGE_W, CARRIAGE_FRONT_BRIDGE_D, CARRIAGE_L)
    front_bridge = front_bridge.translate((0.0, CARRIAGE_FRONT_BRIDGE_CENTER_Y_LOCAL, 0.0))

    left_shoe = cq.Workplane("XY").box(CARRIAGE_SIDE_SHOE_W, CARRIAGE_D, CARRIAGE_SIDE_SHOE_L)
    left_shoe = left_shoe.translate((-CARRIAGE_SIDE_SHOE_CENTER_X, 0.0, 0.0))

    right_shoe = cq.Workplane("XY").box(CARRIAGE_SIDE_SHOE_W, CARRIAGE_D, CARRIAGE_SIDE_SHOE_L)
    right_shoe = right_shoe.translate((CARRIAGE_SIDE_SHOE_CENTER_X, 0.0, 0.0))

    left_pad = cq.Workplane("XY").box(CARRIAGE_PAD_W, CARRIAGE_PAD_D, CARRIAGE_PAD_L)
    left_pad = left_pad.translate(
        (-CARRIAGE_PAD_CENTER_X, CARRIAGE_PAD_CENTER_Y_LOCAL, 0.0)
    )

    right_pad = cq.Workplane("XY").box(CARRIAGE_PAD_W, CARRIAGE_PAD_D, CARRIAGE_PAD_L)
    right_pad = right_pad.translate(
        (CARRIAGE_PAD_CENTER_X, CARRIAGE_PAD_CENTER_Y_LOCAL, 0.0)
    )

    carriage = front_bridge.union(left_shoe).union(right_shoe).union(left_pad).union(right_pad)
    carriage = carriage.edges("|Y").fillet(CARRIAGE_FRONT_FILLET)

    carriage = (
        carriage.faces(">Y")
        .workplane(centerOption="CenterOfMass")
        .pushPoints(
            [
                (-0.030, -0.030),
                (-0.030, 0.030),
                (0.030, -0.030),
                (0.030, 0.030),
            ]
        )
        .circle(CARRIAGE_FRONT_POCKET_R)
        .cutBlind(-CARRIAGE_FRONT_POCKET_D)
    )

    return carriage


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_backed_carriage_slide")

    model.material("plate_graphite", rgba=(0.28, 0.31, 0.35, 1.0))
    model.material("guideway_steel", rgba=(0.72, 0.74, 0.77, 1.0))
    model.material("carriage_red", rgba=(0.71, 0.20, 0.15, 1.0))

    backplate = model.part("backplate")
    backplate.visual(
        mesh_from_cadquery(_build_backplate_shape(), "backplate"),
        name="backplate_shell",
        material="plate_graphite",
    )
    backplate.inertial = Inertial.from_geometry(
        Box((BACKPLATE_W, BACKPLATE_T, BACKPLATE_H)),
        mass=7.5,
        origin=Origin(),
    )

    guideway = model.part("guideway")
    guideway.visual(
        mesh_from_cadquery(_build_guideway_shape(), "guideway"),
        name="guideway_shell",
        material="guideway_steel",
    )
    guideway.inertial = Inertial.from_geometry(
        Box((GUIDEWAY_BODY_W, GUIDEWAY_TOTAL_D, GUIDEWAY_L)),
        mass=3.1,
        origin=Origin(),
    )

    carriage = model.part("carriage")
    carriage.visual(
        mesh_from_cadquery(_build_carriage_shape(), "carriage"),
        name="carriage_shell",
        material="carriage_red",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((CARRIAGE_W, CARRIAGE_D, CARRIAGE_L)),
        mass=2.2,
        origin=Origin(),
    )

    model.articulation(
        "backplate_to_guideway",
        ArticulationType.FIXED,
        parent=backplate,
        child=guideway,
        origin=Origin(xyz=(0.0, GUIDEWAY_CENTER_Y, 0.0)),
    )
    model.articulation(
        "guideway_to_carriage",
        ArticulationType.PRISMATIC,
        parent=guideway,
        child=carriage,
        origin=Origin(xyz=(0.0, CARRIAGE_CENTER_Y - GUIDEWAY_CENTER_Y, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=SLIDE_LOWER,
            upper=SLIDE_UPPER,
            effort=400.0,
            velocity=0.35,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    backplate = object_model.get_part("backplate")
    guideway = object_model.get_part("guideway")
    carriage = object_model.get_part("carriage")
    slide = object_model.get_articulation("guideway_to_carriage")

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

    ctx.expect_contact(
        guideway,
        backplate,
        name="guideway_contacts_backplate",
    )
    ctx.expect_origin_gap(
        guideway,
        backplate,
        axis="y",
        min_gap=0.020,
        max_gap=0.021,
        name="guideway_stands_off_from_backplate_origin",
    )
    ctx.expect_contact(
        carriage,
        guideway,
        name="carriage_contacts_guideway",
    )
    ctx.expect_gap(
        carriage,
        backplate,
        axis="y",
        min_gap=0.0065,
        max_gap=0.0075,
        name="carriage_clears_wall_plate",
    )
    ctx.expect_overlap(
        carriage,
        guideway,
        axes="xz",
        min_overlap=0.050,
        name="carriage_overlaps_guideway_footprint",
    )
    ctx.expect_origin_distance(
        carriage,
        guideway,
        axes="x",
        max_dist=0.001,
        name="carriage_is_centered_on_rail",
    )

    with ctx.pose({slide: SLIDE_LOWER}):
        ctx.expect_origin_gap(
            guideway,
            carriage,
            axis="z",
            min_gap=0.180,
            max_gap=0.200,
            name="lower_limit_moves_carriage_down_the_rail",
        )
        ctx.expect_gap(
            carriage,
            backplate,
            axis="y",
            min_gap=0.0065,
            max_gap=0.0075,
            name="lower_limit_keeps_wall_clearance",
        )

    with ctx.pose({slide: SLIDE_UPPER}):
        ctx.expect_origin_gap(
            carriage,
            guideway,
            axis="z",
            min_gap=0.180,
            max_gap=0.200,
            name="upper_limit_moves_carriage_up_the_rail",
        )
        ctx.expect_gap(
            carriage,
            backplate,
            axis="y",
            min_gap=0.0065,
            max_gap=0.0075,
            name="upper_limit_keeps_wall_clearance",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
