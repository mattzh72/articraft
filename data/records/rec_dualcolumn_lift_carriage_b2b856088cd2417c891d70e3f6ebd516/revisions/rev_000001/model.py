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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


FOOT_WIDTH = 0.82
FOOT_DEPTH = 0.34
FOOT_HEIGHT = 0.065
POST_BOSS_HEIGHT = 0.030

POST_RADIUS = 0.040
POST_SPACING = 0.420
POST_HEIGHT = 1.280

TIE_BAR_WIDTH = 0.620
TIE_BAR_DEPTH = 0.130
TIE_BAR_HEIGHT = 0.080

CARRIAGE_WIDTH = 0.660
CARRIAGE_DEPTH = 0.140
CARRIAGE_HEIGHT = 0.220
GUIDE_OUTER_RADIUS = 0.072
GUIDE_BORE_RADIUS = POST_RADIUS + 0.007
PAD_WIDTH = 0.022
PAD_DEPTH = 0.018
PAD_HEIGHT = 0.180

HOME_CARRIAGE_GAP = 0.010
LIFT_TRAVEL = 0.820

POST_POINTS = (
    (-POST_SPACING / 2.0, 0.0),
    (POST_SPACING / 2.0, 0.0),
)

FRAME_HEIGHT = FOOT_HEIGHT + POST_HEIGHT + TIE_BAR_HEIGHT
CARRIAGE_HOME_Z = FOOT_HEIGHT + POST_BOSS_HEIGHT + HOME_CARRIAGE_GAP + CARRIAGE_HEIGHT / 2.0


def _make_foot_shape() -> cq.Workplane:
    foot = (
        cq.Workplane("XY")
        .box(FOOT_WIDTH, FOOT_DEPTH, FOOT_HEIGHT, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.018)
        .faces(">Z")
        .edges()
        .fillet(0.006)
    )
    bosses = (
        cq.Workplane("XY")
        .pushPoints(POST_POINTS)
        .box(0.130, 0.160, POST_BOSS_HEIGHT, centered=(True, True, False))
        .edges("|Z")
        .fillet(0.010)
        .translate((0.0, 0.0, FOOT_HEIGHT))
    )
    return foot.union(bosses)


def _make_tie_bar_shape() -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(TIE_BAR_WIDTH, TIE_BAR_DEPTH, TIE_BAR_HEIGHT)
        .edges("|Z")
        .fillet(0.012)
    )


def _make_carriage_shape() -> cq.Workplane:
    backplate = cq.Workplane("XY").box(CARRIAGE_WIDTH, 0.050, CARRIAGE_HEIGHT)
    lower_plate = cq.Workplane("XY").box(
        CARRIAGE_WIDTH * 0.88,
        CARRIAGE_DEPTH,
        CARRIAGE_HEIGHT * 0.42,
    ).translate((0.0, 0.0, -0.040))
    upper_plate = cq.Workplane("XY").box(
        CARRIAGE_WIDTH * 0.64,
        CARRIAGE_DEPTH * 0.82,
        CARRIAGE_HEIGHT * 0.22,
    ).translate((0.0, 0.0, 0.060))
    guide_tubes = (
        cq.Workplane("XY")
        .pushPoints(POST_POINTS)
        .circle(GUIDE_OUTER_RADIUS)
        .extrude(CARRIAGE_HEIGHT / 2.0, both=True)
    )
    bores = (
        cq.Workplane("XY")
        .pushPoints(POST_POINTS)
        .circle(GUIDE_BORE_RADIUS)
        .extrude(CARRIAGE_HEIGHT / 2.0 + 0.010, both=True)
    )
    return backplate.union(lower_plate).union(upper_plate).union(guide_tubes).cut(bores)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_twin_post_lift")

    model.material("frame_gray", rgba=(0.28, 0.30, 0.33, 1.0))
    model.material("post_steel", rgba=(0.70, 0.73, 0.76, 1.0))
    model.material("carriage_orange", rgba=(0.88, 0.50, 0.12, 1.0))
    model.material("wear_pad", rgba=(0.54, 0.45, 0.18, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_make_foot_shape(), "lift_foot"),
        material="frame_gray",
        name="foot",
    )
    base.visual(
        Cylinder(radius=POST_RADIUS, length=POST_HEIGHT),
        origin=Origin(xyz=(-POST_SPACING / 2.0, 0.0, FOOT_HEIGHT + POST_HEIGHT / 2.0)),
        material="post_steel",
        name="left_post",
    )
    base.visual(
        Cylinder(radius=POST_RADIUS, length=POST_HEIGHT),
        origin=Origin(xyz=(POST_SPACING / 2.0, 0.0, FOOT_HEIGHT + POST_HEIGHT / 2.0)),
        material="post_steel",
        name="right_post",
    )
    base.visual(
        mesh_from_cadquery(_make_tie_bar_shape(), "lift_tie_bar"),
        origin=Origin(xyz=(0.0, 0.0, FOOT_HEIGHT + POST_HEIGHT + TIE_BAR_HEIGHT / 2.0)),
        material="frame_gray",
        name="tie_bar",
    )
    base.inertial = Inertial.from_geometry(
        Box((FOOT_WIDTH, FOOT_DEPTH, FRAME_HEIGHT)),
        mass=220.0,
        origin=Origin(xyz=(0.0, 0.0, FRAME_HEIGHT / 2.0)),
    )

    carriage = model.part("carriage")
    carriage.visual(
        mesh_from_cadquery(_make_carriage_shape(), "lift_carriage"),
        material="carriage_orange",
        name="carriage_body",
    )
    for side_name, x_pos in (("left", -POST_SPACING / 2.0), ("right", POST_SPACING / 2.0)):
        carriage.visual(
            Box((PAD_WIDTH, PAD_DEPTH, PAD_HEIGHT)),
            origin=Origin(xyz=(x_pos, POST_RADIUS + PAD_DEPTH / 2.0, 0.0)),
            material="wear_pad",
            name=f"{side_name}_front_pad",
        )
        carriage.visual(
            Box((PAD_WIDTH, PAD_DEPTH, PAD_HEIGHT)),
            origin=Origin(xyz=(x_pos, -(POST_RADIUS + PAD_DEPTH / 2.0), 0.0)),
            material="wear_pad",
            name=f"{side_name}_rear_pad",
        )
    carriage.inertial = Inertial.from_geometry(
        Box((CARRIAGE_WIDTH, CARRIAGE_DEPTH, CARRIAGE_HEIGHT)),
        mass=55.0,
        origin=Origin(),
    )

    model.articulation(
        "base_to_carriage",
        ArticulationType.PRISMATIC,
        parent=base,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, CARRIAGE_HOME_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=LIFT_TRAVEL,
            effort=18000.0,
            velocity=0.25,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    carriage = object_model.get_part("carriage")
    lift = object_model.get_articulation("base_to_carriage")

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

    ctx.check(
        "parts_present",
        base is not None and carriage is not None and lift is not None,
        "Expected base, carriage, and prismatic lift articulation.",
    )
    ctx.check(
        "lift_joint_is_vertical_prismatic",
        lift.articulation_type == ArticulationType.PRISMATIC and tuple(lift.axis) == (0.0, 0.0, 1.0),
        f"Expected a vertical prismatic joint, got type={lift.articulation_type} axis={lift.axis}.",
    )

    ctx.expect_gap(
        carriage,
        base,
        axis="z",
        positive_elem="carriage_body",
        negative_elem="foot",
        min_gap=0.008,
        max_gap=0.020,
        name="carriage_starts_just_above_foot",
    )
    ctx.expect_overlap(
        base,
        carriage,
        axes="yz",
        elem_a="left_post",
        elem_b="carriage_body",
        min_overlap=0.075,
        name="carriage_wraps_left_post",
    )
    ctx.expect_overlap(
        base,
        carriage,
        axes="yz",
        elem_a="right_post",
        elem_b="carriage_body",
        min_overlap=0.075,
        name="carriage_wraps_right_post",
    )
    ctx.expect_contact(
        base,
        carriage,
        elem_a="left_post",
        elem_b="left_front_pad",
        name="left_post_guided_by_carriage_pad",
    )
    ctx.expect_contact(
        base,
        carriage,
        elem_a="right_post",
        elem_b="right_front_pad",
        name="right_post_guided_by_carriage_pad",
    )

    z0 = ctx.part_world_position(carriage)
    with ctx.pose({lift: LIFT_TRAVEL}):
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_at_top_of_travel")
        ctx.expect_gap(
            base,
            carriage,
            axis="z",
            positive_elem="tie_bar",
            negative_elem="carriage_body",
            min_gap=0.150,
            name="raised_carriage_clears_tie_bar",
        )
        z1 = ctx.part_world_position(carriage)

    ctx.check(
        "positive_travel_lifts_carriage_upward",
        z0 is not None and z1 is not None and z1[2] > z0[2] + 0.75,
        f"Expected upward lift travel greater than 0.75 m, got z0={z0}, z1={z1}.",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
