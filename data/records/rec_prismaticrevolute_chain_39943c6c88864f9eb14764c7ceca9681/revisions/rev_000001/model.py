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


RAIL_LENGTH = 0.42
RAIL_BASE_WIDTH = 0.10
RAIL_BASE_HEIGHT = 0.016
GUIDE_LENGTH = 0.38
GUIDE_WIDTH = 0.074
GUIDE_HEIGHT = 0.016
GUIDE_CHAMFER = 0.006
RAIL_TOP_Z = RAIL_BASE_HEIGHT + GUIDE_HEIGHT

STOP_LENGTH = 0.016
STOP_WIDTH = 0.086
STOP_HEIGHT = 0.018
STOP_CENTER_X = (RAIL_LENGTH - STOP_LENGTH) / 2.0

CARRIAGE_LENGTH = 0.11
CARRIAGE_WIDTH = 0.132
PAD_LENGTH = 0.086
PAD_WIDTH = 0.014
PAD_HEIGHT = 0.010
PAD_OFFSET_Y = 0.018
SKIRT_LENGTH = 0.096
SKIRT_THICKNESS = 0.012
SKIRT_HEIGHT = 0.018
DECK_HEIGHT = 0.018
DECK_TOP_Z = PAD_HEIGHT + DECK_HEIGHT
BRACKET_LENGTH = 0.052
BRACKET_WIDTH = 0.064
BRACKET_HEIGHT = 0.014
CHEEK_LENGTH = 0.024
CHEEK_THICKNESS = 0.008
CHEEK_HEIGHT = 0.034
CHEEK_OUTER_Y = 0.022
PIVOT_X = 0.044
PIVOT_Z = 0.064
SHAFT_RADIUS = 0.0065
SHAFT_LENGTH = 0.052
CAP_RADIUS = 0.011
CAP_PROTRUSION = 0.004

ARM_HUB_RADIUS = 0.017
ARM_HUB_WIDTH = 0.036
ARM_THICKNESS = 0.014
PAD_FACE_THICKNESS = 0.012
PAD_FACE_SIZE = 0.032
PAD_FACE_CENTER_X = 0.108
ARM_Y_OFFSET = 0.0
PIVOT_FACE_Y = CARRIAGE_WIDTH / 2.0
PIVOT_AXIS_Y = PIVOT_FACE_Y + ARM_HUB_WIDTH / 2.0

SLIDE_TRAVEL = 0.105
ARM_LOWER = -0.25
ARM_UPPER = 1.05


def _rail_body_shape() -> cq.Workplane:
    base = cq.Workplane("XY").box(
        RAIL_LENGTH,
        RAIL_BASE_WIDTH,
        RAIL_BASE_HEIGHT,
        centered=(True, True, False),
    )
    guide = (
        cq.Workplane("XY")
        .box(
            GUIDE_LENGTH,
            GUIDE_WIDTH,
            GUIDE_HEIGHT,
            centered=(True, True, False),
        )
        .edges("|X and >Z")
        .chamfer(GUIDE_CHAMFER)
        .translate((0.0, 0.0, RAIL_BASE_HEIGHT))
    )
    rail = base.union(guide)
    return rail.edges("|Z").fillet(0.0025)


def _stop_shape(sign: float) -> cq.Workplane:
    stop = (
        cq.Workplane("XY")
        .box(
            STOP_LENGTH,
            STOP_WIDTH,
            STOP_HEIGHT,
            centered=(True, True, False),
        )
        .translate((sign * STOP_CENTER_X, 0.0, RAIL_BASE_HEIGHT))
        .edges("|Z")
        .fillet(0.0018)
        .edges("|X and >Z")
        .chamfer(0.0015)
    )
    return stop


def _y_cylinder(radius: float, length: float, center_xyz: tuple[float, float, float]) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(radius)
        .extrude(length)
        .translate((0.0, 0.0, -length / 2.0))
        .rotate((0.0, 0.0, 0.0), (1.0, 0.0, 0.0), 90.0)
        .translate(center_xyz)
    )


def _carriage_shape() -> cq.Workplane:
    deck = (
        cq.Workplane("XY")
        .box(CARRIAGE_LENGTH, CARRIAGE_WIDTH, DECK_HEIGHT, centered=(True, True, True))
        .translate((0.0, 0.0, PAD_HEIGHT + DECK_HEIGHT / 2.0))
    )
    left_pad = (
        cq.Workplane("XY")
        .box(PAD_LENGTH, PAD_WIDTH, PAD_HEIGHT, centered=(True, True, True))
        .translate((0.0, PAD_OFFSET_Y, PAD_HEIGHT / 2.0))
    )
    right_pad = left_pad.mirror("XZ")
    left_skirt = (
        cq.Workplane("XY")
        .box(SKIRT_LENGTH, SKIRT_THICKNESS, SKIRT_HEIGHT, centered=(True, True, True))
        .translate((0.0, 0.054, SKIRT_HEIGHT / 2.0))
    )
    right_skirt = left_skirt.mirror("XZ")

    side_mount = (
        cq.Workplane("XY")
        .box(0.026, 0.020, 0.028, centered=(True, True, True))
        .translate((PIVOT_X - 0.010, PIVOT_FACE_Y - 0.010, DECK_TOP_Z + 0.014))
    )
    side_ear = (
        cq.Workplane("XY")
        .box(CHEEK_LENGTH, CHEEK_THICKNESS, CHEEK_HEIGHT, centered=(True, True, True))
        .translate((PIVOT_X, PIVOT_FACE_Y - CHEEK_THICKNESS / 2.0, PIVOT_Z))
    )
    rear_rib = (
        cq.Workplane("XY")
        .box(0.028, 0.010, 0.022, centered=(True, True, True))
        .translate((PIVOT_X - 0.022, PIVOT_FACE_Y - 0.011, DECK_TOP_Z + 0.011))
    )
    upper_bridge = (
        cq.Workplane("XY")
        .box(0.018, 0.020, 0.012, centered=(True, True, True))
        .translate((PIVOT_X - 0.010, PIVOT_FACE_Y - 0.010, PIVOT_Z + 0.016))
    )

    carriage = (
        deck.union(left_pad)
        .union(right_pad)
        .union(left_skirt)
        .union(right_skirt)
        .union(side_mount)
        .union(side_ear)
        .union(rear_rib)
        .union(upper_bridge)
    )
    return carriage


def _pivot_pin_shape() -> cq.Workplane:
    return _y_cylinder(SHAFT_RADIUS, SHAFT_LENGTH, (PIVOT_X, PIVOT_AXIS_Y, PIVOT_Z))


def _pivot_cap_shape() -> cq.Workplane:
    return _y_cylinder(
        CAP_RADIUS,
        CAP_PROTRUSION,
        (
            PIVOT_X,
            PIVOT_AXIS_Y + SHAFT_LENGTH / 2.0 + CAP_PROTRUSION / 2.0,
            PIVOT_Z,
        ),
    )


def _arm_hub_shape() -> cq.Workplane:
    return _y_cylinder(ARM_HUB_RADIUS, ARM_HUB_WIDTH, (0.0, 0.0, 0.0))


def _arm_link_shape() -> cq.Workplane:
    root_block = (
        cq.Workplane("XY")
        .box(0.020, ARM_THICKNESS, 0.020, centered=(True, True, True))
        .translate((0.020, 0.0, 0.0))
    )
    beam = (
        cq.Workplane("XY")
        .box(0.070, ARM_THICKNESS, 0.014, centered=(True, True, True))
        .translate((0.062, 0.0, 0.0))
    )
    pad_neck = (
        cq.Workplane("XY")
        .box(0.018, ARM_THICKNESS, 0.020, centered=(True, True, True))
        .translate((0.095, 0.0, 0.0))
    )
    pad = (
        cq.Workplane("XY")
        .box(PAD_FACE_THICKNESS, PAD_FACE_SIZE, PAD_FACE_SIZE, centered=(True, True, True))
        .translate((PAD_FACE_CENTER_X, 0.0, 0.0))
    )
    return root_block.union(beam).union(pad_neck).union(pad)


def _arm_shape() -> cq.Workplane:
    hub = _arm_hub_shape()
    bore = _y_cylinder(SHAFT_RADIUS + 0.0008, ARM_HUB_WIDTH + 0.010, (0.0, 0.0, 0.0))
    hub = hub.cut(bore)
    link = _arm_link_shape()
    return hub.union(link)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="linear_slide_output_arm")

    rail_metal = model.material("rail_metal", color=(0.55, 0.58, 0.60))
    carriage_metal = model.material("carriage_metal", color=(0.22, 0.24, 0.26))
    arm_metal = model.material("arm_metal", color=(0.36, 0.42, 0.48))

    rail = model.part("rail")
    rail.visual(
        mesh_from_cadquery(_rail_body_shape(), "rail_body"),
        material=rail_metal,
        name="rail_body",
    )
    rail.visual(
        mesh_from_cadquery(_stop_shape(-1.0), "left_stop"),
        material=rail_metal,
        name="left_stop",
    )
    rail.visual(
        mesh_from_cadquery(_stop_shape(1.0), "right_stop"),
        material=rail_metal,
        name="right_stop",
    )
    rail.inertial = Inertial.from_geometry(
        Box((RAIL_LENGTH, RAIL_BASE_WIDTH, RAIL_TOP_Z + STOP_HEIGHT)),
        mass=6.5,
        origin=Origin(xyz=(0.0, 0.0, (RAIL_TOP_Z + STOP_HEIGHT) / 2.0)),
    )

    carriage = model.part("carriage")
    carriage.visual(
        mesh_from_cadquery(_carriage_shape(), "carriage_body"),
        material=carriage_metal,
        name="carriage_body",
    )
    carriage.visual(
        mesh_from_cadquery(_pivot_pin_shape(), "pivot_pin"),
        material=rail_metal,
        name="pivot_pin",
    )
    carriage.visual(
        mesh_from_cadquery(_pivot_cap_shape(), "pivot_cap"),
        material=rail_metal,
        name="pivot_cap",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((CARRIAGE_LENGTH, CARRIAGE_WIDTH + 0.05, PIVOT_Z + ARM_HUB_RADIUS)),
        mass=1.6,
        origin=Origin(xyz=(0.0, 0.01, (PIVOT_Z + ARM_HUB_RADIUS) / 2.0)),
    )

    arm = model.part("arm")
    arm.visual(
        mesh_from_cadquery(_arm_shape(), "arm_body"),
        material=arm_metal,
        name="arm_body",
    )
    arm.inertial = Inertial.from_geometry(
        Box((PAD_FACE_CENTER_X + PAD_FACE_THICKNESS / 2.0, PAD_FACE_SIZE, 0.05)),
        mass=0.65,
        origin=Origin(
            xyz=(
                (PAD_FACE_CENTER_X + PAD_FACE_THICKNESS / 2.0) / 2.0,
                0.0,
                0.0,
            )
        ),
    )

    model.articulation(
        "rail_to_carriage",
        ArticulationType.PRISMATIC,
        parent=rail,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, RAIL_TOP_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=-SLIDE_TRAVEL,
            upper=SLIDE_TRAVEL,
            effort=320.0,
            velocity=0.25,
        ),
    )
    model.articulation(
        "carriage_to_arm",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=arm,
        origin=Origin(xyz=(PIVOT_X, PIVOT_AXIS_Y, PIVOT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            lower=ARM_LOWER,
            upper=ARM_UPPER,
            effort=90.0,
            velocity=2.5,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    rail = object_model.get_part("rail")
    carriage = object_model.get_part("carriage")
    arm = object_model.get_part("arm")
    slide = object_model.get_articulation("rail_to_carriage")
    pivot = object_model.get_articulation("carriage_to_arm")

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
        "slide_axis_aligned_to_rail",
        tuple(slide.axis) == (1.0, 0.0, 0.0),
        f"expected prismatic axis (1, 0, 0), got {slide.axis}",
    )
    ctx.check(
        "arm_pivot_axis_transverse",
        tuple(pivot.axis) == (0.0, 1.0, 0.0),
        f"expected revolute axis (0, 1, 0), got {pivot.axis}",
    )

    ctx.expect_contact(
        carriage,
        rail,
        name="carriage_has_supported_slide_contact",
    )
    ctx.expect_contact(
        arm,
        carriage,
        name="arm_is_rooted_to_carriage_pivot",
    )
    ctx.expect_gap(
        arm,
        rail,
        axis="z",
        min_gap=0.010,
        name="arm_clears_rail_in_rest_pose",
    )

    with ctx.pose({slide: -SLIDE_TRAVEL}):
        ctx.expect_contact(
            carriage,
            rail,
            name="carriage_stays_supported_at_left_travel_limit",
        )
        ctx.expect_gap(
            carriage,
            rail,
            axis="x",
            min_gap=0.012,
            negative_elem="left_stop",
            name="carriage_clears_left_stop",
        )

    with ctx.pose({slide: SLIDE_TRAVEL}):
        ctx.expect_contact(
            carriage,
            rail,
            name="carriage_stays_supported_at_right_travel_limit",
        )
        ctx.expect_gap(
            rail,
            carriage,
            axis="x",
            min_gap=0.012,
            positive_elem="right_stop",
            name="carriage_clears_right_stop",
        )

    with ctx.pose({pivot: ARM_LOWER}):
        ctx.expect_contact(
            arm,
            carriage,
            name="arm_remains_bearing_on_pivot_at_lower_limit",
        )
        ctx.expect_gap(
            arm,
            rail,
            axis="z",
            min_gap=0.008,
            name="arm_clears_rail_when_lowered",
        )

    with ctx.pose({pivot: ARM_UPPER}):
        ctx.expect_contact(
            arm,
            carriage,
            name="arm_remains_bearing_on_pivot_at_upper_limit",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
