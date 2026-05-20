from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq
from math import pi

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


BEAM_LENGTH = 0.90
BEAM_DEPTH = 0.08
BEAM_HEIGHT = 0.05

CARRIAGE_LENGTH = 0.14
CARRIAGE_DEPTH = 0.108
CARRIAGE_TOP_HEIGHT = 0.02
CARRIAGE_TOP_CENTER_Z = 0.033

ROLLER_AXIS_Y = 0.114
ROLLER_AXIS_Z = -0.078
ROLLER_SHAFT_RADIUS = 0.008
ROLLER_SHAFT_LENGTH = 0.100
ROLLER_DRUM_RADIUS = 0.024
ROLLER_DRUM_LENGTH = 0.086


def cylinder_x(length: float, radius: float) -> cq.Workplane:
    return cq.Workplane("YZ").circle(radius).extrude(length / 2.0, both=True)


def make_beam_shape() -> cq.Workplane:
    beam = cq.Workplane("XY").box(BEAM_LENGTH, BEAM_DEPTH, BEAM_HEIGHT)
    beam = beam.edges("|X").fillet(0.004)

    stop_x = BEAM_LENGTH / 2.0 - 0.055
    stop_z = -(BEAM_HEIGHT + 0.014) / 2.0
    stop = cq.Workplane("XY").box(0.026, 0.10, 0.014).translate((stop_x, 0.0, stop_z))
    stop = stop.union(
        cq.Workplane("XY").box(0.026, 0.10, 0.014).translate((-stop_x, 0.0, stop_z))
    )

    strap_height = 0.11
    strap_z = BEAM_HEIGHT / 2.0 + strap_height / 2.0
    strap = cq.Workplane("XY").box(0.045, 0.012, strap_height).translate((0.25, 0.0, strap_z))
    strap = strap.union(
        cq.Workplane("XY").box(0.045, 0.012, strap_height).translate((-0.25, 0.0, strap_z))
    )

    top_plate_z = BEAM_HEIGHT / 2.0 + strap_height + 0.006
    top_plate = cq.Workplane("XY").box(0.64, 0.10, 0.012).translate((0.0, 0.0, top_plate_z))
    top_plate = top_plate.edges("|X").fillet(0.003)

    return beam.union(stop).union(strap).union(top_plate)


def make_carriage_shape() -> cq.Workplane:
    crown = (
        cq.Workplane("XY")
        .box(CARRIAGE_LENGTH, 0.032, 0.018)
        .translate((0.0, 0.0, CARRIAGE_TOP_CENTER_Z))
    )

    side_plate_y = BEAM_DEPTH / 2.0 + 0.004
    side_plate = cq.Workplane("XY").box(CARRIAGE_LENGTH, 0.008, 0.116).translate(
        (0.0, side_plate_y, -0.033)
    )
    side_plate = side_plate.union(
        cq.Workplane("XY").box(CARRIAGE_LENGTH, 0.008, 0.116).translate((0.0, -side_plate_y, -0.033))
    )

    lower_body = cq.Workplane("XY").box(0.102, 0.036, 0.084).translate((0.0, 0.074, -0.078))

    connector = cq.Workplane("XY").box(0.022, 0.014, 0.068).translate((0.048, 0.053, -0.058))
    connector = connector.union(
        cq.Workplane("XY").box(0.022, 0.014, 0.068).translate((-0.048, 0.053, -0.058))
    )

    front_face = cq.Workplane("XY").box(0.110, 0.012, 0.082).translate((0.0, 0.090, -0.076))

    arm_offset_x = 0.064
    arm_link = cq.Workplane("XY").box(0.014, 0.018, 0.034).translate((arm_offset_x, 0.111, ROLLER_AXIS_Z))
    arm_link = arm_link.union(
        cq.Workplane("XY").box(0.014, 0.018, 0.034).translate((-arm_offset_x, 0.111, ROLLER_AXIS_Z))
    )
    arm_link = arm_link.union(
        cq.Workplane("XY").box(0.020, 0.016, 0.032).translate((0.050, 0.100, -0.078))
    )
    arm_link = arm_link.union(
        cq.Workplane("XY").box(0.020, 0.016, 0.032).translate((-0.050, 0.100, -0.078))
    )

    return crown.union(side_plate).union(lower_body).union(connector).union(front_face).union(arm_link)


def make_roller_drum_shape() -> cq.Workplane:
    drum = cylinder_x(ROLLER_DRUM_LENGTH, ROLLER_DRUM_RADIUS)
    return drum


def make_roller_core_shape() -> cq.Workplane:
    hub_offset = ROLLER_DRUM_LENGTH / 2.0 + 0.005
    hub = cylinder_x(0.010, 0.016).translate((hub_offset, 0.0, 0.0))
    hub = hub.union(cylinder_x(0.010, 0.016).translate((-hub_offset, 0.0, 0.0)))
    core = cylinder_x(ROLLER_SHAFT_LENGTH, ROLLER_SHAFT_RADIUS)
    return hub.union(core)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hanging_beam_slide")

    beam_steel = model.material("beam_steel", rgba=(0.24, 0.27, 0.30, 1.0))
    shuttle_paint = model.material("shuttle_paint", rgba=(0.90, 0.54, 0.18, 1.0))
    roller_black = model.material("roller_black", rgba=(0.13, 0.13, 0.14, 1.0))
    hardware_steel = model.material("hardware_steel", rgba=(0.73, 0.74, 0.76, 1.0))

    beam = model.part("beam")
    beam.visual(
        Box((BEAM_LENGTH, BEAM_DEPTH, BEAM_HEIGHT)),
        material=beam_steel,
        name="rail",
    )
    beam.visual(
        Box((0.026, 0.10, 0.014)),
        origin=Origin(xyz=(BEAM_LENGTH / 2.0 - 0.055, 0.0, -(BEAM_HEIGHT + 0.014) / 2.0)),
        material=beam_steel,
        name="stop_right",
    )
    beam.visual(
        Box((0.026, 0.10, 0.014)),
        origin=Origin(xyz=(-(BEAM_LENGTH / 2.0 - 0.055), 0.0, -(BEAM_HEIGHT + 0.014) / 2.0)),
        material=beam_steel,
        name="stop_left",
    )
    beam.visual(
        Box((0.045, 0.012, 0.11)),
        origin=Origin(xyz=(0.25, 0.0, BEAM_HEIGHT / 2.0 + 0.055)),
        material=hardware_steel,
        name="strap_right",
    )
    beam.visual(
        Box((0.045, 0.012, 0.11)),
        origin=Origin(xyz=(-0.25, 0.0, BEAM_HEIGHT / 2.0 + 0.055)),
        material=hardware_steel,
        name="strap_left",
    )
    beam.visual(
        Box((0.64, 0.10, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, BEAM_HEIGHT / 2.0 + 0.116)),
        material=hardware_steel,
        name="hanger_plate",
    )
    beam.inertial = Inertial.from_geometry(
        Box((BEAM_LENGTH, 0.10, 0.19)),
        mass=16.0,
        origin=Origin(xyz=(0.0, 0.0, 0.075)),
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((CARRIAGE_LENGTH, 0.104, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, 0.033)),
        material=shuttle_paint,
        name="roof",
    )
    carriage.visual(
        Box((CARRIAGE_LENGTH, 0.012, 0.110)),
        origin=Origin(xyz=(0.0, 0.046, -0.030)),
        material=shuttle_paint,
        name="side_left",
    )
    carriage.visual(
        Box((CARRIAGE_LENGTH, 0.012, 0.110)),
        origin=Origin(xyz=(0.0, -0.046, -0.030)),
        material=shuttle_paint,
        name="side_right",
    )
    carriage.visual(
        Box((0.108, 0.024, 0.100)),
        origin=Origin(xyz=(0.0, 0.052, -0.025)),
        material=shuttle_paint,
        name="front_plate",
    )
    carriage.visual(
        Box((0.102, 0.018, 0.070)),
        origin=Origin(xyz=(0.0, 0.073, -0.067)),
        material=shuttle_paint,
        name="tool_block",
    )
    carriage.visual(
        Box((0.016, 0.012, 0.036)),
        origin=Origin(xyz=(0.050, 0.088, -0.076)),
        material=shuttle_paint,
        name="gusset_left",
    )
    carriage.visual(
        Box((0.016, 0.012, 0.036)),
        origin=Origin(xyz=(-0.050, 0.088, -0.076)),
        material=shuttle_paint,
        name="gusset_right",
    )
    carriage.visual(
        Box((0.010, 0.018, 0.038)),
        origin=Origin(xyz=(0.060, 0.103, ROLLER_AXIS_Z)),
        material=hardware_steel,
        name="arm_left",
    )
    carriage.visual(
        Box((0.010, 0.018, 0.038)),
        origin=Origin(xyz=(-0.060, 0.103, ROLLER_AXIS_Z)),
        material=hardware_steel,
        name="arm_right",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((CARRIAGE_LENGTH, CARRIAGE_DEPTH, 0.17)),
        mass=4.5,
        origin=Origin(xyz=(0.0, 0.02, -0.03)),
    )

    roller_head = model.part("roller_head")
    roller_head.visual(
        Cylinder(radius=ROLLER_DRUM_RADIUS, length=ROLLER_DRUM_LENGTH),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=roller_black,
        name="roller_drum",
    )
    roller_head.visual(
        Cylinder(radius=ROLLER_SHAFT_RADIUS, length=ROLLER_SHAFT_LENGTH),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=hardware_steel,
        name="roller_core",
    )
    roller_head.visual(
        Cylinder(radius=0.016, length=0.006),
        origin=Origin(xyz=(0.050, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=hardware_steel,
        name="hub_left",
    )
    roller_head.visual(
        Cylinder(radius=0.016, length=0.006),
        origin=Origin(xyz=(-0.050, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=hardware_steel,
        name="hub_right",
    )
    roller_head.inertial = Inertial.from_geometry(
        Cylinder(radius=0.024, length=ROLLER_SHAFT_LENGTH),
        mass=1.8,
        origin=Origin(),
    )

    model.articulation(
        "beam_to_carriage",
        ArticulationType.PRISMATIC,
        parent=beam,
        child=carriage,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=350.0,
            velocity=0.35,
            lower=-0.29,
            upper=0.29,
        ),
    )
    model.articulation(
        "carriage_to_roller_head",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=roller_head,
        origin=Origin(xyz=(0.0, ROLLER_AXIS_Y, ROLLER_AXIS_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=12.0,
            lower=-3.0,
            upper=3.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    beam = object_model.get_part("beam")
    carriage = object_model.get_part("carriage")
    roller_head = object_model.get_part("roller_head")
    slide = object_model.get_articulation("beam_to_carriage")
    roller_spin = object_model.get_articulation("carriage_to_roller_head")
    rail = beam.get_visual("rail")
    roof = carriage.get_visual("roof")
    side_left = carriage.get_visual("side_left")
    side_right = carriage.get_visual("side_right")
    front_plate = carriage.get_visual("front_plate")
    tool_block = carriage.get_visual("tool_block")
    roller_drum = roller_head.get_visual("roller_drum")

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
        "parts present",
        beam is not None and carriage is not None and roller_head is not None,
        "Beam, carriage, and roller head should all exist.",
    )
    ctx.check(
        "slide articulation configuration",
        slide.articulation_type == ArticulationType.PRISMATIC and tuple(slide.axis) == (1.0, 0.0, 0.0),
        f"Expected prismatic slide on +X, got type={slide.articulation_type} axis={slide.axis}.",
    )
    ctx.check(
        "roller articulation configuration",
        roller_spin.articulation_type == ArticulationType.REVOLUTE
        and tuple(roller_spin.axis) == (1.0, 0.0, 0.0),
        f"Expected revolute roller on +X, got type={roller_spin.articulation_type} axis={roller_spin.axis}.",
    )

    ctx.expect_gap(
        carriage,
        beam,
        axis="z",
        positive_elem=roof,
        negative_elem=rail,
        min_gap=0.0,
        max_gap=0.001,
        name="roof stays flush over the beam",
    )
    ctx.expect_gap(
        carriage,
        beam,
        axis="y",
        positive_elem=side_left,
        negative_elem=rail,
        min_gap=0.0,
        max_gap=0.001,
        name="left guide plate hugs beam side",
    )
    ctx.expect_gap(
        beam,
        carriage,
        axis="y",
        positive_elem=rail,
        negative_elem=side_right,
        min_gap=0.0,
        max_gap=0.001,
        name="right guide plate hugs beam side",
    )
    ctx.expect_overlap(carriage, beam, axes="x", min_overlap=0.12, name="carriage remains on beam span")
    ctx.expect_overlap(roller_head, carriage, axes="xz", min_overlap=0.03, name="roller stays aligned with support arms")
    ctx.expect_gap(
        roller_head,
        carriage,
        axis="y",
        positive_elem=roller_drum,
        negative_elem=front_plate,
        min_gap=0.020,
        max_gap=0.035,
        name="roller projects ahead of carriage face",
    )
    ctx.expect_origin_gap(
        roller_head,
        carriage,
        axis="y",
        min_gap=0.11,
        max_gap=0.129,
        name="roller axis projects forward of carriage",
    )

    with ctx.pose({slide: slide.motion_limits.lower}):
        ctx.expect_gap(
            carriage,
            beam,
            axis="z",
            positive_elem=roof,
            negative_elem=rail,
            min_gap=0.0,
            max_gap=0.001,
            name="lower travel keeps roof seated over beam",
        )
    with ctx.pose({slide: slide.motion_limits.upper}):
        ctx.expect_gap(
            carriage,
            beam,
            axis="z",
            positive_elem=roof,
            negative_elem=rail,
            min_gap=0.0,
            max_gap=0.001,
            name="upper travel keeps roof seated over beam",
        )
    with ctx.pose({roller_spin: 1.2}):
        ctx.expect_overlap(
            roller_head,
            carriage,
            axes="x",
            min_overlap=0.08,
            name="roller stays captured laterally while rotated",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
