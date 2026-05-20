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


PLATE_WIDTH = 0.170
PLATE_HEIGHT = 0.460
PLATE_THICKNESS = 0.012

RAIL_BASE_DEPTH = 0.012
RAIL_BASE_WIDTH = 0.072
RAIL_BASE_HEIGHT = 0.348

RAIL_TONGUE_DEPTH = 0.012
RAIL_TONGUE_WIDTH = 0.048
RAIL_TONGUE_HEIGHT = 0.328

CARRIAGE_DEPTH = 0.068
CARRIAGE_WIDTH = 0.100
CARRIAGE_HEIGHT = 0.090
CARRIAGE_GUIDE_X = 0.030
CARRIAGE_Z_LOWER = -0.110
CARRIAGE_TRAVEL = 0.220

HEAD_AXIS_X = 0.068
HEAD_SHAFT_RADIUS = 0.012
HEAD_FACE_RADIUS = 0.030


def _backplate_shape() -> cq.Workplane:
    plate = cq.Workplane("XY").box(
        PLATE_THICKNESS,
        PLATE_WIDTH,
        PLATE_HEIGHT,
    )
    plate = plate.edges("|Z").fillet(0.005)

    mount_holes = (
        cq.Workplane("YZ")
        .pushPoints(
            [
                (-0.055, -0.165),
                (0.055, -0.165),
                (-0.055, 0.165),
                (0.055, 0.165),
            ]
        )
        .circle(0.0045)
        .extrude(PLATE_THICKNESS * 3.0)
        .translate((-PLATE_THICKNESS * 1.5, 0.0, 0.0))
    )
    plate = plate.cut(mount_holes)

    rail_base = (
        cq.Workplane("XY")
        .box(RAIL_BASE_DEPTH, RAIL_BASE_WIDTH, RAIL_BASE_HEIGHT)
        .translate((PLATE_THICKNESS * 0.5 + RAIL_BASE_DEPTH * 0.5, 0.0, 0.0))
        .edges("|Z")
        .fillet(0.0035)
    )
    rail_tongue = (
        cq.Workplane("XY")
        .box(RAIL_TONGUE_DEPTH, RAIL_TONGUE_WIDTH, RAIL_TONGUE_HEIGHT)
        .translate(
            (
                PLATE_THICKNESS * 0.5 + RAIL_BASE_DEPTH + RAIL_TONGUE_DEPTH * 0.5,
                0.0,
                0.0,
            )
        )
        .edges("|Z")
        .fillet(0.0025)
    )

    top_stop = cq.Workplane("XY").box(0.010, 0.084, 0.016).translate(
        (
            PLATE_THICKNESS * 0.5 + 0.010,
            0.0,
            RAIL_BASE_HEIGHT * 0.5 + 0.008,
        )
    )
    bottom_stop = cq.Workplane("XY").box(0.010, 0.084, 0.016).translate(
        (
            PLATE_THICKNESS * 0.5 + 0.010,
            0.0,
            -(RAIL_BASE_HEIGHT * 0.5 + 0.008),
        )
    )

    return plate.union(rail_base).union(rail_tongue).union(top_stop).union(bottom_stop)


def _carriage_shape() -> cq.Workplane:
    body_length = 0.056
    boss_length = CARRIAGE_DEPTH - body_length

    body = (
        cq.Workplane("XY")
        .box(body_length, CARRIAGE_WIDTH, CARRIAGE_HEIGHT)
        .translate((body_length * 0.5, 0.0, 0.0))
        .edges("|Z")
        .fillet(0.004)
    )

    rear_stage_pocket = (
        cq.Workplane("YZ")
        .rect(0.058, 0.070)
        .extrude(0.010)
    )
    rear_tongue_witness = (
        cq.Workplane("YZ")
        .rect(0.040, 0.058)
        .extrude(0.020)
    )
    upper_relief = (
        cq.Workplane("XZ")
        .pushPoints([(0.026, 0.0)])
        .slot2D(0.036, 0.016, 90)
        .extrude(0.012, both=True)
        .translate((0.0, 0.028, 0.0))
    )
    body = (
        body.cut(rear_stage_pocket)
        .cut(rear_tongue_witness)
        .cut(upper_relief)
        .cut(upper_relief.mirror("XZ"))
    )

    face_boss = (
        cq.Workplane("YZ")
        .circle(0.024)
        .extrude(boss_length)
        .translate((body_length, 0.0, 0.0))
    )
    bore = (
        cq.Workplane("YZ")
        .workplane(offset=CARRIAGE_DEPTH)
        .circle(HEAD_SHAFT_RADIUS)
        .extrude(-0.006)
    )

    return body.union(face_boss).cut(bore)


def _head_main_shape() -> cq.Workplane:
    rear_hub = (
        cq.Workplane("YZ")
        .circle(HEAD_SHAFT_RADIUS)
        .extrude(0.006)
    )
    face = (
        cq.Workplane("YZ")
        .circle(HEAD_FACE_RADIUS)
        .extrude(0.014)
        .translate((0.006, 0.0, 0.0))
    )
    front_recess = (
        cq.Workplane("YZ")
        .workplane(offset=0.020)
        .circle(0.020)
        .extrude(-0.003)
    )
    return rear_hub.union(face).cut(front_recess)


def _head_knob_shape() -> cq.Workplane:
    return (
        cq.Workplane("YZ")
        .circle(0.006)
        .extrude(0.012)
        .translate((0.008, 0.0, 0.022))
    )


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    low, high = aabb
    return tuple((lo + hi) * 0.5 for lo, hi in zip(low, high))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_backed_linear_axis")

    plate_finish = model.material("plate_finish", color=(0.23, 0.25, 0.27, 1.0))
    carriage_finish = model.material("carriage_finish", color=(0.70, 0.72, 0.74, 1.0))
    head_finish = model.material("head_finish", color=(0.16, 0.17, 0.18, 1.0))
    knob_finish = model.material("knob_finish", color=(0.75, 0.16, 0.12, 1.0))

    backplate = model.part("backplate")
    backplate.visual(
        mesh_from_cadquery(_backplate_shape(), "backplate"),
        origin=Origin(),
        material=plate_finish,
        name="backplate_shell",
    )

    carriage = model.part("carriage")
    carriage.visual(
        mesh_from_cadquery(_carriage_shape(), "carriage"),
        origin=Origin(),
        material=carriage_finish,
        name="carriage_shell",
    )

    head = model.part("turning_head")
    head.visual(
        mesh_from_cadquery(_head_main_shape(), "turning_head"),
        origin=Origin(),
        material=head_finish,
        name="head_shell",
    )
    head.visual(
        mesh_from_cadquery(_head_knob_shape(), "head_knob"),
        origin=Origin(),
        material=knob_finish,
        name="grip_knob",
    )

    model.articulation(
        "backplate_to_carriage",
        ArticulationType.PRISMATIC,
        parent=backplate,
        child=carriage,
        origin=Origin(xyz=(CARRIAGE_GUIDE_X, 0.0, CARRIAGE_Z_LOWER)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=140.0,
            velocity=0.18,
            lower=0.0,
            upper=CARRIAGE_TRAVEL,
        ),
    )
    model.articulation(
        "carriage_to_head",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=head,
        origin=Origin(xyz=(HEAD_AXIS_X, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.8,
            lower=-1.6,
            upper=1.6,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    backplate = object_model.get_part("backplate")
    carriage = object_model.get_part("carriage")
    head = object_model.get_part("turning_head")
    slide = object_model.get_articulation("backplate_to_carriage")
    spin = object_model.get_articulation("carriage_to_head")
    grip = head.get_visual("grip_knob")

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

    ctx.expect_contact(carriage, backplate, name="carriage_is_supported_by_stage")
    ctx.expect_contact(head, carriage, name="turning_head_is_supported_by_carriage")
    ctx.expect_overlap(
        carriage,
        backplate,
        axes="yz",
        min_overlap=0.060,
        name="carriage_tracks_over_the_guide_stage",
    )
    ctx.expect_gap(
        head,
        backplate,
        axis="x",
        min_gap=0.045,
        name="rotary_face_projects_forward_of_backplate",
    )
    ctx.check(
        "joint_axes_match_prompted_mechanisms",
        slide.axis == (0.0, 0.0, 1.0) and spin.axis == (1.0, 0.0, 0.0),
        details=f"slide axis={slide.axis}, spin axis={spin.axis}",
    )

    with ctx.pose({slide: CARRIAGE_TRAVEL, spin: 0.0}):
        ctx.expect_contact(
            carriage,
            backplate,
            name="carriage_remains_carried_at_upper_travel",
        )
        ctx.expect_contact(
            head,
            carriage,
            name="head_remains_carried_at_upper_travel",
        )

    lower_position = None
    upper_position = None
    grip_rest = None
    grip_turned = None

    with ctx.pose({slide: 0.0, spin: 0.0}):
        lower_position = ctx.part_world_position(carriage)
    with ctx.pose({slide: CARRIAGE_TRAVEL, spin: 0.0}):
        upper_position = ctx.part_world_position(carriage)
    with ctx.pose({slide: CARRIAGE_TRAVEL * 0.5, spin: 0.0}):
        grip_rest = _aabb_center(ctx.part_element_world_aabb(head, elem=grip))
    with ctx.pose({slide: CARRIAGE_TRAVEL * 0.5, spin: 1.2}):
        grip_turned = _aabb_center(ctx.part_element_world_aabb(head, elem=grip))

    slide_ok = (
        lower_position is not None
        and upper_position is not None
        and abs((upper_position[2] - lower_position[2]) - CARRIAGE_TRAVEL) < 1e-4
        and abs(upper_position[0] - lower_position[0]) < 1e-5
        and abs(upper_position[1] - lower_position[1]) < 1e-5
    )
    ctx.check(
        "prismatic_stage_moves_carriage_straight_up_the_backplate",
        slide_ok,
        details=f"lower={lower_position}, upper={upper_position}",
    )

    spin_ok = (
        grip_rest is not None
        and grip_turned is not None
        and abs(grip_turned[0] - grip_rest[0]) < 0.002
        and abs(grip_turned[1] - grip_rest[1]) > 0.015
        and abs(grip_turned[2] - grip_rest[2]) > 0.005
    )
    ctx.check(
        "front_grip_orbits_when_head_rotates",
        spin_ok,
        details=f"rest={grip_rest}, turned={grip_turned}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
