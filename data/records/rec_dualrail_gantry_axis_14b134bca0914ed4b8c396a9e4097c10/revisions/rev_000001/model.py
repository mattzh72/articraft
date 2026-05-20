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


BASE_LENGTH = 1.02
BASE_WIDTH = 0.70
FOOT_HEIGHT = 0.016
FOOT_RADIUS = 0.026
DECK_THICKNESS = 0.018

RAIL_LENGTH = 0.88
RAIL_CENTER_Y = 0.28
RAIL_BODY_WIDTH = 0.060
RAIL_BODY_HEIGHT = 0.065
RAIL_CAP_LENGTH = 0.82
RAIL_CAP_WIDTH = 0.022
RAIL_CAP_THICKNESS = 0.008
RAIL_CAP_TOP_Z = FOOT_HEIGHT + DECK_THICKNESS + RAIL_BODY_HEIGHT + RAIL_CAP_THICKNESS
RAIL_CAP_CENTER_Z = RAIL_CAP_TOP_Z - (RAIL_CAP_THICKNESS / 2.0)

BRIDGE_RUNNER_LENGTH = 0.11
BRIDGE_RUNNER_WIDTH = 0.045
BRIDGE_RUNNER_THICKNESS = 0.018
TRUCK_WIDTH = 0.075
TRUCK_HEIGHT = 0.160
BEAM_DEPTH = 0.095
BEAM_SPAN = 0.51
BEAM_HEIGHT = 0.080
BEAM_CENTER_Z = 0.160
BEAM_GUIDE_DEPTH = 0.042
BEAM_GUIDE_LENGTH = 0.40
BEAM_GUIDE_THICKNESS = 0.018
BEAM_GUIDE_CENTER_Z = (BEAM_CENTER_Z - (BEAM_HEIGHT / 2.0)) - (BEAM_GUIDE_THICKNESS / 2.0)
BEAM_GUIDE_BOTTOM_Z = BEAM_GUIDE_CENTER_Z - (BEAM_GUIDE_THICKNESS / 2.0)

SLED_PAD_DEPTH = 0.075
SLED_PAD_WIDTH = 0.10
SLED_PAD_THICKNESS = 0.022

BRIDGE_TRAVEL = 0.27
SLED_TRAVEL = 0.13


def _box(center: tuple[float, float, float], size: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _cylinder(
    center: tuple[float, float, float], radius: float, height: float
) -> cq.Workplane:
    return cq.Workplane("XY").circle(radius).extrude(height).translate(
        (center[0], center[1], center[2] - (height / 2.0))
    )


def build_base_chassis() -> cq.Workplane:
    feet = cq.Workplane("XY").pushPoints(
        [
            (-0.43, -0.27),
            (-0.43, 0.27),
            (0.43, -0.27),
            (0.43, 0.27),
        ]
    ).circle(FOOT_RADIUS).extrude(FOOT_HEIGHT)

    deck_outer = (
        cq.Workplane("XY")
        .box(BASE_LENGTH, BASE_WIDTH, DECK_THICKNESS)
        .edges("|Z")
        .fillet(0.012)
        .translate((0.0, 0.0, FOOT_HEIGHT + (DECK_THICKNESS / 2.0)))
    )
    deck_window = _box(
        (0.0, 0.0, FOOT_HEIGHT + (DECK_THICKNESS / 2.0)),
        (0.84, 0.44, DECK_THICKNESS + 0.002),
    )
    deck = deck_outer.cut(deck_window)

    left_rail = _box(
        (
            0.0,
            RAIL_CENTER_Y,
            FOOT_HEIGHT + DECK_THICKNESS + (RAIL_BODY_HEIGHT / 2.0),
        ),
        (RAIL_LENGTH, RAIL_BODY_WIDTH, RAIL_BODY_HEIGHT),
    )
    right_rail = _box(
        (
            0.0,
            -RAIL_CENTER_Y,
            FOOT_HEIGHT + DECK_THICKNESS + (RAIL_BODY_HEIGHT / 2.0),
        ),
        (RAIL_LENGTH, RAIL_BODY_WIDTH, RAIL_BODY_HEIGHT),
    )

    return feet.union(deck).union(left_rail).union(right_rail)


def build_bridge_frame() -> cq.Workplane:
    left_truck = _box(
        (
            0.0,
            RAIL_CENTER_Y,
            BRIDGE_RUNNER_THICKNESS + (TRUCK_HEIGHT / 2.0),
        ),
        (0.15, TRUCK_WIDTH, TRUCK_HEIGHT),
    )
    right_truck = _box(
        (
            0.0,
            -RAIL_CENTER_Y,
            BRIDGE_RUNNER_THICKNESS + (TRUCK_HEIGHT / 2.0),
        ),
        (0.15, TRUCK_WIDTH, TRUCK_HEIGHT),
    )

    beam_outer = _box((0.0, 0.0, BEAM_CENTER_Z), (BEAM_DEPTH, BEAM_SPAN, BEAM_HEIGHT))
    beam_inner = _box((0.0, 0.0, BEAM_CENTER_Z), (0.063, 0.45, 0.050))
    beam = beam_outer.cut(beam_inner)

    return left_truck.union(right_truck).union(beam)


def build_sled_body() -> cq.Workplane:
    hanger = _box((0.0, 0.0, -0.082), (0.020, 0.12, 0.120))
    carriage_face = _box((0.024, 0.0, -0.116), (0.058, 0.084, 0.070))
    spindle = _cylinder((0.026, 0.0, -0.154), 0.026, 0.055)
    collet = _cylinder((0.026, 0.0, -0.194), 0.010, 0.025)
    nozzle = _cylinder((0.026, 0.0, -0.214), 0.0045, 0.015)

    return hanger.union(carriage_face).union(spindle).union(collet).union(nozzle)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tabletop_gantry_axis")

    graphite = model.material("graphite", rgba=(0.23, 0.24, 0.26, 1.0))
    aluminum = model.material("aluminum", rgba=(0.72, 0.74, 0.77, 1.0))
    bridge_blue = model.material("bridge_blue", rgba=(0.18, 0.40, 0.77, 1.0))
    sled_orange = model.material("sled_orange", rgba=(0.91, 0.45, 0.12, 1.0))
    tool_dark = model.material("tool_dark", rgba=(0.15, 0.16, 0.18, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(build_base_chassis(), "base_chassis"),
        material=graphite,
        name="base_chassis",
    )
    base.visual(
        Box((RAIL_CAP_LENGTH, RAIL_CAP_WIDTH, RAIL_CAP_THICKNESS)),
        origin=Origin(xyz=(0.0, RAIL_CENTER_Y, RAIL_CAP_CENTER_Z)),
        material=aluminum,
        name="left_rail_cap",
    )
    base.visual(
        Box((RAIL_CAP_LENGTH, RAIL_CAP_WIDTH, RAIL_CAP_THICKNESS)),
        origin=Origin(xyz=(0.0, -RAIL_CENTER_Y, RAIL_CAP_CENTER_Z)),
        material=aluminum,
        name="right_rail_cap",
    )
    base.inertial = Inertial.from_geometry(
        Box((BASE_LENGTH, BASE_WIDTH, RAIL_CAP_TOP_Z)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, RAIL_CAP_TOP_Z / 2.0)),
    )

    bridge = model.part("bridge")
    bridge.visual(
        Box((BRIDGE_RUNNER_LENGTH, BRIDGE_RUNNER_WIDTH, BRIDGE_RUNNER_THICKNESS)),
        origin=Origin(
            xyz=(
                0.0,
                RAIL_CENTER_Y,
                BRIDGE_RUNNER_THICKNESS / 2.0,
            )
        ),
        material=aluminum,
        name="left_truck_runner",
    )
    bridge.visual(
        Box((BRIDGE_RUNNER_LENGTH, BRIDGE_RUNNER_WIDTH, BRIDGE_RUNNER_THICKNESS)),
        origin=Origin(
            xyz=(
                0.0,
                -RAIL_CENTER_Y,
                BRIDGE_RUNNER_THICKNESS / 2.0,
            )
        ),
        material=aluminum,
        name="right_truck_runner",
    )
    bridge.visual(
        mesh_from_cadquery(build_bridge_frame(), "bridge_frame"),
        material=bridge_blue,
        name="bridge_frame",
    )
    bridge.visual(
        Box((BEAM_GUIDE_DEPTH, BEAM_GUIDE_LENGTH, BEAM_GUIDE_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, BEAM_GUIDE_CENTER_Z)),
        material=aluminum,
        name="beam_guide",
    )
    bridge.inertial = Inertial.from_geometry(
        Box((0.15, 0.64, 0.20)),
        mass=8.0,
        origin=Origin(xyz=(0.0, 0.0, 0.10)),
    )

    sled = model.part("sled")
    sled.visual(
        Box((SLED_PAD_DEPTH, SLED_PAD_WIDTH, SLED_PAD_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, -(SLED_PAD_THICKNESS / 2.0))),
        material=aluminum,
        name="sled_pad",
    )
    sled.visual(
        mesh_from_cadquery(build_sled_body(), "sled_body"),
        material=sled_orange,
        name="sled_body",
    )
    sled.visual(
        Box((0.020, 0.020, 0.020)),
        origin=Origin(xyz=(0.026, 0.0, -0.226)),
        material=tool_dark,
        name="tool_tip",
    )
    sled.inertial = Inertial.from_geometry(
        Box((0.08, 0.12, 0.24)),
        mass=2.6,
        origin=Origin(xyz=(0.015, 0.0, -0.11)),
    )

    model.articulation(
        "base_to_bridge",
        ArticulationType.PRISMATIC,
        parent=base,
        child=bridge,
        origin=Origin(xyz=(0.0, 0.0, RAIL_CAP_TOP_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=140.0,
            velocity=0.65,
            lower=-BRIDGE_TRAVEL,
            upper=BRIDGE_TRAVEL,
        ),
    )
    model.articulation(
        "bridge_to_sled",
        ArticulationType.PRISMATIC,
        parent=bridge,
        child=sled,
        origin=Origin(xyz=(0.0, 0.0, BEAM_GUIDE_BOTTOM_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=0.55,
            lower=-SLED_TRAVEL,
            upper=SLED_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    bridge = object_model.get_part("bridge")
    sled = object_model.get_part("sled")
    bridge_slide = object_model.get_articulation("base_to_bridge")
    sled_slide = object_model.get_articulation("bridge_to_sled")

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
        "bridge_is_x_prismatic",
        bridge_slide.articulation_type == ArticulationType.PRISMATIC
        and tuple(bridge_slide.axis) == (1.0, 0.0, 0.0),
        f"type={bridge_slide.articulation_type}, axis={bridge_slide.axis}",
    )
    ctx.check(
        "sled_is_y_prismatic",
        sled_slide.articulation_type == ArticulationType.PRISMATIC
        and tuple(sled_slide.axis) == (0.0, 1.0, 0.0),
        f"type={sled_slide.articulation_type}, axis={sled_slide.axis}",
    )
    ctx.check(
        "bridge_travel_longer_than_sled_travel",
        bridge_slide.motion_limits is not None
        and sled_slide.motion_limits is not None
        and bridge_slide.motion_limits.upper is not None
        and bridge_slide.motion_limits.lower is not None
        and sled_slide.motion_limits.upper is not None
        and sled_slide.motion_limits.lower is not None
        and (bridge_slide.motion_limits.upper - bridge_slide.motion_limits.lower)
        > (sled_slide.motion_limits.upper - sled_slide.motion_limits.lower),
        "Bridge stroke should read visibly larger than sled stroke.",
    )

    ctx.expect_contact(
        bridge,
        base,
        elem_a="left_truck_runner",
        elem_b="left_rail_cap",
        name="left_bridge_runner_contacts_left_rail",
    )
    ctx.expect_contact(
        bridge,
        base,
        elem_a="right_truck_runner",
        elem_b="right_rail_cap",
        name="right_bridge_runner_contacts_right_rail",
    )
    ctx.expect_contact(
        sled,
        bridge,
        elem_a="sled_pad",
        elem_b="beam_guide",
        name="sled_pad_contacts_beam_guide",
    )

    with ctx.pose({bridge_slide: bridge_slide.motion_limits.lower}):
        ctx.expect_within(
            bridge,
            base,
            axes="x",
            inner_elem="left_truck_runner",
            outer_elem="left_rail_cap",
            margin=0.0,
            name="left_runner_stays_on_rail_at_low_bridge_pose",
        )
        ctx.expect_within(
            bridge,
            base,
            axes="x",
            inner_elem="right_truck_runner",
            outer_elem="right_rail_cap",
            margin=0.0,
            name="right_runner_stays_on_rail_at_low_bridge_pose",
        )

    with ctx.pose({bridge_slide: bridge_slide.motion_limits.upper}):
        ctx.expect_within(
            bridge,
            base,
            axes="x",
            inner_elem="left_truck_runner",
            outer_elem="left_rail_cap",
            margin=0.0,
            name="left_runner_stays_on_rail_at_high_bridge_pose",
        )
        ctx.expect_within(
            bridge,
            base,
            axes="x",
            inner_elem="right_truck_runner",
            outer_elem="right_rail_cap",
            margin=0.0,
            name="right_runner_stays_on_rail_at_high_bridge_pose",
        )

    with ctx.pose({sled_slide: sled_slide.motion_limits.lower}):
        ctx.expect_within(
            sled,
            bridge,
            axes="y",
            inner_elem="sled_pad",
            outer_elem="beam_guide",
            margin=0.0,
            name="sled_pad_stays_on_guide_at_left_pose",
        )

    with ctx.pose({sled_slide: sled_slide.motion_limits.upper}):
        ctx.expect_within(
            sled,
            bridge,
            axes="y",
            inner_elem="sled_pad",
            outer_elem="beam_guide",
            margin=0.0,
            name="sled_pad_stays_on_guide_at_right_pose",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
