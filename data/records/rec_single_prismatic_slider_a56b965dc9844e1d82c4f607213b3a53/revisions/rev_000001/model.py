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


BASE_LENGTH = 0.360
BASE_WIDTH = 0.092
BASE_HEIGHT = 0.024

WAY_RELIEF_LENGTH = 0.292
WAY_RELIEF_WIDTH = 0.040
WAY_RELIEF_DEPTH = 0.004

STOP_LENGTH = 0.014
STOP_WIDTH = 0.084
STOP_CENTER_GAP = 0.066
STOP_HEIGHT = 0.018
STOP_X = 0.160

BASE_SCREW_X = 0.118
BASE_SCREW_Y = 0.037

CARRIAGE_LENGTH = 0.086
CARRIAGE_WIDTH = 0.060
CARRIAGE_BODY_HEIGHT = 0.030
CARRIAGE_CLEARANCE_Z = 0.002
CARRIAGE_PEDESTAL_LENGTH = 0.050
CARRIAGE_PEDESTAL_WIDTH = 0.046
CARRIAGE_PEDESTAL_HEIGHT = 0.006
CARRIAGE_WEAR_PAD_LENGTH = 0.062
CARRIAGE_WEAR_PAD_WIDTH = 0.010
CARRIAGE_WEAR_PAD_DROP = CARRIAGE_CLEARANCE_Z
CARRIAGE_WEAR_PAD_Y = 0.021

PAD_SIZE = 0.070
PAD_THICKNESS = 0.010
PAD_MOUNT_Z = CARRIAGE_BODY_HEIGHT + CARRIAGE_PEDESTAL_HEIGHT

PAD_SCREW_OFFSET = 0.021
TRAVEL_HALF = 0.108


def _socket_head_cap_screw(
    *,
    shank_diameter: float,
    shank_length: float,
    head_diameter: float,
    head_height: float,
    socket_diameter: float,
    socket_depth: float,
) -> cq.Workplane:
    screw = cq.Workplane("XY").circle(shank_diameter / 2.0).extrude(shank_length)
    head = (
        cq.Workplane("XY")
        .circle(head_diameter / 2.0)
        .extrude(head_height)
        .translate((0.0, 0.0, shank_length))
    )
    screw = screw.union(head)
    socket = (
        cq.Workplane("XY")
        .circle(socket_diameter / 2.0)
        .extrude(socket_depth)
        .translate((0.0, 0.0, shank_length + head_height - socket_depth))
    )
    screw = screw.cut(socket)
    return screw.edges(">Z").chamfer(min(head_height * 0.18, 0.0007))


def _beam_body_shape() -> cq.Workplane:
    body = cq.Workplane("XY").box(
        BASE_LENGTH,
        BASE_WIDTH,
        BASE_HEIGHT,
        centered=(True, True, False),
    )
    body = body.edges("|Z").chamfer(0.0015)
    body = body.faces(">Z").edges().chamfer(0.0008)
    way_relief = (
        cq.Workplane("XY")
        .box(
            WAY_RELIEF_LENGTH,
            WAY_RELIEF_WIDTH,
            WAY_RELIEF_DEPTH,
            centered=(True, True, False),
        )
        .translate((0.0, 0.0, BASE_HEIGHT - WAY_RELIEF_DEPTH))
    )
    body = body.cut(way_relief)
    body = (
        body.faces(">Z")
        .workplane()
        .pushPoints(
            [
                (-BASE_SCREW_X, -BASE_SCREW_Y),
                (-BASE_SCREW_X, BASE_SCREW_Y),
                (BASE_SCREW_X, -BASE_SCREW_Y),
                (BASE_SCREW_X, BASE_SCREW_Y),
            ]
        )
        .circle(0.0052)
        .cutBlind(-0.0014)
    )
    body = (
        body.faces(">Z")
        .workplane()
        .pushPoints(
            [
                (-BASE_SCREW_X, -BASE_SCREW_Y),
                (-BASE_SCREW_X, BASE_SCREW_Y),
                (BASE_SCREW_X, -BASE_SCREW_Y),
                (BASE_SCREW_X, BASE_SCREW_Y),
            ]
        )
        .circle(0.0029)
        .cutBlind(-0.010)
    )
    return body


def _stop_shape() -> cq.Workplane:
    stop = cq.Workplane("XY").box(
        STOP_LENGTH,
        STOP_WIDTH,
        STOP_HEIGHT,
        centered=(True, True, False),
    )
    notch = cq.Workplane("XY").box(
        STOP_LENGTH + 0.002,
        STOP_CENTER_GAP,
        STOP_HEIGHT + 0.004,
        centered=(True, True, False),
    )
    stop = stop.cut(notch.translate((0.0, 0.0, -0.001)))
    return stop.edges("|Z").chamfer(0.001)


def _carriage_shape() -> cq.Workplane:
    carriage = cq.Workplane("XY").box(
        CARRIAGE_LENGTH,
        CARRIAGE_WIDTH,
        CARRIAGE_BODY_HEIGHT,
        centered=(True, True, False),
    )
    for y_pos in (-CARRIAGE_WEAR_PAD_Y, CARRIAGE_WEAR_PAD_Y):
        wear_pad = (
            cq.Workplane("XY")
            .box(
                CARRIAGE_WEAR_PAD_LENGTH,
                CARRIAGE_WEAR_PAD_WIDTH,
                CARRIAGE_WEAR_PAD_DROP,
                centered=(True, True, False),
            )
            .translate((0.0, y_pos, -CARRIAGE_WEAR_PAD_DROP))
        )
        carriage = carriage.union(wear_pad)
    pedestal = (
        cq.Workplane("XY")
        .box(
            CARRIAGE_PEDESTAL_LENGTH,
            CARRIAGE_PEDESTAL_WIDTH,
            CARRIAGE_PEDESTAL_HEIGHT,
            centered=(True, True, False),
        )
        .translate((0.0, 0.0, CARRIAGE_BODY_HEIGHT))
    )
    carriage = carriage.union(pedestal)
    carriage = carriage.edges("|Z").chamfer(0.0012)
    carriage = carriage.faces(">Z").edges().chamfer(0.0008)
    return carriage


def _pad_shape() -> cq.Workplane:
    pad = cq.Workplane("XY").box(
        PAD_SIZE,
        PAD_SIZE,
        PAD_THICKNESS,
        centered=(True, True, False),
    )
    pad = pad.edges("|Z").chamfer(0.001)
    pad = pad.faces(">Z").edges().chamfer(0.0012)
    points = [
        (-PAD_SCREW_OFFSET, -PAD_SCREW_OFFSET),
        (-PAD_SCREW_OFFSET, PAD_SCREW_OFFSET),
        (PAD_SCREW_OFFSET, -PAD_SCREW_OFFSET),
        (PAD_SCREW_OFFSET, PAD_SCREW_OFFSET),
    ]
    pad = pad.faces(">Z").workplane().pushPoints(points).circle(0.0023).cutBlind(-0.0075)
    return pad


def _part_visual(
    part,
    shape: cq.Workplane,
    mesh_name: str,
    *,
    material: str,
    origin: Origin | None = None,
    visual_name: str | None = None,
) -> None:
    part.visual(
        mesh_from_cadquery(shape, mesh_name),
        origin=origin or Origin(),
        material=material,
        name=visual_name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="precision_single_axis_stage")

    model.material("grounded_beam", rgba=(0.23, 0.25, 0.28, 1.0))
    model.material("way_steel", rgba=(0.73, 0.75, 0.79, 1.0))
    model.material("carriage_alloy", rgba=(0.77, 0.79, 0.82, 1.0))
    model.material("pad_finish", rgba=(0.39, 0.44, 0.53, 1.0))
    model.material("fastener_oxide", rgba=(0.12, 0.12, 0.13, 1.0))

    beam = model.part("base_beam")
    _part_visual(
        beam,
        _beam_body_shape(),
        "base_beam_body",
        material="grounded_beam",
        visual_name="beam_body",
    )
    _part_visual(
        beam,
        _stop_shape(),
        "base_beam_neg_stop",
        material="grounded_beam",
        origin=Origin(xyz=(-STOP_X, 0.0, BASE_HEIGHT)),
        visual_name="neg_stop",
    )
    _part_visual(
        beam,
        _stop_shape(),
        "base_beam_pos_stop",
        material="grounded_beam",
        origin=Origin(xyz=(STOP_X, 0.0, BASE_HEIGHT)),
        visual_name="pos_stop",
    )
    beam_screw_shape = _socket_head_cap_screw(
        shank_diameter=0.0056,
        shank_length=0.010,
        head_diameter=0.009,
        head_height=0.0032,
        socket_diameter=0.0032,
        socket_depth=0.0018,
    )
    for index, (x_pos, y_pos) in enumerate(
        (
            (-BASE_SCREW_X, -BASE_SCREW_Y),
            (-BASE_SCREW_X, BASE_SCREW_Y),
            (BASE_SCREW_X, -BASE_SCREW_Y),
            (BASE_SCREW_X, BASE_SCREW_Y),
        ),
        start=1,
    ):
        _part_visual(
            beam,
            beam_screw_shape,
            f"base_beam_screw_{index}",
            material="fastener_oxide",
            origin=Origin(xyz=(x_pos, y_pos, BASE_HEIGHT - 0.010)),
        )
    beam.inertial = Inertial.from_geometry(
        Box((BASE_LENGTH, BASE_WIDTH, BASE_HEIGHT + STOP_HEIGHT)),
        mass=5.6,
        origin=Origin(xyz=(0.0, 0.0, (BASE_HEIGHT + STOP_HEIGHT) / 2.0)),
    )

    carriage = model.part("carriage_shoe")
    _part_visual(
        carriage,
        _carriage_shape(),
        "carriage_shoe_body",
        material="carriage_alloy",
        visual_name="shoe_body",
    )
    carriage.inertial = Inertial.from_geometry(
        Box(
            (
                CARRIAGE_LENGTH,
                CARRIAGE_WIDTH,
                CARRIAGE_BODY_HEIGHT + CARRIAGE_PEDESTAL_HEIGHT + CARRIAGE_WEAR_PAD_DROP,
            )
        ),
        mass=1.35,
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                (
                    CARRIAGE_BODY_HEIGHT
                    + CARRIAGE_PEDESTAL_HEIGHT
                    - CARRIAGE_WEAR_PAD_DROP
                )
                / 2.0,
            )
        ),
    )

    pad = model.part("instrument_pad")
    _part_visual(
        pad,
        _pad_shape(),
        "instrument_pad_body",
        material="pad_finish",
        visual_name="pad_body",
    )
    pad_screw_shape = _socket_head_cap_screw(
        shank_diameter=0.0050,
        shank_length=PAD_THICKNESS,
        head_diameter=0.0086,
        head_height=0.0034,
        socket_diameter=0.0030,
        socket_depth=0.0018,
    )
    for index, (x_pos, y_pos) in enumerate(
        (
            (-PAD_SCREW_OFFSET, -PAD_SCREW_OFFSET),
            (-PAD_SCREW_OFFSET, PAD_SCREW_OFFSET),
            (PAD_SCREW_OFFSET, -PAD_SCREW_OFFSET),
            (PAD_SCREW_OFFSET, PAD_SCREW_OFFSET),
        ),
        start=1,
    ):
        _part_visual(
            pad,
            pad_screw_shape,
            f"instrument_pad_screw_{index}",
            material="fastener_oxide",
            origin=Origin(xyz=(x_pos, y_pos, 0.0)),
        )
    pad.inertial = Inertial.from_geometry(
        Box((PAD_SIZE, PAD_SIZE, PAD_THICKNESS + 0.0034)),
        mass=0.55,
        origin=Origin(xyz=(0.0, 0.0, (PAD_THICKNESS + 0.0034) / 2.0)),
    )

    model.articulation(
        "beam_to_carriage",
        ArticulationType.PRISMATIC,
        parent=beam,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT + CARRIAGE_CLEARANCE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=-TRAVEL_HALF,
            upper=TRAVEL_HALF,
            effort=2400.0,
            velocity=0.22,
        ),
    )
    model.articulation(
        "carriage_to_pad",
        ArticulationType.FIXED,
        parent=carriage,
        child=pad,
        origin=Origin(xyz=(0.0, 0.0, PAD_MOUNT_Z)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    beam = object_model.get_part("base_beam")
    carriage = object_model.get_part("carriage_shoe")
    pad = object_model.get_part("instrument_pad")
    stage = object_model.get_articulation("beam_to_carriage")
    pad_mount = object_model.get_articulation("carriage_to_pad")
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
    ctx.fail_if_articulation_overlaps(max_pose_samples=21)

    ctx.check(
        "prismatic stage runs on x axis",
        stage.articulation_type == ArticulationType.PRISMATIC and tuple(stage.axis) == (1.0, 0.0, 0.0),
        f"expected x-axis prismatic stage, got type={stage.articulation_type} axis={stage.axis}",
    )
    stage_limits = stage.motion_limits
    ctx.check(
        "stage travel limits are symmetric and finite",
        stage_limits is not None
        and stage_limits.lower is not None
        and stage_limits.upper is not None
        and abs(stage_limits.lower + TRAVEL_HALF) < 1e-9
        and abs(stage_limits.upper - TRAVEL_HALF) < 1e-9,
        f"unexpected stage limits: {stage_limits}",
    )
    ctx.check(
        "instrument pad is rigidly fixed to carriage",
        pad_mount.articulation_type == ArticulationType.FIXED,
        f"expected fixed pad mount, got {pad_mount.articulation_type}",
    )

    with ctx.pose({stage: 0.0}):
        ctx.expect_overlap(carriage, beam, axes="xy", min_overlap=0.050, name="carriage stays centered on beam")
        ctx.expect_contact(carriage, beam, name="carriage remains mechanically supported by beam")
        ctx.expect_contact(pad, carriage, name="instrument pad seats on carriage pedestal")
        ctx.expect_overlap(pad, carriage, axes="xy", min_overlap=0.046, name="pad footprint is carried by shoe")
        ctx.expect_gap(
            pad,
            beam,
            axis="z",
            min_gap=0.018,
            max_gap=0.050,
            name="instrument pad rides well above the grounded beam",
        )

    with ctx.pose({stage: TRAVEL_HALF}):
        ctx.expect_gap(
            beam,
            carriage,
            axis="x",
            positive_elem="pos_stop",
            min_gap=0.0015,
            max_gap=0.0045,
            name="positive stop clears carriage at travel limit",
        )
        ctx.expect_within(carriage, beam, axes="y", margin=0.0, name="carriage remains between beam flanks at positive limit")

    with ctx.pose({stage: -TRAVEL_HALF}):
        ctx.expect_gap(
            carriage,
            beam,
            axis="x",
            negative_elem="neg_stop",
            min_gap=0.0015,
            max_gap=0.0045,
            name="negative stop clears carriage at travel limit",
        )
        ctx.expect_within(carriage, beam, axes="y", margin=0.0, name="carriage remains between beam flanks at negative limit")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
