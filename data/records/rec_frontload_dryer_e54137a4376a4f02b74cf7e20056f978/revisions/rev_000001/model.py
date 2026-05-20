from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BODY_WIDTH = 0.70
BODY_DEPTH = 0.68
BODY_HEIGHT = 1.90
SHEET = 0.026

DRUM_RADIUS = 0.265
DRUM_INNER_RADIUS = 0.240
DRUM_LENGTH = 0.500
DRUM_CENTER = (0.0, -0.040, 1.170)
FRONT_Y = -BODY_DEPTH / 2.0

DOOR_RADIUS = 0.315
DOOR_WINDOW_RADIUS = 0.218
DOOR_THICKNESS = 0.055
DOOR_CENTER_Z = DRUM_CENTER[2]
DOOR_HINGE_X = 0.315
DOOR_HINGE_Y = FRONT_Y - 0.036


def _front_panel_mesh():
    """One continuous front sheet with a circular drum opening."""
    panel = (
        cq.Workplane("XY")
        .moveTo(0.0, BODY_HEIGHT / 2.0)
        .rect(BODY_WIDTH, BODY_HEIGHT)
        .moveTo(0.0, DRUM_CENTER[2])
        .circle(DRUM_RADIUS + 0.038)
        .extrude(SHEET)
        .translate((0.0, 0.0, -SHEET / 2.0))
    )
    return mesh_from_cadquery(panel, "front_panel", tolerance=0.001)


def _annular_disk_mesh(name: str, outer_radius: float, inner_radius: float, thickness: float):
    ring = (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(thickness)
        .translate((0.0, 0.0, -thickness / 2.0))
    )
    return mesh_from_cadquery(ring, name, tolerance=0.001)


def _drum_shell_mesh():
    shell = (
        cq.Workplane("XY")
        .circle(DRUM_RADIUS)
        .circle(DRUM_INNER_RADIUS)
        .extrude(DRUM_LENGTH)
        .translate((0.0, 0.0, -DRUM_LENGTH / 2.0))
    )
    rear = (
        cq.Workplane("XY")
        .circle(DRUM_RADIUS)
        .extrude(0.014)
        .translate((0.0, 0.0, DRUM_LENGTH / 2.0 - 0.014))
    )
    front_lip = (
        cq.Workplane("XY")
        .circle(DRUM_RADIUS + 0.010)
        .circle(DRUM_INNER_RADIUS - 0.010)
        .extrude(0.035)
        .translate((0.0, 0.0, -DRUM_LENGTH / 2.0))
    )
    return mesh_from_cadquery(shell.union(rear).union(front_lip), "drum_shell", tolerance=0.001)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="stacked_front_load_dryer")

    enamel = model.material("warm_white_enamel", rgba=(0.88, 0.90, 0.89, 1.0))
    shadow = model.material("black_recess", rgba=(0.015, 0.017, 0.020, 1.0))
    brushed = model.material("brushed_steel", rgba=(0.62, 0.65, 0.66, 1.0))
    dark_glass = model.material("smoked_blue_glass", rgba=(0.12, 0.18, 0.22, 0.48))
    rubber = model.material("dark_rubber_gasket", rgba=(0.035, 0.035, 0.034, 1.0))

    cabinet = model.part("cabinet")
    cabinet.visual(
        Box((SHEET, BODY_DEPTH, BODY_HEIGHT)),
        origin=Origin(xyz=(-BODY_WIDTH / 2.0 + SHEET / 2.0, 0.0, BODY_HEIGHT / 2.0)),
        material=enamel,
        name="side_panel_0",
    )
    cabinet.visual(
        Box((SHEET, BODY_DEPTH, BODY_HEIGHT)),
        origin=Origin(xyz=(BODY_WIDTH / 2.0 - SHEET / 2.0, 0.0, BODY_HEIGHT / 2.0)),
        material=enamel,
        name="side_panel_1",
    )
    cabinet.visual(
        Box((BODY_WIDTH, BODY_DEPTH, SHEET)),
        origin=Origin(xyz=(0.0, 0.0, SHEET / 2.0)),
        material=enamel,
        name="base_pan",
    )
    cabinet.visual(
        Box((BODY_WIDTH, BODY_DEPTH, SHEET)),
        origin=Origin(xyz=(0.0, 0.0, BODY_HEIGHT - SHEET / 2.0)),
        material=enamel,
        name="top_panel",
    )
    cabinet.visual(
        Box((BODY_WIDTH, SHEET, BODY_HEIGHT)),
        origin=Origin(xyz=(0.0, BODY_DEPTH / 2.0 - SHEET / 2.0, BODY_HEIGHT / 2.0)),
        material=enamel,
        name="rear_panel",
    )
    cabinet.visual(
        _front_panel_mesh(),
        origin=Origin(xyz=(0.0, FRONT_Y + SHEET / 2.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=enamel,
        name="front_panel",
    )
    cabinet.visual(
        _annular_disk_mesh("front_black_gasket", DRUM_RADIUS + 0.045, DRUM_RADIUS + 0.012, 0.010),
        origin=Origin(xyz=(0.0, FRONT_Y - 0.002, DOOR_CENTER_Z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="front_gasket",
    )
    cabinet.visual(
        Box((BODY_WIDTH - 0.050, 0.012, 0.014)),
        origin=Origin(xyz=(0.0, FRONT_Y + 0.003, 0.820)),
        material=shadow,
        name="stack_seam",
    )
    cabinet.visual(
        Box((0.500, 0.010, 0.245)),
        origin=Origin(xyz=(0.0, FRONT_Y + 0.003, 0.445)),
        material=Material("slightly_darker_panel", rgba=(0.78, 0.80, 0.79, 1.0)),
        name="lower_service_panel",
    )
    cabinet.visual(
        Box((0.025, 0.050, 0.660)),
        origin=Origin(xyz=(BODY_WIDTH / 2.0 + 0.0095, FRONT_Y - 0.020, DOOR_CENTER_Z)),
        material=brushed,
        name="right_hinge_mount",
    )
    cabinet.visual(
        Cylinder(radius=0.022, length=0.030),
        origin=Origin(xyz=(0.0, BODY_DEPTH / 2.0 - SHEET - 0.010, DOOR_CENTER_Z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=brushed,
        name="rear_bearing",
    )

    drum = model.part("drum")
    drum.visual(_drum_shell_mesh(), origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)), material=brushed, name="drum_shell")
    for index, theta in enumerate((0.0, 2.0 * math.pi / 3.0, -2.0 * math.pi / 3.0)):
        radial = DRUM_INNER_RADIUS - 0.002
        drum.visual(
            Box((0.075, DRUM_LENGTH - 0.080, 0.035)),
            origin=Origin(
                xyz=(radial * math.sin(theta), -0.010, radial * math.cos(theta)),
                rpy=(0.0, theta, 0.0),
            ),
            material=brushed,
            name=f"tumble_lifter_{index}",
        )
    drum.visual(
        Cylinder(radius=0.018, length=0.088),
        origin=Origin(xyz=(0.0, DRUM_LENGTH / 2.0 + 0.035, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=brushed,
        name="axle_stub",
    )

    door = model.part("door")
    door.visual(
        _annular_disk_mesh("door_outer_ring", DOOR_RADIUS, DOOR_WINDOW_RADIUS, DOOR_THICKNESS),
        origin=Origin(xyz=(-DOOR_RADIUS, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=enamel,
        name="outer_ring",
    )
    door.visual(
        _annular_disk_mesh("door_inner_gasket", DOOR_WINDOW_RADIUS + 0.020, DOOR_WINDOW_RADIUS - 0.010, 0.018),
        origin=Origin(xyz=(-DOOR_RADIUS, -0.004, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="window_gasket",
    )
    door.visual(
        Cylinder(radius=DOOR_WINDOW_RADIUS, length=0.012),
        origin=Origin(xyz=(-DOOR_RADIUS, -0.010, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_glass,
        name="glass_window",
    )
    door.visual(
        Box((0.050, 0.030, 0.650)),
        origin=Origin(xyz=(-0.010, 0.0, 0.0)),
        material=enamel,
        name="hinge_leaf",
    )
    door.visual(
        Box((0.036, 0.040, 0.210)),
        origin=Origin(xyz=(-2.0 * DOOR_RADIUS + 0.018, -0.006, 0.0)),
        material=rubber,
        name="pull_grip",
    )
    for index, z in enumerate((-0.250, 0.0, 0.250)):
        door.visual(
            Cylinder(radius=0.012, length=0.120),
            origin=Origin(xyz=(0.020, 0.0, z)),
            material=brushed,
            name=f"hinge_knuckle_{index}",
        )

    model.articulation(
        "cabinet_to_drum",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=drum,
        origin=Origin(xyz=DRUM_CENTER),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=8.0),
    )
    model.articulation(
        "cabinet_to_door",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=door,
        origin=Origin(xyz=(DOOR_HINGE_X, DOOR_HINGE_Y, DOOR_CENTER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.85, effort=18.0, velocity=1.2),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.
    # If overlap QC reports an intersection, classify it first: intentional
    # embeddings or nested fits should get a scoped allowance; unintended
    # collisions should be fixed in geometry, support, mount, or pose.

    cabinet = object_model.get_part("cabinet")
    drum = object_model.get_part("drum")
    door = object_model.get_part("door")
    door_hinge = object_model.get_articulation("cabinet_to_door")
    drum_axle = object_model.get_articulation("cabinet_to_drum")

    ctx.expect_within(
        drum,
        cabinet,
        axes="xz",
        inner_elem="drum_shell",
        outer_elem="front_gasket",
        margin=0.050,
        name="drum is centered behind the circular front opening",
    )
    ctx.expect_gap(
        cabinet,
        door,
        axis="y",
        positive_elem="front_panel",
        negative_elem="outer_ring",
        min_gap=0.004,
        max_gap=0.025,
        name="closed door sits just proud of the front panel",
    )
    ctx.expect_overlap(
        door,
        cabinet,
        axes="xz",
        elem_a="outer_ring",
        elem_b="front_gasket",
        min_overlap=0.18,
        name="round door covers the drum opening",
    )

    closed_aabb = ctx.part_world_aabb(door)
    with ctx.pose({door_hinge: 1.20}):
        opened_aabb = ctx.part_world_aabb(door)
    ctx.check(
        "right hinge swings the door outward",
        closed_aabb is not None
        and opened_aabb is not None
        and opened_aabb[0][1] < closed_aabb[0][1] - 0.10,
        details=f"closed={closed_aabb}, opened={opened_aabb}",
    )

    rest_pos = ctx.part_world_position(drum)
    with ctx.pose({drum_axle: math.pi / 2.0}):
        spun_pos = ctx.part_world_position(drum)
    ctx.check(
        "drum rotates on a fixed front-to-back axle",
        rest_pos is not None and spun_pos is not None and all(abs(a - b) < 1e-6 for a, b in zip(rest_pos, spun_pos)),
        details=f"rest={rest_pos}, spun={spun_pos}",
    )

    return ctx.report()


object_model = build_object_model()
