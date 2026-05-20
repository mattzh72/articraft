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


OUTER_WIDTH = 0.84
OUTER_HEIGHT = 0.64
FRAME_DEPTH = 0.05
STILE_THICKNESS = 0.07
RAIL_THICKNESS = 0.06
OPENING_WIDTH = OUTER_WIDTH - 2.0 * STILE_THICKNESS
OPENING_HEIGHT = OUTER_HEIGHT - 2.0 * RAIL_THICKNESS

VANE_COUNT = 7
VANE_DEPTH = 0.034
VANE_THICKNESS = 0.008
PIN_RADIUS = 0.005
PIN_CLEARANCE = 0.0007
PIN_HEAD_RADIUS = 0.008
PIN_HEAD_LENGTH = 0.008
BODY_OFFSET_FROM_JOINT = 0.045
OUTBOARD_PIN_PROJECTION = STILE_THICKNESS / 2.0 + 0.006
COLLAR_RADIUS = 0.0072
COLLAR_LENGTH = 0.012

LEFT_HOLE_X = -OUTER_WIDTH / 2.0 + STILE_THICKNESS / 2.0
RIGHT_HOLE_X = OUTER_WIDTH / 2.0 - STILE_THICKNESS / 2.0
HOLE_SPAN = RIGHT_HOLE_X - LEFT_HOLE_X

BODY_START_X = BODY_OFFSET_FROM_JOINT
BODY_END_X = HOLE_SPAN - BODY_OFFSET_FROM_JOINT
BODY_LENGTH = BODY_END_X - BODY_START_X
VANE_MIN_X = -(STILE_THICKNESS / 2.0 + PIN_HEAD_LENGTH)
VANE_MAX_X = HOLE_SPAN + STILE_THICKNESS / 2.0 + PIN_HEAD_LENGTH
VANE_TOTAL_LENGTH = VANE_MAX_X - VANE_MIN_X
VANE_INERTIAL_CENTER_X = 0.5 * (VANE_MIN_X + VANE_MAX_X)


def _vane_z_positions() -> list[float]:
    usable_height = OPENING_HEIGHT - 0.10
    top_z = usable_height / 2.0
    bottom_z = -usable_height / 2.0
    if VANE_COUNT == 1:
        return [0.0]
    step = (top_z - bottom_z) / (VANE_COUNT - 1)
    return [top_z - i * step for i in range(VANE_COUNT)]


def _frame_shape() -> cq.Workplane:
    frame = (
        cq.Workplane("XZ")
        .rect(OUTER_WIDTH, OUTER_HEIGHT)
        .extrude(FRAME_DEPTH / 2.0, both=True)
    )
    opening = (
        cq.Workplane("XZ")
        .rect(OPENING_WIDTH, OPENING_HEIGHT)
        .extrude(FRAME_DEPTH / 2.0 + 0.01, both=True)
    )
    frame = frame.cut(opening)

    hole_radius = PIN_RADIUS + PIN_CLEARANCE
    for z_pos in _vane_z_positions():
        for x_pos in (LEFT_HOLE_X, RIGHT_HOLE_X):
            cutter = (
                cq.Workplane("YZ")
                .center(0.0, z_pos)
                .circle(hole_radius)
                .extrude(STILE_THICKNESS + 0.02, both=True)
                .translate((x_pos, 0.0, 0.0))
            )
            frame = frame.cut(cutter)

    return frame.edges("|Y").fillet(0.003)


def _vane_shape() -> cq.Workplane:
    body = (
        cq.Workplane("YZ")
        .rect(VANE_DEPTH, VANE_THICKNESS)
        .extrude(BODY_LENGTH)
        .translate((BODY_START_X, 0.0, 0.0))
    )

    left_shaft = (
        cq.Workplane("YZ")
        .circle(PIN_RADIUS)
        .extrude(BODY_START_X + OUTBOARD_PIN_PROJECTION)
        .translate((-OUTBOARD_PIN_PROJECTION, 0.0, 0.0))
    )
    right_shaft = (
        cq.Workplane("YZ")
        .circle(PIN_RADIUS)
        .extrude(HOLE_SPAN + OUTBOARD_PIN_PROJECTION - BODY_END_X)
        .translate((BODY_END_X, 0.0, 0.0))
    )

    left_head = (
        cq.Workplane("YZ")
        .circle(PIN_HEAD_RADIUS)
        .extrude(PIN_HEAD_LENGTH)
        .translate((-(STILE_THICKNESS / 2.0 + PIN_HEAD_LENGTH), 0.0, 0.0))
    )
    right_head = (
        cq.Workplane("YZ")
        .circle(PIN_HEAD_RADIUS)
        .extrude(PIN_HEAD_LENGTH)
        .translate((HOLE_SPAN + STILE_THICKNESS / 2.0, 0.0, 0.0))
    )

    left_collar = (
        cq.Workplane("YZ")
        .circle(COLLAR_RADIUS)
        .extrude(COLLAR_LENGTH)
        .translate((BODY_START_X - COLLAR_LENGTH / 2.0, 0.0, 0.0))
    )
    right_collar = (
        cq.Workplane("YZ")
        .circle(COLLAR_RADIUS)
        .extrude(COLLAR_LENGTH)
        .translate((BODY_END_X - COLLAR_LENGTH / 2.0, 0.0, 0.0))
    )

    return (
        body.union(left_shaft)
        .union(right_shaft)
        .union(left_head)
        .union(right_head)
        .union(left_collar)
        .union(right_collar)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="inspection_shutter_array")

    model.material("frame_finish", rgba=(0.20, 0.22, 0.24, 1.0))
    model.material("vane_finish", rgba=(0.72, 0.75, 0.78, 1.0))

    frame_mesh = mesh_from_cadquery(_frame_shape(), "inspection_shutter_frame")
    vane_mesh = mesh_from_cadquery(_vane_shape(), "inspection_shutter_vane")

    frame = model.part("frame")
    frame.visual(frame_mesh, material="frame_finish", name="frame_shell")
    frame.inertial = Inertial.from_geometry(
        Box((OUTER_WIDTH, FRAME_DEPTH, OUTER_HEIGHT)),
        mass=8.0,
    )

    for index, z_pos in enumerate(_vane_z_positions(), start=1):
        vane = model.part(f"vane_{index}")
        vane.visual(vane_mesh, material="vane_finish", name="vane_shell")
        vane.inertial = Inertial.from_geometry(
            Box((VANE_TOTAL_LENGTH, VANE_DEPTH, VANE_THICKNESS)),
            mass=0.55,
            origin=Origin(xyz=(VANE_INERTIAL_CENTER_X, 0.0, 0.0)),
        )
        model.articulation(
            f"frame_to_vane_{index}",
            ArticulationType.REVOLUTE,
            parent=frame,
            child=vane,
            origin=Origin(xyz=(LEFT_HOLE_X, 0.0, z_pos)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=4.0,
                velocity=2.0,
                lower=-0.85,
                upper=0.85,
            ),
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

    frame = object_model.get_part("frame")
    vane_parts = [object_model.get_part(f"vane_{index}") for index in range(1, VANE_COUNT + 1)]
    vane_joints = [
        object_model.get_articulation(f"frame_to_vane_{index}")
        for index in range(1, VANE_COUNT + 1)
    ]

    ctx.check("frame is present", frame is not None, details="Missing fixed outer frame.")

    for vane, joint in zip(vane_parts, vane_joints):
        limits = joint.motion_limits
        ctx.check(
            f"{vane.name} exists",
            vane is not None,
            details=f"Missing part {vane.name}.",
        )
        ctx.check(
            f"{joint.name} uses the vane long-axis pivot",
            tuple(round(value, 6) for value in joint.axis) == (1.0, 0.0, 0.0),
            details=f"axis={joint.axis}",
        )
        ctx.check(
            f"{joint.name} has bidirectional vane travel",
            limits is not None
            and limits.lower is not None
            and limits.upper is not None
            and limits.lower < 0.0 < limits.upper,
            details=f"limits={limits}",
        )
        ctx.expect_within(
            vane,
            frame,
            axes="yz",
            margin=0.0,
            name=f"{vane.name} stays inside the frame envelope",
        )

    vane_pitch = (_vane_z_positions()[0] - _vane_z_positions()[-1]) / (VANE_COUNT - 1)
    for upper_vane, lower_vane in zip(vane_parts, vane_parts[1:]):
        ctx.expect_origin_gap(
            upper_vane,
            lower_vane,
            axis="z",
            min_gap=vane_pitch - 0.002,
            max_gap=vane_pitch + 0.002,
            name=f"{upper_vane.name} and {lower_vane.name} keep even spacing",
        )

    def _axis_extent(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None, axis: int) -> float | None:
        if aabb is None:
            return None
        return aabb[1][axis] - aabb[0][axis]

    active_vane = vane_parts[2]
    passive_vane = vane_parts[3]
    active_joint = vane_joints[2]

    active_rest_aabb = ctx.part_world_aabb(active_vane)
    passive_rest_aabb = ctx.part_world_aabb(passive_vane)
    with ctx.pose({active_joint: 0.65}):
        active_open_aabb = ctx.part_world_aabb(active_vane)
        passive_open_aabb = ctx.part_world_aabb(passive_vane)

    active_rest_z = _axis_extent(active_rest_aabb, 2)
    active_open_z = _axis_extent(active_open_aabb, 2)
    passive_rest_z = _axis_extent(passive_rest_aabb, 2)
    passive_open_z = _axis_extent(passive_open_aabb, 2)

    ctx.check(
        "a driven vane changes section when rotated",
        active_rest_z is not None
        and active_open_z is not None
        and active_open_z > active_rest_z + 0.012,
        details=f"rest_z={active_rest_z}, open_z={active_open_z}",
    )
    ctx.check(
        "neighbor vane stays unchanged when another vane rotates",
        passive_rest_z is not None
        and passive_open_z is not None
        and abs(passive_open_z - passive_rest_z) < 1e-4,
        details=f"rest_z={passive_rest_z}, open_z={passive_open_z}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
