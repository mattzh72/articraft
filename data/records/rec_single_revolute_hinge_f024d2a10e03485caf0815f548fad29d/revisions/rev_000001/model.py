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


LEAF_HEIGHT = 0.09
LEAF_WIDTH = 0.036
LEAF_THICKNESS = 0.0025
LEAF_CORNER_RADIUS = 0.002

PIN_RADIUS = 0.00235
KNUCKLE_BORE_RADIUS = 0.0028
KNUCKLE_OUTER_RADIUS = 0.0057
LEAF_TO_KNUCKLE_OVERLAP = 0.0008

END_MARGIN = 0.005
CENTER_KNUCKLE_LENGTH = 0.03
KNUCKLE_GAP = 0.0015
OUTER_KNUCKLE_LENGTH = (
    LEAF_HEIGHT - (2.0 * END_MARGIN) - CENTER_KNUCKLE_LENGTH - (2.0 * KNUCKLE_GAP)
) / 2.0
OUTER_KNUCKLE_CENTER_OFFSET = (
    CENTER_KNUCKLE_LENGTH / 2.0 + KNUCKLE_GAP + OUTER_KNUCKLE_LENGTH / 2.0
)

PIN_SHAFT_LENGTH = 2.0 * (OUTER_KNUCKLE_CENTER_OFFSET + OUTER_KNUCKLE_LENGTH / 2.0)
PIN_HEAD_RADIUS = 0.0039
PIN_HEAD_THICKNESS = 0.0012
PIN_TAIL_RADIUS = 0.0035
PIN_TAIL_THICKNESS = 0.0008

MOUNT_HOLE_RADIUS = 0.0023
MOUNT_HOLE_X_FRACTION = 0.56
MOUNT_HOLE_ZS = (-0.028, 0.0, 0.028)


def _leaf_x_bounds(side: float) -> tuple[float, float]:
    if side < 0.0:
        inner = -(KNUCKLE_BORE_RADIUS + LEAF_TO_KNUCKLE_OVERLAP)
        return (inner - LEAF_WIDTH, inner)
    inner = KNUCKLE_BORE_RADIUS + LEAF_TO_KNUCKLE_OVERLAP
    return (inner, inner + LEAF_WIDTH)


def _hole_x(side: float) -> float:
    x_min, x_max = _leaf_x_bounds(side)
    return x_min + (x_max - x_min) * MOUNT_HOLE_X_FRACTION


def _leaf_plate(side: float) -> cq.Workplane:
    x_min, x_max = _leaf_x_bounds(side)
    plate = (
        cq.Workplane("XY")
        .box(LEAF_WIDTH, LEAF_THICKNESS, LEAF_HEIGHT, centered=(True, True, True))
        .translate(((x_min + x_max) / 2.0, 0.0, 0.0))
        .edges("|Y")
        .fillet(LEAF_CORNER_RADIUS)
    )
    hole_points = [(_hole_x(side), z_pos) for z_pos in MOUNT_HOLE_ZS]
    plate = (
        plate.faces(">Y")
        .workplane(centerOption="CenterOfMass")
        .pushPoints(hole_points)
        .hole(2.0 * MOUNT_HOLE_RADIUS)
    )
    return plate


def _knuckle_segment(z_center: float, length: float, side: float) -> cq.Workplane:
    ring = (
        cq.Workplane("XY")
        .circle(KNUCKLE_OUTER_RADIUS)
        .circle(KNUCKLE_BORE_RADIUS)
        .extrude(length / 2.0, both=True)
        .translate((0.0, 0.0, z_center))
    )
    half_space_width = 2.2 * KNUCKLE_OUTER_RADIUS
    cutoff = 0.0004
    if side < 0.0:
        keeper = cq.Workplane("XY").box(
            half_space_width,
            2.4 * KNUCKLE_OUTER_RADIUS,
            length + 0.001,
            centered=(False, True, True),
        ).translate((-half_space_width, 0.0, z_center))
        return ring.intersect(keeper)
    keeper = cq.Workplane("XY").box(
        half_space_width,
        2.4 * KNUCKLE_OUTER_RADIUS,
        length + 0.001,
        centered=(False, True, True),
    ).translate((-cutoff, 0.0, z_center))
    return ring.intersect(keeper)


def _grounded_half_shape() -> cq.Workplane:
    plate = _leaf_plate(side=-1.0)
    lower_knuckle = _knuckle_segment(-OUTER_KNUCKLE_CENTER_OFFSET, OUTER_KNUCKLE_LENGTH, -1.0)
    upper_knuckle = _knuckle_segment(OUTER_KNUCKLE_CENTER_OFFSET, OUTER_KNUCKLE_LENGTH, -1.0)
    return plate.union(lower_knuckle).union(upper_knuckle)


def _carried_half_shape() -> cq.Workplane:
    plate = _leaf_plate(side=1.0)
    center_knuckle = _knuckle_segment(0.0, CENTER_KNUCKLE_LENGTH, 1.0)
    return plate.union(center_knuckle)


def _pin_shape() -> cq.Workplane:
    shaft = cq.Workplane("XY").circle(PIN_RADIUS).extrude(PIN_SHAFT_LENGTH / 2.0, both=True)
    top_head = (
        cq.Workplane("XY")
        .workplane(offset=PIN_SHAFT_LENGTH / 2.0)
        .circle(PIN_HEAD_RADIUS)
        .extrude(PIN_HEAD_THICKNESS)
    )
    bottom_tail = (
        cq.Workplane("XY")
        .workplane(offset=-PIN_SHAFT_LENGTH / 2.0)
        .circle(PIN_TAIL_RADIUS)
        .extrude(-PIN_TAIL_THICKNESS)
    )
    return shaft.union(top_head).union(bottom_tail)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="knuckle_stack_hinge")

    model.material("hinge_leaf_steel", rgba=(0.60, 0.62, 0.66, 1.0))
    model.material("pin_steel", rgba=(0.74, 0.76, 0.79, 1.0))

    grounded = model.part("grounded_half")
    grounded.visual(
        mesh_from_cadquery(_grounded_half_shape(), "grounded_half"),
        material="hinge_leaf_steel",
        name="grounded_shell",
    )

    carried = model.part("carried_half")
    carried.visual(
        mesh_from_cadquery(_carried_half_shape(), "carried_half"),
        material="hinge_leaf_steel",
        name="carried_shell",
    )

    pin = model.part("hinge_pin")
    pin.visual(
        mesh_from_cadquery(_pin_shape(), "hinge_pin"),
        material="pin_steel",
        name="pin_shell",
    )

    model.articulation(
        "grounded_to_carried",
        ArticulationType.REVOLUTE,
        parent=grounded,
        child=carried,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.0,
            lower=0.0,
            upper=1.85,
        ),
    )
    model.articulation(
        "grounded_to_pin",
        ArticulationType.FIXED,
        parent=grounded,
        child=pin,
        origin=Origin(),
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
    grounded = object_model.get_part("grounded_half")
    carried = object_model.get_part("carried_half")
    pin = object_model.get_part("hinge_pin")
    hinge = object_model.get_articulation("grounded_to_carried")

    ctx.expect_origin_distance(
        grounded,
        carried,
        axes="xy",
        max_dist=1e-6,
        name="carried half is centered on the hinge pin axis",
    )
    ctx.expect_origin_distance(
        grounded,
        pin,
        axes="xy",
        max_dist=1e-6,
        name="pin stays coaxial with the grounded hinge half",
    )
    ctx.expect_overlap(
        grounded,
        carried,
        axes="z",
        min_overlap=LEAF_HEIGHT - 0.01,
        name="hinge halves share the same knuckle stack height",
    )
    ctx.expect_overlap(
        pin,
        carried,
        axes="z",
        min_overlap=CENTER_KNUCKLE_LENGTH - 0.002,
        name="pin spans the carried center knuckle",
    )

    limits = hinge.motion_limits
    ctx.check(
        "hinge articulation uses the pin axis",
        hinge.axis == (0.0, 0.0, 1.0)
        and limits is not None
        and limits.lower == 0.0
        and limits.upper is not None
        and limits.upper >= 1.7,
        details=f"axis={hinge.axis}, limits={limits}",
    )

    rest_aabb = ctx.part_world_aabb(carried)
    with ctx.pose({hinge: 1.15}):
        opened_aabb = ctx.part_world_aabb(carried)

    def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None):
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple((lo + hi) / 2.0 for lo, hi in zip(mins, maxs))

    rest_center = _aabb_center(rest_aabb)
    opened_center = _aabb_center(opened_aabb)
    ctx.check(
        "carried half swings from +X toward +Y about the pin",
        rest_center is not None
        and opened_center is not None
        and rest_center[0] > 0.015
        and abs(rest_center[1]) < 0.01
        and opened_center[1] > 0.015
        and opened_center[0] < rest_center[0] - 0.01,
        details=f"rest_center={rest_center}, opened_center={opened_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
