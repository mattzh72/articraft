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


HINGE_HEIGHT = 0.260
LEAF_WIDTH = 0.105
LEAF_THICKNESS = 0.008
LEAF_CORNER_RADIUS = 0.012

BARREL_RADIUS = 0.014
WRAP_OVERLAP = 0.004

KNUCKLE_GAP = 0.0
KNUCKLE_LENGTH = (HINGE_HEIGHT - 4.0 * KNUCKLE_GAP) / 5.0

MOUNT_HOLE_DIAMETER = 0.010
HOLE_X_OFFSET = 0.018
HOLE_Z_OFFSETS = (-0.085, 0.0, 0.085)


def _leaf_plate(sign: float) -> cq.Workplane:
    plate_width = LEAF_WIDTH + WRAP_OVERLAP
    center_x = sign * (BARREL_RADIUS + LEAF_WIDTH / 2.0 - WRAP_OVERLAP / 2.0)

    hole_points = [
        (x, z)
        for z in HOLE_Z_OFFSETS
        for x in (-HOLE_X_OFFSET, HOLE_X_OFFSET)
    ]

    plate = (
        cq.Workplane("XY")
        .box(plate_width, LEAF_THICKNESS, HINGE_HEIGHT)
        .translate((center_x, 0.0, 0.0))
        .edges("|Y")
        .fillet(LEAF_CORNER_RADIUS)
        .faces(">Y")
        .workplane(centerOption="CenterOfMass")
        .pushPoints(hole_points)
        .hole(MOUNT_HOLE_DIAMETER)
    )
    return plate


def _knuckle_segment(sign: float, length: float, z_center: float) -> cq.Workplane:
    barrel = (
        cq.Workplane("XY")
        .circle(BARREL_RADIUS)
        .extrude(length / 2.0, both=True)
        .translate((0.0, 0.0, z_center))
    )
    half_mask = (
        cq.Workplane("XY")
        .box(BARREL_RADIUS, 2.0 * BARREL_RADIUS, length)
        .translate((sign * BARREL_RADIUS / 2.0, 0.0, z_center))
    )
    return barrel.intersect(half_mask)


def _knuckle_centers(indices: tuple[int, ...]) -> list[float]:
    start = -HINGE_HEIGHT / 2.0 + KNUCKLE_LENGTH / 2.0
    pitch = KNUCKLE_LENGTH + KNUCKLE_GAP
    return [start + i * pitch for i in indices]


def _build_back_leaf_shape() -> cq.Workplane:
    body = _leaf_plate(sign=-1.0)
    for z_center in _knuckle_centers((0, 2, 4)):
        body = body.union(_knuckle_segment(-1.0, KNUCKLE_LENGTH, z_center))
    return body


def _build_moving_leaf_shape() -> cq.Workplane:
    body = _leaf_plate(sign=1.0)
    for z_center in _knuckle_centers((1, 3)):
        body = body.union(_knuckle_segment(1.0, KNUCKLE_LENGTH, z_center))
    return body


def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple((mins[i] + maxs[i]) / 2.0 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="equipment_access_hinge")

    dark_steel = model.material("dark_steel", rgba=(0.34, 0.36, 0.39, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.54, 0.56, 0.58, 1.0))

    back_leaf = model.part("back_leaf")
    back_leaf.visual(
        mesh_from_cadquery(_build_back_leaf_shape(), "back_leaf_body"),
        material=dark_steel,
        name="back_leaf_body",
    )
    back_leaf.inertial = Inertial.from_geometry(
        Box((LEAF_WIDTH + 2.0 * BARREL_RADIUS, 2.0 * BARREL_RADIUS, HINGE_HEIGHT)),
        mass=2.7,
        origin=Origin(xyz=(-LEAF_WIDTH / 2.0, 0.0, 0.0)),
    )

    moving_leaf = model.part("moving_leaf")
    moving_leaf.visual(
        mesh_from_cadquery(_build_moving_leaf_shape(), "moving_leaf_body"),
        material=satin_steel,
        name="moving_leaf_body",
    )
    moving_leaf.inertial = Inertial.from_geometry(
        Box((LEAF_WIDTH + BARREL_RADIUS, 2.0 * BARREL_RADIUS, HINGE_HEIGHT)),
        mass=2.2,
        origin=Origin(xyz=(LEAF_WIDTH / 2.0, 0.0, 0.0)),
    )

    model.articulation(
        "leaf_hinge",
        ArticulationType.REVOLUTE,
        parent=back_leaf,
        child=moving_leaf,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=1.2,
            lower=0.0,
            upper=1.57,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    back_leaf = object_model.get_part("back_leaf")
    moving_leaf = object_model.get_part("moving_leaf")
    hinge = object_model.get_articulation("leaf_hinge")

    ctx.check(
        "hinge axis follows barrel",
        hinge.axis == (0.0, 0.0, 1.0)
        and hinge.motion_limits is not None
        and hinge.motion_limits.lower == 0.0
        and hinge.motion_limits.upper is not None
        and 1.4 <= hinge.motion_limits.upper <= 1.7,
        details=f"axis={hinge.axis}, limits={hinge.motion_limits}",
    )

    ctx.expect_origin_distance(
        back_leaf,
        moving_leaf,
        axes="xy",
        max_dist=1e-6,
        name="leaf frames share the hinge axis",
    )

    with ctx.pose({hinge: 0.0}):
        back_rest = _aabb_center(ctx.part_world_aabb(back_leaf))
        moving_rest = _aabb_center(ctx.part_world_aabb(moving_leaf))
        ctx.check(
            "open pose places leaves on opposite sides of the barrel",
            back_rest is not None
            and moving_rest is not None
            and back_rest[0] < -0.03
            and moving_rest[0] > 0.03
            and abs(back_rest[1]) < 0.01
            and abs(moving_rest[1]) < 0.01,
            details=f"back_center={back_rest}, moving_center={moving_rest}",
        )

    moving_rest = None
    moving_folded = None
    with ctx.pose({hinge: 0.0}):
        moving_rest = _aabb_center(ctx.part_world_aabb(moving_leaf))
    with ctx.pose({hinge: 1.2}):
        moving_folded = _aabb_center(ctx.part_world_aabb(moving_leaf))

    ctx.check(
        "moving leaf swings outward around the barrel axis",
        moving_rest is not None
        and moving_folded is not None
        and moving_folded[1] > moving_rest[1] + 0.04
        and moving_folded[0] < moving_rest[0] - 0.02,
        details=f"rest_center={moving_rest}, folded_center={moving_folded}",
    )

    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
