from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
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
)


LEAF_LENGTH = 0.066
BRACKET_LEAF_WIDTH = 0.028
COVER_LEAF_WIDTH = 0.036
LEAF_THICKNESS = 0.0024
BARREL_RADIUS = 0.0048
END_KNUCKLE_LENGTH = 0.022
CENTER_KNUCKLE_LENGTH = 0.018
KNUCKLE_CENTER_OFFSET = 0.022
COVER_FLANGE_DEPTH = 0.012
LEAF_INNER_CLEARANCE = 0.0012
HINGE_WEB_DEPTH = 0.005

PLATE_Z_CENTER = -BARREL_RADIUS + LEAF_THICKNESS / 2.0

BRACKET_PLATE_MAX_X = -LEAF_INNER_CLEARANCE
BRACKET_PLATE_CENTER_X = BRACKET_PLATE_MAX_X - BRACKET_LEAF_WIDTH / 2.0
BRACKET_RELIEF_MAX_X = -(LEAF_INNER_CLEARANCE + HINGE_WEB_DEPTH)
BRACKET_MAIN_PLATE_WIDTH = BRACKET_LEAF_WIDTH - HINGE_WEB_DEPTH
BRACKET_MAIN_PLATE_CENTER_X = BRACKET_RELIEF_MAX_X - BRACKET_MAIN_PLATE_WIDTH / 2.0
BRACKET_WEB_CENTER_X = -(LEAF_INNER_CLEARANCE + HINGE_WEB_DEPTH / 2.0)

COVER_PLATE_MIN_X = LEAF_INNER_CLEARANCE
COVER_PLATE_CENTER_X = COVER_PLATE_MIN_X + COVER_LEAF_WIDTH / 2.0
COVER_PLATE_MAX_X = COVER_PLATE_MIN_X + COVER_LEAF_WIDTH
COVER_RELIEF_MIN_X = LEAF_INNER_CLEARANCE + HINGE_WEB_DEPTH
COVER_MAIN_PLATE_WIDTH = COVER_LEAF_WIDTH - HINGE_WEB_DEPTH
COVER_MAIN_PLATE_CENTER_X = COVER_RELIEF_MIN_X + COVER_MAIN_PLATE_WIDTH / 2.0
COVER_WEB_CENTER_X = LEAF_INNER_CLEARANCE + HINGE_WEB_DEPTH / 2.0

BRACKET_WEB_LENGTH = END_KNUCKLE_LENGTH + 0.004
COVER_WEB_LENGTH = CENTER_KNUCKLE_LENGTH + 0.004

FLANGE_X_CENTER = COVER_PLATE_MAX_X - LEAF_THICKNESS / 2.0
FLANGE_Z_CENTER = -0.0096

HINGE_OPEN_LIMIT = 1.55


def _aabb_center(aabb):
    if aabb is None:
        return None
    min_corner, max_corner = aabb
    return tuple((a + b) / 2.0 for a, b in zip(min_corner, max_corner))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="boxed_cover_hinge")

    model.material("zinc_plate", rgba=(0.73, 0.75, 0.78, 1.0))
    model.material("painted_cover", rgba=(0.88, 0.89, 0.86, 1.0))
    model.material("shadow_finish", rgba=(0.24, 0.25, 0.27, 1.0))

    bracket = model.part("bracket_leaf")
    bracket.visual(
        Box((BRACKET_MAIN_PLATE_WIDTH, LEAF_LENGTH, LEAF_THICKNESS)),
        origin=Origin(xyz=(BRACKET_MAIN_PLATE_CENTER_X, 0.0, PLATE_Z_CENTER)),
        material="zinc_plate",
        name="bracket_plate",
    )
    for name, y_center in (
        ("bracket_hinge_web_lower", -KNUCKLE_CENTER_OFFSET),
        ("bracket_hinge_web_upper", KNUCKLE_CENTER_OFFSET),
    ):
        bracket.visual(
            Box((HINGE_WEB_DEPTH, BRACKET_WEB_LENGTH, LEAF_THICKNESS)),
            origin=Origin(xyz=(BRACKET_WEB_CENTER_X, y_center, PLATE_Z_CENTER)),
            material="zinc_plate",
            name=name,
        )
    for name, y_center in (
        ("bracket_knuckle_lower", -KNUCKLE_CENTER_OFFSET),
        ("bracket_knuckle_upper", KNUCKLE_CENTER_OFFSET),
    ):
        bracket.visual(
            Cylinder(radius=BARREL_RADIUS, length=END_KNUCKLE_LENGTH),
            origin=Origin(xyz=(0.0, y_center, 0.0), rpy=(-1.5707963267948966, 0.0, 0.0)),
            material="zinc_plate",
            name=name,
        )
    bracket.inertial = Inertial.from_geometry(
        Box((BRACKET_LEAF_WIDTH + 2.0 * BARREL_RADIUS, LEAF_LENGTH, 2.0 * BARREL_RADIUS)),
        mass=0.09,
        origin=Origin(xyz=(-0.012, 0.0, -0.001)),
    )

    cover = model.part("cover_leaf")
    cover.visual(
        Box((COVER_MAIN_PLATE_WIDTH, LEAF_LENGTH, LEAF_THICKNESS)),
        origin=Origin(xyz=(COVER_MAIN_PLATE_CENTER_X, 0.0, PLATE_Z_CENTER)),
        material="painted_cover",
        name="cover_plate",
    )
    cover.visual(
        Box((HINGE_WEB_DEPTH, COVER_WEB_LENGTH, LEAF_THICKNESS)),
        origin=Origin(xyz=(COVER_WEB_CENTER_X, 0.0, PLATE_Z_CENTER)),
        material="painted_cover",
        name="cover_hinge_web",
    )
    cover.visual(
        Box((LEAF_THICKNESS, LEAF_LENGTH, COVER_FLANGE_DEPTH)),
        origin=Origin(xyz=(FLANGE_X_CENTER, 0.0, FLANGE_Z_CENTER)),
        material="painted_cover",
        name="cover_edge_flange",
    )
    cover.visual(
        Cylinder(radius=BARREL_RADIUS, length=CENTER_KNUCKLE_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-1.5707963267948966, 0.0, 0.0)),
        material="painted_cover",
        name="cover_knuckle",
    )
    cover.inertial = Inertial.from_geometry(
        Box((COVER_LEAF_WIDTH + LEAF_THICKNESS, LEAF_LENGTH, COVER_FLANGE_DEPTH + 2.0 * BARREL_RADIUS)),
        mass=0.12,
        origin=Origin(xyz=(0.019, 0.0, -0.007)),
    )

    model.articulation(
        "bracket_to_cover",
        ArticulationType.REVOLUTE,
        parent=bracket,
        child=cover,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=2.0,
            lower=0.0,
            upper=HINGE_OPEN_LIMIT,
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

    bracket = object_model.get_part("bracket_leaf")
    cover = object_model.get_part("cover_leaf")
    hinge = object_model.get_articulation("bracket_to_cover")

    limits = hinge.motion_limits
    ctx.check(
        "hinge axis and limits match cover hinge motion",
        hinge.axis == (0.0, -1.0, 0.0)
        and limits is not None
        and limits.lower == 0.0
        and limits.upper == HINGE_OPEN_LIMIT,
        details=f"axis={hinge.axis}, limits={limits}",
    )

    ctx.expect_overlap(
        bracket,
        cover,
        axes="y",
        min_overlap=0.055,
        name="leaves remain supported along the barrel length",
    )

    rest_center = _aabb_center(ctx.part_world_aabb(cover))
    with ctx.pose({hinge: HINGE_OPEN_LIMIT}):
        open_center = _aabb_center(ctx.part_world_aabb(cover))

    ctx.check(
        "cover rotates upward about the barrel axis",
        rest_center is not None
        and open_center is not None
        and open_center[2] > rest_center[2] + 0.015
        and open_center[0] < rest_center[0] - 0.01
        and abs(open_center[1] - rest_center[1]) < 1e-6,
        details=f"rest_center={rest_center}, open_center={open_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
