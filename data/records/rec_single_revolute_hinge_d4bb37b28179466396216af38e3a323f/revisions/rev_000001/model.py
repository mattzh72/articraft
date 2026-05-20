from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


PLATE_THICKNESS = 0.012
PLATE_WIDTH = 0.105
PLATE_HEIGHT = 0.160
PLATE_CENTER_X = -0.040

BARREL_RADIUS = 0.017
FIXED_KNUCKLE_LENGTH = 0.040
MOVING_KNUCKLE_LENGTH = 0.040
KNUCKLE_CENTER_OFFSET_Z = 0.040

LUG_LENGTH = 0.036
LUG_WIDTH = 0.026
LUG_HEIGHT = 0.036
BRACKET_SUPPORT_CENTER_Y = -0.020

LEAF_LENGTH = 0.180
LEAF_THICKNESS = 0.008
LEAF_HEIGHT = 0.126
LEAF_ROOT_BLOCK_LENGTH = 0.028
LEAF_ROOT_BLOCK_THICKNESS = 0.026
LEAF_ROOT_BLOCK_HEIGHT = 0.030
LEAF_PLATE_CENTER_X = BARREL_RADIUS + LEAF_LENGTH / 2.0
LEAF_OFFSET_Y = 0.024


def _make_side_gusset() -> cq.Workplane:
    return (
        cq.Workplane("XZ")
        .polyline(
            [
                (PLATE_CENTER_X + PLATE_THICKNESS / 2.0, -0.070),
                (PLATE_CENTER_X + PLATE_THICKNESS / 2.0, 0.070),
                (-0.008, 0.044),
                (-0.008, -0.044),
            ]
        )
        .close()
        .extrude(0.012)
        .translate((0.0, -0.034, 0.0))
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="gusset_backed_industrial_hinge")

    bracket_finish = model.material("bracket_finish", rgba=(0.27, 0.30, 0.33, 1.0))
    leaf_finish = model.material("leaf_finish", rgba=(0.64, 0.67, 0.71, 1.0))

    bracket = model.part("ground_bracket")
    bracket.visual(
        Box((PLATE_THICKNESS, PLATE_WIDTH, PLATE_HEIGHT)),
        material=bracket_finish,
        origin=Origin(xyz=(PLATE_CENTER_X, 0.0, 0.0)),
        name="bracket_plate",
    )
    bracket.visual(
        mesh_from_cadquery(_make_side_gusset(), "side_gusset"),
        material=bracket_finish,
        origin=Origin(),
        name="side_rib",
    )
    bracket.visual(
        Box((LUG_LENGTH, LUG_WIDTH, LUG_HEIGHT)),
        material=bracket_finish,
        origin=Origin(
            xyz=(
                -(LUG_LENGTH / 2.0) + 0.002,
                BRACKET_SUPPORT_CENTER_Y,
                KNUCKLE_CENTER_OFFSET_Z,
            )
        ),
        name="top_lug",
    )
    bracket.visual(
        Box((LUG_LENGTH, LUG_WIDTH, LUG_HEIGHT)),
        material=bracket_finish,
        origin=Origin(
            xyz=(
                -(LUG_LENGTH / 2.0) + 0.002,
                BRACKET_SUPPORT_CENTER_Y,
                -KNUCKLE_CENTER_OFFSET_Z,
            )
        ),
        name="bottom_lug",
    )
    bracket.visual(
        Cylinder(radius=BARREL_RADIUS, length=FIXED_KNUCKLE_LENGTH),
        material=bracket_finish,
        origin=Origin(xyz=(0.0, 0.0, KNUCKLE_CENTER_OFFSET_Z)),
        name="top_knuckle",
    )
    bracket.visual(
        Cylinder(radius=BARREL_RADIUS, length=FIXED_KNUCKLE_LENGTH),
        material=bracket_finish,
        origin=Origin(xyz=(0.0, 0.0, -KNUCKLE_CENTER_OFFSET_Z)),
        name="bottom_knuckle",
    )

    leaf = model.part("moving_leaf")
    leaf.visual(
        Box((LEAF_LENGTH, LEAF_THICKNESS, LEAF_HEIGHT)),
        material=leaf_finish,
        origin=Origin(xyz=(LEAF_PLATE_CENTER_X, LEAF_OFFSET_Y, 0.0)),
        name="leaf_plate",
    )
    leaf.visual(
        Box(
            (
                LEAF_ROOT_BLOCK_LENGTH,
                LEAF_ROOT_BLOCK_THICKNESS,
                LEAF_ROOT_BLOCK_HEIGHT,
            )
        ),
        material=leaf_finish,
        origin=Origin(xyz=(LEAF_ROOT_BLOCK_LENGTH / 2.0, 0.015, 0.0)),
        name="root_arm",
    )
    leaf.visual(
        Cylinder(radius=BARREL_RADIUS, length=MOVING_KNUCKLE_LENGTH),
        material=leaf_finish,
        origin=Origin(),
        name="moving_knuckle",
    )

    model.articulation(
        "barrel_hinge",
        ArticulationType.REVOLUTE,
        parent=bracket,
        child=leaf,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=1.4,
            lower=0.0,
            upper=1.55,
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

    bracket = object_model.get_part("ground_bracket")
    leaf = object_model.get_part("moving_leaf")
    hinge = object_model.get_articulation("barrel_hinge")
    hinge_limits = hinge.motion_limits

    ctx.check(
        "single revolute barrel articulation",
        hinge.articulation_type == ArticulationType.REVOLUTE
        and hinge.axis == (0.0, 0.0, 1.0)
        and hinge_limits is not None
        and hinge_limits.lower == 0.0
        and hinge_limits.upper is not None
        and hinge_limits.upper >= 1.5,
        details=f"type={hinge.articulation_type}, axis={hinge.axis}, limits={hinge_limits}",
    )

    with ctx.pose({hinge: 0.0}):
        closed_leaf_aabb = ctx.part_element_world_aabb(leaf, elem="leaf_plate")
        ctx.expect_overlap(
            leaf,
            bracket,
            axes="z",
            min_overlap=0.120,
            name="leaf stays aligned with bracket height",
        )
        ctx.check(
            "closed leaf extends well outboard of the barrel",
            closed_leaf_aabb is not None and closed_leaf_aabb[1][0] >= 0.180,
            details=f"closed_leaf_aabb={closed_leaf_aabb}",
        )

    with ctx.pose({hinge: hinge_limits.upper if hinge_limits is not None else 1.90}):
        open_leaf_aabb = ctx.part_element_world_aabb(leaf, elem="leaf_plate")

    ctx.check(
        "leaf swings toward positive y when opened",
        closed_leaf_aabb is not None
        and open_leaf_aabb is not None
        and open_leaf_aabb[1][1] > closed_leaf_aabb[1][1] + 0.090,
        details=f"closed_leaf_aabb={closed_leaf_aabb}, open_leaf_aabb={open_leaf_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
