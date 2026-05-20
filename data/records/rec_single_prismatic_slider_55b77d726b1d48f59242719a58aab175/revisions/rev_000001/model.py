from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


RAIL_LENGTH = 0.48
RAIL_WIDTH = 0.09
RAIL_HEIGHT = 0.042

SHUTTLE_LENGTH = 0.11
SHUTTLE_OUTER_WIDTH = 0.118
SHUTTLE_SIDE_THICKNESS = 0.012
SHUTTLE_BOTTOM_CLEARANCE = 0.0025
SHUTTLE_ROOF_GAP = 0.0
SHUTTLE_SIDE_HEIGHT = RAIL_HEIGHT + SHUTTLE_ROOF_GAP - SHUTTLE_BOTTOM_CLEARANCE
SHUTTLE_BRIDGE_THICKNESS = 0.033
SHUTTLE_TOTAL_HEIGHT = SHUTTLE_SIDE_HEIGHT + SHUTTLE_BRIDGE_THICKNESS
SHUTTLE_ROOF_Z = SHUTTLE_BOTTOM_CLEARANCE + SHUTTLE_SIDE_HEIGHT

FRONT_PLATE_THICKNESS = 0.012
FRONT_PLATE_SIDE = 0.06

SLIDE_TRAVEL = 0.15


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="box_rail_shuttle")

    model.material("rail_black", rgba=(0.20, 0.22, 0.25, 1.0))
    model.material("shuttle_gray", rgba=(0.71, 0.74, 0.78, 1.0))
    model.material("plate_silver", rgba=(0.86, 0.88, 0.91, 1.0))

    rail = model.part("rail")
    rail.visual(
        Box((RAIL_LENGTH, RAIL_WIDTH, RAIL_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, RAIL_HEIGHT / 2.0)),
        material="rail_black",
        name="rail_body",
    )
    rail.inertial = Inertial.from_geometry(
        Box((RAIL_LENGTH, RAIL_WIDTH, RAIL_HEIGHT)),
        mass=4.2,
        origin=Origin(xyz=(0.0, 0.0, RAIL_HEIGHT / 2.0)),
    )

    shuttle = model.part("shuttle")
    shuttle.visual(
        Box((SHUTTLE_LENGTH, SHUTTLE_OUTER_WIDTH, SHUTTLE_BRIDGE_THICKNESS)),
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                SHUTTLE_ROOF_Z + SHUTTLE_BRIDGE_THICKNESS / 2.0,
            )
        ),
        material="shuttle_gray",
        name="top_bridge",
    )
    shuttle.visual(
        Box((SHUTTLE_LENGTH, SHUTTLE_SIDE_THICKNESS, SHUTTLE_SIDE_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                SHUTTLE_OUTER_WIDTH / 2.0 - SHUTTLE_SIDE_THICKNESS / 2.0,
                SHUTTLE_BOTTOM_CLEARANCE + SHUTTLE_SIDE_HEIGHT / 2.0,
            )
        ),
        material="shuttle_gray",
        name="left_skirt",
    )
    shuttle.visual(
        Box((SHUTTLE_LENGTH, SHUTTLE_SIDE_THICKNESS, SHUTTLE_SIDE_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                -SHUTTLE_OUTER_WIDTH / 2.0 + SHUTTLE_SIDE_THICKNESS / 2.0,
                SHUTTLE_BOTTOM_CLEARANCE + SHUTTLE_SIDE_HEIGHT / 2.0,
            )
        ),
        material="shuttle_gray",
        name="right_skirt",
    )
    shuttle.visual(
        Box((FRONT_PLATE_THICKNESS, FRONT_PLATE_SIDE, FRONT_PLATE_SIDE)),
        origin=Origin(
            xyz=(
                SHUTTLE_LENGTH / 2.0 + FRONT_PLATE_THICKNESS / 2.0,
                0.0,
                SHUTTLE_ROOF_Z + FRONT_PLATE_SIDE / 2.0,
            )
        ),
        material="plate_silver",
        name="front_plate",
    )
    shuttle.inertial = Inertial.from_geometry(
        Box(
            (
                SHUTTLE_LENGTH + FRONT_PLATE_THICKNESS,
                SHUTTLE_OUTER_WIDTH,
                max(SHUTTLE_TOTAL_HEIGHT, FRONT_PLATE_SIDE),
            )
        ),
        mass=1.1,
        origin=Origin(
            xyz=(
                FRONT_PLATE_THICKNESS * 0.25,
                0.0,
                SHUTTLE_BOTTOM_CLEARANCE + max(SHUTTLE_TOTAL_HEIGHT, FRONT_PLATE_SIDE) / 2.0,
            )
        ),
    )

    model.articulation(
        "rail_to_shuttle",
        ArticulationType.PRISMATIC,
        parent=rail,
        child=shuttle,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=-SLIDE_TRAVEL,
            upper=SLIDE_TRAVEL,
            effort=180.0,
            velocity=0.35,
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

    rail = object_model.get_part("rail")
    shuttle = object_model.get_part("shuttle")
    slide = object_model.get_articulation("rail_to_shuttle")

    rail_body = rail.get_visual("rail_body")
    top_bridge = shuttle.get_visual("top_bridge")

    rail_aabb = ctx.part_world_aabb(rail)
    ctx.check(
        "rail sits flat on the ground plane",
        rail_aabb is not None
        and abs(rail_aabb[0][2]) <= 1e-9
        and abs(rail_aabb[1][2] - RAIL_HEIGHT) <= 1e-9,
        details=f"rail_aabb={rail_aabb}",
    )
    ctx.check(
        "prismatic axis follows the rail length",
        tuple(slide.axis) == (1.0, 0.0, 0.0),
        details=f"axis={slide.axis}",
    )

    ctx.expect_contact(
        shuttle,
        rail,
        elem_a=top_bridge,
        elem_b=rail_body,
        name="bridge bears on the rail top",
    )
    ctx.expect_within(
        rail,
        shuttle,
        axes="y",
        inner_elem=rail_body,
        margin=0.0,
        name="rail stays captured between the shuttle skirts",
    )
    ctx.expect_overlap(
        shuttle,
        rail,
        axes="x",
        min_overlap=0.10,
        name="shuttle keeps a clear guide overlap at rest",
    )

    rest_pos = ctx.part_world_position(shuttle)
    with ctx.pose({slide: SLIDE_TRAVEL}):
        ctx.expect_within(
            rail,
            shuttle,
            axes="y",
            inner_elem=rail_body,
            margin=0.0,
            name="rail stays captured between the shuttle skirts at full extension",
        )
        ctx.expect_overlap(
            shuttle,
            rail,
            axes="x",
            min_overlap=0.10,
            name="shuttle keeps guide overlap at full extension",
        )
        extended_pos = ctx.part_world_position(shuttle)

    ctx.check(
        "shuttle translates along +X",
        rest_pos is not None
        and extended_pos is not None
        and extended_pos[0] > rest_pos[0] + 0.12,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
