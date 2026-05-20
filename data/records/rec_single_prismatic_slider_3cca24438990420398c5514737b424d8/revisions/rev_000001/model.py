from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_LENGTH = 1.20
BASE_WIDTH = 0.62
BASE_THICKNESS = 0.032

GUIDE_LENGTH = 0.92
GUIDE_BASE_WIDTH = 0.15
GUIDE_BASE_HEIGHT = 0.024
GUIDE_RIB_WIDTH = 0.072
GUIDE_RIB_HEIGHT = 0.028

CARRIAGE_LENGTH = 0.20
CARRIAGE_WIDTH = 0.18
CARRIAGE_BODY_HEIGHT = 0.068
CARRIAGE_CHANNEL_WIDTH = 0.088
CARRIAGE_CHANNEL_DEPTH = 0.040
RUNNER_WIDTH = (CARRIAGE_WIDTH - CARRIAGE_CHANNEL_WIDTH) / 2.0
BRIDGE_HEIGHT = CARRIAGE_BODY_HEIGHT - CARRIAGE_CHANNEL_DEPTH

TOOLING_SURFACE_LENGTH = 0.14
TOOLING_SURFACE_WIDTH = 0.11
TOOLING_SURFACE_THICKNESS = 0.010

SLIDE_HOME_X = -0.22
SLIDE_TRAVEL = 0.44


def _base_plate_shape() -> cq.Workplane:
    mount_x = BASE_LENGTH * 0.40
    mount_y = BASE_WIDTH * 0.32
    return (
        cq.Workplane("XY")
        .box(BASE_LENGTH, BASE_WIDTH, BASE_THICKNESS, centered=(True, True, False))
        .faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .pushPoints(
            [
                (-mount_x, -mount_y),
                (-mount_x, mount_y),
                (mount_x, -mount_y),
                (mount_x, mount_y),
            ]
        )
        .hole(0.018)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="base_plate_slide_table")

    model.material("painted_steel", rgba=(0.44, 0.46, 0.49, 1.0))
    model.material("guide_steel", rgba=(0.62, 0.64, 0.68, 1.0))
    model.material("carriage_steel", rgba=(0.72, 0.74, 0.77, 1.0))
    model.material("machined_face", rgba=(0.82, 0.84, 0.87, 1.0))

    base_plate = model.part("base_plate")
    base_plate.visual(
        mesh_from_cadquery(_base_plate_shape(), "base_plate_mesh"),
        material="painted_steel",
        name="plate_shell",
    )

    guide = model.part("guide")
    guide.visual(
        Box((GUIDE_LENGTH, GUIDE_BASE_WIDTH, GUIDE_BASE_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, GUIDE_BASE_HEIGHT / 2.0)),
        material="guide_steel",
        name="guide_base",
    )
    guide.visual(
        Box((GUIDE_LENGTH, GUIDE_RIB_WIDTH, GUIDE_RIB_HEIGHT)),
        origin=Origin(
            xyz=(0.0, 0.0, GUIDE_BASE_HEIGHT + (GUIDE_RIB_HEIGHT / 2.0))
        ),
        material="guide_steel",
        name="guide_rib",
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((CARRIAGE_LENGTH, RUNNER_WIDTH, CARRIAGE_CHANNEL_DEPTH)),
        origin=Origin(
            xyz=(
                0.0,
                -((CARRIAGE_CHANNEL_WIDTH / 2.0) + (RUNNER_WIDTH / 2.0)),
                CARRIAGE_CHANNEL_DEPTH / 2.0,
            )
        ),
        material="carriage_steel",
        name="left_runner",
    )
    carriage.visual(
        Box((CARRIAGE_LENGTH, RUNNER_WIDTH, CARRIAGE_CHANNEL_DEPTH)),
        origin=Origin(
            xyz=(
                0.0,
                (CARRIAGE_CHANNEL_WIDTH / 2.0) + (RUNNER_WIDTH / 2.0),
                CARRIAGE_CHANNEL_DEPTH / 2.0,
            )
        ),
        material="carriage_steel",
        name="right_runner",
    )
    carriage.visual(
        Box((CARRIAGE_LENGTH, CARRIAGE_WIDTH, BRIDGE_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                CARRIAGE_CHANNEL_DEPTH + (BRIDGE_HEIGHT / 2.0),
            )
        ),
        material="carriage_steel",
        name="carriage_bridge",
    )
    carriage.visual(
        Box(
            (
                TOOLING_SURFACE_LENGTH,
                TOOLING_SURFACE_WIDTH,
                TOOLING_SURFACE_THICKNESS,
            )
        ),
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                CARRIAGE_BODY_HEIGHT + (TOOLING_SURFACE_THICKNESS / 2.0),
            )
        ),
        material="machined_face",
        name="tooling_surface",
    )

    model.articulation(
        "base_to_guide",
        ArticulationType.FIXED,
        parent=base_plate,
        child=guide,
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS)),
    )
    model.articulation(
        "guide_to_carriage",
        ArticulationType.PRISMATIC,
        parent=guide,
        child=carriage,
        origin=Origin(xyz=(SLIDE_HOME_X, 0.0, GUIDE_BASE_HEIGHT)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=450.0,
            velocity=0.25,
            lower=0.0,
            upper=SLIDE_TRAVEL,
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

    base_plate = object_model.get_part("base_plate")
    guide = object_model.get_part("guide")
    carriage = object_model.get_part("carriage")
    slide = object_model.get_articulation("guide_to_carriage")

    guide_base = guide.get_visual("guide_base")
    left_runner = carriage.get_visual("left_runner")
    right_runner = carriage.get_visual("right_runner")
    carriage_bridge = carriage.get_visual("carriage_bridge")
    tooling_surface = carriage.get_visual("tooling_surface")

    ctx.expect_gap(
        guide,
        base_plate,
        axis="z",
        min_gap=0.0,
        max_gap=0.001,
        max_penetration=0.0,
        name="guide sits directly on the base plate",
    )
    ctx.expect_overlap(
        guide,
        base_plate,
        axes="xy",
        min_overlap=0.12,
        name="guide footprint is supported by the base plate",
    )
    ctx.expect_origin_gap(
        carriage,
        guide,
        axis="z",
        min_gap=GUIDE_BASE_HEIGHT - 1e-4,
        max_gap=GUIDE_BASE_HEIGHT + 1e-4,
        name="carriage rides at the guide-base height",
    )
    ctx.expect_origin_distance(
        carriage,
        guide,
        axes="y",
        max_dist=1e-4,
        name="carriage stays centered over the guide",
    )
    ctx.expect_gap(
        carriage,
        guide,
        axis="z",
        positive_elem=left_runner,
        negative_elem=guide_base,
        min_gap=0.0,
        max_gap=0.001,
        max_penetration=0.0,
        name="left runner seats on the guide lands",
    )
    ctx.expect_gap(
        carriage,
        guide,
        axis="z",
        positive_elem=right_runner,
        negative_elem=guide_base,
        min_gap=0.0,
        max_gap=0.001,
        max_penetration=0.0,
        name="right runner seats on the guide lands",
    )
    ctx.expect_gap(
        carriage,
        base_plate,
        axis="z",
        positive_elem=tooling_surface,
        min_gap=0.09,
        name="tooling surface stands clearly above the grounded plate",
    )

    rest_pos = ctx.part_world_position(carriage)
    with ctx.pose({slide: SLIDE_TRAVEL}):
        ctx.expect_origin_gap(
            carriage,
            guide,
            axis="z",
            min_gap=GUIDE_BASE_HEIGHT - 1e-4,
            max_gap=GUIDE_BASE_HEIGHT + 1e-4,
            name="extended carriage keeps the same ride height",
        )
        ctx.expect_origin_distance(
            carriage,
            guide,
            axes="y",
            max_dist=1e-4,
            name="extended carriage remains centered on the guide",
        )
        ctx.expect_gap(
            carriage,
            guide,
            axis="z",
            positive_elem=left_runner,
            negative_elem=guide_base,
            min_gap=0.0,
            max_gap=0.001,
            max_penetration=0.0,
            name="extended left runner still rides on the guide lands",
        )
        ctx.expect_gap(
            carriage,
            guide,
            axis="z",
            positive_elem=right_runner,
            negative_elem=guide_base,
            min_gap=0.0,
            max_gap=0.001,
            max_penetration=0.0,
            name="extended right runner still rides on the guide lands",
        )
        ctx.expect_overlap(
            carriage,
            guide,
            axes="x",
            elem_a=carriage_bridge,
            elem_b=guide_base,
            min_overlap=0.20,
            name="extended carriage remains well within the guide length",
        )
        extended_pos = ctx.part_world_position(carriage)

    ctx.check(
        "carriage translates along +X",
        rest_pos is not None
        and extended_pos is not None
        and extended_pos[0] > rest_pos[0] + 0.20,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
