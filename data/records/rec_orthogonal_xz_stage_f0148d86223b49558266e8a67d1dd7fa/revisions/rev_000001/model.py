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


GUIDE_LENGTH = 0.72
GUIDE_WIDTH = 0.22
GUIDE_BASE_THICKNESS = 0.03
GUIDE_SADDLE_LENGTH = 0.64
GUIDE_SADDLE_WIDTH = 0.15
GUIDE_SADDLE_THICKNESS = 0.012
GUIDE_RAIL_LENGTH = 0.62
GUIDE_RAIL_WIDTH = 0.028
GUIDE_RAIL_HEIGHT = 0.010
GUIDE_RAIL_OFFSET_Y = 0.065
GUIDE_TOP_Z = GUIDE_BASE_THICKNESS + GUIDE_SADDLE_THICKNESS + GUIDE_RAIL_HEIGHT

CARRIAGE_LENGTH = 0.24
CARRIAGE_WIDTH = 0.20
CARRIAGE_RUNNER_LENGTH = 0.18
CARRIAGE_RUNNER_WIDTH = 0.032
CARRIAGE_RUNNER_HEIGHT = 0.018
CARRIAGE_DECK_THICKNESS = 0.028
CARRIAGE_DECK_TOP_Z = CARRIAGE_RUNNER_HEIGHT + CARRIAGE_DECK_THICKNESS
CARRIAGE_PEDESTAL_LENGTH = 0.105
CARRIAGE_PEDESTAL_WIDTH = 0.15
CARRIAGE_PEDESTAL_THICKNESS = 0.020
CARRIAGE_RIB_LENGTH = 0.034
CARRIAGE_RIB_WIDTH = 0.030
CARRIAGE_RIB_HEIGHT = CARRIAGE_PEDESTAL_THICKNESS
CARRIAGE_RIB_OFFSET_Y = 0.048

SLIDE_WIDTH = 0.090
SLIDE_DEPTH = 0.055
SLIDE_HEIGHT = 0.105
SLIDE_TOP_PAD_WIDTH = 0.068
SLIDE_TOP_PAD_DEPTH = 0.040
SLIDE_TOP_PAD_HEIGHT = 0.010

X_TRAVEL = 0.22
Z_TRAVEL = 0.13


def _box(length: float, width: float, height: float) -> cq.Workplane:
    return cq.Workplane("XY").box(length, width, height, centered=(True, True, False))


def _build_guide_shape() -> cq.Workplane:
    guide = _box(GUIDE_LENGTH, GUIDE_WIDTH, GUIDE_BASE_THICKNESS)
    guide = guide.union(
        _box(GUIDE_SADDLE_LENGTH, GUIDE_SADDLE_WIDTH, GUIDE_SADDLE_THICKNESS).translate(
            (0.0, 0.0, GUIDE_BASE_THICKNESS)
        )
    )
    for rail_y in (-GUIDE_RAIL_OFFSET_Y, GUIDE_RAIL_OFFSET_Y):
        guide = guide.union(
            _box(GUIDE_RAIL_LENGTH, GUIDE_RAIL_WIDTH, GUIDE_RAIL_HEIGHT).translate(
                (0.0, rail_y, GUIDE_BASE_THICKNESS + GUIDE_SADDLE_THICKNESS)
            )
        )
    return guide


def _build_carriage_shape() -> cq.Workplane:
    carriage = _box(
        CARRIAGE_LENGTH,
        CARRIAGE_WIDTH,
        CARRIAGE_DECK_THICKNESS,
    ).translate((0.0, 0.0, CARRIAGE_RUNNER_HEIGHT))

    for runner_y in (-GUIDE_RAIL_OFFSET_Y, GUIDE_RAIL_OFFSET_Y):
        carriage = carriage.union(
            _box(
                CARRIAGE_RUNNER_LENGTH,
                CARRIAGE_RUNNER_WIDTH,
                CARRIAGE_RUNNER_HEIGHT,
            ).translate((0.0, runner_y, 0.0))
        )

    carriage = carriage.union(
        _box(
            CARRIAGE_PEDESTAL_LENGTH,
            CARRIAGE_PEDESTAL_WIDTH,
            CARRIAGE_PEDESTAL_THICKNESS,
        ).translate((0.0, 0.0, CARRIAGE_DECK_TOP_Z))
    )
    for rib_y in (-CARRIAGE_RIB_OFFSET_Y, CARRIAGE_RIB_OFFSET_Y):
        carriage = carriage.union(
            _box(
                CARRIAGE_RIB_LENGTH,
                CARRIAGE_RIB_WIDTH,
                CARRIAGE_RIB_HEIGHT,
            ).translate(
                (
                    0.0,
                    rib_y,
                    CARRIAGE_DECK_TOP_Z,
                )
            )
        )
    return carriage


def _build_slide_shape() -> cq.Workplane:
    slide = _box(SLIDE_WIDTH, SLIDE_DEPTH, SLIDE_HEIGHT).edges("|Z").fillet(0.004)
    slide = slide.union(
        _box(
            SLIDE_TOP_PAD_WIDTH,
            SLIDE_TOP_PAD_DEPTH,
            SLIDE_TOP_PAD_HEIGHT,
        ).translate((0.0, 0.0, SLIDE_HEIGHT))
    )
    return slide


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="guideway_xz_stage_block")

    model.material("ground_cast", rgba=(0.24, 0.26, 0.29, 1.0))
    model.material("machined_aluminum", rgba=(0.73, 0.76, 0.80, 1.0))
    model.material("slide_silver", rgba=(0.82, 0.84, 0.87, 1.0))

    guide = model.part("guide")
    guide.visual(
        mesh_from_cadquery(_build_guide_shape(), "guideway_xz_stage_guide"),
        material="ground_cast",
        name="guide_body",
    )
    guide.inertial = Inertial.from_geometry(
        Box((GUIDE_LENGTH, GUIDE_WIDTH, GUIDE_TOP_Z)),
        mass=12.0,
        origin=Origin(xyz=(0.0, 0.0, GUIDE_TOP_Z / 2.0)),
    )

    carriage = model.part("carriage")
    carriage.visual(
        mesh_from_cadquery(_build_carriage_shape(), "guideway_xz_stage_carriage"),
        material="machined_aluminum",
        name="carriage_body",
    )
    carriage.inertial = Inertial.from_geometry(
        Box(
            (
                CARRIAGE_LENGTH,
                CARRIAGE_WIDTH,
                CARRIAGE_DECK_TOP_Z + CARRIAGE_PEDESTAL_THICKNESS,
            )
        ),
        mass=5.5,
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                (CARRIAGE_DECK_TOP_Z + CARRIAGE_PEDESTAL_THICKNESS) / 2.0,
            )
        ),
    )

    slide = model.part("z_slide")
    slide.visual(
        mesh_from_cadquery(_build_slide_shape(), "guideway_xz_stage_z_slide"),
        material="slide_silver",
        name="slide_body",
    )
    slide.inertial = Inertial.from_geometry(
        Box((SLIDE_WIDTH, SLIDE_DEPTH, SLIDE_HEIGHT + SLIDE_TOP_PAD_HEIGHT)),
        mass=1.8,
        origin=Origin(xyz=(0.0, 0.0, (SLIDE_HEIGHT + SLIDE_TOP_PAD_HEIGHT) / 2.0)),
    )

    x_stage = model.articulation(
        "guide_to_carriage",
        ArticulationType.PRISMATIC,
        parent=guide,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, GUIDE_TOP_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=-X_TRAVEL,
            upper=X_TRAVEL,
            effort=2400.0,
            velocity=0.45,
        ),
    )

    model.articulation(
        "carriage_to_z_slide",
        ArticulationType.PRISMATIC,
        parent=carriage,
        child=slide,
        origin=Origin(
            xyz=(0.0, 0.0, CARRIAGE_DECK_TOP_Z + CARRIAGE_PEDESTAL_THICKNESS)
        ),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=Z_TRAVEL,
            effort=1200.0,
            velocity=0.30,
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

    guide = object_model.get_part("guide")
    carriage = object_model.get_part("carriage")
    slide = object_model.get_part("z_slide")
    x_stage = object_model.get_articulation("guide_to_carriage")
    z_stage = object_model.get_articulation("carriage_to_z_slide")

    ctx.expect_contact(
        carriage,
        guide,
        name="carriage runners contact the horizontal guide",
    )
    ctx.expect_overlap(
        carriage,
        guide,
        axes="xy",
        min_overlap=0.16,
        name="broad carriage stays over the guide footprint",
    )
    ctx.expect_overlap(
        slide,
        carriage,
        axes="xy",
        min_overlap=0.05,
        name="vertical slide stays centered over the carriage",
    )

    carriage_rest = ctx.part_world_position(carriage)
    with ctx.pose({x_stage: X_TRAVEL}):
        ctx.expect_contact(
            carriage,
            guide,
            name="carriage stays supported at maximum x travel",
        )
        ctx.expect_overlap(
            carriage,
            guide,
            axes="x",
            min_overlap=0.18,
            name="carriage retains guide engagement at maximum x travel",
        )
        carriage_extended = ctx.part_world_position(carriage)

    ctx.check(
        "carriage translates along +X",
        carriage_rest is not None
        and carriage_extended is not None
        and carriage_extended[0] > carriage_rest[0] + 0.15,
        details=f"rest={carriage_rest}, extended={carriage_extended}",
    )

    slide_rest = ctx.part_world_position(slide)
    with ctx.pose({z_stage: Z_TRAVEL}):
        ctx.expect_overlap(
            slide,
            carriage,
            axes="xy",
            min_overlap=0.05,
            name="raised slide remains guided over the carriage center mount",
        )
        slide_raised = ctx.part_world_position(slide)

    ctx.check(
        "vertical slide lifts along +Z",
        slide_rest is not None
        and slide_raised is not None
        and slide_raised[2] > slide_rest[2] + 0.08,
        details=f"rest={slide_rest}, raised={slide_raised}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
