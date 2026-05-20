from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


FRAME_BASE_SIZE = (0.11, 0.20, 0.016)
FRAME_UPRIGHT_SIZE = (0.014, 0.028, 0.34)
VERTICAL_RAIL_SIZE = (0.012, 0.010, 0.26)
FRAME_CROSSBAR_SIZE = (0.056, 0.010, 0.018)

CARRIAGE_BODY_SIZE = (0.072, 0.024, 0.090)
HORIZONTAL_GUIDE_SIZE = (0.030, 0.120, 0.012)

SLIDE_BEAM_SIZE = (0.022, 0.140, 0.032)
END_PLATFORM_SIZE = (0.055, 0.036, 0.014)
PLATFORM_PAD_RADIUS = 0.010
PLATFORM_PAD_HEIGHT = 0.008

VERTICAL_HOME = (0.0, -0.022, 0.110)
VERTICAL_TRAVEL = 0.160
SIDEWAYS_TRAVEL = 0.080


def _add_box(
    part,
    size: tuple[float, float, float],
    xyz: tuple[float, float, float],
    *,
    material: str,
    name: str,
) -> None:
    part.visual(
        Box(size),
        origin=Origin(xyz=xyz),
        material=material,
        name=name,
    )


def _add_cylinder(
    part,
    radius: float,
    length: float,
    xyz: tuple[float, float, float],
    *,
    material: str,
    name: str,
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="side_mounted_yz_stage")

    model.material("frame_black", rgba=(0.14, 0.15, 0.17, 1.0))
    model.material("rail_steel", rgba=(0.63, 0.65, 0.69, 1.0))
    model.material("machined_aluminum", rgba=(0.78, 0.80, 0.83, 1.0))
    model.material("stage_blue", rgba=(0.25, 0.39, 0.70, 1.0))
    model.material("pad_dark", rgba=(0.12, 0.13, 0.15, 1.0))

    frame = model.part("frame")
    _add_box(
        frame,
        FRAME_BASE_SIZE,
        (0.0, 0.0, FRAME_BASE_SIZE[2] / 2.0),
        material="frame_black",
        name="frame_base",
    )
    _add_box(
        frame,
        FRAME_UPRIGHT_SIZE,
        (0.0, -0.058, FRAME_BASE_SIZE[2] + FRAME_UPRIGHT_SIZE[2] / 2.0),
        material="frame_black",
        name="frame_upright",
    )
    _add_box(
        frame,
        VERTICAL_RAIL_SIZE,
        (-0.020, -0.039, 0.182),
        material="rail_steel",
        name="left_vertical_rail",
    )
    _add_box(
        frame,
        VERTICAL_RAIL_SIZE,
        (0.020, -0.039, 0.182),
        material="rail_steel",
        name="right_vertical_rail",
    )
    _add_box(
        frame,
        FRAME_CROSSBAR_SIZE,
        (0.0, -0.039, 0.058),
        material="rail_steel",
        name="lower_crossbar",
    )
    _add_box(
        frame,
        FRAME_CROSSBAR_SIZE,
        (0.0, -0.039, 0.306),
        material="rail_steel",
        name="upper_crossbar",
    )

    carriage = model.part("vertical_carriage")
    _add_box(
        carriage,
        CARRIAGE_BODY_SIZE,
        (0.0, 0.0, 0.0),
        material="machined_aluminum",
        name="carriage_body",
    )
    _add_box(
        carriage,
        HORIZONTAL_GUIDE_SIZE,
        (0.0, 0.072, 0.022),
        material="machined_aluminum",
        name="upper_slide_guide",
    )
    _add_box(
        carriage,
        HORIZONTAL_GUIDE_SIZE,
        (0.0, 0.072, -0.022),
        material="machined_aluminum",
        name="lower_slide_guide",
    )

    slide = model.part("sideways_slide")
    _add_box(
        slide,
        SLIDE_BEAM_SIZE,
        (0.0, 0.070, 0.0),
        material="machined_aluminum",
        name="slide_beam",
    )
    _add_box(
        slide,
        END_PLATFORM_SIZE,
        (0.0, 0.158, 0.0),
        material="stage_blue",
        name="end_platform",
    )
    _add_cylinder(
        slide,
        PLATFORM_PAD_RADIUS,
        PLATFORM_PAD_HEIGHT,
        (0.0, 0.158, END_PLATFORM_SIZE[2] / 2.0 + PLATFORM_PAD_HEIGHT / 2.0),
        material="pad_dark",
        name="platform_pad",
    )

    model.articulation(
        "frame_to_vertical_carriage",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=carriage,
        origin=Origin(xyz=VERTICAL_HOME),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=VERTICAL_TRAVEL,
            effort=180.0,
            velocity=0.12,
        ),
    )
    model.articulation(
        "carriage_to_sideways_slide",
        ArticulationType.PRISMATIC,
        parent=carriage,
        child=slide,
        origin=Origin(xyz=(0.0, CARRIAGE_BODY_SIZE[1] / 2.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=SIDEWAYS_TRAVEL,
            effort=120.0,
            velocity=0.10,
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
    carriage = object_model.get_part("vertical_carriage")
    slide = object_model.get_part("sideways_slide")
    z_stage = object_model.get_articulation("frame_to_vertical_carriage")
    y_stage = object_model.get_articulation("carriage_to_sideways_slide")

    ctx.expect_contact(
        carriage,
        frame,
        name="vertical carriage stays mounted on the frame rails",
    )
    ctx.expect_contact(
        slide,
        carriage,
        name="sideways slide stays captured by the carriage guides",
    )
    ctx.expect_overlap(
        slide,
        carriage,
        axes="y",
        min_overlap=0.03,
        name="rest pose keeps the sideways slide inserted in the carriage",
    )

    carriage_rest = ctx.part_world_position(carriage)
    with ctx.pose({z_stage: VERTICAL_TRAVEL}):
        ctx.expect_contact(
            carriage,
            frame,
            name="raised carriage remains supported by the frame rails",
        )
        carriage_raised = ctx.part_world_position(carriage)

    slide_rest = ctx.part_world_position(slide)
    with ctx.pose({y_stage: SIDEWAYS_TRAVEL}):
        ctx.expect_contact(
            slide,
            carriage,
            name="extended slide remains supported by the carriage guides",
        )
        ctx.expect_overlap(
            slide,
            carriage,
            axes="y",
            min_overlap=0.03,
            name="extended slide still retains insertion in the carriage",
        )
        slide_extended = ctx.part_world_position(slide)

    ctx.check(
        "vertical carriage moves upward along +Z",
        carriage_rest is not None
        and carriage_raised is not None
        and carriage_raised[2] > carriage_rest[2] + 0.10,
        details=f"rest={carriage_rest}, raised={carriage_raised}",
    )
    ctx.check(
        "sideways slide moves outward along +Y",
        slide_rest is not None
        and slide_extended is not None
        and slide_extended[1] > slide_rest[1] + 0.05,
        details=f"rest={slide_rest}, extended={slide_extended}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
