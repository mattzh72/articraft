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


BASE_LENGTH = 0.34
BASE_WIDTH = 0.14
BASE_PLATE_THICKNESS = 0.012
BASE_FOOT_LENGTH = 0.062
BASE_FOOT_THICKNESS = 0.008

LOWER_GUIDE_LENGTH = 0.30
LOWER_GUIDE_WIDTH = 0.060
LOWER_GUIDE_HEIGHT = 0.026
LOWER_GUIDE_TOP_Z = BASE_PLATE_THICKNESS + LOWER_GUIDE_HEIGHT

X_TRAVEL = 0.070
Y_TRAVEL = 0.045

SLIDE_PAD_THICKNESS = 0.004

X_CARRIAGE_LENGTH = 0.180
X_CARRIAGE_WIDTH = 0.116
X_CARRIAGE_SKIRT_DEPTH = 0.012
X_CARRIAGE_TOP_Z = 0.036
X_CARRIAGE_PAD_LENGTH = 0.144
X_CARRIAGE_PAD_WIDTH = 0.016

CROSS_RAIL_WIDTH = 0.058
CROSS_RAIL_LENGTH = 0.170
CROSS_RAIL_HEIGHT = 0.018
CROSS_RAIL_TOP_Z = X_CARRIAGE_TOP_Z + CROSS_RAIL_HEIGHT

Y_SLIDE_WIDTH = 0.096
Y_SLIDE_LENGTH = 0.136
Y_SLIDE_SKIRT_DEPTH = 0.010
Y_SLIDE_PAD_WIDTH = 0.016
Y_SLIDE_PAD_LENGTH = 0.116

OUTPUT_FACE_SIZE = 0.060
OUTPUT_FACE_THICKNESS = 0.010
OUTPUT_NECK_LENGTH = 0.018
OUTPUT_NECK_WIDTH = 0.040
OUTPUT_NECK_HEIGHT = 0.030


def _approx_box_inertial(
    size: tuple[float, float, float],
    center_xyz: tuple[float, float, float],
    mass: float,
) -> Inertial:
    return Inertial.from_geometry(Box(size), mass=mass, origin=Origin(xyz=center_xyz))


def _base_frame_shape() -> cq.Workplane:
    plate = (
        cq.Workplane("XY")
        .box(BASE_LENGTH, BASE_WIDTH, BASE_PLATE_THICKNESS)
        .translate((0.0, 0.0, BASE_PLATE_THICKNESS / 2.0))
        .val()
    )

    for x_pos in (-0.110, 0.110):
        foot = (
            cq.Workplane("XY")
            .box(BASE_FOOT_LENGTH, BASE_WIDTH * 0.78, BASE_FOOT_THICKNESS)
            .translate(
                (x_pos, 0.0, BASE_PLATE_THICKNESS + (BASE_FOOT_THICKNESS / 2.0))
            )
            .val()
        )
        plate = plate.fuse(foot)

    for x_pos in (-0.115, 0.115):
        for y_pos in (-0.044, 0.044):
            hole = (
                cq.Workplane("XY")
                .circle(0.005)
                .extrude(BASE_PLATE_THICKNESS + BASE_FOOT_THICKNESS + 0.002)
                .translate((x_pos, y_pos, 0.0))
                .val()
            )
            plate = plate.cut(hole)

    return plate


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="service_xy_axis")

    model.material("base_black", rgba=(0.14, 0.15, 0.17, 1.0))
    model.material("rail_steel", rgba=(0.60, 0.63, 0.68, 1.0))
    model.material("carriage_aluminum", rgba=(0.73, 0.75, 0.79, 1.0))
    model.material("output_face", rgba=(0.82, 0.84, 0.87, 1.0))
    model.material("pad_polymer", rgba=(0.14, 0.14, 0.15, 1.0))

    lower_guide = model.part("lower_guide")
    lower_guide.visual(
        mesh_from_cadquery(_base_frame_shape(), "lower_guide_frame"),
        material="base_black",
        name="base_frame",
    )
    lower_guide.visual(
        Box((LOWER_GUIDE_LENGTH, LOWER_GUIDE_WIDTH, LOWER_GUIDE_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, BASE_PLATE_THICKNESS + (LOWER_GUIDE_HEIGHT / 2.0))),
        material="rail_steel",
        name="guide_rail",
    )
    lower_guide.inertial = _approx_box_inertial(
        (BASE_LENGTH, BASE_WIDTH, LOWER_GUIDE_TOP_Z),
        (0.0, 0.0, LOWER_GUIDE_TOP_Z / 2.0),
        mass=7.0,
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((X_CARRIAGE_LENGTH, X_CARRIAGE_WIDTH, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material="carriage_aluminum",
        name="carriage_bridge",
    )
    carriage.visual(
        Box((X_CARRIAGE_LENGTH, 0.020, 0.024)),
        origin=Origin(xyz=(0.0, -0.038, 0.024)),
        material="carriage_aluminum",
        name="left_carriage_rib",
    )
    carriage.visual(
        Box((X_CARRIAGE_LENGTH, 0.020, 0.024)),
        origin=Origin(xyz=(0.0, 0.038, 0.024)),
        material="carriage_aluminum",
        name="right_carriage_rib",
    )
    carriage.visual(
        Box((CROSS_RAIL_WIDTH, CROSS_RAIL_LENGTH, CROSS_RAIL_HEIGHT)),
        origin=Origin(xyz=(0.0, 0.0, X_CARRIAGE_TOP_Z + (CROSS_RAIL_HEIGHT / 2.0))),
        material="rail_steel",
        name="cross_rail",
    )
    for y_pos, pad_name in (
        (-0.018, "left_runner_pad"),
        (0.018, "right_runner_pad"),
    ):
        carriage.visual(
            Box((X_CARRIAGE_PAD_LENGTH, X_CARRIAGE_PAD_WIDTH, SLIDE_PAD_THICKNESS)),
            origin=Origin(xyz=(0.0, y_pos, SLIDE_PAD_THICKNESS / 2.0)),
            material="pad_polymer",
            name=pad_name,
        )
    carriage.inertial = _approx_box_inertial(
        (
            X_CARRIAGE_LENGTH,
            X_CARRIAGE_WIDTH,
            CROSS_RAIL_TOP_Z + X_CARRIAGE_SKIRT_DEPTH,
        ),
        (0.0, 0.0, (CROSS_RAIL_TOP_Z - X_CARRIAGE_SKIRT_DEPTH) / 2.0),
        mass=3.6,
    )

    cross_slide = model.part("cross_slide")
    cross_slide.visual(
        Box((Y_SLIDE_WIDTH, Y_SLIDE_LENGTH, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material="carriage_aluminum",
        name="slide_bridge",
    )
    cross_slide.visual(
        Box((0.018, Y_SLIDE_LENGTH, 0.022)),
        origin=Origin(xyz=(-0.034, 0.0, 0.023)),
        material="carriage_aluminum",
        name="left_slide_rib",
    )
    cross_slide.visual(
        Box((0.018, Y_SLIDE_LENGTH, 0.022)),
        origin=Origin(xyz=(0.034, 0.0, 0.023)),
        material="carriage_aluminum",
        name="right_slide_rib",
    )
    cross_slide.visual(
        Box((OUTPUT_NECK_WIDTH, OUTPUT_NECK_LENGTH, OUTPUT_NECK_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                (Y_SLIDE_LENGTH / 2.0) + (OUTPUT_NECK_LENGTH / 2.0),
                OUTPUT_NECK_HEIGHT / 2.0 + 0.004,
            )
        ),
        material="carriage_aluminum",
        name="output_neck",
    )
    cross_slide.visual(
        Box((OUTPUT_FACE_SIZE, OUTPUT_FACE_THICKNESS, OUTPUT_FACE_SIZE)),
        origin=Origin(
            xyz=(
                0.0,
                (Y_SLIDE_LENGTH / 2.0) + OUTPUT_NECK_LENGTH + (OUTPUT_FACE_THICKNESS / 2.0),
                OUTPUT_FACE_SIZE / 2.0,
            )
        ),
        material="output_face",
        name="output_face_plate",
    )
    for x_pos, pad_name in (
        (-0.015, "left_cross_pad"),
        (0.015, "right_cross_pad"),
    ):
        cross_slide.visual(
            Box((Y_SLIDE_PAD_WIDTH, Y_SLIDE_PAD_LENGTH, SLIDE_PAD_THICKNESS)),
            origin=Origin(xyz=(x_pos, 0.0, SLIDE_PAD_THICKNESS / 2.0)),
            material="pad_polymer",
            name=pad_name,
        )
    cross_slide.inertial = _approx_box_inertial(
        (
            OUTPUT_FACE_SIZE,
            Y_SLIDE_LENGTH + OUTPUT_NECK_LENGTH + OUTPUT_FACE_THICKNESS,
            OUTPUT_FACE_SIZE,
        ),
        (
            0.0,
            0.018,
            (OUTPUT_FACE_SIZE - Y_SLIDE_SKIRT_DEPTH) / 2.0,
        ),
        mass=1.9,
    )

    model.articulation(
        "lower_stage",
        ArticulationType.PRISMATIC,
        parent=lower_guide,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, LOWER_GUIDE_TOP_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=-X_TRAVEL,
            upper=X_TRAVEL,
            effort=180.0,
            velocity=0.35,
        ),
    )
    model.articulation(
        "upper_stage",
        ArticulationType.PRISMATIC,
        parent=carriage,
        child=cross_slide,
        origin=Origin(xyz=(0.0, 0.0, CROSS_RAIL_TOP_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            lower=-Y_TRAVEL,
            upper=Y_TRAVEL,
            effort=110.0,
            velocity=0.28,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    lower_guide = object_model.get_part("lower_guide")
    carriage = object_model.get_part("carriage")
    cross_slide = object_model.get_part("cross_slide")
    lower_stage = object_model.get_articulation("lower_stage")
    upper_stage = object_model.get_articulation("upper_stage")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(
        carriage,
        lower_guide,
        name="long carriage is supported on the lower guide",
    )
    ctx.expect_contact(
        cross_slide,
        carriage,
        name="cross slide is supported on the carriage rail",
    )
    ctx.expect_overlap(
        carriage,
        lower_guide,
        axes="x",
        elem_a="carriage_bridge",
        elem_b="guide_rail",
        min_overlap=0.17,
        name="lower stage keeps substantial guide engagement",
    )
    ctx.expect_overlap(
        cross_slide,
        carriage,
        axes="y",
        elem_a="slide_bridge",
        elem_b="cross_rail",
        min_overlap=0.10,
        name="upper stage keeps substantial cross-rail engagement",
    )

    carriage_rest = ctx.part_world_position(carriage)
    with ctx.pose({lower_stage: X_TRAVEL}):
        ctx.expect_contact(
            carriage,
            lower_guide,
            name="lower stage remains supported at positive X travel",
        )
        ctx.expect_overlap(
            carriage,
            lower_guide,
            axes="x",
            elem_a="carriage_bridge",
            elem_b="guide_rail",
            min_overlap=0.16,
            name="lower stage still overlaps the guide at travel limit",
        )
        carriage_extended = ctx.part_world_position(carriage)

    ctx.check(
        "lower stage translates along +X",
        carriage_rest is not None
        and carriage_extended is not None
        and carriage_extended[0] > carriage_rest[0] + 0.05,
        details=f"rest={carriage_rest}, extended={carriage_extended}",
    )

    cross_slide_rest = ctx.part_world_position(cross_slide)
    with ctx.pose({upper_stage: Y_TRAVEL}):
        ctx.expect_contact(
            cross_slide,
            carriage,
            name="upper stage remains supported at positive Y travel",
        )
        ctx.expect_overlap(
            cross_slide,
            carriage,
            axes="y",
            elem_a="slide_bridge",
            elem_b="cross_rail",
            min_overlap=0.10,
            name="upper stage still overlaps the cross rail at travel limit",
        )
        cross_slide_extended = ctx.part_world_position(cross_slide)

    ctx.check(
        "upper stage translates along +Y",
        cross_slide_rest is not None
        and cross_slide_extended is not None
        and cross_slide_extended[1] > cross_slide_rest[1] + 0.03,
        details=f"rest={cross_slide_rest}, extended={cross_slide_extended}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
