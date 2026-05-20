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


PLATE_LENGTH = 0.46
PLATE_HEIGHT = 0.18
PLATE_THICKNESS = 0.003

OUTER_LENGTH = 0.43
OUTER_DEPTH = 0.0135
OUTER_HEIGHT = 0.046
OUTER_SHEET = 0.0012

MIDDLE_LENGTH = 0.345
MIDDLE_DEPTH = 0.0111
MIDDLE_HEIGHT = OUTER_HEIGHT - (2.0 * OUTER_SHEET)
MIDDLE_SHEET = 0.0011

INNER_LENGTH = 0.27
INNER_DEPTH = 0.0087
INNER_HEIGHT = MIDDLE_HEIGHT - (2.0 * MIDDLE_SHEET)
INNER_SHEET = 0.0010

OUTER_ON_PLATE_X = -0.205
OUTER_ON_PLATE_Y = PLATE_THICKNESS / 2.0
OUTER_ON_PLATE_Z = 0.0

MIDDLE_HOME_X = OUTER_LENGTH - MIDDLE_LENGTH - 0.018
INNER_HOME_X = MIDDLE_LENGTH - INNER_LENGTH - 0.015

OUTER_TO_MIDDLE_TRAVEL = 0.15
MIDDLE_TO_INNER_TRAVEL = 0.14


def _x_box(length: float, depth: float, height: float) -> cq.Workplane:
    return cq.Workplane("XY").box(length, depth, height).translate((length / 2.0, 0.0, 0.0))


def _box_origin(
    *,
    x_min: float,
    y_min: float,
    length: float,
    depth: float,
    z_center: float,
) -> Origin:
    return Origin(xyz=(x_min + (length / 2.0), y_min + (depth / 2.0), z_center))


def _add_outer_channel_visuals(model_part, *, material: str) -> None:
    flange_depth = OUTER_DEPTH - OUTER_SHEET
    for name, size, origin in (
        (
            "outer_web",
            Box((OUTER_LENGTH, OUTER_SHEET, OUTER_HEIGHT)),
            _box_origin(
                x_min=0.0,
                y_min=0.0,
                length=OUTER_LENGTH,
                depth=OUTER_SHEET,
                z_center=0.0,
            ),
        ),
        (
            "outer_top_flange",
            Box((OUTER_LENGTH, flange_depth, OUTER_SHEET)),
            _box_origin(
                x_min=0.0,
                y_min=OUTER_SHEET,
                length=OUTER_LENGTH,
                depth=flange_depth,
                z_center=(OUTER_HEIGHT / 2.0) - (OUTER_SHEET / 2.0),
            ),
        ),
        (
            "outer_bottom_flange",
            Box((OUTER_LENGTH, flange_depth, OUTER_SHEET)),
            _box_origin(
                x_min=0.0,
                y_min=OUTER_SHEET,
                length=OUTER_LENGTH,
                depth=flange_depth,
                z_center=-(OUTER_HEIGHT / 2.0) + (OUTER_SHEET / 2.0),
            ),
        ),
    ):
        model_part.visual(size, origin=origin, material=material, name=name)


def _add_middle_runner_visuals(model_part, *, material: str) -> None:
    flange_depth = MIDDLE_DEPTH - MIDDLE_SHEET
    for name, size, origin in (
        (
            "middle_front_web",
            Box((MIDDLE_LENGTH, MIDDLE_SHEET, MIDDLE_HEIGHT)),
            _box_origin(
                x_min=0.0,
                y_min=MIDDLE_DEPTH - MIDDLE_SHEET,
                length=MIDDLE_LENGTH,
                depth=MIDDLE_SHEET,
                z_center=0.0,
            ),
        ),
        (
            "middle_top_flange",
            Box((MIDDLE_LENGTH, flange_depth, MIDDLE_SHEET)),
            _box_origin(
                x_min=0.0,
                y_min=0.0,
                length=MIDDLE_LENGTH,
                depth=flange_depth,
                z_center=(MIDDLE_HEIGHT / 2.0) - (MIDDLE_SHEET / 2.0),
            ),
        ),
        (
            "middle_bottom_flange",
            Box((MIDDLE_LENGTH, flange_depth, MIDDLE_SHEET)),
            _box_origin(
                x_min=0.0,
                y_min=0.0,
                length=MIDDLE_LENGTH,
                depth=flange_depth,
                z_center=-(MIDDLE_HEIGHT / 2.0) + (MIDDLE_SHEET / 2.0),
            ),
        ),
    ):
        model_part.visual(size, origin=origin, material=material, name=name)


def _add_inner_runner_visuals(model_part, *, material: str) -> None:
    flange_depth = INNER_DEPTH - INNER_SHEET
    for name, size, origin in (
        (
            "inner_back_web",
            Box((INNER_LENGTH, INNER_SHEET, INNER_HEIGHT)),
            _box_origin(
                x_min=0.0,
                y_min=0.0,
                length=INNER_LENGTH,
                depth=INNER_SHEET,
                z_center=0.0,
            ),
        ),
        (
            "inner_top_flange",
            Box((INNER_LENGTH, flange_depth, INNER_SHEET)),
            _box_origin(
                x_min=0.0,
                y_min=INNER_SHEET,
                length=INNER_LENGTH,
                depth=flange_depth,
                z_center=(INNER_HEIGHT / 2.0) - (INNER_SHEET / 2.0),
            ),
        ),
        (
            "inner_bottom_flange",
            Box((INNER_LENGTH, flange_depth, INNER_SHEET)),
            _box_origin(
                x_min=0.0,
                y_min=INNER_SHEET,
                length=INNER_LENGTH,
                depth=flange_depth,
                z_center=-(INNER_HEIGHT / 2.0) + (INNER_SHEET / 2.0),
            ),
        ),
        (
            "inner_mount_pad",
            Box((0.11, 0.0024, 0.018)),
            _box_origin(
                x_min=INNER_LENGTH - 0.13,
                y_min=0.0,
                length=0.11,
                depth=0.0024,
                z_center=0.0,
            ),
        ),
    ):
        model_part.visual(size, origin=origin, material=material, name=name)


def _side_plate_shape() -> cq.Workplane:
    plate = (
        cq.Workplane("XZ")
        .rect(PLATE_LENGTH, PLATE_HEIGHT)
        .extrude(PLATE_THICKNESS / 2.0, both=True)
    )

    slot_points = [(-0.16, 0.055), (0.0, 0.055), (0.16, 0.055), (-0.08, -0.055), (0.08, -0.055)]
    plate = (
        plate.faces(">Y")
        .workplane(centerOption="CenterOfMass")
        .pushPoints(slot_points)
        .slot2D(0.020, 0.006, angle=90)
        .cutThruAll()
    )

    rib = _x_box(0.30, 0.0012, 0.008)
    lower_rib = rib.translate((0.08, -(PLATE_THICKNESS / 2.0) - 0.0006, -0.050))
    upper_rib = rib.translate((0.08, -(PLATE_THICKNESS / 2.0) - 0.0006, 0.050))
    return plate.union(lower_rib).union(upper_rib)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="three_stage_side_extension_unit")

    model.material("plate_gray", rgba=(0.56, 0.58, 0.61, 1.0))
    model.material("outer_steel", rgba=(0.26, 0.28, 0.31, 1.0))
    model.material("middle_steel", rgba=(0.53, 0.55, 0.58, 1.0))
    model.material("inner_steel", rgba=(0.74, 0.75, 0.77, 1.0))

    side_plate = model.part("side_plate")
    side_plate.visual(
        mesh_from_cadquery(_side_plate_shape(), "side_plate"),
        material="plate_gray",
        name="plate_body",
    )
    side_plate.inertial = Inertial.from_geometry(
        Box((PLATE_LENGTH, PLATE_THICKNESS, PLATE_HEIGHT)),
        mass=2.2,
    )

    outer_channel = model.part("outer_channel")
    _add_outer_channel_visuals(outer_channel, material="outer_steel")
    outer_channel.inertial = Inertial.from_geometry(
        Box((OUTER_LENGTH, OUTER_DEPTH, OUTER_HEIGHT)),
        mass=1.1,
        origin=Origin(xyz=(OUTER_LENGTH / 2.0, OUTER_DEPTH / 2.0, 0.0)),
    )

    middle_runner = model.part("middle_runner")
    _add_middle_runner_visuals(middle_runner, material="middle_steel")
    middle_runner.inertial = Inertial.from_geometry(
        Box((MIDDLE_LENGTH, MIDDLE_DEPTH, MIDDLE_HEIGHT)),
        mass=0.7,
        origin=Origin(xyz=(MIDDLE_LENGTH / 2.0, MIDDLE_DEPTH / 2.0, 0.0)),
    )

    inner_runner = model.part("inner_runner")
    _add_inner_runner_visuals(inner_runner, material="inner_steel")
    inner_runner.inertial = Inertial.from_geometry(
        Box((INNER_LENGTH, INNER_DEPTH, INNER_HEIGHT)),
        mass=0.5,
        origin=Origin(xyz=(INNER_LENGTH / 2.0, INNER_DEPTH / 2.0, 0.0)),
    )

    model.articulation(
        "plate_to_outer",
        ArticulationType.FIXED,
        parent=side_plate,
        child=outer_channel,
        origin=Origin(xyz=(OUTER_ON_PLATE_X, OUTER_ON_PLATE_Y, OUTER_ON_PLATE_Z)),
    )
    model.articulation(
        "outer_to_middle",
        ArticulationType.PRISMATIC,
        parent=outer_channel,
        child=middle_runner,
        origin=Origin(xyz=(MIDDLE_HOME_X, OUTER_SHEET, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=OUTER_TO_MIDDLE_TRAVEL,
            effort=120.0,
            velocity=0.45,
        ),
    )
    model.articulation(
        "middle_to_inner",
        ArticulationType.PRISMATIC,
        parent=middle_runner,
        child=inner_runner,
        origin=Origin(xyz=(INNER_HOME_X, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=MIDDLE_TO_INNER_TRAVEL,
            effort=90.0,
            velocity=0.45,
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    side_plate = object_model.get_part("side_plate")
    outer_channel = object_model.get_part("outer_channel")
    middle_runner = object_model.get_part("middle_runner")
    inner_runner = object_model.get_part("inner_runner")
    outer_to_middle = object_model.get_articulation("outer_to_middle")
    middle_to_inner = object_model.get_articulation("middle_to_inner")

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
        side_plate,
        outer_channel,
        name="side_plate_supports_outer_channel",
    )
    ctx.expect_contact(
        outer_channel,
        middle_runner,
        name="outer_channel_guides_middle_runner",
    )
    ctx.expect_contact(
        middle_runner,
        inner_runner,
        name="middle_runner_guides_inner_runner",
    )
    ctx.expect_within(
        middle_runner,
        outer_channel,
        axes="yz",
        margin=0.0002,
        name="middle_runner_stays_nested_in_outer_channel",
    )
    ctx.expect_within(
        inner_runner,
        middle_runner,
        axes="yz",
        margin=0.0002,
        name="inner_runner_stays_nested_in_middle_runner",
    )
    ctx.expect_overlap(
        outer_channel,
        middle_runner,
        axes="x",
        min_overlap=0.27,
        name="outer_and_middle_have_closed_overlap",
    )
    ctx.expect_overlap(
        middle_runner,
        inner_runner,
        axes="x",
        min_overlap=0.20,
        name="middle_and_inner_have_closed_overlap",
    )

    with ctx.pose({outer_to_middle: OUTER_TO_MIDDLE_TRAVEL}):
        ctx.expect_origin_gap(
            middle_runner,
            outer_channel,
            axis="x",
            min_gap=MIDDLE_HOME_X + OUTER_TO_MIDDLE_TRAVEL - 0.001,
            max_gap=MIDDLE_HOME_X + OUTER_TO_MIDDLE_TRAVEL + 0.001,
            name="middle_runner_extends_forward_along_x",
        )
        ctx.expect_within(
            middle_runner,
            outer_channel,
            axes="yz",
            margin=0.0002,
            name="middle_runner_remains_laterally_captured_when_extended",
        )

    with ctx.pose(
        {
            outer_to_middle: OUTER_TO_MIDDLE_TRAVEL,
            middle_to_inner: MIDDLE_TO_INNER_TRAVEL,
        }
    ):
        ctx.expect_origin_gap(
            inner_runner,
            middle_runner,
            axis="x",
            min_gap=INNER_HOME_X + MIDDLE_TO_INNER_TRAVEL - 0.001,
            max_gap=INNER_HOME_X + MIDDLE_TO_INNER_TRAVEL + 0.001,
            name="inner_runner_extends_forward_along_x",
        )
        ctx.expect_within(
            inner_runner,
            middle_runner,
            axes="yz",
            margin=0.0002,
            name="inner_runner_remains_laterally_captured_when_extended",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
