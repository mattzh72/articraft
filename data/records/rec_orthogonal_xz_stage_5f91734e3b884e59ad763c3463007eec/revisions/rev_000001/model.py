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


BASE_LENGTH = 0.46
BASE_WIDTH = 0.20
BASE_THICKNESS = 0.024
BED_LENGTH = 0.40
BED_WIDTH = 0.12
BED_THICKNESS = 0.012
X_RAIL_LENGTH = 0.36
X_RAIL_WIDTH = 0.018
X_RAIL_HEIGHT = 0.012
X_RAIL_Y_OFFSET = 0.042
X_RAIL_TOP_Z = BASE_THICKNESS + BED_THICKNESS + X_RAIL_HEIGHT

CARRIAGE_LENGTH = 0.14
CARRIAGE_WIDTH = 0.15
CARRIAGE_HEIGHT = 0.05
COLUMN_WIDTH = 0.11
COLUMN_DEPTH = 0.018
COLUMN_HEIGHT = 0.19
CHEEK_WIDTH = 0.014
CHEEK_DEPTH = 0.036
CHEEK_HEIGHT = 0.17
CHEEK_X_OFFSET = 0.048
CROSSBAR_WIDTH = 0.082
CROSSBAR_DEPTH = 0.012
CROSSBAR_HEIGHT = 0.015
GUIDE_WIDTH = 0.014
GUIDE_DEPTH = 0.006
GUIDE_HEIGHT = 0.15
GUIDE_X_OFFSET = 0.028

Z_JOINT_Y = 0.024
Z_JOINT_Z = 0.065
SLIDE_WIDTH = 0.078
SLIDE_DEPTH = 0.018
SLIDE_HEIGHT = 0.15
TOP_PAD_WIDTH = 0.07
TOP_PAD_DEPTH = 0.048
TOP_PAD_THICKNESS = 0.012

X_TRAVEL_LOWER = -0.10
X_TRAVEL_UPPER = 0.10
Z_TRAVEL_LOWER = 0.0
Z_TRAVEL_UPPER = 0.09


def _add_box(
    part,
    size: tuple[float, float, float],
    xyz: tuple[float, float, float],
    material: str,
    name: str,
) -> None:
    part.visual(Box(size), origin=Origin(xyz=xyz), material=material, name=name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="orthogonal_xz_stage")

    model.material("base_graphite", rgba=(0.22, 0.24, 0.27, 1.0))
    model.material("machined_aluminum", rgba=(0.76, 0.78, 0.81, 1.0))
    model.material("guide_steel", rgba=(0.63, 0.66, 0.70, 1.0))
    model.material("pad_black", rgba=(0.10, 0.11, 0.12, 1.0))

    base = model.part("base")
    _add_box(
        base,
        (BASE_LENGTH, BASE_WIDTH, BASE_THICKNESS),
        (0.0, 0.0, BASE_THICKNESS / 2.0),
        "base_graphite",
        "base_plate",
    )
    _add_box(
        base,
        (BED_LENGTH, BED_WIDTH, BED_THICKNESS),
        (0.0, 0.0, BASE_THICKNESS + (BED_THICKNESS / 2.0)),
        "machined_aluminum",
        "x_bed",
    )
    _add_box(
        base,
        (X_RAIL_LENGTH, X_RAIL_WIDTH, X_RAIL_HEIGHT),
        (
            0.0,
            X_RAIL_Y_OFFSET,
            BASE_THICKNESS + BED_THICKNESS + (X_RAIL_HEIGHT / 2.0),
        ),
        "guide_steel",
        "left_x_rail",
    )
    _add_box(
        base,
        (X_RAIL_LENGTH, X_RAIL_WIDTH, X_RAIL_HEIGHT),
        (
            0.0,
            -X_RAIL_Y_OFFSET,
            BASE_THICKNESS + BED_THICKNESS + (X_RAIL_HEIGHT / 2.0),
        ),
        "guide_steel",
        "right_x_rail",
    )
    _add_box(
        base,
        (0.01, 0.11, 0.018),
        (0.195, 0.0, BASE_THICKNESS + BED_THICKNESS + 0.009),
        "base_graphite",
        "right_end_stop",
    )
    _add_box(
        base,
        (0.01, 0.11, 0.018),
        (-0.195, 0.0, BASE_THICKNESS + BED_THICKNESS + 0.009),
        "base_graphite",
        "left_end_stop",
    )
    base.inertial = Inertial.from_geometry(
        Box((BASE_LENGTH, BASE_WIDTH, X_RAIL_TOP_Z)),
        mass=7.5,
        origin=Origin(xyz=(0.0, 0.0, X_RAIL_TOP_Z / 2.0)),
    )

    x_carriage = model.part("x_carriage")
    _add_box(
        x_carriage,
        (CARRIAGE_LENGTH, CARRIAGE_WIDTH, CARRIAGE_HEIGHT),
        (0.0, 0.0, CARRIAGE_HEIGHT / 2.0),
        "machined_aluminum",
        "saddle",
    )
    _add_box(
        x_carriage,
        (COLUMN_WIDTH, COLUMN_DEPTH, COLUMN_HEIGHT),
        (
            0.0,
            COLUMN_DEPTH / 2.0,
            CARRIAGE_HEIGHT + (COLUMN_HEIGHT / 2.0),
        ),
        "machined_aluminum",
        "z_backplate",
    )
    _add_box(
        x_carriage,
        (CHEEK_WIDTH, CHEEK_DEPTH, CHEEK_HEIGHT),
        (
            -CHEEK_X_OFFSET,
            CHEEK_DEPTH / 2.0,
            CARRIAGE_HEIGHT + (CHEEK_HEIGHT / 2.0),
        ),
        "machined_aluminum",
        "left_cheek",
    )
    _add_box(
        x_carriage,
        (CHEEK_WIDTH, CHEEK_DEPTH, CHEEK_HEIGHT),
        (
            CHEEK_X_OFFSET,
            CHEEK_DEPTH / 2.0,
            CARRIAGE_HEIGHT + (CHEEK_HEIGHT / 2.0),
        ),
        "machined_aluminum",
        "right_cheek",
    )
    _add_box(
        x_carriage,
        (CROSSBAR_WIDTH, CROSSBAR_DEPTH, CROSSBAR_HEIGHT),
        (
            0.0,
            0.024 + (CROSSBAR_DEPTH / 2.0),
            CARRIAGE_HEIGHT + (CROSSBAR_HEIGHT / 2.0),
        ),
        "machined_aluminum",
        "lower_crossbar",
    )
    _add_box(
        x_carriage,
        (GUIDE_WIDTH, GUIDE_DEPTH, GUIDE_HEIGHT),
        (
            -GUIDE_X_OFFSET,
            0.018 + (GUIDE_DEPTH / 2.0),
            CARRIAGE_HEIGHT + (GUIDE_HEIGHT / 2.0),
        ),
        "guide_steel",
        "left_z_way",
    )
    _add_box(
        x_carriage,
        (GUIDE_WIDTH, GUIDE_DEPTH, GUIDE_HEIGHT),
        (
            GUIDE_X_OFFSET,
            0.018 + (GUIDE_DEPTH / 2.0),
            CARRIAGE_HEIGHT + (GUIDE_HEIGHT / 2.0),
        ),
        "guide_steel",
        "right_z_way",
    )
    x_carriage.inertial = Inertial.from_geometry(
        Box((CARRIAGE_LENGTH, CARRIAGE_WIDTH, CARRIAGE_HEIGHT + COLUMN_HEIGHT)),
        mass=2.4,
        origin=Origin(
            xyz=(0.0, 0.0, (CARRIAGE_HEIGHT + COLUMN_HEIGHT) / 2.0),
        ),
    )

    z_slide = model.part("z_slide")
    _add_box(
        z_slide,
        (SLIDE_WIDTH, SLIDE_DEPTH, SLIDE_HEIGHT),
        (0.0, SLIDE_DEPTH / 2.0, SLIDE_HEIGHT / 2.0),
        "machined_aluminum",
        "moving_plate",
    )
    _add_box(
        z_slide,
        (TOP_PAD_WIDTH, TOP_PAD_DEPTH, TOP_PAD_THICKNESS),
        (
            0.0,
            TOP_PAD_DEPTH / 2.0,
            SLIDE_HEIGHT + (TOP_PAD_THICKNESS / 2.0),
        ),
        "pad_black",
        "top_pad",
    )
    z_slide.inertial = Inertial.from_geometry(
        Box((TOP_PAD_WIDTH, TOP_PAD_DEPTH, SLIDE_HEIGHT + TOP_PAD_THICKNESS)),
        mass=1.1,
        origin=Origin(
            xyz=(
                0.0,
                TOP_PAD_DEPTH / 2.0,
                (SLIDE_HEIGHT + TOP_PAD_THICKNESS) / 2.0,
            ),
        ),
    )

    model.articulation(
        "x_axis",
        ArticulationType.PRISMATIC,
        parent=base,
        child=x_carriage,
        origin=Origin(xyz=(0.0, 0.0, X_RAIL_TOP_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=X_TRAVEL_LOWER,
            upper=X_TRAVEL_UPPER,
            effort=280.0,
            velocity=0.25,
        ),
    )
    model.articulation(
        "z_axis",
        ArticulationType.PRISMATIC,
        parent=x_carriage,
        child=z_slide,
        origin=Origin(xyz=(0.0, Z_JOINT_Y, Z_JOINT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=Z_TRAVEL_LOWER,
            upper=Z_TRAVEL_UPPER,
            effort=180.0,
            velocity=0.18,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    x_carriage = object_model.get_part("x_carriage")
    z_slide = object_model.get_part("z_slide")
    x_axis = object_model.get_articulation("x_axis")
    z_axis = object_model.get_articulation("z_axis")

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

    ctx.check(
        "x axis is prismatic along +x",
        x_axis.articulation_type == ArticulationType.PRISMATIC
        and tuple(x_axis.axis) == (1.0, 0.0, 0.0),
        details=f"type={x_axis.articulation_type}, axis={x_axis.axis}",
    )
    ctx.check(
        "z axis is prismatic along +z",
        z_axis.articulation_type == ArticulationType.PRISMATIC
        and tuple(z_axis.axis) == (0.0, 0.0, 1.0),
        details=f"type={z_axis.articulation_type}, axis={z_axis.axis}",
    )

    ctx.expect_contact(
        x_carriage,
        base,
        name="x carriage remains mounted on x rails",
    )
    ctx.expect_contact(
        z_slide,
        x_carriage,
        name="z slide remains mounted on carriage guides",
    )
    ctx.expect_overlap(
        x_carriage,
        base,
        axes="xy",
        min_overlap=0.09,
        name="x carriage footprint stays over base stage",
    )
    ctx.expect_overlap(
        z_slide,
        x_carriage,
        axes="x",
        min_overlap=0.05,
        name="z slide stays centered within carriage frame width",
    )

    rest_x_pos = ctx.part_world_position(x_carriage)
    rest_z_pos = ctx.part_world_position(z_slide)

    with ctx.pose(x_axis=0.09):
        x_moved_pos = ctx.part_world_position(x_carriage)
        ctx.expect_contact(
            x_carriage,
            base,
            name="x carriage stays supported at positive travel",
        )
    ctx.check(
        "x carriage translates only in x",
        rest_x_pos is not None
        and x_moved_pos is not None
        and x_moved_pos[0] > rest_x_pos[0] + 0.08
        and abs(x_moved_pos[1] - rest_x_pos[1]) < 1e-6
        and abs(x_moved_pos[2] - rest_x_pos[2]) < 1e-6,
        details=f"rest={rest_x_pos}, moved={x_moved_pos}",
    )

    with ctx.pose(z_axis=0.08):
        z_moved_pos = ctx.part_world_position(z_slide)
        ctx.expect_contact(
            z_slide,
            x_carriage,
            name="z slide stays guided when raised",
        )
    ctx.check(
        "z slide translates only in z",
        rest_z_pos is not None
        and z_moved_pos is not None
        and z_moved_pos[2] > rest_z_pos[2] + 0.07
        and abs(z_moved_pos[0] - rest_z_pos[0]) < 1e-6
        and abs(z_moved_pos[1] - rest_z_pos[1]) < 1e-6,
        details=f"rest={rest_z_pos}, moved={z_moved_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
