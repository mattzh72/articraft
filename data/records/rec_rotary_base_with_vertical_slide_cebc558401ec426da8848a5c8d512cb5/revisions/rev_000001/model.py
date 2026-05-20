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


BASE_RADIUS = 0.23
BASE_THICKNESS = 0.045
PLINTH_RADIUS = 0.125
PLINTH_HEIGHT = 0.050
BEARING_CAP_RADIUS = 0.095
BEARING_CAP_HEIGHT = 0.018

BASE_TO_COLUMN_Z = BASE_THICKNESS + PLINTH_HEIGHT + BEARING_CAP_HEIGHT

COLUMN_FLANGE_RADIUS = 0.085
COLUMN_FLANGE_HEIGHT = 0.020
MAST_RADIUS = 0.055
MAST_HEIGHT = 0.620
TOP_CAP_RADIUS = 0.070
TOP_CAP_HEIGHT = 0.035

GUIDE_BACKBONE_WIDTH = 0.090
GUIDE_BACKBONE_DEPTH = 0.030
GUIDE_BACKBONE_HEIGHT = 0.430
GUIDE_BACKBONE_BOTTOM_Z = 0.160
GUIDE_BACKBONE_CENTER_Y = MAST_RADIUS + GUIDE_BACKBONE_DEPTH / 2.0 - 0.004

GUIDE_TRACK_WIDTH = 0.015
GUIDE_TRACK_DEPTH = 0.012
GUIDE_TRACK_HEIGHT = 0.390
GUIDE_TRACK_BOTTOM_Z = 0.180
GUIDE_TRACK_CENTER_Y = (
    GUIDE_BACKBONE_CENTER_Y
    + GUIDE_BACKBONE_DEPTH / 2.0
    + GUIDE_TRACK_DEPTH / 2.0
    - 0.002
)
GUIDE_TRACK_OFFSET_X = 0.028

CARRIAGE_WIDTH = 0.150
CARRIAGE_HEIGHT = 0.160
CARRIAGE_SHELL_DEPTH = 0.070
CARRIAGE_FRONT_PLATE_DEPTH = 0.024
CARRIAGE_FRONT_PAD_DEPTH = 0.020
CARRIAGE_CHEEK_THICKNESS = 0.025
CARRIAGE_BRIDGE_THICKNESS = 0.020
CARRIAGE_BRIDGE_BLOCK_WIDTH = 0.030
CARRIAGE_FRAME_Y = 0.079
CARRIAGE_REST_Z = 0.280
LIFT_TRAVEL = 0.180

YAW_LIMIT = 2.7


def _carriage_shell_center_y_world() -> float:
    return 0.102


def _carriage_front_plate_center_y_world() -> float:
    return 0.125


def _carriage_front_pad_center_y_world() -> float:
    return 0.147


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rotary_column_with_lift_carriage")

    model.material("base_paint", rgba=(0.20, 0.22, 0.24, 1.0))
    model.material("machine_gray", rgba=(0.67, 0.69, 0.72, 1.0))
    model.material("rail_steel", rgba=(0.78, 0.80, 0.82, 1.0))
    model.material("safety_orange", rgba=(0.92, 0.47, 0.14, 1.0))
    model.material("dark_polymer", rgba=(0.17, 0.18, 0.20, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=BASE_RADIUS, length=BASE_THICKNESS),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS / 2.0)),
        material="base_paint",
        name="base_plate",
    )
    base.visual(
        Cylinder(radius=PLINTH_RADIUS, length=PLINTH_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS + PLINTH_HEIGHT / 2.0)),
        material="base_paint",
        name="plinth",
    )
    base.visual(
        Cylinder(radius=BEARING_CAP_RADIUS, length=BEARING_CAP_HEIGHT),
        origin=Origin(
            xyz=(0.0, 0.0, BASE_THICKNESS + PLINTH_HEIGHT + BEARING_CAP_HEIGHT / 2.0)
        ),
        material="rail_steel",
        name="bearing_cap",
    )
    base.inertial = Inertial.from_geometry(
        Cylinder(radius=BASE_RADIUS, length=BASE_THICKNESS),
        mass=36.0,
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS / 2.0)),
    )

    column = model.part("column")
    column.visual(
        Cylinder(radius=COLUMN_FLANGE_RADIUS, length=COLUMN_FLANGE_HEIGHT),
        origin=Origin(xyz=(0.0, 0.0, COLUMN_FLANGE_HEIGHT / 2.0)),
        material="rail_steel",
        name="rotary_flange",
    )
    column.visual(
        Cylinder(radius=MAST_RADIUS, length=MAST_HEIGHT),
        origin=Origin(
            xyz=(0.0, 0.0, COLUMN_FLANGE_HEIGHT + MAST_HEIGHT / 2.0)
        ),
        material="machine_gray",
        name="mast",
    )
    column.visual(
        Cylinder(radius=TOP_CAP_RADIUS, length=TOP_CAP_HEIGHT),
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                COLUMN_FLANGE_HEIGHT + MAST_HEIGHT + TOP_CAP_HEIGHT / 2.0,
            )
        ),
        material="rail_steel",
        name="top_cap",
    )
    column.visual(
        Box(
            (
                GUIDE_BACKBONE_WIDTH,
                GUIDE_BACKBONE_DEPTH,
                GUIDE_BACKBONE_HEIGHT,
            )
        ),
        origin=Origin(
            xyz=(
                0.0,
                GUIDE_BACKBONE_CENTER_Y,
                GUIDE_BACKBONE_BOTTOM_Z + GUIDE_BACKBONE_HEIGHT / 2.0,
            )
        ),
        material="machine_gray",
        name="guide_backbone",
    )
    column.visual(
        Box((GUIDE_TRACK_WIDTH, GUIDE_TRACK_DEPTH, GUIDE_TRACK_HEIGHT)),
        origin=Origin(
            xyz=(
                -GUIDE_TRACK_OFFSET_X,
                GUIDE_TRACK_CENTER_Y,
                GUIDE_TRACK_BOTTOM_Z + GUIDE_TRACK_HEIGHT / 2.0,
            )
        ),
        material="rail_steel",
        name="guide_track_left",
    )
    column.visual(
        Box((GUIDE_TRACK_WIDTH, GUIDE_TRACK_DEPTH, GUIDE_TRACK_HEIGHT)),
        origin=Origin(
            xyz=(
                GUIDE_TRACK_OFFSET_X,
                GUIDE_TRACK_CENTER_Y,
                GUIDE_TRACK_BOTTOM_Z + GUIDE_TRACK_HEIGHT / 2.0,
            )
        ),
        material="rail_steel",
        name="guide_track_right",
    )
    column.inertial = Inertial.from_geometry(
        Cylinder(radius=MAST_RADIUS, length=MAST_HEIGHT + COLUMN_FLANGE_HEIGHT),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, (MAST_HEIGHT + COLUMN_FLANGE_HEIGHT) / 2.0)),
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box(
            (
                CARRIAGE_WIDTH,
                CARRIAGE_FRONT_PLATE_DEPTH,
                CARRIAGE_HEIGHT,
            )
        ),
        origin=Origin(
            xyz=(
                0.0,
                _carriage_front_plate_center_y_world() - CARRIAGE_FRAME_Y,
                0.0,
            )
        ),
        material="safety_orange",
        name="front_plate",
    )
    carriage.visual(
        Box(
            (
                CARRIAGE_CHEEK_THICKNESS,
                CARRIAGE_SHELL_DEPTH,
                CARRIAGE_HEIGHT - 2.0 * CARRIAGE_BRIDGE_THICKNESS,
            )
        ),
        origin=Origin(
            xyz=(
                -(CARRIAGE_WIDTH - CARRIAGE_CHEEK_THICKNESS) / 2.0,
                _carriage_shell_center_y_world() - CARRIAGE_FRAME_Y,
                0.0,
            )
        ),
        material="dark_polymer",
        name="left_cheek",
    )
    carriage.visual(
        Box(
            (
                CARRIAGE_CHEEK_THICKNESS,
                CARRIAGE_SHELL_DEPTH,
                CARRIAGE_HEIGHT - 2.0 * CARRIAGE_BRIDGE_THICKNESS,
            )
        ),
        origin=Origin(
            xyz=(
                (CARRIAGE_WIDTH - CARRIAGE_CHEEK_THICKNESS) / 2.0,
                _carriage_shell_center_y_world() - CARRIAGE_FRAME_Y,
                0.0,
            )
        ),
        material="dark_polymer",
        name="right_cheek",
    )
    carriage.visual(
        Box(
            (
                CARRIAGE_BRIDGE_BLOCK_WIDTH,
                CARRIAGE_SHELL_DEPTH,
                CARRIAGE_BRIDGE_THICKNESS,
            )
        ),
        origin=Origin(
            xyz=(
                -(CARRIAGE_WIDTH - CARRIAGE_BRIDGE_BLOCK_WIDTH) / 2.0,
                _carriage_shell_center_y_world() - CARRIAGE_FRAME_Y,
                (CARRIAGE_HEIGHT - CARRIAGE_BRIDGE_THICKNESS) / 2.0,
            )
        ),
        material="dark_polymer",
        name="top_bridge_left",
    )
    carriage.visual(
        Box(
            (
                CARRIAGE_BRIDGE_BLOCK_WIDTH,
                CARRIAGE_SHELL_DEPTH,
                CARRIAGE_BRIDGE_THICKNESS,
            )
        ),
        origin=Origin(
            xyz=(
                (CARRIAGE_WIDTH - CARRIAGE_BRIDGE_BLOCK_WIDTH) / 2.0,
                _carriage_shell_center_y_world() - CARRIAGE_FRAME_Y,
                (CARRIAGE_HEIGHT - CARRIAGE_BRIDGE_THICKNESS) / 2.0,
            )
        ),
        material="dark_polymer",
        name="top_bridge_right",
    )
    carriage.visual(
        Box(
            (
                CARRIAGE_BRIDGE_BLOCK_WIDTH,
                CARRIAGE_SHELL_DEPTH,
                CARRIAGE_BRIDGE_THICKNESS,
            )
        ),
        origin=Origin(
            xyz=(
                -(CARRIAGE_WIDTH - CARRIAGE_BRIDGE_BLOCK_WIDTH) / 2.0,
                _carriage_shell_center_y_world() - CARRIAGE_FRAME_Y,
                -(CARRIAGE_HEIGHT - CARRIAGE_BRIDGE_THICKNESS) / 2.0,
            )
        ),
        material="dark_polymer",
        name="bottom_bridge_left",
    )
    carriage.visual(
        Box(
            (
                CARRIAGE_BRIDGE_BLOCK_WIDTH,
                CARRIAGE_SHELL_DEPTH,
                CARRIAGE_BRIDGE_THICKNESS,
            )
        ),
        origin=Origin(
            xyz=(
                (CARRIAGE_WIDTH - CARRIAGE_BRIDGE_BLOCK_WIDTH) / 2.0,
                _carriage_shell_center_y_world() - CARRIAGE_FRAME_Y,
                -(CARRIAGE_HEIGHT - CARRIAGE_BRIDGE_THICKNESS) / 2.0,
            )
        ),
        material="dark_polymer",
        name="bottom_bridge_right",
    )
    carriage.visual(
        Box((0.100, CARRIAGE_FRONT_PAD_DEPTH, 0.075)),
        origin=Origin(
            xyz=(
                0.0,
                _carriage_front_pad_center_y_world() - CARRIAGE_FRAME_Y,
                0.0,
            )
        ),
        material="safety_orange",
        name="mount_pad",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((CARRIAGE_WIDTH, 0.090, CARRIAGE_HEIGHT)),
        mass=4.5,
        origin=Origin(xyz=(0.0, 0.030, 0.0)),
    )

    model.articulation(
        "base_yaw",
        ArticulationType.REVOLUTE,
        parent=base,
        child=column,
        origin=Origin(xyz=(0.0, 0.0, BASE_TO_COLUMN_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=140.0,
            velocity=1.2,
            lower=-YAW_LIMIT,
            upper=YAW_LIMIT,
        ),
    )
    model.articulation(
        "column_lift",
        ArticulationType.PRISMATIC,
        parent=column,
        child=carriage,
        origin=Origin(xyz=(0.0, CARRIAGE_FRAME_Y, CARRIAGE_REST_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=450.0,
            velocity=0.30,
            lower=0.0,
            upper=LIFT_TRAVEL,
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

    base = object_model.get_part("base")
    column = object_model.get_part("column")
    carriage = object_model.get_part("carriage")
    yaw = object_model.get_articulation("base_yaw")
    lift = object_model.get_articulation("column_lift")

    ctx.expect_contact(
        column,
        base,
        elem_a="rotary_flange",
        elem_b="bearing_cap",
        contact_tol=1e-6,
        name="rotary flange seats on base bearing cap",
    )
    ctx.expect_overlap(
        column,
        base,
        axes="xy",
        elem_a="rotary_flange",
        elem_b="bearing_cap",
        min_overlap=0.16,
        name="column stays centered on the circular base",
    )
    ctx.expect_overlap(
        carriage,
        column,
        axes="xy",
        elem_b="guide_backbone",
        min_overlap=0.014,
        name="carriage straddles the guide backbone at rest",
    )
    ctx.expect_overlap(
        carriage,
        column,
        axes="z",
        elem_b="guide_backbone",
        min_overlap=0.15,
        name="carriage overlaps the vertical guide at rest",
    )

    rest_pos = ctx.part_world_position(carriage)
    with ctx.pose({yaw: 1.0}):
        ctx.expect_contact(
            column,
            base,
            elem_a="rotary_flange",
            elem_b="bearing_cap",
            contact_tol=1e-6,
            name="rotary flange remains seated while yawed",
        )
        yawed_pos = ctx.part_world_position(carriage)

    with ctx.pose({lift: LIFT_TRAVEL}):
        ctx.expect_overlap(
            carriage,
            column,
            axes="z",
            elem_b="guide_backbone",
            min_overlap=0.15,
            name="carriage remains engaged with the guide at full lift",
        )
        lifted_pos = ctx.part_world_position(carriage)

    ctx.check(
        "yaw swings the carriage around the base centerline",
        rest_pos is not None
        and yawed_pos is not None
        and abs(yawed_pos[0] - rest_pos[0]) > 0.05
        and abs(yawed_pos[2] - rest_pos[2]) < 0.001,
        details=f"rest={rest_pos}, yawed={yawed_pos}",
    )
    ctx.check(
        "prismatic stage raises the carriage upward",
        rest_pos is not None
        and lifted_pos is not None
        and lifted_pos[2] > rest_pos[2] + 0.15,
        details=f"rest={rest_pos}, lifted={lifted_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
