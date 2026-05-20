from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

ASSETS = AssetContext.from_script(__file__)

GROUND_RADIUS = 0.28
GROUND_THICKNESS = 0.024

ROCKER_LENGTH = 0.34
ROCKER_WIDTH = 0.34
ROCKER_BOTTOM_THICKNESS = 0.024
SIDE_PANEL_THICKNESS = 0.020
SIDE_PANEL_HEIGHT = 0.46
SIDE_PANEL_OFFSET_Y = 0.16
FRONT_BOARD_THICKNESS = 0.018
FRONT_BOARD_HEIGHT = 0.10
REAR_BOARD_HEIGHT = 0.07

ALTITUDE_AXIS_Z = 0.33
ALTITUDE_BEARING_RADIUS = 0.19
ALTITUDE_BEARING_LENGTH = 0.026
ALTITUDE_BEARING_CENTER_Y = (
    SIDE_PANEL_OFFSET_Y - SIDE_PANEL_THICKNESS / 2.0 - ALTITUDE_BEARING_LENGTH / 2.0
)

TUBE_HALF_LENGTH = 0.28
TUBE_CENTER_X = 0.12
TUBE_FRONT_X = TUBE_CENTER_X + TUBE_HALF_LENGTH
TUBE_REAR_X = TUBE_CENTER_X - TUBE_HALF_LENGTH
TUBE_OUTER_RADIUS = 0.148
TUBE_INNER_RADIUS = 0.136


def _annular_shell_mesh(
    filename: str,
    *,
    outer_radius: float,
    inner_radius: float,
    half_length: float,
    segments: int = 72,
):
    geometry = LatheGeometry.from_shell_profiles(
        [(outer_radius, -half_length), (outer_radius, half_length)],
        [(inner_radius, -half_length), (inner_radius, half_length)],
        segments=segments,
        start_cap="flat",
        end_cap="flat",
    )
    return mesh_from_geometry(geometry, ASSETS.mesh_path(filename))


def _tube_shell_mesh():
    geometry = LatheGeometry.from_shell_profiles(
        [
            (TUBE_OUTER_RADIUS - 0.004, -TUBE_HALF_LENGTH),
            (TUBE_OUTER_RADIUS, -TUBE_HALF_LENGTH + 0.020),
            (TUBE_OUTER_RADIUS, TUBE_HALF_LENGTH - 0.030),
            (TUBE_OUTER_RADIUS - 0.003, TUBE_HALF_LENGTH),
        ],
        [
            (TUBE_INNER_RADIUS - 0.002, -TUBE_HALF_LENGTH + 0.004),
            (TUBE_INNER_RADIUS, -TUBE_HALF_LENGTH + 0.022),
            (TUBE_INNER_RADIUS, TUBE_HALF_LENGTH - 0.032),
            (TUBE_INNER_RADIUS - 0.002, TUBE_HALF_LENGTH - 0.002),
        ],
        segments=80,
        start_cap="flat",
        end_cap="flat",
    )
    return mesh_from_geometry(geometry, ASSETS.mesh_path("dobsonian_tube_shell.obj"))
def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dobsonian_reflector", assets=ASSETS)

    plywood = model.material("plywood", rgba=(0.67, 0.53, 0.34, 1.0))
    tube_finish = model.material("tube_finish", rgba=(0.16, 0.18, 0.20, 1.0))
    hardware = model.material("hardware", rgba=(0.10, 0.11, 0.12, 1.0))
    trim = model.material("trim", rgba=(0.05, 0.05, 0.06, 1.0))

    tube_shell_mesh = _tube_shell_mesh()
    aperture_band_mesh = _annular_shell_mesh(
        "dobsonian_front_band.obj",
        outer_radius=TUBE_OUTER_RADIUS + 0.004,
        inner_radius=TUBE_INNER_RADIUS,
        half_length=0.006,
        segments=72,
    )

    base = model.part("ground_base")
    base.visual(
        Cylinder(radius=GROUND_RADIUS, length=GROUND_THICKNESS),
        origin=Origin(xyz=(0.0, 0.0, GROUND_THICKNESS / 2.0)),
        material=plywood,
        name="ground_board",
    )

    rocker = model.part("rocker_box")
    rocker.visual(
        Box((ROCKER_LENGTH, ROCKER_WIDTH, ROCKER_BOTTOM_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, ROCKER_BOTTOM_THICKNESS / 2.0)),
        material=plywood,
        name="bottom_board",
    )
    rocker.visual(
        Box((ROCKER_LENGTH, SIDE_PANEL_THICKNESS, SIDE_PANEL_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                SIDE_PANEL_OFFSET_Y,
                ROCKER_BOTTOM_THICKNESS + SIDE_PANEL_HEIGHT / 2.0,
            )
        ),
        material=plywood,
        name="left_side_panel",
    )
    rocker.visual(
        Box((ROCKER_LENGTH, SIDE_PANEL_THICKNESS, SIDE_PANEL_HEIGHT)),
        origin=Origin(
            xyz=(
                0.0,
                -SIDE_PANEL_OFFSET_Y,
                ROCKER_BOTTOM_THICKNESS + SIDE_PANEL_HEIGHT / 2.0,
            )
        ),
        material=plywood,
        name="right_side_panel",
    )
    rocker.visual(
        Box((FRONT_BOARD_THICKNESS, ROCKER_WIDTH - 2.0 * SIDE_PANEL_THICKNESS, FRONT_BOARD_HEIGHT)),
        origin=Origin(
            xyz=(
                ROCKER_LENGTH / 2.0 - FRONT_BOARD_THICKNESS / 2.0,
                0.0,
                ROCKER_BOTTOM_THICKNESS + FRONT_BOARD_HEIGHT / 2.0,
            )
        ),
        material=plywood,
        name="front_board",
    )
    rocker.visual(
        Box((FRONT_BOARD_THICKNESS, ROCKER_WIDTH - 2.0 * SIDE_PANEL_THICKNESS, REAR_BOARD_HEIGHT)),
        origin=Origin(
            xyz=(
                -ROCKER_LENGTH / 2.0 + FRONT_BOARD_THICKNESS / 2.0,
                0.0,
                ROCKER_BOTTOM_THICKNESS + REAR_BOARD_HEIGHT / 2.0,
            )
        ),
        material=plywood,
        name="rear_board",
    )

    tube = model.part("optical_tube")
    tube.visual(
        tube_shell_mesh,
        origin=Origin(xyz=(TUBE_CENTER_X, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=tube_finish,
        name="tube_shell",
    )
    tube.visual(
        aperture_band_mesh,
        origin=Origin(xyz=(TUBE_FRONT_X - 0.008, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=trim,
        name="front_aperture_band",
    )
    tube.visual(
        Cylinder(radius=ALTITUDE_BEARING_RADIUS, length=ALTITUDE_BEARING_LENGTH),
        origin=Origin(xyz=(0.0, ALTITUDE_BEARING_CENTER_Y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=plywood,
        name="left_alt_bearing",
    )
    tube.visual(
        Cylinder(radius=ALTITUDE_BEARING_RADIUS, length=ALTITUDE_BEARING_LENGTH),
        origin=Origin(xyz=(0.0, -ALTITUDE_BEARING_CENTER_Y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=plywood,
        name="right_alt_bearing",
    )
    tube.visual(
        Cylinder(radius=TUBE_INNER_RADIUS, length=0.018),
        origin=Origin(xyz=(TUBE_REAR_X + 0.016, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hardware,
        name="mirror_cell",
    )
    tube.visual(
        Box((0.055, 0.048, 0.020)),
        origin=Origin(xyz=(0.225, 0.076, 0.104), rpy=(0.0, 0.22, -0.14)),
        material=hardware,
        name="focuser_body",
    )
    tube.visual(
        Cylinder(radius=0.016, length=0.050),
        origin=Origin(xyz=(0.232, 0.095, 0.146)),
        material=trim,
        name="eyepiece",
    )

    model.articulation(
        "azimuth_swivel",
        ArticulationType.REVOLUTE,
        parent=base,
        child=rocker,
        origin=Origin(xyz=(0.0, 0.0, GROUND_THICKNESS)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.6,
            lower=-math.pi,
            upper=math.pi,
        ),
    )
    model.articulation(
        "altitude_trunnion",
        ArticulationType.REVOLUTE,
        parent=rocker,
        child=tube,
        origin=Origin(xyz=(0.0, 0.0, ALTITUDE_AXIS_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=1.2,
            lower=-0.05,
            upper=1.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    base = object_model.get_part("ground_base")
    rocker = object_model.get_part("rocker_box")
    tube = object_model.get_part("optical_tube")
    azimuth = object_model.get_articulation("azimuth_swivel")
    altitude = object_model.get_articulation("altitude_trunnion")
    ground_board = base.get_visual("ground_board")
    bottom_board = rocker.get_visual("bottom_board")
    left_side_panel = rocker.get_visual("left_side_panel")
    right_side_panel = rocker.get_visual("right_side_panel")
    front_board = rocker.get_visual("front_board")
    tube_shell = tube.get_visual("tube_shell")
    front_aperture_band = tube.get_visual("front_aperture_band")
    mirror_cell = tube.get_visual("mirror_cell")
    left_bearing = tube.get_visual("left_alt_bearing")
    right_bearing = tube.get_visual("right_alt_bearing")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Default exact visual sensor for joint mounting; keep unless scale makes it irrelevant.
    ctx.warn_if_articulation_origin_near_geometry(tol=0.15)
    # Default exact visual sensor for floating/disconnected subassemblies inside one part.
    ctx.warn_if_part_geometry_disconnected()
    # Default articulated-joint clearance gate; adapt only if the model is not articulated.
    ctx.check_articulation_overlaps(max_pose_samples=128)
    # Default broad overlap warning backstop; conservative and non-blocking by default.
    ctx.warn_if_overlaps(max_pose_samples=128, ignore_adjacent=True, ignore_fixed=True)

    ctx.expect_contact(rocker, base, elem_a=bottom_board, elem_b=ground_board)
    ctx.expect_overlap(rocker, base, axes="xy", min_overlap=0.20)
    ctx.expect_origin_distance(rocker, base, axes="xy", max_dist=0.01)

    ctx.expect_contact(tube, rocker, elem_a=left_bearing, elem_b=left_side_panel)
    ctx.expect_contact(tube, rocker, elem_a=right_bearing, elem_b=right_side_panel)
    ctx.expect_origin_distance(tube, rocker, axes="xy", max_dist=0.01)
    ctx.expect_gap(
        tube,
        tube,
        axis="x",
        positive_elem=front_aperture_band,
        negative_elem=mirror_cell,
        min_gap=0.48,
        name="aperture_sits_far_ahead_of_primary_cell",
    )
    ctx.expect_within(
        tube,
        tube,
        axes="yz",
        inner_elem=mirror_cell,
        outer_elem=tube_shell,
    )
    ctx.expect_gap(
        tube,
        rocker,
        axis="z",
        positive_elem=tube_shell,
        negative_elem=front_board,
        min_gap=0.04,
        name="tube_clears_front_board_in_rest_pose",
    )
    with ctx.pose({altitude: 1.0}):
        ctx.expect_contact(tube, rocker, elem_a=left_bearing, elem_b=left_side_panel)
        ctx.expect_contact(tube, rocker, elem_a=right_bearing, elem_b=right_side_panel)
        ctx.expect_gap(
            tube,
            rocker,
            axis="z",
            positive_elem=tube_shell,
            negative_elem=bottom_board,
            min_gap=0.015,
            name="tube_clears_bottom_board_at_high_altitude",
        )
    with ctx.pose({altitude: -0.05}):
        ctx.expect_contact(tube, rocker, elem_a=left_bearing, elem_b=left_side_panel)
        ctx.expect_contact(tube, rocker, elem_a=right_bearing, elem_b=right_side_panel)
        ctx.expect_gap(
            tube,
            rocker,
            axis="z",
            positive_elem=tube_shell,
            negative_elem=front_board,
            min_gap=0.02,
            name="tube_clears_front_board_near_horizon",
        )
    with ctx.pose({azimuth: 1.7, altitude: 0.7}):
        ctx.expect_contact(rocker, base, elem_a=bottom_board, elem_b=ground_board)
        ctx.expect_overlap(rocker, base, axes="xy", min_overlap=0.20)
        ctx.expect_contact(tube, rocker, elem_a=left_bearing, elem_b=left_side_panel)
        ctx.expect_contact(tube, rocker, elem_a=right_bearing, elem_b=right_side_panel)
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
