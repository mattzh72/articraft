from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _tube_shell_mesh(length: float, outer_radius: float, wall: float):
    front_z = length / 2.0
    back_z = -length / 2.0
    outer_profile = [
        (outer_radius - 0.003, back_z),
        (outer_radius, back_z + 0.010),
        (outer_radius, front_z - 0.018),
        (outer_radius + 0.003, front_z - 0.004),
        (outer_radius + 0.002, front_z),
    ]
    inner_profile = [
        (outer_radius - wall - 0.004, back_z + 0.010),
        (outer_radius - wall, back_z + 0.018),
        (outer_radius - wall, front_z - 0.012),
        (outer_radius - wall - 0.001, front_z),
    ]
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer_profile,
            inner_profile,
            segments=60,
            start_cap="flat",
            end_cap="round",
            lip_samples=8,
        ),
        "tube_shell",
    )


def _aabb_center(aabb):
    if aabb is None:
        return None
    lower, upper = aabb
    return tuple((lower[i] + upper[i]) * 0.5 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="travel_tabletop_reflector")

    laminate = model.material("laminate", rgba=(0.70, 0.56, 0.38, 1.0))
    black_paint = model.material("black_paint", rgba=(0.08, 0.08, 0.09, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.20, 0.22, 0.24, 1.0))
    aluminum = model.material("aluminum", rgba=(0.73, 0.75, 0.78, 1.0))
    glass = model.material("glass", rgba=(0.62, 0.73, 0.80, 0.95))

    disc_radius = 0.130
    disc_thickness = 0.020

    rocker_outer_width = 0.230
    rocker_depth = 0.170
    board_thickness = 0.012
    wall_height = 0.180
    wall_center_y = (rocker_outer_width / 2.0) - (board_thickness / 2.0)
    inner_face_y = wall_center_y - (board_thickness / 2.0)

    tube_length = 0.360
    tube_outer_radius = 0.090
    tube_wall = 0.008
    tube_shell = _tube_shell_mesh(tube_length, tube_outer_radius, tube_wall)
    tube_center_x = 0.045
    altitude_axis_z = 0.155

    stub_radius = 0.018
    stub_length = inner_face_y - (tube_outer_radius - 0.013)
    stub_half = stub_length / 2.0
    stub_center_y = inner_face_y - stub_half

    tabletop_disc = model.part("tabletop_disc")
    tabletop_disc.visual(
        Cylinder(radius=disc_radius, length=disc_thickness),
        origin=Origin(xyz=(0.0, 0.0, disc_thickness / 2.0)),
        material=black_paint,
        name="base_disc",
    )
    tabletop_disc.visual(
        Cylinder(radius=0.028, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=dark_metal,
        name="center_pivot_pad",
    )

    rocker_box = model.part("rocker_box")
    rocker_box.visual(
        Cylinder(radius=0.105, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=dark_metal,
        name="azimuth_bearing_plate",
    )
    rocker_box.visual(
        Box((0.230, rocker_depth, board_thickness)),
        origin=Origin(xyz=(0.0, 0.0, 0.010 + (board_thickness / 2.0))),
        material=laminate,
        name="bottom_board",
    )
    rocker_box.visual(
        Box((0.210, board_thickness, wall_height)),
        origin=Origin(xyz=(0.0, wall_center_y, 0.010 + (wall_height / 2.0))),
        material=laminate,
        name="left_wall",
    )
    rocker_box.visual(
        Box((0.210, board_thickness, wall_height)),
        origin=Origin(xyz=(0.0, -wall_center_y, 0.010 + (wall_height / 2.0))),
        material=laminate,
        name="right_wall",
    )
    rocker_box.visual(
        Box((board_thickness, rocker_outer_width - (2.0 * board_thickness), 0.050)),
        origin=Origin(xyz=(0.099, 0.0, 0.010 + (0.050 / 2.0))),
        material=laminate,
        name="front_panel",
    )
    rocker_box.visual(
        Box((board_thickness, rocker_outer_width - (2.0 * board_thickness), 0.042)),
        origin=Origin(xyz=(-0.099, 0.0, 0.010 + (0.042 / 2.0))),
        material=laminate,
        name="rear_panel",
    )
    optical_tube = model.part("optical_tube")
    optical_tube.visual(
        tube_shell,
        origin=Origin(xyz=(tube_center_x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black_paint,
        name="tube_shell",
    )
    optical_tube.visual(
        Cylinder(radius=stub_radius, length=stub_length),
        origin=Origin(xyz=(0.0, stub_center_y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="left_stub",
    )
    optical_tube.visual(
        Cylinder(radius=stub_radius, length=stub_length),
        origin=Origin(xyz=(0.0, -stub_center_y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="right_stub",
    )
    optical_tube.visual(
        Cylinder(radius=0.083, length=0.018),
        origin=Origin(xyz=(-0.112, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="mirror_cell",
    )
    optical_tube.visual(
        Cylinder(radius=0.064, length=0.006),
        origin=Origin(xyz=(-0.100, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=glass,
        name="primary_mirror",
    )
    optical_tube.visual(
        Box((0.025, 0.028, 0.020)),
        origin=Origin(xyz=(0.132, 0.0, 0.0), rpy=(math.pi / 4.0, 0.0, 0.0)),
        material=aluminum,
        name="secondary_hub",
    )
    optical_tube.visual(
        Box((0.0025, 0.168, 0.008)),
        origin=Origin(xyz=(0.132, 0.0, 0.0)),
        material=aluminum,
        name="spider_vane_y",
    )
    optical_tube.visual(
        Box((0.0025, 0.008, 0.168)),
        origin=Origin(xyz=(0.132, 0.0, 0.0)),
        material=aluminum,
        name="spider_vane_z",
    )
    optical_tube.visual(
        Box((0.070, 0.036, 0.028)),
        origin=Origin(xyz=(0.118, 0.058, 0.058)),
        material=dark_metal,
        name="focuser_body",
    )
    optical_tube.visual(
        Cylinder(radius=0.020, length=0.060),
        origin=Origin(xyz=(0.122, 0.078, 0.078), rpy=(-math.pi / 4.0, 0.0, 0.0)),
        material=black_paint,
        name="drawtube",
    )
    optical_tube.visual(
        Cylinder(radius=0.014, length=0.060),
        origin=Origin(xyz=(0.122, 0.105, 0.105), rpy=(-math.pi / 4.0, 0.0, 0.0)),
        material=black_paint,
        name="eyepiece",
    )

    model.articulation(
        "azimuth_bearing",
        ArticulationType.CONTINUOUS,
        parent=tabletop_disc,
        child=rocker_box,
        origin=Origin(xyz=(0.0, 0.0, disc_thickness)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=25.0, velocity=2.0),
    )
    model.articulation(
        "altitude_bearing",
        ArticulationType.REVOLUTE,
        parent=rocker_box,
        child=optical_tube,
        origin=Origin(xyz=(0.0, 0.0, altitude_axis_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.5,
            lower=-0.20,
            upper=1.35,
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
    tabletop_disc = object_model.get_part("tabletop_disc")
    rocker_box = object_model.get_part("rocker_box")
    optical_tube = object_model.get_part("optical_tube")
    azimuth_bearing = object_model.get_articulation("azimuth_bearing")
    altitude_bearing = object_model.get_articulation("altitude_bearing")

    ctx.expect_gap(
        rocker_box,
        tabletop_disc,
        axis="z",
        positive_elem="azimuth_bearing_plate",
        negative_elem="base_disc",
        max_gap=0.001,
        name="rocker bearing plate seats on tabletop disc",
    )
    ctx.expect_overlap(
        rocker_box,
        tabletop_disc,
        axes="xy",
        elem_a="azimuth_bearing_plate",
        elem_b="base_disc",
        min_overlap=0.20,
        name="azimuth bearing remains centered on tabletop disc",
    )
    ctx.expect_contact(
        optical_tube,
        rocker_box,
        elem_a="left_stub",
        elem_b="left_wall",
        contact_tol=0.0005,
        name="left altitude stub bears against left wall",
    )
    ctx.expect_contact(
        optical_tube,
        rocker_box,
        elem_a="right_stub",
        elem_b="right_wall",
        contact_tol=0.0005,
        name="right altitude stub bears against right wall",
    )
    ctx.expect_gap(
        optical_tube,
        rocker_box,
        axis="z",
        positive_elem="tube_shell",
        negative_elem="bottom_board",
        min_gap=0.010,
        name="tube clears rocker floor at rest",
    )

    rest_tube_center = _aabb_center(ctx.part_element_world_aabb(optical_tube, elem="tube_shell"))
    with ctx.pose({altitude_bearing: 0.80}):
        raised_tube_center = _aabb_center(ctx.part_element_world_aabb(optical_tube, elem="tube_shell"))
    ctx.check(
        "positive altitude raises the optical tube",
        rest_tube_center is not None
        and raised_tube_center is not None
        and raised_tube_center[2] > rest_tube_center[2] + 0.025,
        details=f"rest={rest_tube_center}, raised={raised_tube_center}",
    )

    rest_focuser_center = _aabb_center(ctx.part_element_world_aabb(optical_tube, elem="focuser_body"))
    with ctx.pose({azimuth_bearing: 1.0}):
        turned_focuser_center = _aabb_center(ctx.part_element_world_aabb(optical_tube, elem="focuser_body"))
    ctx.check(
        "positive azimuth swings the rocker and tube around the tabletop disc",
        rest_focuser_center is not None
        and turned_focuser_center is not None
        and turned_focuser_center[1] > rest_focuser_center[1] + 0.040,
        details=f"rest={rest_focuser_center}, turned={turned_focuser_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
