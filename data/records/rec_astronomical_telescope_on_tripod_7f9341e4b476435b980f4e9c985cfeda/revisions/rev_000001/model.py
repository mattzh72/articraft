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
    ExtrudeWithHolesGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _circle_profile(
    radius: float,
    *,
    segments: int = 40,
    center: tuple[float, float] = (0.0, 0.0),
) -> list[tuple[float, float]]:
    cx, cy = center
    return [
        (
            cx + radius * math.cos((2.0 * math.pi * index) / segments),
            cy + radius * math.sin((2.0 * math.pi * index) / segments),
        )
        for index in range(segments)
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="grab_and_go_refractor_alt_az")

    birch_ply = model.material("birch_ply", rgba=(0.69, 0.56, 0.36, 1.0))
    baltic_edge = model.material("baltic_edge", rgba=(0.58, 0.45, 0.28, 1.0))
    gloss_white = model.material("gloss_white", rgba=(0.92, 0.93, 0.95, 1.0))
    satin_black = model.material("satin_black", rgba=(0.10, 0.11, 0.12, 1.0))
    anodized = model.material("anodized", rgba=(0.34, 0.36, 0.39, 1.0))
    aluminum = model.material("aluminum", rgba=(0.75, 0.77, 0.80, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.09, 1.0))

    board_radius = 0.24
    board_thickness = 0.024
    bearing_radius = 0.115
    bearing_height = 0.010
    socket_outer_radius = 0.019
    socket_inner_radius = 0.0085
    socket_depth = 0.018

    side_wall_width = 0.300
    side_wall_height = 0.330
    side_wall_thickness = 0.018
    rocker_inner_width = 0.180
    side_wall_center_y = (rocker_inner_width * 0.5) + (side_wall_thickness * 0.5)
    floor_length = 0.280
    floor_width = (2.0 * side_wall_center_y) + side_wall_thickness
    floor_thickness = 0.018
    azimuth_disk_thickness = 0.014
    side_wall_base_z = azimuth_disk_thickness + floor_thickness
    trunnion_shaft_radius = 0.014
    trunnion_hole_radius = 0.017
    trunnion_disc_radius = 0.040
    trunnion_disc_thickness = 0.012
    trunnion_shaft_length = 0.028
    trunnion_axis_z = side_wall_base_z + 0.223

    side_wall_profile = [
        (-0.145, 0.000),
        (0.125, 0.000),
        (0.145, 0.040),
        (0.145, 0.305),
        (0.115, 0.330),
        (-0.145, 0.330),
    ]
    side_wall_holes = [_circle_profile(trunnion_hole_radius, segments=44, center=(0.000, 0.255))]
    side_wall_geom = ExtrudeWithHolesGeometry(
        side_wall_profile,
        side_wall_holes,
        height=side_wall_thickness,
        center=True,
    ).rotate_x(math.pi / 2.0)
    side_wall_mesh = _save_mesh("rocker_side_wall", side_wall_geom)

    socket_geom = LatheGeometry.from_shell_profiles(
        [(socket_outer_radius, 0.000), (socket_outer_radius, socket_depth)],
        [(socket_inner_radius, 0.000), (socket_inner_radius, socket_depth)],
        segments=40,
    )
    socket_mesh = _save_mesh("foot_socket", socket_geom)

    azimuth_socket_geom = LatheGeometry.from_shell_profiles(
        [(0.032, -0.024), (0.032, 0.024)],
        [(0.0255, -0.024), (0.0255, 0.024)],
        segments=48,
    )
    azimuth_socket_mesh = _save_mesh("azimuth_pivot_socket", azimuth_socket_geom)
    azimuth_disk_geom = LatheGeometry.from_shell_profiles(
        [(0.110, -azimuth_disk_thickness * 0.5), (0.110, azimuth_disk_thickness * 0.5)],
        [(0.0255, -azimuth_disk_thickness * 0.5), (0.0255, azimuth_disk_thickness * 0.5)],
        segments=56,
    )
    azimuth_disk_mesh = _save_mesh("rocker_azimuth_disk", azimuth_disk_geom)

    tube_shell_geom = LatheGeometry.from_shell_profiles(
        [
            (0.051, -0.340),
            (0.055, -0.328),
            (0.055, 0.200),
            (0.060, 0.205),
            (0.062, 0.378),
            (0.064, 0.402),
        ],
        [
            (0.045, -0.334),
            (0.048, -0.318),
            (0.048, 0.192),
            (0.053, 0.198),
            (0.055, 0.366),
            (0.056, 0.392),
        ],
        segments=64,
    ).rotate_y(math.pi / 2.0)
    tube_shell_mesh = _save_mesh("refractor_tube_shell", tube_shell_geom)
    focuser_adapter_geom = LatheGeometry.from_shell_profiles(
        [(0.056, -0.013), (0.056, 0.013)],
        [(0.032, -0.013), (0.032, 0.013)],
        segments=48,
    ).rotate_y(math.pi / 2.0)
    focuser_adapter_mesh = _save_mesh("refractor_focuser_adapter", focuser_adapter_geom)

    base_board = model.part("base_board")
    base_board.visual(
        Cylinder(radius=board_radius, length=board_thickness),
        origin=Origin(xyz=(0.0, 0.0, board_thickness * 0.5)),
        material=birch_ply,
        name="base_disk",
    )
    base_board.visual(
        Cylinder(radius=bearing_radius, length=bearing_height),
        origin=Origin(xyz=(0.0, 0.0, board_thickness + (bearing_height * 0.5))),
        material=anodized,
        name="azimuth_bearing_ring",
    )
    base_board.visual(
        Cylinder(radius=0.024, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, board_thickness + 0.018)),
        material=satin_black,
        name="center_pivot_boss",
    )

    foot_mounts = {
        "front": (0.170, 0.000),
        "rear_left": (-0.092, 0.160),
        "rear_right": (-0.092, -0.160),
    }
    for mount_name, (mx, my) in foot_mounts.items():
        base_board.visual(
            socket_mesh,
            origin=Origin(xyz=(mx, my, -socket_depth)),
            material=anodized,
            name=f"{mount_name}_socket",
        )
    base_board.inertial = Inertial.from_geometry(
        Box((0.500, 0.500, 0.080)),
        mass=3.8,
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
    )

    rocker_box = model.part("rocker_box")
    rocker_box.visual(
        azimuth_disk_mesh,
        origin=Origin(xyz=(0.0, 0.0, azimuth_disk_thickness * 0.5)),
        material=anodized,
        name="azimuth_disk",
    )
    rocker_box.visual(
        Box((floor_length, floor_width, floor_thickness)),
        origin=Origin(xyz=(0.0, 0.0, azimuth_disk_thickness + (floor_thickness * 0.5))),
        material=birch_ply,
        name="rocker_floor",
    )
    rocker_box.visual(
        Box((0.060, side_wall_thickness, side_wall_height)),
        origin=Origin(xyz=(-0.115, side_wall_center_y, side_wall_base_z + (side_wall_height * 0.5))),
        material=baltic_edge,
        name="left_rear_stile",
    )
    rocker_box.visual(
        Box((0.160, side_wall_thickness, 0.092)),
        origin=Origin(xyz=(-0.005, side_wall_center_y, 0.316)),
        material=baltic_edge,
        name="left_side_wall_cap",
    )
    rocker_box.visual(
        Box((0.040, side_wall_thickness, 0.152)),
        origin=Origin(xyz=(0.105, side_wall_center_y, 0.286)),
        material=baltic_edge,
        name="left_front_pillar",
    )
    rocker_box.visual(
        Box((0.070, side_wall_thickness, 0.178)),
        origin=Origin(xyz=(0.110, side_wall_center_y, 0.121)),
        material=baltic_edge,
        name="left_front_cheek",
    )
    rocker_box.visual(
        Box((0.060, side_wall_thickness, side_wall_height)),
        origin=Origin(xyz=(-0.115, -side_wall_center_y, side_wall_base_z + (side_wall_height * 0.5))),
        material=baltic_edge,
        name="right_rear_stile",
    )
    rocker_box.visual(
        Box((0.160, side_wall_thickness, 0.092)),
        origin=Origin(xyz=(-0.005, -side_wall_center_y, 0.316)),
        material=baltic_edge,
        name="right_side_wall_cap",
    )
    rocker_box.visual(
        Box((0.040, side_wall_thickness, 0.152)),
        origin=Origin(xyz=(0.105, -side_wall_center_y, 0.286)),
        material=baltic_edge,
        name="right_front_pillar",
    )
    rocker_box.visual(
        Box((0.070, side_wall_thickness, 0.178)),
        origin=Origin(xyz=(0.110, -side_wall_center_y, 0.121)),
        material=baltic_edge,
        name="right_front_cheek",
    )
    rocker_box.visual(
        Box((0.018, rocker_inner_width, 0.100)),
        origin=Origin(xyz=(0.128, 0.0, side_wall_base_z + 0.050)),
        material=baltic_edge,
        name="front_panel",
    )
    rocker_box.visual(
        Box((0.018, rocker_inner_width, 0.160)),
        origin=Origin(xyz=(-0.128, 0.0, side_wall_base_z + 0.080)),
        material=baltic_edge,
        name="rear_panel",
    )
    rocker_box.visual(
        azimuth_socket_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.024)),
        material=satin_black,
        name="pivot_socket",
    )
    rocker_box.inertial = Inertial.from_geometry(
        Box((0.320, 0.260, 0.360)),
        mass=4.6,
        origin=Origin(xyz=(0.0, 0.0, 0.180)),
    )

    tube_assembly = model.part("tube_assembly")
    tube_assembly.visual(
        tube_shell_mesh,
        material=gloss_white,
        name="optical_tube_shell",
    )
    tube_assembly.visual(
        focuser_adapter_mesh,
        origin=Origin(xyz=(-0.347, 0.0, 0.0)),
        material=anodized,
        name="focuser_adapter",
    )
    tube_assembly.visual(
        Cylinder(radius=0.064, length=0.028),
        origin=Origin(xyz=(0.392, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=anodized,
        name="objective_cell",
    )
    tube_assembly.visual(
        Cylinder(radius=0.032, length=0.100),
        origin=Origin(xyz=(-0.386, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_black,
        name="focuser_body",
    )
    tube_assembly.visual(
        Cylinder(radius=0.020, length=0.080),
        origin=Origin(xyz=(-0.466, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=aluminum,
        name="drawtube",
    )
    tube_assembly.visual(
        Cylinder(radius=0.013, length=0.050),
        origin=Origin(xyz=(-0.454, 0.0, 0.008), rpy=(0.0, -math.pi / 4.0, 0.0)),
        material=aluminum,
        name="diagonal_nosepiece",
    )
    tube_assembly.visual(
        Cylinder(radius=0.018, length=0.100),
        origin=Origin(xyz=(-0.490, 0.0, 0.040), rpy=(0.0, -math.pi / 4.0, 0.0)),
        material=satin_black,
        name="diagonal_body",
    )
    tube_assembly.visual(
        Cylinder(radius=0.018, length=0.050),
        origin=Origin(xyz=(-0.530, 0.0, 0.083)),
        material=satin_black,
        name="eyepiece_holder",
    )
    tube_assembly.visual(
        Cylinder(radius=0.015, length=0.085),
        origin=Origin(xyz=(-0.530, 0.0, 0.112)),
        material=satin_black,
        name="eyepiece",
    )
    tube_assembly.visual(
        Box((0.120, 0.132, 0.048)),
        origin=Origin(xyz=(0.005, 0.0, -0.048)),
        material=anodized,
        name="saddle_block",
    )
    tube_assembly.visual(
        Box((0.096, 0.034, 0.074)),
        origin=Origin(xyz=(0.005, 0.071, -0.006)),
        material=anodized,
        name="left_ear",
    )
    tube_assembly.visual(
        Box((0.096, 0.034, 0.074)),
        origin=Origin(xyz=(0.005, -0.071, -0.006)),
        material=anodized,
        name="right_ear",
    )
    tube_assembly.visual(
        Cylinder(radius=trunnion_shaft_radius, length=trunnion_shaft_length),
        origin=Origin(xyz=(0.0, side_wall_center_y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=anodized,
        name="left_trunnion_shaft",
    )
    tube_assembly.visual(
        Cylinder(radius=trunnion_disc_radius, length=trunnion_disc_thickness),
        origin=Origin(
            xyz=(0.0, side_wall_center_y + (side_wall_thickness * 0.5) + (trunnion_disc_thickness * 0.5), 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=anodized,
        name="left_trunnion",
    )
    tube_assembly.visual(
        Cylinder(radius=trunnion_shaft_radius, length=trunnion_shaft_length),
        origin=Origin(xyz=(0.0, -side_wall_center_y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=anodized,
        name="right_trunnion_shaft",
    )
    tube_assembly.visual(
        Cylinder(radius=trunnion_disc_radius, length=trunnion_disc_thickness),
        origin=Origin(
            xyz=(0.0, -side_wall_center_y - (side_wall_thickness * 0.5) - (trunnion_disc_thickness * 0.5), 0.0),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=anodized,
        name="right_trunnion",
    )
    tube_assembly.inertial = Inertial.from_geometry(
        Box((0.900, 0.230, 0.240)),
        mass=4.9,
        origin=Origin(xyz=(0.000, 0.0, 0.0)),
    )

    foot_part_specs = (
        ("front_foot", "front", 0.20),
        ("rear_left_foot", "rear_left", 0.20),
        ("rear_right_foot", "rear_right", 0.20),
    )
    for part_name, _, mass in foot_part_specs:
        foot = model.part(part_name)
        foot.visual(
            Cylinder(radius=0.007, length=0.050),
            origin=Origin(xyz=(0.0, 0.0, -0.025)),
            material=aluminum,
            name="threaded_stem",
        )
        foot.visual(
            Cylinder(radius=0.012, length=0.008),
            origin=Origin(xyz=(0.0, 0.0, -0.004)),
            material=anodized,
            name="lock_collar",
        )
        foot.visual(
            Cylinder(radius=0.010, length=0.014),
            origin=Origin(xyz=(0.0, 0.0, -0.057)),
            material=anodized,
            name="ball_post",
        )
        foot.visual(
            Cylinder(radius=0.018, length=0.006),
            origin=Origin(xyz=(0.0, 0.0, -0.067)),
            material=rubber,
            name="foot_pad",
        )
        foot.inertial = Inertial.from_geometry(
            Box((0.040, 0.040, 0.080)),
            mass=mass,
            origin=Origin(xyz=(0.0, 0.0, -0.040)),
        )

    model.articulation(
        "azimuth_rotation",
        ArticulationType.CONTINUOUS,
        parent=base_board,
        child=rocker_box,
        origin=Origin(xyz=(0.0, 0.0, board_thickness + bearing_height)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=1.5,
        ),
    )
    model.articulation(
        "altitude_axis",
        ArticulationType.REVOLUTE,
        parent=rocker_box,
        child=tube_assembly,
        origin=Origin(xyz=(0.0, 0.0, trunnion_axis_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.2,
            lower=-0.20,
            upper=1.25,
        ),
    )

    for part_name, mount_name, _ in foot_part_specs:
        mx, my = foot_mounts[mount_name]
        model.articulation(
            f"{mount_name}_foot_height",
            ArticulationType.PRISMATIC,
            parent=base_board,
            child=part_name,
            origin=Origin(xyz=(mx, my, -socket_depth)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=8.0,
                velocity=0.040,
                lower=0.0,
                upper=0.030,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base_board = object_model.get_part("base_board")
    rocker_box = object_model.get_part("rocker_box")
    tube_assembly = object_model.get_part("tube_assembly")
    front_foot = object_model.get_part("front_foot")
    rear_left_foot = object_model.get_part("rear_left_foot")
    rear_right_foot = object_model.get_part("rear_right_foot")

    azimuth_rotation = object_model.get_articulation("azimuth_rotation")
    altitude_axis = object_model.get_articulation("altitude_axis")
    front_foot_height = object_model.get_articulation("front_foot_height")

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
        rocker_box,
        base_board,
        elem_a="azimuth_disk",
        elem_b="azimuth_bearing_ring",
        name="rocker rides on the azimuth bearing ring",
    )
    ctx.expect_overlap(
        rocker_box,
        base_board,
        axes="xy",
        elem_a="azimuth_disk",
        elem_b="azimuth_bearing_ring",
        min_overlap=0.200,
        name="azimuth bearing has broad support footprint",
    )
    ctx.expect_contact(
        tube_assembly,
        rocker_box,
        elem_a="left_trunnion",
        elem_b="left_side_wall_cap",
        name="left altitude trunnion is seated in the left wall bearing",
    )
    ctx.expect_contact(
        tube_assembly,
        rocker_box,
        elem_a="right_trunnion",
        elem_b="right_side_wall_cap",
        name="right altitude trunnion is seated in the right wall bearing",
    )
    ctx.expect_contact(
        front_foot,
        base_board,
        elem_a="lock_collar",
        elem_b="front_socket",
        name="front leveling foot seats in its socket",
    )
    ctx.expect_contact(
        rear_left_foot,
        base_board,
        elem_a="lock_collar",
        elem_b="rear_left_socket",
        name="rear left leveling foot seats in its socket",
    )
    ctx.expect_contact(
        rear_right_foot,
        base_board,
        elem_a="lock_collar",
        elem_b="rear_right_socket",
        name="rear right leveling foot seats in its socket",
    )

    def _aabb_center(aabb):
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple((mins[index] + maxs[index]) * 0.5 for index in range(3))

    rest_objective = _aabb_center(ctx.part_element_world_aabb(tube_assembly, elem="objective_cell"))
    with ctx.pose({altitude_axis: math.radians(42.0)}):
        raised_objective = _aabb_center(ctx.part_element_world_aabb(tube_assembly, elem="objective_cell"))
    ctx.check(
        "positive altitude raises the refractor objective",
        rest_objective is not None
        and raised_objective is not None
        and raised_objective[2] > rest_objective[2] + 0.22,
        details=f"rest={rest_objective}, raised={raised_objective}",
    )

    rest_azimuth_objective = _aabb_center(ctx.part_element_world_aabb(tube_assembly, elem="objective_cell"))
    with ctx.pose({azimuth_rotation: math.pi / 2.0}):
        turned_objective = _aabb_center(ctx.part_element_world_aabb(tube_assembly, elem="objective_cell"))
    ctx.check(
        "azimuth joint swings the tube around the circular base",
        rest_azimuth_objective is not None
        and turned_objective is not None
        and abs(rest_azimuth_objective[0]) > 0.30
        and abs(turned_objective[1]) > 0.30
        and abs(turned_objective[0]) < 0.10,
        details=f"rest={rest_azimuth_objective}, turned={turned_objective}",
    )

    rest_front_foot = ctx.part_world_position(front_foot)
    with ctx.pose({front_foot_height: 0.025}):
        extended_front_foot = ctx.part_world_position(front_foot)
    ctx.check(
        "front leveling foot extends downward on its prismatic joint",
        rest_front_foot is not None
        and extended_front_foot is not None
        and extended_front_foot[2] < rest_front_foot[2] - 0.020,
        details=f"rest={rest_front_foot}, extended={extended_front_foot}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
