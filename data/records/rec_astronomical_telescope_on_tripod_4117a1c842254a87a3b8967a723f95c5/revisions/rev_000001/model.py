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
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_cylinder(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_member(part, a, b, *, radius: float, material, name: str) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _aabb_center(aabb):
    if aabb is None:
        return None
    return tuple((aabb[0][i] + aabb[1][i]) * 0.5 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dobsonian_truss_newtonian")

    birch_ply = model.material("birch_ply", rgba=(0.63, 0.49, 0.31, 1.0))
    ebony_laminate = model.material("ebony_laminate", rgba=(0.10, 0.10, 0.11, 1.0))
    matte_black = model.material("matte_black", rgba=(0.08, 0.08, 0.09, 1.0))
    powder_gray = model.material("powder_gray", rgba=(0.34, 0.36, 0.38, 1.0))
    steel = model.material("steel", rgba=(0.52, 0.55, 0.58, 1.0))
    aluminum = model.material("aluminum", rgba=(0.74, 0.76, 0.79, 1.0))
    mirror_glass = model.material("mirror_glass", rgba=(0.72, 0.80, 0.88, 1.0))
    white_detail = model.material("white_detail", rgba=(0.88, 0.88, 0.86, 1.0))

    ground_board_profile = [
        (0.42, 0.00),
        (0.22, 0.34),
        (-0.24, 0.31),
        (-0.41, 0.00),
        (-0.24, -0.31),
        (0.22, -0.34),
    ]
    side_panel_profile = [
        (-0.31, 0.00),
        (-0.31, 0.76),
        (-0.06, 0.76),
        (0.08, 0.70),
        (0.22, 0.58),
        (0.31, 0.56),
        (0.31, 0.00),
    ]

    ground_board_mesh = _mesh(
        "ground_board_shell",
        ExtrudeGeometry.from_z0(ground_board_profile, 0.032),
    )
    side_panel_mesh = _mesh(
        "rocker_side_panel",
        ExtrudeGeometry(side_panel_profile, 0.028, center=True),
    )
    cage_ring_mesh = _mesh(
        "secondary_cage_ring",
        TorusGeometry(radius=0.29, tube=0.014, radial_segments=18, tubular_segments=72),
    )

    ground_board = model.part("ground_board")
    ground_board.visual(
        ground_board_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=birch_ply,
        name="ground_board_panel",
    )
    for index, (x, y) in enumerate(((0.30, 0.00), (-0.18, 0.25), (-0.18, -0.25))):
        ground_board.visual(
            Cylinder(radius=0.030, length=0.025),
            origin=Origin(xyz=(x, y, 0.0125)),
            material=matte_black,
            name=f"ground_foot_{index}",
        )
    ground_board.visual(
        Cylinder(radius=0.095, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.067)),
        material=white_detail,
        name="azimuth_bearing_disk",
    )
    ground_board.visual(
        Cylinder(radius=0.012, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.050)),
        material=steel,
        name="azimuth_pivot_bolt",
    )
    for index, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        ground_board.visual(
            Box((0.050, 0.035, 0.004)),
            origin=Origin(
                xyz=(0.14 * math.cos(angle), 0.14 * math.sin(angle), 0.059),
                rpy=(0.0, 0.0, angle),
            ),
            material=white_detail,
            name=f"azimuth_pad_{index}",
        )
    ground_board.inertial = Inertial.from_geometry(
        Box((0.84, 0.72, 0.12)),
        mass=10.0,
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
    )

    rocker_box = model.part("rocker_box")
    rocker_box.visual(
        Box((0.66, 0.674, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=birch_ply,
        name="rocker_floor",
    )
    rocker_box.visual(
        side_panel_mesh,
        origin=Origin(xyz=(0.0, 0.351, 0.024), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=birch_ply,
        name="left_side_panel",
    )
    rocker_box.visual(
        side_panel_mesh,
        origin=Origin(xyz=(0.0, -0.351, 0.024), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=birch_ply,
        name="right_side_panel",
    )
    rocker_box.visual(
        Box((0.024, 0.618, 0.240)),
        origin=Origin(xyz=(0.318, 0.0, 0.144)),
        material=birch_ply,
        name="front_panel",
    )
    rocker_box.visual(
        Box((0.024, 0.618, 0.470)),
        origin=Origin(xyz=(-0.318, 0.0, 0.259)),
        material=birch_ply,
        name="rear_panel",
    )
    rocker_box.visual(
        Cylinder(radius=0.110, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=ebony_laminate,
        name="azimuth_bearing_ring",
    )
    rocker_box.visual(
        Box((0.070, 0.008, 0.060)),
        origin=Origin(xyz=(0.100, 0.334, 0.724)),
        material=white_detail,
        name="left_bearing_pad_upper",
    )
    rocker_box.visual(
        Box((0.070, 0.008, 0.060)),
        origin=Origin(xyz=(-0.120, 0.334, 0.498)),
        material=white_detail,
        name="left_bearing_pad_lower",
    )
    rocker_box.visual(
        Box((0.070, 0.008, 0.060)),
        origin=Origin(xyz=(0.100, -0.334, 0.724)),
        material=white_detail,
        name="right_bearing_pad_upper",
    )
    rocker_box.visual(
        Box((0.070, 0.008, 0.060)),
        origin=Origin(xyz=(-0.120, -0.334, 0.498)),
        material=white_detail,
        name="right_bearing_pad_lower",
    )
    rocker_box.inertial = Inertial.from_geometry(
        Box((0.70, 0.73, 0.80)),
        mass=16.0,
        origin=Origin(xyz=(0.0, 0.0, 0.400)),
    )

    telescope = model.part("telescope_assembly")
    telescope.visual(
        Box((0.620, 0.600, 0.024)),
        origin=Origin(xyz=(-0.020, 0.0, -0.298)),
        material=ebony_laminate,
        name="mirror_box_floor",
    )
    telescope.visual(
        Box((0.620, 0.024, 0.536)),
        origin=Origin(xyz=(-0.020, 0.288, -0.018)),
        material=birch_ply,
        name="mirror_box_left_wall",
    )
    telescope.visual(
        Box((0.620, 0.024, 0.536)),
        origin=Origin(xyz=(-0.020, -0.288, -0.018)),
        material=birch_ply,
        name="mirror_box_right_wall",
    )
    telescope.visual(
        Box((0.024, 0.552, 0.160)),
        origin=Origin(xyz=(0.286, 0.0, -0.206)),
        material=birch_ply,
        name="mirror_box_front_skirt",
    )
    telescope.visual(
        Box((0.024, 0.552, 0.160)),
        origin=Origin(xyz=(-0.326, 0.0, -0.206)),
        material=birch_ply,
        name="mirror_box_rear_skirt",
    )
    telescope.visual(
        Box((0.024, 0.552, 0.060)),
        origin=Origin(xyz=(0.286, 0.0, 0.220)),
        material=birch_ply,
        name="mirror_box_front_rail",
    )
    telescope.visual(
        Box((0.024, 0.552, 0.060)),
        origin=Origin(xyz=(-0.326, 0.0, 0.220)),
        material=birch_ply,
        name="mirror_box_rear_rail",
    )
    telescope.visual(
        Box((0.050, 0.490, 0.490)),
        origin=Origin(xyz=(-0.250, 0.0, -0.030)),
        material=powder_gray,
        name="mirror_cell_plate",
    )
    for index, (y, z) in enumerate(
        (
            (0.2605, 0.185),
            (0.2605, -0.185),
            (-0.2605, 0.185),
            (-0.2605, -0.185),
        )
    ):
        telescope.visual(
            Box((0.050, 0.031, 0.031)),
            origin=Origin(xyz=(-0.250, y, z)),
            material=powder_gray,
            name=f"cell_side_support_{index}",
        )
    telescope.visual(
        Cylinder(radius=0.245, length=0.045),
        origin=Origin(xyz=(-0.190, 0.0, -0.020), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=mirror_glass,
        name="primary_mirror",
    )
    for index, (y, z) in enumerate(((0.165, 0.155), (0.165, -0.155), (-0.190, 0.000))):
        telescope.visual(
            Box((0.016, 0.030, 0.030)),
            origin=Origin(xyz=(-0.217, y, z)),
            material=aluminum,
            name=f"mirror_clip_{index}",
        )
    telescope.visual(
        Cylinder(radius=0.065, length=0.040),
        origin=Origin(xyz=(-0.030, 0.315, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=ebony_laminate,
        name="left_trunnion_hub",
    )
    telescope.visual(
        Cylinder(radius=0.260, length=0.030),
        origin=Origin(xyz=(-0.030, 0.315, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=ebony_laminate,
        name="left_trunnion",
    )
    telescope.visual(
        Cylinder(radius=0.065, length=0.040),
        origin=Origin(xyz=(-0.030, -0.315, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=ebony_laminate,
        name="right_trunnion_hub",
    )
    telescope.visual(
        Cylinder(radius=0.260, length=0.030),
        origin=Origin(xyz=(-0.030, -0.315, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=ebony_laminate,
        name="right_trunnion",
    )

    lower_attach_points = [
        (0.190, 0.245, 0.200),
        (0.110, 0.245, 0.130),
        (0.110, 0.245, -0.130),
        (0.190, 0.245, -0.200),
        (0.190, -0.245, 0.200),
        (0.110, -0.245, 0.130),
        (0.110, -0.245, -0.130),
        (0.190, -0.245, -0.200),
    ]
    upper_attach_points = [
        (0.995, 0.255, 0.230),
        (0.995, 0.225, 0.130),
        (0.995, 0.225, -0.130),
        (0.995, 0.255, -0.230),
        (0.995, -0.255, 0.230),
        (0.995, -0.225, 0.130),
        (0.995, -0.225, -0.130),
        (0.995, -0.255, -0.230),
    ]
    for index, point in enumerate(lower_attach_points):
        telescope.visual(
            Box((0.048, 0.028, 0.028)),
            origin=Origin(xyz=point),
            material=powder_gray,
            name=f"lower_truss_block_{index}",
        )
    for index, point in enumerate(upper_attach_points):
        telescope.visual(
            Box((0.048, 0.028, 0.028)),
            origin=Origin(xyz=point),
            material=powder_gray,
            name=f"upper_truss_block_{index}",
        )
    for index, (lower, upper) in enumerate(zip(lower_attach_points, upper_attach_points)):
        _add_member(
            telescope,
            lower,
            upper,
            radius=0.011,
            material=steel,
            name=f"truss_pole_{index}",
        )

    telescope.visual(
        cage_ring_mesh,
        origin=Origin(xyz=(1.010, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=matte_black,
        name="secondary_ring_lower",
    )
    telescope.visual(
        cage_ring_mesh,
        origin=Origin(xyz=(1.190, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=matte_black,
        name="secondary_ring_upper",
    )
    for index, angle in enumerate(
        (math.pi / 4.0, 3.0 * math.pi / 4.0, 5.0 * math.pi / 4.0, 7.0 * math.pi / 4.0)
    ):
        lower = (1.010, 0.290 * math.cos(angle), 0.290 * math.sin(angle))
        upper = (1.190, 0.290 * math.cos(angle), 0.290 * math.sin(angle))
        _add_member(
            telescope,
            lower,
            upper,
            radius=0.009,
            material=aluminum,
            name=f"cage_strut_{index}",
        )
    telescope.visual(
        Box((0.006, 0.580, 0.014)),
        origin=Origin(xyz=(1.080, 0.0, 0.0)),
        material=aluminum,
        name="spider_vane_horizontal",
    )
    telescope.visual(
        Box((0.006, 0.014, 0.580)),
        origin=Origin(xyz=(1.080, 0.0, 0.0)),
        material=aluminum,
        name="spider_vane_vertical",
    )
    telescope.visual(
        Cylinder(radius=0.010, length=0.110),
        origin=Origin(xyz=(1.080, 0.055, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=powder_gray,
        name="secondary_stalk",
    )
    telescope.visual(
        Box((0.070, 0.050, 0.008)),
        origin=Origin(xyz=(1.080, 0.0, 0.0), rpy=(0.0, math.pi / 4.0, math.pi / 2.0)),
        material=mirror_glass,
        name="secondary_mirror",
    )
    telescope.visual(
        Box((0.210, 0.030, 0.085)),
        origin=Origin(xyz=(1.100, 0.285, 0.095)),
        material=birch_ply,
        name="focuser_board",
    )
    telescope.visual(
        Cylinder(radius=0.030, length=0.100),
        origin=Origin(xyz=(1.095, 0.315, 0.095), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=powder_gray,
        name="focuser_drawtube",
    )
    telescope.visual(
        Cylinder(radius=0.018, length=0.120),
        origin=Origin(xyz=(1.095, 0.405, 0.095), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=matte_black,
        name="eyepiece",
    )
    telescope.visual(
        Box((0.120, 0.010, 0.220)),
        origin=Origin(xyz=(1.080, -0.275, 0.020)),
        material=matte_black,
        name="light_baffle",
    )
    telescope.visual(
        Box((0.140, 0.030, 0.055)),
        origin=Origin(xyz=(1.145, -0.255, 0.215)),
        material=powder_gray,
        name="finder_shoe",
    )
    telescope.visual(
        Box((0.040, 0.030, 0.115)),
        origin=Origin(xyz=(1.125, -0.255, 0.155)),
        material=powder_gray,
        name="finder_riser",
    )
    telescope.visual(
        Cylinder(radius=0.022, length=0.180),
        origin=Origin(xyz=(1.155, -0.255, 0.215), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=white_detail,
        name="finder_scope_body",
    )
    telescope.visual(
        Cylinder(radius=0.026, length=0.035),
        origin=Origin(xyz=(1.240, -0.255, 0.215), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=matte_black,
        name="finder_scope_objective",
    )
    telescope.inertial = Inertial.from_geometry(
        Box((1.60, 0.72, 0.82)),
        mass=28.0,
        origin=Origin(xyz=(0.450, 0.0, 0.0)),
    )

    model.articulation(
        "azimuth_bearing",
        ArticulationType.CONTINUOUS,
        parent=ground_board,
        child=rocker_box,
        origin=Origin(xyz=(0.0, 0.0, 0.077)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=45.0, velocity=1.2),
    )
    model.articulation(
        "altitude_trunnion",
        ArticulationType.REVOLUTE,
        parent=rocker_box,
        child=telescope,
        origin=Origin(xyz=(0.050, 0.0, 0.600)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=0.8,
            lower=0.0,
            upper=1.45,
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

    ground_board = object_model.get_part("ground_board")
    rocker_box = object_model.get_part("rocker_box")
    telescope = object_model.get_part("telescope_assembly")
    azimuth = object_model.get_articulation("azimuth_bearing")
    altitude = object_model.get_articulation("altitude_trunnion")

    ctx.check(
        "azimuth bearing is continuous",
        azimuth.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={azimuth.articulation_type}",
    )
    ctx.check(
        "altitude trunnion is limited revolute",
        altitude.articulation_type == ArticulationType.REVOLUTE
        and altitude.motion_limits is not None
        and altitude.motion_limits.lower == 0.0
        and altitude.motion_limits.upper is not None
        and altitude.motion_limits.upper >= 1.40,
        details=f"limits={altitude.motion_limits}",
    )

    truss_poles = [
        visual.name
        for visual in telescope.visuals
        if visual.name is not None and visual.name.startswith("truss_pole_")
    ]
    ctx.check(
        "telescope has eight truss poles",
        len(truss_poles) == 8,
        details=f"truss_poles={truss_poles}",
    )

    with ctx.pose({altitude: 0.0}):
        ctx.expect_gap(
            telescope,
            rocker_box,
            axis="z",
            positive_elem="mirror_box_floor",
            negative_elem="rocker_floor",
            min_gap=0.200,
            max_gap=0.300,
            name="mirror box clears rocker floor",
        )
        ctx.expect_gap(
            rocker_box,
            telescope,
            axis="y",
            positive_elem="left_side_panel",
            negative_elem="left_trunnion",
            min_gap=0.001,
            max_gap=0.012,
            name="left trunnion clears rocker side",
        )
        ctx.expect_gap(
            telescope,
            rocker_box,
            axis="y",
            positive_elem="right_trunnion",
            negative_elem="right_side_panel",
            min_gap=0.001,
            max_gap=0.012,
            name="right trunnion clears rocker side",
        )
        ctx.expect_contact(
            telescope,
            rocker_box,
            elem_a="left_trunnion",
            elem_b="left_bearing_pad_upper",
            name="left trunnion rests on upper left bearing pad",
        )
        ctx.expect_contact(
            telescope,
            rocker_box,
            elem_a="left_trunnion",
            elem_b="left_bearing_pad_lower",
            name="left trunnion rests on lower left bearing pad",
        )
        ctx.expect_contact(
            telescope,
            rocker_box,
            elem_a="right_trunnion",
            elem_b="right_bearing_pad_upper",
            name="right trunnion rests on upper right bearing pad",
        )
        ctx.expect_contact(
            telescope,
            rocker_box,
            elem_a="right_trunnion",
            elem_b="right_bearing_pad_lower",
            name="right trunnion rests on lower right bearing pad",
        )

    rest_ring = _aabb_center(ctx.part_element_world_aabb(telescope, elem="secondary_ring_upper"))
    with ctx.pose({altitude: 1.10}):
        raised_ring = _aabb_center(
            ctx.part_element_world_aabb(telescope, elem="secondary_ring_upper")
        )
    ctx.check(
        "altitude motion raises the secondary cage",
        rest_ring is not None
        and raised_ring is not None
        and raised_ring[2] > rest_ring[2] + 0.65
        and raised_ring[0] < rest_ring[0] - 0.35,
        details=f"rest_ring={rest_ring}, raised_ring={raised_ring}",
    )

    with ctx.pose({azimuth: math.pi / 2.0}):
        swiveled_ring = _aabb_center(
            ctx.part_element_world_aabb(telescope, elem="secondary_ring_upper")
        )
    ctx.check(
        "azimuth motion swings the optical tube around the ground board",
        rest_ring is not None
        and swiveled_ring is not None
        and abs(swiveled_ring[0]) < 0.18
        and swiveled_ring[1] > rest_ring[0] - 0.18,
        details=f"rest_ring={rest_ring}, swiveled_ring={swiveled_ring}",
    )

    ctx.expect_origin_distance(
        ground_board,
        rocker_box,
        axes="xy",
        max_dist=0.001,
        name="rocker stays centered over the ground board",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
