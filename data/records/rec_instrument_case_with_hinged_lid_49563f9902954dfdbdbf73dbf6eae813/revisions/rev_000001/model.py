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
    CylinderGeometry,
    Inertial,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    repair_loft,
    section_loft,
    tube_from_spline_points,
)


HINGE_SIDE_Y = -0.095
WALL_THICKNESS = 0.008
LOWER_FLOOR_THICKNESS = 0.0045
LOWER_RIM_Z = 0.070

X_STATIONS = (-0.30, -0.21, -0.08, 0.08, 0.22, 0.30)
LATCH_SIDE_Y = (0.072, 0.092, 0.128, 0.150, 0.128, 0.095)
LOWER_DEPTHS = (0.050, 0.056, 0.064, 0.067, 0.062, 0.054)
LID_DOMES = (0.040, 0.052, 0.072, 0.078, 0.060, 0.046)


def _merge_geometries(geometries: list[MeshGeometry]) -> MeshGeometry:
    merged = MeshGeometry()
    for geometry in geometries:
        merged.merge(geometry)
    return merged


def _lower_shell_section(x_pos: float, latch_y: float, rim_z: float) -> list[tuple[float, float, float]]:
    y_left = HINGE_SIDE_Y
    y_right = latch_y
    y_inner_left = y_left + WALL_THICKNESS
    y_inner_right = y_right - WALL_THICKNESS
    span = y_right - y_left
    bottom_inset = min(0.028, span * 0.18)
    side_inset = min(0.016, span * 0.12)
    inner_z = LOWER_FLOOR_THICKNESS

    outer_mid_1 = y_left + bottom_inset + 0.32 * (span - 2.0 * bottom_inset)
    outer_mid_2 = y_left + bottom_inset + 0.74 * (span - 2.0 * bottom_inset)
    inner_span = y_inner_right - y_inner_left
    inner_mid_1 = y_inner_left + 0.30 * inner_span
    inner_mid_2 = y_inner_left + 0.72 * inner_span

    return [
        (x_pos, y_left, rim_z),
        (x_pos, y_left + 0.004, rim_z * 0.76),
        (x_pos, y_left + side_inset, 0.014),
        (x_pos, y_left + bottom_inset, 0.0),
        (x_pos, outer_mid_1, 0.0),
        (x_pos, outer_mid_2, 0.0),
        (x_pos, y_right - bottom_inset, 0.0),
        (x_pos, y_right - side_inset, 0.014),
        (x_pos, y_right - 0.004, rim_z * 0.76),
        (x_pos, y_right, rim_z),
        (x_pos, y_inner_right, rim_z),
        (x_pos, y_inner_right - 0.003, inner_z + 0.012),
        (x_pos, inner_mid_2, inner_z),
        (x_pos, inner_mid_1, inner_z),
        (x_pos, y_inner_left + 0.003, inner_z + 0.012),
        (x_pos, y_inner_left, rim_z),
    ]


def _lid_section(x_pos: float, latch_y: float, dome_height: float) -> list[tuple[float, float, float]]:
    total_width = (latch_y - HINGE_SIDE_Y) + 0.012
    inner_crown = max(0.020, dome_height - 0.020)

    return [
        (x_pos, 0.000, 0.000),
        (x_pos, 0.010, dome_height * 0.30),
        (x_pos, total_width * 0.24, dome_height * 0.78),
        (x_pos, total_width * 0.52, dome_height),
        (x_pos, total_width * 0.82, dome_height * 0.70),
        (x_pos, total_width - 0.020, dome_height * 0.28),
        (x_pos, total_width - 0.010, 0.010),
        (x_pos, total_width, 0.000),
        (x_pos, total_width - (WALL_THICKNESS + 0.002), 0.000),
        (x_pos, total_width - (WALL_THICKNESS + 0.014), inner_crown * 0.40),
        (x_pos, total_width * 0.72, inner_crown * 0.78),
        (x_pos, total_width * 0.48, inner_crown),
        (x_pos, total_width * 0.22, inner_crown * 0.72),
        (x_pos, 0.026, inner_crown * 0.22),
        (x_pos, 0.016, 0.006),
        (x_pos, WALL_THICKNESS + 0.002, 0.000),
    ]


def _build_lower_shell_mesh() -> MeshGeometry:
    sections = [
        _lower_shell_section(x_pos, latch_y, rim_z)
        for x_pos, latch_y, rim_z in zip(X_STATIONS, LATCH_SIDE_Y, LOWER_DEPTHS)
    ]
    return repair_loft(section_loft(sections))


def _build_lid_shell_mesh() -> MeshGeometry:
    sections = [
        _lid_section(x_pos, latch_y, dome_height)
        for x_pos, latch_y, dome_height in zip(X_STATIONS, LATCH_SIDE_Y, LID_DOMES)
    ]
    return repair_loft(section_loft(sections))


def _build_handle_mesh() -> MeshGeometry:
    pivot_radius = 0.0055
    arm_radius = 0.0042
    grip_radius = 0.007

    left_pivot = (
        CylinderGeometry(radius=pivot_radius, height=0.018, radial_segments=20)
        .rotate_y(math.pi / 2.0)
        .translate(0.009, 0.0, 0.0)
    )
    right_pivot = (
        CylinderGeometry(radius=pivot_radius, height=0.018, radial_segments=20)
        .rotate_y(math.pi / 2.0)
        .translate(0.111, 0.0, 0.0)
    )

    left_arm = tube_from_spline_points(
        [
            (0.009, 0.0, 0.0),
            (0.009, 0.012, -0.005),
            (0.012, 0.025, -0.010),
            (0.017, 0.034, -0.012),
        ],
        radius=arm_radius,
        samples_per_segment=16,
        radial_segments=16,
        cap_ends=True,
    )
    right_arm = tube_from_spline_points(
        [
            (0.111, 0.0, 0.0),
            (0.111, 0.012, -0.005),
            (0.108, 0.025, -0.010),
            (0.103, 0.034, -0.012),
        ],
        radius=arm_radius,
        samples_per_segment=16,
        radial_segments=16,
        cap_ends=True,
    )
    grip = (
        CylinderGeometry(radius=grip_radius, height=0.086, radial_segments=24)
        .rotate_y(math.pi / 2.0)
        .translate(0.060, 0.034, -0.012)
    )

    return _merge_geometries([left_pivot, right_pivot, left_arm, right_arm, grip])


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="brass_instrument_case")

    shell_black = model.material("shell_black", rgba=(0.14, 0.14, 0.15, 1.0))
    lining_dark = model.material("lining_dark", rgba=(0.18, 0.17, 0.15, 1.0))
    hardware = model.material("hardware", rgba=(0.67, 0.68, 0.70, 1.0))
    handle_black = model.material("handle_black", rgba=(0.07, 0.07, 0.07, 1.0))
    rubber = model.material("rubber", rgba=(0.05, 0.05, 0.05, 1.0))

    lower_shell = model.part("lower_shell")
    lower_shell.visual(
        mesh_from_geometry(_build_lower_shell_mesh(), "lower_shell_body"),
        material=shell_black,
        name="lower_shell_body",
    )
    lower_shell.visual(
        Box((0.082, 0.003, 0.022)),
        origin=Origin(xyz=(-0.22, HINGE_SIDE_Y - 0.0015, 0.056)),
        material=hardware,
        name="hinge_plate_0",
    )
    lower_shell.visual(
        Box((0.094, 0.003, 0.022)),
        origin=Origin(xyz=(0.00, HINGE_SIDE_Y - 0.0015, 0.056)),
        material=hardware,
        name="hinge_plate_1",
    )
    lower_shell.visual(
        Box((0.082, 0.003, 0.022)),
        origin=Origin(xyz=(0.22, HINGE_SIDE_Y - 0.0015, 0.056)),
        material=hardware,
        name="hinge_plate_2",
    )
    lower_shell.visual(
        Box((0.320, 0.024, 0.034)),
        origin=Origin(xyz=(0.0, 0.139, 0.045)),
        material=hardware,
        name="side_hardware_band",
    )
    lower_shell.visual(
        Box((0.240, 0.024, 0.010)),
        origin=Origin(xyz=(-0.13, 0.000, -0.001)),
        material=rubber,
        name="bottom_runner_left",
    )
    lower_shell.visual(
        Box((0.240, 0.024, 0.010)),
        origin=Origin(xyz=(0.13, 0.032, -0.001)),
        material=rubber,
        name="bottom_runner_right",
    )
    lower_shell.inertial = Inertial.from_geometry(
        Box((0.64, 0.27, 0.09)),
        mass=3.4,
        origin=Origin(xyz=(0.0, 0.03, 0.032)),
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_geometry(_build_lid_shell_mesh(), "lid_shell_body"),
        material=shell_black,
        name="lid_shell_body",
    )
    lid.visual(
        Box((0.074, 0.003, 0.018)),
        origin=Origin(xyz=(-0.11, -0.0015, -0.002)),
        material=hardware,
        name="hinge_leaf_0",
    )
    lid.visual(
        Box((0.074, 0.003, 0.018)),
        origin=Origin(xyz=(0.11, -0.0015, -0.002)),
        material=hardware,
        name="hinge_leaf_1",
    )
    lid.visual(
        Box((0.300, 0.014, 0.014)),
        origin=Origin(xyz=(0.0, 0.228, 0.004)),
        material=hardware,
        name="lid_edge_trim",
    )
    lid.inertial = Inertial.from_geometry(
        Box((0.64, 0.27, 0.10)),
        mass=2.4,
        origin=Origin(xyz=(0.0, 0.125, 0.038)),
    )

    handle = model.part("handle")
    handle.visual(
        mesh_from_geometry(_build_handle_mesh(), "carry_handle"),
        material=handle_black,
        name="handle_assembly",
    )
    handle.visual(
        Cylinder(radius=0.0045, length=0.016),
        origin=Origin(xyz=(0.009, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hardware,
        name="left_pivot_core",
    )
    handle.visual(
        Cylinder(radius=0.0045, length=0.016),
        origin=Origin(xyz=(0.111, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hardware,
        name="right_pivot_core",
    )
    handle.inertial = Inertial.from_geometry(
        Box((0.13, 0.05, 0.03)),
        mass=0.18,
        origin=Origin(xyz=(0.060, 0.018, -0.008)),
    )

    lid_hinge = model.articulation(
        "lower_shell_to_lid",
        ArticulationType.REVOLUTE,
        parent=lower_shell,
        child=lid,
        origin=Origin(xyz=(0.0, HINGE_SIDE_Y, LOWER_RIM_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.2,
            lower=0.0,
            upper=math.radians(92.0),
        ),
    )

    model.articulation(
        "lower_shell_to_handle",
        ArticulationType.REVOLUTE,
        parent=lower_shell,
        child=handle,
        origin=Origin(xyz=(-0.055, 0.1565, 0.042)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.5,
            lower=0.0,
            upper=math.radians(75.0),
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

    lower_shell = object_model.get_part("lower_shell")
    lid = object_model.get_part("lid")
    handle = object_model.get_part("handle")
    lid_hinge = object_model.get_articulation("lower_shell_to_lid")
    handle_hinge = object_model.get_articulation("lower_shell_to_handle")

    def _aabb_center(aabb):
        if aabb is None:
            return None
        (min_corner, max_corner) = aabb
        return tuple((min_corner[i] + max_corner[i]) * 0.5 for i in range(3))

    ctx.check(
        "lid hinge axis runs along the case length",
        lid_hinge.axis == (1.0, 0.0, 0.0),
        details=f"axis={lid_hinge.axis}",
    )
    ctx.check(
        "handle pivots run along the case length",
        handle_hinge.axis == (1.0, 0.0, 0.0),
        details=f"axis={handle_hinge.axis}",
    )

    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_gap(
            lid,
            lower_shell,
            axis="z",
            max_gap=0.004,
            max_penetration=0.0,
            positive_elem="lid_shell_body",
            negative_elem="lower_shell_body",
            name="closed lid seats on the lower shell rim",
        )
        ctx.expect_overlap(
            lid,
            lower_shell,
            axes="xy",
            min_overlap=0.10,
            elem_a="lid_shell_body",
            elem_b="lower_shell_body",
            name="closed lid covers the case footprint",
        )

    lid_rest = ctx.part_element_world_aabb(lid, elem="lid_shell_body")
    with ctx.pose({lid_hinge: math.radians(70.0)}):
        lid_open = ctx.part_element_world_aabb(lid, elem="lid_shell_body")
    lid_rest_center = _aabb_center(lid_rest)
    lid_open_center = _aabb_center(lid_open)
    ctx.check(
        "lid opens upward from the side hinge",
        lid_rest_center is not None
        and lid_open_center is not None
        and lid_open_center[2] > lid_rest_center[2] + 0.08,
        details=f"rest_center={lid_rest_center}, open_center={lid_open_center}",
    )

    handle_rest = ctx.part_element_world_aabb(handle, elem="handle_assembly")
    with ctx.pose({handle_hinge: math.radians(55.0)}):
        handle_raised = ctx.part_element_world_aabb(handle, elem="handle_assembly")
    handle_rest_center = _aabb_center(handle_rest)
    handle_raised_center = _aabb_center(handle_raised)
    ctx.check(
        "carry handle lifts from the shell wall",
        handle_rest_center is not None
        and handle_raised_center is not None
        and handle_raised_center[2] > handle_rest_center[2] + 0.010,
        details=f"rest_center={handle_rest_center}, raised_center={handle_raised_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
