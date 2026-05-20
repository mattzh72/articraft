from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    mesh_from_geometry,
    rounded_rect_profile,
)


CASE_LENGTH = 0.45
CASE_WIDTH = 0.135
LOWER_HEIGHT = 0.055
LID_TOP_THICKNESS = 0.004
LID_SKIRT_DEPTH = 0.022
LOWER_WALL = 0.008
LID_SKIRT_WALL = 0.006
CORNER_RADIUS = 0.020
HINGE_RADIUS = 0.0045
HINGE_AXIS_Y = -(CASE_WIDTH / 2.0) + 0.006
HINGE_AXIS_Z = LOWER_HEIGHT + (LID_TOP_THICKNESS / 2.0)
LATCH_PIVOT_RADIUS = 0.003
LATCH_PIVOT_Y = (CASE_WIDTH / 2.0) + 0.0055
LATCH_PIVOT_Z = 0.024
LEFT_LATCH_X = -0.150
RIGHT_LATCH_X = 0.150


def _merge_geometries(*geometries: MeshGeometry) -> MeshGeometry:
    merged = MeshGeometry()
    for geometry in geometries:
        merged.merge(geometry)
    return merged


def _x_cylinder(radius: float, length: float, center: tuple[float, float, float]) -> MeshGeometry:
    geom = CylinderGeometry(radius=radius, height=length, radial_segments=28, closed=True)
    geom.rotate_y(pi / 2.0)
    geom.translate(*center)
    return geom


def _z_cylinder(radius: float, length: float, center: tuple[float, float, float]) -> MeshGeometry:
    geom = CylinderGeometry(radius=radius, height=length, radial_segments=24, closed=True)
    geom.translate(*center)
    return geom


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> MeshGeometry:
    geom = BoxGeometry(size)
    geom.translate(*center)
    return geom


def _rounded_shell_ring(
    *,
    outer_size: tuple[float, float],
    inner_size: tuple[float, float],
    outer_radius: float,
    inner_radius: float,
    height: float,
    center: tuple[float, float, float],
) -> MeshGeometry:
    ring = ExtrudeWithHolesGeometry(
        rounded_rect_profile(outer_size[0], outer_size[1], outer_radius, corner_segments=10),
        [rounded_rect_profile(inner_size[0], inner_size[1], inner_radius, corner_segments=10)],
        height,
        cap=True,
        center=True,
        closed=True,
    )
    ring.translate(*center)
    return ring


def _rounded_plate(
    *,
    size: tuple[float, float],
    radius: float,
    thickness: float,
    center: tuple[float, float, float],
) -> MeshGeometry:
    plate = ExtrudeGeometry.from_z0(
        rounded_rect_profile(size[0], size[1], radius, corner_segments=10),
        thickness,
        cap=True,
        closed=True,
    )
    plate.translate(center[0], center[1], center[2] - (thickness / 2.0))
    return plate


def _hinge_knuckle_centers() -> list[tuple[float, float]]:
    hinge_margin = 0.018
    segment_gap = 0.002
    segment_count = 9
    span = CASE_LENGTH - (2.0 * hinge_margin)
    segment_length = (span - ((segment_count - 1) * segment_gap)) / segment_count
    x_start = (-span / 2.0) + (segment_length / 2.0)
    return [
        (x_start + (index * (segment_length + segment_gap)), segment_length)
        for index in range(segment_count)
    ]


def _build_lower_shell_mesh() -> MeshGeometry:
    lower_inner_x = CASE_LENGTH - (2.0 * LOWER_WALL)
    lower_inner_y = CASE_WIDTH - (2.0 * LOWER_WALL)

    floor = _rounded_plate(
        size=(CASE_LENGTH, CASE_WIDTH),
        radius=CORNER_RADIUS,
        thickness=0.004,
        center=(0.0, 0.0, 0.002),
    )
    wall_ring = _rounded_shell_ring(
        outer_size=(CASE_LENGTH, CASE_WIDTH),
        inner_size=(lower_inner_x, lower_inner_y),
        outer_radius=CORNER_RADIUS,
        inner_radius=CORNER_RADIUS - (LOWER_WALL * 0.65),
        height=LOWER_HEIGHT,
        center=(0.0, 0.0, LOWER_HEIGHT / 2.0),
    )

    hinge_knuckles = []
    for index, (x_center, segment_length) in enumerate(_hinge_knuckle_centers()):
        if index % 2 == 0:
            hinge_knuckles.append(
                _x_cylinder(
                    HINGE_RADIUS,
                    segment_length,
                    (x_center, HINGE_AXIS_Y, HINGE_AXIS_Z),
                )
            )

    latch_supports = []
    for latch_x in (LEFT_LATCH_X, RIGHT_LATCH_X):
        latch_supports.extend(
            [
                _box((0.005, 0.008, 0.010), (latch_x - 0.0085, (CASE_WIDTH / 2.0) + 0.0015, LATCH_PIVOT_Z)),
                _box((0.005, 0.008, 0.010), (latch_x + 0.0085, (CASE_WIDTH / 2.0) + 0.0015, LATCH_PIVOT_Z)),
                _x_cylinder(
                    LATCH_PIVOT_RADIUS,
                    0.005,
                    (latch_x - 0.0085, LATCH_PIVOT_Y, LATCH_PIVOT_Z),
                ),
                _x_cylinder(
                    LATCH_PIVOT_RADIUS,
                    0.005,
                    (latch_x + 0.0085, LATCH_PIVOT_Y, LATCH_PIVOT_Z),
                ),
            ]
        )

    foot_centers = [
        (-0.170, -0.032),
        (0.170, -0.032),
        (-0.170, 0.032),
        (0.170, 0.032),
    ]
    feet = [
        _z_cylinder(0.008, 0.003, (x_pos, y_pos, -0.0015))
        for x_pos, y_pos in foot_centers
    ]

    return _merge_geometries(floor, wall_ring, *hinge_knuckles, *latch_supports, *feet)


def _build_lid_mesh() -> MeshGeometry:
    lower_inner_x = CASE_LENGTH - (2.0 * LOWER_WALL)
    lower_inner_y = CASE_WIDTH - (2.0 * LOWER_WALL)
    skirt_outer_x = lower_inner_x - 0.004
    skirt_outer_y = lower_inner_y - 0.004
    skirt_inner_x = skirt_outer_x - (2.0 * LID_SKIRT_WALL)
    skirt_inner_y = skirt_outer_y - (2.0 * LID_SKIRT_WALL)
    lid_center_y = (CASE_WIDTH / 2.0) - 0.006

    top_panel = _rounded_plate(
        size=(CASE_LENGTH, CASE_WIDTH),
        radius=CORNER_RADIUS,
        thickness=LID_TOP_THICKNESS,
        center=(0.0, lid_center_y, 0.0),
    )
    skirt_ring = _rounded_shell_ring(
        outer_size=(skirt_outer_x, skirt_outer_y),
        inner_size=(skirt_inner_x, skirt_inner_y),
        outer_radius=CORNER_RADIUS - 0.006,
        inner_radius=CORNER_RADIUS - 0.011,
        height=LID_SKIRT_DEPTH,
        center=(0.0, lid_center_y, -(LID_SKIRT_DEPTH / 2.0) - (LID_TOP_THICKNESS / 2.0)),
    )
    hinge_knuckles = []
    for index, (x_center, segment_length) in enumerate(_hinge_knuckle_centers()):
        if index % 2 == 1:
            hinge_knuckles.append(
                _x_cylinder(
                    HINGE_RADIUS,
                    segment_length,
                    (x_center, 0.0, 0.0),
                )
            )

    return _merge_geometries(top_panel, skirt_ring, *hinge_knuckles)


def _build_latch_mesh() -> MeshGeometry:
    pivot = _x_cylinder(0.0025, 0.011, (0.0, 0.0, 0.0))
    body = _box((0.026, 0.005, 0.026), (0.0, -0.002, 0.013))
    top_tab = _box((0.024, 0.006, 0.004), (0.0, -0.0015, 0.026))
    finger_tab = _box((0.018, 0.008, 0.005), (0.0, 0.0005, 0.0055))
    return _merge_geometries(pivot, body, top_tab, finger_tab)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="clarinet_case")

    shell_material = model.material("shell_vinyl", rgba=(0.12, 0.12, 0.14, 1.0))
    trim_material = model.material("trim_black", rgba=(0.08, 0.08, 0.09, 1.0))
    metal_material = model.material("hardware", rgba=(0.72, 0.74, 0.78, 1.0))

    lower_shell = model.part("lower_shell")
    lower_shell.visual(
        mesh_from_geometry(_build_lower_shell_mesh(), "lower_shell_body"),
        material=shell_material,
        name="elem_lower_shell",
    )
    lower_shell.visual(
        Box((CASE_LENGTH - 0.050, CASE_WIDTH - 0.045, 0.003)),
        origin=Origin(xyz=(0.0, 0.0, 0.0045)),
        material=trim_material,
        name="elem_inner_pad",
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_geometry(_build_lid_mesh(), "lid_body"),
        material=shell_material,
        name="elem_lid_shell",
    )
    lid.visual(
        Box((CASE_LENGTH - 0.040, CASE_WIDTH - 0.040, 0.002)),
        origin=Origin(xyz=(0.0, (CASE_WIDTH / 2.0) - 0.006, -0.003)),
        material=trim_material,
        name="elem_lid_lining",
    )
    lid.visual(
        Box((CASE_LENGTH - 0.060, 0.004, 0.010)),
        origin=Origin(
            xyz=(0.0, (CASE_WIDTH / 2.0) - 0.006 + (CASE_WIDTH / 2.0) - 0.002, -0.007)
        ),
        material=metal_material,
        name="elem_front_fascia",
    )

    left_latch = model.part("left_latch")
    left_latch.visual(
        mesh_from_geometry(_build_latch_mesh(), "left_latch_mesh"),
        material=metal_material,
        name="elem_latch_body",
    )

    right_latch = model.part("right_latch")
    right_latch.visual(
        mesh_from_geometry(_build_latch_mesh(), "right_latch_mesh"),
        material=metal_material,
        name="elem_latch_body",
    )

    model.articulation(
        "lower_shell_to_lid",
        ArticulationType.REVOLUTE,
        parent=lower_shell,
        child=lid,
        origin=Origin(xyz=(0.0, HINGE_AXIS_Y, HINGE_AXIS_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=2.5, lower=0.0, upper=2.35),
    )
    model.articulation(
        "lower_shell_to_left_latch",
        ArticulationType.REVOLUTE,
        parent=lower_shell,
        child=left_latch,
        origin=Origin(xyz=(LEFT_LATCH_X, LATCH_PIVOT_Y, LATCH_PIVOT_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=4.0, lower=0.0, upper=1.20),
    )
    model.articulation(
        "lower_shell_to_right_latch",
        ArticulationType.REVOLUTE,
        parent=lower_shell,
        child=right_latch,
        origin=Origin(xyz=(RIGHT_LATCH_X, LATCH_PIVOT_Y, LATCH_PIVOT_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=4.0, lower=0.0, upper=1.20),
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
    left_latch = object_model.get_part("left_latch")
    right_latch = object_model.get_part("right_latch")
    lid_hinge = object_model.get_articulation("lower_shell_to_lid")
    left_latch_joint = object_model.get_articulation("lower_shell_to_left_latch")
    right_latch_joint = object_model.get_articulation("lower_shell_to_right_latch")

    ctx.check(
        "lid hinge uses rear x-axis",
        lid_hinge.axis == (1.0, 0.0, 0.0),
        details=f"axis={lid_hinge.axis}",
    )
    ctx.check(
        "latches pivot on negative x-axis",
        left_latch_joint.axis == (-1.0, 0.0, 0.0) and right_latch_joint.axis == (-1.0, 0.0, 0.0),
        details=f"left={left_latch_joint.axis}, right={right_latch_joint.axis}",
    )

    with ctx.pose({lid_hinge: 0.0, left_latch_joint: 0.0, right_latch_joint: 0.0}):
        ctx.expect_overlap(
            lid,
            lower_shell,
            axes="x",
            min_overlap=0.38,
            elem_a="elem_lid_shell",
            elem_b="elem_lower_shell",
            name="lid spans the clarinet case length",
        )
        ctx.expect_overlap(
            lid,
            lower_shell,
            axes="y",
            min_overlap=0.10,
            elem_a="elem_lid_shell",
            elem_b="elem_lower_shell",
            name="lid covers the case opening width",
        )
        ctx.expect_origin_gap(
            right_latch,
            left_latch,
            axis="x",
            min_gap=0.20,
            name="side latches are spaced across the front edge",
        )

        left_gap = ctx.part_element_world_aabb(left_latch, elem="elem_latch_body")
        right_gap = ctx.part_element_world_aabb(right_latch, elem="elem_latch_body")
        ctx.check(
            "latches sit at the front edge",
            left_gap is not None
            and right_gap is not None
            and left_gap[0][1] > (CASE_WIDTH / 2.0) - 0.001
            and right_gap[0][1] > (CASE_WIDTH / 2.0) - 0.001,
            details=f"left={left_gap}, right={right_gap}",
        )

    closed_lid_aabb = ctx.part_element_world_aabb(lid, elem="elem_front_fascia")
    with ctx.pose({lid_hinge: 1.20}):
        open_lid_aabb = ctx.part_element_world_aabb(lid, elem="elem_front_fascia")
    ctx.check(
        "lid opens upward from the rear hinge",
        closed_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.06
        and open_lid_aabb[1][1] < closed_lid_aabb[1][1] - 0.03,
        details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
    )

    closed_left_latch = ctx.part_element_world_aabb(left_latch, elem="elem_latch_body")
    closed_right_latch = ctx.part_element_world_aabb(right_latch, elem="elem_latch_body")
    with ctx.pose({left_latch_joint: 1.0, right_latch_joint: 1.0}):
        open_left_latch = ctx.part_element_world_aabb(left_latch, elem="elem_latch_body")
        open_right_latch = ctx.part_element_world_aabb(right_latch, elem="elem_latch_body")
    ctx.check(
        "left latch rotates outward",
        closed_left_latch is not None
        and open_left_latch is not None
        and open_left_latch[1][1] > closed_left_latch[1][1] + 0.01,
        details=f"closed={closed_left_latch}, open={open_left_latch}",
    )
    ctx.check(
        "right latch rotates outward",
        closed_right_latch is not None
        and open_right_latch is not None
        and open_right_latch[1][1] > closed_right_latch[1][1] + 0.01,
        details=f"closed={closed_right_latch}, open={open_right_latch}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
