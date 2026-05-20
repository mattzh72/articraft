from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    Inertial,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    sample_catmull_rom_spline_2d,
)


CASE_LENGTH = 0.85
CASE_WALL = 0.006
HINGE_AXIS_Y = -0.129
HINGE_AXIS_Z = 0.006
LATCH_X = 0.115


def _add_quad(geometry: MeshGeometry, a: int, b: int, c: int, d: int) -> None:
    geometry.add_face(a, b, c)
    geometry.add_face(a, c, d)


def _stitch_loops(
    geometry: MeshGeometry,
    loop_a: list[int],
    loop_b: list[int],
) -> None:
    count = len(loop_a)
    for index in range(count):
        next_index = (index + 1) % count
        _add_quad(
            geometry,
            loop_a[index],
            loop_a[next_index],
            loop_b[next_index],
            loop_b[index],
        )


def _scaled_loop(
    loop: list[tuple[float, float]],
    *,
    sx: float,
    sy: float,
    center: tuple[float, float] = (0.0, 0.0),
    dx: float = 0.0,
    dy: float = 0.0,
) -> list[tuple[float, float]]:
    cx, cy = center
    return [
        (cx + ((x - cx) * sx) + dx, cy + ((y - cy) * sy) + dy)
        for x, y in loop
    ]


def _case_outline() -> list[tuple[float, float]]:
    control_points = [
        (-0.420, -0.020),
        (-0.405, -0.055),
        (-0.370, -0.080),
        (-0.270, -0.100),
        (-0.140, -0.112),
        (0.010, -0.122),
        (0.160, -0.124),
        (0.285, -0.115),
        (0.375, -0.090),
        (0.425, -0.020),
        (0.420, 0.055),
        (0.350, 0.115),
        (0.220, 0.145),
        (0.060, 0.136),
        (-0.080, 0.108),
        (-0.210, 0.088),
        (-0.325, 0.073),
        (-0.405, 0.040),
    ]
    outline = sample_catmull_rom_spline_2d(
        control_points,
        samples_per_segment=5,
        closed=True,
    )
    if outline and outline[0] == outline[-1]:
        outline = outline[:-1]
    return outline


def _bell_weight(x_pos: float) -> float:
    normalized = (x_pos + (CASE_LENGTH * 0.5)) / CASE_LENGTH
    normalized = max(0.0, min(1.0, normalized))
    return normalized**1.25


def _lower_depth(x_pos: float, y_pos: float) -> float:
    front_bias = max(0.0, y_pos) / 0.15
    return 0.082 + (0.044 * _bell_weight(x_pos)) + (0.007 * front_bias * front_bias)


def _lid_depth(x_pos: float, y_pos: float) -> float:
    front_bias = max(0.0, y_pos) / 0.15
    return 0.036 + (0.014 * _bell_weight(x_pos)) + (0.0035 * front_bias)


def _build_shell_mesh(
    outer_rim_xy: list[tuple[float, float]],
    outer_mid_xy: list[tuple[float, float]],
    inner_mid_xy: list[tuple[float, float]],
    inner_rim_xy: list[tuple[float, float]],
    *,
    outer_mid_z_fn,
    inner_mid_z_fn,
    outer_rim_z: float,
    inner_rim_z: float,
) -> MeshGeometry:
    geometry = MeshGeometry()
    outer_rim_ids = [
        geometry.add_vertex(x_pos, y_pos, outer_rim_z) for x_pos, y_pos in outer_rim_xy
    ]
    outer_mid_ids = [
        geometry.add_vertex(x_pos, y_pos, outer_mid_z_fn(x_pos, y_pos))
        for x_pos, y_pos in outer_mid_xy
    ]
    inner_mid_ids = [
        geometry.add_vertex(x_pos, y_pos, inner_mid_z_fn(x_pos, y_pos))
        for x_pos, y_pos in inner_mid_xy
    ]
    inner_rim_ids = [
        geometry.add_vertex(x_pos, y_pos, inner_rim_z) for x_pos, y_pos in inner_rim_xy
    ]

    _stitch_loops(geometry, outer_rim_ids, outer_mid_ids)
    _stitch_loops(geometry, outer_mid_ids, inner_mid_ids)
    _stitch_loops(geometry, inner_mid_ids, inner_rim_ids)
    _stitch_loops(geometry, inner_rim_ids, outer_rim_ids)
    return geometry


def _hinge_segments() -> list[tuple[float, float, str]]:
    return [
        (-0.355, -0.215, "lower"),
        (-0.205, -0.065, "lid"),
        (-0.055, 0.105, "lower"),
        (0.115, 0.265, "lid"),
        (0.275, 0.375, "lower"),
    ]


def _build_lower_shell_mesh() -> MeshGeometry:
    outline = _case_outline()
    outer_bottom_xy = _scaled_loop(
        outline,
        sx=0.952,
        sy=0.885,
        center=(0.030, 0.010),
        dy=-0.004,
    )
    inner_bottom_xy = _scaled_loop(
        outline,
        sx=0.902,
        sy=0.780,
        center=(0.030, 0.008),
        dy=-0.006,
    )
    inner_rim_xy = _scaled_loop(
        outline,
        sx=0.968,
        sy=0.932,
        center=(0.030, 0.006),
    )

    shell = _build_shell_mesh(
        outline,
        outer_bottom_xy,
        inner_bottom_xy,
        inner_rim_xy,
        outer_mid_z_fn=lambda x_pos, y_pos: -_lower_depth(x_pos, y_pos),
        inner_mid_z_fn=lambda x_pos, y_pos: -_lower_depth(x_pos, y_pos) + CASE_WALL,
        outer_rim_z=0.0,
        inner_rim_z=-CASE_WALL,
    )

    shell.merge(
        BoxGeometry((0.200, 0.030, 0.010)).translate(
            0.295,
            0.084,
            -0.030,
        )
    )
    shell.merge(
        BoxGeometry((0.210, 0.028, 0.010)).translate(
            -0.215,
            0.060,
            -0.024,
        )
    )
    shell.merge(
        BoxGeometry((0.760, 0.012, 0.010)).translate(
            0.0,
            -0.112,
            -0.010,
        )
    )

    for seg_start, seg_end, owner in _hinge_segments():
        if owner != "lower":
            continue
        length = seg_end - seg_start
        center_x = 0.5 * (seg_start + seg_end)
        shell.merge(
            BoxGeometry((length, 0.018, 0.014)).translate(
                center_x,
                -0.118,
                -0.006,
            )
        )
        shell.merge(
            CylinderGeometry(radius=0.0065, height=length, radial_segments=24)
            .rotate_y(math.pi / 2.0)
            .translate(center_x, HINGE_AXIS_Y, HINGE_AXIS_Z)
        )

    housing_center_y = 0.149
    shell.merge(
        BoxGeometry((0.056, 0.012, 0.024)).translate(
            LATCH_X,
            housing_center_y,
            -0.012,
        )
    )
    shell.merge(
        BoxGeometry((0.008, 0.010, 0.018)).translate(
            LATCH_X - 0.017,
            housing_center_y + 0.005,
            -0.009,
        )
    )
    shell.merge(
        BoxGeometry((0.008, 0.010, 0.018)).translate(
            LATCH_X + 0.017,
            housing_center_y + 0.005,
            -0.009,
        )
    )
    shell.merge(
        BoxGeometry((0.044, 0.010, 0.012)).translate(
            LATCH_X,
            0.145,
            -0.018,
        )
    )
    return shell


def _build_lid_mesh() -> MeshGeometry:
    outline = _case_outline()
    outer_top_xy = _scaled_loop(
        outline,
        sx=0.958,
        sy=0.898,
        center=(0.030, 0.010),
        dy=-0.003,
    )
    inner_top_xy = _scaled_loop(
        outline,
        sx=0.912,
        sy=0.806,
        center=(0.030, 0.008),
        dy=-0.005,
    )
    inner_rim_xy = _scaled_loop(
        outline,
        sx=0.970,
        sy=0.935,
        center=(0.030, 0.006),
    )

    lid = _build_shell_mesh(
        outline,
        outer_top_xy,
        inner_top_xy,
        inner_rim_xy,
        outer_mid_z_fn=lambda x_pos, y_pos: _lid_depth(x_pos, y_pos),
        inner_mid_z_fn=lambda x_pos, y_pos: _lid_depth(x_pos, y_pos) - CASE_WALL,
        outer_rim_z=0.0,
        inner_rim_z=CASE_WALL,
    )

    for seg_start, seg_end, owner in _hinge_segments():
        if owner != "lid":
            continue
        length = seg_end - seg_start
        center_x = 0.5 * (seg_start + seg_end)
        lid.merge(
            BoxGeometry((length, 0.024, 0.020)).translate(
                center_x,
                -0.120,
                0.011,
            )
        )
        lid.merge(
            CylinderGeometry(radius=0.0062, height=length, radial_segments=24)
            .rotate_y(math.pi / 2.0)
            .translate(center_x, HINGE_AXIS_Y, HINGE_AXIS_Z)
        )

    lid.merge(
        BoxGeometry((0.055, 0.010, 0.017)).translate(
            LATCH_X,
            0.139,
            0.014,
        )
    )
    lid.merge(
        BoxGeometry((0.032, 0.008, 0.008)).translate(
            LATCH_X,
            0.145,
            0.004,
        )
    )
    lid.translate(0.0, -HINGE_AXIS_Y, -HINGE_AXIS_Z)
    return lid


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="saxophone_case")

    shell_black = model.material("shell_black", rgba=(0.16, 0.17, 0.18, 1.0))
    lid_black = model.material("lid_black", rgba=(0.14, 0.15, 0.16, 1.0))
    hardware_metal = model.material("hardware_metal", rgba=(0.58, 0.60, 0.63, 1.0))
    latch_rubber = model.material("latch_rubber", rgba=(0.11, 0.11, 0.12, 1.0))

    lower_shell = model.part("lower_shell")
    lower_shell.visual(
        mesh_from_geometry(_build_lower_shell_mesh(), "lower_shell_mesh"),
        material=shell_black,
        name="lower_shell_body",
    )
    lower_shell.inertial = Inertial.from_geometry(
        Box((0.850, 0.285, 0.140)),
        mass=2.8,
        origin=Origin(xyz=(0.0, 0.010, -0.065)),
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_geometry(_build_lid_mesh(), "lid_mesh"),
        material=lid_black,
        name="lid_shell_body",
    )
    lid.inertial = Inertial.from_geometry(
        Box((0.850, 0.285, 0.062)),
        mass=1.7,
        origin=Origin(xyz=(0.0, 0.145, 0.020)),
    )

    latch_lever = model.part("latch_lever")
    latch_lever.visual(
        Cylinder(radius=0.0035, length=0.026),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hardware_metal,
        name="lever_pivot",
    )
    latch_lever.visual(
        Box((0.026, 0.010, 0.034)),
        origin=Origin(xyz=(0.0, 0.003, -0.018)),
        material=hardware_metal,
        name="lever_body",
    )
    latch_lever.visual(
        Box((0.030, 0.016, 0.008)),
        origin=Origin(xyz=(0.0, 0.011, -0.036)),
        material=latch_rubber,
        name="lever_tab",
    )
    latch_lever.visual(
        Box((0.018, 0.018, 0.006)),
        origin=Origin(xyz=(0.0, 0.004, -0.028)),
        material=hardware_metal,
        name="lever_hook",
    )
    latch_lever.inertial = Inertial.from_geometry(
        Box((0.032, 0.020, 0.046)),
        mass=0.08,
        origin=Origin(xyz=(0.0, 0.010, -0.020)),
    )

    model.articulation(
        "lower_shell_to_lid",
        ArticulationType.REVOLUTE,
        parent=lower_shell,
        child=lid,
        origin=Origin(xyz=(0.0, HINGE_AXIS_Y, HINGE_AXIS_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.1,
            lower=0.0,
            upper=math.radians(105.0),
        ),
    )
    model.articulation(
        "lower_shell_to_latch_lever",
        ArticulationType.REVOLUTE,
        parent=lower_shell,
        child=latch_lever,
        origin=Origin(xyz=(LATCH_X, 0.161, 0.002)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=3.0,
            lower=0.0,
            upper=1.10,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    lower_shell = object_model.get_part("lower_shell")
    lid = object_model.get_part("lid")
    latch_lever = object_model.get_part("latch_lever")
    lid_hinge = object_model.get_articulation("lower_shell_to_lid")
    latch_pivot = object_model.get_articulation("lower_shell_to_latch_lever")

    ctx.check(
        "lid hinge axis runs along the case length",
        tuple(lid_hinge.axis) == (1.0, 0.0, 0.0),
        details=f"axis={lid_hinge.axis}",
    )
    ctx.check(
        "latch pivot axis runs along the case length",
        tuple(latch_pivot.axis) == (1.0, 0.0, 0.0),
        details=f"axis={latch_pivot.axis}",
    )

    with ctx.pose({lid_hinge: 0.0, latch_pivot: 0.0}):
        ctx.expect_overlap(
            lid,
            lower_shell,
            axes="xy",
            min_overlap=0.18,
            name="lid follows the molded lower-shell outline",
        )

        closed_lid_aabb = ctx.part_world_aabb(lid)
        closed_tab_aabb = ctx.part_element_world_aabb(latch_lever, elem="lever_tab")

    ctx.check(
        "closed lid sits close to the seam plane",
        closed_lid_aabb is not None and -0.003 <= closed_lid_aabb[0][2] <= 0.008,
        details=f"closed={closed_lid_aabb}",
    )

    lid_upper = 0.0
    if lid_hinge.motion_limits is not None and lid_hinge.motion_limits.upper is not None:
        lid_upper = lid_hinge.motion_limits.upper
    latch_upper = 0.0
    if latch_pivot.motion_limits is not None and latch_pivot.motion_limits.upper is not None:
        latch_upper = latch_pivot.motion_limits.upper

    with ctx.pose({lid_hinge: lid_upper}):
        open_lid_aabb = ctx.part_world_aabb(lid)

    with ctx.pose({latch_pivot: latch_upper}):
        open_tab_aabb = ctx.part_element_world_aabb(latch_lever, elem="lever_tab")

    ctx.check(
        "lid opens upward from the rear hinge",
        closed_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][2] > closed_lid_aabb[1][2] + 0.18,
        details=f"closed={closed_lid_aabb}, open={open_lid_aabb}",
    )
    ctx.check(
        "front latch lever lifts when opened",
        closed_tab_aabb is not None
        and open_tab_aabb is not None
        and open_tab_aabb[1][2] > closed_tab_aabb[1][2] + 0.020,
        details=f"closed={closed_tab_aabb}, open={open_tab_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
