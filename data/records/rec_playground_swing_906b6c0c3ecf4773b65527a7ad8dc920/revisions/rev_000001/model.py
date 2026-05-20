from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi, radians

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    BoxGeometry,
    Cylinder,
    CylinderGeometry,
    ExtrudeGeometry,
    Inertial,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    sample_catmull_rom_spline_2d,
    section_loft,
    tube_from_spline_points,
)


def _save_mesh(name: str, geometry) -> object:
    return mesh_from_geometry(geometry, name)


def _merge_geometries(*geometries) -> MeshGeometry:
    merged = MeshGeometry()
    for geometry in geometries:
        merged.merge(geometry)
    return merged


def _yz_section(
    x_pos: float,
    *,
    width: float,
    height: float,
    radius: float,
    z_center: float = 0.0,
) -> list[tuple[float, float, float]]:
    return [
        (x_pos, y_pos, z_pos + z_center)
        for z_pos, y_pos in rounded_rect_profile(height, width, radius, corner_segments=6)
    ]


def _build_frame_legs_mesh() -> MeshGeometry:
    tube_radius = 0.027
    left_top = (0.0, 0.47, 1.46)
    right_top = (0.0, -0.47, 1.46)
    left_front_foot = (0.30, 0.56, 0.03)
    left_rear_foot = (-0.30, 0.56, 0.03)
    right_front_foot = (0.30, -0.56, 0.03)
    right_rear_foot = (-0.30, -0.56, 0.03)

    return _merge_geometries(
        tube_from_spline_points([left_top, left_front_foot], radius=tube_radius, samples_per_segment=2, radial_segments=18),
        tube_from_spline_points([left_top, left_rear_foot], radius=tube_radius, samples_per_segment=2, radial_segments=18),
        tube_from_spline_points([right_top, right_front_foot], radius=tube_radius, samples_per_segment=2, radial_segments=18),
        tube_from_spline_points([right_top, right_rear_foot], radius=tube_radius, samples_per_segment=2, radial_segments=18),
        tube_from_spline_points([left_front_foot, left_rear_foot], radius=tube_radius, samples_per_segment=2, radial_segments=18),
        tube_from_spline_points([right_front_foot, right_rear_foot], radius=tube_radius, samples_per_segment=2, radial_segments=18),
    )


def _build_crossbeam_mesh() -> MeshGeometry:
    return BoxGeometry((0.10, 1.14, 0.07)).translate(0.0, 0.0, 1.50)


def _build_hanger_mesh() -> MeshGeometry:
    left_sign = 1.0
    right_sign = -1.0
    return _merge_geometries(
        BoxGeometry((0.020, 0.034, 0.50)).translate(0.0, left_sign * 0.22, -0.255),
        BoxGeometry((0.020, 0.034, 0.50)).translate(0.0, right_sign * 0.22, -0.255),
        BoxGeometry((0.042, 0.050, 0.032)).translate(0.0, left_sign * 0.22, -0.006),
        BoxGeometry((0.042, 0.050, 0.032)).translate(0.0, right_sign * 0.22, -0.006),
        BoxGeometry((0.050, 0.042, 0.084)).translate(0.0, left_sign * 0.22, -0.500),
        BoxGeometry((0.050, 0.042, 0.084)).translate(0.0, right_sign * 0.22, -0.500),
        BoxGeometry((0.024, 0.020, 0.024)).translate(0.008, left_sign * 0.231, -0.490),
        BoxGeometry((0.024, 0.020, 0.024)).translate(0.008, right_sign * 0.231, -0.490),
        CylinderGeometry(radius=0.008, height=0.052, radial_segments=18).rotate_y(pi / 2.0).translate(0.008, left_sign * 0.236244, -0.490),
        CylinderGeometry(radius=0.008, height=0.052, radial_segments=18).rotate_y(pi / 2.0).translate(0.008, right_sign * 0.236244, -0.490),
    )


def _build_seat_mesh() -> MeshGeometry:
    seat_body = section_loft(
        [
            _yz_section(-0.11, width=0.32, height=0.08, radius=0.026, z_center=0.026),
            _yz_section(-0.04, width=0.42, height=0.11, radius=0.036, z_center=0.004),
            _yz_section(0.04, width=0.45, height=0.105, radius=0.038, z_center=-0.012),
            _yz_section(0.12, width=0.38, height=0.074, radius=0.030, z_center=0.016),
        ]
    )
    seat_body.translate(0.0, 0.0, -0.61)
    seat_body.merge(BoxGeometry((0.18, 0.34, 0.030)).translate(0.01, 0.0, -0.665))
    seat_body.merge(BoxGeometry((0.028, 0.34, 0.026)).translate(0.132, 0.0, -0.596))
    seat_body.merge(BoxGeometry((0.046, 0.26, 0.024)).translate(-0.102, 0.0, -0.556))
    seat_body.merge(BoxGeometry((0.060, 0.038, 0.076)).translate(0.0, 0.204, -0.550))
    seat_body.merge(BoxGeometry((0.060, 0.038, 0.076)).translate(0.0, -0.204, -0.550))
    return seat_body


def _cover_profile() -> list[tuple[float, float]]:
    return sample_catmull_rom_spline_2d(
        [
            (-0.048, -0.048),
            (-0.018, -0.080),
            (0.046, -0.040),
            (0.050, 0.040),
            (0.010, 0.078),
            (-0.050, 0.028),
        ],
        samples_per_segment=8,
        closed=True,
    )


def _build_cover_mesh(side_sign: float) -> MeshGeometry:
    shell = ExtrudeGeometry(_cover_profile(), 0.016, center=True)
    shell.rotate_x(pi / 2.0)
    shell.translate(0.010, side_sign * 0.032, -0.078)
    barrel = CylinderGeometry(radius=0.008, height=0.060, radial_segments=18).rotate_y(pi / 2.0)
    neck = BoxGeometry((0.022, 0.014, 0.018)).translate(0.006, side_sign * 0.010, -0.012)
    bridge = BoxGeometry((0.018, 0.010, 0.032)).translate(0.010, side_sign * 0.021, -0.026)
    rib = BoxGeometry((0.018, 0.010, 0.054)).translate(0.008, side_sign * 0.028, -0.046)
    return _merge_geometries(barrel, neck, bridge, rib, shell)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="backyard_swing")

    frame_paint = model.material("frame_paint", rgba=(0.70, 0.73, 0.76, 1.0))
    dark_hardware = model.material("dark_hardware", rgba=(0.23, 0.24, 0.25, 1.0))
    seat_green = model.material("seat_green", rgba=(0.30, 0.46, 0.23, 1.0))

    frame = model.part("frame")
    frame.visual(_save_mesh("frame_legs", _build_frame_legs_mesh()), material=frame_paint, name="frame_legs")
    frame.visual(_save_mesh("crossbeam", _build_crossbeam_mesh()), material=frame_paint, name="crossbeam")
    frame.inertial = Inertial.from_geometry(
        Box((0.72, 1.22, 1.56)),
        mass=24.0,
        origin=Origin(xyz=(0.0, 0.0, 0.78)),
    )

    swing_assembly = model.part("swing_assembly")
    swing_assembly.visual(_save_mesh("hanger_links", _build_hanger_mesh()), material=dark_hardware, name="hanger_links")
    swing_assembly.visual(_save_mesh("seat_pan", _build_seat_mesh()), material=seat_green, name="seat_pan")
    swing_assembly.inertial = Inertial.from_geometry(
        Box((0.50, 0.50, 0.72)),
        mass=6.5,
        origin=Origin(xyz=(0.0, 0.0, -0.36)),
    )

    left_cover = model.part("left_cover")
    left_cover.visual(_save_mesh("left_cover_shell", _build_cover_mesh(1.0)), material=seat_green, name="cover_shell")
    left_cover.inertial = Inertial.from_geometry(
        Box((0.12, 0.03, 0.16)),
        mass=0.12,
        origin=Origin(xyz=(0.010, 0.016, -0.078)),
    )

    right_cover = model.part("right_cover")
    right_cover.visual(_save_mesh("right_cover_shell", _build_cover_mesh(-1.0)), material=seat_green, name="cover_shell")
    right_cover.inertial = Inertial.from_geometry(
        Box((0.12, 0.03, 0.16)),
        mass=0.12,
        origin=Origin(xyz=(0.010, -0.016, -0.078)),
    )

    model.articulation(
        "frame_to_swing",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=swing_assembly,
        origin=Origin(xyz=(0.0, 0.0, 1.455)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=1.5,
            lower=radians(-24.0),
            upper=radians(24.0),
        ),
    )
    model.articulation(
        "swing_to_left_cover",
        ArticulationType.REVOLUTE,
        parent=swing_assembly,
        child=left_cover,
        origin=Origin(xyz=(0.008, 0.252, -0.490)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=radians(85.0)),
    )
    model.articulation(
        "swing_to_right_cover",
        ArticulationType.REVOLUTE,
        parent=swing_assembly,
        child=right_cover,
        origin=Origin(xyz=(0.008, -0.252, -0.490)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=radians(85.0)),
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

    frame = object_model.get_part("frame")
    swing_assembly = object_model.get_part("swing_assembly")
    left_cover = object_model.get_part("left_cover")
    right_cover = object_model.get_part("right_cover")
    swing_joint = object_model.get_articulation("frame_to_swing")
    left_cover_joint = object_model.get_articulation("swing_to_left_cover")
    right_cover_joint = object_model.get_articulation("swing_to_right_cover")

    ctx.check("frame part exists", frame is not None)
    ctx.check("swing assembly exists", swing_assembly is not None)
    ctx.check("left cover exists", left_cover is not None)
    ctx.check("right cover exists", right_cover is not None)
    ctx.check(
        "seat swings on a lateral axis",
        swing_joint.axis == (0.0, -1.0, 0.0),
        details=f"axis={swing_joint.axis}",
    )
    ctx.check(
        "side covers hinge on mirrored x axes",
        left_cover_joint.axis == (1.0, 0.0, 0.0) and right_cover_joint.axis == (-1.0, 0.0, 0.0),
        details=f"left={left_cover_joint.axis}, right={right_cover_joint.axis}",
    )

    def _elem_center(part, elem: str) -> tuple[float, float, float] | None:
        aabb = ctx.part_element_world_aabb(part, elem=elem)
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple((mins[index] + maxs[index]) * 0.5 for index in range(3))

    def _elem_aabb(part, elem: str):
        return ctx.part_element_world_aabb(part, elem=elem)

    seat_aabb = _elem_aabb(swing_assembly, "seat_pan")
    left_aabb = _elem_aabb(left_cover, "cover_shell")
    right_aabb = _elem_aabb(right_cover, "cover_shell")

    ctx.check(
        "left cover is mounted outboard of the seat body",
        seat_aabb is not None and left_aabb is not None and left_aabb[0][1] > seat_aabb[1][1] - 0.010,
        details=f"seat={seat_aabb}, left_cover={left_aabb}",
    )
    ctx.check(
        "right cover is mounted outboard of the seat body",
        seat_aabb is not None and right_aabb is not None and right_aabb[1][1] < seat_aabb[0][1] + 0.010,
        details=f"seat={seat_aabb}, right_cover={right_aabb}",
    )

    seat_closed = _elem_center(swing_assembly, "seat_pan")
    with ctx.pose({swing_joint: radians(22.0)}):
        seat_forward = _elem_center(swing_assembly, "seat_pan")

    ctx.check(
        "seat swings forward and rises",
        seat_closed is not None
        and seat_forward is not None
        and seat_forward[0] > seat_closed[0] + 0.10
        and seat_forward[2] > seat_closed[2] + 0.04,
        details=f"closed={seat_closed}, swung={seat_forward}",
    )

    left_closed = _elem_center(left_cover, "cover_shell")
    right_closed = _elem_center(right_cover, "cover_shell")
    with ctx.pose({left_cover_joint: radians(68.0), right_cover_joint: radians(68.0)}):
        left_open = _elem_center(left_cover, "cover_shell")
        right_open = _elem_center(right_cover, "cover_shell")

    ctx.check(
        "left cover flips outward and upward",
        left_closed is not None
        and left_open is not None
        and left_open[1] > left_closed[1] + 0.01
        and left_open[2] > left_closed[2] + 0.04,
        details=f"closed={left_closed}, open={left_open}",
    )
    ctx.check(
        "right cover flips outward and upward",
        right_closed is not None
        and right_open is not None
        and right_open[1] < right_closed[1] - 0.01
        and right_open[2] > right_closed[2] + 0.04,
        details=f"closed={right_closed}, open={right_open}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
