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
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _xz_section(
    width: float,
    height: float,
    *,
    y: float,
    z_center: float,
    radius: float,
    corner_segments: int = 8,
) -> list[tuple[float, float, float]]:
    return [
        (x, y, z + z_center)
        for x, z in rounded_rect_profile(width, height, radius, corner_segments=corner_segments)
    ]


def _xy_section(
    width: float,
    depth: float,
    *,
    z: float,
    y_center: float = 0.0,
    radius: float,
    corner_segments: int = 8,
) -> list[tuple[float, float, float]]:
    return [
        (x, y + y_center, z)
        for x, y in rounded_rect_profile(width, depth, radius, corner_segments=corner_segments)
    ]


def _aabb_center(aabb):
    if aabb is None:
        return None
    lo, hi = aabb
    return tuple((lo[i] + hi[i]) * 0.5 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="swivel_lounge_chair")

    pedestal_paint = model.material("pedestal_paint", rgba=(0.18, 0.19, 0.21, 1.0))
    upholstery = model.material("upholstery", rgba=(0.68, 0.64, 0.58, 1.0))
    upholstery_dark = model.material("upholstery_dark", rgba=(0.47, 0.44, 0.40, 1.0))
    bearing_metal = model.material("bearing_metal", rgba=(0.53, 0.55, 0.58, 1.0))

    pedestal_base = model.part("pedestal_base")
    pedestal_profile = [
        (0.0, 0.0),
        (0.34, 0.0),
        (0.34, 0.018),
        (0.30, 0.032),
        (0.22, 0.050),
        (0.14, 0.090),
        (0.11, 0.180),
        (0.11, 0.290),
        (0.17, 0.314),
        (0.20, 0.340),
        (0.0, 0.340),
    ]
    pedestal_base.visual(
        _mesh("chair_pedestal_body", LatheGeometry(pedestal_profile, segments=72)),
        material=pedestal_paint,
        name="pedestal_body",
    )
    pedestal_base.inertial = Inertial.from_geometry(
        Cylinder(radius=0.34, length=0.34),
        mass=24.0,
        origin=Origin(xyz=(0.0, 0.0, 0.17)),
    )

    seat_shell = model.part("seat_shell")
    seat_cushion = section_loft(
        [
            _xz_section(0.54, 0.082, y=0.295, z_center=0.083, radius=0.030),
            _xz_section(0.66, 0.110, y=0.100, z_center=0.086, radius=0.045),
            _xz_section(0.72, 0.128, y=-0.090, z_center=0.083, radius=0.050),
            _xz_section(0.64, 0.110, y=-0.300, z_center=0.088, radius=0.040),
        ]
    )
    seat_shell.visual(
        _mesh("chair_seat_shell", seat_cushion),
        material=upholstery,
        name="seat_shell_body",
    )
    seat_shell.visual(
        Box((0.60, 0.50, 0.060)),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=upholstery_dark,
        name="seat_undertray",
    )
    seat_shell.visual(
        Box((0.10, 0.34, 0.090)),
        origin=Origin(xyz=(-0.285, 0.010, 0.075)),
        material=upholstery,
        name="left_seat_bolster",
    )
    seat_shell.visual(
        Box((0.10, 0.34, 0.090)),
        origin=Origin(xyz=(0.285, 0.010, 0.075)),
        material=upholstery,
        name="right_seat_bolster",
    )
    seat_shell.visual(
        Cylinder(radius=0.120, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=bearing_metal,
        name="seat_swivel_hub",
    )
    seat_shell.visual(
        Box((0.60, 0.080, 0.050)),
        origin=Origin(xyz=(0.0, -0.290, 0.070)),
        material=upholstery_dark,
        name="rear_hinge_bridge",
    )
    seat_shell.visual(
        Box((0.070, 0.040, 0.030)),
        origin=Origin(xyz=(-0.225, -0.323, 0.103)),
        material=upholstery_dark,
        name="left_back_hinge_cheek",
    )
    seat_shell.visual(
        Box((0.070, 0.040, 0.030)),
        origin=Origin(xyz=(0.225, -0.323, 0.103)),
        material=upholstery_dark,
        name="right_back_hinge_cheek",
    )
    seat_shell.visual(
        Box((0.54, 0.070, 0.034)),
        origin=Origin(xyz=(0.0, 0.285, 0.017)),
        material=upholstery_dark,
        name="front_hinge_bridge",
    )
    seat_shell.inertial = Inertial.from_geometry(
        Box((0.74, 0.66, 0.18)),
        mass=14.0,
        origin=Origin(xyz=(0.0, 0.0, 0.09)),
    )

    backrest = model.part("backrest")
    back_shell = section_loft(
        [
            _xy_section(0.62, 0.115, z=0.000, y_center=-0.045, radius=0.030),
            _xy_section(0.70, 0.108, z=0.250, y_center=-0.060, radius=0.042),
            _xy_section(0.66, 0.095, z=0.470, y_center=-0.078, radius=0.038),
            _xy_section(0.54, 0.080, z=0.650, y_center=-0.098, radius=0.030),
        ]
    )
    backrest.visual(
        Cylinder(radius=0.018, length=0.600),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bearing_metal,
        name="back_hinge_tube",
    )
    backrest.visual(
        _mesh("chair_back_shell", back_shell),
        origin=Origin(rpy=(0.18, 0.0, 0.0)),
        material=upholstery,
        name="back_shell",
    )
    backrest.inertial = Inertial.from_geometry(
        Box((0.72, 0.22, 0.70)),
        mass=8.0,
        origin=Origin(xyz=(0.0, -0.08, 0.34)),
    )

    leg_panel = model.part("leg_panel")
    leg_shell = section_loft(
        [
            _xy_section(0.54, 0.060, z=-0.015, y_center=0.055, radius=0.020),
            _xy_section(0.58, 0.060, z=-0.210, y_center=0.072, radius=0.026),
            _xy_section(0.54, 0.058, z=-0.405, y_center=0.095, radius=0.022),
        ]
    )
    leg_panel.visual(
        Cylinder(radius=0.016, length=0.560),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bearing_metal,
        name="leg_hinge_tube",
    )
    leg_panel.visual(
        Box((0.030, 0.050, 0.100)),
        origin=Origin(xyz=(-0.295, 0.022, -0.060)),
        material=upholstery_dark,
        name="left_leg_link",
    )
    leg_panel.visual(
        Box((0.030, 0.050, 0.100)),
        origin=Origin(xyz=(0.295, 0.022, -0.060)),
        material=upholstery_dark,
        name="right_leg_link",
    )
    leg_panel.visual(
        _mesh("chair_leg_panel", leg_shell),
        material=upholstery,
        name="leg_shell",
    )
    leg_panel.inertial = Inertial.from_geometry(
        Box((0.60, 0.08, 0.42)),
        mass=4.5,
        origin=Origin(xyz=(0.0, 0.075, -0.21)),
    )

    model.articulation(
        "pedestal_swivel",
        ArticulationType.CONTINUOUS,
        parent=pedestal_base,
        child=seat_shell,
        origin=Origin(xyz=(0.0, 0.0, 0.340)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.8),
    )
    model.articulation(
        "seat_to_backrest",
        ArticulationType.REVOLUTE,
        parent=seat_shell,
        child=backrest,
        origin=Origin(xyz=(0.0, -0.330, 0.135)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.2,
            lower=0.0,
            upper=0.80,
        ),
    )
    model.articulation(
        "seat_to_leg_panel",
        ArticulationType.REVOLUTE,
        parent=seat_shell,
        child=leg_panel,
        origin=Origin(xyz=(0.0, 0.310, 0.055)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.5,
            lower=0.0,
            upper=1.25,
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

    pedestal = object_model.get_part("pedestal_base")
    seat = object_model.get_part("seat_shell")
    backrest = object_model.get_part("backrest")
    leg_panel = object_model.get_part("leg_panel")

    swivel = object_model.get_articulation("pedestal_swivel")
    back_joint = object_model.get_articulation("seat_to_backrest")
    leg_joint = object_model.get_articulation("seat_to_leg_panel")

    ctx.check("pedestal exists", pedestal is not None)
    ctx.check("seat exists", seat is not None)
    ctx.check("backrest exists", backrest is not None)
    ctx.check("leg panel exists", leg_panel is not None)

    ctx.check(
        "swivel axis is vertical",
        tuple(round(v, 6) for v in swivel.axis) == (0.0, 0.0, 1.0),
        details=f"axis={swivel.axis}",
    )
    ctx.check(
        "backrest hinge axis is transverse",
        tuple(round(v, 6) for v in back_joint.axis) == (1.0, 0.0, 0.0),
        details=f"axis={back_joint.axis}",
    )
    ctx.check(
        "leg panel hinge axis is transverse",
        tuple(round(v, 6) for v in leg_joint.axis) == (1.0, 0.0, 0.0),
        details=f"axis={leg_joint.axis}",
    )

    with ctx.pose({swivel: 0.0, back_joint: 0.0, leg_joint: 0.0}):
        ctx.expect_gap(
            seat,
            pedestal,
            axis="z",
            positive_elem="seat_swivel_hub",
            negative_elem="pedestal_body",
            min_gap=0.0,
            max_gap=0.002,
            name="seat swivel hub sits on pedestal top",
        )
        ctx.expect_origin_gap(
            seat,
            backrest,
            axis="y",
            min_gap=0.30,
            max_gap=0.36,
            name="backrest hinge sits at the rear of the seat",
        )
        ctx.expect_gap(
            leg_panel,
            seat,
            axis="y",
            positive_elem="leg_shell",
            negative_elem="front_hinge_bridge",
            min_gap=0.0,
            max_gap=0.080,
            name="leg panel nests just ahead of the front hinge bridge",
        )

        back_rest_aabb = ctx.part_element_world_aabb(backrest, elem="back_shell")
        leg_rest_aabb = ctx.part_element_world_aabb(leg_panel, elem="leg_shell")
        swivel_rest_aabb = ctx.part_element_world_aabb(backrest, elem="back_shell")

    with ctx.pose({back_joint: 0.62}):
        back_reclined_aabb = ctx.part_element_world_aabb(backrest, elem="back_shell")

    with ctx.pose({leg_joint: 1.10}):
        leg_extended_aabb = ctx.part_element_world_aabb(leg_panel, elem="leg_shell")

    with ctx.pose({swivel: 0.75}):
        swivel_rotated_aabb = ctx.part_element_world_aabb(backrest, elem="back_shell")

    back_rest_center = _aabb_center(back_rest_aabb)
    back_reclined_center = _aabb_center(back_reclined_aabb)
    leg_rest_center = _aabb_center(leg_rest_aabb)
    leg_extended_center = _aabb_center(leg_extended_aabb)
    swivel_rest_center = _aabb_center(swivel_rest_aabb)
    swivel_rotated_center = _aabb_center(swivel_rotated_aabb)

    ctx.check(
        "backrest reclines backward",
        back_rest_center is not None
        and back_reclined_center is not None
        and back_reclined_center[1] < back_rest_center[1] - 0.08,
        details=f"rest={back_rest_center}, reclined={back_reclined_center}",
    )
    ctx.check(
        "leg panel folds forward",
        leg_rest_center is not None
        and leg_extended_center is not None
        and leg_extended_center[1] > leg_rest_center[1] + 0.12,
        details=f"rest={leg_rest_center}, extended={leg_extended_center}",
    )
    ctx.check(
        "swivel rotates chair plan view",
        swivel_rest_center is not None
        and swivel_rotated_center is not None
        and (
            abs(swivel_rotated_center[0] - swivel_rest_center[0]) > 0.05
            or abs(swivel_rotated_center[1] - swivel_rest_center[1]) > 0.05
        ),
        details=f"rest={swivel_rest_center}, rotated={swivel_rotated_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
