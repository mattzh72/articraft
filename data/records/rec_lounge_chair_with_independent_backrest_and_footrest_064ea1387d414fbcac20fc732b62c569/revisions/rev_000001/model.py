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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
    sweep_profile_along_spline,
)


def _yz_section(
    x_pos: float,
    *,
    width: float,
    height: float,
    radius: float,
    y_center: float = 0.0,
    z_center: float = 0.0,
) -> list[tuple[float, float, float]]:
    return [
        (x_pos, y_center + y_pos, z_center + z_pos)
        for y_pos, z_pos in rounded_rect_profile(width, height, radius)
    ]


def _xy_section(
    z_pos: float,
    *,
    width: float,
    depth: float,
    radius: float,
    x_center: float = 0.0,
    y_center: float = 0.0,
) -> list[tuple[float, float, float]]:
    return [
        (x_center + x_pos, y_center + y_pos, z_pos)
        for x_pos, y_pos in rounded_rect_profile(width, depth, radius)
    ]


def _seat_side_mesh(side_sign: float, mesh_name: str):
    return mesh_from_geometry(
        section_loft(
            [
                _yz_section(
                    0.26,
                    width=0.090,
                    height=0.082,
                    radius=0.022,
                    y_center=side_sign * 0.270,
                    z_center=0.075,
                ),
                _yz_section(
                    0.10,
                    width=0.092,
                    height=0.100,
                    radius=0.024,
                    y_center=side_sign * 0.293,
                    z_center=0.095,
                ),
                _yz_section(
                    -0.06,
                    width=0.094,
                    height=0.112,
                    radius=0.025,
                    y_center=side_sign * 0.305,
                    z_center=0.118,
                ),
                _yz_section(
                    -0.22,
                    width=0.090,
                    height=0.108,
                    radius=0.023,
                    y_center=side_sign * 0.282,
                    z_center=0.135,
                ),
            ]
        ),
        mesh_name,
    )


def _leg_mesh(angle: float, mesh_name: str):
    c = math.cos(angle)
    s = math.sin(angle)
    return mesh_from_geometry(
        sweep_profile_along_spline(
            [
                (0.050 * c, 0.050 * s, 0.068),
                (0.165 * c, 0.165 * s, 0.056),
                (0.315 * c, 0.315 * s, 0.018),
            ],
            profile=rounded_rect_profile(0.052, 0.018, radius=0.006, corner_segments=6),
            samples_per_segment=18,
            cap_profile=True,
        ),
        mesh_name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="reading_chair")

    base_metal = model.material("base_metal", rgba=(0.18, 0.19, 0.20, 1.0))
    glide_rubber = model.material("glide_rubber", rgba=(0.08, 0.08, 0.09, 1.0))
    shell_finish = model.material("shell_finish", rgba=(0.44, 0.46, 0.48, 1.0))
    upholstery = model.material("upholstery", rgba=(0.70, 0.66, 0.60, 1.0))
    accent_dark = model.material("accent_dark", rgba=(0.12, 0.13, 0.14, 1.0))

    pedestal = model.part("pedestal")
    pedestal.visual(
        Cylinder(radius=0.060, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.065)),
        material=base_metal,
        name="base_hub",
    )
    for index in range(5):
        angle = index * (2.0 * math.pi / 5.0)
        c = math.cos(angle)
        s = math.sin(angle)
        pedestal.visual(
            _leg_mesh(angle, f"star_leg_{index}"),
            material=base_metal,
            name=f"leg_{index}",
        )
        pedestal.visual(
            Cylinder(radius=0.017, length=0.016),
            origin=Origin(xyz=(0.315 * c, 0.315 * s, 0.008)),
            material=glide_rubber,
            name=f"glide_{index}",
        )
    pedestal.visual(
        Cylinder(radius=0.030, length=0.240),
        origin=Origin(xyz=(0.0, 0.0, 0.190)),
        material=base_metal,
        name="pedestal_column",
    )
    pedestal.visual(
        Cylinder(radius=0.050, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.305)),
        material=accent_dark,
        name="bearing_collar",
    )
    pedestal.visual(
        Cylinder(radius=0.085, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.330)),
        material=accent_dark,
        name="swivel_plate",
    )
    pedestal.inertial = Inertial.from_geometry(
        Box((0.70, 0.70, 0.36)),
        mass=15.0,
        origin=Origin(xyz=(0.0, 0.0, 0.180)),
    )

    seat_shell = model.part("seat_shell")
    seat_pan_mesh = mesh_from_geometry(
        section_loft(
            [
                _yz_section(0.28, width=0.54, height=0.050, radius=0.020, z_center=0.055),
                _yz_section(0.12, width=0.58, height=0.065, radius=0.024, z_center=0.060),
                _yz_section(-0.06, width=0.60, height=0.070, radius=0.026, z_center=0.068),
                _yz_section(-0.24, width=0.52, height=0.058, radius=0.021, z_center=0.078),
            ]
        ),
        "seat_pan",
    )
    seat_cushion_mesh = mesh_from_geometry(
        section_loft(
            [
                _yz_section(0.24, width=0.50, height=0.035, radius=0.016, z_center=0.081),
                _yz_section(0.08, width=0.53, height=0.045, radius=0.019, z_center=0.084),
                _yz_section(-0.10, width=0.54, height=0.046, radius=0.019, z_center=0.088),
                _yz_section(-0.22, width=0.46, height=0.036, radius=0.016, z_center=0.092),
            ]
        ),
        "seat_cushion",
    )
    seat_shell.visual(
        Cylinder(radius=0.080, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=accent_dark,
        name="swivel_mount",
    )
    seat_shell.visual(
        Box((0.34, 0.24, 0.050)),
        origin=Origin(xyz=(0.02, 0.0, 0.035)),
        material=shell_finish,
        name="center_core",
    )
    seat_shell.visual(
        seat_pan_mesh,
        material=shell_finish,
        name="seat_pan",
    )
    seat_shell.visual(
        seat_cushion_mesh,
        material=upholstery,
        name="seat_cushion",
    )
    seat_shell.visual(
        _seat_side_mesh(1.0, "left_seat_side"),
        material=shell_finish,
        name="left_side_shell",
    )
    seat_shell.visual(
        _seat_side_mesh(-1.0, "right_seat_side"),
        material=shell_finish,
        name="right_side_shell",
    )
    seat_shell.visual(
        Box((0.050, 0.540, 0.080)),
        origin=Origin(xyz=(0.275, 0.0, 0.082)),
        material=shell_finish,
        name="front_apron",
    )
    seat_shell.visual(
        Box((0.040, 0.420, 0.014)),
        origin=Origin(xyz=(0.275, 0.0, 0.025)),
        material=accent_dark,
        name="support_hinge_beam",
    )
    seat_shell.visual(
        Box((0.080, 0.460, 0.040)),
        origin=Origin(xyz=(-0.240, 0.0, 0.105)),
        material=accent_dark,
        name="rear_beam",
    )
    seat_shell.visual(
        Box((0.050, 0.050, 0.100)),
        origin=Origin(xyz=(-0.240, 0.240, 0.145)),
        material=accent_dark,
        name="left_hinge_cheek",
    )
    seat_shell.visual(
        Box((0.050, 0.050, 0.100)),
        origin=Origin(xyz=(-0.240, -0.240, 0.145)),
        material=accent_dark,
        name="right_hinge_cheek",
    )
    seat_shell.inertial = Inertial.from_geometry(
        Box((0.66, 0.64, 0.22)),
        mass=9.0,
        origin=Origin(xyz=(0.0, 0.0, 0.085)),
    )

    back = model.part("back")
    back_shell_mesh = mesh_from_geometry(
        section_loft(
            [
                _xy_section(0.06, width=0.43, depth=0.08, radius=0.020, x_center=-0.015),
                _xy_section(0.24, width=0.52, depth=0.09, radius=0.024, x_center=-0.045),
                _xy_section(0.42, width=0.50, depth=0.08, radius=0.022, x_center=-0.080),
                _xy_section(0.54, width=0.42, depth=0.06, radius=0.018, x_center=-0.105),
            ]
        ),
        "back_shell",
    )
    back_pad_mesh = mesh_from_geometry(
        section_loft(
            [
                _xy_section(0.08, width=0.35, depth=0.040, radius=0.015, x_center=0.010),
                _xy_section(0.24, width=0.44, depth=0.052, radius=0.019, x_center=-0.020),
                _xy_section(0.40, width=0.42, depth=0.046, radius=0.017, x_center=-0.040),
                _xy_section(0.50, width=0.34, depth=0.032, radius=0.014, x_center=-0.055),
            ]
        ),
        "back_pad",
    )
    back.visual(
        Box((0.080, 0.420, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=accent_dark,
        name="lower_rail",
    )
    back.visual(
        Box((0.090, 0.180, 0.120)),
        origin=Origin(xyz=(-0.025, 0.0, 0.090)),
        material=shell_finish,
        name="back_spine",
    )
    back.visual(
        back_shell_mesh,
        origin=Origin(rpy=(0.0, -0.18, 0.0)),
        material=shell_finish,
        name="back_shell",
    )
    back.visual(
        back_pad_mesh,
        origin=Origin(rpy=(0.0, -0.18, 0.0)),
        material=upholstery,
        name="back_pad",
    )
    back.inertial = Inertial.from_geometry(
        Box((0.22, 0.54, 0.58)),
        mass=5.0,
        origin=Origin(xyz=(-0.05, 0.0, 0.290)),
    )

    calf_support = model.part("calf_support")
    calf_support.visual(
        Box((0.040, 0.400, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, -0.007)),
        material=accent_dark,
        name="hinge_block",
    )
    calf_support.visual(
        Box((0.035, 0.440, 0.180)),
        origin=Origin(xyz=(0.0, 0.0, -0.104)),
        material=shell_finish,
        name="support_shell",
    )
    calf_support.visual(
        Box((0.020, 0.340, 0.140)),
        origin=Origin(xyz=(0.010, 0.0, -0.105)),
        material=upholstery,
        name="support_pad",
    )
    calf_support.inertial = Inertial.from_geometry(
        Box((0.050, 0.440, 0.200)),
        mass=1.0,
        origin=Origin(xyz=(0.0, 0.0, -0.100)),
    )

    model.articulation(
        "pedestal_swivel",
        ArticulationType.CONTINUOUS,
        parent=pedestal,
        child=seat_shell,
        origin=Origin(xyz=(0.0, 0.0, 0.345)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=40.0, velocity=1.5),
    )
    model.articulation(
        "back_tilt",
        ArticulationType.REVOLUTE,
        parent=seat_shell,
        child=back,
        origin=Origin(xyz=(-0.240, 0.0, 0.125)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=1.0,
            lower=-0.18,
            upper=0.35,
        ),
    )
    model.articulation(
        "calf_support_hinge",
        ArticulationType.REVOLUTE,
        parent=seat_shell,
        child=calf_support,
        origin=Origin(xyz=(0.275, 0.0, 0.018)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=1.2,
            lower=0.0,
            upper=1.10,
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

    pedestal = object_model.get_part("pedestal")
    seat_shell = object_model.get_part("seat_shell")
    back = object_model.get_part("back")
    calf_support = object_model.get_part("calf_support")

    pedestal_swivel = object_model.get_articulation("pedestal_swivel")
    back_tilt = object_model.get_articulation("back_tilt")
    calf_hinge = object_model.get_articulation("calf_support_hinge")

    ctx.expect_origin_distance(
        seat_shell,
        pedestal,
        axes="xy",
        max_dist=0.001,
        name="seat shell stays centered on swivel pedestal",
    )
    ctx.expect_contact(
        seat_shell,
        pedestal,
        elem_a="swivel_mount",
        elem_b="swivel_plate",
        name="seat shell sits on the swivel plate",
    )
    ctx.expect_contact(
        back,
        seat_shell,
        elem_a="lower_rail",
        elem_b="rear_beam",
        name="back rail meets the shell rear beam at the hinge line",
    )
    ctx.expect_within(
        back,
        seat_shell,
        axes="y",
        inner_elem="lower_rail",
        outer_elem="rear_beam",
        margin=0.0,
        name="back rail stays centered between the shell hinge cheeks",
    )
    ctx.expect_contact(
        calf_support,
        seat_shell,
        elem_a="hinge_block",
        elem_b="support_hinge_beam",
        name="calf support nests against the underside hinge beam",
    )
    ctx.expect_within(
        calf_support,
        seat_shell,
        axes="y",
        inner_elem="hinge_block",
        outer_elem="support_hinge_beam",
        margin=0.0,
        name="calf support hinge block stays centered under the seat front",
    )

    rest_back_pos = ctx.part_world_position(back)
    with ctx.pose({pedestal_swivel: math.pi / 2.0}):
        turned_back_pos = ctx.part_world_position(back)
    ctx.check(
        "pedestal swivel rotates the chair around a vertical axis",
        rest_back_pos is not None
        and turned_back_pos is not None
        and abs(rest_back_pos[0]) > 0.18
        and abs(turned_back_pos[1]) > 0.18
        and abs(turned_back_pos[0]) < 0.05,
        details=f"rest_back_pos={rest_back_pos}, turned_back_pos={turned_back_pos}",
    )

    rest_back_aabb = ctx.part_world_aabb(back)
    with ctx.pose({back_tilt: 0.30}):
        tilted_back_aabb = ctx.part_world_aabb(back)
    ctx.check(
        "back tilts rearward from the shell rear hinge",
        rest_back_aabb is not None
        and tilted_back_aabb is not None
        and tilted_back_aabb[0][0] < rest_back_aabb[0][0] - 0.05,
        details=f"rest_back_aabb={rest_back_aabb}, tilted_back_aabb={tilted_back_aabb}",
    )

    rest_support_aabb = ctx.part_world_aabb(calf_support)
    with ctx.pose({calf_hinge: 1.00}):
        deployed_support_aabb = ctx.part_world_aabb(calf_support)
    ctx.check(
        "calf support swings forward from beneath the front edge",
        rest_support_aabb is not None
        and deployed_support_aabb is not None
        and deployed_support_aabb[1][0] > rest_support_aabb[1][0] + 0.12
        and deployed_support_aabb[0][2] > rest_support_aabb[0][2] + 0.04,
        details=(
            f"rest_support_aabb={rest_support_aabb}, "
            f"deployed_support_aabb={deployed_support_aabb}"
        ),
    )

    pedestal_aabb = ctx.part_world_aabb(pedestal)
    back_aabb = ctx.part_world_aabb(back)
    base_span = None
    chair_top = None
    if pedestal_aabb is not None:
        base_span = max(
            pedestal_aabb[1][0] - pedestal_aabb[0][0],
            pedestal_aabb[1][1] - pedestal_aabb[0][1],
        )
    if back_aabb is not None:
        chair_top = back_aabb[1][2]
    ctx.check(
        "chair proportions stay compact and full scale",
        base_span is not None
        and chair_top is not None
        and 0.58 <= base_span <= 0.78
        and 0.90 <= chair_top <= 1.05,
        details=f"base_span={base_span}, chair_top={chair_top}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
