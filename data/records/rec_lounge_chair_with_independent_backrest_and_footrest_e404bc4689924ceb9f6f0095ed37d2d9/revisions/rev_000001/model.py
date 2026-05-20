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
    rounded_rect_profile,
    section_loft,
    sweep_profile_along_spline,
)


def _mesh(name: str, geometry: MeshGeometry):
    return mesh_from_geometry(geometry, name)


def _merge(*geometries: MeshGeometry) -> MeshGeometry:
    merged = MeshGeometry()
    for geometry in geometries:
        merged.merge(geometry)
    return merged


def _yz_section(
    x_pos: float,
    *,
    width: float,
    height: float,
    z_center: float,
    corner: float,
) -> list[tuple[float, float, float]]:
    return [(x_pos, y, z_center + z) for y, z in rounded_rect_profile(width, height, corner)]


def _xy_section(
    z_pos: float,
    *,
    width: float,
    depth: float,
    x_center: float,
    corner: float,
) -> list[tuple[float, float, float]]:
    return [(x_center + x, y, z_pos) for x, y in rounded_rect_profile(depth, width, corner)]


def _build_star_base_mesh() -> MeshGeometry:
    arm_profile = rounded_rect_profile(0.058, 0.030, 0.010)
    arm_path = [
        (0.085, 0.0, 0.050),
        (0.165, 0.0, 0.044),
        (0.255, 0.0, 0.032),
        (0.365, 0.0, 0.017),
    ]

    base = _merge(
        CylinderGeometry(radius=0.112, height=0.040, radial_segments=40).translate(0.0, 0.0, 0.038),
        CylinderGeometry(radius=0.074, height=0.055, radial_segments=36).translate(0.0, 0.0, 0.065),
    )

    for index in range(5):
        angle = index * (2.0 * math.pi / 5.0)
        base.merge(
            sweep_profile_along_spline(
                arm_path,
                profile=arm_profile,
                samples_per_segment=12,
                cap_profile=True,
            ).rotate_z(angle)
        )
        base.merge(
            CylinderGeometry(radius=0.028, height=0.010, radial_segments=24)
            .translate(0.365, 0.0, 0.005)
            .rotate_z(angle)
        )

    return base


def _build_seat_shell_mesh() -> MeshGeometry:
    return section_loft(
        [
            _yz_section(-0.280, width=0.500, height=0.110, z_center=0.215, corner=0.040),
            _yz_section(-0.110, width=0.610, height=0.120, z_center=0.205, corner=0.045),
            _yz_section(0.060, width=0.650, height=0.112, z_center=0.190, corner=0.044),
            _yz_section(0.220, width=0.600, height=0.102, z_center=0.175, corner=0.040),
            _yz_section(0.305, width=0.540, height=0.090, z_center=0.165, corner=0.034),
        ]
    )


def _build_side_flange_mesh(sign: float) -> MeshGeometry:
    return sweep_profile_along_spline(
        [
            (-0.235, sign * 0.225, 0.225),
            (-0.080, sign * 0.295, 0.220),
            (0.105, sign * 0.318, 0.200),
            (0.275, sign * 0.265, 0.167),
        ],
        profile=rounded_rect_profile(0.045, 0.085, 0.012),
        samples_per_segment=16,
        cap_profile=True,
    )


def _build_back_panel_mesh() -> MeshGeometry:
    return section_loft(
        [
            _xy_section(0.080, width=0.560, depth=0.100, x_center=-0.070, corner=0.032),
            _xy_section(0.250, width=0.610, depth=0.102, x_center=-0.095, corner=0.034),
            _xy_section(0.490, width=0.570, depth=0.095, x_center=-0.135, corner=0.032),
            _xy_section(0.710, width=0.490, depth=0.082, x_center=-0.165, corner=0.028),
        ]
    )


def _build_foot_panel_mesh() -> MeshGeometry:
    return section_loft(
        [
            _yz_section(0.018, width=0.455, height=0.075, z_center=0.000, corner=0.024),
            _yz_section(0.115, width=0.505, height=0.078, z_center=-0.004, corner=0.024),
            _yz_section(0.235, width=0.545, height=0.074, z_center=-0.010, corner=0.022),
            _yz_section(0.338, width=0.520, height=0.068, z_center=-0.018, corner=0.020),
        ]
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="swivel_lounge_chair")

    base_metal = model.material("base_metal", rgba=(0.70, 0.72, 0.76, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.18, 0.19, 0.21, 1.0))
    shell_wood = model.material("shell_wood", rgba=(0.33, 0.23, 0.15, 1.0))
    upholstery = model.material("upholstery", rgba=(0.60, 0.46, 0.31, 1.0))

    star_base = model.part("star_base")
    star_base.visual(
        _mesh("star_base_assembly", _build_star_base_mesh()),
        material=base_metal,
        name="star_base_assembly",
    )
    star_base.visual(
        Cylinder(radius=0.088, length=0.140),
        origin=Origin(xyz=(0.0, 0.0, 0.110)),
        material=dark_metal,
        name="bearing_housing",
    )
    star_base.inertial = Inertial.from_geometry(
        Cylinder(radius=0.370, length=0.180),
        mass=9.0,
        origin=Origin(xyz=(0.0, 0.0, 0.090)),
    )

    seat_pedestal = model.part("seat_pedestal")
    seat_pedestal.visual(
        Cylinder(radius=0.082, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=dark_metal,
        name="swivel_collar",
    )
    seat_pedestal.visual(
        Cylinder(radius=0.046, length=0.240),
        origin=Origin(xyz=(0.0, 0.0, 0.150)),
        material=base_metal,
        name="pedestal_column",
    )
    seat_pedestal.visual(
        Box((0.240, 0.180, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, 0.240)),
        material=dark_metal,
        name="top_mount_plate",
    )
    seat_pedestal.visual(
        Box((0.180, 0.150, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, 0.210)),
        material=dark_metal,
        name="underseat_mount",
    )
    seat_pedestal.visual(
        _mesh("seat_shell", _build_seat_shell_mesh()),
        material=upholstery,
        name="seat_shell",
    )
    seat_pedestal.visual(
        _mesh("left_shell_flange", _build_side_flange_mesh(1.0)),
        material=shell_wood,
        name="left_shell_flange",
    )
    seat_pedestal.visual(
        _mesh("right_shell_flange", _build_side_flange_mesh(-1.0)),
        material=shell_wood,
        name="right_shell_flange",
    )
    seat_pedestal.visual(
        Box((0.084, 0.380, 0.050)),
        origin=Origin(xyz=(0.082, 0.0, 0.105)),
        material=dark_metal,
        name="front_hinge_support",
    )
    seat_pedestal.visual(
        Box((0.058, 0.080, 0.056)),
        origin=Origin(xyz=(-0.266, 0.165, 0.252)),
        material=dark_metal,
        name="rear_hinge_mount_left",
    )
    seat_pedestal.visual(
        Box((0.058, 0.080, 0.056)),
        origin=Origin(xyz=(-0.266, -0.165, 0.252)),
        material=dark_metal,
        name="rear_hinge_mount_right",
    )
    seat_pedestal.visual(
        Cylinder(radius=0.016, length=0.100),
        origin=Origin(xyz=(0.140, 0.155, 0.080), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="front_hinge_left",
    )
    seat_pedestal.visual(
        Cylinder(radius=0.016, length=0.100),
        origin=Origin(xyz=(0.140, -0.155, 0.080), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="front_hinge_right",
    )
    seat_pedestal.visual(
        Cylinder(radius=0.016, length=0.110),
        origin=Origin(xyz=(-0.295, 0.165, 0.294), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="rear_hinge_left",
    )
    seat_pedestal.visual(
        Cylinder(radius=0.016, length=0.110),
        origin=Origin(xyz=(-0.295, -0.165, 0.294), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="rear_hinge_right",
    )
    seat_pedestal.inertial = Inertial.from_geometry(
        Box((0.700, 0.670, 0.350)),
        mass=12.0,
        origin=Origin(xyz=(0.0, 0.0, 0.205)),
    )

    backrest = model.part("backrest")
    backrest.visual(
        Cylinder(radius=0.015, length=0.220),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="back_hinge_barrel",
    )
    backrest.visual(
        Box((0.055, 0.210, 0.090)),
        origin=Origin(xyz=(-0.030, 0.0, 0.050)),
        material=dark_metal,
        name="back_lower_bridge",
    )
    backrest.visual(
        _mesh("back_panel", _build_back_panel_mesh()),
        material=upholstery,
        name="back_panel",
    )
    backrest.inertial = Inertial.from_geometry(
        Box((0.200, 0.620, 0.760)),
        mass=5.5,
        origin=Origin(xyz=(-0.115, 0.0, 0.360)),
    )

    foot_panel = model.part("foot_panel")
    foot_panel.visual(
        Cylinder(radius=0.014, length=0.210),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="foot_hinge_barrel",
    )
    foot_panel.visual(
        Box((0.078, 0.180, 0.050)),
        origin=Origin(xyz=(0.050, 0.0, 0.000)),
        material=dark_metal,
        name="foot_bracket",
    )
    foot_panel.visual(
        _mesh("foot_panel_surface", _build_foot_panel_mesh()),
        material=upholstery,
        name="foot_panel_surface",
    )
    foot_panel.inertial = Inertial.from_geometry(
        Box((0.380, 0.560, 0.090)),
        mass=2.2,
        origin=Origin(xyz=(0.190, 0.0, -0.005)),
    )

    swivel = model.articulation(
        "base_swivel",
        ArticulationType.CONTINUOUS,
        parent=star_base,
        child=seat_pedestal,
        origin=Origin(xyz=(0.0, 0.0, 0.180)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=50.0, velocity=2.5),
    )
    back_hinge = model.articulation(
        "shell_to_backrest",
        ArticulationType.REVOLUTE,
        parent=seat_pedestal,
        child=backrest,
        origin=Origin(xyz=(-0.295, 0.0, 0.294)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.0,
            lower=-0.18,
            upper=0.42,
        ),
    )
    foot_hinge = model.articulation(
        "shell_to_foot_panel",
        ArticulationType.REVOLUTE,
        parent=seat_pedestal,
        child=foot_panel,
        origin=Origin(xyz=(0.140, 0.0, 0.080)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.4,
            lower=-0.20,
            upper=0.95,
        ),
    )

    seat_pedestal.meta["named_joints"] = {
        "swivel": swivel.name,
        "back_hinge": back_hinge.name,
        "foot_hinge": foot_hinge.name,
    }

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    star_base = object_model.get_part("star_base")
    seat_pedestal = object_model.get_part("seat_pedestal")
    backrest = object_model.get_part("backrest")
    foot_panel = object_model.get_part("foot_panel")

    swivel = object_model.get_articulation("base_swivel")
    back_hinge = object_model.get_articulation("shell_to_backrest")
    foot_hinge = object_model.get_articulation("shell_to_foot_panel")

    def _aabb_center(aabb):
        if aabb is None:
            return None
        min_corner, max_corner = aabb
        return tuple((min_corner[index] + max_corner[index]) * 0.5 for index in range(3))

    with ctx.pose({swivel: 0.0, back_hinge: 0.0, foot_hinge: 0.0}):
        ctx.expect_gap(
            seat_pedestal,
            star_base,
            axis="z",
            positive_elem="swivel_collar",
            negative_elem="bearing_housing",
            min_gap=0.0,
            max_gap=0.004,
            name="swivel collar sits just above bearing housing",
        )
        ctx.expect_overlap(
            seat_pedestal,
            star_base,
            axes="xy",
            elem_a="swivel_collar",
            elem_b="bearing_housing",
            min_overlap=0.150,
            name="swivel collar stays concentric over bearing housing",
        )
        ctx.expect_overlap(
            backrest,
            seat_pedestal,
            axes="xz",
            elem_a="back_hinge_barrel",
            elem_b="rear_hinge_left",
            min_overlap=0.028,
            name="back hinge barrel aligns with the rear hinge axis",
        )
        ctx.expect_overlap(
            foot_panel,
            seat_pedestal,
            axes="xz",
            elem_a="foot_hinge_barrel",
            elem_b="front_hinge_left",
            min_overlap=0.024,
            name="foot panel hinge barrel aligns with the front hinge axis",
        )
        ctx.expect_contact(
            foot_panel,
            seat_pedestal,
            elem_a="foot_hinge_barrel",
            elem_b="front_hinge_left",
            name="foot panel hinge barrel meets the left knuckle",
        )
        ctx.expect_contact(
            foot_panel,
            seat_pedestal,
            elem_a="foot_hinge_barrel",
            elem_b="front_hinge_right",
            name="foot panel hinge barrel meets the right knuckle",
        )

    left_rest = _aabb_center(ctx.part_element_world_aabb(seat_pedestal, elem="left_shell_flange"))
    with ctx.pose({swivel: math.pi / 2.0}):
        left_turned = _aabb_center(ctx.part_element_world_aabb(seat_pedestal, elem="left_shell_flange"))
    ctx.check(
        "base swivel rotates seat assembly around vertical axis",
        left_rest is not None
        and left_turned is not None
        and left_rest[1] > 0.18
        and left_turned[0] < left_rest[0] - 0.18
        and abs(left_turned[1]) < 0.10,
        details=f"rest={left_rest}, turned={left_turned}",
    )

    back_rest = _aabb_center(ctx.part_element_world_aabb(backrest, elem="back_panel"))
    with ctx.pose({back_hinge: 0.40}):
        back_reclined = _aabb_center(ctx.part_element_world_aabb(backrest, elem="back_panel"))
    ctx.check(
        "backrest reclines rearward about the transverse shell hinge",
        back_rest is not None
        and back_reclined is not None
        and back_reclined[0] < back_rest[0] - 0.07,
        details=f"rest={back_rest}, reclined={back_reclined}",
    )

    foot_rest = _aabb_center(ctx.part_element_world_aabb(foot_panel, elem="foot_panel_surface"))
    with ctx.pose({foot_hinge: 0.80}):
        foot_folded = _aabb_center(ctx.part_element_world_aabb(foot_panel, elem="foot_panel_surface"))
    ctx.check(
        "foot panel rotates downward from its own transverse hinge",
        foot_rest is not None
        and foot_folded is not None
        and foot_folded[2] < foot_rest[2] - 0.08,
        details=f"rest={foot_rest}, folded={foot_folded}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
