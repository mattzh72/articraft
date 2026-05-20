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


def _yz_section(
    x: float,
    width: float,
    height: float,
    radius: float,
    *,
    z_center: float,
):
    return [
        (x, y, z_center + z)
        for y, z in rounded_rect_profile(width, height, radius, corner_segments=10)
    ]


def _xy_section(
    z: float,
    width: float,
    depth: float,
    radius: float,
    *,
    x_center: float = 0.0,
):
    return [
        (x_center + x, y, z)
        for x, y in rounded_rect_profile(depth, width, radius, corner_segments=10)
    ]


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="swivel_lounge_chair")

    pedestal_metal = model.material("pedestal_metal", rgba=(0.18, 0.18, 0.20, 1.0))
    upholstery = model.material("upholstery", rgba=(0.78, 0.72, 0.66, 1.0))
    accent = model.material("accent", rgba=(0.28, 0.24, 0.22, 1.0))

    pedestal_profile = [
        (0.0, 0.0),
        (0.25, 0.0),
        (0.33, 0.008),
        (0.33, 0.025),
        (0.18, 0.034),
        (0.11, 0.050),
        (0.074, 0.100),
        (0.060, 0.205),
        (0.058, 0.235),
        (0.094, 0.255),
        (0.112, 0.280),
        (0.0, 0.280),
    ]
    pedestal_mesh = _mesh("pedestal_shell", LatheGeometry(pedestal_profile, segments=64))

    seat_mesh = _mesh(
        "seat_shell",
        section_loft(
            [
                _yz_section(-0.36, 0.56, 0.128, 0.042, z_center=0.092),
                _yz_section(-0.15, 0.68, 0.176, 0.064, z_center=0.116),
                _yz_section(0.08, 0.74, 0.164, 0.060, z_center=0.110),
                _yz_section(0.24, 0.70, 0.132, 0.048, z_center=0.094),
                _yz_section(0.35, 0.58, 0.088, 0.034, z_center=0.072),
            ]
        ),
    )

    backrest_pad_mesh = _mesh(
        "backrest_pad_shell",
        section_loft(
            [
                _xy_section(-0.37, 0.54, 0.06, 0.016, x_center=0.05),
                _xy_section(-0.10, 0.66, 0.11, 0.032, x_center=0.02),
                _xy_section(0.14, 0.72, 0.16, 0.040, x_center=-0.02),
                _xy_section(0.35, 0.60, 0.10, 0.026, x_center=-0.05),
            ]
        ),
    )

    leg_panel_pad_mesh = _mesh(
        "leg_panel_pad_shell",
        section_loft(
            [
                _yz_section(-0.02, 0.46, 0.20, 0.018, z_center=-0.02),
                _yz_section(-0.006, 0.54, 0.28, 0.022, z_center=0.0),
                _yz_section(0.006, 0.54, 0.30, 0.022, z_center=-0.01),
                _yz_section(0.02, 0.48, 0.22, 0.018, z_center=-0.04),
            ]
        ),
    )

    pedestal = model.part("pedestal")
    pedestal.visual(pedestal_mesh, material=pedestal_metal, name="pedestal_shell")
    pedestal.visual(
        Cylinder(radius=0.12, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.289)),
        material=accent,
        name="swivel_bearing_cap",
    )
    pedestal.inertial = Inertial.from_geometry(
        Cylinder(radius=0.33, length=0.30),
        mass=10.0,
        origin=Origin(xyz=(0.0, 0.0, 0.15)),
    )

    seat_shell = model.part("seat_shell")
    seat_shell.visual(
        Box((0.28, 0.24, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=accent,
        name="mount_plate",
    )
    seat_shell.visual(seat_mesh, material=upholstery, name="seat_body")
    seat_shell.visual(
        Box((0.04, 0.46, 0.055)),
        origin=Origin(xyz=(-0.355, 0.0, 0.132)),
        material=accent,
        name="back_hinge_beam",
    )
    seat_shell.visual(
        Box((0.05, 0.46, 0.050)),
        origin=Origin(xyz=(0.34, 0.0, 0.020)),
        material=accent,
        name="leg_hinge_beam",
    )
    seat_shell.inertial = Inertial.from_geometry(
        Box((0.76, 0.74, 0.22)),
        mass=9.0,
        origin=Origin(xyz=(0.0, 0.0, 0.11)),
    )

    backrest = model.part("backrest")
    backrest.visual(
        Box((0.035, 0.46, 0.050)),
        origin=Origin(xyz=(-0.0175, 0.0, 0.025)),
        material=accent,
        name="backrest_shoe",
    )
    backrest.visual(
        Box((0.040, 0.46, 0.180)),
        origin=Origin(xyz=(-0.055, 0.0, 0.140)),
        material=accent,
        name="backrest_spine",
    )
    backrest.visual(
        backrest_pad_mesh,
        origin=Origin(xyz=(-0.120, 0.0, 0.580)),
        material=upholstery,
        name="backrest_pad",
    )
    backrest.inertial = Inertial.from_geometry(
        Box((0.22, 0.74, 0.78)),
        mass=6.0,
        origin=Origin(xyz=(-0.11, 0.0, 0.46)),
    )

    leg_panel = model.part("leg_panel")
    leg_panel.visual(
        Box((0.035, 0.46, 0.050)),
        origin=Origin(xyz=(0.0175, 0.0, -0.025)),
        material=accent,
        name="leg_panel_shoe",
    )
    leg_panel.visual(
        Box((0.040, 0.46, 0.140)),
        origin=Origin(xyz=(0.055, 0.0, -0.100)),
        material=accent,
        name="leg_panel_link",
    )
    leg_panel.visual(
        leg_panel_pad_mesh,
        origin=Origin(xyz=(0.085, 0.0, -0.300)),
        material=upholstery,
        name="leg_panel_pad",
    )
    leg_panel.inertial = Inertial.from_geometry(
        Box((0.14, 0.56, 0.42)),
        mass=2.0,
        origin=Origin(xyz=(0.075, 0.0, -0.18)),
    )

    model.articulation(
        "pedestal_swivel",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=seat_shell,
        origin=Origin(xyz=(0.0, 0.0, 0.298)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=1.5,
            lower=-math.pi,
            upper=math.pi,
        ),
    )
    model.articulation(
        "backrest_recline",
        ArticulationType.REVOLUTE,
        parent=seat_shell,
        child=backrest,
        origin=Origin(xyz=(-0.375, 0.0, 0.132)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.0,
            lower=0.0,
            upper=0.50,
        ),
    )
    model.articulation(
        "leg_panel_hinge",
        ArticulationType.REVOLUTE,
        parent=seat_shell,
        child=leg_panel,
        origin=Origin(xyz=(0.365, 0.0, 0.045)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.2,
            lower=0.0,
            upper=0.95,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal")
    seat_shell = object_model.get_part("seat_shell")
    backrest = object_model.get_part("backrest")
    leg_panel = object_model.get_part("leg_panel")

    pedestal_swivel = object_model.get_articulation("pedestal_swivel")
    backrest_recline = object_model.get_articulation("backrest_recline")
    leg_panel_hinge = object_model.get_articulation("leg_panel_hinge")

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

    ctx.check(
        "pedestal swivel axis is vertical",
        pedestal_swivel.axis == (0.0, 0.0, 1.0),
        details=f"axis={pedestal_swivel.axis}",
    )
    ctx.check(
        "backrest hinge uses transverse axis",
        backrest_recline.axis in ((0.0, 1.0, 0.0), (0.0, -1.0, 0.0)),
        details=f"axis={backrest_recline.axis}",
    )
    ctx.check(
        "leg panel hinge uses transverse axis",
        leg_panel_hinge.axis in ((0.0, 1.0, 0.0), (0.0, -1.0, 0.0)),
        details=f"axis={leg_panel_hinge.axis}",
    )

    ctx.expect_contact(seat_shell, pedestal, contact_tol=0.003, name="seat sits on pedestal")
    ctx.expect_contact(backrest, seat_shell, contact_tol=0.003, name="backrest is seated on rear hinge beam")
    ctx.expect_contact(leg_panel, seat_shell, contact_tol=0.003, name="leg panel nests under seat front")
    ctx.expect_within(backrest, seat_shell, axes="y", margin=0.12, name="backrest width aligns with seat")
    ctx.expect_within(leg_panel, seat_shell, axes="y", margin=0.08, name="leg panel stays within seat width")

    rest_back_aabb = ctx.part_world_aabb(backrest)
    rest_leg_aabb = ctx.part_world_aabb(leg_panel)
    assert rest_back_aabb is not None
    assert rest_leg_aabb is not None

    rest_back_center = tuple((lo + hi) * 0.5 for lo, hi in zip(rest_back_aabb[0], rest_back_aabb[1]))
    rest_leg_max_x = rest_leg_aabb[1][0]
    rest_leg_min_z = rest_leg_aabb[0][2]

    recline_demo = 0.40
    leg_deploy_demo = 0.78

    with ctx.pose({backrest_recline: recline_demo}):
        reclined_back_aabb = ctx.part_world_aabb(backrest)
        assert reclined_back_aabb is not None
        ctx.check(
            "backrest reclines rearward",
            reclined_back_aabb[0][0] < rest_back_aabb[0][0] - 0.09,
            details=f"rest min x={rest_back_aabb[0][0]:.4f}, reclined min x={reclined_back_aabb[0][0]:.4f}",
        )

    with ctx.pose({leg_panel_hinge: leg_deploy_demo}):
        deployed_leg_aabb = ctx.part_world_aabb(leg_panel)
        assert deployed_leg_aabb is not None
        ctx.check(
            "leg panel deploys forward",
            deployed_leg_aabb[1][0] > rest_leg_max_x + 0.18,
            details=f"rest max x={rest_leg_max_x:.4f}, deployed max x={deployed_leg_aabb[1][0]:.4f}",
        )
        ctx.check(
            "leg panel lifts out of its under-seat stow position",
            deployed_leg_aabb[0][2] > rest_leg_min_z + 0.16,
            details=f"rest min z={rest_leg_min_z:.4f}, deployed min z={deployed_leg_aabb[0][2]:.4f}",
        )

    with ctx.pose({pedestal_swivel: math.pi / 2.0}):
        swiveled_back_aabb = ctx.part_world_aabb(backrest)
        assert swiveled_back_aabb is not None
        swiveled_back_center = tuple(
            (lo + hi) * 0.5 for lo, hi in zip(swiveled_back_aabb[0], swiveled_back_aabb[1])
        )
        ctx.check(
            "pedestal swivel rotates seat assembly about z",
            abs(swiveled_back_center[0]) < 0.10 and swiveled_back_center[1] < -0.12,
            details=(
                f"rest center={rest_back_center}, "
                f"swiveled center={swiveled_back_center}"
            ),
        )

    with ctx.pose({backrest_recline: recline_demo, leg_panel_hinge: leg_deploy_demo}):
        ctx.fail_if_parts_overlap_in_current_pose(name="open lounge pose remains clear")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
