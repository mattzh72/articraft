from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    repair_loft,
    rounded_rect_profile,
    section_loft,
)

ASSETS = AssetContext.from_script(__file__)

HOUSING_WIDTH = 0.285
HOUSING_DEPTH = 0.175
BASE_THICKNESS = 0.008
ANTENNA_X_POSITIONS = (-0.115, -0.038, 0.038, 0.115)
ANTENNA_ROOT_Y = 0.072
ANTENNA_ROOT_Z = 0.044
ANTENNA_TILT_LIMIT = math.radians(60.0)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, ASSETS.mesh_path(name))


def _xz_section(
    *,
    width: float,
    height: float,
    radius: float,
    y_pos: float,
    base_z: float,
    corner_segments: int = 6,
) -> list[tuple[float, float, float]]:
    z_center = base_z + (height * 0.5)
    return [
        (x, y_pos, z_center + z)
        for x, z in rounded_rect_profile(width, height, radius, corner_segments=corner_segments)
    ]


def _xy_section(
    *,
    width: float,
    depth: float,
    radius: float,
    z_pos: float,
    corner_segments: int = 6,
) -> list[tuple[float, float, float]]:
    return [
        (x, y, z_pos)
        for x, y in rounded_rect_profile(width, depth, radius, corner_segments=corner_segments)
    ]


def _build_router_shell_mesh():
    shell_geom = repair_loft(
        section_loft(
            [
                _xz_section(
                    width=0.266,
                    height=0.016,
                    radius=0.008,
                    y_pos=-0.084,
                    base_z=0.0072,
                ),
                _xz_section(
                    width=0.274,
                    height=0.020,
                    radius=0.009,
                    y_pos=-0.020,
                    base_z=0.0072,
                ),
                _xz_section(
                    width=0.280,
                    height=0.025,
                    radius=0.010,
                    y_pos=0.040,
                    base_z=0.0072,
                ),
                _xz_section(
                    width=0.276,
                    height=0.028,
                    radius=0.010,
                    y_pos=0.084,
                    base_z=0.0072,
                ),
            ]
        )
    )
    return _save_mesh("wifi_router_top_shell.obj", shell_geom)


def _build_antenna_blade_mesh():
    blade_geom = repair_loft(
        section_loft(
            [
                _xy_section(width=0.018, depth=0.0062, radius=0.0020, z_pos=0.010),
                _xy_section(width=0.018, depth=0.0060, radius=0.0020, z_pos=0.028),
                _xy_section(width=0.016, depth=0.0056, radius=0.0019, z_pos=0.078),
                _xy_section(width=0.012, depth=0.0046, radius=0.0015, z_pos=0.108),
                _xy_section(width=0.004, depth=0.0020, radius=0.0010, z_pos=0.118),
            ]
        )
    )
    return _save_mesh("wifi_router_antenna_blade.obj", blade_geom)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desktop_wifi_router", assets=ASSETS)

    shell_charcoal = model.material("shell_charcoal", rgba=(0.15, 0.16, 0.18, 1.0))
    shell_graphite = model.material("shell_graphite", rgba=(0.21, 0.22, 0.24, 1.0))
    indicator_smoke = model.material("indicator_smoke", rgba=(0.12, 0.20, 0.14, 0.55))
    antenna_black = model.material("antenna_black", rgba=(0.08, 0.09, 0.10, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.05, 0.05, 0.05, 1.0))

    shell_mesh = _build_router_shell_mesh()
    antenna_blade_mesh = _build_antenna_blade_mesh()

    housing = model.part("housing")
    housing.visual(
        Box((HOUSING_WIDTH, HOUSING_DEPTH, BASE_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS * 0.5)),
        material=shell_charcoal,
        name="bottom_plate",
    )
    housing.visual(shell_mesh, material=shell_graphite, name="top_shell")
    housing.visual(
        Box((0.248, 0.012, 0.009)),
        origin=Origin(xyz=(0.0, 0.079, 0.0305)),
        material=shell_charcoal,
        name="rear_spine",
    )
    housing.visual(
        Box((0.160, 0.004, 0.003)),
        origin=Origin(xyz=(0.0, -0.086, 0.013)),
        material=indicator_smoke,
        name="status_strip",
    )
    for index, x_pos in enumerate(ANTENNA_X_POSITIONS):
        housing.visual(
            Box((0.020, 0.014, 0.008)),
            origin=Origin(xyz=(x_pos, ANTENNA_ROOT_Y, 0.035)),
            material=shell_charcoal,
            name=f"mount_block_{index}",
        )
    housing.inertial = Inertial.from_geometry(
        Box((HOUSING_WIDTH, HOUSING_DEPTH, 0.040)),
        mass=0.95,
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
    )

    foot_positions = {
        "foot_front_left": (-0.103, -0.061),
        "foot_front_right": (0.103, -0.061),
        "foot_rear_left": (-0.103, 0.061),
        "foot_rear_right": (0.103, 0.061),
    }
    for foot_name, (x_pos, y_pos) in foot_positions.items():
        foot = model.part(foot_name)
        foot.visual(
            Cylinder(radius=0.0075, length=0.004),
            origin=Origin(xyz=(0.0, 0.0, -0.002)),
            material=rubber_black,
            name="foot_pad",
        )
        foot.inertial = Inertial.from_geometry(
            Cylinder(radius=0.0075, length=0.004),
            mass=0.008,
            origin=Origin(xyz=(0.0, 0.0, -0.002)),
        )
        model.articulation(
            f"housing_to_{foot_name}",
            ArticulationType.FIXED,
            parent=housing,
            child=foot,
            origin=Origin(xyz=(x_pos, y_pos, 0.0)),
        )

    for index, x_pos in enumerate(ANTENNA_X_POSITIONS):
        antenna = model.part(f"antenna_{index}")
        antenna.visual(
            Cylinder(radius=0.005, length=0.018),
            origin=Origin(rpy=(0.0, math.pi * 0.5, 0.0)),
            material=antenna_black,
            name="hinge_barrel",
        )
        antenna.visual(
            Box((0.014, 0.006, 0.014)),
            origin=Origin(xyz=(0.0, 0.0, 0.007)),
            material=antenna_black,
            name="hinge_bridge",
        )
        antenna.visual(
            antenna_blade_mesh,
            material=antenna_black,
            name="blade",
        )
        antenna.inertial = Inertial.from_geometry(
            Box((0.020, 0.010, 0.122)),
            mass=0.050,
            origin=Origin(xyz=(0.0, 0.0, 0.061)),
        )
        model.articulation(
            f"housing_to_antenna_{index}",
            ArticulationType.REVOLUTE,
            parent=housing,
            child=antenna,
            origin=Origin(xyz=(x_pos, ANTENNA_ROOT_Y, ANTENNA_ROOT_Z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=1.0,
                velocity=1.5,
                lower=-ANTENNA_TILT_LIMIT,
                upper=ANTENNA_TILT_LIMIT,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    housing = object_model.get_part("housing")
    housing_shell = housing.get_visual("top_shell")
    antenna_mounts = [housing.get_visual(f"mount_block_{index}") for index in range(4)]
    feet = [object_model.get_part(name) for name in (
        "foot_front_left",
        "foot_front_right",
        "foot_rear_left",
        "foot_rear_right",
    )]
    antennas = [object_model.get_part(f"antenna_{index}") for index in range(4)]
    hinges = [object_model.get_articulation(f"housing_to_antenna_{index}") for index in range(4)]

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
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
    ctx.fail_if_articulation_overlaps(max_pose_samples=20)

    housing_aabb = ctx.part_world_aabb(housing)
    ctx.check(
        "router_part_count",
        len(object_model.parts) == 9,
        f"Expected 9 parts (housing, 4 feet, 4 antennas), found {len(object_model.parts)}.",
    )
    ctx.check(
        "router_joint_count",
        len(object_model.articulations) == 8,
        f"Expected 8 articulations, found {len(object_model.articulations)}.",
    )
    ctx.check(
        "housing_aabb_available",
        housing_aabb is not None,
        "Housing world AABB could not be evaluated.",
    )
    if housing_aabb is not None:
        housing_size = tuple(housing_aabb[1][axis] - housing_aabb[0][axis] for axis in range(3))
        ctx.check(
            "housing_is_low_rectangle",
            0.26 <= housing_size[0] <= 0.30
            and 0.15 <= housing_size[1] <= 0.19
            and 0.03 <= housing_size[2] <= 0.05
            and housing_size[0] > housing_size[1] > housing_size[2] * 3.0,
            f"Unexpected housing proportions: {housing_size}.",
        )

    antenna_positions = [ctx.part_world_position(antenna) for antenna in antennas]
    ctx.check(
        "antenna_root_positions_available",
        all(position is not None for position in antenna_positions),
        "One or more antenna root positions could not be evaluated.",
    )
    if all(position is not None for position in antenna_positions):
        xs = [position[0] for position in antenna_positions if position is not None]
        ys = [position[1] for position in antenna_positions if position is not None]
        ctx.check(
            "antenna_roots_at_rear_edge",
            min(ys) > 0.06 and max(ys) - min(ys) < 1e-6,
            f"Antenna roots should align along the rear edge, got y positions {ys}.",
        )
        ctx.check(
            "outer_antennas_at_corners",
            xs[0] < -0.10 and xs[3] > 0.10,
            f"Outer antenna x positions should sit near the housing corners, got {xs}.",
        )
        ctx.check(
            "inner_antennas_near_centerline",
            abs(xs[1]) < 0.05 and abs(xs[2]) < 0.05,
            f"Inner antenna x positions should sit near the centerline, got {xs}.",
        )

    for index, foot in enumerate(feet):
        ctx.expect_overlap(
            foot,
            housing,
            axes="xy",
            min_overlap=0.010,
            name=f"foot_{index}_under_base",
        )
        ctx.expect_gap(
            housing,
            foot,
            axis="z",
            max_gap=0.0005,
            max_penetration=0.0,
            name=f"foot_{index}_contacts_underside",
        )

    for index, (antenna, hinge, mount_visual) in enumerate(zip(antennas, hinges, antenna_mounts)):
        hinge_barrel = antenna.get_visual("hinge_barrel")
        blade = antenna.get_visual("blade")

        ctx.expect_overlap(
            antenna,
            housing,
            axes="xy",
            min_overlap=0.009,
            elem_a=hinge_barrel,
            elem_b=mount_visual,
            name=f"antenna_{index}_hinge_aligned_to_mount",
        )
        ctx.expect_gap(
            antenna,
            housing,
            axis="z",
            max_gap=0.0005,
            max_penetration=1e-6,
            positive_elem=hinge_barrel,
            negative_elem=mount_visual,
            name=f"antenna_{index}_hinge_seated_on_mount",
        )
        ctx.expect_gap(
            antenna,
            housing,
            axis="z",
            min_gap=0.015,
            positive_elem=blade,
            negative_elem=housing_shell,
            name=f"antenna_{index}_blade_clears_housing_at_rest",
        )

        axis = tuple(round(value, 6) for value in hinge.axis)
        limits = hinge.motion_limits
        ctx.check(
            f"antenna_{index}_hinge_axis_is_x",
            axis == (1.0, 0.0, 0.0),
            f"Antenna hinge axis should be +X, got {hinge.axis}.",
        )
        ctx.check(
            f"antenna_{index}_hinge_limits_span_120_deg",
            limits is not None
            and abs(limits.lower + ANTENNA_TILT_LIMIT) < 1e-6
            and abs(limits.upper - ANTENNA_TILT_LIMIT) < 1e-6,
            f"Antenna hinge limits should be ±60 degrees, got {limits}.",
        )

        rest_blade_aabb = ctx.part_element_world_aabb(antenna, elem="blade")
        ctx.check(
            f"antenna_{index}_rest_blade_aabb_available",
            rest_blade_aabb is not None,
            "Could not measure antenna blade AABB in rest pose.",
        )
        if rest_blade_aabb is not None:
            rest_center_y = 0.5 * (rest_blade_aabb[0][1] + rest_blade_aabb[1][1])
            rest_max_z = rest_blade_aabb[1][2]
            with ctx.pose({hinge: math.radians(55.0)}):
                posed_blade_aabb = ctx.part_element_world_aabb(antenna, elem="blade")
                ctx.check(
                    f"antenna_{index}_posed_blade_aabb_available",
                    posed_blade_aabb is not None,
                    "Could not measure antenna blade AABB in tilted pose.",
                )
                if posed_blade_aabb is not None:
                    posed_center_y = 0.5 * (posed_blade_aabb[0][1] + posed_blade_aabb[1][1])
                    ctx.check(
                        f"antenna_{index}_tilts_in_vertical_plane",
                        abs(posed_center_y - rest_center_y) > 0.025
                        and posed_blade_aabb[1][2] < rest_max_z - 0.010,
                        (
                            f"Antenna {index} did not tilt convincingly; "
                            f"rest center/max z {(rest_center_y, rest_max_z)} vs "
                            f"posed {(posed_center_y, posed_blade_aabb[1][2])}."
                        ),
                    )
                ctx.expect_gap(
                    antenna,
                    housing,
                    axis="z",
                    max_gap=0.0005,
                    max_penetration=1e-6,
                    positive_elem=hinge_barrel,
                    negative_elem=mount_visual,
                    name=f"antenna_{index}_hinge_stays_seated_when_tilted",
                )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
