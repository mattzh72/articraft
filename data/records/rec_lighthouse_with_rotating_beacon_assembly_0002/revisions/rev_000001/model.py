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
    DomeGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    ASSETS.mesh_dir.mkdir(parents=True, exist_ok=True)

    def save_mesh(name: str, geometry):
        return mesh_from_geometry(geometry, ASSETS.mesh_path(name))

    def ring_shell_mesh(
        name: str,
        *,
        outer_radius: float,
        inner_radius: float,
        lower_z: float,
        upper_z: float,
        segments: int = 48,
    ):
        return save_mesh(
            name,
            LatheGeometry.from_shell_profiles(
                [(outer_radius, lower_z), (outer_radius, upper_z)],
                [(inner_radius, lower_z), (inner_radius, upper_z)],
                segments=segments,
                start_cap="flat",
                end_cap="flat",
            ),
        )

    model = ArticulatedObject(name="lighthouse_lantern", assets=ASSETS)

    white_paint = model.material("white_paint", rgba=(0.93, 0.94, 0.92, 1.0))
    lantern_red = model.material("lantern_red", rgba=(0.63, 0.10, 0.12, 1.0))
    dark_iron = model.material("dark_iron", rgba=(0.19, 0.20, 0.22, 1.0))
    steel = model.material("steel", rgba=(0.58, 0.60, 0.62, 1.0))
    brass = model.material("brass", rgba=(0.73, 0.58, 0.28, 1.0))
    reflector_metal = model.material("reflector_metal", rgba=(0.85, 0.83, 0.77, 1.0))
    glass = model.material("glass", rgba=(0.78, 0.89, 0.95, 0.30))
    lamp_glow = model.material("lamp_glow", rgba=(0.98, 0.88, 0.56, 0.85))

    tower_shell_mesh = save_mesh(
        "tower_top_shell.obj",
        LatheGeometry(
            [
                (0.0, 0.0),
                (0.235, 0.0),
                (0.228, 0.032),
                (0.210, 0.110),
                (0.178, 0.205),
                (0.160, 0.240),
                (0.0, 0.240),
            ],
            segments=72,
        ),
    )
    glass_shell_mesh = ring_shell_mesh(
        "lantern_glass_shell.obj",
        outer_radius=0.122,
        inner_radius=0.118,
        lower_z=0.0,
        upper_z=0.242,
        segments=72,
    )
    bearing_collar_mesh = ring_shell_mesh(
        "beacon_bearing_collar.obj",
        outer_radius=0.028,
        inner_radius=0.0215,
        lower_z=-0.048,
        upper_z=0.048,
        segments=64,
    )
    reflector_shell_mesh = save_mesh(
        "beacon_reflector_shell.obj",
        LatheGeometry.from_shell_profiles(
            [
                (0.009, -0.026),
                (0.016, -0.025),
                (0.026, -0.021),
                (0.034, -0.012),
                (0.039, -0.004),
                (0.041, 0.000),
            ],
            [
                (0.000, -0.022),
                (0.008, -0.022),
                (0.015, -0.019),
                (0.030, -0.011),
                (0.035, -0.004),
                (0.037, -0.001),
            ],
            segments=64,
            start_cap="flat",
            end_cap="flat",
        ),
    )
    roof_dome_mesh = save_mesh(
        "lantern_roof_dome.obj",
        DomeGeometry(radius=(0.142, 0.142, 0.075), radial_segments=56, height_segments=24, closed=True),
    )
    vent_cap_mesh = save_mesh(
        "lantern_vent_cap.obj",
        DomeGeometry(radius=(0.046, 0.046, 0.026), radial_segments=40, height_segments=18, closed=True),
    )

    tower_top = model.part("tower_top")
    tower_top.visual(tower_shell_mesh, material=white_paint, name="tower_shell")
    tower_top.visual(
        Cylinder(radius=0.188, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.255)),
        material=dark_iron,
        name="gallery_deck",
    )
    tower_top.visual(
        Cylinder(radius=0.135, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.282)),
        material=dark_iron,
        name="lantern_floor",
    )
    for index in range(8):
        angle = (2.0 * math.pi * index) / 8.0
        tower_top.visual(
            Box((0.018, 0.028, 0.014)),
            origin=Origin(
                xyz=(0.118 * math.cos(angle), 0.118 * math.sin(angle), 0.301),
                rpy=(0.0, 0.0, angle),
            ),
            material=lantern_red,
            name=f"base_curb_segment_{index:02d}",
        )
    tower_top.visual(
        glass_shell_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.308)),
        material=glass,
        name="glass_enclosure",
    )
    for index in range(8):
        angle = (2.0 * math.pi * index) / 8.0
        radius = 0.118
        tower_top.visual(
            Box((0.010, 0.014, 0.242)),
            origin=Origin(
                xyz=(radius * math.cos(angle), radius * math.sin(angle), 0.429),
                rpy=(0.0, 0.0, angle),
            ),
            material=lantern_red,
            name=f"mullion_{index:02d}",
        )
    tower_top.visual(
        Cylinder(radius=0.144, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.561)),
        material=lantern_red,
        name="roof_skirt",
    )
    tower_top.visual(
        roof_dome_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.572)),
        material=lantern_red,
        name="roof_dome",
    )
    tower_top.visual(
        Cylinder(radius=0.028, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.672)),
        material=dark_iron,
        name="vent_stack",
    )
    tower_top.visual(
        vent_cap_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.697)),
        material=dark_iron,
        name="vent_cap",
    )
    tower_top.inertial = Inertial.from_geometry(
        Cylinder(radius=0.235, length=0.723),
        mass=22.0,
        origin=Origin(xyz=(0.0, 0.0, 0.3615)),
    )

    central_shaft = model.part("central_shaft")
    central_shaft.visual(
        Cylinder(radius=0.045, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=dark_iron,
        name="shaft_base",
    )
    central_shaft.visual(
        Cylinder(radius=0.022, length=0.237),
        origin=Origin(xyz=(0.0, 0.0, 0.1365)),
        material=steel,
        name="shaft_main",
    )
    central_shaft.inertial = Inertial.from_geometry(
        Cylinder(radius=0.045, length=0.255),
        mass=2.2,
        origin=Origin(xyz=(0.0, 0.0, 0.1275)),
    )

    beacon_carriage = model.part("beacon_carriage")
    beacon_carriage.visual(
        Box((0.012, 0.042, 0.100)),
        origin=Origin(xyz=(0.028, 0.0, 0.000)),
        material=brass,
        name="bearing_collar",
    )
    beacon_carriage.visual(
        Box((0.030, 0.042, 0.058)),
        origin=Origin(xyz=(0.049, 0.0, 0.000)),
        material=dark_iron,
        name="drive_box",
    )
    beacon_carriage.visual(
        Box((0.040, 0.048, 0.010)),
        origin=Origin(xyz=(0.068, 0.0, -0.036)),
        material=dark_iron,
        name="lower_platform",
    )
    beacon_carriage.visual(
        Box((0.040, 0.010, 0.010)),
        origin=Origin(xyz=(0.072, 0.0, -0.024)),
        material=brass,
        name="lower_frame_rail",
    )
    beacon_carriage.visual(
        Box((0.040, 0.010, 0.010)),
        origin=Origin(xyz=(0.072, 0.0, 0.024)),
        material=brass,
        name="upper_frame_rail",
    )
    beacon_carriage.visual(
        Box((0.036, 0.010, 0.010)),
        origin=Origin(xyz=(0.074, 0.014, 0.000)),
        material=brass,
        name="support_arm_port",
    )
    beacon_carriage.visual(
        Box((0.036, 0.010, 0.010)),
        origin=Origin(xyz=(0.074, -0.014, 0.000)),
        material=brass,
        name="support_arm_starboard",
    )
    beacon_carriage.visual(
        Box((0.010, 0.042, 0.062)),
        origin=Origin(xyz=(0.092, 0.0, 0.000)),
        material=brass,
        name="front_frame",
    )
    beacon_carriage.visual(
        reflector_shell_mesh,
        origin=Origin(xyz=(0.094, 0.0, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=reflector_metal,
        name="reflector_shell",
    )
    beacon_carriage.visual(
        Cylinder(radius=0.011, length=0.038),
        origin=Origin(xyz=(0.097, 0.0, 0.000), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_iron,
        name="lamp_housing",
    )
    beacon_carriage.visual(
        Sphere(radius=0.008),
        origin=Origin(xyz=(0.097, 0.0, 0.000)),
        material=lamp_glow,
        name="lamp_bulb",
    )
    beacon_carriage.inertial = Inertial.from_geometry(
        Box((0.118, 0.072, 0.112)),
        mass=1.3,
        origin=Origin(xyz=(0.060, 0.0, 0.0)),
    )

    model.articulation(
        "tower_to_shaft",
        ArticulationType.FIXED,
        parent=tower_top,
        child=central_shaft,
        origin=Origin(xyz=(0.0, 0.0, 0.294)),
    )
    model.articulation(
        "beacon_rotation",
        ArticulationType.CONTINUOUS,
        parent=central_shaft,
        child=beacon_carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.130)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.6),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    tower_top = object_model.get_part("tower_top")
    central_shaft = object_model.get_part("central_shaft")
    beacon_carriage = object_model.get_part("beacon_carriage")
    beacon_rotation = object_model.get_articulation("beacon_rotation")

    lantern_floor = tower_top.get_visual("lantern_floor")
    glass_enclosure = tower_top.get_visual("glass_enclosure")
    roof_skirt = tower_top.get_visual("roof_skirt")
    shaft_base = central_shaft.get_visual("shaft_base")
    shaft_main = central_shaft.get_visual("shaft_main")
    bearing_collar = beacon_carriage.get_visual("bearing_collar")
    lower_platform = beacon_carriage.get_visual("lower_platform")
    upper_frame_rail = beacon_carriage.get_visual("upper_frame_rail")
    lamp_housing = beacon_carriage.get_visual("lamp_housing")

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
    ctx.fail_if_parts_overlap_in_sampled_poses(max_pose_samples=12, ignore_adjacent=False, ignore_fixed=False)

    ctx.expect_origin_distance(central_shaft, tower_top, axes="xy", max_dist=0.001)
    ctx.expect_origin_distance(beacon_carriage, central_shaft, axes="xy", max_dist=0.001)
    ctx.expect_contact(
        central_shaft,
        tower_top,
        elem_a=shaft_base,
        elem_b=lantern_floor,
        contact_tol=1e-4,
        name="shaft_base_seats_on_lantern_floor",
    )
    ctx.expect_contact(
        beacon_carriage,
        central_shaft,
        elem_a=bearing_collar,
        elem_b=shaft_main,
        contact_tol=1e-4,
        name="bearing_collar_contacts_shaft",
    )
    ctx.expect_overlap(
        beacon_carriage,
        central_shaft,
        axes="yz",
        elem_a=bearing_collar,
        elem_b=shaft_main,
        min_overlap=0.03,
        name="bearing_collar_stays_aligned_with_shaft",
    )
    ctx.expect_within(
        beacon_carriage,
        tower_top,
        axes="xy",
        outer_elem=glass_enclosure,
        margin=0.0,
        name="carriage_stays_within_lantern_glass_footprint",
    )
    ctx.expect_gap(
        beacon_carriage,
        tower_top,
        axis="z",
        positive_elem=lower_platform,
        negative_elem=lantern_floor,
        min_gap=0.055,
        name="carriage_clears_lantern_floor",
    )
    ctx.expect_gap(
        tower_top,
        beacon_carriage,
        axis="z",
        positive_elem=roof_skirt,
        negative_elem=upper_frame_rail,
        min_gap=0.055,
        name="carriage_clears_roof_skirt",
    )

    glass_aabb = ctx.part_element_world_aabb(tower_top, elem=glass_enclosure)
    if glass_aabb is None:
        ctx.fail("glass_enclosure_measurable", "Lantern glass enclosure AABB unavailable.")
    else:
        glass_height = glass_aabb[1][2] - glass_aabb[0][2]
        glass_diameter_x = glass_aabb[1][0] - glass_aabb[0][0]
        ctx.check(
            "glass_enclosure_proportions",
            0.235 <= glass_height <= 0.249 and 0.240 <= glass_diameter_x <= 0.248,
            details=(
                f"glass height={glass_height:.4f} m diameter_x={glass_diameter_x:.4f} m "
                "outside expected lighthouse lantern proportions"
            ),
        )

    lamp_rest = ctx.part_element_world_aabb(beacon_carriage, elem=lamp_housing)
    if lamp_rest is None:
        ctx.fail("lamp_housing_measurable_rest", "Lamp housing AABB unavailable at rest pose.")
    else:
        lamp_rest_center = tuple((lamp_rest[0][i] + lamp_rest[1][i]) * 0.5 for i in range(3))
        ctx.check(
            "lamp_starts_forward_of_axis",
            lamp_rest_center[0] > 0.06 and abs(lamp_rest_center[1]) < 0.01,
            details=f"rest lamp center={lamp_rest_center!r}",
        )

        with ctx.pose({beacon_rotation: math.pi / 2.0}):
            ctx.expect_within(
                beacon_carriage,
                tower_top,
                axes="xy",
                outer_elem=glass_enclosure,
                margin=0.0,
                name="quarter_turn_carriage_within_glass",
            )
            lamp_quarter = ctx.part_element_world_aabb(beacon_carriage, elem=lamp_housing)
            if lamp_quarter is None:
                ctx.fail(
                    "lamp_housing_measurable_quarter_turn",
                    "Lamp housing AABB unavailable at quarter-turn pose.",
                )
            else:
                lamp_quarter_center = tuple((lamp_quarter[0][i] + lamp_quarter[1][i]) * 0.5 for i in range(3))
                ctx.check(
                    "lamp_rotates_to_port_at_quarter_turn",
                    lamp_quarter_center[1] < -0.06 and abs(lamp_quarter_center[0]) < 0.015,
                    details=f"quarter-turn lamp center={lamp_quarter_center!r}",
                )

        with ctx.pose({beacon_rotation: math.pi}):
            ctx.expect_within(
                beacon_carriage,
                tower_top,
                axes="xy",
                outer_elem=glass_enclosure,
                margin=0.0,
                name="half_turn_carriage_within_glass",
            )
            lamp_half = ctx.part_element_world_aabb(beacon_carriage, elem=lamp_housing)
            if lamp_half is None:
                ctx.fail("lamp_housing_measurable_half_turn", "Lamp housing AABB unavailable at half-turn pose.")
            else:
                lamp_half_center = tuple((lamp_half[0][i] + lamp_half[1][i]) * 0.5 for i in range(3))
                ctx.check(
                    "lamp_swings_aft_at_half_turn",
                    lamp_half_center[0] < -0.06 and abs(lamp_half_center[1]) < 0.01,
                    details=f"half-turn lamp center={lamp_half_center!r}",
                )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
