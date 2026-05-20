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
    repair_loft,
    section_loft,
)


TRACK_TOP_Z = 1.20
DOME_RADIUS = 2.42
DOME_THICKNESS = 0.08
SLIT_HALF_ANGLE = 0.18
SHUTTER_HALF_ANGLE = 0.22


UPPER_SHELL_PROFILE: list[tuple[float, float]] = [
    (0.56, 2.42),
    (1.05, 2.39),
    (1.55, 2.24),
    (2.05, 1.92),
    (2.45, 1.45),
    (2.80, 0.92),
    (2.95, 0.72),
]


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _interp_radius(z: float, profile: list[tuple[float, float]]) -> float:
    if z <= profile[0][0]:
        return profile[0][1]
    if z >= profile[-1][0]:
        return profile[-1][1]
    for (z0, r0), (z1, r1) in zip(profile, profile[1:]):
        if z0 <= z <= z1:
            if abs(z1 - z0) < 1e-9:
                return r1
            t = (z - z0) / (z1 - z0)
            return (1.0 - t) * r0 + t * r1
    return profile[-1][1]


def _arc_shell_loop(
    *,
    z: float,
    angle_start: float,
    angle_end: float,
    outer_radius: float,
    inner_radius: float,
    samples: int = 40,
) -> list[tuple[float, float, float]]:
    outer: list[tuple[float, float, float]] = []
    inner: list[tuple[float, float, float]] = []
    for index in range(samples):
        t = index / (samples - 1)
        angle = angle_start + (angle_end - angle_start) * t
        outer.append((outer_radius * math.cos(angle), outer_radius * math.sin(angle), z))
    for index in range(samples - 1, -1, -1):
        t = index / (samples - 1)
        angle = angle_start + (angle_end - angle_start) * t
        inner.append((inner_radius * math.cos(angle), inner_radius * math.sin(angle), z))
    return outer + inner


def _sector_shell_mesh(
    *,
    z_samples: list[float],
    angle_start: float,
    angle_end: float,
    profile: list[tuple[float, float]],
    thickness: float,
    outer_offset: float = 0.0,
    inner_offset: float = 0.0,
    samples: int = 40,
):
    sections = []
    for z in z_samples:
        outer_radius = _interp_radius(z, profile) + outer_offset
        inner_radius = max(0.05, outer_radius - thickness - inner_offset)
        sections.append(
            _arc_shell_loop(
                z=z,
                angle_start=angle_start,
                angle_end=angle_end,
                outer_radius=outer_radius,
                inner_radius=inner_radius,
                samples=samples,
            )
        )
    return repair_loft(section_loft(sections), repair="mesh")


def _add_track_module(part, *, angle: float, steel) -> None:
    c = math.cos(angle)
    s = math.sin(angle)
    pedestal_radius = 2.18
    roller_radius = 2.47
    part.visual(
        Box((0.26, 0.22, 0.18)),
        origin=Origin(
            xyz=(pedestal_radius * c, pedestal_radius * s, TRACK_TOP_Z + 0.09),
            rpy=(0.0, 0.0, angle),
        ),
        material=steel,
        name=f"roller_pedestal_{int(round(angle * 1000)):04d}",
    )
    part.visual(
        Box((0.30, 0.10, 0.04)),
        origin=Origin(
            xyz=(2.30 * c, 2.30 * s, TRACK_TOP_Z - 0.02),
            rpy=(0.0, 0.0, angle),
        ),
        material=steel,
        name=f"roller_arm_{int(round(angle * 1000)):04d}",
    )
    part.visual(
        Box((0.08, 0.10, 0.18)),
        origin=Origin(
            xyz=(2.46 * c, 2.46 * s, TRACK_TOP_Z + 0.09),
            rpy=(0.0, 0.0, angle),
        ),
        material=steel,
        name=f"roller_post_{int(round(angle * 1000)):04d}",
    )
    part.visual(
        Cylinder(radius=0.05, length=0.16),
        origin=Origin(xyz=(roller_radius * c, roller_radius * s, TRACK_TOP_Z + 0.26)),
        material=steel,
        name=f"guide_roller_{int(round(angle * 1000)):04d}",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="service_observatory_dome")

    concrete = model.material("concrete", rgba=(0.62, 0.63, 0.64, 1.0))
    observatory_white = model.material("observatory_white", rgba=(0.89, 0.91, 0.93, 1.0))
    machinery_gray = model.material("machinery_gray", rgba=(0.38, 0.41, 0.44, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.20, 0.22, 0.24, 1.0))
    wear_steel = model.material("wear_steel", rgba=(0.55, 0.57, 0.60, 1.0))
    seal_black = model.material("seal_black", rgba=(0.10, 0.10, 0.11, 1.0))

    base = model.part("base_stationary")
    base.visual(
        Cylinder(radius=2.72, length=0.46),
        origin=Origin(xyz=(0.0, 0.0, 0.23)),
        material=concrete,
        name="foundation_ring",
    )
    base.visual(
        Cylinder(radius=2.50, length=0.70),
        origin=Origin(xyz=(0.0, 0.0, 0.81)),
        material=machinery_gray,
        name="service_drum",
    )
    base.visual(
        Cylinder(radius=2.38, length=0.05),
        origin=Origin(xyz=(0.0, 0.0, TRACK_TOP_Z - 0.025)),
        material=wear_steel,
        name="track_ring",
    )
    base.visual(
        Box((0.84, 0.72, 0.72)),
        origin=Origin(xyz=(-1.88, 0.0, 0.82)),
        material=machinery_gray,
        name="drive_service_cabinet",
    )
    base.visual(
        Box((0.52, 0.10, 0.24)),
        origin=Origin(xyz=(-2.31, 0.0, 0.88)),
        material=dark_steel,
        name="cabinet_door",
    )
    base.visual(
        Box((0.48, 0.18, 0.10)),
        origin=Origin(xyz=(2.16, 0.0, 1.15)),
        material=dark_steel,
        name="maintenance_step",
    )
    for index in range(8):
        _add_track_module(base, angle=index * math.tau / 8.0, steel=wear_steel)
    base.inertial = Inertial.from_geometry(
        Cylinder(radius=2.72, length=1.25),
        mass=9800.0,
        origin=Origin(xyz=(0.0, 0.0, 0.625)),
    )

    dome = model.part("dome_shell")
    lower_band_mesh = LatheGeometry.from_shell_profiles(
        [(DOME_RADIUS, 0.0), (DOME_RADIUS, 0.60)],
        [(DOME_RADIUS - DOME_THICKNESS, 0.0), (DOME_RADIUS - DOME_THICKNESS, 0.60)],
        segments=88,
        start_cap="flat",
        end_cap="flat",
    )
    dome.visual(
        _mesh("dome_lower_band", lower_band_mesh),
        material=wear_steel,
        name="wear_band",
    )
    dome.visual(
        _mesh(
            "dome_upper_shell",
            _sector_shell_mesh(
                z_samples=[0.56, 0.95, 1.45, 1.95, 2.40, 2.75, 2.95],
                angle_start=SLIT_HALF_ANGLE,
                angle_end=(2.0 * math.pi) - SLIT_HALF_ANGLE,
                profile=UPPER_SHELL_PROFILE,
                thickness=DOME_THICKNESS,
                samples=44,
            ),
        ),
        material=observatory_white,
        name="upper_shell",
    )
    crown_cap_mesh = LatheGeometry.from_shell_profiles(
        [(0.82, 2.84), (0.58, 3.02), (0.26, 3.18), (0.0, 3.28)],
        [(0.72, 2.84), (0.48, 3.02), (0.18, 3.18), (0.0, 3.28)],
        segments=72,
        start_cap="flat",
        end_cap="round",
        lip_samples=8,
    )
    dome.visual(
        _mesh("dome_crown_cap", crown_cap_mesh),
        material=observatory_white,
        name="crown_cap",
    )
    dome.visual(
        Box((0.18, 1.02, 0.12)),
        origin=Origin(xyz=(2.33, 0.0, 0.62)),
        material=dark_steel,
        name="slit_sill",
    )
    dome.visual(
        Box((0.10, 0.10, 2.22)),
        origin=Origin(xyz=(2.31, 0.49, 1.73)),
        material=dark_steel,
        name="left_jamb",
    )
    dome.visual(
        Box((0.10, 0.10, 2.22)),
        origin=Origin(xyz=(2.31, -0.49, 1.73)),
        material=dark_steel,
        name="right_jamb",
    )
    dome.visual(
        Box((0.08, 0.12, 3.95)),
        origin=Origin(xyz=(2.40, 0.62, 2.525)),
        material=machinery_gray,
        name="left_guide_rail",
    )
    dome.visual(
        Box((0.08, 0.12, 3.95)),
        origin=Origin(xyz=(2.40, -0.62, 2.525)),
        material=machinery_gray,
        name="right_guide_rail",
    )
    for side_sign, prefix in ((1.0, "left"), (-1.0, "right")):
        rail_y = side_sign * 0.62
        dome.visual(
            Box((0.18, 0.16, 0.14)),
            origin=Origin(xyz=(2.31, rail_y, 0.98)),
            material=dark_steel,
            name=f"{prefix}_rail_standoff_low",
        )
        dome.visual(
            Box((0.18, 0.16, 0.14)),
            origin=Origin(xyz=(2.20, rail_y, 1.68)),
            material=dark_steel,
            name=f"{prefix}_rail_standoff_mid",
        )
        dome.visual(
            Box((0.38, 0.16, 0.16)),
            origin=Origin(xyz=(2.18, rail_y, 2.42)),
            material=dark_steel,
            name=f"{prefix}_rail_standoff_high",
        )
    dome.visual(
        Box((0.10, 1.36, 0.14)),
        origin=Origin(xyz=(2.38, 0.0, 3.55)),
        material=machinery_gray,
        name="rail_top_bridge",
    )
    dome.visual(
        Box((0.16, 1.20, 0.10)),
        origin=Origin(xyz=(2.06, 0.0, 2.42)),
        material=dark_steel,
        name="crown_service_bridge",
    )
    dome.inertial = Inertial.from_geometry(
        Cylinder(radius=2.45, length=3.30),
        mass=2600.0,
        origin=Origin(xyz=(0.0, 0.0, 1.65)),
    )

    shutter = model.part("slit_shutter")
    shutter.visual(
        _mesh(
            "slit_shutter_panel",
            _sector_shell_mesh(
                z_samples=[0.62, 1.20, 1.85, 2.45, 3.02],
                angle_start=-SHUTTER_HALF_ANGLE,
                angle_end=SHUTTER_HALF_ANGLE,
                profile=[
                    (0.62, 2.58),
                    (1.20, 2.58),
                    (1.85, 2.58),
                    (2.45, 2.56),
                    (3.02, 2.53),
                ],
                thickness=0.05,
                samples=32,
            ),
        ),
        material=observatory_white,
        name="panel_skin",
    )
    shutter.visual(
        Box((0.10, 0.14, 2.80)),
        origin=Origin(xyz=(2.49, 0.62, 1.92)),
        material=machinery_gray,
        name="left_carriage",
    )
    shutter.visual(
        Box((0.10, 0.14, 2.80)),
        origin=Origin(xyz=(2.49, -0.62, 1.92)),
        material=machinery_gray,
        name="right_carriage",
    )
    shutter.visual(
        Box((0.08, 0.10, 2.26)),
        origin=Origin(xyz=(2.53, 0.59, 1.82)),
        material=dark_steel,
        name="left_panel_rib",
    )
    shutter.visual(
        Box((0.08, 0.10, 2.26)),
        origin=Origin(xyz=(2.53, -0.59, 1.82)),
        material=dark_steel,
        name="right_panel_rib",
    )
    shutter.visual(
        Box((0.10, 1.30, 0.16)),
        origin=Origin(xyz=(2.49, 0.0, 3.10)),
        material=machinery_gray,
        name="top_storage_beam",
    )
    shutter.visual(
        Box((0.10, 1.20, 0.12)),
        origin=Origin(xyz=(2.50, 0.0, 0.74)),
        material=seal_black,
        name="bottom_seal_beam",
    )
    shutter.visual(
        Cylinder(radius=0.03, length=0.12),
        origin=Origin(xyz=(2.50, 0.62, 1.10)),
        material=wear_steel,
        name="left_lower_roller",
    )
    shutter.visual(
        Cylinder(radius=0.03, length=0.12),
        origin=Origin(xyz=(2.50, 0.62, 2.70)),
        material=wear_steel,
        name="left_upper_roller",
    )
    shutter.visual(
        Cylinder(radius=0.03, length=0.12),
        origin=Origin(xyz=(2.50, -0.62, 1.10)),
        material=wear_steel,
        name="right_lower_roller",
    )
    shutter.visual(
        Cylinder(radius=0.03, length=0.12),
        origin=Origin(xyz=(2.50, -0.62, 2.70)),
        material=wear_steel,
        name="right_upper_roller",
    )
    shutter.inertial = Inertial.from_geometry(
        Box((0.34, 1.40, 3.10)),
        mass=420.0,
        origin=Origin(xyz=(2.50, 0.0, 1.60)),
    )

    dome_rotation = model.articulation(
        "dome_rotation",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=dome,
        origin=Origin(xyz=(0.0, 0.0, TRACK_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=6500.0, velocity=0.28),
    )
    model.articulation(
        "shutter_travel",
        ArticulationType.PRISMATIC,
        parent=dome,
        child=shutter,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1800.0, velocity=0.18, lower=0.0, upper=1.20),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_stationary")
    dome = object_model.get_part("dome_shell")
    shutter = object_model.get_part("slit_shutter")
    dome_rotation = object_model.get_articulation("dome_rotation")
    shutter_travel = object_model.get_articulation("shutter_travel")

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

    ctx.expect_contact(
        base,
        dome,
        elem_a="track_ring",
        elem_b="wear_band",
        name="dome wear band sits on track ring",
    )
    ctx.expect_overlap(
        base,
        dome,
        axes="xy",
        min_overlap=4.0,
        elem_a="track_ring",
        elem_b="wear_band",
        name="dome centered over rotating track",
    )

    with ctx.pose({shutter_travel: 0.0}):
        ctx.expect_contact(
            shutter,
            dome,
            elem_a="left_carriage",
            elem_b="left_guide_rail",
            name="left shutter carriage rides left guide rail",
        )
        ctx.expect_contact(
            shutter,
            dome,
            elem_a="right_carriage",
            elem_b="right_guide_rail",
            name="right shutter carriage rides right guide rail",
        )
        ctx.expect_gap(
            shutter,
            dome,
            axis="z",
            positive_elem="bottom_seal_beam",
            negative_elem="slit_sill",
            min_gap=0.0,
            max_gap=0.16,
            max_penetration=0.0,
            name="closed shutter lands near slit sill",
        )

    with ctx.pose({shutter_travel: 1.20}):
        ctx.expect_contact(
            shutter,
            dome,
            elem_a="left_carriage",
            elem_b="left_guide_rail",
            name="left carriage stays guided when shutter is raised",
        )
        ctx.expect_contact(
            shutter,
            dome,
            elem_a="right_carriage",
            elem_b="right_guide_rail",
            name="right carriage stays guided when shutter is raised",
        )
        ctx.expect_origin_gap(
            shutter,
            dome,
            axis="z",
            min_gap=1.0,
            name="open shutter lifts clear of slit",
        )

    with ctx.pose({dome_rotation: 1.1}):
        ctx.expect_contact(
            base,
            dome,
            elem_a="track_ring",
            elem_b="wear_band",
            name="rotation keeps dome seated on ring track",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
