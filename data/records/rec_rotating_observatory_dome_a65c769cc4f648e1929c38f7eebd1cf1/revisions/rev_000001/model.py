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
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    section_loft,
)


DOME_OUTER_PROFILE = [
    (2.68, 0.00),
    (2.68, 0.52),
    (2.66, 0.80),
    (2.56, 1.20),
    (2.36, 1.68),
    (2.02, 2.16),
    (1.50, 2.62),
    (0.86, 2.92),
    (0.18, 3.10),
]

DOME_INNER_PROFILE = [
    (2.59, 0.02),
    (2.59, 0.52),
    (2.57, 0.78),
    (2.48, 1.16),
    (2.29, 1.62),
    (1.96, 2.08),
    (1.45, 2.53),
    (0.82, 2.82),
    (0.10, 2.96),
]

SLIT_BOTTOM_Z = 1.55
HINGE_Z = 2.92


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _radial_profile_radius(profile: list[tuple[float, float]], z: float) -> float:
    if z <= profile[0][1]:
        return profile[0][0]
    for (r0, z0), (r1, z1) in zip(profile, profile[1:]):
        if z0 <= z <= z1:
            if abs(z1 - z0) < 1e-9:
                return r1
            t = (z - z0) / (z1 - z0)
            return r0 + (r1 - r0) * t
    return profile[-1][0]


def _annulus(outer_radius: float, inner_radius: float, z0: float, z1: float, *, segments: int = 72):
    return LatheGeometry.from_shell_profiles(
        [(outer_radius, z0), (outer_radius, z1)],
        [(inner_radius, z0), (inner_radius, z1)],
        segments=segments,
        start_cap="flat",
        end_cap="flat",
    )


def _slit_half_angle(z: float) -> float:
    crown_t = min(max((z - 1.10) / 2.00, 0.0), 1.0)
    return 0.20 + (0.42 * crown_t * crown_t)


def _shell_section(z: float, *, segments: int = 44) -> list[tuple[float, float, float]]:
    outer_radius = _radial_profile_radius(DOME_OUTER_PROFILE, z)
    inner_radius = _radial_profile_radius(DOME_INNER_PROFILE, z)
    slit_angle = _slit_half_angle(z)
    sweep = (2.0 * math.pi) - (2.0 * slit_angle)

    outer_loop = []
    for step in range(segments + 1):
        angle = slit_angle + (sweep * step / segments)
        outer_loop.append((outer_radius * math.cos(angle), outer_radius * math.sin(angle), z))

    inner_loop = []
    for step in range(segments, -1, -1):
        angle = slit_angle + (sweep * step / segments)
        inner_loop.append((inner_radius * math.cos(angle), inner_radius * math.sin(angle), z))

    return outer_loop + inner_loop


def _build_dome_shell():
    shell_sections = [
        _shell_section(0.18),
        _shell_section(0.52),
        _shell_section(0.80),
        _shell_section(1.20),
        _shell_section(1.68),
        _shell_section(2.16),
        _shell_section(2.62),
        _shell_section(2.82),
        _shell_section(2.92),
        _shell_section(3.02),
        _shell_section(3.10),
    ]
    return section_loft(shell_sections)


def _build_shutter_leaf():
    sections = []
    for x_pos, width, thickness, crown_rise in (
        (0.00, 0.82, 0.060, 0.000),
        (0.24, 0.80, 0.056, 0.060),
        (0.50, 0.70, 0.050, 0.135),
        (0.78, 0.46, 0.038, 0.220),
    ):
        z0 = crown_rise - (0.5 * thickness)
        z1 = crown_rise + (0.5 * thickness)
        half_width = 0.5 * width
        sections.append(
            [
                (x_pos, -half_width, z0),
                (x_pos, half_width, z0),
                (x_pos, half_width, z1),
                (x_pos, -half_width, z1),
            ]
        )
    return section_loft(sections)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="observatory_dome")

    concrete = model.material("concrete", rgba=(0.73, 0.74, 0.76, 1.0))
    rail_steel = model.material("rail_steel", rgba=(0.28, 0.31, 0.35, 1.0))
    shell_white = model.material("shell_white", rgba=(0.92, 0.93, 0.95, 1.0))
    shutter_white = model.material("shutter_white", rgba=(0.88, 0.90, 0.93, 1.0))
    trim_dark = model.material("trim_dark", rgba=(0.17, 0.18, 0.20, 1.0))
    hinge_mount_x = _radial_profile_radius(DOME_OUTER_PROFILE, HINGE_Z) + 0.015

    base_ring = model.part("base_ring")
    base_ring.visual(
        _mesh("observatory_base_ring", _annulus(3.00, 2.18, 0.00, 0.92)),
        material=concrete,
        name="curb_wall",
    )
    base_ring.visual(
        _mesh("observatory_track_ring", _annulus(2.88, 2.54, 0.92, 1.06)),
        material=rail_steel,
        name="rotation_track",
    )
    base_ring.visual(
        _mesh("observatory_inner_ledge", _annulus(2.22, 1.66, 0.70, 0.78)),
        material=trim_dark,
        name="inner_service_ledge",
    )
    base_ring.visual(
        Cylinder(radius=3.10, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        material=concrete,
        name="foundation_flange",
    )
    base_ring.inertial = Inertial.from_geometry(
        Cylinder(radius=3.05, length=1.08),
        mass=9000.0,
        origin=Origin(xyz=(0.0, 0.0, 0.54)),
    )

    dome_shell = model.part("dome_shell")
    dome_shell.visual(
        _mesh("observatory_dome_shell", _build_dome_shell()),
        material=shell_white,
        name="shell_skin",
    )
    dome_shell.visual(
        _mesh("observatory_drive_skirt", _annulus(2.74, 2.48, 0.00, 0.18)),
        material=trim_dark,
        name="drive_skirt",
    )
    dome_shell.visual(
        Cylinder(radius=0.055, length=0.84),
        origin=Origin(
            xyz=(_radial_profile_radius(DOME_OUTER_PROFILE, HINGE_Z) - 0.05, 0.0, HINGE_Z - 0.02),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=trim_dark,
        name="slit_lintel",
    )
    dome_shell.visual(
        Box((0.17, 0.14, 0.22)),
        origin=Origin(xyz=(hinge_mount_x - 0.085, -0.42, HINGE_Z - 0.02)),
        material=trim_dark,
        name="left_crown_cheek",
    )
    dome_shell.visual(
        Box((0.17, 0.14, 0.22)),
        origin=Origin(xyz=(hinge_mount_x - 0.085, 0.42, HINGE_Z - 0.02)),
        material=trim_dark,
        name="right_crown_cheek",
    )
    dome_shell.visual(
        Box((0.10, 0.12, 0.42)),
        origin=Origin(xyz=(2.27, -0.50, 1.72)),
        material=trim_dark,
        name="left_slit_guide",
    )
    dome_shell.visual(
        Box((0.10, 0.12, 0.42)),
        origin=Origin(xyz=(2.27, 0.50, 1.72)),
        material=trim_dark,
        name="right_slit_guide",
    )
    dome_shell.inertial = Inertial.from_geometry(
        Cylinder(radius=2.70, length=3.10),
        mass=2800.0,
        origin=Origin(xyz=(0.0, 0.0, 1.55)),
    )

    shutter_leaf = model.part("shutter_leaf")
    shutter_leaf.visual(
        _mesh("observatory_crown_shutter", _build_shutter_leaf()),
        origin=Origin(xyz=(0.05, 0.0, 0.02), rpy=(0.0, 0.62, 0.0)),
        material=shutter_white,
        name="crown_panel",
    )
    shutter_leaf.visual(
        Cylinder(radius=0.042, length=0.64),
        origin=Origin(xyz=(0.03, 0.0, 0.01), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim_dark,
        name="hinge_barrel",
    )
    shutter_leaf.visual(
        Box((0.04, 0.10, 0.14)),
        origin=Origin(xyz=(0.02, -0.39, 0.03)),
        material=trim_dark,
        name="left_hinge_bracket",
    )
    shutter_leaf.visual(
        Box((0.04, 0.10, 0.14)),
        origin=Origin(xyz=(0.02, 0.39, 0.03)),
        material=trim_dark,
        name="right_hinge_bracket",
    )
    shutter_leaf.visual(
        Box((0.24, 0.84, 0.04)),
        origin=Origin(xyz=(0.16, 0.0, 0.00), rpy=(0.0, 0.50, 0.0)),
        material=trim_dark,
        name="stiffener_web",
    )
    shutter_leaf.inertial = Inertial.from_geometry(
        Box((0.90, 0.84, 0.34)),
        mass=160.0,
        origin=Origin(xyz=(0.36, 0.0, 0.12)),
    )

    model.articulation(
        "azimuth_rotation",
        ArticulationType.CONTINUOUS,
        parent=base_ring,
        child=dome_shell,
        origin=Origin(xyz=(0.0, 0.0, 1.06)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18000.0, velocity=0.45),
    )
    model.articulation(
        "slit_shutter",
        ArticulationType.REVOLUTE,
        parent=dome_shell,
        child=shutter_leaf,
        origin=Origin(
            xyz=(hinge_mount_x, 0.0, HINGE_Z)
        ),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2400.0,
            velocity=0.9,
            lower=0.0,
            upper=1.45,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base_ring = object_model.get_part("base_ring")
    dome_shell = object_model.get_part("dome_shell")
    shutter_leaf = object_model.get_part("shutter_leaf")
    azimuth = object_model.get_articulation("azimuth_rotation")
    shutter = object_model.get_articulation("slit_shutter")

    ctx.expect_origin_distance(
        dome_shell,
        base_ring,
        axes="xy",
        max_dist=0.001,
        name="dome rotation axis stays centered on the support ring",
    )
    ctx.expect_overlap(
        dome_shell,
        base_ring,
        axes="xy",
        min_overlap=4.8,
        elem_a="drive_skirt",
        elem_b="rotation_track",
        name="rotating skirt remains broadly supported by the base track",
    )
    ctx.expect_gap(
        dome_shell,
        base_ring,
        axis="z",
        positive_elem="drive_skirt",
        negative_elem="rotation_track",
        min_gap=0.0,
        max_gap=0.05,
        name="dome shell rides just above the support ring",
    )

    with ctx.pose({azimuth: 1.7}):
        ctx.expect_origin_distance(
            dome_shell,
            base_ring,
            axes="xy",
            max_dist=0.001,
            name="dome remains centered while rotating in azimuth",
        )

    def _aabb_center(aabb):
        if aabb is None:
            return None
        (min_x, min_y, min_z), (max_x, max_y, max_z) = aabb
        return (
            0.5 * (min_x + max_x),
            0.5 * (min_y + max_y),
            0.5 * (min_z + max_z),
        )

    closed_aabb = ctx.part_element_world_aabb(shutter_leaf, elem="crown_panel")
    with ctx.pose({shutter: 1.10}):
        open_aabb = ctx.part_element_world_aabb(shutter_leaf, elem="crown_panel")

    closed_pos = _aabb_center(closed_aabb)
    open_pos = _aabb_center(open_aabb)
    closed_tip_z = closed_aabb[1][2] if closed_aabb is not None else None
    open_tip_z = open_aabb[1][2] if open_aabb is not None else None

    ctx.check(
        "shutter leaf opens upward from the crown slit",
        closed_pos is not None
        and open_pos is not None
        and closed_tip_z is not None
        and open_tip_z is not None
        and open_tip_z > closed_tip_z + 0.45
        and open_pos[2] > closed_pos[2] + 0.05,
        details=f"closed_center={closed_pos}, open_center={open_pos}, closed_tip_z={closed_tip_z}, open_tip_z={open_tip_z}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
