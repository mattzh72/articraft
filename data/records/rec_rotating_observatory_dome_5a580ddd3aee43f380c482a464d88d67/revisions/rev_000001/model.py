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
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    section_loft,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


_DOME_SHELL_PROFILE = [
    (2.10, 0.00),
    (2.10, 0.55),
    (2.04, 0.85),
    (1.84, 1.28),
    (1.48, 1.70),
    (0.96, 2.05),
    (0.42, 2.31),
    (0.18, 2.42),
    (0.14, 2.46),
    (0.10, 2.39),
    (0.16, 2.34),
    (0.38, 2.24),
    (0.88, 1.99),
    (1.36, 1.67),
    (1.71, 1.28),
    (1.88, 0.86),
    (1.94, 0.55),
    (1.94, 0.00),
]

_DOME_OUTER_SKIN = _DOME_SHELL_PROFILE[:9]
_SHUTTER_HINGE_Z = 2.26
_SLIT_HALF_ANGLE = 0.34


def _build_dome_shell_mesh():
    profile = _DOME_SHELL_PROFILE
    profile_count = len(profile)
    angle_count = 120
    start_angle = math.pi / 2.0 + _SLIT_HALF_ANGLE
    end_angle = math.pi / 2.0 + math.tau - _SLIT_HALF_ANGLE
    geometry = MeshGeometry()
    vertex_rows: list[list[int]] = []

    for angle_index in range(angle_count + 1):
        t = angle_index / angle_count
        theta = start_angle + (end_angle - start_angle) * t
        cos_theta = math.cos(theta)
        sin_theta = math.sin(theta)
        row: list[int] = []
        for radius, z_pos in profile:
            row.append(geometry.add_vertex(radius * cos_theta, radius * sin_theta, z_pos))
        vertex_rows.append(row)

    for angle_index in range(angle_count):
        row_a = vertex_rows[angle_index]
        row_b = vertex_rows[angle_index + 1]
        for profile_index in range(profile_count):
            next_index = (profile_index + 1) % profile_count
            a = row_a[profile_index]
            b = row_a[next_index]
            c = row_b[next_index]
            d = row_b[profile_index]
            geometry.add_face(a, b, c)
            geometry.add_face(a, c, d)
    return geometry


def _dome_outer_radius_at_z(z: float) -> float:
    if z <= _DOME_OUTER_SKIN[0][1]:
        return _DOME_OUTER_SKIN[0][0]
    if z >= _DOME_OUTER_SKIN[-1][1]:
        return _DOME_OUTER_SKIN[-1][0]
    for (r0, z0), (r1, z1) in zip(_DOME_OUTER_SKIN, _DOME_OUTER_SKIN[1:]):
        if z0 <= z <= z1:
            t = (z - z0) / (z1 - z0)
            return r0 + (r1 - r0) * t
    return _DOME_OUTER_SKIN[-1][0]


_SHUTTER_HINGE_Y = _dome_outer_radius_at_z(_SHUTTER_HINGE_Z) + 0.085


def _shutter_loop(z: float, half_width: float, y_back: float, y_front: float) -> list[tuple[float, float, float]]:
    return [
        (-half_width, y_back, z),
        (-half_width, y_front, z),
        (half_width, y_front, z),
        (half_width, y_back, z),
    ]


def _build_shutter_panel_mesh():
    sections = [
        _shutter_loop(0.012, 0.072, 0.054, 0.088),
        _shutter_loop(-0.18, 0.10, 0.062, 0.100),
        _shutter_loop(-0.54, 0.20, 0.046, 0.086),
        _shutter_loop(-0.98, 0.30, 0.038, 0.082),
    ]
    return section_loft(sections)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="observatory_dome")

    concrete = model.material("concrete", rgba=(0.69, 0.69, 0.70, 1.0))
    steel = model.material("steel", rgba=(0.48, 0.50, 0.53, 1.0))
    bearing_dark = model.material("bearing_dark", rgba=(0.18, 0.19, 0.21, 1.0))
    dome_white = model.material("dome_white", rgba=(0.92, 0.94, 0.96, 1.0))
    shutter_gray = model.material("shutter_gray", rgba=(0.80, 0.83, 0.86, 1.0))
    aperture_black = model.material("aperture_black", rgba=(0.05, 0.05, 0.06, 1.0))

    base_ring = model.part("base_ring")
    base_ring.visual(
        Cylinder(radius=2.45, length=0.14),
        origin=Origin(xyz=(0.0, 0.0, 0.07)),
        material=concrete,
        name="foundation_pad",
    )
    base_ring.visual(
        Cylinder(radius=1.92, length=0.26),
        origin=Origin(xyz=(0.0, 0.0, 0.27)),
        material=concrete,
        name="support_drum",
    )
    base_ring.visual(
        Cylinder(radius=2.02, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, 0.43)),
        material=steel,
        name="bearing_curb",
    )
    base_ring.visual(
        _mesh(
            "azimuth_rail",
            TorusGeometry(radius=1.97, tube=0.04, radial_segments=18, tubular_segments=72),
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.39)),
        material=bearing_dark,
        name="azimuth_rail",
    )
    base_ring.inertial = Inertial.from_geometry(
        Box((4.90, 4.90, 0.50)),
        mass=1800.0,
        origin=Origin(xyz=(0.0, 0.0, 0.25)),
    )

    dome_shell = model.part("dome_shell")
    dome_shell.visual(
        _mesh("dome_shell_body", _build_dome_shell_mesh()),
        material=dome_white,
        name="dome_shell_body",
    )
    dome_shell.visual(
        Cylinder(radius=2.00, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
        material=steel,
        name="rotation_skirt",
    )
    dome_shell.visual(
        Box((0.72, 0.08, 0.56)),
        origin=Origin(xyz=(0.0, 0.48, 2.02)),
        material=aperture_black,
        name="crown_aperture",
    )
    dome_shell.visual(
        Box((0.74, 0.06, 0.10)),
        origin=Origin(xyz=(0.0, 0.53, 2.18)),
        material=steel,
        name="hinge_header",
    )
    dome_shell.visual(
        Cylinder(radius=0.05, length=0.16),
        origin=Origin(xyz=(-0.19, _SHUTTER_HINGE_Y, _SHUTTER_HINGE_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="shell_hinge_left",
    )
    dome_shell.visual(
        Box((0.08, 0.08, 0.16)),
        origin=Origin(xyz=(-0.19, 0.57, 2.18)),
        material=steel,
        name="hinge_cheek_left",
    )
    dome_shell.visual(
        Cylinder(radius=0.05, length=0.16),
        origin=Origin(xyz=(0.19, _SHUTTER_HINGE_Y, _SHUTTER_HINGE_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="shell_hinge_right",
    )
    dome_shell.visual(
        Box((0.08, 0.08, 0.16)),
        origin=Origin(xyz=(0.19, 0.57, 2.18)),
        material=steel,
        name="hinge_cheek_right",
    )
    dome_shell.inertial = Inertial.from_geometry(
        Cylinder(radius=2.10, length=2.46),
        mass=1200.0,
        origin=Origin(xyz=(0.0, 0.0, 1.23)),
    )

    shutter_leaf = model.part("shutter_leaf")
    shutter_leaf.visual(
        _mesh("shutter_panel", _build_shutter_panel_mesh()),
        material=shutter_gray,
        name="shutter_panel",
    )
    shutter_leaf.visual(
        Box((0.18, 0.032, 0.12)),
        origin=Origin(xyz=(0.0, 0.061, -0.048)),
        material=steel,
        name="shutter_hinge_flange",
    )
    shutter_leaf.visual(
        Cylinder(radius=0.05, length=0.22),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="shutter_leaf_knuckle",
    )
    shutter_leaf.inertial = Inertial.from_geometry(
        Box((0.72, 0.18, 0.24)),
        mass=95.0,
        origin=Origin(xyz=(0.0, 0.26, -0.12)),
    )

    model.articulation(
        "base_to_dome_rotation",
        ArticulationType.CONTINUOUS,
        parent=base_ring,
        child=dome_shell,
        origin=Origin(xyz=(0.0, 0.0, 0.46)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2500.0, velocity=0.35),
    )
    model.articulation(
        "dome_to_shutter",
        ArticulationType.REVOLUTE,
        parent=dome_shell,
        child=shutter_leaf,
        origin=Origin(xyz=(0.0, _SHUTTER_HINGE_Y, _SHUTTER_HINGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=500.0, velocity=0.8, lower=0.0, upper=1.10),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base_ring = object_model.get_part("base_ring")
    dome_shell = object_model.get_part("dome_shell")
    shutter_leaf = object_model.get_part("shutter_leaf")
    dome_rotation = object_model.get_articulation("base_to_dome_rotation")
    shutter_joint = object_model.get_articulation("dome_to_shutter")

    with ctx.pose({dome_rotation: 0.0}):
        ctx.expect_gap(
            dome_shell,
            base_ring,
            axis="z",
            min_gap=0.0,
            max_gap=0.002,
            positive_elem="dome_shell_body",
            negative_elem="bearing_curb",
            name="dome shell sits on the support curb without penetration",
        )
        ctx.expect_overlap(
            dome_shell,
            base_ring,
            axes="xy",
            min_overlap=3.7,
            elem_a="dome_shell_body",
            elem_b="support_drum",
            name="dome shell is large relative to the fixed support footprint",
        )
        ctx.expect_overlap(
            shutter_leaf,
            dome_shell,
            axes="xz",
            min_overlap=0.50,
            elem_a="shutter_panel",
            elem_b="crown_aperture",
            name="closed shutter spans the slit opening near the crown",
        )
        ctx.expect_contact(
            dome_shell,
            dome_shell,
            elem_a="shell_hinge_left",
            elem_b="hinge_cheek_left",
            contact_tol=0.002,
            name="left hinge barrel is physically carried by its shell-side cheek bracket",
        )
        ctx.expect_contact(
            shutter_leaf,
            dome_shell,
            elem_a="shutter_leaf_knuckle",
            elem_b="shell_hinge_left",
            contact_tol=0.0005,
            name="leaf knuckle bears against the left shell-side hinge barrel",
        )
        ctx.expect_contact(
            shutter_leaf,
            dome_shell,
            elem_a="shutter_leaf_knuckle",
            elem_b="shell_hinge_right",
            contact_tol=0.0005,
            name="leaf knuckle bears against the right shell-side hinge barrel",
        )

    with ctx.pose({dome_rotation: 1.4}):
        ctx.expect_gap(
            dome_shell,
            base_ring,
            axis="z",
            min_gap=0.0,
            max_gap=0.002,
            positive_elem="dome_shell_body",
            negative_elem="bearing_curb",
            name="bearing support contact is preserved while rotating",
        )

    closed_aabb = ctx.part_element_world_aabb(shutter_leaf, elem="shutter_panel")
    with ctx.pose({shutter_joint: 1.0}):
        open_aabb = ctx.part_element_world_aabb(shutter_leaf, elem="shutter_panel")
        ctx.check(
            "shutter opens upward above the crown line",
            closed_aabb is not None
            and open_aabb is not None
            and open_aabb[0][2] > closed_aabb[0][2] + 0.30,
            details=f"closed_aabb={closed_aabb}, open_aabb={open_aabb}",
        )
        ctx.check(
            "shutter swings outward from the slit",
            closed_aabb is not None
            and open_aabb is not None
            and open_aabb[1][1] > closed_aabb[1][1] + 0.18,
            details=f"closed_aabb={closed_aabb}, open_aabb={open_aabb}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
