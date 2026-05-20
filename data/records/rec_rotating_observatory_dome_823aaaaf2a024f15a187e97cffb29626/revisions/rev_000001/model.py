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
    CylinderGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    boolean_difference,
    boolean_intersection,
    mesh_from_geometry,
)


OUTER_PROFILE = [
    (2.18, 0.00),
    (2.18, 0.62),
    (2.12, 1.08),
    (1.96, 1.52),
    (1.68, 1.95),
    (1.28, 2.32),
    (0.78, 2.62),
    (0.24, 2.82),
    (0.00, 2.88),
]
INNER_PROFILE = [
    (2.08, 0.06),
    (2.08, 0.62),
    (2.03, 1.05),
    (1.88, 1.47),
    (1.62, 1.87),
    (1.24, 2.21),
    (0.76, 2.48),
    (0.30, 2.68),
    (0.00, 2.74),
]

OPENING_CENTER_X = 1.65
OPENING_WIDTH_Y = 1.12
OPENING_HEIGHT_Z = 1.96
OPENING_CENTER_Z = 1.72

PANEL_CENTER_X = 1.62
PANEL_WIDTH_Y = 0.80
PANEL_HEIGHT_Z = 1.70
PANEL_CENTER_Z = 1.69
SHUTTER_OUTER_OFFSET = 0.12
SHUTTER_INNER_OFFSET = 0.06
HINGE_OFFSET_X = 0.44


def _radius_at_z(profile: list[tuple[float, float]], z_value: float) -> float:
    if z_value <= profile[0][1]:
        return profile[0][0]
    for (r0, z0), (r1, z1) in zip(profile, profile[1:]):
        if z0 <= z_value <= z1:
            if abs(z1 - z0) < 1e-9:
                return r1
            t = (z_value - z0) / (z1 - z0)
            return r0 + (r1 - r0) * t
    return profile[-1][0]


def _annulus_geometry(
    *,
    outer_radius: float,
    inner_radius: float,
    height: float,
    z_center: float = 0.0,
):
    ring = boolean_difference(
        CylinderGeometry(radius=outer_radius, height=height, radial_segments=72),
        CylinderGeometry(radius=inner_radius, height=height + 0.01, radial_segments=72),
    )
    return ring.translate(0.0, 0.0, z_center)


def _dome_shell_geometry():
    shell = LatheGeometry.from_shell_profiles(
        OUTER_PROFILE,
        INNER_PROFILE,
        segments=104,
        start_cap="flat",
        end_cap="flat",
        lip_samples=10,
    )
    opening_cut = BoxGeometry((2.70, OPENING_WIDTH_Y, OPENING_HEIGHT_Z)).translate(
        OPENING_CENTER_X,
        0.0,
        OPENING_CENTER_Z,
    )
    return boolean_difference(shell, opening_cut)


def _shutter_leaf_geometry(hinge_x: float, hinge_z: float):
    shutter_outer = [(radius + SHUTTER_OUTER_OFFSET, z) for radius, z in OUTER_PROFILE]
    shutter_inner = [(radius + SHUTTER_INNER_OFFSET, z) for radius, z in OUTER_PROFILE]
    leaf_shell = LatheGeometry.from_shell_profiles(
        shutter_outer,
        shutter_inner,
        segments=104,
        start_cap="flat",
        end_cap="flat",
        lip_samples=10,
    )
    leaf_cut = BoxGeometry((2.60, PANEL_WIDTH_Y, PANEL_HEIGHT_Z)).translate(
        PANEL_CENTER_X,
        0.0,
        PANEL_CENTER_Z,
    )
    leaf = boolean_intersection(leaf_shell, leaf_cut)
    return leaf.translate(-hinge_x, 0.0, -hinge_z)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="observatory_dome")

    concrete = model.material("concrete", rgba=(0.67, 0.67, 0.68, 1.0))
    white_shell = model.material("white_shell", rgba=(0.93, 0.94, 0.96, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.18, 0.20, 0.22, 1.0))
    steel = model.material("steel", rgba=(0.54, 0.57, 0.61, 1.0))
    shutter_gray = model.material("shutter_gray", rgba=(0.81, 0.83, 0.85, 1.0))

    hinge_z = OPENING_CENTER_Z + (OPENING_HEIGHT_Z * 0.5)
    hinge_x = _radius_at_z(OUTER_PROFILE, hinge_z) + HINGE_OFFSET_X
    hinge_support_length = 1.88 - hinge_x + 0.20
    hinge_support_center_x = 0.5 * (1.88 + hinge_x + 0.02)

    base = model.part("base")
    base.visual(
        Cylinder(radius=2.85, length=0.20),
        origin=Origin(xyz=(0.0, 0.0, 0.10)),
        material=concrete,
        name="foundation_pad",
    )
    base.visual(
        Cylinder(radius=2.42, length=0.62),
        origin=Origin(xyz=(0.0, 0.0, 0.51)),
        material=concrete,
        name="base_ring",
    )
    base.visual(
        mesh_from_geometry(
            _annulus_geometry(outer_radius=2.62, inner_radius=2.30, height=0.12, z_center=0.88),
            "support_track_ring",
        ),
        material=dark_trim,
        name="support_track_ring",
    )
    base.visual(
        mesh_from_geometry(
            _annulus_geometry(outer_radius=2.00, inner_radius=1.76, height=0.08, z_center=0.86),
            "inner_weather_ring",
        ),
        material=steel,
        name="inner_weather_ring",
    )
    for side, sign in (("left", 1.0), ("right", -1.0)):
        pier_y = sign * 2.84
        head_y = sign * 2.78
        roller_y = sign * 2.57
        bracket_y = sign * 2.70
        base.visual(
            Box((0.44, 0.38, 1.20)),
            origin=Origin(xyz=(0.0, pier_y, 0.70)),
            material=concrete,
            name=f"{side}_support_pier",
        )
        base.visual(
            Box((1.38, 0.24, 0.20)),
            origin=Origin(xyz=(0.0, head_y, 1.26)),
            material=steel,
            name=f"{side}_support_head",
        )
        base.visual(
            Cylinder(radius=0.13, length=0.90),
            origin=Origin(xyz=(0.0, roller_y, 1.08), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark_trim,
            name=f"{side}_support_roller",
        )
        base.visual(
            Box((0.26, 0.22, 0.50)),
            origin=Origin(xyz=(0.0, bracket_y, 1.05)),
            material=steel,
            name=f"{side}_roller_bracket",
        )
    base.inertial = Inertial.from_geometry(
        Cylinder(radius=2.85, length=1.10),
        mass=11000.0,
        origin=Origin(xyz=(0.0, 0.0, 0.55)),
    )

    dome = model.part("dome_shell")
    dome.visual(
        mesh_from_geometry(_dome_shell_geometry(), "dome_shell_body"),
        material=white_shell,
        name="dome_body",
    )
    dome.visual(
        mesh_from_geometry(
            _annulus_geometry(outer_radius=2.44, inner_radius=2.10, height=0.12, z_center=0.02),
            "dome_rotation_flange",
        ),
        material=steel,
        name="rotation_flange",
    )
    dome.visual(
        Box((0.20, 0.10, 1.92)),
        origin=Origin(xyz=(1.88, 0.61, 1.72)),
        material=dark_trim,
        name="slit_left_jamb",
    )
    dome.visual(
        Box((0.20, 0.10, 1.92)),
        origin=Origin(xyz=(1.88, -0.61, 1.72)),
        material=dark_trim,
        name="slit_right_jamb",
    )
    dome.visual(
        Box((hinge_support_length, 0.12, 0.14)),
        origin=Origin(xyz=(hinge_support_center_x, 0.62, hinge_z - 0.02)),
        material=dark_trim,
        name="slit_hinge_bracket_left",
    )
    dome.visual(
        Box((hinge_support_length, 0.12, 0.14)),
        origin=Origin(xyz=(hinge_support_center_x, -0.62, hinge_z - 0.02)),
        material=dark_trim,
        name="slit_hinge_bracket_right",
    )
    dome.visual(
        Box((0.24, 1.10, 0.10)),
        origin=Origin(xyz=(2.02, 0.0, OPENING_CENTER_Z - (OPENING_HEIGHT_Z * 0.5) + 0.03)),
        material=dark_trim,
        name="slit_bottom_sill",
    )
    dome.inertial = Inertial.from_geometry(
        Cylinder(radius=2.22, length=3.00),
        mass=2800.0,
        origin=Origin(xyz=(0.0, 0.0, 1.44)),
    )

    shutter = model.part("shutter_leaf")
    shutter.visual(
        mesh_from_geometry(_shutter_leaf_geometry(hinge_x, hinge_z), "shutter_leaf_panel"),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=shutter_gray,
        name="shutter_panel",
    )
    shutter.visual(
        Cylinder(radius=0.046, length=0.12),
        origin=Origin(xyz=(0.0, 0.50, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_trim,
        name="shutter_barrel_left",
    )
    shutter.visual(
        Cylinder(radius=0.046, length=0.12),
        origin=Origin(xyz=(0.0, -0.50, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_trim,
        name="shutter_barrel_right",
    )
    shutter.visual(
        Box((0.16, 0.18, 0.20)),
        origin=Origin(xyz=(0.09, 0.47, -0.10)),
        material=dark_trim,
        name="shutter_hinge_strap_left",
    )
    shutter.visual(
        Box((0.16, 0.18, 0.20)),
        origin=Origin(xyz=(0.09, -0.47, -0.10)),
        material=dark_trim,
        name="shutter_hinge_strap_right",
    )
    shutter.visual(
        Box((0.12, 1.12, 0.10)),
        origin=Origin(xyz=(0.10, 0.0, -0.11)),
        material=dark_trim,
        name="shutter_hinge_header",
    )
    shutter.visual(
        Box((0.06, 0.42, 0.05)),
        origin=Origin(xyz=(1.17, 0.0, -1.18)),
        material=dark_trim,
        name="shutter_handle_rail",
    )
    shutter.visual(
        Box((0.12, 0.05, 0.14)),
        origin=Origin(xyz=(1.09, 0.15, -1.18)),
        material=dark_trim,
        name="shutter_handle_bracket_left",
    )
    shutter.visual(
        Box((0.12, 0.05, 0.14)),
        origin=Origin(xyz=(1.09, -0.15, -1.18)),
        material=dark_trim,
        name="shutter_handle_bracket_right",
    )
    shutter.inertial = Inertial.from_geometry(
        Box((1.70, 0.90, 2.10)),
        mass=220.0,
        origin=Origin(xyz=(0.82, 0.0, -0.85)),
    )

    model.articulation(
        "base_to_dome_rotation",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=dome,
        origin=Origin(xyz=(0.0, 0.0, 0.98)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=5000.0, velocity=0.45),
    )
    model.articulation(
        "dome_to_shutter",
        ArticulationType.REVOLUTE,
        parent=dome,
        child=shutter,
        origin=Origin(xyz=(hinge_x, 0.0, hinge_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=450.0,
            velocity=0.8,
            lower=0.0,
            upper=1.25,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    dome = object_model.get_part("dome_shell")
    shutter = object_model.get_part("shutter_leaf")
    dome_rotation = object_model.get_articulation("base_to_dome_rotation")
    shutter_hinge = object_model.get_articulation("dome_to_shutter")

    ctx.expect_contact(
        dome,
        shutter,
        elem_a="slit_hinge_bracket_left",
        elem_b="shutter_hinge_header",
        name="left hinge support contacts shutter header when closed",
    )
    ctx.expect_contact(
        dome,
        shutter,
        elem_a="slit_hinge_bracket_right",
        elem_b="shutter_hinge_header",
        name="right hinge support contacts shutter header when closed",
    )

    closed_panel_aabb = ctx.part_element_world_aabb(shutter, elem="shutter_panel")
    closed_shutter_pos = ctx.part_world_position(shutter)

    with ctx.pose({shutter_hinge: 1.25}):
        open_panel_aabb = ctx.part_element_world_aabb(shutter, elem="shutter_panel")
        ctx.expect_contact(
            dome,
            shutter,
            elem_a="slit_hinge_bracket_left",
            elem_b="shutter_barrel_left",
            name="left hinge barrel stays seated in the side support when open",
        )
        ctx.expect_contact(
            dome,
            shutter,
            elem_a="slit_hinge_bracket_right",
            elem_b="shutter_barrel_right",
            name="right hinge barrel stays seated in the side support when open",
        )

    open_motion_ok = (
        closed_panel_aabb is not None
        and open_panel_aabb is not None
        and open_panel_aabb[1][0] > closed_panel_aabb[1][0] + 0.75
        and open_panel_aabb[1][2] > closed_panel_aabb[1][2] + 1.0
    )
    ctx.check(
        "shutter panel lifts above the crown when opened",
        open_motion_ok,
        details=f"closed_panel_aabb={closed_panel_aabb}, open_panel_aabb={open_panel_aabb}",
    )

    with ctx.pose({dome_rotation: 1.0}):
        rotated_shutter_pos = ctx.part_world_position(shutter)

    rotation_ok = (
        closed_shutter_pos is not None
        and rotated_shutter_pos is not None
        and abs(rotated_shutter_pos[1] - closed_shutter_pos[1]) > 0.6
        and abs(rotated_shutter_pos[0] - closed_shutter_pos[0]) > 0.3
    )
    ctx.check(
        "dome rotation carries the shutter assembly around the base ring",
        rotation_ok,
        details=f"closed_shutter_pos={closed_shutter_pos}, rotated_shutter_pos={rotated_shutter_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
