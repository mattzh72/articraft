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
    TorusGeometry,
    boolean_difference,
    mesh_from_geometry,
    section_loft,
)


GROUND_RADIUS = 2.18
DRUM_RADIUS = 1.48
BEARING_Z = 0.78
DOME_OUTER_RADIUS = 1.72
DOME_INNER_RADIUS = 1.64
SKIRT_HEIGHT = 0.46
SLIT_HALF_ANGLE = 0.145
SHUTTER_HINGE_Z = 2.04
SHUTTER_CROWN_Z = 2.17
SHUTTER_PROUD = 0.055
SHUTTER_HINGE_OFFSET = 0.045
SHUTTER_THICKNESS = 0.035


def _ring_band(
    *,
    outer_radius: float,
    inner_radius: float,
    z_center: float,
    height: float,
    radial_segments: int = 72,
):
    outer = CylinderGeometry(radius=outer_radius, height=height, radial_segments=radial_segments)
    inner = CylinderGeometry(
        radius=inner_radius,
        height=height + 0.01,
        radial_segments=radial_segments,
    )
    return boolean_difference(outer, inner).translate(0.0, 0.0, z_center)


def _dome_radius(radius: float, z_pos: float) -> float:
    if z_pos <= SKIRT_HEIGHT:
        return radius
    dz = z_pos - SKIRT_HEIGHT
    return math.sqrt(max(radius * radius - dz * dz, 0.0))


def _shell_profiles() -> tuple[list[tuple[float, float]], list[tuple[float, float]]]:
    outer_z = [0.0, 0.18, SKIRT_HEIGHT, 0.82, 1.20, 1.56, 1.86, 2.06, 2.14, 2.18]
    inner_z = [0.0, 0.18, SKIRT_HEIGHT, 0.82, 1.18, 1.50, 1.80, 1.98, 2.08, 2.10]
    outer = [(_dome_radius(DOME_OUTER_RADIUS, z_pos), z_pos) for z_pos in outer_z]
    inner = [(_dome_radius(DOME_INNER_RADIUS, z_pos), z_pos) for z_pos in inner_z]
    return outer, inner


def _build_dome_shell_mesh():
    outer_profile, inner_profile = _shell_profiles()
    shell = LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=96,
    )
    return boolean_difference(shell, _build_slit_cut_mesh())


def _shutter_section(
    *,
    hinge_x: float,
    z_abs: float,
    width: float,
) -> list[tuple[float, float, float]]:
    outer_x = _dome_radius(DOME_OUTER_RADIUS, z_abs) + SHUTTER_PROUD + SHUTTER_THICKNESS - hinge_x
    inner_x = outer_x - SHUTTER_THICKNESS
    half_width = width * 0.5
    z_local = z_abs - SHUTTER_HINGE_Z
    return [
        (outer_x, -half_width, z_local),
        (outer_x, half_width, z_local),
        (inner_x, half_width, z_local),
        (inner_x, -half_width, z_local),
    ]


def _build_slit_cut_mesh():
    lower_cut = BoxGeometry((2.35, 0.44, 1.72)).translate(1.00, 0.0, 1.56)
    crown_cut = BoxGeometry((1.30, 0.56, 0.52)).translate(0.68, 0.0, 2.02)
    return lower_cut.merge(crown_cut)


def _build_shutter_mesh():
    hinge_x = _dome_radius(DOME_OUTER_RADIUS, SHUTTER_HINGE_Z) + SHUTTER_HINGE_OFFSET
    sections = [
        _shutter_section(hinge_x=hinge_x, z_abs=SHUTTER_HINGE_Z, width=0.26),
        _shutter_section(hinge_x=hinge_x, z_abs=2.09, width=0.24),
        _shutter_section(hinge_x=hinge_x, z_abs=2.14, width=0.20),
        _shutter_section(hinge_x=hinge_x, z_abs=SHUTTER_CROWN_Z, width=0.15),
    ]
    shutter = section_loft(sections)
    shutter.merge(
        BoxGeometry((0.07, 0.18, 0.04)).translate(0.035, 0.0, 0.015)
    )
    shutter.merge(
        CylinderGeometry(radius=0.018, height=0.12, radial_segments=24)
        .rotate_x(math.pi / 2.0)
        .translate(0.0, 0.0, 0.0)
    )
    return shutter


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="observatory_dome")

    concrete = model.material("concrete", rgba=(0.70, 0.71, 0.72, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.23, 0.25, 0.28, 1.0))
    painted_white = model.material("painted_white", rgba=(0.88, 0.90, 0.92, 1.0))
    rail_grey = model.material("rail_grey", rgba=(0.56, 0.60, 0.63, 1.0))
    shutter_white = model.material("shutter_white", rgba=(0.90, 0.91, 0.92, 1.0))

    base_frame = model.part("base_frame")
    base_frame.visual(
        Cylinder(radius=GROUND_RADIUS, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
        material=concrete,
        name="ground_pad",
    )
    base_frame.visual(
        Cylinder(radius=DRUM_RADIUS, length=0.70),
        origin=Origin(xyz=(0.0, 0.0, 0.35)),
        material=concrete,
        name="support_drum",
    )
    base_frame.visual(
        mesh_from_geometry(
            _ring_band(
                outer_radius=1.62,
                inner_radius=1.30,
                z_center=0.72,
                height=0.12,
            ),
            "observatory_support_ring",
        ),
        material=dark_steel,
        name="support_ring",
    )
    base_frame.visual(
        mesh_from_geometry(
            TorusGeometry(
                radius=2.05,
                tube=0.035,
                radial_segments=18,
                tubular_segments=72,
            ).translate(0.0, 0.0, 0.98),
            "observatory_guard_hoop",
        ),
        material=rail_grey,
        name="guard_hoop",
    )

    for index in range(6):
        angle = index * math.tau / 6.0
        c = math.cos(angle)
        s = math.sin(angle)
        base_frame.visual(
            Cylinder(radius=0.04, length=0.98),
            origin=Origin(xyz=(2.05 * c, 2.05 * s, 0.49)),
            material=rail_grey,
            name=f"guard_post_{index}",
        )

    base_frame.inertial = Inertial.from_geometry(
        Box((GROUND_RADIUS * 2.0, GROUND_RADIUS * 2.0, 1.05)),
        mass=5200.0,
        origin=Origin(xyz=(0.0, 0.0, 0.525)),
    )

    dome_shell = model.part("dome_shell")
    dome_shell.visual(
        mesh_from_geometry(_build_dome_shell_mesh(), "observatory_dome_shell"),
        material=painted_white,
        name="dome_skin",
    )
    dome_shell.visual(
        mesh_from_geometry(
            _ring_band(
                outer_radius=1.70,
                inner_radius=1.56,
                z_center=0.06,
                height=0.12,
            ),
            "observatory_skirt_stiffener",
        ),
        material=dark_steel,
        name="skirt_stiffener",
    )
    hinge_x = _dome_radius(DOME_OUTER_RADIUS, SHUTTER_HINGE_Z) + SHUTTER_HINGE_OFFSET
    dome_shell.visual(
        Box((0.10, 0.36, 0.06)),
        origin=Origin(xyz=(hinge_x + 0.12, 0.0, SHUTTER_HINGE_Z + 0.05)),
        material=dark_steel,
        name="hinge_hood",
    )
    for side, y_pos in (("left", -0.175), ("right", 0.175)):
        dome_shell.visual(
            Box((0.16, 0.05, 0.10)),
            origin=Origin(
                xyz=(hinge_x + 0.02, y_pos, SHUTTER_HINGE_Z),
            ),
            material=dark_steel,
            name=f"hinge_strut_{side}",
        )
        dome_shell.visual(
            Cylinder(radius=0.018, length=0.05),
            origin=Origin(
                xyz=(hinge_x + 0.08, y_pos, SHUTTER_HINGE_Z + 0.03),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=dark_steel,
            name=f"hinge_knuckle_{side}",
        )
    dome_shell.inertial = Inertial.from_geometry(
        Box((DOME_OUTER_RADIUS * 2.0, DOME_OUTER_RADIUS * 2.0, 2.18)),
        mass=1850.0,
        origin=Origin(xyz=(0.0, 0.0, 1.09)),
    )

    model.articulation(
        "base_to_dome_rotation",
        ArticulationType.CONTINUOUS,
        parent=base_frame,
        child=dome_shell,
        origin=Origin(xyz=(0.0, 0.0, BEARING_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=280.0, velocity=0.35),
    )

    shutter_leaf = model.part("shutter_leaf")
    shutter_leaf.visual(
        mesh_from_geometry(_build_shutter_mesh(), "observatory_shutter_leaf"),
        material=shutter_white,
        name="shutter_panel",
    )
    shutter_leaf.inertial = Inertial.from_geometry(
        Box((0.74, 0.30, 0.18)),
        mass=140.0,
        origin=Origin(xyz=(-0.27, 0.0, 0.06)),
    )

    model.articulation(
        "dome_to_shutter",
        ArticulationType.REVOLUTE,
        parent=dome_shell,
        child=shutter_leaf,
        origin=Origin(
            xyz=(
                _dome_radius(DOME_OUTER_RADIUS, SHUTTER_HINGE_Z) + SHUTTER_HINGE_OFFSET,
                0.0,
                SHUTTER_HINGE_Z,
            )
        ),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.35,
            lower=0.0,
            upper=1.22,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base_frame = object_model.get_part("base_frame")
    dome_shell = object_model.get_part("dome_shell")
    shutter_leaf = object_model.get_part("shutter_leaf")
    dome_rotation = object_model.get_articulation("base_to_dome_rotation")
    shutter_joint = object_model.get_articulation("dome_to_shutter")

    ctx.expect_overlap(
        dome_shell,
        base_frame,
        axes="xy",
        elem_a="dome_skin",
        elem_b="support_ring",
        min_overlap=2.5,
        name="dome shell sits over the support ring",
    )
    ctx.expect_within(
        dome_shell,
        base_frame,
        axes="xy",
        inner_elem="dome_skin",
        outer_elem="guard_hoop",
        margin=0.05,
        name="guard hoop surrounds the rotating shell",
    )

    hood_rest = None
    hood_turned = None
    with ctx.pose({dome_rotation: 0.0}):
        hood_rest = ctx.part_element_world_aabb(dome_shell, elem="hinge_hood")
    with ctx.pose({dome_rotation: math.pi / 2.0}):
        hood_turned = ctx.part_element_world_aabb(dome_shell, elem="hinge_hood")

    ctx.check(
        "dome shell rotates around the support drum",
        hood_rest is not None
        and hood_turned is not None
        and ((hood_rest[0][0] + hood_rest[1][0]) * 0.5) > 0.75
        and ((hood_turned[0][1] + hood_turned[1][1]) * 0.5) > 0.75,
        details=f"rest={hood_rest}, turned={hood_turned}",
    )

    closed_aabb = None
    opened_aabb = None
    with ctx.pose({shutter_joint: 0.0}):
        closed_aabb = ctx.part_element_world_aabb(shutter_leaf, elem="shutter_panel")
    with ctx.pose({shutter_joint: 1.0}):
        opened_aabb = ctx.part_element_world_aabb(shutter_leaf, elem="shutter_panel")

    ctx.check(
        "shutter swings upward from the slit",
        closed_aabb is not None
        and opened_aabb is not None
        and opened_aabb[1][2] > closed_aabb[1][2] + 0.30
        and opened_aabb[1][0] > closed_aabb[1][0] + 0.05,
        details=f"closed={closed_aabb}, opened={opened_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
