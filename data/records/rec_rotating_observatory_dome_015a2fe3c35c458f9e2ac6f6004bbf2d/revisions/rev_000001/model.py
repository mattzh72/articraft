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
    mesh_from_geometry,
    section_loft,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


OUTER_DOME_PROFILE = [
    (2.08, 0.08),
    (2.08, 0.18),
    (2.02, 0.58),
    (1.90, 0.98),
    (1.70, 1.34),
    (1.38, 1.67),
    (0.94, 1.95),
    (0.42, 2.13),
    (0.10, 2.19),
    (0.0, 2.20),
]
INNER_DOME_PROFILE = [
    (1.94, 0.08),
    (1.94, 0.16),
    (1.88, 0.54),
    (1.76, 0.90),
    (1.58, 1.22),
    (1.30, 1.51),
    (0.88, 1.76),
    (0.38, 1.93),
    (0.09, 1.99),
    (0.0, 2.00),
]

SLIT_BOTTOM_Z = 1.82
SLIT_TOP_Z = 2.06
SLIT_HALF_WIDTH = 0.24
LEAF_STANDOFF = 0.010
LEAF_THICKNESS = 0.018
HINGE_AXIS_PROUD = 0.075


def _interp_radius(profile, z: float) -> float:
    if z <= profile[0][1]:
        return profile[0][0]
    if z >= profile[-1][1]:
        return profile[-1][0]

    for (r0, z0), (r1, z1) in zip(profile, profile[1:]):
        if z0 <= z <= z1:
            span = z1 - z0
            if span <= 1e-9:
                return r1
            t = (z - z0) / span
            return r0 + (r1 - r0) * t
    return profile[-1][0]


def _build_dome_shell_mesh():
    full_shell = LatheGeometry.from_shell_profiles(
        OUTER_DOME_PROFILE,
        INNER_DOME_PROFILE,
        segments=96,
        start_cap="flat",
        end_cap="flat",
        lip_samples=8,
    )

    slit_cutter = BoxGeometry((1.38, 0.68, 0.44)).translate(0.96, 0.0, 1.94)
    slit_cutter.merge(BoxGeometry((0.78, 0.72, 0.22)).translate(0.72, 0.0, 2.11))
    return boolean_difference(full_shell, slit_cutter)


def _build_leaf_section(z_abs: float, hinge_axis_x: float, half_width: float):
    shell_radius = _interp_radius(OUTER_DOME_PROFILE, z_abs)
    inner_radius = shell_radius + LEAF_STANDOFF
    outer_radius = inner_radius + LEAF_THICKNESS
    y_samples = [
        -half_width,
        -half_width * 0.66,
        -half_width * 0.33,
        0.0,
        half_width * 0.33,
        half_width * 0.66,
        half_width,
    ]

    outer_points = []
    inner_points = []
    for y in y_samples:
        outer_x = math.sqrt(max(outer_radius * outer_radius - y * y, 1e-6)) - hinge_axis_x
        inner_x = math.sqrt(max(inner_radius * inner_radius - y * y, 1e-6)) - hinge_axis_x
        local_z = z_abs - SLIT_TOP_Z
        outer_points.append((outer_x, y, local_z))
        inner_points.append((inner_x, y, local_z))
    return tuple(outer_points + list(reversed(inner_points)))


def _build_shutter_leaf_mesh(hinge_axis_x: float):
    half_width = SLIT_HALF_WIDTH - 0.015
    leaf_patch = section_loft(
        [
            _build_leaf_section(1.82, hinge_axis_x, half_width),
            _build_leaf_section(1.88, hinge_axis_x, half_width),
            _build_leaf_section(1.94, hinge_axis_x, half_width),
            _build_leaf_section(2.00, hinge_axis_x, half_width * 0.97),
            _build_leaf_section(2.055, hinge_axis_x, half_width * 0.92),
        ]
    )
    hinge_barrel = (
        CylinderGeometry(radius=0.018, height=half_width * 2.0 + 0.030, radial_segments=28)
        .rotate_x(math.pi / 2.0)
        .translate(0.0, 0.0, 0.0)
    )
    left_strap = BoxGeometry((0.040, 0.060, 0.020)).translate(0.010, -(half_width - 0.035), -0.012)
    right_strap = BoxGeometry((0.040, 0.060, 0.020)).translate(0.010, half_width - 0.035, -0.012)
    return leaf_patch.merge(hinge_barrel).merge(left_strap).merge(right_strap)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="observatory_dome")

    concrete = model.material("concrete", rgba=(0.70, 0.71, 0.72, 1.0))
    painted_steel = model.material("painted_steel", rgba=(0.87, 0.89, 0.92, 1.0))
    machinery_gray = model.material("machinery_gray", rgba=(0.34, 0.37, 0.40, 1.0))
    slit_dark = model.material("slit_dark", rgba=(0.10, 0.11, 0.13, 1.0))

    base_ring = model.part("base_ring")
    base_ring.visual(
        Cylinder(radius=2.36, length=0.56),
        origin=Origin(xyz=(0.0, 0.0, 0.28)),
        material=concrete,
        name="curb_wall",
    )
    base_ring.visual(
        Cylinder(radius=2.18, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, 0.62)),
        material=machinery_gray,
        name="bearing_plinth",
    )
    base_ring.visual(
        Cylinder(radius=2.08, length=0.05),
        origin=Origin(xyz=(0.0, 0.0, 0.705)),
        material=slit_dark,
        name="support_track",
    )
    base_ring.inertial = Inertial.from_geometry(
        Cylinder(radius=2.36, length=0.72),
        mass=4200.0,
        origin=Origin(xyz=(0.0, 0.0, 0.36)),
    )

    dome_shell = model.part("dome_shell")
    dome_shell.visual(
        _save_mesh("dome_shell", _build_dome_shell_mesh()),
        material=painted_steel,
        name="shell_body",
    )
    dome_shell.visual(
        Cylinder(radius=2.10, length=0.07),
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
        material=machinery_gray,
        name="rotating_bearing_band",
    )
    hinge_surface_x = _interp_radius(OUTER_DOME_PROFILE, SLIT_TOP_Z)
    hinge_axis_x = hinge_surface_x + HINGE_AXIS_PROUD
    bracket_y = SLIT_HALF_WIDTH - 0.015 + 0.015 + 0.075 / 2.0
    dome_shell.visual(
        Box((0.13, 0.075, 0.090)),
        origin=Origin(xyz=(hinge_axis_x - 0.040, -bracket_y, SLIT_TOP_Z - 0.030)),
        material=machinery_gray,
        name="left_hinge_bracket",
    )
    dome_shell.visual(
        Box((0.13, 0.075, 0.090)),
        origin=Origin(xyz=(hinge_axis_x - 0.040, bracket_y, SLIT_TOP_Z - 0.030)),
        material=machinery_gray,
        name="right_hinge_bracket",
    )
    dome_shell.inertial = Inertial.from_geometry(
        Cylinder(radius=2.10, length=2.20),
        mass=1350.0,
        origin=Origin(xyz=(0.0, 0.0, 1.00)),
    )

    shutter_leaf = model.part("shutter_leaf")
    shutter_leaf.visual(
        _save_mesh("shutter_leaf", _build_shutter_leaf_mesh(hinge_axis_x)),
        material=painted_steel,
        name="shutter_panel",
    )
    shutter_leaf.inertial = Inertial.from_geometry(
        Box((0.44, 0.48, 0.26)),
        mass=28.0,
        origin=Origin(
            xyz=(
                0.10,
                0.0,
                -0.11,
            ),
        ),
    )

    model.articulation(
        "base_to_dome_rotation",
        ArticulationType.CONTINUOUS,
        parent=base_ring,
        child=dome_shell,
        origin=Origin(xyz=(0.0, 0.0, 0.72)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2800.0,
            velocity=0.45,
        ),
    )
    model.articulation(
        "dome_to_shutter_leaf",
        ArticulationType.REVOLUTE,
        parent=dome_shell,
        child=shutter_leaf,
        origin=Origin(xyz=(hinge_axis_x, 0.0, SLIT_TOP_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=320.0,
            velocity=0.65,
            lower=0.0,
            upper=1.05,
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

    base_ring = object_model.get_part("base_ring")
    dome_shell = object_model.get_part("dome_shell")
    shutter_leaf = object_model.get_part("shutter_leaf")
    shutter_joint = object_model.get_articulation("dome_to_shutter_leaf")

    ctx.expect_overlap(
        dome_shell,
        base_ring,
        axes="xy",
        min_overlap=3.8,
        name="dome remains centered over the support ring",
    )
    ctx.expect_gap(
        dome_shell,
        base_ring,
        axis="z",
        positive_elem="rotating_bearing_band",
        negative_elem="support_track",
        min_gap=0.0,
        max_gap=0.05,
        name="rotating stage stays compact above the support track",
    )

    closed_panel = ctx.part_element_world_aabb(shutter_leaf, elem="shutter_panel")
    with ctx.pose({shutter_joint: shutter_joint.motion_limits.upper}):
        open_panel = ctx.part_element_world_aabb(shutter_leaf, elem="shutter_panel")

    opens_upward = (
        closed_panel is not None
        and open_panel is not None
        and open_panel[1][2] > closed_panel[1][2] + 0.05
        and open_panel[0][0] < closed_panel[0][0] - 0.05
    )
    ctx.check(
        "shutter leaf opens upward away from the slit",
        opens_upward,
        details=f"closed_panel={closed_panel}, open_panel={open_panel}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
