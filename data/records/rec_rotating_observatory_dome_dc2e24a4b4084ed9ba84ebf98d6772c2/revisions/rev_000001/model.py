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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    repair_loft,
    section_loft,
    tube_from_spline_points,
)


ROTARY_AXIS_Z = 1.73


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _sector_shell_section(
    outer_radius: float,
    inner_radius: float,
    z_pos: float,
    *,
    slit_half_angle: float,
    arc_samples: int = 44,
):
    outer = []
    inner = []
    start = slit_half_angle
    end = (2.0 * math.pi) - slit_half_angle
    for index in range(arc_samples):
        angle = start + (end - start) * index / (arc_samples - 1)
        outer.append((outer_radius * math.cos(angle), outer_radius * math.sin(angle), z_pos))
        inner.append((inner_radius * math.cos(angle), inner_radius * math.sin(angle), z_pos))
    return outer + list(reversed(inner))


def _build_dome_shell_mesh():
    profile_sections = [
        (3.02, 2.86, 0.26, 0.255),
        (2.98, 2.82, 0.56, 0.255),
        (2.86, 2.72, 1.02, 0.250),
        (2.60, 2.48, 1.52, 0.270),
        (2.18, 2.10, 2.06, 0.300),
        (1.58, 1.52, 2.60, 0.360),
        (0.94, 0.90, 3.00, 0.560),
        (0.34, 0.30, 3.27, 1.050),
    ]
    sections = [
        _sector_shell_section(
            outer_radius,
            inner_radius,
            z_pos,
            slit_half_angle=slit_half_angle,
        )
        for outer_radius, inner_radius, z_pos, slit_half_angle in profile_sections
    ]
    return repair_loft(
        section_loft(
            sections,
            cap=True,
            solid=True,
        )
    )


def _build_shutter_leaf_mesh():
    sections = []
    samples = [
        (-0.01, 0.78, 1.02, 0.05),
        (-0.10, 0.90, 1.00, 0.05),
        (-0.22, 1.06, 0.96, 0.05),
        (-0.36, 1.28, 0.90, 0.045),
        (-0.52, 1.54, 0.82, 0.04),
    ]
    for z_pos, x_pos, width, thickness in samples:
        half_w = 0.5 * width
        half_t = 0.5 * thickness
        sections.append(
            [
                (x_pos, -half_w, z_pos - half_t),
                (x_pos, half_w, z_pos - half_t),
                (x_pos, half_w, z_pos + half_t),
                (x_pos, -half_w, z_pos + half_t),
            ]
        )
    return repair_loft(section_loft(sections))


def _build_slit_rib_mesh(side: float):
    return tube_from_spline_points(
        [
            (2.92, side * 0.76, 0.52),
            (2.74, side * 0.74, 1.00),
            (2.36, side * 0.70, 1.70),
            (1.70, side * 0.62, 2.46),
            (0.94, side * 0.50, 3.07),
        ],
        radius=0.09,
        samples_per_segment=14,
        radial_segments=18,
        cap_ends=True,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="observatory_dome")

    concrete = model.material("concrete", rgba=(0.70, 0.70, 0.72, 1.0))
    painted_white = model.material("painted_white", rgba=(0.90, 0.92, 0.94, 1.0))
    metal_gray = model.material("metal_gray", rgba=(0.48, 0.50, 0.54, 1.0))
    dark_metal = model.material("dark_metal", rgba=(0.22, 0.23, 0.25, 1.0))
    track_metal = model.material("track_metal", rgba=(0.34, 0.35, 0.38, 1.0))

    base_ring = model.part("base_ring")
    base_ring.visual(
        Cylinder(radius=4.10, length=0.40),
        origin=Origin(xyz=(0.0, 0.0, 0.20)),
        material=concrete,
        name="foundation_slab",
    )
    base_ring.visual(
        Cylinder(radius=3.42, length=1.26),
        origin=Origin(xyz=(0.0, 0.0, 0.83)),
        material=concrete,
        name="support_drum",
    )
    base_ring.visual(
        Cylinder(radius=3.58, length=0.28),
        origin=Origin(xyz=(0.0, 0.0, 1.60)),
        material=track_metal,
        name="bearing_ring",
    )
    base_ring.visual(
        Cylinder(radius=2.55, length=0.20),
        origin=Origin(xyz=(0.0, 0.0, 1.49)),
        material=dark_metal,
        name="inner_service_ring",
    )
    for index, (x_pos, y_pos, yaw, roller_axis_y) in enumerate(
        (
            (2.70, 0.0, 0.0, True),
            (-2.70, 0.0, 0.0, True),
            (0.0, 2.70, math.pi / 2.0, False),
            (0.0, -2.70, math.pi / 2.0, False),
        )
    ):
        base_ring.visual(
            Box((1.60, 1.18, 1.12)),
            origin=Origin(xyz=(x_pos, y_pos, 0.98), rpy=(0.0, 0.0, yaw)),
            material=concrete,
            name=f"side_support_{index}",
        )
        base_ring.visual(
            Box((1.82, 1.36, 0.42)),
            origin=Origin(xyz=(x_pos, y_pos, 1.47), rpy=(0.0, 0.0, yaw)),
            material=metal_gray,
            name=f"bearing_plinth_{index}",
        )
        base_ring.visual(
            Cylinder(radius=0.27, length=1.22),
            origin=Origin(
                xyz=(x_pos, y_pos, 1.37),
                rpy=((math.pi / 2.0, 0.0, 0.0) if roller_axis_y else (0.0, math.pi / 2.0, 0.0)),
            ),
            material=dark_metal,
            name=f"roller_housing_{index}",
        )
    base_ring.inertial = Inertial.from_geometry(
        Cylinder(radius=4.10, length=1.88),
        mass=28000.0,
        origin=Origin(xyz=(0.0, 0.0, 0.94)),
    )

    dome_shell = model.part("dome_shell")
    dome_shell.visual(
        Cylinder(radius=3.16, length=0.34),
        origin=Origin(xyz=(0.0, 0.0, 0.18)),
        material=track_metal,
        name="rotary_skirt",
    )
    dome_shell.visual(
        Cylinder(radius=3.02, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, 0.38)),
        material=dark_metal,
        name="equatorial_flange",
    )
    dome_shell.visual(
        _mesh("dome_shell_body", _build_dome_shell_mesh()),
        material=painted_white,
        name="shell_body",
    )
    dome_shell.visual(
        _mesh("slit_rib_left", _build_slit_rib_mesh(1.0)),
        material=metal_gray,
        name="slit_rib_left",
    )
    dome_shell.visual(
        _mesh("slit_rib_right", _build_slit_rib_mesh(-1.0)),
        material=metal_gray,
        name="slit_rib_right",
    )
    dome_shell.visual(
        Box((0.34, 1.04, 0.18)),
        origin=Origin(xyz=(0.52, 0.0, 2.90)),
        material=metal_gray,
        name="crown_header",
    )
    dome_shell.visual(
        Box((0.30, 0.24, 0.44)),
        origin=Origin(xyz=(0.64, 0.52, 2.98)),
        material=dark_metal,
        name="crown_side_web_left",
    )
    dome_shell.visual(
        Box((0.30, 0.24, 0.44)),
        origin=Origin(xyz=(0.64, -0.52, 2.98)),
        material=dark_metal,
        name="crown_side_web_right",
    )
    dome_shell.visual(
        Box((0.22, 0.22, 0.54)),
        origin=Origin(xyz=(0.78, 0.42, 2.97)),
        material=dark_metal,
        name="hinge_cheek_left",
    )
    dome_shell.visual(
        Box((0.22, 0.22, 0.54)),
        origin=Origin(xyz=(0.78, -0.42, 2.97)),
        material=dark_metal,
        name="hinge_cheek_right",
    )
    dome_shell.visual(
        Cylinder(radius=0.08, length=0.12),
        origin=Origin(xyz=(0.78, 0.38, 3.12), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="hinge_lug_left",
    )
    dome_shell.visual(
        Cylinder(radius=0.08, length=0.12),
        origin=Origin(xyz=(0.78, -0.38, 3.12), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_metal,
        name="hinge_lug_right",
    )
    dome_shell.visual(
        Cylinder(radius=0.24, length=0.50),
        origin=Origin(xyz=(0.78, 0.76, 3.05), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="hinge_side_boss_left",
    )
    dome_shell.visual(
        Cylinder(radius=0.24, length=0.50),
        origin=Origin(xyz=(0.78, -0.76, 3.05), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="hinge_side_boss_right",
    )
    dome_shell.inertial = Inertial.from_geometry(
        Cylinder(radius=3.18, length=3.44),
        mass=6800.0,
        origin=Origin(xyz=(0.0, 0.0, 1.72)),
    )

    shutter_leaf = model.part("shutter_leaf")
    shutter_leaf.visual(
        Cylinder(radius=0.065, length=0.56),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal_gray,
        name="hinge_knuckle_center",
    )
    shutter_leaf.visual(
        Box((0.30, 0.62, 0.14)),
        origin=Origin(xyz=(0.15, 0.0, -0.09)),
        material=metal_gray,
        name="hinge_stiffener",
    )
    shutter_leaf.visual(
        Box((1.04, 0.58, 0.06)),
        origin=Origin(xyz=(0.53, 0.0, -0.18)),
        material=painted_white,
        name="leaf_panel",
    )
    shutter_leaf.visual(
        Box((0.34, 0.44, 0.18)),
        origin=Origin(xyz=(1.22, 0.0, -0.22)),
        material=painted_white,
        name="crown_cap",
    )
    shutter_leaf.visual(
        Box((0.86, 0.06, 0.12)),
        origin=Origin(xyz=(0.57, 0.23, -0.15)),
        material=metal_gray,
        name="side_rail_left",
    )
    shutter_leaf.visual(
        Box((0.86, 0.06, 0.12)),
        origin=Origin(xyz=(0.57, -0.23, -0.15)),
        material=metal_gray,
        name="side_rail_right",
    )
    shutter_leaf.inertial = Inertial.from_geometry(
        Box((1.72, 1.12, 0.62)),
        mass=420.0,
        origin=Origin(xyz=(0.88, 0.0, -0.26)),
    )

    model.articulation(
        "base_to_dome",
        ArticulationType.CONTINUOUS,
        parent=base_ring,
        child=dome_shell,
        origin=Origin(xyz=(0.0, 0.0, ROTARY_AXIS_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=50000.0, velocity=0.25),
    )
    model.articulation(
        "dome_to_shutter",
        ArticulationType.REVOLUTE,
        parent=dome_shell,
        child=shutter_leaf,
        origin=Origin(xyz=(0.78, 0.0, 3.12)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1200.0,
            velocity=0.60,
            lower=0.0,
            upper=1.15,
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
    dome_joint = object_model.get_articulation("base_to_dome")
    shutter_joint = object_model.get_articulation("dome_to_shutter")

    with ctx.pose({dome_joint: 0.0, shutter_joint: 0.0}):
        ctx.expect_gap(
            dome_shell,
            base_ring,
            axis="z",
            max_penetration=0.001,
            max_gap=0.020,
            name="rotary skirt sits just above the bearing ring",
        )
        ctx.expect_overlap(
            dome_shell,
            base_ring,
            axes="xy",
            min_overlap=5.80,
            name="dome shell remains centered over the support ring",
        )

    with ctx.pose({shutter_joint: 0.0}):
        closed_aabb = ctx.part_world_aabb(shutter_leaf)
    with ctx.pose({shutter_joint: 1.0}):
        open_aabb = ctx.part_world_aabb(shutter_leaf)

    ctx.check(
        "shutter leaf lifts upward when opened",
        closed_aabb is not None
        and open_aabb is not None
        and open_aabb[1][2] > closed_aabb[1][2] + 0.55,
        details=f"closed_aabb={closed_aabb}, open_aabb={open_aabb}",
    )

    ctx.check(
        "continuous dome rotation is about the vertical axis",
        tuple(round(value, 6) for value in dome_joint.axis) == (0.0, 0.0, 1.0),
        details=f"axis={dome_joint.axis}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
