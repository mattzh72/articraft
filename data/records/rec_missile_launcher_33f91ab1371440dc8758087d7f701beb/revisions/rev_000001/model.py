from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    LatheGeometry,
    Material,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _midpoint(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_cylinder(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_member(
    part,
    a: tuple[float, float, float],
    b: tuple[float, float, float],
    radius: float,
    material: Material,
    *,
    name: str | None = None,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _open_tube_mesh(name: str, *, outer_radius: float, inner_radius: float, length: float):
    shell = LatheGeometry.from_shell_profiles(
        [(outer_radius, -length * 0.5), (outer_radius, length * 0.5)],
        [(inner_radius, -length * 0.5), (inner_radius, length * 0.5)],
        segments=56,
        start_cap="flat",
        end_cap="flat",
        lip_samples=4,
    )
    return mesh_from_geometry(shell, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="elevating_missile_launcher")

    olive = model.material("matte_olive_drab", rgba=(0.22, 0.29, 0.16, 1.0))
    dark_olive = model.material("dark_olive", rgba=(0.13, 0.18, 0.10, 1.0))
    black = model.material("matte_black", rgba=(0.02, 0.025, 0.02, 1.0))
    graphite = model.material("gunmetal_graphite", rgba=(0.18, 0.19, 0.18, 1.0))
    steel = model.material("worn_steel", rgba=(0.52, 0.54, 0.50, 1.0))
    rubber = model.material("black_rubber", rgba=(0.035, 0.035, 0.032, 1.0))
    amber = model.material("smoked_amber_lens", rgba=(0.95, 0.58, 0.16, 0.65))
    white = model.material("stenciled_white", rgba=(0.86, 0.88, 0.80, 1.0))

    base = model.part("support_base")
    base.visual(
        Box((1.90, 1.34, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
        material=dark_olive,
        name="ground_plate",
    )
    for y in (-0.53, 0.53):
        base.visual(
            Box((1.92, 0.12, 0.13)),
            origin=Origin(xyz=(0.0, y, 0.155)),
            material=olive,
            name=f"skid_{0 if y < 0 else 1}",
        )
    for x in (-0.64, 0.64):
        base.visual(
            Box((0.16, 1.90, 0.10)),
            origin=Origin(xyz=(x, 0.0, 0.18)),
            material=olive,
            name=f"cross_beam_{0 if x < 0 else 1}",
        )
    for x in (-0.64, 0.64):
        for y in (-0.94, 0.94):
            base.visual(
                Box((0.34, 0.26, 0.055)),
                origin=Origin(xyz=(x, y, 0.065)),
                material=rubber,
                name=f"outrigger_pad_{0 if x < 0 else 1}_{0 if y < 0 else 1}",
            )
            base.visual(
                Cylinder(radius=0.028, length=0.12),
                origin=Origin(xyz=(x, y, 0.145)),
                material=steel,
                name=f"leveling_jack_{0 if x < 0 else 1}_{0 if y < 0 else 1}",
            )

    base.visual(Cylinder(radius=0.43, length=0.08), origin=Origin(xyz=(0.0, 0.0, 0.25)), material=graphite, name="slew_ring_lower")
    base.visual(Cylinder(radius=0.34, length=0.10), origin=Origin(xyz=(0.0, 0.0, 0.34)), material=olive, name="slew_ring_upper")
    base.visual(Box((0.54, 0.42, 0.58)), origin=Origin(xyz=(0.0, 0.0, 0.66)), material=olive, name="armored_pedestal")
    base.visual(Cylinder(radius=0.30, length=0.18), origin=Origin(xyz=(0.0, 0.0, 0.165)), material=dark_olive, name="center_riser")
    base.visual(Box((0.74, 0.50, 0.12)), origin=Origin(xyz=(0.0, 0.0, 0.985)), material=dark_olive, name="top_saddle")
    base.visual(Box((0.46, 0.20, 0.20)), origin=Origin(xyz=(-0.40, 0.0, 0.88)), material=graphite, name="rear_counterweight")

    # The forked trunnion support is the static half of the elevation mechanism.
    for y, suffix in ((-0.66, "0"), (0.66, "1")):
        base.visual(
            Box((0.34, 0.14, 0.58)),
            origin=Origin(xyz=(0.0, y, 1.17)),
            material=olive,
            name=f"yoke_cheek_{suffix}",
        )
        base.visual(
            Box((0.40, 0.16, 0.08)),
            origin=Origin(xyz=(-0.02, y, 1.46)),
            material=dark_olive,
            name=f"yoke_cap_{suffix}",
        )
        base.visual(
            Cylinder(radius=0.175, length=0.080),
            origin=Origin(xyz=(0.0, 0.555 if y > 0 else -0.555, 1.14), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=graphite,
            name=f"pivot_bushing_{suffix}",
        )
        base.visual(
            Cylinder(radius=0.085, length=0.026),
            origin=Origin(xyz=(0.0, 0.533 if y > 0 else -0.533, 1.14), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=steel,
            name=f"bearing_face_{suffix}",
        )
    base.visual(Box((0.28, 1.10, 0.12)), origin=Origin(xyz=(0.0, 0.0, 0.93)), material=olive, name="lower_yoke_bridge")

    # Side-mounted elevation drive details fixed to the base.
    base.visual(
        Cylinder(radius=0.075, length=0.18),
        origin=Origin(xyz=(-0.24, -0.78, 1.02), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=graphite,
        name="elevation_motor",
    )
    base.visual(
        Cylinder(radius=0.045, length=0.08),
        origin=Origin(xyz=(-0.17, -0.78, 1.11), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="drive_pinion",
    )
    base.visual(Box((0.34, 0.08, 0.18)), origin=Origin(xyz=(-0.37, -0.69, 0.88)), material=graphite, name="gearbox_case")
    base.visual(Box((0.12, 0.18, 0.12)), origin=Origin(xyz=(0.23, -0.28, 0.86)), material=olive, name="service_bracket")
    base.visual(Box((0.30, 0.045, 0.09)), origin=Origin(xyz=(0.30, -0.39, 0.86)), material=graphite, name="service_box")
    base.visual(Cylinder(radius=0.018, length=0.08), origin=Origin(xyz=(0.21, -0.42, 0.91), rpy=(0.0, math.pi / 2.0, 0.0)), material=amber, name="status_lamp")

    for x in (-0.22, 0.0, 0.22):
        for y in (-0.18, 0.18):
            base.visual(
                Cylinder(radius=0.018, length=0.022),
                origin=Origin(xyz=(x, y, 1.050)),
                material=steel,
                name=f"saddle_bolt_{int((x + 0.22) / 0.22)}_{0 if y < 0 else 1}",
            )
    base.inertial = Inertial.from_geometry(Box((2.0, 2.1, 1.5)), mass=850.0, origin=Origin(xyz=(0.0, 0.0, 0.65)))

    launcher = model.part("launcher_frame")
    tube_meshes = {}
    tube_length = 2.70
    tube_outer = 0.105
    tube_inner = 0.082
    tube_center_x = 0.90
    y_positions = (-0.36, -0.12, 0.12, 0.36)
    z_positions = (0.16, 0.40)
    for row, z in enumerate(z_positions):
        for col, y in enumerate(y_positions):
            mesh_name = f"launch_tube_mesh_{row}_{col}"
            tube_meshes[(row, col)] = _open_tube_mesh(mesh_name, outer_radius=tube_outer, inner_radius=tube_inner, length=tube_length)
            launcher.visual(
                tube_meshes[(row, col)],
                origin=Origin(xyz=(tube_center_x, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
                material=olive,
                name=f"tube_{row}_{col}",
            )
            launcher.visual(
                Cylinder(radius=tube_inner * 0.94, length=0.018),
                origin=Origin(xyz=(tube_center_x + tube_length * 0.5 - 0.09, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
                material=black,
                name=f"dark_bore_{row}_{col}",
            )

    # End clamps, side rails, and armor panels deliberately intersect the tube shells
    # within this rigid part so the launcher reads as one welded/captured assembly.
    launcher.visual(Box((0.12, 1.05, 0.64)), origin=Origin(xyz=(-0.40, 0.0, 0.28)), material=dark_olive, name="rear_tube_clamp")
    launcher.visual(Box((0.10, 1.05, 0.64)), origin=Origin(xyz=(2.17, 0.0, 0.28)), material=dark_olive, name="front_tube_clamp")
    launcher.visual(Box((2.55, 0.055, 0.62)), origin=Origin(xyz=(0.88, -0.485, 0.28)), material=olive, name="side_plate_0")
    launcher.visual(Box((2.55, 0.055, 0.62)), origin=Origin(xyz=(0.88, 0.485, 0.28)), material=olive, name="side_plate_1")
    launcher.visual(Box((2.52, 1.02, 0.045)), origin=Origin(xyz=(0.88, 0.0, 0.585)), material=olive, name="top_tie_plate")
    launcher.visual(Box((2.52, 1.02, 0.045)), origin=Origin(xyz=(0.88, 0.0, 0.035)), material=olive, name="bottom_tie_plate")
    launcher.visual(Box((0.60, 1.00, 0.09)), origin=Origin(xyz=(-0.12, 0.0, 0.00)), material=dark_olive, name="trunnion_saddle")
    launcher.visual(
        Cylinder(radius=0.082, length=1.15),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="trunnion_pin",
    )
    launcher.visual(
        Cylinder(radius=0.135, length=0.055),
        origin=Origin(xyz=(0.0, -0.475, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=graphite,
        name="trunnion_hub_0",
    )
    launcher.visual(
        Cylinder(radius=0.135, length=0.055),
        origin=Origin(xyz=(0.0, 0.475, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=graphite,
        name="trunnion_hub_1",
    )
    _add_member(launcher, (-0.03, -0.46, 0.0), (0.66, -0.51, -0.035), 0.035, graphite, name="lower_side_rail_0")
    _add_member(launcher, (-0.03, 0.46, 0.0), (0.66, 0.51, -0.035), 0.035, graphite, name="lower_side_rail_1")
    _add_member(launcher, (-0.03, -0.46, 0.0), (0.72, -0.51, 0.585), 0.028, graphite, name="diagonal_brace_0")
    _add_member(launcher, (-0.03, 0.46, 0.0), (0.72, 0.51, 0.585), 0.028, graphite, name="diagonal_brace_1")
    _add_member(launcher, (0.12, -0.49, 0.03), (2.02, -0.49, 0.03), 0.018, steel, name="cable_conduit_0")
    _add_member(launcher, (0.12, 0.49, 0.03), (2.02, 0.49, 0.03), 0.018, steel, name="cable_conduit_1")

    # A toothed elevation quadrant on one cheek and a small sensor package on top
    # add the heavy military-hardware character without adding extra mechanisms.
    launcher.visual(Cylinder(radius=0.225, length=0.028), origin=Origin(xyz=(-0.06, -0.490, 0.11), rpy=(math.pi / 2.0, 0.0, 0.0)), material=graphite, name="elevation_quadrant")
    for i in range(11):
        angle = -0.75 + i * 0.15
        launcher.visual(
            Box((0.025, 0.022, 0.055)),
            origin=Origin(
                xyz=(-0.06 + 0.235 * math.cos(angle), -0.506, 0.11 + 0.235 * math.sin(angle)),
                rpy=(0.0, -angle, 0.0),
            ),
            material=steel,
            name=f"quadrant_tooth_{i}",
        )
    launcher.visual(Box((0.36, 0.20, 0.14)), origin=Origin(xyz=(0.32, 0.0, 0.74)), material=graphite, name="sight_housing")
    launcher.visual(Box((0.16, 0.18, 0.09)), origin=Origin(xyz=(0.54, 0.0, 0.77)), material=black, name="optic_window")
    launcher.visual(Box((0.20, 0.18, 0.12)), origin=Origin(xyz=(0.28, 0.0, 0.655)), material=graphite, name="sight_pedestal")
    launcher.visual(Box((0.38, 0.008, 0.045)), origin=Origin(xyz=(1.76, -0.514, 0.53)), material=white, name="stencil_bar_0")
    launcher.visual(Box((0.38, 0.008, 0.045)), origin=Origin(xyz=(1.76, 0.514, 0.53)), material=white, name="stencil_bar_1")
    launcher.inertial = Inertial.from_geometry(Box((2.85, 1.20, 0.90)), mass=520.0, origin=Origin(xyz=(0.90, 0.0, 0.25)))

    model.articulation(
        "elevation",
        ArticulationType.REVOLUTE,
        parent=base,
        child=launcher,
        origin=Origin(xyz=(0.0, 0.0, 1.14)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=2200.0, velocity=0.35, lower=0.0, upper=0.95),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("support_base")
    launcher = object_model.get_part("launcher_frame")
    elevation = object_model.get_articulation("elevation")

    for suffix in ("0", "1"):
        ctx.allow_overlap(
            base,
            launcher,
            elem_a=f"pivot_bushing_{suffix}",
            elem_b="trunnion_pin",
            reason="The launcher trunnion pin is intentionally captured inside the fork bushing.",
        )
        ctx.allow_overlap(
            base,
            launcher,
            elem_a=f"bearing_face_{suffix}",
            elem_b="trunnion_pin",
            reason="The visible bearing face is a seated bushing cap around the captured trunnion pin.",
        )
        ctx.expect_overlap(
            base,
            launcher,
            axes="y",
            min_overlap=0.040,
            elem_a=f"pivot_bushing_{suffix}",
            elem_b="trunnion_pin",
            name=f"trunnion pin captured in bushing {suffix}",
        )
        ctx.expect_overlap(
            base,
            launcher,
            axes="y",
            min_overlap=0.015,
            elem_a=f"bearing_face_{suffix}",
            elem_b="trunnion_pin",
            name=f"trunnion pin passes through bearing face {suffix}",
        )

    ctx.expect_overlap(
        base,
        launcher,
        axes="y",
        min_overlap=0.040,
        elem_a="pivot_bushing_0",
        elem_b="trunnion_pin",
        name="trunnion centered between fork cheeks",
    )

    with ctx.pose({elevation: 0.0}):
        rest_front = ctx.part_element_world_aabb(launcher, elem="front_tube_clamp")
        rest_pivot = ctx.part_world_position(launcher)
        ctx.expect_gap(
            launcher,
            base,
            axis="z",
            min_gap=-0.05,
            max_gap=0.80,
            positive_elem="bottom_tie_plate",
            negative_elem="top_saddle",
            name="launcher frame clears saddle at rest",
        )

    with ctx.pose({elevation: 0.95}):
        raised_front = ctx.part_element_world_aabb(launcher, elem="front_tube_clamp")
        raised_pivot = ctx.part_world_position(launcher)

    rest_front_z = (rest_front[0][2] + rest_front[1][2]) * 0.5 if rest_front else None
    raised_front_z = (raised_front[0][2] + raised_front[1][2]) * 0.5 if raised_front else None

    ctx.check(
        "elevation raises the tube muzzle end",
        rest_front_z is not None and raised_front_z is not None and raised_front_z > rest_front_z + 1.0,
        details=f"rest_front_z={rest_front_z}, raised_front_z={raised_front_z}",
    )
    ctx.check(
        "launcher rotates about fixed trunnion",
        rest_pivot is not None
        and raised_pivot is not None
        and math.sqrt(sum((raised_pivot[i] - rest_pivot[i]) ** 2 for i in range(3))) < 0.005,
        details=f"rest_pivot={rest_pivot}, raised_pivot={raised_pivot}",
    )

    return ctx.report()


object_model = build_object_model()
