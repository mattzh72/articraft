from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


STEEL_YELLOW = Material("painted_safety_yellow", color=(0.96, 0.76, 0.08, 1.0))
DARK_STEEL = Material("dark_steel", color=(0.10, 0.11, 0.12, 1.0))
RUBBER = Material("black_rubber", color=(0.02, 0.02, 0.018, 1.0))
GALVANIZED = Material("galvanized_steel", color=(0.62, 0.64, 0.63, 1.0))
COUNTERWEIGHT = Material("counterweight_concrete", color=(0.34, 0.35, 0.34, 1.0))
HOOK_RED = Material("hook_red", color=(0.78, 0.06, 0.035, 1.0))


def _bar_origin_and_size(p1, p2, thickness: float):
    """Return a box visual pose whose local X axis runs between two points."""
    x1, y1, z1 = p1
    x2, y2, z2 = p2
    dx, dy, dz = x2 - x1, y2 - y1, z2 - z1
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    yaw = math.atan2(dy, dx)
    pitch = -math.asin(dz / length)
    center = ((x1 + x2) * 0.5, (y1 + y2) * 0.5, (z1 + z2) * 0.5)
    return Origin(xyz=center, rpy=(0.0, pitch, yaw)), (length, thickness, thickness)


def _add_bar(part, p1, p2, thickness: float, material, name: str) -> None:
    origin, size = _bar_origin_and_size(p1, p2, thickness)
    part.visual(Box(size), origin=origin, material=material, name=name)


def _yaw_point(origin, yaw: float, local):
    ox, oy, oz = origin
    x, y, z = local
    c = math.cos(yaw)
    s = math.sin(yaw)
    return (ox + c * x - s * y, oy + s * x + c * y, oz + z)


def _add_yaw_box(part, frame_origin, yaw: float, local_center, size, material, name: str) -> None:
    part.visual(
        Box(size),
        origin=Origin(xyz=_yaw_point(frame_origin, yaw, local_center), rpy=(0.0, 0.0, yaw)),
        material=material,
        name=name,
    )


def _add_lattice_mast(mast, height: float = 4.6, width: float = 0.58) -> None:
    half = width * 0.5
    panel = 0.72
    chord_t = 0.075
    brace_t = 0.045

    mast.visual(
        Box((0.74, 0.74, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
        material=STEEL_YELLOW,
        name="mast_foot_plate",
    )

    for sx in (-1.0, 1.0):
        for sy in (-1.0, 1.0):
            mast.visual(
                Box((chord_t, chord_t, height - 0.08)),
                origin=Origin(xyz=(sx * half, sy * half, height * 0.5 + 0.04)),
                material=STEEL_YELLOW,
                name=f"corner_chord_{sx:+.0f}_{sy:+.0f}",
            )

    levels = [0.08]
    z = panel
    while z < height - 0.12:
        levels.append(z)
        z += panel
    levels.append(height - 0.06)

    for li, z in enumerate(levels):
        corners = [
            (-half, -half, z),
            (half, -half, z),
            (half, half, z),
            (-half, half, z),
        ]
        for edge_i in range(4):
            _add_bar(
                mast,
                corners[edge_i],
                corners[(edge_i + 1) % 4],
                brace_t,
                STEEL_YELLOW,
                f"mast_ring_{li}_{edge_i}",
            )

    for li, (z0, z1) in enumerate(zip(levels[:-1], levels[1:])):
        # Cross-brace every face with alternating diagonals so the mast reads as
        # a welded square lattice rather than a simple solid tower.
        _add_bar(mast, (-half, -half, z0), (half, -half, z1), brace_t, STEEL_YELLOW, f"brace_front_{li}_a")
        _add_bar(mast, (half, -half, z0), (-half, -half, z1), brace_t, STEEL_YELLOW, f"brace_front_{li}_b")
        _add_bar(mast, (-half, half, z0), (half, half, z1), brace_t, STEEL_YELLOW, f"brace_rear_{li}_a")
        _add_bar(mast, (half, half, z0), (-half, half, z1), brace_t, STEEL_YELLOW, f"brace_rear_{li}_b")
        _add_bar(mast, (-half, -half, z0), (-half, half, z1), brace_t, STEEL_YELLOW, f"brace_side_0_{li}_a")
        _add_bar(mast, (-half, half, z0), (-half, -half, z1), brace_t, STEEL_YELLOW, f"brace_side_0_{li}_b")
        _add_bar(mast, (half, -half, z0), (half, half, z1), brace_t, STEEL_YELLOW, f"brace_side_1_{li}_a")
        _add_bar(mast, (half, half, z0), (half, -half, z1), brace_t, STEEL_YELLOW, f"brace_side_1_{li}_b")

    mast.visual(
        Box((0.66, 0.66, 0.055)),
        origin=Origin(xyz=(0.0, 0.0, height - 0.0275)),
        material=STEEL_YELLOW,
        name="mast_top_bearing_plate",
    )


def _add_jib_truss(jib) -> None:
    lower_z = 0.22
    top_z = 0.82
    rail_y = 0.23
    jib_len = 4.75
    brace_t = 0.045

    jib.visual(
        Cylinder(radius=0.42, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
        material=GALVANIZED,
        name="slewing_ring",
    )
    jib.visual(
        Box((0.88, 0.58, 0.10)),
        origin=Origin(xyz=(0.22, 0.0, 0.17)),
        material=STEEL_YELLOW,
        name="upper_carriage_frame",
    )

    _add_bar(jib, (0.12, -rail_y, lower_z), (jib_len, -rail_y, lower_z), 0.060, STEEL_YELLOW, "jib_lower_rail_0")
    _add_bar(jib, (0.12, rail_y, lower_z), (jib_len, rail_y, lower_z), 0.060, STEEL_YELLOW, "jib_lower_rail_1")
    _add_bar(jib, (0.18, 0.0, top_z), (jib_len, 0.0, 0.64), 0.060, STEEL_YELLOW, "jib_top_chord")

    panel_count = 8
    for i in range(panel_count):
        x0 = 0.25 + (jib_len - 0.45) * i / panel_count
        x1 = 0.25 + (jib_len - 0.45) * (i + 1) / panel_count
        z_top_0 = top_z + (0.64 - top_z) * i / panel_count
        z_top_1 = top_z + (0.64 - top_z) * (i + 1) / panel_count
        for sy, side_name in ((-rail_y, "front"), (rail_y, "rear")):
            _add_bar(jib, (x0, sy, lower_z), (x1, 0.0, z_top_1), brace_t, STEEL_YELLOW, f"jib_{side_name}_diag_{i}_a")
            _add_bar(jib, (x1, sy, lower_z), (x0, 0.0, z_top_0), brace_t, STEEL_YELLOW, f"jib_{side_name}_diag_{i}_b")

    # Short counter-jib with visible ballast, so the slewing top has the
    # silhouette and balance of a real mobile tower crane.
    _add_bar(jib, (-1.55, -0.20, lower_z), (0.12, -0.20, lower_z), 0.055, STEEL_YELLOW, "counterjib_rail_0")
    _add_bar(jib, (-1.55, 0.20, lower_z), (0.12, 0.20, lower_z), 0.055, STEEL_YELLOW, "counterjib_rail_1")
    _add_bar(jib, (-1.55, 0.0, 0.62), (0.18, 0.0, top_z), 0.055, STEEL_YELLOW, "counterjib_top_chord")
    for i in range(3):
        x = -1.42 + i * 0.26
        jib.visual(
            Box((0.22, 0.46, 0.425)),
            origin=Origin(xyz=(x, 0.0, -0.02)),
            material=COUNTERWEIGHT,
            name=f"counterweight_{i}",
        )

    jib.visual(
        Cylinder(radius=0.09, length=0.42),
        origin=Origin(xyz=(jib_len + 0.03, 0.0, lower_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=GALVANIZED,
        name="tip_sheave",
    )


def _add_trolley_details(trolley) -> None:
    trolley.visual(
        Box((0.36, 0.28, 0.09)),
        origin=Origin(xyz=(0.0, 0.0, -0.30)),
        material=DARK_STEEL,
        name="trolley_crosshead",
    )
    trolley.visual(
        Box((0.34, 0.70, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, -0.255)),
        material=DARK_STEEL,
        name="lower_trolley_frame",
    )
    for sx, suffix in ((-0.11, "0"), (0.11, "1")):
        trolley.visual(
            Box((0.055, 0.70, 0.052)),
            origin=Origin(xyz=(sx, 0.0, -0.082)),
            material=DARK_STEEL,
            name=f"wheel_axle_beam_{suffix}",
        )
    for sy, suffix in ((-1.0, "0"), (1.0, "1")):
        trolley.visual(
            Box((0.36, 0.045, 0.24)),
            origin=Origin(xyz=(0.0, sy * 0.3225, -0.155)),
            material=DARK_STEEL,
            name=f"trolley_side_plate_{suffix}",
        )
        for sx in (-0.11, 0.11):
            trolley.visual(
                Cylinder(radius=0.052, length=0.040),
                origin=Origin(xyz=(sx, sy * 0.280, -0.082), rpy=(math.pi / 2.0, 0.0, 0.0)),
                material=GALVANIZED,
                name=f"rail_wheel_{suffix}_{'a' if sx < 0 else 'b'}",
            )

    trolley.visual(
        Cylinder(radius=0.014, length=1.04),
        origin=Origin(xyz=(0.0, 0.0, -0.86)),
        material=GALVANIZED,
        name="hoist_line",
    )
    trolley.visual(
        Box((0.26, 0.22, 0.22)),
        origin=Origin(xyz=(0.0, 0.0, -1.48)),
        material=STEEL_YELLOW,
        name="hook_block",
    )
    trolley.visual(
        Cylinder(radius=0.055, length=0.27),
        origin=Origin(xyz=(0.0, 0.0, -1.48), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=DARK_STEEL,
        name="block_sheave",
    )
    trolley.visual(
        Cylinder(radius=0.026, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, -1.67)),
        material=HOOK_RED,
        name="hook_shank",
    )
    hook_mesh = mesh_from_geometry(
        tube_from_spline_points(
            [
                (0.0, 0.0, -1.74),
                (0.10, 0.0, -1.81),
                (0.11, 0.0, -1.95),
                (0.00, 0.0, -2.06),
                (-0.13, 0.0, -1.97),
            ],
            radius=0.024,
            samples_per_segment=14,
            radial_segments=16,
            cap_ends=True,
        ),
        "red_cargo_hook",
    )
    trolley.visual(hook_mesh, material=HOOK_RED, name="cargo_hook")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mobile_tower_crane")

    base = model.part("carrier_base")
    base.visual(
        Box((1.35, 1.08, 0.18)),
        origin=Origin(xyz=(0.0, 0.0, 0.25)),
        material=DARK_STEEL,
        name="carrier_chassis",
    )
    base.visual(
        Cylinder(radius=0.44, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, 0.63)),
        material=GALVANIZED,
        name="lower_turntable",
    )
    base.visual(
        Box((0.92, 0.92, 0.09)),
        origin=Origin(xyz=(0.0, 0.0, 0.535)),
        material=STEEL_YELLOW,
        name="mast_socket",
    )

    for x in (-0.43, 0.43):
        base.visual(
            Cylinder(radius=0.045, length=1.48),
            origin=Origin(xyz=(x, 0.0, 0.16), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=GALVANIZED,
            name=f"wheel_axle_{'front' if x > 0 else 'rear'}",
        )
        for y in (-0.68, 0.68):
            base.visual(
                Cylinder(radius=0.18, length=0.12),
                origin=Origin(xyz=(x, y, 0.16), rpy=(math.pi / 2.0, 0.0, 0.0)),
                material=RUBBER,
                name=f"transport_wheel_{x:+.0f}_{y:+.0f}",
            )

    outrigger_exit_radius = 0.58
    outrigger_z = 0.41
    outrigger_yaws = (math.pi / 4.0, 3.0 * math.pi / 4.0, -3.0 * math.pi / 4.0, -math.pi / 4.0)
    for i, yaw in enumerate(outrigger_yaws):
        exit_origin = (
            outrigger_exit_radius * math.cos(yaw),
            outrigger_exit_radius * math.sin(yaw),
            outrigger_z,
        )
        _add_yaw_box(base, exit_origin, yaw, (0.30, 0.0, 0.08), (0.90, 0.24, 0.04), STEEL_YELLOW, f"outrigger_sleeve_{i}_top")
        _add_yaw_box(base, exit_origin, yaw, (0.30, 0.0, -0.08), (0.90, 0.24, 0.04), STEEL_YELLOW, f"outrigger_sleeve_{i}_bottom")
        _add_yaw_box(base, exit_origin, yaw, (0.30, 0.105, 0.0), (0.90, 0.04, 0.16), STEEL_YELLOW, f"outrigger_sleeve_{i}_side_0")
        _add_yaw_box(base, exit_origin, yaw, (0.30, -0.105, 0.0), (0.90, 0.04, 0.16), STEEL_YELLOW, f"outrigger_sleeve_{i}_side_1")

    outrigger_parts = []
    for i, yaw in enumerate(outrigger_yaws):
        leg = model.part(f"outrigger_{i}")
        leg.visual(
            Box((1.40, 0.16, 0.12)),
            origin=Origin(xyz=(0.50, 0.0, 0.0)),
            material=STEEL_YELLOW,
            name="telescoping_beam",
        )
        leg.visual(
            Cylinder(radius=0.052, length=0.34),
            origin=Origin(xyz=(0.98, 0.0, -0.22)),
            material=GALVANIZED,
            name="screw_jack",
        )
        leg.visual(
            Cylinder(radius=0.18, length=0.060),
            origin=Origin(xyz=(0.98, 0.0, -0.38)),
            material=DARK_STEEL,
            name="ground_pad",
        )
        model.articulation(
            f"base_to_outrigger_{i}",
            ArticulationType.PRISMATIC,
            parent=base,
            child=leg,
            origin=Origin(
                xyz=(
                    outrigger_exit_radius * math.cos(yaw),
                    outrigger_exit_radius * math.sin(yaw),
                    outrigger_z,
                ),
                rpy=(0.0, 0.0, yaw),
            ),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=250.0, velocity=0.18, lower=0.0, upper=0.78),
        )
        outrigger_parts.append(leg)

    mast = model.part("mast")
    _add_lattice_mast(mast)
    model.articulation(
        "base_to_mast",
        ArticulationType.FIXED,
        parent=base,
        child=mast,
        origin=Origin(xyz=(0.0, 0.0, 0.72)),
    )

    jib = model.part("jib")
    _add_jib_truss(jib)
    slewing = model.articulation(
        "mast_to_jib",
        ArticulationType.REVOLUTE,
        parent=mast,
        child=jib,
        origin=Origin(xyz=(0.0, 0.0, 4.6)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=600.0, velocity=0.35, lower=-math.pi, upper=math.pi),
    )
    slewing.meta["qc_samples"] = [-1.2, 0.0, 1.2]

    trolley = model.part("trolley")
    _add_trolley_details(trolley)
    trolley_joint = model.articulation(
        "jib_to_trolley",
        ArticulationType.PRISMATIC,
        parent=jib,
        child=trolley,
        origin=Origin(xyz=(1.00, 0.0, 0.22)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.55, lower=0.0, upper=3.55),
    )
    trolley_joint.meta["qc_samples"] = [0.0, 1.8, 3.55]

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("carrier_base")
    mast = object_model.get_part("mast")
    jib = object_model.get_part("jib")
    trolley = object_model.get_part("trolley")
    slewing = object_model.get_articulation("mast_to_jib")
    trolley_slide = object_model.get_articulation("jib_to_trolley")
    outriggers = [object_model.get_part(f"outrigger_{i}") for i in range(4)]
    outrigger_slides = [object_model.get_articulation(f"base_to_outrigger_{i}") for i in range(4)]

    ctx.expect_contact(
        mast,
        base,
        elem_a="mast_foot_plate",
        elem_b="lower_turntable",
        contact_tol=0.002,
        name="mast is seated on base turntable",
    )
    ctx.expect_contact(
        jib,
        mast,
        elem_a="slewing_ring",
        elem_b="mast_top_bearing_plate",
        contact_tol=0.002,
        name="jib slewing ring sits on mast top",
    )
    ctx.expect_overlap(
        trolley,
        jib,
        axes="x",
        elem_a="trolley_crosshead",
        elem_b="jib_lower_rail_0",
        min_overlap=0.20,
        name="trolley is carried on the jib rails",
    )

    rest_positions = [ctx.part_world_position(part) for part in outriggers]
    rest_trolley = ctx.part_world_position(trolley)
    rest_jib = ctx.part_world_position(jib)
    with ctx.pose({joint: 0.78 for joint in outrigger_slides}):
        extended_positions = [ctx.part_world_position(part) for part in outriggers]
    with ctx.pose({trolley_slide: 3.55}):
        ctx.expect_overlap(
            trolley,
            jib,
            axes="x",
            elem_a="trolley_crosshead",
            elem_b="jib_lower_rail_0",
            min_overlap=0.20,
            name="extended trolley remains under the jib rail",
        )
        extended_trolley = ctx.part_world_position(trolley)
    with ctx.pose({slewing: 1.0}):
        slewed_jib = ctx.part_world_position(jib)

    for i, (rest, extended) in enumerate(zip(rest_positions, extended_positions)):
        ctx.check(
            f"outrigger {i} extends outward",
            rest is not None
            and extended is not None
            and math.hypot(extended[0], extended[1]) > math.hypot(rest[0], rest[1]) + 0.70,
            details=f"rest={rest}, extended={extended}",
        )

    ctx.check(
        "trolley translates toward jib tip",
        rest_trolley is not None and extended_trolley is not None and extended_trolley[0] > rest_trolley[0] + 3.4,
        details=f"rest={rest_trolley}, extended={extended_trolley}",
    )
    ctx.check(
        "jib slews about vertical mast axis",
        rest_jib is not None and slewed_jib is not None and abs(slewed_jib[0] - rest_jib[0]) < 1e-6,
        details=f"rest={rest_jib}, slewed={slewed_jib}",
    )

    return ctx.report()


object_model = build_object_model()
