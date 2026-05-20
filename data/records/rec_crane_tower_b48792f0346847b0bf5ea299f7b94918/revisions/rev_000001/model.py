from __future__ import annotations

from math import atan2, pi, sqrt

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _cylinder_between(
    part,
    p0: tuple[float, float, float],
    p1: tuple[float, float, float],
    radius: float,
    material: Material,
    name: str,
):
    """Add a round steel member whose local cylinder axis runs from p0 to p1."""

    vx, vy, vz = (p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2])
    length = sqrt(vx * vx + vy * vy + vz * vz)
    if length <= 1e-9:
        raise ValueError(f"zero-length member {name}")
    ux, uy, uz = (vx / length, vy / length, vz / length)
    yaw = atan2(uy, ux)
    pitch = atan2(sqrt(ux * ux + uy * uy), uz)
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(
            xyz=((p0[0] + p1[0]) * 0.5, (p0[1] + p1[1]) * 0.5, (p0[2] + p1[2]) * 0.5),
            rpy=(0.0, pitch, yaw),
        ),
        material=material,
        name=name,
    )


def _node(part, p: tuple[float, float, float], radius: float, material: Material, name: str):
    part.visual(Sphere(radius=radius), origin=Origin(xyz=p), material=material, name=name)


def _build_mast_lattice(part, yellow: Material, dark: Material):
    bottom_z = 0.50
    panel = 2.00
    panels = 9
    top_z = bottom_z + panels * panel
    half = 0.58
    corners = [(-half, -half), (half, -half), (half, half), (-half, half)]

    for i, (x, y) in enumerate(corners):
        _cylinder_between(part, (x, y, bottom_z), (x, y, top_z), 0.075, yellow, f"mast_post_{i}")

    for level in range(panels + 1):
        z = bottom_z + level * panel
        for i, (x, y) in enumerate(corners):
            _node(part, (x, y, z), 0.12, yellow, f"mast_node_{level}_{i}")
        for i in range(4):
            x0, y0 = corners[i]
            x1, y1 = corners[(i + 1) % 4]
            _cylinder_between(part, (x0, y0, z), (x1, y1, z), 0.045, yellow, f"mast_ring_{level}_{i}")

    for panel_i in range(panels):
        z0 = bottom_z + panel_i * panel
        z1 = z0 + panel
        # Four faces, with alternating X bracing so the tower reads as a true lattice.
        for face_i, (a0, a1, b0, b1) in enumerate(
            [
                ((-half, -half, z0), (half, -half, z0), (-half, -half, z1), (half, -half, z1)),
                ((half, -half, z0), (half, half, z0), (half, -half, z1), (half, half, z1)),
                ((half, half, z0), (-half, half, z0), (half, half, z1), (-half, half, z1)),
                ((-half, half, z0), (-half, -half, z0), (-half, half, z1), (-half, -half, z1)),
            ]
        ):
            if (panel_i + face_i) % 2 == 0:
                _cylinder_between(part, a0, b1, 0.034, yellow, f"mast_diag_{panel_i}_{face_i}_a")
                _cylinder_between(part, a1, b0, 0.034, yellow, f"mast_diag_{panel_i}_{face_i}_b")
            else:
                _cylinder_between(part, a0, b0, 0.034, yellow, f"mast_diag_{panel_i}_{face_i}_a")
                _cylinder_between(part, a1, b1, 0.034, yellow, f"mast_diag_{panel_i}_{face_i}_b")

    # Ladder and service platforms are attached directly to one face.
    for rung in range(16):
        z = bottom_z + 0.6 + rung * 1.05
        _cylinder_between(part, (-0.22, -half - 0.08, z), (0.22, -half - 0.08, z), 0.018, dark, f"ladder_rung_{rung}")
    _cylinder_between(part, (-0.22, -half - 0.08, bottom_z + 0.4), (-0.22, -half - 0.08, top_z - 0.7), 0.018, dark, "ladder_rail_0")
    _cylinder_between(part, (0.22, -half - 0.08, bottom_z + 0.4), (0.22, -half - 0.08, top_z - 0.7), 0.018, dark, "ladder_rail_1")
    for level in range(1, panels):
        z = bottom_z + level * panel
        _cylinder_between(part, (-0.22, -half - 0.08, z), (-0.22, -half, z), 0.016, dark, f"ladder_standoff_{level}_0")
        _cylinder_between(part, (0.22, -half - 0.08, z), (0.22, -half, z), 0.016, dark, f"ladder_standoff_{level}_1")


def _build_main_jib(part, yellow: Material, dark: Material):
    start_x = 0.70
    length = 20.0
    panels = 10
    dx = length / panels
    y0 = -0.38
    y1 = 0.38
    low_z = 0.78
    high_z = 1.95

    # Three principal chords make a triangular tower-crane jib.
    _cylinder_between(part, (start_x, y0, low_z), (start_x + length, y0, low_z), 0.055, yellow, "jib_lower_chord_0")
    _cylinder_between(part, (start_x, y1, low_z), (start_x + length, y1, low_z), 0.055, yellow, "jib_lower_chord_1")
    _cylinder_between(part, (start_x, 0.0, high_z), (start_x + length, 0.0, high_z), 0.060, yellow, "jib_top_chord")

    for i in range(panels + 1):
        x = start_x + i * dx
        for p_i, p in enumerate(((x, y0, low_z), (x, y1, low_z), (x, 0.0, high_z))):
            _node(part, p, 0.083, yellow, f"jib_node_{i}_{p_i}")
        _cylinder_between(part, (x, y0, low_z), (x, 0.0, high_z), 0.034, yellow, f"jib_web_a_{i}")
        _cylinder_between(part, (x, y1, low_z), (x, 0.0, high_z), 0.034, yellow, f"jib_web_b_{i}")
        _cylinder_between(part, (x, y0, low_z), (x, y1, low_z), 0.030, yellow, f"jib_cross_{i}")

    for i in range(panels):
        xa = start_x + i * dx
        xb = xa + dx
        if i % 2 == 0:
            _cylinder_between(part, (xa, y0, low_z), (xb, 0.0, high_z), 0.030, yellow, f"jib_diag_l_{i}")
            _cylinder_between(part, (xa, y1, low_z), (xb, 0.0, high_z), 0.030, yellow, f"jib_diag_r_{i}")
        else:
            _cylinder_between(part, (xb, y0, low_z), (xa, 0.0, high_z), 0.030, yellow, f"jib_diag_l_{i}")
            _cylinder_between(part, (xb, y1, low_z), (xa, 0.0, high_z), 0.030, yellow, f"jib_diag_r_{i}")

    # Rail stringers under the lower chords for the traveling trolley.
    _cylinder_between(part, (start_x + 0.8, -0.30, 0.55), (start_x + length - 0.9, -0.30, 0.55), 0.030, dark, "trolley_rail_0")
    _cylinder_between(part, (start_x + 0.8, 0.30, 0.55), (start_x + length - 0.9, 0.30, 0.55), 0.030, dark, "trolley_rail_1")
    for i in range(0, panels + 1, 2):
        x = start_x + i * dx
        _cylinder_between(part, (x, -0.30, 0.55), (x, -0.38, low_z), 0.020, dark, f"rail_hanger_{i}_0")
        _cylinder_between(part, (x, 0.30, 0.55), (x, 0.38, low_z), 0.020, dark, f"rail_hanger_{i}_1")


def _build_counter_jib(part, yellow: Material, dark: Material, ballast: Material):
    start_x = -0.65
    end_x = -7.2
    y0 = -0.42
    y1 = 0.42
    low_z = 0.80
    high_z = 1.65
    panels = 4
    dx = (end_x - start_x) / panels

    for y in (y0, y1):
        _cylinder_between(part, (start_x, y, low_z), (end_x, y, low_z), 0.060, yellow, f"counter_lower_{int(y > 0)}")
    _cylinder_between(part, (start_x, 0.0, high_z), (end_x, 0.0, high_z), 0.055, yellow, "counter_top")

    for i in range(panels + 1):
        x = start_x + i * dx
        _node(part, (x, y0, low_z), 0.08, yellow, f"counter_node_{i}_0")
        _node(part, (x, y1, low_z), 0.08, yellow, f"counter_node_{i}_1")
        _node(part, (x, 0.0, high_z), 0.08, yellow, f"counter_node_{i}_2")
        _cylinder_between(part, (x, y0, low_z), (x, 0.0, high_z), 0.032, yellow, f"counter_web_{i}_0")
        _cylinder_between(part, (x, y1, low_z), (x, 0.0, high_z), 0.032, yellow, f"counter_web_{i}_1")

    for i in range(panels):
        xa = start_x + i * dx
        xb = xa + dx
        _cylinder_between(part, (xa, y0, low_z), (xb, 0.0, high_z), 0.030, yellow, f"counter_diag_{i}_0")
        _cylinder_between(part, (xa, y1, low_z), (xb, 0.0, high_z), 0.030, yellow, f"counter_diag_{i}_1")

    # Counterweight stack and machinery deck bolted to the counter-jib end.
    for i, x in enumerate((-5.8, -6.5, -7.2)):
        part.visual(
            Box((0.56, 1.10, 0.82)),
            origin=Origin(xyz=(x, 0.0, 0.44)),
            material=ballast,
            name=f"ballast_block_{i}",
        )
    _cylinder_between(part, (-5.45, -0.48, 0.86), (-7.55, -0.48, 0.86), 0.025, dark, "ballast_tie_0")
    _cylinder_between(part, (-5.45, 0.48, 0.86), (-7.55, 0.48, 0.86), 0.025, dark, "ballast_tie_1")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tower_crane")

    yellow = model.material("crane_yellow", color=(1.0, 0.78, 0.06, 1.0))
    dark = model.material("dark_galvanized_steel", color=(0.08, 0.09, 0.09, 1.0))
    concrete = model.material("weathered_concrete", color=(0.56, 0.55, 0.51, 1.0))
    ballast = model.material("ballast_concrete", color=(0.42, 0.43, 0.42, 1.0))
    red = model.material("safety_red", color=(0.85, 0.05, 0.03, 1.0))
    glass = model.material("cab_glass_blue", color=(0.30, 0.55, 0.72, 0.72))
    cable = model.material("black_hoist_cable", color=(0.01, 0.01, 0.01, 1.0))

    tower = model.part("tower")
    tower.visual(Box((4.8, 4.8, 0.50)), origin=Origin(xyz=(0.0, 0.0, 0.25)), material=concrete, name="foundation_slab")
    tower.visual(Box((2.3, 2.3, 0.18)), origin=Origin(xyz=(0.0, 0.0, 0.59)), material=dark, name="base_frame")
    for i, (x, y) in enumerate(((-0.58, -0.58), (0.58, -0.58), (0.58, 0.58), (-0.58, 0.58))):
        tower.visual(Box((0.42, 0.42, 0.16)), origin=Origin(xyz=(x, y, 0.58)), material=dark, name=f"anchor_plate_{i}")
        tower.visual(Cylinder(radius=0.055, length=0.18), origin=Origin(xyz=(x + 0.12, y + 0.12, 0.75)), material=dark, name=f"anchor_bolt_{i}_0")
        tower.visual(Cylinder(radius=0.055, length=0.18), origin=Origin(xyz=(x - 0.12, y - 0.12, 0.75)), material=dark, name=f"anchor_bolt_{i}_1")
    _build_mast_lattice(tower, yellow, dark)
    tower.visual(Cylinder(radius=0.86, length=0.16), origin=Origin(xyz=(0.0, 0.0, 18.57)), material=dark, name="slew_bearing_base")
    tower.visual(Box((1.55, 1.55, 0.10)), origin=Origin(xyz=(0.0, 0.0, 18.49)), material=yellow, name="mast_top_cap")

    slewing = model.part("slewing_head")
    slewing.visual(Cylinder(radius=0.82, length=0.22), origin=Origin(xyz=(0.0, 0.0, 0.11)), material=dark, name="slew_bearing_ring")
    slewing.visual(Box((2.20, 1.80, 0.28)), origin=Origin(xyz=(0.0, 0.0, 0.36)), material=yellow, name="turntable_deck")
    slewing.visual(Box((1.25, 1.10, 0.80)), origin=Origin(xyz=(-0.15, 0.0, 0.82)), material=dark, name="machinery_house")
    slewing.visual(Box((0.72, 0.82, 0.66)), origin=Origin(xyz=(0.82, -1.02, 0.81)), material=yellow, name="operator_cab_frame")
    slewing.visual(Box((0.62, 0.05, 0.38)), origin=Origin(xyz=(0.82, -1.44, 0.85)), material=glass, name="operator_cab_window")
    _cylinder_between(slewing, (0.0, 0.0, 0.50), (0.0, 0.0, 3.10), 0.075, yellow, "cathead_post")
    _cylinder_between(slewing, (0.0, 0.0, 3.05), (2.9, 0.0, 1.95), 0.035, dark, "jib_suspension_0")
    _cylinder_between(slewing, (0.0, 0.0, 3.05), (11.0, 0.0, 1.95), 0.027, dark, "jib_suspension_1")
    _cylinder_between(slewing, (0.0, 0.0, 3.05), (-5.7, 0.0, 1.65), 0.035, dark, "counter_suspension")
    _build_main_jib(slewing, yellow, dark)
    _build_counter_jib(slewing, yellow, dark, ballast)
    # Deck rails and maintenance details.
    _cylinder_between(slewing, (-0.9, -0.87, 0.88), (0.95, -0.87, 0.88), 0.022, dark, "deck_guard_0")
    _cylinder_between(slewing, (-0.9, 0.87, 0.88), (0.95, 0.87, 0.88), 0.022, dark, "deck_guard_1")
    for i, x in enumerate((-0.80, 0.85)):
        _cylinder_between(slewing, (x, -0.87, 0.50), (x, -0.87, 0.88), 0.018, dark, f"deck_guard_post_{i}_0")
        _cylinder_between(slewing, (x, 0.87, 0.50), (x, 0.87, 0.88), 0.018, dark, f"deck_guard_post_{i}_1")
    slewing.visual(Cylinder(radius=0.22, length=0.70), origin=Origin(xyz=(-1.0, 0.0, 0.80), rpy=(pi / 2, 0.0, 0.0)), material=dark, name="winch_drum")
    slewing.visual(Sphere(radius=0.11), origin=Origin(xyz=(0.0, 0.0, 3.20)), material=red, name="aviation_light")

    trolley = model.part("trolley")
    trolley.visual(Box((0.68, 0.78, 0.18)), origin=Origin(xyz=(0.0, 0.0, -0.22)), material=dark, name="trolley_carriage")
    trolley.visual(Box((0.52, 0.10, 0.24)), origin=Origin(xyz=(0.0, -0.38, -0.16)), material=yellow, name="rail_yoke_0")
    trolley.visual(Box((0.52, 0.10, 0.24)), origin=Origin(xyz=(0.0, 0.38, -0.16)), material=yellow, name="rail_yoke_1")
    for i, x in enumerate((-0.24, 0.24)):
        trolley.visual(Cylinder(radius=0.055, length=0.08), origin=Origin(xyz=(x, -0.30, -0.084), rpy=(pi / 2, 0.0, 0.0)), material=dark, name=f"roller_{i}_0")
        trolley.visual(Cylinder(radius=0.055, length=0.08), origin=Origin(xyz=(x, 0.30, -0.084), rpy=(pi / 2, 0.0, 0.0)), material=dark, name=f"roller_{i}_1")
    _cylinder_between(trolley, (-0.22, 0.0, -0.10), (-0.22, 0.0, -0.644), 0.015, cable, "hoist_line_0")
    _cylinder_between(trolley, (0.22, 0.0, -0.10), (0.22, 0.0, -0.644), 0.015, cable, "hoist_line_1")
    trolley.visual(Box((0.70, 0.12, 0.10)), origin=Origin(xyz=(0.0, 0.0, -0.10)), material=dark, name="cable_fairlead")

    hook = model.part("hook_block")
    hook.visual(Box((0.36, 0.06, 0.46)), origin=Origin(xyz=(0.0, -0.20, 0.02)), material=red, name="side_plate_0")
    hook.visual(Box((0.36, 0.06, 0.46)), origin=Origin(xyz=(0.0, 0.20, 0.02)), material=red, name="side_plate_1")
    hook.visual(Cylinder(radius=0.16, length=0.06), origin=Origin(xyz=(0.0, 0.0, 0.08), rpy=(pi / 2, 0.0, 0.0)), material=dark, name="sheave_wheel")
    hook.visual(Cylinder(radius=0.040, length=0.52), origin=Origin(xyz=(0.0, 0.0, 0.08), rpy=(pi / 2, 0.0, 0.0)), material=dark, name="sheave_axle")
    hook.visual(Box((0.10, 0.46, 0.10)), origin=Origin(xyz=(0.0, 0.0, -0.23)), material=dark, name="hook_swivel_block")
    _cylinder_between(hook, (-0.22, 0.0, 0.40), (-0.12, 0.0, 0.18), 0.015, cable, "reeve_line_0")
    _cylinder_between(hook, (0.22, 0.0, 0.40), (0.12, 0.0, 0.18), 0.015, cable, "reeve_line_1")
    _cylinder_between(hook, (0.0, 0.0, -0.20), (0.0, 0.0, -0.38), 0.045, dark, "hook_shank")
    hook_curve = tube_from_spline_points(
        [
            (0.0, 0.0, -0.34),
            (0.10, 0.0, -0.43),
            (0.19, 0.0, -0.60),
            (0.09, 0.0, -0.80),
            (-0.11, 0.0, -0.82),
            (-0.20, 0.0, -0.66),
            (-0.11, 0.0, -0.55),
        ],
        radius=0.038,
        samples_per_segment=14,
        radial_segments=18,
        cap_ends=True,
    )
    hook.visual(mesh_from_geometry(hook_curve, "forged_hook_curve"), material=dark, name="forged_hook")

    model.articulation(
        "tower_to_slewing",
        ArticulationType.CONTINUOUS,
        parent=tower,
        child=slewing,
        origin=Origin(xyz=(0.0, 0.0, 18.65)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=250000.0, velocity=0.35),
    )
    model.articulation(
        "slewing_to_trolley",
        ArticulationType.PRISMATIC,
        parent=slewing,
        child=trolley,
        origin=Origin(xyz=(2.0, 0.0, 0.55)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=16.5, effort=6000.0, velocity=0.65),
    )
    model.articulation(
        "trolley_to_hook",
        ArticulationType.PRISMATIC,
        parent=trolley,
        child=hook,
        origin=Origin(xyz=(0.0, 0.0, -1.05)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(lower=0.0, upper=5.6, effort=9000.0, velocity=0.45),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower = object_model.get_part("tower")
    slewing = object_model.get_part("slewing_head")
    trolley = object_model.get_part("trolley")
    hook = object_model.get_part("hook_block")
    slew_joint = object_model.get_articulation("tower_to_slewing")
    trolley_joint = object_model.get_articulation("slewing_to_trolley")
    hoist_joint = object_model.get_articulation("trolley_to_hook")

    ctx.expect_contact(tower, slewing, elem_a="slew_bearing_base", elem_b="slew_bearing_ring", contact_tol=0.002, name="slewing bearing is seated on mast")
    ctx.expect_overlap(trolley, slewing, axes="y", min_overlap=0.40, name="trolley is laterally centered below jib")

    rest_trolley = ctx.part_world_position(trolley)
    with ctx.pose({trolley_joint: 16.5}):
        extended_trolley = ctx.part_world_position(trolley)
        ctx.expect_overlap(trolley, slewing, axes="y", min_overlap=0.40, name="trolley stays laterally aligned at full travel")

    ctx.check(
        "trolley travels outward along the horizontal jib",
        rest_trolley is not None and extended_trolley is not None and extended_trolley[0] > rest_trolley[0] + 15.0,
        details=f"rest={rest_trolley}, extended={extended_trolley}",
    )

    rest_hook = ctx.part_world_position(hook)
    with ctx.pose({hoist_joint: 5.6}):
        lowered_hook = ctx.part_world_position(hook)
    ctx.check(
        "hoist lowers the hook block",
        rest_hook is not None and lowered_hook is not None and lowered_hook[2] < rest_hook[2] - 5.0,
        details=f"rest={rest_hook}, lowered={lowered_hook}",
    )

    with ctx.pose({slew_joint: pi / 2.0}):
        slewed_trolley = ctx.part_world_position(trolley)
    ctx.check(
        "slewing rotation carries the jib and trolley around the tower",
        slewed_trolley is not None and rest_trolley is not None and slewed_trolley[1] > rest_trolley[0] - 0.2,
        details=f"rest={rest_trolley}, slewed={slewed_trolley}",
    )

    return ctx.report()


object_model = build_object_model()
