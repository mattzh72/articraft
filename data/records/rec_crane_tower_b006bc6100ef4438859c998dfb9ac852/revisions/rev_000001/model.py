from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _rpy_from_z_axis_to_vector(vector: tuple[float, float, float]) -> tuple[float, float, float]:
    x, y, z = vector
    length = math.sqrt(x * x + y * y + z * z)
    if length <= 1e-9:
        return (0.0, 0.0, 0.0)
    ux, uy, uz = x / length, y / length, z / length
    pitch = math.atan2(math.sqrt(ux * ux + uy * uy), uz)
    yaw = math.atan2(uy, ux)
    return (0.0, pitch, yaw)


def _add_tube(
    part,
    name: str,
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    radius: float,
    material,
) -> None:
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    dz = end[2] - start[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if length <= 1e-9:
        return
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(
            xyz=((start[0] + end[0]) * 0.5, (start[1] + end[1]) * 0.5, (start[2] + end[2]) * 0.5),
            rpy=_rpy_from_z_axis_to_vector((dx, dy, dz)),
        ),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="luffing_jib_tower_crane")

    crane_yellow = model.material("crane_yellow", rgba=(1.0, 0.78, 0.05, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.05, 0.055, 0.06, 1.0))
    weathered_steel = model.material("weathered_steel", rgba=(0.28, 0.30, 0.31, 1.0))
    concrete = model.material("concrete", rgba=(0.48, 0.47, 0.43, 1.0))
    counterweight_mat = model.material("counterweight_grey", rgba=(0.34, 0.34, 0.32, 1.0))
    glass = model.material("blue_tinted_glass", rgba=(0.25, 0.50, 0.72, 0.72))
    hook_red = model.material("safety_red", rgba=(0.82, 0.05, 0.03, 1.0))

    # Root: square lattice mast on a real-sized foundation.
    mast = model.part("mast")
    mast.visual(
        Box((1.65, 1.65, 0.25)),
        origin=Origin(xyz=(0.0, 0.0, 0.125)),
        material=concrete,
        name="foundation",
    )
    mast.visual(
        Cylinder(radius=0.46, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 5.95)),
        material=weathered_steel,
        name="mast_head_pedestal",
    )

    leg_xy = (-0.34, 0.34)
    leg_bottom = 0.25
    leg_top = 5.90
    for xi in leg_xy:
        for yi in leg_xy:
            _add_tube(
                mast,
                f"mast_leg_{0 if xi < 0 else 1}_{0 if yi < 0 else 1}",
                (xi, yi, leg_bottom),
                (xi, yi, leg_top),
                0.045,
                crane_yellow,
            )
            mast.visual(
                Box((0.18, 0.18, 0.035)),
                origin=Origin(xyz=(xi, yi, 0.275)),
                material=weathered_steel,
                name=f"foot_plate_{0 if xi < 0 else 1}_{0 if yi < 0 else 1}",
            )

    levels = [0.35, 1.15, 1.95, 2.75, 3.55, 4.35, 5.15, 5.80]
    for idx, z in enumerate(levels):
        _add_tube(mast, f"front_girt_{idx}", (-0.34, -0.34, z), (0.34, -0.34, z), 0.024, crane_yellow)
        _add_tube(mast, f"rear_girt_{idx}", (-0.34, 0.34, z), (0.34, 0.34, z), 0.024, crane_yellow)
        _add_tube(mast, f"side_girt_{idx}_0", (-0.34, -0.34, z), (-0.34, 0.34, z), 0.024, crane_yellow)
        _add_tube(mast, f"side_girt_{idx}_1", (0.34, -0.34, z), (0.34, 0.34, z), 0.024, crane_yellow)

    for bay, (z0, z1) in enumerate(zip(levels[:-1], levels[1:])):
        flip = bay % 2 == 0
        a, b = (-0.34, 0.34) if flip else (0.34, -0.34)
        _add_tube(mast, f"front_diagonal_{bay}", (a, -0.34, z0), (b, -0.34, z1), 0.020, crane_yellow)
        _add_tube(mast, f"rear_diagonal_{bay}", (b, 0.34, z0), (a, 0.34, z1), 0.020, crane_yellow)
        _add_tube(mast, f"side_diagonal_{bay}_0", (-0.34, a, z0), (-0.34, b, z1), 0.020, crane_yellow)
        _add_tube(mast, f"side_diagonal_{bay}_1", (0.34, b, z0), (0.34, a, z1), 0.020, crane_yellow)

    # Slewing upper structure: bearing ring, machinery deck, cab, counterweight, and hinge towers.
    slewing_ring = model.part("slewing_ring")
    slewing_ring.visual(
        Cylinder(radius=0.58, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
        material=dark_steel,
        name="bearing_ring",
    )
    slewing_ring.visual(
        Cylinder(radius=0.48, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 0.17)),
        material=crane_yellow,
        name="turntable_disc",
    )
    slewing_ring.visual(
        Box((1.35, 0.78, 0.12)),
        origin=Origin(xyz=(0.03, 0.0, 0.28)),
        material=crane_yellow,
        name="machinery_deck",
    )
    slewing_ring.visual(
        Box((0.42, 0.36, 0.38)),
        origin=Origin(xyz=(0.40, -0.43, 0.55)),
        material=crane_yellow,
        name="operator_cab",
    )
    slewing_ring.visual(
        Box((0.36, 0.025, 0.22)),
        origin=Origin(xyz=(0.42, -0.616, 0.57)),
        material=glass,
        name="cab_window",
    )
    slewing_ring.visual(
        Box((0.48, 0.68, 0.36)),
        origin=Origin(xyz=(-0.72, 0.0, 0.47)),
        material=counterweight_mat,
        name="counterweight",
    )
    _add_tube(slewing_ring, "counter_jib_chord_0", (-0.05, -0.24, 0.40), (-1.18, -0.18, 0.42), 0.035, crane_yellow)
    _add_tube(slewing_ring, "counter_jib_chord_1", (-0.05, 0.24, 0.40), (-1.18, 0.18, 0.42), 0.035, crane_yellow)
    _add_tube(slewing_ring, "a_frame_strut_0", (-0.20, -0.30, 0.32), (0.15, 0.0, 1.46), 0.035, crane_yellow)
    _add_tube(slewing_ring, "a_frame_strut_1", (-0.20, 0.30, 0.32), (0.15, 0.0, 1.46), 0.035, crane_yellow)
    slewing_ring.visual(
        Sphere(radius=0.07),
        origin=Origin(xyz=(0.15, 0.0, 1.46)),
        material=crane_yellow,
        name="a_frame_apex",
    )
    slewing_ring.visual(
        Cylinder(radius=0.055, length=0.56),
        origin=Origin(xyz=(0.66, 0.0, 0.58), rpy=_rpy_from_z_axis_to_vector((0.0, 1.0, 0.0))),
        material=dark_steel,
        name="luffing_axis_pin",
    )
    for side, y in enumerate((-0.28, 0.28)):
        slewing_ring.visual(
            Box((0.22, 0.08, 0.54)),
            origin=Origin(xyz=(0.66, y, 0.58)),
            material=crane_yellow,
            name=f"hinge_cheek_{side}",
        )

    model.articulation(
        "mast_to_slewing_ring",
        ArticulationType.REVOLUTE,
        parent=mast,
        child=slewing_ring,
        origin=Origin(xyz=(0.0, 0.0, 6.00)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=200000.0, velocity=0.35, lower=-math.pi, upper=math.pi),
    )

    # Luffing jib: a tapered triangular lattice boom whose child frame is on the hinge line.
    jib = model.part("jib")
    jib.visual(
        Cylinder(radius=0.075, length=0.28),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=_rpy_from_z_axis_to_vector((0.0, 1.0, 0.0))),
        material=crane_yellow,
        name="root_hinge_lug",
    )
    jib_length = 8.0
    lower_l = lambda x: (x, -0.21 + 0.10 * (x / jib_length), -0.08)
    lower_r = lambda x: (x, 0.21 - 0.10 * (x / jib_length), -0.08)
    top = lambda x: (x, 0.0, 0.34 - 0.22 * (x / jib_length))
    _add_tube(jib, "lower_chord_0", lower_l(0.05), lower_l(jib_length), 0.033, crane_yellow)
    _add_tube(jib, "lower_chord_1", lower_r(0.05), lower_r(jib_length), 0.033, crane_yellow)
    _add_tube(jib, "top_chord", top(0.12), top(jib_length), 0.033, crane_yellow)
    _add_tube(jib, "hoist_runway_chord", (0.55, 0.0, -0.20), (jib_length - 0.22, 0.0, -0.20), 0.040, dark_steel)
    _add_tube(jib, "root_gusset_0", (0.08, -0.13, 0.04), lower_l(0.45), 0.022, crane_yellow)
    _add_tube(jib, "root_gusset_1", (0.08, 0.13, 0.04), lower_r(0.45), 0.022, crane_yellow)
    for i, x in enumerate([0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0]):
        _add_tube(jib, f"cross_tie_{i}", lower_l(x), lower_r(x), 0.019, crane_yellow)
        if i > 0:
            _add_tube(jib, f"vertical_web_{i}", (x, 0.0, -0.08), top(x), 0.018, crane_yellow)
        if i < 8:
            x2 = x + 1.0
            _add_tube(jib, f"side_web_a_{i}", lower_l(x), top(x2), 0.018, crane_yellow)
            _add_tube(jib, f"side_web_b_{i}", lower_r(x2), top(x), 0.018, crane_yellow)
            hanger_x = x + 0.5
            _add_tube(jib, f"runway_hanger_{i}", (hanger_x, 0.0, -0.20), (hanger_x, 0.0, -0.08), 0.016, dark_steel)
            _add_tube(jib, f"runway_brace_{i}_0", (hanger_x, 0.0, -0.20), lower_l(hanger_x), 0.014, dark_steel)
            _add_tube(jib, f"runway_brace_{i}_1", (hanger_x, 0.0, -0.20), lower_r(hanger_x), 0.014, dark_steel)
    jib.visual(
        Cylinder(radius=0.13, length=0.09),
        origin=Origin(xyz=(jib_length + 0.06, 0.0, -0.10), rpy=_rpy_from_z_axis_to_vector((0.0, 1.0, 0.0))),
        material=dark_steel,
        name="tip_sheave",
    )
    _add_tube(jib, "tip_fairlead", (jib_length - 0.24, -0.10, -0.20), (jib_length + 0.06, 0.0, -0.10), 0.018, dark_steel)
    _add_tube(jib, "tip_fairlead_side", (jib_length - 0.24, 0.10, -0.20), (jib_length + 0.06, 0.0, -0.10), 0.018, dark_steel)

    model.articulation(
        "slewing_ring_to_jib",
        ArticulationType.REVOLUTE,
        parent=slewing_ring,
        child=jib,
        origin=Origin(xyz=(0.66, 0.0, 0.58)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=85000.0, velocity=0.22, lower=-0.18, upper=1.05),
    )

    # Pendant hoist block that runs along the lower jib chord on a prismatic carriage.
    hoist_block = model.part("hoist_block")
    hoist_block.visual(
        Cylinder(radius=0.065, length=0.28),
        origin=Origin(xyz=(0.0, 0.0, -0.105), rpy=_rpy_from_z_axis_to_vector((0.0, 1.0, 0.0))),
        material=weathered_steel,
        name="roller",
    )
    hoist_block.visual(
        Cylinder(radius=0.018, length=0.38),
        origin=Origin(xyz=(0.0, 0.0, -0.105), rpy=_rpy_from_z_axis_to_vector((0.0, 1.0, 0.0))),
        material=dark_steel,
        name="roller_axle",
    )
    hoist_block.visual(
        Box((0.28, 0.030, 0.24)),
        origin=Origin(xyz=(0.0, -0.155, -0.23)),
        material=crane_yellow,
        name="side_plate_0",
    )
    hoist_block.visual(
        Box((0.28, 0.030, 0.24)),
        origin=Origin(xyz=(0.0, 0.155, -0.23)),
        material=crane_yellow,
        name="side_plate_1",
    )
    hoist_block.visual(
        Box((0.20, 0.32, 0.055)),
        origin=Origin(xyz=(0.0, 0.0, -0.35)),
        material=crane_yellow,
        name="block_yoke",
    )
    _add_tube(hoist_block, "wire_rope", (0.0, 0.0, -0.35), (0.0, 0.0, -1.23), 0.010, dark_steel)
    hoist_block.visual(
        Box((0.30, 0.18, 0.22)),
        origin=Origin(xyz=(0.0, 0.0, -1.32)),
        material=crane_yellow,
        name="load_block",
    )
    hoist_block.visual(
        Cylinder(radius=0.075, length=0.20),
        origin=Origin(xyz=(0.0, 0.0, -1.32), rpy=_rpy_from_z_axis_to_vector((0.0, 1.0, 0.0))),
        material=dark_steel,
        name="block_sheave",
    )
    _add_tube(hoist_block, "hook_shank", (0.0, 0.0, -1.43), (0.0, 0.0, -1.56), 0.015, hook_red)
    hook_mesh = tube_from_spline_points(
        [
            (0.0, 0.0, -1.55),
            (0.00, 0.0, -1.68),
            (0.08, 0.0, -1.77),
            (0.18, 0.0, -1.70),
            (0.14, 0.0, -1.58),
        ],
        radius=0.017,
        samples_per_segment=10,
        radial_segments=14,
        cap_ends=True,
    )
    hoist_block.visual(
        mesh_from_geometry(hook_mesh, "crane_hook"),
        origin=Origin(),
        material=hook_red,
        name="hook",
    )

    model.articulation(
        "jib_to_hoist_block",
        ArticulationType.PRISMATIC,
        parent=jib,
        child=hoist_block,
        origin=Origin(xyz=(3.20, 0.0, -0.20)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=15000.0, velocity=0.50, lower=-2.10, upper=4.35),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    mast = object_model.get_part("mast")
    slewing_ring = object_model.get_part("slewing_ring")
    jib = object_model.get_part("jib")
    hoist_block = object_model.get_part("hoist_block")
    slew = object_model.get_articulation("mast_to_slewing_ring")
    luff = object_model.get_articulation("slewing_ring_to_jib")
    trolley = object_model.get_articulation("jib_to_hoist_block")

    # The bearing sits at the mast head, the jib hinges from the slewing upper,
    # and the pendant trolley is carried directly on the underside runway chord.
    ctx.allow_overlap(
        jib,
        slewing_ring,
        elem_a="root_hinge_lug",
        elem_b="luffing_axis_pin",
        reason="The luffing boom lug is intentionally captured around the horizontal hinge pin.",
    )
    ctx.expect_overlap(
        jib,
        slewing_ring,
        axes="y",
        elem_a="root_hinge_lug",
        elem_b="luffing_axis_pin",
        min_overlap=0.20,
        name="luffing hinge pin is captured through boom lug",
    )
    ctx.expect_contact(
        mast,
        slewing_ring,
        elem_a="mast_head_pedestal",
        elem_b="bearing_ring",
        contact_tol=0.002,
        name="slewing bearing is seated on mast head",
    )
    ctx.expect_gap(
        jib,
        hoist_block,
        axis="z",
        positive_elem="hoist_runway_chord",
        negative_elem="roller",
        max_gap=0.003,
        max_penetration=0.001,
        name="hoist roller touches underside of jib runway",
    )

    # Moving the luffing joint raises the sheave end around the horizontal hinge.
    with ctx.pose({luff: 0.0}):
        tip_aabb_low = ctx.part_element_world_aabb(jib, elem="tip_sheave")
    with ctx.pose({luff: 0.75}):
        tip_aabb_high = ctx.part_element_world_aabb(jib, elem="tip_sheave")
    low_z = None if tip_aabb_low is None else (tip_aabb_low[0][2] + tip_aabb_low[1][2]) * 0.5
    high_z = None if tip_aabb_high is None else (tip_aabb_high[0][2] + tip_aabb_high[1][2]) * 0.5
    ctx.check(
        "luffing jib raises",
        low_z is not None and high_z is not None and high_z > low_z + 2.0,
        details=f"low_z={low_z}, high_z={high_z}",
    )

    # Slewing rotates the whole upper assembly and jib about the vertical mast axis.
    with ctx.pose({slew: 0.0, luff: 0.0}):
        tip_aabb_forward = ctx.part_element_world_aabb(jib, elem="tip_sheave")
    with ctx.pose({slew: math.pi / 2.0, luff: 0.0}):
        tip_aabb_side = ctx.part_element_world_aabb(jib, elem="tip_sheave")
    fwd = None if tip_aabb_forward is None else (
        (tip_aabb_forward[0][0] + tip_aabb_forward[1][0]) * 0.5,
        (tip_aabb_forward[0][1] + tip_aabb_forward[1][1]) * 0.5,
    )
    side = None if tip_aabb_side is None else (
        (tip_aabb_side[0][0] + tip_aabb_side[1][0]) * 0.5,
        (tip_aabb_side[0][1] + tip_aabb_side[1][1]) * 0.5,
    )
    ctx.check(
        "slewing joint rotates jib around mast",
        fwd is not None and side is not None and fwd[0] > 6.0 and side[1] > 6.0,
        details=f"forward_tip_xy={fwd}, side_tip_xy={side}",
    )

    # The pendant hoist block travels along the jib chord.
    with ctx.pose({trolley: -1.50, luff: 0.0, slew: 0.0}):
        near_pos = ctx.part_world_position(hoist_block)
    with ctx.pose({trolley: 3.75, luff: 0.0, slew: 0.0}):
        far_pos = ctx.part_world_position(hoist_block)
    ctx.check(
        "hoist block slides along jib chord",
        near_pos is not None and far_pos is not None and far_pos[0] > near_pos[0] + 5.0,
        details=f"near={near_pos}, far={far_pos}",
    )

    return ctx.report()


object_model = build_object_model()
