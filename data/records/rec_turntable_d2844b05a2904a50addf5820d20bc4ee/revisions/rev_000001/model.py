from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _add_box(part, name, size, xyz, material, rpy=(0.0, 0.0, 0.0)):
    return part.visual(
        Box(size),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
        name=name,
    )


def _add_cylinder(part, name, radius, length, xyz, material, rpy=(0.0, 0.0, 0.0)):
    return part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
        name=name,
    )


def _arc_points(center, radius, z, start_deg, end_deg, count):
    return [
        (
            center[0] + radius * math.cos(math.radians(angle)),
            center[1] + radius * math.sin(math.radians(angle)),
            z,
        )
        for angle in [
            start_deg + (end_deg - start_deg) * i / (count - 1)
            for i in range(count)
        ]
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="industrial_safety_turntable")

    steel = model.material("dark_blasted_steel", rgba=(0.10, 0.11, 0.12, 1.0))
    plate = model.material("oiled_top_plate", rgba=(0.18, 0.19, 0.18, 1.0))
    stainless = model.material("brushed_stainless", rgba=(0.72, 0.72, 0.68, 1.0))
    black = model.material("black_oxide", rgba=(0.02, 0.02, 0.018, 1.0))
    yellow = model.material("safety_yellow", rgba=(1.0, 0.72, 0.03, 1.0))
    red = model.material("lockout_red", rgba=(0.86, 0.04, 0.02, 1.0))
    rubber = model.material("matte_rubber", rgba=(0.015, 0.015, 0.014, 1.0))
    bolt = model.material("zinc_bolt_heads", rgba=(0.46, 0.48, 0.46, 1.0))

    plinth = model.part("plinth")

    # Welded plinth and thick service plate.
    _add_box(plinth, "base_weldment", (0.90, 0.65, 0.100), (0.0, 0.0, 0.050), steel)
    _add_box(plinth, "top_plate", (0.86, 0.61, 0.020), (0.0, 0.0, 0.109), plate)
    _add_box(plinth, "front_channel", (0.90, 0.035, 0.080), (0.0, -0.315, 0.055), steel)
    _add_box(plinth, "rear_channel", (0.90, 0.035, 0.080), (0.0, 0.315, 0.055), steel)
    _add_box(plinth, "left_channel", (0.035, 0.65, 0.080), (-0.435, 0.0, 0.055), steel)
    _add_box(plinth, "right_channel", (0.035, 0.65, 0.080), (0.435, 0.0, 0.055), steel)

    for i, (x, y) in enumerate(((-0.36, -0.25), (0.36, -0.25), (-0.36, 0.25), (0.36, 0.25))):
        _add_cylinder(plinth, f"foot_{i}", 0.040, 0.035, (x, y, -0.017), rubber)

    # Platter bearing, guard rail, and bolted load path.
    platter_center = (-0.18, 0.0)
    _add_cylinder(plinth, "platter_bearing_sleeve", 0.046, 0.054, (platter_center[0], platter_center[1], 0.133), black)
    _add_cylinder(plinth, "platter_bearing_flange", 0.095, 0.018, (platter_center[0], platter_center[1], 0.127), steel)
    for i in range(6):
        angle = 2.0 * math.pi * i / 6.0
        _add_cylinder(
            plinth,
            f"bearing_bolt_{i}",
            0.010,
            0.012,
            (platter_center[0] + 0.073 * math.cos(angle), platter_center[1] + 0.073 * math.sin(angle), 0.142),
            bolt,
        )

    guard_points = _arc_points(platter_center, 0.335, 0.335, 35.0, 285.0, 13)
    guard_mesh = tube_from_spline_points(
        guard_points,
        radius=0.012,
        samples_per_segment=8,
        radial_segments=16,
        cap_ends=True,
    )
    plinth.visual(mesh_from_geometry(guard_mesh, "platter_guard_rail"), material=yellow, name="platter_guard_rail")
    for i, angle in enumerate((35.0, 85.0, 135.0, 185.0, 235.0, 285.0)):
        x = platter_center[0] + 0.335 * math.cos(math.radians(angle))
        y = platter_center[1] + 0.335 * math.sin(math.radians(angle))
        _add_cylinder(plinth, f"guard_post_{i}", 0.018, 0.222, (x, y, 0.224), yellow)
        _add_cylinder(plinth, f"guard_foot_{i}", 0.035, 0.012, (x, y, 0.124), steel)

    _add_box(plinth, "front_warning_rail", (0.38, 0.026, 0.055), (-0.18, -0.355, 0.150), yellow)

    # Tonearm pedestal with reinforced bearing housing and over-travel stops.
    pivot_xy = (0.28, -0.20)
    _add_box(plinth, "tonearm_pedestal_plate", (0.19, 0.15, 0.028), (pivot_xy[0], pivot_xy[1], 0.131), steel)
    _add_cylinder(plinth, "tonearm_bearing_housing", 0.046, 0.064, (pivot_xy[0], pivot_xy[1], 0.165), black)
    _add_box(plinth, "tonearm_brace_front", (0.018, 0.105, 0.060), (pivot_xy[0] + 0.060, pivot_xy[1], 0.150), steel)
    _add_box(plinth, "tonearm_brace_rear", (0.018, 0.105, 0.060), (pivot_xy[0] - 0.060, pivot_xy[1], 0.150), steel)
    _add_box(plinth, "tonearm_brace_outer", (0.130, 0.018, 0.055), (pivot_xy[0], pivot_xy[1] - 0.060, 0.148), steel)

    for i, (dx, dy) in enumerate(((-0.070, -0.050), (0.070, -0.050), (-0.070, 0.050), (0.070, 0.050))):
        _add_cylinder(plinth, f"tonearm_bolt_{i}", 0.010, 0.012, (pivot_xy[0] + dx, pivot_xy[1] + dy, 0.151), bolt)

    for i, angle in enumerate((-40.0, 78.0)):
        _add_cylinder(
            plinth,
            f"tonearm_stop_{i}",
            0.019,
            0.140,
            (
                pivot_xy[0] + 0.112 * math.cos(math.radians(angle)),
                pivot_xy[1] + 0.112 * math.sin(math.radians(angle)),
                0.188,
            ),
            yellow,
        )

    # Lockout pivot support and fixed stop block.
    lock_xy = (0.27, 0.23)
    _add_box(plinth, "lockout_base_plate", (0.13, 0.09, 0.024), (lock_xy[0], lock_xy[1], 0.130), steel)
    _add_cylinder(plinth, "lockout_pivot_pin", 0.014, 0.076, (lock_xy[0], lock_xy[1], 0.176), stainless)
    _add_box(plinth, "lockout_stop_block", (0.040, 0.055, 0.034), (lock_xy[0] + 0.060, lock_xy[1] - 0.060, 0.159), yellow)
    for i, (dx, dy) in enumerate(((-0.045, -0.030), (0.045, -0.030), (-0.045, 0.030), (0.045, 0.030))):
        _add_cylinder(plinth, f"lockout_bolt_{i}", 0.008, 0.010, (lock_xy[0] + dx, lock_xy[1] + dy, 0.147), bolt)

    # The rotating platter carries its spindle/shaft and an asymmetric index stripe
    # so the continuous rotation is visible.
    platter = model.part("platter")
    _add_cylinder(platter, "platter_shaft", 0.024, 0.070, (0.0, 0.0, -0.001), stainless)
    _add_cylinder(platter, "platter_hub", 0.078, 0.026, (0.0, 0.0, 0.022), black)
    _add_cylinder(platter, "platter_disk", 0.275, 0.048, (0.0, 0.0, 0.052), stainless)
    _add_cylinder(platter, "platter_rim", 0.286, 0.014, (0.0, 0.0, 0.060), black)
    _add_cylinder(platter, "center_spindle", 0.016, 0.055, (0.0, 0.0, 0.103), stainless)
    _add_box(platter, "platter_index", (0.145, 0.026, 0.006), (0.172, 0.0, 0.079), yellow)
    for i, angle in enumerate((90.0, 210.0, 330.0)):
        _add_box(
            platter,
            f"drive_lug_{i}",
            (0.050, 0.022, 0.014),
            (0.205 * math.cos(math.radians(angle)), 0.205 * math.sin(math.radians(angle)), 0.082),
            black,
            rpy=(0.0, 0.0, math.radians(angle)),
        )

    model.articulation(
        "plinth_to_platter",
        ArticulationType.CONTINUOUS,
        parent=plinth,
        child=platter,
        origin=Origin(xyz=(platter_center[0], platter_center[1], 0.155)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=220.0, velocity=3.0),
        motion_properties=MotionProperties(damping=0.03, friction=0.05),
    )

    # Pivoting tonearm stage: one supported vertical bearing and a welded arm/headshell.
    tonearm = model.part("tonearm_stage")
    _add_cylinder(tonearm, "tonearm_pivot_shaft", 0.023, 0.120, (0.0, 0.0, 0.045), stainless)
    _add_cylinder(tonearm, "tonearm_pivot_cap", 0.055, 0.030, (0.0, 0.0, 0.108), black)
    _add_box(tonearm, "tonearm_stop_tab", (0.105, 0.020, 0.025), (0.060, 0.0, 0.035), yellow)
    _add_box(tonearm, "tonearm_yoke", (0.090, 0.055, 0.035), (-0.025, 0.0, 0.130), steel)
    _add_cylinder(tonearm, "tonearm_tube", 0.012, 0.440, (-0.240, 0.0, 0.136), stainless, rpy=(0.0, math.pi / 2.0, 0.0))
    _add_cylinder(tonearm, "counterweight", 0.035, 0.080, (0.075, 0.0, 0.136), black, rpy=(0.0, math.pi / 2.0, 0.0))
    _add_box(tonearm, "head_shell", (0.090, 0.048, 0.018), (-0.500, 0.0, 0.126), black)
    _add_box(tonearm, "cartridge", (0.045, 0.028, 0.026), (-0.525, 0.0, 0.105), red)
    _add_cylinder(tonearm, "stylus_guard", 0.007, 0.038, (-0.545, 0.0, 0.090), yellow, rpy=(0.0, math.pi / 2.0, 0.0))

    model.articulation(
        "plinth_to_tonearm",
        ArticulationType.REVOLUTE,
        parent=plinth,
        child=tonearm,
        origin=Origin(xyz=(pivot_xy[0], pivot_xy[1], 0.170)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=55.0, velocity=1.2, lower=-0.45, upper=0.95),
        motion_properties=MotionProperties(damping=0.18, friction=0.20),
    )

    lockout = model.part("lockout_lever")
    _add_cylinder(lockout, "lockout_hub", 0.036, 0.026, (0.0, 0.0, 0.016), red)
    _add_box(lockout, "lockout_handle", (0.200, 0.035, 0.025), (0.125, 0.0, 0.030), red)
    _add_box(lockout, "lockout_flag", (0.060, 0.070, 0.030), (0.245, 0.0, 0.034), yellow)
    _add_cylinder(lockout, "grip_end", 0.024, 0.045, (0.225, 0.0, 0.036), black, rpy=(math.pi / 2.0, 0.0, 0.0))

    model.articulation(
        "plinth_to_lockout",
        ArticulationType.REVOLUTE,
        parent=plinth,
        child=lockout,
        origin=Origin(xyz=(lock_xy[0], lock_xy[1], 0.160)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.8, lower=0.0, upper=1.15),
        motion_properties=MotionProperties(damping=0.08, friction=0.12),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    plinth = object_model.get_part("plinth")
    platter = object_model.get_part("platter")
    tonearm = object_model.get_part("tonearm_stage")
    lockout = object_model.get_part("lockout_lever")
    platter_joint = object_model.get_articulation("plinth_to_platter")
    tonearm_joint = object_model.get_articulation("plinth_to_tonearm")
    lockout_joint = object_model.get_articulation("plinth_to_lockout")

    ctx.allow_overlap(
        plinth,
        platter,
        elem_a="platter_bearing_sleeve",
        elem_b="platter_shaft",
        reason="The rotating spindle shaft is intentionally captured inside the fixed bearing sleeve.",
    )
    ctx.expect_within(
        platter,
        plinth,
        axes="xy",
        inner_elem="platter_shaft",
        outer_elem="platter_bearing_sleeve",
        margin=0.001,
        name="platter shaft centered in bearing sleeve",
    )
    ctx.expect_overlap(
        platter,
        plinth,
        axes="z",
        elem_a="platter_shaft",
        elem_b="platter_bearing_sleeve",
        min_overlap=0.035,
        name="platter spindle retained in bearing sleeve",
    )
    ctx.allow_overlap(
        platter,
        plinth,
        elem_a="platter_shaft",
        elem_b="platter_bearing_flange",
        reason="The simplified bearing flange is a solid proxy for a bolted flange with a central shaft clearance hole.",
    )
    ctx.expect_within(
        platter,
        plinth,
        axes="xy",
        inner_elem="platter_shaft",
        outer_elem="platter_bearing_flange",
        margin=0.001,
        name="platter shaft passes through flange center",
    )
    ctx.expect_overlap(
        platter,
        plinth,
        axes="z",
        elem_a="platter_shaft",
        elem_b="platter_bearing_flange",
        min_overlap=0.010,
        name="platter shaft passes through flange thickness",
    )
    ctx.expect_gap(
        platter,
        plinth,
        axis="z",
        positive_elem="platter_disk",
        negative_elem="top_plate",
        min_gap=0.055,
        name="platter disk clears top plate",
    )

    ctx.allow_overlap(
        plinth,
        tonearm,
        elem_a="tonearm_bearing_housing",
        elem_b="tonearm_pivot_shaft",
        reason="The tonearm pivot shaft is intentionally captured by the fixed reinforced bearing housing.",
    )
    ctx.expect_within(
        tonearm,
        plinth,
        axes="xy",
        inner_elem="tonearm_pivot_shaft",
        outer_elem="tonearm_bearing_housing",
        margin=0.001,
        name="tonearm pivot shaft centered in housing",
    )
    ctx.expect_overlap(
        tonearm,
        plinth,
        axes="z",
        elem_a="tonearm_pivot_shaft",
        elem_b="tonearm_bearing_housing",
        min_overlap=0.035,
        name="tonearm pivot retained in reinforced housing",
    )

    ctx.allow_overlap(
        plinth,
        lockout,
        elem_a="lockout_pivot_pin",
        elem_b="lockout_hub",
        reason="The lockout lever hub rotates around a captured fixed pivot pin.",
    )
    ctx.expect_within(
        plinth,
        lockout,
        axes="xy",
        inner_elem="lockout_pivot_pin",
        outer_elem="lockout_hub",
        margin=0.001,
        name="lockout pin lies inside lever hub",
    )
    ctx.expect_overlap(
        plinth,
        lockout,
        axes="z",
        elem_a="lockout_pivot_pin",
        elem_b="lockout_hub",
        min_overlap=0.020,
        name="lockout hub retained on pivot pin",
    )

    ctx.check(
        "platter has continuous spindle rotation",
        platter_joint.articulation_type == ArticulationType.CONTINUOUS and platter_joint.axis == (0.0, 0.0, 1.0),
        details=f"type={platter_joint.articulation_type}, axis={platter_joint.axis}",
    )
    limits = tonearm_joint.motion_limits
    ctx.check(
        "tonearm has bounded pivot travel",
        limits is not None and limits.lower is not None and limits.upper is not None and limits.lower < 0.0 < limits.upper,
        details=f"limits={limits}",
    )
    lock_limits = lockout_joint.motion_limits
    ctx.check(
        "lockout lever has finite swing",
        lock_limits is not None and lock_limits.lower == 0.0 and lock_limits.upper is not None and lock_limits.upper > 0.5,
        details=f"limits={lock_limits}",
    )

    return ctx.report()


object_model = build_object_model()
