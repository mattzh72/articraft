from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rugged_searchlight_tower")

    safety_yellow = model.material("safety_yellow", rgba=(0.95, 0.68, 0.12, 1.0))
    dark_olive = model.material("dark_olive_paint", rgba=(0.22, 0.26, 0.19, 1.0))
    charcoal = model.material("charcoal_molded", rgba=(0.05, 0.06, 0.055, 1.0))
    galvanized = model.material("galvanized_steel", rgba=(0.56, 0.58, 0.55, 1.0))
    worn_steel = model.material("worn_steel", rgba=(0.33, 0.34, 0.32, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.015, 0.015, 0.014, 1.0))
    glass = model.material("blue_tinted_glass", rgba=(0.48, 0.74, 0.95, 0.42))
    reflector = model.material("brushed_reflector", rgba=(0.86, 0.84, 0.76, 1.0))
    warm_bulb = model.material("warm_bulb_glow", rgba=(1.0, 0.86, 0.42, 1.0))

    # Revolved, thin-walled lamp body.  It is deliberately open at the front and
    # rear so the lens, reflector cup, and serviceable rear cap read as separate
    # seated components rather than a single solid cylinder.
    lamp_shell = _mesh(
        "lamp_shell",
        LatheGeometry(
            [
                (0.275, -0.340),
                (0.305, -0.080),
                (0.355, 0.420),
                (0.318, 0.438),
                (0.292, 0.080),
                (0.242, -0.300),
            ],
            segments=72,
        ),
    )
    reflector_cup = _mesh(
        "reflector_cup",
        LatheGeometry(
            [
                (0.070, -0.160),
                (0.145, -0.105),
                (0.225, 0.030),
                (0.298, 0.235),
                (0.276, 0.250),
                (0.190, 0.020),
                (0.055, -0.135),
            ],
            segments=72,
        ),
    )
    front_guard_ring = _mesh(
        "front_guard_ring",
        TorusGeometry(radius=0.322, tube=0.012, radial_segments=12, tubular_segments=72),
    )

    tower = model.part("tower")
    tower.visual(
        Box((1.55, 1.55, 0.16)),
        origin=Origin(xyz=(0.0, 0.0, 0.08)),
        material=dark_olive,
        name="skid_base",
    )
    for i, (x, y) in enumerate(((-0.72, -0.72), (-0.72, 0.72), (0.72, -0.72), (0.72, 0.72))):
        tower.visual(
            Box((0.34, 0.22, 0.055)),
            origin=Origin(xyz=(x, y, 0.027)),
            material=worn_steel,
            name=f"anchor_foot_{i}",
        )
        tower.visual(
            Cylinder(radius=0.028, length=0.026),
            origin=Origin(xyz=(x - 0.08, y, 0.066)),
            material=charcoal,
            name=f"anchor_bolt_{i}_0",
        )
        tower.visual(
            Cylinder(radius=0.028, length=0.026),
            origin=Origin(xyz=(x + 0.08, y, 0.066)),
            material=charcoal,
            name=f"anchor_bolt_{i}_1",
        )

    tower.visual(
        Cylinder(radius=0.42, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, 0.22)),
        material=worn_steel,
        name="mast_foot_flange",
    )
    for i, (x, y) in enumerate(((-0.30, -0.30), (-0.30, 0.30), (0.30, -0.30), (0.30, 0.30))):
        tower.visual(
            Box((0.095, 0.095, 3.18)),
            origin=Origin(xyz=(x, y, 1.75)),
            material=safety_yellow,
            name=f"tower_post_{i}",
        )

    for level, z in enumerate((0.66, 1.34, 2.02, 2.70, 3.22)):
        tower.visual(
            Box((0.72, 0.055, 0.055)),
            origin=Origin(xyz=(0.0, 0.326, z)),
            material=safety_yellow,
            name=f"front_crossbar_{level}",
        )
        tower.visual(
            Box((0.72, 0.055, 0.055)),
            origin=Origin(xyz=(0.0, -0.326, z)),
            material=safety_yellow,
            name=f"rear_crossbar_{level}",
        )
        tower.visual(
            Box((0.055, 0.72, 0.055)),
            origin=Origin(xyz=(0.326, 0.0, z)),
            material=safety_yellow,
            name=f"side_crossbar_{level}_0",
        )
        tower.visual(
            Box((0.055, 0.72, 0.055)),
            origin=Origin(xyz=(-0.326, 0.0, z)),
            material=safety_yellow,
            name=f"side_crossbar_{level}_1",
        )

    brace_angle = math.atan2(0.68, 0.60)
    brace_len = math.hypot(0.60, 0.68)
    for bay, z_mid in enumerate((1.00, 1.68, 2.36)):
        tower.visual(
            Box((brace_len, 0.040, 0.040)),
            origin=Origin(xyz=(0.0, 0.335, z_mid), rpy=(0.0, -brace_angle, 0.0)),
            material=galvanized,
            name=f"front_diag_{bay}_0",
        )
        tower.visual(
            Box((brace_len, 0.040, 0.040)),
            origin=Origin(xyz=(0.0, -0.335, z_mid), rpy=(0.0, brace_angle, 0.0)),
            material=galvanized,
            name=f"rear_diag_{bay}_0",
        )
        tower.visual(
            Box((brace_len, 0.040, 0.040)),
            origin=Origin(xyz=(0.335, 0.0, z_mid), rpy=(0.0, -brace_angle, math.pi / 2.0)),
            material=galvanized,
            name=f"side_diag_{bay}_0",
        )
        tower.visual(
            Box((brace_len, 0.040, 0.040)),
            origin=Origin(xyz=(-0.335, 0.0, z_mid), rpy=(0.0, brace_angle, math.pi / 2.0)),
            material=galvanized,
            name=f"side_diag_{bay}_1",
        )

    tower.visual(
        Box((1.88, 1.88, 0.11)),
        origin=Origin(xyz=(0.0, 0.0, 3.38)),
        material=galvanized,
        name="service_deck",
    )
    for index, y in enumerate((-0.66, -0.44, -0.22, 0.0, 0.22, 0.44, 0.66)):
        tower.visual(
            Box((1.74, 0.025, 0.020)),
            origin=Origin(xyz=(0.0, y, 3.442)),
            material=worn_steel,
            name=f"deck_grip_bar_{index}",
        )
    for i, (x, y) in enumerate(((-0.86, -0.86), (-0.86, 0.86), (0.86, -0.86), (0.86, 0.86))):
        tower.visual(
            Box((0.055, 0.055, 0.78)),
            origin=Origin(xyz=(x, y, 3.80)),
            material=safety_yellow,
            name=f"rail_post_{i}",
        )
    tower.visual(Box((1.82, 0.055, 0.065)), origin=Origin(xyz=(0.0, 0.89, 4.18)), material=safety_yellow, name="front_handrail")
    tower.visual(Box((1.82, 0.055, 0.065)), origin=Origin(xyz=(0.0, -0.89, 4.18)), material=safety_yellow, name="rear_handrail")
    tower.visual(Box((0.055, 1.82, 0.065)), origin=Origin(xyz=(0.89, 0.0, 4.18)), material=safety_yellow, name="side_handrail_0")
    tower.visual(Box((0.055, 1.82, 0.065)), origin=Origin(xyz=(-0.89, 0.0, 4.18)), material=safety_yellow, name="side_handrail_1")
    tower.visual(Box((1.90, 0.060, 0.13)), origin=Origin(xyz=(0.0, 0.92, 3.50)), material=dark_olive, name="front_toe_board")
    tower.visual(Box((1.90, 0.060, 0.13)), origin=Origin(xyz=(0.0, -0.92, 3.50)), material=dark_olive, name="rear_toe_board")
    tower.visual(Box((0.060, 1.90, 0.13)), origin=Origin(xyz=(0.92, 0.0, 3.50)), material=dark_olive, name="side_toe_board_0")
    tower.visual(Box((0.060, 1.90, 0.13)), origin=Origin(xyz=(-0.92, 0.0, 3.50)), material=dark_olive, name="side_toe_board_1")

    tower.visual(Box((0.055, 0.055, 3.24)), origin=Origin(xyz=(-0.74, -0.17, 1.78)), material=galvanized, name="ladder_rail_0")
    tower.visual(Box((0.055, 0.055, 3.24)), origin=Origin(xyz=(-0.74, 0.17, 1.78)), material=galvanized, name="ladder_rail_1")
    for rung, z in enumerate((0.42, 0.72, 1.02, 1.32, 1.62, 1.92, 2.22, 2.52, 2.82, 3.12)):
        tower.visual(
            Box((0.048, 0.40, 0.036)),
            origin=Origin(xyz=(-0.74, 0.0, z)),
            material=galvanized,
            name=f"ladder_rung_{rung}",
        )
    for bracket, z in enumerate((0.62, 1.54, 2.46, 3.20)):
        tower.visual(
            Box((0.40, 0.045, 0.045)),
            origin=Origin(xyz=(-0.54, -0.17, z)),
            material=galvanized,
            name=f"ladder_bracket_{bracket}_0",
        )
        tower.visual(
            Box((0.40, 0.045, 0.045)),
            origin=Origin(xyz=(-0.54, 0.17, z)),
            material=galvanized,
            name=f"ladder_bracket_{bracket}_1",
        )

    tower.visual(Cylinder(radius=0.30, length=0.50), origin=Origin(xyz=(0.0, 0.0, 3.68)), material=dark_olive, name="fixed_pedestal")
    tower.visual(Cylinder(radius=0.46, length=0.16), origin=Origin(xyz=(0.0, 0.0, 4.01)), material=worn_steel, name="fixed_bearing_base")
    tower.visual(Cylinder(radius=0.36, length=0.16), origin=Origin(xyz=(0.0, 0.0, 4.17)), material=charcoal, name="top_bearing_plate")

    pan_stage = model.part("pan_stage")
    pan_stage.visual(Cylinder(radius=0.38, length=0.16), origin=Origin(xyz=(0.0, 0.0, 0.08)), material=charcoal, name="turntable_bottom")
    pan_stage.visual(Cylinder(radius=0.25, length=0.24), origin=Origin(xyz=(0.0, 0.0, 0.22)), material=worn_steel, name="bearing_collar")
    pan_stage.visual(Box((0.88, 1.14, 0.20)), origin=Origin(xyz=(0.0, 0.0, 0.37)), material=dark_olive, name="yoke_saddle")
    pan_stage.visual(Box((0.70, 0.11, 0.16)), origin=Origin(xyz=(0.0, 0.51, 0.52)), material=worn_steel, name="saddle_rib_0")
    pan_stage.visual(Box((0.70, 0.11, 0.16)), origin=Origin(xyz=(0.0, -0.51, 0.52)), material=worn_steel, name="saddle_rib_1")
    for side, y, bearing_name in ((0, -0.48, "tilt_bearing_0"), (1, 0.48, "tilt_bearing_1")):
        pan_stage.visual(Box((0.15, 0.15, 0.48)), origin=Origin(xyz=(0.0, y, 0.60)), material=dark_olive, name=f"lower_yoke_{side}")
        pan_stage.visual(Box((0.17, 0.15, 0.32)), origin=Origin(xyz=(0.0, y, 1.28)), material=dark_olive, name=f"upper_yoke_{side}")
        pan_stage.visual(
            Cylinder(radius=0.18, length=0.18),
            origin=Origin(xyz=(0.0, y, 0.98), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=charcoal,
            name=bearing_name,
        )
        pan_stage.visual(
            Cylinder(radius=0.036, length=0.018),
            origin=Origin(xyz=(-0.11, y + (0.082 if y > 0 else -0.082), 1.09), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=worn_steel,
            name=f"bearing_bolt_{side}_0",
        )
        pan_stage.visual(
            Cylinder(radius=0.036, length=0.018),
            origin=Origin(xyz=(0.11, y + (0.082 if y > 0 else -0.082), 1.09), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=worn_steel,
            name=f"bearing_bolt_{side}_1",
        )
        pan_stage.visual(
            Cylinder(radius=0.036, length=0.018),
            origin=Origin(xyz=(-0.11, y + (0.082 if y > 0 else -0.082), 0.87), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=worn_steel,
            name=f"bearing_bolt_{side}_2",
        )
        pan_stage.visual(
            Cylinder(radius=0.036, length=0.018),
            origin=Origin(xyz=(0.11, y + (0.082 if y > 0 else -0.082), 0.87), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=worn_steel,
            name=f"bearing_bolt_{side}_3",
        )

    spotlight_head = model.part("spotlight_head")
    spotlight_head.visual(
        Cylinder(radius=0.073, length=1.16),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=worn_steel,
        name="trunnion_shaft",
    )
    spotlight_head.visual(lamp_shell, origin=Origin(xyz=(0.25, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)), material=dark_olive, name="lamp_shell")
    spotlight_head.visual(reflector_cup, origin=Origin(xyz=(0.25, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)), material=reflector, name="reflector_cup")
    spotlight_head.visual(Cylinder(radius=0.250, length=0.18), origin=Origin(xyz=(-0.15, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)), material=charcoal, name="rear_service_cap")
    spotlight_head.visual(Box((0.10, 0.50, 0.16)), origin=Origin(xyz=(-0.22, 0.0, 0.22)), material=charcoal, name="rear_heat_sink")
    spotlight_head.visual(Cylinder(radius=0.120, length=0.08), origin=Origin(xyz=(-0.29, 0.0, -0.19), rpy=(0.0, math.pi / 2.0, 0.0)), material=black_rubber, name="cable_gland")
    spotlight_head.visual(Box((0.92, 0.12, 0.16)), origin=Origin(xyz=(0.26, 0.0, 0.20)), material=charcoal, name="top_mount_spine")
    spotlight_head.visual(Box((0.18, 0.76, 0.12)), origin=Origin(xyz=(0.0, 0.0, 0.13)), material=dark_olive, name="trunnion_bridge")
    spotlight_head.visual(Box((0.11, 0.13, 0.36)), origin=Origin(xyz=(-0.25, 0.0, -0.03)), material=charcoal, name="rear_cap_bridge")
    spotlight_head.visual(Cylinder(radius=0.306, length=0.032), origin=Origin(xyz=(0.697, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)), material=glass, name="front_lens")
    spotlight_head.visual(front_guard_ring, origin=Origin(xyz=(0.724, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)), material=charcoal, name="front_guard_ring")
    spotlight_head.visual(Cylinder(radius=0.012, length=0.68), origin=Origin(xyz=(0.740, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)), material=charcoal, name="guard_bar_vertical")
    spotlight_head.visual(Box((0.022, 0.64, 0.022)), origin=Origin(xyz=(0.740, 0.0, 0.0)), material=charcoal, name="guard_bar_horizontal")
    spotlight_head.visual(Sphere(radius=0.050), origin=Origin(xyz=(0.39, 0.0, 0.0)), material=warm_bulb, name="lamp_bulb")
    for side, y in enumerate((-0.32, 0.32)):
        spotlight_head.visual(
            Cylinder(radius=0.155, length=0.080),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_olive,
            name=f"trunnion_boss_{side}",
        )
    for i in range(8):
        angle = (2.0 * math.pi * i) / 8.0
        y = 0.335 * math.cos(angle)
        z = 0.335 * math.sin(angle)
        spotlight_head.visual(
            Cylinder(radius=0.016, length=0.020),
            origin=Origin(xyz=(0.745, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=worn_steel,
            name=f"front_clamp_bolt_{i}",
        )

    model.articulation(
        "pan_bearing",
        ArticulationType.CONTINUOUS,
        parent=tower,
        child=pan_stage,
        origin=Origin(xyz=(0.0, 0.0, 4.25)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=260.0, velocity=0.55),
    )
    model.articulation(
        "tilt_axis",
        ArticulationType.REVOLUTE,
        parent=pan_stage,
        child=spotlight_head,
        origin=Origin(xyz=(0.0, 0.0, 0.98)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=140.0, velocity=0.65, lower=-0.35, upper=0.90),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower = object_model.get_part("tower")
    pan_stage = object_model.get_part("pan_stage")
    spotlight_head = object_model.get_part("spotlight_head")
    tilt_axis = object_model.get_articulation("tilt_axis")

    ctx.allow_overlap(
        pan_stage,
        spotlight_head,
        elem_a="tilt_bearing_0",
        elem_b="trunnion_shaft",
        reason="The spotlight trunnion shaft is intentionally captured inside the yoke bearing collar.",
    )
    ctx.allow_overlap(
        pan_stage,
        spotlight_head,
        elem_a="tilt_bearing_1",
        elem_b="trunnion_shaft",
        reason="The opposite yoke bearing collar intentionally captures the same trunnion shaft.",
    )

    ctx.expect_gap(
        pan_stage,
        tower,
        axis="z",
        positive_elem="turntable_bottom",
        negative_elem="top_bearing_plate",
        max_gap=0.001,
        max_penetration=0.0,
        name="turntable sits on top bearing",
    )
    ctx.expect_overlap(
        pan_stage,
        tower,
        axes="xy",
        elem_a="turntable_bottom",
        elem_b="top_bearing_plate",
        min_overlap=0.25,
        name="pan hub centered on tower bearing",
    )
    for bearing in ("tilt_bearing_0", "tilt_bearing_1"):
        ctx.expect_within(
            spotlight_head,
            pan_stage,
            axes="xz",
            inner_elem="trunnion_shaft",
            outer_elem=bearing,
            margin=0.0,
            name=f"{bearing} surrounds shaft radially",
        )
        ctx.expect_overlap(
            spotlight_head,
            pan_stage,
            axes="y",
            elem_a="trunnion_shaft",
            elem_b=bearing,
            min_overlap=0.13,
            name=f"{bearing} retains shaft insertion",
        )

    rest_aabb = ctx.part_element_world_aabb(spotlight_head, elem="front_lens")
    rest_front_z = (rest_aabb[0][2] + rest_aabb[1][2]) * 0.5 if rest_aabb else None
    with ctx.pose({tilt_axis: 0.75}):
        raised_aabb = ctx.part_element_world_aabb(spotlight_head, elem="front_lens")
        raised_front_z = (raised_aabb[0][2] + raised_aabb[1][2]) * 0.5 if raised_aabb else None
    ctx.check(
        "positive tilt raises spotlight lens",
        rest_front_z is not None and raised_front_z is not None and raised_front_z > rest_front_z + 0.30,
        details=f"rest_front_z={rest_front_z}, raised_front_z={raised_front_z}",
    )

    return ctx.report()


object_model = build_object_model()
