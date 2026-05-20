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
)


def _add_box(part, name, size, xyz, material):
    part.visual(Box(size), origin=Origin(xyz=xyz), material=material, name=name)


def _add_door(model, cabinet, *, name, joint_name, hinge_z, door_x):
    steel = Material("brushed_door_steel", rgba=(0.50, 0.51, 0.49, 1.0))
    dark_steel = Material("dark_hinge_steel", rgba=(0.06, 0.065, 0.065, 1.0))
    glass = Material("smoked_heat_glass", rgba=(0.03, 0.055, 0.07, 0.72))
    handle_mat = Material("satin_handle_bar", rgba=(0.68, 0.68, 0.64, 1.0))

    door = model.part(name)

    # The child frame is the bottom hinge line.  Door metal extends upward in
    # local +Z and sits just proud of the cabinet front in local -Y.
    _add_box(door, "bottom_rail", (1.14, 0.055, 0.110), (0.0, 0.0, 0.055), steel)
    _add_box(door, "top_rail", (1.14, 0.055, 0.110), (0.0, 0.0, 0.505), steel)
    _add_box(door, "side_stile_0", (0.135, 0.055, 0.560), (-0.5025, 0.0, 0.280), steel)
    _add_box(door, "side_stile_1", (0.135, 0.055, 0.560), (0.5025, 0.0, 0.280), steel)
    _add_box(door, "glass_pane", (0.900, 0.020, 0.355), (0.0, -0.022, 0.292), glass)

    # A continuous lower hinge knuckle and two handle standoffs are made part of
    # the moving door, visibly attached to the lower rail and handle bar.
    door.visual(
        Cylinder(radius=0.026, length=1.10),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="hinge_knuckle",
    )
    door.visual(
        Cylinder(radius=0.018, length=0.86),
        origin=Origin(xyz=(0.0, -0.087, 0.415), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=handle_mat,
        name="handle_bar",
    )
    for i, x in enumerate((-0.37, 0.37)):
        door.visual(
            Cylinder(radius=0.012, length=0.066),
            origin=Origin(xyz=(x, -0.055, 0.415), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=handle_mat,
            name=f"handle_post_{i}",
        )

    model.articulation(
        joint_name,
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=door,
        origin=Origin(xyz=(door_x, -0.5625, hinge_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=70.0, velocity=1.4, lower=0.0, upper=1.45),
    )
    return door


def _add_rack(model, cabinet, *, name, joint_name, z, rail_prefix, x_center):
    rack_mat = Material("pale_firebrick_hearth", rgba=(0.78, 0.70, 0.58, 1.0))
    runner_mat = Material("blackened_rack_runners", rgba=(0.08, 0.075, 0.065, 1.0))
    lip_mat = Material("brushed_front_lip", rgba=(0.62, 0.62, 0.58, 1.0))

    rack = model.part(name)
    _add_box(rack, "hearth_slab", (1.02, 0.70, 0.036), (0.0, 0.000, 0.0175), rack_mat)
    _add_box(rack, "side_runner_0", (0.042, 0.800, 0.040), (-0.480, 0.0, -0.020), runner_mat)
    _add_box(rack, "side_runner_1", (0.042, 0.800, 0.040), (0.480, 0.0, -0.020), runner_mat)
    _add_box(rack, "front_lip", (1.02, 0.036, 0.060), (0.0, -0.365, 0.020), lip_mat)
    for i, y in enumerate((-0.18, 0.0, 0.18)):
        _add_box(rack, f"cross_bar_{i}", (1.02, 0.018, 0.018), (0.0, y, 0.041), runner_mat)

    model.articulation(
        joint_name,
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=rack,
        origin=Origin(xyz=(x_center, 0.020, z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.35, lower=0.0, upper=0.45),
    )

    # Store fixed guide names for the tests without adding another dependency.
    rack.meta["guide_visuals"] = (f"{rail_prefix}_left_guide", f"{rail_prefix}_right_guide")
    return rack


def _add_control_knob(model, cabinet, *, name, joint_name, xyz):
    knob_mat = Material("black_bakelite_knob", rgba=(0.015, 0.015, 0.014, 1.0))
    mark_mat = Material("white_pointer_mark", rgba=(0.92, 0.90, 0.82, 1.0))

    knob = model.part(name)
    knob.visual(
        Cylinder(radius=0.046, length=0.060),
        origin=Origin(xyz=(0.0, -0.030, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=knob_mat,
        name="knob_body",
    )
    _add_box(knob, "pointer_mark", (0.010, 0.006, 0.040), (0.0, -0.063, 0.020), mark_mat)

    model.articulation(
        joint_name,
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=knob,
        origin=Origin(xyz=xyz),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=0.6, velocity=4.0),
    )
    return knob


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="commercial_double_deck_oven")

    stainless = Material("brushed_stainless_steel", rgba=(0.58, 0.59, 0.56, 1.0))
    dark_liner = Material("dark_refractory_liner", rgba=(0.045, 0.043, 0.039, 1.0))
    black_steel = Material("blackened_guide_steel", rgba=(0.025, 0.025, 0.023, 1.0))
    rubber = Material("black_rubber_feet", rgba=(0.02, 0.02, 0.018, 1.0))
    label = Material("screen_printed_labels", rgba=(0.88, 0.86, 0.75, 1.0))
    amber = Material("amber_pilot_lamp", rgba=(1.0, 0.48, 0.08, 1.0))

    cabinet = model.part("cabinet")

    # Main rectangular commercial cabinet, built as a connected hollow shell
    # around two dark-lined baking chambers.
    _add_box(cabinet, "left_outer_side", (0.055, 1.060, 1.450), (-0.7225, 0.000, 0.905), stainless)
    _add_box(cabinet, "right_outer_side", (0.055, 1.060, 1.450), (0.7225, 0.000, 0.905), stainless)
    _add_box(cabinet, "back_panel", (1.500, 0.055, 1.450), (0.000, 0.5025, 0.905), stainless)
    _add_box(cabinet, "top_cap", (1.500, 1.060, 0.070), (0.000, 0.000, 1.595), stainless)
    _add_box(cabinet, "bottom_cap", (1.500, 1.060, 0.070), (0.000, 0.000, 0.215), stainless)
    _add_box(cabinet, "middle_deck_divider", (1.500, 1.060, 0.075), (0.000, 0.000, 0.865), stainless)

    # Front frame with a right-hand control column and two large oven openings.
    _add_box(cabinet, "left_front_jamb", (0.065, 0.060, 1.450), (-0.7175, -0.505, 0.905), stainless)
    _add_box(cabinet, "inner_front_jamb", (0.065, 0.060, 1.450), (0.455, -0.505, 0.905), stainless)
    _add_box(cabinet, "control_column", (0.255, 0.060, 1.450), (0.620, -0.505, 0.905), stainless)
    _add_box(cabinet, "front_top_rail", (1.200, 0.060, 0.100), (-0.130, -0.505, 1.565), stainless)
    _add_box(cabinet, "front_bottom_rail", (1.200, 0.060, 0.100), (-0.130, -0.505, 0.245), stainless)
    _add_box(cabinet, "front_middle_rail", (1.200, 0.060, 0.100), (-0.130, -0.505, 0.865), stainless)

    chamber_x = -0.130
    for prefix, chamber_z, rack_z in (("lower", 0.560, 0.405), ("upper", 1.170, 1.015)):
        _add_box(cabinet, f"{prefix}_back_liner", (1.090, 0.035, 0.510), (chamber_x, 0.4575, chamber_z), dark_liner)
        _add_box(cabinet, f"{prefix}_left_liner", (0.040, 0.920, 0.510), (-0.675, -0.010, chamber_z), dark_liner)
        _add_box(cabinet, f"{prefix}_right_liner", (0.035, 0.920, 0.510), (0.405, -0.010, chamber_z), dark_liner)
        _add_box(cabinet, f"{prefix}_floor_liner", (1.090, 0.920, 0.026), (chamber_x, -0.010, chamber_z - 0.250), dark_liner)
        _add_box(cabinet, f"{prefix}_ceiling_liner", (1.090, 0.920, 0.026), (chamber_x, -0.010, chamber_z + 0.250), dark_liner)
        _add_box(cabinet, f"{prefix}_left_guide", (0.070, 0.760, 0.040), (-0.620, 0.040, rack_z - 0.060), black_steel)
        _add_box(cabinet, f"{prefix}_right_guide", (0.070, 0.760, 0.040), (0.360, 0.040, rack_z - 0.060), black_steel)

    # Small service stand/feet are tied into the bottom cap.
    _add_box(cabinet, "front_toe_kick", (1.42, 0.080, 0.110), (0.0, -0.470, 0.105), stainless)
    for i, x in enumerate((-0.58, 0.58)):
        for j, y in enumerate((-0.38, 0.38)):
            _add_box(cabinet, f"foot_{i}_{j}", (0.105, 0.105, 0.180), (x, y, 0.090), rubber)

    # Printed control legends and pilot lights are flush-mounted on the column.
    for i, (z, text_y) in enumerate(((1.255, 1.0), (0.645, 0.0))):
        _add_box(cabinet, f"temperature_label_{i}", (0.150, 0.006, 0.030), (0.620, -0.538, z + 0.090), label)
        _add_box(cabinet, f"pilot_lamp_{i}", (0.030, 0.008, 0.030), (0.700, -0.539, z - 0.085 + 0.0 * text_y), amber)

    _add_door(model, cabinet, name="door_0", joint_name="cabinet_to_door_0", hinge_z=0.295, door_x=chamber_x)
    _add_door(model, cabinet, name="door_1", joint_name="cabinet_to_door_1", hinge_z=0.905, door_x=chamber_x)
    _add_rack(model, cabinet, name="rack_0", joint_name="cabinet_to_rack_0", z=0.405, rail_prefix="lower", x_center=chamber_x)
    _add_rack(model, cabinet, name="rack_1", joint_name="cabinet_to_rack_1", z=1.015, rail_prefix="upper", x_center=chamber_x)
    _add_control_knob(model, cabinet, name="knob_0", joint_name="cabinet_to_knob_0", xyz=(0.620, -0.535, 1.255))
    _add_control_knob(model, cabinet, name="knob_1", joint_name="cabinet_to_knob_1", xyz=(0.620, -0.535, 0.645))

    return model


def _aabb_center(aabb, axis_index):
    return 0.5 * (aabb[0][axis_index] + aabb[1][axis_index])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")

    for i, door_name in enumerate(("door_0", "door_1")):
        door = object_model.get_part(door_name)
        hinge = object_model.get_articulation(f"cabinet_to_{door_name}")
        limits = hinge.motion_limits
        ctx.check(
            f"{door_name} has drop-down hinge travel",
            hinge.articulation_type == ArticulationType.REVOLUTE
            and limits is not None
            and limits.lower == 0.0
            and limits.upper is not None
            and limits.upper > 1.2,
            details=f"hinge={hinge}",
        )
        ctx.expect_gap(
            cabinet,
            door,
            axis="y",
            max_gap=0.006,
            max_penetration=0.0,
            positive_elem="inner_front_jamb",
            negative_elem="side_stile_1",
            name=f"{door_name} closes just proud of the front frame",
        )
        closed_top = ctx.part_element_world_aabb(door, elem="top_rail")
        with ctx.pose({hinge: limits.upper}):
            open_top = ctx.part_element_world_aabb(door, elem="top_rail")
        ctx.check(
            f"{door_name} top edge drops outward",
            closed_top is not None
            and open_top is not None
            and _aabb_center(open_top, 2) < _aabb_center(closed_top, 2) - 0.18
            and _aabb_center(open_top, 1) < _aabb_center(closed_top, 1) - 0.25,
            details=f"closed={closed_top}, open={open_top}",
        )

    for rack_name, prefix in (("rack_0", "lower"), ("rack_1", "upper")):
        rack = object_model.get_part(rack_name)
        slide = object_model.get_articulation(f"cabinet_to_{rack_name}")
        limits = slide.motion_limits
        ctx.check(
            f"{rack_name} has front-sliding travel",
            slide.articulation_type == ArticulationType.PRISMATIC
            and limits is not None
            and limits.lower == 0.0
            and limits.upper is not None
            and limits.upper >= 0.40,
            details=f"slide={slide}",
        )
        ctx.expect_contact(
            rack,
            cabinet,
            elem_a="side_runner_0",
            elem_b=f"{prefix}_left_guide",
            contact_tol=0.001,
            name=f"{rack_name} left runner sits on its guide",
        )
        ctx.expect_contact(
            rack,
            cabinet,
            elem_a="side_runner_1",
            elem_b=f"{prefix}_right_guide",
            contact_tol=0.001,
            name=f"{rack_name} right runner sits on its guide",
        )
        ctx.expect_overlap(
            rack,
            cabinet,
            axes="y",
            min_overlap=0.50,
            elem_a="side_runner_0",
            elem_b=f"{prefix}_left_guide",
            name=f"{rack_name} is deeply supported when stowed",
        )
        rest_aabb = ctx.part_element_world_aabb(rack, elem="front_lip")
        with ctx.pose({slide: limits.upper}):
            ctx.expect_overlap(
                rack,
                cabinet,
                axes="y",
                min_overlap=0.25,
                elem_a="side_runner_0",
                elem_b=f"{prefix}_left_guide",
                name=f"{rack_name} remains captured when extended",
            )
            extended_aabb = ctx.part_element_world_aabb(rack, elem="front_lip")
        ctx.check(
            f"{rack_name} slides out toward the operator",
            rest_aabb is not None
            and extended_aabb is not None
            and _aabb_center(extended_aabb, 1) < _aabb_center(rest_aabb, 1) - 0.35,
            details=f"rest={rest_aabb}, extended={extended_aabb}",
        )

    return ctx.report()


object_model = build_object_model()
