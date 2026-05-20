from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _box_wp(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _union_all(solids: list[cq.Workplane]) -> cq.Workplane:
    body = solids[0]
    for solid in solids[1:]:
        body = body.union(solid)
    return body


def _rack_geometry(
    *,
    width: float,
    depth: float,
    height: float,
    rod: float,
    tine_height: float,
    name: str,
) -> object:
    """Fused rectangular-rod rack: open wire volume with welded intersections."""

    solids: list[cq.Workplane] = []
    half_w = width / 2.0
    half_d = depth / 2.0

    # Bottom welded grid.
    for x in (-0.18, -0.09, 0.0, 0.09, 0.18):
        if abs(x) < half_w:
            solids.append(_box_wp((rod, depth, rod), (x, 0.0, rod / 2.0)))
    for y in (-0.18, -0.09, 0.0, 0.09, 0.18):
        if abs(y) < half_d:
            solids.append(_box_wp((width, rod, rod), (0.0, y, rod / 2.0)))

    # Lower perimeter and upper basket rails.
    for z in (rod / 2.0, height):
        solids.extend(
            [
                _box_wp((rod, depth, rod), (-half_w, 0.0, z)),
                _box_wp((rod, depth, rod), (half_w, 0.0, z)),
                _box_wp((width, rod, rod), (0.0, -half_d, z)),
                _box_wp((width, rod, rod), (0.0, half_d, z)),
            ]
        )

    # Corner posts and small front/back hoops.
    for x in (-half_w, half_w):
        for y in (-half_d, half_d):
            solids.append(_box_wp((rod, rod, height), (x, y, height / 2.0)))

    # Dish tines welded into the bottom grid.
    for x in (-0.18, -0.09, 0.0, 0.09, 0.18):
        for y in (-0.18, 0.0, 0.18):
            if abs(x) < half_w - 0.03 and abs(y) < half_d - 0.03:
                solids.append(
                    _box_wp(
                        (rod * 0.75, rod * 0.75, tine_height),
                        (x, y, tine_height / 2.0),
                    )
                )

    return _union_all(solids)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="residential_dishwasher")

    stainless = model.material("brushed_stainless", rgba=(0.68, 0.70, 0.68, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.08, 0.085, 0.09, 1.0))
    black = model.material("black_plastic", rgba=(0.015, 0.015, 0.017, 1.0))
    liner = model.material("molded_gray_liner", rgba=(0.70, 0.74, 0.76, 1.0))
    rack_mat = model.material("vinyl_wire_rack", rgba=(0.88, 0.90, 0.88, 1.0))
    blue_gray = model.material("blue_gray_plastic", rgba=(0.35, 0.43, 0.47, 1.0))
    button_mat = model.material("charcoal_buttons", rgba=(0.03, 0.034, 0.04, 1.0))

    cabinet = model.part("cabinet")
    # Under-counter body: about 24 in wide, 34 in high, 24 in deep.
    cabinet.visual(Box((0.025, 0.68, 0.78)), origin=Origin(xyz=(-0.31, -0.02, 0.47)), material=dark_steel, name="side_panel_0")
    cabinet.visual(Box((0.025, 0.68, 0.78)), origin=Origin(xyz=(0.31, -0.02, 0.47)), material=dark_steel, name="side_panel_1")
    cabinet.visual(Box((0.62, 0.025, 0.78)), origin=Origin(xyz=(0.0, 0.315, 0.47)), material=dark_steel, name="rear_panel")
    cabinet.visual(Box((0.62, 0.68, 0.035)), origin=Origin(xyz=(0.0, -0.02, 0.862)), material=dark_steel, name="countertop_shadow")
    cabinet.visual(Box((0.62, 0.68, 0.08)), origin=Origin(xyz=(0.0, -0.02, 0.04)), material=black, name="toe_kick")

    # Thin molded tub walls with an open front mouth.
    cabinet.visual(Box((0.025, 0.58, 0.70)), origin=Origin(xyz=(-0.285, -0.02, 0.47)), material=liner, name="tub_wall_0")
    cabinet.visual(Box((0.025, 0.58, 0.70)), origin=Origin(xyz=(0.285, -0.02, 0.47)), material=liner, name="tub_wall_1")
    cabinet.visual(Box((0.56, 0.025, 0.70)), origin=Origin(xyz=(0.0, 0.29, 0.47)), material=liner, name="tub_rear_wall")
    cabinet.visual(Box((0.56, 0.58, 0.020)), origin=Origin(xyz=(0.0, -0.02, 0.11)), material=liner, name="tub_floor")
    cabinet.visual(Box((0.56, 0.58, 0.020)), origin=Origin(xyz=(0.0, -0.02, 0.82)), material=liner, name="tub_ceiling")
    cabinet.visual(Box((0.045, 0.025, 0.72)), origin=Origin(xyz=(-0.285, -0.323, 0.47)), material=black, name="front_gasket_0")
    cabinet.visual(Box((0.045, 0.025, 0.72)), origin=Origin(xyz=(0.285, -0.323, 0.47)), material=black, name="front_gasket_1")
    cabinet.visual(Box((0.58, 0.025, 0.035)), origin=Origin(xyz=(0.0, -0.323, 0.82)), material=black, name="front_gasket_top")
    cabinet.visual(Box((0.58, 0.025, 0.030)), origin=Origin(xyz=(0.0, -0.323, 0.12)), material=black, name="front_gasket_bottom")

    # Side rails support sliding rack frames.
    for idx, x in enumerate((-0.255, 0.255)):
        cabinet.visual(Box((0.035, 0.52, 0.012)), origin=Origin(xyz=(x, -0.03, 0.204)), material=stainless, name=f"lower_rail_{idx}")
        cabinet.visual(Box((0.035, 0.50, 0.012)), origin=Origin(xyz=(x, -0.03, 0.494)), material=stainless, name=f"upper_rail_{idx}")
        cabinet.visual(Box((0.035, 0.045, 0.028)), origin=Origin(xyz=(x, -0.275, 0.204)), material=stainless, name=f"lower_roller_{idx}")
        cabinet.visual(Box((0.035, 0.045, 0.028)), origin=Origin(xyz=(x, -0.275, 0.494)), material=stainless, name=f"upper_roller_{idx}")

    # Water feed and hinge hardware remain on the fixed cabinet side.
    cabinet.visual(Cylinder(radius=0.008, length=0.60), origin=Origin(xyz=(0.0, -0.35, 0.10), rpy=(0.0, math.pi / 2.0, 0.0)), material=dark_steel, name="door_hinge_pin")
    cabinet.visual(Box((0.040, 0.055, 0.050)), origin=Origin(xyz=(-0.292, -0.35, 0.10)), material=dark_steel, name="hinge_bracket_0")
    cabinet.visual(Box((0.040, 0.055, 0.050)), origin=Origin(xyz=(0.292, -0.35, 0.10)), material=dark_steel, name="hinge_bracket_1")
    cabinet.visual(Cylinder(radius=0.018, length=0.034), origin=Origin(xyz=(0.0, -0.02, 0.137)), material=blue_gray, name="lower_spray_pedestal")
    cabinet.visual(Cylinder(radius=0.012, length=0.012), origin=Origin(xyz=(0.0, -0.02, 0.415)), material=blue_gray, name="upper_spray_nipple")
    cabinet.visual(Cylinder(radius=0.010, length=0.31), origin=Origin(xyz=(0.0, 0.135, 0.409), rpy=(-math.pi / 2.0, 0.0, 0.0)), material=blue_gray, name="upper_water_tube")

    door = model.part("door")
    door.visual(Box((0.60, 0.715, 0.043)), origin=Origin(xyz=(0.0, -0.3825, -0.0185)), material=stainless, name="stainless_panel")
    door.visual(Box((0.54, 0.58, 0.018)), origin=Origin(xyz=(0.0, -0.36, 0.009)), material=liner, name="inner_liner")
    door.visual(Box((0.56, 0.090, 0.020)), origin=Origin(xyz=(0.0, -0.695, 0.018)), material=black, name="control_strip")
    door.visual(Box((0.30, 0.030, 0.016)), origin=Origin(xyz=(0.0, -0.63, -0.043)), material=black, name="recess_handle")
    door.visual(Cylinder(radius=0.014, length=0.50), origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)), material=stainless, name="door_hinge_knuckle")
    door.visual(Box((0.52, 0.050, 0.008)), origin=Origin(xyz=(0.0, -0.025, -0.018)), material=stainless, name="hinge_leaf")
    door.visual(Box((0.18, 0.13, 0.012)), origin=Origin(xyz=(-0.14, -0.315, 0.020)), material=blue_gray, name="detergent_recess")
    door.visual(Box((0.11, 0.07, 0.008)), origin=Origin(xyz=(0.12, -0.36, 0.021)), material=blue_gray, name="rinse_aid_cap")

    door_hinge = model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=door,
        origin=Origin(xyz=(0.0, -0.35, 0.10)),
        axis=(1.0, 0.0, 0.0),
        # q=0 is a service-open door lying downward; increasing from the
        # closed negative limit rotates the front door downward to open.
        motion_limits=MotionLimits(effort=80.0, velocity=1.0, lower=-1.45, upper=0.0),
    )

    # Separate movable top-edge controls.
    for i, x in enumerate((-0.225, -0.165, -0.105, -0.045, 0.015, 0.075)):
        button = model.part(f"button_{i}")
        button.visual(Box((0.044, 0.026, 0.010)), origin=Origin(xyz=(0.0, 0.0, 0.005)), material=button_mat, name="button_cap")
        model.articulation(
            f"door_to_button_{i}",
            ArticulationType.PRISMATIC,
            parent=door,
            child=button,
            origin=Origin(xyz=(x, -0.695, 0.028)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(effort=4.0, velocity=0.08, lower=0.0, upper=0.006),
        )

    cycle_knob = model.part("cycle_knob")
    knob_mesh = mesh_from_geometry(
        KnobGeometry(
            0.056,
            0.020,
            body_style="faceted",
            grip=KnobGrip(style="ribbed", count=16, depth=0.0012),
            indicator=KnobIndicator(style="line", mode="raised", depth=0.0008),
            center=False,
        ),
        "cycle_selector_knob",
    )
    cycle_knob.visual(knob_mesh, origin=Origin(), material=button_mat, name="knob_cap")
    model.articulation(
        "door_to_knob",
        ArticulationType.CONTINUOUS,
        parent=door,
        child=cycle_knob,
        origin=Origin(xyz=(0.195, -0.695, 0.028)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=4.0),
    )

    cup_cover = model.part("detergent_cover")
    cup_cover.visual(Box((0.155, 0.105, 0.008)), origin=Origin(xyz=(0.0, -0.0525, 0.004)), material=blue_gray, name="cover_panel")
    cup_cover.visual(Box((0.050, 0.010, 0.008)), origin=Origin(xyz=(0.0, -0.095, 0.012)), material=button_mat, name="cover_latch")
    model.articulation(
        "door_to_cover",
        ArticulationType.REVOLUTE,
        parent=door,
        child=cup_cover,
        origin=Origin(xyz=(-0.14, -0.2625, 0.026)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=1.5, lower=0.0, upper=1.2),
    )

    lower_rack = model.part("lower_rack")
    lower_rack.visual(mesh_from_cadquery(_rack_geometry(width=0.51, depth=0.46, height=0.135, rod=0.008, tine_height=0.105, name="lower_rack"), "lower_wire_rack", tolerance=0.002), material=rack_mat, name="wire_basket")
    model.articulation(
        "cabinet_to_lower_rack",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=lower_rack,
        origin=Origin(xyz=(0.0, -0.02, 0.210)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=0.35, lower=0.0, upper=0.36),
    )

    upper_rack = model.part("upper_rack")
    upper_rack.visual(mesh_from_cadquery(_rack_geometry(width=0.51, depth=0.43, height=0.115, rod=0.007, tine_height=0.070, name="upper_rack"), "upper_wire_rack", tolerance=0.002), material=rack_mat, name="wire_basket")
    model.articulation(
        "cabinet_to_upper_rack",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=upper_rack,
        origin=Origin(xyz=(0.0, -0.02, 0.500)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=0.30, lower=0.0, upper=0.30),
    )

    def _spray_arm(part_name: str, joint_name: str, z: float, arm_len: float) -> None:
        spray = model.part(part_name)
        spray.visual(Box((arm_len, 0.036, 0.012)), origin=Origin(xyz=(0.0, 0.0, 0.0)), material=blue_gray, name="spray_blade")
        spray.visual(Box((0.055, arm_len * 0.45, 0.010)), origin=Origin(xyz=(0.0, 0.0, 0.002)), material=blue_gray, name="cross_blade")
        spray.visual(Cylinder(radius=0.035, length=0.024), origin=Origin(xyz=(0.0, 0.0, 0.0)), material=blue_gray, name="spray_hub")
        for idx, x in enumerate((-arm_len * 0.34, -arm_len * 0.17, arm_len * 0.17, arm_len * 0.34)):
            spray.visual(Box((0.012, 0.010, 0.004)), origin=Origin(xyz=(x, 0.013, 0.008)), material=black, name=f"jet_nozzle_{idx}")
        model.articulation(
            joint_name,
            ArticulationType.CONTINUOUS,
            parent=cabinet,
            child=spray,
            origin=Origin(xyz=(0.0, -0.02, z)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=0.8, velocity=20.0),
        )

    _spray_arm("lower_spray_arm", "cabinet_to_lower_spray", 0.165, 0.42)
    _spray_arm("upper_spray_arm", "cabinet_to_upper_spray", 0.430, 0.34)

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    door = object_model.get_part("door")
    lower_rack = object_model.get_part("lower_rack")
    upper_rack = object_model.get_part("upper_rack")
    cover = object_model.get_part("detergent_cover")
    door_hinge = object_model.get_articulation("door_hinge")
    lower_slide = object_model.get_articulation("cabinet_to_lower_rack")
    upper_slide = object_model.get_articulation("cabinet_to_upper_rack")
    cover_hinge = object_model.get_articulation("door_to_cover")

    ctx.allow_overlap(
        cabinet,
        door,
        elem_a="door_hinge_pin",
        elem_b="door_hinge_knuckle",
        reason="The fixed hinge pin is intentionally captured inside the rotating door knuckle.",
    )
    ctx.expect_within(
        cabinet,
        door,
        axes="yz",
        inner_elem="door_hinge_pin",
        outer_elem="door_hinge_knuckle",
        margin=0.003,
        name="hinge pin sits inside door knuckle",
    )
    ctx.expect_overlap(
        cabinet,
        door,
        axes="x",
        elem_a="door_hinge_pin",
        elem_b="door_hinge_knuckle",
        min_overlap=0.45,
        name="hinge pin spans the knuckle",
    )

    # The open service pose should show the deep hollow tub and door lying down.
    open_aabb = ctx.part_world_aabb(door)
    ctx.check(
        "door opens downward at rest",
        open_aabb is not None and open_aabb[0][1] < -0.95 and open_aabb[1][2] < 0.16,
        details=f"open_aabb={open_aabb}",
    )
    with ctx.pose({door_hinge: -1.45}):
        closed_aabb = ctx.part_world_aabb(door)
        ctx.check(
            "door closes to full height",
            closed_aabb is not None and closed_aabb[1][2] > 0.78 and closed_aabb[0][1] < -0.47,
            details=f"closed_aabb={closed_aabb}",
        )

    # Rack sliders move out toward the user while retained in the tub.
    lower_rest = ctx.part_world_position(lower_rack)
    with ctx.pose({lower_slide: 0.34}):
        lower_extended = ctx.part_world_position(lower_rack)
        ctx.expect_overlap(lower_rack, cabinet, axes="y", min_overlap=0.10, name="lower rack remains partly inserted")
    ctx.check(
        "lower rack slides forward",
        lower_rest is not None and lower_extended is not None and lower_extended[1] < lower_rest[1] - 0.30,
        details=f"rest={lower_rest}, extended={lower_extended}",
    )

    upper_rest = ctx.part_world_position(upper_rack)
    with ctx.pose({upper_slide: 0.28}):
        upper_extended = ctx.part_world_position(upper_rack)
        ctx.expect_overlap(upper_rack, cabinet, axes="y", min_overlap=0.12, name="upper rack remains partly inserted")
    ctx.check(
        "upper rack slides forward",
        upper_rest is not None and upper_extended is not None and upper_extended[1] < upper_rest[1] - 0.24,
        details=f"rest={upper_rest}, extended={upper_extended}",
    )

    # Detergent cup cover flips up from the inner door.
    cover_rest = ctx.part_world_aabb(cover)
    with ctx.pose({cover_hinge: 1.0}):
        cover_open = ctx.part_world_aabb(cover)
    ctx.check(
        "detergent cover opens upward",
        cover_rest is not None and cover_open is not None and cover_open[1][2] > cover_rest[1][2] + 0.025,
        details=f"rest={cover_rest}, opened={cover_open}",
    )

    ctx.check(
        "all visible controls are articulated",
        all(object_model.get_articulation(f"door_to_button_{i}") is not None for i in range(6))
        and object_model.get_articulation("door_to_knob").articulation_type == ArticulationType.CONTINUOUS,
        details="six prismatic buttons and one continuous selector knob are present",
    )

    return ctx.report()


object_model = build_object_model()
