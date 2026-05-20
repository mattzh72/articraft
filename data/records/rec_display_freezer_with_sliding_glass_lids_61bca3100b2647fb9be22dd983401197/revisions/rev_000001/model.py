from __future__ import annotations

import math

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
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="store_convenience_chest_freezer")

    white = model.material("powder_coated_white", rgba=(0.88, 0.91, 0.92, 1.0))
    liner = model.material("deep_freezer_liner", rgba=(0.18, 0.23, 0.27, 1.0))
    aluminum = model.material("brushed_aluminum", rgba=(0.64, 0.67, 0.68, 1.0))
    dark = model.material("black_rubber", rgba=(0.03, 0.035, 0.04, 1.0))
    glass = model.material("green_tinted_glass", rgba=(0.58, 0.86, 0.92, 0.38))
    lock_metal = model.material("lock_brushed_metal", rgba=(0.78, 0.76, 0.70, 1.0))
    knob_plastic = model.material("warm_gray_knob", rgba=(0.28, 0.28, 0.26, 1.0))

    # Root part: a store-scale insulated chest freezer cabinet, modeled as a
    # hollow body with an open top, dark inner tub, raised rim, and lid tracks.
    cabinet = model.part("cabinet")
    cabinet.visual(Box((2.00, 0.90, 0.10)), origin=Origin(xyz=(0.0, 0.0, 0.05)), material=white, name="base_plinth")
    cabinet.visual(Box((2.00, 0.07, 0.78)), origin=Origin(xyz=(0.0, -0.415, 0.45)), material=white, name="front_wall")
    cabinet.visual(Box((2.00, 0.07, 0.78)), origin=Origin(xyz=(0.0, 0.415, 0.45)), material=white, name="rear_wall")
    cabinet.visual(Box((0.07, 0.90, 0.78)), origin=Origin(xyz=(-0.965, 0.0, 0.45)), material=white, name="end_wall_0")
    cabinet.visual(Box((0.07, 0.90, 0.78)), origin=Origin(xyz=(0.965, 0.0, 0.45)), material=white, name="end_wall_1")

    cabinet.visual(Box((1.82, 0.72, 0.025)), origin=Origin(xyz=(0.0, 0.0, 0.105)), material=liner, name="inner_floor")
    cabinet.visual(Box((1.82, 0.020, 0.62)), origin=Origin(xyz=(0.0, -0.371, 0.425)), material=liner, name="front_liner")
    cabinet.visual(Box((1.82, 0.020, 0.62)), origin=Origin(xyz=(0.0, 0.371, 0.425)), material=liner, name="rear_liner")
    cabinet.visual(Box((0.020, 0.72, 0.62)), origin=Origin(xyz=(-0.922, 0.0, 0.425)), material=liner, name="end_liner_0")
    cabinet.visual(Box((0.020, 0.72, 0.62)), origin=Origin(xyz=(0.922, 0.0, 0.425)), material=liner, name="end_liner_1")

    cabinet.visual(Box((2.00, 0.10, 0.06)), origin=Origin(xyz=(0.0, -0.40, 0.84)), material=white, name="front_top_rim")
    cabinet.visual(Box((2.00, 0.10, 0.06)), origin=Origin(xyz=(0.0, 0.40, 0.84)), material=white, name="rear_top_rim")
    cabinet.visual(Box((0.10, 0.90, 0.06)), origin=Origin(xyz=(-0.95, 0.0, 0.84)), material=white, name="end_top_rim_0")
    cabinet.visual(Box((0.10, 0.90, 0.06)), origin=Origin(xyz=(0.95, 0.0, 0.84)), material=white, name="end_top_rim_1")
    cabinet.visual(Box((1.88, 0.030, 0.030)), origin=Origin(xyz=(0.0, -0.352, 0.883)), material=aluminum, name="front_track")
    cabinet.visual(Box((1.88, 0.034, 0.030)), origin=Origin(xyz=(0.0, 0.0, 0.883)), material=aluminum, name="center_track")
    cabinet.visual(Box((1.88, 0.030, 0.030)), origin=Origin(xyz=(0.0, 0.352, 0.883)), material=aluminum, name="rear_track")
    cabinet.visual(Box((1.86, 0.006, 0.004)), origin=Origin(xyz=(0.0, -0.334, 0.893)), material=dark, name="front_track_gasket")
    cabinet.visual(Box((1.86, 0.006, 0.004)), origin=Origin(xyz=(0.0, 0.334, 0.893)), material=dark, name="rear_track_gasket")

    # Fixed lock details under a rotating protective flap.
    cabinet.visual(
        Cylinder(radius=0.023, length=0.018),
        origin=Origin(xyz=(-0.355, -0.459, 0.680), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=lock_metal,
        name="key_cylinder",
    )
    cabinet.visual(Box((0.032, 0.024, 0.145)), origin=Origin(xyz=(-0.415, -0.462, 0.680)), material=aluminum, name="lock_hinge_leaf")

    # End-wall thermostat graphics and the shaft support for the separate knob.
    cabinet.visual(Box((0.004, 0.150, 0.105)), origin=Origin(xyz=(1.002, -0.265, 0.500)), material=dark, name="control_plate")
    for i, dz in enumerate((-0.035, -0.018, 0.000, 0.018, 0.035)):
        cabinet.visual(
            Box((0.006, 0.030 if i == 2 else 0.020, 0.004)),
            origin=Origin(xyz=(1.006, -0.320 + abs(dz) * 0.45, 0.500 + dz)),
            material=white,
            name=f"temperature_tick_{i}",
        )

    def add_glass_slider(part_name: str, y_center: float, track_y: float) -> object:
        slider = model.part(part_name)
        slider.visual(Box((0.924, 0.324, 0.018)), origin=Origin(xyz=(0.0, 0.0, 0.000)), material=glass, name="glass_pane")
        slider.visual(Box((0.940, 0.018, 0.035)), origin=Origin(xyz=(0.0, -0.170, 0.000)), material=aluminum, name="front_frame")
        slider.visual(Box((0.940, 0.018, 0.035)), origin=Origin(xyz=(0.0, 0.170, 0.000)), material=aluminum, name="rear_frame")
        slider.visual(Box((0.018, 0.340, 0.035)), origin=Origin(xyz=(-0.470, 0.0, 0.000)), material=aluminum, name="end_frame_0")
        slider.visual(Box((0.018, 0.340, 0.035)), origin=Origin(xyz=(0.470, 0.0, 0.000)), material=aluminum, name="end_frame_1")
        slider.meta["track_y"] = track_y
        return slider

    glass_lid_0 = add_glass_slider("glass_lid_0", -0.180, -0.352)
    glass_lid_1 = add_glass_slider("glass_lid_1", 0.180, 0.352)

    model.articulation(
        "cabinet_to_glass_lid_0",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=glass_lid_0,
        origin=Origin(xyz=(-0.390, -0.180, 0.9155)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=0.45, lower=0.0, upper=0.60),
    )
    model.articulation(
        "cabinet_to_glass_lid_1",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=glass_lid_1,
        origin=Origin(xyz=(0.390, 0.180, 0.9155)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=0.45, lower=0.0, upper=0.60),
    )

    lock_flap = model.part("lock_flap")
    lock_flap.visual(Cylinder(radius=0.009, length=0.145), origin=Origin(xyz=(0.0, 0.0, 0.0)), material=lock_metal, name="hinge_barrel")
    lock_flap.visual(Box((0.112, 0.006, 0.128)), origin=Origin(xyz=(0.056, -0.002, 0.0)), material=lock_metal, name="flap_plate")
    lock_flap.visual(Box((0.070, 0.003, 0.035)), origin=Origin(xyz=(0.060, -0.006, 0.0)), material=dark, name="finger_recess")
    model.articulation(
        "cabinet_to_lock_flap",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=lock_flap,
        origin=Origin(xyz=(-0.415, -0.483, 0.680)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=1.5, velocity=2.0, lower=0.0, upper=1.35),
    )

    thermostat_knob = model.part("thermostat_knob")
    thermostat_knob.visual(
        Cylinder(radius=0.012, length=0.032),
        origin=Origin(xyz=(0.016, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=lock_metal,
        name="short_shaft",
    )
    knob_mesh = mesh_from_geometry(
        KnobGeometry(
            0.072,
            0.034,
            body_style="skirted",
            top_diameter=0.056,
            edge_radius=0.0012,
            grip=KnobGrip(style="fluted", count=18, depth=0.0018),
            indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
            center=False,
        ),
        "freezer_temperature_knob",
    )
    thermostat_knob.visual(
        knob_mesh,
        origin=Origin(xyz=(0.028, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=knob_plastic,
        name="knob_cap",
    )
    model.articulation(
        "cabinet_to_thermostat_knob",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=thermostat_knob,
        origin=Origin(xyz=(1.004, -0.265, 0.500)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.8, velocity=6.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    glass_lid_0 = object_model.get_part("glass_lid_0")
    glass_lid_1 = object_model.get_part("glass_lid_1")
    lock_flap = object_model.get_part("lock_flap")
    thermostat_knob = object_model.get_part("thermostat_knob")

    slide_0 = object_model.get_articulation("cabinet_to_glass_lid_0")
    slide_1 = object_model.get_articulation("cabinet_to_glass_lid_1")
    flap_joint = object_model.get_articulation("cabinet_to_lock_flap")
    knob_joint = object_model.get_articulation("cabinet_to_thermostat_knob")

    cabinet_aabb = ctx.part_world_aabb(cabinet)
    ctx.check(
        "cabinet is store freezer scale and deep",
        cabinet_aabb is not None
        and (cabinet_aabb[1][0] - cabinet_aabb[0][0]) >= 1.85
        and (cabinet_aabb[1][1] - cabinet_aabb[0][1]) >= 0.85
        and (cabinet_aabb[1][2] - cabinet_aabb[0][2]) >= 0.85,
        details=f"cabinet_aabb={cabinet_aabb}",
    )

    ctx.expect_gap(
        glass_lid_0,
        cabinet,
        axis="z",
        positive_elem="front_frame",
        negative_elem="front_track",
        max_gap=0.002,
        max_penetration=0.0,
        name="front glass lid rides on front track",
    )
    ctx.expect_gap(
        glass_lid_1,
        cabinet,
        axis="z",
        positive_elem="rear_frame",
        negative_elem="rear_track",
        max_gap=0.002,
        max_penetration=0.0,
        name="rear glass lid rides on rear track",
    )

    rest_0 = ctx.part_world_position(glass_lid_0)
    rest_1 = ctx.part_world_position(glass_lid_1)
    with ctx.pose({slide_0: 0.50, slide_1: 0.50}):
        moved_0 = ctx.part_world_position(glass_lid_0)
        moved_1 = ctx.part_world_position(glass_lid_1)
        ctx.expect_gap(
            glass_lid_0,
            cabinet,
            axis="z",
            positive_elem="front_frame",
            negative_elem="front_track",
            max_gap=0.002,
            max_penetration=0.0,
            name="front glass lid remains supported while slid",
        )
        ctx.expect_gap(
            glass_lid_1,
            cabinet,
            axis="z",
            positive_elem="rear_frame",
            negative_elem="rear_track",
            max_gap=0.002,
            max_penetration=0.0,
            name="rear glass lid remains supported while slid",
        )
    ctx.check(
        "glass lids slide in opposite directions",
        rest_0 is not None
        and rest_1 is not None
        and moved_0 is not None
        and moved_1 is not None
        and moved_0[0] > rest_0[0] + 0.45
        and moved_1[0] < rest_1[0] - 0.45,
        details=f"rest_0={rest_0}, moved_0={moved_0}, rest_1={rest_1}, moved_1={moved_1}",
    )

    closed_flap_aabb = ctx.part_element_world_aabb(lock_flap, elem="flap_plate")
    with ctx.pose({flap_joint: 1.15}):
        open_flap_aabb = ctx.part_element_world_aabb(lock_flap, elem="flap_plate")
    ctx.check(
        "lock flap swings outward from key cylinder",
        closed_flap_aabb is not None
        and open_flap_aabb is not None
        and open_flap_aabb[0][1] < closed_flap_aabb[0][1] - 0.035,
        details=f"closed={closed_flap_aabb}, open={open_flap_aabb}",
    )

    ctx.expect_gap(
        thermostat_knob,
        cabinet,
        axis="x",
        positive_elem="short_shaft",
        negative_elem="control_plate",
        max_gap=0.002,
        max_penetration=0.0,
        name="thermostat knob is mounted on end control panel",
    )
    ctx.check(
        "temperature knob uses continuous rotation",
        knob_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"joint_type={knob_joint.articulation_type}",
    )

    return ctx.report()


object_model = build_object_model()
