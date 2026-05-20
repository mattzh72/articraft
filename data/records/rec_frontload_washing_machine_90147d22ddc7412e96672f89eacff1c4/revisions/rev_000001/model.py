from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    CylinderGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="high_capacity_front_load_washer")

    white = model.material("warm_white_enamel", rgba=(0.92, 0.94, 0.94, 1.0))
    dark = model.material("dark_display_glass", rgba=(0.02, 0.025, 0.03, 1.0))
    rubber = model.material("charcoal_rubber", rgba=(0.025, 0.024, 0.023, 1.0))
    steel = model.material("brushed_stainless", rgba=(0.68, 0.70, 0.69, 1.0))
    chrome = model.material("soft_chrome", rgba=(0.78, 0.80, 0.79, 1.0))
    glass = model.material("smoky_blue_glass", rgba=(0.35, 0.50, 0.62, 0.42))
    shadow = model.material("deep_shadow", rgba=(0.01, 0.012, 0.015, 1.0))

    cabinet = model.part("cabinet")
    # Overall appliance envelope: about 0.78 m wide, 0.72 m deep, and 1.07 m tall.
    cabinet.visual(Box((0.040, 0.720, 1.020)), origin=Origin(xyz=(-0.380, 0.000, 0.540)), material=white, name="side_panel_0")
    cabinet.visual(Box((0.040, 0.720, 1.020)), origin=Origin(xyz=(0.380, 0.000, 0.540)), material=white, name="side_panel_1")
    cabinet.visual(Box((0.780, 0.720, 0.045)), origin=Origin(xyz=(0.000, 0.000, 1.047)), material=white, name="top_panel")
    cabinet.visual(Box((0.780, 0.720, 0.060)), origin=Origin(xyz=(0.000, 0.000, 0.030)), material=white, name="bottom_plinth")
    cabinet.visual(Box((0.780, 0.040, 1.020)), origin=Origin(xyz=(0.000, 0.340, 0.540)), material=white, name="rear_panel")
    cabinet.visual(Box((0.780, 0.035, 0.150)), origin=Origin(xyz=(0.000, -0.360, 0.955)), material=white, name="front_control_band")
    cabinet.visual(Box((0.780, 0.035, 0.205)), origin=Origin(xyz=(0.000, -0.360, 0.132)), material=white, name="front_kick_panel")
    cabinet.visual(Box((0.085, 0.035, 0.655)), origin=Origin(xyz=(-0.347, -0.360, 0.552)), material=white, name="front_stile_0")
    cabinet.visual(Box((0.085, 0.035, 0.655)), origin=Origin(xyz=(0.347, -0.360, 0.552)), material=white, name="front_stile_1")

    # Raised slide rails and a front-stop cassette on the top panel for the soap drawer.
    cabinet.visual(Box((0.018, 0.395, 0.026)), origin=Origin(xyz=(-0.315, -0.178, 1.076)), material=white, name="drawer_rail_0")
    cabinet.visual(Box((0.018, 0.395, 0.026)), origin=Origin(xyz=(-0.085, -0.178, 1.076)), material=white, name="drawer_rail_1")
    cabinet.visual(Box((0.248, 0.022, 0.034)), origin=Origin(xyz=(-0.200, 0.022, 1.080)), material=white, name="drawer_back_stop")

    cabinet.visual(Box((0.245, 0.010, 0.070)), origin=Origin(xyz=(0.168, -0.381, 0.958)), material=dark, name="display_window")
    cabinet.visual(Box((0.050, 0.019, 0.560)), origin=Origin(xyz=(-0.347, -0.3835, 0.550)), material=chrome, name="hinge_mount_plate")
    cabinet.visual(Box((0.120, 0.070, 0.120)), origin=Origin(xyz=(0.000, 0.295, 0.550)), material=steel, name="rear_bearing")

    gasket_mesh = mesh_from_geometry(TorusGeometry(0.295, 0.025, radial_segments=24, tubular_segments=64), "rubber_gasket")
    cabinet.visual(
        gasket_mesh,
        origin=Origin(xyz=(0.000, -0.345, 0.550), rpy=(math.pi / 2, 0.0, 0.0)),
        material=rubber,
        name="front_gasket",
    )

    drum = model.part("drum")
    drum_shell_mesh = mesh_from_geometry(CylinderGeometry(0.255, 0.440, radial_segments=72, closed=True), "drum_shell")
    drum_front_mesh = mesh_from_geometry(TorusGeometry(0.235, 0.018, radial_segments=18, tubular_segments=72), "drum_front_rim")
    drum.visual(drum_shell_mesh, origin=Origin(rpy=(math.pi / 2, 0.0, 0.0)), material=steel, name="perforated_basket_shell")
    drum.visual(drum_front_mesh, origin=Origin(xyz=(0.000, -0.215, 0.000), rpy=(math.pi / 2, 0.0, 0.0)), material=steel, name="drum_front_rim")
    drum.visual(Cylinder(0.225, 0.018), origin=Origin(xyz=(0.000, 0.222, 0.000), rpy=(math.pi / 2, 0.0, 0.0)), material=shadow, name="shadowed_back")
    drum.visual(Cylinder(0.034, 0.110), origin=Origin(xyz=(0.000, 0.285, 0.000), rpy=(math.pi / 2, 0.0, 0.0)), material=steel, name="axle_stub")
    model.articulation(
        "drum_axle",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=drum,
        origin=Origin(xyz=(0.000, -0.080, 0.550)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=20.0),
    )

    door = model.part("door")
    door_ring_mesh = mesh_from_geometry(TorusGeometry(0.285, 0.035, radial_segments=24, tubular_segments=72), "door_outer_ring")
    door.visual(door_ring_mesh, origin=Origin(xyz=(0.340, -0.010, 0.000), rpy=(math.pi / 2, 0.0, 0.0)), material=chrome, name="outer_ring")
    door.visual(Cylinder(0.252, 0.026), origin=Origin(xyz=(0.340, -0.023, 0.000), rpy=(math.pi / 2, 0.0, 0.0)), material=glass, name="glass_porthole")
    door.visual(Cylinder(0.018, 0.420), origin=Origin(xyz=(0.610, -0.082, 0.000)), material=chrome, name="bar_handle")
    door.visual(Box((0.105, 0.065, 0.034)), origin=Origin(xyz=(0.560, -0.052, 0.145)), material=chrome, name="handle_standoff_0")
    door.visual(Box((0.105, 0.065, 0.034)), origin=Origin(xyz=(0.560, -0.052, -0.145)), material=chrome, name="handle_standoff_1")
    door.visual(Cylinder(0.017, 0.520), origin=Origin(xyz=(0.000, -0.005, 0.000)), material=chrome, name="hinge_barrel")
    door.visual(Box((0.115, 0.040, 0.045)), origin=Origin(xyz=(0.058, -0.025, 0.175)), material=chrome, name="hinge_leaf_0")
    door.visual(Box((0.115, 0.040, 0.045)), origin=Origin(xyz=(0.058, -0.025, -0.175)), material=chrome, name="hinge_leaf_1")
    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=door,
        origin=Origin(xyz=(-0.340, -0.405, 0.550)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.4, lower=0.0, upper=1.85),
    )

    dispenser = model.part("dispenser_drawer")
    dispenser.visual(Box((0.205, 0.350, 0.040)), origin=Origin(xyz=(0.000, 0.000, 0.000)), material=white, name="drawer_tray")
    dispenser.visual(Box((0.205, 0.035, 0.085)), origin=Origin(xyz=(0.000, -0.190, 0.008)), material=white, name="drawer_face")
    dispenser.visual(Box((0.120, 0.012, 0.018)), origin=Origin(xyz=(0.000, -0.211, 0.016)), material=dark, name="finger_recess")
    model.articulation(
        "dispenser_slide",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=dispenser,
        origin=Origin(xyz=(-0.200, -0.190, 1.0895)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=0.35, lower=0.0, upper=0.230),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    drum = object_model.get_part("drum")
    door = object_model.get_part("door")
    dispenser = object_model.get_part("dispenser_drawer")
    door_hinge = object_model.get_articulation("door_hinge")
    dispenser_slide = object_model.get_articulation("dispenser_slide")

    ctx.expect_gap(
        cabinet,
        door,
        axis="y",
        max_gap=0.020,
        max_penetration=0.0,
        positive_elem="front_gasket",
        negative_elem="outer_ring",
        name="closed door sits just proud of the rubber gasket",
    )
    ctx.expect_gap(
        drum,
        cabinet,
        axis="y",
        max_gap=0.015,
        max_penetration=0.0,
        positive_elem="drum_front_rim",
        negative_elem="front_gasket",
        name="drum mouth sits behind the front gasket",
    )
    ctx.expect_gap(
        dispenser,
        cabinet,
        axis="z",
        max_gap=0.006,
        max_penetration=0.0,
        positive_elem="drawer_tray",
        negative_elem="top_panel",
        name="soap drawer rides on the top panel",
    )

    closed_door_aabb = ctx.part_world_aabb(door)
    with ctx.pose({door_hinge: 1.25}):
        open_door_aabb = ctx.part_world_aabb(door)
    ctx.check(
        "door hinge swings the porthole outward",
        closed_door_aabb is not None
        and open_door_aabb is not None
        and open_door_aabb[0][1] < closed_door_aabb[0][1] - 0.18,
        details=f"closed={closed_door_aabb}, open={open_door_aabb}",
    )

    closed_drawer_pos = ctx.part_world_position(dispenser)
    with ctx.pose({dispenser_slide: 0.230}):
        extended_drawer_pos = ctx.part_world_position(dispenser)
    ctx.check(
        "soap drawer slides forward from the top panel",
        closed_drawer_pos is not None
        and extended_drawer_pos is not None
        and extended_drawer_pos[1] < closed_drawer_pos[1] - 0.20,
        details=f"closed={closed_drawer_pos}, extended={extended_drawer_pos}",
    )

    return ctx.report()


object_model = build_object_model()
