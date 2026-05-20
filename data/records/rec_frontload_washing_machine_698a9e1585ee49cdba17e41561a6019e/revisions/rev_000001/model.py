from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BezelFace,
    BezelGeometry,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pedestal_mounted_washer")

    white = model.material("warm_white_enamel", rgba=(0.94, 0.95, 0.93, 1.0))
    soft_white = model.material("pedestal_white", rgba=(0.90, 0.91, 0.89, 1.0))
    dark = model.material("black_rubber", rgba=(0.015, 0.014, 0.013, 1.0))
    smoked_glass = model.material("smoked_glass", rgba=(0.05, 0.08, 0.10, 0.55))
    chrome = model.material("polished_chrome", rgba=(0.82, 0.84, 0.86, 1.0))
    rail_steel = model.material("zinc_plated_rail", rgba=(0.56, 0.58, 0.59, 1.0))
    panel_gray = model.material("dark_control_panel", rgba=(0.12, 0.13, 0.14, 1.0))
    display_blue = model.material("blue_display", rgba=(0.03, 0.16, 0.36, 1.0))

    cabinet = model.part("cabinet")

    # Pedestal drawer shell: a real open-front support frame rather than a
    # solid block, with the washer cabinet sitting on its top panel.
    cabinet.visual(Box((0.76, 0.72, 0.04)), origin=Origin(xyz=(0.0, 0.0, 0.02)), material=soft_white, name="pedestal_bottom")
    cabinet.visual(Box((0.76, 0.72, 0.04)), origin=Origin(xyz=(0.0, 0.0, 0.34)), material=soft_white, name="pedestal_top")
    cabinet.visual(Box((0.04, 0.72, 0.28)), origin=Origin(xyz=(-0.36, 0.0, 0.18)), material=soft_white, name="pedestal_side_0")
    cabinet.visual(Box((0.04, 0.72, 0.28)), origin=Origin(xyz=(0.36, 0.0, 0.18)), material=soft_white, name="pedestal_side_1")
    cabinet.visual(Box((0.68, 0.04, 0.28)), origin=Origin(xyz=(0.0, 0.34, 0.18)), material=soft_white, name="pedestal_back")
    cabinet.visual(Box((0.76, 0.035, 0.050)), origin=Origin(xyz=(0.0, -0.352, 0.333)), material=soft_white, name="pedestal_front_lip")
    cabinet.visual(Box((0.76, 0.035, 0.020)), origin=Origin(xyz=(0.0, -0.352, 0.045)), material=soft_white, name="pedestal_plinth")

    # Fixed outer members of the full-extension drawer slides, mounted to the
    # pedestal side panels inside the open storage bay.
    cabinet.visual(Box((0.035, 0.55, 0.026)), origin=Origin(xyz=(-0.322, 0.05, 0.205)), material=rail_steel, name="fixed_rail_0")
    cabinet.visual(Box((0.035, 0.55, 0.026)), origin=Origin(xyz=(0.322, 0.05, 0.205)), material=rail_steel, name="fixed_rail_1")

    # Main tall washer body and front panel details.
    cabinet.visual(Box((0.72, 0.72, 1.05)), origin=Origin(xyz=(0.0, 0.0, 0.885)), material=white, name="body_shell")
    cabinet.visual(Box((0.68, 0.018, 0.15)), origin=Origin(xyz=(0.0, -0.369, 1.315)), material=panel_gray, name="control_panel")
    cabinet.visual(Box((0.18, 0.004, 0.055)), origin=Origin(xyz=(-0.18, -0.380, 1.315)), material=display_blue, name="display_window")
    cabinet.visual(Box((0.18, 0.006, 0.035)), origin=Origin(xyz=(-0.26, -0.379, 1.258)), material=white, name="detergent_drawer_face")

    # A dark boot/gasket and visible drum shadow behind the glass porthole.
    cabinet.visual(
        Cylinder(radius=0.235, length=0.006),
        origin=Origin(xyz=(0.0, -0.357, 0.90), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark,
        name="porthole_gasket",
    )
    cabinet.visual(
        Cylinder(radius=0.190, length=0.004),
        origin=Origin(xyz=(0.0, -0.361, 0.90), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=panel_gray,
        name="drum_shadow",
    )

    # Fixed center hinge knuckle and cabinet leaf mounted on the left side of
    # the washer face. Door knuckles alternate above and below it.
    cabinet.visual(Box((0.070, 0.012, 0.135)), origin=Origin(xyz=(-0.338, -0.366, 0.90)), material=chrome, name="hinge_leaf")
    cabinet.visual(Cylinder(radius=0.006, length=0.505), origin=Origin(xyz=(-0.340, -0.390, 0.900)), material=chrome, name="hinge_pin")
    cabinet.visual(Cylinder(radius=0.018, length=0.110), origin=Origin(xyz=(-0.340, -0.390, 0.90)), material=chrome, name="fixed_hinge_barrel")

    door = model.part("door")
    door_ring = BezelGeometry(
        (0.410, 0.410),
        (0.580, 0.580),
        0.055,
        opening_shape="circle",
        outer_shape="circle",
        face=BezelFace(style="radiused_step", front_lip=0.006, fillet=0.004),
    )
    door.visual(
        mesh_from_geometry(door_ring, "chrome_porthole_bezel"),
        origin=Origin(xyz=(0.340, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="chrome_ring",
    )
    door.visual(
        Cylinder(radius=0.210, length=0.016),
        origin=Origin(xyz=(0.340, -0.004, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=smoked_glass,
        name="glass_porthole",
    )
    door.visual(Box((0.036, 0.040, 0.220)), origin=Origin(xyz=(0.615, -0.042, 0.0)), material=chrome, name="pull_handle")
    for z, name in ((0.175, "hinge_tab_0"), (-0.175, "hinge_tab_1")):
        door.visual(Box((0.130, 0.035, 0.042)), origin=Origin(xyz=(0.083, 0.0, z)), material=chrome, name=name)
    door.visual(Cylinder(radius=0.018, length=0.105), origin=Origin(xyz=(0.0, 0.0, 0.175)), material=chrome, name="door_hinge_barrel_0")
    door.visual(Cylinder(radius=0.018, length=0.105), origin=Origin(xyz=(0.0, 0.0, -0.175)), material=chrome, name="door_hinge_barrel_1")

    drawer = model.part("drawer")
    drawer.visual(Box((0.660, 0.035, 0.250)), origin=Origin(xyz=(0.0, -0.006, 0.0)), material=soft_white, name="drawer_front")
    drawer.visual(Box((0.580, 0.500, 0.030)), origin=Origin(xyz=(0.0, 0.250, -0.095)), material=soft_white, name="drawer_floor")
    drawer.visual(Box((0.026, 0.500, 0.155)), origin=Origin(xyz=(-0.290, 0.250, -0.030)), material=soft_white, name="drawer_side_0")
    drawer.visual(Box((0.026, 0.500, 0.155)), origin=Origin(xyz=(0.290, 0.250, -0.030)), material=soft_white, name="drawer_side_1")
    drawer.visual(Box((0.580, 0.026, 0.155)), origin=Origin(xyz=(0.0, 0.500, -0.030)), material=soft_white, name="drawer_back")
    drawer.visual(Box((0.480, 0.018, 0.030)), origin=Origin(xyz=(0.0, -0.028, 0.045)), material=dark, name="recessed_pull")
    drawer.visual(Box((0.026, 0.550, 0.024)), origin=Origin(xyz=(-0.285, 0.310, 0.025)), material=rail_steel, name="drawer_rail_0")
    drawer.visual(Box((0.026, 0.550, 0.024)), origin=Origin(xyz=(0.285, 0.310, 0.025)), material=rail_steel, name="drawer_rail_1")

    cycle_dial = model.part("cycle_dial")
    cycle_dial.visual(
        Cylinder(radius=0.046, length=0.025),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="dial_cap",
    )
    cycle_dial.visual(Box((0.007, 0.004, 0.035)), origin=Origin(xyz=(0.0, -0.0145, 0.028)), material=dark, name="dial_marker")

    model.articulation(
        "cabinet_to_door",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=door,
        origin=Origin(xyz=(-0.340, -0.390, 0.900)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.4, lower=0.0, upper=1.85),
    )
    model.articulation(
        "cabinet_to_drawer",
        ArticulationType.PRISMATIC,
        parent=cabinet,
        child=drawer,
        origin=Origin(xyz=(0.0, -0.375, 0.180)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.45, lower=0.0, upper=0.380),
    )
    model.articulation(
        "cabinet_to_dial",
        ArticulationType.CONTINUOUS,
        parent=cabinet,
        child=cycle_dial,
        origin=Origin(xyz=(0.170, -0.3905, 1.315)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=2.5),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    door = object_model.get_part("door")
    drawer = object_model.get_part("drawer")
    dial = object_model.get_part("cycle_dial")
    door_joint = object_model.get_articulation("cabinet_to_door")
    drawer_joint = object_model.get_articulation("cabinet_to_drawer")

    ctx.allow_overlap(
        cabinet,
        door,
        elem_a="hinge_pin",
        elem_b="door_hinge_barrel_0",
        reason="The chrome hinge pin is intentionally captured inside the upper moving door barrel.",
    )
    ctx.allow_overlap(
        cabinet,
        door,
        elem_a="hinge_pin",
        elem_b="door_hinge_barrel_1",
        reason="The chrome hinge pin is intentionally captured inside the lower moving door barrel.",
    )
    ctx.expect_within(
        cabinet,
        door,
        axes="xy",
        inner_elem="hinge_pin",
        outer_elem="door_hinge_barrel_0",
        margin=0.0,
        name="upper barrel wraps the hinge pin",
    )
    ctx.expect_overlap(
        cabinet,
        door,
        axes="z",
        elem_a="hinge_pin",
        elem_b="door_hinge_barrel_0",
        min_overlap=0.09,
        name="upper barrel engages hinge pin length",
    )
    ctx.expect_within(
        cabinet,
        door,
        axes="xy",
        inner_elem="hinge_pin",
        outer_elem="door_hinge_barrel_1",
        margin=0.0,
        name="lower barrel wraps the hinge pin",
    )
    ctx.expect_overlap(
        cabinet,
        door,
        axes="z",
        elem_a="hinge_pin",
        elem_b="door_hinge_barrel_1",
        min_overlap=0.09,
        name="lower barrel engages hinge pin length",
    )

    ctx.expect_gap(
        cabinet,
        door,
        axis="y",
        positive_elem="body_shell",
        negative_elem="chrome_ring",
        min_gap=0.001,
        max_gap=0.008,
        name="closed door sits just proud of washer face",
    )
    ctx.expect_overlap(
        door,
        cabinet,
        axes="xz",
        elem_a="chrome_ring",
        elem_b="porthole_gasket",
        min_overlap=0.40,
        name="chrome porthole ring centers over gasket",
    )
    ctx.expect_gap(
        cabinet,
        drawer,
        axis="y",
        positive_elem="pedestal_top",
        negative_elem="drawer_front",
        min_gap=0.001,
        max_gap=0.012,
        name="drawer front is flush with pedestal face",
    )
    ctx.expect_overlap(
        drawer,
        cabinet,
        axes="y",
        elem_a="drawer_rail_1",
        elem_b="fixed_rail_1",
        min_overlap=0.40,
        name="closed drawer rails are nested in the pedestal guides",
    )
    ctx.expect_gap(
        cabinet,
        dial,
        axis="y",
        positive_elem="control_panel",
        negative_elem="dial_cap",
        min_gap=0.0,
        max_gap=0.004,
        name="cycle dial is mounted on the control panel",
    )

    closed_door_aabb = ctx.part_element_world_aabb(door, elem="chrome_ring")
    with ctx.pose({door_joint: 1.2}):
        open_door_aabb = ctx.part_element_world_aabb(door, elem="chrome_ring")
    ctx.check(
        "door opens outward from the left barrel hinge",
        closed_door_aabb is not None
        and open_door_aabb is not None
        and open_door_aabb[0][1] < closed_door_aabb[0][1] - 0.12,
        details=f"closed={closed_door_aabb}, open={open_door_aabb}",
    )

    rest_drawer_pos = ctx.part_world_position(drawer)
    with ctx.pose({drawer_joint: 0.38}):
        extended_drawer_pos = ctx.part_world_position(drawer)
        ctx.expect_overlap(
            drawer,
            cabinet,
            axes="y",
            elem_a="drawer_rail_1",
            elem_b="fixed_rail_1",
            min_overlap=0.045,
            name="full-extension rail keeps retained insertion",
        )
    ctx.check(
        "pedestal drawer extends forward",
        rest_drawer_pos is not None
        and extended_drawer_pos is not None
        and extended_drawer_pos[1] < rest_drawer_pos[1] - 0.34,
        details=f"rest={rest_drawer_pos}, extended={extended_drawer_pos}",
    )

    return ctx.report()


object_model = build_object_model()
