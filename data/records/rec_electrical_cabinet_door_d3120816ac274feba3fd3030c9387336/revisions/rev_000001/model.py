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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="weatherproof_enclosure")

    powder = model.material("powder_coated_steel", rgba=(0.36, 0.39, 0.38, 1.0))
    dark = model.material("dark_recess", rgba=(0.055, 0.060, 0.058, 1.0))
    hinge_metal = model.material("galvanized_hinge", rgba=(0.55, 0.57, 0.55, 1.0))
    gasket = model.material("black_gasket", rgba=(0.01, 0.012, 0.010, 1.0))
    stainless = model.material("stainless_lockwork", rgba=(0.74, 0.74, 0.70, 1.0))
    warning = model.material("yellow_lock_label", rgba=(0.95, 0.72, 0.12, 1.0))

    body = model.part("body")

    # Deep welded back box: 780 mm wide, 300 mm deep, and 1.2 m tall.
    body.visual(Box((0.780, 0.025, 1.200)), origin=Origin(xyz=(0.0, -0.1475, 0.600)), material=powder, name="back_panel")
    body.visual(Box((0.025, 0.290, 1.200)), origin=Origin(xyz=(-0.3775, -0.015, 0.600)), material=powder, name="side_wall_0")
    body.visual(Box((0.025, 0.290, 1.200)), origin=Origin(xyz=(0.3775, -0.015, 0.600)), material=powder, name="side_wall_1")
    body.visual(Box((0.780, 0.290, 0.025)), origin=Origin(xyz=(0.0, -0.015, 0.0125)), material=powder, name="bottom_wall")
    body.visual(Box((0.780, 0.290, 0.025)), origin=Origin(xyz=(0.0, -0.015, 1.1875)), material=powder, name="top_wall")

    # Front return frame around the opening, leaving the enclosure visibly hollow.
    body.visual(Box((0.055, 0.035, 1.110)), origin=Origin(xyz=(-0.3625, 0.120, 0.600)), material=powder, name="front_jamb_0")
    body.visual(Box((0.055, 0.035, 1.110)), origin=Origin(xyz=(0.3625, 0.120, 0.600)), material=powder, name="front_jamb_1")
    body.visual(Box((0.780, 0.035, 0.055)), origin=Origin(xyz=(0.0, 0.120, 0.0275)), material=powder, name="front_sill")
    body.visual(Box((0.780, 0.035, 0.055)), origin=Origin(xyz=(0.0, 0.120, 1.1725)), material=powder, name="front_header")
    body.visual(Box((0.645, 0.012, 1.020)), origin=Origin(xyz=(0.0, -0.129, 0.600)), material=dark, name="open_cavity")

    # Raised compressible gasket land on the body frame.
    body.visual(Box((0.030, 0.010, 1.020)), origin=Origin(xyz=(-0.3575, 0.142, 0.600)), material=gasket, name="gasket_rail_0")
    body.visual(Box((0.030, 0.010, 1.020)), origin=Origin(xyz=(0.3575, 0.142, 0.600)), material=gasket, name="gasket_rail_1")
    body.visual(Box((0.645, 0.010, 0.030)), origin=Origin(xyz=(0.0, 0.142, 0.070)), material=gasket, name="gasket_rail_2")
    body.visual(Box((0.645, 0.010, 0.030)), origin=Origin(xyz=(0.0, 0.142, 1.130)), material=gasket, name="gasket_rail_3")

    # Three strike plates on the latch-side jamb.  They are slotted frames, not
    # solid blocks, so the vertical rod lugs can sit in them without collision.
    for i, zc in ((0, 0.190), (2, 1.010)):
        body.visual(Box((0.080, 0.010, 0.105)), origin=Origin(xyz=(0.288, 0.110, zc + 0.025)), material=hinge_metal, name=f"strike_back_{i}")
        body.visual(Box((0.060, 0.020, 0.012)), origin=Origin(xyz=(0.365, 0.125, zc - 0.020)), material=hinge_metal, name=f"strike_lower_{i}")
        body.visual(Box((0.060, 0.020, 0.012)), origin=Origin(xyz=(0.365, 0.125, zc + 0.072)), material=hinge_metal, name=f"strike_upper_{i}")
        body.visual(Box((0.012, 0.020, 0.092)), origin=Origin(xyz=(0.334, 0.125, zc + 0.026)), material=hinge_metal, name=f"strike_side_{i}")
    body.visual(Box((0.080, 0.010, 0.105)), origin=Origin(xyz=(0.288, 0.110, 0.625)), material=hinge_metal, name="strike_back_1")
    body.visual(Box((0.060, 0.020, 0.012)), origin=Origin(xyz=(0.365, 0.125, 0.580)), material=hinge_metal, name="strike_lower_1")
    body.visual(Box((0.060, 0.020, 0.012)), origin=Origin(xyz=(0.365, 0.125, 0.672)), material=hinge_metal, name="strike_upper_1")
    body.visual(Box((0.012, 0.020, 0.092)), origin=Origin(xyz=(0.334, 0.125, 0.626)), material=hinge_metal, name="strike_side_1")

    # Two heavy external hinge mounts fixed to the body.
    hinge_centers = (0.315, 0.895)
    for i, zc in enumerate(hinge_centers):
        body.visual(Box((0.115, 0.030, 0.245)), origin=Origin(xyz=(-0.4475, 0.132, zc)), material=hinge_metal, name=f"hinge_stand_{i}")
        body.visual(Box((0.105, 0.040, 0.230)), origin=Origin(xyz=(-0.470, 0.160, zc)), material=hinge_metal, name=f"hinge_web_{i}")
        body.visual(Box((0.075, 0.012, 0.225)), origin=Origin(xyz=(-0.4925, 0.180, zc)), material=hinge_metal, name=f"hinge_leaf_{i}")
        # Fixed leaf knuckles: upper and lower barrels on the body side.
        body.visual(Cylinder(radius=0.018, length=0.070), origin=Origin(xyz=(-0.435, 0.205, zc - 0.075)), material=hinge_metal, name=f"hinge_barrel_low_{i}")
        body.visual(Cylinder(radius=0.018, length=0.070), origin=Origin(xyz=(-0.435, 0.205, zc + 0.075)), material=hinge_metal, name=f"hinge_barrel_high_{i}")
        body.visual(Box((0.040, 0.014, 0.064)), origin=Origin(xyz=(-0.453, 0.190, zc - 0.075)), material=hinge_metal, name=f"hinge_tab_low_{i}")
        body.visual(Box((0.040, 0.014, 0.064)), origin=Origin(xyz=(-0.453, 0.190, zc + 0.075)), material=hinge_metal, name=f"hinge_tab_high_{i}")

    door = model.part("door")
    # The door frame is the hinge pin line at x=0, z vertical; the panel starts
    # just right of the hinge barrels and seals over the body front frame.
    door.visual(Box((0.800, 0.050, 1.185)), origin=Origin(xyz=(0.435, -0.035, 0.600)), material=powder, name="door_panel")
    door.visual(Box((0.770, 0.012, 0.030)), origin=Origin(xyz=(0.435, -0.066, 1.2075)), material=powder, name="rain_lip")
    door.visual(Box((0.030, 0.012, 1.075)), origin=Origin(xyz=(0.070, -0.066, 0.600)), material=powder, name="door_flange_0")
    door.visual(Box((0.030, 0.012, 1.075)), origin=Origin(xyz=(0.800, -0.066, 0.600)), material=powder, name="door_flange_1")
    door.visual(Box((0.730, 0.012, 0.030)), origin=Origin(xyz=(0.435, -0.066, 0.0725)), material=powder, name="door_flange_2")
    door.visual(Box((0.730, 0.012, 0.030)), origin=Origin(xyz=(0.435, -0.066, 1.1275)), material=powder, name="door_flange_3")
    door.visual(Box((0.130, 0.006, 0.060)), origin=Origin(xyz=(0.650, -0.007, 0.610)), material=warning, name="lock_label")
    door.visual(Cylinder(radius=0.030, length=0.010), origin=Origin(xyz=(0.650, -0.005, 0.600), rpy=(math.pi / 2, 0.0, 0.0)), material=stainless, name="lock_escutcheon")

    # Moving hinge leaves and middle knuckles attached to the door.
    for i, zc in enumerate(hinge_centers):
        door.visual(Box((0.110, 0.012, 0.225)), origin=Origin(xyz=(0.075, -0.004, zc)), material=hinge_metal, name=f"door_leaf_{i}")
        door.visual(Box((0.038, 0.014, 0.064)), origin=Origin(xyz=(0.017, -0.004, zc)), material=hinge_metal, name=f"door_tab_{i}")
        door.visual(Cylinder(radius=0.018, length=0.070), origin=Origin(xyz=(0.0, 0.0, zc)), material=hinge_metal, name=f"door_barrel_{i}")

    # Interior rod guide brackets.  Pads touch the inner skin; ears form
    # clearanced saddles that visually capture the sliding rod.
    for i, zc in enumerate((0.300, 0.710, 0.930)):
        door.visual(Box((0.085, 0.012, 0.058)), origin=Origin(xyz=(0.735, -0.066, zc)), material=hinge_metal, name=f"guide_pad_{i}")
        door.visual(Box((0.014, 0.030, 0.058)), origin=Origin(xyz=(0.712, -0.080, zc)), material=hinge_metal, name=f"guide_ear_{i}_0")
        door.visual(Box((0.014, 0.030, 0.058)), origin=Origin(xyz=(0.758, -0.080, zc)), material=hinge_metal, name=f"guide_ear_{i}_1")

    rod = model.part("lock_rod")
    rod.visual(Cylinder(radius=0.009, length=1.085), origin=Origin(xyz=(0.0, 0.0, 0.600)), material=stainless, name="rod_bar")
    for i, zc in ((0, 0.190), (2, 1.010)):
        rod.visual(Box((0.060, 0.018, 0.014)), origin=Origin(xyz=(0.0, 0.0, zc)), material=stainless, name=f"rod_lug_{i}")
    rod.visual(Box((0.060, 0.018, 0.014)), origin=Origin(xyz=(0.0, 0.0, 0.600)), material=stainless, name="rod_lug_1")
    rod.visual(Box((0.040, 0.020, 0.090)), origin=Origin(xyz=(-0.018, 0.0, 0.520)), material=stainless, name="actuator_clevis")

    door_hinge = model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(-0.435, 0.205, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=180.0, velocity=1.0, lower=0.0, upper=1.65),
    )
    model.articulation(
        "door_to_rod",
        ArticulationType.PRISMATIC,
        parent=door,
        child=rod,
        origin=Origin(xyz=(0.735, -0.080, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.15, lower=0.0, upper=0.045),
    )
    door_hinge.meta["description"] = "single-leaf front door swinging outward on the two visible left-edge hinges"

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    door = object_model.get_part("door")
    rod = object_model.get_part("lock_rod")
    door_hinge = object_model.get_articulation("body_to_door")
    rod_slide = object_model.get_articulation("door_to_rod")

    ctx.allow_overlap(
        body,
        door,
        elem_a="gasket_rail_2",
        elem_b="door_flange_2",
        reason="The bottom return flange intentionally compresses the weather seal when the enclosure is latched.",
    )
    ctx.allow_overlap(
        body,
        door,
        elem_a="gasket_rail_3",
        elem_b="door_flange_3",
        reason="The top return flange intentionally compresses the weather seal when the enclosure is latched.",
    )
    ctx.allow_overlap(
        body,
        door,
        elem_a="gasket_rail_0",
        elem_b="door_flange_0",
        reason="The hinge-side return flange intentionally compresses the weather seal when the enclosure is latched.",
    )
    ctx.allow_overlap(
        body,
        door,
        elem_a="gasket_rail_1",
        elem_b="door_flange_1",
        reason="The latch-side return flange intentionally compresses the weather seal when the enclosure is latched.",
    )

    with ctx.pose({door_hinge: 0.0, rod_slide: 0.0}):
        ctx.expect_gap(
            door,
            body,
            axis="y",
            positive_elem="door_flange_2",
            negative_elem="gasket_rail_2",
            max_penetration=0.015,
            max_gap=0.001,
            name="bottom flange compresses gasket locally",
        )
        ctx.expect_overlap(
            door,
            body,
            axes="x",
            elem_a="door_flange_2",
            elem_b="gasket_rail_2",
            min_overlap=0.50,
            name="bottom flange spans gasket seal",
        )
        ctx.expect_gap(
            door,
            body,
            axis="y",
            positive_elem="door_flange_3",
            negative_elem="gasket_rail_3",
            max_penetration=0.015,
            max_gap=0.001,
            name="top flange compresses gasket locally",
        )
        ctx.expect_overlap(
            door,
            body,
            axes="x",
            elem_a="door_flange_3",
            elem_b="gasket_rail_3",
            min_overlap=0.50,
            name="top flange spans gasket seal",
        )
        ctx.expect_gap(
            door,
            body,
            axis="y",
            positive_elem="door_flange_0",
            negative_elem="gasket_rail_0",
            max_penetration=0.015,
            max_gap=0.001,
            name="hinge-side flange compresses gasket locally",
        )
        ctx.expect_overlap(
            door,
            body,
            axes="z",
            elem_a="door_flange_0",
            elem_b="gasket_rail_0",
            min_overlap=0.80,
            name="hinge-side flange spans gasket seal",
        )
        ctx.expect_gap(
            door,
            body,
            axis="y",
            positive_elem="door_flange_1",
            negative_elem="gasket_rail_1",
            max_penetration=0.015,
            max_gap=0.001,
            name="latch-side flange compresses gasket locally",
        )
        ctx.expect_overlap(
            door,
            body,
            axes="z",
            elem_a="door_flange_1",
            elem_b="gasket_rail_1",
            min_overlap=0.80,
            name="latch-side flange spans gasket seal",
        )
        ctx.expect_gap(
            door,
            body,
            axis="y",
            positive_elem="door_panel",
            negative_elem="front_jamb_1",
            min_gap=0.003,
            max_gap=0.020,
            name="closed door stands proud of frame",
        )
        ctx.expect_overlap(
            door,
            body,
            axes="xz",
            elem_a="door_panel",
            elem_b="front_jamb_1",
            min_overlap=0.05,
            name="door covers latch jamb",
        )
        ctx.expect_gap(
            rod,
            body,
            axis="y",
            positive_elem="rod_lug_1",
            negative_elem="strike_back_1",
            min_gap=0.0,
            max_gap=0.004,
            name="middle lug bears on strike plate",
        )
        ctx.expect_overlap(
            rod,
            body,
            axes="xz",
            elem_a="rod_lug_1",
            elem_b="strike_back_1",
            min_overlap=0.010,
            name="middle lug sits in strike slot",
        )

    closed_aabb = ctx.part_element_world_aabb(door, elem="door_panel")
    locked_aabb = ctx.part_element_world_aabb(rod, elem="rod_lug_1")
    with ctx.pose({door_hinge: 1.20, rod_slide: 0.045}):
        opened_aabb = ctx.part_element_world_aabb(door, elem="door_panel")
        raised_aabb = ctx.part_element_world_aabb(rod, elem="rod_lug_1")

    ctx.check(
        "door opens outward",
        closed_aabb is not None
        and opened_aabb is not None
        and opened_aabb[1][1] > closed_aabb[1][1] + 0.18,
        details=f"closed={closed_aabb}, opened={opened_aabb}",
    )
    ctx.check(
        "lock rod slides upward",
        locked_aabb is not None
        and raised_aabb is not None
        and raised_aabb[0][2] > locked_aabb[0][2] + 0.035,
        details=f"locked={locked_aabb}, raised={raised_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
