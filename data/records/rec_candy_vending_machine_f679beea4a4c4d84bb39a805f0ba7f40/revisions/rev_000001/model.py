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
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="triple_dial_candy_vendor")

    model.material("red_enamel", rgba=(0.78, 0.05, 0.035, 1.0))
    model.material("dark_shadow", rgba=(0.035, 0.032, 0.030, 1.0))
    model.material("brushed_metal", rgba=(0.70, 0.68, 0.60, 1.0))
    model.material("polished_chrome", rgba=(0.90, 0.88, 0.80, 1.0))
    model.material("clear_acrylic", rgba=(0.60, 0.88, 1.0, 0.34))
    model.material("amber_candy", rgba=(1.0, 0.58, 0.10, 0.88))
    model.material("green_candy", rgba=(0.20, 0.75, 0.22, 0.88))
    model.material("pink_candy", rgba=(0.95, 0.20, 0.45, 0.88))
    model.material("black_rubber", rgba=(0.015, 0.014, 0.013, 1.0))

    body = model.part("body")

    # Tall red cabinet and connected front frame.
    body.visual(Box((0.60, 0.40, 0.12)), origin=Origin(xyz=(0.0, 0.0, 0.06)), material="red_enamel", name="base_plinth")
    body.visual(Box((0.58, 0.38, 0.08)), origin=Origin(xyz=(0.0, 0.0, 1.32)), material="red_enamel", name="top_cap")
    body.visual(Box((0.55, 0.026, 1.18)), origin=Origin(xyz=(0.0, 0.157, 0.70)), material="red_enamel", name="rear_panel")
    for side, x in enumerate((-0.257, 0.257)):
        body.visual(Box((0.036, 0.34, 1.18)), origin=Origin(xyz=(x, 0.0, 0.70)), material="red_enamel", name=f"side_wall_{side}")
        body.visual(Box((0.040, 0.034, 1.17)), origin=Origin(xyz=(x, -0.177, 0.705)), material="red_enamel", name=f"front_post_{side}")

    for z, rail_name in (
        (0.50, "lower_bin_rail"),
        (0.82, "bin_rail_0"),
        (1.06, "bin_rail_1"),
        (1.285, "upper_bin_rail"),
    ):
        body.visual(Box((0.55, 0.034, 0.036)), origin=Origin(xyz=(0.0, -0.177, z)), material="red_enamel", name=rail_name)

    # Lower metal control panel around the selectors and pickup opening.
    body.visual(Box((0.47, 0.030, 0.34)), origin=Origin(xyz=(0.0, -0.176, 0.315)), material="brushed_metal", name="control_panel")
    body.visual(Box((0.44, 0.020, 0.030)), origin=Origin(xyz=(0.0, -0.198, 0.385)), material="polished_chrome", name="door_lintel")
    for side, x in enumerate((-0.225, 0.225)):
        body.visual(Box((0.030, 0.020, 0.245)), origin=Origin(xyz=(x, -0.198, 0.255)), material="polished_chrome", name=f"door_jamb_{side}")

    # Three transparent bins with colored candy piles; all panes overlap the red
    # frame or shelves so the cabinet reads as one manufactured assembly.
    bin_centers = (0.695, 0.935, 1.175)
    candy_mats = ("amber_candy", "green_candy", "pink_candy")
    for i, (z, candy_mat) in enumerate(zip(bin_centers, candy_mats)):
        body.visual(Box((0.430, 0.006, 0.185)), origin=Origin(xyz=(0.0, -0.198, z)), material="clear_acrylic", name=f"bin_front_{i}")
        body.visual(Box((0.006, 0.285, 0.185)), origin=Origin(xyz=(-0.218, -0.054, z)), material="clear_acrylic", name=f"bin_side_{i}_0")
        body.visual(Box((0.006, 0.285, 0.185)), origin=Origin(xyz=(0.218, -0.054, z)), material="clear_acrylic", name=f"bin_side_{i}_1")
        body.visual(Box((0.430, 0.300, 0.008)), origin=Origin(xyz=(0.0, -0.052, z - 0.096)), material="clear_acrylic", name=f"bin_shelf_{i}")
        body.visual(Box((0.360, 0.210, 0.060)), origin=Origin(xyz=(0.0, -0.045, z - 0.063)), material=candy_mat, name=f"candy_fill_{i}")
        for j, x in enumerate((-0.140, -0.070, 0.000, 0.075, 0.150)):
            y = -0.125 + 0.040 * (j % 2)
            body.visual(Sphere(0.020), origin=Origin(xyz=(x, y, z - 0.020)), material=candy_mat, name=f"candy_{i}_{j}")

    # Fixed selector bushings on the front panel, one per bin.
    selector_z = (0.550, 0.790, 1.030)
    for i, z in enumerate(selector_z):
        body.visual(
            Cylinder(radius=0.055, length=0.018),
            origin=Origin(xyz=(0.0, -0.203, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material="polished_chrome",
            name=f"selector_bushing_{i}",
        )
        body.visual(
            Cylinder(radius=0.025, length=0.006),
            origin=Origin(xyz=(0.0, -0.209, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material="dark_shadow",
            name=f"selector_socket_{i}",
        )

    # Alternating hinge knuckles and brackets carried by the cabinet.
    for side, x in enumerate((-0.152, 0.152)):
        body.visual(Box((0.100, 0.030, 0.042)), origin=Origin(xyz=(x, -0.203, 0.155)), material="polished_chrome", name=f"hinge_bracket_{side}")
        body.visual(
            Cylinder(radius=0.014, length=0.104),
            origin=Origin(xyz=(x, -0.215, 0.155), rpy=(0.0, math.pi / 2.0, 0.0)),
            material="polished_chrome",
            name=f"body_hinge_knuckle_{side}",
        )

    # Three continuously-rotating selector dials.  Each child frame sits on the
    # front face of its own bushing; the shaft face touches the bushing face.
    for i, z in enumerate(selector_z):
        selector = model.part(f"selector_{i}")
        selector.visual(
            Cylinder(radius=0.025, length=0.026),
            origin=Origin(xyz=(0.0, -0.013, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material="polished_chrome",
            name="shaft",
        )
        selector.visual(
            Cylinder(radius=0.077, length=0.036),
            origin=Origin(xyz=(0.0, -0.044, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material="brushed_metal",
            name="dial_face",
        )
        selector.visual(
            Cylinder(radius=0.036, length=0.016),
            origin=Origin(xyz=(0.0, -0.070, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material="polished_chrome",
            name="center_hub",
        )
        selector.visual(Box((0.026, 0.018, 0.112)), origin=Origin(xyz=(0.0, -0.078, 0.0)), material="black_rubber", name="grip")
        selector.visual(Box((0.010, 0.012, 0.030)), origin=Origin(xyz=(0.052, -0.066, 0.0)), material="red_enamel", name="pointer_tip")
        model.articulation(
            f"selector_{i}_spin",
            ArticulationType.CONTINUOUS,
            parent=body,
            child=selector,
            origin=Origin(xyz=(0.0, -0.212, z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=2.0, velocity=6.0),
        )

    # One lower retrieval flap.  Its local frame is the bottom hinge axis, so a
    # positive rotation about +X drops the top edge outward and downward.
    door = model.part("retrieval_door")
    door.visual(
        Cylinder(radius=0.012, length=0.180),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="polished_chrome",
        name="door_hinge_knuckle",
    )
    door.visual(Box((0.180, 0.012, 0.032)), origin=Origin(xyz=(0.0, -0.014, 0.018)), material="polished_chrome", name="hinge_leaf")
    door.visual(Box((0.380, 0.018, 0.190)), origin=Origin(xyz=(0.0, -0.022, 0.100)), material="brushed_metal", name="door_panel")
    door.visual(Box((0.160, 0.018, 0.028)), origin=Origin(xyz=(0.0, -0.039, 0.150)), material="black_rubber", name="pull_lip")
    door.visual(Box((0.220, 0.006, 0.052)), origin=Origin(xyz=(0.0, -0.033, 0.070)), material="clear_acrylic", name="pickup_window")
    model.articulation(
        "door_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(0.0, -0.215, 0.155)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.0, lower=0.0, upper=1.15),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    door = object_model.get_part("retrieval_door")
    door_hinge = object_model.get_articulation("door_hinge")
    selectors = [object_model.get_part(f"selector_{i}") for i in range(3)]
    selector_joints = [object_model.get_articulation(f"selector_{i}_spin") for i in range(3)]

    ctx.check(
        "three independent continuous selector dials",
        len(selector_joints) == 3
        and all(j.articulation_type == ArticulationType.CONTINUOUS for j in selector_joints)
        and len({j.name for j in selector_joints}) == 3,
        details=f"selector_joints={[getattr(j, 'name', None) for j in selector_joints]}",
    )
    ctx.check(
        "selector shafts are parallel and horizontal",
        all(tuple(round(v, 3) for v in j.axis) == (0.0, 1.0, 0.0) for j in selector_joints),
        details=f"axes={[getattr(j, 'axis', None) for j in selector_joints]}",
    )

    for i, selector in enumerate(selectors):
        ctx.expect_gap(
            body,
            selector,
            axis="y",
            positive_elem=f"selector_bushing_{i}",
            negative_elem="shaft",
            max_gap=0.0015,
            max_penetration=0.00001,
            name=f"selector {i} shaft seats on bushing",
        )

    # A rotated selector should visibly move its grip around the horizontal
    # shaft; at a quarter-turn the formerly vertical grip becomes wide in X.
    rest_grip = ctx.part_element_world_aabb("selector_1", elem="grip")
    with ctx.pose({"selector_1_spin": math.pi / 2.0}):
        turned_grip = ctx.part_element_world_aabb("selector_1", elem="grip")
    ctx.check(
        "selector grip visibly rotates",
        rest_grip is not None
        and turned_grip is not None
        and (turned_grip[1][0] - turned_grip[0][0]) > (rest_grip[1][0] - rest_grip[0][0]) + 0.050,
        details=f"rest={rest_grip}, turned={turned_grip}",
    )

    ctx.check(
        "retrieval door has bottom horizontal hinge limits",
        door_hinge.articulation_type == ArticulationType.REVOLUTE
        and tuple(round(v, 3) for v in door_hinge.axis) == (1.0, 0.0, 0.0)
        and door_hinge.motion_limits is not None
        and door_hinge.motion_limits.lower == 0.0
        and door_hinge.motion_limits.upper >= 1.0,
        details=f"axis={door_hinge.axis}, limits={door_hinge.motion_limits}",
    )

    closed_door = ctx.part_world_aabb(door)
    with ctx.pose({door_hinge: 1.05}):
        open_door = ctx.part_world_aabb(door)
    ctx.check(
        "retrieval door swings downward and outward",
        closed_door is not None
        and open_door is not None
        and open_door[1][2] < closed_door[1][2] - 0.055
        and open_door[0][1] < closed_door[0][1] - 0.060,
        details=f"closed={closed_door}, open={open_door}",
    )

    return ctx.report()


object_model = build_object_model()
