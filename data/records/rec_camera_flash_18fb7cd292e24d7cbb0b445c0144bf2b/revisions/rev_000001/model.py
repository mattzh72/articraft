from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BezelGeometry,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TrunnionYokeGeometry,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_speedlight_flash")

    plastic = model.material("satin_black_plastic", rgba=(0.035, 0.037, 0.040, 1.0))
    rubber = model.material("matte_rubber_black", rgba=(0.010, 0.011, 0.012, 1.0))
    dark_trim = model.material("dark_seam_trim", rgba=(0.0, 0.0, 0.0, 1.0))
    metal = model.material("brushed_hotshoe_metal", rgba=(0.62, 0.62, 0.58, 1.0))
    lens = model.material("milky_flash_lens", rgba=(0.86, 0.90, 0.92, 0.55))
    warm_reflector = model.material("warm_reflector_glow", rgba=(1.0, 0.78, 0.36, 0.65))

    body = model.part("body")
    body.visual(
        Box((0.050, 0.040, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=metal,
        name="shoe_plate",
    )
    body.visual(
        Box((0.036, 0.028, 0.009)),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=metal,
        name="shoe_foot",
    )
    body.visual(
        Box((0.007, 0.040, 0.005)),
        origin=Origin(xyz=(-0.025, 0.0, 0.007)),
        material=metal,
        name="shoe_rail_0",
    )
    body.visual(
        Box((0.007, 0.040, 0.005)),
        origin=Origin(xyz=(0.025, 0.0, 0.007)),
        material=metal,
        name="shoe_rail_1",
    )
    body.visual(
        Box((0.064, 0.050, 0.076)),
        origin=Origin(xyz=(0.0, 0.0, 0.052)),
        material=plastic,
        name="battery_body",
    )
    body.visual(
        Box((0.056, 0.043, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.099)),
        material=plastic,
        name="upper_step",
    )
    body.visual(
        Cylinder(radius=0.023, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.117)),
        material=plastic,
        name="swivel_socket",
    )
    body.visual(
        Box((0.054, 0.0016, 0.027)),
        origin=Origin(xyz=(0.0, -0.0258, 0.078)),
        material=rubber,
        name="rear_grip_inset",
    )
    body.visual(
        Box((0.028, 0.0014, 0.012)),
        origin=Origin(xyz=(0.0, 0.0257, 0.056)),
        material=dark_trim,
        name="front_sensor_window",
    )

    battery_door = model.part("battery_door")
    battery_door.visual(
        Box((0.0040, 0.038, 0.058)),
        origin=Origin(xyz=(0.0020, 0.020, 0.0)),
        material=plastic,
        name="door_panel",
    )
    battery_door.visual(
        Cylinder(radius=0.0030, length=0.064),
        origin=Origin(xyz=(0.0030, 0.0, 0.0)),
        material=dark_trim,
        name="door_hinge_barrel",
    )
    battery_door.visual(
        Box((0.0008, 0.041, 0.0020)),
        origin=Origin(xyz=(0.0044, 0.020, 0.030)),
        material=dark_trim,
        name="door_top_seam",
    )
    battery_door.visual(
        Box((0.0008, 0.041, 0.0020)),
        origin=Origin(xyz=(0.0044, 0.020, -0.030)),
        material=dark_trim,
        name="door_bottom_seam",
    )
    battery_door.visual(
        Box((0.0008, 0.0020, 0.058)),
        origin=Origin(xyz=(0.0044, 0.040, 0.0)),
        material=dark_trim,
        name="door_front_seam",
    )
    battery_door.visual(
        Box((0.0009, 0.020, 0.0022)),
        origin=Origin(xyz=(0.00445, 0.026, 0.010)),
        material=rubber,
        name="door_grip_rib_0",
    )
    battery_door.visual(
        Box((0.0009, 0.020, 0.0022)),
        origin=Origin(xyz=(0.00445, 0.026, -0.002)),
        material=rubber,
        name="door_grip_rib_1",
    )

    swivel_neck = model.part("swivel_neck")
    swivel_neck.visual(
        Cylinder(radius=0.021, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=plastic,
        name="lower_swivel_collar",
    )
    swivel_neck.visual(
        Cylinder(radius=0.011, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.022)),
        material=plastic,
        name="vertical_neck_post",
    )
    swivel_neck.visual(
        mesh_from_geometry(
            TrunnionYokeGeometry(
                (0.104, 0.028, 0.052),
                span_width=0.084,
                trunnion_diameter=0.010,
                trunnion_center_z=0.035,
                base_thickness=0.010,
                corner_radius=0.002,
                center=False,
            ),
            "speedlight_trunnion_yoke",
        ),
        origin=Origin(xyz=(0.0, 0.004, 0.022)),
        material=plastic,
        name="tilt_yoke",
    )
    swivel_neck.visual(
        Box((0.090, 0.012, 0.004)),
        origin=Origin(xyz=(0.0, 0.010, 0.051)),
        material=rubber,
        name="head_saddle_pad",
    )

    head = model.part("lamp_head")
    head.visual(
        Box((0.074, 0.052, 0.040)),
        origin=Origin(xyz=(0.0, 0.029, 0.016)),
        material=plastic,
        name="head_shell",
    )
    head.visual(
        Cylinder(radius=0.006, length=0.004),
        origin=Origin(xyz=(-0.039, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=plastic,
        name="pivot_cap_0",
    )
    head.visual(
        Cylinder(radius=0.006, length=0.004),
        origin=Origin(xyz=(0.039, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=plastic,
        name="pivot_cap_1",
    )
    head.visual(
        mesh_from_geometry(
            BezelGeometry(
                (0.054, 0.021),
                (0.068, 0.033),
                0.004,
                opening_shape="rounded_rect",
                outer_shape="rounded_rect",
                opening_corner_radius=0.0025,
                outer_corner_radius=0.004,
            ),
            "speedlight_lamp_bezel",
        ),
        origin=Origin(xyz=(0.0, 0.057, 0.017), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_trim,
        name="lamp_bezel",
    )
    head.visual(
        Box((0.052, 0.002, 0.020)),
        origin=Origin(xyz=(0.0, 0.060, 0.017)),
        material=lens,
        name="diffuser_lens",
    )
    head.visual(
        Box((0.038, 0.0012, 0.011)),
        origin=Origin(xyz=(0.0, 0.0616, 0.017)),
        material=warm_reflector,
        name="flash_tube_glow",
    )

    model.articulation(
        "body_to_battery_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=battery_door,
        origin=Origin(xyz=(0.032, -0.022, 0.063)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=0.6, velocity=2.0, lower=0.0, upper=1.75),
    )
    model.articulation(
        "body_to_swivel_neck",
        ArticulationType.REVOLUTE,
        parent=body,
        child=swivel_neck,
        origin=Origin(xyz=(0.0, 0.0, 0.120)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.5, velocity=2.5, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "swivel_neck_to_lamp_head",
        ArticulationType.REVOLUTE,
        parent=swivel_neck,
        child=head,
        origin=Origin(xyz=(0.0, 0.004, 0.057)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.2, velocity=2.0, lower=-0.25, upper=1.55),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    door = object_model.get_part("battery_door")
    neck = object_model.get_part("swivel_neck")
    head = object_model.get_part("lamp_head")
    door_joint = object_model.get_articulation("body_to_battery_door")
    swivel_joint = object_model.get_articulation("body_to_swivel_neck")
    tilt_joint = object_model.get_articulation("swivel_neck_to_lamp_head")

    ctx.check(
        "lower joint uses vertical swivel axis",
        tuple(swivel_joint.axis) == (0.0, 0.0, 1.0),
        details=f"axis={swivel_joint.axis}",
    )
    ctx.check(
        "upper joint uses horizontal tilt axis",
        tuple(tilt_joint.axis) == (1.0, 0.0, 0.0),
        details=f"axis={tilt_joint.axis}",
    )
    ctx.check(
        "battery door hinge swings outward from side",
        tuple(door_joint.axis) == (0.0, 0.0, -1.0),
        details=f"axis={door_joint.axis}",
    )
    ctx.expect_contact(
        body,
        neck,
        elem_a="swivel_socket",
        elem_b="lower_swivel_collar",
        contact_tol=0.003,
        name="neck is visibly seated on body socket",
    )
    ctx.expect_contact(
        neck,
        head,
        elem_a="head_saddle_pad",
        elem_b="head_shell",
        contact_tol=0.001,
        name="saddle pad supports the lamp head",
    )
    ctx.expect_overlap(
        neck,
        head,
        axes="xz",
        elem_a="tilt_yoke",
        elem_b="head_shell",
        min_overlap=0.020,
        name="head is carried inside the yoke span",
    )
    ctx.expect_gap(
        head,
        body,
        axis="z",
        min_gap=0.035,
        name="head stays separated above body by aiming neck",
    )

    closed_door_aabb = ctx.part_element_world_aabb(door, elem="door_panel")
    with ctx.pose({door_joint: 1.2}):
        open_door_aabb = ctx.part_element_world_aabb(door, elem="door_panel")
    ctx.check(
        "battery door panel opens outward from side shell",
        closed_door_aabb is not None
        and open_door_aabb is not None
        and open_door_aabb[1][0] > closed_door_aabb[1][0] + 0.012,
        details=f"closed={closed_door_aabb}, open={open_door_aabb}",
    )

    closed_head_aabb = ctx.part_element_world_aabb(head, elem="diffuser_lens")
    with ctx.pose({tilt_joint: 0.8}):
        tilted_head_aabb = ctx.part_element_world_aabb(head, elem="diffuser_lens")
    ctx.check(
        "tilt joint lifts lamp face",
        closed_head_aabb is not None
        and tilted_head_aabb is not None
        and tilted_head_aabb[1][2] > closed_head_aabb[1][2] + 0.020,
        details=f"closed={closed_head_aabb}, tilted={tilted_head_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
