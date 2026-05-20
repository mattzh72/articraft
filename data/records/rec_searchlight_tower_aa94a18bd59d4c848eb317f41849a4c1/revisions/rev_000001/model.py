from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BezelGeometry,
    Box,
    Cylinder,
    KnobGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="desktop_searchlight_tower")

    graphite = model.material("graphite", rgba=(0.11, 0.12, 0.13, 1.0))
    satin_black = model.material("satin_black", rgba=(0.025, 0.027, 0.030, 1.0))
    warm_white = model.material("warm_white", rgba=(0.92, 0.88, 0.78, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.63, 0.64, 0.62, 1.0))
    rubber = model.material("rubber", rgba=(0.015, 0.015, 0.014, 1.0))
    lens_blue = model.material("cool_lens", rgba=(0.55, 0.78, 0.95, 0.42))
    amber = model.material("amber_switch", rgba=(1.0, 0.48, 0.12, 1.0))

    bezel_mesh = mesh_from_geometry(
        BezelGeometry(
            (0.088, 0.088),
            (0.122, 0.122),
            0.014,
            opening_shape="circle",
            outer_shape="circle",
            center=True,
        ),
        "spotlight_front_bezel",
    )
    knob_mesh = mesh_from_geometry(
        KnobGeometry(
            0.036,
            0.018,
            body_style="lobed",
            crown_radius=0.0012,
            edge_radius=0.0008,
            center=False,
        ),
        "tilt_lock_knob",
    )

    base = model.part("base")
    base.visual(
        Box((0.46, 0.28, 0.035)),
        origin=Origin(xyz=(0.04, 0.0, 0.0175)),
        material=graphite,
        name="base_plate",
    )
    base.visual(
        Box((0.33, 0.18, 0.016)),
        origin=Origin(xyz=(0.07, 0.0, 0.043)),
        material=satin_black,
        name="top_inset",
    )
    for y in (-0.058, 0.058):
        base.visual(
            Box((0.055, 0.022, 0.075)),
            origin=Origin(xyz=(-0.15, y, 0.059)),
            material=brushed_steel,
            name=f"hinge_ear_{0 if y < 0 else 1}",
        )
        base.visual(
            Cylinder(radius=0.019, length=0.006),
            origin=Origin(xyz=(-0.15, y * 1.24, 0.059), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=brushed_steel,
            name=f"hinge_washer_{0 if y < 0 else 1}",
        )
    base.visual(
        Box((0.11, 0.145, 0.012)),
        origin=Origin(xyz=(0.235, 0.0, 0.047)),
        material=rubber,
        name="stow_pad",
    )
    for y in (-0.061, 0.061):
        base.visual(
            Box((0.095, 0.022, 0.036)),
            origin=Origin(xyz=(0.235, y, 0.064)),
            material=rubber,
            name=f"stow_cradle_{0 if y < 0 else 1}",
        )
    for x, y in [(-0.14, -0.10), (-0.14, 0.10), (0.23, -0.10), (0.23, 0.10)]:
        base.visual(
            Cylinder(radius=0.028, length=0.006),
            origin=Origin(xyz=(x, y, 0.003)),
            material=rubber,
            name=f"foot_{x}_{y}",
        )

    folding_mast = model.part("folding_mast")
    folding_mast.visual(
        Cylinder(radius=0.018, length=0.094),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brushed_steel,
        name="hinge_knuckle",
    )
    folding_mast.visual(
        Box((0.040, 0.040, 0.318)),
        origin=Origin(xyz=(0.0, 0.0, 0.174)),
        material=graphite,
        name="mast_tube",
    )
    folding_mast.visual(
        Box((0.060, 0.052, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, 0.034)),
        material=brushed_steel,
        name="root_collar",
    )
    folding_mast.visual(
        Cylinder(radius=0.032, length=0.034),
        origin=Origin(xyz=(0.0, 0.0, 0.350)),
        material=brushed_steel,
        name="pan_socket",
    )
    folding_mast.visual(
        Box((0.026, 0.048, 0.250)),
        origin=Origin(xyz=(0.021, 0.0, 0.185)),
        material=satin_black,
        name="cable_channel",
    )

    pan_yoke = model.part("pan_yoke")
    pan_yoke.visual(
        Cylinder(radius=0.052, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
        material=brushed_steel,
        name="pan_turntable",
    )
    pan_yoke.visual(
        Box((0.075, 0.230, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, 0.032)),
        material=graphite,
        name="lower_crossbar",
    )
    for y, arm_name, cup_name in (
        (-0.092, "yoke_arm_0", "bearing_cup_0"),
        (0.092, "yoke_arm_1", "bearing_cup_1"),
    ):
        pan_yoke.visual(
            Box((0.038, 0.026, 0.218)),
            origin=Origin(xyz=(0.0, y, 0.126)),
            material=graphite,
            name=arm_name,
        )
        pan_yoke.visual(
            Cylinder(radius=0.021, length=0.010),
            origin=Origin(xyz=(0.0, 0.076 if y > 0 else -0.076, 0.205), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=brushed_steel,
            name=cup_name,
        )
    pan_yoke.visual(
        Box((0.055, 0.060, 0.018)),
        origin=Origin(xyz=(-0.030, 0.0, 0.054)),
        material=satin_black,
        name="pan_stop_block",
    )

    spotlight_head = model.part("spotlight_head")
    spotlight_head.visual(
        Cylinder(radius=0.055, length=0.140),
        origin=Origin(xyz=(0.025, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=warm_white,
        name="housing_shell",
    )
    spotlight_head.visual(
        Cylinder(radius=0.050, length=0.020),
        origin=Origin(xyz=(-0.055, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=graphite,
        name="rear_cap",
    )
    spotlight_head.visual(
        bezel_mesh,
        origin=Origin(xyz=(0.102, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=graphite,
        name="front_bezel",
    )
    spotlight_head.visual(
        Cylinder(radius=0.045, length=0.006),
        origin=Origin(xyz=(0.111, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=lens_blue,
        name="front_lens",
    )
    spotlight_head.visual(
        Cylinder(radius=0.038, length=0.010),
        origin=Origin(xyz=(0.076, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_steel,
        name="reflector_bowl",
    )
    for y, trunnion_name in ((-0.062, "trunnion_0"), (0.062, "trunnion_1")):
        spotlight_head.visual(
            Cylinder(radius=0.014, length=0.018),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=brushed_steel,
            name=trunnion_name,
        )
    spotlight_head.visual(
        Box((0.035, 0.018, 0.036)),
        origin=Origin(xyz=(-0.030, -0.020, 0.068)),
        material=graphite,
        name="handle_post_0",
    )
    spotlight_head.visual(
        Box((0.035, 0.018, 0.036)),
        origin=Origin(xyz=(0.050, -0.020, 0.068)),
        material=graphite,
        name="handle_post_1",
    )
    spotlight_head.visual(
        Box((0.116, 0.022, 0.024)),
        origin=Origin(xyz=(0.010, -0.020, 0.092)),
        material=graphite,
        name="top_handle",
    )
    for z in (-0.024, 0.0, 0.024):
        spotlight_head.visual(
            Box((0.004, 0.062, 0.006)),
            origin=Origin(xyz=(-0.067, 0.0, z)),
            material=satin_black,
            name=f"rear_vent_{z}",
        )

    power_switch = model.part("power_switch")
    power_switch.visual(
        Box((0.046, 0.028, 0.009)),
        origin=Origin(xyz=(0.0, 0.0, 0.0045)),
        material=amber,
        name="rocker_cap",
    )
    power_switch.visual(
        Box((0.052, 0.034, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.0015)),
        material=satin_black,
        name="switch_bezel",
    )

    tilt_knob = model.part("tilt_knob")
    tilt_knob.visual(knob_mesh, material=graphite, name="lobed_cap")

    model.articulation(
        "base_to_mast",
        ArticulationType.REVOLUTE,
        parent=base,
        child=folding_mast,
        origin=Origin(xyz=(-0.15, 0.0, 0.059)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.2, lower=0.0, upper=1.25),
    )
    model.articulation(
        "mast_to_yoke",
        ArticulationType.REVOLUTE,
        parent=folding_mast,
        child=pan_yoke,
        origin=Origin(xyz=(0.0, 0.0, 0.367)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.8, lower=-2.75, upper=2.75),
    )
    model.articulation(
        "yoke_to_head",
        ArticulationType.REVOLUTE,
        parent=pan_yoke,
        child=spotlight_head,
        origin=Origin(xyz=(0.0, 0.0, 0.205)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=1.5, lower=-1.25, upper=1.25),
    )
    model.articulation(
        "base_to_switch",
        ArticulationType.REVOLUTE,
        parent=base,
        child=power_switch,
        origin=Origin(xyz=(0.175, -0.085, 0.051)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=5.0, lower=-0.18, upper=0.18),
    )
    model.articulation(
        "yoke_to_knob",
        ArticulationType.CONTINUOUS,
        parent=pan_yoke,
        child=tilt_knob,
        origin=Origin(xyz=(0.0, 0.105, 0.205), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.25, velocity=8.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    mast = object_model.get_part("folding_mast")
    yoke = object_model.get_part("pan_yoke")
    head = object_model.get_part("spotlight_head")
    base = object_model.get_part("base")
    pan = object_model.get_articulation("mast_to_yoke")
    tilt = object_model.get_articulation("yoke_to_head")
    fold = object_model.get_articulation("base_to_mast")

    ctx.check(
        "separated pan and tilt axes",
        pan.axis == (0.0, 0.0, 1.0)
        and tilt.axis == (0.0, 1.0, 0.0)
        and tilt.origin is not None
        and tilt.origin.xyz[2] > 0.18,
        details=f"pan_axis={pan.axis}, tilt_axis={tilt.axis}, tilt_origin={tilt.origin}",
    )

    ctx.expect_contact(
        yoke,
        head,
        elem_a="bearing_cup_1",
        elem_b="trunnion_1",
        contact_tol=0.001,
        name="positive side bearing carries tilt axle",
    )
    ctx.expect_contact(
        yoke,
        head,
        elem_a="bearing_cup_0",
        elem_b="trunnion_0",
        contact_tol=0.001,
        name="negative side bearing carries tilt axle",
    )

    for angle, label in ((1.25, "down"), (-1.25, "up")):
        with ctx.pose({tilt: angle}):
            ctx.expect_gap(
                head,
                yoke,
                axis="z",
                positive_elem="housing_shell",
                negative_elem="lower_crossbar",
                min_gap=0.020,
                name=f"tilted {label} head clears yoke crossbar",
            )

    upright_pos = ctx.part_world_position(head)
    with ctx.pose({fold: 1.25, tilt: -1.25, pan: 0.0}):
        ctx.expect_gap(
            head,
            base,
            axis="z",
            min_gap=0.035,
            max_gap=0.20,
            name="folded head clears base cradle",
        )
        stowed_pos = ctx.part_world_position(head)

    ctx.check(
        "folded pose lowers the spotlight for stowage",
        upright_pos is not None
        and stowed_pos is not None
        and stowed_pos[2] < upright_pos[2] - 0.30,
        details=f"upright={upright_pos}, stowed={stowed_pos}",
    )

    return ctx.report()


object_model = build_object_model()
