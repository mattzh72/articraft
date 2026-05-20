from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    Material,
    MotionLimits,
    Origin,
    TorusGeometry,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="open_support_record_turntable")

    satin_wood = Material("satin_walnut", rgba=(0.48, 0.28, 0.13, 1.0))
    black = Material("matte_black", rgba=(0.01, 0.01, 0.012, 1.0))
    rubber = Material("black_rubber", rgba=(0.005, 0.005, 0.006, 1.0))
    metal = Material("brushed_aluminum", rgba=(0.72, 0.70, 0.66, 1.0))
    dark_metal = Material("dark_anodized_metal", rgba=(0.08, 0.085, 0.09, 1.0))
    brass = Material("warm_brass", rgba=(0.90, 0.62, 0.25, 1.0))
    red = Material("record_label_red", rgba=(0.58, 0.04, 0.035, 1.0))

    platter_center = (-0.055, 0.0, 0.098)
    tonearm_pivot = (0.165, 0.075, 0.168)

    plinth = model.part("plinth")
    # A light, open rectangular support rather than a solid plinth block.
    plinth.visual(
        Box((0.55, 0.034, 0.038)),
        origin=Origin(xyz=(0.0, -0.19, 0.039)),
        material=satin_wood,
        name="front_rail",
    )
    plinth.visual(
        Box((0.55, 0.034, 0.038)),
        origin=Origin(xyz=(0.0, 0.19, 0.039)),
        material=satin_wood,
        name="rear_rail",
    )
    plinth.visual(
        Box((0.034, 0.38, 0.038)),
        origin=Origin(xyz=(-0.258, 0.0, 0.039)),
        material=satin_wood,
        name="side_rail_0",
    )
    plinth.visual(
        Box((0.034, 0.38, 0.038)),
        origin=Origin(xyz=(0.258, 0.0, 0.039)),
        material=satin_wood,
        name="side_rail_1",
    )
    plinth.visual(
        Box((0.435, 0.018, 0.024)),
        origin=Origin(xyz=(-0.055, 0.0, 0.050)),
        material=dark_metal,
        name="cross_member_x",
    )
    plinth.visual(
        Box((0.018, 0.360, 0.024)),
        origin=Origin(xyz=(-0.055, 0.0, 0.050)),
        material=dark_metal,
        name="cross_member_y",
    )
    for i, x in enumerate((-0.250, 0.250)):
        for j, y in enumerate((-0.172, 0.172)):
            plinth.visual(
                Cylinder(radius=0.017, length=0.040),
                origin=Origin(xyz=(x, y, 0.020)),
                material=dark_metal,
                name=f"foot_{i}_{j}",
            )

    plinth.visual(
        Cylinder(radius=0.046, length=0.016),
        origin=Origin(xyz=(-0.055, 0.0, 0.071)),
        material=dark_metal,
        name="bearing_plate",
    )
    plinth.visual(
        Cylinder(radius=0.024, length=0.042),
        origin=Origin(xyz=(-0.055, 0.0, 0.073)),
        material=metal,
        name="bearing_column",
    )
    plinth.visual(
        Cylinder(radius=0.031, length=0.006),
        origin=Origin(xyz=(-0.055, 0.0, 0.097)),
        material=brass,
        name="bearing_cap",
    )

    plinth.visual(
        Box((0.026, 0.150, 0.020)),
        origin=Origin(xyz=(tonearm_pivot[0], 0.0375, 0.052)),
        material=dark_metal,
        name="armboard_bridge",
    )
    plinth.visual(
        Cylinder(radius=0.030, length=0.034),
        origin=Origin(xyz=(tonearm_pivot[0], tonearm_pivot[1], 0.069)),
        material=dark_metal,
        name="armboard_post",
    )
    plinth.visual(
        Cylinder(radius=0.044, length=0.010),
        origin=Origin(xyz=(tonearm_pivot[0], tonearm_pivot[1], 0.088)),
        material=dark_metal,
        name="armboard_base",
    )
    plinth.visual(
        Cylinder(radius=0.020, length=0.070),
        origin=Origin(xyz=(tonearm_pivot[0], tonearm_pivot[1], 0.123)),
        material=metal,
        name="tonearm_pillar",
    )
    plinth.visual(
        Cylinder(radius=0.016, length=0.004),
        origin=Origin(xyz=(tonearm_pivot[0], tonearm_pivot[1], 0.160)),
        material=brass,
        name="pivot_cup",
    )
    plinth.visual(
        mesh_from_geometry(
            TorusGeometry(radius=0.023, tube=0.0035, radial_segments=18, tubular_segments=32),
            "pivot_ring",
        ),
        origin=Origin(xyz=(tonearm_pivot[0], tonearm_pivot[1], 0.1575)),
        material=dark_metal,
        name="pivot_ring",
    )

    platter = model.part("platter")
    platter_profile = [
        (0.000, 0.000),
        (0.122, 0.000),
        (0.156, 0.006),
        (0.160, 0.018),
        (0.148, 0.029),
        (0.000, 0.029),
    ]
    platter.visual(
        mesh_from_geometry(
            LatheGeometry(platter_profile, segments=96, closed=True),
            "stepped_platter",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=metal,
        name="platter_body",
    )
    platter.visual(
        Cylinder(radius=0.151, length=0.0045),
        origin=Origin(xyz=(0.0, 0.0, 0.034)),
        material=rubber,
        name="record_disc",
    )
    platter.visual(
        Cylinder(radius=0.038, length=0.0016),
        origin=Origin(xyz=(0.0, 0.0, 0.03705)),
        material=red,
        name="center_label",
    )
    platter.visual(
        Cylinder(radius=0.005, length=0.048),
        origin=Origin(xyz=(0.0, 0.0, 0.042)),
        material=brass,
        name="spindle",
    )

    tonearm = model.part("tonearm")
    tonearm.visual(
        Cylinder(radius=0.017, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=dark_metal,
        name="pivot_hub",
    )
    tonearm.visual(
        Cylinder(radius=0.010, length=0.036),
        origin=Origin(xyz=(0.0, 0.0, 0.006), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal,
        name="gimbal_barrel",
    )
    arm_tube = tube_from_spline_points(
        [
            (-0.016, 0.000, 0.004),
            (-0.070, -0.004, 0.004),
            (-0.150, -0.010, 0.000),
            (-0.228, -0.012, -0.006),
            (-0.248, -0.012, -0.011),
        ],
        radius=0.0042,
        samples_per_segment=12,
        radial_segments=20,
        cap_ends=True,
    )
    tonearm.visual(
        mesh_from_geometry(arm_tube, "tonearm_tube"),
        material=metal,
        name="arm_tube",
    )
    tonearm.visual(
        Box((0.049, 0.026, 0.006)),
        origin=Origin(xyz=(-0.271, -0.012, -0.013), rpy=(0.0, 0.0, -0.08)),
        material=black,
        name="headshell",
    )
    tonearm.visual(
        Box((0.018, 0.014, 0.010)),
        origin=Origin(xyz=(-0.286, -0.012, -0.0205)),
        material=dark_metal,
        name="cartridge",
    )
    tonearm.visual(
        Cylinder(radius=0.0012, length=0.006),
        origin=Origin(xyz=(-0.294, -0.012, -0.028), rpy=(0.18, 0.0, 0.0)),
        material=brass,
        name="stylus",
    )
    tonearm.visual(
        Cylinder(radius=0.005, length=0.045),
        origin=Origin(xyz=(0.0275, 0.0, 0.012), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal,
        name="counterweight_shaft",
    )
    tonearm.visual(
        Cylinder(radius=0.014, length=0.034),
        origin=Origin(xyz=(0.065, 0.0, 0.021), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="counterweight",
    )

    model.articulation(
        "platter_spin",
        ArticulationType.CONTINUOUS,
        parent=plinth,
        child=platter,
        origin=Origin(xyz=platter_center),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=20.0),
    )
    model.articulation(
        "tonearm_swing",
        ArticulationType.REVOLUTE,
        parent=plinth,
        child=tonearm,
        origin=Origin(xyz=tonearm_pivot),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.25, velocity=1.2, lower=-0.55, upper=0.55),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    plinth = object_model.get_part("plinth")
    platter = object_model.get_part("platter")
    tonearm = object_model.get_part("tonearm")
    platter_spin = object_model.get_articulation("platter_spin")
    tonearm_swing = object_model.get_articulation("tonearm_swing")

    ctx.check(
        "platter has continuous vertical spin",
        platter_spin.articulation_type == ArticulationType.CONTINUOUS
        and tuple(platter_spin.axis) == (0.0, 0.0, 1.0),
        details=f"type={platter_spin.articulation_type}, axis={platter_spin.axis}",
    )
    ctx.check(
        "tonearm has limited vertical pivot swing",
        tonearm_swing.articulation_type == ArticulationType.REVOLUTE
        and tuple(tonearm_swing.axis) == (0.0, 0.0, 1.0)
        and tonearm_swing.motion_limits is not None
        and tonearm_swing.motion_limits.lower < 0.0
        and tonearm_swing.motion_limits.upper > 0.0,
        details=f"type={tonearm_swing.articulation_type}, axis={tonearm_swing.axis}, limits={tonearm_swing.motion_limits}",
    )
    ctx.expect_gap(
        platter,
        plinth,
        axis="z",
        positive_elem="platter_body",
        negative_elem="bearing_cap",
        min_gap=0.0,
        max_gap=0.001,
        name="exposed platter clears bearing cap",
    )
    ctx.expect_overlap(
        platter,
        plinth,
        axes="xy",
        elem_a="platter_body",
        elem_b="bearing_cap",
        min_overlap=0.025,
        name="platter is centered over exposed bearing",
    )
    ctx.expect_gap(
        tonearm,
        platter,
        axis="z",
        positive_elem="stylus",
        negative_elem="record_disc",
        min_gap=0.001,
        max_gap=0.012,
        name="stylus hovers just above the record",
    )

    rest_headshell_aabb = ctx.part_element_world_aabb(tonearm, elem="headshell")
    with ctx.pose({tonearm_swing: 0.45}):
        swept_headshell_aabb = ctx.part_element_world_aabb(tonearm, elem="headshell")
        ctx.expect_gap(
            tonearm,
            platter,
            axis="z",
            positive_elem="headshell",
            negative_elem="record_disc",
            min_gap=0.010,
            name="headshell clears record while swung",
        )
    rest_headshell_y = (
        (rest_headshell_aabb[0][1] + rest_headshell_aabb[1][1]) * 0.5
        if rest_headshell_aabb is not None
        else None
    )
    swept_headshell_y = (
        (swept_headshell_aabb[0][1] + swept_headshell_aabb[1][1]) * 0.5
        if swept_headshell_aabb is not None
        else None
    )
    ctx.check(
        "tonearm swing moves headshell across record",
        rest_headshell_y is not None
        and swept_headshell_y is not None
        and abs(swept_headshell_y - rest_headshell_y) > 0.05,
        details=f"rest_y={rest_headshell_y}, swept_y={swept_headshell_y}",
    )

    return ctx.report()


object_model = build_object_model()
