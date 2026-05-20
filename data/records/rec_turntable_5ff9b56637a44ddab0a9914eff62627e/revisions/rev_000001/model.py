from __future__ import annotations

from math import pi

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


PLINTH_TOP_Z = 0.040
PLATTER_BEARING_TOP_Z = 0.075
TONEARM_PIVOT_XYZ = (0.180, 0.105, 0.130)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="record_turntable")

    walnut = model.material("satin_walnut", rgba=(0.34, 0.19, 0.09, 1.0))
    black = model.material("matte_black", rgba=(0.015, 0.015, 0.014, 1.0))
    rubber = model.material("soft_rubber", rgba=(0.035, 0.035, 0.034, 1.0))
    brushed = model.material("brushed_aluminum", rgba=(0.72, 0.72, 0.68, 1.0))
    dark_metal = model.material("dark_anodized_metal", rgba=(0.12, 0.13, 0.14, 1.0))
    needle = model.material("diamond_tip", rgba=(0.86, 0.88, 0.90, 1.0))
    label_red = model.material("record_label_red", rgba=(0.55, 0.04, 0.035, 1.0))

    plinth = model.part("plinth")
    plinth_body = (
        cq.Workplane("XY")
        .box(0.480, 0.360, PLINTH_TOP_Z)
        .edges("|Z")
        .fillet(0.030)
    )
    plinth.visual(
        mesh_from_cadquery(plinth_body, "rounded_plinth"),
        origin=Origin(xyz=(0.0, 0.0, PLINTH_TOP_Z * 0.5)),
        material=walnut,
        name="plinth_body",
    )
    plinth.visual(
        Box((0.440, 0.020, 0.010)),
        origin=Origin(xyz=(0.0, -0.181, 0.026)),
        material=black,
        name="front_shadow_gap",
    )
    plinth.visual(
        Cylinder(radius=0.036, length=PLATTER_BEARING_TOP_Z - PLINTH_TOP_Z),
        origin=Origin(xyz=(0.0, 0.0, (PLINTH_TOP_Z + PLATTER_BEARING_TOP_Z) * 0.5)),
        material=brushed,
        name="center_support",
    )
    plinth.visual(
        Cylinder(radius=0.044, length=0.010),
        origin=Origin(xyz=(TONEARM_PIVOT_XYZ[0], TONEARM_PIVOT_XYZ[1], PLINTH_TOP_Z + 0.004)),
        material=dark_metal,
        name="tonearm_base_foot",
    )
    plinth.visual(
        Cylinder(radius=0.024, length=TONEARM_PIVOT_XYZ[2] - PLINTH_TOP_Z),
        origin=Origin(xyz=(TONEARM_PIVOT_XYZ[0], TONEARM_PIVOT_XYZ[1], (PLINTH_TOP_Z + TONEARM_PIVOT_XYZ[2]) * 0.5)),
        material=brushed,
        name="tonearm_pedestal",
    )
    plinth.visual(
        Cylinder(radius=0.014, length=0.012),
        origin=Origin(xyz=(TONEARM_PIVOT_XYZ[0], TONEARM_PIVOT_XYZ[1], TONEARM_PIVOT_XYZ[2] - 0.006)),
        material=black,
        name="pivot_bushing",
    )

    platter = model.part("platter")
    platter.visual(
        Cylinder(radius=0.164, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=brushed,
        name="platter_disc",
    )
    platter.visual(
        Cylinder(radius=0.148, length=0.005),
        origin=Origin(xyz=(0.0, 0.0, 0.0265)),
        material=rubber,
        name="rubber_mat",
    )
    platter.visual(
        Cylinder(radius=0.136, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, 0.0305)),
        material=black,
        name="vinyl_record",
    )
    platter.visual(
        Cylinder(radius=0.035, length=0.0035),
        origin=Origin(xyz=(0.0, 0.0, 0.03375)),
        material=label_red,
        name="record_label",
    )
    platter.visual(
        Cylinder(radius=0.007, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.039)),
        material=brushed,
        name="center_spindle",
    )

    tonearm = model.part("tonearm")
    tonearm.visual(
        Cylinder(radius=0.020, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=dark_metal,
        name="pivot_cap",
    )
    tonearm.visual(
        Cylinder(radius=0.0042, length=0.168),
        origin=Origin(xyz=(-0.088, 0.0, 0.008), rpy=(0.0, pi / 2.0, 0.0)),
        material=brushed,
        name="arm_tube",
    )
    tonearm.visual(
        Cylinder(radius=0.015, length=0.046),
        origin=Origin(xyz=(0.035, 0.0, 0.018), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_metal,
        name="counterweight",
    )
    tonearm.visual(
        Box((0.012, 0.010, 0.011)),
        origin=Origin(xyz=(-0.158, 0.0, 0.003)),
        material=dark_metal,
        name="headshell_bracket",
    )
    tonearm.visual(
        Box((0.040, 0.024, 0.005)),
        origin=Origin(xyz=(-0.176, 0.0, -0.001)),
        material=dark_metal,
        name="headshell",
    )
    tonearm.visual(
        Box((0.019, 0.014, 0.014)),
        origin=Origin(xyz=(-0.187, 0.0, -0.010)),
        material=black,
        name="cartridge",
    )
    tonearm.visual(
        Cylinder(radius=0.0011, length=0.006),
        origin=Origin(xyz=(-0.190, 0.0, -0.019)),
        material=needle,
        name="stylus",
    )

    model.articulation(
        "platter_spin",
        ArticulationType.CONTINUOUS,
        parent=plinth,
        child=platter,
        origin=Origin(xyz=(0.0, 0.0, PLATTER_BEARING_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=4.0),
    )
    model.articulation(
        "tonearm_swing",
        ArticulationType.REVOLUTE,
        parent=plinth,
        child=tonearm,
        origin=Origin(xyz=TONEARM_PIVOT_XYZ),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.7, velocity=1.2, lower=-0.25, upper=0.32),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    plinth = object_model.get_part("plinth")
    platter = object_model.get_part("platter")
    tonearm = object_model.get_part("tonearm")
    platter_spin = object_model.get_articulation("platter_spin")
    tonearm_swing = object_model.get_articulation("tonearm_swing")

    ctx.expect_contact(
        platter,
        plinth,
        elem_a="platter_disc",
        elem_b="center_support",
        contact_tol=0.0015,
        name="platter rests on the single central support",
    )
    ctx.expect_within(
        plinth,
        platter,
        axes="xy",
        inner_elem="center_support",
        outer_elem="platter_disc",
        margin=0.0,
        name="central support is under platter footprint",
    )
    ctx.expect_gap(
        tonearm,
        platter,
        axis="z",
        positive_elem="stylus",
        negative_elem="vinyl_record",
        min_gap=0.0003,
        max_gap=0.004,
        name="stylus hovers just above the record surface",
    )

    ctx.check(
        "platter joint is continuous vertical spin",
        platter_spin.articulation_type == ArticulationType.CONTINUOUS
        and tuple(platter_spin.axis) == (0.0, 0.0, 1.0),
        details=f"type={platter_spin.articulation_type}, axis={platter_spin.axis}",
    )
    ctx.check(
        "tonearm joint is a limited vertical swing",
        tonearm_swing.articulation_type == ArticulationType.REVOLUTE
        and tuple(tonearm_swing.axis) == (0.0, 0.0, 1.0)
        and tonearm_swing.motion_limits is not None
        and tonearm_swing.motion_limits.lower < 0.0
        and tonearm_swing.motion_limits.upper > 0.25,
        details=f"type={tonearm_swing.articulation_type}, axis={tonearm_swing.axis}, limits={tonearm_swing.motion_limits}",
    )

    pivot_xy_distance = (TONEARM_PIVOT_XYZ[0] ** 2 + TONEARM_PIVOT_XYZ[1] ** 2) ** 0.5
    ctx.check(
        "tonearm base is off to one side of the platter",
        pivot_xy_distance > 0.18,
        details=f"pivot_xy_distance={pivot_xy_distance:.3f}",
    )

    rest_box = ctx.part_element_world_aabb(tonearm, elem="headshell")
    with ctx.pose({tonearm_swing: tonearm_swing.motion_limits.upper}):
        ctx.expect_gap(
            tonearm,
            platter,
            axis="y",
            positive_elem="stylus",
            negative_elem="record_label",
            min_gap=0.004,
            name="tonearm swing stops before the record label",
        )
        swept_box = ctx.part_element_world_aabb(tonearm, elem="headshell")
    if rest_box is not None and swept_box is not None:
        rest_center_y = (rest_box[0][1] + rest_box[1][1]) * 0.5
        swept_center_y = (swept_box[0][1] + swept_box[1][1]) * 0.5
        swing_delta = abs(swept_center_y - rest_center_y)
    else:
        swing_delta = 0.0
    ctx.check(
        "tonearm headshell sweeps across the record",
        swing_delta > 0.045,
        details=f"swing_delta_y={swing_delta:.3f}",
    )

    return ctx.report()


object_model = build_object_model()
