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
    model = ArticulatedObject(name="record_turntable")

    walnut = model.material("dark_walnut", rgba=(0.28, 0.15, 0.07, 1.0))
    black = model.material("satin_black", rgba=(0.01, 0.01, 0.012, 1.0))
    rubber = model.material("matte_rubber", rgba=(0.02, 0.02, 0.018, 1.0))
    vinyl = model.material("gloss_black_vinyl", rgba=(0.004, 0.004, 0.005, 1.0))
    aluminum = model.material("brushed_aluminum", rgba=(0.70, 0.72, 0.70, 1.0))
    red_label = model.material("red_record_label", rgba=(0.70, 0.05, 0.04, 1.0))
    brass = model.material("brass_stylus", rgba=(0.88, 0.68, 0.28, 1.0))

    plinth_h = 0.055
    platter_x = -0.065
    platter_y = 0.0
    platter_joint_z = 0.065

    # Main sprung plinth with a dark recessed deck, visible bearing washer,
    # and small vibration-isolating feet.
    plinth = model.part("plinth")
    plinth.visual(
        Box((0.60, 0.42, plinth_h)),
        origin=Origin(xyz=(0.0, 0.0, plinth_h / 2.0)),
        material=walnut,
        name="wood_body",
    )
    plinth.visual(
        Box((0.42, 0.30, 0.005)),
        origin=Origin(xyz=(platter_x, platter_y, plinth_h + 0.002)),
        material=black,
        name="recessed_deck",
    )
    plinth.visual(
        Cylinder(radius=0.055, length=0.006),
        origin=Origin(xyz=(platter_x, platter_y, plinth_h + 0.007)),
        material=aluminum,
        name="bearing_washer",
    )
    for idx, (x, y) in enumerate(
        ((-0.245, -0.155), (-0.245, 0.155), (0.245, -0.155), (0.245, 0.155))
    ):
        plinth.visual(
            Cylinder(radius=0.028, length=0.022),
            origin=Origin(xyz=(x, y, -0.009)),
            material=rubber,
            name=f"foot_{idx}",
        )

    # Paired side rails frame the rotating stage without touching it.
    side_supports = []
    for idx, y in enumerate((0.185, -0.185)):
        support = model.part(f"side_support_{idx}")
        support.visual(
            Box((0.42, 0.030, 0.065)),
            origin=Origin(xyz=(0.0, 0.0, 0.0325)),
            material=aluminum,
            name="side_rail",
        )
        model.articulation(
            f"plinth_to_side_support_{idx}",
            ArticulationType.FIXED,
            parent=plinth,
            child=support,
            origin=Origin(xyz=(platter_x, y, plinth_h)),
        )
        side_supports.append(support)

    # The entire stacked platter/record/spindle assembly is the continuously
    # rotating stage.
    platter = model.part("platter")
    platter.visual(
        Cylinder(radius=0.145, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=aluminum,
        name="platter_disk",
    )
    platter.visual(
        Cylinder(radius=0.133, length=0.005),
        origin=Origin(xyz=(0.0, 0.0, 0.032)),
        material=rubber,
        name="rubber_mat",
    )
    platter.visual(
        Cylinder(radius=0.128, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, 0.036)),
        material=vinyl,
        name="record_disc",
    )
    platter.visual(
        Cylinder(radius=0.028, length=0.0035),
        origin=Origin(xyz=(0.0, 0.0, 0.0395)),
        material=red_label,
        name="record_label",
    )
    platter.visual(
        Cylinder(radius=0.0055, length=0.054),
        origin=Origin(xyz=(0.0, 0.0, 0.057)),
        material=aluminum,
        name="center_spindle",
    )
    model.articulation(
        "platter_spin",
        ArticulationType.CONTINUOUS,
        parent=plinth,
        child=platter,
        origin=Origin(xyz=(platter_x, platter_y, platter_joint_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=12.0),
    )

    # Fixed tonearm bearing base, mounted off to the side of the platter.
    pivot_x = 0.205
    pivot_y = -0.135
    tonearm_base = model.part("tonearm_base")
    tonearm_base.visual(
        Cylinder(radius=0.035, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material=black,
        name="pedestal",
    )
    tonearm_base.visual(
        Cylinder(radius=0.026, length=0.025),
        origin=Origin(xyz=(0.0, 0.0, 0.0575)),
        material=aluminum,
        name="pivot_cap",
    )
    tonearm_base.visual(
        Box((0.085, 0.050, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=aluminum,
        name="mounting_plate",
    )
    model.articulation(
        "plinth_to_tonearm_base",
        ArticulationType.FIXED,
        parent=plinth,
        child=tonearm_base,
        origin=Origin(xyz=(pivot_x, pivot_y, plinth_h)),
    )

    # Tonearm local +X runs from its vertical pivot toward the record.  The
    # articulation origin carries the yaw that places the closed/rest pose over
    # the record surface; joint motion then sweeps around the vertical pivot.
    stylus_target = (-0.145, -0.025)
    rest_yaw = math.atan2(stylus_target[1] - pivot_y, stylus_target[0] - pivot_x)

    tonearm = model.part("tonearm")
    tonearm.visual(
        Cylinder(radius=0.018, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=aluminum,
        name="pivot_collar",
    )
    tonearm.visual(
        Cylinder(radius=0.006, length=0.335),
        origin=Origin(xyz=(0.1675, 0.0, 0.012), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=aluminum,
        name="arm_wand",
    )
    tonearm.visual(
        Cylinder(radius=0.019, length=0.060),
        origin=Origin(xyz=(-0.045, 0.0, 0.026), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="counterweight",
    )
    tonearm.visual(
        Box((0.050, 0.026, 0.009)),
        origin=Origin(xyz=(0.355, 0.0, 0.006)),
        material=black,
        name="headshell",
    )
    tonearm.visual(
        Cylinder(radius=0.0018, length=0.028),
        origin=Origin(xyz=(0.371, 0.0, -0.012)),
        material=brass,
        name="stylus",
    )
    model.articulation(
        "tonearm_swing",
        ArticulationType.REVOLUTE,
        parent=tonearm_base,
        child=tonearm,
        origin=Origin(xyz=(0.0, 0.0, 0.070), rpy=(0.0, 0.0, rest_yaw)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.0, velocity=1.8, lower=-0.42, upper=0.46),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    plinth = object_model.get_part("plinth")
    platter = object_model.get_part("platter")
    support_0 = object_model.get_part("side_support_0")
    support_1 = object_model.get_part("side_support_1")
    tonearm_base = object_model.get_part("tonearm_base")
    tonearm = object_model.get_part("tonearm")
    platter_spin = object_model.get_articulation("platter_spin")
    tonearm_swing = object_model.get_articulation("tonearm_swing")

    ctx.expect_contact(
        platter,
        plinth,
        elem_a="platter_disk",
        elem_b="bearing_washer",
        contact_tol=0.001,
        name="platter sits on bearing washer",
    )
    ctx.expect_gap(
        support_0,
        platter,
        axis="y",
        min_gap=0.015,
        positive_elem="side_rail",
        negative_elem="platter_disk",
        name="positive side support clears platter",
    )
    ctx.expect_gap(
        platter,
        support_1,
        axis="y",
        min_gap=0.015,
        positive_elem="platter_disk",
        negative_elem="side_rail",
        name="negative side support clears platter",
    )
    ctx.expect_contact(
        tonearm,
        tonearm_base,
        elem_a="pivot_collar",
        elem_b="pivot_cap",
        contact_tol=0.001,
        name="tonearm collar seats on pivot cap",
    )

    rest_platter_pos = ctx.part_world_position(platter)
    with ctx.pose({platter_spin: 1.7}):
        spun_platter_pos = ctx.part_world_position(platter)
    ctx.check(
        "platter continuous joint stays centered",
        rest_platter_pos is not None
        and spun_platter_pos is not None
        and all(abs(a - b) < 1e-6 for a, b in zip(rest_platter_pos, spun_platter_pos)),
        details=f"rest={rest_platter_pos}, spun={spun_platter_pos}",
    )

    def elem_center_zxy(part, elem):
        aabb = ctx.part_element_world_aabb(part, elem=elem)
        if aabb is None:
            return None
        lo, hi = aabb
        return (
            (lo[0] + hi[0]) / 2.0,
            (lo[1] + hi[1]) / 2.0,
            (lo[2] + hi[2]) / 2.0,
        )

    rest_stylus = elem_center_zxy(tonearm, "stylus")
    with ctx.pose({tonearm_swing: 0.35}):
        swept_stylus = elem_center_zxy(tonearm, "stylus")
    ctx.check(
        "tonearm swing moves stylus over record arc",
        rest_stylus is not None
        and swept_stylus is not None
        and math.dist(rest_stylus[:2], swept_stylus[:2]) > 0.08,
        details=f"rest={rest_stylus}, swept={swept_stylus}",
    )

    return ctx.report()


object_model = build_object_model()
