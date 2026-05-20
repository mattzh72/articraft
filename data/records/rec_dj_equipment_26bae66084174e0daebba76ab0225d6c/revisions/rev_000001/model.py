from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dj_turntable")

    body_mat = model.material("satin_black_body", rgba=(0.025, 0.026, 0.028, 1.0))
    top_mat = model.material("dark_top_plate", rgba=(0.055, 0.058, 0.062, 1.0))
    slot_mat = model.material("black_recess", rgba=(0.005, 0.005, 0.006, 1.0))
    metal_mat = model.material("brushed_aluminum", rgba=(0.68, 0.68, 0.64, 1.0))
    vinyl_mat = model.material("gloss_black_vinyl", rgba=(0.003, 0.003, 0.004, 1.0))
    label_mat = model.material("record_label_red", rgba=(0.70, 0.05, 0.03, 1.0))
    rubber_mat = model.material("matte_rubber", rgba=(0.012, 0.012, 0.013, 1.0))
    arm_mat = model.material("silver_tonearm", rgba=(0.82, 0.82, 0.78, 1.0))
    cartridge_mat = model.material("black_cartridge", rgba=(0.015, 0.015, 0.017, 1.0))
    fader_mat = model.material("light_gray_fader_cap", rgba=(0.77, 0.77, 0.72, 1.0))

    body = model.part("body")
    body.visual(
        Box((0.48, 0.38, 0.060)),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=body_mat,
        name="body_shell",
    )
    body.visual(
        Box((0.455, 0.355, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.062)),
        material=top_mat,
        name="top_plate",
    )

    # Rubber feet touch the underside of the plinth and make the slab read as a
    # real tabletop appliance instead of a floating block.
    for index, (x, y) in enumerate(
        ((-0.195, -0.145), (-0.195, 0.145), (0.195, -0.145), (0.195, 0.145))
    ):
        body.visual(
            Cylinder(radius=0.023, length=0.012),
            origin=Origin(xyz=(x, y, -0.006)),
            material=rubber_mat,
            name=f"rubber_foot_{index}",
        )

    # Pitch-control track, carried by the body; the moving cap below rides on it.
    body.visual(
        Box((0.024, 0.180, 0.004)),
        origin=Origin(xyz=(0.175, -0.075, 0.066)),
        material=slot_mat,
        name="fader_slot",
    )

    # Stationary tonearm base and bearing post.
    body.visual(
        Cylinder(radius=0.028, length=0.032),
        origin=Origin(xyz=(0.155, 0.115, 0.078)),
        material=metal_mat,
        name="tonearm_base",
    )
    body.visual(
        Cylinder(radius=0.012, length=0.018),
        origin=Origin(xyz=(0.155, 0.115, 0.103)),
        material=metal_mat,
        name="pivot_post",
    )

    # A low arm rest beside the hinge gives the tonearm a visible support point.
    body.visual(
        Box((0.042, 0.018, 0.014)),
        origin=Origin(xyz=(0.190, 0.060, 0.071)),
        material=rubber_mat,
        name="arm_rest",
    )

    platter = model.part("platter")
    platter.visual(
        Cylinder(radius=0.150, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
        material=metal_mat,
        name="metal_platter",
    )
    platter.visual(
        Cylinder(radius=0.142, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.029)),
        material=vinyl_mat,
        name="vinyl_record",
    )
    platter.visual(
        Cylinder(radius=0.036, length=0.002),
        origin=Origin(xyz=(0.0, 0.0, 0.033)),
        material=label_mat,
        name="record_label",
    )
    platter.visual(
        Cylinder(radius=0.004, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.041)),
        material=metal_mat,
        name="center_spindle",
    )

    tonearm = model.part("tonearm")
    tonearm.visual(
        Cylinder(radius=0.021, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=metal_mat,
        name="pivot_collar",
    )
    tonearm.visual(
        Cylinder(radius=0.006, length=0.210),
        origin=Origin(xyz=(-0.105, 0.0, 0.002), rpy=(0.0, pi / 2.0, 0.0)),
        material=arm_mat,
        name="arm_tube",
    )
    tonearm.visual(
        Cylinder(radius=0.017, length=0.034),
        origin=Origin(xyz=(0.029, 0.0, 0.002), rpy=(0.0, pi / 2.0, 0.0)),
        material=rubber_mat,
        name="counterweight",
    )
    tonearm.visual(
        Box((0.046, 0.030, 0.006)),
        origin=Origin(xyz=(-0.224, 0.0, 0.006)),
        material=cartridge_mat,
        name="headshell",
    )
    tonearm.visual(
        Box((0.006, 0.006, 0.018)),
        origin=Origin(xyz=(-0.244, 0.0, -0.006)),
        material=arm_mat,
        name="stylus",
    )

    pitch_fader = model.part("pitch_fader")
    pitch_fader.visual(
        Box((0.012, 0.020, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=metal_mat,
        name="slider_stem",
    )
    pitch_fader.visual(
        Box((0.050, 0.034, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.017)),
        material=fader_mat,
        name="fader_cap",
    )

    model.articulation(
        "body_to_platter",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=platter,
        origin=Origin(xyz=(-0.080, 0.030, 0.064)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=20.0),
    )
    model.articulation(
        "body_to_tonearm",
        ArticulationType.REVOLUTE,
        parent=body,
        child=tonearm,
        origin=Origin(xyz=(0.155, 0.115, 0.112)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.3, velocity=1.5, lower=-0.65, upper=0.35),
    )
    model.articulation(
        "body_to_pitch_fader",
        ArticulationType.PRISMATIC,
        parent=body,
        child=pitch_fader,
        origin=Origin(xyz=(0.175, -0.075, 0.068)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=0.35, lower=-0.060, upper=0.060),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    platter = object_model.get_part("platter")
    tonearm = object_model.get_part("tonearm")
    pitch_fader = object_model.get_part("pitch_fader")
    platter_joint = object_model.get_articulation("body_to_platter")
    tonearm_joint = object_model.get_articulation("body_to_tonearm")
    fader_joint = object_model.get_articulation("body_to_pitch_fader")

    ctx.check(
        "platter spins about a vertical axle",
        platter_joint.axis == (0.0, 0.0, 1.0),
        details=f"axis={platter_joint.axis}",
    )
    ctx.check(
        "tonearm uses a vertical revolute hinge",
        tonearm_joint.articulation_type == ArticulationType.REVOLUTE
        and tonearm_joint.axis == (0.0, 0.0, 1.0),
        details=f"type={tonearm_joint.articulation_type}, axis={tonearm_joint.axis}",
    )
    ctx.check(
        "pitch fader is a straight sliding control",
        fader_joint.articulation_type == ArticulationType.PRISMATIC
        and fader_joint.axis == (0.0, 1.0, 0.0),
        details=f"type={fader_joint.articulation_type}, axis={fader_joint.axis}",
    )

    ctx.expect_contact(
        platter,
        body,
        elem_a="metal_platter",
        elem_b="top_plate",
        name="platter rests on top plate",
    )
    ctx.expect_gap(
        tonearm,
        platter,
        axis="z",
        positive_elem="stylus",
        negative_elem="vinyl_record",
        max_gap=0.002,
        max_penetration=0.0,
        name="stylus sits just above record surface",
    )
    ctx.expect_contact(
        tonearm,
        body,
        elem_a="pivot_collar",
        elem_b="pivot_post",
        name="tonearm collar is seated on pivot post",
    )
    ctx.expect_contact(
        pitch_fader,
        body,
        elem_a="slider_stem",
        elem_b="fader_slot",
        name="fader stem rides on the slot",
    )
    ctx.expect_within(
        pitch_fader,
        body,
        axes="xy",
        inner_elem="slider_stem",
        outer_elem="fader_slot",
        margin=0.0,
        name="fader stem stays inside slot at center",
    )

    rest_pos = ctx.part_world_position(pitch_fader)
    with ctx.pose({fader_joint: 0.060}):
        ctx.expect_within(
            pitch_fader,
            body,
            axes="xy",
            inner_elem="slider_stem",
            outer_elem="fader_slot",
            margin=0.0,
            name="fader stem stays inside slot at high pitch",
        )
        high_pos = ctx.part_world_position(pitch_fader)
    with ctx.pose({tonearm_joint: -0.45}):
        ctx.expect_contact(
            tonearm,
            body,
            elem_a="pivot_collar",
            elem_b="pivot_post",
            name="tonearm remains seated while swung",
        )

    ctx.check(
        "pitch fader moves along slot",
        rest_pos is not None and high_pos is not None and high_pos[1] > rest_pos[1] + 0.045,
        details=f"rest={rest_pos}, high={high_pos}",
    )

    return ctx.report()


object_model = build_object_model()
