from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _build_base_body() -> cq.Workplane:
    foot = cq.Workplane("XY").box(0.230, 0.180, 0.045, centered=(True, True, False))
    tower = (
        cq.Workplane("XY")
        .box(0.128, 0.106, 0.170, centered=(True, True, False))
        .translate((0.0, 0.0, 0.035))
    )
    collar = cq.Workplane("XY").circle(0.068).extrude(0.012).translate((0.0, 0.0, 0.205))
    return foot.union(tower).union(collar)


def _build_bowl_shell() -> cq.Workplane:
    outer = cq.Workplane("XY").circle(0.072).extrude(0.244)
    cavity = cq.Workplane("XY").circle(0.066).extrude(0.230).translate((0.0, 0.0, 0.014))
    socket = cq.Workplane("XY").circle(0.013).extrude(0.010).translate((0.0, 0.0, 0.004))
    rim = cq.Workplane("XY").circle(0.076).circle(0.066).extrude(0.010).translate((0.0, 0.0, 0.234))
    return outer.cut(cavity).cut(socket).union(rim)


def _build_lid_shell() -> cq.Workplane:
    outer_rim = cq.Workplane("XY").circle(0.078).circle(0.060).extrude(0.011)
    top_skin = cq.Workplane("XY").circle(0.078).extrude(0.003).translate((0.0, 0.0, 0.011))
    chimney = cq.Workplane("XY").center(0.0, 0.028).circle(0.046).extrude(0.114)
    chimney_hole = (
        cq.Workplane("XY")
        .center(0.0, 0.028)
        .circle(0.038)
        .extrude(0.128)
        .translate((0.0, 0.0, -0.008))
    )
    return outer_rim.union(top_skin).union(chimney).cut(chimney_hole)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="upright_food_processor")

    body_white = model.material("body_white", rgba=(0.94, 0.94, 0.92, 1.0))
    strip_black = model.material("strip_black", rgba=(0.10, 0.11, 0.12, 1.0))
    trim_silver = model.material("trim_silver", rgba=(0.78, 0.79, 0.80, 1.0))
    smoke_clear = model.material("smoke_clear", rgba=(0.70, 0.78, 0.83, 0.36))
    lid_clear = model.material("lid_clear", rgba=(0.82, 0.87, 0.90, 0.30))
    knob_black = model.material("knob_black", rgba=(0.08, 0.08, 0.09, 1.0))
    button_black = model.material("button_black", rgba=(0.14, 0.14, 0.15, 1.0))
    pusher_clear = model.material("pusher_clear", rgba=(0.90, 0.92, 0.93, 0.62))
    steel = model.material("steel", rgba=(0.79, 0.80, 0.82, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_build_base_body(), "food_processor_base"),
        material=body_white,
        name="body_shell",
    )
    base.visual(
        Box((0.082, 0.008, 0.152)),
        origin=Origin(xyz=(0.0, 0.056, 0.120)),
        material=strip_black,
        name="control_strip",
    )
    base.visual(
        Cylinder(radius=0.060, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.220)),
        material=trim_silver,
        name="bowl_seat",
    )

    bowl = model.part("bowl")
    bowl.visual(
        mesh_from_cadquery(_build_bowl_shell(), "food_processor_bowl"),
        material=smoke_clear,
        name="bowl_shell",
    )

    lid = model.part("lid")
    lid.visual(
        mesh_from_cadquery(_build_lid_shell(), "food_processor_lid"),
        material=lid_clear,
        name="lid_shell",
    )

    spindle = model.part("spindle")
    spindle.visual(
        Cylinder(radius=0.011, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=steel,
        name="foot",
    )
    spindle.visual(
        Cylinder(radius=0.017, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=steel,
        name="hub",
    )
    spindle.visual(
        Cylinder(radius=0.009, length=0.108),
        origin=Origin(xyz=(0.0, 0.0, 0.066)),
        material=steel,
        name="shaft",
    )
    spindle.visual(
        Box((0.094, 0.016, 0.0035)),
        origin=Origin(xyz=(0.0, 0.0, 0.070), rpy=(0.0, 0.18, 0.38)),
        material=steel,
        name="blade_bar",
    )
    spindle.visual(
        Box((0.074, 0.013, 0.0035)),
        origin=Origin(xyz=(0.0, 0.0, 0.080), rpy=(0.0, -0.16, -1.02)),
        material=steel,
        name="blade_fin",
    )

    pusher = model.part("pusher")
    pusher.visual(
        Cylinder(radius=0.034, length=0.122),
        origin=Origin(xyz=(0.0, 0.0, 0.061)),
        material=pusher_clear,
        name="pusher_body",
    )
    pusher.visual(
        Cylinder(radius=0.051, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.129)),
        material=pusher_clear,
        name="pusher_cap",
    )
    pusher.visual(
        Cylinder(radius=0.022, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.141)),
        material=trim_silver,
        name="pusher_grip",
    )

    dial = model.part("dial")
    dial.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.044,
                0.022,
                body_style="skirted",
                top_diameter=0.034,
                skirt=KnobSkirt(0.056, 0.006, flare=0.06),
                grip=KnobGrip(style="fluted", count=18, depth=0.0012),
                indicator=KnobIndicator(style="line", mode="engraved", depth=0.0007, angle_deg=0.0),
                center=False,
            ),
            "food_processor_dial",
        ),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=knob_black,
        name="dial_shell",
    )

    for index, x_pos in enumerate((-0.022, 0.022)):
        button = model.part(f"button_{index}")
        button.visual(
            Box((0.032, 0.012, 0.018)),
            origin=Origin(xyz=(0.0, 0.002, 0.0)),
            material=button_black,
            name="rocker_cap",
        )
        button.visual(
            Cylinder(radius=0.0025, length=0.028),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=button_black,
            name="pivot_barrel",
        )
        model.articulation(
            f"base_to_button_{index}",
            ArticulationType.REVOLUTE,
            parent=base,
            child=button,
            origin=Origin(xyz=(x_pos, 0.064, 0.096)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=1.0,
                velocity=2.5,
                lower=-0.18,
                upper=0.18,
            ),
        )

    model.articulation(
        "base_to_bowl",
        ArticulationType.FIXED,
        parent=base,
        child=bowl,
        origin=Origin(xyz=(0.0, 0.0, 0.223)),
    )
    model.articulation(
        "bowl_to_lid",
        ArticulationType.FIXED,
        parent=bowl,
        child=lid,
        origin=Origin(xyz=(0.0, 0.0, 0.244)),
    )
    model.articulation(
        "bowl_to_spindle",
        ArticulationType.CONTINUOUS,
        parent=bowl,
        child=spindle,
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=18.0),
    )
    model.articulation(
        "lid_to_pusher",
        ArticulationType.PRISMATIC,
        parent=lid,
        child=pusher,
        origin=Origin(xyz=(0.0, 0.028, -0.008)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=0.25,
            lower=0.0,
            upper=0.065,
        ),
    )
    model.articulation(
        "base_to_dial",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=dial,
        origin=Origin(xyz=(0.0, 0.060, 0.156)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.5, velocity=6.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    bowl = object_model.get_part("bowl")
    lid = object_model.get_part("lid")
    spindle = object_model.get_part("spindle")
    pusher = object_model.get_part("pusher")
    dial = object_model.get_part("dial")
    button_0 = object_model.get_part("button_0")
    button_1 = object_model.get_part("button_1")

    spindle_joint = object_model.get_articulation("bowl_to_spindle")
    pusher_joint = object_model.get_articulation("lid_to_pusher")
    dial_joint = object_model.get_articulation("base_to_dial")
    button_joint_0 = object_model.get_articulation("base_to_button_0")
    button_joint_1 = object_model.get_articulation("base_to_button_1")

    ctx.allow_overlap(
        bowl,
        spindle,
        elem_a="bowl_shell",
        elem_b="foot",
        reason="The removable spindle foot is intentionally seated into the bowl-floor drive socket.",
    )

    ctx.expect_gap(
        bowl,
        base,
        axis="z",
        positive_elem="bowl_shell",
        negative_elem="bowl_seat",
        max_gap=0.004,
        max_penetration=0.0,
        name="bowl seats on pedestal base",
    )
    ctx.expect_overlap(
        lid,
        bowl,
        axes="xy",
        elem_a="lid_shell",
        elem_b="bowl_shell",
        min_overlap=0.135,
        name="lid covers bowl opening",
    )
    ctx.expect_origin_distance(
        spindle,
        bowl,
        axes="xy",
        max_dist=0.001,
        name="spindle stays on bowl centerline",
    )
    ctx.expect_gap(
        dial,
        base,
        axis="y",
        positive_elem="dial_shell",
        negative_elem="control_strip",
        max_gap=0.002,
        max_penetration=0.0,
        name="dial mounts flush on control strip",
    )
    ctx.expect_overlap(
        dial,
        base,
        axes="xz",
        elem_a="dial_shell",
        elem_b="control_strip",
        min_overlap=0.024,
        name="dial stays centered on control strip",
    )

    for index, button in enumerate((button_0, button_1)):
        ctx.expect_gap(
            button,
            base,
            axis="y",
            positive_elem="rocker_cap",
            negative_elem="control_strip",
            max_gap=0.002,
            max_penetration=0.0,
            name=f"button_{index} sits on control strip",
        )
        ctx.expect_overlap(
            button,
            base,
            axes="xz",
            elem_a="rocker_cap",
            elem_b="control_strip",
            min_overlap=0.016,
            name=f"button_{index} is framed by the control strip",
        )

    pusher_limits = pusher_joint.motion_limits
    rest_pos = None
    raised_pos = None
    if pusher_limits is not None and pusher_limits.upper is not None:
        with ctx.pose({pusher_joint: 0.0}):
            ctx.expect_gap(
                pusher,
                lid,
                axis="z",
                positive_elem="pusher_cap",
                negative_elem="lid_shell",
                min_gap=0.0,
                max_gap=0.002,
                name="inserted pusher cap seats on chimney rim",
            )
            ctx.expect_overlap(
                pusher,
                lid,
                axes="z",
                elem_a="pusher_body",
                elem_b="lid_shell",
                min_overlap=0.050,
                name="inserted pusher remains deep in the chimney",
            )
            rest_pos = ctx.part_world_position(pusher)

        with ctx.pose({pusher_joint: pusher_limits.upper}):
            ctx.expect_gap(
                pusher,
                lid,
                axis="z",
                positive_elem="pusher_cap",
                negative_elem="lid_shell",
                min_gap=0.056,
                name="raised pusher cap clears the chimney",
            )
            ctx.expect_overlap(
                pusher,
                lid,
                axes="z",
                elem_a="pusher_body",
                elem_b="lid_shell",
                min_overlap=0.028,
                name="raised pusher keeps retained insertion",
            )
            raised_pos = ctx.part_world_position(pusher)

    ctx.check(
        "pusher moves upward along chimney axis",
        rest_pos is not None and raised_pos is not None and raised_pos[2] > rest_pos[2] + 0.05,
        details=f"rest={rest_pos}, raised={raised_pos}",
    )
    ctx.check(
        "spindle uses continuous rotation",
        spindle_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={spindle_joint.articulation_type}",
    )
    ctx.check(
        "dial uses continuous rotation",
        dial_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={dial_joint.articulation_type}",
    )

    for joint in (button_joint_0, button_joint_1):
        limits = joint.motion_limits
        ctx.check(
            f"{joint.name} rocks both directions",
            limits is not None
            and limits.lower is not None
            and limits.upper is not None
            and limits.lower < 0.0 < limits.upper,
            details=f"limits={limits}",
        )

    return ctx.report()


object_model = build_object_model()
