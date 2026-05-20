from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

# Advanced; only use CadQuery if the native sdk is not enough to represent the shapes you want:
# import cadquery as cq
# from sdk import mesh_from_cadquery


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="commercial_contact_grill")

    model.material("stainless", color=(0.72, 0.73, 0.75))
    model.material("dark_steel", color=(0.28, 0.30, 0.32))
    model.material("black", color=(0.10, 0.10, 0.11))
    model.material("rubber", color=(0.08, 0.08, 0.08))
    model.material("button_orange", color=(0.86, 0.42, 0.08))
    model.material("button_red", color=(0.72, 0.14, 0.10))

    browning_knob = mesh_from_geometry(
        KnobGeometry(
            0.036,
            0.020,
            body_style="skirted",
            top_diameter=0.030,
            base_diameter=0.040,
            center=False,
        ),
        "browning_knob",
    )

    base = model.part("base")
    base.visual(
        Box((0.52, 0.44, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, 0.085)),
        material="stainless",
        name="lower_shell",
    )
    base.visual(
        Box((0.50, 0.42, 0.03)),
        origin=Origin(xyz=(0.0, 0.0, 0.150)),
        material="dark_steel",
        name="deck",
    )
    base.visual(
        Box((0.42, 0.33, 0.028)),
        origin=Origin(xyz=(0.015, 0.0, 0.161)),
        material="black",
        name="lower_platen",
    )
    base.visual(
        Box((0.08, 0.44, 0.05)),
        origin=Origin(xyz=(0.22, 0.0, 0.085)),
        material="dark_steel",
        name="front_fascia",
    )
    base.visual(
        Cylinder(radius=0.028, length=0.012),
        origin=Origin(xyz=(0.120, 0.226, 0.122), rpy=(-pi / 2.0, 0.0, 0.0)),
        material="dark_steel",
        name="knob_boss",
    )
    base.visual(
        Box((0.006, 0.085, 0.065)),
        origin=Origin(xyz=(0.257, 0.118, 0.109)),
        material="dark_steel",
        name="control_pod",
    )
    base.visual(
        Box((0.004, 0.032, 0.028)),
        origin=Origin(xyz=(0.260, 0.095, 0.109)),
        material="black",
        name="button_seat_0",
    )
    base.visual(
        Box((0.004, 0.032, 0.028)),
        origin=Origin(xyz=(0.260, 0.141, 0.109)),
        material="black",
        name="button_seat_1",
    )
    base.visual(
        Box((0.035, 0.04, 0.11)),
        origin=Origin(xyz=(-0.225, -0.180, 0.190)),
        material="dark_steel",
        name="hinge_tower_0",
    )
    base.visual(
        Box((0.035, 0.04, 0.11)),
        origin=Origin(xyz=(-0.225, 0.180, 0.190)),
        material="dark_steel",
        name="hinge_tower_1",
    )
    base.visual(
        Box((0.03, 0.36, 0.07)),
        origin=Origin(xyz=(-0.245, 0.0, 0.170)),
        material="dark_steel",
        name="hinge_bridge",
    )
    for index, y in enumerate((-0.17, 0.17)):
        for x in (-0.19, 0.19):
            base.visual(
                Cylinder(radius=0.018, length=0.025),
                origin=Origin(xyz=(x, y, 0.0125)),
                material="rubber",
                name=f"foot_{index}_{0 if x < 0 else 1}",
            )

    top = model.part("top_platen")
    top.visual(
        Box((0.48, 0.43, 0.045)),
        origin=Origin(xyz=(0.255, 0.0, 0.025)),
        material="stainless",
        name="upper_shell",
    )
    top.visual(
        Box((0.42, 0.33, 0.025)),
        origin=Origin(xyz=(0.24, 0.0, -0.0235)),
        material="black",
        name="upper_platen",
    )
    top.visual(
        Box((0.44, 0.02, 0.07)),
        origin=Origin(xyz=(0.23, -0.205, -0.005)),
        material="dark_steel",
        name="side_skirt_0",
    )
    top.visual(
        Box((0.44, 0.02, 0.07)),
        origin=Origin(xyz=(0.23, 0.205, -0.005)),
        material="dark_steel",
        name="side_skirt_1",
    )
    top.visual(
        Box((0.03, 0.39, 0.075)),
        origin=Origin(xyz=(0.040, 0.0, -0.0025)),
        material="dark_steel",
        name="rear_spine",
    )
    top.visual(
        Box((0.04, 0.03, 0.05)),
        origin=Origin(xyz=(0.040, -0.180, -0.015)),
        material="dark_steel",
        name="hinge_arm_0",
    )
    top.visual(
        Box((0.04, 0.03, 0.05)),
        origin=Origin(xyz=(0.040, 0.180, -0.015)),
        material="dark_steel",
        name="hinge_arm_1",
    )
    top.visual(
        Box((0.025, 0.06, 0.045)),
        origin=Origin(xyz=(0.455, -0.145, -0.004)),
        material="dark_steel",
        name="handle_bracket_0",
    )
    top.visual(
        Box((0.025, 0.06, 0.045)),
        origin=Origin(xyz=(0.455, 0.145, -0.004)),
        material="dark_steel",
        name="handle_bracket_1",
    )
    top.visual(
        Cylinder(radius=0.022, length=0.26),
        origin=Origin(xyz=(0.470, 0.0, -0.018), rpy=(pi / 2.0, 0.0, 0.0)),
        material="black",
        name="front_handle",
    )

    model.articulation(
        "base_to_top_platen",
        ArticulationType.REVOLUTE,
        parent=base,
        child=top,
        origin=Origin(xyz=(-0.220, 0.0, 0.215)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=180.0, velocity=1.5, lower=0.0, upper=1.35),
    )

    knob = model.part("side_knob")
    knob.visual(
        browning_knob,
        origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
        material="black",
        name="knob_body",
    )
    knob.visual(
        Cylinder(radius=0.006, length=0.010),
        origin=Origin(xyz=(0.0, 0.005, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material="dark_steel",
        name="knob_shaft",
    )
    knob.visual(
        Box((0.004, 0.004, 0.012)),
        origin=Origin(xyz=(0.0, 0.017, 0.012)),
        material="stainless",
        name="knob_pointer",
    )
    model.articulation(
        "base_to_side_knob",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=knob,
        origin=Origin(xyz=(0.120, 0.232, 0.122)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=8.0),
    )

    for index, (y, material) in enumerate(((0.095, "button_orange"), (0.141, "button_red"))):
        button = model.part(f"program_button_{index}")
        button.visual(
            Box((0.008, 0.026, 0.022)),
            origin=Origin(),
            material=material,
            name="button_cap",
        )
        button.visual(
            Box((0.002, 0.012, 0.012)),
            origin=Origin(xyz=(-0.005, 0.0, 0.0)),
            material="black",
            name="button_stem",
        )
        model.articulation(
            f"base_to_program_button_{index}",
            ArticulationType.PRISMATIC,
            parent=base,
            child=button,
            origin=Origin(xyz=(0.268, y, 0.109)),
            axis=(-1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=10.0, velocity=0.04, lower=0.0, upper=0.0025),
        )

    return model


def run_tests() -> TestReport:
    def aabb_center(aabb):
        return tuple((lo + hi) * 0.5 for lo, hi in zip(aabb[0], aabb[1]))

    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    top = object_model.get_part("top_platen")
    knob = object_model.get_part("side_knob")
    button_0 = object_model.get_part("program_button_0")
    button_1 = object_model.get_part("program_button_1")
    top_hinge = object_model.get_articulation("base_to_top_platen")
    knob_joint = object_model.get_articulation("base_to_side_knob")
    button_joint_0 = object_model.get_articulation("base_to_program_button_0")
    button_joint_1 = object_model.get_articulation("base_to_program_button_1")

    with ctx.pose({top_hinge: 0.0}):
        ctx.expect_gap(
            top,
            base,
            axis="z",
            positive_elem="upper_platen",
            negative_elem="lower_platen",
            min_gap=0.003,
            max_gap=0.008,
            name="closed platens stay nearly touching",
        )
        ctx.expect_overlap(
            top,
            base,
            axes="xy",
            elem_a="upper_platen",
            elem_b="lower_platen",
            min_overlap=0.28,
            name="closed platens cover the cooking area",
        )

    closed_platen = ctx.part_element_world_aabb(top, elem="upper_platen")
    with ctx.pose({top_hinge: 1.2}):
        ctx.expect_gap(
            top,
            base,
            axis="z",
            positive_elem="front_handle",
            negative_elem="deck",
            min_gap=0.30,
            name="open handle lifts well above the lower frame",
        )
        open_platen = ctx.part_element_world_aabb(top, elem="upper_platen")

    ctx.check(
        "top platen opens upward",
        closed_platen is not None
        and open_platen is not None
        and open_platen[1][2] > closed_platen[1][2] + 0.12,
        details=f"closed={closed_platen}, open={open_platen}",
    )

    ctx.expect_gap(
        button_0,
        base,
        axis="x",
        positive_elem="button_cap",
        negative_elem="button_seat_0",
        min_gap=0.001,
        max_gap=0.006,
        name="program button 0 stands proud of its seat",
    )
    ctx.expect_gap(
        button_1,
        base,
        axis="x",
        positive_elem="button_cap",
        negative_elem="button_seat_1",
        min_gap=0.001,
        max_gap=0.006,
        name="program button 1 stands proud of its seat",
    )

    knob_pointer_rest = ctx.part_element_world_aabb(knob, elem="knob_pointer")
    with ctx.pose({knob_joint: pi / 2.0}):
        knob_pointer_turned = ctx.part_element_world_aabb(knob, elem="knob_pointer")

    ctx.check(
        "side knob rotates on its shaft",
        knob_pointer_rest is not None
        and knob_pointer_turned is not None
        and aabb_center(knob_pointer_turned)[0] > aabb_center(knob_pointer_rest)[0] + 0.008,
        details=f"rest={knob_pointer_rest}, turned={knob_pointer_turned}",
    )

    button_0_rest = ctx.part_element_world_aabb(button_0, elem="button_cap")
    button_1_rest = ctx.part_element_world_aabb(button_1, elem="button_cap")
    with ctx.pose({button_joint_0: 0.0025}):
        button_0_pressed = ctx.part_element_world_aabb(button_0, elem="button_cap")
        button_1_during_press = ctx.part_element_world_aabb(button_1, elem="button_cap")

    ctx.check(
        "program button 0 depresses downward",
        button_0_rest is not None
        and button_0_pressed is not None
        and button_0_pressed[0][0] < button_0_rest[0][0] - 0.0015,
        details=f"rest={button_0_rest}, pressed={button_0_pressed}",
    )
    ctx.check(
        "program buttons articulate independently",
        button_1_rest is not None and button_1_during_press == button_1_rest,
        details=f"rest={button_1_rest}, during_press={button_1_during_press}",
    )

    with ctx.pose({button_joint_1: 0.0025}):
        button_1_pressed = ctx.part_element_world_aabb(button_1, elem="button_cap")

    ctx.check(
        "program button 1 depresses inward",
        button_1_rest is not None
        and button_1_pressed is not None
        and button_1_pressed[0][0] < button_1_rest[0][0] - 0.0015,
        details=f"rest={button_1_rest}, pressed={button_1_pressed}",
    )

    return ctx.report()


object_model = build_object_model()
