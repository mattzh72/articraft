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
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="industrial_multimeter")

    housing_gray = model.material("housing_gray", rgba=(0.24, 0.26, 0.29, 1.0))
    overmold_orange = model.material("overmold_orange", rgba=(0.93, 0.58, 0.16, 1.0))
    bezel_black = model.material("bezel_black", rgba=(0.10, 0.10, 0.11, 1.0))
    screen_dark = model.material("screen_dark", rgba=(0.22, 0.42, 0.30, 0.50))
    rubber_dark = model.material("rubber_dark", rgba=(0.14, 0.15, 0.16, 1.0))
    key_gray = model.material("key_gray", rgba=(0.48, 0.50, 0.53, 1.0))
    key_blue = model.material("key_blue", rgba=(0.16, 0.38, 0.74, 1.0))
    jack_red = model.material("jack_red", rgba=(0.75, 0.14, 0.12, 1.0))
    jack_black = model.material("jack_black", rgba=(0.09, 0.09, 0.10, 1.0))

    dial_ring_mesh = mesh_from_cadquery(
        cq.Workplane("XY").circle(0.026).circle(0.009).extrude(0.006),
        "dial_ring",
    )
    dial_inner_ring_mesh = mesh_from_cadquery(
        cq.Workplane("XY").circle(0.019).circle(0.008).extrude(0.006),
        "dial_inner_ring",
    )

    body = model.part("body")

    def add_body_visual(
        geometry,
        *,
        xyz: tuple[float, float, float] = (0.0, 0.0, 0.0),
        rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
        material=None,
        name: str,
    ) -> None:
        body.visual(
            geometry,
            origin=Origin(xyz=xyz, rpy=rpy),
            material=material,
            name=name,
        )

    add_body_visual(
        Box((0.072, 0.158, 0.030)),
        xyz=(0.0, 0.0, -0.004),
        material=housing_gray,
        name="main_shell",
    )
    add_body_visual(
        Box((0.066, 0.150, 0.012)),
        xyz=(0.0, 0.0, -0.016),
        material=bezel_black,
        name="back_shell",
    )

    add_body_visual(
        Box((0.012, 0.172, 0.038)),
        xyz=(-0.039, 0.0, -0.002),
        material=overmold_orange,
        name="side_guard_0",
    )
    add_body_visual(
        Box((0.012, 0.172, 0.038)),
        xyz=(0.039, 0.0, -0.002),
        material=overmold_orange,
        name="side_guard_1",
    )
    add_body_visual(
        Box((0.060, 0.018, 0.038)),
        xyz=(0.0, 0.082, -0.002),
        material=overmold_orange,
        name="top_guard",
    )
    add_body_visual(
        Box((0.060, 0.018, 0.038)),
        xyz=(0.0, -0.082, -0.002),
        material=overmold_orange,
        name="bottom_guard",
    )
    for index, (x_pos, y_pos) in enumerate(
        (
            (-0.033, 0.078),
            (0.033, 0.078),
            (-0.033, -0.078),
            (0.033, -0.078),
        )
    ):
        add_body_visual(
            Sphere(radius=0.012),
            xyz=(x_pos, y_pos, -0.001),
            material=overmold_orange,
            name=f"corner_guard_{index}",
        )

    add_body_visual(
        Box((0.064, 0.058, 0.006)),
        xyz=(0.0, 0.050, 0.014),
        material=bezel_black,
        name="upper_bezel",
    )
    add_body_visual(
        Box((0.008, 0.038, 0.006)),
        xyz=(-0.030, 0.050, 0.014),
        material=housing_gray,
        name="screen_side_0",
    )
    add_body_visual(
        Box((0.008, 0.038, 0.006)),
        xyz=(0.030, 0.050, 0.014),
        material=housing_gray,
        name="screen_side_1",
    )
    add_body_visual(
        Box((0.052, 0.038, 0.003)),
        xyz=(0.0, 0.050, 0.0155),
        material=bezel_black,
        name="screen_recess",
    )
    add_body_visual(
        Box((0.046, 0.032, 0.002)),
        xyz=(0.0, 0.050, 0.017),
        material=screen_dark,
        name="display",
    )

    add_body_visual(
        Box((0.012, 0.076, 0.006)),
        xyz=(-0.027, -0.004, 0.014),
        material=housing_gray,
        name="dial_side_0",
    )
    add_body_visual(
        Box((0.012, 0.076, 0.006)),
        xyz=(0.027, -0.004, 0.014),
        material=housing_gray,
        name="dial_side_1",
    )
    add_body_visual(
        Box((0.060, 0.018, 0.006)),
        xyz=(0.0, -0.040, 0.014),
        material=housing_gray,
        name="lower_bridge",
    )
    add_body_visual(
        dial_ring_mesh,
        xyz=(0.0, 0.001, 0.011),
        material=bezel_black,
        name="dial_ring",
    )
    add_body_visual(
        dial_inner_ring_mesh,
        xyz=(0.0, 0.001, 0.011),
        material=housing_gray,
        name="dial_inner_ring",
    )

    add_body_visual(
        Box((0.068, 0.024, 0.006)),
        xyz=(0.0, -0.076, 0.014),
        material=housing_gray,
        name="lower_panel",
    )

    add_body_visual(
        Box((0.062, 0.0025, 0.006)),
        xyz=(0.0, -0.0415, 0.014),
        material=bezel_black,
        name="button_frame_top",
    )
    add_body_visual(
        Box((0.062, 0.0025, 0.006)),
        xyz=(0.0, -0.0545, 0.014),
        material=bezel_black,
        name="button_frame_bottom",
    )
    for index, x_pos in enumerate((-0.031, -0.010, 0.010, 0.031)):
        add_body_visual(
            Box((0.003, 0.015, 0.006)),
            xyz=(x_pos, -0.048, 0.014),
            material=bezel_black,
            name=f"button_divider_{index}",
        )

    add_body_visual(
        Cylinder(radius=0.008, length=0.004),
        xyz=(0.028, 0.071, 0.015),
        material=bezel_black,
        name="flashlight_ring",
    )

    jack_positions = (-0.024, 0.0, 0.024)
    jack_materials = (jack_black, jack_black, jack_red)
    for index, (x_pos, material) in enumerate(zip(jack_positions, jack_materials)):
        add_body_visual(
            Cylinder(radius=0.0075, length=0.006),
            xyz=(x_pos, -0.076, 0.014),
            material=bezel_black,
            name=f"jack_collar_{index}",
        )
        add_body_visual(
            Cylinder(radius=0.0048, length=0.003),
            xyz=(x_pos, -0.076, 0.0175),
            material=material,
            name=f"jack_insert_{index}",
        )

    for index, x_pos in enumerate((-0.027, 0.027)):
        add_body_visual(
            Box((0.018, 0.010, 0.006)),
            xyz=(x_pos, -0.074, -0.016),
            material=housing_gray,
            name=f"support_pivot_cheek_{index}",
        )

    selector_knob = model.part("selector_knob")
    selector_mesh = mesh_from_geometry(
        KnobGeometry(
            0.038,
            0.017,
            body_style="skirted",
            top_diameter=0.034,
            skirt=KnobSkirt(0.046, 0.004, flare=0.08),
            grip=KnobGrip(style="knurled", count=40, depth=0.0010, helix_angle_deg=18.0),
            indicator=KnobIndicator(style="line", mode="engraved", depth=0.0008),
            center=False,
        ),
        "selector_knob",
    )
    selector_knob.visual(
        selector_mesh,
        material=rubber_dark,
        name="knob_cap",
    )
    selector_knob.visual(
        Cylinder(radius=0.006, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, -0.004)),
        material=bezel_black,
        name="knob_stem",
    )

    model.articulation(
        "body_to_selector_knob",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=selector_knob,
        origin=Origin(xyz=(0.0, 0.001, 0.017)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.15, velocity=6.0),
    )

    rear_support = model.part("rear_support")
    for index, x_pos in enumerate((-0.024, 0.024)):
        rear_support.visual(
            Box((0.010, 0.126, 0.004)),
            origin=Origin(xyz=(x_pos, 0.065, -0.002)),
            material=rubber_dark,
            name=f"support_rail_{index}",
        )
    rear_support.visual(
        Box((0.048, 0.006, 0.004)),
        origin=Origin(xyz=(0.0, 0.003, -0.002)),
        material=rubber_dark,
        name="support_lower_bridge",
    )
    rear_support.visual(
        Box((0.058, 0.010, 0.004)),
        origin=Origin(xyz=(0.0, 0.125, -0.002)),
        material=rubber_dark,
        name="support_top_bridge",
    )
    rear_support.visual(
        Box((0.040, 0.008, 0.004)),
        origin=Origin(xyz=(0.0, 0.066, -0.002)),
        material=rubber_dark,
        name="support_crossbar",
    )
    rear_support.visual(
        Box((0.058, 0.010, 0.008)),
        origin=Origin(xyz=(0.0, 0.127, -0.004)),
        material=rubber_dark,
        name="support_foot",
    )
    for index, x_pos in enumerate((-0.027, 0.027)):
        rear_support.visual(
            Cylinder(radius=0.0045, length=0.012),
            origin=Origin(
                xyz=(x_pos, 0.0, -0.002),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=rubber_dark,
            name=f"pivot_barrel_{index}",
        )

    model.articulation(
        "body_to_rear_support",
        ArticulationType.REVOLUTE,
        parent=body,
        child=rear_support,
        origin=Origin(xyz=(0.0, -0.076, -0.022)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(68.0),
        ),
    )

    def add_function_key(
        name: str,
        joint_name: str,
        *,
        x_pos: float,
        material,
    ) -> None:
        key = model.part(name)
        key.visual(
            Box((0.016, 0.009, 0.0036)),
            origin=Origin(xyz=(0.0, 0.0, 0.0018)),
            material=material,
            name="key_cap",
        )
        key.visual(
            Box((0.010, 0.005, 0.0045)),
            origin=Origin(xyz=(0.0, 0.0, -0.0017)),
            material=bezel_black,
            name="key_stem",
        )
        model.articulation(
            joint_name,
            ArticulationType.PRISMATIC,
            parent=body,
            child=key,
            origin=Origin(xyz=(x_pos, -0.048, 0.017)),
            axis=(0.0, 0.0, -1.0),
            motion_limits=MotionLimits(
                effort=2.0,
                velocity=0.08,
                lower=0.0,
                upper=0.0012,
            ),
        )

    add_function_key(
        "function_key_0",
        "body_to_function_key_0",
        x_pos=-0.0205,
        material=key_gray,
    )
    add_function_key(
        "function_key_1",
        "body_to_function_key_1",
        x_pos=0.0,
        material=key_blue,
    )
    add_function_key(
        "function_key_2",
        "body_to_function_key_2",
        x_pos=0.0205,
        material=key_gray,
    )

    flashlight_button = model.part("flashlight_button")
    flashlight_button.visual(
        Cylinder(radius=0.0055, length=0.0034),
        origin=Origin(xyz=(0.0, 0.0, 0.0017)),
        material=key_blue,
        name="button_cap",
    )
    flashlight_button.visual(
        Cylinder(radius=0.0038, length=0.005),
        origin=Origin(xyz=(0.0, 0.0, -0.0015)),
        material=bezel_black,
        name="button_stem",
    )
    model.articulation(
        "body_to_flashlight_button",
        ArticulationType.PRISMATIC,
        parent=body,
        child=flashlight_button,
        origin=Origin(xyz=(0.028, 0.071, 0.017)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=0.08,
            lower=0.0,
            upper=0.0010,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    selector_knob = object_model.get_part("selector_knob")
    rear_support = object_model.get_part("rear_support")
    flashlight_button = object_model.get_part("flashlight_button")
    function_key_0 = object_model.get_part("function_key_0")
    function_key_1 = object_model.get_part("function_key_1")

    selector_joint = object_model.get_articulation("body_to_selector_knob")
    rear_support_joint = object_model.get_articulation("body_to_rear_support")
    flashlight_joint = object_model.get_articulation("body_to_flashlight_button")
    function_key_0_joint = object_model.get_articulation("body_to_function_key_0")

    ctx.expect_gap(
        selector_knob,
        body,
        axis="z",
        positive_elem="knob_cap",
        negative_elem="dial_ring",
        max_gap=0.0008,
        max_penetration=0.0,
        name="selector knob seats on dial ring",
    )
    ctx.expect_gap(
        flashlight_button,
        body,
        axis="z",
        positive_elem="button_cap",
        negative_elem="flashlight_ring",
        max_gap=0.0008,
        max_penetration=0.0,
        name="flashlight button sits proud of the front face",
    )
    ctx.expect_gap(
        body,
        rear_support,
        axis="z",
        positive_elem="back_shell",
        negative_elem="support_lower_bridge",
        max_gap=0.001,
        max_penetration=0.0,
        name="rear support nests against the back shell when closed",
    )

    selector_rest = ctx.part_world_position(selector_knob)
    with ctx.pose({selector_joint: math.pi / 2.0}):
        selector_rotated = ctx.part_world_position(selector_knob)
    ctx.check(
        "selector rotates about its own axis",
        selector_rest is not None
        and selector_rotated is not None
        and max(abs(a - b) for a, b in zip(selector_rest, selector_rotated)) < 1e-6,
        details=f"rest={selector_rest}, rotated={selector_rotated}",
    )

    key_rest = ctx.part_world_aabb(function_key_0)
    other_key_rest = ctx.part_world_aabb(function_key_1)
    key_upper = function_key_0_joint.motion_limits.upper if function_key_0_joint.motion_limits is not None else None
    if key_upper is not None:
        with ctx.pose({function_key_0_joint: key_upper}):
            key_pressed = ctx.part_world_aabb(function_key_0)
            other_key_pressed = ctx.part_world_aabb(function_key_1)
        ctx.check(
            "function key depresses inward",
            key_rest is not None
            and key_pressed is not None
            and key_pressed[1][2] < key_rest[1][2] - 0.001,
            details=f"rest={key_rest}, pressed={key_pressed}",
        )
        ctx.check(
            "function keys travel independently",
            other_key_rest is not None
            and other_key_pressed is not None
            and max(abs(a - b) for a, b in zip(other_key_rest[0], other_key_pressed[0])) < 1e-6
            and max(abs(a - b) for a, b in zip(other_key_rest[1], other_key_pressed[1])) < 1e-6,
            details=f"rest={other_key_rest}, pressed_pose={other_key_pressed}",
        )

    flashlight_rest = ctx.part_world_aabb(flashlight_button)
    flashlight_upper = flashlight_joint.motion_limits.upper if flashlight_joint.motion_limits is not None else None
    if flashlight_upper is not None:
        with ctx.pose({flashlight_joint: flashlight_upper}):
            flashlight_pressed = ctx.part_world_aabb(flashlight_button)
        ctx.check(
            "flashlight button depresses inward",
            flashlight_rest is not None
            and flashlight_pressed is not None
            and flashlight_pressed[1][2] < flashlight_rest[1][2] - 0.0007,
            details=f"rest={flashlight_rest}, pressed={flashlight_pressed}",
        )

    support_closed = ctx.part_element_world_aabb(rear_support, elem="support_foot")
    support_upper = rear_support_joint.motion_limits.upper if rear_support_joint.motion_limits is not None else None
    if support_upper is not None:
        with ctx.pose({rear_support_joint: support_upper}):
            support_open = ctx.part_element_world_aabb(rear_support, elem="support_foot")
        ctx.check(
            "rear support folds out behind the meter",
            support_closed is not None
            and support_open is not None
            and support_open[0][2] < support_closed[0][2] - 0.040,
            details=f"closed={support_closed}, open={support_open}",
        )

    return ctx.report()


object_model = build_object_model()
