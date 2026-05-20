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
    model = ArticulatedObject(name="utility_pickup_tailgate")

    body_blue = model.material("painted_utility_blue", rgba=(0.05, 0.18, 0.33, 1.0))
    dark_blue = model.material("shadowed_pressed_blue", rgba=(0.035, 0.12, 0.23, 1.0))
    bed_black = model.material("black_bedliner", rgba=(0.015, 0.016, 0.016, 1.0))
    satin_black = model.material("satin_black_plastic", rgba=(0.005, 0.005, 0.004, 1.0))
    hinge_steel = model.material("galvanized_hinge_steel", rgba=(0.55, 0.56, 0.53, 1.0))
    bumper_grey = model.material("dark_step_bumper", rgba=(0.12, 0.12, 0.11, 1.0))
    reflector_red = model.material("red_reflector_lens", rgba=(0.8, 0.05, 0.025, 1.0))

    hinge_y = -0.10
    hinge_z = 0.52
    hinge_centers = (-0.55, 0.55)

    rear_frame = model.part("rear_frame")
    rear_frame.visual(
        Box((2.06, 0.145, 0.12)),
        origin=Origin(xyz=(0.0, 0.0075, 0.44)),
        material=body_blue,
        name="lower_sill",
    )
    rear_frame.visual(
        Box((1.90, 0.82, 0.06)),
        origin=Origin(xyz=(0.0, 0.36, 0.53)),
        material=bed_black,
        name="bed_floor",
    )
    for idx, x in enumerate((-1.04, 1.04)):
        rear_frame.visual(
            Box((0.12, 0.20, 1.00)),
            origin=Origin(xyz=(x, 0.0, 0.90)),
            material=body_blue,
            name=f"rear_post_{idx}",
        )
        rear_frame.visual(
            Box((0.12, 0.80, 0.68)),
            origin=Origin(xyz=(x, 0.36, 0.84)),
            material=body_blue,
            name=f"bed_side_{idx}",
        )

    rear_frame.visual(
        Box((2.12, 0.16, 0.10)),
        origin=Origin(xyz=(0.0, -0.17, 0.30)),
        material=bumper_grey,
        name="step_bumper",
    )
    for idx, x in enumerate((-0.55, 0.55)):
        rear_frame.visual(
            Box((0.08, 0.08, 0.14)),
            origin=Origin(xyz=(x, -0.10, 0.365)),
            material=bumper_grey,
            name=f"bumper_bracket_{idx}",
        )

    for idx, x in enumerate(hinge_centers):
        for side, dx in enumerate((-0.115, 0.115)):
            rear_frame.visual(
                Box((0.075, 0.050, 0.08)),
                origin=Origin(xyz=(x + dx, -0.070, 0.505)),
                material=hinge_steel,
                name=f"body_hinge_plate_{idx}_{side}",
            )
            rear_frame.visual(
                Cylinder(radius=0.032, length=0.055),
                origin=Origin(
                    xyz=(x + dx, hinge_y, hinge_z),
                    rpy=(0.0, math.pi / 2.0, 0.0),
                ),
                material=hinge_steel,
                name=f"body_hinge_knuckle_{idx}_{side}",
            )

    tailgate = model.part("tailgate")
    tailgate.visual(
        Box((1.72, 0.074, 0.78)),
        origin=Origin(xyz=(0.0, -0.047, 0.435)),
        material=body_blue,
        name="panel_shell",
    )
    tailgate.visual(
        Box((1.74, 0.09, 0.055)),
        origin=Origin(xyz=(0.0, -0.048, 0.852)),
        material=satin_black,
        name="top_cap",
    )
    tailgate.visual(
        Box((1.22, 0.010, 0.34)),
        origin=Origin(xyz=(0.0, -0.088, 0.45)),
        material=dark_blue,
        name="pressed_center_panel",
    )
    tailgate.visual(
        Box((1.36, 0.024, 0.045)),
        origin=Origin(xyz=(0.0, -0.098, 0.635)),
        material=body_blue,
        name="upper_stamp_rib",
    )
    tailgate.visual(
        Box((1.36, 0.024, 0.045)),
        origin=Origin(xyz=(0.0, -0.098, 0.265)),
        material=body_blue,
        name="lower_stamp_rib",
    )
    for idx, x in enumerate((-0.68, 0.68)):
        tailgate.visual(
            Box((0.045, 0.024, 0.34)),
            origin=Origin(xyz=(x, -0.098, 0.45)),
            material=body_blue,
            name=f"side_stamp_rib_{idx}",
        )
        tailgate.visual(
            Box((0.18, 0.010, 0.055)),
            origin=Origin(xyz=(x, -0.088, 0.18)),
            material=reflector_red,
            name=f"reflector_{idx}",
        )

    tailgate.visual(
        Box((0.26, 0.045, 0.12)),
        origin=Origin(xyz=(0.0, -0.105, 0.68)),
        material=satin_black,
        name="latch_housing",
    )
    tailgate.visual(
        Box((0.18, 0.010, 0.028)),
        origin=Origin(xyz=(0.0, -0.132, 0.62)),
        material=hinge_steel,
        name="release_label_plate",
    )

    for idx, x in enumerate(hinge_centers):
        tailgate.visual(
            Box((0.15, 0.035, 0.12)),
            origin=Origin(xyz=(x, -0.045, 0.065)),
            material=hinge_steel,
            name=f"gate_hinge_leaf_{idx}",
        )
        tailgate.visual(
            Cylinder(radius=0.032, length=0.155),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=hinge_steel,
            name=f"gate_hinge_barrel_{idx}",
        )

    release_button = model.part("release_button")
    release_button.visual(
        Cylinder(radius=0.045, length=0.032),
        origin=Origin(xyz=(0.0, -0.016, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=satin_black,
        name="button_cap",
    )
    release_button.visual(
        Cylinder(radius=0.030, length=0.004),
        origin=Origin(xyz=(0.0, -0.034, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=Material("slightly_worn_button_face", rgba=(0.03, 0.03, 0.028, 1.0)),
        name="button_face",
    )

    model.articulation(
        "rear_frame_to_tailgate",
        ArticulationType.REVOLUTE,
        parent=rear_frame,
        child=tailgate,
        origin=Origin(xyz=(0.0, hinge_y, hinge_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=220.0, velocity=1.2, lower=0.0, upper=1.57),
    )
    model.articulation(
        "tailgate_to_release_button",
        ArticulationType.PRISMATIC,
        parent=tailgate,
        child=release_button,
        origin=Origin(xyz=(0.0, -0.1275, 0.68)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=0.12, lower=0.0, upper=0.026),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    rear_frame = object_model.get_part("rear_frame")
    tailgate = object_model.get_part("tailgate")
    button = object_model.get_part("release_button")
    gate_joint = object_model.get_articulation("rear_frame_to_tailgate")
    button_joint = object_model.get_articulation("tailgate_to_release_button")

    ctx.expect_overlap(
        tailgate,
        rear_frame,
        axes="x",
        elem_a="panel_shell",
        elem_b="lower_sill",
        min_overlap=1.60,
        name="tailgate panel spans the rear opening width",
    )
    ctx.expect_gap(
        rear_frame,
        tailgate,
        axis="y",
        positive_elem="lower_sill",
        negative_elem="panel_shell",
        min_gap=0.035,
        max_gap=0.055,
        name="closed panel sits just outside the rear sill",
    )
    ctx.expect_gap(
        tailgate,
        rear_frame,
        axis="z",
        positive_elem="panel_shell",
        negative_elem="lower_sill",
        min_gap=0.055,
        max_gap=0.075,
        name="panel bottom clears the lower sill above the hinge line",
    )
    ctx.expect_overlap(
        button,
        tailgate,
        axes="xz",
        elem_a="button_cap",
        elem_b="latch_housing",
        min_overlap=0.08,
        name="release button is centered in the latch housing",
    )
    ctx.expect_gap(
        tailgate,
        button,
        axis="y",
        positive_elem="latch_housing",
        negative_elem="button_cap",
        max_gap=0.001,
        max_penetration=0.001,
        name="button cap is seated against the housing face",
    )

    def _aabb_center(aabb):
        if aabb is None:
            return None
        lo, hi = aabb
        return tuple((lo[i] + hi[i]) * 0.5 for i in range(3))

    closed_top = _aabb_center(ctx.part_element_world_aabb(tailgate, elem="top_cap"))
    with ctx.pose({gate_joint: 1.20}):
        dropped_top = _aabb_center(ctx.part_element_world_aabb(tailgate, elem="top_cap"))
    ctx.check(
        "tailgate drops rearward and downward on the lower hinge",
        closed_top is not None
        and dropped_top is not None
        and dropped_top[1] < closed_top[1] - 0.45
        and dropped_top[2] < closed_top[2] - 0.35,
        details=f"closed_top={closed_top}, dropped_top={dropped_top}",
    )

    rest_button = ctx.part_world_position(button)
    with ctx.pose({button_joint: 0.026}):
        pressed_button = ctx.part_world_position(button)
        ctx.expect_overlap(
            button,
            tailgate,
            axes="y",
            elem_a="button_cap",
            elem_b="latch_housing",
            min_overlap=0.020,
            name="pressed release button travels into the latch housing",
        )
    ctx.check(
        "release button moves inward along the tailgate normal",
        rest_button is not None
        and pressed_button is not None
        and pressed_button[1] > rest_button[1] + 0.020,
        details=f"rest={rest_button}, pressed={pressed_button}",
    )

    return ctx.report()


object_model = build_object_model()
