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
    model = ArticulatedObject(name="underslung_twin_jaw_gripper")

    satin_aluminum = model.material("satin_aluminum", rgba=(0.64, 0.66, 0.67, 1.0))
    dark_anodized = model.material("dark_anodized", rgba=(0.09, 0.10, 0.11, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.015, 0.014, 0.013, 1.0))
    polished_steel = model.material("polished_steel", rgba=(0.78, 0.80, 0.82, 1.0))
    bolt_black = model.material("black_oxide_bolts", rgba=(0.02, 0.02, 0.022, 1.0))

    support = model.part("top_support")
    support.visual(
        Box((0.56, 0.18, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.350)),
        material=dark_anodized,
        name="mounting_plate",
    )
    support.visual(
        Box((0.24, 0.16, 0.110)),
        origin=Origin(xyz=(0.0, 0.0, 0.286)),
        material=satin_aluminum,
        name="center_body",
    )
    support.visual(
        Box((0.17, 0.045, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, 0.226)),
        material=satin_aluminum,
        name="center_neck",
    )
    support.visual(
        Box((0.48, 0.055, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.205)),
        material=polished_steel,
        name="guide_bar",
    )
    support.visual(
        Box((0.44, 0.030, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.194)),
        material=dark_anodized,
        name="underside_wear_strip",
    )

    for index, x_pos in enumerate((-0.205, 0.205)):
        for y_pos in (-0.055, 0.055):
            support.visual(
                Cylinder(radius=0.011, length=0.007),
                origin=Origin(xyz=(x_pos, y_pos, 0.369), rpy=(0.0, 0.0, 0.0)),
                material=bolt_black,
                name=f"mount_bolt_{index}_{0 if y_pos < 0 else 1}",
            )

    def add_carriage(name: str, jaw_sign: float) -> object:
        carriage = model.part(name)
        carriage.visual(
            Box((0.110, 0.095, 0.038)),
            origin=Origin(xyz=(0.0, 0.0, -0.019)),
            material=dark_anodized,
            name="slider_shoe",
        )
        for side, y_pos in enumerate((-0.043, 0.043)):
            carriage.visual(
                Box((0.092, 0.012, 0.070)),
                origin=Origin(xyz=(0.0, y_pos, 0.005)),
                material=dark_anodized,
                name=f"rail_cheek_{side}",
            )
            for bolt_index, x_pos in enumerate((-0.030, 0.030)):
                carriage.visual(
                    Cylinder(radius=0.007, length=0.007),
                    origin=Origin(
                        xyz=(x_pos, y_pos + (0.007 if y_pos > 0 else -0.007), -0.001),
                        rpy=(math.pi / 2.0, 0.0, 0.0),
                    ),
                    material=bolt_black,
                    name=f"cheek_bolt_{side}_{bolt_index}",
                )

        carriage.visual(
            Box((0.055, 0.070, 0.140)),
            origin=Origin(xyz=(0.0, 0.0, -0.105)),
            material=satin_aluminum,
            name="hanger_web",
        )
        carriage.visual(
            Box((0.080, 0.120, 0.040)),
            origin=Origin(xyz=(jaw_sign * 0.022, 0.0, -0.195)),
            material=satin_aluminum,
            name="jaw_palm",
        )
        carriage.visual(
            Box((0.030, 0.120, 0.100)),
            origin=Origin(xyz=(jaw_sign * 0.061, 0.0, -0.255)),
            material=satin_aluminum,
            name="jaw_finger",
        )
        carriage.visual(
            Box((0.012, 0.105, 0.075)),
            origin=Origin(xyz=(jaw_sign * 0.080, 0.0, -0.260)),
            material=black_rubber,
            name="grip_pad",
        )
        return carriage

    carriage_0 = add_carriage("carriage_0", jaw_sign=1.0)
    carriage_1 = add_carriage("carriage_1", jaw_sign=-1.0)

    model.articulation(
        "support_to_carriage_0",
        ArticulationType.PRISMATIC,
        parent=support,
        child=carriage_0,
        origin=Origin(xyz=(-0.170, 0.0, 0.185)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=130.0, velocity=0.18, lower=0.0, upper=0.080),
    )
    model.articulation(
        "support_to_carriage_1",
        ArticulationType.PRISMATIC,
        parent=support,
        child=carriage_1,
        origin=Origin(xyz=(0.170, 0.0, 0.185)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=130.0, velocity=0.18, lower=-0.080, upper=0.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    support = object_model.get_part("top_support")
    carriage_0 = object_model.get_part("carriage_0")
    carriage_1 = object_model.get_part("carriage_1")
    slide_0 = object_model.get_articulation("support_to_carriage_0")
    slide_1 = object_model.get_articulation("support_to_carriage_1")

    ctx.check(
        "two independent prismatic slides share the closing axis",
        slide_0.articulation_type == ArticulationType.PRISMATIC
        and slide_1.articulation_type == ArticulationType.PRISMATIC
        and tuple(slide_0.axis) == (1.0, 0.0, 0.0)
        and tuple(slide_1.axis) == (1.0, 0.0, 0.0)
        and slide_0.mimic is None
        and slide_1.mimic is None,
        details=f"slide_0={slide_0.articulation_type}, axis={slide_0.axis}; "
        f"slide_1={slide_1.articulation_type}, axis={slide_1.axis}",
    )

    ctx.expect_gap(
        support,
        carriage_0,
        axis="z",
        positive_elem="guide_bar",
        negative_elem="slider_shoe",
        max_gap=0.001,
        max_penetration=0.0,
        name="carriage 0 is seated under the guide bar",
    )
    ctx.expect_gap(
        support,
        carriage_1,
        axis="z",
        positive_elem="guide_bar",
        negative_elem="slider_shoe",
        max_gap=0.001,
        max_penetration=0.0,
        name="carriage 1 is seated under the guide bar",
    )
    ctx.expect_overlap(
        support,
        carriage_0,
        axes="xy",
        elem_a="guide_bar",
        elem_b="slider_shoe",
        min_overlap=0.04,
        name="carriage 0 slider remains under the linear guide",
    )
    ctx.expect_overlap(
        support,
        carriage_1,
        axes="xy",
        elem_a="guide_bar",
        elem_b="slider_shoe",
        min_overlap=0.04,
        name="carriage 1 slider remains under the linear guide",
    )

    ctx.expect_gap(
        carriage_1,
        carriage_0,
        axis="x",
        positive_elem="grip_pad",
        negative_elem="grip_pad",
        min_gap=0.15,
        name="open jaws start with a wide central gap",
    )

    rest_0 = ctx.part_world_position(carriage_0)
    rest_1 = ctx.part_world_position(carriage_1)
    with ctx.pose({slide_0: 0.080, slide_1: -0.080}):
        ctx.expect_gap(
            carriage_1,
            carriage_0,
            axis="x",
            positive_elem="grip_pad",
            negative_elem="grip_pad",
            min_gap=0.004,
            max_gap=0.015,
            name="closed jaws nearly meet without crossing",
        )
        closed_0 = ctx.part_world_position(carriage_0)
        closed_1 = ctx.part_world_position(carriage_1)

    with ctx.pose({slide_0: 0.080}):
        solo_0 = ctx.part_world_position(carriage_0)
        solo_1 = ctx.part_world_position(carriage_1)

    ctx.check(
        "both carriages close toward the center",
        rest_0 is not None
        and rest_1 is not None
        and closed_0 is not None
        and closed_1 is not None
        and closed_0[0] > rest_0[0] + 0.075
        and closed_1[0] < rest_1[0] - 0.075,
        details=f"rest=({rest_0}, {rest_1}), closed=({closed_0}, {closed_1})",
    )
    ctx.check(
        "each carriage can move independently",
        rest_1 is not None
        and solo_0 is not None
        and solo_1 is not None
        and solo_0[0] > (rest_0[0] if rest_0 else 0.0) + 0.075
        and abs(solo_1[0] - rest_1[0]) < 1e-6,
        details=f"rest=({rest_0}, {rest_1}), left-only=({solo_0}, {solo_1})",
    )

    return ctx.report()


object_model = build_object_model()
