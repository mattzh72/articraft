from __future__ import annotations

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


HUB_RADIUS = 0.018
HUB_THICKNESS = 0.036
BAR_WIDTH = 0.026
BAR_THICKNESS = 0.020

PALM_X = 0.120
PALM_Y = 0.460
PALM_Z = 0.080
ROOT_X = 0.095
ROOT_Y = 0.190

PROXIMAL_LENGTH = 0.115
MIDDLE_LENGTH = 0.095
DISTAL_BAR_LENGTH = 0.075
PAD_LENGTH = 0.030
PAD_WIDTH = 0.028
PAD_THICKNESS = 0.026


def _add_straight_link(part, *, length: float, material: Material, hub_material: Material) -> None:
    """Add one straight phalanx link whose local origin is the proximal pivot."""
    part.visual(
        Cylinder(radius=HUB_RADIUS, length=HUB_THICKNESS),
        origin=Origin(),
        material=hub_material,
        name="hub",
    )
    part.visual(
        Box((length - HUB_RADIUS, BAR_WIDTH, BAR_THICKNESS)),
        origin=Origin(xyz=((length - HUB_RADIUS) * 0.5, 0.0, 0.0)),
        material=material,
        name="bar",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dual_finger_gripper")

    aluminum = model.material("satin_aluminum", rgba=(0.58, 0.61, 0.64, 1.0))
    dark_aluminum = model.material("dark_anodized_aluminum", rgba=(0.05, 0.055, 0.060, 1.0))
    steel = model.material("brushed_steel", rgba=(0.72, 0.72, 0.68, 1.0))
    rubber = model.material("matte_black_rubber", rgba=(0.015, 0.014, 0.013, 1.0))

    palm = model.part("palm")
    palm.visual(
        Box((PALM_X, PALM_Y, PALM_Z)),
        origin=Origin(),
        material=aluminum,
        name="palm_block",
    )
    palm.visual(
        Box((0.026, 0.065, 0.050)),
        origin=Origin(xyz=(0.066, ROOT_Y, 0.0)),
        material=aluminum,
        name="finger_0_mount",
    )
    palm.visual(
        Box((0.026, 0.065, 0.050)),
        origin=Origin(xyz=(0.066, -ROOT_Y, 0.0)),
        material=aluminum,
        name="finger_1_mount",
    )
    palm.visual(
        Box((0.018, PALM_Y * 0.82, 0.058)),
        origin=Origin(xyz=(-0.069, 0.0, 0.0)),
        material=dark_aluminum,
        name="rear_plate",
    )

    upper_axis = (0.0, 0.0, -1.0)
    lower_axis = (0.0, 0.0, 1.0)
    base_limits = MotionLimits(effort=35.0, velocity=1.4, lower=0.0, upper=0.28)
    knuckle_limits = MotionLimits(effort=25.0, velocity=1.8, lower=0.0, upper=0.32)
    tip_limits = MotionLimits(effort=15.0, velocity=2.0, lower=0.0, upper=0.24)

    for index, root_y, axis in (
        (0, ROOT_Y, upper_axis),
        (1, -ROOT_Y, lower_axis),
    ):
        proximal = model.part(f"finger_{index}_proximal")
        _add_straight_link(
            proximal,
            length=PROXIMAL_LENGTH,
            material=dark_aluminum,
            hub_material=steel,
        )

        middle = model.part(f"finger_{index}_middle")
        _add_straight_link(
            middle,
            length=MIDDLE_LENGTH,
            material=dark_aluminum,
            hub_material=steel,
        )

        distal = model.part(f"finger_{index}_distal")
        _add_straight_link(
            distal,
            # Terminal phalanx has no following hub, so let its straight bar
            # run all the way to the rubber pad instead of stopping one hub
            # radius short.
            length=DISTAL_BAR_LENGTH + HUB_RADIUS,
            material=dark_aluminum,
            hub_material=steel,
        )
        distal.visual(
            Box((PAD_LENGTH, PAD_WIDTH, PAD_THICKNESS)),
            origin=Origin(xyz=(DISTAL_BAR_LENGTH + PAD_LENGTH * 0.5, 0.0, 0.0)),
            material=rubber,
            name="pad",
        )

        model.articulation(
            f"finger_{index}_base",
            ArticulationType.REVOLUTE,
            parent=palm,
            child=proximal,
            origin=Origin(xyz=(ROOT_X, root_y, 0.0)),
            axis=axis,
            motion_limits=base_limits,
        )
        model.articulation(
            f"finger_{index}_knuckle",
            ArticulationType.REVOLUTE,
            parent=proximal,
            child=middle,
            origin=Origin(xyz=(PROXIMAL_LENGTH, 0.0, 0.0)),
            axis=axis,
            motion_limits=knuckle_limits,
        )
        model.articulation(
            f"finger_{index}_tip",
            ArticulationType.REVOLUTE,
            parent=middle,
            child=distal,
            origin=Origin(xyz=(MIDDLE_LENGTH, 0.0, 0.0)),
            axis=axis,
            motion_limits=tip_limits,
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    palm = object_model.get_part("palm")

    f0_proximal = object_model.get_part("finger_0_proximal")
    f0_middle = object_model.get_part("finger_0_middle")
    f0_distal = object_model.get_part("finger_0_distal")
    f1_proximal = object_model.get_part("finger_1_proximal")
    f1_middle = object_model.get_part("finger_1_middle")
    f1_distal = object_model.get_part("finger_1_distal")

    ctx.check(
        "two independent phalanx chains",
        len(object_model.parts) == 7 and len(object_model.articulations) == 6,
        details=f"parts={len(object_model.parts)}, joints={len(object_model.articulations)}",
    )

    ctx.expect_contact(
        f0_proximal,
        palm,
        elem_a="hub",
        elem_b="finger_0_mount",
        contact_tol=0.001,
        name="finger 0 mounted to palm",
    )
    ctx.expect_contact(
        f1_proximal,
        palm,
        elem_a="hub",
        elem_b="finger_1_mount",
        contact_tol=0.001,
        name="finger 1 mounted to palm",
    )
    ctx.expect_contact(
        f0_proximal,
        f0_middle,
        elem_a="bar",
        elem_b="hub",
        contact_tol=0.001,
        name="finger 0 proximal link reaches middle hub",
    )
    ctx.expect_contact(
        f0_middle,
        f0_distal,
        elem_a="bar",
        elem_b="hub",
        contact_tol=0.001,
        name="finger 0 middle link reaches distal hub",
    )
    ctx.expect_contact(
        f1_proximal,
        f1_middle,
        elem_a="bar",
        elem_b="hub",
        contact_tol=0.001,
        name="finger 1 proximal link reaches middle hub",
    )
    ctx.expect_contact(
        f1_middle,
        f1_distal,
        elem_a="bar",
        elem_b="hub",
        contact_tol=0.001,
        name="finger 1 middle link reaches distal hub",
    )

    f0_base = object_model.get_articulation("finger_0_base")
    f0_knuckle = object_model.get_articulation("finger_0_knuckle")
    f0_tip = object_model.get_articulation("finger_0_tip")
    f1_base = object_model.get_articulation("finger_1_base")
    f1_knuckle = object_model.get_articulation("finger_1_knuckle")
    f1_tip = object_model.get_articulation("finger_1_tip")

    f0_rest = ctx.part_element_world_aabb(f0_distal, elem="pad")
    f1_rest = ctx.part_element_world_aabb(f1_distal, elem="pad")
    f0_rest_y = (f0_rest[0][1] + f0_rest[1][1]) * 0.5 if f0_rest else None
    f1_rest_y = (f1_rest[0][1] + f1_rest[1][1]) * 0.5 if f1_rest else None

    with ctx.pose(
        {
            f0_base: 0.28,
            f0_knuckle: 0.32,
            f0_tip: 0.24,
            f1_base: 0.28,
            f1_knuckle: 0.32,
            f1_tip: 0.24,
        }
    ):
        f0_flexed = ctx.part_element_world_aabb(f0_distal, elem="pad")
        f1_flexed = ctx.part_element_world_aabb(f1_distal, elem="pad")
        f0_flexed_y = (f0_flexed[0][1] + f0_flexed[1][1]) * 0.5 if f0_flexed else None
        f1_flexed_y = (f1_flexed[0][1] + f1_flexed[1][1]) * 0.5 if f1_flexed else None
        ctx.check(
            "finger 0 flexes inward",
            f0_rest_y is not None
            and f0_flexed_y is not None
            and f0_flexed_y < f0_rest_y - 0.10,
            details=f"rest_y={f0_rest_y}, flexed_y={f0_flexed_y}",
        )
        ctx.check(
            "finger 1 flexes inward",
            f1_rest_y is not None
            and f1_flexed_y is not None
            and f1_flexed_y > f1_rest_y + 0.10,
            details=f"rest_y={f1_rest_y}, flexed_y={f1_flexed_y}",
        )
        ctx.expect_gap(
            f0_distal,
            f1_distal,
            axis="y",
            positive_elem="pad",
            negative_elem="pad",
            min_gap=0.004,
            name="closed pads remain separated",
        )

    return ctx.report()


object_model = build_object_model()
