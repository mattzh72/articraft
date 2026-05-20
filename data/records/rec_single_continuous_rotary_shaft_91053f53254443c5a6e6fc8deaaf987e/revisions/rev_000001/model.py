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
    model = ArticulatedObject(name="open_rotary_shaft_module")

    cast_iron = model.material("cast_iron", rgba=(0.18, 0.20, 0.22, 1.0))
    black_rail = model.material("black_oxide_rail", rgba=(0.06, 0.065, 0.070, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.70, 0.72, 0.74, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.25, 0.26, 0.27, 1.0))
    bolt_black = model.material("bolt_black", rgba=(0.02, 0.02, 0.022, 1.0))

    axis_z = 0.290
    bearing_x = 0.240

    base = model.part("base")
    base.visual(
        Box((0.780, 0.160, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=black_rail,
        name="base_rail",
    )
    base.visual(
        Box((0.700, 0.060, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
        material=cast_iron,
        name="raised_rib",
    )

    for idx, x in enumerate((-bearing_x, bearing_x)):
        base.visual(
            Box((0.150, 0.180, 0.028)),
            origin=Origin(xyz=(x, 0.0, 0.064)),
            material=cast_iron,
            name=f"foot_{idx}",
        )
        base.visual(
            Box((0.085, 0.110, 0.110)),
            origin=Origin(xyz=(x, 0.0, 0.176)),
            material=cast_iron,
            name=f"center_web_{idx}",
        )
        pad_names = (
            ("bearing_pad_0_0", "bearing_pad_0_1"),
            ("bearing_pad_1_0", "bearing_pad_1_1"),
        )[idx]
        for side, y in enumerate((-0.060, 0.060)):
            base.visual(
                Box((0.085, 0.030, 0.225)),
                origin=Origin(xyz=(x, y, 0.173)),
                material=cast_iron,
                name=f"side_cheek_{idx}_{side}",
            )
            base.visual(
                Cylinder(radius=0.026, length=0.088),
                origin=Origin(xyz=(x, y, axis_z), rpy=(0.0, pi / 2.0, 0.0)),
                material=dark_steel,
                name=pad_names[side],
            )
        for side, y in enumerate((-0.060, 0.060)):
            base.visual(
                Cylinder(radius=0.012, length=0.012),
                origin=Origin(xyz=(x, y, 0.084)),
                material=bolt_black,
                name=f"bolt_{idx}_{side}",
            )

    shaft = model.part("shaft")
    spin_origin = Origin(rpy=(0.0, pi / 2.0, 0.0))
    shaft.visual(
        Cylinder(radius=0.018, length=0.780),
        origin=spin_origin,
        material=brushed_steel,
        name="straight_shaft",
    )
    shaft.visual(
        Cylinder(radius=0.034, length=0.060),
        origin=Origin(xyz=(-bearing_x, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="bearing_collar_0",
    )
    shaft.visual(
        Cylinder(radius=0.034, length=0.060),
        origin=Origin(xyz=(bearing_x, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="bearing_collar_1",
    )
    shaft.visual(
        Cylinder(radius=0.055, length=0.105),
        origin=spin_origin,
        material=dark_steel,
        name="center_hub",
    )
    shaft.visual(
        Box((0.070, 0.018, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.058)),
        material=brushed_steel,
        name="hub_key",
    )
    shaft.visual(
        Cylinder(radius=0.072, length=0.036),
        origin=Origin(xyz=(0.335, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=dark_steel,
        name="end_flange",
    )
    for idx, (y, z) in enumerate(((0.038, 0.038), (-0.038, 0.038), (-0.038, -0.038), (0.038, -0.038))):
        shaft.visual(
            Cylinder(radius=0.0065, length=0.008),
            origin=Origin(xyz=(0.357, y, z), rpy=(0.0, pi / 2.0, 0.0)),
            material=bolt_black,
            name=f"flange_bolt_{idx}",
        )

    model.articulation(
        "shaft_spin",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=shaft,
        origin=Origin(xyz=(0.0, 0.0, axis_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=40.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    shaft = object_model.get_part("shaft")
    spin = object_model.get_articulation("shaft_spin")

    ctx.check(
        "shaft has one continuous spin joint",
        spin.articulation_type == ArticulationType.CONTINUOUS and spin.axis == (1.0, 0.0, 0.0),
        details=f"type={spin.articulation_type}, axis={spin.axis}",
    )

    ctx.expect_overlap(
        shaft,
        base,
        axes="x",
        elem_a="bearing_collar_0",
        elem_b="bearing_pad_0_0",
        min_overlap=0.055,
        name="first bearing collar is captured in its pedestal",
    )
    ctx.expect_overlap(
        shaft,
        base,
        axes="x",
        elem_a="bearing_collar_1",
        elem_b="bearing_pad_1_1",
        min_overlap=0.055,
        name="second bearing collar is captured in its pedestal",
    )
    ctx.expect_gap(
        base,
        shaft,
        axis="y",
        positive_elem="bearing_pad_0_1",
        negative_elem="bearing_collar_0",
        min_gap=0.0,
        max_gap=0.001,
        name="first pedestal pad just touches the rotating collar",
    )
    ctx.expect_gap(
        shaft,
        base,
        axis="y",
        positive_elem="bearing_collar_1",
        negative_elem="bearing_pad_1_0",
        min_gap=0.0,
        max_gap=0.001,
        name="second pedestal pad just touches the rotating collar",
    )
    ctx.expect_gap(
        shaft,
        base,
        axis="x",
        positive_elem="end_flange",
        negative_elem="bearing_pad_1_1",
        min_gap=0.025,
        name="end flange clears the outer support",
    )
    ctx.expect_gap(
        shaft,
        base,
        axis="z",
        positive_elem="center_hub",
        negative_elem="base_rail",
        min_gap=0.170,
        name="center hub is raised above the base rail",
    )

    def element_center(part, elem: str) -> tuple[float, float, float] | None:
        bounds = ctx.part_element_world_aabb(part, elem=elem)
        if bounds is None:
            return None
        lower, upper = bounds
        return tuple((lower[i] + upper[i]) * 0.5 for i in range(3))

    rest_key_center = element_center(shaft, "hub_key")
    rest_shaft_position = ctx.part_world_position(shaft)
    with ctx.pose({spin: pi / 2.0}):
        spun_key_center = element_center(shaft, "hub_key")
        spun_shaft_position = ctx.part_world_position(shaft)

    ctx.check(
        "visible key rotates about the supported shaft axis",
        rest_key_center is not None
        and spun_key_center is not None
        and rest_key_center[2] > 0.340
        and spun_key_center[1] < -0.050
        and abs(spun_key_center[2] - 0.290) < 0.010,
        details=f"rest={rest_key_center}, spun={spun_key_center}",
    )
    ctx.check(
        "continuous spin does not translate the shaft axis",
        rest_shaft_position is not None
        and spun_shaft_position is not None
        and all(abs(rest_shaft_position[i] - spun_shaft_position[i]) < 1e-6 for i in range(3)),
        details=f"rest={rest_shaft_position}, spun={spun_shaft_position}",
    )

    return ctx.report()


object_model = build_object_model()
