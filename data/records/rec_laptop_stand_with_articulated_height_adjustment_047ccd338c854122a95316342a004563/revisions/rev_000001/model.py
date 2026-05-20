from __future__ import annotations

import math

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
    model = ArticulatedObject(name="laptop_stand_guided_tray")

    matte_black = model.material("matte_black", rgba=(0.015, 0.016, 0.018, 1.0))
    satin_graphite = model.material("satin_graphite", rgba=(0.18, 0.19, 0.20, 1.0))
    dark_anodized = model.material("dark_anodized", rgba=(0.10, 0.11, 0.12, 1.0))
    polished_steel = model.material("polished_steel", rgba=(0.72, 0.74, 0.76, 1.0))
    rubber = model.material("soft_black_rubber", rgba=(0.005, 0.005, 0.005, 1.0))

    rod_x = -0.135
    rod_spacing = 0.320
    rod_y = rod_spacing / 2.0
    rod_radius = 0.012
    rod_length = 0.550
    base_top = 0.024

    base = model.part("base")
    base.visual(
        Box((0.380, 0.520, base_top)),
        origin=Origin(xyz=(0.0, 0.0, base_top / 2.0)),
        material=matte_black,
        name="wide_slab",
    )
    for x, y in ((-0.145, -0.215), (-0.145, 0.215), (0.150, -0.215), (0.150, 0.215)):
        base.visual(
            Box((0.070, 0.055, 0.008)),
            origin=Origin(xyz=(x, y, -0.004)),
            material=rubber,
            name=f"foot_{len([v for v in base.visuals if v.name and v.name.startswith('foot_')])}",
        )

    for index, y in enumerate((-rod_y, rod_y)):
        base.visual(
            Cylinder(radius=0.032, length=0.036),
            origin=Origin(xyz=(rod_x, y, base_top + 0.018)),
            material=satin_graphite,
            name=f"rod_socket_{index}",
        )
        base.visual(
            Cylinder(radius=rod_radius, length=rod_length),
            origin=Origin(xyz=(rod_x, y, base_top + rod_length / 2.0 - 0.002)),
            material=polished_steel,
            name=f"guide_rod_{index}",
        )

    base.visual(
        Cylinder(radius=0.014, length=0.390),
        origin=Origin(xyz=(rod_x, 0.0, base_top + rod_length - 0.002), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_graphite,
        name="top_bridge",
    )
    base.visual(
        Box((0.035, 0.410, 0.018)),
        origin=Origin(xyz=(rod_x, 0.0, base_top + 0.020)),
        material=satin_graphite,
        name="rear_tie_bar",
    )

    carriage = model.part("carriage")
    for index, y in enumerate((-rod_y, rod_y)):
        sign = -1.0 if y < 0.0 else 1.0
        carriage.visual(
            Box((0.014, 0.056, 0.070)),
            origin=Origin(xyz=(-0.019, y, 0.0)),
            material=satin_graphite,
            name=f"bushing_rear_{index}",
        )
        carriage.visual(
            Box((0.016, 0.056, 0.070)),
            origin=Origin(xyz=(0.020, y, 0.0)),
            material=satin_graphite,
            name=f"bushing_front_{index}",
        )
        carriage.visual(
            Box((0.060, 0.014, 0.070)),
            origin=Origin(xyz=(0.0, y + sign * 0.019, 0.0)),
            material=satin_graphite,
            name=f"bushing_outer_{index}",
        )
    carriage.visual(
        Box((0.055, 0.285, 0.050)),
        origin=Origin(xyz=(0.035, 0.0, 0.0)),
        material=satin_graphite,
        name="cross_carriage",
    )
    for index, y in enumerate((-0.095, 0.095)):
        carriage.visual(
            Box((0.032, 0.075, 0.070)),
            origin=Origin(xyz=(0.065, y, 0.045)),
            material=satin_graphite,
            name=f"hinge_upright_{index}",
        )
        carriage.visual(
            Cylinder(radius=0.016, length=0.100),
            origin=Origin(xyz=(0.065, y, 0.078), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=satin_graphite,
            name=f"carriage_knuckle_{index}",
        )

    model.articulation(
        "base_to_carriage",
        ArticulationType.PRISMATIC,
        parent=base,
        child=carriage,
        origin=Origin(xyz=(rod_x, 0.0, 0.180)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.18, lower=0.0, upper=0.220),
    )

    tray = model.part("tray")
    tray.visual(
        Box((0.320, 0.420, 0.016)),
        origin=Origin(xyz=(0.180, 0.0, -0.027)),
        material=dark_anodized,
        name="tray_panel",
    )
    tray.visual(
        Box((0.050, 0.420, 0.030)),
        origin=Origin(xyz=(0.045, 0.0, -0.018)),
        material=dark_anodized,
        name="rear_spine",
    )
    tray.visual(
        Box((0.024, 0.390, 0.045)),
        origin=Origin(xyz=(0.342, 0.0, -0.002)),
        material=dark_anodized,
        name="front_lip",
    )
    for index, y in enumerate((-0.212, 0.212)):
        tray.visual(
            Box((0.285, 0.018, 0.032)),
            origin=Origin(xyz=(0.195, y, -0.008)),
            material=dark_anodized,
            name=f"side_lip_{index}",
        )
    for index, x in enumerate((0.130, 0.255)):
        tray.visual(
            Box((0.070, 0.340, 0.004)),
            origin=Origin(xyz=(x, 0.0, -0.017)),
            material=rubber,
            name=f"rubber_strip_{index}",
        )

    tray_knuckles = (
        ("tray_knuckle_0", -0.1775, 0.065),
        ("tray_knuckle_1", 0.0, 0.090),
        ("tray_knuckle_2", 0.1775, 0.065),
    )
    for index, (name, y, length) in enumerate(tray_knuckles):
        tray.visual(
            Cylinder(radius=0.0145, length=length),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_anodized,
            name=name,
        )
        tray.visual(
            Box((0.034, length * 0.82, 0.018)),
            origin=Origin(xyz=(0.018, y, -0.013)),
            material=dark_anodized,
            name=f"tray_hinge_lug_{index}",
        )

    model.articulation(
        "carriage_to_tray",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=tray,
        origin=Origin(xyz=(0.065, 0.0, 0.078)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.2, lower=-0.25, upper=0.95),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    carriage = object_model.get_part("carriage")
    tray = object_model.get_part("tray")
    slide = object_model.get_articulation("base_to_carriage")
    tilt = object_model.get_articulation("carriage_to_tray")

    ctx.check(
        "one carriage carries both rods",
        slide.articulation_type == ArticulationType.PRISMATIC
        and slide.child == "carriage"
        and slide.parent == "base",
        details=f"slide={slide}",
    )
    ctx.check(
        "tray is the sole tilt child",
        tilt.articulation_type == ArticulationType.REVOLUTE
        and tilt.child == "tray"
        and tilt.parent == "carriage",
        details=f"tilt={tilt}",
    )
    ctx.check(
        "slide axis is vertical",
        tuple(round(value, 6) for value in slide.axis) == (0.0, 0.0, 1.0),
        details=f"axis={slide.axis}",
    )
    ctx.check(
        "tilt hinge axis is horizontal",
        abs(tilt.axis[2]) < 1e-6 and abs(tilt.axis[1]) > 0.99,
        details=f"axis={tilt.axis}",
    )

    for index in (0, 1):
        ctx.expect_contact(
            base,
            carriage,
            elem_a=f"guide_rod_{index}",
            elem_b=f"bushing_front_{index}",
            contact_tol=0.002,
            name=f"rod {index} guides carriage",
        )
        ctx.expect_overlap(
            base,
            carriage,
            axes="z",
            elem_a=f"guide_rod_{index}",
            elem_b=f"bushing_front_{index}",
            min_overlap=0.060,
            name=f"rod {index} remains engaged vertically",
        )

    ctx.expect_contact(
        carriage,
        tray,
        elem_a="carriage_knuckle_0",
        elem_b="tray_knuckle_0",
        contact_tol=0.002,
        name="interleaved hinge supports tray",
    )

    rest_pos = ctx.part_world_position(carriage)
    slide_upper = slide.motion_limits.upper if slide.motion_limits is not None else 0.0
    with ctx.pose({slide: slide_upper}):
        raised_pos = ctx.part_world_position(carriage)
        for index in (0, 1):
            ctx.expect_overlap(
                base,
                carriage,
                axes="z",
                elem_a=f"guide_rod_{index}",
                elem_b=f"bushing_front_{index}",
                min_overlap=0.060,
                name=f"raised carriage keeps rod {index} engaged",
            )

    ctx.check(
        "carriage slides upward on matched rods",
        rest_pos is not None and raised_pos is not None and raised_pos[2] > rest_pos[2] + 0.20,
        details=f"rest={rest_pos}, raised={raised_pos}",
    )

    front_rest = ctx.part_element_world_aabb(tray, elem="front_lip")
    with ctx.pose({tilt: 0.65}):
        front_tilted = ctx.part_element_world_aabb(tray, elem="front_lip")

    def _center_z(aabb):
        if aabb is None:
            return None
        return (aabb[0][2] + aabb[1][2]) / 2.0

    rest_z = _center_z(front_rest)
    tilted_z = _center_z(front_tilted)
    ctx.check(
        "tilt hinge raises tray front",
        rest_z is not None and tilted_z is not None and tilted_z > rest_z + 0.15,
        details=f"rest_z={rest_z}, tilted_z={tilted_z}",
    )

    return ctx.report()


object_model = build_object_model()
