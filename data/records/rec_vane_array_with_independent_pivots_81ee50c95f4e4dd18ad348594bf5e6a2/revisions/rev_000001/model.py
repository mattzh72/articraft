from __future__ import annotations

import cadquery as cq
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
    mesh_from_cadquery,
)


BLADE_COUNT = 6
BLADE_WIDTH = 0.130
BLADE_HEIGHT = 0.720
BLADE_TOP_DROP = 0.105
BLADE_SPACING = 0.180
RAIL_UNDERSIDE_Z = -0.035


def _faceted_blade_shape() -> cq.Workplane:
    """Crisp extruded shutter blade with folded hard-surface facets."""
    w = BLADE_WIDTH
    t = 0.026
    profile = [
        (-0.50 * w, 0.000),
        (-0.45 * w, -0.010),
        (-0.17 * w, -0.013),
        (0.000, -0.006),
        (0.17 * w, -0.013),
        (0.45 * w, -0.010),
        (0.50 * w, 0.000),
        (0.45 * w, 0.010),
        (0.17 * w, 0.013),
        (0.000, 0.006),
        (-0.17 * w, 0.013),
        (-0.45 * w, 0.010),
    ]
    return (
        cq.Workplane("XY")
        .polyline(profile)
        .close()
        .extrude(BLADE_HEIGHT)
        .translate((0.0, 0.0, -(BLADE_TOP_DROP + BLADE_HEIGHT)))
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="underslung_shutter_blade_assembly")

    dark_steel = model.material("dark_powder_coat", rgba=(0.035, 0.038, 0.042, 1.0))
    blade_metal = model.material("satin_aluminum_blades", rgba=(0.66, 0.68, 0.70, 1.0))
    bearing_black = model.material("black_pivot_hardware", rgba=(0.01, 0.011, 0.012, 1.0))

    support = model.part("top_support")
    support.visual(
        Box((1.180, 0.120, 0.070)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=dark_steel,
        name="top_rail",
    )
    support.visual(
        Box((1.180, 0.026, 0.155)),
        origin=Origin(xyz=(0.0, 0.066, 0.025)),
        material=dark_steel,
        name="rear_flange",
    )
    support.visual(
        Box((1.120, 0.020, 0.014)),
        origin=Origin(xyz=(0.0, -0.047, -0.041)),
        material=dark_steel,
        name="front_underside_strip",
    )
    support.visual(
        Box((1.120, 0.020, 0.014)),
        origin=Origin(xyz=(0.0, 0.047, -0.041)),
        material=dark_steel,
        name="rear_underside_strip",
    )

    blade_mesh = mesh_from_cadquery(_faceted_blade_shape(), "faceted_shutter_blade")
    start_x = -0.5 * (BLADE_COUNT - 1) * BLADE_SPACING

    for index in range(BLADE_COUNT):
        x = start_x + index * BLADE_SPACING

        for side, y in (("front", -0.028), ("rear", 0.028)):
            support.visual(
                Box((0.052, 0.014, 0.064)),
                origin=Origin(xyz=(x, y, -0.064)),
                material=dark_steel,
                name=f"bearing_{index}_{side}",
            )

        blade = model.part(f"blade_{index}")
        blade.visual(
            Cylinder(radius=0.007, length=0.060),
            origin=Origin(xyz=(0.0, 0.0, -0.030)),
            material=bearing_black,
            name="pivot_pin",
        )
        blade.visual(
            Cylinder(radius=0.022, length=0.012),
            origin=Origin(xyz=(0.0, 0.0, -0.061)),
            material=bearing_black,
            name="bearing_washer",
        )
        blade.visual(
            Box((0.052, 0.022, 0.052)),
            origin=Origin(xyz=(0.0, 0.0, -0.089)),
            material=blade_metal,
            name="top_lug",
        )
        blade.visual(
            blade_mesh,
            origin=Origin(),
            material=blade_metal,
            name="blade_panel",
        )

        model.articulation(
            f"top_support_to_blade_{index}",
            ArticulationType.REVOLUTE,
            parent=support,
            child=blade,
            origin=Origin(xyz=(x, 0.0, RAIL_UNDERSIDE_Z)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=3.0, velocity=2.5, lower=-1.15, upper=1.15),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    blade_names = [f"blade_{index}" for index in range(BLADE_COUNT)]
    root_names = [part.name for part in object_model.root_parts()]
    ctx.check(
        "top support is the single root",
        root_names == ["top_support"],
        details=f"roots={root_names}",
    )
    ctx.check(
        "six independently named hanging blades",
        all(object_model.get_part(name) is not None for name in blade_names),
        details=f"blade_names={blade_names}",
    )

    for index, blade_name in enumerate(blade_names):
        joint = object_model.get_articulation(f"top_support_to_blade_{index}")
        limits = joint.motion_limits
        ctx.check(
            f"{blade_name} has its own vertical revolute pivot",
            joint.articulation_type == ArticulationType.REVOLUTE
            and tuple(joint.axis) == (0.0, 0.0, 1.0)
            and limits is not None
            and limits.lower is not None
            and limits.upper is not None
            and limits.lower < -1.0
            and limits.upper > 1.0,
            details=f"joint={joint.name}, type={joint.articulation_type}, axis={joint.axis}, limits={limits}",
        )
        ctx.expect_gap(
            "top_support",
            blade_name,
            axis="z",
            positive_elem="top_rail",
            negative_elem="pivot_pin",
            max_gap=0.001,
            max_penetration=0.001,
            name=f"{blade_name} pivot pin seats under the rail",
        )
        ctx.expect_overlap(
            blade_name,
            "top_support",
            axes="xy",
            elem_a="pivot_pin",
            elem_b="top_rail",
            min_overlap=0.010,
            name=f"{blade_name} pivot hangs from the rail footprint",
        )

    sample_blade = object_model.get_part("blade_2")
    sample_joint = object_model.get_articulation("top_support_to_blade_2")
    closed_aabb = ctx.part_element_world_aabb(sample_blade, elem="blade_panel")
    with ctx.pose({sample_joint: 0.95}):
        turned_aabb = ctx.part_element_world_aabb(sample_blade, elem="blade_panel")

    closed_y = closed_aabb[1][1] - closed_aabb[0][1] if closed_aabb is not None else 0.0
    turned_y = turned_aabb[1][1] - turned_aabb[0][1] if turned_aabb is not None else 0.0
    ctx.check(
        "a blade visibly yaws on its individual pivot",
        turned_y > closed_y + 0.060,
        details=f"closed_y={closed_y}, turned_y={turned_y}",
    )

    return ctx.report()


object_model = build_object_model()
