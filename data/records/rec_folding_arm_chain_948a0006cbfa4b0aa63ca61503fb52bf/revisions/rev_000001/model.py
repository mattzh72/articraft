from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Cylinder,
    ExtrudeWithHolesGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


LINK_WIDTH = 0.035
LINK_THICKNESS = 0.006
PIN_CLEARANCE_RADIUS = 0.009
PIN_RADIUS = 0.0072

SHORT_LEN = 0.180
LONG_LEN = 0.300
OUTER_LEN = 0.240


def _circle_profile(
    cx: float,
    cy: float,
    radius: float,
    *,
    segments: int = 28,
    clockwise: bool = False,
) -> list[tuple[float, float]]:
    angles = [
        2.0 * math.pi * i / segments
        for i in (range(segments, 0, -1) if clockwise else range(segments))
    ]
    return [(cx + radius * math.cos(a), cy + radius * math.sin(a)) for a in angles]


def _capsule_profile(
    length: float,
    width: float,
    *,
    segments_per_end: int = 16,
    clockwise: bool = False,
) -> list[tuple[float, float]]:
    """A flat rounded bar outline with pivot centers at x=0 and x=length."""
    radius = width * 0.5
    profile: list[tuple[float, float]] = []
    # Counter-clockwise loop: right lower edge -> right nose -> right upper edge,
    # then around the left nose back to the lower edge.
    for i in range(segments_per_end + 1):
        a = -math.pi / 2.0 + math.pi * i / segments_per_end
        profile.append((length + radius * math.cos(a), radius * math.sin(a)))
    for i in range(segments_per_end + 1):
        a = math.pi / 2.0 + math.pi * i / segments_per_end
        profile.append((radius * math.cos(a), radius * math.sin(a)))
    if clockwise:
        profile.reverse()
    return profile


def _translated_profile(
    profile: list[tuple[float, float]],
    dx: float,
    dy: float,
) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def _flat_bar_geometry(length: float) -> ExtrudeWithHolesGeometry:
    center_slot_length = max(0.030, length - 0.105)
    slot = _capsule_profile(center_slot_length, 0.011, segments_per_end=10, clockwise=True)
    return ExtrudeWithHolesGeometry(
        _capsule_profile(length, LINK_WIDTH, segments_per_end=18),
        [
            _circle_profile(0.0, 0.0, PIN_CLEARANCE_RADIUS, clockwise=True),
            _circle_profile(length, 0.0, PIN_CLEARANCE_RADIUS, clockwise=True),
            _translated_profile(slot, (length - center_slot_length) * 0.5, 0.0),
        ],
        LINK_THICKNESS,
        center=True,
    )


def _shoe_geometry() -> ExtrudeWithHolesGeometry:
    # Triangular mounting shoe with a hinge hole at the apex and two screw holes.
    outer = [(-0.020, -0.046), (0.112, 0.0), (-0.020, 0.046)]
    return ExtrudeWithHolesGeometry(
        outer,
        [
            _circle_profile(0.0, 0.0, PIN_CLEARANCE_RADIUS, clockwise=True),
            _circle_profile(0.054, -0.012, 0.0055, clockwise=True),
            _circle_profile(0.054, 0.012, 0.0055, clockwise=True),
        ],
        LINK_THICKNESS,
        center=True,
    )


def _root_plate_geometry() -> ExtrudeWithHolesGeometry:
    outer = _translated_profile(rounded_rect_profile(0.130, 0.074, 0.010), -0.047, 0.0)
    return ExtrudeWithHolesGeometry(
        outer,
        [
            _circle_profile(-0.086, -0.020, 0.0062, clockwise=True),
            _circle_profile(-0.086, 0.020, 0.0062, clockwise=True),
            _circle_profile(-0.035, 0.0, 0.0062, clockwise=True),
        ],
        0.006,
        center=True,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_support_stay")

    galvanized = model.material("galvanized_steel", rgba=(0.62, 0.64, 0.61, 1.0))
    pin_metal = model.material("polished_pin", rgba=(0.82, 0.80, 0.74, 1.0))

    root = model.part("root_bracket")
    root.visual(
        mesh_from_geometry(_root_plate_geometry(), "root_plate"),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=galvanized,
        name="mounting_plate",
    )
    root.visual(
        Cylinder(radius=0.015, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=galvanized,
        name="root_boss",
    )
    root.visual(
        Cylinder(radius=PIN_RADIUS, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
        material=pin_metal,
        name="root_pin",
    )
    root.visual(
        Cylinder(radius=0.0135, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=pin_metal,
        name="root_rivet_head",
    )
    short_bar = model.part("short_bar")
    short_bar.visual(
        mesh_from_geometry(_flat_bar_geometry(SHORT_LEN), "short_bar_plate"),
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material=galvanized,
        name="short_bar_plate",
    )
    short_bar.visual(
        Cylinder(radius=PIN_RADIUS, length=0.020),
        origin=Origin(xyz=(SHORT_LEN, 0.0, 0.029)),
        material=pin_metal,
        name="middle_pin",
    )
    short_bar.visual(
        Cylinder(radius=0.0135, length=0.004),
        origin=Origin(xyz=(SHORT_LEN, 0.0, 0.039)),
        material=pin_metal,
        name="middle_rivet_head",
    )

    long_bar = model.part("long_bar")
    long_bar.visual(
        mesh_from_geometry(_flat_bar_geometry(LONG_LEN), "long_bar_plate"),
        origin=Origin(xyz=(0.0, 0.0, 0.028)),
        material=galvanized,
        name="long_bar_plate",
    )
    long_bar.visual(
        Cylinder(radius=PIN_RADIUS, length=0.020),
        origin=Origin(xyz=(LONG_LEN, 0.0, 0.039)),
        material=pin_metal,
        name="outer_pin",
    )
    long_bar.visual(
        Cylinder(radius=0.0135, length=0.004),
        origin=Origin(xyz=(LONG_LEN, 0.0, 0.049)),
        material=pin_metal,
        name="outer_rivet_head",
    )

    outer_bar = model.part("outer_bar")
    outer_bar.visual(
        mesh_from_geometry(_flat_bar_geometry(OUTER_LEN), "outer_bar_plate"),
        origin=Origin(xyz=(0.0, 0.0, 0.038)),
        material=galvanized,
        name="outer_bar_plate",
    )

    end_shoe = model.part("end_shoe")
    end_shoe.visual(
        mesh_from_geometry(_shoe_geometry(), "triangular_end_shoe"),
        origin=Origin(xyz=(0.0, 0.0, 0.050)),
        material=galvanized,
        name="triangular_plate",
    )
    end_shoe.visual(
        Cylinder(radius=PIN_RADIUS, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.044)),
        material=pin_metal,
        name="shoe_pin",
    )
    end_shoe.visual(
        Cylinder(radius=0.0135, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.056)),
        material=pin_metal,
        name="shoe_rivet_head",
    )

    model.articulation(
        "root_to_short",
        ArticulationType.REVOLUTE,
        parent=root,
        child=short_bar,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=3.0, lower=-0.35, upper=0.85),
    )
    model.articulation(
        "short_to_long",
        ArticulationType.REVOLUTE,
        parent=short_bar,
        child=long_bar,
        origin=Origin(xyz=(SHORT_LEN, 0.0, 0.0), rpy=(0.0, 0.0, math.pi)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=3.0, lower=-2.95, upper=0.12),
    )
    model.articulation(
        "long_to_outer",
        ArticulationType.REVOLUTE,
        parent=long_bar,
        child=outer_bar,
        origin=Origin(xyz=(LONG_LEN, 0.0, 0.0), rpy=(0.0, 0.0, math.pi)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=3.0, lower=-2.95, upper=0.12),
    )
    model.articulation(
        "outer_to_shoe",
        ArticulationType.FIXED,
        parent=outer_bar,
        child=end_shoe,
        origin=Origin(xyz=(OUTER_LEN, 0.0, 0.0)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    root = object_model.get_part("root_bracket")
    short_bar = object_model.get_part("short_bar")
    long_bar = object_model.get_part("long_bar")
    outer_bar = object_model.get_part("outer_bar")
    shoe = object_model.get_part("end_shoe")

    root_to_short = object_model.get_articulation("root_to_short")
    short_to_long = object_model.get_articulation("short_to_long")
    long_to_outer = object_model.get_articulation("long_to_outer")

    ctx.allow_overlap(
        root,
        short_bar,
        elem_a="root_pin",
        elem_b="short_bar_plate",
        reason="The root rivet shaft is intentionally represented as captured through the first flat link.",
    )
    ctx.allow_overlap(
        long_bar,
        short_bar,
        elem_a="long_bar_plate",
        elem_b="middle_pin",
        reason="The middle rivet shaft is intentionally represented through the next stacked link plate.",
    )
    ctx.allow_overlap(
        long_bar,
        outer_bar,
        elem_a="outer_pin",
        elem_b="outer_bar_plate",
        reason="The outer rivet shaft is intentionally represented through the top stacked link plate.",
    )
    ctx.allow_overlap(
        shoe,
        outer_bar,
        elem_a="shoe_pin",
        elem_b="outer_bar_plate",
        reason="The shoe rivet shaft is intentionally represented through the end link plate.",
    )

    for joint in (root_to_short, short_to_long, long_to_outer):
        ctx.check(
            f"{joint.name} is a planar revolute joint",
            joint.articulation_type == ArticulationType.REVOLUTE
            and tuple(round(v, 6) for v in (joint.axis or ())) == (0.0, 0.0, 1.0),
            details=f"type={joint.articulation_type}, axis={joint.axis}",
        )

    with ctx.pose({root_to_short: 0.0, short_to_long: 0.0, long_to_outer: 0.0}):
        ctx.expect_overlap(
            short_bar,
            long_bar,
            axes="xy",
            min_overlap=0.030,
            elem_a="short_bar_plate",
            elem_b="long_bar_plate",
            name="short and long bars stack in folded projection",
        )
        ctx.expect_overlap(
            long_bar,
            outer_bar,
            axes="xy",
            min_overlap=0.030,
            elem_a="long_bar_plate",
            elem_b="outer_bar_plate",
            name="long and outer bars stack in folded projection",
        )
        ctx.expect_gap(
            long_bar,
            short_bar,
            axis="z",
            min_gap=0.002,
            max_gap=0.008,
            positive_elem="long_bar_plate",
            negative_elem="short_bar_plate",
            name="folded middle plate has spacer clearance",
        )
        ctx.expect_gap(
            outer_bar,
            long_bar,
            axis="z",
            min_gap=0.002,
            max_gap=0.008,
            positive_elem="outer_bar_plate",
            negative_elem="long_bar_plate",
            name="folded top plate has spacer clearance",
        )
        ctx.expect_overlap(
            root,
            short_bar,
            axes="z",
            min_overlap=0.004,
            elem_a="root_pin",
            elem_b="short_bar_plate",
            name="root rivet shaft passes through first link thickness",
        )
        ctx.expect_overlap(
            long_bar,
            short_bar,
            axes="z",
            min_overlap=0.004,
            elem_a="long_bar_plate",
            elem_b="middle_pin",
            name="middle rivet shaft passes through long link thickness",
        )
        ctx.expect_overlap(
            long_bar,
            outer_bar,
            axes="z",
            min_overlap=0.004,
            elem_a="outer_pin",
            elem_b="outer_bar_plate",
            name="outer rivet shaft passes through outer link thickness",
        )
        ctx.expect_overlap(
            shoe,
            outer_bar,
            axes="z",
            min_overlap=0.004,
            elem_a="shoe_pin",
            elem_b="outer_bar_plate",
            name="shoe rivet shaft passes through end link thickness",
        )
        folded_shoe_pos = ctx.part_world_position(shoe)

    with ctx.pose({root_to_short: 0.10, short_to_long: -2.95, long_to_outer: -2.95}):
        extended_shoe_pos = ctx.part_world_position(shoe)
        ctx.expect_origin_distance(
            root,
            shoe,
            axes="xy",
            min_dist=0.55,
            name="extended stay reaches outward from the fixed bracket",
        )

    ctx.check(
        "extended shoe moves farther from root than folded shoe",
        folded_shoe_pos is not None
        and extended_shoe_pos is not None
        and extended_shoe_pos[0] > folded_shoe_pos[0] + 0.35,
        details=f"folded={folded_shoe_pos}, extended={extended_shoe_pos}",
    )

    return ctx.report()


object_model = build_object_model()
