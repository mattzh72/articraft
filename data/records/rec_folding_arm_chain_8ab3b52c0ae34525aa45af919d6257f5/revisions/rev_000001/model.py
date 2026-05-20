from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BAR_LENGTH = 0.38
BAR_WIDTH = 0.078
BAR_PLATE_THICKNESS = 0.010
BAR_BOSS_HEIGHT = 0.0035
BAR_HOLE_RADIUS = 0.020
BAR_BOSS_RADIUS = 0.033
LAYER_OFFSET = 0.016
PIN_RADIUS = BAR_HOLE_RADIUS
PIN_LENGTH = 0.112
PIN_CAP_RADIUS = 0.032
PIN_CAP_LENGTH = 0.008


def _capsule_plate_shape(length: float, width: float, thickness: float, hole_radius: float, boss_radius: float, boss_height: float):
    """Flat-bar plate, authored in XY and later rotated into a vertical linkage plane."""
    r = width / 2.0
    total_depth = thickness + 2.0 * boss_height
    outline = (
        cq.Workplane("XY")
        .moveTo(0.0, -r)
        .lineTo(length, -r)
        .threePointArc((length + r, 0.0), (length, r))
        .lineTo(0.0, r)
        .threePointArc((-r, 0.0), (0.0, -r))
        .close()
        .extrude(thickness)
    )
    top_bosses = (
        cq.Workplane("XY")
        .pushPoints([(0.0, 0.0), (length, 0.0)])
        .circle(boss_radius)
        .extrude(boss_height)
        .translate((0.0, 0.0, thickness))
    )
    bottom_bosses = (
        cq.Workplane("XY")
        .pushPoints([(0.0, 0.0), (length, 0.0)])
        .circle(boss_radius)
        .extrude(boss_height)
        .translate((0.0, 0.0, -boss_height))
    )
    hole_cutters = (
        cq.Workplane("XY")
        .pushPoints([(0.0, 0.0), (length, 0.0)])
        .circle(hole_radius)
        .extrude(total_depth + 0.020)
        .translate((0.0, 0.0, -boss_height - 0.010))
    )
    return (
        outline.union(top_bosses)
        .union(bottom_bosses)
        .cut(hole_cutters)
        .translate((0.0, 0.0, -thickness / 2.0))
    )


def _base_cheek_shape():
    """One side plate of the ground clevis, with a real pin clearance hole."""
    hole_radius = BAR_HOLE_RADIUS
    thickness = 0.008
    cheek = (
        cq.Workplane("XY")
        .moveTo(-0.090, -0.150)
        .lineTo(0.070, -0.150)
        .lineTo(0.070, 0.030)
        .threePointArc((0.0, 0.080), (-0.070, 0.030))
        .lineTo(-0.090, -0.150)
        .close()
        .extrude(thickness)
    )
    cutter = (
        cq.Workplane("XY")
        .circle(hole_radius)
        .extrude(thickness + 0.020)
        .translate((0.0, 0.0, -0.010))
    )
    return (
        cheek.cut(cutter)
        .translate((0.0, 0.0, -thickness / 2.0))
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="folding_arm_chain")

    safety_yellow = model.material("powder_coated_yellow", rgba=(0.96, 0.68, 0.08, 1.0))
    deep_blue = model.material("painted_blue", rgba=(0.05, 0.17, 0.38, 1.0))
    dark_steel = model.material("dark_blued_steel", rgba=(0.08, 0.09, 0.10, 1.0))
    zinc = model.material("zinc_plated_pins", rgba=(0.72, 0.72, 0.68, 1.0))
    rubber = model.material("black_rubber_feet", rgba=(0.015, 0.014, 0.012, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.36, 0.24, 0.024)),
        origin=Origin(xyz=(-0.060, 0.0, -0.162)),
        material=dark_steel,
        name="floor_plate",
    )
    base.visual(
        Box((0.18, 0.16, 0.030)),
        origin=Origin(xyz=(-0.040, 0.0, -0.135)),
        material=dark_steel,
        name="pedestal_block",
    )
    base.visual(
        Box((0.16, 0.095, 0.070)),
        origin=Origin(xyz=(-0.020, 0.0, -0.085)),
        material=dark_steel,
        name="upright_web",
    )
    cheek_mesh = mesh_from_cadquery(_base_cheek_shape(), "base_cheek", tolerance=0.0008)
    for side, y in (("near", -0.050), ("far", 0.050)):
        base.visual(
            cheek_mesh,
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_steel,
            name=f"{side}_cheek",
        )
    for i, y in enumerate((-0.085, 0.085)):
        base.visual(
            Box((0.055, 0.030, 0.010)),
            origin=Origin(xyz=(-0.185, y, -0.179)),
            material=rubber,
            name=f"foot_{i}",
        )
    for i, y in enumerate((-0.085, 0.085), start=2):
        base.visual(
            Box((0.055, 0.030, 0.010)),
            origin=Origin(xyz=(0.065, y, -0.179)),
            material=rubber,
            name=f"foot_{i}",
        )

    bar_meshes = [
        mesh_from_cadquery(
            _capsule_plate_shape(
                BAR_LENGTH,
                BAR_WIDTH,
                BAR_PLATE_THICKNESS,
                BAR_HOLE_RADIUS,
                BAR_BOSS_RADIUS,
                BAR_BOSS_HEIGHT,
            ),
            f"link_{i}_bar",
            tolerance=0.0008,
            angular_tolerance=0.08,
        )
        for i in range(5)
    ]

    links = []
    for i, mesh in enumerate(bar_meshes):
        link = model.part(f"link_{i}")
        y_layer = -LAYER_OFFSET if i % 2 == 0 else LAYER_OFFSET
        link.visual(
            mesh,
            origin=Origin(xyz=(0.0, y_layer, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=safety_yellow if i % 2 == 0 else deep_blue,
            name="bar",
        )
        links.append(link)

    pin_parents = [base] + links[:-1]
    pins = []
    for i, parent in enumerate(pin_parents):
        pin = model.part(f"pin_{i}")
        pin.visual(
            Cylinder(radius=PIN_RADIUS, length=PIN_LENGTH),
            origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=zinc,
            name="shaft",
        )
        pin.visual(
            Cylinder(radius=PIN_CAP_RADIUS, length=PIN_CAP_LENGTH),
            origin=Origin(
                xyz=(0.0, PIN_LENGTH / 2.0 + PIN_CAP_LENGTH / 2.0 - 0.001, 0.0),
                rpy=(-math.pi / 2.0, 0.0, 0.0),
            ),
            material=zinc,
            name="cap_0",
        )
        pin.visual(
            Cylinder(radius=PIN_CAP_RADIUS, length=PIN_CAP_LENGTH),
            origin=Origin(
                xyz=(0.0, -PIN_LENGTH / 2.0 - PIN_CAP_LENGTH / 2.0 + 0.001, 0.0),
                rpy=(-math.pi / 2.0, 0.0, 0.0),
            ),
            material=zinc,
            name="cap_1",
        )
        pins.append(pin)

        if i == 0:
            pin_origin = Origin()
        else:
            pin_origin = Origin(xyz=(BAR_LENGTH, 0.0, 0.0))
        model.articulation(
            f"{parent.name}_to_pin_{i}",
            ArticulationType.FIXED,
            parent=parent,
            child=pin,
            origin=pin_origin,
        )

    rest_bends = [-0.42, 1.05, -1.12, 1.02, -0.82]
    model.articulation(
        "base_to_link_0",
        ArticulationType.REVOLUTE,
        parent=base,
        child=links[0],
        origin=Origin(rpy=(0.0, rest_bends[0], 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=70.0, velocity=1.4, lower=-0.70, upper=0.75),
        motion_properties=MotionProperties(damping=0.06, friction=0.04),
    )
    for i in range(1, len(links)):
        model.articulation(
            f"link_{i-1}_to_link_{i}",
            ArticulationType.REVOLUTE,
            parent=links[i - 1],
            child=links[i],
            origin=Origin(xyz=(BAR_LENGTH, 0.0, 0.0), rpy=(0.0, rest_bends[i], 0.0)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=55.0, velocity=1.6, lower=-0.80, upper=0.80),
            motion_properties=MotionProperties(damping=0.05, friction=0.035),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    links = [object_model.get_part(f"link_{i}") for i in range(5)]
    pins = [object_model.get_part(f"pin_{i}") for i in range(5)]
    base = object_model.get_part("base")
    revolute_joints = [object_model.get_articulation("base_to_link_0")] + [
        object_model.get_articulation(f"link_{i-1}_to_link_{i}") for i in range(1, 5)
    ]

    for cheek in ("near_cheek", "far_cheek"):
        ctx.allow_overlap(
            base,
            pins[0],
            elem_a=cheek,
            elem_b="shaft",
            reason="The base clevis pin is intentionally captured through a modeled clearance hole; the shaft is a bearing-contact proxy.",
        )
        ctx.expect_overlap(
            base,
            pins[0],
            axes="xz",
            elem_a=cheek,
            elem_b="shaft",
            min_overlap=0.030,
            name=f"base pin passes through {cheek}",
        )
    link_pin_pairs = [(0, 0), (0, 1), (1, 1), (1, 2), (2, 2), (2, 3), (3, 3), (3, 4), (4, 4)]
    for link_i, pin_i in link_pin_pairs:
        ctx.allow_overlap(
            links[link_i],
            pins[pin_i],
            elem_a="bar",
            elem_b="shaft",
            reason="Each pivot shaft is intentionally captured through the flat-bar eye as a revolute-bearing proxy.",
        )
        ctx.expect_overlap(
            links[link_i],
            pins[pin_i],
            axes="xz",
            elem_a="bar",
            elem_b="shaft",
            min_overlap=0.030,
            name=f"pin_{pin_i}_passes_through_link_{link_i}",
        )

    ctx.check(
        "five revolute pivots",
        all(j.articulation_type == ArticulationType.REVOLUTE for j in revolute_joints),
        details="Every flat-bar segment should be carried by its own revolute pivot.",
    )
    ctx.expect_origin_gap(links[-1], base, axis="x", min_gap=1.10, name="folded chain reaches outward")
    for i in range(1, 5):
        ctx.expect_origin_distance(
            links[i - 1],
            links[i],
            axes="xz",
            min_dist=BAR_LENGTH - 0.002,
            max_dist=BAR_LENGTH + 0.002,
            name=f"link_{i}_pivot_spacing",
        )

    tip = links[-1]
    rest_tip = ctx.part_world_position(tip)
    with ctx.pose(
        {
            revolute_joints[1]: -0.45,
            revolute_joints[2]: 0.45,
            revolute_joints[3]: -0.40,
            revolute_joints[4]: 0.35,
        }
    ):
        deployed_tip = ctx.part_world_position(tip)
        ctx.expect_origin_gap(tip, base, axis="x", min_gap=1.25, name="deployed pose extends farther")

    ctx.check(
        "articulation changes tip position",
        rest_tip is not None and deployed_tip is not None and deployed_tip[0] > rest_tip[0] + 0.08,
        details=f"rest_tip={rest_tip}, deployed_tip={deployed_tip}",
    )

    return ctx.report()


object_model = build_object_model()
