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
    mesh_from_geometry,
    wire_from_points,
)


def _tube_mesh(points, name: str, radius: float = 0.006):
    """One-piece bent-wire/tube mesh for low-cost rack frames."""
    return mesh_from_geometry(
        wire_from_points(
            points,
            radius=radius,
            radial_segments=14,
            closed_path=False,
            cap_ends=True,
            corner_mode="fillet",
            corner_radius=0.018,
            corner_segments=6,
        ),
        name,
    )


def _y_cylinder_origin(x: float, y: float, z: float) -> Origin:
    return Origin(xyz=(x, y, z), rpy=(-math.pi / 2.0, 0.0, 0.0))


def _add_wing_geometry(wing, metal: Material, plastic: Material, *, index: int) -> None:
    """A single bent wire wing with integrated hanging rails and hinge links."""
    # The child part frame sits on the hinge line.  The grid extends along local +X.
    rail_path = [
        (0.045, -0.480, 0.000),
        (0.820, -0.480, 0.000),
        (0.820, 0.480, 0.000),
        (0.700, 0.480, 0.000),
        (0.700, -0.480, 0.000),
        (0.580, -0.480, 0.000),
        (0.580, 0.480, 0.000),
        (0.460, 0.480, 0.000),
        (0.460, -0.480, 0.000),
        (0.340, -0.480, 0.000),
        (0.340, 0.480, 0.000),
        (0.220, 0.480, 0.000),
        (0.220, -0.480, 0.000),
        (0.100, -0.480, 0.000),
        (0.100, 0.480, 0.000),
        (0.045, 0.480, 0.000),
    ]
    wing.visual(
        _tube_mesh(rail_path, f"wing_wire_{index}", radius=0.0055),
        material=metal,
        name="wing_wire",
    )

    # Three molded hinge links snap over the wire and carry simple barrel knuckles.
    # The middle names are literals because the exact tests target that capture.
    wing.visual(
        Box((0.052, 0.070, 0.030)),
        origin=Origin(xyz=(0.026, 0.000, 0.000)),
        material=plastic,
        name="hinge_link_mid",
    )
    wing.visual(
        Cylinder(radius=0.012, length=0.050),
        origin=_y_cylinder_origin(0.000, 0.000, 0.000),
        material=metal,
        name="hinge_barrel_mid",
    )
    for suffix, y in (("rear", -0.420), ("front", 0.420)):
        wing.visual(
            Box((0.052, 0.070, 0.030)),
            origin=Origin(xyz=(0.026, y, 0.000)),
            material=plastic,
            name=f"hinge_link_{suffix}",
        )
        wing.visual(
            Cylinder(radius=0.012, length=0.050),
            origin=_y_cylinder_origin(0.000, y, 0.000),
            material=metal,
            name=f"hinge_barrel_{suffix}",
        )

    # Small bridges from every hinge link into the first rail make the snap-on
    # clips read as captured on the wire rather than as loose plastic islands.
    for suffix, y in (("rear", -0.420), ("mid", 0.000), ("front", 0.420)):
        wing.visual(
            Box((0.110, 0.020, 0.020)),
            origin=Origin(xyz=(0.075, y, 0.000)),
            material=plastic,
            name=f"hinge_bridge_{suffix}",
        )

    # Low-cost clipped end caps double as rounded hand stops.
    for suffix, y in (("rear", -0.480), ("front", 0.480)):
        wing.visual(
            Box((0.060, 0.035, 0.030)),
            origin=Origin(xyz=(0.820, y, 0.000)),
            material=plastic,
            name=f"end_cap_{suffix}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="simple_foldable_drying_rack")

    white_powder_coat = model.material("white_powder_coat", rgba=(0.90, 0.92, 0.90, 1.0))
    cool_gray_plastic = model.material("cool_gray_plastic", rgba=(0.34, 0.38, 0.40, 1.0))
    dark_screw = model.material("dark_screw_heads", rgba=(0.05, 0.05, 0.045, 1.0))

    base = model.part("base_frame")

    # One continuous side/leg tube: simple bent/stamped support with four feet.
    leg_path = [
        (-0.450, -0.600, 0.040),
        (-0.300, -0.520, 0.760),
        (-0.300, 0.520, 0.760),
        (-0.450, 0.600, 0.040),
        (0.450, 0.600, 0.040),
        (0.300, 0.520, 0.760),
        (0.300, -0.520, 0.760),
        (0.450, -0.600, 0.040),
        (-0.450, -0.600, 0.040),
    ]
    base.visual(
        _tube_mesh(leg_path, "base_leg_wire", radius=0.008),
        material=white_powder_coat,
        name="leg_wire",
    )

    # Central fixed drying deck; it reuses the same side rails and avoids
    # separate loose rungs by being a single serpentine wire form.
    deck_path = [
        (-0.300, -0.520, 0.760),
        (0.300, -0.520, 0.760),
        (0.300, -0.350, 0.760),
        (-0.300, -0.350, 0.760),
        (-0.300, -0.180, 0.760),
        (0.300, -0.180, 0.760),
        (0.300, -0.010, 0.760),
        (-0.300, -0.010, 0.760),
        (-0.300, 0.160, 0.760),
        (0.300, 0.160, 0.760),
        (0.300, 0.330, 0.760),
        (-0.300, 0.330, 0.760),
        (-0.300, 0.520, 0.760),
        (0.300, 0.520, 0.760),
    ]
    base.visual(
        _tube_mesh(deck_path, "base_deck_wire", radius=0.006),
        material=white_powder_coat,
        name="deck_wire",
    )

    # Rubbery feet snap over the tube ends and provide a wide stable stance.
    for ix, x in enumerate((-0.450, 0.450)):
        for iy, y in enumerate((-0.600, 0.600)):
            base.visual(
                Box((0.160, 0.060, 0.035)),
                origin=Origin(xyz=(x, y, 0.018)),
                material=cool_gray_plastic,
                name=f"foot_{ix}_{iy}",
            )

    # Identical molded hinge blocks are clamped to the two side rails.  Their
    # flat shelves are the visible open/fold stop faces, while the dark bosses
    # read as bolt/snap interfaces for assembly order.
    for side_index, x in enumerate((-0.3205, 0.3205)):
        mid_block_name = "hinge_block_0_mid" if side_index == 0 else "hinge_block_1_mid"
        mid_stop_name = "fold_stop_0_mid" if side_index == 0 else "fold_stop_1_mid"
        mid_bolt_name = "bolt_head_0_mid" if side_index == 0 else "bolt_head_1_mid"
        base.visual(
            Box((0.035, 0.095, 0.042)),
            origin=Origin(xyz=(x, 0.000, 0.760)),
            material=cool_gray_plastic,
            name=mid_block_name,
        )
        base.visual(
            Box((0.050, 0.055, 0.014)),
            origin=Origin(xyz=(x, 0.000, 0.788)),
            material=cool_gray_plastic,
            name=mid_stop_name,
        )
        base.visual(
            Cylinder(radius=0.006, length=0.004),
            origin=Origin(xyz=(x, 0.000, 0.797)),
            material=dark_screw,
            name=mid_bolt_name,
        )
        for suffix, y in (("rear", -0.420), ("front", 0.420)):
            base.visual(
                Box((0.035, 0.095, 0.042)),
                origin=Origin(xyz=(x, y, 0.760)),
                material=cool_gray_plastic,
                name=f"hinge_block_{side_index}_{suffix}",
            )
            base.visual(
                Box((0.050, 0.055, 0.014)),
                origin=Origin(xyz=(x, y, 0.788)),
                material=cool_gray_plastic,
                name=f"fold_stop_{side_index}_{suffix}",
            )
            base.visual(
                Cylinder(radius=0.006, length=0.004),
                origin=Origin(xyz=(x, y, 0.797)),
                material=dark_screw,
                name=f"bolt_head_{side_index}_{suffix}",
            )

    wing_0 = model.part("wing_0")
    _add_wing_geometry(wing_0, white_powder_coat, cool_gray_plastic, index=0)

    wing_1 = model.part("wing_1")
    _add_wing_geometry(wing_1, white_powder_coat, cool_gray_plastic, index=1)

    # Both wing frames are identical extruded/bent parts.  The left-side joint
    # rotates the same local wing by 180 degrees about Z so local +X points
    # outward from its hinge line.
    model.articulation(
        "wing_hinge_0",
        ArticulationType.REVOLUTE,
        parent=base,
        child=wing_0,
        origin=Origin(xyz=(-0.350, 0.000, 0.760), rpy=(0.0, 0.0, math.pi)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.6, lower=0.0, upper=1.82),
    )
    model.articulation(
        "wing_hinge_1",
        ArticulationType.REVOLUTE,
        parent=base,
        child=wing_1,
        origin=Origin(xyz=(0.350, 0.000, 0.760), rpy=(0.0, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.6, lower=0.0, upper=1.82),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_frame")
    wing_0 = object_model.get_part("wing_0")
    wing_1 = object_model.get_part("wing_1")
    hinge_0 = object_model.get_articulation("wing_hinge_0")
    hinge_1 = object_model.get_articulation("wing_hinge_1")

    # The hinge barrels sit in close, visible clearance to the molded clamp
    # blocks rather than floating out in space.
    ctx.expect_gap(
        base,
        wing_0,
        axis="x",
        positive_elem="hinge_block_0_mid",
        negative_elem="hinge_barrel_mid",
        max_gap=0.002,
        max_penetration=0.0005,
        name="wing 0 hinge barrel is captured beside the base clamp",
    )
    ctx.expect_gap(
        wing_1,
        base,
        axis="x",
        positive_elem="hinge_barrel_mid",
        negative_elem="hinge_block_1_mid",
        max_gap=0.002,
        max_penetration=0.0005,
        name="wing 1 hinge barrel is captured beside the base clamp",
    )

    # In the open stop position the two wings are flat and overlap the base in
    # rack length, forming one usable drying surface.
    ctx.expect_overlap(
        wing_0,
        base,
        axes="y",
        min_overlap=0.85,
        elem_a="wing_wire",
        elem_b="deck_wire",
        name="wing 0 rails align with base drying length",
    )
    ctx.expect_overlap(
        wing_1,
        base,
        axes="y",
        min_overlap=0.85,
        elem_a="wing_wire",
        elem_b="deck_wire",
        name="wing 1 rails align with base drying length",
    )

    # At the fold stop the free edges rise well above the central rack, proving
    # a clear hinge line and a realistic stopped fold angle.
    with ctx.pose({hinge_0: 1.82, hinge_1: 1.82}):
        aabb_0 = ctx.part_element_world_aabb(wing_0, elem="wing_wire")
        aabb_1 = ctx.part_element_world_aabb(wing_1, elem="wing_wire")
        ctx.check(
            "wing 0 folds upward to stop",
            aabb_0 is not None and aabb_0[1][2] > 1.45,
            details=f"folded AABB={aabb_0}",
        )
        ctx.check(
            "wing 1 folds upward to stop",
            aabb_1 is not None and aabb_1[1][2] > 1.45,
            details=f"folded AABB={aabb_1}",
        )

    return ctx.report()


object_model = build_object_model()
