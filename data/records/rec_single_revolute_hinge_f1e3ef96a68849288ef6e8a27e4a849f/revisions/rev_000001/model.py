from __future__ import annotations

from math import pi

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


HINGE_LENGTH = 0.320
LEAF_WIDTH = 0.120
LEAF_THICKNESS = 0.008
BARREL_OUTER_RADIUS = 0.018
PIN_RADIUS = 0.008
# Parent-side knuckles grip the fixed pin with a tiny hidden press fit, while
# child-side knuckles have a visibly larger running bore for free rotation.
PARENT_BORE_RADIUS = 0.0076
CHILD_BORE_RADIUS = 0.0105
KNUCKLE_GAP = 0.004


def _box(center: tuple[float, float, float], size: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(size[0], size[1], size[2]).translate(center)


def _tube_segment(z_min: float, z_max: float, inner_radius: float) -> cq.Workplane:
    length = z_max - z_min
    outer = cq.Workplane("XY").circle(BARREL_OUTER_RADIUS).extrude(length).translate((0.0, 0.0, z_min))
    bore = (
        cq.Workplane("XY")
        .circle(inner_radius)
        .extrude(length + 0.004)
        .translate((0.0, 0.0, z_min - 0.002))
    )
    return outer.cut(bore)


def _knuckle_intervals(for_leaf: int) -> list[tuple[float, float]]:
    segment_length = (HINGE_LENGTH - 4.0 * KNUCKLE_GAP) / 5.0
    z = -HINGE_LENGTH / 2.0
    intervals: list[tuple[float, float]] = []
    for index in range(5):
        z_next = z + segment_length
        if (for_leaf == 0 and index in (0, 2, 4)) or (for_leaf == 1 and index in (1, 3)):
            intervals.append((z, z_next))
        z = z_next + KNUCKLE_GAP
    return intervals


def _leaf_shape(for_leaf: int) -> cq.Workplane:
    """Build one heavy hinge leaf with real alternating hollow knuckles."""
    sign = -1.0 if for_leaf == 0 else 1.0
    bore_radius = PARENT_BORE_RADIUS if for_leaf == 0 else CHILD_BORE_RADIUS
    plate_y = -BARREL_OUTER_RADIUS - LEAF_THICKNESS / 2.0

    # The leaf stops just short of the pin centerline.  Small local curled
    # straps at each knuckle reach under the barrel and physically join the
    # knuckle tubes to the plate without letting the two leaves collide.
    plate_center_x = sign * (LEAF_WIDTH / 2.0 + 0.002)
    leaf = _box(
        (plate_center_x, plate_y, 0.0),
        (LEAF_WIDTH, LEAF_THICKNESS, HINGE_LENGTH),
    )

    # Reinforced hinge-edge rib, as found on heavy-duty leaf hinges.
    rib_center_x = sign * 0.014
    leaf = leaf.union(
        _box(
            (rib_center_x, plate_y + 0.001, 0.0),
            (0.018, LEAF_THICKNESS + 0.002, HINGE_LENGTH),
        )
    )

    for z_min, z_max in _knuckle_intervals(for_leaf):
        leaf = leaf.union(_tube_segment(z_min, z_max, bore_radius))
        leaf = leaf.union(
            _box(
                (sign * 0.004, -BARREL_OUTER_RADIUS - 0.0025, (z_min + z_max) / 2.0),
                (0.020, 0.008, z_max - z_min),
            )
        )

    # Through-bolt holes in a staggered heavy-duty pattern.  They cut through
    # the plate in the thickness direction and leave true visible openings.
    hole_radius = 0.0065
    x_cols = [sign * 0.043, sign * 0.090]
    z_rows = [-0.105, 0.0, 0.105]
    for x in x_cols:
        for z in z_rows:
            cutter = cq.Workplane("XZ").center(x, z).circle(hole_radius).extrude(0.080, both=True)
            leaf = leaf.cut(cutter)

    return leaf.clean()


def _add_leaf_primitives(part, for_leaf: int, metal, dark_recess) -> None:
    """Assemble one leaf from element-sized primitives to preserve knuckle gaps."""
    sign = -1.0 if for_leaf == 0 else 1.0
    plate_y = -BARREL_OUTER_RADIUS - LEAF_THICKNESS / 2.0
    plate_names = ("leaf_0_plate", "leaf_1_plate")
    rib_names = ("leaf_0_edge_rib", "leaf_1_edge_rib")
    knuckle_names = (
        ("knuckle_0_0", "knuckle_0_1", "knuckle_0_2"),
        ("knuckle_1_0", "knuckle_1_1"),
    )
    strap_names = (
        ("knuckle_strap_0_0", "knuckle_strap_0_1", "knuckle_strap_0_2"),
        ("knuckle_strap_1_0", "knuckle_strap_1_1"),
    )

    part.visual(
        Box((LEAF_WIDTH, LEAF_THICKNESS, HINGE_LENGTH)),
        origin=Origin(xyz=(sign * (LEAF_WIDTH / 2.0 + 0.002), plate_y, 0.0)),
        material=metal,
        name=plate_names[for_leaf],
    )
    part.visual(
        Box((0.018, LEAF_THICKNESS + 0.002, HINGE_LENGTH)),
        origin=Origin(xyz=(sign * 0.014, plate_y + 0.001, 0.0)),
        material=metal,
        name=rib_names[for_leaf],
    )

    for index, (z_min, z_max) in enumerate(_knuckle_intervals(for_leaf)):
        length = z_max - z_min
        z_mid = (z_min + z_max) / 2.0
        part.visual(
            mesh_from_cadquery(
                _tube_segment(
                    z_min,
                    z_max,
                    PARENT_BORE_RADIUS if for_leaf == 0 else CHILD_BORE_RADIUS,
                ),
                f"knuckle_{for_leaf}_{index}",
                tolerance=0.0007,
                angular_tolerance=0.06,
            ),
            material=metal,
            name=knuckle_names[for_leaf][index],
        )
        # Short curled strap under each knuckle; it overlaps the leaf plate and
        # the bottom of the barrel but stays on its own side of the centerline.
        part.visual(
            Box((0.0135, 0.008, length)),
            origin=Origin(xyz=(sign * 0.00725, -BARREL_OUTER_RADIUS - 0.0025, z_mid)),
            material=metal,
            name=strap_names[for_leaf][index],
        )

    # Dark, shallow circular seats read as through-bolt/countersink locations
    # while remaining mechanically seated in the leaf plate.
    for col, x in enumerate((sign * 0.043, sign * 0.090)):
        for row, z in enumerate((-0.105, 0.0, 0.105)):
            part.visual(
                Cylinder(radius=0.0067, length=0.002),
                origin=Origin(
                    xyz=(x, -BARREL_OUTER_RADIUS - LEAF_THICKNESS - 0.0002, z),
                    rpy=(pi / 2.0, 0.0, 0.0),
                ),
                material=dark_recess,
                name=f"bolt_recess_{for_leaf}_{col}_{row}",
            )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="heavy_duty_leaf_hinge")

    zinc = model.material("brushed_zinc_steel", rgba=(0.55, 0.57, 0.58, 1.0))
    dark = model.material("shadowed_hole", rgba=(0.03, 0.032, 0.035, 1.0))
    polished = model.material("polished_pin", rgba=(0.78, 0.79, 0.76, 1.0))

    leaf_0 = model.part("leaf_0")
    _add_leaf_primitives(leaf_0, 0, zinc, dark)
    # A real hinge pin is captured by the parent-side knuckles.  It is slightly
    # proud of the barrel and passes freely through the child-side hollow tubes.
    leaf_0.visual(
        Cylinder(radius=PIN_RADIUS, length=HINGE_LENGTH + 0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=polished,
        name="pin_rod",
    )
    leaf_0.visual(
        Cylinder(radius=0.012, length=0.005),
        origin=Origin(xyz=(0.0, 0.0, HINGE_LENGTH / 2.0 + 0.012)),
        material=polished,
        name="pin_head",
    )
    leaf_0.visual(
        Cylinder(radius=0.0105, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, -HINGE_LENGTH / 2.0 - 0.012)),
        material=polished,
        name="pin_peen",
    )

    leaf_1 = model.part("leaf_1")
    _add_leaf_primitives(leaf_1, 1, zinc, dark)

    model.articulation(
        "barrel_axis",
        ArticulationType.REVOLUTE,
        parent=leaf_0,
        child=leaf_1,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=-2.65, upper=2.65, effort=120.0, velocity=2.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    leaf_0 = object_model.get_part("leaf_0")
    leaf_1 = object_model.get_part("leaf_1")
    joint = object_model.get_articulation("barrel_axis")

    ctx.check(
        "single revolute barrel joint",
        len(object_model.articulations) == 1
        and joint.articulation_type == ArticulationType.REVOLUTE
        and tuple(joint.axis) == (0.0, 0.0, 1.0),
        details=f"articulations={object_model.articulations}",
    )
    ctx.expect_overlap(
        leaf_0,
        leaf_1,
        axes="z",
        min_overlap=0.055,
        elem_a="pin_rod",
        elem_b="knuckle_1_0",
        name="pin passes through the lower child knuckle",
    )
    ctx.expect_overlap(
        leaf_0,
        leaf_1,
        axes="z",
        min_overlap=0.055,
        elem_a="pin_rod",
        elem_b="knuckle_1_1",
        name="pin passes through the upper child knuckle",
    )
    ctx.expect_gap(
        leaf_1,
        leaf_0,
        axis="x",
        min_gap=0.001,
        positive_elem="leaf_1_plate",
        negative_elem="leaf_0_plate",
        name="flat leaves are separated across the hinge centerline",
    )

    rest_aabb = ctx.part_world_aabb(leaf_1)
    with ctx.pose({joint: 0.75}):
        turned_aabb = ctx.part_world_aabb(leaf_1)
    ctx.check(
        "leaf rotates around the barrel axis",
        rest_aabb is not None
        and turned_aabb is not None
        and turned_aabb[1][1] > rest_aabb[1][1] + 0.035,
        details=f"rest={rest_aabb}, turned={turned_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
