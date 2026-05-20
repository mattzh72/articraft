from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


PLATE_LENGTH = 0.18
BARREL_LENGTH = 0.08
PLATE_THICKNESS = 0.004
BARREL_RADIUS = 0.009
PLATE_ROOT_OFFSET = 0.006
PLATE_CENTER_Z = -0.0105
HOLE_RADIUS = 0.0045
BARREL_GAP = 0.0
KNuckle_COUNT = 5
PIN_RADIUS = 0.0048


def _box_shape(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Shape:
    sx, sy, sz = size
    cx, cy, cz = center
    return cq.Solid.makeBox(sx, sy, sz, cq.Vector(cx - sx / 2.0, cy - sy / 2.0, cz - sz / 2.0))


def _cylinder_shape(
    radius: float,
    length: float,
    start: tuple[float, float, float],
    direction: tuple[float, float, float],
) -> cq.Shape:
    return cq.Solid.makeCylinder(radius, length, cq.Vector(*start), cq.Vector(*direction))


def _plate_body_shape(sign: float) -> cq.Shape:
    half_width = BARREL_LENGTH / 2.0
    tip_center_x = sign * (PLATE_LENGTH - half_width)
    body_start = sign * PLATE_ROOT_OFFSET
    body_end = sign * (PLATE_LENGTH - half_width)
    body_center_x = 0.5 * (body_start + body_end)
    body_length = abs(body_end - body_start)

    plate = _box_shape(
        (body_length, BARREL_LENGTH, PLATE_THICKNESS),
        (body_center_x, 0.0, PLATE_CENTER_Z),
    )
    tip = _cylinder_shape(
        half_width,
        PLATE_THICKNESS,
        (tip_center_x, 0.0, PLATE_CENTER_Z - PLATE_THICKNESS / 2.0),
        (0.0, 0.0, 1.0),
    )
    plate = plate.fuse(tip)

    hole_positions = (0.042, 0.092, 0.138)
    cut_depth = PLATE_THICKNESS + 0.006
    cut_start_z = PLATE_CENTER_Z - cut_depth / 2.0
    for pos in hole_positions:
        x_pos = sign * pos
        hole = _cylinder_shape(
            HOLE_RADIUS,
            cut_depth,
            (x_pos, 0.0, cut_start_z),
            (0.0, 0.0, 1.0),
        )
        plate = plate.cut(hole)

    return plate


def _knuckle_span_y(index: int, segment_length: float) -> tuple[float, float]:
    start_y = -BARREL_LENGTH / 2.0 + index * (segment_length + BARREL_GAP)
    end_y = start_y + segment_length
    return start_y, end_y


def _knuckle_shape(sign: float, indices: tuple[int, ...], *, hollow: bool) -> cq.Shape:
    segment_length = (BARREL_LENGTH - BARREL_GAP * (KNuckle_COUNT - 1)) / KNuckle_COUNT
    bridge_inner_x = PIN_RADIUS + 0.0008 if hollow else 0.0
    bridge_outer_x = PLATE_ROOT_OFFSET + 0.0048
    bridge_x = bridge_outer_x - bridge_inner_x
    bridge_z = 0.009
    bridge_center_x = sign * (bridge_inner_x + bridge_x / 2.0)
    bridge_center_z = -bridge_z / 2.0

    shape: cq.Shape | None = None
    for index in indices:
        start_y, end_y = _knuckle_span_y(index, segment_length)
        cylinder = _cylinder_shape(
            BARREL_RADIUS,
            segment_length,
            (0.0, start_y, 0.0),
            (0.0, 1.0, 0.0),
        )
        if hollow:
            bore = _cylinder_shape(
                PIN_RADIUS,
                segment_length + 0.0008,
                (0.0, start_y - 0.0004, 0.0),
                (0.0, 1.0, 0.0),
            )
            cylinder = cylinder.cut(bore)
        bridge = _box_shape(
            (bridge_x, segment_length, bridge_z),
            (bridge_center_x, 0.5 * (start_y + end_y), bridge_center_z),
        )
        segment = cylinder.fuse(bridge)
        shape = segment if shape is None else shape.fuse(segment)

    assert shape is not None
    return shape


def _leaf_inertial(sign: float) -> Inertial:
    return Inertial.from_geometry(
        Box((PLATE_LENGTH, BARREL_LENGTH, 0.026)),
        mass=0.65,
        origin=Origin(xyz=(sign * (PLATE_LENGTH / 2.0), 0.0, -0.002)),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="strap_hinge_module")

    model.material("fixed_steel", rgba=(0.33, 0.35, 0.38, 1.0))
    model.material("carried_steel", rgba=(0.42, 0.44, 0.47, 1.0))

    fixed_plate = model.part("fixed_strap_plate")
    fixed_plate.visual(
        mesh_from_cadquery(_plate_body_shape(-1.0), "fixed_plate_panel"),
        material="fixed_steel",
        name="fixed_plate_panel",
    )
    fixed_plate.visual(
        mesh_from_cadquery(_knuckle_shape(-1.0, (0, 2, 4), hollow=False), "fixed_plate_knuckles"),
        material="fixed_steel",
        name="fixed_plate_knuckles",
    )
    fixed_plate.inertial = _leaf_inertial(-1.0)

    carried_plate = model.part("carried_strap_plate")
    carried_plate.visual(
        mesh_from_cadquery(_plate_body_shape(1.0), "carried_plate_panel"),
        material="carried_steel",
        name="carried_plate_panel",
    )
    carried_plate.visual(
        mesh_from_cadquery(_knuckle_shape(1.0, (1, 3), hollow=True), "carried_plate_knuckles"),
        material="carried_steel",
        name="carried_plate_knuckles",
    )
    carried_plate.inertial = _leaf_inertial(1.0)

    model.articulation(
        "barrel_hinge",
        ArticulationType.REVOLUTE,
        parent=fixed_plate,
        child=carried_plate,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=pi, effort=20.0, velocity=2.5),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    fixed_plate = object_model.get_part("fixed_strap_plate")
    carried_plate = object_model.get_part("carried_strap_plate")
    hinge = object_model.get_articulation("barrel_hinge")

    ctx.check(
        "hinge uses barrel axis",
        hinge.axis == (0.0, -1.0, 0.0),
        details=f"axis={hinge.axis}",
    )

    with ctx.pose({hinge: 0.0}):
        ctx.expect_gap(
            carried_plate,
            fixed_plate,
            axis="x",
            positive_elem="carried_plate_panel",
            negative_elem="fixed_plate_panel",
            min_gap=0.011,
            max_gap=0.013,
            name="strap plates stop just outside the barrel seam",
        )
        ctx.expect_overlap(
            carried_plate,
            fixed_plate,
            axes="yz",
            elem_a="carried_plate_knuckles",
            elem_b="fixed_plate_knuckles",
            min_overlap=0.016,
            name="alternating knuckles share a common barrel envelope",
        )

    rest_panel_aabb = ctx.part_element_world_aabb(carried_plate, elem="carried_plate_panel")
    with ctx.pose({hinge: 1.15}):
        opened_panel_aabb = ctx.part_element_world_aabb(carried_plate, elem="carried_plate_panel")

    ctx.check(
        "carried plate swings upward when opened",
        rest_panel_aabb is not None
        and opened_panel_aabb is not None
        and opened_panel_aabb[1][2] > rest_panel_aabb[1][2] + 0.09
        and opened_panel_aabb[1][0] < rest_panel_aabb[1][0] - 0.03,
        details=f"rest={rest_panel_aabb}, opened={opened_panel_aabb}",
    )

    with ctx.pose({hinge: pi}):
        ctx.expect_gap(
            carried_plate,
            fixed_plate,
            axis="z",
            positive_elem="carried_plate_panel",
            negative_elem="fixed_plate_panel",
            min_gap=0.014,
            name="fully folded carried plate stacks above the fixed plate",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
