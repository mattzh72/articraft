from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


HINGE_HEIGHT = 0.060
FIXED_LEAF_WIDTH = 0.026
MOVING_LEAF_WIDTH = 0.024
LEAF_THICKNESS = 0.0026
MOVING_LEAF_OFFSET = 0.0044

BARREL_RADIUS = 0.0042
PIN_RADIUS = 0.00155
PIN_CLEAR_RADIUS = 0.0019
KNUCKLE_GAP = 0.0005
MOVING_KNUCKLE_LENGTH = 0.023
FIXED_KNUCKLE_LENGTH = (
    HINGE_HEIGHT - MOVING_KNUCKLE_LENGTH - (2.0 * KNUCKLE_GAP)
) / 2.0
FIXED_KNUCKLE_CENTER_Z = (
    (MOVING_KNUCKLE_LENGTH / 2.0) + KNUCKLE_GAP + (FIXED_KNUCKLE_LENGTH / 2.0)
)

PLATE_TO_BARREL_OVERLAP = 0.0006
MOVING_BRIDGE_START_X = 0.0018
MOVING_BRIDGE_END_X = 0.0110
MOVING_MAIN_INNER_X = 0.0103
MOUNT_HOLE_DIAMETER = 0.0048
HOLE_Z_OFFSET = 0.017
LEAF_CORNER_RADIUS = 0.0012
BRIDGE_EDGE_RADIUS = 0.0008


def _cyl_segment(
    outer_radius: float,
    length: float,
    center_z: float,
    *,
    inner_radius: float | None = None,
) -> cq.Workplane:
    shape = cq.Workplane("XY").circle(outer_radius)
    if inner_radius is not None and inner_radius > 0.0:
        shape = shape.circle(inner_radius)
    return shape.extrude(length).translate((0.0, 0.0, center_z - (length / 2.0)))


def _hole_cutter(points: list[tuple[float, float]]) -> cq.Workplane:
    return (
        cq.Workplane("XZ")
        .pushPoints(points)
        .circle(MOUNT_HOLE_DIAMETER / 2.0)
        .extrude(0.020, both=True)
    )


def _fixed_leaf_body_shape() -> cq.Workplane:
    center_x = -(BARREL_RADIUS + (FIXED_LEAF_WIDTH / 2.0) - PLATE_TO_BARREL_OVERLAP)
    body = (
        cq.Workplane("XY")
        .box(FIXED_LEAF_WIDTH, LEAF_THICKNESS, HINGE_HEIGHT)
        .edges("|Z")
        .fillet(LEAF_CORNER_RADIUS)
    )
    body = body.translate((center_x, 0.0, 0.0))
    body = body.cut(_hole_cutter([(center_x, -HOLE_Z_OFFSET), (center_x, HOLE_Z_OFFSET)]))
    return body


def _fixed_knuckle_shape() -> cq.Workplane:
    pin = (
        cq.Workplane("XY")
        .circle(PIN_RADIUS)
        .extrude(HINGE_HEIGHT)
        .translate((0.0, 0.0, -(HINGE_HEIGHT / 2.0)))
    )
    top_knuckle = _cyl_segment(
        BARREL_RADIUS,
        FIXED_KNUCKLE_LENGTH,
        FIXED_KNUCKLE_CENTER_Z,
    )
    bottom_knuckle = _cyl_segment(
        BARREL_RADIUS,
        FIXED_KNUCKLE_LENGTH,
        -FIXED_KNUCKLE_CENTER_Z,
    )
    return pin.union(top_knuckle).union(bottom_knuckle)


def _moving_leaf_main_shape() -> cq.Workplane:
    center_x = MOVING_MAIN_INNER_X + (MOVING_LEAF_WIDTH / 2.0)
    main_leaf = (
        cq.Workplane("XY")
        .box(MOVING_LEAF_WIDTH, LEAF_THICKNESS, HINGE_HEIGHT)
        .edges("|Z")
        .fillet(LEAF_CORNER_RADIUS)
    )
    main_leaf = main_leaf.translate((center_x, MOVING_LEAF_OFFSET, 0.0))
    main_leaf = main_leaf.cut(
        _hole_cutter([(center_x, -HOLE_Z_OFFSET), (center_x, HOLE_Z_OFFSET)])
    )
    return main_leaf


def _moving_leaf_bridge_shape() -> cq.Workplane:
    half_t = LEAF_THICKNESS / 2.0
    points = [
        (MOVING_BRIDGE_START_X, -half_t),
        (0.0085, -half_t),
        (MOVING_BRIDGE_END_X, MOVING_LEAF_OFFSET - half_t),
        (MOVING_BRIDGE_END_X, MOVING_LEAF_OFFSET + half_t),
        (0.0075, half_t),
        (MOVING_BRIDGE_START_X, half_t),
    ]
    return (
        cq.Workplane("XY")
        .polyline(points)
        .close()
        .extrude(HINGE_HEIGHT)
        .edges("|Z")
        .fillet(BRIDGE_EDGE_RADIUS)
        .translate((0.0, 0.0, -(HINGE_HEIGHT / 2.0)))
    )


def _moving_knuckle_shape() -> cq.Workplane:
    return _cyl_segment(
        BARREL_RADIUS,
        MOVING_KNUCKLE_LENGTH,
        0.0,
        inner_radius=PIN_CLEAR_RADIUS,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="offset_leaf_enclosure_hinge")

    steel = model.material("zinc_plated_steel", rgba=(0.73, 0.75, 0.79, 1.0))

    fixed_leaf = model.part("fixed_leaf")
    fixed_leaf.visual(
        mesh_from_cadquery(_fixed_leaf_body_shape(), "fixed_leaf_body"),
        material=steel,
        name="fixed_main_leaf",
    )
    fixed_leaf.visual(
        mesh_from_cadquery(_fixed_knuckle_shape(), "fixed_leaf_knuckles"),
        material=steel,
        name="fixed_knuckles",
    )

    moving_leaf = model.part("moving_leaf")
    moving_leaf.visual(
        mesh_from_cadquery(_moving_leaf_main_shape(), "moving_leaf_main"),
        material=steel,
        name="moving_main_leaf",
    )
    moving_leaf.visual(
        mesh_from_cadquery(_moving_leaf_bridge_shape(), "moving_leaf_bridge"),
        material=steel,
        name="moving_offset_bridge",
    )
    moving_leaf.visual(
        mesh_from_cadquery(_moving_knuckle_shape(), "moving_leaf_knuckle"),
        material=steel,
        name="moving_knuckle",
    )

    model.articulation(
        "leaf_hinge",
        ArticulationType.REVOLUTE,
        parent=fixed_leaf,
        child=moving_leaf,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.0,
            lower=0.0,
            upper=1.95,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    fixed_leaf = object_model.get_part("fixed_leaf")
    moving_leaf = object_model.get_part("moving_leaf")
    hinge = object_model.get_articulation("leaf_hinge")

    ctx.check("fixed leaf exists", fixed_leaf is not None, details="")
    ctx.check("moving leaf exists", moving_leaf is not None, details="")
    ctx.check("hinge articulation exists", hinge is not None, details="")

    with ctx.pose({hinge: 0.0}):
        ctx.expect_gap(
            moving_leaf,
            fixed_leaf,
            axis="y",
            positive_elem="moving_main_leaf",
            negative_elem="fixed_main_leaf",
            min_gap=0.0015,
            max_gap=0.0023,
            name="offset moving leaf stands proud of the fixed leaf",
        )
        ctx.expect_overlap(
            moving_leaf,
            fixed_leaf,
            axes="z",
            elem_a="moving_main_leaf",
            elem_b="fixed_main_leaf",
            min_overlap=0.055,
            name="leaf plates share the hinge height",
        )
        ctx.expect_contact(
            moving_leaf,
            fixed_leaf,
            elem_a="moving_knuckle",
            elem_b="fixed_knuckles",
            contact_tol=0.00045,
            name="compact barrel keeps the knuckles tightly nested around the pin",
        )

    def _aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None):
        if aabb is None:
            return None
        return tuple((aabb[0][i] + aabb[1][i]) / 2.0 for i in range(3))

    closed_center = _aabb_center(
        ctx.part_element_world_aabb(moving_leaf, elem="moving_main_leaf")
    )
    with ctx.pose({hinge: 1.35}):
        open_center = _aabb_center(
            ctx.part_element_world_aabb(moving_leaf, elem="moving_main_leaf")
        )

    ctx.check(
        "moving leaf opens outward with positive rotation",
        closed_center is not None
        and open_center is not None
        and open_center[1] > closed_center[1] + 0.012,
        details=f"closed_center={closed_center}, open_center={open_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
