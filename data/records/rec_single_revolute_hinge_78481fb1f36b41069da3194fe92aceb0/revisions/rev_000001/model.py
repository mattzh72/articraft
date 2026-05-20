from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from typing import Iterable

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


HINGE_LENGTH = 0.165
BACK_LEAF_LENGTH = HINGE_LENGTH
FRONT_LEAF_LENGTH = 0.118

BACK_LEAF_WIDTH = 0.014
FRONT_LEAF_WIDTH = 0.011
LEAF_THICKNESS = 0.0018

BARREL_RADIUS = 0.0036
PIN_RADIUS = 0.0020
PIN_HEAD_RADIUS = 0.0027
PIN_HEAD_LENGTH = 0.0012
CLEAR_LINE_GAP = 0.0006
PIN_CLEARANCE = 0.00035
WEB_OVERLAP = 0.00025
WEB_TOP_Z = -0.0007
PLATE_Z_CENTER = -(BARREL_RADIUS - 0.58 * LEAF_THICKNESS)

BACK_KNUCKLE_LENGTH = 0.034
END_MARGIN = 0.0105
GAP_LENGTH = (HINGE_LENGTH - 2.0 * END_MARGIN - 3.0 * BACK_KNUCKLE_LENGTH) / 2.0
AXIAL_CLEARANCE = 0.002
FRONT_KNUCKLE_LENGTH = GAP_LENGTH - AXIAL_CLEARANCE

FASTENER_HOLE_RADIUS = 0.0017
PLATE_EDGE_FILLET = 0.0007
OPEN_ANGLE = 1.25


def _fuse_all(base: cq.Shape, *shapes: cq.Shape) -> cq.Shape:
    result = base
    for shape in shapes:
        result = result.fuse(shape)
    return result


def _cut_all(base: cq.Shape, *cutters: cq.Shape) -> cq.Shape:
    result = base
    for cutter in cutters:
        result = result.cut(cutter)
    return result


def _y_cylinder(radius: float, length: float, y_min: float) -> cq.Shape:
    return cq.Solid.makeCylinder(radius, length, cq.Vector(0.0, y_min, 0.0), cq.Vector(0.0, 1.0, 0.0))


def _z_cylinder(radius: float, length: float, x: float, y: float, z_min: float) -> cq.Shape:
    return cq.Solid.makeCylinder(radius, length, cq.Vector(x, y, z_min), cq.Vector(0.0, 0.0, 1.0))


def _plate_x_bounds(side: str, width: float) -> tuple[float, float]:
    if side == "back":
        x_max = -BARREL_RADIUS - CLEAR_LINE_GAP
        return x_max - width, x_max
    x_min = BARREL_RADIUS + CLEAR_LINE_GAP
    return x_min, x_min + width


def _knuckle_sleeve(y_min: float, length: float, *, inner_radius: float) -> cq.Shape:
    outer = _y_cylinder(BARREL_RADIUS, length, y_min)
    inner = _y_cylinder(inner_radius, length + 0.002, y_min - 0.001)
    return outer.cut(inner)


def _knuckle_web(side: str, width: float, y_min: float, length: float) -> cq.Shape:
    plate_x_min, plate_x_max = _plate_x_bounds(side, width)
    z_min = PLATE_Z_CENTER - LEAF_THICKNESS / 2.0
    if side == "back":
        x_min = plate_x_max - WEB_OVERLAP
        x_max = -BARREL_RADIUS + WEB_OVERLAP
    else:
        x_min = BARREL_RADIUS - WEB_OVERLAP
        x_max = plate_x_min + WEB_OVERLAP
    return cq.Solid.makeBox(x_max - x_min, length, WEB_TOP_Z - z_min, cq.Vector(x_min, y_min, z_min))


def _leaf_plate(
    *,
    width: float,
    length: float,
    side: str,
    hole_ys: Iterable[float],
) -> cq.Shape:
    x_min, x_max = _plate_x_bounds(side, width)
    z_min = PLATE_Z_CENTER - LEAF_THICKNESS / 2.0
    hole_x = x_min + 0.55 * width if side == "back" else x_max - 0.55 * width

    plate = cq.Solid.makeBox(width, length, LEAF_THICKNESS, cq.Vector(x_min, -length / 2.0, z_min))
    plate = cq.Workplane(obj=plate).edges("|Z").fillet(PLATE_EDGE_FILLET).val()
    cutters = [
        _z_cylinder(
            FASTENER_HOLE_RADIUS,
            LEAF_THICKNESS + 0.010,
            hole_x,
            y_pos,
            z_min - 0.005,
        )
        for y_pos in hole_ys
    ]
    return _cut_all(plate, *cutters)


def _plate_with_webs(
    *,
    side: str,
    width: float,
    length: float,
    hole_ys: Iterable[float],
    knuckle_ymins: Iterable[float],
    knuckle_length: float,
) -> cq.Shape:
    shape = _leaf_plate(width=width, length=length, side=side, hole_ys=hole_ys)
    for y_min in knuckle_ymins:
        shape = _fuse_all(shape, _knuckle_web(side, width, y_min, knuckle_length))
    return shape


def _knuckle_ymins() -> tuple[list[float], list[float]]:
    back_start = -HINGE_LENGTH / 2.0 + END_MARGIN
    back_ymins = [
        back_start,
        back_start + BACK_KNUCKLE_LENGTH + GAP_LENGTH,
        back_start + 2.0 * (BACK_KNUCKLE_LENGTH + GAP_LENGTH),
    ]
    front_ymins = [
        back_start + BACK_KNUCKLE_LENGTH + AXIAL_CLEARANCE / 2.0,
        back_start + 2.0 * BACK_KNUCKLE_LENGTH + GAP_LENGTH + AXIAL_CLEARANCE / 2.0,
    ]
    return back_ymins, front_ymins


def _back_plate_shape() -> cq.Shape:
    back_ymins, _ = _knuckle_ymins()
    return _plate_with_webs(
        side="back",
        width=BACK_LEAF_WIDTH,
        length=BACK_LEAF_LENGTH,
        hole_ys=(-0.052, 0.0, 0.052),
        knuckle_ymins=back_ymins,
        knuckle_length=BACK_KNUCKLE_LENGTH,
    )

def _back_knuckles_shape() -> cq.Shape:
    back_ymins, _ = _knuckle_ymins()
    shape = _y_cylinder(BARREL_RADIUS, BACK_KNUCKLE_LENGTH, back_ymins[0])
    for y_min in back_ymins[1:]:
        shape = _fuse_all(shape, _y_cylinder(BARREL_RADIUS, BACK_KNUCKLE_LENGTH, y_min))
    return shape


def _pin_shape() -> cq.Shape:
    pin = _y_cylinder(PIN_RADIUS, HINGE_LENGTH, -HINGE_LENGTH / 2.0)
    head_low = _y_cylinder(PIN_HEAD_RADIUS, PIN_HEAD_LENGTH, -HINGE_LENGTH / 2.0 - PIN_HEAD_LENGTH)
    head_high = _y_cylinder(PIN_HEAD_RADIUS, PIN_HEAD_LENGTH, HINGE_LENGTH / 2.0)
    return _fuse_all(pin, head_low, head_high)


def _front_plate_shape() -> cq.Shape:
    _, front_ymins = _knuckle_ymins()
    return _plate_with_webs(
        side="front",
        width=FRONT_LEAF_WIDTH,
        length=FRONT_LEAF_LENGTH,
        hole_ys=(-0.034, 0.034),
        knuckle_ymins=front_ymins,
        knuckle_length=FRONT_KNUCKLE_LENGTH,
    )


def _front_knuckles_shape() -> cq.Shape:
    _, front_ymins = _knuckle_ymins()
    shape = _knuckle_sleeve(front_ymins[0], FRONT_KNUCKLE_LENGTH, inner_radius=PIN_RADIUS + PIN_CLEARANCE)
    for y_min in front_ymins[1:]:
        shape = _fuse_all(
            shape,
            _knuckle_sleeve(y_min, FRONT_KNUCKLE_LENGTH, inner_radius=PIN_RADIUS + PIN_CLEARANCE),
        )
    return shape


def _part_bounds(side: str, width: float, length: float) -> tuple[float, float, float, float, float]:
    plate_x_min, plate_x_max = _plate_x_bounds(side, width)
    x_min = min(plate_x_min, -BARREL_RADIUS)
    x_max = max(plate_x_max, BARREL_RADIUS)
    if side == "back":
        x_min = min(x_min, -PIN_HEAD_RADIUS)
        x_max = max(x_max, PIN_HEAD_RADIUS)
    z_min = min(PLATE_Z_CENTER - LEAF_THICKNESS / 2.0, -BARREL_RADIUS)
    z_max = BARREL_RADIUS
    return x_min, x_max, length, z_min, z_max


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="access_panel_hinge")
    satin_steel = model.material("satin_steel", rgba=(0.72, 0.74, 0.77, 1.0))

    back_leaf = model.part("back_leaf")
    back_leaf.visual(
        mesh_from_cadquery(_back_plate_shape(), "back_leaf_plate"),
        material=satin_steel,
        name="back_plate",
    )
    back_leaf.visual(
        mesh_from_cadquery(_back_knuckles_shape(), "back_leaf_knuckles"),
        material=satin_steel,
        name="back_knuckles",
    )
    back_leaf.visual(
        mesh_from_cadquery(_pin_shape(), "hinge_pin"),
        material=satin_steel,
        name="hinge_pin",
    )
    back_x_min, back_x_max, back_len, back_z_min, back_z_max = _part_bounds("back", BACK_LEAF_WIDTH, BACK_LEAF_LENGTH)
    back_leaf.inertial = Inertial.from_geometry(
        Box((back_x_max - back_x_min, back_len, back_z_max - back_z_min)),
        mass=0.12,
        origin=Origin(xyz=((back_x_min + back_x_max) / 2.0, 0.0, (back_z_min + back_z_max) / 2.0)),
    )

    front_leaf = model.part("front_leaf")
    front_leaf.visual(
        mesh_from_cadquery(_front_plate_shape(), "front_leaf_plate"),
        material=satin_steel,
        name="front_plate",
    )
    front_leaf.visual(
        mesh_from_cadquery(_front_knuckles_shape(), "front_leaf_knuckles"),
        material=satin_steel,
        name="front_knuckles",
    )
    front_x_min, front_x_max, front_len, front_z_min, front_z_max = _part_bounds("front", FRONT_LEAF_WIDTH, FRONT_LEAF_LENGTH)
    front_leaf.inertial = Inertial.from_geometry(
        Box((front_x_max - front_x_min, front_len, front_z_max - front_z_min)),
        mass=0.08,
        origin=Origin(xyz=((front_x_min + front_x_max) / 2.0, 0.0, (front_z_min + front_z_max) / 2.0)),
    )

    model.articulation(
        "back_leaf_to_front_leaf",
        ArticulationType.REVOLUTE,
        parent=back_leaf,
        child=front_leaf,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.0, lower=0.0, upper=1.85),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    back_leaf = object_model.get_part("back_leaf")
    front_leaf = object_model.get_part("front_leaf")
    hinge = object_model.get_articulation("back_leaf_to_front_leaf")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts(contact_tol=0.0005)
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "two_leaves_one_joint",
        len(object_model.parts) == 2
        and len(object_model.articulations) == 1
        and hinge.articulation_type == ArticulationType.REVOLUTE,
        details="Expected exactly two leaves connected by a single revolute joint.",
    )
    ctx.check(
        "hinge_axis_matches_pin_direction",
        tuple(round(value, 6) for value in hinge.axis) == (0.0, -1.0, 0.0),
        details=f"Expected hinge axis (0, -1, 0), got {hinge.axis}.",
    )

    ctx.expect_within(
        front_leaf,
        back_leaf,
        axes="y",
        margin=0.0,
        name="front_leaf_is_shorter_than_back_leaf",
    )
    ctx.expect_gap(
        front_leaf,
        back_leaf,
        axis="x",
        positive_elem="front_plate",
        negative_elem="back_plate",
        min_gap=0.006,
        max_gap=0.0085,
        name="support_line_clear_between_leaf_plates",
    )

    closed_aabb = ctx.part_world_aabb(front_leaf)
    opened_aabb = None
    with ctx.pose({hinge: OPEN_ANGLE}):
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_in_open_pose")
        opened_aabb = ctx.part_world_aabb(front_leaf)

    if closed_aabb is not None and opened_aabb is not None:
        closed_center_z = 0.5 * (closed_aabb[0][2] + closed_aabb[1][2])
        open_center_z = 0.5 * (opened_aabb[0][2] + opened_aabb[1][2])
        ctx.check(
            "positive_rotation_opens_upward",
            open_center_z > closed_center_z + 0.003,
            details=(
                f"Closed center z={closed_center_z:.4f}, "
                f"open center z={open_center_z:.4f}; positive rotation should lift the leaf."
            ),
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
