from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
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


HINGE_HEIGHT = 0.140
BARREL_RADIUS = 0.0064
PIN_RADIUS = 0.0036
MOVING_BORE_RADIUS = 0.0036
FIXED_PLATE_Y = -0.0036
MOVING_PLATE_Y = 0.0024
FIXED_KNUCKLE_Y_MAX = 0.0
MOVING_KNUCKLE_Y_MIN = 0.0006

FIXED_LEAF_WIDTH = 0.046
FIXED_LEAF_THICKNESS = 0.0040
FIXED_HINGE_EDGE_X = -0.0100

MOVING_LEAF_WIDTH = 0.036
MOVING_LEAF_THICKNESS = 0.0032
MOVING_HINGE_EDGE_X = 0.0095

FIXED_SEGMENTS = (
    (-0.0700, -0.0390),
    (-0.0110, 0.0110),
    (0.0390, 0.0700),
)
MOVING_SEGMENTS = (
    (-0.0390, -0.0110),
    (0.0110, 0.0390),
)


def _union_all(shapes: list[cq.Workplane]) -> cq.Workplane:
    result = shapes[0]
    for shape in shapes[1:]:
        result = result.union(shape)
    return result


def _segment_cylinder(radius: float, z0: float, z1: float) -> cq.Workplane:
    return cq.Workplane("XY").circle(radius).extrude(z1 - z0).translate((0.0, 0.0, z0))


def _segment_tube(outer_radius: float, inner_radius: float, z0: float, z1: float) -> cq.Workplane:
    length = z1 - z0
    outer = cq.Workplane("XY").circle(outer_radius).extrude(length)
    inner = (
        cq.Workplane("XY")
        .circle(inner_radius)
        .extrude(length + 0.002)
        .translate((0.0, 0.0, -0.001))
    )
    return outer.cut(inner).translate((0.0, 0.0, z0))


def _segment_tube_sector(
    outer_radius: float,
    inner_radius: float,
    z0: float,
    z1: float,
    *,
    y_min: float,
    y_max: float,
) -> cq.Workplane:
    length = z1 - z0
    tube = _segment_tube(outer_radius, inner_radius, z0, z1)
    clip = (
        cq.Workplane("XY")
        .box(outer_radius * 2.2, y_max - y_min, length, centered=(True, True, True))
        .translate((0.0, (y_min + y_max) / 2.0, (z0 + z1) / 2.0))
    )
    return tube.intersect(clip)


def _bridge_block(
    x0: float,
    x1: float,
    thickness: float,
    z0: float,
    z1: float,
    *,
    y_center: float,
) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(x1 - x0, thickness, z1 - z0, centered=(True, True, True))
        .translate(((x0 + x1) / 2.0, y_center, (z0 + z1) / 2.0))
    )


def _leaf_plate(
    *,
    width: float,
    thickness: float,
    x_center: float,
    y_center: float,
    hole_positions: tuple[tuple[float, float], ...],
    hole_diameter: float,
    csk_diameter: float,
) -> cq.Workplane:
    plate = (
        cq.Workplane("XY")
        .box(width, thickness, HINGE_HEIGHT, centered=(True, True, True))
        .translate((x_center, y_center, 0.0))
        .edges("|Y")
        .fillet(0.003)
    )
    return (
        plate.faces(">Y")
        .workplane(centerOption="CenterOfMass")
        .pushPoints(hole_positions)
        .cskHole(hole_diameter, csk_diameter, 82.0)
    )


def _fixed_leaf_body() -> cq.Workplane:
    fixed_plate_center_x = FIXED_HINGE_EDGE_X - FIXED_LEAF_WIDTH / 2.0
    plate = _leaf_plate(
        width=FIXED_LEAF_WIDTH,
        thickness=FIXED_LEAF_THICKNESS,
        x_center=fixed_plate_center_x,
        y_center=FIXED_PLATE_Y,
        hole_positions=((0.0, -0.045), (0.0, -0.015), (0.0, 0.015), (0.0, 0.045)),
        hole_diameter=0.0052,
        csk_diameter=0.0092,
    )
    bridges = [
        _bridge_block(
            FIXED_HINGE_EDGE_X,
            -BARREL_RADIUS + 0.0010,
            FIXED_LEAF_THICKNESS,
            z0,
            z1,
            y_center=FIXED_PLATE_Y,
        )
        for z0, z1 in FIXED_SEGMENTS
    ]
    knuckles = [
        _segment_tube_sector(
            BARREL_RADIUS,
            MOVING_BORE_RADIUS,
            z0,
            z1,
            y_min=-BARREL_RADIUS,
            y_max=FIXED_KNUCKLE_Y_MAX,
        )
        for z0, z1 in FIXED_SEGMENTS
    ]
    return _union_all([plate, *bridges, *knuckles])


def _moving_leaf_body() -> cq.Workplane:
    moving_plate_center_x = MOVING_HINGE_EDGE_X + MOVING_LEAF_WIDTH / 2.0
    plate = _leaf_plate(
        width=MOVING_LEAF_WIDTH,
        thickness=MOVING_LEAF_THICKNESS,
        x_center=moving_plate_center_x,
        y_center=MOVING_PLATE_Y,
        hole_positions=((0.0, -0.035), (0.0, 0.0), (0.0, 0.035)),
        hole_diameter=0.0048,
        csk_diameter=0.0084,
    )
    bridges = [
        _bridge_block(
            BARREL_RADIUS - 0.0010,
            MOVING_HINGE_EDGE_X,
            MOVING_LEAF_THICKNESS,
            z0,
            z1,
            y_center=MOVING_PLATE_Y,
        )
        for z0, z1 in MOVING_SEGMENTS
    ]
    knuckles = [
        _segment_tube_sector(
            BARREL_RADIUS,
            MOVING_BORE_RADIUS,
            z0,
            z1,
            y_min=MOVING_KNUCKLE_Y_MIN,
            y_max=BARREL_RADIUS,
        )
        for z0, z1 in MOVING_SEGMENTS
    ]
    return _union_all([plate, *bridges, *knuckles])


def _pin_shape() -> cq.Workplane:
    pin_length = HINGE_HEIGHT
    head_height = 0.0014
    head_radius = 0.0046
    rod = (
        cq.Workplane("XY")
        .circle(PIN_RADIUS)
        .extrude(pin_length)
        .translate((0.0, 0.0, -HINGE_HEIGHT / 2.0))
    )
    top_head = (
        cq.Workplane("XY")
        .circle(head_radius)
        .extrude(head_height)
        .translate((0.0, 0.0, HINGE_HEIGHT / 2.0))
    )
    bottom_head = (
        cq.Workplane("XY")
        .circle(head_radius)
        .extrude(head_height)
        .translate((0.0, 0.0, -HINGE_HEIGHT / 2.0 - head_height))
    )
    return _union_all([rod, top_head, bottom_head])


def _mesh(shape: cq.Workplane, name: str):
    return mesh_from_cadquery(
        shape,
        name,
        tolerance=0.00035,
        angular_tolerance=0.05,
        unit_scale=1.0,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mortised_equipment_hinge")

    fixed_finish = model.material("fixed_finish", rgba=(0.40, 0.43, 0.46, 1.0))
    moving_finish = model.material("moving_finish", rgba=(0.67, 0.69, 0.72, 1.0))
    pin_finish = model.material("pin_finish", rgba=(0.83, 0.84, 0.85, 1.0))

    fixed_leaf = model.part("fixed_leaf")
    moving_leaf = model.part("moving_leaf")

    fixed_leaf.visual(_mesh(_fixed_leaf_body(), "fixed_leaf_body"), material=fixed_finish, name="fixed_body")
    fixed_leaf.visual(_mesh(_pin_shape(), "hinge_pin"), material=pin_finish, name="pin")
    fixed_leaf.inertial = Inertial.from_geometry(
        Box((0.064, 0.014, 0.142)),
        mass=0.46,
        origin=Origin(xyz=(-0.025, 0.0, 0.0)),
    )

    moving_leaf.visual(_mesh(_moving_leaf_body(), "moving_leaf_body"), material=moving_finish, name="moving_body")
    moving_leaf.inertial = Inertial.from_geometry(
        Box((0.052, 0.013, 0.140)),
        mass=0.30,
        origin=Origin(xyz=(0.019, 0.0, 0.0)),
    )

    model.articulation(
        "leaf_hinge",
        ArticulationType.REVOLUTE,
        parent=fixed_leaf,
        child=moving_leaf,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=25.0, velocity=2.5, lower=0.0, upper=1.85),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    fixed_leaf = object_model.get_part("fixed_leaf")
    moving_leaf = object_model.get_part("moving_leaf")
    hinge = object_model.get_articulation("leaf_hinge")
    fixed_body = fixed_leaf.get_visual("fixed_body")
    moving_body = moving_leaf.get_visual("moving_body")
    pin = fixed_leaf.get_visual("pin")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
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

    ctx.expect_contact(
        moving_leaf,
        fixed_leaf,
        elem_a=moving_body,
        elem_b=pin,
        contact_tol=0.00005,
        name="moving_leaf_wraps_and_contacts_center_pin",
    )

    fixed_body_box = ctx.part_element_world_aabb(fixed_leaf, elem=fixed_body)
    moving_body_box = ctx.part_element_world_aabb(moving_leaf, elem=moving_body)
    pin_box = ctx.part_element_world_aabb(fixed_leaf, elem=pin)

    fixed_width = None
    moving_width = None
    fixed_thickness = None
    moving_thickness = None
    if fixed_body_box is not None and moving_body_box is not None:
        fixed_width = fixed_body_box[1][0] - fixed_body_box[0][0]
        moving_width = moving_body_box[1][0] - moving_body_box[0][0]
        fixed_thickness = fixed_body_box[1][1] - fixed_body_box[0][1]
        moving_thickness = moving_body_box[1][1] - moving_body_box[0][1]
    ctx.check(
        "fixed_leaf_reads_as_heavier_support",
        fixed_width is not None
        and moving_width is not None
        and fixed_thickness is not None
        and moving_thickness is not None
        and fixed_width > moving_width + 0.006
        and fixed_thickness > moving_thickness + 0.0005,
        details=(
            f"fixed width/thickness={fixed_width}/{fixed_thickness}, "
            f"moving width/thickness={moving_width}/{moving_thickness}"
        ),
    )

    ctx.check(
        "pin_projects_beyond_leaf_ends",
        pin_box is not None and pin_box[0][2] < -HINGE_HEIGHT / 2.0 and pin_box[1][2] > HINGE_HEIGHT / 2.0,
        details=f"pin aabb={pin_box}",
    )

    with ctx.pose({hinge: 1.15}):
        moving_open_box = ctx.part_element_world_aabb(moving_leaf, elem=moving_body)
        open_center_y = None
        open_max_y = None
        if moving_open_box is not None:
            open_center_y = (moving_open_box[0][1] + moving_open_box[1][1]) / 2.0
            open_max_y = moving_open_box[1][1]
        ctx.check(
            "moving_leaf_swings_outward_about_pin_axis",
            open_center_y is not None and open_max_y is not None and open_center_y > 0.012 and open_max_y > 0.035,
            details=f"open-pose moving body aabb={moving_open_box}, center_y={open_center_y}, max_y={open_max_y}",
        )
        ctx.expect_contact(
            moving_leaf,
            fixed_leaf,
            elem_a=moving_body,
            elem_b=pin,
            contact_tol=0.00005,
            name="moving_leaf_remains_captured_on_pin_when_open",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
