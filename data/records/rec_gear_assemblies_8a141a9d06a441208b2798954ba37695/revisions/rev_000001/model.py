from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

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


def _box(center: tuple[float, float, float], size: tuple[float, float, float]):
    return cq.Workplane("XY").box(*size).translate(center).val()


def _cylinder(
    axis: str,
    radius: float,
    length: float,
    start: tuple[float, float, float],
):
    directions = {
        "x": cq.Vector(1.0, 0.0, 0.0),
        "y": cq.Vector(0.0, 1.0, 0.0),
        "z": cq.Vector(0.0, 0.0, 1.0),
    }
    return cq.Solid.makeCylinder(radius, length, cq.Vector(*start), directions[axis])


def _fuse(shapes):
    wp = cq.Workplane("XY")
    for shape in shapes:
        wp = wp.add(shape)
    return wp.combine(clean=True).val()


def _bearing_block(
    center: tuple[float, float, float],
    size: tuple[float, float, float],
    bore_axis: str,
    bore_radius: float,
    bore_center: tuple[float, float, float] | None = None,
):
    sx, sy, sz = size
    x, y, z = center
    bx, by, bz = bore_center if bore_center is not None else center
    block = cq.Workplane(obj=_box(center, size)).edges("|Z").fillet(2.0).val()
    if bore_axis == "z":
        cutter = _cylinder("z", bore_radius, sz + 8.0, (bx, by, bz - (sz + 8.0) / 2.0))
    elif bore_axis == "x":
        cutter = _cylinder("x", bore_radius, sx + 8.0, (bx - (sx + 8.0) / 2.0, by, bz))
    else:
        raise ValueError(f"unsupported bore axis: {bore_axis}")
    return cq.Workplane(obj=block).cut(cutter).val()


def _bevel_gear_along_negative_z(
    *,
    face_width: float,
    large_radius: float,
    small_radius: float,
    hub_radius: float,
    hub_length: float,
    tooth_count: int,
    tooth_height: float,
    tooth_width: float,
):
    gear = _fuse(
        [
            cq.Solid.makeCone(
                large_radius,
                small_radius,
                face_width,
                cq.Vector(0.0, 0.0, -face_width),
                cq.Vector(0.0, 0.0, 1.0),
            ),
            _cylinder("z", hub_radius, hub_length, (0.0, 0.0, -hub_length)),
            _cylinder("z", large_radius * 0.55, 3.0, (0.0, 0.0, -face_width - 2.0)),
        ]
    )
    window_cutters = []
    for angle in (0.0, 90.0, 180.0, 270.0):
        window = (
            cq.Workplane("XY")
            .box(large_radius * 0.9, 4.2, face_width * 0.62)
            .translate((large_radius * 0.46, 0.0, -face_width * 0.72))
            .rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), angle)
            .val()
        )
        window_cutters.append(window)
    tooth_cutters = []
    for index in range(tooth_count):
        angle = 360.0 * (index + 0.5) / tooth_count
        cutter = (
            cq.Workplane("XY")
            .box(tooth_height, tooth_width, face_width * 0.7)
            .translate((large_radius - tooth_height * 0.2, 0.0, -face_width * 0.28))
            .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), -18.0)
            .rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), angle)
            .val()
        )
        tooth_cutters.append(cutter)
    return cq.Workplane(obj=gear).cut(_fuse([*window_cutters, *tooth_cutters])).val()


def _build_bevel_gears():
    vertical_gear = _bevel_gear_along_negative_z(
        face_width=10.0,
        large_radius=16.5,
        small_radius=6.0,
        hub_radius=8.0,
        hub_length=16.0,
        tooth_count=16,
        tooth_height=4.0,
        tooth_width=4.6,
    )
    horizontal_gear = (
        cq.Workplane(obj=vertical_gear)
        .rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), -90.0)
        .translate((19.5, 0.0, 0.0))
        .val()
    )
    return vertical_gear, horizontal_gear


def _aabb_center(aabb):
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple((lo + hi) * 0.5 for lo, hi in zip(mins, maxs))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="right_angle_bevel_gear_assembly")

    dark_frame = model.material("dark_frame", rgba=(0.24, 0.25, 0.28, 1.0))
    shaft_steel = model.material("shaft_steel", rgba=(0.78, 0.80, 0.83, 1.0))
    brass = model.material("brass", rgba=(0.77, 0.62, 0.28, 1.0))
    clear_bearing = model.material("clear_bearing", rgba=(0.80, 0.93, 1.0, 0.35))
    black_handle = model.material("black_handle", rgba=(0.12, 0.12, 0.13, 1.0))

    base = model.part("base_bracket")
    vertical = model.part("vertical_shaft")
    horizontal = model.part("horizontal_shaft")

    base_plate = _box((85.0, 0.0, -59.0), (290.0, 160.0, 18.0))
    rear_upright = _box((-35.0, 0.0, 35.0), (18.0, 110.0, 170.0))
    top_bridge = _box((5.0, 0.0, 68.0), (80.0, 95.0, 16.0))
    bracket_shape = _fuse([base_plate, rear_upright, top_bridge])
    mounting_holes = [
        _cylinder("z", 5.5, 26.0, (20.0, 50.0, -72.0)),
        _cylinder("z", 5.5, 26.0, (20.0, -50.0, -72.0)),
        _cylinder("z", 5.5, 26.0, (190.0, 50.0, -72.0)),
        _cylinder("z", 5.5, 26.0, (190.0, -50.0, -72.0)),
    ]
    vertical_clearance = _cylinder("z", 7.2, 240.0, (0.0, 0.0, -90.0))
    bracket_shape = (
        cq.Workplane(obj=bracket_shape)
        .cut(_fuse([*mounting_holes, vertical_clearance]))
        .edges("|Z")
        .fillet(3.0)
        .val()
    )

    vertical_lower_bearing = _bearing_block(
        center=(0.0, 0.0, -38.2),
        size=(38.0, 50.0, 24.0),
        bore_axis="z",
        bore_radius=6.1,
    )
    vertical_upper_bearing = _bearing_block(
        center=(0.0, 0.0, 48.2),
        size=(38.0, 50.0, 24.0),
        bore_axis="z",
        bore_radius=6.1,
    )
    horizontal_inner_bearing = _bearing_block(
        center=(70.0, 0.0, -19.2),
        size=(30.0, 50.0, 62.0),
        bore_axis="x",
        bore_radius=6.1,
        bore_center=(70.0, 0.0, 0.0),
    )
    horizontal_outer_bearing = _bearing_block(
        center=(150.0, 0.0, -19.2),
        size=(30.0, 50.0, 62.0),
        bore_axis="x",
        bore_radius=6.1,
        bore_center=(150.0, 0.0, 0.0),
    )

    vertical_gear, horizontal_gear = _build_bevel_gears()

    vertical_rod_shape = _fuse(
        [
            _cylinder("z", 5.0, 148.0, (0.0, 0.0, -60.0)),
            _cylinder("z", 8.0, 5.0, (0.0, 0.0, -25.0)),
            _cylinder("z", 8.0, 5.0, (0.0, 0.0, 77.0)),
        ]
    )
    vertical_handle_shape = _cylinder("z", 18.0, 6.0, (0.0, 0.0, 88.0))
    vertical_lower_collar = _cylinder("z", 9.0, 4.0, (0.0, 0.0, -54.2))
    vertical_upper_collar = _cylinder("z", 9.0, 4.0, (0.0, 0.0, 60.2))
    vertical_pin_shape = _cylinder("z", 2.8, 16.0, (12.0, 0.0, 94.0))

    horizontal_rod_shape = _fuse(
        [
            _cylinder("x", 5.0, 207.0, (24.0, 0.0, 0.0)),
            _cylinder("x", 8.0, 6.0, (46.0, 0.0, 0.0)),
            _cylinder("x", 8.0, 6.0, (168.0, 0.0, 0.0)),
        ]
    )
    horizontal_handle_shape = _cylinder("x", 14.0, 6.0, (225.0, 0.0, 0.0))
    horizontal_inner_collar = _cylinder("x", 9.0, 6.0, (49.0, 0.0, 0.0))
    horizontal_outer_collar = _cylinder("x", 9.0, 6.0, (165.0, 0.0, 0.0))
    horizontal_pin_shape = _cylinder("x", 2.6, 14.0, (231.0, 0.0, 10.0))

    base.visual(
        mesh_from_cadquery(bracket_shape, "bracket_shape", unit_scale=0.001),
        material=dark_frame,
        name="bracket",
    )
    base.visual(
        mesh_from_cadquery(vertical_lower_bearing, "vertical_lower_bearing", unit_scale=0.001),
        material=clear_bearing,
        name="vertical_lower_bearing",
    )
    base.visual(
        mesh_from_cadquery(vertical_upper_bearing, "vertical_upper_bearing", unit_scale=0.001),
        material=clear_bearing,
        name="vertical_upper_bearing",
    )
    base.visual(
        mesh_from_cadquery(horizontal_inner_bearing, "horizontal_inner_bearing", unit_scale=0.001),
        material=clear_bearing,
        name="horizontal_inner_bearing",
    )
    base.visual(
        mesh_from_cadquery(horizontal_outer_bearing, "horizontal_outer_bearing", unit_scale=0.001),
        material=clear_bearing,
        name="horizontal_outer_bearing",
    )

    vertical.visual(
        mesh_from_cadquery(vertical_rod_shape, "vertical_rod_shape", unit_scale=0.001),
        material=shaft_steel,
        name="vertical_rod",
    )
    vertical.visual(
        mesh_from_cadquery(vertical_handle_shape, "vertical_handle_shape", unit_scale=0.001),
        material=shaft_steel,
        name="vertical_handle",
    )
    vertical.visual(
        mesh_from_cadquery(vertical_lower_collar, "vertical_lower_collar", unit_scale=0.001),
        material=shaft_steel,
        name="vertical_lower_collar",
    )
    vertical.visual(
        mesh_from_cadquery(vertical_upper_collar, "vertical_upper_collar", unit_scale=0.001),
        material=shaft_steel,
        name="vertical_upper_collar",
    )
    vertical.visual(
        mesh_from_cadquery(vertical_gear, "vertical_bevel_gear", unit_scale=0.001),
        material=brass,
        name="vertical_gear",
    )
    vertical.visual(
        mesh_from_cadquery(vertical_pin_shape, "vertical_handle_pin", unit_scale=0.001),
        material=black_handle,
        name="vertical_pin",
    )

    horizontal.visual(
        mesh_from_cadquery(horizontal_rod_shape, "horizontal_rod_shape", unit_scale=0.001),
        material=shaft_steel,
        name="horizontal_rod",
    )
    horizontal.visual(
        mesh_from_cadquery(horizontal_handle_shape, "horizontal_handle_shape", unit_scale=0.001),
        material=shaft_steel,
        name="horizontal_handle",
    )
    horizontal.visual(
        mesh_from_cadquery(horizontal_inner_collar, "horizontal_inner_collar", unit_scale=0.001),
        material=shaft_steel,
        name="horizontal_inner_collar",
    )
    horizontal.visual(
        mesh_from_cadquery(horizontal_outer_collar, "horizontal_outer_collar", unit_scale=0.001),
        material=shaft_steel,
        name="horizontal_outer_collar",
    )
    horizontal.visual(
        mesh_from_cadquery(horizontal_gear, "horizontal_bevel_gear", unit_scale=0.001),
        material=brass,
        name="horizontal_gear",
    )
    horizontal.visual(
        mesh_from_cadquery(horizontal_pin_shape, "horizontal_handle_pin", unit_scale=0.001),
        material=black_handle,
        name="horizontal_pin",
    )

    model.articulation(
        "base_to_vertical_shaft",
        ArticulationType.REVOLUTE,
        parent=base,
        child=vertical,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=6.0, lower=-2.0 * pi, upper=2.0 * pi),
    )
    model.articulation(
        "base_to_horizontal_shaft",
        ArticulationType.REVOLUTE,
        parent=base,
        child=horizontal,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=6.0, lower=-2.0 * pi, upper=2.0 * pi),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_bracket")
    vertical = object_model.get_part("vertical_shaft")
    horizontal = object_model.get_part("horizontal_shaft")
    vertical_joint = object_model.get_articulation("base_to_vertical_shaft")
    horizontal_joint = object_model.get_articulation("base_to_horizontal_shaft")

    vertical_lower_bearing = base.get_visual("vertical_lower_bearing")
    vertical_upper_bearing = base.get_visual("vertical_upper_bearing")
    horizontal_inner_bearing = base.get_visual("horizontal_inner_bearing")
    horizontal_outer_bearing = base.get_visual("horizontal_outer_bearing")
    vertical_rod = vertical.get_visual("vertical_rod")
    horizontal_rod = horizontal.get_visual("horizontal_rod")
    vertical_pin = vertical.get_visual("vertical_pin")
    horizontal_pin = horizontal.get_visual("horizontal_pin")

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

    ctx.check(
        "three_primary_parts_present",
        {part.name for part in (base, vertical, horizontal)}
        == {"base_bracket", "vertical_shaft", "horizontal_shaft"},
        "Expected base bracket plus two articulated shafts.",
    )
    ctx.check(
        "vertical_joint_is_supported_revolute_z",
        vertical_joint.articulation_type == ArticulationType.REVOLUTE
        and tuple(vertical_joint.axis) == (0.0, 0.0, 1.0),
        f"vertical joint type/axis was {vertical_joint.articulation_type} {vertical_joint.axis}",
    )
    ctx.check(
        "horizontal_joint_is_supported_revolute_x",
        horizontal_joint.articulation_type == ArticulationType.REVOLUTE
        and tuple(horizontal_joint.axis) == (1.0, 0.0, 0.0),
        f"horizontal joint type/axis was {horizontal_joint.articulation_type} {horizontal_joint.axis}",
    )

    ctx.expect_within(
        vertical,
        base,
        axes="xy",
        inner_elem=vertical_rod,
        outer_elem=vertical_lower_bearing,
        margin=0.0,
        name="vertical_rod_passes_through_lower_bearing",
    )
    ctx.expect_within(
        vertical,
        base,
        axes="xy",
        inner_elem=vertical_rod,
        outer_elem=vertical_upper_bearing,
        margin=0.0,
        name="vertical_rod_passes_through_upper_bearing",
    )
    ctx.expect_within(
        horizontal,
        base,
        axes="yz",
        inner_elem=horizontal_rod,
        outer_elem=horizontal_inner_bearing,
        margin=0.0,
        name="horizontal_rod_passes_through_inner_bearing",
    )
    ctx.expect_within(
        horizontal,
        base,
        axes="yz",
        inner_elem=horizontal_rod,
        outer_elem=horizontal_outer_bearing,
        margin=0.0,
        name="horizontal_rod_passes_through_outer_bearing",
    )

    with ctx.pose({vertical_joint: 0.0}):
        vertical_pin_closed = _aabb_center(ctx.part_element_world_aabb(vertical, elem=vertical_pin))
    with ctx.pose({vertical_joint: pi / 2.0}):
        vertical_pin_open = _aabb_center(ctx.part_element_world_aabb(vertical, elem=vertical_pin))
    ctx.check(
        "vertical_shaft_rotates_about_z",
        vertical_pin_closed is not None
        and vertical_pin_open is not None
        and vertical_pin_closed[0] > 0.010
        and abs(vertical_pin_closed[1]) < 0.003
        and abs(vertical_pin_open[0]) < 0.003
        and vertical_pin_open[1] > 0.010
        and abs(vertical_pin_closed[2] - vertical_pin_open[2]) < 0.002,
        f"vertical pin centers: closed={vertical_pin_closed}, quarter_turn={vertical_pin_open}",
    )

    with ctx.pose({horizontal_joint: 0.0}):
        horizontal_pin_closed = _aabb_center(ctx.part_element_world_aabb(horizontal, elem=horizontal_pin))
    with ctx.pose({horizontal_joint: pi / 2.0}):
        horizontal_pin_open = _aabb_center(ctx.part_element_world_aabb(horizontal, elem=horizontal_pin))
    ctx.check(
        "horizontal_shaft_rotates_about_x",
        horizontal_pin_closed is not None
        and horizontal_pin_open is not None
        and abs(horizontal_pin_closed[1]) < 0.003
        and horizontal_pin_closed[2] > 0.005
        and abs(horizontal_pin_open[2]) < 0.003
        and abs(horizontal_pin_open[1]) > 0.005
        and abs(horizontal_pin_closed[0] - horizontal_pin_open[0]) < 0.002,
        f"horizontal pin centers: closed={horizontal_pin_closed}, quarter_turn={horizontal_pin_open}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
