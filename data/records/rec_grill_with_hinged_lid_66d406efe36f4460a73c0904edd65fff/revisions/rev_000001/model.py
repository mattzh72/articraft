from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


CHAMBER_LENGTH = 0.72
CHAMBER_RADIUS = 0.22
CHAMBER_WALL = 0.01
END_PLATE_THICKNESS = 0.008
CHAMBER_CENTER_Z = 0.88

FRAME_HALF_SPAN_X = 0.31
FRAME_HALF_DEPTH = 0.22
FRAME_TUBE = 0.04
FRAME_TOP_Z = 0.62
FRAME_LOWER_Z = 0.20
AXLE_Y = -FRAME_HALF_DEPTH
AXLE_Z = 0.18

SHELF_OUTER_X = -0.72
SHELF_INNER_X = -0.46
SHELF_Z = 0.76

WHEEL_RADIUS = 0.16
WHEEL_WIDTH = 0.05


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _half_disc_profile(radius: float, *, upper: bool, segments: int = 28) -> list[tuple[float, float]]:
    start = 0.0
    end = math.pi if upper else -math.pi
    return [
        (
            -radius * math.sin(start + (end - start) * (index / segments)),
            radius * math.cos(start + (end - start) * (index / segments)),
        )
        for index in range(segments + 1)
    ]


def _chamber_shell_mesh(*, upper: bool, name: str):
    geom = ExtrudeWithHolesGeometry(
        _half_disc_profile(CHAMBER_RADIUS, upper=upper),
        [_half_disc_profile(CHAMBER_RADIUS - CHAMBER_WALL, upper=upper)],
        CHAMBER_LENGTH,
        center=True,
    )
    geom.rotate_y(math.pi / 2.0)
    return _save_mesh(name, geom)


def _end_plate_mesh(*, upper: bool, name: str):
    geom = ExtrudeGeometry(
        _half_disc_profile(CHAMBER_RADIUS - 0.002, upper=upper),
        END_PLATE_THICKNESS,
        center=True,
    )
    geom.rotate_y(math.pi / 2.0)
    return _save_mesh(name, geom)


def _add_wheel_visuals(part, *, tire_material, steel_material, stem_offset_sign: float) -> None:
    wheel_axis = Origin(rpy=(0.0, math.pi / 2.0, 0.0))
    part.visual(
        Cylinder(radius=WHEEL_RADIUS, length=WHEEL_WIDTH),
        origin=wheel_axis,
        material=tire_material,
        name="tire",
    )
    part.visual(
        Cylinder(radius=0.112, length=WHEEL_WIDTH * 1.04),
        origin=wheel_axis,
        material=steel_material,
        name="rim",
    )
    part.visual(
        Cylinder(radius=0.034, length=0.090),
        origin=wheel_axis,
        material=steel_material,
        name="hub",
    )
    for spoke_index, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        part.visual(
            Box((0.012, 0.026, 0.190)),
            origin=Origin(rpy=(angle, 0.0, 0.0)),
            material=steel_material,
            name=f"spoke_{spoke_index}",
        )
    part.visual(
        Box((0.008, 0.018, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, stem_offset_sign * 0.135)),
        material=steel_material,
        name="valve_stem",
    )


def _aabb_center(aabb) -> tuple[float, float, float]:
    return tuple((lo + hi) * 0.5 for lo, hi in zip(aabb[0], aabb[1]))


def _vec_close(a: tuple[float, float, float], b: tuple[float, float, float], tol: float = 1e-6) -> bool:
    return all(abs(av - bv) <= tol for av, bv in zip(a, b))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="barrel_outdoor_grill")

    frame_black = model.material("frame_black", rgba=(0.17, 0.17, 0.18, 1.0))
    grill_black = model.material("grill_black", rgba=(0.12, 0.12, 0.12, 1.0))
    steel = model.material("steel", rgba=(0.58, 0.60, 0.62, 1.0))
    wood = model.material("wood", rgba=(0.55, 0.36, 0.20, 1.0))
    rubber = model.material("rubber", rgba=(0.07, 0.07, 0.07, 1.0))
    grate_steel = model.material("grate_steel", rgba=(0.46, 0.48, 0.50, 1.0))

    cart_frame = model.part("cart_frame")
    cart_frame.inertial = Inertial.from_geometry(
        Box((1.10, 0.56, 0.82)),
        mass=24.0,
        origin=Origin(xyz=(-0.04, 0.0, 0.41)),
    )
    for x_pos in (-FRAME_HALF_SPAN_X, FRAME_HALF_SPAN_X):
        for y_pos in (-FRAME_HALF_DEPTH, FRAME_HALF_DEPTH):
            cart_frame.visual(
                Box((FRAME_TUBE, FRAME_TUBE, FRAME_TOP_Z)),
                origin=Origin(xyz=(x_pos, y_pos, FRAME_TOP_Z * 0.5)),
                material=frame_black,
                name=f"main_leg_{'l' if x_pos < 0 else 'r'}_{'rear' if y_pos < 0 else 'front'}",
            )
    cart_frame.visual(
        Box((2.0 * FRAME_HALF_SPAN_X, FRAME_TUBE, FRAME_TUBE)),
        origin=Origin(xyz=(0.0, FRAME_HALF_DEPTH, FRAME_TOP_Z)),
        material=frame_black,
        name="front_top_rail",
    )
    cart_frame.visual(
        Box((2.0 * FRAME_HALF_SPAN_X, FRAME_TUBE, FRAME_TUBE)),
        origin=Origin(xyz=(0.0, -FRAME_HALF_DEPTH, FRAME_TOP_Z)),
        material=frame_black,
        name="rear_top_rail",
    )
    cart_frame.visual(
        Box((2.0 * FRAME_HALF_SPAN_X, FRAME_TUBE, FRAME_TUBE)),
        origin=Origin(xyz=(0.0, FRAME_HALF_DEPTH, FRAME_LOWER_Z)),
        material=frame_black,
        name="front_lower_rail",
    )
    cart_frame.visual(
        Box((2.0 * FRAME_HALF_SPAN_X, FRAME_TUBE, FRAME_TUBE)),
        origin=Origin(xyz=(0.0, -FRAME_HALF_DEPTH, FRAME_LOWER_Z)),
        material=frame_black,
        name="rear_lower_rail",
    )
    for x_pos, side_name in ((-FRAME_HALF_SPAN_X, "left"), (FRAME_HALF_SPAN_X, "right")):
        cart_frame.visual(
            Box((FRAME_TUBE, 2.0 * FRAME_HALF_DEPTH, FRAME_TUBE)),
            origin=Origin(xyz=(x_pos, 0.0, FRAME_TOP_Z)),
            material=frame_black,
            name=f"{side_name}_top_rail",
        )
        cart_frame.visual(
            Box((FRAME_TUBE, 2.0 * FRAME_HALF_DEPTH, FRAME_TUBE)),
            origin=Origin(xyz=(x_pos, 0.0, FRAME_LOWER_Z)),
            material=frame_black,
            name=f"{side_name}_lower_rail",
        )

    for x_pos, mount_name in ((-0.18, "left"), (0.18, "right")):
        cart_frame.visual(
            Box((0.24, 0.32, 0.02)),
            origin=Origin(xyz=(x_pos, 0.0, FRAME_TOP_Z - 0.015)),
            material=frame_black,
            name=f"chamber_mount_{mount_name}",
        )

    cart_frame.visual(
        Cylinder(radius=0.015, length=0.82),
        origin=Origin(xyz=(0.0, AXLE_Y, AXLE_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="rear_axle",
    )

    for index, y_pos in enumerate((-0.18, -0.06, 0.06, 0.18)):
        cart_frame.visual(
            Box((0.58, 0.12, 0.012)),
            origin=Origin(xyz=(0.0, y_pos, FRAME_LOWER_Z + 0.026)),
            material=frame_black,
            name=f"lower_shelf_slat_{index}",
        )

    side_shelf = model.part("side_shelf")
    side_shelf.inertial = Inertial.from_geometry(
        Box((0.30, 0.42, 0.80)),
        mass=6.5,
        origin=Origin(xyz=((SHELF_OUTER_X + SHELF_INNER_X) * 0.5, 0.0, 0.40)),
    )
    for x_pos, x_name in ((SHELF_OUTER_X, "outer"), (SHELF_INNER_X, "inner")):
        for y_pos, y_name in ((-0.18, "rear"), (0.18, "front")):
            side_shelf.visual(
                Box((FRAME_TUBE, FRAME_TUBE, SHELF_Z)),
                origin=Origin(xyz=(x_pos, y_pos, SHELF_Z * 0.5)),
                material=frame_black,
                name=f"{x_name}_post_{y_name}",
            )
    side_shelf.visual(
        Box((SHELF_INNER_X - SHELF_OUTER_X, FRAME_TUBE, FRAME_TUBE)),
        origin=Origin(xyz=((SHELF_OUTER_X + SHELF_INNER_X) * 0.5, -0.18, SHELF_Z)),
        material=frame_black,
        name="shelf_rear_rail",
    )
    side_shelf.visual(
        Box((SHELF_INNER_X - SHELF_OUTER_X, FRAME_TUBE, FRAME_TUBE)),
        origin=Origin(xyz=((SHELF_OUTER_X + SHELF_INNER_X) * 0.5, 0.18, SHELF_Z)),
        material=frame_black,
        name="shelf_front_rail",
    )
    side_shelf.visual(
        Box((FRAME_TUBE, 0.36, FRAME_TUBE)),
        origin=Origin(xyz=(SHELF_OUTER_X, 0.0, SHELF_Z)),
        material=frame_black,
        name="shelf_outer_rail",
    )
    side_shelf.visual(
        Box((FRAME_TUBE, 0.36, FRAME_TUBE)),
        origin=Origin(xyz=(SHELF_INNER_X, 0.0, SHELF_Z)),
        material=frame_black,
        name="shelf_inner_rail",
    )
    for y_pos, connector_name in ((-0.16, "rear"), (0.16, "front")):
        side_shelf.visual(
            Box((0.12, 0.08, FRAME_TUBE)),
            origin=Origin(xyz=(-0.39, y_pos, FRAME_TOP_Z)),
            material=frame_black,
            name=f"frame_bracket_{connector_name}",
        )
    for index, y_pos in enumerate((-0.135, -0.045, 0.045, 0.135)):
        side_shelf.visual(
            Box((SHELF_INNER_X - SHELF_OUTER_X - 0.005, 0.09, 0.018)),
            origin=Origin(xyz=((SHELF_OUTER_X + SHELF_INNER_X) * 0.5, y_pos, SHELF_Z + 0.029)),
            material=wood,
            name=f"side_shelf_slat_{index}",
        )

    lower_chamber = model.part("lower_chamber")
    lower_chamber.inertial = Inertial.from_geometry(
        Box((CHAMBER_LENGTH, CHAMBER_RADIUS * 2.0, CHAMBER_RADIUS * 1.2)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, -0.04)),
    )
    lower_chamber.visual(
        _chamber_shell_mesh(upper=False, name="lower_chamber_shell_v2"),
        material=grill_black,
        name="lower_shell",
    )
    for side_name, x_pos in (("left", -CHAMBER_LENGTH * 0.5 + END_PLATE_THICKNESS * 0.5), ("right", CHAMBER_LENGTH * 0.5 - END_PLATE_THICKNESS * 0.5)):
        lower_chamber.visual(
            _end_plate_mesh(upper=False, name=f"lower_end_plate_mesh_{side_name}_v2"),
            origin=Origin(xyz=(x_pos, 0.0, 0.0)),
            material=grill_black,
            name=f"lower_end_plate_{side_name}",
        )
    lower_chamber.visual(
        Cylinder(radius=0.012, length=CHAMBER_LENGTH - 0.06),
        origin=Origin(
            xyz=(0.0, -CHAMBER_RADIUS, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=steel,
        name="hinge_rod",
    )
    lower_chamber.visual(
        Cylinder(radius=0.010, length=CHAMBER_LENGTH - 0.05),
        origin=Origin(
            xyz=(0.0, CHAMBER_RADIUS, -0.006),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=steel,
        name="front_lip",
    )
    for index, y_pos in enumerate((-0.14, -0.08, -0.02, 0.04, 0.10, 0.16)):
        lower_chamber.visual(
            Box((0.704, 0.008, 0.008)),
            origin=Origin(xyz=(0.0, y_pos, -0.035)),
            material=grate_steel,
            name=f"grate_bar_{index}",
        )
    for x_pos, side_name in ((-0.346, "left"), (0.346, "right")):
        lower_chamber.visual(
            Box((0.012, 0.32, 0.012)),
            origin=Origin(xyz=(x_pos, 0.01, -0.035)),
            material=grate_steel,
            name=f"grate_side_rail_{side_name}",
        )
    for x_pos, side_name in ((-0.18, "left"), (0.18, "right")):
        lower_chamber.visual(
            Box((0.05, 0.08, 0.06)),
            origin=Origin(xyz=(x_pos, 0.0, -0.235)),
            material=frame_black,
            name=f"mounting_foot_{side_name}",
        )

    lid = model.part("lid")
    lid.inertial = Inertial.from_geometry(
        Box((CHAMBER_LENGTH, CHAMBER_RADIUS * 2.0, CHAMBER_RADIUS)),
        mass=11.0,
        origin=Origin(xyz=(0.0, 0.0, 0.10)),
    )
    lid.visual(
        _chamber_shell_mesh(upper=True, name="upper_lid_shell_v2"),
        material=grill_black,
        name="lid_shell",
    )
    for side_name, x_pos in (("left", -CHAMBER_LENGTH * 0.5 + END_PLATE_THICKNESS * 0.5), ("right", CHAMBER_LENGTH * 0.5 - END_PLATE_THICKNESS * 0.5)):
        lid.visual(
            _end_plate_mesh(upper=True, name=f"lid_end_plate_mesh_{side_name}_v2"),
            origin=Origin(xyz=(x_pos, 0.0, 0.0)),
            material=grill_black,
            name=f"lid_end_plate_{side_name}",
        )
    for index, x_pos in enumerate((-0.23, 0.0, 0.23)):
        lid.visual(
            Cylinder(radius=0.016, length=0.08),
            origin=Origin(
                xyz=(x_pos, -CHAMBER_RADIUS, 0.0),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=steel,
            name=f"hinge_knuckle_{index}",
        )
    lid.visual(
        Cylinder(radius=0.008, length=0.075),
        origin=Origin(
            xyz=(-0.17, 0.218, 0.125),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=steel,
        name="handle_post_left",
    )
    lid.visual(
        Cylinder(radius=0.008, length=0.075),
        origin=Origin(
            xyz=(0.17, 0.218, 0.125),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=steel,
        name="handle_post_right",
    )
    lid.visual(
        Cylinder(radius=0.010, length=0.34),
        origin=Origin(
            xyz=(0.0, 0.255, 0.125),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=steel,
        name="handle_bar",
    )

    left_wheel = model.part("left_wheel")
    left_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=WHEEL_RADIUS, length=WHEEL_WIDTH),
        mass=2.8,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
    )
    _add_wheel_visuals(
        left_wheel,
        tire_material=rubber,
        steel_material=steel,
        stem_offset_sign=1.0,
    )

    right_wheel = model.part("right_wheel")
    right_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=WHEEL_RADIUS, length=WHEEL_WIDTH),
        mass=2.8,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
    )
    _add_wheel_visuals(
        right_wheel,
        tire_material=rubber,
        steel_material=steel,
        stem_offset_sign=-1.0,
    )

    damper = model.part("damper")
    damper.inertial = Inertial.from_geometry(
        Box((0.06, 0.11, 0.09)),
        mass=0.4,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
    )
    damper.visual(
        Cylinder(radius=0.006, length=0.08),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="damper_shaft",
    )
    damper.visual(
        Box((0.004, 0.095, 0.075)),
        origin=Origin(xyz=(-0.013, 0.0, 0.0)),
        material=steel,
        name="butterfly_plate",
    )
    damper.visual(
        Cylinder(radius=0.045, length=0.003),
        origin=Origin(xyz=(0.015, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="damper_outer_plate",
    )
    damper.visual(
        Box((0.006, 0.080, 0.014)),
        origin=Origin(xyz=(0.027, 0.043, 0.0)),
        material=steel,
        name="damper_handle",
    )

    model.articulation(
        "frame_to_side_shelf",
        ArticulationType.FIXED,
        parent=cart_frame,
        child=side_shelf,
        origin=Origin(),
    )
    model.articulation(
        "frame_to_lower_chamber",
        ArticulationType.FIXED,
        parent=cart_frame,
        child=lower_chamber,
        origin=Origin(xyz=(0.0, 0.0, CHAMBER_CENTER_Z)),
    )
    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=lower_chamber,
        child=lid,
        origin=Origin(xyz=(0.0, -CHAMBER_RADIUS, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=22.0,
            velocity=1.4,
            lower=0.0,
            upper=math.radians(105.0),
        ),
    )
    model.articulation(
        "left_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=cart_frame,
        child=left_wheel,
        origin=Origin(xyz=(-0.34, AXLE_Y, AXLE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=20.0),
    )
    model.articulation(
        "right_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=cart_frame,
        child=right_wheel,
        origin=Origin(xyz=(0.34, AXLE_Y, AXLE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=20.0),
    )
    model.articulation(
        "damper_rotate",
        ArticulationType.REVOLUTE,
        parent=lower_chamber,
        child=damper,
        origin=Origin(
            xyz=(CHAMBER_LENGTH * 0.5 - END_PLATE_THICKNESS * 0.5, 0.09, -0.03),
        ),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=3.0,
            lower=0.0,
            upper=math.pi / 2.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    cart_frame = object_model.get_part("cart_frame")
    side_shelf = object_model.get_part("side_shelf")
    lower_chamber = object_model.get_part("lower_chamber")
    lid = object_model.get_part("lid")
    left_wheel = object_model.get_part("left_wheel")
    right_wheel = object_model.get_part("right_wheel")
    damper = object_model.get_part("damper")

    lid_hinge = object_model.get_articulation("lid_hinge")
    left_wheel_spin = object_model.get_articulation("left_wheel_spin")
    right_wheel_spin = object_model.get_articulation("right_wheel_spin")
    damper_rotate = object_model.get_articulation("damper_rotate")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()

    ctx.allow_overlap(
        cart_frame,
        left_wheel,
        reason="Left wheel hub spins around the cart axle.",
    )
    ctx.allow_overlap(
        cart_frame,
        right_wheel,
        reason="Right wheel hub spins around the cart axle.",
    )
    ctx.allow_overlap(
        lower_chamber,
        lid,
        reason="Rear hinge rod passes through the lid hinge knuckles.",
    )
    ctx.allow_overlap(
        lower_chamber,
        damper,
        reason="Damper shaft passes through the chamber end wall.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "lid_hinge_axis_is_longitudinal",
        _vec_close(lid_hinge.axis, (1.0, 0.0, 0.0)),
        f"Expected longitudinal x-axis hinge, got {lid_hinge.axis}.",
    )
    ctx.check(
        "wheel_axes_are_longitudinal",
        _vec_close(left_wheel_spin.axis, (1.0, 0.0, 0.0))
        and _vec_close(right_wheel_spin.axis, (1.0, 0.0, 0.0)),
        f"Wheel axes were {left_wheel_spin.axis} and {right_wheel_spin.axis}.",
    )
    ctx.check(
        "damper_axis_is_longitudinal",
        _vec_close(damper_rotate.axis, (1.0, 0.0, 0.0)),
        f"Expected x-axis damper shaft, got {damper_rotate.axis}.",
    )

    lid_limits = lid_hinge.motion_limits
    damper_limits = damper_rotate.motion_limits
    ctx.check(
        "lid_limit_is_realistic",
        lid_limits is not None
        and lid_limits.lower == 0.0
        and lid_limits.upper is not None
        and math.radians(95.0) <= lid_limits.upper <= math.radians(115.0),
        f"Lid limits were {None if lid_limits is None else (lid_limits.lower, lid_limits.upper)}.",
    )
    ctx.check(
        "damper_limit_is_quarter_turn",
        damper_limits is not None
        and damper_limits.lower == 0.0
        and damper_limits.upper is not None
        and abs(damper_limits.upper - math.pi / 2.0) <= 1e-6,
        f"Damper limits were {None if damper_limits is None else (damper_limits.lower, damper_limits.upper)}.",
    )

    ctx.expect_contact(cart_frame, side_shelf, name="side_shelf_supported_by_frame")
    ctx.expect_contact(cart_frame, lower_chamber, name="lower_chamber_supported_by_frame")
    ctx.expect_contact(lid, lower_chamber, name="lid_connected_to_lower_chamber")
    ctx.expect_contact(left_wheel, cart_frame, name="left_wheel_connected_to_axle")
    ctx.expect_contact(right_wheel, cart_frame, name="right_wheel_connected_to_axle")
    ctx.expect_contact(
        damper,
        lower_chamber,
        elem_a="damper_shaft",
        elem_b="lower_end_plate_right",
        name="damper_connected_to_chamber",
    )
    ctx.expect_overlap(lid, lower_chamber, axes="x", min_overlap=0.55, name="lid_spans_chamber_length")
    ctx.expect_overlap(left_wheel, cart_frame, axes="z", min_overlap=0.25, name="left_wheel_overlaps_axle_height")
    ctx.expect_overlap(right_wheel, cart_frame, axes="z", min_overlap=0.25, name="right_wheel_overlaps_axle_height")

    frame_aabb = ctx.part_world_aabb(cart_frame)
    shelf_part_aabb = ctx.part_world_aabb(side_shelf)
    lower_aabb = ctx.part_world_aabb(lower_chamber)
    lid_closed_aabb = ctx.part_world_aabb(lid)
    shelf_aabb = ctx.part_element_world_aabb(side_shelf, elem="side_shelf_slat_0")
    assert frame_aabb is not None
    assert shelf_part_aabb is not None
    assert lower_aabb is not None
    assert lid_closed_aabb is not None
    assert shelf_aabb is not None
    ctx.check(
        "side_shelf_projects_beyond_chamber",
        shelf_aabb[0][0] < lower_aabb[0][0] - 0.12,
        f"Shelf min x {shelf_aabb[0][0]:.3f} did not extend far enough past chamber min x {lower_aabb[0][0]:.3f}.",
    )
    overall_max_z = max(frame_aabb[1][2], shelf_part_aabb[1][2], lower_aabb[1][2], lid_closed_aabb[1][2])
    ctx.check(
        "grill_has_realistic_height",
        0.95 <= overall_max_z <= 1.20,
        f"Overall max z was {overall_max_z:.3f} m.",
    )

    lid_rest_aabb = ctx.part_world_aabb(lid)
    handle_rest_aabb = ctx.part_element_world_aabb(lid, elem="handle_bar")
    assert lid_rest_aabb is not None
    assert handle_rest_aabb is not None
    handle_rest_center = _aabb_center(handle_rest_aabb)
    if lid_limits is not None and lid_limits.upper is not None:
        with ctx.pose({lid_hinge: lid_limits.upper}):
            lid_open_aabb = ctx.part_world_aabb(lid)
            handle_open_aabb = ctx.part_element_world_aabb(lid, elem="handle_bar")
            assert lid_open_aabb is not None
            assert handle_open_aabb is not None
            handle_open_center = _aabb_center(handle_open_aabb)
            ctx.fail_if_parts_overlap_in_current_pose(name="lid_open_pose_no_overlap")
            ctx.fail_if_isolated_parts(name="lid_open_pose_no_floating")
            ctx.expect_contact(lid, lower_chamber, name="lid_open_pose_hinge_contact")
            ctx.check(
                "lid_rises_when_opened",
                handle_open_center[1] < handle_rest_center[1] - 0.30
                and handle_open_center[2] > handle_rest_center[2] + 0.06,
                f"Handle center moved from {handle_rest_center} to {handle_open_center}.",
            )

    damper_handle_rest = ctx.part_element_world_aabb(damper, elem="damper_handle")
    assert damper_handle_rest is not None
    damper_handle_rest_center = _aabb_center(damper_handle_rest)
    if damper_limits is not None and damper_limits.upper is not None:
        with ctx.pose({damper_rotate: damper_limits.upper}):
            damper_handle_open = ctx.part_element_world_aabb(damper, elem="damper_handle")
            assert damper_handle_open is not None
            damper_handle_open_center = _aabb_center(damper_handle_open)
            ctx.fail_if_parts_overlap_in_current_pose(name="damper_open_pose_no_overlap")
            ctx.fail_if_isolated_parts(name="damper_open_pose_no_floating")
            ctx.expect_contact(
                damper,
                lower_chamber,
                elem_a="damper_shaft",
                elem_b="lower_end_plate_right",
                name="damper_open_pose_contact",
            )
            ctx.check(
                "damper_handle_rotates_out_of_plane",
                abs(damper_handle_open_center[2] - damper_handle_rest_center[2]) > 0.03,
                f"Damper handle center z changed from {damper_handle_rest_center[2]:.3f} to {damper_handle_open_center[2]:.3f}.",
            )

    left_stem_rest = ctx.part_element_world_aabb(left_wheel, elem="valve_stem")
    right_stem_rest = ctx.part_element_world_aabb(right_wheel, elem="valve_stem")
    assert left_stem_rest is not None
    assert right_stem_rest is not None
    left_stem_rest_center = _aabb_center(left_stem_rest)
    right_stem_rest_center = _aabb_center(right_stem_rest)
    with ctx.pose({left_wheel_spin: math.pi / 2.0, right_wheel_spin: math.pi / 3.0}):
        left_stem_turn = ctx.part_element_world_aabb(left_wheel, elem="valve_stem")
        right_stem_turn = ctx.part_element_world_aabb(right_wheel, elem="valve_stem")
        assert left_stem_turn is not None
        assert right_stem_turn is not None
        left_stem_turn_center = _aabb_center(left_stem_turn)
        right_stem_turn_center = _aabb_center(right_stem_turn)
        ctx.fail_if_parts_overlap_in_current_pose(name="wheel_rotation_pose_no_overlap")
        ctx.fail_if_isolated_parts(name="wheel_rotation_pose_no_floating")
        ctx.expect_contact(left_wheel, cart_frame, name="left_wheel_rotation_pose_contact")
        ctx.expect_contact(right_wheel, cart_frame, name="right_wheel_rotation_pose_contact")
        ctx.check(
            "left_wheel_rotation_is_observable",
            abs(left_stem_turn_center[1] - left_stem_rest_center[1]) > 0.09
            or abs(left_stem_turn_center[2] - left_stem_rest_center[2]) > 0.09,
            f"Left valve stem moved from {left_stem_rest_center} to {left_stem_turn_center}.",
        )
        ctx.check(
            "right_wheel_rotation_is_observable",
            abs(right_stem_turn_center[1] - right_stem_rest_center[1]) > 0.07
            or abs(right_stem_turn_center[2] - right_stem_rest_center[2]) > 0.07,
            f"Right valve stem moved from {right_stem_rest_center} to {right_stem_turn_center}.",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
