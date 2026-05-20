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
    ExtrudeWithHolesGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    wire_from_points,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _circle_profile(radius: float, *, segments: int = 28) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos(2.0 * math.pi * index / segments),
            radius * math.sin(2.0 * math.pi * index / segments),
        )
        for index in range(segments)
    ]


def _annulus_tube_geometry(
    *,
    length: float,
    outer_radius: float,
    inner_radius: float,
) -> object:
    return ExtrudeWithHolesGeometry(
        _circle_profile(outer_radius),
        [_circle_profile(inner_radius)],
        length,
        center=True,
    ).rotate_y(math.pi / 2.0)


def _build_side_rail_mesh(
    name: str,
    *,
    rail_length: float,
    rail_height: float,
    tube_radius: float,
    inward_sign: float,
    sleeve_outer_radius: float,
    sleeve_inner_radius: float,
) -> object:
    half_length = rail_length * 0.5
    frame_z = 0.016
    lower_y = inward_sign * 0.020
    upper_y = inward_sign * rail_height
    mid_y = inward_sign * (rail_height * 0.55)

    geom = wire_from_points(
        [
            (-half_length, lower_y, frame_z),
            (-half_length, upper_y, frame_z),
            (half_length, upper_y, frame_z),
            (half_length, lower_y, frame_z),
        ],
        radius=tube_radius,
        radial_segments=18,
        closed_path=True,
        corner_mode="fillet",
        corner_radius=0.030,
        corner_segments=10,
    )
    geom.merge(
        wire_from_points(
            [
                (-half_length, mid_y, frame_z),
                (half_length, mid_y, frame_z),
            ],
            radius=tube_radius * 0.82,
            radial_segments=16,
            cap_ends=True,
            corner_mode="miter",
        )
    )

    for sleeve_x in (-0.24, 0.0, 0.24):
        geom.merge(
            _annulus_tube_geometry(
                length=0.090,
                outer_radius=sleeve_outer_radius,
                inner_radius=sleeve_inner_radius,
            ).translate(sleeve_x, 0.0, 0.0)
        )
        geom.merge(
            wire_from_points(
                [
                    (sleeve_x - 0.035, inward_sign * 0.006, 0.006),
                    (sleeve_x + 0.035, inward_sign * 0.006, 0.006),
                ],
                radius=0.007,
                radial_segments=12,
                cap_ends=True,
                corner_mode="miter",
            )
        )

    return _save_mesh(name, geom)


def _build_push_handle_mesh(
    name: str,
    *,
    width: float,
    height: float,
    tube_radius: float,
) -> object:
    half_width = width * 0.5
    geom = wire_from_points(
        [
            (0.0, -half_width, 0.0),
            (-0.028, -half_width, height * 0.55),
            (-0.065, -half_width, height),
            (-0.065, half_width, height),
            (-0.028, half_width, height * 0.55),
            (0.0, half_width, 0.0),
        ],
        radius=tube_radius,
        radial_segments=18,
        cap_ends=True,
        corner_mode="fillet",
        corner_radius=0.060,
        corner_segments=12,
    )
    geom.merge(
        wire_from_points(
            [
                (-0.030, -half_width, height * 0.34),
                (-0.030, half_width, height * 0.34),
            ],
            radius=tube_radius * 0.74,
            radial_segments=14,
            cap_ends=True,
            corner_mode="miter",
        )
    )
    return _save_mesh(name, geom)


def _build_caster_wheel_mesh(
    name: str,
    *,
    radius: float,
    width: float,
) -> object:
    geom = ExtrudeWithHolesGeometry(
        _circle_profile(radius, segments=40),
        [_circle_profile(radius * 0.34, segments=28)],
        width,
        center=True,
    ).rotate_x(math.pi / 2.0)
    return _save_mesh(name, geom)


def _axis_matches(
    axis: tuple[float, float, float] | None,
    expected: tuple[float, float, float],
) -> bool:
    if axis is None:
        return False
    return all(abs(component - goal) <= 1e-9 for component, goal in zip(axis, expected))


def _add_caster_assembly(
    model: ArticulatedObject,
    deck,
    *,
    prefix: str,
    position: tuple[float, float],
    caster_metal,
    wheel_rubber,
    wheel_mesh,
) -> tuple[object, object, object, object]:
    x_pos, y_pos = position

    caster = model.part(f"{prefix}_caster")
    caster.visual(
        Cylinder(radius=0.022, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, -0.003)),
        material=caster_metal,
        name="swivel_lower_race",
    )
    caster.visual(
        Cylinder(radius=0.009, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, -0.014)),
        material=caster_metal,
        name="swivel_stem",
    )
    caster.visual(
        Box((0.038, 0.040, 0.008)),
        origin=Origin(xyz=(0.010, 0.0, -0.026)),
        material=caster_metal,
        name="fork_crown",
    )
    caster.visual(
        Box((0.006, 0.004, 0.096)),
        origin=Origin(xyz=(0.026, -0.013, -0.070)),
        material=caster_metal,
        name="fork_left_leg",
    )
    caster.visual(
        Box((0.006, 0.004, 0.096)),
        origin=Origin(xyz=(0.026, 0.013, -0.070)),
        material=caster_metal,
        name="fork_right_leg",
    )
    caster.visual(
        Cylinder(radius=0.006, length=0.006),
        origin=Origin(xyz=(0.026, -0.013, -0.088), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=caster_metal,
        name="left_axle_stub",
    )
    caster.visual(
        Cylinder(radius=0.006, length=0.006),
        origin=Origin(xyz=(0.026, 0.013, -0.088), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=caster_metal,
        name="right_axle_stub",
    )
    caster.inertial = Inertial.from_geometry(
        Box((0.050, 0.032, 0.126)),
        mass=1.2,
        origin=Origin(xyz=(0.014, 0.0, -0.066)),
    )

    wheel = model.part(f"{prefix}_wheel")
    wheel.visual(
        Cylinder(radius=0.056, length=0.020),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=wheel_rubber,
        name="tire",
    )
    wheel.visual(
        Cylinder(radius=0.018, length=0.016),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=caster_metal,
        name="hub",
    )
    wheel.visual(
        Cylinder(radius=0.012, length=0.004),
        origin=Origin(xyz=(0.0, -0.008, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=caster_metal,
        name="left_hub_cap",
    )
    wheel.visual(
        Cylinder(radius=0.012, length=0.004),
        origin=Origin(xyz=(0.0, 0.008, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=caster_metal,
        name="right_hub_cap",
    )
    wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.056, length=0.020),
        mass=1.0,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
    )

    swivel = model.articulation(
        f"{prefix}_caster_swivel",
        ArticulationType.CONTINUOUS,
        parent=deck,
        child=caster,
        origin=Origin(xyz=(x_pos, y_pos, 0.144)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=5.0),
    )
    spin = model.articulation(
        f"{prefix}_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=caster,
        child=wheel,
        origin=Origin(xyz=(0.026, 0.0, -0.088)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=14.0, velocity=20.0),
    )
    return caster, wheel, swivel, spin


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="platform_cart")

    deck_blue = model.material("deck_blue", rgba=(0.24, 0.42, 0.78, 1.0))
    rail_steel = model.material("rail_steel", rgba=(0.78, 0.80, 0.82, 1.0))
    caster_metal = model.material("caster_metal", rgba=(0.66, 0.68, 0.70, 1.0))
    wheel_rubber = model.material("wheel_rubber", rgba=(0.08, 0.08, 0.09, 1.0))
    deck_pad = model.material("deck_pad", rgba=(0.16, 0.17, 0.18, 1.0))
    bumper_rubber = model.material("bumper_rubber", rgba=(0.14, 0.14, 0.15, 1.0))

    rail_length = 0.720
    rail_height = 0.188
    rail_tube_radius = 0.011
    hinge_axis_z = 0.216
    hinge_y = 0.258
    wheel_radius = 0.056
    wheel_width = 0.024

    left_rail_mesh = _build_side_rail_mesh(
        "platform_cart_left_side_rail",
        rail_length=rail_length,
        rail_height=rail_height,
        tube_radius=rail_tube_radius,
        inward_sign=-1.0,
        sleeve_outer_radius=0.010,
        sleeve_inner_radius=0.0065,
    )
    right_rail_mesh = _build_side_rail_mesh(
        "platform_cart_right_side_rail",
        rail_length=rail_length,
        rail_height=rail_height,
        tube_radius=rail_tube_radius,
        inward_sign=1.0,
        sleeve_outer_radius=0.010,
        sleeve_inner_radius=0.0065,
    )
    push_handle_mesh = _build_push_handle_mesh(
        "platform_cart_push_handle",
        width=0.420,
        height=0.740,
        tube_radius=0.015,
    )
    caster_wheel_mesh = _build_caster_wheel_mesh(
        "platform_cart_caster_wheel",
        radius=wheel_radius,
        width=wheel_width,
    )

    deck = model.part("deck")
    deck.visual(
        Box((0.920, 0.580, 0.028)),
        origin=Origin(xyz=(0.0, 0.0, 0.192)),
        material=deck_blue,
        name="deck_plate",
    )
    deck.visual(
        Box((0.840, 0.500, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.208)),
        material=deck_pad,
        name="deck_pad",
    )
    deck.visual(
        Box((0.920, 0.040, 0.036)),
        origin=Origin(xyz=(0.0, -0.240, 0.160)),
        material=deck_blue,
        name="left_side_channel",
    )
    deck.visual(
        Box((0.920, 0.040, 0.036)),
        origin=Origin(xyz=(0.0, 0.240, 0.160)),
        material=deck_blue,
        name="right_side_channel",
    )
    deck.visual(
        Box((0.060, 0.500, 0.036)),
        origin=Origin(xyz=(0.400, 0.0, 0.160)),
        material=deck_blue,
        name="front_cross_member",
    )
    deck.visual(
        Box((0.080, 0.520, 0.036)),
        origin=Origin(xyz=(-0.400, 0.0, 0.160)),
        material=deck_blue,
        name="rear_cross_member",
    )
    deck.visual(
        Box((0.055, 0.048, 0.028)),
        origin=Origin(xyz=(-0.424, -0.180, 0.192)),
        material=deck_blue,
        name="left_handle_socket",
    )
    deck.visual(
        Box((0.055, 0.048, 0.028)),
        origin=Origin(xyz=(-0.424, 0.180, 0.192)),
        material=deck_blue,
        name="right_handle_socket",
    )
    deck.visual(
        Box((0.740, 0.032, 0.020)),
        origin=Origin(xyz=(0.0, hinge_y, 0.196)),
        material=deck_blue,
        name="left_hinge_base",
    )
    deck.visual(
        Box((0.740, 0.032, 0.020)),
        origin=Origin(xyz=(0.0, -hinge_y, 0.196)),
        material=deck_blue,
        name="right_hinge_base",
    )
    for corner_name, x_pos, y_pos in (
        ("front_left", 0.340, 0.220),
        ("front_right", 0.340, -0.220),
        ("rear_left", -0.340, 0.220),
        ("rear_right", -0.340, -0.220),
    ):
        deck.visual(
            Box((0.080, 0.060, 0.010)),
            origin=Origin(xyz=(x_pos, y_pos, 0.173)),
            material=caster_metal,
            name=f"{corner_name}_caster_plate",
        )
        deck.visual(
            Cylinder(radius=0.026, length=0.012),
            origin=Origin(xyz=(x_pos, y_pos, 0.174)),
            material=caster_metal,
            name=f"{corner_name}_caster_race",
        )
    for x_pos, y_pos in (
        (0.440, 0.270),
        (0.440, -0.270),
        (-0.440, 0.270),
        (-0.440, -0.270),
    ):
        deck.visual(
            Cylinder(radius=0.020, length=0.022),
            origin=Origin(xyz=(x_pos, y_pos, 0.184)),
            material=bumper_rubber,
            name=f"corner_bumper_{'p' if x_pos > 0 else 'n'}{'p' if y_pos > 0 else 'n'}",
        )
    deck.inertial = Inertial.from_geometry(
        Box((0.920, 0.580, 0.220)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, 0.170)),
    )

    push_handle = model.part("push_handle")
    push_handle.visual(
        push_handle_mesh,
        material=rail_steel,
        name="handle_tube",
    )
    push_handle.visual(
        Box((0.052, 0.040, 0.010)),
        origin=Origin(xyz=(0.0, -0.210, 0.005)),
        material=rail_steel,
        name="left_handle_foot",
    )
    push_handle.visual(
        Box((0.052, 0.040, 0.010)),
        origin=Origin(xyz=(0.0, 0.210, 0.005)),
        material=rail_steel,
        name="right_handle_foot",
    )
    push_handle.inertial = Inertial.from_geometry(
        Box((0.080, 0.460, 0.760)),
        mass=3.4,
        origin=Origin(xyz=(-0.030, 0.0, 0.370)),
    )

    left_side_rail = model.part("left_side_rail")
    left_side_rail.visual(left_rail_mesh, material=rail_steel, name="left_side_rail_frame")
    for index, x_pos in enumerate((-0.200, 0.0, 0.200)):
        left_side_rail.visual(
            Box((0.048, 0.026, 0.026)),
            origin=Origin(xyz=(x_pos, -0.100, 0.003)),
            material=bumper_rubber,
            name=f"left_rail_rest_pad_{index}",
        )
    left_side_rail.inertial = Inertial.from_geometry(
        Box((0.720, 0.210, 0.060)),
        mass=2.8,
        origin=Origin(xyz=(0.0, -0.094, 0.020)),
    )

    right_side_rail = model.part("right_side_rail")
    right_side_rail.visual(right_rail_mesh, material=rail_steel, name="right_side_rail_frame")
    for index, x_pos in enumerate((-0.200, 0.0, 0.200)):
        right_side_rail.visual(
            Box((0.048, 0.026, 0.026)),
            origin=Origin(xyz=(x_pos, 0.100, 0.003)),
            material=bumper_rubber,
            name=f"right_rail_rest_pad_{index}",
        )
    right_side_rail.inertial = Inertial.from_geometry(
        Box((0.720, 0.210, 0.060)),
        mass=2.8,
        origin=Origin(xyz=(0.0, 0.094, 0.020)),
    )

    model.articulation(
        "deck_to_push_handle",
        ArticulationType.FIXED,
        parent=deck,
        child=push_handle,
        origin=Origin(xyz=(-0.424, 0.0, 0.206)),
    )
    model.articulation(
        "left_side_hinge",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=left_side_rail,
        origin=Origin(xyz=(0.0, hinge_y, hinge_axis_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.5,
            lower=-math.pi / 2.0,
            upper=0.0,
        ),
    )
    model.articulation(
        "right_side_hinge",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=right_side_rail,
        origin=Origin(xyz=(0.0, -hinge_y, hinge_axis_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.5,
            lower=0.0,
            upper=math.pi / 2.0,
        ),
    )

    _add_caster_assembly(
        model,
        deck,
        prefix="front_left",
        position=(0.340, 0.220),
        caster_metal=caster_metal,
        wheel_rubber=wheel_rubber,
        wheel_mesh=caster_wheel_mesh,
    )
    _add_caster_assembly(
        model,
        deck,
        prefix="front_right",
        position=(0.340, -0.220),
        caster_metal=caster_metal,
        wheel_rubber=wheel_rubber,
        wheel_mesh=caster_wheel_mesh,
    )
    _add_caster_assembly(
        model,
        deck,
        prefix="rear_left",
        position=(-0.340, 0.220),
        caster_metal=caster_metal,
        wheel_rubber=wheel_rubber,
        wheel_mesh=caster_wheel_mesh,
    )
    _add_caster_assembly(
        model,
        deck,
        prefix="rear_right",
        position=(-0.340, -0.220),
        caster_metal=caster_metal,
        wheel_rubber=wheel_rubber,
        wheel_mesh=caster_wheel_mesh,
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
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

    deck = object_model.get_part("deck")
    push_handle = object_model.get_part("push_handle")
    left_side_rail = object_model.get_part("left_side_rail")
    right_side_rail = object_model.get_part("right_side_rail")

    left_hinge = object_model.get_articulation("left_side_hinge")
    right_hinge = object_model.get_articulation("right_side_hinge")

    caster_joint_names = (
        "front_left_caster_swivel",
        "front_right_caster_swivel",
        "rear_left_caster_swivel",
        "rear_right_caster_swivel",
    )
    wheel_joint_names = (
        "front_left_wheel_spin",
        "front_right_wheel_spin",
        "rear_left_wheel_spin",
        "rear_right_wheel_spin",
    )

    caster_parts = [object_model.get_part(name) for name in (
        "front_left_caster",
        "front_right_caster",
        "rear_left_caster",
        "rear_right_caster",
    )]
    wheel_parts = [object_model.get_part(name) for name in (
        "front_left_wheel",
        "front_right_wheel",
        "rear_left_wheel",
        "rear_right_wheel",
    )]

    ctx.expect_contact(push_handle, deck, name="push handle mounts to deck")
    ctx.expect_contact(left_side_rail, deck, name="left rail rests on deck")
    ctx.expect_contact(right_side_rail, deck, name="right rail rests on deck")
    ctx.expect_within(left_side_rail, deck, axes="xy", margin=0.025, name="left rail folds within deck")
    ctx.expect_within(right_side_rail, deck, axes="xy", margin=0.025, name="right rail folds within deck")

    for caster_part in caster_parts:
        ctx.expect_contact(caster_part, deck, name=f"{caster_part.name} seats against deck")
    for caster_part, wheel_part in zip(caster_parts, wheel_parts):
        ctx.expect_contact(wheel_part, caster_part, name=f"{wheel_part.name} mounts in fork")

    ctx.check(
        "left rail hinge axis",
        _axis_matches(left_hinge.axis, (1.0, 0.0, 0.0)),
        details=f"expected x-axis hinge, got {left_hinge.axis}",
    )
    ctx.check(
        "right rail hinge axis",
        _axis_matches(right_hinge.axis, (1.0, 0.0, 0.0)),
        details=f"expected x-axis hinge, got {right_hinge.axis}",
    )

    left_limits = left_hinge.motion_limits
    right_limits = right_hinge.motion_limits
    ctx.check(
        "left rail hinge limits",
        left_limits is not None
        and left_limits.lower is not None
        and left_limits.upper is not None
        and left_limits.lower <= -1.5
        and abs(left_limits.upper) <= 1e-9,
        details=f"unexpected left hinge limits: {left_limits}",
    )
    ctx.check(
        "right rail hinge limits",
        right_limits is not None
        and right_limits.lower is not None
        and right_limits.upper is not None
        and abs(right_limits.lower) <= 1e-9
        and right_limits.upper >= 1.5,
        details=f"unexpected right hinge limits: {right_limits}",
    )

    for joint_name in caster_joint_names:
        joint = object_model.get_articulation(joint_name)
        ctx.check(
            f"{joint_name} axis",
            _axis_matches(joint.axis, (0.0, 0.0, 1.0)),
            details=f"expected vertical swivel axis, got {joint.axis}",
        )
    for joint_name in wheel_joint_names:
        joint = object_model.get_articulation(joint_name)
        ctx.check(
            f"{joint_name} axis",
            _axis_matches(joint.axis, (0.0, 1.0, 0.0)),
            details=f"expected axle-aligned wheel axis, got {joint.axis}",
        )

    deck_aabb = ctx.part_world_aabb(deck)
    handle_aabb = ctx.part_world_aabb(push_handle)
    assert deck_aabb is not None
    assert handle_aabb is not None
    ctx.check(
        "push handle rises above deck",
        handle_aabb[1][2] > deck_aabb[1][2] + 0.65,
        details=f"handle top z={handle_aabb[1][2]:.3f}, deck top z={deck_aabb[1][2]:.3f}",
    )

    with ctx.pose({left_hinge: -math.pi / 2.0, right_hinge: math.pi / 2.0}):
        upright_left_aabb = ctx.part_world_aabb(left_side_rail)
        upright_right_aabb = ctx.part_world_aabb(right_side_rail)
        upright_deck_aabb = ctx.part_world_aabb(deck)
        assert upright_left_aabb is not None
        assert upright_right_aabb is not None
        assert upright_deck_aabb is not None
        ctx.check(
            "left rail stands upright",
            upright_left_aabb[1][2] > upright_deck_aabb[1][2] + 0.17,
            details=f"left rail top z={upright_left_aabb[1][2]:.3f}",
        )
        ctx.check(
            "right rail stands upright",
            upright_right_aabb[1][2] > upright_deck_aabb[1][2] + 0.17,
            details=f"right rail top z={upright_right_aabb[1][2]:.3f}",
        )
        ctx.fail_if_parts_overlap_in_current_pose(name="no overlaps with both side rails upright")

    swivel_pose = {
        object_model.get_articulation("front_left_caster_swivel"): 0.7,
        object_model.get_articulation("front_right_caster_swivel"): -0.8,
        object_model.get_articulation("rear_left_caster_swivel"): 0.5,
        object_model.get_articulation("rear_right_caster_swivel"): -0.6,
    }
    with ctx.pose(swivel_pose):
        ctx.fail_if_parts_overlap_in_current_pose(name="no overlaps with casters swiveled")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
