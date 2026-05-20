from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

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
    mesh_from_geometry,
    wire_from_points,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _add_wheel_visuals(
    part,
    *,
    radius: float,
    width: float,
    tread_material,
    hub_material,
    cap_material,
    hub_radius_ratio: float = 0.58,
    cap_radius_ratio: float = 0.22,
) -> None:
    roll_origin = Origin(rpy=(pi / 2.0, 0.0, 0.0))
    part.visual(
        Cylinder(radius=radius, length=width),
        origin=roll_origin,
        material=tread_material,
        name="tire",
    )
    part.visual(
        Cylinder(radius=radius * hub_radius_ratio, length=width * 0.72),
        origin=roll_origin,
        material=hub_material,
        name="hub_shell",
    )
    hub_cap_length = width * 0.18
    hub_cap = Cylinder(radius=radius * 0.42, length=hub_cap_length)
    part.visual(
        hub_cap,
        origin=Origin(
            xyz=(0.0, width * 0.29, 0.0),
            rpy=(pi / 2.0, 0.0, 0.0),
        ),
        material=cap_material,
        name="outboard_cap",
    )
    part.visual(
        hub_cap,
        origin=Origin(
            xyz=(0.0, -width * 0.29, 0.0),
            rpy=(pi / 2.0, 0.0, 0.0),
        ),
        material=cap_material,
        name="inboard_cap",
    )
    part.visual(
        Cylinder(radius=radius * cap_radius_ratio, length=width * 0.94),
        origin=roll_origin,
        material=cap_material,
        name="axle_boss",
    )


def _handle_tube_mesh() -> object:
    return _mesh(
        "platform_cart_handle_tube",
        wire_from_points(
            [
                (-0.125, 0.010, -0.004),
                (-0.125, 0.055, -0.016),
                (-0.125, 0.108, -0.084),
                (0.125, 0.108, -0.084),
                (0.125, 0.055, -0.016),
                (0.125, 0.010, -0.004),
            ],
            radius=0.012,
            radial_segments=18,
            cap_ends=True,
            corner_mode="fillet",
            corner_radius=0.040,
            corner_segments=10,
        ),
    )


def _add_caster_fork_visuals(part, *, steel_material, dark_steel_material) -> None:
    part.visual(
        Box((0.050, 0.055, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, -0.002)),
        material=steel_material,
        name="top_plate",
    )
    part.visual(
        Cylinder(radius=0.012, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, -0.013)),
        material=dark_steel_material,
        name="swivel_stem",
    )
    part.visual(
        Box((0.040, 0.050, 0.012)),
        origin=Origin(xyz=(0.010, 0.0, -0.013)),
        material=steel_material,
        name="yoke_bridge",
    )
    side_plate_size = (0.008, 0.006, 0.050)
    for y_offset, name in ((0.022, "left_cheek"), (-0.022, "right_cheek")):
        part.visual(
            Box(side_plate_size),
            origin=Origin(xyz=(0.026, y_offset, -0.044)),
            material=steel_material,
            name=name,
        )
    for y_offset, name in ((0.022, "left_axle_stub"), (-0.022, "right_axle_stub")):
        part.visual(
            Cylinder(radius=0.008, length=0.012),
            origin=Origin(
                xyz=(0.026, y_offset, -0.064),
                rpy=(pi / 2.0, 0.0, 0.0),
            ),
            material=dark_steel_material,
            name=name,
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="steel_platform_cart")

    steel = model.material("steel", rgba=(0.63, 0.67, 0.72, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.31, 0.34, 0.38, 1.0))
    wheel_rubber = model.material("wheel_rubber", rgba=(0.10, 0.10, 0.11, 1.0))
    grip_rubber = model.material("grip_rubber", rgba=(0.14, 0.14, 0.15, 1.0))

    deck = model.part("deck")
    deck.visual(
        Box((0.90, 0.60, 0.003)),
        origin=Origin(xyz=(0.0, 0.0, 0.1235)),
        material=steel,
        name="deck_top",
    )
    deck.visual(
        Box((0.90, 0.030, 0.024)),
        origin=Origin(xyz=(0.0, 0.285, 0.118)),
        material=steel,
        name="side_rail_plus_y",
    )
    deck.visual(
        Box((0.90, 0.030, 0.024)),
        origin=Origin(xyz=(0.0, -0.285, 0.118)),
        material=steel,
        name="side_rail_minus_y",
    )
    deck.visual(
        Box((0.030, 0.540, 0.024)),
        origin=Origin(xyz=(0.435, 0.0, 0.118)),
        material=steel,
        name="front_rail",
    )
    deck.visual(
        Box((0.030, 0.540, 0.024)),
        origin=Origin(xyz=(-0.435, 0.0, 0.118)),
        material=steel,
        name="rear_rail",
    )
    deck.visual(
        Box((0.760, 0.050, 0.046)),
        origin=Origin(xyz=(0.0, 0.160, 0.100)),
        material=dark_steel,
        name="left_longitudinal_beam",
    )
    deck.visual(
        Box((0.760, 0.050, 0.046)),
        origin=Origin(xyz=(0.0, -0.160, 0.100)),
        material=dark_steel,
        name="right_longitudinal_beam",
    )
    deck.visual(
        Box((0.085, 0.340, 0.046)),
        origin=Origin(xyz=(0.305, 0.0, 0.100)),
        material=dark_steel,
        name="front_crossmember",
    )
    deck.visual(
        Box((0.085, 0.340, 0.046)),
        origin=Origin(xyz=(-0.305, 0.0, 0.100)),
        material=dark_steel,
        name="rear_crossmember",
    )
    deck.visual(
        Box((0.055, 0.080, 0.010)),
        origin=Origin(xyz=(0.335, 0.230, 0.109)),
        material=steel,
        name="left_caster_mount",
    )
    deck.visual(
        Box((0.055, 0.080, 0.010)),
        origin=Origin(xyz=(0.335, -0.230, 0.109)),
        material=steel,
        name="right_caster_mount",
    )
    deck.visual(
        Box((0.020, 0.030, 0.060)),
        origin=Origin(xyz=(-0.335, 0.185, 0.089)),
        material=dark_steel,
        name="left_fixed_hanger",
    )
    deck.visual(
        Box((0.020, 0.030, 0.060)),
        origin=Origin(xyz=(-0.335, -0.185, 0.089)),
        material=dark_steel,
        name="right_fixed_hanger",
    )
    deck.visual(
        Cylinder(radius=0.012, length=0.028),
        origin=Origin(
            xyz=(-0.335, 0.192, 0.055),
            rpy=(pi / 2.0, 0.0, 0.0),
        ),
        material=dark_steel,
        name="left_axle_stub",
    )
    deck.visual(
        Cylinder(radius=0.012, length=0.028),
        origin=Origin(
            xyz=(-0.335, -0.192, 0.055),
            rpy=(pi / 2.0, 0.0, 0.0),
        ),
        material=dark_steel,
        name="right_axle_stub",
    )
    deck.visual(
        Box((0.032, 0.028, 0.040)),
        origin=Origin(xyz=(-0.115, 0.286, 0.112)),
        material=steel,
        name="left_handle_lug",
    )
    deck.visual(
        Box((0.032, 0.028, 0.040)),
        origin=Origin(xyz=(0.115, 0.286, 0.112)),
        material=steel,
        name="right_handle_lug",
    )
    deck.inertial = Inertial.from_geometry(
        Box((0.90, 0.60, 0.13)),
        mass=28.0,
        origin=Origin(xyz=(0.0, 0.0, 0.075)),
    )

    handle = model.part("handle")
    handle.visual(_handle_tube_mesh(), material=steel, name="handle_frame")
    handle.visual(
        Cylinder(radius=0.010, length=0.038),
        origin=Origin(
            xyz=(-0.115, 0.0, 0.0),
            rpy=(0.0, pi / 2.0, 0.0),
        ),
        material=dark_steel,
        name="left_hinge_barrel",
    )
    handle.visual(
        Cylinder(radius=0.010, length=0.038),
        origin=Origin(
            xyz=(0.115, 0.0, 0.0),
            rpy=(0.0, pi / 2.0, 0.0),
        ),
        material=dark_steel,
        name="right_hinge_barrel",
    )
    handle.visual(
        Box((0.026, 0.018, 0.018)),
        origin=Origin(xyz=(-0.115, 0.010, -0.004)),
        material=dark_steel,
        name="left_hinge_connector",
    )
    handle.visual(
        Box((0.026, 0.018, 0.018)),
        origin=Origin(xyz=(0.115, 0.010, -0.004)),
        material=dark_steel,
        name="right_hinge_connector",
    )
    handle.visual(
        Cylinder(radius=0.016, length=0.180),
        origin=Origin(
            xyz=(0.0, 0.108, -0.084),
            rpy=(0.0, pi / 2.0, 0.0),
        ),
        material=grip_rubber,
        name="handle_grip",
    )
    handle.inertial = Inertial.from_geometry(
        Box((0.29, 0.14, 0.11)),
        mass=2.5,
        origin=Origin(xyz=(0.0, 0.072, -0.050)),
    )

    rear_left_wheel = model.part("rear_left_wheel")
    _add_wheel_visuals(
        rear_left_wheel,
        radius=0.055,
        width=0.048,
        tread_material=wheel_rubber,
        hub_material=steel,
        cap_material=dark_steel,
    )
    rear_left_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.055, length=0.048),
        mass=2.4,
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
    )

    rear_right_wheel = model.part("rear_right_wheel")
    _add_wheel_visuals(
        rear_right_wheel,
        radius=0.055,
        width=0.048,
        tread_material=wheel_rubber,
        hub_material=steel,
        cap_material=dark_steel,
    )
    rear_right_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.055, length=0.048),
        mass=2.4,
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
    )

    front_left_caster_fork = model.part("front_left_caster_fork")
    _add_caster_fork_visuals(
        front_left_caster_fork,
        steel_material=steel,
        dark_steel_material=dark_steel,
    )
    front_left_caster_fork.inertial = Inertial.from_geometry(
        Box((0.050, 0.055, 0.070)),
        mass=0.9,
        origin=Origin(xyz=(0.012, 0.0, -0.034)),
    )

    front_right_caster_fork = model.part("front_right_caster_fork")
    _add_caster_fork_visuals(
        front_right_caster_fork,
        steel_material=steel,
        dark_steel_material=dark_steel,
    )
    front_right_caster_fork.inertial = Inertial.from_geometry(
        Box((0.050, 0.055, 0.070)),
        mass=0.9,
        origin=Origin(xyz=(0.012, 0.0, -0.034)),
    )

    front_left_caster_wheel = model.part("front_left_caster_wheel")
    _add_wheel_visuals(
        front_left_caster_wheel,
        radius=0.040,
        width=0.032,
        tread_material=wheel_rubber,
        hub_material=steel,
        cap_material=dark_steel,
        hub_radius_ratio=0.62,
        cap_radius_ratio=0.24,
    )
    front_left_caster_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.040, length=0.032),
        mass=0.8,
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
    )

    front_right_caster_wheel = model.part("front_right_caster_wheel")
    _add_wheel_visuals(
        front_right_caster_wheel,
        radius=0.040,
        width=0.032,
        tread_material=wheel_rubber,
        hub_material=steel,
        cap_material=dark_steel,
        hub_radius_ratio=0.62,
        cap_radius_ratio=0.24,
    )
    front_right_caster_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.040, length=0.032),
        mass=0.8,
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
    )

    model.articulation(
        "deck_to_handle",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=handle,
        origin=Origin(xyz=(0.0, 0.310, 0.115)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=2.2,
            lower=0.0,
            upper=1.45,
        ),
    )
    model.articulation(
        "deck_to_rear_left_wheel",
        ArticulationType.CONTINUOUS,
        parent=deck,
        child=rear_left_wheel,
        origin=Origin(xyz=(-0.335, 0.230, 0.055)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=20.0),
    )
    model.articulation(
        "deck_to_rear_right_wheel",
        ArticulationType.CONTINUOUS,
        parent=deck,
        child=rear_right_wheel,
        origin=Origin(xyz=(-0.335, -0.230, 0.055)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=20.0),
    )
    model.articulation(
        "deck_to_front_left_caster_swivel",
        ArticulationType.CONTINUOUS,
        parent=deck,
        child=front_left_caster_fork,
        origin=Origin(xyz=(0.335, 0.230, 0.104)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=8.0),
    )
    model.articulation(
        "deck_to_front_right_caster_swivel",
        ArticulationType.CONTINUOUS,
        parent=deck,
        child=front_right_caster_fork,
        origin=Origin(xyz=(0.335, -0.230, 0.104)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=8.0),
    )
    model.articulation(
        "front_left_caster_to_wheel",
        ArticulationType.CONTINUOUS,
        parent=front_left_caster_fork,
        child=front_left_caster_wheel,
        origin=Origin(xyz=(0.026, 0.0, -0.064)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=20.0),
    )
    model.articulation(
        "front_right_caster_to_wheel",
        ArticulationType.CONTINUOUS,
        parent=front_right_caster_fork,
        child=front_right_caster_wheel,
        origin=Origin(xyz=(0.026, 0.0, -0.064)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=20.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    deck = object_model.get_part("deck")
    handle = object_model.get_part("handle")
    rear_left_wheel = object_model.get_part("rear_left_wheel")
    rear_right_wheel = object_model.get_part("rear_right_wheel")
    front_left_caster_wheel = object_model.get_part("front_left_caster_wheel")
    front_right_caster_wheel = object_model.get_part("front_right_caster_wheel")

    handle_joint = object_model.get_articulation("deck_to_handle")
    rear_left_spin = object_model.get_articulation("deck_to_rear_left_wheel")
    rear_right_spin = object_model.get_articulation("deck_to_rear_right_wheel")
    front_left_swivel = object_model.get_articulation("deck_to_front_left_caster_swivel")
    front_right_swivel = object_model.get_articulation("deck_to_front_right_caster_swivel")
    front_left_spin = object_model.get_articulation("front_left_caster_to_wheel")
    front_right_spin = object_model.get_articulation("front_right_caster_to_wheel")

    def _axis_close(axis, target) -> bool:
        return axis is not None and all(abs(a - b) < 1e-6 for a, b in zip(axis, target))

    def _center_z(aabb) -> float | None:
        if aabb is None:
            return None
        return (aabb[0][2] + aabb[1][2]) * 0.5

    ctx.check(
        "handle hinge uses side-fold axis",
        handle_joint.articulation_type == ArticulationType.REVOLUTE
        and _axis_close(handle_joint.axis, (1.0, 0.0, 0.0))
        and handle_joint.motion_limits is not None
        and handle_joint.motion_limits.lower == 0.0
        and handle_joint.motion_limits.upper is not None
        and handle_joint.motion_limits.upper > 1.2,
        details=f"type={handle_joint.articulation_type}, axis={handle_joint.axis}, limits={handle_joint.motion_limits}",
    )
    for name, joint in (
        ("rear left wheel spin", rear_left_spin),
        ("rear right wheel spin", rear_right_spin),
        ("front left caster wheel spin", front_left_spin),
        ("front right caster wheel spin", front_right_spin),
    ):
        ctx.check(
            f"{name} uses rolling axis",
            joint.articulation_type == ArticulationType.CONTINUOUS
            and _axis_close(joint.axis, (0.0, 1.0, 0.0)),
            details=f"type={joint.articulation_type}, axis={joint.axis}",
        )
    for name, joint in (
        ("front left caster swivel", front_left_swivel),
        ("front right caster swivel", front_right_swivel),
    ):
        ctx.check(
            f"{name} swivels around vertical axis",
            joint.articulation_type == ArticulationType.CONTINUOUS
            and _axis_close(joint.axis, (0.0, 0.0, 1.0)),
            details=f"type={joint.articulation_type}, axis={joint.axis}",
        )

    ctx.expect_gap(
        handle,
        deck,
        axis="y",
        positive_elem="handle_grip",
        negative_elem="side_rail_plus_y",
        min_gap=0.09,
        max_gap=0.15,
        name="folded handle rests outboard of the deck edge",
    )

    folded_grip = ctx.part_element_world_aabb(handle, elem="handle_grip")
    with ctx.pose({handle_joint: handle_joint.motion_limits.upper}):
        opened_grip = ctx.part_element_world_aabb(handle, elem="handle_grip")
        ctx.expect_gap(
            handle,
            deck,
            axis="z",
            positive_elem="handle_grip",
            negative_elem="deck_top",
            min_gap=0.07,
            name="opened handle rises above the deck",
        )

    folded_z = _center_z(folded_grip)
    opened_z = _center_z(opened_grip)
    ctx.check(
        "handle grip clearly rises between folded and towing poses",
        folded_z is not None and opened_z is not None and opened_z > folded_z + 0.12,
        details=f"folded_z={folded_z}, opened_z={opened_z}",
    )

    rest_left_caster_pos = ctx.part_world_position(front_left_caster_wheel)
    with ctx.pose({front_left_swivel: 1.0}):
        turned_left_caster_pos = ctx.part_world_position(front_left_caster_wheel)
    ctx.check(
        "left caster wheel moves around the swivel pivot",
        rest_left_caster_pos is not None
        and turned_left_caster_pos is not None
        and abs(turned_left_caster_pos[1] - rest_left_caster_pos[1]) > 0.015
        and abs(turned_left_caster_pos[0] - rest_left_caster_pos[0]) > 0.005,
        details=f"rest={rest_left_caster_pos}, turned={turned_left_caster_pos}",
    )

    for wheel in (rear_left_wheel, rear_right_wheel, front_left_caster_wheel, front_right_caster_wheel):
        ctx.check(
            f"{wheel.name} sits below the deck platform",
            ctx.part_world_position(wheel) is not None
            and ctx.part_world_position(wheel)[2] < 0.08,
            details=f"wheel_pos={ctx.part_world_position(wheel)}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
