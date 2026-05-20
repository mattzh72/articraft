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
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    wire_from_points,
)


def _caster_wheel_mesh():
    profile = [
        (0.0, -0.007),
        (0.018, -0.007),
        (0.030, -0.010),
        (0.052, -0.015),
        (0.060, -0.016),
        (0.060, 0.016),
        (0.052, 0.015),
        (0.030, 0.010),
        (0.018, 0.007),
        (0.0, 0.007),
    ]
    return mesh_from_geometry(
        LatheGeometry(profile, segments=56).rotate_x(pi / 2.0),
        "platform_cart_caster_wheel",
    )


def _handle_main_mesh():
    loop = wire_from_points(
            [
                (0.000, -0.190, 0.010),
                (-0.020, -0.190, 0.360),
                (-0.040, -0.190, 0.780),
                (-0.040, 0.190, 0.780),
                (-0.020, 0.190, 0.360),
                (0.000, 0.190, 0.010),
            ],
            radius=0.016,
            radial_segments=18,
            cap_ends=True,
            corner_mode="fillet",
            corner_radius=0.055,
            corner_segments=12,
        )
    brace = wire_from_points(
        [
            (-0.018, -0.190, 0.344),
            (-0.018, 0.190, 0.344),
        ],
        radius=0.011,
        radial_segments=16,
        cap_ends=True,
        corner_mode="miter",
    )
    loop.merge(brace)
    return mesh_from_geometry(
        loop,
        "platform_cart_push_handle",
    )


def _aabb_center(aabb):
    return tuple((aabb[0][axis] + aabb[1][axis]) * 0.5 for axis in range(3))


def _add_caster(
    model: ArticulatedObject,
    deck,
    *,
    prefix: str,
    x: float,
    y: float,
    caster_metal,
    wheel_material,
    wheel_mesh,
) -> None:
    yoke = model.part(f"{prefix}_caster_swivel")
    yoke.visual(
        Cylinder(radius=0.033, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, -0.003)),
        material=caster_metal,
        name="top_plate",
    )
    yoke.visual(
        Cylinder(radius=0.012, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, -0.019)),
        material=caster_metal,
        name="swivel_stem",
    )
    yoke.visual(
        Box((0.064, 0.032, 0.008)),
        origin=Origin(xyz=(0.020, 0.0, -0.024)),
        material=caster_metal,
        name="fork_bridge",
    )
    yoke.visual(
        Box((0.012, 0.010, 0.062)),
        origin=Origin(xyz=(0.050, -0.0205, -0.059)),
        material=caster_metal,
        name="left_fork_leg",
    )
    yoke.visual(
        Box((0.012, 0.010, 0.062)),
        origin=Origin(xyz=(0.050, 0.0205, -0.059)),
        material=caster_metal,
        name="right_fork_leg",
    )
    yoke.inertial = Inertial.from_geometry(
        Box((0.10, 0.06, 0.10)),
        mass=1.1,
        origin=Origin(xyz=(0.030, 0.0, -0.050)),
    )

    wheel = model.part(f"{prefix}_caster_wheel")
    wheel.visual(wheel_mesh, material=wheel_material, name="wheel_body")
    wheel.visual(
        Cylinder(radius=0.0016, length=0.012),
        origin=Origin(xyz=(0.0, 0.012, 0.046), rpy=(0.0, pi / 2.0, 0.0)),
        material=caster_metal,
        name="valve_stem",
    )
    wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.062, length=0.032),
        mass=0.85,
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
    )

    model.articulation(
        f"{prefix}_swivel",
        ArticulationType.REVOLUTE,
        parent=deck,
        child=yoke,
        origin=Origin(xyz=(x, y, -0.051)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=5.0,
            lower=-pi,
            upper=pi,
        ),
    )
    model.articulation(
        f"{prefix}_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=yoke,
        child=wheel,
        origin=Origin(xyz=(0.050, 0.0, -0.090)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=24.0),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="warehouse_platform_cart")

    frame_blue = model.material("frame_blue", rgba=(0.16, 0.36, 0.62, 1.0))
    deck_gray = model.material("deck_gray", rgba=(0.64, 0.67, 0.70, 1.0))
    tray_gray = model.material("tray_gray", rgba=(0.73, 0.75, 0.77, 1.0))
    caster_metal = model.material("caster_metal", rgba=(0.63, 0.65, 0.69, 1.0))
    wheel_black = model.material("wheel_black", rgba=(0.10, 0.10, 0.11, 1.0))
    grip_black = model.material("grip_black", rgba=(0.08, 0.08, 0.08, 1.0))

    deck = model.part("deck")
    deck.visual(
        Box((0.880, 0.540, 0.015)),
        origin=Origin(xyz=(0.0, 0.0, 0.0075)),
        material=deck_gray,
        name="top_panel",
    )
    deck.visual(
        Box((0.920, 0.040, 0.045)),
        origin=Origin(xyz=(0.0, -0.270, -0.0225)),
        material=frame_blue,
        name="left_frame_rail",
    )
    deck.visual(
        Box((0.920, 0.040, 0.045)),
        origin=Origin(xyz=(0.0, 0.270, -0.0225)),
        material=frame_blue,
        name="right_frame_rail",
    )
    deck.visual(
        Box((0.040, 0.500, 0.045)),
        origin=Origin(xyz=(-0.440, 0.0, -0.0225)),
        material=frame_blue,
        name="rear_frame_rail",
    )
    deck.visual(
        Box((0.040, 0.500, 0.045)),
        origin=Origin(xyz=(0.440, 0.0, -0.0225)),
        material=frame_blue,
        name="front_frame_rail",
    )
    deck.visual(
        Box((0.055, 0.040, 0.006)),
        origin=Origin(xyz=(-0.400, -0.190, 0.018)),
        material=frame_blue,
        name="left_handle_pad",
    )
    deck.visual(
        Box((0.055, 0.040, 0.006)),
        origin=Origin(xyz=(-0.400, 0.190, 0.018)),
        material=frame_blue,
        name="right_handle_pad",
    )

    rail_length = 0.660
    for side_name, side_sign in (("left", -1.0), ("right", 1.0)):
        deck.visual(
            Box((rail_length, 0.016, 0.010)),
            origin=Origin(xyz=(0.0, side_sign * 0.165, -0.005)),
            material=caster_metal,
            name=f"{side_name}_tray_rail_upper",
        )
        deck.visual(
            Box((rail_length, 0.004, 0.018)),
            origin=Origin(xyz=(0.0, side_sign * 0.169, -0.014)),
            material=caster_metal,
            name=f"{side_name}_tray_rail_web",
        )
        deck.visual(
            Box((rail_length, 0.010, 0.004)),
            origin=Origin(xyz=(0.0, side_sign * 0.163, -0.021)),
            material=caster_metal,
            name=f"{side_name}_tray_rail_lower",
        )

    for mount_name, mount_x, mount_y in (
        ("front_left", 0.360, -0.220),
        ("front_right", 0.360, 0.220),
        ("rear_left", -0.360, -0.220),
        ("rear_right", -0.360, 0.220),
    ):
        deck.visual(
            Box((0.075, 0.060, 0.006)),
            origin=Origin(xyz=(mount_x, mount_y, -0.048)),
            material=caster_metal,
            name=f"{mount_name}_caster_pad",
        )

    deck.inertial = Inertial.from_geometry(
        Box((0.920, 0.580, 0.075)),
        mass=27.0,
        origin=Origin(xyz=(0.0, 0.0, -0.012)),
    )

    handle = model.part("push_handle")
    handle.visual(
        Box((0.048, 0.034, 0.010)),
        origin=Origin(xyz=(0.0, -0.190, 0.005)),
        material=frame_blue,
        name="left_foot",
    )
    handle.visual(
        Box((0.048, 0.034, 0.010)),
        origin=Origin(xyz=(0.0, 0.190, 0.005)),
        material=frame_blue,
        name="right_foot",
    )
    handle.visual(_handle_main_mesh(), material=frame_blue, name="main_loop")
    handle.visual(
        Cylinder(radius=0.019, length=0.220),
        origin=Origin(xyz=(-0.040, 0.0, 0.780), rpy=(pi / 2.0, 0.0, 0.0)),
        material=grip_black,
        name="grip",
    )
    handle.inertial = Inertial.from_geometry(
        Box((0.090, 0.420, 0.820)),
        mass=4.8,
        origin=Origin(xyz=(-0.020, 0.0, 0.380)),
    )
    model.articulation(
        "deck_to_push_handle",
        ArticulationType.FIXED,
        parent=deck,
        child=handle,
        origin=Origin(xyz=(-0.400, 0.0, 0.021)),
    )

    tray = model.part("under_deck_tray")
    tray.visual(
        Box((0.500, 0.008, 0.009)),
        origin=Origin(xyz=(0.0, -0.160, 0.0)),
        material=caster_metal,
        name="left_runner",
    )
    tray.visual(
        Box((0.500, 0.008, 0.009)),
        origin=Origin(xyz=(0.0, 0.160, 0.0)),
        material=caster_metal,
        name="right_runner",
    )
    tray.visual(
        Box((0.532, 0.316, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, -0.027)),
        material=tray_gray,
        name="tray_bottom",
    )
    tray.visual(
        Box((0.532, 0.012, 0.030)),
        origin=Origin(xyz=(0.0, -0.150, -0.010)),
        material=tray_gray,
        name="left_wall",
    )
    tray.visual(
        Box((0.532, 0.012, 0.030)),
        origin=Origin(xyz=(0.0, 0.150, -0.010)),
        material=tray_gray,
        name="right_wall",
    )
    tray.visual(
        Box((0.012, 0.316, 0.026)),
        origin=Origin(xyz=(-0.266, 0.0, -0.012)),
        material=tray_gray,
        name="rear_wall",
    )
    tray.visual(
        Box((0.012, 0.316, 0.026)),
        origin=Origin(xyz=(0.266, 0.0, -0.012)),
        material=tray_gray,
        name="front_wall",
    )
    tray.visual(
        Cylinder(radius=0.006, length=0.180),
        origin=Origin(xyz=(0.278, 0.0, -0.002), rpy=(pi / 2.0, 0.0, 0.0)),
        material=caster_metal,
        name="tray_pull",
    )
    tray.inertial = Inertial.from_geometry(
        Box((0.560, 0.340, 0.040)),
        mass=3.8,
        origin=Origin(xyz=(0.0, 0.0, -0.014)),
    )
    model.articulation(
        "deck_to_under_deck_tray",
        ArticulationType.PRISMATIC,
        parent=deck,
        child=tray,
        origin=Origin(xyz=(0.0, 0.0, -0.0145)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=0.45,
            lower=0.0,
            upper=0.220,
        ),
    )

    wheel_mesh = _caster_wheel_mesh()
    for prefix, x, y in (
        ("front_left", 0.360, -0.220),
        ("front_right", 0.360, 0.220),
        ("rear_left", -0.360, -0.220),
        ("rear_right", -0.360, 0.220),
    ):
        _add_caster(
            model,
            deck,
            prefix=prefix,
            x=x,
            y=y,
            caster_metal=caster_metal,
            wheel_material=wheel_black,
            wheel_mesh=wheel_mesh,
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
    handle = object_model.get_part("push_handle")
    tray = object_model.get_part("under_deck_tray")
    tray_slide = object_model.get_articulation("deck_to_under_deck_tray")
    handle_mount = object_model.get_articulation("deck_to_push_handle")

    ctx.check(
        "handle_mount_fixed",
        handle_mount.articulation_type == ArticulationType.FIXED,
        f"Expected fixed handle mount, got {handle_mount.articulation_type!r}",
    )
    ctx.expect_contact(handle, deck, name="handle_seated_on_deck")

    left_runner = tray.get_visual("left_runner")
    right_runner = tray.get_visual("right_runner")
    left_rail_upper = deck.get_visual("left_tray_rail_upper")
    left_rail_lower = deck.get_visual("left_tray_rail_lower")
    right_rail_upper = deck.get_visual("right_tray_rail_upper")
    right_rail_lower = deck.get_visual("right_tray_rail_lower")

    ctx.check(
        "tray_slide_axis",
        tray_slide.axis == (1.0, 0.0, 0.0),
        f"Tray axis should be +X, got {tray_slide.axis!r}",
    )
    limits = tray_slide.motion_limits
    ctx.check(
        "tray_slide_limits",
        limits is not None and limits.lower == 0.0 and limits.upper is not None and limits.upper >= 0.22,
        f"Unexpected tray limits: {limits!r}",
    )

    ctx.expect_contact(tray, deck, elem_a=left_runner, elem_b=left_rail_upper, name="left_runner_upper_contact_rest")
    ctx.expect_contact(tray, deck, elem_a=left_runner, elem_b=left_rail_lower, name="left_runner_lower_contact_rest")
    ctx.expect_contact(tray, deck, elem_a=right_runner, elem_b=right_rail_upper, name="right_runner_upper_contact_rest")
    ctx.expect_contact(tray, deck, elem_a=right_runner, elem_b=right_rail_lower, name="right_runner_lower_contact_rest")
    ctx.expect_overlap(tray, deck, axes="xy", min_overlap=0.30, name="tray_nested_under_deck")
    ctx.expect_within(tray, deck, axes="y", margin=0.02, name="tray_stays_between_side_rails")

    tray_rest = ctx.part_world_position(tray)
    assert tray_rest is not None
    with ctx.pose({tray_slide: 0.20}):
        tray_extended = ctx.part_world_position(tray)
        assert tray_extended is not None
        ctx.check(
            "tray_moves_outward",
            tray_extended[0] > tray_rest[0] + 0.15,
            f"Tray center should shift in +X; rest={tray_rest}, extended={tray_extended}",
        )
        ctx.expect_contact(tray, deck, elem_a=left_runner, elem_b=left_rail_upper, name="left_runner_upper_contact_extended")
        ctx.expect_contact(tray, deck, elem_a=left_runner, elem_b=left_rail_lower, name="left_runner_lower_contact_extended")
        ctx.expect_contact(tray, deck, elem_a=right_runner, elem_b=right_rail_upper, name="right_runner_upper_contact_extended")
        ctx.expect_contact(tray, deck, elem_a=right_runner, elem_b=right_rail_lower, name="right_runner_lower_contact_extended")
        ctx.expect_overlap(tray, deck, axes="x", min_overlap=0.25, name="tray_remains_captured_in_rails")
        ctx.expect_within(tray, deck, axes="y", margin=0.02, name="tray_remains_centered_between_rails_extended")

    caster_prefixes = ("front_left", "front_right", "rear_left", "rear_right")
    for prefix in caster_prefixes:
        swivel_part = object_model.get_part(f"{prefix}_caster_swivel")
        wheel_part = object_model.get_part(f"{prefix}_caster_wheel")
        swivel_joint = object_model.get_articulation(f"{prefix}_swivel")
        wheel_joint = object_model.get_articulation(f"{prefix}_wheel_spin")

        ctx.check(
            f"{prefix}_swivel_axis",
            swivel_joint.axis == (0.0, 0.0, 1.0),
            f"{prefix} swivel axis should be +Z, got {swivel_joint.axis!r}",
        )
        ctx.check(
            f"{prefix}_wheel_axis",
            wheel_joint.axis == (0.0, 1.0, 0.0),
            f"{prefix} wheel axis should be +Y, got {wheel_joint.axis!r}",
        )
        ctx.check(
            f"{prefix}_wheel_is_continuous",
            wheel_joint.articulation_type == ArticulationType.CONTINUOUS,
            f"{prefix} wheel joint should be continuous, got {wheel_joint.articulation_type!r}",
        )
        ctx.expect_contact(swivel_part, deck, name=f"{prefix}_caster_contacts_deck")
        ctx.expect_contact(wheel_part, swivel_part, name=f"{prefix}_wheel_contacts_fork")

    front_left_wheel = object_model.get_part("front_left_caster_wheel")
    front_left_swivel = object_model.get_articulation("front_left_swivel")
    front_left_spin = object_model.get_articulation("front_left_wheel_spin")

    wheel_rest = ctx.part_world_position(front_left_wheel)
    assert wheel_rest is not None
    with ctx.pose({front_left_swivel: 1.0}):
        wheel_swiveled = ctx.part_world_position(front_left_wheel)
        assert wheel_swiveled is not None
        ctx.check(
            "front_left_caster_trails_when_swiveled",
            abs(wheel_swiveled[1] - wheel_rest[1]) > 0.035,
            f"Caster wheel center should swing laterally when swiveling; rest={wheel_rest}, swiveled={wheel_swiveled}",
        )
        ctx.expect_contact(front_left_wheel, object_model.get_part("front_left_caster_swivel"))

    valve_rest_aabb = ctx.part_element_world_aabb(front_left_wheel, elem="valve_stem")
    assert valve_rest_aabb is not None
    valve_rest = _aabb_center(valve_rest_aabb)
    with ctx.pose({front_left_spin: 1.5}):
        valve_spin_aabb = ctx.part_element_world_aabb(front_left_wheel, elem="valve_stem")
        assert valve_spin_aabb is not None
        valve_spun = _aabb_center(valve_spin_aabb)
        ctx.check(
            "front_left_wheel_rotates_on_axle",
            abs(valve_spun[2] - valve_rest[2]) > 0.020 or abs(valve_spun[0] - valve_rest[0]) > 0.020,
            f"Valve stem should move when the wheel spins; rest={valve_rest}, spun={valve_spun}",
        )
        ctx.expect_contact(front_left_wheel, object_model.get_part("front_left_caster_swivel"))

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
