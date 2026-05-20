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
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    wire_from_points,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _deck_shell_mesh(width: float, length: float, thickness: float):
    return ExtrudeGeometry.from_z0(
        rounded_rect_profile(width, length, radius=0.045, corner_segments=8),
        thickness,
        cap=True,
        closed=True,
    )


def _handle_frame_mesh(span: float, height: float, tube_radius: float):
    frame = wire_from_points(
        [
            (-span * 0.5, 0.0, 0.0),
            (-span * 0.5, 0.0, height * 0.88),
            (0.0, 0.0, height),
            (span * 0.5, 0.0, height * 0.88),
            (span * 0.5, 0.0, 0.0),
        ],
        radius=tube_radius,
        radial_segments=18,
        cap_ends=True,
        corner_mode="fillet",
        corner_radius=0.05,
        corner_segments=10,
    )
    return frame


def _x_cylinder(radius: float, length: float, *, xyz=(0.0, 0.0, 0.0)):
    return Cylinder(radius=radius, length=length), Origin(xyz=xyz, rpy=(0.0, math.pi / 2.0, 0.0))


def _add_outboard_wheel(
    part,
    *,
    side_sign: float,
    tire_radius: float,
    tire_width: float,
    hub_radius: float,
    hub_length: float,
    rubber,
    metal,
) -> None:
    tread, tread_origin = _x_cylinder(
        tire_radius,
        tire_width * 0.58,
        xyz=(side_sign * tire_width * 0.042, 0.0, 0.0),
    )
    shoulder_in, shoulder_in_origin = _x_cylinder(
        tire_radius * 0.94,
        tire_width * 0.34,
        xyz=(side_sign * tire_width * 0.17, 0.0, 0.0),
    )
    shoulder_out, shoulder_out_origin = _x_cylinder(
        tire_radius * 0.90,
        tire_width * 0.22,
        xyz=(side_sign * tire_width * 0.39, 0.0, 0.0),
    )
    hub, hub_origin = _x_cylinder(hub_radius, hub_length, xyz=(0.0, 0.0, 0.0))
    cap, cap_origin = _x_cylinder(
        hub_radius * 0.60,
        tire_width * 0.15,
        xyz=(side_sign * tire_width * 0.30, 0.0, 0.0),
    )

    part.visual(tread, origin=tread_origin, material=rubber, name="tread")
    part.visual(shoulder_in, origin=shoulder_in_origin, material=rubber, name="inner_shoulder")
    part.visual(shoulder_out, origin=shoulder_out_origin, material=rubber, name="outer_shoulder")
    part.visual(hub, origin=hub_origin, material=metal, name="hub_barrel")
    part.visual(cap, origin=cap_origin, material=metal, name="hub_cap")


def _add_centered_caster_wheel(
    part,
    *,
    tire_radius: float,
    tire_width: float,
    hub_radius: float,
    boss_radius: float,
    boss_length: float,
    boss_offset: float,
    rubber,
    metal,
) -> None:
    tread, tread_origin = _x_cylinder(tire_radius, tire_width * 0.56)
    shoulder_l, shoulder_l_origin = _x_cylinder(
        tire_radius * 0.93,
        tire_width * 0.24,
        xyz=(-tire_width * 0.28, 0.0, 0.0),
    )
    shoulder_r, shoulder_r_origin = _x_cylinder(
        tire_radius * 0.93,
        tire_width * 0.24,
        xyz=(tire_width * 0.28, 0.0, 0.0),
    )
    hub, hub_origin = _x_cylinder(hub_radius, tire_width * 0.72)
    boss_l, boss_l_origin = _x_cylinder(boss_radius, boss_length, xyz=(-boss_offset, 0.0, 0.0))
    boss_r, boss_r_origin = _x_cylinder(boss_radius, boss_length, xyz=(boss_offset, 0.0, 0.0))

    part.visual(tread, origin=tread_origin, material=rubber, name="tread")
    part.visual(shoulder_l, origin=shoulder_l_origin, material=rubber, name="left_shoulder")
    part.visual(shoulder_r, origin=shoulder_r_origin, material=rubber, name="right_shoulder")
    part.visual(hub, origin=hub_origin, material=metal, name="hub_barrel")
    part.visual(boss_l, origin=boss_l_origin, material=metal, name="left_boss")
    part.visual(boss_r, origin=boss_r_origin, material=metal, name="right_boss")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="stock_cart")

    deck_blue = model.material("deck_blue", rgba=(0.24, 0.40, 0.56, 1.0))
    mat_black = model.material("mat_black", rgba=(0.16, 0.17, 0.18, 1.0))
    frame_grey = model.material("frame_grey", rgba=(0.34, 0.36, 0.38, 1.0))
    steel = model.material("steel", rgba=(0.71, 0.74, 0.77, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.24, 0.25, 0.27, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.09, 1.0))

    deck_width = 0.60
    deck_length = 0.98
    deck_thickness = 0.028
    hinge_y = 0.46
    hinge_z = 0.048
    handle_span = 0.46
    handle_height = 0.82
    rear_axle_y = -0.335
    rear_axle_z = -0.102
    caster_y = 0.308
    caster_z = -0.056
    rear_wheel_mount_x = 0.247
    caster_x = 0.182

    deck = model.part("deck")
    deck.visual(
        _save_mesh("stock_cart_deck_shell", _deck_shell_mesh(deck_width, deck_length, deck_thickness)),
        material=deck_blue,
        name="deck_shell",
    )
    deck.visual(
        Box((0.48, 0.84, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, deck_thickness + 0.002)),
        material=mat_black,
        name="deck_mat",
    )
    deck.visual(
        Box((0.056, 0.78, 0.040)),
        origin=Origin(xyz=(-0.18, 0.0, -0.020)),
        material=frame_grey,
        name="left_longitudinal_rail",
    )
    deck.visual(
        Box((0.056, 0.78, 0.040)),
        origin=Origin(xyz=(0.18, 0.0, -0.020)),
        material=frame_grey,
        name="right_longitudinal_rail",
    )
    deck.visual(
        Box((0.416, 0.060, 0.040)),
        origin=Origin(xyz=(0.0, 0.332, -0.020)),
        material=frame_grey,
        name="front_crossmember",
    )
    deck.visual(
        Box((0.416, 0.060, 0.040)),
        origin=Origin(xyz=(0.0, -0.332, -0.020)),
        material=frame_grey,
        name="rear_crossmember",
    )
    deck.visual(
        Box((0.080, 0.055, 0.016)),
        origin=Origin(xyz=(-0.13, rear_axle_y, -0.048)),
        material=dark_steel,
        name="left_rear_axle_hanger",
    )
    deck.visual(
        Box((0.080, 0.055, 0.016)),
        origin=Origin(xyz=(0.13, rear_axle_y, -0.048)),
        material=dark_steel,
        name="right_rear_axle_hanger",
    )
    deck.visual(
        Box((0.100, 0.082, 0.016)),
        origin=Origin(xyz=(-caster_x, caster_y, -0.048)),
        material=dark_steel,
        name="front_left_caster_pad",
    )
    deck.visual(
        Box((0.100, 0.082, 0.016)),
        origin=Origin(xyz=(caster_x, caster_y, -0.048)),
        material=dark_steel,
        name="front_right_caster_pad",
    )

    ear_size = (0.006, 0.026, 0.040)
    for end_name, y_sign in (("front", 1.0), ("rear", -1.0)):
        y_pos = y_sign * hinge_y
        for side_name, x_center in (("left", -handle_span * 0.5), ("right", handle_span * 0.5)):
            deck.visual(
                Box(ear_size),
                origin=Origin(xyz=(x_center - 0.014, y_pos, hinge_z)),
                material=dark_steel,
                name=f"{end_name}_{side_name}_outer_ear",
            )
            deck.visual(
                Box(ear_size),
                origin=Origin(xyz=(x_center + 0.014, y_pos, hinge_z)),
                material=dark_steel,
                name=f"{end_name}_{side_name}_inner_ear",
            )

    deck.inertial = Inertial.from_geometry(
        Box((deck_width, deck_length, 0.18)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, -0.005)),
    )

    rear_axle = model.part("rear_axle")
    axle_tube, axle_tube_origin = _x_cylinder(0.018, 0.412)
    spindle_l, spindle_l_origin = _x_cylinder(0.018, 0.012, xyz=(-0.212, 0.0, 0.0))
    spindle_r, spindle_r_origin = _x_cylinder(0.018, 0.012, xyz=(0.212, 0.0, 0.0))
    rear_axle.visual(axle_tube, origin=axle_tube_origin, material=dark_steel, name="axle_tube")
    rear_axle.visual(
        Box((0.080, 0.055, 0.028)),
        origin=Origin(xyz=(-0.13, 0.0, 0.032)),
        material=dark_steel,
        name="left_saddle",
    )
    rear_axle.visual(
        Box((0.080, 0.055, 0.028)),
        origin=Origin(xyz=(0.13, 0.0, 0.032)),
        material=dark_steel,
        name="right_saddle",
    )
    rear_axle.visual(spindle_l, origin=spindle_l_origin, material=steel, name="left_spindle")
    rear_axle.visual(spindle_r, origin=spindle_r_origin, material=steel, name="right_spindle")
    rear_axle.inertial = Inertial.from_geometry(
        Box((0.45, 0.08, 0.10)),
        mass=3.2,
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
    )

    rear_left_wheel = model.part("rear_left_wheel")
    _add_outboard_wheel(
        rear_left_wheel,
        side_sign=-1.0,
        tire_radius=0.100,
        tire_width=0.086,
        hub_radius=0.040,
        hub_length=0.058,
        rubber=rubber,
        metal=steel,
    )
    rear_left_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.100, length=0.086),
        mass=2.1,
        origin=Origin(
            xyz=(-0.043, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
    )

    rear_right_wheel = model.part("rear_right_wheel")
    _add_outboard_wheel(
        rear_right_wheel,
        side_sign=1.0,
        tire_radius=0.100,
        tire_width=0.086,
        hub_radius=0.040,
        hub_length=0.058,
        rubber=rubber,
        metal=steel,
    )
    rear_right_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.100, length=0.086),
        mass=2.1,
        origin=Origin(
            xyz=(0.043, 0.0, 0.0),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
    )

    for side_name, x_sign in (("left", -1.0), ("right", 1.0)):
        caster = model.part(f"front_{side_name}_caster")
        caster.visual(
            Box((0.086, 0.064, 0.008)),
            origin=Origin(xyz=(0.0, 0.0, -0.004)),
            material=dark_steel,
            name="top_plate",
        )
        caster.visual(
            Cylinder(radius=0.021, length=0.016),
            origin=Origin(xyz=(0.0, 0.0, -0.016)),
            material=dark_steel,
            name="swivel_housing",
        )
        caster.visual(
            Cylinder(radius=0.012, length=0.028),
            origin=Origin(xyz=(0.0, 0.0, -0.038)),
            material=steel,
            name="stem",
        )
        caster.visual(
            Box((0.046, 0.030, 0.010)),
            origin=Origin(xyz=(0.0, 0.0, -0.057)),
            material=dark_steel,
            name="fork_bridge",
        )
        caster.visual(
            Box((0.008, 0.050, 0.050)),
            origin=Origin(xyz=(-0.027, 0.0, -0.087)),
            material=dark_steel,
            name="left_fork_leg",
        )
        caster.visual(
            Box((0.008, 0.050, 0.050)),
            origin=Origin(xyz=(0.027, 0.0, -0.087)),
            material=dark_steel,
            name="right_fork_leg",
        )
        caster.inertial = Inertial.from_geometry(
            Box((0.086, 0.064, 0.120)),
            mass=1.2,
            origin=Origin(xyz=(0.0, 0.0, -0.050)),
        )

        wheel = model.part(f"front_{side_name}_wheel")
        _add_centered_caster_wheel(
            wheel,
            tire_radius=0.062,
            tire_width=0.038,
            hub_radius=0.028,
            boss_radius=0.017,
            boss_length=0.009,
            boss_offset=0.0185,
            rubber=rubber,
            metal=steel,
        )
        wheel.inertial = Inertial.from_geometry(
            Cylinder(radius=0.062, length=0.038),
            mass=0.8,
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        )

        model.articulation(
            f"front_{side_name}_caster_swivel",
            ArticulationType.CONTINUOUS,
            parent=deck,
            child=caster,
            origin=Origin(xyz=(x_sign * caster_x, caster_y, caster_z)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=8.0, velocity=6.0),
        )
        model.articulation(
            f"front_{side_name}_wheel_spin",
            ArticulationType.CONTINUOUS,
            parent=caster,
            child=wheel,
            origin=Origin(xyz=(0.0, 0.0, -0.123)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=6.0, velocity=18.0),
        )

    handle_mesh = _save_mesh(
        "stock_cart_end_handle",
        _handle_frame_mesh(handle_span, handle_height, 0.012),
    )
    for handle_name in ("front_handle", "rear_handle"):
        handle = model.part(handle_name)
        handle.visual(
            handle_mesh,
            origin=Origin(xyz=(0.0, -0.032, 0.0)),
            material=frame_grey,
            name="handle_frame",
        )
        left_sleeve, left_sleeve_origin = _x_cylinder(0.014, 0.022, xyz=(-handle_span * 0.5, 0.0, 0.0))
        right_sleeve, right_sleeve_origin = _x_cylinder(0.014, 0.022, xyz=(handle_span * 0.5, 0.0, 0.0))
        grip, grip_origin = _x_cylinder(0.017, 0.220, xyz=(0.0, -0.032, handle_height))
        handle.visual(left_sleeve, origin=left_sleeve_origin, material=steel, name="left_hinge_sleeve")
        handle.visual(right_sleeve, origin=right_sleeve_origin, material=steel, name="right_hinge_sleeve")
        handle.visual(
            Box((0.024, 0.032, 0.012)),
            origin=Origin(xyz=(-handle_span * 0.5, -0.016, 0.0)),
            material=frame_grey,
            name="left_foot_connector",
        )
        handle.visual(
            Box((0.024, 0.032, 0.012)),
            origin=Origin(xyz=(handle_span * 0.5, -0.016, 0.0)),
            material=frame_grey,
            name="right_foot_connector",
        )
        handle.visual(grip, origin=grip_origin, material=mat_black, name="top_grip")
        handle.inertial = Inertial.from_geometry(
            Box((handle_span + 0.03, 0.06, handle_height + 0.04)),
            mass=2.3,
            origin=Origin(xyz=(0.0, 0.0, handle_height * 0.45)),
        )

    model.articulation(
        "deck_to_rear_axle",
        ArticulationType.FIXED,
        parent=deck,
        child=rear_axle,
        origin=Origin(xyz=(0.0, rear_axle_y, rear_axle_z)),
    )
    model.articulation(
        "rear_left_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=rear_axle,
        child=rear_left_wheel,
        origin=Origin(xyz=(-rear_wheel_mount_x, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=14.0, velocity=20.0),
    )
    model.articulation(
        "rear_right_wheel_spin",
        ArticulationType.CONTINUOUS,
        parent=rear_axle,
        child=rear_right_wheel,
        origin=Origin(xyz=(rear_wheel_mount_x, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=14.0, velocity=20.0),
    )
    model.articulation(
        "front_handle_hinge",
        ArticulationType.REVOLUTE,
        parent=deck,
        child="front_handle",
        origin=Origin(xyz=(0.0, hinge_y, hinge_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.8,
            lower=0.0,
            upper=1.45,
        ),
    )
    model.articulation(
        "rear_handle_hinge",
        ArticulationType.REVOLUTE,
        parent=deck,
        child="rear_handle",
        origin=Origin(xyz=(0.0, -hinge_y, hinge_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.8,
            lower=0.0,
            upper=1.45,
        ),
    )

    return model


def _aabb_size(aabb) -> tuple[float, float, float]:
    return (
        aabb[1][0] - aabb[0][0],
        aabb[1][1] - aabb[0][1],
        aabb[1][2] - aabb[0][2],
    )


def _aabb_center(aabb) -> tuple[float, float, float]:
    return (
        (aabb[0][0] + aabb[1][0]) * 0.5,
        (aabb[0][1] + aabb[1][1]) * 0.5,
        (aabb[0][2] + aabb[1][2]) * 0.5,
    )


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    deck = object_model.get_part("deck")
    rear_axle = object_model.get_part("rear_axle")
    rear_left_wheel = object_model.get_part("rear_left_wheel")
    rear_right_wheel = object_model.get_part("rear_right_wheel")
    front_left_caster = object_model.get_part("front_left_caster")
    front_right_caster = object_model.get_part("front_right_caster")
    front_left_wheel = object_model.get_part("front_left_wheel")
    front_right_wheel = object_model.get_part("front_right_wheel")
    front_handle = object_model.get_part("front_handle")
    rear_handle = object_model.get_part("rear_handle")

    front_left_caster_swivel = object_model.get_articulation("front_left_caster_swivel")
    front_right_caster_swivel = object_model.get_articulation("front_right_caster_swivel")
    front_left_wheel_spin = object_model.get_articulation("front_left_wheel_spin")
    front_right_wheel_spin = object_model.get_articulation("front_right_wheel_spin")
    rear_left_wheel_spin = object_model.get_articulation("rear_left_wheel_spin")
    rear_right_wheel_spin = object_model.get_articulation("rear_right_wheel_spin")
    front_handle_hinge = object_model.get_articulation("front_handle_hinge")
    rear_handle_hinge = object_model.get_articulation("rear_handle_hinge")

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

    ctx.expect_contact(rear_axle, deck)
    ctx.expect_contact(rear_left_wheel, rear_axle, contact_tol=0.0015)
    ctx.expect_contact(rear_right_wheel, rear_axle, contact_tol=0.0015)
    ctx.expect_contact(front_left_caster, deck)
    ctx.expect_contact(front_right_caster, deck)
    ctx.expect_contact(front_left_wheel, front_left_caster)
    ctx.expect_contact(front_right_wheel, front_right_caster)
    ctx.expect_contact(front_handle, deck)
    ctx.expect_contact(rear_handle, deck)

    ctx.check(
        "wheel and swivel axes are correct",
        front_left_caster_swivel.axis == (0.0, 0.0, 1.0)
        and front_right_caster_swivel.axis == (0.0, 0.0, 1.0)
        and front_left_wheel_spin.axis == (1.0, 0.0, 0.0)
        and front_right_wheel_spin.axis == (1.0, 0.0, 0.0)
        and rear_left_wheel_spin.axis == (1.0, 0.0, 0.0)
        and rear_right_wheel_spin.axis == (1.0, 0.0, 0.0),
        details="Rear wheels should spin on the axle and front casters should swivel on vertical stems.",
    )
    ctx.check(
        "end handle hinge axes fold inward",
        front_handle_hinge.axis == (1.0, 0.0, 0.0) and rear_handle_hinge.axis == (-1.0, 0.0, 0.0),
        details="Front and rear handles should hinge about deck-end pivots and fold toward the deck center.",
    )

    front_handle_rest = ctx.part_world_aabb(front_handle)
    rear_handle_rest = ctx.part_world_aabb(rear_handle)
    caster_wheel_rest = ctx.part_world_aabb(front_left_wheel)
    assert front_handle_rest is not None
    assert rear_handle_rest is not None
    assert caster_wheel_rest is not None

    with ctx.pose({front_handle_hinge: 1.20, rear_handle_hinge: 1.20}):
        front_handle_folded = ctx.part_world_aabb(front_handle)
        rear_handle_folded = ctx.part_world_aabb(rear_handle)
        assert front_handle_folded is not None
        assert rear_handle_folded is not None

        front_rest_center = _aabb_center(front_handle_rest)
        front_fold_center = _aabb_center(front_handle_folded)
        rear_rest_center = _aabb_center(rear_handle_rest)
        rear_fold_center = _aabb_center(rear_handle_folded)

        ctx.check(
            "front handle folds down toward deck",
            front_handle_folded[1][2] < front_handle_rest[1][2] - 0.25
            and front_fold_center[1] < front_rest_center[1] - 0.18,
            details="Front handle should drop and move inward when folded.",
        )
        ctx.check(
            "rear handle folds down toward deck",
            rear_handle_folded[1][2] < rear_handle_rest[1][2] - 0.25
            and rear_fold_center[1] > rear_rest_center[1] + 0.18,
            details="Rear handle should drop and move inward when folded.",
        )
        ctx.expect_contact(front_handle, deck)
        ctx.expect_contact(rear_handle, deck)

    with ctx.pose({front_left_caster_swivel: math.pi / 2.0}):
        caster_wheel_swiveled = ctx.part_world_aabb(front_left_wheel)
        assert caster_wheel_swiveled is not None
        rest_size = _aabb_size(caster_wheel_rest)
        swivel_size = _aabb_size(caster_wheel_swiveled)
        ctx.check(
            "front caster wheel turns with swivel stem",
            rest_size[1] > rest_size[0] + 0.06 and swivel_size[0] > swivel_size[1] + 0.06,
            details="A 90-degree caster swivel should rotate the wheel footprint from fore-aft to side-to-side.",
        )
        ctx.expect_contact(front_left_wheel, front_left_caster)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
