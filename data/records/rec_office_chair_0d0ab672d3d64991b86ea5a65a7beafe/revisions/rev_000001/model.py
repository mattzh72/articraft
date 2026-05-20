from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import atan2, cos, hypot, pi, sin

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
    rounded_rect_profile,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _verticalize(geometry):
    return geometry.rotate_z(pi / 2.0).rotate_y(pi / 2.0)


def _horizontal_frame_mesh(
    *,
    length: float,
    width: float,
    wall: float,
    thickness: float,
    radius: float,
    name: str,
):
    outer = rounded_rect_profile(length, width, radius, corner_segments=8)
    inner = rounded_rect_profile(
        length - 2.0 * wall,
        width - 2.0 * wall,
        max(radius - wall * 0.45, wall * 0.8),
        corner_segments=8,
    )
    return _save_mesh(
        name,
        ExtrudeWithHolesGeometry(
            outer,
            [inner],
            thickness,
            center=True,
        ),
    )


def _horizontal_panel_mesh(
    *,
    length: float,
    width: float,
    thickness: float,
    radius: float,
    name: str,
):
    return _save_mesh(
        name,
        ExtrudeGeometry.centered(
            rounded_rect_profile(length, width, radius, corner_segments=8),
            thickness,
        ),
    )


def _vertical_frame_mesh(
    *,
    width: float,
    height: float,
    wall: float,
    thickness: float,
    radius: float,
    name: str,
):
    outer = rounded_rect_profile(width, height, radius, corner_segments=8)
    inner = rounded_rect_profile(
        width - 2.0 * wall,
        height - 2.0 * wall,
        max(radius - wall * 0.45, wall * 0.8),
        corner_segments=8,
    )
    geom = ExtrudeWithHolesGeometry(
        outer,
        [inner],
        thickness,
        center=True,
    )
    return _save_mesh(name, _verticalize(geom))


def _vertical_panel_mesh(
    *,
    width: float,
    height: float,
    thickness: float,
    radius: float,
    name: str,
):
    geom = ExtrudeGeometry.centered(
        rounded_rect_profile(width, height, radius, corner_segments=8),
        thickness,
    )
    return _save_mesh(name, _verticalize(geom))


def _build_armrest_part(model: ArticulatedObject, name: str, side_sign: float, arm_frame, arm_pad):
    arm = model.part(name)
    arm.visual(
        Cylinder(radius=0.014, length=0.07),
        origin=Origin(
            xyz=(0.0, side_sign * 0.014, 0.0),
            rpy=(0.0, pi / 2.0, 0.0),
        ),
        material=arm_frame,
        name="arm_barrel",
    )
    arm.visual(
        Box((0.05, 0.032, 0.030)),
        origin=Origin(xyz=(0.0, side_sign * 0.028, 0.015)),
        material=arm_frame,
        name="arm_knuckle",
    )
    arm.visual(
        Box((0.035, 0.045, 0.20)),
        origin=Origin(xyz=(0.0, side_sign * 0.052, 0.10)),
        material=arm_frame,
        name="arm_post",
    )
    arm.visual(
        Box((0.11, 0.030, 0.035)),
        origin=Origin(xyz=(0.0, side_sign * 0.053, 0.185)),
        material=arm_frame,
        name="arm_upper_link",
    )
    arm.visual(
        arm_pad,
        origin=Origin(xyz=(0.0, side_sign * 0.055, 0.215)),
        material=arm_frame,
        name="arm_pad",
    )
    arm.inertial = Inertial.from_geometry(
        Box((0.30, 0.12, 0.25)),
        mass=1.4,
        origin=Origin(xyz=(0.0, side_sign * 0.05, 0.12)),
    )
    return arm


def _build_caster_parts(model: ArticulatedObject, index: int, caster_stem, wheel_rubber, fork_frame):
    fork = model.part(f"caster_{index}_fork")
    fork.visual(
        Cylinder(radius=0.008, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, -0.011)),
        material=caster_stem,
        name="fork_stem",
    )
    fork.visual(
        Box((0.024, 0.048, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, -0.028)),
        material=fork_frame,
        name="fork_crown",
    )
    for side_sign in (-1.0, 1.0):
        fork.visual(
            Box((0.014, 0.005, 0.042)),
            origin=Origin(xyz=(0.0, side_sign * 0.0145, -0.051)),
            material=fork_frame,
            name=f"fork_leg_{'r' if side_sign < 0.0 else 'l'}",
        )
    fork.inertial = Inertial.from_geometry(
        Box((0.05, 0.06, 0.09)),
        mass=0.18,
        origin=Origin(xyz=(0.0, 0.0, -0.04)),
    )

    wheel = model.part(f"caster_{index}_wheel")
    wheel.visual(
        Cylinder(radius=0.028, length=0.024),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=wheel_rubber,
        name="wheel_tire",
    )
    wheel.visual(
        Cylinder(radius=0.017, length=0.020),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=fork_frame,
        name="wheel_hub",
    )
    wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.028, length=0.024),
        mass=0.22,
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
    )
    return fork, wheel


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mesh_office_chair")

    base_frame = model.material("base_frame", rgba=(0.16, 0.17, 0.18, 1.0))
    column_metal = model.material("column_metal", rgba=(0.60, 0.63, 0.67, 1.0))
    seat_frame = model.material("seat_frame", rgba=(0.10, 0.11, 0.12, 1.0))
    mesh_fabric = model.material("mesh_fabric", rgba=(0.22, 0.24, 0.26, 0.95))
    arm_frame = model.material("arm_frame", rgba=(0.14, 0.15, 0.16, 1.0))
    fork_frame = model.material("fork_frame", rgba=(0.12, 0.13, 0.14, 1.0))
    caster_stem = model.material("caster_stem", rgba=(0.56, 0.58, 0.61, 1.0))
    wheel_rubber = model.material("wheel_rubber", rgba=(0.05, 0.05, 0.05, 1.0))

    seat_ring_mesh = _horizontal_frame_mesh(
        length=0.49,
        width=0.54,
        wall=0.030,
        thickness=0.024,
        radius=0.060,
        name="seat_ring",
    )
    seat_panel_mesh = _horizontal_panel_mesh(
        length=0.442,
        width=0.482,
        thickness=0.006,
        radius=0.045,
        name="seat_panel",
    )
    back_half_frame_mesh = _vertical_frame_mesh(
        width=0.17,
        height=0.54,
        wall=0.022,
        thickness=0.014,
        radius=0.055,
        name="split_back_half_frame",
    )
    back_half_panel_mesh = _vertical_panel_mesh(
        width=0.134,
        height=0.50,
        thickness=0.004,
        radius=0.040,
        name="split_back_half_panel",
    )
    arm_pad_mesh = _horizontal_panel_mesh(
        length=0.28,
        width=0.085,
        thickness=0.028,
        radius=0.020,
        name="arm_pad_mesh",
    )

    lower_base = model.part("lower_base")
    lower_base.visual(
        Cylinder(radius=0.07, length=0.05),
        origin=Origin(xyz=(0.0, 0.0, 0.08)),
        material=base_frame,
        name="hub_shell",
    )
    lower_base.visual(
        Cylinder(radius=0.05, length=0.14),
        origin=Origin(xyz=(0.0, 0.0, 0.15)),
        material=column_metal,
        name="column_shroud",
    )
    lower_base.visual(
        Cylinder(radius=0.038, length=0.30),
        origin=Origin(xyz=(0.0, 0.0, 0.235)),
        material=column_metal,
        name="column_shell",
    )
    for spoke_index in range(5):
        angle = spoke_index * (2.0 * pi / 5.0)
        lower_base.visual(
            Box((0.26, 0.055, 0.025)),
            origin=Origin(
                xyz=(cos(angle) * 0.17, sin(angle) * 0.17, 0.062),
                rpy=(0.0, 0.0, angle),
            ),
            material=base_frame,
            name=f"spoke_{spoke_index}",
        )
    lower_base.inertial = Inertial.from_geometry(
        Box((0.78, 0.78, 0.42)),
        mass=12.0,
        origin=Origin(xyz=(0.0, 0.0, 0.18)),
    )

    seat = model.part("seat")
    seat.visual(
        Cylinder(radius=0.03, length=0.04),
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
        material=column_metal,
        name="seat_socket",
    )
    seat.visual(
        Box((0.20, 0.24, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, 0.066)),
        material=base_frame,
        name="undercarriage",
    )
    seat.visual(
        Box((0.45, 0.49, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.099)),
        material=seat_frame,
        name="seat_support_pan",
    )
    seat.visual(
        seat_ring_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.108)),
        material=seat_frame,
        name="seat_frame",
    )
    seat.visual(
        seat_panel_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.120)),
        material=mesh_fabric,
        name="seat_mesh",
    )
    seat.visual(
        Box((0.05, 0.24, 0.06)),
        origin=Origin(xyz=(-0.235, 0.0, 0.126)),
        material=base_frame,
        name="back_hinge_beam",
    )
    seat.visual(
        Box((0.08, 0.03, 0.06)),
        origin=Origin(xyz=(-0.02, 0.275, 0.126)),
        material=arm_frame,
        name="left_arm_mount",
    )
    seat.visual(
        Box((0.08, 0.03, 0.06)),
        origin=Origin(xyz=(-0.02, -0.275, 0.126)),
        material=arm_frame,
        name="right_arm_mount",
    )
    seat.inertial = Inertial.from_geometry(
        Box((0.56, 0.60, 0.24)),
        mass=7.2,
        origin=Origin(xyz=(0.0, 0.0, 0.11)),
    )

    backrest = model.part("backrest")
    backrest.visual(
        Box((0.06, 0.24, 0.07)),
        origin=Origin(xyz=(-0.03, 0.0, 0.035)),
        material=base_frame,
        name="back_mount",
    )
    backrest.visual(
        back_half_frame_mesh,
        origin=Origin(xyz=(-0.06, 0.112, 0.29)),
        material=seat_frame,
        name="left_back_frame",
    )
    backrest.visual(
        back_half_frame_mesh,
        origin=Origin(xyz=(-0.06, -0.112, 0.29)),
        material=seat_frame,
        name="right_back_frame",
    )
    backrest.visual(
        back_half_panel_mesh,
        origin=Origin(xyz=(-0.058, 0.112, 0.29)),
        material=mesh_fabric,
        name="left_back_mesh",
    )
    backrest.visual(
        back_half_panel_mesh,
        origin=Origin(xyz=(-0.058, -0.112, 0.29)),
        material=mesh_fabric,
        name="right_back_mesh",
    )
    backrest.inertial = Inertial.from_geometry(
        Box((0.24, 0.50, 0.62)),
        mass=4.2,
        origin=Origin(xyz=(-0.06, 0.0, 0.28)),
    )

    left_armrest = _build_armrest_part(model, "left_armrest", 1.0, arm_frame, arm_pad_mesh)
    right_armrest = _build_armrest_part(model, "right_armrest", -1.0, arm_frame, arm_pad_mesh)

    for caster_index in range(5):
        _build_caster_parts(model, caster_index, caster_stem, wheel_rubber, fork_frame)

    model.articulation(
        "base_to_seat_swivel",
        ArticulationType.CONTINUOUS,
        parent=lower_base,
        child=seat,
        origin=Origin(xyz=(0.0, 0.0, 0.385)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=4.5),
    )
    model.articulation(
        "seat_to_backrest_recline",
        ArticulationType.REVOLUTE,
        parent=seat,
        child=backrest,
        origin=Origin(xyz=(-0.26, 0.0, 0.156)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=14.0,
            velocity=1.4,
            lower=0.0,
            upper=0.55,
        ),
    )
    model.articulation(
        "seat_to_left_armrest",
        ArticulationType.REVOLUTE,
        parent=seat,
        child=left_armrest,
        origin=Origin(xyz=(-0.02, 0.29, 0.126)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.2,
            lower=0.0,
            upper=1.45,
        ),
    )
    model.articulation(
        "seat_to_right_armrest",
        ArticulationType.REVOLUTE,
        parent=seat,
        child=right_armrest,
        origin=Origin(xyz=(-0.02, -0.29, 0.126)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.2,
            lower=0.0,
            upper=1.45,
        ),
    )

    for caster_index in range(5):
        angle = caster_index * (2.0 * pi / 5.0)
        tip = (cos(angle) * 0.29, sin(angle) * 0.29, 0.05)
        model.articulation(
            f"base_to_caster_{caster_index}_swivel",
            ArticulationType.CONTINUOUS,
            parent=lower_base,
            child=f"caster_{caster_index}_fork",
            origin=Origin(xyz=tip),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=2.0, velocity=12.0),
        )
        model.articulation(
            f"caster_{caster_index}_fork_to_wheel",
            ArticulationType.CONTINUOUS,
            parent=f"caster_{caster_index}_fork",
            child=f"caster_{caster_index}_wheel",
            origin=Origin(xyz=(0.0, 0.0, -0.072)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=1.5, velocity=25.0),
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
    lower_base = object_model.get_part("lower_base")
    seat = object_model.get_part("seat")
    backrest = object_model.get_part("backrest")
    left_armrest = object_model.get_part("left_armrest")
    right_armrest = object_model.get_part("right_armrest")
    front_fork = object_model.get_part("caster_0_fork")
    front_wheel = object_model.get_part("caster_0_wheel")

    swivel = object_model.get_articulation("base_to_seat_swivel")
    recline = object_model.get_articulation("seat_to_backrest_recline")
    left_flip = object_model.get_articulation("seat_to_left_armrest")
    right_flip = object_model.get_articulation("seat_to_right_armrest")
    front_caster_swivel = object_model.get_articulation("base_to_caster_0_swivel")
    front_wheel_spin = object_model.get_articulation("caster_0_fork_to_wheel")

    ctx.check(
        "primary joint axes match chair mechanisms",
        swivel.articulation_type == ArticulationType.CONTINUOUS
        and swivel.axis == (0.0, 0.0, 1.0)
        and recline.articulation_type == ArticulationType.REVOLUTE
        and recline.axis == (0.0, -1.0, 0.0)
        and left_flip.axis == (1.0, 0.0, 0.0)
        and right_flip.axis == (-1.0, 0.0, 0.0),
        details=(
            f"swivel={swivel.axis}, recline={recline.axis}, "
            f"left_flip={left_flip.axis}, right_flip={right_flip.axis}"
        ),
    )
    ctx.check(
        "front caster joints are continuous swivel plus wheel spin",
        front_caster_swivel.articulation_type == ArticulationType.CONTINUOUS
        and front_caster_swivel.axis == (0.0, 0.0, 1.0)
        and front_wheel_spin.articulation_type == ArticulationType.CONTINUOUS
        and front_wheel_spin.axis == (0.0, 1.0, 0.0),
        details=(
            f"caster_swivel={front_caster_swivel.axis}, "
            f"wheel_spin={front_wheel_spin.axis}"
        ),
    )

    ctx.expect_contact(
        seat,
        lower_base,
        elem_a="seat_socket",
        elem_b="column_shell",
        contact_tol=0.0015,
        name="seat socket rests on the swivel column",
    )
    ctx.expect_contact(
        backrest,
        seat,
        elem_a="back_mount",
        elem_b="back_hinge_beam",
        contact_tol=0.0015,
        name="backrest mounts against the rear hinge beam",
    )
    ctx.expect_contact(
        left_armrest,
        seat,
        elem_a="arm_barrel",
        elem_b="left_arm_mount",
        contact_tol=0.0015,
        name="left armrest barrel sits on its side mount",
    )
    ctx.expect_contact(
        right_armrest,
        seat,
        elem_a="arm_barrel",
        elem_b="right_arm_mount",
        contact_tol=0.0015,
        name="right armrest barrel sits on its side mount",
    )
    ctx.expect_contact(
        front_fork,
        lower_base,
        elem_a="fork_stem",
        contact_tol=0.0015,
        name="front caster fork hangs from the star base",
    )
    ctx.expect_contact(
        front_wheel,
        front_fork,
        elem_a="wheel_tire",
        contact_tol=0.0015,
        name="front caster wheel is captured inside the fork",
    )

    left_rest = ctx.part_world_aabb(left_armrest)
    right_rest = ctx.part_world_aabb(right_armrest)
    back_rest = ctx.part_world_aabb(backrest)
    left_pivot_rest = ctx.part_world_position(left_armrest)
    with ctx.pose({swivel: 0.85}):
        left_pivot_swiveled = ctx.part_world_position(left_armrest)
    with ctx.pose({recline: 0.45}):
        back_reclined = ctx.part_world_aabb(backrest)
    with ctx.pose({left_flip: 1.30, right_flip: 1.30}):
        left_up = ctx.part_world_aabb(left_armrest)
        right_up = ctx.part_world_aabb(right_armrest)

    swivel_ok = False
    if left_pivot_rest is not None and left_pivot_swiveled is not None:
        r0 = hypot(left_pivot_rest[0], left_pivot_rest[1])
        r1 = hypot(left_pivot_swiveled[0], left_pivot_swiveled[1])
        angle0 = atan2(left_pivot_rest[1], left_pivot_rest[0])
        angle1 = atan2(left_pivot_swiveled[1], left_pivot_swiveled[0])
        swivel_ok = abs(r1 - r0) < 0.02 and abs(angle1 - angle0) > 0.35
    ctx.check(
        "seat assembly swivels around the center column",
        swivel_ok,
        details=f"rest={left_pivot_rest}, swiveled={left_pivot_swiveled}",
    )

    recline_ok = (
        back_rest is not None
        and back_reclined is not None
        and back_reclined[0][0] < back_rest[0][0] - 0.07
    )
    ctx.check(
        "backrest reclines backward",
        recline_ok,
        details=f"rest_aabb={back_rest}, reclined_aabb={back_reclined}",
    )

    armrests_raise = False
    if left_rest is not None and right_rest is not None and left_up is not None and right_up is not None:
        left_rest_center_y = 0.5 * (left_rest[0][1] + left_rest[1][1])
        right_rest_center_y = 0.5 * (right_rest[0][1] + right_rest[1][1])
        left_up_center_y = 0.5 * (left_up[0][1] + left_up[1][1])
        right_up_center_y = 0.5 * (right_up[0][1] + right_up[1][1])
        left_rest_height = left_rest[1][2] - left_rest[0][2]
        right_rest_height = right_rest[1][2] - right_rest[0][2]
        left_up_height = left_up[1][2] - left_up[0][2]
        right_up_height = right_up[1][2] - right_up[0][2]
        armrests_raise = (
            abs(left_up_center_y) < abs(left_rest_center_y) - 0.10
            and abs(right_up_center_y) < abs(right_rest_center_y) - 0.10
            and left_up_height < left_rest_height - 0.05
            and right_up_height < right_rest_height - 0.05
        )
    ctx.check(
        "armrests flip up into a stowed posture",
        armrests_raise,
        details=(
            f"left_rest={left_rest}, left_up={left_up}, "
            f"right_rest={right_rest}, right_up={right_up}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
