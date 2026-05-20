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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    sweep_profile_along_spline,
    tube_from_spline_points,
)


def _distance(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _rpy_for_cylinder(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_member(part, a, b, *, radius: float, material, name: str | None = None) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _rounded_pad_mesh(name: str, width: float, depth: float, thickness: float, radius: float):
    return mesh_from_geometry(
        ExtrudeGeometry(rounded_rect_profile(width, depth, radius), thickness),
        name,
    )


def _aabb_center(aabb) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    mins, maxs = aabb
    return (
        (mins[0] + maxs[0]) * 0.5,
        (mins[1] + maxs[1]) * 0.5,
        (mins[2] + maxs[2]) * 0.5,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="ergonomic_office_chair")

    base_plastic = model.material("base_plastic", rgba=(0.12, 0.12, 0.13, 1.0))
    frame_black = model.material("frame_black", rgba=(0.10, 0.10, 0.11, 1.0))
    mesh_graphite = model.material("mesh_graphite", rgba=(0.20, 0.22, 0.24, 1.0))
    cushion_fabric = model.material("cushion_fabric", rgba=(0.16, 0.17, 0.18, 1.0))
    hardware_dark = model.material("hardware_dark", rgba=(0.22, 0.24, 0.27, 1.0))
    metal_gray = model.material("metal_gray", rgba=(0.58, 0.60, 0.63, 1.0))
    wheel_rubber = model.material("wheel_rubber", rgba=(0.05, 0.05, 0.06, 1.0))
    arm_pad = model.material("arm_pad", rgba=(0.14, 0.14, 0.15, 1.0))

    seat_cushion_mesh = _rounded_pad_mesh("seat_cushion", 0.52, 0.49, 0.075, 0.06)
    arm_pad_mesh = _rounded_pad_mesh("chair_arm_pad", 0.24, 0.09, 0.032, 0.02)
    headrest_pad_mesh = _rounded_pad_mesh("chair_headrest_pad", 0.270, 0.145, 0.058, 0.032)

    base_arm_mesh = mesh_from_geometry(
        sweep_profile_along_spline(
            [
                (0.055, 0.0, 0.078),
                (0.18, 0.0, 0.076),
                (0.31, 0.0, 0.070),
            ],
            profile=rounded_rect_profile(0.068, 0.024, 0.010),
            samples_per_segment=14,
            cap_profile=True,
        ),
        "chair_star_base_arm",
    )

    left_back_rail_mesh = mesh_from_geometry(
        tube_from_spline_points(
            [
                (0.008, 0.182, 0.030),
                (0.002, 0.192, 0.180),
                (-0.018, 0.182, 0.430),
                (-0.052, 0.145, 0.720),
            ],
            radius=0.015,
            samples_per_segment=16,
            radial_segments=18,
        ),
        "chair_back_left_rail",
    )
    right_back_rail_mesh = mesh_from_geometry(
        tube_from_spline_points(
            [
                (0.008, -0.182, 0.030),
                (0.002, -0.192, 0.180),
                (-0.018, -0.182, 0.430),
                (-0.052, -0.145, 0.720),
            ],
            radius=0.015,
            samples_per_segment=16,
            radial_segments=18,
        ),
        "chair_back_right_rail",
    )
    back_top_bar_mesh = mesh_from_geometry(
        tube_from_spline_points(
            [
                (-0.052, 0.145, 0.720),
                (-0.062, 0.000, 0.760),
                (-0.052, -0.145, 0.720),
            ],
            radius=0.015,
            samples_per_segment=14,
            radial_segments=18,
        ),
        "chair_back_top_bar",
    )

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.074, length=0.052),
        origin=Origin(xyz=(0.0, 0.0, 0.073)),
        material=base_plastic,
        name="base_hub",
    )
    base.visual(
        Cylinder(radius=0.050, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.097)),
        material=hardware_dark,
        name="column_collar",
    )
    base.visual(
        Cylinder(radius=0.032, length=0.308),
        origin=Origin(xyz=(0.0, 0.0, 0.266)),
        material=metal_gray,
        name="central_column",
    )
    base.visual(
        Cylinder(radius=0.060, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.420)),
        material=hardware_dark,
        name="column_cap",
    )
    for index in range(5):
        angle = (2.0 * math.pi * index) / 5.0
        x_tip = 0.31 * math.cos(angle)
        y_tip = 0.31 * math.sin(angle)
        base.visual(
            base_arm_mesh,
            origin=Origin(rpy=(0.0, 0.0, angle)),
            material=base_plastic,
            name=f"star_arm_{index}",
        )
        base.visual(
            Cylinder(radius=0.013, length=0.012),
            origin=Origin(xyz=(x_tip, y_tip, 0.064)),
            material=hardware_dark,
            name=f"caster_socket_{index}",
        )
    base.inertial = Inertial.from_geometry(
        Box((0.72, 0.72, 0.44)),
        mass=10.0,
        origin=Origin(xyz=(0.0, 0.0, 0.22)),
    )

    seat = model.part("seat")
    seat.visual(
        seat_cushion_mesh,
        origin=Origin(xyz=(0.075, 0.0, 0.058)),
        material=cushion_fabric,
        name="seat_cushion",
    )
    seat.visual(
        Box((0.28, 0.24, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
        material=frame_black,
        name="seat_pan",
    )
    seat.visual(
        Box((0.22, 0.18, 0.032)),
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
        material=hardware_dark,
        name="seat_mount_plate",
    )
    seat.visual(
        Box((0.12, 0.20, 0.070)),
        origin=Origin(xyz=(-0.145, 0.0, 0.055)),
        material=hardware_dark,
        name="tilt_mechanism_housing",
    )
    seat.visual(
        Cylinder(radius=0.010, length=0.315),
        origin=Origin(xyz=(-0.229, 0.0, 0.085), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=metal_gray,
        name="seat_hinge_bar",
    )
    seat.visual(
        Box((0.060, 0.030, 0.044)),
        origin=Origin(xyz=(-0.218, 0.086, 0.084)),
        material=hardware_dark,
        name="seat_hinge_support_left",
    )
    seat.visual(
        Box((0.060, 0.030, 0.044)),
        origin=Origin(xyz=(-0.218, -0.086, 0.084)),
        material=hardware_dark,
        name="seat_hinge_support_right",
    )
    for side, sign in (("left", 1.0), ("right", -1.0)):
        seat.visual(
            Box((0.028, 0.034, 0.165)),
            origin=Origin(xyz=(-0.020, sign * 0.245, 0.138)),
            material=frame_black,
            name=f"armrest_post_{side}",
        )
        seat.visual(
            Box((0.120, 0.028, 0.040)),
            origin=Origin(xyz=(0.012, sign * 0.245, 0.235)),
            material=frame_black,
            name=f"armrest_support_{side}",
        )
        seat.visual(
            arm_pad_mesh,
            origin=Origin(xyz=(0.045, sign * 0.245, 0.270)),
            material=arm_pad,
            name=f"armrest_pad_{side}",
        )
    seat.inertial = Inertial.from_geometry(
        Box((0.56, 0.54, 0.32)),
        mass=5.5,
        origin=Origin(xyz=(0.0, 0.0, 0.14)),
    )

    back = model.part("back")
    back.visual(left_back_rail_mesh, material=frame_black, name="back_left_rail")
    back.visual(right_back_rail_mesh, material=frame_black, name="back_right_rail")
    back.visual(back_top_bar_mesh, material=frame_black, name="back_top_bar")
    back.visual(
        Box((0.020, 0.360, 0.380)),
        origin=Origin(xyz=(-0.030, 0.0, 0.270)),
        material=mesh_graphite,
        name="back_mesh_panel",
    )
    back.visual(
        Box((0.030, 0.300, 0.080)),
        origin=Origin(xyz=(-0.018, 0.0, 0.150)),
        material=hardware_dark,
        name="lumbar_bridge",
    )
    back.visual(
        Box((0.028, 0.092, 0.300)),
        origin=Origin(xyz=(-0.054, 0.0, 0.595)),
        material=hardware_dark,
        name="headrest_guide_mast",
    )
    back.visual(
        Box((0.024, 0.048, 0.070)),
        origin=Origin(xyz=(0.018, 0.142, 0.039)),
        material=hardware_dark,
        name="back_hinge_lug_left",
    )
    back.visual(
        Box((0.024, 0.048, 0.070)),
        origin=Origin(xyz=(0.018, -0.142, 0.039)),
        material=hardware_dark,
        name="back_hinge_lug_right",
    )
    back.visual(
        Box((0.028, 0.060, 0.120)),
        origin=Origin(xyz=(-0.002, 0.163, 0.095)),
        material=hardware_dark,
        name="back_lower_bracket_left",
    )
    back.visual(
        Box((0.028, 0.060, 0.120)),
        origin=Origin(xyz=(-0.002, -0.163, 0.095)),
        material=hardware_dark,
        name="back_lower_bracket_right",
    )
    back.inertial = Inertial.from_geometry(
        Box((0.16, 0.42, 0.78)),
        mass=4.0,
        origin=Origin(xyz=(-0.020, 0.0, 0.390)),
    )

    headrest = model.part("headrest")
    headrest.visual(
        Box((0.036, 0.100, 0.090)),
        origin=Origin(xyz=(0.018, 0.0, 0.045)),
        material=hardware_dark,
        name="headrest_carriage",
    )
    headrest.visual(
        Box((0.020, 0.042, 0.100)),
        origin=Origin(xyz=(0.034, 0.056, 0.120)),
        material=frame_black,
        name="headrest_strut_left",
    )
    headrest.visual(
        Box((0.020, 0.042, 0.100)),
        origin=Origin(xyz=(0.034, -0.056, 0.120)),
        material=frame_black,
        name="headrest_strut_right",
    )
    headrest.visual(
        headrest_pad_mesh,
        origin=Origin(
            xyz=(0.050, 0.0, 0.195), rpy=(math.pi / 2.0, 0.0, math.pi / 2.0)
        ),
        material=cushion_fabric,
        name="headrest_pad",
    )
    headrest.inertial = Inertial.from_geometry(
        Box((0.08, 0.28, 0.25)),
        mass=1.2,
        origin=Origin(xyz=(0.045, 0.0, 0.160)),
    )

    seat_swivel = model.articulation(
        "base_to_seat_swivel",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=seat,
        origin=Origin(xyz=(0.0, 0.0, 0.430)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=40.0, velocity=1.8),
    )
    back_recline = model.articulation(
        "seat_to_back_recline",
        ArticulationType.REVOLUTE,
        parent=seat,
        child=back,
        origin=Origin(xyz=(-0.225, 0.0, 0.080)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=1.2,
            lower=0.0,
            upper=math.radians(32.0),
        ),
    )
    headrest_slide = model.articulation(
        "back_to_headrest_slide",
        ArticulationType.PRISMATIC,
        parent=back,
        child=headrest,
        origin=Origin(xyz=(-0.040, 0.0, 0.520)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=0.10,
            lower=0.0,
            upper=0.100,
        ),
    )

    for index in range(5):
        angle = (2.0 * math.pi * index) / 5.0
        caster_yoke = model.part(f"caster_yoke_{index}")
        caster_yoke.visual(
            Cylinder(radius=0.009, length=0.018),
            origin=Origin(xyz=(0.0, 0.0, -0.009)),
            material=metal_gray,
            name="caster_stem",
        )
        caster_yoke.visual(
            Box((0.020, 0.028, 0.018)),
            origin=Origin(xyz=(0.0, 0.0, -0.024)),
            material=hardware_dark,
            name="caster_swivel_knuckle",
        )
        caster_yoke.visual(
            Box((0.036, 0.042, 0.010)),
            origin=Origin(xyz=(0.0, 0.0, -0.038)),
            material=hardware_dark,
            name="caster_fork_bridge",
        )
        caster_yoke.visual(
            Box((0.010, 0.008, 0.052)),
            origin=Origin(xyz=(0.0, 0.017, -0.059)),
            material=hardware_dark,
            name="caster_fork_left",
        )
        caster_yoke.visual(
            Box((0.010, 0.008, 0.052)),
            origin=Origin(xyz=(0.0, -0.017, -0.059)),
            material=hardware_dark,
            name="caster_fork_right",
        )

        caster_wheel = model.part(f"caster_wheel_{index}")
        caster_wheel.visual(
            Cylinder(radius=0.026, length=0.018),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=wheel_rubber,
            name="wheel_tire",
        )
        caster_wheel.visual(
            Cylinder(radius=0.014, length=0.022),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=hardware_dark,
            name="wheel_hub",
        )
        caster_wheel.inertial = Inertial.from_geometry(
            Cylinder(radius=0.026, length=0.018),
            mass=0.22,
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        )

        model.articulation(
            f"base_to_caster_swivel_{index}",
            ArticulationType.CONTINUOUS,
            parent=base,
            child=caster_yoke,
            origin=Origin(
                xyz=(0.310 * math.cos(angle), 0.310 * math.sin(angle), 0.058)
            ),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=4.0, velocity=8.0),
        )
        model.articulation(
            f"caster_to_wheel_spin_{index}",
            ArticulationType.CONTINUOUS,
            parent=caster_yoke,
            child=caster_wheel,
            origin=Origin(xyz=(0.0, 0.0, -0.064)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=3.0, velocity=25.0),
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
    base = object_model.get_part("base")
    seat = object_model.get_part("seat")
    back = object_model.get_part("back")
    headrest = object_model.get_part("headrest")
    seat_swivel = object_model.get_articulation("base_to_seat_swivel")
    back_recline = object_model.get_articulation("seat_to_back_recline")
    headrest_slide = object_model.get_articulation("back_to_headrest_slide")

    ctx.check(
        "primary joints use intended articulation types",
        seat_swivel.articulation_type == ArticulationType.CONTINUOUS
        and back_recline.articulation_type == ArticulationType.REVOLUTE
        and headrest_slide.articulation_type == ArticulationType.PRISMATIC,
        details=(
            f"seat={seat_swivel.articulation_type}, "
            f"back={back_recline.articulation_type}, "
            f"headrest={headrest_slide.articulation_type}"
        ),
    )
    ctx.check(
        "primary joint axes match mechanism directions",
        seat_swivel.axis == (0.0, 0.0, 1.0)
        and back_recline.axis == (0.0, -1.0, 0.0)
        and headrest_slide.axis == (0.0, 0.0, 1.0),
        details=(
            f"seat_axis={seat_swivel.axis}, "
            f"back_axis={back_recline.axis}, "
            f"headrest_axis={headrest_slide.axis}"
        ),
    )

    ctx.expect_contact(
        seat,
        base,
        elem_a="seat_mount_plate",
        elem_b="column_cap",
        contact_tol=0.001,
        name="seat turntable sits on column cap",
    )
    ctx.expect_contact(
        back,
        seat,
        elem_a="back_hinge_lug_left",
        elem_b="seat_hinge_bar",
        contact_tol=0.0015,
        name="back hinge lugs meet seat hinge bar",
    )
    ctx.expect_contact(
        headrest,
        back,
        elem_a="headrest_carriage",
        elem_b="headrest_guide_mast",
        contact_tol=0.001,
        name="headrest carriage rides on guide mast",
    )
    ctx.expect_overlap(
        seat,
        base,
        axes="xy",
        elem_a="seat_mount_plate",
        elem_b="column_cap",
        min_overlap=0.10,
        name="seat remains centered over the column",
    )

    rest_seat_pos = ctx.part_world_position(seat)
    with ctx.pose({seat_swivel: math.pi / 3.0}):
        swivel_seat_pos = ctx.part_world_position(seat)
    ctx.check(
        "seat swivel keeps the seat centered on the column",
        rest_seat_pos is not None
        and swivel_seat_pos is not None
        and abs(swivel_seat_pos[0] - rest_seat_pos[0]) < 1e-6
        and abs(swivel_seat_pos[1] - rest_seat_pos[1]) < 1e-6
        and abs(swivel_seat_pos[2] - rest_seat_pos[2]) < 1e-6,
        details=f"rest={rest_seat_pos}, swiveled={swivel_seat_pos}",
    )

    back_top_rest = _aabb_center(ctx.part_element_world_aabb(back, elem="back_top_bar"))
    with ctx.pose({back_recline: math.radians(28.0)}):
        back_top_reclined = _aabb_center(ctx.part_element_world_aabb(back, elem="back_top_bar"))
    ctx.check(
        "back recline moves the upper back rearward",
        back_top_rest is not None
        and back_top_reclined is not None
        and back_top_reclined[0] < back_top_rest[0] - 0.06,
        details=f"rest={back_top_rest}, reclined={back_top_reclined}",
    )

    headrest_pad_rest = _aabb_center(ctx.part_element_world_aabb(headrest, elem="headrest_pad"))
    with ctx.pose({headrest_slide: 0.10}):
        headrest_pad_high = _aabb_center(ctx.part_element_world_aabb(headrest, elem="headrest_pad"))
        ctx.expect_contact(
            headrest,
            back,
            elem_a="headrest_carriage",
            elem_b="headrest_guide_mast",
            contact_tol=0.001,
            name="headrest carriage stays guided at full extension",
        )
    ctx.check(
        "headrest support slides upward",
        headrest_pad_rest is not None
        and headrest_pad_high is not None
        and headrest_pad_high[2] > headrest_pad_rest[2] + 0.08,
        details=f"rest={headrest_pad_rest}, high={headrest_pad_high}",
    )

    for index in range(5):
        caster_swivel = object_model.get_articulation(f"base_to_caster_swivel_{index}")
        wheel_spin = object_model.get_articulation(f"caster_to_wheel_spin_{index}")
        ctx.check(
            f"caster {index} articulations are continuous and aligned",
            caster_swivel.articulation_type == ArticulationType.CONTINUOUS
            and caster_swivel.axis == (0.0, 0.0, 1.0)
            and wheel_spin.articulation_type == ArticulationType.CONTINUOUS
            and wheel_spin.axis == (0.0, 1.0, 0.0),
            details=(
                f"swivel=({caster_swivel.articulation_type}, {caster_swivel.axis}), "
                f"wheel=({wheel_spin.articulation_type}, {wheel_spin.axis})"
            ),
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
