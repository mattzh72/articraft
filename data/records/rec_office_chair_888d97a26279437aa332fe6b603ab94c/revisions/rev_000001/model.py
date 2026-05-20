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
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _rounded_plate_mesh(
    name: str,
    *,
    size_x: float,
    size_y: float,
    thickness: float,
    corner_radius: float,
):
    return mesh_from_geometry(
        ExtrudeGeometry(
            rounded_rect_profile(size_x, size_y, corner_radius),
            thickness,
            center=True,
            cap=True,
            closed=True,
        ),
        name,
    )


def _shell_mesh(
    name: str,
    *,
    outer_radius: float,
    inner_radius: float,
    height: float,
):
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [(outer_radius, 0.0), (outer_radius, height)],
            [(inner_radius, 0.0), (inner_radius, height)],
            segments=56,
        ),
        name,
    )


def _spoke_tip(radius: float, angle: float, z: float) -> tuple[float, float, float]:
    return (radius * math.cos(angle), radius * math.sin(angle), z)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="five_star_task_chair")

    frame_black = model.material("frame_black", rgba=(0.10, 0.10, 0.11, 1.0))
    dark_grey = model.material("dark_grey", rgba=(0.18, 0.19, 0.21, 1.0))
    aluminum = model.material("aluminum", rgba=(0.60, 0.62, 0.66, 1.0))
    chrome = model.material("chrome", rgba=(0.82, 0.84, 0.88, 1.0))
    seat_fabric = model.material("seat_fabric", rgba=(0.17, 0.20, 0.26, 1.0))
    back_fabric = model.material("back_fabric", rgba=(0.15, 0.19, 0.24, 1.0))
    wheel_rubber = model.material("wheel_rubber", rgba=(0.05, 0.05, 0.05, 1.0))

    seat_cushion_mesh = _rounded_plate_mesh(
        "task_chair_seat_cushion_v2",
        size_x=0.48,
        size_y=0.50,
        thickness=0.055,
        corner_radius=0.050,
    )
    back_panel_mesh = _rounded_plate_mesh(
        "task_chair_back_panel_v3",
        size_x=0.38,
        size_y=0.42,
        thickness=0.055,
        corner_radius=0.075,
    )
    lumbar_pad_mesh = _rounded_plate_mesh(
        "task_chair_lumbar_pad_v2",
        size_x=0.11,
        size_y=0.28,
        thickness=0.035,
        corner_radius=0.035,
    )

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.055, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.085)),
        material=frame_black,
        name="hub_lower",
    )
    base.visual(
        Cylinder(radius=0.045, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.110)),
        material=dark_grey,
        name="hub_upper",
    )

    spoke_center_radius = 0.200
    spoke_tip_radius = 0.350
    spoke_z = 0.073
    for index in range(5):
        angle = 2.0 * math.pi * index / 5.0
        base.visual(
            Box((0.300, 0.045, 0.016)),
            origin=Origin(
                xyz=(
                    spoke_center_radius * math.cos(angle),
                    spoke_center_radius * math.sin(angle),
                    spoke_z,
                ),
                rpy=(0.0, 0.0, angle),
            ),
            material=aluminum,
            name=f"spoke_{index}",
        )

    column_outer = model.part("column_outer")
    column_outer.visual(
        Cylinder(radius=0.040, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.145)),
        material=aluminum,
        name="lower_cover",
    )
    column_outer.visual(
        Cylinder(radius=0.032, length=0.140),
        origin=Origin(xyz=(0.0, 0.0, 0.240)),
        material=aluminum,
        name="outer_tube",
    )
    column_outer.visual(
        Cylinder(radius=0.036, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.316)),
        material=dark_grey,
        name="top_collar",
    )
    model.articulation(
        "base_to_column_outer",
        ArticulationType.FIXED,
        parent=base,
        child=column_outer,
        origin=Origin(),
    )

    column_inner = model.part("column_inner")
    column_inner.visual(
        Cylinder(radius=0.022, length=0.175),
        origin=Origin(xyz=(0.0, 0.0, 0.0875)),
        material=chrome,
        name="piston",
    )
    column_inner.visual(
        Cylinder(radius=0.030, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.185)),
        material=frame_black,
        name="top_head",
    )
    model.articulation(
        "column_height",
        ArticulationType.PRISMATIC,
        parent=column_outer,
        child=column_inner,
        origin=Origin(xyz=(0.0, 0.0, 0.170)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=220.0,
            velocity=0.25,
            lower=0.0,
            upper=0.08,
        ),
    )

    seat = model.part("seat")
    seat.visual(
        Box((0.080, 0.080, 0.060)),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=frame_black,
        name="support_pillar",
    )
    seat.visual(
        Box((0.240, 0.050, 0.020)),
        origin=Origin(xyz=(-0.030, -0.055, 0.070)),
        material=frame_black,
        name="rail_left",
    )
    seat.visual(
        Box((0.240, 0.050, 0.020)),
        origin=Origin(xyz=(-0.030, 0.055, 0.070)),
        material=frame_black,
        name="rail_right",
    )
    seat.visual(
        Box((0.180, 0.120, 0.020)),
        origin=Origin(xyz=(-0.180, 0.0, 0.070)),
        material=frame_black,
        name="rear_plate",
    )
    seat.visual(
        Box((0.020, 0.100, 0.140)),
        origin=Origin(xyz=(-0.245, 0.0, 0.070)),
        material=frame_black,
        name="back_mount",
    )
    seat.visual(
        seat_cushion_mesh,
        origin=Origin(xyz=(0.025, 0.0, 0.1075)),
        material=seat_fabric,
        name="seat_cushion",
    )
    model.articulation(
        "seat_swivel",
        ArticulationType.CONTINUOUS,
        parent=column_inner,
        child=seat,
        origin=Origin(xyz=(0.0, 0.0, 0.195)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=40.0, velocity=4.0),
    )

    backrest = model.part("backrest")
    backrest.visual(
        Box((0.010, 0.100, 0.020)),
        origin=Origin(xyz=(-0.015, 0.0, 0.010)),
        material=frame_black,
        name="lower_crossbar",
    )
    backrest.visual(
        Box((0.020, 0.030, 0.200)),
        origin=Origin(xyz=(-0.022, -0.045, 0.120)),
        material=frame_black,
        name="upright_left",
    )
    backrest.visual(
        Box((0.020, 0.030, 0.200)),
        origin=Origin(xyz=(-0.022, 0.045, 0.120)),
        material=frame_black,
        name="upright_right",
    )
    backrest.visual(
        Box((0.030, 0.120, 0.280)),
        origin=Origin(xyz=(-0.040, 0.0, 0.240)),
        material=frame_black,
        name="spine",
    )
    backrest.visual(
        Box((0.030, 0.180, 0.020)),
        origin=Origin(xyz=(-0.040, 0.0, 0.390)),
        material=frame_black,
        name="top_rail",
    )
    backrest.visual(
        back_panel_mesh,
        origin=Origin(xyz=(-0.080, 0.0, 0.410), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=back_fabric,
        name="back_panel",
    )
    backrest.visual(
        lumbar_pad_mesh,
        origin=Origin(xyz=(-0.095, 0.0, 0.295), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=back_fabric,
        name="lumbar_pad",
    )
    model.articulation(
        "back_recline",
        ArticulationType.REVOLUTE,
        parent=seat,
        child=backrest,
        origin=Origin(xyz=(-0.245, 0.0, 0.140)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=1.5,
            lower=-0.55,
            upper=0.0,
        ),
    )

    for index in range(5):
        angle = 2.0 * math.pi * index / 5.0
        fork = model.part(f"caster_fork_{index}")
        fork.visual(
            Cylinder(radius=0.005, length=0.020),
            origin=Origin(xyz=(0.0, 0.0, -0.010)),
            material=frame_black,
            name="fork_stem",
        )
        fork.visual(
            Box((0.018, 0.032, 0.008)),
            origin=Origin(xyz=(0.0, 0.0, -0.024)),
            material=frame_black,
            name="fork_crown",
        )
        fork.visual(
            Box((0.006, 0.004, 0.035)),
            origin=Origin(xyz=(0.0, -0.016, -0.0455)),
            material=frame_black,
            name="fork_leg_left",
        )
        fork.visual(
            Box((0.006, 0.004, 0.035)),
            origin=Origin(xyz=(0.0, 0.016, -0.0455)),
            material=frame_black,
            name="fork_leg_right",
        )
        model.articulation(
            f"caster_swivel_{index}",
            ArticulationType.CONTINUOUS,
            parent=base,
            child=fork,
            origin=Origin(xyz=_spoke_tip(spoke_tip_radius, angle, 0.065)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=4.0, velocity=8.0),
        )

        wheel = model.part(f"caster_wheel_{index}")
        wheel.visual(
            Cylinder(radius=0.028, length=0.016),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=wheel_rubber,
            name="wheel_tire",
        )
        wheel.visual(
            Cylinder(radius=0.006, length=0.028),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=chrome,
            name="wheel_hub",
        )
        model.articulation(
            f"caster_wheel_spin_{index}",
            ArticulationType.CONTINUOUS,
            parent=fork,
            child=wheel,
            origin=Origin(xyz=(0.0, 0.0, -0.056)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=2.0, velocity=20.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    column_outer = object_model.get_part("column_outer")
    column_inner = object_model.get_part("column_inner")
    seat = object_model.get_part("seat")
    backrest = object_model.get_part("backrest")

    base_to_column_outer = object_model.get_articulation("base_to_column_outer")
    column_height = object_model.get_articulation("column_height")
    seat_swivel = object_model.get_articulation("seat_swivel")
    back_recline = object_model.get_articulation("back_recline")

    seat_cushion = seat.get_visual("seat_cushion")
    support_pillar = seat.get_visual("support_pillar")
    back_mount = seat.get_visual("back_mount")
    top_head = column_inner.get_visual("top_head")
    lower_crossbar = backrest.get_visual("lower_crossbar")
    back_panel = backrest.get_visual("back_panel")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.allow_overlap(
        column_inner,
        column_outer,
        reason="Gas-lift piston intentionally telescopes inside the outer column shroud.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check("base_to_column_outer_is_fixed", base_to_column_outer.joint_type == ArticulationType.FIXED)
    ctx.check("column_height_axis_is_vertical", column_height.axis == (0.0, 0.0, 1.0))
    ctx.check("seat_swivel_axis_is_vertical", seat_swivel.axis == (0.0, 0.0, 1.0))
    ctx.check("back_recline_axis_is_transverse", back_recline.axis == (0.0, 1.0, 0.0))

    ctx.expect_contact(column_outer, base, elem_a="lower_cover", elem_b="hub_upper")
    ctx.expect_contact(seat, column_inner, elem_a=support_pillar, elem_b=top_head)
    ctx.expect_overlap(seat, column_inner, axes="xy", min_overlap=0.05, elem_a=support_pillar, elem_b=top_head)
    ctx.expect_contact(backrest, seat, elem_a=lower_crossbar, elem_b=back_mount)
    ctx.expect_gap(seat, base, axis="z", min_gap=0.22, max_gap=0.34)

    seat_cushion_aabb = ctx.part_element_world_aabb(seat, elem=seat_cushion)
    back_panel_aabb = ctx.part_element_world_aabb(backrest, elem=back_panel)
    base_aabb = ctx.part_world_aabb(base)
    if seat_cushion_aabb is not None:
        seat_top = seat_cushion_aabb[1][2]
        ctx.check(
            "seat_height_realistic",
            0.48 <= seat_top <= 0.59,
            details=f"seat top {seat_top:.3f} m outside realistic task-chair range",
        )
    if back_panel_aabb is not None:
        back_top = back_panel_aabb[1][2]
        ctx.check(
            "back_height_realistic",
            0.98 <= back_top <= 1.12,
            details=f"back top {back_top:.3f} m outside realistic task-chair range",
        )
    if base_aabb is not None:
        base_dx = base_aabb[1][0] - base_aabb[0][0]
        base_dy = base_aabb[1][1] - base_aabb[0][1]
        base_span = max(base_dx, base_dy)
        ctx.check(
            "five_star_base_span_realistic",
            0.67 <= base_span <= 0.78,
            details=f"base span {base_span:.3f} m outside realistic office-chair range",
        )

    for index in range(5):
        fork = object_model.get_part(f"caster_fork_{index}")
        wheel = object_model.get_part(f"caster_wheel_{index}")
        swivel = object_model.get_articulation(f"caster_swivel_{index}")
        spin = object_model.get_articulation(f"caster_wheel_spin_{index}")
        ctx.check(f"caster_swivel_{index}_axis_is_vertical", swivel.axis == (0.0, 0.0, 1.0))
        ctx.check(f"caster_wheel_spin_{index}_axis_is_lateral", spin.axis == (0.0, 1.0, 0.0))
        ctx.expect_contact(fork, base, elem_a="fork_stem", elem_b=f"spoke_{index}", name=f"caster_{index}_fork_mount")
        ctx.expect_contact(wheel, fork, elem_a="wheel_hub", elem_b="fork_leg_left", name=f"caster_{index}_hub_left_contact")
        ctx.expect_contact(wheel, fork, elem_a="wheel_hub", elem_b="fork_leg_right", name=f"caster_{index}_hub_right_contact")

    height_limits = column_height.motion_limits
    if height_limits is not None and height_limits.lower is not None and height_limits.upper is not None:
        seat_low_top = None
        with ctx.pose({column_height: height_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="column_height_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="column_height_lower_no_floating")
            ctx.expect_contact(seat, column_inner, elem_a=support_pillar, elem_b=top_head)
            low_aabb = ctx.part_element_world_aabb(seat, elem=seat_cushion)
            if low_aabb is not None:
                seat_low_top = low_aabb[1][2]
        with ctx.pose({column_height: height_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="column_height_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="column_height_upper_no_floating")
            ctx.expect_contact(seat, column_inner, elem_a=support_pillar, elem_b=top_head)
            high_aabb = ctx.part_element_world_aabb(seat, elem=seat_cushion)
            if seat_low_top is not None and high_aabb is not None:
                ctx.check(
                    "seat_height_adjusts_upward",
                    high_aabb[1][2] >= seat_low_top + 0.075,
                    details=f"seat vertical travel {high_aabb[1][2] - seat_low_top:.3f} m was smaller than expected",
                )

    recline_limits = back_recline.motion_limits
    if recline_limits is not None and recline_limits.lower is not None and recline_limits.upper is not None:
        with ctx.pose({back_recline: recline_limits.upper}):
            rest_back_panel_aabb = ctx.part_element_world_aabb(backrest, elem=back_panel)
            ctx.fail_if_parts_overlap_in_current_pose(name="back_recline_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="back_recline_upper_no_floating")
            ctx.expect_contact(backrest, seat, elem_a=lower_crossbar, elem_b=back_mount)
        with ctx.pose({back_recline: recline_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="back_recline_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="back_recline_lower_no_floating")
            ctx.expect_contact(backrest, seat, elem_a=lower_crossbar, elem_b=back_mount)
            reclined_back_panel_aabb = ctx.part_element_world_aabb(backrest, elem=back_panel)
            if rest_back_panel_aabb is not None and reclined_back_panel_aabb is not None:
                ctx.check(
                    "backrest_reclines_rearward",
                    reclined_back_panel_aabb[1][0] < rest_back_panel_aabb[1][0] - 0.10,
                    details=(
                        f"backrest rearward travel only "
                        f"{rest_back_panel_aabb[1][0] - reclined_back_panel_aabb[1][0]:.3f} m"
                    ),
                )

    with ctx.pose(
        {
            seat_swivel: math.pi / 2.0,
            object_model.get_articulation("caster_swivel_0"): math.pi / 3.0,
            object_model.get_articulation("caster_wheel_spin_0"): math.pi / 2.0,
        }
    ):
        ctx.fail_if_parts_overlap_in_current_pose(name="sample_swivel_pose_no_overlap")
        ctx.fail_if_isolated_parts(name="sample_swivel_pose_no_floating")
        ctx.expect_contact(seat, column_inner, elem_a=support_pillar, elem_b=top_head)
        ctx.expect_contact(
            object_model.get_part("caster_wheel_0"),
            object_model.get_part("caster_fork_0"),
            elem_a="wheel_hub",
            elem_b="fork_leg_left",
            name="caster_0_left_contact_in_swivel_pose",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
