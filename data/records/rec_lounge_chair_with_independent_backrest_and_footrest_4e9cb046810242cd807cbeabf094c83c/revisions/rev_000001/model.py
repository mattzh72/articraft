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
    section_loft,
)


def _soft_box_along_x(
    length_x: float,
    width_y: float,
    height_z: float,
    radius: float,
    mesh_name: str,
):
    geom = ExtrudeGeometry(
        rounded_rect_profile(height_z, width_y, radius, corner_segments=10),
        length_x,
    )
    geom.rotate_y(math.pi / 2.0)
    return mesh_from_geometry(geom, mesh_name)


def _yz_section(
    x: float,
    width_y: float,
    height_z: float,
    radius: float,
    *,
    z_center: float = 0.0,
):
    return [
        (x, y, z_center + z)
        for z, y in rounded_rect_profile(height_z, width_y, radius, corner_segments=10)
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="chaise_recliner")

    upholstery = model.material("upholstery", rgba=(0.63, 0.57, 0.50, 1.0))
    frame_finish = model.material("frame_finish", rgba=(0.18, 0.17, 0.16, 1.0))
    hardware = model.material("hardware", rgba=(0.52, 0.54, 0.57, 1.0))

    seat_pad_mesh = _soft_box_along_x(0.72, 0.54, 0.20, 0.055, "seat_pad_mesh")
    rear_bridge_mesh = _soft_box_along_x(0.12, 0.54, 0.10, 0.025, "rear_bridge_mesh")
    front_bridge_mesh = _soft_box_along_x(0.08, 0.54, 0.06, 0.018, "front_bridge_mesh")
    back_beam_mesh = _soft_box_along_x(0.10, 0.56, 0.10, 0.028, "back_beam_mesh")
    back_panel_mesh = _soft_box_along_x(0.16, 0.56, 0.76, 0.065, "back_panel_mesh")
    head_pillow_mesh = _soft_box_along_x(0.12, 0.50, 0.18, 0.05, "head_pillow_mesh")
    leg_hinge_beam_mesh = _soft_box_along_x(0.12, 0.60, 0.08, 0.025, "leg_hinge_beam_mesh")
    leg_panel_mesh = mesh_from_geometry(
        section_loft(
            (
                _yz_section(-0.31, 0.60, 0.10, 0.035, z_center=-0.01),
                _yz_section(-0.12, 0.58, 0.12, 0.040, z_center=0.0),
                _yz_section(0.16, 0.56, 0.12, 0.040, z_center=0.0),
                _yz_section(0.31, 0.48, 0.08, 0.025, z_center=-0.02),
            )
        ),
        "leg_panel_shell_v2",
    )
    arm_mesh = mesh_from_geometry(
        section_loft(
            (
                _yz_section(-0.78, 0.15, 0.46, 0.045, z_center=-0.03),
                _yz_section(-0.42, 0.18, 0.56, 0.060, z_center=0.02),
                _yz_section(0.30, 0.18, 0.54, 0.060, z_center=0.01),
                _yz_section(0.78, 0.12, 0.36, 0.035, z_center=-0.08),
            )
        ),
        "arm_mesh",
    )

    base_frame = model.part("base_frame")
    base_frame.visual(
        Box((1.42, 0.74, 0.22)),
        origin=Origin(xyz=(0.78, 0.0, 0.11)),
        material=frame_finish,
        name="lower_chassis",
    )
    base_frame.visual(
        seat_pad_mesh,
        origin=Origin(xyz=(0.61, 0.0, 0.32)),
        material=upholstery,
        name="seat_pad",
    )
    base_frame.visual(
        arm_mesh,
        origin=Origin(xyz=(0.78, 0.41, 0.31)),
        material=upholstery,
        name="left_arm",
    )
    base_frame.visual(
        arm_mesh,
        origin=Origin(xyz=(0.78, -0.41, 0.31)),
        material=upholstery,
        name="right_arm",
    )
    base_frame.visual(
        rear_bridge_mesh,
        origin=Origin(xyz=(0.19, 0.0, 0.27)),
        material=upholstery,
        name="rear_bridge",
    )
    base_frame.visual(
        front_bridge_mesh,
        origin=Origin(xyz=(0.94, 0.0, 0.19)),
        material=upholstery,
        name="front_bridge",
    )
    base_frame.visual(
        Cylinder(radius=0.045, length=0.04),
        origin=Origin(xyz=(0.18, 0.325, 0.43), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hardware,
        name="left_back_boss",
    )
    base_frame.visual(
        Cylinder(radius=0.045, length=0.04),
        origin=Origin(xyz=(0.18, -0.325, 0.43), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hardware,
        name="right_back_boss",
    )
    base_frame.visual(
        Cylinder(radius=0.038, length=0.04),
        origin=Origin(xyz=(1.03, 0.325, 0.37), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hardware,
        name="left_front_boss",
    )
    base_frame.visual(
        Cylinder(radius=0.038, length=0.04),
        origin=Origin(xyz=(1.03, -0.325, 0.37), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hardware,
        name="right_front_boss",
    )
    base_frame.inertial = Inertial.from_geometry(
        Box((1.56, 0.92, 0.58)),
        mass=45.0,
        origin=Origin(xyz=(0.78, 0.0, 0.29)),
    )

    backrest = model.part("backrest")
    backrest.visual(
        Cylinder(radius=0.045, length=0.03),
        origin=Origin(xyz=(0.0, 0.29, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hardware,
        name="left_back_trunnion",
    )
    backrest.visual(
        Cylinder(radius=0.045, length=0.03),
        origin=Origin(xyz=(0.0, -0.29, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hardware,
        name="right_back_trunnion",
    )
    backrest.visual(
        back_beam_mesh,
        origin=Origin(xyz=(-0.03, 0.0, 0.05)),
        material=upholstery,
        name="back_hinge_beam",
    )
    backrest.visual(
        back_panel_mesh,
        origin=Origin(xyz=(-0.11, 0.0, 0.43)),
        material=upholstery,
        name="back_panel",
    )
    backrest.visual(
        head_pillow_mesh,
        origin=Origin(xyz=(-0.10, 0.0, 0.72)),
        material=upholstery,
        name="head_pillow",
    )
    backrest.inertial = Inertial.from_geometry(
        Box((0.24, 0.60, 0.86)),
        mass=12.0,
        origin=Origin(xyz=(-0.08, 0.0, 0.40)),
    )

    leg_rest = model.part("leg_rest")
    leg_rest.visual(
        Cylinder(radius=0.038, length=0.03),
        origin=Origin(xyz=(0.0, 0.29, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hardware,
        name="left_front_trunnion",
    )
    leg_rest.visual(
        Cylinder(radius=0.038, length=0.03),
        origin=Origin(xyz=(0.0, -0.29, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hardware,
        name="right_front_trunnion",
    )
    leg_rest.visual(
        leg_hinge_beam_mesh,
        origin=Origin(xyz=(0.02, 0.0, 0.0)),
        material=upholstery,
        name="leg_hinge_beam",
    )
    leg_rest.visual(
        leg_panel_mesh,
        origin=Origin(xyz=(0.33, 0.0, 0.0)),
        material=upholstery,
        name="leg_panel",
    )
    leg_rest.inertial = Inertial.from_geometry(
        Box((0.62, 0.60, 0.14)),
        mass=8.0,
        origin=Origin(xyz=(0.31, 0.0, 0.0)),
    )

    model.articulation(
        "backrest_pivot",
        ArticulationType.REVOLUTE,
        parent=base_frame,
        child=backrest,
        origin=Origin(xyz=(0.18, 0.0, 0.43)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.0,
            lower=-0.12,
            upper=0.42,
        ),
    )
    model.articulation(
        "leg_rest_pivot",
        ArticulationType.REVOLUTE,
        parent=base_frame,
        child=leg_rest,
        origin=Origin(xyz=(1.03, 0.0, 0.37)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=14.0,
            velocity=1.2,
            lower=0.0,
            upper=1.05,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base_frame = object_model.get_part("base_frame")
    backrest = object_model.get_part("backrest")
    leg_rest = object_model.get_part("leg_rest")
    backrest_pivot = object_model.get_articulation("backrest_pivot")
    leg_rest_pivot = object_model.get_articulation("leg_rest_pivot")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_overlaps(max_pose_samples=24)

    ctx.check(
        "backrest_axis_transverse",
        backrest_pivot.axis == (0.0, -1.0, 0.0),
        details=f"Expected transverse backrest axis, got {backrest_pivot.axis!r}",
    )
    ctx.check(
        "leg_rest_axis_transverse",
        leg_rest_pivot.axis == (0.0, -1.0, 0.0),
        details=f"Expected transverse leg-rest axis, got {leg_rest_pivot.axis!r}",
    )

    back_limits = backrest_pivot.motion_limits
    leg_limits = leg_rest_pivot.motion_limits
    ctx.check(
        "backrest_motion_range_realistic",
        back_limits is not None
        and back_limits.lower is not None
        and back_limits.upper is not None
        and back_limits.lower < 0.0 < back_limits.upper
        and (back_limits.upper - back_limits.lower) <= 0.8,
        details="Backrest should allow modest upright adjustment and deeper recline.",
    )
    ctx.check(
        "leg_rest_motion_range_realistic",
        leg_limits is not None
        and leg_limits.lower is not None
        and leg_limits.upper is not None
        and leg_limits.lower == 0.0
        and 0.8 <= leg_limits.upper <= 1.2,
        details="Leg rest should raise from the seat front through a realistic chaise angle.",
    )

    ctx.expect_contact(
        backrest,
        base_frame,
        elem_a="left_back_trunnion",
        elem_b="left_back_boss",
    )
    ctx.expect_contact(
        backrest,
        base_frame,
        elem_a="right_back_trunnion",
        elem_b="right_back_boss",
    )
    ctx.expect_contact(
        leg_rest,
        base_frame,
        elem_a="left_front_trunnion",
        elem_b="left_front_boss",
    )
    ctx.expect_contact(
        leg_rest,
        base_frame,
        elem_a="right_front_trunnion",
        elem_b="right_front_boss",
    )

    ctx.expect_overlap(
        backrest,
        base_frame,
        axes="y",
        elem_a="back_panel",
        elem_b="seat_pad",
        min_overlap=0.52,
    )
    ctx.expect_gap(
        backrest,
        base_frame,
        axis="z",
        positive_elem="back_panel",
        negative_elem="seat_pad",
        min_gap=0.01,
        max_gap=0.18,
    )
    ctx.expect_overlap(
        leg_rest,
        base_frame,
        axes="y",
        elem_a="leg_panel",
        elem_b="seat_pad",
        min_overlap=0.52,
    )
    ctx.expect_gap(
        leg_rest,
        base_frame,
        axis="x",
        positive_elem="leg_panel",
        negative_elem="seat_pad",
        min_gap=0.0,
        max_gap=0.10,
    )
    ctx.expect_gap(
        leg_rest,
        base_frame,
        axis="z",
        positive_elem="leg_panel",
        negative_elem="lower_chassis",
        min_gap=0.08,
    )

    seat_aabb = ctx.part_element_world_aabb(base_frame, elem="seat_pad")
    base_aabb = ctx.part_world_aabb(base_frame)
    leg_rest_aabb = ctx.part_world_aabb(leg_rest)
    assert seat_aabb is not None
    assert base_aabb is not None
    assert leg_rest_aabb is not None

    overall_length = max(base_aabb[1][0], leg_rest_aabb[1][0]) - min(
        base_aabb[0][0], leg_rest_aabb[0][0]
    )
    ctx.check(
        "seat_height_realistic",
        0.40 <= seat_aabb[1][2] <= 0.46,
        details=f"Seat top should be near lounge-chair height, got {seat_aabb[1][2]:.3f} m.",
    )
    ctx.check(
        "overall_length_realistic",
        1.50 <= overall_length <= 1.75,
        details=f"Chaise recliner should be full-length, got {overall_length:.3f} m.",
    )

    assert back_limits is not None
    assert back_limits.lower is not None
    assert back_limits.upper is not None
    upright_back_aabb = None
    reclined_back_aabb = None
    with ctx.pose({backrest_pivot: back_limits.lower}):
        ctx.fail_if_parts_overlap_in_current_pose(name="backrest_lower_no_overlap")
        ctx.fail_if_isolated_parts(name="backrest_lower_no_floating")
        ctx.expect_contact(
            backrest,
            base_frame,
            elem_a="left_back_trunnion",
            elem_b="left_back_boss",
            name="backrest_lower_left_hinge_contact",
        )
        ctx.expect_contact(
            backrest,
            base_frame,
            elem_a="right_back_trunnion",
            elem_b="right_back_boss",
            name="backrest_lower_right_hinge_contact",
        )
        upright_back_aabb = ctx.part_world_aabb(backrest)
        assert upright_back_aabb is not None
    with ctx.pose({backrest_pivot: back_limits.upper}):
        ctx.fail_if_parts_overlap_in_current_pose(name="backrest_upper_no_overlap")
        ctx.fail_if_isolated_parts(name="backrest_upper_no_floating")
        ctx.expect_contact(
            backrest,
            base_frame,
            elem_a="left_back_trunnion",
            elem_b="left_back_boss",
            name="backrest_upper_left_hinge_contact",
        )
        ctx.expect_contact(
            backrest,
            base_frame,
            elem_a="right_back_trunnion",
            elem_b="right_back_boss",
            name="backrest_upper_right_hinge_contact",
        )
        reclined_back_aabb = ctx.part_world_aabb(backrest)
        assert reclined_back_aabb is not None

    assert upright_back_aabb is not None
    assert reclined_back_aabb is not None
    ctx.check(
        "backrest_recline_changes_pose",
        reclined_back_aabb[0][0] < upright_back_aabb[0][0] - 0.08
        and reclined_back_aabb[1][2] < upright_back_aabb[1][2] - 0.05,
        details="Backrest should swing rearward and lower at the top as it reclines.",
    )

    assert leg_limits is not None
    assert leg_limits.lower is not None
    assert leg_limits.upper is not None
    rest_leg_aabb = ctx.part_world_aabb(leg_rest)
    raised_leg_aabb = None
    assert rest_leg_aabb is not None
    with ctx.pose({leg_rest_pivot: leg_limits.upper}):
        ctx.fail_if_parts_overlap_in_current_pose(name="leg_rest_upper_no_overlap")
        ctx.fail_if_isolated_parts(name="leg_rest_upper_no_floating")
        ctx.expect_contact(
            leg_rest,
            base_frame,
            elem_a="left_front_trunnion",
            elem_b="left_front_boss",
            name="leg_rest_upper_left_hinge_contact",
        )
        ctx.expect_contact(
            leg_rest,
            base_frame,
            elem_a="right_front_trunnion",
            elem_b="right_front_boss",
            name="leg_rest_upper_right_hinge_contact",
        )
        raised_leg_aabb = ctx.part_world_aabb(leg_rest)
        assert raised_leg_aabb is not None

    assert raised_leg_aabb is not None
    ctx.check(
        "leg_rest_raises_clear_of_seat",
        raised_leg_aabb[1][2] > rest_leg_aabb[1][2] + 0.20
        and raised_leg_aabb[1][0] < rest_leg_aabb[1][0] - 0.10,
        details="Leg rest should arc upward and back from the front edge when raised.",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
