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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def add_box_visual(
    part,
    name: str,
    size: tuple[float, float, float],
    xyz: tuple[float, float, float],
    *,
    material,
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> None:
    part.visual(
        Box(size),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
        name=name,
    )


def add_cylinder_visual(
    part,
    name: str,
    radius: float,
    length: float,
    xyz: tuple[float, float, float],
    *,
    material,
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
        name=name,
    )


def add_rect_tube(
    part,
    prefix: str,
    *,
    length: float,
    outer_y: float,
    outer_z: float,
    wall: float,
    material,
    x0: float = 0.0,
    rear_cap: float = 0.0,
    front_cap: float = 0.0,
) -> None:
    body_length = length - rear_cap - front_cap
    inner_y = outer_y - 2.0 * wall
    inner_z = outer_z - 2.0 * wall
    if body_length <= 0.0 or inner_y <= 0.0 or inner_z <= 0.0:
        raise ValueError("Invalid rectangular tube dimensions")

    body_center_x = x0 + rear_cap + body_length / 2.0
    side_center_z = 0.0
    top_center_z = outer_z / 2.0 - wall / 2.0
    side_center_y = outer_y / 2.0 - wall / 2.0

    add_box_visual(
        part,
        f"{prefix}_top",
        (body_length, outer_y, wall),
        (body_center_x, 0.0, top_center_z),
        material=material,
    )
    add_box_visual(
        part,
        f"{prefix}_bottom",
        (body_length, outer_y, wall),
        (body_center_x, 0.0, -top_center_z),
        material=material,
    )
    add_box_visual(
        part,
        f"{prefix}_left",
        (body_length, wall, inner_z),
        (body_center_x, side_center_y, side_center_z),
        material=material,
    )
    add_box_visual(
        part,
        f"{prefix}_right",
        (body_length, wall, inner_z),
        (body_center_x, -side_center_y, side_center_z),
        material=material,
    )

    if rear_cap > 0.0:
        add_box_visual(
            part,
            f"{prefix}_rear_cap",
            (rear_cap, outer_y, outer_z),
            (x0 + rear_cap / 2.0, 0.0, 0.0),
            material=material,
        )
    if front_cap > 0.0:
        add_box_visual(
            part,
            f"{prefix}_front_cap",
            (front_cap, outer_y, outer_z),
            (x0 + length - front_cap / 2.0, 0.0, 0.0),
            material=material,
        )


def add_guide_pads(
    part,
    prefix: str,
    *,
    x0: float,
    length: float,
    tube_outer_y: float,
    tube_outer_z: float,
    side_pad: float,
    top_pad: float,
    side_height: float,
    top_width: float,
    material,
) -> None:
    center_x = x0 + length / 2.0
    if side_pad > 0.0:
        add_box_visual(
            part,
            f"{prefix}_left_pad",
            (length, side_pad, side_height),
            (center_x, tube_outer_y / 2.0 + side_pad / 2.0, 0.0),
            material=material,
        )
        add_box_visual(
            part,
            f"{prefix}_right_pad",
            (length, side_pad, side_height),
            (center_x, -(tube_outer_y / 2.0 + side_pad / 2.0), 0.0),
            material=material,
        )
    if top_pad > 0.0:
        add_box_visual(
            part,
            f"{prefix}_top_pad",
            (length, top_width, top_pad),
            (center_x, 0.0, tube_outer_z / 2.0 + top_pad / 2.0),
            material=material,
        )
        add_box_visual(
            part,
            f"{prefix}_bottom_pad",
            (length, top_width, top_pad),
            (center_x, 0.0, -(tube_outer_z / 2.0 + top_pad / 2.0)),
            material=material,
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="side_wall_telescoping_boom")

    support_steel = model.material("support_steel", rgba=(0.20, 0.22, 0.24, 1.0))
    boom_paint = model.material("boom_paint", rgba=(0.92, 0.76, 0.12, 1.0))
    inner_stage_paint = model.material("inner_stage_paint", rgba=(0.95, 0.83, 0.24, 1.0))
    tip_paint = model.material("tip_paint", rgba=(0.90, 0.48, 0.10, 1.0))
    wear_pad = model.material("wear_pad", rgba=(0.08, 0.08, 0.08, 1.0))
    hardware = model.material("hardware", rgba=(0.62, 0.65, 0.68, 1.0))

    support = model.part("side_support")
    add_box_visual(
        support,
        "wall_plate",
        (0.04, 0.58, 0.96),
        (-0.02, 0.0, 0.0),
        material=support_steel,
    )
    add_box_visual(
        support,
        "carriage_block",
        (0.18, 0.24, 0.30),
        (0.09, 0.0, 0.0),
        material=support_steel,
    )

    brace_angle = math.atan2(-0.20, 0.22)
    brace_length = math.hypot(0.22, 0.20)
    add_box_visual(
        support,
        "upper_brace",
        (brace_length, 0.18, 0.06),
        (0.10, 0.0, 0.22),
        material=support_steel,
        rpy=(0.0, brace_angle, 0.0),
    )
    add_box_visual(
        support,
        "lower_brace",
        (brace_length, 0.18, 0.06),
        (0.10, 0.0, -0.22),
        material=support_steel,
        rpy=(0.0, -brace_angle, 0.0),
    )

    add_rect_tube(
        support,
        "outer_sleeve",
        length=0.96,
        outer_y=0.18,
        outer_z=0.22,
        wall=0.012,
        material=support_steel,
        x0=0.18,
        rear_cap=0.014,
    )

    for idx, (y_pos, z_pos) in enumerate(
        (
            (-0.20, -0.28),
            (0.20, -0.28),
            (-0.20, 0.28),
            (0.20, 0.28),
        ),
        start=1,
    ):
        add_cylinder_visual(
            support,
            f"anchor_bolt_{idx}",
            radius=0.02,
            length=0.012,
            xyz=(0.006, y_pos, z_pos),
            material=hardware,
            rpy=(0.0, math.pi / 2.0, 0.0),
        )

    first_stage = model.part("first_stage")
    add_rect_tube(
        first_stage,
        "first_stage_tube",
        length=1.02,
        outer_y=0.146,
        outer_z=0.186,
        wall=0.009,
        material=boom_paint,
        rear_cap=0.009,
    )
    add_guide_pads(
        first_stage,
        "first_stage_guides",
        x0=0.06,
        length=0.28,
        tube_outer_y=0.146,
        tube_outer_z=0.186,
        side_pad=0.005,
        top_pad=0.005,
        side_height=0.12,
        top_width=0.11,
        material=wear_pad,
    )

    second_stage = model.part("second_stage")
    add_rect_tube(
        second_stage,
        "second_stage_tube",
        length=0.90,
        outer_y=0.120,
        outer_z=0.160,
        wall=0.008,
        material=inner_stage_paint,
        rear_cap=0.008,
    )
    add_guide_pads(
        second_stage,
        "second_stage_guides",
        x0=0.06,
        length=0.24,
        tube_outer_y=0.120,
        tube_outer_z=0.160,
        side_pad=0.004,
        top_pad=0.004,
        side_height=0.11,
        top_width=0.09,
        material=wear_pad,
    )

    tip_stage = model.part("tip_stage")
    add_rect_tube(
        tip_stage,
        "tip_stage_tube",
        length=0.78,
        outer_y=0.096,
        outer_z=0.136,
        wall=0.007,
        material=tip_paint,
        rear_cap=0.007,
        front_cap=0.018,
    )
    add_guide_pads(
        tip_stage,
        "tip_stage_guides",
        x0=0.06,
        length=0.22,
        tube_outer_y=0.096,
        tube_outer_z=0.136,
        side_pad=0.004,
        top_pad=0.004,
        side_height=0.10,
        top_width=0.075,
        material=wear_pad,
    )
    add_box_visual(
        tip_stage,
        "tip_nose",
        (0.10, 0.06, 0.09),
        (0.83, 0.0, 0.0),
        material=tip_paint,
    )
    add_box_visual(
        tip_stage,
        "padeye_plate",
        (0.016, 0.05, 0.12),
        (0.87, 0.0, 0.08),
        material=hardware,
    )
    add_cylinder_visual(
        tip_stage,
        "padeye_pin",
        radius=0.015,
        length=0.07,
        xyz=(0.87, 0.0, 0.11),
        material=hardware,
        rpy=(math.pi / 2.0, 0.0, 0.0),
    )

    model.articulation(
        "outer_to_first_stage",
        ArticulationType.PRISMATIC,
        parent=support,
        child=first_stage,
        origin=Origin(xyz=(0.20, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=4500.0,
            velocity=0.30,
            lower=0.0,
            upper=0.48,
        ),
    )
    model.articulation(
        "first_to_second_stage",
        ArticulationType.PRISMATIC,
        parent=first_stage,
        child=second_stage,
        origin=Origin(xyz=(0.18, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=3200.0,
            velocity=0.32,
            lower=0.0,
            upper=0.42,
        ),
    )
    model.articulation(
        "second_to_tip_stage",
        ArticulationType.PRISMATIC,
        parent=second_stage,
        child=tip_stage,
        origin=Origin(xyz=(0.16, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2200.0,
            velocity=0.35,
            lower=0.0,
            upper=0.34,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support = object_model.get_part("side_support")
    first_stage = object_model.get_part("first_stage")
    second_stage = object_model.get_part("second_stage")
    tip_stage = object_model.get_part("tip_stage")

    outer_to_first = object_model.get_articulation("outer_to_first_stage")
    first_to_second = object_model.get_articulation("first_to_second_stage")
    second_to_tip = object_model.get_articulation("second_to_tip_stage")

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

    for articulation in (outer_to_first, first_to_second, second_to_tip):
        ctx.check(
            f"{articulation.name}_is_prismatic",
            articulation.articulation_type == ArticulationType.PRISMATIC,
            f"{articulation.name} should be a prismatic boom section joint",
        )
        ctx.check(
            f"{articulation.name}_axis_points_forward",
            tuple(round(value, 6) for value in articulation.axis) == (1.0, 0.0, 0.0),
            f"{articulation.name} axis is {articulation.axis}, expected +X boom extension",
        )

    with ctx.pose(
        {
            outer_to_first: 0.0,
            first_to_second: 0.0,
            second_to_tip: 0.0,
        }
    ):
        ctx.expect_contact(
            first_stage,
            support,
            contact_tol=0.001,
            name="first_stage_supported_by_outer_sleeve",
        )
        ctx.expect_contact(
            second_stage,
            first_stage,
            contact_tol=0.001,
            name="second_stage_supported_by_first_stage",
        )
        ctx.expect_contact(
            tip_stage,
            second_stage,
            contact_tol=0.001,
            name="tip_stage_supported_by_second_stage",
        )
        ctx.expect_origin_gap(
            first_stage,
            support,
            axis="x",
            min_gap=0.19,
            max_gap=0.21,
            name="first_stage_retracted_origin_position",
        )
        ctx.expect_origin_gap(
            second_stage,
            first_stage,
            axis="x",
            min_gap=0.17,
            max_gap=0.19,
            name="second_stage_retracted_origin_position",
        )
        ctx.expect_origin_gap(
            tip_stage,
            second_stage,
            axis="x",
            min_gap=0.15,
            max_gap=0.17,
            name="tip_stage_retracted_origin_position",
        )

    with ctx.pose(
        {
            outer_to_first: 0.48,
            first_to_second: 0.42,
            second_to_tip: 0.34,
        }
    ):
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_fully_extended")
        ctx.expect_contact(
            first_stage,
            support,
            contact_tol=0.001,
            name="first_stage_remains_guided_when_extended",
        )
        ctx.expect_contact(
            second_stage,
            first_stage,
            contact_tol=0.001,
            name="second_stage_remains_guided_when_extended",
        )
        ctx.expect_contact(
            tip_stage,
            second_stage,
            contact_tol=0.001,
            name="tip_stage_remains_guided_when_extended",
        )
        ctx.expect_origin_gap(
            first_stage,
            support,
            axis="x",
            min_gap=0.67,
            max_gap=0.69,
            name="first_stage_extends_forward",
        )
        ctx.expect_origin_gap(
            second_stage,
            first_stage,
            axis="x",
            min_gap=0.59,
            max_gap=0.61,
            name="second_stage_extends_forward",
        )
        ctx.expect_origin_gap(
            tip_stage,
            second_stage,
            axis="x",
            min_gap=0.49,
            max_gap=0.51,
            name="tip_stage_extends_forward",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
