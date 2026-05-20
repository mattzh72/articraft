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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _rect_profile(width: float, depth: float) -> list[tuple[float, float]]:
    half_w = width / 2.0
    half_d = depth / 2.0
    return [
        (-half_w, -half_d),
        (half_w, -half_d),
        (half_w, half_d),
        (-half_w, half_d),
    ]


def _tube_mesh(
    *,
    outer_x: float,
    outer_y: float,
    inner_x: float,
    inner_y: float,
    length: float,
    name: str,
):
    return mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            _rect_profile(outer_x, outer_y),
            [_rect_profile(inner_x, inner_y)],
            length,
            center=True,
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="drafting_table_desk")

    powder_coat = model.material("powder_coat", rgba=(0.18, 0.19, 0.20, 1.0))
    slider_gray = model.material("slider_gray", rgba=(0.48, 0.50, 0.53, 1.0))
    walnut = model.material("walnut", rgba=(0.54, 0.38, 0.23, 1.0))
    edge_wood = model.material("edge_wood", rgba=(0.45, 0.31, 0.19, 1.0))
    steel_dark = model.material("steel_dark", rgba=(0.26, 0.27, 0.30, 1.0))

    leg_x = 0.34
    leg_y = -0.10
    sleeve_outer_x = 0.068
    sleeve_outer_y = 0.058
    sleeve_inner_x = 0.058
    sleeve_inner_y = 0.048
    inner_leg_outer_x = 0.050
    inner_leg_outer_y = 0.040
    inner_leg_hole_x = 0.042
    inner_leg_hole_y = 0.032
    guide_pad_thickness = 0.004
    guide_pad_width = 0.020
    lower_guide_length = 0.11
    upper_guide_length = 0.11

    outer_sleeve_length = 0.52
    outer_sleeve_center_z = 0.30
    leg_joint_z = outer_sleeve_center_z + outer_sleeve_length / 2.0

    inner_leg_length = 0.62
    inner_leg_center_z = 0.03
    leg_travel = 0.20

    receiver_length = 0.18
    receiver_center_z = -0.09
    upper_frame_z_from_leg = 0.30

    top_width = 1.20
    top_depth = 0.78
    top_thickness = 0.03
    hinge_to_panel_gap = 0.11

    base_frame = model.part("base_frame")
    base_frame.visual(
        Box((0.09, 0.58, 0.04)),
        origin=Origin(xyz=(-leg_x, 0.08, 0.02)),
        material=powder_coat,
        name="left_foot_runner",
    )
    base_frame.visual(
        Box((0.09, 0.58, 0.04)),
        origin=Origin(xyz=(leg_x, 0.08, 0.02)),
        material=powder_coat,
        name="right_foot_runner",
    )
    base_frame.visual(
        Box((0.62, 0.05, 0.04)),
        origin=Origin(xyz=(0.0, 0.27, 0.02)),
        material=powder_coat,
        name="front_stretcher",
    )
    base_frame.visual(
        Box((0.62, 0.05, 0.04)),
        origin=Origin(xyz=(0.0, -0.17, 0.02)),
        material=powder_coat,
        name="rear_stretcher",
    )
    base_frame.visual(
        _tube_mesh(
            outer_x=sleeve_outer_x,
            outer_y=sleeve_outer_y,
            inner_x=sleeve_inner_x,
            inner_y=sleeve_inner_y,
            length=outer_sleeve_length,
            name="left_outer_sleeve",
        ),
        origin=Origin(xyz=(-leg_x, leg_y, outer_sleeve_center_z)),
        material=powder_coat,
        name="left_outer_sleeve",
    )
    base_frame.visual(
        _tube_mesh(
            outer_x=sleeve_outer_x,
            outer_y=sleeve_outer_y,
            inner_x=sleeve_inner_x,
            inner_y=sleeve_inner_y,
            length=outer_sleeve_length,
            name="right_outer_sleeve",
        ),
        origin=Origin(xyz=(leg_x, leg_y, outer_sleeve_center_z)),
        material=powder_coat,
        name="right_outer_sleeve",
    )
    base_frame.inertial = Inertial.from_geometry(
        Box((0.80, 0.66, 0.58)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.06, 0.29)),
    )

    left_inner_leg = model.part("left_inner_leg")
    left_inner_leg.visual(
        _tube_mesh(
            outer_x=inner_leg_outer_x,
            outer_y=inner_leg_outer_y,
            inner_x=inner_leg_hole_x,
            inner_y=inner_leg_hole_y,
            length=inner_leg_length,
            name="left_inner_leg_tube",
        ),
        origin=Origin(xyz=(0.0, 0.0, inner_leg_center_z)),
        material=slider_gray,
        name="tube",
    )
    left_inner_leg.visual(
        Box((inner_leg_outer_x + 0.006, inner_leg_outer_y + 0.006, 0.01)),
        origin=Origin(
            xyz=(0.0, 0.0, inner_leg_center_z + inner_leg_length / 2.0 + 0.005)
        ),
        material=steel_dark,
        name="top_cap",
    )
    left_inner_leg.visual(
        Box((guide_pad_thickness, guide_pad_width, lower_guide_length)),
        origin=Origin(xyz=(0.027, 0.0, -0.14)),
        material=slider_gray,
        name="lower_right_guide",
    )
    left_inner_leg.visual(
        Box((guide_pad_thickness, guide_pad_width, lower_guide_length)),
        origin=Origin(xyz=(-0.027, 0.0, -0.14)),
        material=slider_gray,
        name="lower_left_guide",
    )
    left_inner_leg.visual(
        Box((guide_pad_thickness, guide_pad_width, upper_guide_length)),
        origin=Origin(xyz=(0.027, 0.0, 0.20)),
        material=slider_gray,
        name="upper_right_guide",
    )
    left_inner_leg.visual(
        Box((guide_pad_thickness, guide_pad_width, upper_guide_length)),
        origin=Origin(xyz=(-0.027, 0.0, 0.20)),
        material=slider_gray,
        name="upper_left_guide",
    )
    left_inner_leg.inertial = Inertial.from_geometry(
        Box((inner_leg_outer_x + 0.006, inner_leg_outer_y + 0.006, inner_leg_length + 0.01)),
        mass=2.1,
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
    )

    right_inner_leg = model.part("right_inner_leg")
    right_inner_leg.visual(
        _tube_mesh(
            outer_x=inner_leg_outer_x,
            outer_y=inner_leg_outer_y,
            inner_x=inner_leg_hole_x,
            inner_y=inner_leg_hole_y,
            length=inner_leg_length,
            name="right_inner_leg_tube",
        ),
        origin=Origin(xyz=(0.0, 0.0, inner_leg_center_z)),
        material=slider_gray,
        name="tube",
    )
    right_inner_leg.visual(
        Box((inner_leg_outer_x + 0.006, inner_leg_outer_y + 0.006, 0.01)),
        origin=Origin(
            xyz=(0.0, 0.0, inner_leg_center_z + inner_leg_length / 2.0 + 0.005)
        ),
        material=steel_dark,
        name="top_cap",
    )
    right_inner_leg.visual(
        Box((guide_pad_thickness, guide_pad_width, lower_guide_length)),
        origin=Origin(xyz=(0.027, 0.0, -0.14)),
        material=slider_gray,
        name="lower_right_guide",
    )
    right_inner_leg.visual(
        Box((guide_pad_thickness, guide_pad_width, lower_guide_length)),
        origin=Origin(xyz=(-0.027, 0.0, -0.14)),
        material=slider_gray,
        name="lower_left_guide",
    )
    right_inner_leg.visual(
        Box((guide_pad_thickness, guide_pad_width, upper_guide_length)),
        origin=Origin(xyz=(0.027, 0.0, 0.20)),
        material=slider_gray,
        name="upper_right_guide",
    )
    right_inner_leg.visual(
        Box((guide_pad_thickness, guide_pad_width, upper_guide_length)),
        origin=Origin(xyz=(-0.027, 0.0, 0.20)),
        material=slider_gray,
        name="upper_left_guide",
    )
    right_inner_leg.inertial = Inertial.from_geometry(
        Box((inner_leg_outer_x + 0.006, inner_leg_outer_y + 0.006, inner_leg_length + 0.01)),
        mass=2.1,
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
    )

    model.articulation(
        "base_to_left_inner_leg",
        ArticulationType.PRISMATIC,
        parent=base_frame,
        child=left_inner_leg,
        origin=Origin(xyz=(-leg_x, leg_y, leg_joint_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.10,
            lower=0.0,
            upper=leg_travel,
        ),
    )
    model.articulation(
        "base_to_right_inner_leg",
        ArticulationType.PRISMATIC,
        parent=base_frame,
        child=right_inner_leg,
        origin=Origin(xyz=(leg_x, leg_y, leg_joint_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.10,
            lower=0.0,
            upper=leg_travel,
        ),
    )

    upper_frame = model.part("upper_frame")
    upper_frame.visual(
        _tube_mesh(
            outer_x=sleeve_outer_x,
            outer_y=sleeve_outer_y,
            inner_x=sleeve_inner_x,
            inner_y=sleeve_inner_y,
            length=receiver_length,
            name="left_receiver_sleeve",
        ),
        origin=Origin(xyz=(-leg_x, 0.01, receiver_center_z)),
        material=powder_coat,
        name="left_receiver_sleeve",
    )
    upper_frame.visual(
        _tube_mesh(
            outer_x=sleeve_outer_x,
            outer_y=sleeve_outer_y,
            inner_x=sleeve_inner_x,
            inner_y=sleeve_inner_y,
            length=receiver_length,
            name="right_receiver_sleeve",
        ),
        origin=Origin(xyz=(leg_x, 0.01, receiver_center_z)),
        material=powder_coat,
        name="right_receiver_sleeve",
    )
    upper_frame.visual(
        Cylinder(radius=0.014, length=0.54),
        origin=Origin(xyz=(0.0, -0.049, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=powder_coat,
        name="rear_rail",
    )
    upper_frame.visual(
        Cylinder(radius=0.014, length=0.08),
        origin=Origin(xyz=(-0.43, -0.049, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=powder_coat,
        name="left_hinge_stub",
    )
    upper_frame.visual(
        Cylinder(radius=0.014, length=0.08),
        origin=Origin(xyz=(0.43, -0.049, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=powder_coat,
        name="right_hinge_stub",
    )
    upper_frame.visual(
        Box((0.86, 0.05, 0.03)),
        origin=Origin(xyz=(0.0, 0.15, -0.015)),
        material=powder_coat,
        name="front_support_bar",
    )
    upper_frame.visual(
        Box((0.03, 0.20, 0.02)),
        origin=Origin(xyz=(-0.405, 0.10, -0.015)),
        material=powder_coat,
        name="left_outboard_rail",
    )
    upper_frame.visual(
        Box((0.03, 0.20, 0.02)),
        origin=Origin(xyz=(0.405, 0.10, -0.015)),
        material=powder_coat,
        name="right_outboard_rail",
    )
    upper_frame.visual(
        Box((0.784, 0.06, 0.02)),
        origin=Origin(xyz=(0.0, -0.049, -0.01)),
        material=steel_dark,
        name="hinge_backing_plate",
    )
    upper_frame.visual(
        Box((0.05, 0.02, 0.05)),
        origin=Origin(xyz=(-0.18, -0.049, 0.015)),
        material=steel_dark,
        name="left_hinge_ear",
    )
    upper_frame.visual(
        Box((0.05, 0.02, 0.05)),
        origin=Origin(xyz=(0.18, -0.049, 0.015)),
        material=steel_dark,
        name="right_hinge_ear",
    )
    upper_frame.visual(
        Box((0.016, 0.058, 0.18)),
        origin=Origin(xyz=(-0.382, 0.01, -0.09)),
        material=steel_dark,
        name="left_receiver_bridge",
    )
    upper_frame.visual(
        Box((0.016, 0.058, 0.18)),
        origin=Origin(xyz=(0.382, 0.01, -0.09)),
        material=steel_dark,
        name="right_receiver_bridge",
    )
    upper_frame.inertial = Inertial.from_geometry(
        Box((0.96, 0.20, 0.24)),
        mass=4.2,
        origin=Origin(xyz=(0.0, 0.06, -0.08)),
    )

    model.articulation(
        "left_inner_leg_to_upper_frame",
        ArticulationType.FIXED,
        parent=left_inner_leg,
        child=upper_frame,
        origin=Origin(xyz=(leg_x, -0.01, upper_frame_z_from_leg)),
    )

    tabletop = model.part("tabletop")
    tabletop.visual(
        Box((top_width, top_depth, top_thickness)),
        origin=Origin(
            xyz=(0.0, hinge_to_panel_gap + top_depth / 2.0, top_thickness / 2.0)
        ),
        material=walnut,
        name="panel",
    )
    tabletop.visual(
        Box((1.08, 0.08, 0.016)),
        origin=Origin(xyz=(0.0, 0.12, 0.008)),
        material=edge_wood,
        name="rear_reinforcement",
    )
    tabletop.inertial = Inertial.from_geometry(
        Box((top_width, top_depth, 0.05)),
        mass=12.0,
        origin=Origin(xyz=(0.0, hinge_to_panel_gap + top_depth / 2.0, 0.025)),
    )

    model.articulation(
        "upper_frame_to_tabletop",
        ArticulationType.REVOLUTE,
        parent=upper_frame,
        child=tabletop,
        origin=Origin(xyz=(0.0, -0.049, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=1.5,
            lower=0.0,
            upper=1.10,
        ),
    )

    pencil_ledge = model.part("pencil_ledge")
    pencil_ledge.visual(
        Box((1.12, 0.032, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=edge_wood,
        name="ledge_base",
    )
    pencil_ledge.visual(
        Box((1.12, 0.010, 0.020)),
        origin=Origin(xyz=(0.0, 0.011, 0.010)),
        material=edge_wood,
        name="ledge_fence",
    )
    pencil_ledge.inertial = Inertial.from_geometry(
        Box((1.12, 0.032, 0.020)),
        mass=0.8,
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
    )

    model.articulation(
        "tabletop_to_pencil_ledge",
        ArticulationType.FIXED,
        parent=tabletop,
        child=pencil_ledge,
        origin=Origin(xyz=(0.0, hinge_to_panel_gap + top_depth - 0.016, top_thickness)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base_frame = object_model.get_part("base_frame")
    left_inner_leg = object_model.get_part("left_inner_leg")
    right_inner_leg = object_model.get_part("right_inner_leg")
    upper_frame = object_model.get_part("upper_frame")
    tabletop = object_model.get_part("tabletop")
    pencil_ledge = object_model.get_part("pencil_ledge")

    left_slide = object_model.get_articulation("base_to_left_inner_leg")
    right_slide = object_model.get_articulation("base_to_right_inner_leg")
    top_tilt = object_model.get_articulation("upper_frame_to_tabletop")

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

    for part_name in (
        "base_frame",
        "left_inner_leg",
        "right_inner_leg",
        "upper_frame",
        "tabletop",
        "pencil_ledge",
    ):
        ctx.check(
            f"{part_name} exists",
            object_model.get_part(part_name) is not None,
        )

    ctx.expect_contact(
        tabletop,
        upper_frame,
        elem_a="panel",
        elem_b="front_support_bar",
        name="tabletop rests on front support bar when flat",
    )
    ctx.expect_contact(
        pencil_ledge,
        tabletop,
        elem_a="ledge_base",
        elem_b="panel",
        name="pencil ledge is mounted onto the tabletop front edge",
    )

    rest_left_pos = ctx.part_world_position(left_inner_leg)
    rest_right_pos = ctx.part_world_position(right_inner_leg)
    rest_upper_frame_pos = ctx.part_world_position(upper_frame)
    rest_ledge_pos = ctx.part_world_position(pencil_ledge)

    with ctx.pose({left_slide: 0.20, right_slide: 0.20}):
        ctx.expect_within(
            left_inner_leg,
            base_frame,
            axes="xy",
            inner_elem="tube",
            outer_elem="left_outer_sleeve",
            margin=0.001,
            name="left inner leg stays guided within the left base sleeve",
        )
        ctx.expect_within(
            right_inner_leg,
            base_frame,
            axes="xy",
            inner_elem="tube",
            outer_elem="right_outer_sleeve",
            margin=0.001,
            name="right inner leg stays guided within the right base sleeve",
        )
        ctx.expect_overlap(
            left_inner_leg,
            base_frame,
            axes="z",
            elem_a="tube",
            elem_b="left_outer_sleeve",
            min_overlap=0.08,
            name="left inner leg retains insertion at full height",
        )
        ctx.expect_overlap(
            right_inner_leg,
            base_frame,
            axes="z",
            elem_a="tube",
            elem_b="right_outer_sleeve",
            min_overlap=0.08,
            name="right inner leg retains insertion at full height",
        )
        ctx.expect_within(
            left_inner_leg,
            upper_frame,
            axes="xy",
            inner_elem="tube",
            outer_elem="left_receiver_sleeve",
            margin=0.001,
            name="left upper receiver stays centered on the left telescoping leg",
        )
        ctx.expect_within(
            right_inner_leg,
            upper_frame,
            axes="xy",
            inner_elem="tube",
            outer_elem="right_receiver_sleeve",
            margin=0.001,
            name="right upper receiver stays centered on the right telescoping leg",
        )
        ctx.expect_overlap(
            left_inner_leg,
            upper_frame,
            axes="z",
            elem_a="tube",
            elem_b="left_receiver_sleeve",
            min_overlap=0.16,
            name="left receiver sleeve keeps a substantial grip on the leg",
        )
        ctx.expect_overlap(
            right_inner_leg,
            upper_frame,
            axes="z",
            elem_a="tube",
            elem_b="right_receiver_sleeve",
            min_overlap=0.16,
            name="right receiver sleeve keeps a substantial grip on the leg",
        )

        raised_left_pos = ctx.part_world_position(left_inner_leg)
        raised_right_pos = ctx.part_world_position(right_inner_leg)
        raised_upper_frame_pos = ctx.part_world_position(upper_frame)

    ctx.check(
        "left telescoping leg extends upward",
        rest_left_pos is not None
        and raised_left_pos is not None
        and raised_left_pos[2] > rest_left_pos[2] + 0.18,
        details=f"rest={rest_left_pos}, raised={raised_left_pos}",
    )
    ctx.check(
        "right telescoping leg extends upward",
        rest_right_pos is not None
        and raised_right_pos is not None
        and raised_right_pos[2] > rest_right_pos[2] + 0.18,
        details=f"rest={rest_right_pos}, raised={raised_right_pos}",
    )
    ctx.check(
        "upper frame rises with the telescoping height adjustment",
        rest_upper_frame_pos is not None
        and raised_upper_frame_pos is not None
        and raised_upper_frame_pos[2] > rest_upper_frame_pos[2] + 0.18,
        details=f"rest={rest_upper_frame_pos}, raised={raised_upper_frame_pos}",
    )

    with ctx.pose({top_tilt: 1.00}):
        tilted_ledge_pos = ctx.part_world_position(pencil_ledge)

    ctx.check(
        "drafting surface tilts upward from the rear hinge rail",
        rest_ledge_pos is not None
        and tilted_ledge_pos is not None
        and tilted_ledge_pos[2] > rest_ledge_pos[2] + 0.45,
        details=f"rest={rest_ledge_pos}, tilted={tilted_ledge_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
