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
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _rounded_panel_mesh(name: str, *, width: float, depth: float, height: float, radius: float):
    return mesh_from_geometry(
        ExtrudeGeometry.from_z0(
            rounded_rect_profile(width, depth, radius, corner_segments=8),
            height,
            cap=True,
            closed=True,
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="book_scanner_flatbed")

    body_dark = model.material("body_dark", rgba=(0.23, 0.24, 0.26, 1.0))
    body_mid = model.material("body_mid", rgba=(0.31, 0.32, 0.34, 1.0))
    lid_light = model.material("lid_light", rgba=(0.82, 0.84, 0.86, 1.0))
    lid_pad = model.material("lid_pad", rgba=(0.95, 0.95, 0.93, 1.0))
    glass = model.material("glass", rgba=(0.42, 0.60, 0.68, 0.35))
    platen_black = model.material("platen_black", rgba=(0.08, 0.09, 0.10, 1.0))
    hinge_dark = model.material("hinge_dark", rgba=(0.14, 0.15, 0.16, 1.0))
    hinge_pin = model.material("hinge_pin", rgba=(0.62, 0.64, 0.67, 1.0))
    rubber = model.material("rubber", rgba=(0.09, 0.09, 0.09, 1.0))
    panel_blue = model.material("panel_blue", rgba=(0.28, 0.47, 0.67, 1.0))

    body_width = 0.520
    body_depth = 0.360
    foot_height = 0.006
    base_height = 0.050
    shoulder_height = 0.015
    frame_height = 0.008
    total_body_height = foot_height + base_height + shoulder_height + frame_height

    platen_outer_width = 0.338
    platen_outer_depth = 0.236
    platen_glass_width = 0.308
    platen_glass_depth = 0.226

    hinge_axis_y = -0.208
    hinge_axis_z = 0.080
    upper_link_offset_y = 0.040
    upper_link_offset_z = 0.018

    lid_width = 0.500
    lid_depth = 0.342
    lid_height = 0.026
    lid_rear_offset = 0.006

    body = model.part("body")
    body.inertial = Inertial.from_geometry(
        Box((body_width, body_depth, total_body_height)),
        mass=4.8,
        origin=Origin(xyz=(0.0, 0.0, total_body_height * 0.5)),
    )

    top_frame_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            rounded_rect_profile(body_width, body_depth, 0.022, corner_segments=8),
            [rounded_rect_profile(platen_outer_width, platen_outer_depth, 0.010, corner_segments=8)],
            frame_height,
            center=True,
            cap=True,
            closed=True,
        ),
        "scanner_top_frame",
    )

    body.visual(
        Box((body_width - 0.004, body_depth - 0.004, base_height)),
        origin=Origin(xyz=(0.0, 0.0, foot_height + base_height * 0.5)),
        material=body_dark,
        name="lower_base",
    )
    body.visual(
        Box((0.092, body_depth - 0.030, shoulder_height)),
        origin=Origin(xyz=(-(platen_outer_width * 0.5 + 0.046), 0.0, foot_height + base_height + shoulder_height * 0.5)),
        material=body_mid,
        name="left_shoulder",
    )
    body.visual(
        Box((0.092, body_depth - 0.030, shoulder_height)),
        origin=Origin(xyz=((platen_outer_width * 0.5 + 0.046), 0.0, foot_height + base_height + shoulder_height * 0.5)),
        material=body_mid,
        name="right_shoulder",
    )
    body.visual(
        Box((platen_outer_width, 0.048, shoulder_height)),
        origin=Origin(xyz=(0.0, 0.156, foot_height + base_height + shoulder_height * 0.5)),
        material=body_mid,
        name="front_shoulder",
    )
    body.visual(
        Box((platen_outer_width + 0.040, 0.034, shoulder_height)),
        origin=Origin(xyz=(0.0, -0.163, foot_height + base_height + shoulder_height * 0.5)),
        material=body_mid,
        name="rear_shoulder",
    )
    body.visual(
        top_frame_mesh,
        origin=Origin(xyz=(0.0, 0.0, foot_height + base_height + shoulder_height + frame_height * 0.5)),
        material=body_mid,
        name="top_frame",
    )
    body.visual(
        Box((platen_glass_width, platen_glass_depth, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.0605)),
        material=platen_black,
        name="scan_well",
    )
    body.visual(
        Box((platen_glass_width, platen_glass_depth, 0.003)),
        origin=Origin(xyz=(0.0, 0.0, 0.0690)),
        material=glass,
        name="platen_glass",
    )
    body.visual(
        Box((0.120, 0.040, 0.002)),
        origin=Origin(xyz=(0.134, 0.114, total_body_height + 0.001)),
        material=panel_blue,
        name="control_panel",
    )
    for index, x in enumerate((0.095, 0.126, 0.157)):
        body.visual(
            Box((0.018, 0.012, 0.003)),
            origin=Origin(xyz=(x, 0.114, total_body_height + 0.0035)),
            material=body_dark,
            name=f"button_{index}",
        )

    body.visual(
        Box((0.220, 0.026, 0.022)),
        origin=Origin(xyz=(0.0, -0.176, 0.067)),
        material=hinge_dark,
        name="rear_hinge_rail",
    )
    for side_name, side_sign in (("left", -1.0), ("right", 1.0)):
        side_x = side_sign * 0.190
        body.visual(
            Box((0.072, 0.034, 0.030)),
            origin=Origin(xyz=(side_x, -0.194, 0.056)),
            material=hinge_dark,
            name=f"{side_name}_hinge_bracket",
        )
        for sleeve_name, center_x in (("outer", side_sign * 0.215), ("inner", side_sign * 0.165)):
            body.visual(
                Cylinder(radius=0.009, length=0.018),
                origin=Origin(
                    xyz=(center_x, hinge_axis_y, hinge_axis_z),
                    rpy=(0.0, math.pi * 0.5, 0.0),
                ),
                material=hinge_pin,
                name=f"{side_name}_lower_{sleeve_name}_sleeve",
            )

    for side_sign, side_name in ((-1.0, "rear_left"), (1.0, "rear_right"), (-1.0, "front_left"), (1.0, "front_right")):
        foot_x = side_sign * 0.210
        foot_y = -0.140 if "rear" in side_name else 0.140
        body.visual(
            Cylinder(radius=0.012, length=foot_height),
            origin=Origin(xyz=(foot_x, foot_y, foot_height * 0.5)),
            material=rubber,
            name=f"{side_name}_foot",
        )

    riser = model.part("riser_assembly")
    riser.inertial = Inertial.from_geometry(
        Box((0.460, 0.070, 0.050)),
        mass=0.55,
        origin=Origin(xyz=(0.0, 0.020, 0.012)),
    )

    riser.visual(
        Box((0.420, 0.014, 0.010)),
        origin=Origin(xyz=(0.0, 0.020, 0.009)),
        material=hinge_dark,
        name="mid_crossbar",
    )

    arm_angle = math.atan2(upper_link_offset_z, upper_link_offset_y)
    arm_length = math.hypot(upper_link_offset_y, upper_link_offset_z) + 0.012
    for side_name, side_sign in (("left", -1.0), ("right", 1.0)):
        side_x = side_sign * 0.190
        riser.visual(
            Box((0.010, arm_length, 0.010)),
            origin=Origin(
                xyz=(side_x, upper_link_offset_y * 0.5, upper_link_offset_z * 0.5),
                rpy=(arm_angle, 0.0, 0.0),
            ),
            material=hinge_dark,
            name=f"{side_name}_riser_arm",
        )
        riser.visual(
            Cylinder(radius=0.0088, length=0.032),
            origin=Origin(
                xyz=(side_x, 0.0, 0.0),
                rpy=(0.0, math.pi * 0.5, 0.0),
            ),
            material=hinge_pin,
            name=f"{side_name}_lower_knuckle",
        )
        riser.visual(
            Cylinder(radius=0.0088, length=0.032),
            origin=Origin(
                xyz=(side_x, upper_link_offset_y, upper_link_offset_z),
                rpy=(0.0, math.pi * 0.5, 0.0),
            ),
            material=hinge_pin,
            name=f"{side_name}_upper_knuckle",
        )

    lid = model.part("lid")
    lid.inertial = Inertial.from_geometry(
        Box((lid_width, lid_depth, lid_height)),
        mass=1.15,
        origin=Origin(xyz=(0.0, 0.178, -0.001)),
    )

    lid.visual(
        _rounded_panel_mesh(
            "scanner_lid_shell",
            width=lid_width,
            depth=lid_depth,
            height=lid_height,
            radius=0.020,
        ),
        origin=Origin(xyz=(0.0, lid_rear_offset + lid_depth * 0.5, -0.014)),
        material=lid_light,
        name="outer_shell",
    )
    lid.visual(
        Box((0.458, 0.308, 0.006)),
        origin=Origin(xyz=(0.0, 0.176, -0.0125)),
        material=lid_pad,
        name="inner_pad",
    )
    lid.visual(
        Box((0.430, 0.014, 0.016)),
        origin=Origin(xyz=(0.0, 0.012, -0.002)),
        material=lid_light,
        name="rear_spine",
    )
    lid.visual(
        Box((0.140, 0.012, 0.010)),
        origin=Origin(xyz=(0.0, 0.336, 0.015)),
        material=lid_light,
        name="front_handle",
    )
    for side_name, side_sign in (("left", -1.0), ("right", 1.0)):
        lid.visual(
            Box((0.064, 0.014, 0.010)),
            origin=Origin(xyz=(side_sign * 0.190, 0.014, -0.002)),
            material=lid_light,
            name=f"{side_name}_hinge_leaf",
        )
        for sleeve_name, center_x in (("outer", side_sign * 0.215), ("inner", side_sign * 0.165)):
            lid.visual(
                Cylinder(radius=0.0086, length=0.018),
                origin=Origin(
                    xyz=(center_x, 0.0, 0.0),
                    rpy=(0.0, math.pi * 0.5, 0.0),
                ),
                material=hinge_pin,
                name=f"{side_name}_lid_{sleeve_name}_sleeve",
            )

    model.articulation(
        "body_to_riser",
        ArticulationType.REVOLUTE,
        parent=body,
        child=riser,
        origin=Origin(xyz=(0.0, hinge_axis_y, hinge_axis_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.2,
            lower=0.0,
            upper=0.78,
        ),
    )
    model.articulation(
        "riser_to_lid",
        ArticulationType.REVOLUTE,
        parent=riser,
        child=lid,
        origin=Origin(xyz=(0.0, upper_link_offset_y, upper_link_offset_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.5,
            lower=-0.75,
            upper=1.35,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    riser = object_model.get_part("riser_assembly")
    lid = object_model.get_part("lid")
    lift_joint = object_model.get_articulation("body_to_riser")
    lid_joint = object_model.get_articulation("riser_to_lid")

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

    with ctx.pose({lift_joint: 0.0, lid_joint: 0.0}):
        ctx.expect_contact(
            riser,
            body,
            elem_a="left_lower_knuckle",
            elem_b="left_lower_outer_sleeve",
            contact_tol=1e-4,
            name="left lower hinge knuckle is mounted to the body pedestal",
        )
        ctx.expect_contact(
            riser,
            lid,
            elem_a="left_upper_knuckle",
            elem_b="left_lid_outer_sleeve",
            contact_tol=1e-4,
            name="left upper hinge knuckle is mounted to the lid",
        )
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="inner_pad",
            negative_elem="top_frame",
            max_gap=0.010,
            max_penetration=0.0,
            name="closed lid sits just above the platen frame",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            elem_a="outer_shell",
            elem_b="top_frame",
            min_overlap=0.300,
            name="lid covers the full scanner bed footprint",
        )

    closed_lid_pos = ctx.part_world_position(lid)
    with ctx.pose({lift_joint: 0.65, lid_joint: -0.60}):
        raised_lid_pos = ctx.part_world_position(lid)
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="inner_pad",
            negative_elem="top_frame",
            min_gap=0.020,
            name="riser arm creates clearance for a thick open book",
        )
        ctx.check(
            "secondary riser raises the lid hinge line",
            closed_lid_pos is not None
            and raised_lid_pos is not None
            and raised_lid_pos[2] > closed_lid_pos[2] + 0.015,
            details=f"closed={closed_lid_pos}, raised={raised_lid_pos}",
        )

    closed_front_aabb = ctx.part_element_world_aabb(lid, elem="front_handle")
    with ctx.pose({lift_joint: 0.0, lid_joint: 1.10}):
        open_front_aabb = ctx.part_element_world_aabb(lid, elem="front_handle")
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="front_handle",
            negative_elem="top_frame",
            min_gap=0.180,
            name="opened lid front edge swings well above the body",
        )
        ctx.check(
            "main lid hinge opens upward",
            closed_front_aabb is not None
            and open_front_aabb is not None
            and open_front_aabb[1][2] > closed_front_aabb[1][2] + 0.180,
            details=f"closed={closed_front_aabb}, open={open_front_aabb}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
