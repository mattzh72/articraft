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


def _rounded_shell_mesh(
    outer_size: tuple[float, float],
    inner_size: tuple[float, float],
    height: float,
    *,
    outer_radius: float,
    inner_radius: float,
):
    return ExtrudeWithHolesGeometry(
        rounded_rect_profile(
            outer_size[0],
            outer_size[1],
            outer_radius,
            corner_segments=10,
        ),
        [
            rounded_rect_profile(
                inner_size[0],
                inner_size[1],
                inner_radius,
                corner_segments=10,
            )
        ],
        height,
        center=True,
        cap=True,
    )


def _rounded_plate_mesh(
    size: tuple[float, float],
    thickness: float,
    *,
    radius: float,
):
    return ExtrudeGeometry(
        rounded_rect_profile(size[0], size[1], radius, corner_segments=10),
        thickness,
        cap=True,
        center=True,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hospital_specimen_chest_freezer")

    cabinet_white = model.material("cabinet_white", rgba=(0.93, 0.95, 0.97, 1.0))
    liner_white = model.material("liner_white", rgba=(0.90, 0.93, 0.95, 1.0))
    gasket_grey = model.material("gasket_grey", rgba=(0.69, 0.71, 0.73, 1.0))
    steel = model.material("steel", rgba=(0.60, 0.63, 0.66, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.21, 0.23, 0.26, 1.0))
    display_black = model.material("display_black", rgba=(0.08, 0.10, 0.12, 1.0))
    tray_white = model.material("tray_white", rgba=(0.94, 0.96, 0.98, 1.0))
    tray_blue = model.material("tray_blue", rgba=(0.59, 0.69, 0.80, 1.0))

    body_length = 1.20
    body_depth = 0.62
    body_height = 0.86
    wall_thickness = 0.045
    base_thickness = 0.025
    lid_thickness = 0.070
    outer_corner_radius = 0.060
    inner_corner_radius = 0.018
    lid_overhang = 0.012
    hinge_radius = 0.012
    hinge_axis_y = -(body_depth * 0.5 + hinge_radius)
    hinge_axis_z = body_height + hinge_radius

    interior_length = body_length - 2.0 * wall_thickness
    interior_depth = body_depth - 2.0 * wall_thickness

    body = model.part("body")
    body.visual(
        mesh_from_geometry(
            _rounded_shell_mesh(
                (body_length, body_depth),
                (interior_length, interior_depth),
                body_height - base_thickness,
                outer_radius=outer_corner_radius,
                inner_radius=inner_corner_radius,
            ),
            "freezer_body_shell",
        ),
        origin=Origin(xyz=(0.0, 0.0, base_thickness + 0.5 * (body_height - base_thickness))),
        material=cabinet_white,
        name="body_shell",
    )
    body.visual(
        mesh_from_geometry(
            _rounded_plate_mesh(
                (body_length, body_depth),
                base_thickness,
                radius=outer_corner_radius,
            ),
            "freezer_body_floor",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.5 * base_thickness)),
        material=liner_white,
        name="floor_plate",
    )
    body.visual(
        Box((body_length - 0.08, body_depth - 0.08, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=dark_trim,
        name="base_plinth",
    )
    body.visual(
        Box((interior_length + 0.010, 0.024, 0.014)),
        origin=Origin(
            xyz=(0.0, interior_depth * 0.5 - 0.018, 0.760),
        ),
        material=steel,
        name="front_rail",
    )
    body.visual(
        Box((interior_length + 0.010, 0.024, 0.014)),
        origin=Origin(
            xyz=(0.0, -(interior_depth * 0.5 - 0.018), 0.760),
        ),
        material=steel,
        name="rear_rail",
    )
    body.visual(
        Box((0.122, 0.026, 0.068)),
        origin=Origin(xyz=(0.365, body_depth * 0.5 + 0.009, 0.705)),
        material=dark_trim,
        name="controller_housing",
    )
    body.visual(
        Box((0.080, 0.006, 0.036)),
        origin=Origin(xyz=(0.365, body_depth * 0.5 + 0.021, 0.715)),
        material=display_black,
        name="controller_display",
    )

    left_hinge_xs = (-0.391, -0.329)
    right_hinge_xs = (0.329, 0.391)
    for index, x_pos in enumerate(left_hinge_xs + right_hinge_xs):
        body.visual(
            Cylinder(radius=hinge_radius, length=0.028),
            origin=Origin(
                xyz=(x_pos, hinge_axis_y, hinge_axis_z),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=steel,
            name=f"body_knuckle_{index}",
        )
        body.visual(
            Box((0.026, 0.028, 0.020)),
            origin=Origin(xyz=(x_pos, -0.318, 0.850)),
            material=steel,
            name=f"body_hinge_bracket_{index}",
        )

    body.inertial = Inertial.from_geometry(
        Box((body_length, body_depth, body_height)),
        mass=82.0,
        origin=Origin(xyz=(0.0, 0.0, 0.5 * body_height)),
    )

    lid = model.part("lid")
    lid_length = body_length + 0.020
    lid_depth = body_depth + 0.024
    lid.visual(
        mesh_from_geometry(
            _rounded_plate_mesh(
                (lid_length, lid_depth),
                lid_thickness,
                radius=0.055,
            ),
            "freezer_lid_panel",
        ),
        origin=Origin(
            xyz=(0.0, 0.5 * lid_depth - lid_overhang, -hinge_radius + 0.5 * lid_thickness),
        ),
        material=liner_white,
        name="lid_panel",
    )
    lid.visual(
        Box((0.230, 0.038, 0.028)),
        origin=Origin(
            xyz=(0.0, lid_depth - lid_overhang - 0.060, -hinge_radius + lid_thickness + 0.014),
        ),
        material=dark_trim,
        name="pull_handle",
    )
    lid.visual(
        Box((0.090, 0.026, 0.012)),
        origin=Origin(
            xyz=(0.0, lid_depth - lid_overhang - 0.090, -hinge_radius + lid_thickness + 0.006),
        ),
        material=gasket_grey,
        name="handle_mount",
    )

    for name, x_pos in (("left_lid_knuckle", -0.360), ("right_lid_knuckle", 0.360)):
        lid.visual(
            Cylinder(radius=hinge_radius, length=0.028),
            origin=Origin(
                xyz=(x_pos, 0.0, 0.0),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=steel,
            name=name,
        )
        lid.visual(
            Box((0.028, 0.028, 0.024)),
            origin=Origin(xyz=(x_pos, 0.010, 0.004)),
            material=steel,
            name=f"{name}_strap",
        )

    lid.inertial = Inertial.from_geometry(
        Box((lid_length, lid_depth, lid_thickness + 0.040)),
        mass=14.0,
        origin=Origin(
            xyz=(0.0, 0.5 * lid_depth - lid_overhang, 0.010),
        ),
    )

    tray = model.part("divider_tray")
    tray_length = 0.340
    tray_width = 0.420
    tray_depth = 0.100
    tray_base = 0.005
    tray.visual(
        mesh_from_geometry(
            _rounded_shell_mesh(
                (tray_length, tray_width),
                (tray_length - 0.040, tray_width - 0.040),
                tray_depth - tray_base,
                outer_radius=0.024,
                inner_radius=0.016,
            ),
            "divider_tray_shell",
        ),
        origin=Origin(xyz=(0.0, 0.0, tray_base + 0.5 * (tray_depth - tray_base))),
        material=tray_white,
        name="tray_shell",
    )
    tray.visual(
        mesh_from_geometry(
            _rounded_plate_mesh((tray_length, tray_width), tray_base, radius=0.024),
            "divider_tray_floor",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.5 * tray_base)),
        material=tray_blue,
        name="tray_floor",
    )
    tray.visual(
        Box((0.008, tray_width - 0.060, tray_depth + 0.020)),
        origin=Origin(xyz=(0.040, 0.0, 0.5 * (tray_depth + 0.020))),
        material=tray_blue,
        name="partition_panel",
    )
    tray.visual(
        Box((tray_length, 0.060, 0.008)),
        origin=Origin(xyz=(0.0, tray_width * 0.5 + 0.013, 0.096)),
        material=steel,
        name="front_glide",
    )
    tray.visual(
        Box((tray_length, 0.060, 0.008)),
        origin=Origin(xyz=(0.0, -(tray_width * 0.5 + 0.013), 0.096)),
        material=steel,
        name="rear_glide",
    )
    tray.visual(
        Box((0.004, 0.110, 0.040)),
        origin=Origin(xyz=(0.046, 0.0, 0.076)),
        material=tray_blue,
        name="specimen_bin_label",
    )
    tray.inertial = Inertial.from_geometry(
        Box((tray_length, tray_width + 0.060, tray_depth + 0.040)),
        mass=3.2,
        origin=Origin(xyz=(0.0, 0.0, 0.070)),
    )

    lid_hinge = model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, hinge_axis_y, hinge_axis_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.5,
            lower=0.0,
            upper=1.35,
        ),
    )

    tray_slide = model.articulation(
        "divider_tray_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=tray,
        origin=Origin(xyz=(0.0, 0.0, 0.675)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=0.40,
            lower=-0.36,
            upper=0.36,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    tray = object_model.get_part("divider_tray")
    lid_hinge = object_model.get_articulation("lid_hinge")
    tray_slide = object_model.get_articulation("divider_tray_slide")

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

    ctx.check(
        "lid hinge is a rear-edge x-axis revolute joint",
        lid_hinge.axis == (1.0, 0.0, 0.0)
        and lid_hinge.motion_limits is not None
        and lid_hinge.motion_limits.lower == 0.0
        and lid_hinge.motion_limits.upper is not None
        and lid_hinge.motion_limits.upper > 1.2,
        details=f"axis={lid_hinge.axis}, limits={lid_hinge.motion_limits}",
    )
    ctx.check(
        "divider tray uses full-length x-axis slide travel",
        tray_slide.axis == (1.0, 0.0, 0.0)
        and tray_slide.motion_limits is not None
        and tray_slide.motion_limits.lower is not None
        and tray_slide.motion_limits.upper is not None
        and tray_slide.motion_limits.upper - tray_slide.motion_limits.lower >= 0.70,
        details=f"axis={tray_slide.axis}, limits={tray_slide.motion_limits}",
    )

    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_contact(
            lid,
            body,
            name="closed lid seats on cabinet rim",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            min_overlap=0.45,
            name="closed lid covers freezer opening footprint",
        )
        ctx.expect_contact(
            tray,
            body,
            elem_a="front_glide",
            elem_b="front_rail",
            name="tray front glide rests on front rail",
        )
        ctx.expect_contact(
            tray,
            body,
            elem_a="rear_glide",
            elem_b="rear_rail",
            name="tray rear glide rests on rear rail",
        )
        ctx.expect_within(
            tray,
            body,
            axes="xyz",
            margin=0.001,
            name="divider tray stays inside cabinet in centered pose",
        )

    closed_panel_aabb = ctx.part_element_world_aabb(lid, elem="lid_panel")
    tray_rest_pos = ctx.part_world_position(tray)

    with ctx.pose({lid_hinge: lid_hinge.motion_limits.upper}):
        open_panel_aabb = ctx.part_element_world_aabb(lid, elem="lid_panel")

    ctx.check(
        "lid front rises noticeably when opened",
        closed_panel_aabb is not None
        and open_panel_aabb is not None
        and open_panel_aabb[1][2] > closed_panel_aabb[1][2] + 0.25,
        details=f"closed={closed_panel_aabb}, open={open_panel_aabb}",
    )

    with ctx.pose({tray_slide: tray_slide.motion_limits.upper}):
        tray_extended_pos = ctx.part_world_position(tray)
        ctx.expect_contact(
            tray,
            body,
            elem_a="front_glide",
            elem_b="front_rail",
            name="tray front glide remains supported at full extension",
        )
        ctx.expect_contact(
            tray,
            body,
            elem_a="rear_glide",
            elem_b="rear_rail",
            name="tray rear glide remains supported at full extension",
        )
        ctx.expect_within(
            tray,
            body,
            axes="yz",
            margin=0.001,
            name="divider tray stays centered between side walls at full extension",
        )

    ctx.check(
        "divider tray moves toward positive x when extended",
        tray_rest_pos is not None
        and tray_extended_pos is not None
        and tray_extended_pos[0] > tray_rest_pos[0] + 0.30,
        details=f"rest={tray_rest_pos}, extended={tray_extended_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
