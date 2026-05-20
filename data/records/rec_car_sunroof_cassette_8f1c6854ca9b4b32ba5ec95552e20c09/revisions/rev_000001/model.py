from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import radians

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


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _rounded_ring_mesh(
    name: str,
    *,
    outer_size: tuple[float, float],
    outer_radius: float,
    inner_size: tuple[float, float],
    inner_radius: float,
    height: float,
):
    return _mesh(
        name,
        ExtrudeWithHolesGeometry(
            rounded_rect_profile(outer_size[0], outer_size[1], outer_radius),
            [rounded_rect_profile(inner_size[0], inner_size[1], inner_radius)],
            height,
            center=True,
        ),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tilt_slide_moonroof_cassette")

    satin_aluminum = model.material("satin_aluminum", rgba=(0.72, 0.74, 0.76, 1.0))
    dark_aluminum = model.material("dark_aluminum", rgba=(0.34, 0.36, 0.39, 1.0))
    anodized_rail = model.material("anodized_rail", rgba=(0.24, 0.25, 0.27, 1.0))
    glass_tint = model.material("glass_tint", rgba=(0.18, 0.26, 0.32, 0.52))
    ceramic_black = model.material("ceramic_black", rgba=(0.07, 0.07, 0.08, 1.0))

    outer_length = 0.94
    outer_width = 0.68
    roof_open_length = 0.82
    roof_open_width = 0.56
    top_flange_thickness = 0.012

    guide_cover_length = 0.78
    guide_cover_width = 0.020
    guide_cover_height = 0.006
    guide_cover_y = 0.285

    rail_floor_length = 0.82
    rail_floor_width = 0.055
    rail_floor_height = 0.006
    rail_floor_y = 0.245
    rail_floor_top_z = -0.025

    panel_length = 0.80
    panel_width = 0.54
    panel_thickness = 0.005
    border_inner_length = 0.72
    border_inner_width = 0.46

    slide_travel = 0.30
    tilt_open_angle = radians(5.5)

    frame = model.part("frame")
    frame.inertial = Inertial.from_geometry(
        Box((outer_length, outer_width, 0.08)),
        mass=12.5,
        origin=Origin(xyz=(0.0, 0.0, -0.04)),
    )
    frame.visual(
        _rounded_ring_mesh(
            "cassette_top_ring",
            outer_size=(outer_length, outer_width),
            outer_radius=0.055,
            inner_size=(roof_open_length, roof_open_width),
            inner_radius=0.040,
            height=top_flange_thickness,
        ),
        origin=Origin(xyz=(0.0, 0.0, -top_flange_thickness * 0.5)),
        material=satin_aluminum,
        name="top_ring",
    )
    frame.visual(
        Box((guide_cover_length, guide_cover_width, guide_cover_height)),
        origin=Origin(xyz=(0.0, guide_cover_y, -guide_cover_height * 0.5)),
        material=anodized_rail,
        name="left_guide_cover",
    )
    frame.visual(
        Box((guide_cover_length, guide_cover_width, guide_cover_height)),
        origin=Origin(xyz=(0.0, -guide_cover_y, -guide_cover_height * 0.5)),
        material=anodized_rail,
        name="right_guide_cover",
    )
    frame.visual(
        Box((rail_floor_length, rail_floor_width, rail_floor_height)),
        origin=Origin(
            xyz=(0.0, rail_floor_y, rail_floor_top_z - rail_floor_height * 0.5)
        ),
        material=anodized_rail,
        name="left_rail_floor",
    )
    frame.visual(
        Box((rail_floor_length, rail_floor_width, rail_floor_height)),
        origin=Origin(
            xyz=(0.0, -rail_floor_y, rail_floor_top_z - rail_floor_height * 0.5)
        ),
        material=anodized_rail,
        name="right_rail_floor",
    )
    frame.visual(
        Box((rail_floor_length, 0.010, 0.040)),
        origin=Origin(xyz=(0.0, 0.2775, -0.026)),
        material=dark_aluminum,
        name="left_outer_channel_wall",
    )
    frame.visual(
        Box((rail_floor_length, 0.010, 0.040)),
        origin=Origin(xyz=(0.0, -0.2775, -0.026)),
        material=dark_aluminum,
        name="right_outer_channel_wall",
    )
    frame.visual(
        Box((0.10, 0.570, 0.026)),
        origin=Origin(xyz=(-0.36, 0.0, -0.041)),
        material=dark_aluminum,
        name="front_drain_trough",
    )
    frame.visual(
        Box((0.14, 0.620, 0.026)),
        origin=Origin(xyz=(0.35, 0.0, -0.041)),
        material=dark_aluminum,
        name="rear_crossmember",
    )

    slide_carriage = model.part("slide_carriage")
    slide_carriage.inertial = Inertial.from_geometry(
        Box((0.48, 0.50, 0.03)),
        mass=1.6,
        origin=Origin(xyz=(0.24, 0.0, -0.015)),
    )
    slide_carriage.visual(
        Box((0.048, 0.400, 0.012)),
        origin=Origin(xyz=(0.030, 0.0, -0.014)),
        material=dark_aluminum,
        name="front_hinge_bridge",
    )
    slide_carriage.visual(
        Box((0.460, 0.028, 0.012)),
        origin=Origin(xyz=(0.230, rail_floor_y, -0.019)),
        material=anodized_rail,
        name="left_shoe",
    )
    slide_carriage.visual(
        Box((0.460, 0.028, 0.012)),
        origin=Origin(xyz=(0.230, -rail_floor_y, -0.019)),
        material=anodized_rail,
        name="right_shoe",
    )
    slide_carriage.visual(
        Box((0.048, 0.040, 0.020)),
        origin=Origin(xyz=(0.030, 0.220, -0.015)),
        material=dark_aluminum,
        name="left_hinge_carrier",
    )
    slide_carriage.visual(
        Box((0.048, 0.040, 0.020)),
        origin=Origin(xyz=(0.030, -0.220, -0.015)),
        material=dark_aluminum,
        name="right_hinge_carrier",
    )
    slide_carriage.visual(
        Cylinder(radius=0.006, length=0.412),
        origin=Origin(xyz=(0.0, 0.0, -0.011), rpy=(1.57079632679, 0.0, 0.0)),
        material=anodized_rail,
        name="hinge_rod",
    )

    glass_panel = model.part("glass_panel")
    glass_panel.inertial = Inertial.from_geometry(
        Box((panel_length, panel_width, 0.018)),
        mass=8.0,
        origin=Origin(xyz=(panel_length * 0.5, 0.0, 0.004)),
    )
    glass_panel.visual(
        _mesh(
            "moonroof_glass_shell",
            ExtrudeGeometry(
                rounded_rect_profile(panel_length, panel_width, 0.042),
                panel_thickness,
                center=True,
            ),
        ),
        origin=Origin(xyz=(panel_length * 0.5, 0.0, panel_thickness * 0.5)),
        material=glass_tint,
        name="glass_shell",
    )
    glass_panel.visual(
        _rounded_ring_mesh(
            "moonroof_frit_border",
            outer_size=(panel_length, panel_width),
            outer_radius=0.042,
            inner_size=(border_inner_length, border_inner_width),
            inner_radius=0.026,
            height=0.0015,
        ),
        origin=Origin(xyz=(panel_length * 0.5, 0.0, 0.0012)),
        material=ceramic_black,
        name="frit_border",
    )

    model.articulation(
        "frame_to_slide_carriage",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=slide_carriage,
        origin=Origin(xyz=(-panel_length * 0.5, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=150.0,
            velocity=0.35,
            lower=0.0,
            upper=slide_travel,
        ),
    )
    model.articulation(
        "carriage_to_glass_panel",
        ArticulationType.REVOLUTE,
        parent=slide_carriage,
        child=glass_panel,
        origin=Origin(xyz=(0.0, 0.0, -panel_thickness)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=1.2,
            lower=0.0,
            upper=tilt_open_angle,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    slide_carriage = object_model.get_part("slide_carriage")
    glass_panel = object_model.get_part("glass_panel")
    slide_joint = object_model.get_articulation("frame_to_slide_carriage")
    tilt_joint = object_model.get_articulation("carriage_to_glass_panel")

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

    ctx.expect_contact(
        slide_carriage,
        frame,
        elem_a="left_shoe",
        elem_b="left_rail_floor",
        contact_tol=1e-5,
        name="left carriage shoe bears on left guide rail",
    )
    ctx.expect_contact(
        slide_carriage,
        frame,
        elem_a="right_shoe",
        elem_b="right_rail_floor",
        contact_tol=1e-5,
        name="right carriage shoe bears on right guide rail",
    )
    ctx.expect_within(
        slide_carriage,
        frame,
        axes="y",
        inner_elem="left_shoe",
        outer_elem="left_rail_floor",
        margin=0.0,
        name="left shoe stays laterally captured by left rail",
    )
    ctx.expect_within(
        slide_carriage,
        frame,
        axes="y",
        inner_elem="right_shoe",
        outer_elem="right_rail_floor",
        margin=0.0,
        name="right shoe stays laterally captured by right rail",
    )

    panel_aabb = ctx.part_element_world_aabb(glass_panel, elem="glass_shell")
    guide_cover_aabb = ctx.part_element_world_aabb(frame, elem="left_guide_cover")
    flush_ok = (
        panel_aabb is not None
        and guide_cover_aabb is not None
        and abs(panel_aabb[1][2] - guide_cover_aabb[1][2]) <= 0.0015
    )
    ctx.check(
        "panel sits flush with roof-line guide covers when closed",
        flush_ok,
        details=f"panel_aabb={panel_aabb}, guide_cover_aabb={guide_cover_aabb}",
    )

    closed_panel_pos = ctx.part_world_position(glass_panel)
    closed_panel_aabb = ctx.part_element_world_aabb(glass_panel, elem="glass_shell")
    with ctx.pose({slide_joint: 0.0, tilt_joint: tilt_joint.motion_limits.upper}):
        tilted_panel_pos = ctx.part_world_position(glass_panel)
        tilted_panel_aabb = ctx.part_element_world_aabb(glass_panel, elem="glass_shell")
        ctx.check(
            "tilt articulation raises the rear edge upward",
            closed_panel_aabb is not None
            and tilted_panel_aabb is not None
            and tilted_panel_aabb[1][2] > closed_panel_aabb[1][2] + 0.035,
            details=f"closed={closed_panel_aabb}, tilted={tilted_panel_aabb}",
        )
        ctx.check(
            "tilt articulation pivots about the front hinge line",
            closed_panel_pos is not None
            and tilted_panel_pos is not None
            and abs(tilted_panel_pos[0] - closed_panel_pos[0]) <= 1e-6
            and abs(tilted_panel_pos[1] - closed_panel_pos[1]) <= 1e-6
            and abs(tilted_panel_pos[2] - closed_panel_pos[2]) <= 1e-6,
            details=f"closed={closed_panel_pos}, tilted={tilted_panel_pos}",
        )

    with ctx.pose({tilt_joint: 0.0, slide_joint: slide_joint.motion_limits.upper}):
        extended_panel_pos = ctx.part_world_position(glass_panel)
        ctx.check(
            "panel slides rearward on the override rails",
            closed_panel_pos is not None
            and extended_panel_pos is not None
            and extended_panel_pos[0] > closed_panel_pos[0] + 0.25,
            details=f"closed={closed_panel_pos}, extended={extended_panel_pos}",
        )
        ctx.expect_overlap(
            slide_carriage,
            frame,
            axes="x",
            elem_a="left_shoe",
            elem_b="left_rail_floor",
            min_overlap=0.40,
            name="left shoe retains substantial rail engagement at full slide",
        )
        ctx.expect_overlap(
            slide_carriage,
            frame,
            axes="x",
            elem_a="right_shoe",
            elem_b="right_rail_floor",
            min_overlap=0.40,
            name="right shoe retains substantial rail engagement at full slide",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
