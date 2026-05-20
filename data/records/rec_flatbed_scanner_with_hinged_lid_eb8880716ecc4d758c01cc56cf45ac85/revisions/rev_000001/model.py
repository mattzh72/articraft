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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)


def _rounded_rect_section(
    width: float,
    depth: float,
    radius: float,
    z: float,
    *,
    center_x: float = 0.0,
    center_y: float = 0.0,
    corner_segments: int = 8,
) -> list[tuple[float, float, float]]:
    return [
        (x + center_x, y + center_y, z)
        for x, y in rounded_rect_profile(width, depth, radius, corner_segments=corner_segments)
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="a3_flatbed_scanner")

    body_plastic = model.material("body_plastic", rgba=(0.20, 0.22, 0.24, 1.0))
    lid_plastic = model.material("lid_plastic", rgba=(0.27, 0.29, 0.31, 1.0))
    bezel_black = model.material("bezel_black", rgba=(0.09, 0.10, 0.11, 1.0))
    smoked_glass = model.material("smoked_glass", rgba=(0.50, 0.63, 0.68, 0.35))
    trim_gray = model.material("trim_gray", rgba=(0.50, 0.52, 0.55, 1.0))
    guide_gray = model.material("guide_gray", rgba=(0.72, 0.74, 0.76, 1.0))

    body = model.part("body")

    body_shell_geom = section_loft(
        [
            _rounded_rect_section(0.620, 0.430, 0.028, 0.000),
            _rounded_rect_section(0.612, 0.424, 0.028, 0.040),
            _rounded_rect_section(0.606, 0.416, 0.026, 0.072),
        ]
    )
    body.visual(
        mesh_from_geometry(body_shell_geom, "scanner_body_shell"),
        material=body_plastic,
        name="body_shell",
    )
    body.visual(
        Box((0.066, 0.336, 0.006)),
        origin=Origin(xyz=(-0.277, -0.005, 0.075)),
        material=bezel_black,
        name="left_bezel",
    )
    body.visual(
        Box((0.066, 0.336, 0.006)),
        origin=Origin(xyz=(0.277, -0.005, 0.075)),
        material=bezel_black,
        name="right_bezel",
    )
    body.visual(
        Box((0.500, 0.042, 0.006)),
        origin=Origin(xyz=(0.000, -0.194, 0.075)),
        material=bezel_black,
        name="front_bezel",
    )
    body.visual(
        Box((0.500, 0.052, 0.006)),
        origin=Origin(xyz=(0.000, 0.189, 0.075)),
        material=bezel_black,
        name="rear_bezel",
    )
    body.visual(
        Box((0.484, 0.332, 0.006)),
        origin=Origin(xyz=(0.000, -0.005, 0.069)),
        material=bezel_black,
        name="scan_bed",
    )
    body.visual(
        Box((0.488, 0.336, 0.002)),
        origin=Origin(xyz=(0.000, -0.005, 0.073)),
        material=smoked_glass,
        name="platen_glass",
    )
    body.visual(
        Box((0.140, 0.026, 0.003)),
        origin=Origin(xyz=(0.188, -0.194, 0.0735)),
        material=trim_gray,
        name="control_strip",
    )
    body.visual(
        Cylinder(radius=0.007, length=0.003),
        origin=Origin(xyz=(0.130, -0.194, 0.0735)),
        material=trim_gray,
        name="power_button",
    )
    body.visual(
        Box((0.036, 0.010, 0.0015)),
        origin=Origin(xyz=(0.205, -0.194, 0.07425)),
        material=bezel_black,
        name="status_window",
    )

    for name, x_center in (
        ("left_outer_hinge_tab", -0.255),
        ("left_inner_hinge_tab", -0.125),
        ("right_inner_hinge_tab", 0.125),
        ("right_outer_hinge_tab", 0.255),
    ):
        body.visual(
            Box((0.050, 0.022, 0.008)),
            origin=Origin(xyz=(x_center, 0.206, 0.076)),
            material=trim_gray,
            name=name,
        )

    for name, x_center in (
        ("left_outer_body_knuckle", -0.255),
        ("left_inner_body_knuckle", -0.125),
        ("right_inner_body_knuckle", 0.125),
        ("right_outer_body_knuckle", 0.255),
    ):
        body.visual(
            Cylinder(radius=0.013, length=0.040),
            origin=Origin(xyz=(x_center, 0.206, 0.087), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=trim_gray,
            name=name,
        )

    body.inertial = Inertial.from_geometry(
        Box((0.620, 0.430, 0.090)),
        mass=8.2,
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
    )

    lid = model.part("lid")

    lid_shell_geom = section_loft(
        [
            _rounded_rect_section(0.612, 0.406, 0.020, -0.004, center_y=-0.223),
            _rounded_rect_section(0.608, 0.402, 0.020, 0.002, center_y=-0.223),
            _rounded_rect_section(0.604, 0.398, 0.018, 0.008, center_y=-0.223),
        ]
    )
    lid.visual(
        mesh_from_geometry(lid_shell_geom, "scanner_lid_shell"),
        material=lid_plastic,
        name="lid_outer_shell",
    )
    lid.visual(
        Box((0.560, 0.340, 0.002)),
        origin=Origin(xyz=(0.000, -0.225, -0.005)),
        material=trim_gray,
        name="inner_liner",
    )
    lid.visual(
        Box((0.552, 0.014, 0.005)),
        origin=Origin(xyz=(0.000, -0.419, -0.0065)),
        material=trim_gray,
        name="front_skirt",
    )
    lid.visual(
        Box((0.014, 0.356, 0.005)),
        origin=Origin(xyz=(-0.291, -0.235, -0.0065)),
        material=trim_gray,
        name="left_skirt",
    )
    lid.visual(
        Box((0.014, 0.356, 0.005)),
        origin=Origin(xyz=(0.291, -0.235, -0.0065)),
        material=trim_gray,
        name="right_skirt",
    )
    lid.visual(
        Box((0.180, 0.018, 0.005)),
        origin=Origin(xyz=(0.000, -0.015, -0.0065)),
        material=trim_gray,
        name="rear_inner_brace",
    )
    lid.visual(
        Box((0.074, 0.024, 0.013)),
        origin=Origin(xyz=(-0.190, -0.022, -0.0025)),
        material=trim_gray,
        name="left_hinge_ear",
    )
    lid.visual(
        Box((0.074, 0.024, 0.013)),
        origin=Origin(xyz=(0.190, -0.022, -0.0025)),
        material=trim_gray,
        name="right_hinge_ear",
    )
    lid.visual(
        Cylinder(radius=0.013, length=0.070),
        origin=Origin(xyz=(-0.190, 0.000, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=trim_gray,
        name="left_lid_knuckle",
    )
    lid.visual(
        Cylinder(radius=0.013, length=0.070),
        origin=Origin(xyz=(0.190, 0.000, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=trim_gray,
        name="right_lid_knuckle",
    )
    lid.visual(
        Box((0.055, 0.170, 0.002)),
        origin=Origin(xyz=(-0.235, -0.225, -0.007)),
        material=trim_gray,
        name="left_track",
    )
    lid.visual(
        Box((0.055, 0.170, 0.002)),
        origin=Origin(xyz=(0.235, -0.225, -0.007)),
        material=trim_gray,
        name="right_track",
    )

    lid.inertial = Inertial.from_geometry(
        Box((0.612, 0.406, 0.030)),
        mass=2.8,
        origin=Origin(xyz=(0.0, -0.220, 0.000)),
    )

    guide_strip = model.part("guide_strip")
    guide_strip.visual(
        Box((0.040, 0.050, 0.0015)),
        origin=Origin(xyz=(-0.235, 0.000, -0.00075)),
        material=guide_gray,
        name="left_shoe",
    )
    guide_strip.visual(
        Box((0.040, 0.050, 0.0015)),
        origin=Origin(xyz=(0.235, 0.000, -0.00075)),
        material=guide_gray,
        name="right_shoe",
    )
    guide_strip.visual(
        Box((0.500, 0.012, 0.004)),
        origin=Origin(xyz=(0.000, 0.000, -0.0035)),
        material=guide_gray,
        name="guide_bar",
    )
    guide_strip.inertial = Inertial.from_geometry(
        Box((0.500, 0.050, 0.008)),
        mass=0.20,
        origin=Origin(xyz=(0.0, 0.0, -0.003)),
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.000, 0.206, 0.087)),
        # The closed lid extends forward along local -Y from the hinge barrels,
        # so -X makes positive q lift the front edge upward.
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.2,
            lower=0.0,
            upper=math.radians(78.0),
        ),
    )
    model.articulation(
        "lid_to_guide_strip",
        ArticulationType.PRISMATIC,
        parent=lid,
        child=guide_strip,
        origin=Origin(xyz=(0.000, -0.170, -0.008)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=0.08,
            lower=0.0,
            upper=0.090,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    guide_strip = object_model.get_part("guide_strip")
    lid_hinge = object_model.get_articulation("body_to_lid")
    guide_slide = object_model.get_articulation("lid_to_guide_strip")

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

    lid_limits = lid_hinge.motion_limits
    guide_limits = guide_slide.motion_limits
    ctx.check(
        "lid hinge uses full-width rear barrel axis",
        lid_hinge.axis == (-1.0, 0.0, 0.0)
        and lid_limits is not None
        and lid_limits.lower == 0.0
        and lid_limits.upper is not None
        and lid_limits.upper > 1.2,
        details=f"axis={lid_hinge.axis}, limits={lid_limits}",
    )
    ctx.check(
        "guide strip slides along lid depth",
        guide_slide.axis == (0.0, -1.0, 0.0)
        and guide_limits is not None
        and guide_limits.upper is not None
        and 0.07 <= guide_limits.upper <= 0.12,
        details=f"axis={guide_slide.axis}, limits={guide_limits}",
    )

    with ctx.pose({lid_hinge: 0.0, guide_slide: 0.0}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="front_skirt",
            negative_elem="front_bezel",
            max_gap=0.0005,
            max_penetration=1e-6,
            name="closed lid front skirt seats on front bezel",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            elem_a="lid_outer_shell",
            elem_b="body_shell",
            min_overlap=0.30,
            name="closed lid covers the scanning body footprint",
        )
        ctx.expect_contact(
            guide_strip,
            lid,
            elem_a="left_shoe",
            elem_b="left_track",
            name="left guide shoe hangs from left lid track",
        )
        ctx.expect_contact(
            guide_strip,
            lid,
            elem_a="right_shoe",
            elem_b="right_track",
            name="right guide shoe hangs from right lid track",
        )
        ctx.expect_within(
            guide_strip,
            lid,
            axes="xy",
            inner_elem="left_shoe",
            outer_elem="left_track",
            margin=0.0,
            name="left guide shoe stays within its track at rest",
        )
        ctx.expect_within(
            guide_strip,
            lid,
            axes="xy",
            inner_elem="right_shoe",
            outer_elem="right_track",
            margin=0.0,
            name="right guide shoe stays within its track at rest",
        )

        front_closed = ctx.part_element_world_aabb(lid, elem="front_skirt")
        guide_retracted = ctx.part_world_position(guide_strip)

    guide_upper = 0.09 if guide_limits is None or guide_limits.upper is None else guide_limits.upper
    lid_upper = 1.30 if lid_limits is None or lid_limits.upper is None else lid_limits.upper

    with ctx.pose({lid_hinge: 0.0, guide_slide: guide_upper}):
        ctx.expect_contact(
            guide_strip,
            lid,
            elem_a="left_shoe",
            elem_b="left_track",
            name="left guide shoe stays supported when extended",
        )
        ctx.expect_contact(
            guide_strip,
            lid,
            elem_a="right_shoe",
            elem_b="right_track",
            name="right guide shoe stays supported when extended",
        )
        ctx.expect_overlap(
            guide_strip,
            lid,
            axes="y",
            elem_a="left_shoe",
            elem_b="left_track",
            min_overlap=0.045,
            name="left guide shoe keeps retained overlap in its track",
        )
        ctx.expect_overlap(
            guide_strip,
            lid,
            axes="y",
            elem_a="right_shoe",
            elem_b="right_track",
            min_overlap=0.045,
            name="right guide shoe keeps retained overlap in its track",
        )
        guide_extended = ctx.part_world_position(guide_strip)

    with ctx.pose({lid_hinge: lid_upper, guide_slide: 0.0}):
        front_open = ctx.part_element_world_aabb(lid, elem="front_skirt")

    ctx.check(
        "lid front edge lifts upward when opened",
        front_closed is not None
        and front_open is not None
        and front_open[1][2] > front_closed[1][2] + 0.16,
        details=f"closed={front_closed}, open={front_open}",
    )
    ctx.check(
        "guide strip extends toward the scanner front",
        guide_retracted is not None
        and guide_extended is not None
        and guide_extended[1] < guide_retracted[1] - 0.05,
        details=f"retracted={guide_retracted}, extended={guide_extended}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
