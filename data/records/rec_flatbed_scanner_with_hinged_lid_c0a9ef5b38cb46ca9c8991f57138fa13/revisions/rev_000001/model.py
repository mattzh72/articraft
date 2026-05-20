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
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="scanner_platform")

    housing = model.material("housing", rgba=(0.21, 0.23, 0.25, 1.0))
    fascia = model.material("fascia", rgba=(0.30, 0.32, 0.35, 1.0))
    smoked_glass = model.material("smoked_glass", rgba=(0.32, 0.42, 0.48, 0.65))
    matte_black = model.material("matte_black", rgba=(0.08, 0.09, 0.10, 1.0))
    anodized_aluminum = model.material("anodized_aluminum", rgba=(0.77, 0.79, 0.81, 1.0))
    rubber = model.material("rubber", rgba=(0.12, 0.13, 0.14, 1.0))
    calibration_orange = model.material("calibration_orange", rgba=(0.90, 0.47, 0.16, 1.0))
    marker_white = model.material("marker_white", rgba=(0.93, 0.94, 0.95, 1.0))

    body_width = 0.44
    body_depth = 0.62
    base_height = 0.082
    frame_height = 0.038
    total_height = base_height + frame_height
    side_rail = 0.045
    front_rail = 0.050
    rear_rail = 0.075

    platen_center_y = (rear_rail - front_rail) * 0.5
    platen_width = body_width - (2.0 * side_rail) - 0.014
    platen_depth = body_depth - front_rail - rear_rail - 0.014
    platen_thickness = 0.006
    platen_center_z = base_height + (platen_thickness * 0.5)
    platen_top_z = platen_center_z + (platen_thickness * 0.5)

    body = model.part("scanner_body")
    body.visual(
        Box((body_width, body_depth, base_height)),
        origin=Origin(xyz=(0.0, 0.0, base_height * 0.5)),
        material=housing,
        name="lower_housing",
    )
    body.visual(
        Box((side_rail, body_depth, frame_height)),
        origin=Origin(
            xyz=(
                -(body_width - side_rail) * 0.5,
                0.0,
                base_height + (frame_height * 0.5),
            )
        ),
        material=fascia,
        name="left_frame_rail",
    )
    body.visual(
        Box((side_rail, body_depth, frame_height)),
        origin=Origin(
            xyz=(
                (body_width - side_rail) * 0.5,
                0.0,
                base_height + (frame_height * 0.5),
            )
        ),
        material=fascia,
        name="right_frame_rail",
    )
    body.visual(
        Box((body_width - (2.0 * side_rail), front_rail, frame_height)),
        origin=Origin(
            xyz=(
                0.0,
                -(body_depth - front_rail) * 0.5,
                base_height + (frame_height * 0.5),
            )
        ),
        material=fascia,
        name="front_frame_rail",
    )
    body.visual(
        Box((body_width - (2.0 * side_rail), rear_rail, frame_height)),
        origin=Origin(
            xyz=(
                0.0,
                (body_depth - rear_rail) * 0.5,
                base_height + (frame_height * 0.5),
            )
        ),
        material=fascia,
        name="rear_frame_rail",
    )
    body.visual(
        Cylinder(radius=0.009, length=0.080),
        origin=Origin(
            xyz=(-0.155, (body_depth * 0.5) + 0.009, total_height),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=matte_black,
        name="hinge_barrel_left",
    )
    body.visual(
        Cylinder(radius=0.009, length=0.070),
        origin=Origin(
            xyz=(0.0, (body_depth * 0.5) + 0.009, total_height),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=matte_black,
        name="hinge_barrel_center",
    )
    body.visual(
        Cylinder(radius=0.009, length=0.080),
        origin=Origin(
            xyz=(0.155, (body_depth * 0.5) + 0.009, total_height),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=matte_black,
        name="hinge_barrel_right",
    )
    body.visual(
        Box((platen_width, platen_depth, platen_thickness)),
        origin=Origin(xyz=(0.0, platen_center_y, platen_center_z)),
        material=smoked_glass,
        name="platen_panel",
    )
    body.visual(
        Box((0.022, 0.390, 0.008)),
        origin=Origin(xyz=(-(platen_width * 0.5) + 0.020, platen_center_y, base_height + 0.004)),
        material=matte_black,
        name="calibration_track",
    )
    body.visual(
        Box((0.090, 0.024, 0.010)),
        origin=Origin(xyz=(0.0, -(body_depth * 0.5) + 0.020, 0.016)),
        material=matte_black,
        name="front_status_band",
    )
    for x in (-0.15, 0.15):
        for y in (-0.22, 0.22):
            body.visual(
                Cylinder(radius=0.016, length=0.008),
                origin=Origin(xyz=(x, y, 0.004)),
                material=rubber,
            )
    body.inertial = Inertial.from_geometry(
        Box((body_width, body_depth, total_height)),
        mass=8.5,
        origin=Origin(xyz=(0.0, 0.0, total_height * 0.5)),
    )

    lid = model.part("scanner_lid")
    lid.visual(
        Box((body_width, body_depth, 0.018)),
        origin=Origin(xyz=(0.0, -(body_depth * 0.5), 0.009)),
        material=fascia,
        name="lid_shell",
    )
    lid.visual(
        Box((body_width - 0.060, body_depth - 0.110, 0.004)),
        origin=Origin(xyz=(0.0, -(body_depth * 0.5) + 0.015, 0.002)),
        material=matte_black,
        name="lid_inner_liner",
    )
    lid.visual(
        Box((body_width - 0.110, body_depth - 0.170, 0.003)),
        origin=Origin(xyz=(0.0, -(body_depth * 0.5) + 0.030, 0.0165)),
        material=smoked_glass,
        name="lid_window",
    )
    lid.visual(
        Box((body_width, 0.035, 0.024)),
        origin=Origin(xyz=(0.0, -0.018, 0.012)),
        material=housing,
        name="lid_hinge_reinforcement",
    )
    lid.visual(
        Cylinder(radius=0.009, length=0.070),
        origin=Origin(
            xyz=(-0.075, 0.009, 0.0),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=matte_black,
        name="lid_hinge_knuckle_left",
    )
    lid.visual(
        Cylinder(radius=0.009, length=0.070),
        origin=Origin(
            xyz=(0.075, 0.009, 0.0),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=matte_black,
        name="lid_hinge_knuckle_right",
    )
    lid.visual(
        Box((0.180, 0.020, 0.010)),
        origin=Origin(xyz=(0.0, -body_depth + 0.018, 0.008)),
        material=matte_black,
        name="front_grip",
    )
    lid.inertial = Inertial.from_geometry(
        Box((body_width, body_depth, 0.026)),
        mass=1.6,
        origin=Origin(xyz=(0.0, -(body_depth * 0.5), 0.013)),
    )

    turntable = model.part("turntable")
    turntable.visual(
        Cylinder(radius=0.115, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=anodized_aluminum,
        name="platter_disk",
    )
    turntable.visual(
        Cylinder(radius=0.098, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, 0.0145)),
        material=rubber,
        name="platter_mat",
    )
    turntable.visual(
        Cylinder(radius=0.020, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material=matte_black,
        name="platter_hub",
    )
    turntable.inertial = Inertial.from_geometry(
        Cylinder(radius=0.115, length=0.024),
        mass=0.9,
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
    )

    calibration_bar = model.part("calibration_bar")
    calibration_bar.visual(
        Box((0.032, 0.060, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=matte_black,
        name="slider_block",
    )
    calibration_bar.visual(
        Box((0.018, 0.200, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.019)),
        material=calibration_orange,
        name="reference_bar",
    )
    calibration_bar.visual(
        Box((0.004, 0.180, 0.006)),
        origin=Origin(xyz=(0.009, 0.0, 0.025)),
        material=marker_white,
        name="reference_strip",
    )
    calibration_bar.inertial = Inertial.from_geometry(
        Box((0.032, 0.200, 0.028)),
        mass=0.25,
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
    )

    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, body_depth * 0.5, total_height)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=1.2,
            lower=0.0,
            upper=math.radians(82.0),
        ),
    )
    model.articulation(
        "turntable_spin",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=turntable,
        origin=Origin(xyz=(0.0, platen_center_y, platen_top_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=6.0,
        ),
    )
    model.articulation(
        "calibration_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=calibration_bar,
        origin=Origin(
            xyz=(
                -(platen_width * 0.5) + 0.020,
                platen_center_y - 0.120,
                base_height + 0.008,
            )
        ),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=5.0,
            velocity=0.15,
            lower=0.0,
            upper=0.240,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("scanner_body")
    lid = object_model.get_part("scanner_lid")
    turntable = object_model.get_part("turntable")
    calibration_bar = object_model.get_part("calibration_bar")
    lid_hinge = object_model.get_articulation("lid_hinge")
    turntable_spin = object_model.get_articulation("turntable_spin")
    calibration_slide = object_model.get_articulation("calibration_slide")

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
        "scanner joints use intended axes",
        lid_hinge.axis == (-1.0, 0.0, 0.0)
        and turntable_spin.axis == (0.0, 0.0, 1.0)
        and calibration_slide.axis == (0.0, 1.0, 0.0),
        details=(
            f"lid={lid_hinge.axis}, turntable={turntable_spin.axis}, "
            f"bar={calibration_slide.axis}"
        ),
    )

    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="lid_shell",
            negative_elem="rear_frame_rail",
            max_gap=0.001,
            max_penetration=0.0,
            name="lid seats onto rear frame at the hinge edge",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            elem_a="lid_shell",
            elem_b="lower_housing",
            min_overlap=0.40,
            name="lid covers the scanner body footprint when closed",
        )

    closed_grip_aabb = ctx.part_element_world_aabb(lid, elem="front_grip")
    with ctx.pose({lid_hinge: lid_hinge.motion_limits.upper}):
        open_grip_aabb = ctx.part_element_world_aabb(lid, elem="front_grip")
    ctx.check(
        "lid front grip lifts when opened",
        closed_grip_aabb is not None
        and open_grip_aabb is not None
        and open_grip_aabb[1][2] > closed_grip_aabb[1][2] + 0.18,
        details=f"closed={closed_grip_aabb}, open={open_grip_aabb}",
    )

    ctx.expect_gap(
        turntable,
        body,
        axis="z",
        positive_elem="platter_disk",
        negative_elem="platen_panel",
        max_gap=0.001,
        max_penetration=1e-6,
        name="turntable sits on the platen without sinking into it",
    )
    ctx.expect_overlap(
        turntable,
        body,
        axes="xy",
        elem_a="platter_disk",
        elem_b="platen_panel",
        min_overlap=0.20,
        name="turntable remains centered over the platen",
    )

    ctx.expect_contact(
        calibration_bar,
        body,
        elem_a="slider_block",
        elem_b="calibration_track",
        name="calibration bar carriage is mounted on the guide track",
    )
    ctx.expect_within(
        calibration_bar,
        body,
        axes="y",
        inner_elem="slider_block",
        outer_elem="calibration_track",
        margin=0.17,
        name="calibration carriage stays within the long guide track span",
    )

    bar_rest = ctx.part_world_position(calibration_bar)
    with ctx.pose({calibration_slide: calibration_slide.motion_limits.upper}):
        bar_extended = ctx.part_world_position(calibration_bar)
        ctx.expect_contact(
            calibration_bar,
            body,
            elem_a="slider_block",
            elem_b="calibration_track",
            name="calibration carriage stays supported at max travel",
        )
    ctx.check(
        "calibration bar slides toward the rear long edge direction",
        bar_rest is not None
        and bar_extended is not None
        and bar_extended[1] > bar_rest[1] + 0.15,
        details=f"rest={bar_rest}, extended={bar_extended}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
