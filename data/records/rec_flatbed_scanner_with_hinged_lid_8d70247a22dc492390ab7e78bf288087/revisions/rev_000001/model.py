from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="inkjet_printer_scanner")

    body_white = model.material("body_white", rgba=(0.92, 0.93, 0.95, 1.0))
    trim_gray = model.material("trim_gray", rgba=(0.67, 0.69, 0.73, 1.0))
    charcoal = model.material("charcoal", rgba=(0.20, 0.21, 0.23, 1.0))
    glass = model.material("glass", rgba=(0.24, 0.36, 0.42, 0.45))
    tray_black = model.material("tray_black", rgba=(0.14, 0.15, 0.16, 1.0))
    cavity_black = model.material("cavity_black", rgba=(0.09, 0.09, 0.10, 1.0))

    body_width = 0.470
    body_depth = 0.382
    body_height = 0.185
    output_slot_width = 0.348
    output_slot_height = 0.028
    output_slot_center_z = 0.084
    output_slot_depth = 0.065

    lid_width = 0.454
    lid_depth = 0.338
    lid_thickness = 0.018

    tray_width = 0.392
    tray_length = 0.096
    tray_thickness = 0.010

    printer_body = model.part("printer_body")
    printer_body.visual(
        Box((body_width, body_depth, body_height)),
        origin=Origin(xyz=(0.0, 0.0, body_height * 0.5)),
        material=body_white,
        name="body_shell",
    )

    scanner_glass_width = 0.308
    scanner_glass_depth = 0.216
    scanner_glass_center_y = 0.020
    scanner_feature_height = 0.004
    scanner_feature_center_z = body_height - (scanner_feature_height * 0.5)

    printer_body.visual(
        Box((scanner_glass_width, scanner_glass_depth, 0.003)),
        origin=Origin(
            xyz=(
                0.0,
                scanner_glass_center_y,
                body_height - 0.0015,
            )
        ),
        material=glass,
        name="scanner_glass",
    )
    printer_body.visual(
        Box((scanner_glass_width + 0.030, 0.016, scanner_feature_height)),
        origin=Origin(
            xyz=(
                0.0,
                scanner_glass_center_y - (scanner_glass_depth * 0.5) - 0.008,
                scanner_feature_center_z,
            )
        ),
        material=trim_gray,
        name="scanner_front_frame",
    )
    printer_body.visual(
        Box((scanner_glass_width + 0.030, 0.016, scanner_feature_height)),
        origin=Origin(
            xyz=(
                0.0,
                scanner_glass_center_y + (scanner_glass_depth * 0.5) + 0.008,
                scanner_feature_center_z,
            )
        ),
        material=trim_gray,
        name="scanner_rear_frame",
    )
    printer_body.visual(
        Box((0.014, scanner_glass_depth + 0.032, scanner_feature_height)),
        origin=Origin(
            xyz=(
                -((scanner_glass_width * 0.5) + 0.007),
                scanner_glass_center_y,
                scanner_feature_center_z,
            )
        ),
        material=trim_gray,
        name="scanner_left_frame",
    )
    printer_body.visual(
        Box((0.014, scanner_glass_depth + 0.032, scanner_feature_height)),
        origin=Origin(
            xyz=(
                (scanner_glass_width * 0.5) + 0.007,
                scanner_glass_center_y,
                scanner_feature_center_z,
            )
        ),
        material=trim_gray,
        name="scanner_right_frame",
    )

    printer_body.visual(
        Box((0.230, 0.032, 0.010)),
        origin=Origin(
            xyz=(
                0.0,
                -(body_depth * 0.5) + 0.015,
                0.149,
            ),
            rpy=(-0.34, 0.0, 0.0),
        ),
        material=charcoal,
        name="control_panel",
    )
    printer_body.visual(
        Box((0.070, 0.012, 0.0025)),
        origin=Origin(
            xyz=(
                -0.055,
                -(body_depth * 0.5) + 0.0015,
                0.151,
            ),
            rpy=(-0.34, 0.0, 0.0),
        ),
        material=glass,
        name="status_display",
    )
    printer_body.visual(
        Box((output_slot_width * 0.92, 0.004, output_slot_height * 1.7)),
        origin=Origin(
            xyz=(
                0.0,
                -(body_depth * 0.5) + output_slot_depth - 0.002,
                output_slot_center_z,
            )
        ),
        material=cavity_black,
        name="output_cavity_back",
    )
    printer_body.inertial = Inertial.from_geometry(
        Box((body_width, body_depth, body_height)),
        mass=8.8,
        origin=Origin(xyz=(0.0, 0.0, body_height * 0.5)),
    )

    scanner_lid = model.part("scanner_lid")
    scanner_lid.visual(
        Box((lid_width, lid_depth, lid_thickness)),
        origin=Origin(
            xyz=(
                0.0,
                -(lid_depth * 0.5),
                lid_thickness * 0.5,
            )
        ),
        material=body_white,
        name="lid_shell",
    )
    scanner_lid.visual(
        Box((lid_width - 0.050, lid_depth - 0.056, 0.0035)),
        origin=Origin(
            xyz=(
                0.0,
                -(lid_depth * 0.5) + 0.006,
                lid_thickness - 0.00175,
            )
        ),
        material=trim_gray,
        name="lid_top_panel",
    )
    scanner_lid.visual(
        Box((0.164, 0.024, 0.006)),
        origin=Origin(
            xyz=(
                0.0,
                -lid_depth + 0.012,
                0.006,
            )
        ),
        material=charcoal,
        name="finger_lip",
    )
    scanner_lid.inertial = Inertial.from_geometry(
        Box((lid_width, lid_depth, lid_thickness)),
        mass=1.5,
        origin=Origin(
            xyz=(
                0.0,
                -(lid_depth * 0.5),
                lid_thickness * 0.5,
            )
        ),
    )

    output_tray = model.part("output_tray")
    output_tray.visual(
        Box((tray_width, tray_thickness, tray_length)),
        origin=Origin(
            xyz=(
                0.0,
                -(tray_thickness * 0.5),
                tray_length * 0.5,
            ),
        ),
        material=tray_black,
        name="tray_panel",
    )
    output_tray.visual(
        Box((tray_width - 0.060, 0.003, tray_length - 0.022)),
        origin=Origin(
            xyz=(
                0.0,
                -(tray_thickness * 0.5) - 0.0005,
                tray_length * 0.5 + 0.004,
            ),
        ),
        material=charcoal,
        name="tray_inset",
    )
    output_tray.inertial = Inertial.from_geometry(
        Box((tray_width, tray_thickness, tray_length)),
        mass=0.45,
        origin=Origin(
            xyz=(
                0.0,
                -(tray_thickness * 0.5),
                tray_length * 0.5,
            )
        ),
    )

    model.articulation(
        "rear_lid_hinge",
        ArticulationType.REVOLUTE,
        parent=printer_body,
        child=scanner_lid,
        origin=Origin(xyz=(0.0, body_depth * 0.5, body_height)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=1.5,
            lower=0.0,
            upper=1.32,
        ),
    )
    model.articulation(
        "front_output_tray_hinge",
        ArticulationType.REVOLUTE,
        parent=printer_body,
        child=output_tray,
        origin=Origin(xyz=(0.0, -(body_depth * 0.5), 0.036)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=2.0,
            lower=0.0,
            upper=1.48,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    printer_body = object_model.get_part("printer_body")
    scanner_lid = object_model.get_part("scanner_lid")
    output_tray = object_model.get_part("output_tray")
    rear_lid_hinge = object_model.get_articulation("rear_lid_hinge")
    front_output_tray_hinge = object_model.get_articulation("front_output_tray_hinge")

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
        "scanner_lid_hinge_axis",
        rear_lid_hinge.axis == (-1.0, 0.0, 0.0),
        details=f"expected (-1, 0, 0), got {rear_lid_hinge.axis}",
    )
    ctx.check(
        "output_tray_hinge_axis",
        front_output_tray_hinge.axis == (1.0, 0.0, 0.0),
        details=f"expected (1, 0, 0), got {front_output_tray_hinge.axis}",
    )
    ctx.check(
        "scanner_lid_motion_range",
        rear_lid_hinge.motion_limits is not None
        and rear_lid_hinge.motion_limits.lower == 0.0
        and rear_lid_hinge.motion_limits.upper is not None
        and rear_lid_hinge.motion_limits.upper >= 1.2,
        details=f"limits={rear_lid_hinge.motion_limits}",
    )
    ctx.check(
        "output_tray_motion_range",
        front_output_tray_hinge.motion_limits is not None
        and front_output_tray_hinge.motion_limits.lower == 0.0
        and front_output_tray_hinge.motion_limits.upper is not None
        and front_output_tray_hinge.motion_limits.upper >= 1.3,
        details=f"limits={front_output_tray_hinge.motion_limits}",
    )

    with ctx.pose({rear_lid_hinge: 0.0, front_output_tray_hinge: 0.0}):
        ctx.expect_gap(
            scanner_lid,
            printer_body,
            axis="z",
            max_gap=0.0015,
            max_penetration=0.0,
            name="lid_seats_on_body",
        )
        ctx.expect_overlap(
            scanner_lid,
            printer_body,
            axes="xy",
            min_overlap=0.300,
            name="lid_covers_flatbed",
        )
        ctx.expect_gap(
            printer_body,
            output_tray,
            axis="y",
            max_gap=0.0015,
            max_penetration=0.0,
            positive_elem="body_shell",
            negative_elem="tray_panel",
            name="tray_closes_against_front",
        )
        ctx.expect_overlap(
            printer_body,
            output_tray,
            axes="xz",
            min_overlap=0.080,
            elem_a="body_shell",
            elem_b="tray_panel",
            name="tray_spans_front_output_area",
        )
        ctx.expect_contact(
            scanner_lid,
            printer_body,
            name="lid_physically_contacts_body",
        )
        ctx.expect_contact(
            output_tray,
            printer_body,
            elem_a="tray_panel",
            elem_b="body_shell",
            name="tray_physically_contacts_body",
        )

    ctx.warn_if_articulation_overlaps(max_pose_samples=16)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
