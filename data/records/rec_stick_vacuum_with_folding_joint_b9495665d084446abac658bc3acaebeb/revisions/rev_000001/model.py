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
    rounded_rect_profile,
    section_loft,
)


def _xy_section(
    width: float,
    depth: float,
    radius: float,
    z: float,
    *,
    x_shift: float = 0.0,
    y_shift: float = 0.0,
) -> list[tuple[float, float, float]]:
    return [(x + x_shift, y + y_shift, z) for x, y in rounded_rect_profile(width, depth, radius)]


def _yz_section(
    width_y: float,
    height_z: float,
    radius: float,
    x: float,
    *,
    z_center: float = 0.0,
) -> list[tuple[float, float, float]]:
    return [(x, y, z_center + z) for y, z in rounded_rect_profile(width_y, height_z, radius)]


def _aabb_center(aabb) -> tuple[float, float, float] | None:
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple((mins[index] + maxs[index]) * 0.5 for index in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="precision_professional_stick_vacuum")

    housing_charcoal = model.material("housing_charcoal", rgba=(0.20, 0.22, 0.24, 1.0))
    trim_black = model.material("trim_black", rgba=(0.08, 0.09, 0.10, 1.0))
    satin_aluminum = model.material("satin_aluminum", rgba=(0.72, 0.74, 0.77, 1.0))
    smoked_clear = model.material("smoked_clear", rgba=(0.46, 0.56, 0.62, 0.34))
    datum_gray = model.material("datum_gray", rgba=(0.62, 0.66, 0.70, 1.0))
    signal_orange = model.material("signal_orange", rgba=(0.91, 0.43, 0.08, 1.0))

    body_shell_mesh = mesh_from_geometry(
        section_loft(
            [
                _xy_section(0.082, 0.074, 0.016, 0.098, x_shift=-0.004),
                _xy_section(0.104, 0.082, 0.020, 0.274, x_shift=0.004),
                _xy_section(0.090, 0.076, 0.018, 0.492, x_shift=-0.010),
            ]
        ),
        "vacuum_body_shell",
    )
    wand_tube_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            rounded_rect_profile(0.036, 0.024, 0.0055),
            [rounded_rect_profile(0.028, 0.016, 0.0035)],
            0.50,
            center=True,
        ),
        "vacuum_wand_tube",
    )
    floor_head_shell_mesh = mesh_from_geometry(
        section_loft(
            [
                _yz_section(0.060, 0.028, 0.007, 0.020, z_center=-0.048),
                _yz_section(0.076, 0.034, 0.008, 0.108, z_center=-0.046),
                _yz_section(0.056, 0.020, 0.006, 0.245, z_center=-0.040),
            ]
        ),
        "vacuum_floor_head_shell",
    )

    upper_body = model.part("upper_body")
    upper_body.visual(body_shell_mesh, material=housing_charcoal, name="main_shell")
    upper_body.visual(
        Box((0.028, 0.062, 0.150)),
        origin=Origin(xyz=(-0.030, 0.0, 0.085)),
        material=housing_charcoal,
        name="lower_spine",
    )
    upper_body.visual(
        Box((0.020, 0.060, 0.024)),
        origin=Origin(xyz=(-0.026, 0.0, 0.022)),
        material=trim_black,
        name="fold_bridge",
    )
    upper_body.visual(
        Box((0.014, 0.012, 0.044)),
        origin=Origin(xyz=(-0.019, -0.024, 0.000)),
        material=satin_aluminum,
        name="fold_clevis_left",
    )
    upper_body.visual(
        Box((0.014, 0.012, 0.044)),
        origin=Origin(xyz=(-0.019, 0.024, 0.000)),
        material=satin_aluminum,
        name="fold_clevis_right",
    )
    upper_body.visual(
        Cylinder(radius=0.006, length=0.004),
        origin=Origin(xyz=(-0.019, 0.030, 0.010), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim_black,
        name="fold_adjuster_disc",
    )
    upper_body.visual(
        Box((0.012, 0.004, 0.036)),
        origin=Origin(xyz=(-0.018, -0.028, -0.004)),
        material=datum_gray,
        name="fold_index_plate",
    )
    for index, z_pos in enumerate((-0.017, -0.004, 0.011)):
        upper_body.visual(
            Box((0.010 + 0.003 * index, 0.004, 0.002)),
            origin=Origin(xyz=(-0.018, -0.028, z_pos)),
            material=signal_orange,
            name=f"fold_index_mark_{index}",
        )
    upper_body.visual(
        Box((0.050, 0.060, 0.100)),
        origin=Origin(xyz=(0.030, 0.0, 0.245)),
        material=trim_black,
        name="dust_cup_collar",
    )
    upper_body.visual(
        Cylinder(radius=0.031, length=0.150),
        origin=Origin(xyz=(0.068, 0.0, 0.245), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=smoked_clear,
        name="dust_cup",
    )
    upper_body.visual(
        Box((0.040, 0.050, 0.210)),
        origin=Origin(xyz=(-0.028, 0.0, 0.438)),
        material=housing_charcoal,
        name="handle_tower",
    )
    upper_body.visual(
        Box((0.082, 0.040, 0.038)),
        origin=Origin(xyz=(0.008, 0.0, 0.558)),
        material=trim_black,
        name="top_grip",
    )
    upper_body.visual(
        Box((0.030, 0.040, 0.145)),
        origin=Origin(xyz=(0.030, 0.0, 0.485)),
        material=trim_black,
        name="front_grip_post",
    )
    upper_body.visual(
        Box((0.056, 0.050, 0.070)),
        origin=Origin(xyz=(-0.018, 0.0, 0.118)),
        material=trim_black,
        name="battery_heel",
    )
    upper_body.visual(
        Box((0.024, 0.010, 0.090)),
        origin=Origin(xyz=(-0.018, -0.041, 0.156)),
        material=datum_gray,
        name="datum_pad_left",
    )
    upper_body.visual(
        Box((0.024, 0.010, 0.090)),
        origin=Origin(xyz=(-0.018, 0.041, 0.156)),
        material=datum_gray,
        name="datum_pad_right",
    )
    upper_body.inertial = Inertial.from_geometry(
        Box((0.150, 0.100, 0.600)),
        mass=2.6,
        origin=Origin(xyz=(0.010, 0.0, 0.300)),
    )

    wand = model.part("wand")
    wand.visual(
        Cylinder(radius=0.014, length=0.036),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_aluminum,
        name="fold_knuckle",
    )
    wand.visual(
        Box((0.036, 0.030, 0.040)),
        origin=Origin(xyz=(0.016, 0.0, -0.016)),
        material=trim_black,
        name="upper_collar",
    )
    wand.visual(
        Box((0.028, 0.020, 0.520)),
        origin=Origin(xyz=(0.016, 0.0, -0.292)),
        material=satin_aluminum,
        name="wand_tube",
    )
    wand.visual(
        Box((0.012, 0.004, 0.160)),
        origin=Origin(xyz=(0.010, -0.010, -0.045)),
        material=datum_gray,
        name="alignment_rail",
    )
    wand.visual(
        Box((0.050, 0.040, 0.070)),
        origin=Origin(xyz=(-0.002, 0.0, -0.579)),
        material=trim_black,
        name="lower_collar",
    )
    for index, z_pos in enumerate((-0.095, -0.072, -0.049)):
        wand.visual(
            Box((0.010 + 0.004 * (index % 2), 0.004, 0.002)),
            origin=Origin(xyz=(0.010, -0.010, z_pos)),
            material=signal_orange,
            name=f"alignment_mark_{index}",
        )
    wand.visual(
        Box((0.012, 0.052, 0.060)),
        origin=Origin(xyz=(-0.020, 0.0, -0.602)),
        material=trim_black,
        name="nozzle_bridge",
    )
    wand.visual(
        Box((0.010, 0.012, 0.090)),
        origin=Origin(xyz=(-0.020, -0.019, -0.587)),
        material=trim_black,
        name="nozzle_web_left",
    )
    wand.visual(
        Box((0.010, 0.012, 0.090)),
        origin=Origin(xyz=(-0.020, 0.019, -0.587)),
        material=trim_black,
        name="nozzle_web_right",
    )
    wand.visual(
        Box((0.008, 0.008, 0.052)),
        origin=Origin(xyz=(-0.019, -0.022, -0.630)),
        material=satin_aluminum,
        name="nozzle_fork_left",
    )
    wand.visual(
        Box((0.008, 0.008, 0.052)),
        origin=Origin(xyz=(-0.019, 0.022, -0.630)),
        material=satin_aluminum,
        name="nozzle_fork_right",
    )
    wand.visual(
        Cylinder(radius=0.011, length=0.010),
        origin=Origin(xyz=(-0.019, -0.027, -0.630), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=datum_gray,
        name="nozzle_adjuster_left",
    )
    wand.visual(
        Cylinder(radius=0.011, length=0.010),
        origin=Origin(xyz=(-0.019, 0.027, -0.630), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=datum_gray,
        name="nozzle_adjuster_right",
    )
    wand.visual(
        Cylinder(radius=0.006, length=0.010),
        origin=Origin(xyz=(-0.019, -0.027, -0.630), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim_black,
        name="nozzle_pin_left",
    )
    wand.visual(
        Cylinder(radius=0.006, length=0.010),
        origin=Origin(xyz=(-0.019, 0.027, -0.630), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim_black,
        name="nozzle_pin_right",
    )
    wand.inertial = Inertial.from_geometry(
        Box((0.050, 0.060, 0.680)),
        mass=1.1,
        origin=Origin(xyz=(0.006, 0.0, -0.340)),
    )

    floor_head = model.part("floor_head")
    floor_head.visual(
        Cylinder(radius=0.014, length=0.036),
        origin=Origin(xyz=(-0.006, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=satin_aluminum,
        name="pitch_knuckle",
    )
    floor_head.visual(
        Box((0.030, 0.032, 0.050)),
        origin=Origin(xyz=(0.018, 0.0, -0.026)),
        material=trim_black,
        name="neck_block",
    )
    floor_head.visual(
        floor_head_shell_mesh,
        material=housing_charcoal,
        name="head_shell",
    )
    floor_head.visual(
        Box((0.018, 0.070, 0.024)),
        origin=Origin(xyz=(-0.006, 0.0, -0.058)),
        material=trim_black,
        name="rear_heel",
    )
    floor_head.visual(
        Cylinder(radius=0.010, length=0.236),
        origin=Origin(xyz=(0.246, 0.0, -0.042), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=trim_black,
        name="front_bumper",
    )
    floor_head.visual(
        Box((0.195, 0.014, 0.004)),
        origin=Origin(xyz=(0.096, -0.024, -0.061)),
        material=datum_gray,
        name="datum_skid_left",
    )
    floor_head.visual(
        Box((0.195, 0.014, 0.004)),
        origin=Origin(xyz=(0.096, 0.024, -0.061)),
        material=datum_gray,
        name="datum_skid_right",
    )
    floor_head.visual(
        Box((0.100, 0.022, 0.004)),
        origin=Origin(xyz=(0.118, 0.0, -0.028)),
        material=smoked_clear,
        name="inspection_window",
    )
    floor_head.visual(
        Box((0.050, 0.012, 0.006)),
        origin=Origin(xyz=(0.024, -0.030, -0.031)),
        material=datum_gray,
        name="side_datum_left",
    )
    floor_head.visual(
        Box((0.050, 0.012, 0.006)),
        origin=Origin(xyz=(0.024, 0.030, -0.031)),
        material=datum_gray,
        name="side_datum_right",
    )
    floor_head.inertial = Inertial.from_geometry(
        Box((0.300, 0.080, 0.070)),
        mass=0.9,
        origin=Origin(xyz=(0.110, 0.0, -0.042)),
    )

    model.articulation(
        "body_to_wand_fold",
        ArticulationType.REVOLUTE,
        parent=upper_body,
        child=wand,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=1.4,
            lower=0.0,
            upper=math.radians(112.0),
        ),
    )
    model.articulation(
        "wand_to_floor_head",
        ArticulationType.REVOLUTE,
        parent=wand,
        child=floor_head,
        origin=Origin(xyz=(0.006, 0.0, -0.630)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=2.0,
            lower=math.radians(-18.0),
            upper=math.radians(42.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    upper_body = object_model.get_part("upper_body")
    wand = object_model.get_part("wand")
    floor_head = object_model.get_part("floor_head")
    fold_joint = object_model.get_articulation("body_to_wand_fold")
    nozzle_joint = object_model.get_articulation("wand_to_floor_head")

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
        "primary_parts_and_joints_present",
        all(item is not None for item in (upper_body, wand, floor_head, fold_joint, nozzle_joint)),
        details="Upper body, wand, floor head, and both primary articulations must exist.",
    )
    ctx.expect_contact(
        wand,
        upper_body,
        contact_tol=5e-4,
        name="fold_joint_is_physically_supported",
    )
    ctx.expect_contact(
        floor_head,
        wand,
        contact_tol=5e-4,
        name="nozzle_joint_is_physically_supported",
    )

    with ctx.pose({fold_joint: 0.0, nozzle_joint: 0.0}):
        ctx.expect_gap(
            upper_body,
            floor_head,
            axis="z",
            positive_elem="main_shell",
            negative_elem="head_shell",
            min_gap=0.60,
            max_gap=0.82,
            name="operating_stack_has_realistic_height",
        )
        index_plate_aabb = ctx.part_element_world_aabb(upper_body, elem="fold_index_plate")
        alignment_rail_aabb = ctx.part_element_world_aabb(wand, elem="alignment_rail")
        controlled_gap_ok = False
        controlled_gap_details = "Missing fold calibration geometry."
        if index_plate_aabb is not None and alignment_rail_aabb is not None:
            y_gap = alignment_rail_aabb[0][1] - index_plate_aabb[1][1]
            z_overlap = min(index_plate_aabb[1][2], alignment_rail_aabb[1][2]) - max(
                index_plate_aabb[0][2], alignment_rail_aabb[0][2]
            )
            controlled_gap_ok = 0.007 <= y_gap <= 0.015 and z_overlap >= 0.020
            controlled_gap_details = (
                f"Expected a controlled 7-15 mm witness gap with >=20 mm vertical overlap; "
                f"got y_gap={y_gap:.4f}, z_overlap={z_overlap:.4f}."
            )
        ctx.check(
            "fold_calibration_gap_is_controlled",
            controlled_gap_ok,
            details=controlled_gap_details,
        )

    rest_front_bumper = ctx.part_element_world_aabb(floor_head, elem="front_bumper")
    with ctx.pose({nozzle_joint: math.radians(32.0)}):
        pitched_front_bumper = ctx.part_element_world_aabb(floor_head, elem="front_bumper")
        nozzle_lift_ok = False
        nozzle_lift_details = "Front bumper AABB unavailable for nozzle articulation check."
        if rest_front_bumper is not None and pitched_front_bumper is not None:
            rest_z = rest_front_bumper[1][2]
            pitched_z = pitched_front_bumper[1][2]
            nozzle_lift_ok = pitched_z >= rest_z + 0.028
            nozzle_lift_details = (
                f"Expected pitched front edge to rise by at least 28 mm; "
                f"got {pitched_z - rest_z:.4f} m."
            )
        ctx.check("nozzle_pitch_lifts_front_edge", nozzle_lift_ok, details=nozzle_lift_details)
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_with_nozzle_pitched_up")

    with ctx.pose({fold_joint: math.radians(108.0), nozzle_joint: math.radians(18.0)}):
        folded_head = ctx.part_element_world_aabb(floor_head, elem="head_shell")
        body_shell = ctx.part_element_world_aabb(upper_body, elem="main_shell")
        storage_motion_ok = False
        storage_motion_details = "Missing folded-pose shell geometry."
        head_center = _aabb_center(folded_head)
        body_center = _aabb_center(body_shell)
        if head_center is not None and body_center is not None:
            storage_motion_ok = head_center[0] >= body_center[0] + 0.17
            storage_motion_details = (
                f"Expected folded floor head center to move at least 170 mm forward of the body; "
                f"got delta_x={head_center[0] - body_center[0]:.4f} m."
            )
        ctx.check(
            "fold_joint_swings_floor_head_forward_for_storage",
            storage_motion_ok,
            details=storage_motion_details,
        )
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_in_folded_service_pose")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
