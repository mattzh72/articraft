from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

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
    tube_from_spline_points,
)


def _tube_mesh(
    name: str,
    points: list[tuple[float, float, float]],
    *,
    radius: float,
    samples_per_segment: int = 14,
    radial_segments: int = 18,
):
    return mesh_from_geometry(
        tube_from_spline_points(
            points,
            radius=radius,
            samples_per_segment=samples_per_segment,
            radial_segments=radial_segments,
            cap_ends=True,
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="field_service_vacuum_cleaner")

    housing = model.material("housing", rgba=(0.31, 0.36, 0.33, 1.0))
    housing_dark = model.material("housing_dark", rgba=(0.18, 0.20, 0.20, 1.0))
    steel = model.material("steel", rgba=(0.60, 0.62, 0.64, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.24, 0.26, 0.28, 1.0))
    rubber = model.material("rubber", rgba=(0.06, 0.06, 0.06, 1.0))
    service_orange = model.material("service_orange", rgba=(0.80, 0.38, 0.12, 1.0))
    filter_gray = model.material("filter_gray", rgba=(0.52, 0.56, 0.58, 1.0))

    body = model.part("body")
    body.visual(
        Box((0.44, 0.24, 0.02)),
        origin=Origin(xyz=(0.0, 0.0, 0.01)),
        material=housing_dark,
        name="base_plate",
    )
    body.visual(
        Box((0.40, 0.02, 0.18)),
        origin=Origin(xyz=(0.0, 0.11, 0.11)),
        material=housing,
        name="left_wall",
    )
    body.visual(
        Box((0.40, 0.02, 0.18)),
        origin=Origin(xyz=(0.0, -0.11, 0.11)),
        material=housing,
        name="right_wall",
    )
    body.visual(
        Box((0.04, 0.20, 0.18)),
        origin=Origin(xyz=(0.20, 0.0, 0.11)),
        material=housing,
        name="front_wall",
    )
    body.visual(
        Box((0.04, 0.20, 0.18)),
        origin=Origin(xyz=(-0.20, 0.0, 0.11)),
        material=housing,
        name="rear_wall",
    )
    body.visual(
        Box((0.36, 0.02, 0.03)),
        origin=Origin(xyz=(0.0, 0.09, 0.215)),
        material=housing_dark,
        name="left_rim",
    )
    body.visual(
        Box((0.36, 0.02, 0.03)),
        origin=Origin(xyz=(0.0, -0.09, 0.215)),
        material=housing_dark,
        name="right_rim",
    )
    body.visual(
        Box((0.02, 0.18, 0.03)),
        origin=Origin(xyz=(0.19, 0.0, 0.215)),
        material=housing_dark,
        name="top_rim_front",
    )
    body.visual(
        Box((0.02, 0.18, 0.03)),
        origin=Origin(xyz=(-0.19, 0.0, 0.215)),
        material=housing_dark,
        name="top_rim_rear",
    )
    body.visual(
        Cylinder(radius=0.07, length=0.04),
        origin=Origin(xyz=(-0.14, 0.14, 0.07), rpy=(pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="rear_wheel_left",
    )
    body.visual(
        Cylinder(radius=0.07, length=0.04),
        origin=Origin(xyz=(-0.14, -0.14, 0.07), rpy=(pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="rear_wheel_right",
    )
    body.visual(
        Cylinder(radius=0.016, length=0.30),
        origin=Origin(xyz=(-0.14, 0.0, 0.07), rpy=(pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="rear_axle",
    )
    body.visual(
        Cylinder(radius=0.03, length=0.14),
        origin=Origin(xyz=(0.18, 0.0, 0.03), rpy=(pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="front_roller",
    )
    body.visual(
        Box((0.05, 0.02, 0.055)),
        origin=Origin(xyz=(0.18, 0.045, 0.0525)),
        material=dark_steel,
        name="front_roller_left_fork",
    )
    body.visual(
        Box((0.05, 0.02, 0.055)),
        origin=Origin(xyz=(0.18, -0.045, 0.0525)),
        material=dark_steel,
        name="front_roller_right_fork",
    )
    body.visual(
        Box((0.05, 0.12, 0.025)),
        origin=Origin(xyz=(0.185, 0.0, 0.0725)),
        material=dark_steel,
        name="front_roller_bridge",
    )
    body.visual(
        Box((0.03, 0.08, 0.10)),
        origin=Origin(xyz=(0.21, 0.0, 0.23)),
        material=dark_steel,
        name="wand_mount_bridge",
    )
    body.visual(
        Cylinder(radius=0.028, length=0.04),
        origin=Origin(xyz=(0.205, 0.0, 0.285), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="intake_collar",
    )
    body.visual(
        Box((0.03, 0.02, 0.07)),
        origin=Origin(xyz=(0.21, 0.05, 0.285)),
        material=dark_steel,
        name="shoulder_clevis_left",
    )
    body.visual(
        Box((0.03, 0.02, 0.07)),
        origin=Origin(xyz=(0.21, -0.05, 0.285)),
        material=dark_steel,
        name="shoulder_clevis_right",
    )
    body.visual(
        Box((0.05, 0.12, 0.02)),
        origin=Origin(xyz=(0.205, 0.0, 0.325)),
        material=dark_steel,
        name="shoulder_clevis_cap",
    )
    body.visual(
        Box((0.025, 0.02, 0.07)),
        origin=Origin(xyz=(-0.19, 0.09, 0.245)),
        material=dark_steel,
        name="handle_left_post",
    )
    body.visual(
        Box((0.025, 0.02, 0.07)),
        origin=Origin(xyz=(-0.19, -0.09, 0.245)),
        material=dark_steel,
        name="handle_right_post",
    )
    body.visual(
        _tube_mesh(
            "vacuum_carry_handle",
            [
                (-0.19, 0.09, 0.28),
                (-0.22, 0.09, 0.30),
                (-0.23, 0.0, 0.31),
                (-0.22, -0.09, 0.30),
                (-0.19, -0.09, 0.28),
            ],
            radius=0.012,
            samples_per_segment=12,
            radial_segments=16,
        ),
        material=steel,
        name="carry_handle",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.50, 0.30, 0.34)),
        mass=17.0,
        origin=Origin(xyz=(0.0, 0.0, 0.13)),
    )

    filter_cassette = model.part("filter_cassette")
    filter_cassette.visual(
        Box((0.20, 0.16, 0.11)),
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
        material=filter_gray,
        name="cassette_core",
    )
    filter_cassette.visual(
        Box((0.16, 0.12, 0.01)),
        origin=Origin(xyz=(0.0, 0.0, 0.115)),
        material=housing_dark,
        name="service_cap",
    )
    filter_cassette.visual(
        Box((0.018, 0.018, 0.04)),
        origin=Origin(xyz=(0.05, 0.0, 0.122)),
        material=service_orange,
        name="handle_left_post",
    )
    filter_cassette.visual(
        Box((0.018, 0.018, 0.04)),
        origin=Origin(xyz=(-0.05, 0.0, 0.122)),
        material=service_orange,
        name="handle_right_post",
    )
    filter_cassette.visual(
        Box((0.12, 0.022, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, 0.150)),
        material=service_orange,
        name="service_handle",
    )
    for index, x_pos in enumerate((-0.07, -0.035, 0.0, 0.035, 0.07)):
        filter_cassette.visual(
            Box((0.01, 0.15, 0.07)),
            origin=Origin(xyz=(x_pos, 0.0, 0.065)),
            material=housing_dark,
            name=f"filter_rib_{index}",
        )
    filter_cassette.inertial = Inertial.from_geometry(
        Box((0.20, 0.16, 0.16)),
        mass=2.0,
        origin=Origin(xyz=(0.0, 0.0, 0.08)),
    )

    service_lid = model.part("service_lid")
    service_lid.visual(
        Box((0.36, 0.20, 0.014)),
        origin=Origin(xyz=(0.18, 0.0, 0.007)),
        material=housing,
        name="main_panel",
    )
    service_lid.visual(
        Box((0.02, 0.16, 0.022)),
        origin=Origin(xyz=(0.35, 0.0, 0.011)),
        material=housing_dark,
        name="front_grip",
    )
    service_lid.visual(
        Box((0.30, 0.016, 0.026)),
        origin=Origin(xyz=(0.19, 0.082, 0.013)),
        material=housing_dark,
        name="left_stiffener",
    )
    service_lid.visual(
        Box((0.30, 0.016, 0.026)),
        origin=Origin(xyz=(0.19, -0.082, 0.013)),
        material=housing_dark,
        name="right_stiffener",
    )
    service_lid.visual(
        Cylinder(radius=0.014, length=0.14),
        origin=Origin(xyz=(0.01, 0.0, 0.012), rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="hinge_barrel",
    )
    service_lid.inertial = Inertial.from_geometry(
        Box((0.36, 0.20, 0.03)),
        mass=1.6,
        origin=Origin(xyz=(0.18, 0.0, 0.015)),
    )

    lower_wand = model.part("lower_wand")
    lower_wand.visual(
        Box((0.03, 0.05, 0.05)),
        origin=Origin(xyz=(0.015, 0.0, 0.025)),
        material=steel,
        name="shoulder_lug",
    )
    lower_wand.visual(
        Cylinder(radius=0.028, length=0.07),
        origin=Origin(xyz=(0.045, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="root_cuff",
    )
    lower_wand.visual(
        _tube_mesh(
            "vacuum_lower_wand_tube",
            [
                (0.07, 0.0, 0.0),
                (0.11, 0.0, -0.015),
                (0.16, 0.0, -0.050),
                (0.20, 0.0, -0.090),
            ],
            radius=0.021,
        ),
        material=dark_steel,
        name="main_tube",
    )
    lower_wand.visual(
        Box((0.10, 0.05, 0.036)),
        origin=Origin(xyz=(0.13, 0.0, -0.020)),
        material=service_orange,
        name="service_collar",
    )
    lower_wand.visual(
        Box((0.10, 0.06, 0.07)),
        origin=Origin(xyz=(0.21, 0.0, -0.095)),
        material=dark_steel,
        name="elbow_housing",
    )
    lower_wand.visual(
        Cylinder(radius=0.022, length=0.07),
        origin=Origin(xyz=(0.28, 0.0, -0.10), rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="distal_pivot",
    )
    lower_wand.inertial = Inertial.from_geometry(
        Box((0.34, 0.10, 0.18)),
        mass=1.2,
        origin=Origin(xyz=(0.16, 0.0, -0.04)),
    )

    upper_wand = model.part("upper_wand")
    upper_wand.visual(
        Box((0.03, 0.05, 0.05)),
        origin=Origin(xyz=(0.037, 0.0, 0.0)),
        material=steel,
        name="shoulder_lug",
    )
    upper_wand.visual(
        Cylinder(radius=0.027, length=0.07),
        origin=Origin(xyz=(0.06, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="root_cuff",
    )
    upper_wand.visual(
        _tube_mesh(
            "vacuum_upper_wand_tube",
            [
                (0.07, 0.0, 0.0),
                (0.13, 0.0, -0.018),
                (0.19, 0.0, -0.056),
                (0.24, 0.0, -0.102),
            ],
            radius=0.020,
        ),
        material=dark_steel,
        name="main_tube",
    )
    upper_wand.visual(
        Box((0.10, 0.05, 0.036)),
        origin=Origin(xyz=(0.14, 0.0, -0.020)),
        material=service_orange,
        name="service_collar",
    )
    upper_wand.visual(
        Box((0.10, 0.06, 0.07)),
        origin=Origin(xyz=(0.27, 0.0, -0.11)),
        material=dark_steel,
        name="nozzle_housing",
    )
    upper_wand.visual(
        Cylinder(radius=0.022, length=0.07),
        origin=Origin(xyz=(0.32, 0.0, -0.125), rpy=(pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="distal_pivot",
    )
    upper_wand.inertial = Inertial.from_geometry(
        Box((0.38, 0.10, 0.20)),
        mass=1.0,
        origin=Origin(xyz=(0.17, 0.0, -0.05)),
    )

    floor_nozzle = model.part("floor_nozzle")
    floor_nozzle.visual(
        Box((0.03, 0.05, 0.05)),
        origin=Origin(xyz=(0.037, 0.0, 0.0)),
        material=steel,
        name="pitch_lug",
    )
    floor_nozzle.visual(
        Cylinder(radius=0.028, length=0.08),
        origin=Origin(xyz=(0.07, 0.0, -0.005), rpy=(0.0, pi / 2.0, 0.0)),
        material=steel,
        name="neck_socket",
    )
    floor_nozzle.visual(
        Box((0.12, 0.06, 0.05)),
        origin=Origin(xyz=(0.10, 0.0, -0.020)),
        material=dark_steel,
        name="neck_arm",
    )
    floor_nozzle.visual(
        Box((0.31, 0.12, 0.024)),
        origin=Origin(xyz=(0.19, 0.0, -0.017)),
        material=housing_dark,
        name="head_shell",
    )
    floor_nozzle.visual(
        Box((0.26, 0.072, 0.022)),
        origin=Origin(xyz=(0.20, 0.0, -0.039)),
        material=dark_steel,
        name="wear_shoe",
    )
    floor_nozzle.visual(
        Box((0.25, 0.016, 0.012)),
        origin=Origin(xyz=(0.205, 0.0, -0.047)),
        material=service_orange,
        name="front_squeegee",
    )
    floor_nozzle.visual(
        Cylinder(radius=0.012, length=0.07),
        origin=Origin(xyz=(0.07, 0.0, -0.038), rpy=(pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="rear_roller",
    )
    floor_nozzle.visual(
        Box((0.03, 0.02, 0.03)),
        origin=Origin(xyz=(0.33, 0.05, -0.02)),
        material=service_orange,
        name="left_bumper",
    )
    floor_nozzle.visual(
        Box((0.03, 0.02, 0.03)),
        origin=Origin(xyz=(0.33, -0.05, -0.02)),
        material=service_orange,
        name="right_bumper",
    )
    floor_nozzle.inertial = Inertial.from_geometry(
        Box((0.34, 0.14, 0.07)),
        mass=1.8,
        origin=Origin(xyz=(0.18, 0.0, -0.02)),
    )

    wear_pad_left = model.part("wear_pad_left")
    wear_pad_left.visual(
        Box((0.10, 0.022, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, -0.005)),
        material=service_orange,
        name="pad_block",
    )
    wear_pad_left.visual(
        Cylinder(radius=0.004, length=0.004),
        origin=Origin(xyz=(-0.03, 0.0, -0.008)),
        material=steel,
        name="rear_fastener",
    )
    wear_pad_left.visual(
        Cylinder(radius=0.004, length=0.004),
        origin=Origin(xyz=(0.03, 0.0, -0.008)),
        material=steel,
        name="front_fastener",
    )
    wear_pad_left.inertial = Inertial.from_geometry(
        Box((0.10, 0.022, 0.014)),
        mass=0.08,
        origin=Origin(xyz=(0.0, 0.0, -0.006)),
    )

    wear_pad_right = model.part("wear_pad_right")
    wear_pad_right.visual(
        Box((0.10, 0.022, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, -0.005)),
        material=service_orange,
        name="pad_block",
    )
    wear_pad_right.visual(
        Cylinder(radius=0.004, length=0.004),
        origin=Origin(xyz=(-0.03, 0.0, -0.008)),
        material=steel,
        name="rear_fastener",
    )
    wear_pad_right.visual(
        Cylinder(radius=0.004, length=0.004),
        origin=Origin(xyz=(0.03, 0.0, -0.008)),
        material=steel,
        name="front_fastener",
    )
    wear_pad_right.inertial = Inertial.from_geometry(
        Box((0.10, 0.022, 0.014)),
        mass=0.08,
        origin=Origin(xyz=(0.0, 0.0, -0.006)),
    )

    model.articulation(
        "body_to_filter_cassette",
        ArticulationType.FIXED,
        parent=body,
        child=filter_cassette,
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
    )
    model.articulation(
        "body_to_service_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=service_lid,
        origin=Origin(xyz=(-0.18, 0.0, 0.23)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.4, lower=0.0, upper=1.2),
    )
    model.articulation(
        "body_to_lower_wand",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lower_wand,
        origin=Origin(xyz=(0.23, 0.0, 0.285)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=16.0, velocity=1.6, lower=-0.35, upper=0.95),
    )
    model.articulation(
        "lower_to_upper_wand",
        ArticulationType.REVOLUTE,
        parent=lower_wand,
        child=upper_wand,
        origin=Origin(xyz=(0.28, 0.0, -0.10)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=14.0, velocity=1.8, lower=-0.25, upper=1.1),
    )
    model.articulation(
        "upper_to_floor_nozzle",
        ArticulationType.REVOLUTE,
        parent=upper_wand,
        child=floor_nozzle,
        origin=Origin(xyz=(0.32, 0.0, -0.125)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=2.0, lower=-0.2, upper=0.6),
    )
    model.articulation(
        "floor_nozzle_to_wear_pad_left",
        ArticulationType.FIXED,
        parent=floor_nozzle,
        child=wear_pad_left,
        origin=Origin(xyz=(0.19, 0.045, -0.05)),
    )
    model.articulation(
        "floor_nozzle_to_wear_pad_right",
        ArticulationType.FIXED,
        parent=floor_nozzle,
        child=wear_pad_right,
        origin=Origin(xyz=(0.19, -0.045, -0.05)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    filter_cassette = object_model.get_part("filter_cassette")
    service_lid = object_model.get_part("service_lid")
    lower_wand = object_model.get_part("lower_wand")
    upper_wand = object_model.get_part("upper_wand")
    floor_nozzle = object_model.get_part("floor_nozzle")
    wear_pad_left = object_model.get_part("wear_pad_left")
    wear_pad_right = object_model.get_part("wear_pad_right")

    lid_joint = object_model.get_articulation("body_to_service_lid")
    shoulder_joint = object_model.get_articulation("body_to_lower_wand")
    elbow_joint = object_model.get_articulation("lower_to_upper_wand")
    nozzle_joint = object_model.get_articulation("upper_to_floor_nozzle")

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

    ctx.expect_contact(filter_cassette, body, name="filter_cassette_is_supported")
    ctx.expect_within(filter_cassette, body, axes="xy", margin=0.0, name="filter_cassette_sits_in_body")
    ctx.expect_gap(
        service_lid,
        filter_cassette,
        axis="z",
        min_gap=0.05,
        max_gap=0.10,
        name="lid_clears_filter_cassette_when_closed",
    )
    ctx.expect_contact(service_lid, body, name="service_lid_seats_on_body_rim")
    ctx.expect_contact(lower_wand, body, name="lower_wand_is_bracketed_to_body")
    ctx.expect_contact(upper_wand, lower_wand, name="upper_wand_is_bracketed_to_lower_wand")
    ctx.expect_contact(floor_nozzle, upper_wand, name="floor_nozzle_is_bracketed_to_upper_wand")
    ctx.expect_contact(wear_pad_left, floor_nozzle, name="left_wear_pad_is_mounted")
    ctx.expect_contact(wear_pad_right, floor_nozzle, name="right_wear_pad_is_mounted")
    ctx.expect_origin_distance(
        floor_nozzle,
        body,
        axes="x",
        min_dist=0.75,
        name="floor_nozzle_reaches_forward_of_body",
    )

    with ctx.pose({lid_joint: 1.1}):
        ctx.expect_gap(
            service_lid,
            body,
            axis="z",
            min_gap=0.22,
            positive_elem="front_grip",
            negative_elem="top_rim_front",
            name="service_lid_opens_high_for_access",
        )

    with ctx.pose({shoulder_joint: 0.6, elbow_joint: 0.55, nozzle_joint: 0.30}):
        ctx.expect_gap(
            floor_nozzle,
            body,
            axis="z",
            min_gap=0.11,
            positive_elem="head_shell",
            negative_elem="top_rim_front",
            name="raised_wand_clears_body_for_service_pose",
        )
        ctx.expect_origin_distance(
            floor_nozzle,
            body,
            axes="x",
            min_dist=0.50,
            name="raised_wand_keeps_nozzle_forward",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
