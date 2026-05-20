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
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _circle_profile(
    radius: float,
    *,
    segments: int = 32,
    cx: float = 0.0,
    cy: float = 0.0,
) -> list[tuple[float, float]]:
    return [
        (
            cx + radius * math.cos(2.0 * math.pi * i / segments),
            cy + radius * math.sin(2.0 * math.pi * i / segments),
        )
        for i in range(segments)
    ]


def _link_plate_mesh(
    name: str,
    *,
    hole_spacing: float,
    width: float,
    end_radius: float,
    hole_radius: float,
    thickness: float,
):
    outer = rounded_rect_profile(hole_spacing + 2.0 * end_radius, width, end_radius)
    center_x = hole_spacing * 0.5
    outer = [(x + center_x, y) for x, y in outer]
    holes = [
        _circle_profile(hole_radius, segments=24, cx=0.0, cy=0.0),
        _circle_profile(hole_radius, segments=24, cx=hole_spacing, cy=0.0),
    ]
    return mesh_from_geometry(
        ExtrudeWithHolesGeometry(outer, holes, thickness, center=True),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="outdoor_weatherproof_wiper_assembly")

    coated_steel = model.material("coated_steel", rgba=(0.19, 0.21, 0.22, 1.0))
    stainless = model.material("stainless", rgba=(0.73, 0.76, 0.79, 1.0))
    anodized = model.material("anodized", rgba=(0.53, 0.56, 0.60, 1.0))
    rubber = model.material("rubber", rgba=(0.07, 0.07, 0.08, 1.0))
    plastic = model.material("plastic", rgba=(0.12, 0.13, 0.14, 1.0))

    frame = model.part("cowl_frame")
    frame.inertial = Inertial.from_geometry(
        Box((0.84, 0.12, 0.09)),
        mass=8.0,
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
    )

    base_plate_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            rounded_rect_profile(0.84, 0.10, 0.014),
            [
                _circle_profile(0.012, cx=-0.28, cy=0.0),
                _circle_profile(0.011, cx=-0.04, cy=0.0),
                _circle_profile(0.011, cx=0.24, cy=0.0),
            ],
            0.006,
            center=True,
        ),
        "wiper_base_plate",
    )
    frame.visual(
        base_plate_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=coated_steel,
        name="base_plate",
    )

    for side, x_pos, name in (
        ("left", -0.04, "left"),
        ("right", 0.24, "right"),
    ):
        tower_mesh = mesh_from_geometry(
            ExtrudeWithHolesGeometry(
                _circle_profile(0.029),
                [_circle_profile(0.011)],
                0.052,
                center=True,
            ),
            f"{side}_wiper_tower",
        )
        cap_mesh = mesh_from_geometry(
            ExtrudeWithHolesGeometry(
                _circle_profile(0.036),
                [_circle_profile(0.011)],
                0.004,
                center=True,
            ),
            f"{side}_tower_cap",
        )
        frame.visual(
            tower_mesh,
            origin=Origin(xyz=(x_pos, 0.0, 0.032)),
            material=coated_steel,
            name=f"{name}_tower",
        )
        frame.visual(
            cap_mesh,
            origin=Origin(xyz=(x_pos, 0.0, 0.060)),
            material=coated_steel,
            name=f"{name}_tower_cap",
        )

    frame.visual(
        Box((0.56, 0.012, 0.024)),
        origin=Origin(xyz=(0.08, 0.054, 0.018)),
        material=coated_steel,
        name="front_drip_lip",
    )
    frame.visual(
        Box((0.06, 0.012, 0.024)),
        origin=Origin(xyz=(-0.39, 0.054, 0.018)),
        material=coated_steel,
        name="front_left_return",
    )
    frame.visual(
        Box((0.56, 0.012, 0.028)),
        origin=Origin(xyz=(0.08, -0.054, 0.020)),
        material=coated_steel,
        name="rear_stiffener",
    )
    frame.visual(
        Box((0.06, 0.012, 0.028)),
        origin=Origin(xyz=(-0.39, -0.054, 0.020)),
        material=coated_steel,
        name="rear_left_return",
    )
    frame.visual(
        Box((0.058, 0.028, 0.008)),
        origin=Origin(xyz=(-0.332, -0.014, 0.002)),
        material=coated_steel,
        name="motor_left_pad",
    )
    frame.visual(
        Box((0.058, 0.028, 0.008)),
        origin=Origin(xyz=(-0.228, -0.014, 0.002)),
        material=coated_steel,
        name="motor_right_pad",
    )
    motor = model.part("motor_housing")
    motor.inertial = Inertial.from_geometry(
        Box((0.18, 0.17, 0.10)),
        mass=4.2,
        origin=Origin(xyz=(0.0, -0.04, 0.015)),
    )

    gearbox_cover = mesh_from_geometry(
        LatheGeometry(
            [
                (0.030, 0.000),
                (0.048, 0.008),
                (0.060, 0.020),
                (0.061, 0.040),
                (0.052, 0.056),
                (0.036, 0.066),
                (0.020, 0.070),
            ],
            segments=48,
        ),
        "gearbox_cover",
    )
    motor.visual(
        gearbox_cover,
        origin=Origin(xyz=(0.0, 0.0, -0.010)),
        material=coated_steel,
        name="gearbox_cover",
    )
    motor.visual(
        Cylinder(radius=0.039, length=0.130),
        origin=Origin(xyz=(-0.010, -0.126, 0.018), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=coated_steel,
        name="motor_can",
    )
    motor.visual(
        Box((0.050, 0.020, 0.010)),
        origin=Origin(xyz=(0.052, -0.014, -0.005)),
        material=coated_steel,
        name="mount_foot_right",
    )
    motor.visual(
        Box((0.050, 0.020, 0.010)),
        origin=Origin(xyz=(-0.052, -0.014, -0.005)),
        material=coated_steel,
        name="mount_foot_left",
    )
    motor.visual(
        Cylinder(radius=0.016, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, -0.013)),
        material=rubber,
        name="output_seal",
    )
    motor.visual(
        Box((0.032, 0.006, 0.008)),
        origin=Origin(xyz=(0.016, 0.018, -0.006)),
        material=coated_steel,
        name="seal_retainer_upper",
    )
    motor.visual(
        Box((0.032, 0.006, 0.008)),
        origin=Origin(xyz=(0.016, -0.018, -0.006)),
        material=coated_steel,
        name="seal_retainer_lower",
    )
    motor.visual(
        Box((0.082, 0.024, 0.018)),
        origin=Origin(xyz=(0.008, 0.028, 0.030)),
        material=plastic,
        name="front_shroud",
    )
    motor.visual(
        Box((0.030, 0.022, 0.026)),
        origin=Origin(xyz=(-0.012, -0.178, 0.018)),
        material=plastic,
        name="sealed_connector",
    )

    crank = model.part("drive_crank")
    crank.inertial = Inertial.from_geometry(
        Box((0.10, 0.04, 0.03)),
        mass=0.35,
        origin=Origin(xyz=(0.030, 0.0, -0.002)),
    )
    crank.visual(
        Cylinder(radius=0.015, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, -0.004)),
        material=stainless,
        name="hub",
    )
    crank.visual(
        Box((0.078, 0.016, 0.004)),
        origin=Origin(xyz=(0.039, 0.0, -0.005)),
        material=stainless,
        name="crank_web",
    )
    crank.visual(
        Cylinder(radius=0.006, length=0.012),
        origin=Origin(xyz=(0.074, 0.0, 0.000)),
        material=stainless,
        name="crank_pin",
    )
    crank.visual(
        Cylinder(radius=0.009, length=0.002),
        origin=Origin(xyz=(0.074, 0.0, 0.004)),
        material=stainless,
        name="crank_pin_cap",
    )
    crank.visual(
        Box((0.022, 0.018, 0.006)),
        origin=Origin(xyz=(-0.010, 0.0, -0.004)),
        material=stainless,
        name="counterweight",
    )

    drive_link = model.part("drive_link")
    drive_link.inertial = Inertial.from_geometry(
        Box((0.14, 0.03, 0.02)),
        mass=0.20,
        origin=Origin(xyz=(0.053, 0.0, 0.0)),
    )
    drive_link.visual(
        _link_plate_mesh(
            "drive_link_plate",
            hole_spacing=0.106,
            width=0.020,
            end_radius=0.010,
            hole_radius=0.006,
            thickness=0.006,
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=stainless,
        name="drive_link_plate",
    )
    drive_link.visual(
        Box((0.054, 0.008, 0.003)),
        origin=Origin(xyz=(0.053, 0.0, 0.0125)),
        material=stainless,
        name="drive_link_stiffener",
    )

    left_wiper = model.part("left_wiper")
    left_wiper.inertial = Inertial.from_geometry(
        Box((0.12, 0.44, 0.11)),
        mass=1.0,
        origin=Origin(xyz=(0.006, 0.200, 0.050)),
    )
    left_wiper.visual(
        Cylinder(radius=0.007, length=0.096),
        origin=Origin(xyz=(0.0, 0.0, 0.036)),
        material=stainless,
        name="spindle_shaft",
    )
    left_wiper.visual(
        Cylinder(radius=0.0105, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=rubber,
        name="spindle_boot",
    )
    left_wiper.visual(
        Cylinder(radius=0.018, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.048)),
        material=stainless,
        name="support_washer",
    )
    left_wiper.visual(
        Cylinder(radius=0.010, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.058)),
        material=stainless,
        name="spindle_nut",
    )
    left_wiper.visual(
        Box((0.060, 0.014, 0.004)),
        origin=Origin(xyz=(-0.030, 0.0, -0.005)),
        material=stainless,
        name="drive_lever",
    )
    left_wiper.visual(
        Cylinder(radius=0.006, length=0.012),
        origin=Origin(xyz=(-0.060, 0.0, 0.000)),
        material=stainless,
        name="drive_pin",
    )
    left_wiper.visual(
        Cylinder(radius=0.009, length=0.002),
        origin=Origin(xyz=(-0.060, 0.0, 0.004)),
        material=stainless,
        name="drive_pin_cap",
    )
    left_wiper.visual(
        Box((0.060, 0.012, 0.004)),
        origin=Origin(xyz=(0.030, 0.0, 0.001)),
        material=stainless,
        name="transfer_lever",
    )
    left_wiper.visual(
        Cylinder(radius=0.006, length=0.012),
        origin=Origin(xyz=(0.060, 0.0, 0.006)),
        material=stainless,
        name="transfer_pin_left",
    )
    left_wiper.visual(
        Cylinder(radius=0.009, length=0.002),
        origin=Origin(xyz=(0.060, 0.0, 0.010)),
        material=stainless,
        name="transfer_pin_left_cap",
    )
    left_wiper.visual(
        Box((0.032, 0.070, 0.012)),
        origin=Origin(xyz=(0.004, 0.042, 0.058)),
        material=anodized,
        name="arm_root",
    )
    left_wiper.visual(
        Box((0.028, 0.180, 0.010)),
        origin=Origin(xyz=(0.008, 0.130, 0.058)),
        material=anodized,
        name="arm_primary",
    )
    left_wiper.visual(
        Box((0.020, 0.120, 0.008)),
        origin=Origin(xyz=(0.014, 0.256, 0.056)),
        material=anodized,
        name="arm_secondary",
    )
    left_wiper.visual(
        Box((0.050, 0.018, 0.014)),
        origin=Origin(xyz=(0.018, 0.330, 0.052)),
        material=stainless,
        name="blade_carrier",
    )
    left_wiper.visual(
        Box((0.020, 0.016, 0.008)),
        origin=Origin(xyz=(0.016, 0.319, 0.055)),
        material=anodized,
        name="carrier_bridge",
    )
    left_wiper.visual(
        Box((0.022, 0.360, 0.010)),
        origin=Origin(xyz=(0.018, 0.330, 0.042)),
        material=stainless,
        name="blade_rail",
    )
    left_wiper.visual(
        Box((0.010, 0.360, 0.020)),
        origin=Origin(xyz=(0.020, 0.330, 0.027)),
        material=rubber,
        name="blade_rubber",
    )

    transfer_link = model.part("transfer_link")
    transfer_link.inertial = Inertial.from_geometry(
        Box((0.20, 0.03, 0.02)),
        mass=0.30,
        origin=Origin(xyz=(0.080, 0.0, 0.0)),
    )
    transfer_link.visual(
        _link_plate_mesh(
            "transfer_link_plate",
            hole_spacing=0.160,
            width=0.022,
            end_radius=0.011,
            hole_radius=0.006,
            thickness=0.006,
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=stainless,
        name="transfer_link_plate",
    )
    transfer_link.visual(
        Box((0.094, 0.008, 0.003)),
        origin=Origin(xyz=(0.080, 0.0, 0.0125)),
        material=stainless,
        name="transfer_link_stiffener",
    )

    right_wiper = model.part("right_wiper")
    right_wiper.inertial = Inertial.from_geometry(
        Box((0.12, 0.44, 0.11)),
        mass=1.0,
        origin=Origin(xyz=(-0.006, 0.200, 0.050)),
    )
    right_wiper.visual(
        Cylinder(radius=0.007, length=0.096),
        origin=Origin(xyz=(0.0, 0.0, 0.036)),
        material=stainless,
        name="spindle_shaft",
    )
    right_wiper.visual(
        Cylinder(radius=0.0105, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=rubber,
        name="spindle_boot",
    )
    right_wiper.visual(
        Cylinder(radius=0.018, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.048)),
        material=stainless,
        name="support_washer",
    )
    right_wiper.visual(
        Cylinder(radius=0.010, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.058)),
        material=stainless,
        name="spindle_nut",
    )
    right_wiper.visual(
        Box((0.060, 0.012, 0.004)),
        origin=Origin(xyz=(-0.030, 0.0, 0.001)),
        material=stainless,
        name="transfer_lever",
    )
    right_wiper.visual(
        Cylinder(radius=0.006, length=0.012),
        origin=Origin(xyz=(-0.060, 0.0, 0.006)),
        material=stainless,
        name="transfer_pin_right",
    )
    right_wiper.visual(
        Cylinder(radius=0.009, length=0.002),
        origin=Origin(xyz=(-0.060, 0.0, 0.010)),
        material=stainless,
        name="transfer_pin_right_cap",
    )
    right_wiper.visual(
        Box((0.032, 0.070, 0.012)),
        origin=Origin(xyz=(-0.004, 0.042, 0.058)),
        material=anodized,
        name="arm_root",
    )
    right_wiper.visual(
        Box((0.028, 0.180, 0.010)),
        origin=Origin(xyz=(-0.008, 0.130, 0.058)),
        material=anodized,
        name="arm_primary",
    )
    right_wiper.visual(
        Box((0.020, 0.120, 0.008)),
        origin=Origin(xyz=(-0.014, 0.256, 0.056)),
        material=anodized,
        name="arm_secondary",
    )
    right_wiper.visual(
        Box((0.050, 0.018, 0.014)),
        origin=Origin(xyz=(-0.018, 0.330, 0.052)),
        material=stainless,
        name="blade_carrier",
    )
    right_wiper.visual(
        Box((0.020, 0.016, 0.008)),
        origin=Origin(xyz=(-0.016, 0.319, 0.055)),
        material=anodized,
        name="carrier_bridge",
    )
    right_wiper.visual(
        Box((0.022, 0.360, 0.010)),
        origin=Origin(xyz=(-0.018, 0.330, 0.042)),
        material=stainless,
        name="blade_rail",
    )
    right_wiper.visual(
        Box((0.010, 0.360, 0.020)),
        origin=Origin(xyz=(-0.020, 0.330, 0.027)),
        material=rubber,
        name="blade_rubber",
    )

    model.articulation(
        "frame_to_motor",
        ArticulationType.FIXED,
        parent=frame,
        child=motor,
        origin=Origin(xyz=(-0.28, 0.0, 0.016)),
    )
    model.articulation(
        "motor_to_crank",
        ArticulationType.CONTINUOUS,
        parent=motor,
        child=crank,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=12.0),
    )
    model.articulation(
        "crank_to_drive_link",
        ArticulationType.REVOLUTE,
        parent=crank,
        child=drive_link,
        origin=Origin(xyz=(0.074, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=6.0,
            lower=-1.10,
            upper=1.10,
        ),
    )
    model.articulation(
        "frame_to_left_wiper",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=left_wiper,
        origin=Origin(xyz=(-0.04, 0.0, 0.016)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=2.0,
            lower=-0.12,
            upper=1.10,
        ),
    )
    model.articulation(
        "left_wiper_to_transfer_link",
        ArticulationType.FIXED,
        parent=left_wiper,
        child=transfer_link,
        origin=Origin(xyz=(0.060, 0.0, 0.006)),
    )
    model.articulation(
        "frame_to_right_wiper",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=right_wiper,
        origin=Origin(xyz=(0.24, 0.0, 0.016)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=2.0,
            lower=-0.12,
            upper=1.10,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("cowl_frame")
    motor = object_model.get_part("motor_housing")
    crank = object_model.get_part("drive_crank")
    drive_link = object_model.get_part("drive_link")
    left_wiper = object_model.get_part("left_wiper")
    transfer_link = object_model.get_part("transfer_link")
    right_wiper = object_model.get_part("right_wiper")

    crank_joint = object_model.get_articulation("motor_to_crank")
    left_joint = object_model.get_articulation("frame_to_left_wiper")
    right_joint = object_model.get_articulation("frame_to_right_wiper")

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
    ctx.allow_overlap(
        frame,
        motor,
        elem_a="base_plate",
        elem_b="output_seal",
        reason="The weather-seal grommet intentionally press-fits into the cowl spindle opening.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "motor_crank_is_continuous",
        crank_joint.articulation_type == ArticulationType.CONTINUOUS,
        f"Expected continuous motor crank, got {crank_joint.articulation_type!r}",
    )
    ctx.check(
        "supported_vertical_wiper_axes",
        left_joint.axis == (0.0, 0.0, -1.0) and right_joint.axis == (0.0, 0.0, 1.0),
        f"Unexpected spindle axes: left={left_joint.axis!r}, right={right_joint.axis!r}",
    )

    ctx.expect_contact(motor, frame, elem_a="mount_foot_left", elem_b="base_plate")
    ctx.expect_contact(motor, frame, elem_a="mount_foot_right", elem_b="base_plate")
    ctx.expect_contact(crank, motor, elem_a="hub", elem_b="output_seal")
    ctx.expect_contact(left_wiper, frame, elem_a="support_washer", elem_b="left_tower_cap")
    ctx.expect_contact(right_wiper, frame, elem_a="support_washer", elem_b="right_tower_cap")
    ctx.expect_contact(drive_link, crank, elem_a="drive_link_plate", elem_b="crank_pin_cap")
    ctx.expect_contact(drive_link, left_wiper, elem_a="drive_link_plate", elem_b="drive_pin_cap")
    ctx.expect_contact(
        transfer_link,
        left_wiper,
        elem_a="transfer_link_plate",
        elem_b="transfer_pin_left_cap",
    )
    ctx.expect_contact(
        transfer_link,
        right_wiper,
        elem_a="transfer_link_plate",
        elem_b="transfer_pin_right_cap",
    )
    ctx.expect_gap(
        left_wiper,
        frame,
        axis="z",
        positive_elem="blade_rubber",
        negative_elem="base_plate",
        min_gap=0.025,
    )
    ctx.expect_gap(
        right_wiper,
        frame,
        axis="z",
        positive_elem="blade_rubber",
        negative_elem="base_plate",
        min_gap=0.025,
    )
    ctx.expect_gap(
        drive_link,
        frame,
        axis="z",
        positive_elem="drive_link_plate",
        negative_elem="base_plate",
        min_gap=0.007,
    )
    ctx.expect_gap(
        transfer_link,
        frame,
        axis="z",
        positive_elem="transfer_link_plate",
        negative_elem="base_plate",
        min_gap=0.013,
    )

    with ctx.pose({left_joint: 0.72, right_joint: 0.72}):
        ctx.expect_contact(left_wiper, frame, elem_a="support_washer", elem_b="left_tower_cap")
        ctx.expect_contact(right_wiper, frame, elem_a="support_washer", elem_b="right_tower_cap")
        ctx.expect_gap(
            left_wiper,
            frame,
            axis="z",
            positive_elem="blade_rubber",
            negative_elem="base_plate",
            min_gap=0.025,
        )
        ctx.expect_gap(
            right_wiper,
            frame,
            axis="z",
            positive_elem="blade_rubber",
            negative_elem="base_plate",
            min_gap=0.025,
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
