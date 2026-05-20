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


def _xy_section(
    width: float,
    depth: float,
    radius: float,
    z_pos: float,
    *,
    x_center: float = 0.0,
    y_center: float = 0.0,
) -> list[tuple[float, float, float]]:
    return [
        (x + x_center, y + y_center, z_pos)
        for x, y in rounded_rect_profile(width, depth, radius)
    ]


def _yz_section(
    width: float,
    height: float,
    radius: float,
    x_pos: float,
    *,
    z_center: float = 0.0,
) -> list[tuple[float, float, float]]:
    return [
        (x_pos, y, z + z_center)
        for y, z in rounded_rect_profile(width, height, radius)
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="weatherproof_stick_vacuum")

    shell_polymer = model.material("shell_polymer", rgba=(0.19, 0.23, 0.22, 1.0))
    wand_alloy = model.material("wand_alloy", rgba=(0.63, 0.67, 0.70, 1.0))
    seal_rubber = model.material("seal_rubber", rgba=(0.11, 0.12, 0.12, 1.0))
    hardware_steel = model.material("hardware_steel", rgba=(0.74, 0.78, 0.80, 1.0))
    bumper_polymer = model.material("bumper_polymer", rgba=(0.29, 0.31, 0.33, 1.0))

    main_body = model.part("main_body")

    body_shell_geom = section_loft(
        [
            _xy_section(0.094, 0.076, 0.018, 0.028, x_center=-0.006),
            _xy_section(0.088, 0.072, 0.017, 0.190, x_center=-0.010),
            _xy_section(0.074, 0.064, 0.015, 0.420, x_center=-0.015),
        ]
    )
    main_body.visual(
        mesh_from_geometry(body_shell_geom, "main_body_shell"),
        material=shell_polymer,
        name="body_shell",
    )
    main_body.visual(
        Box((0.092, 0.078, 0.014)),
        origin=Origin(xyz=(-0.010, 0.0, 0.255)),
        material=seal_rubber,
        name="seal_band",
    )
    main_body.visual(
        Box((0.024, 0.040, 0.190)),
        origin=Origin(xyz=(-0.050, 0.0, 0.260)),
        material=shell_polymer,
        name="rear_grip_spine",
    )
    main_body.visual(
        Box((0.050, 0.040, 0.028)),
        origin=Origin(xyz=(-0.035, 0.0, 0.369)),
        material=shell_polymer,
        name="upper_handle_bridge",
    )
    main_body.visual(
        Box((0.045, 0.040, 0.020)),
        origin=Origin(xyz=(-0.036, 0.0, 0.210)),
        material=shell_polymer,
        name="lower_handle_bridge",
    )
    main_body.visual(
        Box((0.031, 0.070, 0.040)),
        origin=Origin(xyz=(-0.0305, 0.0, 0.020)),
        material=shell_polymer,
        name="fold_mount_block",
    )
    main_body.visual(
        Box((0.056, 0.082, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.051)),
        material=seal_rubber,
        name="fold_drip_cap",
    )
    main_body.visual(
        Box((0.030, 0.008, 0.070)),
        origin=Origin(xyz=(0.0, -0.031, 0.010)),
        material=shell_polymer,
        name="left_fold_cheek",
    )
    main_body.visual(
        Box((0.030, 0.008, 0.070)),
        origin=Origin(xyz=(0.0, 0.031, 0.010)),
        material=shell_polymer,
        name="right_fold_cheek",
    )
    main_body.visual(
        Cylinder(radius=0.008, length=0.004),
        origin=Origin(xyz=(0.0, -0.037, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hardware_steel,
        name="left_fold_pin_cap",
    )
    main_body.visual(
        Cylinder(radius=0.008, length=0.004),
        origin=Origin(xyz=(0.0, 0.037, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hardware_steel,
        name="right_fold_pin_cap",
    )
    main_body.inertial = Inertial.from_geometry(
        Box((0.120, 0.100, 0.430)),
        mass=2.9,
        origin=Origin(xyz=(-0.015, 0.0, 0.215)),
    )

    wand = model.part("wand")
    wand.visual(
        Cylinder(radius=0.012, length=0.030),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hardware_steel,
        name="fold_knuckle",
    )
    wand.visual(
        Cylinder(radius=0.027, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, -0.020)),
        material=seal_rubber,
        name="upper_joint_boot",
    )
    wand.visual(
        Cylinder(radius=0.018, length=0.660),
        origin=Origin(xyz=(0.0, 0.0, -0.330)),
        material=wand_alloy,
        name="wand_tube",
    )
    wand.visual(
        Cylinder(radius=0.022, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, -0.680)),
        material=seal_rubber,
        name="lower_joint_sleeve",
    )
    wand.visual(
        Box((0.048, 0.064, 0.032)),
        origin=Origin(xyz=(0.0, 0.0, -0.684)),
        material=wand_alloy,
        name="nozzle_mount_block",
    )
    wand.visual(
        Box((0.050, 0.084, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, -0.688)),
        material=seal_rubber,
        name="nozzle_joint_hood",
    )
    wand.visual(
        Box((0.034, 0.008, 0.056)),
        origin=Origin(xyz=(0.0, -0.031, -0.718)),
        material=wand_alloy,
        name="left_nozzle_cheek",
    )
    wand.visual(
        Box((0.034, 0.008, 0.056)),
        origin=Origin(xyz=(0.0, 0.031, -0.718)),
        material=wand_alloy,
        name="right_nozzle_cheek",
    )
    wand.visual(
        Cylinder(radius=0.008, length=0.004),
        origin=Origin(xyz=(0.0, -0.037, -0.720), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hardware_steel,
        name="left_nozzle_pin_cap",
    )
    wand.visual(
        Cylinder(radius=0.008, length=0.004),
        origin=Origin(xyz=(0.0, 0.037, -0.720), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hardware_steel,
        name="right_nozzle_pin_cap",
    )
    wand.inertial = Inertial.from_geometry(
        Cylinder(radius=0.025, length=0.740),
        mass=1.1,
        origin=Origin(xyz=(0.0, 0.0, -0.370)),
    )

    floor_head = model.part("floor_head")
    floor_head.visual(
        Cylinder(radius=0.012, length=0.030),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hardware_steel,
        name="nozzle_knuckle",
    )
    floor_head.visual(
        Box((0.036, 0.054, 0.032)),
        origin=Origin(xyz=(0.010, 0.0, -0.018)),
        material=seal_rubber,
        name="neck_boot",
    )
    floor_head.visual(
        Box((0.090, 0.052, 0.026)),
        origin=Origin(xyz=(0.045, 0.0, -0.045)),
        material=bumper_polymer,
        name="air_throat",
    )
    head_shell_geom = section_loft(
        [
            _yz_section(0.082, 0.034, 0.010, 0.030, z_center=-0.058),
            _yz_section(0.112, 0.038, 0.012, 0.115, z_center=-0.056),
            _yz_section(0.120, 0.028, 0.010, 0.200, z_center=-0.050),
        ]
    )
    floor_head.visual(
        mesh_from_geometry(head_shell_geom, "floor_head_shell"),
        material=shell_polymer,
        name="head_shell",
    )
    floor_head.visual(
        Box((0.220, 0.106, 0.008)),
        origin=Origin(xyz=(0.128, 0.0, -0.072)),
        material=bumper_polymer,
        name="sole_plate",
    )
    floor_head.visual(
        Box((0.032, 0.116, 0.014)),
        origin=Origin(xyz=(0.192, 0.0, -0.053)),
        material=bumper_polymer,
        name="front_bumper",
    )
    floor_head.inertial = Inertial.from_geometry(
        Box((0.240, 0.120, 0.080)),
        mass=0.9,
        origin=Origin(xyz=(0.120, 0.0, -0.050)),
    )

    model.articulation(
        "body_to_wand_fold",
        ArticulationType.REVOLUTE,
        parent=main_body,
        child=wand,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=1.5,
            lower=0.0,
            upper=1.20,
        ),
    )
    model.articulation(
        "wand_to_floor_head_pitch",
        ArticulationType.REVOLUTE,
        parent=wand,
        child=floor_head,
        origin=Origin(xyz=(0.0, 0.0, -0.720)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=2.0,
            lower=-0.45,
            upper=0.55,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    main_body = object_model.get_part("main_body")
    wand = object_model.get_part("wand")
    floor_head = object_model.get_part("floor_head")
    fold_joint = object_model.get_articulation("body_to_wand_fold")
    nozzle_joint = object_model.get_articulation("wand_to_floor_head_pitch")

    left_fold_cheek = main_body.get_visual("left_fold_cheek")
    right_fold_cheek = main_body.get_visual("right_fold_cheek")
    fold_knuckle = wand.get_visual("fold_knuckle")
    left_nozzle_cheek = wand.get_visual("left_nozzle_cheek")
    right_nozzle_cheek = wand.get_visual("right_nozzle_cheek")
    nozzle_knuckle = floor_head.get_visual("nozzle_knuckle")
    front_bumper = floor_head.get_visual("front_bumper")

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

    ctx.expect_gap(
        main_body,
        wand,
        axis="y",
        positive_elem=right_fold_cheek,
        negative_elem=fold_knuckle,
        min_gap=0.010,
        max_gap=0.016,
        name="right_fold_bracket_gap",
    )
    ctx.expect_gap(
        wand,
        main_body,
        axis="y",
        positive_elem=fold_knuckle,
        negative_elem=left_fold_cheek,
        min_gap=0.010,
        max_gap=0.016,
        name="left_fold_bracket_gap",
    )
    ctx.expect_overlap(
        main_body,
        wand,
        axes="xz",
        elem_a=right_fold_cheek,
        elem_b=fold_knuckle,
        min_overlap=0.015,
        name="fold_knuckle_registered_in_body_brackets",
    )

    ctx.expect_gap(
        wand,
        floor_head,
        axis="y",
        positive_elem=right_nozzle_cheek,
        negative_elem=nozzle_knuckle,
        min_gap=0.010,
        max_gap=0.016,
        name="right_nozzle_bracket_gap",
    )
    ctx.expect_gap(
        floor_head,
        wand,
        axis="y",
        positive_elem=nozzle_knuckle,
        negative_elem=left_nozzle_cheek,
        min_gap=0.010,
        max_gap=0.016,
        name="left_nozzle_bracket_gap",
    )
    ctx.expect_overlap(
        wand,
        floor_head,
        axes="xz",
        elem_a=right_nozzle_cheek,
        elem_b=nozzle_knuckle,
        min_overlap=0.015,
        name="nozzle_knuckle_registered_in_wand_brackets",
    )

    with ctx.pose({fold_joint: 0.0, nozzle_joint: 0.0}):
        operating_head_position = ctx.part_world_position(floor_head)
        operating_front_bumper = ctx.part_element_world_aabb(floor_head, elem=front_bumper)

    with ctx.pose({fold_joint: 1.05, nozzle_joint: 0.0}):
        folded_head_position = ctx.part_world_position(floor_head)
        fold_ok = (
            operating_head_position is not None
            and folded_head_position is not None
            and folded_head_position[0] > operating_head_position[0] + 0.45
            and folded_head_position[2] > operating_head_position[2] + 0.25
        )
        ctx.check(
            "fold_joint_lifts_and_tucks_head",
            fold_ok,
            details=(
                f"operating={operating_head_position}, folded={folded_head_position}"
            ),
        )
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_in_folded_pose")

    with ctx.pose({fold_joint: 0.0, nozzle_joint: 0.35}):
        pitched_front_bumper = ctx.part_element_world_aabb(floor_head, elem=front_bumper)
        pitch_ok = (
            operating_front_bumper is not None
            and pitched_front_bumper is not None
            and ((pitched_front_bumper[0][2] + pitched_front_bumper[1][2]) * 0.5)
            > ((operating_front_bumper[0][2] + operating_front_bumper[1][2]) * 0.5) + 0.04
        )
        ctx.check(
            "nozzle_pitch_lifts_front_bumper",
            pitch_ok,
            details=(
                f"operating_front={operating_front_bumper}, pitched_front={pitched_front_bumper}"
            ),
        )
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_at_nozzle_pitch")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
