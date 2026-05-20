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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
)


def _xy_section(
    width_x: float,
    width_y: float,
    z: float,
    *,
    x_shift: float = 0.0,
    y_shift: float = 0.0,
    radius: float | None = None,
) -> list[tuple[float, float, float]]:
    corner = radius if radius is not None else min(width_x, width_y) * 0.22
    return [
        (x + x_shift, y + y_shift, z)
        for x, y in rounded_rect_profile(width_x, width_y, corner)
    ]


def _yz_section(
    width_y: float,
    height_z: float,
    x: float,
    *,
    z_shift: float = 0.0,
    radius: float | None = None,
) -> list[tuple[float, float, float]]:
    corner = radius if radius is not None else min(width_y, height_z) * 0.22
    return [
        (x, y, z + z_shift)
        for z, y in rounded_rect_profile(height_z, width_y, corner)
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="retrofit_stick_vacuum")

    painted_steel = model.material("painted_steel", rgba=(0.49, 0.52, 0.46, 1.0))
    charcoal = model.material("charcoal", rgba=(0.18, 0.19, 0.20, 1.0))
    dark_rubber = model.material("dark_rubber", rgba=(0.09, 0.09, 0.10, 1.0))
    warm_metal = model.material("warm_metal", rgba=(0.67, 0.67, 0.63, 1.0))
    hatch_tone = model.material("hatch_tone", rgba=(0.58, 0.60, 0.55, 1.0))

    body = model.part("body")
    body_shell_mesh = mesh_from_geometry(
        section_loft(
            [
                _xy_section(0.076, 0.056, 0.020, x_shift=-0.002),
                _xy_section(0.084, 0.064, 0.130, x_shift=0.002),
                _xy_section(0.100, 0.072, 0.280, x_shift=0.016),
                _xy_section(0.086, 0.064, 0.420, x_shift=0.028),
                _xy_section(0.060, 0.050, 0.500, x_shift=0.060),
            ]
        ),
        "vacuum_body_shell",
    )
    body.visual(body_shell_mesh, material=painted_steel, name="body_shell")
    body.visual(
        Box((0.030, 0.034, 0.220)),
        origin=Origin(xyz=(0.030, 0.000, 0.390)),
        material=charcoal,
        name="rear_handle_spine",
    )
    body.visual(
        Box((0.090, 0.032, 0.022)),
        origin=Origin(xyz=(0.078, 0.000, 0.505)),
        material=charcoal,
        name="top_handle_grip",
    )
    body.visual(
        Box((0.024, 0.032, 0.120)),
        origin=Origin(xyz=(0.108, 0.000, 0.445)),
        material=charcoal,
        name="front_handle_knuckle",
    )
    body.visual(
        Box((0.032, 0.052, 0.028)),
        origin=Origin(xyz=(0.000, 0.000, 0.014)),
        material=warm_metal,
        name="lower_adapter_block",
    )
    body.visual(
        Box((0.016, 0.012, 0.060)),
        origin=Origin(xyz=(0.000, 0.032, -0.010)),
        material=warm_metal,
        name="left_fold_bracket",
    )
    body.visual(
        Box((0.016, 0.012, 0.060)),
        origin=Origin(xyz=(0.000, -0.032, -0.010)),
        material=warm_metal,
        name="right_fold_bracket",
    )
    body.visual(
        Cylinder(radius=0.010, length=0.006),
        origin=Origin(xyz=(0.000, 0.041, -0.014), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=warm_metal,
        name="left_fold_pin_cap",
    )
    body.visual(
        Cylinder(radius=0.010, length=0.006),
        origin=Origin(xyz=(0.000, -0.041, -0.014), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=warm_metal,
        name="right_fold_pin_cap",
    )
    body.visual(
        Box((0.060, 0.004, 0.120)),
        origin=Origin(xyz=(0.020, 0.036, 0.245)),
        material=hatch_tone,
        name="left_service_hatch",
    )
    body.visual(
        Box((0.060, 0.004, 0.120)),
        origin=Origin(xyz=(0.020, -0.036, 0.245)),
        material=hatch_tone,
        name="right_service_hatch",
    )
    for side, y in (("left", 0.039), ("right", -0.039)):
        for idx, z in enumerate((0.205, 0.285)):
            body.visual(
                Cylinder(radius=0.004, length=0.006),
                origin=Origin(xyz=(0.000, y, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
                material=warm_metal,
                name=f"{side}_hatch_bolt_{idx}",
            )
    body.visual(
        Box((0.070, 0.010, 0.026)),
        origin=Origin(xyz=(0.016, 0.000, 0.098)),
        material=warm_metal,
        name="body_reinforcement_band",
    )
    body.visual(
        Box((0.060, 0.028, 0.050)),
        origin=Origin(xyz=(0.056, 0.000, 0.150)),
        material=charcoal,
        name="filter_service_pod",
    )

    wand = model.part("wand")
    wand.visual(
        Cylinder(radius=0.020, length=0.560),
        origin=Origin(xyz=(0.000, 0.000, -0.280)),
        material=charcoal,
        name="wand_tube",
    )
    wand.visual(
        Box((0.018, 0.040, 0.034)),
        origin=Origin(xyz=(0.000, 0.000, -0.017)),
        material=warm_metal,
        name="upper_fold_tongue",
    )
    wand.visual(
        Box((0.026, 0.010, 0.068)),
        origin=Origin(xyz=(0.000, 0.015, -0.050)),
        material=warm_metal,
        name="upper_left_gusset",
    )
    wand.visual(
        Box((0.026, 0.010, 0.068)),
        origin=Origin(xyz=(0.000, -0.015, -0.050)),
        material=warm_metal,
        name="upper_right_gusset",
    )
    wand.visual(
        Cylinder(radius=0.008, length=0.024),
        origin=Origin(xyz=(0.000, 0.000, -0.016), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=warm_metal,
        name="upper_pivot_sleeve",
    )
    wand.visual(
        Box((0.012, 0.024, 0.300)),
        origin=Origin(xyz=(0.012, 0.000, -0.230)),
        material=painted_steel,
        name="wand_spine_reinforcement",
    )
    wand.visual(
        Cylinder(radius=0.023, length=0.026),
        origin=Origin(xyz=(0.000, 0.000, -0.300)),
        material=warm_metal,
        name="mid_service_collar",
    )
    for side, y in (("left", 0.024), ("right", -0.024)):
        wand.visual(
            Cylinder(radius=0.004, length=0.008),
            origin=Origin(xyz=(0.000, y, -0.300), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=warm_metal,
            name=f"{side}_collar_bolt",
        )
    wand.visual(
        Box((0.034, 0.056, 0.022)),
        origin=Origin(xyz=(0.000, 0.000, -0.569)),
        material=warm_metal,
        name="lower_fork_bridge",
    )
    wand.visual(
        Box((0.018, 0.010, 0.052)),
        origin=Origin(xyz=(0.000, 0.031, -0.604)),
        material=warm_metal,
        name="lower_left_fork",
    )
    wand.visual(
        Box((0.018, 0.010, 0.052)),
        origin=Origin(xyz=(0.000, -0.031, -0.604)),
        material=warm_metal,
        name="lower_right_fork",
    )
    wand.visual(
        Cylinder(radius=0.008, length=0.008),
        origin=Origin(xyz=(0.000, 0.038, -0.598), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=warm_metal,
        name="left_nozzle_pin_cap",
    )
    wand.visual(
        Cylinder(radius=0.008, length=0.008),
        origin=Origin(xyz=(0.000, -0.038, -0.598), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=warm_metal,
        name="right_nozzle_pin_cap",
    )

    head = model.part("floor_head")
    head_shell_mesh = mesh_from_geometry(
        section_loft(
            [
                _yz_section(0.044, 0.030, 0.018, z_shift=-0.018),
                _yz_section(0.120, 0.050, 0.096, z_shift=-0.024),
                _yz_section(0.150, 0.048, 0.150, z_shift=-0.024),
                _yz_section(0.110, 0.034, 0.255, z_shift=-0.017),
            ]
        ),
        "vacuum_floor_head_shell",
    )
    head.visual(head_shell_mesh, material=painted_steel, name="head_shell")
    head.visual(
        Box((0.028, 0.036, 0.040)),
        origin=Origin(xyz=(0.014, 0.000, -0.020)),
        material=warm_metal,
        name="neck_block",
    )
    head.visual(
        Box((0.040, 0.012, 0.028)),
        origin=Origin(xyz=(0.032, 0.016, -0.022)),
        material=warm_metal,
        name="left_neck_shoulder",
    )
    head.visual(
        Box((0.040, 0.012, 0.028)),
        origin=Origin(xyz=(0.032, -0.016, -0.022)),
        material=warm_metal,
        name="right_neck_shoulder",
    )
    head.visual(
        Cylinder(radius=0.008, length=0.022),
        origin=Origin(xyz=(0.006, 0.000, -0.010), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=warm_metal,
        name="nozzle_pivot_sleeve",
    )
    head.visual(
        Box((0.110, 0.064, 0.004)),
        origin=Origin(xyz=(0.120, 0.000, -0.002)),
        material=hatch_tone,
        name="service_plate",
    )
    for x in (0.078, 0.162):
        for y in (-0.022, 0.022):
            head.visual(
                Cylinder(radius=0.004, length=0.006),
                origin=Origin(xyz=(x, y, 0.000)),
                material=warm_metal,
                name=f"service_bolt_{int(round(x * 1000))}_{int(round((y + 0.1) * 1000))}",
            )
    head.visual(
        Box((0.022, 0.116, 0.024)),
        origin=Origin(xyz=(0.245, 0.000, -0.020)),
        material=dark_rubber,
        name="front_bumper",
    )
    head.visual(
        Box((0.180, 0.014, 0.016)),
        origin=Origin(xyz=(0.132, 0.050, -0.028)),
        material=dark_rubber,
        name="left_glide_strip",
    )
    head.visual(
        Box((0.180, 0.014, 0.016)),
        origin=Origin(xyz=(0.132, -0.050, -0.028)),
        material=dark_rubber,
        name="right_glide_strip",
    )
    head.visual(
        Box((0.120, 0.030, 0.018)),
        origin=Origin(xyz=(0.160, 0.000, -0.026)),
        material=charcoal,
        name="intake_throat",
    )
    head.visual(
        Box((0.072, 0.020, 0.018)),
        origin=Origin(xyz=(0.064, 0.000, -0.026)),
        material=warm_metal,
        name="intake_adapter_bridge",
    )

    fold_joint = model.articulation(
        "body_to_wand",
        ArticulationType.REVOLUTE,
        parent=body,
        child=wand,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=1.5,
            lower=0.0,
            upper=math.radians(70.0),
        ),
    )
    nozzle_joint = model.articulation(
        "wand_to_head",
        ArticulationType.REVOLUTE,
        parent=wand,
        child=head,
        origin=Origin(xyz=(0.000, 0.000, -0.580)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=2.0,
            lower=math.radians(-18.0),
            upper=math.radians(34.0),
        ),
    )

    body.meta["primary_fold_joint"] = fold_joint.name
    head.meta["primary_nozzle_joint"] = nozzle_joint.name

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    wand = object_model.get_part("wand")
    head = object_model.get_part("floor_head")
    fold_joint = object_model.get_articulation("body_to_wand")
    nozzle_joint = object_model.get_articulation("wand_to_head")

    body.get_visual("left_service_hatch")
    body.get_visual("right_service_hatch")
    wand.get_visual("mid_service_collar")
    head.get_visual("service_plate")
    head.get_visual("front_bumper")

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

    with ctx.pose({fold_joint: 0.0, nozzle_joint: 0.0}):
        ctx.expect_contact(
            body,
            wand,
            contact_tol=0.0015,
            name="fold_joint_is_structurally_supported",
        )
        ctx.expect_contact(
            wand,
            head,
            contact_tol=0.0015,
            name="nozzle_joint_is_structurally_supported",
        )
        ctx.expect_origin_gap(
            body,
            head,
            axis="z",
            min_gap=0.50,
            name="floor_head_sits_below_body",
        )

    with ctx.pose({fold_joint: 0.0, nozzle_joint: 0.0}):
        rest_head_pos = ctx.part_world_position(head)
        rest_front_bumper_aabb = ctx.part_element_world_aabb(head, elem="front_bumper")
    with ctx.pose({fold_joint: fold_joint.motion_limits.upper, nozzle_joint: 0.0}):
        folded_head_pos = ctx.part_world_position(head)
    with ctx.pose({fold_joint: 0.0, nozzle_joint: nozzle_joint.motion_limits.upper}):
        pitched_front_bumper_aabb = ctx.part_element_world_aabb(head, elem="front_bumper")

    if rest_head_pos is None or folded_head_pos is None:
        ctx.fail("fold_joint_pose_measurement_available", "Unable to measure floor head positions.")
    else:
        ctx.check(
            "fold_joint_moves_lower_assembly_forward",
            folded_head_pos[0] > rest_head_pos[0] + 0.18,
            details=(
                f"Expected folded head x to move forward by > 0.18 m, got rest={rest_head_pos} "
                f"folded={folded_head_pos}."
            ),
        )

    if rest_front_bumper_aabb is None or pitched_front_bumper_aabb is None:
        ctx.fail("nozzle_pitch_measurement_available", "Unable to measure nozzle bumper AABBs.")
    else:
        ctx.check(
            "nozzle_pitch_lifts_front_bumper",
            pitched_front_bumper_aabb[1][2] > rest_front_bumper_aabb[1][2] + 0.025,
            details=(
                "Expected nozzle articulation to lift the front bumper. "
                f"rest_max_z={rest_front_bumper_aabb[1][2]:.4f}, "
                f"pitched_max_z={pitched_front_bumper_aabb[1][2]:.4f}"
            ),
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
