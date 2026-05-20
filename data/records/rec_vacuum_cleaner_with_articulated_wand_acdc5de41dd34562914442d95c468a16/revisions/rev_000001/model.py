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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
    tube_from_spline_points,
)


def _save_mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _yz_section(
    *, width: float, height: float, radius: float, x: float, z_center: float
) -> list[tuple[float, float, float]]:
    return [
        (x, y, z_center + z)
        for z, y in rounded_rect_profile(
            height,
            width,
            radius,
            corner_segments=8,
        )
    ]


def _dir_from_pitch(pitch: float) -> tuple[float, float, float]:
    return (math.sin(pitch), 0.0, math.cos(pitch))


def _scale(vec: tuple[float, float, float], length: float) -> tuple[float, float, float]:
    return (vec[0] * length, vec[1] * length, vec[2] * length)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="outdoor_weatherproof_vacuum")

    shell = model.material("shell", rgba=(0.30, 0.34, 0.33, 1.0))
    cap_shell = model.material("cap_shell", rgba=(0.24, 0.27, 0.28, 1.0))
    chassis = model.material("chassis", rgba=(0.17, 0.18, 0.19, 1.0))
    wand_finish = model.material("wand_finish", rgba=(0.69, 0.71, 0.73, 1.0))
    seal = model.material("seal", rgba=(0.07, 0.07, 0.08, 1.0))
    hardware = model.material("hardware", rgba=(0.74, 0.76, 0.79, 1.0))
    wheel_rubber = model.material("wheel_rubber", rgba=(0.08, 0.08, 0.08, 1.0))

    shoulder_origin = (0.255, 0.0, 0.555)
    upper_pitch = 3.0 * math.pi / 4.0
    lower_pitch = 3.0 * math.pi / 4.0
    neck_pitch = 2.52
    upper_length = 0.33
    lower_length = 0.29
    upper_dir = _dir_from_pitch(upper_pitch)
    lower_dir = _dir_from_pitch(lower_pitch)
    neck_dir = _dir_from_pitch(neck_pitch)
    upper_end = _scale(upper_dir, upper_length)
    lower_end = _scale(lower_dir, lower_length)

    vacuum_body = model.part("vacuum_body")
    body_shell_mesh = _save_mesh(
        "vacuum_body_shell",
        section_loft(
            [
                _yz_section(width=0.27, height=0.42, radius=0.055, x=-0.17, z_center=0.29),
                _yz_section(width=0.31, height=0.48, radius=0.060, x=-0.03, z_center=0.31),
                _yz_section(width=0.29, height=0.46, radius=0.055, x=0.09, z_center=0.30),
                _yz_section(width=0.22, height=0.28, radius=0.045, x=0.21, z_center=0.23),
            ]
        ),
    )
    cap_mesh = _save_mesh(
        "vacuum_cap_shell",
        section_loft(
            [
                _yz_section(width=0.29, height=0.10, radius=0.040, x=-0.16, z_center=0.52),
                _yz_section(width=0.33, height=0.12, radius=0.042, x=-0.02, z_center=0.53),
                _yz_section(width=0.31, height=0.11, radius=0.040, x=0.12, z_center=0.52),
                _yz_section(width=0.24, height=0.08, radius=0.030, x=0.22, z_center=0.50),
            ]
        ),
    )
    seal_band_mesh = _save_mesh(
        "vacuum_seal_band",
        section_loft(
            [
                _yz_section(width=0.286, height=0.036, radius=0.016, x=-0.16, z_center=0.45),
                _yz_section(width=0.304, height=0.038, radius=0.017, x=-0.02, z_center=0.46),
                _yz_section(width=0.286, height=0.036, radius=0.016, x=0.12, z_center=0.45),
                _yz_section(width=0.220, height=0.030, radius=0.013, x=0.20, z_center=0.43),
            ]
        ),
    )

    vacuum_body.visual(body_shell_mesh, material=shell, name="body_shell")
    vacuum_body.visual(cap_mesh, material=cap_shell, name="cap_shell")
    vacuum_body.visual(seal_band_mesh, material=seal, name="seal_band")
    vacuum_body.visual(
        Box((0.34, 0.26, 0.08)),
        origin=Origin(xyz=(0.02, 0.0, 0.08)),
        material=chassis,
        name="chassis_base",
    )
    vacuum_body.visual(
        Box((0.16, 0.24, 0.04)),
        origin=Origin(xyz=(0.16, 0.0, 0.02)),
        material=chassis,
        name="front_sled",
    )
    vacuum_body.visual(
        Box((0.06, 0.24, 0.03)),
        origin=Origin(xyz=(0.25, 0.0, 0.035)),
        material=chassis,
        name="front_bumper",
    )
    vacuum_body.visual(
        Cylinder(radius=0.018, length=0.30),
        origin=Origin(xyz=(-0.11, 0.0, 0.095), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hardware,
        name="rear_axle",
    )
    vacuum_body.visual(
        Cylinder(radius=0.095, length=0.05),
        origin=Origin(xyz=(-0.11, 0.175, 0.095), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=wheel_rubber,
        name="wheel_left",
    )
    vacuum_body.visual(
        Cylinder(radius=0.095, length=0.05),
        origin=Origin(xyz=(-0.11, -0.175, 0.095), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=wheel_rubber,
        name="wheel_right",
    )
    vacuum_body.visual(
        Cylinder(radius=0.038, length=0.058),
        origin=Origin(xyz=(-0.11, 0.175, 0.095), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hardware,
        name="wheel_hub_left",
    )
    vacuum_body.visual(
        Cylinder(radius=0.038, length=0.058),
        origin=Origin(xyz=(-0.11, -0.175, 0.095), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hardware,
        name="wheel_hub_right",
    )
    vacuum_body.visual(
        Cylinder(radius=0.062, length=0.09),
        origin=Origin(xyz=(-0.03, 0.0, 0.615)),
        material=cap_shell,
        name="motor_pod",
    )
    vacuum_body.visual(
        Cylinder(radius=0.078, length=0.012),
        origin=Origin(xyz=(-0.03, 0.0, 0.664)),
        material=hardware,
        name="motor_pod_cap",
    )
    vacuum_body.visual(
        Box((0.10, 0.28, 0.018)),
        origin=Origin(xyz=(0.13, 0.0, 0.548)),
        material=cap_shell,
        name="drip_visor",
    )
    vacuum_body.visual(
        Box((0.045, 0.24, 0.020)),
        origin=Origin(xyz=(0.17, 0.0, 0.526)),
        material=cap_shell,
        name="visor_lip",
    )
    vacuum_body.visual(
        Box((0.05, 0.010, 0.09)),
        origin=Origin(xyz=(0.00, 0.145, 0.455)),
        material=hardware,
        name="latch_left",
    )
    vacuum_body.visual(
        Box((0.05, 0.010, 0.09)),
        origin=Origin(xyz=(0.00, -0.145, 0.455)),
        material=hardware,
        name="latch_right",
    )
    vacuum_body.visual(
        Cylinder(radius=0.014, length=0.11),
        origin=Origin(xyz=(-0.13, 0.085, 0.625)),
        material=hardware,
        name="handle_post_left",
    )
    vacuum_body.visual(
        Cylinder(radius=0.014, length=0.11),
        origin=Origin(xyz=(-0.13, -0.085, 0.625)),
        material=hardware,
        name="handle_post_right",
    )
    vacuum_body.visual(
        _save_mesh(
            "vacuum_handle_arch",
            tube_from_spline_points(
                [
                    (-0.13, 0.085, 0.68),
                    (-0.18, 0.045, 0.715),
                    (-0.18, -0.045, 0.715),
                    (-0.13, -0.085, 0.68),
                ],
                radius=0.014,
                samples_per_segment=14,
                radial_segments=16,
            ),
        ),
        material=hardware,
        name="handle_arch",
    )
    vacuum_body.visual(
        Box((0.08, 0.10, 0.08)),
        origin=Origin(xyz=(0.195, 0.0, 0.485)),
        material=chassis,
        name="shoulder_block",
    )
    vacuum_body.visual(
        Box((0.06, 0.14, 0.04)),
        origin=Origin(xyz=(0.205, 0.0, 0.545)),
        material=cap_shell,
        name="shoulder_hood",
    )
    vacuum_body.visual(
        Box((0.024, 0.014, 0.036)),
        origin=Origin(
            xyz=(shoulder_origin[0] - 0.010, 0.027, shoulder_origin[2]),
        ),
        material=hardware,
        name="shoulder_cheek_left",
    )
    vacuum_body.visual(
        Box((0.024, 0.014, 0.036)),
        origin=Origin(
            xyz=(shoulder_origin[0] - 0.010, -0.027, shoulder_origin[2]),
        ),
        material=hardware,
        name="shoulder_cheek_right",
    )
    vacuum_body.inertial = Inertial.from_geometry(
        Box((0.54, 0.38, 0.74)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, 0.37)),
    )

    upper_wand = model.part("upper_wand")
    upper_wand.visual(
        Cylinder(radius=0.015, length=0.040),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hardware,
        name="shoulder_collar",
    )
    upper_wand.visual(
        Box((0.040, 0.024, 0.034)),
        origin=Origin(xyz=_scale(upper_dir, 0.018), rpy=(0.0, upper_pitch, 0.0)),
        material=hardware,
        name="shoulder_knuckle",
    )
    upper_wand.visual(
        Cylinder(radius=0.019, length=0.048),
        origin=Origin(xyz=_scale(upper_dir, 0.052), rpy=(0.0, upper_pitch, 0.0)),
        material=seal,
        name="shoulder_boot",
    )
    upper_wand.visual(
        Cylinder(radius=0.020, length=0.245),
        origin=Origin(xyz=_scale(upper_dir, 0.160), rpy=(0.0, upper_pitch, 0.0)),
        material=wand_finish,
        name="upper_tube",
    )
    upper_wand.visual(
        Box((0.030, 0.030, 0.036)),
        origin=Origin(xyz=_scale(upper_dir, 0.262), rpy=(0.0, upper_pitch, 0.0)),
        material=hardware,
        name="upper_clamp",
    )
    upper_wand.visual(
        Cylinder(radius=0.005, length=0.065),
        origin=Origin(
            xyz=(upper_end[0] - upper_dir[0] * 0.030, 0.019, upper_end[2] - upper_dir[2] * 0.030),
            rpy=(0.0, upper_pitch, 0.0),
        ),
        material=hardware,
        name="elbow_rail_left",
    )
    upper_wand.visual(
        Cylinder(radius=0.005, length=0.065),
        origin=Origin(
            xyz=(upper_end[0] - upper_dir[0] * 0.030, -0.019, upper_end[2] - upper_dir[2] * 0.030),
            rpy=(0.0, upper_pitch, 0.0),
        ),
        material=hardware,
        name="elbow_rail_right",
    )
    upper_wand.visual(
        Box((0.018, 0.012, 0.040)),
        origin=Origin(
            xyz=(upper_end[0] - 0.008, 0.019, upper_end[2]),
        ),
        material=hardware,
        name="elbow_cheek_left",
    )
    upper_wand.visual(
        Box((0.018, 0.012, 0.040)),
        origin=Origin(
            xyz=(upper_end[0] - 0.008, -0.019, upper_end[2]),
        ),
        material=hardware,
        name="elbow_cheek_right",
    )
    upper_wand.inertial = Inertial.from_geometry(
        Box((0.34, 0.10, 0.22)),
        mass=1.5,
        origin=Origin(xyz=(0.12, 0.0, -0.12)),
    )

    lower_wand = model.part("lower_wand")
    lower_wand.visual(
        Cylinder(radius=0.015, length=0.040),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hardware,
        name="elbow_collar",
    )
    lower_wand.visual(
        Box((0.038, 0.024, 0.032)),
        origin=Origin(xyz=_scale(lower_dir, 0.018), rpy=(0.0, lower_pitch, 0.0)),
        material=hardware,
        name="elbow_knuckle",
    )
    lower_wand.visual(
        Cylinder(radius=0.018, length=0.048),
        origin=Origin(xyz=_scale(lower_dir, 0.052), rpy=(0.0, lower_pitch, 0.0)),
        material=seal,
        name="elbow_boot",
    )
    lower_wand.visual(
        Cylinder(radius=0.019, length=0.205),
        origin=Origin(xyz=_scale(lower_dir, 0.135), rpy=(0.0, lower_pitch, 0.0)),
        material=wand_finish,
        name="lower_tube",
    )
    lower_wand.visual(
        Box((0.028, 0.028, 0.034)),
        origin=Origin(xyz=_scale(lower_dir, 0.225), rpy=(0.0, lower_pitch, 0.0)),
        material=hardware,
        name="lower_clamp",
    )
    lower_wand.visual(
        Cylinder(radius=0.005, length=0.060),
        origin=Origin(
            xyz=(lower_end[0] - lower_dir[0] * 0.028, 0.019, lower_end[2] - lower_dir[2] * 0.028),
            rpy=(0.0, lower_pitch, 0.0),
        ),
        material=hardware,
        name="nozzle_rail_left",
    )
    lower_wand.visual(
        Cylinder(radius=0.005, length=0.060),
        origin=Origin(
            xyz=(lower_end[0] - lower_dir[0] * 0.028, -0.019, lower_end[2] - lower_dir[2] * 0.028),
            rpy=(0.0, lower_pitch, 0.0),
        ),
        material=hardware,
        name="nozzle_rail_right",
    )
    lower_wand.visual(
        Box((0.016, 0.012, 0.038)),
        origin=Origin(
            xyz=(lower_end[0] - 0.006, 0.019, lower_end[2]),
        ),
        material=hardware,
        name="nozzle_cheek_left",
    )
    lower_wand.visual(
        Box((0.016, 0.012, 0.038)),
        origin=Origin(
            xyz=(lower_end[0] - 0.006, -0.019, lower_end[2]),
        ),
        material=hardware,
        name="nozzle_cheek_right",
    )
    lower_wand.inertial = Inertial.from_geometry(
        Box((0.32, 0.09, 0.20)),
        mass=1.2,
        origin=Origin(xyz=(0.10, 0.0, -0.10)),
    )

    floor_nozzle = model.part("floor_nozzle")
    floor_nozzle.visual(
        Cylinder(radius=0.015, length=0.040),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hardware,
        name="nozzle_collar",
    )
    floor_nozzle.visual(
        Box((0.034, 0.026, 0.026)),
        origin=Origin(xyz=_scale(neck_dir, 0.012), rpy=(0.0, neck_pitch, 0.0)),
        material=hardware,
        name="neck_block",
    )
    floor_nozzle.visual(
        Cylinder(radius=0.018, length=0.045),
        origin=Origin(xyz=_scale(neck_dir, 0.035), rpy=(0.0, neck_pitch, 0.0)),
        material=seal,
        name="nozzle_boot",
    )
    floor_nozzle.visual(
        Cylinder(radius=0.018, length=0.055),
        origin=Origin(xyz=_scale(neck_dir, 0.062), rpy=(0.0, neck_pitch, 0.0)),
        material=wand_finish,
        name="nozzle_neck",
    )
    floor_nozzle.visual(
        _save_mesh(
            "floor_nozzle_shell",
            ExtrudeGeometry.centered(
                rounded_rect_profile(0.30, 0.095, 0.025, corner_segments=8),
                0.024,
            ),
        ),
        origin=Origin(xyz=(0.11, 0.0, -0.070)),
        material=shell,
        name="nozzle_shell",
    )
    floor_nozzle.visual(
        Box((0.28, 0.012, 0.008)),
        origin=Origin(xyz=(0.11, 0.0, -0.078)),
        material=seal,
        name="squeegee_strip",
    )
    floor_nozzle.visual(
        Box((0.18, 0.018, 0.008)),
        origin=Origin(xyz=(0.10, 0.036, -0.078)),
        material=hardware,
        name="left_skid",
    )
    floor_nozzle.visual(
        Box((0.18, 0.018, 0.008)),
        origin=Origin(xyz=(0.10, -0.036, -0.078)),
        material=hardware,
        name="right_skid",
    )
    floor_nozzle.visual(
        Box((0.05, 0.030, 0.022)),
        origin=Origin(xyz=(0.020, 0.0, -0.015)),
        material=cap_shell,
        name="nozzle_pivot_cover",
    )
    floor_nozzle.visual(
        Box((0.04, 0.095, 0.012)),
        origin=Origin(xyz=(0.235, 0.0, -0.066)),
        material=cap_shell,
        name="spray_lip",
    )
    floor_nozzle.inertial = Inertial.from_geometry(
        Box((0.34, 0.11, 0.12)),
        mass=1.7,
        origin=Origin(xyz=(0.11, 0.0, -0.055)),
    )

    model.articulation(
        "body_to_upper_wand",
        ArticulationType.REVOLUTE,
        parent=vacuum_body,
        child=upper_wand,
        origin=Origin(xyz=shoulder_origin),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=16.0,
            velocity=1.2,
            lower=-0.35,
            upper=0.95,
        ),
    )
    model.articulation(
        "upper_to_lower_wand",
        ArticulationType.REVOLUTE,
        parent=upper_wand,
        child=lower_wand,
        origin=Origin(xyz=upper_end),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.4,
            lower=-0.25,
            upper=1.05,
        ),
    )
    model.articulation(
        "lower_to_floor_nozzle",
        ArticulationType.REVOLUTE,
        parent=lower_wand,
        child=floor_nozzle,
        origin=Origin(xyz=lower_end),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.8,
            lower=-0.20,
            upper=0.55,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    vacuum_body = object_model.get_part("vacuum_body")
    upper_wand = object_model.get_part("upper_wand")
    lower_wand = object_model.get_part("lower_wand")
    floor_nozzle = object_model.get_part("floor_nozzle")

    shoulder = object_model.get_articulation("body_to_upper_wand")
    elbow = object_model.get_articulation("upper_to_lower_wand")
    nozzle_pitch = object_model.get_articulation("lower_to_floor_nozzle")

    nozzle_shell = floor_nozzle.get_visual("nozzle_shell")
    elbow_cheek_left = upper_wand.get_visual("elbow_cheek_left")
    elbow_cheek_right = upper_wand.get_visual("elbow_cheek_right")
    elbow_rail_left = upper_wand.get_visual("elbow_rail_left")
    elbow_rail_right = upper_wand.get_visual("elbow_rail_right")
    elbow_collar = lower_wand.get_visual("elbow_collar")
    nozzle_cheek_left = lower_wand.get_visual("nozzle_cheek_left")
    nozzle_cheek_right = lower_wand.get_visual("nozzle_cheek_right")
    nozzle_rail_left = lower_wand.get_visual("nozzle_rail_left")
    nozzle_rail_right = lower_wand.get_visual("nozzle_rail_right")
    nozzle_collar = floor_nozzle.get_visual("nozzle_collar")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.allow_overlap(
        lower_wand,
        upper_wand,
        elem_a=elbow_collar,
        elem_b=elbow_cheek_left,
        reason="Protected elbow clevis cheek intentionally nests over the sealed collar.",
    )
    ctx.allow_overlap(
        lower_wand,
        upper_wand,
        elem_a=elbow_collar,
        elem_b=elbow_cheek_right,
        reason="Protected elbow clevis cheek intentionally nests over the sealed collar.",
    )
    ctx.allow_overlap(
        floor_nozzle,
        lower_wand,
        elem_a=nozzle_collar,
        elem_b=nozzle_cheek_left,
        reason="Protected nozzle clevis cheek intentionally wraps the collar for weather shielding.",
    )
    ctx.allow_overlap(
        floor_nozzle,
        lower_wand,
        elem_a=nozzle_collar,
        elem_b=nozzle_cheek_right,
        reason="Protected nozzle clevis cheek intentionally wraps the collar for weather shielding.",
    )
    ctx.allow_overlap(
        lower_wand,
        upper_wand,
        elem_a=elbow_collar,
        elem_b=elbow_rail_left,
        reason="Slim elbow guide rail intentionally tucks into the collar envelope as part of the sealed pivot guard.",
    )
    ctx.allow_overlap(
        lower_wand,
        upper_wand,
        elem_a=elbow_collar,
        elem_b=elbow_rail_right,
        reason="Slim elbow guide rail intentionally tucks into the collar envelope as part of the sealed pivot guard.",
    )
    ctx.allow_overlap(
        floor_nozzle,
        lower_wand,
        elem_a=nozzle_collar,
        elem_b=nozzle_rail_left,
        reason="Slim nozzle guide rail intentionally tucks into the collar envelope as part of the splash-protected hinge guard.",
    )
    ctx.allow_overlap(
        floor_nozzle,
        lower_wand,
        elem_a=nozzle_collar,
        elem_b=nozzle_rail_right,
        reason="Slim nozzle guide rail intentionally tucks into the collar envelope as part of the splash-protected hinge guard.",
    )

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
        vacuum_body,
        upper_wand,
        name="shoulder_mount_contacts_wand",
    )
    ctx.expect_contact(
        upper_wand,
        lower_wand,
        name="elbow_mount_contacts_lower_wand",
    )
    ctx.expect_contact(
        lower_wand,
        floor_nozzle,
        name="nozzle_mount_contacts_head",
    )
    ctx.expect_gap(
        floor_nozzle,
        vacuum_body,
        axis="x",
        min_gap=0.30,
        name="nozzle_projects_forward_of_body",
    )
    ctx.expect_overlap(
        floor_nozzle,
        vacuum_body,
        axes="y",
        min_overlap=0.09,
        name="nozzle_tracks_body_centerline",
    )

    rest_elbow = ctx.part_world_position(lower_wand)
    rest_nozzle = ctx.part_world_position(floor_nozzle)
    rest_nozzle_shell = ctx.part_element_world_aabb(floor_nozzle, elem=nozzle_shell)

    with ctx.pose({shoulder: shoulder.motion_limits.upper}):
        shoulder_elbow = ctx.part_world_position(lower_wand)
        shoulder_nozzle = ctx.part_world_position(floor_nozzle)
    ctx.check(
        "shoulder_joint_lifts_wand",
        rest_elbow is not None
        and shoulder_elbow is not None
        and shoulder_elbow[2] > rest_elbow[2] + 0.10,
        f"rest elbow={rest_elbow}, lifted elbow={shoulder_elbow}",
    )
    ctx.check(
        "shoulder_joint_lifts_nozzle",
        rest_nozzle is not None
        and shoulder_nozzle is not None
        and shoulder_nozzle[2] > rest_nozzle[2] + 0.18,
        f"rest nozzle={rest_nozzle}, lifted nozzle={shoulder_nozzle}",
    )

    with ctx.pose({elbow: elbow.motion_limits.upper}):
        elbow_nozzle = ctx.part_world_position(floor_nozzle)
    ctx.check(
        "elbow_joint_breaks_nozzle_upward",
        rest_nozzle is not None
        and elbow_nozzle is not None
        and elbow_nozzle[2] > rest_nozzle[2] + 0.08,
        f"rest nozzle={rest_nozzle}, elbow pose nozzle={elbow_nozzle}",
    )

    with ctx.pose({nozzle_pitch: nozzle_pitch.motion_limits.upper}):
        pitched_nozzle_shell = ctx.part_element_world_aabb(floor_nozzle, elem=nozzle_shell)
    ctx.check(
        "nozzle_pitch_raises_shell",
        rest_nozzle_shell is not None
        and pitched_nozzle_shell is not None
        and pitched_nozzle_shell[1][2] > rest_nozzle_shell[1][2] + 0.025,
        f"rest shell={rest_nozzle_shell}, pitched shell={pitched_nozzle_shell}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
