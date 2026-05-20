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
    sweep_profile_along_spline,
    tube_from_spline_points,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _circle_profile(radius: float, *, segments: int = 28) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos((2.0 * math.pi * index) / segments),
            radius * math.sin((2.0 * math.pi * index) / segments),
        )
        for index in range(segments)
    ]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="retrofit_tripod_device")

    cast_olive = model.material("cast_olive", rgba=(0.34, 0.38, 0.32, 1.0))
    black_oxide = model.material("black_oxide", rgba=(0.11, 0.12, 0.13, 1.0))
    parkerized = model.material("parkerized", rgba=(0.28, 0.30, 0.31, 1.0))
    aged_wood = model.material("aged_wood", rgba=(0.44, 0.31, 0.20, 1.0))
    brass = model.material("brass", rgba=(0.63, 0.53, 0.28, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.09, 1.0))
    glass = model.material("sensor_glass", rgba=(0.16, 0.30, 0.34, 0.45))

    leg_profile = rounded_rect_profile(0.058, 0.030, radius=0.006, corner_segments=6)
    leg_mesh = _mesh(
        "tripod_leg",
        sweep_profile_along_spline(
            [
                (0.074, 0.0, 1.132),
                (0.150, 0.0, 0.905),
                (0.252, 0.0, 0.590),
                (0.388, 0.0, 0.085),
            ],
            profile=leg_profile,
            samples_per_segment=16,
            cap_profile=True,
        ),
    )
    spreader_mesh = _mesh(
        "spreader_arm",
        tube_from_spline_points(
            [
                (0.000, 0.0, 0.560),
                (0.132, 0.0, 0.558),
                (0.255, 0.0, 0.576),
            ],
            radius=0.009,
            samples_per_segment=12,
            radial_segments=14,
            cap_ends=True,
        ),
    )
    crown_brace_mesh = _mesh(
        "crown_brace",
        tube_from_spline_points(
            [
                (0.040, 0.0, 1.108),
                (0.086, 0.0, 1.020),
                (0.132, 0.0, 0.912),
            ],
            radius=0.0075,
            samples_per_segment=10,
            radial_segments=12,
            cap_ends=True,
        ),
    )
    hood_mesh = _mesh(
        "device_hood",
        ExtrudeWithHolesGeometry(
            _circle_profile(0.036, segments=32),
            [_circle_profile(0.026, segments=32)],
            0.060,
            cap=True,
            center=False,
            closed=True,
        ),
    )
    clamp_sleeve_mesh = _mesh(
        "clamp_sleeve",
        ExtrudeWithHolesGeometry(
            _circle_profile(0.012, segments=28),
            [_circle_profile(0.0068, segments=28)],
            0.030,
            cap=True,
            center=True,
            closed=True,
        ),
    )
    right_gusset_mesh = _mesh(
        "right_gusset",
        tube_from_spline_points(
            [(0.055, 0.060, -0.010), (0.120, 0.066, -0.032), (0.192, 0.072, -0.046)],
            radius=0.007,
            samples_per_segment=10,
            radial_segments=12,
            cap_ends=True,
        ),
    )
    left_gusset_mesh = _mesh(
        "left_gusset",
        tube_from_spline_points(
            [(0.055, -0.060, -0.010), (0.120, -0.066, -0.032), (0.192, -0.072, -0.046)],
            radius=0.007,
            samples_per_segment=10,
            radial_segments=12,
            cap_ends=True,
        ),
    )

    tripod_base = model.part("tripod_base")
    tripod_base.visual(
        Cylinder(radius=0.068, length=0.090),
        origin=Origin(xyz=(0.0, 0.0, 1.177)),
        material=cast_olive,
        name="tripod_crown",
    )
    tripod_base.visual(
        Cylinder(radius=0.092, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 1.231)),
        material=parkerized,
        name="head_plate",
    )
    tripod_base.visual(
        Cylinder(radius=0.040, length=0.080),
        origin=Origin(xyz=(0.0, 0.0, 1.138)),
        material=black_oxide,
        name="center_spigot",
    )
    tripod_base.visual(
        Cylinder(radius=0.034, length=0.048),
        origin=Origin(xyz=(0.0, 0.0, 0.560)),
        material=parkerized,
        name="spreader_hub",
    )
    for index, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        tripod_base.visual(
            leg_mesh,
            origin=Origin(rpy=(0.0, 0.0, angle)),
            material=aged_wood,
            name=f"leg_{index}",
        )
        tripod_base.visual(
            Box((0.090, 0.042, 0.046)),
            origin=Origin(xyz=(0.082 * math.cos(angle), 0.082 * math.sin(angle), 1.118), rpy=(0.0, 0.0, angle)),
            material=cast_olive,
            name=f"leg_adapter_{index}",
        )
        tripod_base.visual(
            Box((0.086, 0.038, 0.028)),
            origin=Origin(xyz=(0.362 * math.cos(angle), 0.362 * math.sin(angle), 0.068), rpy=(0.0, 0.0, angle)),
            material=rubber,
            name=f"foot_pad_{index}",
        )
        tripod_base.visual(
            spreader_mesh,
            origin=Origin(rpy=(0.0, 0.0, angle)),
            material=parkerized,
            name=f"spreader_{index}",
        )
        tripod_base.visual(
            crown_brace_mesh,
            origin=Origin(rpy=(0.0, 0.0, angle)),
            material=black_oxide,
            name=f"crown_brace_{index}",
        )
        tripod_base.visual(
            Cylinder(radius=0.006, length=0.020),
            origin=Origin(
                xyz=(0.096 * math.cos(angle), 0.096 * math.sin(angle), 1.144),
                rpy=(0.0, math.pi / 2.0, angle),
            ),
            material=brass,
            name=f"adapter_bolt_{index}",
        )
    tripod_base.inertial = Inertial.from_geometry(
        Box((0.92, 0.92, 1.26)),
        mass=8.5,
        origin=Origin(xyz=(0.0, 0.0, 0.63)),
    )

    pan_head = model.part("pan_head")
    pan_head.visual(
        Cylinder(radius=0.056, length=0.080),
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        material=black_oxide,
        name="pan_bearing_drum",
    )
    pan_head.visual(
        Cylinder(radius=0.078, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.066)),
        material=parkerized,
        name="pan_top_plate",
    )
    pan_head.visual(
        Box((0.090, 0.100, 0.080)),
        origin=Origin(xyz=(-0.028, 0.0, 0.048)),
        material=cast_olive,
        name="pan_body",
    )
    pan_head.visual(
        Box((0.024, 0.018, 0.120)),
        origin=Origin(xyz=(0.024, 0.059, 0.100)),
        material=cast_olive,
        name="left_cheek",
    )
    pan_head.visual(
        Box((0.024, 0.018, 0.120)),
        origin=Origin(xyz=(0.024, -0.059, 0.100)),
        material=cast_olive,
        name="right_cheek",
    )
    pan_head.visual(
        Box((0.030, 0.120, 0.018)),
        origin=Origin(xyz=(0.010, 0.0, 0.047)),
        material=parkerized,
        name="yoke_bridge",
    )
    pan_head.visual(
        Cylinder(radius=0.010, length=0.180),
        origin=Origin(xyz=(-0.125, -0.046, 0.052), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black_oxide,
        name="pan_handle",
    )
    pan_head.visual(
        Cylinder(radius=0.015, length=0.082),
        origin=Origin(xyz=(-0.206, -0.046, 0.052), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=aged_wood,
        name="pan_handle_grip",
    )
    pan_head.visual(
        Cylinder(radius=0.012, length=0.024),
        origin=Origin(xyz=(-0.050, 0.059, 0.044), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="pan_lock_knob",
    )
    pan_head.visual(
        Cylinder(radius=0.011, length=0.022),
        origin=Origin(xyz=(0.030, 0.079, 0.100), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="tilt_lock_knob_left",
    )
    pan_head.visual(
        Cylinder(radius=0.011, length=0.022),
        origin=Origin(xyz=(0.030, -0.079, 0.100), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="tilt_lock_knob_right",
    )
    pan_head.inertial = Inertial.from_geometry(
        Box((0.34, 0.18, 0.18)),
        mass=1.6,
        origin=Origin(xyz=(-0.02, 0.0, 0.09)),
    )

    tilt_head = model.part("tilt_head")
    tilt_head.visual(
        Cylinder(radius=0.012, length=0.100),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=black_oxide,
        name="trunnion_shaft",
    )
    tilt_head.visual(
        Box((0.070, 0.070, 0.030)),
        origin=Origin(xyz=(0.038, 0.0, 0.0)),
        material=cast_olive,
        name="trunnion_block",
    )
    tilt_head.visual(
        Box((0.044, 0.060, 0.050)),
        origin=Origin(xyz=(0.074, 0.0, -0.025)),
        material=cast_olive,
        name="trunnion_pedestal",
    )
    tilt_head.visual(
        Box((0.230, 0.140, 0.014)),
        origin=Origin(xyz=(0.190, 0.0, -0.052)),
        material=parkerized,
        name="bracket_tray",
    )
    tilt_head.visual(
        Box((0.014, 0.150, 0.055)),
        origin=Origin(xyz=(0.108, 0.0, -0.0185)),
        material=cast_olive,
        name="backstop_plate",
    )
    tilt_head.visual(
        Box((0.190, 0.012, 0.028)),
        origin=Origin(xyz=(0.190, -0.074, -0.033)),
        material=cast_olive,
        name="left_locating_rail",
    )
    tilt_head.visual(
        Box((0.110, 0.018, 0.022)),
        origin=Origin(xyz=(0.160, 0.050, -0.068)),
        material=parkerized,
        name="right_gusset",
    )
    tilt_head.visual(
        Box((0.110, 0.018, 0.022)),
        origin=Origin(xyz=(0.160, -0.050, -0.068)),
        material=parkerized,
        name="left_gusset",
    )
    tilt_head.inertial = Inertial.from_geometry(
        Box((0.22, 0.20, 0.18)),
        mass=1.9,
        origin=Origin(xyz=(0.110, 0.0, 0.010)),
    )

    device_module = model.part("device_module")
    device_module.visual(
        Box((0.180, 0.136, 0.014)),
        origin=Origin(xyz=(0.090, 0.0, 0.007)),
        material=parkerized,
        name="mount_adapter",
    )
    device_module.visual(
        Box((0.120, 0.014, 0.032)),
        origin=Origin(xyz=(0.098, 0.045, 0.030)),
        material=parkerized,
        name="adapter_rib_right",
    )
    device_module.visual(
        Box((0.120, 0.014, 0.032)),
        origin=Origin(xyz=(0.098, -0.045, 0.030)),
        material=parkerized,
        name="adapter_rib_left",
    )
    device_module.visual(
        Box((0.220, 0.124, 0.104)),
        origin=Origin(xyz=(0.195, 0.0, 0.066)),
        material=cast_olive,
        name="housing_shell",
    )
    device_module.visual(
        Box((0.055, 0.102, 0.084)),
        origin=Origin(xyz=(0.045, 0.0, 0.062)),
        material=black_oxide,
        name="rear_service_box",
    )
    device_module.visual(
        Box((0.118, 0.012, 0.066)),
        origin=Origin(xyz=(0.150, -0.067, 0.066)),
        material=parkerized,
        name="left_service_hatch",
    )
    device_module.visual(
        Box((0.118, 0.082, 0.012)),
        origin=Origin(xyz=(0.162, 0.0, 0.118)),
        material=parkerized,
        name="top_service_hatch",
    )
    device_module.visual(
        Box((0.054, 0.082, 0.012)),
        origin=Origin(xyz=(0.042, 0.0, 0.107)),
        material=parkerized,
        name="rear_hatch",
    )
    device_module.visual(
        Box((0.110, 0.020, 0.010)),
        origin=Origin(xyz=(0.160, 0.0, 0.123)),
        material=black_oxide,
        name="top_carry_rail",
    )
    device_module.visual(
        Box((0.028, 0.070, 0.070)),
        origin=Origin(xyz=(0.286, 0.0, 0.066)),
        material=black_oxide,
        name="front_adapter_collar",
    )
    device_module.visual(
        hood_mesh,
        origin=Origin(xyz=(0.300, 0.0, 0.066), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black_oxide,
        name="lens_hood",
    )
    device_module.visual(
        Cylinder(radius=0.025, length=0.004),
        origin=Origin(xyz=(0.302, 0.0, 0.066), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=glass,
        name="sensor_window",
    )
    for index, x_pos in enumerate((0.042, 0.138)):
        for side, y_pos in enumerate((-0.046, 0.046)):
            device_module.visual(
                Cylinder(radius=0.004, length=0.003),
                origin=Origin(xyz=(x_pos, y_pos, 0.0155)),
                material=brass,
                name=f"adapter_bolt_{index}_{side}",
            )
    for index, x_pos in enumerate((0.118, 0.194)):
        for z_pos in (0.043, 0.093):
            device_module.visual(
                Cylinder(radius=0.0032, length=0.012),
                origin=Origin(
                    xyz=(x_pos, -0.073, z_pos),
                    rpy=(-math.pi / 2.0, 0.0, 0.0),
                ),
                material=brass,
                name=f"left_hatch_bolt_{index}_{int(round(z_pos * 1000.0))}",
            )
    for x_pos in (0.118, 0.194):
        for y_pos in (-0.028, 0.028):
            device_module.visual(
                Cylinder(radius=0.0032, length=0.003),
                origin=Origin(xyz=(x_pos, y_pos, 0.1255)),
                material=brass,
                name=f"top_hatch_bolt_{int(round(x_pos * 1000.0))}_{int(round((y_pos + 0.03) * 1000.0))}",
            )
    device_module.inertial = Inertial.from_geometry(
        Box((0.350, 0.150, 0.150)),
        mass=2.6,
        origin=Origin(xyz=(0.135, 0.0, 0.075)),
    )

    clamp_jaw = model.part("clamp_jaw")
    clamp_jaw.visual(
        Box((0.030, 0.020, 0.024)),
        origin=Origin(xyz=(0.0, -0.006, 0.010)),
        material=parkerized,
        name="guide_carriage",
    )
    clamp_jaw.visual(
        Box((0.016, 0.018, 0.044)),
        origin=Origin(xyz=(0.0, -0.012, -0.004)),
        material=cast_olive,
        name="jaw_arm",
    )
    clamp_jaw.visual(
        Box((0.048, 0.010, 0.022)),
        origin=Origin(xyz=(-0.002, -0.024, -0.004)),
        material=black_oxide,
        name="jaw_pad",
    )
    clamp_jaw.visual(
        Box((0.040, 0.040, 0.006)),
        origin=Origin(xyz=(0.0, -0.014, -0.007)),
        material=rubber,
        name="guide_skid",
    )
    clamp_jaw.visual(
        Box((0.024, 0.020, 0.012)),
        origin=Origin(xyz=(0.0, 0.004, 0.016)),
        material=parkerized,
        name="knob_bridge",
    )
    clamp_jaw.visual(
        Cylinder(radius=0.012, length=0.020),
        origin=Origin(xyz=(0.0, 0.015, 0.016), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="clamp_knob",
    )
    clamp_jaw.inertial = Inertial.from_geometry(
        Box((0.070, 0.040, 0.150)),
        mass=0.45,
        origin=Origin(xyz=(0.0, -0.005, -0.068)),
    )

    model.articulation(
        "tripod_to_pan",
        ArticulationType.CONTINUOUS,
        parent=tripod_base,
        child=pan_head,
        origin=Origin(xyz=(0.0, 0.0, 1.240)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=40.0, velocity=1.2),
    )
    model.articulation(
        "pan_to_tilt",
        ArticulationType.REVOLUTE,
        parent=pan_head,
        child=tilt_head,
        origin=Origin(xyz=(0.030, 0.0, 0.100)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=1.0,
            lower=math.radians(-20.0),
            upper=math.radians(55.0),
        ),
    )
    model.articulation(
        "tilt_to_device",
        ArticulationType.FIXED,
        parent=tilt_head,
        child=device_module,
        origin=Origin(xyz=(0.115, 0.0, -0.045)),
    )
    model.articulation(
        "tilt_to_clamp",
        ArticulationType.PRISMATIC,
        parent=tilt_head,
        child=clamp_jaw,
        origin=Origin(xyz=(0.225, 0.102, -0.038)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=0.05,
            lower=0.0,
            upper=0.014,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tripod_base = object_model.get_part("tripod_base")
    pan_head = object_model.get_part("pan_head")
    tilt_head = object_model.get_part("tilt_head")
    device_module = object_model.get_part("device_module")
    clamp_jaw = object_model.get_part("clamp_jaw")
    pan_joint = object_model.get_articulation("tripod_to_pan")
    tilt_joint = object_model.get_articulation("pan_to_tilt")
    clamp_joint = object_model.get_articulation("tilt_to_clamp")

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
        "pan axis is vertical",
        tuple(round(value, 3) for value in pan_joint.axis) == (0.0, 0.0, 1.0),
        details=f"axis={pan_joint.axis}",
    )
    ctx.check(
        "tilt axis pitches upward",
        tuple(round(value, 3) for value in tilt_joint.axis) == (0.0, -1.0, 0.0),
        details=f"axis={tilt_joint.axis}",
    )
    ctx.check(
        "clamp axis closes inward",
        tuple(round(value, 3) for value in clamp_joint.axis) == (0.0, -1.0, 0.0),
        details=f"axis={clamp_joint.axis}",
    )

    ctx.expect_contact(
        pan_head,
        tripod_base,
        elem_a="pan_bearing_drum",
        elem_b="head_plate",
        name="pan drum seats on head plate",
    )
    ctx.expect_contact(
        tilt_head,
        pan_head,
        elem_a="trunnion_shaft",
        elem_b="left_cheek",
        name="left trunnion supported in yoke",
    )
    ctx.expect_contact(
        tilt_head,
        pan_head,
        elem_a="trunnion_shaft",
        elem_b="right_cheek",
        name="right trunnion supported in yoke",
    )
    ctx.expect_contact(
        device_module,
        tilt_head,
        elem_a="mount_adapter",
        elem_b="bracket_tray",
        name="device adapter sits on tray",
    )
    ctx.expect_contact(
        device_module,
        tilt_head,
        elem_a="mount_adapter",
        elem_b="backstop_plate",
        name="device adapter locates against backstop",
    )
    ctx.expect_contact(
        device_module,
        tilt_head,
        elem_a="mount_adapter",
        elem_b="left_locating_rail",
        name="device adapter bears on locating rail",
    )
    ctx.expect_contact(
        clamp_jaw,
        tilt_head,
        elem_a="guide_skid",
        elem_b="bracket_tray",
        name="clamp carriage supported on tray",
    )
    ctx.expect_gap(
        clamp_jaw,
        device_module,
        axis="y",
        min_gap=0.003,
        max_gap=0.007,
        positive_elem="jaw_pad",
        negative_elem="mount_adapter",
        name="clamp starts slightly open",
    )

    rest_device_pos = ctx.part_world_position(device_module)
    with ctx.pose({tilt_joint: math.radians(25.0)}):
        raised_device_pos = ctx.part_world_position(device_module)
    ctx.check(
        "tilt raises mounted device",
        rest_device_pos is not None
        and raised_device_pos is not None
        and raised_device_pos[2] > rest_device_pos[2] + 0.025,
        details=f"rest={rest_device_pos}, raised={raised_device_pos}",
    )

    with ctx.pose({clamp_joint: 0.005}):
        ctx.expect_gap(
            clamp_jaw,
            device_module,
            axis="y",
            max_gap=0.0015,
            max_penetration=1e-5,
            positive_elem="jaw_pad",
            negative_elem="mount_adapter",
            name="clamp can nearly seat against adapter",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
