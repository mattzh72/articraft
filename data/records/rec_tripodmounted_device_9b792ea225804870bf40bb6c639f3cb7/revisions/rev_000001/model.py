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
    Sphere,
    TestContext,
    TestReport,
)


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_cylinder(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_member(part, a, b, radius: float, material, *, name: str | None = None) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="precision_tripod_alignment_device")

    carbon = model.material("carbon", rgba=(0.18, 0.19, 0.20, 1.0))
    machined_black = model.material("machined_black", rgba=(0.12, 0.13, 0.14, 1.0))
    anodized_black = model.material("anodized_black", rgba=(0.10, 0.10, 0.11, 1.0))
    aluminum = model.material("aluminum", rgba=(0.72, 0.74, 0.77, 1.0))
    dark_aluminum = model.material("dark_aluminum", rgba=(0.45, 0.47, 0.50, 1.0))
    rubber = model.material("rubber", rgba=(0.05, 0.05, 0.05, 1.0))
    glass = model.material("glass", rgba=(0.25, 0.55, 0.62, 0.45))
    amber = model.material("amber", rgba=(0.88, 0.62, 0.12, 1.0))
    signal_red = model.material("signal_red", rgba=(0.76, 0.19, 0.16, 1.0))

    tripod = model.part("tripod_base")
    tripod.inertial = Inertial.from_geometry(
        Box((0.96, 0.96, 1.30)),
        mass=5.8,
        origin=Origin(xyz=(0.0, 0.0, 0.65)),
    )

    tripod.visual(
        Cylinder(radius=0.072, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 1.170)),
        material=machined_black,
        name="crown",
    )
    tripod.visual(
        Box((0.180, 0.150, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 1.209)),
        material=dark_aluminum,
        name="head_plate",
    )
    tripod.visual(
        Cylinder(radius=0.042, length=0.042),
        origin=Origin(xyz=(0.0, 0.0, 1.239)),
        material=machined_black,
        name="head_spigot",
    )
    tripod.visual(
        Cylinder(radius=0.018, length=0.010),
        origin=Origin(xyz=(0.034, -0.024, 1.223)),
        material=glass,
        name="bubble_level",
    )
    tripod.visual(
        Box((0.016, 0.004, 0.004)),
        origin=Origin(xyz=(0.050, 0.000, 1.258)),
        material=amber,
        name="pan_zero_mark",
    )
    tripod.visual(
        Box((0.010, 0.004, 0.004)),
        origin=Origin(xyz=(0.040, 0.012, 1.258)),
        material=amber,
        name="pan_plus_mark",
    )
    tripod.visual(
        Box((0.010, 0.004, 0.004)),
        origin=Origin(xyz=(0.040, -0.012, 1.258)),
        material=amber,
        name="pan_minus_mark",
    )

    spreader_hub_z = 0.48
    tripod.visual(
        Cylinder(radius=0.032, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, spreader_hub_z)),
        material=dark_aluminum,
        name="spreader_hub",
    )
    tripod.visual(
        Cylinder(radius=0.014, length=0.645),
        origin=Origin(xyz=(0.0, 0.0, 0.8175)),
        material=dark_aluminum,
        name="center_column",
    )

    for index in range(3):
        angle = (2.0 * math.pi * index) / 3.0 + math.pi / 6.0
        c = math.cos(angle)
        s = math.sin(angle)
        top = (0.070 * c, 0.070 * s, 1.160)
        knee = (0.220 * c, 0.220 * s, 0.660)
        foot = (0.430 * c, 0.430 * s, 0.020)
        spreader_t = 0.34
        spreader_attach = (
            knee[0] + (foot[0] - knee[0]) * spreader_t,
            knee[1] + (foot[1] - knee[1]) * spreader_t,
            knee[2] + (foot[2] - knee[2]) * spreader_t,
        )

        _add_member(
            tripod,
            top,
            knee,
            0.016,
            carbon,
            name=f"upper_leg_{index}",
        )
        _add_member(
            tripod,
            knee,
            foot,
            0.013,
            carbon,
            name=f"lower_leg_{index}",
        )
        tripod.visual(
            Sphere(radius=0.024),
            origin=Origin(xyz=knee),
            material=machined_black,
            name=f"leg_lock_{index}",
        )
        tripod.visual(
            Sphere(radius=0.020),
            origin=Origin(xyz=foot),
            material=rubber,
            name=f"foot_{index}",
        )
        _add_member(
            tripod,
            (0.030 * c, 0.030 * s, spreader_hub_z),
            spreader_attach,
            0.007,
            dark_aluminum,
            name=f"spreader_{index}",
        )
        tripod.visual(
            Sphere(radius=0.012),
            origin=Origin(xyz=spreader_attach),
            material=machined_black,
            name=f"spreader_joint_{index}",
        )

    pan_head = model.part("pan_head")
    pan_head.inertial = Inertial.from_geometry(
        Box((0.30, 0.36, 0.22)),
        mass=1.2,
        origin=Origin(xyz=(0.02, 0.0, 0.09)),
    )
    pan_head.visual(
        Cylinder(radius=0.060, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=anodized_black,
        name="pan_base",
    )
    pan_head.visual(
        Box((0.100, 0.080, 0.040)),
        origin=Origin(xyz=(0.015, 0.0, 0.048)),
        material=machined_black,
        name="pan_body",
    )
    pan_head.visual(
        Box((0.070, 0.080, 0.018)),
        origin=Origin(xyz=(0.075, 0.0, 0.060)),
        material=machined_black,
        name="front_bridge",
    )
    pan_head.visual(
        Box((0.050, 0.060, 0.040)),
        origin=Origin(xyz=(-0.005, 0.0, 0.082)),
        material=machined_black,
        name="rear_web",
    )
    pan_head.visual(
        Box((0.050, 0.012, 0.120)),
        origin=Origin(xyz=(0.085, -0.036, 0.090)),
        material=machined_black,
        name="left_yoke",
    )
    pan_head.visual(
        Box((0.050, 0.012, 0.120)),
        origin=Origin(xyz=(0.085, 0.036, 0.090)),
        material=machined_black,
        name="right_yoke",
    )
    pan_head.visual(
        Cylinder(radius=0.022, length=0.012),
        origin=Origin(xyz=(0.090, -0.036, 0.090), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_aluminum,
        name="left_bearing",
    )
    pan_head.visual(
        Cylinder(radius=0.022, length=0.012),
        origin=Origin(xyz=(0.090, 0.036, 0.090), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_aluminum,
        name="right_bearing",
    )
    pan_head.visual(
        Cylinder(radius=0.008, length=0.180),
        origin=Origin(xyz=(-0.015, 0.125, 0.045), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_aluminum,
        name="pan_handle",
    )
    pan_head.visual(
        Cylinder(radius=0.013, length=0.070),
        origin=Origin(xyz=(-0.015, 0.250, 0.045), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="pan_grip",
    )
    pan_head.visual(
        Cylinder(radius=0.015, length=0.020),
        origin=Origin(xyz=(-0.030, -0.049, 0.050), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_aluminum,
        name="pan_drag_knob",
    )
    pan_head.visual(
        Box((0.012, 0.006, 0.003)),
        origin=Origin(xyz=(0.060, 0.0, 0.0275)),
        material=signal_red,
        name="pan_pointer",
    )
    pan_head.visual(
        Box((0.018, 0.016, 0.064)),
        origin=Origin(xyz=(0.040, -0.048, 0.082)),
        material=amber,
        name="tilt_scale_plate",
    )
    for mark_index, z_pos in enumerate((0.060, 0.090, 0.120)):
        pan_head.visual(
            Box((0.010, 0.004, 0.003)),
            origin=Origin(xyz=(0.040, -0.048, z_pos - 0.008)),
            material=machined_black,
            name=f"tilt_tick_{mark_index}",
        )

    tilt_stage = model.part("tilt_stage")
    tilt_stage.inertial = Inertial.from_geometry(
        Box((0.22, 0.16, 0.14)),
        mass=0.85,
        origin=Origin(xyz=(0.08, 0.0, 0.01)),
    )
    tilt_stage.visual(
        Cylinder(radius=0.016, length=0.060),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_aluminum,
        name="trunnion_shaft",
    )
    tilt_stage.visual(
        Box((0.045, 0.060, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=aluminum,
        name="tilt_block",
    )
    tilt_stage.visual(
        Box((0.160, 0.050, 0.020)),
        origin=Origin(xyz=(0.080, 0.0, -0.010)),
        material=aluminum,
        name="tilt_arm",
    )
    tilt_stage.visual(
        Box((0.090, 0.060, 0.008)),
        origin=Origin(xyz=(0.080, 0.0, 0.004)),
        material=dark_aluminum,
        name="quick_release_plate",
    )
    tilt_stage.visual(
        Cylinder(radius=0.014, length=0.030),
        origin=Origin(xyz=(0.040, 0.044, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_aluminum,
        name="tilt_lock_knob",
    )

    bracket = model.part("device_bracket")
    bracket.inertial = Inertial.from_geometry(
        Box((0.17, 0.15, 0.15)),
        mass=0.7,
        origin=Origin(xyz=(0.08, 0.0, 0.07)),
    )
    bracket.visual(
        Box((0.100, 0.060, 0.012)),
        origin=Origin(xyz=(0.050, 0.0, 0.006)),
        material=aluminum,
        name="mount_shoe",
    )
    bracket.visual(
        Box((0.160, 0.120, 0.012)),
        origin=Origin(xyz=(0.080, 0.0, 0.018)),
        material=aluminum,
        name="base_datum",
    )
    bracket.visual(
        Box((0.008, 0.140, 0.120)),
        origin=Origin(xyz=(-0.004, 0.0, 0.072)),
        material=aluminum,
        name="back_datum",
    )
    bracket.visual(
        Box((0.130, 0.010, 0.060)),
        origin=Origin(xyz=(0.065, -0.050, 0.054)),
        material=dark_aluminum,
        name="side_datum",
    )
    bracket.visual(
        Box((0.016, 0.044, 0.112)),
        origin=Origin(xyz=(0.008, 0.082, 0.076)),
        material=dark_aluminum,
        name="guide_support",
    )
    bracket.visual(
        Box((0.130, 0.014, 0.014)),
        origin=Origin(xyz=(0.070, 0.082, 0.138)),
        material=dark_aluminum,
        name="guide_rail",
    )
    bracket.visual(
        Box((0.030, 0.120, 0.012)),
        origin=Origin(xyz=(0.145, 0.0, 0.006)),
        material=dark_aluminum,
        name="front_lip",
    )
    for mark_index, x_pos in enumerate((0.095, 0.110, 0.125)):
        bracket.visual(
            Box((0.003, 0.008, 0.012)),
            origin=Origin(xyz=(x_pos, 0.082, 0.126)),
            material=amber,
            name=f"clamp_scale_mark_{mark_index}",
        )

    device = model.part("alignment_device")
    device.inertial = Inertial.from_geometry(
        Box((0.190, 0.100, 0.095)),
        mass=1.6,
        origin=Origin(xyz=(0.095, 0.0, 0.0475)),
    )
    device.visual(
        Box((0.180, 0.090, 0.090)),
        origin=Origin(xyz=(0.090, 0.0, 0.045)),
        material=machined_black,
        name="housing_shell",
    )
    device.visual(
        Cylinder(radius=0.020, length=0.008),
        origin=Origin(xyz=(0.184, 0.0, 0.045), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_aluminum,
        name="front_bezel",
    )
    device.visual(
        Cylinder(radius=0.013, length=0.003),
        origin=Origin(xyz=(0.1895, 0.0, 0.045), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=glass,
        name="aperture_glass",
    )
    device.visual(
        Box((0.060, 0.020, 0.006)),
        origin=Origin(xyz=(0.060, 0.0, 0.093)),
        material=aluminum,
        name="top_datum_pad",
    )
    device.visual(
        Box((0.040, 0.006, 0.040)),
        origin=Origin(xyz=(0.060, 0.0475, 0.050)),
        material=aluminum,
        name="side_datum_pad",
    )
    device.visual(
        Box((0.018, 0.003, 0.003)),
        origin=Origin(xyz=(0.060, 0.0, 0.0905)),
        material=amber,
        name="top_index_mark",
    )

    clamp_jaw = model.part("clamp_jaw")
    clamp_jaw.inertial = Inertial.from_geometry(
        Box((0.11, 0.10, 0.09)),
        mass=0.22,
        origin=Origin(xyz=(0.0, 0.050, 0.030)),
    )
    clamp_jaw.visual(
        Box((0.100, 0.010, 0.080)),
        origin=Origin(),
        material=aluminum,
        name="jaw_pad",
    )
    clamp_jaw.visual(
        Box((0.040, 0.050, 0.070)),
        origin=Origin(xyz=(0.0, 0.025, 0.020)),
        material=dark_aluminum,
        name="jaw_carriage",
    )
    clamp_jaw.visual(
        Cylinder(radius=0.006, length=0.060),
        origin=Origin(xyz=(0.0, 0.070, 0.055), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_aluminum,
        name="lead_screw_stub",
    )
    clamp_jaw.visual(
        Cylinder(radius=0.018, length=0.014),
        origin=Origin(xyz=(0.0, 0.100, 0.055), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=anodized_black,
        name="clamp_knob",
    )

    model.articulation(
        "tripod_to_pan",
        ArticulationType.CONTINUOUS,
        parent=tripod,
        child=pan_head,
        origin=Origin(xyz=(0.0, 0.0, 1.260)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.5),
    )
    model.articulation(
        "pan_to_tilt",
        ArticulationType.REVOLUTE,
        parent=pan_head,
        child=tilt_stage,
        origin=Origin(xyz=(0.090, 0.0, 0.090)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.0,
            lower=-0.30,
            upper=1.10,
        ),
    )
    model.articulation(
        "tilt_to_bracket",
        ArticulationType.FIXED,
        parent=tilt_stage,
        child=bracket,
        origin=Origin(xyz=(0.035, 0.0, 0.008)),
    )
    model.articulation(
        "bracket_to_device",
        ArticulationType.FIXED,
        parent=bracket,
        child=device,
        origin=Origin(xyz=(0.0, 0.0, 0.024)),
    )
    model.articulation(
        "bracket_to_jaw",
        ArticulationType.PRISMATIC,
        parent=bracket,
        child=clamp_jaw,
        origin=Origin(xyz=(0.090, 0.056, 0.069)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=0.025,
            lower=0.0,
            upper=0.006,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tripod = object_model.get_part("tripod_base")
    pan_head = object_model.get_part("pan_head")
    tilt_stage = object_model.get_part("tilt_stage")
    bracket = object_model.get_part("device_bracket")
    device = object_model.get_part("alignment_device")
    clamp_jaw = object_model.get_part("clamp_jaw")
    pan = object_model.get_articulation("tripod_to_pan")
    tilt = object_model.get_articulation("pan_to_tilt")
    clamp = object_model.get_articulation("bracket_to_jaw")

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
        "core_stack_present",
        all((tripod, pan_head, tilt_stage, bracket, device, clamp_jaw, pan, tilt, clamp)),
        "Tripod, head, bracket, clamp, and device parts plus articulations must exist.",
    )
    ctx.expect_contact(
        pan_head,
        tripod,
        elem_a="pan_base",
        elem_b="head_spigot",
        name="pan_stage_seated_on_tripod_spigot",
    )
    ctx.expect_contact(
        tilt_stage,
        pan_head,
        elem_a="trunnion_shaft",
        elem_b="left_bearing",
        name="left_trunnion_bearing_contact",
    )
    ctx.expect_contact(
        tilt_stage,
        pan_head,
        elem_a="trunnion_shaft",
        elem_b="right_bearing",
        name="right_trunnion_bearing_contact",
    )
    ctx.expect_contact(
        device,
        bracket,
        elem_a="housing_shell",
        elem_b="base_datum",
        name="device_seated_on_base_datum",
    )
    ctx.expect_contact(
        device,
        bracket,
        elem_a="housing_shell",
        elem_b="back_datum",
        name="device_referenced_to_back_datum",
    )
    ctx.expect_contact(
        device,
        bracket,
        elem_a="housing_shell",
        elem_b="side_datum",
        name="device_referenced_to_side_datum",
    )

    with ctx.pose({clamp: 0.0}):
        ctx.expect_gap(
            clamp_jaw,
            device,
            axis="y",
            min_gap=0.005,
            max_gap=0.007,
            positive_elem="jaw_pad",
            negative_elem="housing_shell",
            name="clamp_open_gap_is_controlled",
        )

    with ctx.pose({clamp: 0.006}):
        ctx.expect_contact(
            clamp_jaw,
            device,
            elem_a="jaw_pad",
            elem_b="housing_shell",
            name="clamp_can_close_to_device_side",
        )

    rest_pos = ctx.part_world_position(device)
    rest_bezel = ctx.part_element_world_aabb(device, elem="front_bezel")
    with ctx.pose({tilt: math.radians(40.0)}):
        tilted_pos = ctx.part_world_position(device)
        tilted_bezel = ctx.part_element_world_aabb(device, elem="front_bezel")
    ctx.check(
        "positive_tilt_raises_device",
        rest_bezel is not None
        and tilted_bezel is not None
        and tilted_bezel[1][2] > rest_bezel[1][2] + 0.08,
        "Positive tilt should raise the device nose clearly upward in the tilt-open pose.",
    )

    with ctx.pose({pan: math.radians(60.0)}):
        panned_pos = ctx.part_world_position(device)
    ctx.check(
        "pan_rotates_about_vertical_axis",
        rest_pos is not None
        and panned_pos is not None
        and abs(panned_pos[2] - rest_pos[2]) < 1e-4
        and abs(panned_pos[1]) > 0.08,
        "Pan motion should swing the device around the vertical axis without changing height.",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
