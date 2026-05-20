from __future__ import annotations

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
)


def _cylinder_between(part, name, p0, p1, radius, material):
    """Add a URDF-Z cylinder whose axis runs from p0 to p1."""
    dx = p1[0] - p0[0]
    dy = p1[1] - p0[1]
    dz = p1[2] - p0[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(math.sqrt(dx * dx + dy * dy), dz)
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(
            xyz=((p0[0] + p1[0]) * 0.5, (p0[1] + p1[1]) * 0.5, (p0[2] + p1[2]) * 0.5),
            rpy=(0.0, pitch, yaw),
        ),
        material=material,
        name=name,
    )


def _cylinder_x(part, name, radius, length, xyz, material):
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(0.0, math.pi / 2.0, 0.0)),
        material=material,
        name=name,
    )


def _cylinder_y(part, name, radius, length, xyz, material):
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="retrofit_tripod_device_mount")

    iron = model.material("aged_black_iron", rgba=(0.06, 0.065, 0.06, 1.0))
    olive = model.material("olive_drab_casting", rgba=(0.27, 0.31, 0.22, 1.0))
    steel = model.material("brushed_steel", rgba=(0.62, 0.64, 0.62, 1.0))
    brass = model.material("worn_brass", rgba=(0.82, 0.62, 0.28, 1.0))
    rubber = model.material("black_rubber", rgba=(0.01, 0.01, 0.012, 1.0))
    bakelite = model.material("dark_bakelite", rgba=(0.055, 0.035, 0.028, 1.0))
    hatch = model.material("faded_service_hatch", rgba=(0.18, 0.21, 0.18, 1.0))

    tripod = model.part("tripod")

    # Three old-school tubular legs, each seated into a cast central spider and
    # tied back with spreader struts so the tripod reads as a supported frame.
    leg_top_radius = 0.088
    leg_foot_radius = 0.72
    for index, angle in enumerate((math.pi / 2.0, math.pi / 2.0 + 2.0 * math.pi / 3.0, math.pi / 2.0 + 4.0 * math.pi / 3.0)):
        c = math.cos(angle)
        s = math.sin(angle)
        foot = (leg_foot_radius * c, leg_foot_radius * s, 0.030)
        top = (leg_top_radius * c, leg_top_radius * s, 0.940)
        _cylinder_between(tripod, f"leg_tube_{index}", foot, top, 0.024, iron)

        collar_low = (0.48 * c, 0.48 * s, 0.340)
        collar_high = (0.53 * c, 0.53 * s, 0.260)
        _cylinder_between(tripod, f"leg_clamp_band_{index}", collar_low, collar_high, 0.032, olive)

        tripod.visual(
            Cylinder(radius=0.078, length=0.035),
            origin=Origin(xyz=(foot[0], foot[1], 0.0175)),
            material=rubber,
            name=f"rubber_foot_{index}",
        )
        tripod.visual(
            Box((0.060, 0.045, 0.050)),
            origin=Origin(xyz=(0.106 * c, 0.106 * s, 0.930), rpy=(0.0, 0.0, angle)),
            material=olive,
            name=f"leg_socket_{index}",
        )

        # Low spreader braces connect each leg back to the central sleeve.
        _cylinder_between(
            tripod,
            f"spreader_brace_{index}",
            (0.25 * c, 0.25 * s, 0.315),
            (0.030 * c, 0.030 * s, 0.430),
            0.012,
            steel,
        )

    tripod.visual(
        Cylinder(radius=0.038, length=0.930),
        origin=Origin(xyz=(0.0, 0.0, 0.600)),
        material=steel,
        name="center_mast",
    )
    tripod.visual(
        Cylinder(radius=0.105, length=0.140),
        origin=Origin(xyz=(0.0, 0.0, 0.940)),
        material=olive,
        name="cast_spider",
    )
    tripod.visual(
        Cylinder(radius=0.078, length=0.090),
        origin=Origin(xyz=(0.0, 0.0, 1.105)),
        material=olive,
        name="bolted_adapter",
    )
    tripod.visual(
        Cylinder(radius=0.140, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 1.175)),
        material=steel,
        name="head_plate",
    )
    for index, angle in enumerate((0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0)):
        tripod.visual(
            Cylinder(radius=0.010, length=0.010),
            origin=Origin(xyz=(0.120 * math.cos(angle), 0.120 * math.sin(angle), 1.203)),
            material=brass,
            name=f"head_bolt_{index}",
        )

    # A pragmatic legacy service box is strapped to the mast and visibly bolted.
    tripod.visual(
        Box((0.180, 0.125, 0.220)),
        origin=Origin(xyz=(0.080, 0.0, 0.650)),
        material=olive,
        name="service_box",
    )
    tripod.visual(
        Box((0.014, 0.090, 0.145)),
        origin=Origin(xyz=(0.177, 0.0, 0.650)),
        material=hatch,
        name="service_hatch",
    )
    _cylinder_x(tripod, "hatch_pull", 0.008, 0.035, (0.198, 0.0, 0.650), brass)
    for index, (yy, zz) in enumerate(((-0.046, -0.064), (0.046, -0.064), (-0.046, 0.064), (0.046, 0.064))):
        _cylinder_x(tripod, f"hatch_screw_{index}", 0.005, 0.014, (0.188, yy, 0.650 + zz), brass)

    # The pan head is a real rotating head carried by the tripod head plate.
    pan_head = model.part("pan_head")
    pan_head.visual(
        Cylinder(radius=0.100, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material=iron,
        name="pan_disk",
    )
    pan_head.visual(
        Cylinder(radius=0.076, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
        material=steel,
        name="pan_adapter_ring",
    )
    for index, angle in enumerate((math.pi / 4.0, 3.0 * math.pi / 4.0, 5.0 * math.pi / 4.0, 7.0 * math.pi / 4.0)):
        pan_head.visual(
            Cylinder(radius=0.006, length=0.008),
            origin=Origin(xyz=(0.066 * math.cos(angle), 0.066 * math.sin(angle), 0.078)),
            material=brass,
            name=f"adapter_bolt_{index}",
        )

    pan_head.visual(
        Box((0.160, 0.028, 0.240)),
        origin=Origin(xyz=(0.020, -0.116, 0.160)),
        material=olive,
        name="yoke_arm_0",
    )
    _cylinder_between(
        pan_head,
        "yoke_reinforcement_0",
        (0.080, -0.116, 0.045),
        (-0.020, -0.116, 0.145),
        0.012,
        steel,
    )
    pan_head.visual(
        Box((0.160, 0.028, 0.240)),
        origin=Origin(xyz=(0.020, 0.116, 0.160)),
        material=olive,
        name="yoke_arm_1",
    )
    _cylinder_between(
        pan_head,
        "yoke_reinforcement_1",
        (0.080, 0.116, 0.045),
        (-0.020, 0.116, 0.145),
        0.012,
        steel,
    )
    pan_head.visual(
        Box((0.180, 0.260, 0.035)),
        origin=Origin(xyz=(0.020, 0.0, 0.055)),
        material=olive,
        name="yoke_base_bridge",
    )
    pan_head.visual(
        Box((0.085, 0.260, 0.026)),
        origin=Origin(xyz=(-0.024, 0.0, 0.282)),
        material=olive,
        name="yoke_top_bridge",
    )
    _cylinder_x(pan_head, "pan_handle_stem", 0.012, 0.300, (-0.205, 0.0, 0.058), steel)
    _cylinder_x(pan_head, "pan_handle_grip", 0.024, 0.090, (-0.400, 0.0, 0.058), bakelite)

    model.articulation(
        "tripod_to_pan_head",
        ArticulationType.REVOLUTE,
        parent=tripod,
        child=pan_head,
        origin=Origin(xyz=(0.0, 0.0, 1.200)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.5, lower=-math.pi, upper=math.pi),
    )

    # Separate lock knob for the pan friction collar.
    pan_knob = model.part("pan_knob")
    _cylinder_y(pan_knob, "pan_lock_knob", 0.032, 0.050, (0.0, -0.025, 0.0), bakelite)
    _cylinder_y(pan_knob, "pan_lock_stem", 0.010, 0.022, (0.0, -0.011, 0.0), brass)
    model.articulation(
        "pan_head_to_pan_knob",
        ArticulationType.CONTINUOUS,
        parent=pan_head,
        child=pan_knob,
        origin=Origin(xyz=(0.0, -0.130, 0.048)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.5, velocity=6.0),
    )

    # Tilt cradle suspended between the yoke arms, carrying the device bracket.
    tilt_cradle = model.part("tilt_cradle")
    tilt_cradle.visual(
        Cylinder(radius=0.040, length=0.204),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="tilt_barrel",
    )
    tilt_cradle.visual(
        Box((0.165, 0.130, 0.070)),
        origin=Origin(xyz=(0.083, 0.0, 0.000)),
        material=olive,
        name="barrel_saddle",
    )
    tilt_cradle.visual(
        Box((0.035, 0.240, 0.320)),
        origin=Origin(xyz=(0.180, 0.0, 0.020)),
        material=olive,
        name="device_backplate",
    )
    tilt_cradle.visual(
        Box((0.100, 0.225, 0.035)),
        origin=Origin(xyz=(0.225, 0.0, -0.130)),
        material=iron,
        name="fixed_lower_jaw",
    )
    tilt_cradle.visual(
        Box((0.014, 0.195, 0.022)),
        origin=Origin(xyz=(0.282, 0.0, -0.108)),
        material=rubber,
        name="lower_rubber_pad",
    )
    tilt_cradle.visual(
        Box((0.015, 0.015, 0.310)),
        origin=Origin(xyz=(0.205, -0.116, 0.035)),
        material=steel,
        name="guide_rail_0",
    )
    tilt_cradle.visual(
        Box((0.085, 0.018, 0.130)),
        origin=Origin(xyz=(0.205, -0.116, -0.080)),
        material=olive,
        name="lower_gusset_0",
    )
    tilt_cradle.visual(
        Box((0.015, 0.015, 0.310)),
        origin=Origin(xyz=(0.205, 0.116, 0.035)),
        material=steel,
        name="guide_rail_1",
    )
    tilt_cradle.visual(
        Box((0.085, 0.018, 0.130)),
        origin=Origin(xyz=(0.205, 0.116, -0.080)),
        material=olive,
        name="lower_gusset_1",
    )
    tilt_cradle.visual(
        Box((0.010, 0.110, 0.090)),
        origin=Origin(xyz=(0.158, 0.0, 0.030)),
        material=hatch,
        name="bracket_hatch",
    )
    for index, (yy, zz) in enumerate(((-0.045, -0.032), (0.045, -0.032), (-0.045, 0.032), (0.045, 0.032))):
        _cylinder_x(tilt_cradle, f"bracket_hatch_screw_{index}", 0.0045, 0.011, (0.151, yy, 0.030 + zz), brass)

    model.articulation(
        "pan_head_to_tilt_cradle",
        ArticulationType.REVOLUTE,
        parent=pan_head,
        child=tilt_cradle,
        origin=Origin(xyz=(0.0, 0.0, 0.160)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=14.0, velocity=1.2, lower=-0.45, upper=0.75),
    )

    tilt_knob = model.part("tilt_knob")
    _cylinder_y(tilt_knob, "tilt_lock_knob", 0.038, 0.058, (0.0, 0.029, 0.0), bakelite)
    _cylinder_y(tilt_knob, "tilt_lock_washer", 0.044, 0.010, (0.0, 0.005, 0.0), brass)
    model.articulation(
        "pan_head_to_tilt_knob",
        ArticulationType.CONTINUOUS,
        parent=pan_head,
        child=tilt_knob,
        origin=Origin(xyz=(0.0, 0.130, 0.160)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=5.0),
    )

    # Sliding upper clamp jaw for the adjustable device bracket.
    clamp_jaw = model.part("clamp_jaw")
    clamp_jaw.visual(
        Box((0.100, 0.190, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=iron,
        name="upper_jaw",
    )
    clamp_jaw.visual(
        Box((0.014, 0.170, 0.020)),
        origin=Origin(xyz=(0.057, 0.0, -0.020)),
        material=rubber,
        name="upper_rubber_pad",
    )
    clamp_jaw.visual(
        Box((0.045, 0.018, 0.095)),
        origin=Origin(xyz=(-0.0425, -0.0995, -0.010)),
        material=olive,
        name="jaw_side_ear_0",
    )
    clamp_jaw.visual(
        Box((0.045, 0.018, 0.095)),
        origin=Origin(xyz=(-0.0425, 0.0995, -0.010)),
        material=olive,
        name="jaw_side_ear_1",
    )

    model.articulation(
        "tilt_cradle_to_clamp_jaw",
        ArticulationType.PRISMATIC,
        parent=tilt_cradle,
        child=clamp_jaw,
        origin=Origin(xyz=(0.270, 0.0, 0.070)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=0.18, lower=0.0, upper=0.120),
    )

    clamp_knob = model.part("clamp_knob")
    clamp_knob.visual(
        Cylinder(radius=0.032, length=0.042),
        origin=Origin(xyz=(0.0, 0.0, 0.021)),
        material=bakelite,
        name="clamp_knob_cap",
    )
    clamp_knob.visual(
        Cylinder(radius=0.009, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, -0.011)),
        material=brass,
        name="clamp_screw_tip",
    )
    model.articulation(
        "clamp_jaw_to_clamp_knob",
        ArticulationType.CONTINUOUS,
        parent=clamp_jaw,
        child=clamp_knob,
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=7.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.
    # If overlap QC reports an intersection, classify it first: intentional
    # embeddings or nested fits should get a scoped allowance; unintended
    # collisions should be fixed in geometry, support, mount, or pose.

    tripod = object_model.get_part("tripod")
    pan_head = object_model.get_part("pan_head")
    tilt_cradle = object_model.get_part("tilt_cradle")
    clamp_jaw = object_model.get_part("clamp_jaw")
    pan_joint = object_model.get_articulation("tripod_to_pan_head")
    tilt_joint = object_model.get_articulation("pan_head_to_tilt_cradle")
    clamp_slide = object_model.get_articulation("tilt_cradle_to_clamp_jaw")

    ctx.allow_overlap(
        clamp_jaw,
        object_model.get_part("clamp_knob"),
        elem_a="upper_jaw",
        elem_b="clamp_screw_tip",
        reason="The clamp screw tip is intentionally threaded into the upper jaw casting.",
    )
    ctx.expect_within(
        object_model.get_part("clamp_knob"),
        clamp_jaw,
        axes="xy",
        inner_elem="clamp_screw_tip",
        outer_elem="upper_jaw",
        margin=0.001,
        name="clamp screw is centered in the upper jaw",
    )
    ctx.expect_overlap(
        clamp_jaw,
        object_model.get_part("clamp_knob"),
        axes="z",
        elem_a="upper_jaw",
        elem_b="clamp_screw_tip",
        min_overlap=0.015,
        name="clamp screw is locally captured in the jaw",
    )
    ctx.expect_contact(
        pan_head,
        tripod,
        elem_a="pan_disk",
        elem_b="head_plate",
        contact_tol=0.001,
        name="pan disk is seated on the tripod head plate",
    )
    ctx.expect_contact(
        tilt_cradle,
        pan_head,
        elem_a="tilt_barrel",
        elem_b="yoke_arm_0",
        contact_tol=0.001,
        name="tilt barrel bears on one yoke cheek",
    )
    ctx.expect_contact(
        tilt_cradle,
        pan_head,
        elem_a="tilt_barrel",
        elem_b="yoke_arm_1",
        contact_tol=0.001,
        name="tilt barrel bears on the opposite yoke cheek",
    )
    ctx.expect_overlap(
        clamp_jaw,
        tilt_cradle,
        axes="z",
        elem_a="jaw_side_ear_0",
        elem_b="guide_rail_0",
        min_overlap=0.05,
        name="upper jaw guide ear remains engaged with rail at rest",
    )

    rest_clamp_aabb = ctx.part_world_aabb(clamp_jaw)
    with ctx.pose({clamp_slide: 0.120}):
        raised_clamp_aabb = ctx.part_world_aabb(clamp_jaw)
        ctx.expect_overlap(
            clamp_jaw,
            tilt_cradle,
            axes="z",
            elem_a="jaw_side_ear_0",
            elem_b="guide_rail_0",
            min_overlap=0.05,
            name="upper jaw guide ear remains engaged with rail when raised",
        )
    ctx.check(
        "clamp slide raises the upper jaw",
        rest_clamp_aabb is not None
        and raised_clamp_aabb is not None
        and raised_clamp_aabb[0][2] > rest_clamp_aabb[0][2] + 0.09,
        details=f"rest={rest_clamp_aabb}, raised={raised_clamp_aabb}",
    )

    rest_tilt_aabb = ctx.part_world_aabb(tilt_cradle)
    with ctx.pose({tilt_joint: 0.50}):
        raised_tilt_aabb = ctx.part_world_aabb(tilt_cradle)
    ctx.check(
        "positive tilt lifts the device bracket",
        rest_tilt_aabb is not None
        and raised_tilt_aabb is not None
        and raised_tilt_aabb[1][2] > rest_tilt_aabb[1][2] + 0.03,
        details=f"rest={rest_tilt_aabb}, tilted={raised_tilt_aabb}",
    )

    with ctx.pose({pan_joint: 0.65}):
        ctx.expect_contact(
            pan_head,
            tripod,
            elem_a="pan_disk",
            elem_b="head_plate",
            contact_tol=0.001,
            name="pan head stays seated while yawed",
        )

    return ctx.report()


object_model = build_object_model()
