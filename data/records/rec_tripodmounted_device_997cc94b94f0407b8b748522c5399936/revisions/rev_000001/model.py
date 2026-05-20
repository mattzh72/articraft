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


def _cylinder_between(part, start, end, radius, *, material, name):
    """Add a cylinder whose local Z axis spans start -> end in the part frame."""
    sx, sy, sz = start
    ex, ey, ez = end
    vx, vy, vz = ex - sx, ey - sy, ez - sz
    length = math.sqrt(vx * vx + vy * vy + vz * vz)
    nx, ny, nz = vx / length, vy / length, vz / length
    pitch = math.acos(max(-1.0, min(1.0, nz)))
    yaw = math.atan2(ny, nx)
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(
            xyz=((sx + ex) * 0.5, (sy + ey) * 0.5, (sz + ez) * 0.5),
            rpy=(0.0, pitch, yaw),
        ),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="weatherproof_tripod_device")

    anodized = model.material("dark_anodized_aluminum", rgba=(0.05, 0.055, 0.055, 1.0))
    tube = model.material("brushed_aluminum", rgba=(0.55, 0.58, 0.57, 1.0))
    rubber = model.material("black_epdm_rubber", rgba=(0.01, 0.01, 0.009, 1.0))
    stainless = model.material("stainless_hardware", rgba=(0.78, 0.76, 0.70, 1.0))
    device_shell = model.material("powder_coated_polycarbonate", rgba=(0.16, 0.18, 0.17, 1.0))
    gasket = model.material("deep_black_gasket", rgba=(0.0, 0.0, 0.0, 1.0))
    glass = model.material("smoked_glass_window", rgba=(0.03, 0.08, 0.10, 0.55))

    # Root: one connected tripod assembly with a hub, center column, braces, and
    # three splayed legs.  Dimensions are in meters and sized like a compact
    # field instrument tripod.
    tripod = model.part("tripod")
    tripod.visual(
        Cylinder(radius=0.030, length=0.730),
        origin=Origin(xyz=(0.0, 0.0, 0.425)),
        material=tube,
        name="center_column",
    )
    tripod.visual(
        Cylinder(radius=0.083, length=0.052),
        origin=Origin(xyz=(0.0, 0.0, 0.675)),
        material=anodized,
        name="spider_hub",
    )
    tripod.visual(
        Cylinder(radius=0.118, length=0.025),
        origin=Origin(xyz=(0.0, 0.0, 0.8025)),
        material=anodized,
        name="head_plate",
    )
    tripod.visual(
        Cylinder(radius=0.052, length=0.034),
        origin=Origin(xyz=(0.0, 0.0, 0.735)),
        material=anodized,
        name="column_clamp",
    )

    leg_top_z = 0.660
    foot_z = 0.055
    foot_radius = 0.64
    for index in range(3):
        angle = math.radians(90.0 + 120.0 * index)
        direction = (math.cos(angle), math.sin(angle), 0.0)
        hip = (0.055 * direction[0], 0.055 * direction[1], leg_top_z)
        knee = (0.34 * direction[0], 0.34 * direction[1], 0.345)
        foot = (foot_radius * direction[0], foot_radius * direction[1], foot_z)
        _cylinder_between(
            tripod,
            hip,
            knee,
            0.019,
            material=tube,
            name=f"leg_{index}_upper",
        )
        _cylinder_between(
            tripod,
            knee,
            foot,
            0.017,
            material=tube,
            name=f"leg_{index}_lower",
        )
        _cylinder_between(
            tripod,
            (0.0, 0.0, 0.415),
            (0.31 * direction[0], 0.31 * direction[1], 0.330),
            0.010,
            material=tube,
            name=f"brace_{index}",
        )
        _cylinder_between(
            tripod,
            (0.295 * direction[0], 0.295 * direction[1], 0.375),
            (0.375 * direction[0], 0.375 * direction[1], 0.290),
            0.024,
            material=anodized,
            name=f"leg_lock_{index}",
        )
        tripod.visual(
            Cylinder(radius=0.046, length=0.025),
            origin=Origin(
                xyz=(foot[0], foot[1], 0.0125),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=rubber,
            name=f"foot_{index}",
        )

    # A real pan head sits on the tripod plate.  The upper structure carries the
    # tilt yoke and its side controls, while the child tilt cradle holds the
    # weatherproof device and clamp.
    pan_head = model.part("pan_head")
    pan_head.visual(
        Cylinder(radius=0.130, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=anodized,
        name="drip_skirt",
    )
    pan_head.visual(
        Cylinder(radius=0.096, length=0.036),
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material=anodized,
        name="pan_turntable",
    )
    pan_head.visual(
        Box((0.285, 0.105, 0.034)),
        origin=Origin(xyz=(0.0, 0.0, 0.052)),
        material=anodized,
        name="yoke_base",
    )
    pan_head.visual(
        Box((0.034, 0.086, 0.172)),
        origin=Origin(xyz=(-0.116, 0.0, 0.148)),
        material=anodized,
        name="yoke_cheek_0",
    )
    pan_head.visual(
        Cylinder(radius=0.030, length=0.008),
        origin=Origin(xyz=(-0.116, 0.0, 0.170), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=stainless,
        name="tilt_boss_0",
    )
    pan_head.visual(
        Box((0.034, 0.086, 0.172)),
        origin=Origin(xyz=(0.116, 0.0, 0.148)),
        material=anodized,
        name="yoke_cheek_1",
    )
    pan_head.visual(
        Cylinder(radius=0.030, length=0.008),
        origin=Origin(xyz=(0.116, 0.0, 0.170), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=stainless,
        name="tilt_boss_1",
    )

    tilt_cradle = model.part("tilt_cradle")
    tilt_cradle.visual(
        Cylinder(radius=0.014, length=0.284),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=stainless,
        name="trunnion",
    )
    tilt_cradle.visual(
        Box((0.178, 0.030, 0.164)),
        origin=Origin(xyz=(0.0, 0.047, 0.0)),
        material=anodized,
        name="backplate",
    )
    tilt_cradle.visual(
        Box((0.178, 0.048, 0.030)),
        origin=Origin(xyz=(0.0, 0.020, 0.0)),
        material=anodized,
        name="trunnion_bridge",
    )
    tilt_cradle.visual(
        Box((0.202, 0.012, 0.142)),
        origin=Origin(xyz=(0.0, 0.068, 0.0)),
        material=rubber,
        name="back_pad",
    )
    tilt_cradle.visual(
        Box((0.242, 0.100, 0.018)),
        origin=Origin(xyz=(0.0, 0.104, -0.089)),
        material=anodized,
        name="lower_shelf",
    )
    tilt_cradle.visual(
        Box((0.018, 0.054, 0.160)),
        origin=Origin(xyz=(-0.119, 0.135, 0.0)),
        material=rubber,
        name="fixed_jaw",
    )
    tilt_cradle.visual(
        Box((0.360, 0.026, 0.066)),
        origin=Origin(xyz=(0.0, 0.057, 0.039)),
        material=anodized,
        name="rail_channel",
    )
    tilt_cradle.visual(
        Box((0.206, 0.018, 0.014)),
        origin=Origin(xyz=(0.0, 0.052, 0.106)),
        material=anodized,
        name="top_clamp_rail",
    )
    tilt_cradle.visual(
        Box((0.206, 0.012, 0.034)),
        origin=Origin(xyz=(0.0, 0.052, 0.091)),
        material=anodized,
        name="top_rail_web",
    )

    device = model.part("device")
    device.visual(
        Box((0.220, 0.090, 0.160)),
        origin=Origin(xyz=(0.0, 0.115, 0.0)),
        material=device_shell,
        name="sealed_body",
    )
    device.visual(
        Box((0.272, 0.132, 0.018)),
        origin=Origin(xyz=(0.0, 0.125, 0.089)),
        material=device_shell,
        name="rain_hood",
    )
    device.visual(
        Box((0.274, 0.012, 0.018)),
        origin=Origin(xyz=(0.0, 0.188, 0.071)),
        material=device_shell,
        name="front_drip_lip",
    )
    for side, x in enumerate((-0.136, 0.136)):
        device.visual(
            Box((0.012, 0.132, 0.016)),
            origin=Origin(xyz=(x, 0.125, 0.106)),
            material=device_shell,
            name=f"side_drip_{side}",
        )
    device.visual(
        Box((0.122, 0.006, 0.072)),
        origin=Origin(xyz=(0.0, 0.163, 0.005)),
        material=glass,
        name="front_window",
    )
    device.visual(
        Box((0.144, 0.008, 0.012)),
        origin=Origin(xyz=(0.0, 0.164, 0.043)),
        material=gasket,
        name="window_gasket_top",
    )
    device.visual(
        Box((0.144, 0.008, 0.012)),
        origin=Origin(xyz=(0.0, 0.164, -0.033)),
        material=gasket,
        name="window_gasket_bottom",
    )
    for side, x in enumerate((-0.071, 0.071)):
        device.visual(
            Box((0.012, 0.008, 0.084)),
            origin=Origin(xyz=(0.887 * x, 0.164, 0.005)),
            material=gasket,
            name=f"window_gasket_{side}",
        )
    device.visual(
        Box((0.224, 0.006, 0.008)),
        origin=Origin(xyz=(0.0, 0.161, 0.076)),
        material=gasket,
        name="lid_gasket_top",
    )
    device.visual(
        Box((0.224, 0.006, 0.008)),
        origin=Origin(xyz=(0.0, 0.161, -0.076)),
        material=gasket,
        name="lid_gasket_bottom",
    )
    for side, x in enumerate((-0.101, 0.101)):
        device.visual(
            Box((0.008, 0.006, 0.144)),
            origin=Origin(xyz=(x, 0.161, 0.0)),
            material=gasket,
            name=f"lid_gasket_{side}",
        )
    for index, (x, z) in enumerate(((-0.090, 0.055), (0.090, 0.055), (-0.090, -0.055), (0.090, -0.055))):
        device.visual(
            Cylinder(radius=0.0075, length=0.006),
            origin=Origin(xyz=(x, 0.164, z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=stainless,
            name=f"sealed_screw_{index}",
        )
    device.visual(
        Cylinder(radius=0.018, length=0.044),
        origin=Origin(xyz=(-0.070, 0.182, -0.045), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="cable_gland",
    )
    device.visual(
        Cylinder(radius=0.026, length=0.014),
        origin=Origin(xyz=(-0.070, 0.166, -0.045), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=stainless,
        name="gland_nut",
    )

    clamp_jaw = model.part("clamp_jaw")
    clamp_jaw.visual(
        Box((0.018, 0.054, 0.146)),
        origin=Origin(xyz=(0.009, 0.000, 0.0)),
        material=rubber,
        name="jaw_pad",
    )
    clamp_jaw.visual(
        Box((0.018, 0.084, 0.026)),
        origin=Origin(xyz=(0.009, -0.033, 0.046)),
        material=anodized,
        name="bridge_arm",
    )
    clamp_jaw.visual(
        Box((0.036, 0.022, 0.052)),
        origin=Origin(xyz=(0.009, -0.086, 0.039)),
        material=anodized,
        name="slider_shoe",
    )

    pan_knob = model.part("pan_knob")
    pan_knob.visual(
        Cylinder(radius=0.007, length=0.050),
        origin=Origin(xyz=(0.0, 0.025, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=stainless,
        name="pan_shaft",
    )
    pan_knob.visual(
        Cylinder(radius=0.026, length=0.028),
        origin=Origin(xyz=(0.0, -0.014, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="pan_grip",
    )

    tilt_knob = model.part("tilt_knob")
    tilt_knob.visual(
        Cylinder(radius=0.008, length=0.025),
        origin=Origin(xyz=(0.0125, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=stainless,
        name="tilt_shaft",
    )
    tilt_knob.visual(
        Cylinder(radius=0.030, length=0.026),
        origin=Origin(xyz=(0.038, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber,
        name="tilt_grip",
    )

    clamp_knob = model.part("clamp_knob")
    clamp_knob.visual(
        Cylinder(radius=0.007, length=0.035),
        origin=Origin(xyz=(0.0175, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=stainless,
        name="clamp_screw",
    )
    clamp_knob.visual(
        Cylinder(radius=0.024, length=0.024),
        origin=Origin(xyz=(0.047, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber,
        name="clamp_grip",
    )

    model.articulation(
        "tripod_to_pan",
        ArticulationType.REVOLUTE,
        parent=tripod,
        child=pan_head,
        origin=Origin(xyz=(0.0, 0.0, 0.815)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.0, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "pan_to_tilt",
        ArticulationType.REVOLUTE,
        parent=pan_head,
        child=tilt_cradle,
        origin=Origin(xyz=(0.0, 0.0, 0.170)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=28.0, velocity=0.8, lower=-0.70, upper=0.70),
    )
    model.articulation(
        "cradle_to_device",
        ArticulationType.FIXED,
        parent=tilt_cradle,
        child=device,
        origin=Origin(),
    )
    model.articulation(
        "cradle_to_jaw",
        ArticulationType.PRISMATIC,
        parent=tilt_cradle,
        child=clamp_jaw,
        origin=Origin(xyz=(0.110, 0.143, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=70.0, velocity=0.08, lower=0.0, upper=0.035),
    )
    model.articulation(
        "pan_to_pan_knob",
        ArticulationType.CONTINUOUS,
        parent=pan_head,
        child=pan_knob,
        origin=Origin(xyz=(0.0, -0.102, 0.052)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=5.0),
    )
    model.articulation(
        "pan_to_tilt_knob",
        ArticulationType.CONTINUOUS,
        parent=pan_head,
        child=tilt_knob,
        origin=Origin(xyz=(0.133, 0.0, 0.170)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=4.0),
    )
    model.articulation(
        "jaw_to_clamp_knob",
        ArticulationType.CONTINUOUS,
        parent=clamp_jaw,
        child=clamp_knob,
        origin=Origin(xyz=(0.018, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=4.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tripod = object_model.get_part("tripod")
    pan_head = object_model.get_part("pan_head")
    tilt_cradle = object_model.get_part("tilt_cradle")
    device = object_model.get_part("device")
    clamp_jaw = object_model.get_part("clamp_jaw")
    pan_joint = object_model.get_articulation("tripod_to_pan")
    tilt_joint = object_model.get_articulation("pan_to_tilt")
    clamp_slide = object_model.get_articulation("cradle_to_jaw")

    # The tilt trunnion is intentionally captured by both side cheeks of the
    # yoke.  This is a local metal-on-bushing interface, not a broad collision.
    ctx.allow_overlap(
        pan_head,
        tilt_cradle,
        elem_a="yoke_cheek_0",
        elem_b="trunnion",
        reason="The tilt trunnion passes through the protected yoke cheek bore.",
    )
    ctx.allow_overlap(
        pan_head,
        tilt_cradle,
        elem_a="yoke_cheek_1",
        elem_b="trunnion",
        reason="The tilt trunnion passes through the protected yoke cheek bore.",
    )
    ctx.allow_overlap(
        pan_head,
        tilt_cradle,
        elem_a="tilt_boss_0",
        elem_b="trunnion",
        reason="The stainless boss is a protected bearing washer around the captured tilt trunnion.",
    )
    ctx.allow_overlap(
        pan_head,
        tilt_cradle,
        elem_a="tilt_boss_1",
        elem_b="trunnion",
        reason="The stainless boss is a protected bearing washer around the captured tilt trunnion.",
    )
    ctx.expect_overlap(
        pan_head,
        tilt_cradle,
        axes="x",
        elem_a="yoke_cheek_0",
        elem_b="trunnion",
        min_overlap=0.015,
        name="left yoke captures trunnion",
    )
    ctx.expect_overlap(
        pan_head,
        tilt_cradle,
        axes="x",
        elem_a="yoke_cheek_1",
        elem_b="trunnion",
        min_overlap=0.015,
        name="right yoke captures trunnion",
    )
    ctx.expect_overlap(
        pan_head,
        tilt_cradle,
        axes="x",
        elem_a="tilt_boss_0",
        elem_b="trunnion",
        min_overlap=0.006,
        name="left boss surrounds trunnion",
    )
    ctx.expect_overlap(
        pan_head,
        tilt_cradle,
        axes="x",
        elem_a="tilt_boss_1",
        elem_b="trunnion",
        min_overlap=0.006,
        name="right boss surrounds trunnion",
    )

    ctx.allow_overlap(
        tilt_cradle,
        object_model.get_part("tilt_knob"),
        elem_a="trunnion",
        elem_b="tilt_shaft",
        reason="The tilt lock screw is intentionally threaded into the trunnion end.",
    )
    ctx.expect_overlap(
        tilt_cradle,
        object_model.get_part("tilt_knob"),
        axes="x",
        elem_a="trunnion",
        elem_b="tilt_shaft",
        min_overlap=0.006,
        name="tilt lock screw engages trunnion",
    )

    # The moving jaw has a captive shoe sliding in a metal guide channel.
    ctx.allow_overlap(
        tilt_cradle,
        clamp_jaw,
        elem_a="rail_channel",
        elem_b="slider_shoe",
        reason="The slider shoe is intentionally represented as retained inside the clamp guide channel.",
    )
    ctx.expect_within(
        clamp_jaw,
        tilt_cradle,
        axes="yz",
        inner_elem="slider_shoe",
        outer_elem="rail_channel",
        margin=0.002,
        name="clamp shoe remains in guide",
    )
    ctx.expect_overlap(
        clamp_jaw,
        tilt_cradle,
        axes="x",
        elem_a="slider_shoe",
        elem_b="rail_channel",
        min_overlap=0.035,
        name="clamp shoe retained along rail",
    )

    ctx.expect_contact(
        tripod,
        pan_head,
        elem_a="head_plate",
        elem_b="drip_skirt",
        contact_tol=0.001,
        name="pan head seated on tripod plate",
    )
    ctx.expect_contact(
        tilt_cradle,
        device,
        elem_a="back_pad",
        elem_b="sealed_body",
        contact_tol=0.001,
        name="device sealed against bracket pad",
    )
    ctx.expect_gap(
        clamp_jaw,
        device,
        axis="x",
        positive_elem="jaw_pad",
        negative_elem="sealed_body",
        max_gap=0.001,
        max_penetration=0.001,
        name="clamp jaw bears on device",
    )

    with ctx.pose({clamp_slide: 0.035}):
        ctx.expect_gap(
            clamp_jaw,
            device,
            axis="x",
            positive_elem="jaw_pad",
            negative_elem="sealed_body",
            min_gap=0.032,
            max_gap=0.038,
            name="clamp jaw opens outward",
        )
        ctx.expect_within(
            clamp_jaw,
            tilt_cradle,
            axes="yz",
            inner_elem="slider_shoe",
            outer_elem="rail_channel",
            margin=0.002,
            name="opened clamp shoe still guided",
        )

    rest_aabb = ctx.part_world_aabb(device)
    with ctx.pose({tilt_joint: 0.35}):
        tilted_aabb = ctx.part_world_aabb(device)
    if rest_aabb is not None and tilted_aabb is not None:
        rest_center_z = (rest_aabb[0][2] + rest_aabb[1][2]) * 0.5
        tilted_center_z = (tilted_aabb[0][2] + tilted_aabb[1][2]) * 0.5
    else:
        rest_center_z = tilted_center_z = None
    ctx.check(
        "tilt raises device front",
        rest_center_z is not None and tilted_center_z is not None and tilted_center_z > rest_center_z + 0.020,
        details=f"rest_center_z={rest_center_z}, tilted_center_z={tilted_center_z}",
    )

    with ctx.pose({pan_joint: 0.75}):
        panned_aabb = ctx.part_world_aabb(device)
    if rest_aabb is not None and panned_aabb is not None:
        rest_center_x = (rest_aabb[0][0] + rest_aabb[1][0]) * 0.5
        panned_center_x = (panned_aabb[0][0] + panned_aabb[1][0]) * 0.5
    else:
        rest_center_x = panned_center_x = None
    ctx.check(
        "pan turns off center device",
        rest_center_x is not None and panned_center_x is not None and abs(panned_center_x - rest_center_x) > 0.050,
        details=f"rest_center_x={rest_center_x}, panned_center_x={panned_center_x}",
    )

    return ctx.report()


object_model = build_object_model()
