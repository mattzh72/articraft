from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
)


def _axis_origin(start: tuple[float, float, float], end: tuple[float, float, float]) -> tuple[Origin, float]:
    """Return a cylinder origin that aligns local +Z from start to end."""
    sx, sy, sz = start
    ex, ey, ez = end
    vx, vy, vz = ex - sx, ey - sy, ez - sz
    length = math.sqrt(vx * vx + vy * vy + vz * vz)
    if length <= 0.0:
        raise ValueError("zero length cylinder")
    nx, ny, nz = vx / length, vy / length, vz / length
    yaw = math.atan2(ny, nx)
    pitch = math.atan2(math.sqrt(nx * nx + ny * ny), nz)
    return Origin(xyz=((sx + ex) / 2.0, (sy + ey) / 2.0, (sz + ez) / 2.0), rpy=(0.0, pitch, yaw)), length


def _cylinder_between(part, start, end, radius: float, material, name: str) -> None:
    origin, length = _axis_origin(start, end)
    part.visual(Cylinder(radius=radius, length=length), origin=origin, material=material, name=name)


def _rot_z(x: float, y: float, yaw: float) -> tuple[float, float]:
    return (x * math.cos(yaw) - y * math.sin(yaw), x * math.sin(yaw) + y * math.cos(yaw))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="precision_tripod_alignment_device")

    matte_black = model.material("matte_black", rgba=(0.015, 0.016, 0.018, 1.0))
    anodized = model.material("dark_anodized_aluminum", rgba=(0.12, 0.13, 0.14, 1.0))
    brushed = model.material("brushed_steel", rgba=(0.62, 0.63, 0.60, 1.0))
    rubber = model.material("black_rubber", rgba=(0.02, 0.019, 0.017, 1.0))
    white = model.material("engraved_white", rgba=(0.86, 0.88, 0.84, 1.0))
    red = model.material("red_datum", rgba=(0.86, 0.08, 0.04, 1.0))
    blue = model.material("blue_gap_shim", rgba=(0.08, 0.22, 0.90, 1.0))
    device_mat = model.material("calibration_device_gray", rgba=(0.31, 0.34, 0.36, 1.0))

    tripod = model.part("tripod")
    # Tripod core: one connected welded/cast assembly of column, spider, legs, and spreaders.
    tripod.visual(
        Cylinder(radius=0.033, length=0.93),
        origin=Origin(xyz=(0.0, 0.0, 0.55)),
        material=anodized,
        name="center_column",
    )
    tripod.visual(
        Cylinder(radius=0.115, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 1.02)),
        material=anodized,
        name="leg_spider",
    )
    tripod.visual(
        Cylinder(radius=0.128, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 1.075)),
        material=brushed,
        name="head_plate",
    )
    tripod.visual(
        Cylinder(radius=0.052, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 1.057)),
        material=anodized,
        name="plate_neck",
    )
    tripod.visual(
        Cylinder(radius=0.107, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 1.096)),
        material=blue,
        name="controlled_pan_gap",
    )
    tripod.visual(
        Cylinder(radius=0.043, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.37)),
        material=anodized,
        name="spreader_ring",
    )

    for i, yaw in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        top = (0.095 * math.cos(yaw), 0.095 * math.sin(yaw), 0.995)
        foot = (0.58 * math.cos(yaw), 0.58 * math.sin(yaw), 0.045)
        _cylinder_between(tripod, top, foot, 0.017, matte_black, f"leg_tube_{i}")
        pad_x, pad_y = _rot_z(0.020, 0.0, yaw)
        tripod.visual(
            Box((0.150, 0.060, 0.028)),
            origin=Origin(xyz=(foot[0] + pad_x, foot[1] + pad_y, 0.031), rpy=(0.0, 0.0, yaw)),
            material=rubber,
            name=f"leveling_foot_{i}",
        )
        brace_inner = (0.040 * math.cos(yaw), 0.040 * math.sin(yaw), 0.380)
        brace_outer = (0.365 * math.cos(yaw), 0.365 * math.sin(yaw), 0.440)
        _cylinder_between(tripod, brace_inner, brace_outer, 0.010, brushed, f"spreader_strut_{i}")
        collar = (0.325 * math.cos(yaw), 0.325 * math.sin(yaw), 0.535)
        tripod.visual(
            Box((0.070, 0.035, 0.045)),
            origin=Origin(xyz=collar, rpy=(0.0, 0.0, yaw)),
            material=anodized,
            name=f"leg_lock_collar_{i}",
        )

    # Pan scale ticks on the fixed head plate, with a larger zero datum tick.
    for i in range(24):
        yaw = i * math.tau / 24.0
        r = 0.120
        tick_len = 0.032 if i % 6 == 0 else 0.020
        tick_w = 0.006 if i % 6 == 0 else 0.0035
        tripod.visual(
            Box((tick_w, tick_len, 0.003)),
            origin=Origin(xyz=(r * math.cos(yaw), r * math.sin(yaw), 1.0855), rpy=(0.0, 0.0, yaw)),
            material=red if i == 0 else white,
            name=f"pan_index_{i}",
        )

    # Pan lock screw stem carried by the fixed head collar; the knob is an articulated child.
    tripod.visual(
        Cylinder(radius=0.008, length=0.020),
        origin=Origin(xyz=(-0.117, 0.0, 1.115), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed,
        name="pan_lock_stem",
    )

    pan_stage = model.part("pan_stage")
    pan_stage.visual(
        Cylinder(radius=0.094, length=0.038),
        origin=Origin(xyz=(0.0, 0.0, 0.019)),
        material=anodized,
        name="pan_bearing_disk",
    )
    pan_stage.visual(
        Box((0.036, 0.006, 0.030)),
        origin=Origin(xyz=(0.106, 0.0, 0.035)),
        material=red,
        name="pan_pointer",
    )
    pan_stage.visual(
        Box((0.082, 0.020, 0.052)),
        origin=Origin(xyz=(0.0, 0.0, 0.058)),
        material=anodized,
        name="tilt_pedestal",
    )
    pan_stage.visual(
        Box((0.074, 0.190, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.046)),
        material=anodized,
        name="yoke_crossbar",
    )
    # Real yoke cheeks around a clear tilt gap. The child axle sits between these cheeks.
    for side, y in enumerate((-0.091, 0.091)):
        pan_stage.visual(
            Box((0.052, 0.016, 0.118)),
            origin=Origin(xyz=(0.0, y, 0.100)),
            material=anodized,
            name=f"tilt_yoke_cheek_{side}",
        )
        pan_stage.visual(
            Cylinder(radius=0.029, length=0.004),
            origin=Origin(xyz=(0.0, y * 1.018, 0.102), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=brushed,
            name=f"tilt_scale_boss_{side}",
        )

    # White tick bars on the outside of one cheek describe the repeatable tilt scale.
    for i, z in enumerate((0.063, 0.074, 0.085, 0.096, 0.107, 0.118, 0.129, 0.140)):
        pan_stage.visual(
            Box((0.004, 0.0025, 0.014 if i in (0, 4, 7) else 0.009)),
            origin=Origin(xyz=(-0.024, -0.098, z), rpy=(0.0, 0.0, 0.0)),
            material=red if i == 4 else white,
            name=f"tilt_index_{i}",
        )
    pan_stage.visual(
        Cylinder(radius=0.008, length=0.010),
        origin=Origin(xyz=(0.0, 0.101, 0.102), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brushed,
        name="tilt_lock_stem",
    )

    tilt_stage = model.part("tilt_stage")
    tilt_stage.visual(
        Cylinder(radius=0.018, length=0.162),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=brushed,
        name="tilt_axle",
    )
    tilt_stage.visual(
        Box((0.255, 0.118, 0.018)),
        origin=Origin(xyz=(0.122, 0.0, 0.0)),
        material=anodized,
        name="dovetail_bed",
    )
    tilt_stage.visual(
        Box((0.245, 0.014, 0.010)),
        origin=Origin(xyz=(0.125, -0.058, 0.014)),
        material=brushed,
        name="rear_datum_rail",
    )
    tilt_stage.visual(
        Box((0.245, 0.014, 0.010)),
        origin=Origin(xyz=(0.125, 0.058, 0.014)),
        material=brushed,
        name="front_datum_rail",
    )
    tilt_stage.visual(
        Box((0.004, 0.090, 0.004)),
        origin=Origin(xyz=(0.000, 0.0, 0.011)),
        material=red,
        name="zero_crosshair",
    )
    for i, x in enumerate((0.010, 0.045, 0.080, 0.115, 0.150, 0.185, 0.220, 0.242)):
        tilt_stage.visual(
            Box((0.003, 0.038 if i % 2 == 0 else 0.024, 0.003)),
            origin=Origin(xyz=(x, 0.0, 0.0105)),
            material=white,
            name=f"slide_scale_{i}",
        )

    bracket_slide = model.part("bracket_slide")
    bracket_slide.visual(
        Box((0.182, 0.096, 0.022)),
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
        material=anodized,
        name="carriage_plate",
    )
    bracket_slide.visual(
        Box((0.150, 0.010, 0.090)),
        origin=Origin(xyz=(0.020, -0.040, 0.067)),
        material=brushed,
        name="rear_datum_fence",
    )
    bracket_slide.visual(
        Box((0.126, 0.012, 0.012)),
        origin=Origin(xyz=(0.020, 0.054, 0.028)),
        material=brushed,
        name="front_clamp_guide",
    )
    bracket_slide.visual(
        Box((0.004, 0.090, 0.004)),
        origin=Origin(xyz=(0.020, 0.0, 0.025)),
        material=red,
        name="carriage_datum_line",
    )
    bracket_slide.visual(
        Box((0.146, 0.004, 0.002)),
        origin=Origin(xyz=(0.020, -0.034, 0.112)),
        material=blue,
        name="controlled_rear_gap_gauge",
    )

    calibration_body = model.part("calibration_body")
    calibration_body.visual(
        Box((0.132, 0.070, 0.075)),
        origin=Origin(xyz=(0.0, 0.0, 0.0375)),
        material=device_mat,
        name="instrument_block",
    )
    calibration_body.visual(
        Cylinder(radius=0.025, length=0.018),
        origin=Origin(xyz=(0.066, 0.0, 0.039), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=matte_black,
        name="front_reference_port",
    )
    calibration_body.visual(
        Box((0.105, 0.004, 0.003)),
        origin=Origin(xyz=(0.0, -0.037, 0.065)),
        material=red,
        name="rear_datum_stripe",
    )
    for i, x in enumerate((-0.045, -0.015, 0.015, 0.045)):
        calibration_body.visual(
            Box((0.003, 0.004, 0.020)),
            origin=Origin(xyz=(x, -0.0365, 0.035)),
            material=white,
            name=f"device_index_{i}",
        )

    clamp_jaw = model.part("clamp_jaw")
    clamp_jaw.visual(
        Box((0.126, 0.012, 0.070)),
        origin=Origin(xyz=(0.0, 0.006, 0.0)),
        material=brushed,
        name="moving_jaw_pad",
    )
    clamp_jaw.visual(
        Cylinder(radius=0.007, length=0.022),
        origin=Origin(xyz=(0.0, 0.023, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=brushed,
        name="clamp_screw",
    )
    clamp_jaw.visual(
        Box((0.136, 0.003, 0.058)),
        origin=Origin(xyz=(0.0, -0.0015, 0.0)),
        material=blue,
        name="controlled_clamp_gap",
    )

    pan_knob = model.part("pan_knob")
    pan_knob.visual(
        Cylinder(radius=0.024, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=matte_black,
        name="knurled_pan_knob",
    )
    for i in range(8):
        yaw = i * math.tau / 8.0
        pan_knob.visual(
            Box((0.005, 0.006, 0.026)),
            origin=Origin(xyz=(0.0, 0.024 * math.cos(yaw), 0.024 * math.sin(yaw)), rpy=(0.0, 0.0, yaw)),
            material=brushed,
            name=f"pan_knurl_{i}",
        )

    tilt_knob = model.part("tilt_knob")
    tilt_knob.visual(
        Cylinder(radius=0.027, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=matte_black,
        name="knurled_tilt_knob",
    )
    for i in range(10):
        yaw = i * math.tau / 10.0
        tilt_knob.visual(
            Box((0.006, 0.005, 0.028)),
            origin=Origin(xyz=(0.027 * math.cos(yaw), 0.0, 0.027 * math.sin(yaw)), rpy=(0.0, 0.0, yaw)),
            material=brushed,
            name=f"tilt_knurl_{i}",
        )

    clamp_knob = model.part("clamp_knob")
    clamp_knob.visual(
        Cylinder(radius=0.022, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=matte_black,
        name="knurled_clamp_knob",
    )
    for i in range(8):
        yaw = i * math.tau / 8.0
        clamp_knob.visual(
            Box((0.005, 0.004, 0.024)),
            origin=Origin(xyz=(0.022 * math.cos(yaw), 0.0, 0.022 * math.sin(yaw)), rpy=(0.0, 0.0, yaw)),
            material=brushed,
            name=f"clamp_knurl_{i}",
        )

    model.articulation(
        "pan_axis",
        ArticulationType.REVOLUTE,
        parent=tripod,
        child=pan_stage,
        origin=Origin(xyz=(0.0, 0.0, 1.108)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=24.0, velocity=0.8, lower=-math.pi, upper=math.pi),
        motion_properties=MotionProperties(damping=0.15, friction=0.08),
    )
    model.articulation(
        "tilt_axis",
        ArticulationType.REVOLUTE,
        parent=pan_stage,
        child=tilt_stage,
        origin=Origin(xyz=(0.0, 0.0, 0.102)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=0.55, lower=-0.55, upper=0.70),
        motion_properties=MotionProperties(damping=0.18, friction=0.10),
    )
    model.articulation(
        "slide_adjust",
        ArticulationType.PRISMATIC,
        parent=tilt_stage,
        child=bracket_slide,
        origin=Origin(xyz=(0.122, 0.0, 0.009)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=0.08, lower=-0.040, upper=0.040),
        motion_properties=MotionProperties(damping=0.20, friction=0.20),
    )
    model.articulation(
        "device_mount",
        ArticulationType.FIXED,
        parent=bracket_slide,
        child=calibration_body,
        origin=Origin(xyz=(0.020, 0.0, 0.022)),
    )
    model.articulation(
        "clamp_travel",
        ArticulationType.PRISMATIC,
        parent=bracket_slide,
        child=clamp_jaw,
        origin=Origin(xyz=(0.020, 0.035, 0.0595)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=0.035, lower=0.0, upper=0.032),
        motion_properties=MotionProperties(damping=0.35, friction=0.40),
    )
    model.articulation(
        "pan_lock_turn",
        ArticulationType.CONTINUOUS,
        parent=tripod,
        child=pan_knob,
        origin=Origin(xyz=(-0.140, 0.0, 1.115)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=5.0),
    )
    model.articulation(
        "tilt_lock_turn",
        ArticulationType.CONTINUOUS,
        parent=pan_stage,
        child=tilt_knob,
        origin=Origin(xyz=(0.0, 0.120, 0.102)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.5, velocity=5.0),
    )
    model.articulation(
        "clamp_knob_turn",
        ArticulationType.CONTINUOUS,
        parent=clamp_jaw,
        child=clamp_knob,
        origin=Origin(xyz=(0.0, 0.046, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.5, velocity=5.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tripod = object_model.get_part("tripod")
    pan_stage = object_model.get_part("pan_stage")
    tilt_stage = object_model.get_part("tilt_stage")
    bracket_slide = object_model.get_part("bracket_slide")
    calibration_body = object_model.get_part("calibration_body")
    clamp_jaw = object_model.get_part("clamp_jaw")
    pan_axis = object_model.get_articulation("pan_axis")
    tilt_axis = object_model.get_articulation("tilt_axis")
    slide_adjust = object_model.get_articulation("slide_adjust")
    clamp_travel = object_model.get_articulation("clamp_travel")

    ctx.expect_gap(
        pan_stage,
        tripod,
        axis="z",
        min_gap=0.0,
        max_gap=0.002,
        positive_elem="pan_bearing_disk",
        negative_elem="controlled_pan_gap",
        name="pan bearing rides on fixed blue gap shim",
    )
    ctx.expect_within(
        bracket_slide,
        tilt_stage,
        axes="y",
        margin=0.001,
        inner_elem="carriage_plate",
        outer_elem="dovetail_bed",
        name="slide carriage remains centered between datum rails",
    )
    ctx.expect_gap(
        bracket_slide,
        tilt_stage,
        axis="z",
        min_gap=0.0,
        max_gap=0.001,
        positive_elem="carriage_plate",
        negative_elem="dovetail_bed",
        name="sliding carriage has controlled zero-height datum contact",
    )
    ctx.expect_contact(
        calibration_body,
        bracket_slide,
        elem_a="instrument_block",
        elem_b="carriage_plate",
        contact_tol=0.001,
        name="instrument block sits on carriage datum surface",
    )
    ctx.expect_gap(
        clamp_jaw,
        calibration_body,
        axis="y",
        min_gap=0.0,
        max_gap=0.001,
        positive_elem="moving_jaw_pad",
        negative_elem="instrument_block",
        name="clamp jaw closes to controlled device contact",
    )

    rest_slide = ctx.part_world_position(bracket_slide)
    with ctx.pose({slide_adjust: 0.035}):
        extended_slide = ctx.part_world_position(bracket_slide)
        ctx.expect_within(
            bracket_slide,
            tilt_stage,
            axes="y",
            margin=0.001,
            inner_elem="carriage_plate",
            outer_elem="dovetail_bed",
            name="extended carriage remains on the rail centerline",
        )
        ctx.expect_overlap(
            bracket_slide,
            tilt_stage,
            axes="x",
            min_overlap=0.120,
            elem_a="carriage_plate",
            elem_b="dovetail_bed",
            name="extended slide keeps long bearing engagement",
        )
    ctx.check(
        "slide adjustment translates bracket forward",
        rest_slide is not None and extended_slide is not None and extended_slide[0] > rest_slide[0] + 0.030,
        details=f"rest={rest_slide}, extended={extended_slide}",
    )

    rest_clamp = ctx.part_world_position(clamp_jaw)
    with ctx.pose({clamp_travel: 0.025}):
        open_clamp = ctx.part_world_position(clamp_jaw)
        ctx.expect_gap(
            clamp_jaw,
            calibration_body,
            axis="y",
            min_gap=0.020,
            positive_elem="moving_jaw_pad",
            negative_elem="instrument_block",
            name="clamp jaw opens away from instrument",
        )
    ctx.check(
        "clamp travel moves outward",
        rest_clamp is not None and open_clamp is not None and open_clamp[1] > rest_clamp[1] + 0.020,
        details=f"rest={rest_clamp}, open={open_clamp}",
    )

    rest_head = ctx.part_world_position(tilt_stage)
    with ctx.pose({pan_axis: 0.8, tilt_axis: 0.35}):
        tilted_head = ctx.part_world_position(tilt_stage)
    ctx.check(
        "pan and tilt axes accept a combined alignment pose",
        rest_head is not None and tilted_head is not None,
        details=f"rest={rest_head}, tilted={tilted_head}",
    )

    return ctx.report()


object_model = build_object_model()
