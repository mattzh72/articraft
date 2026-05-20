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


PIN_AXIS = (0.0, 1.0, 0.0)
CYLINDER_ALONG_Y = Origin(rpy=(-math.pi / 2.0, 0.0, 0.0))


def _pin_disk_origin(x: float, y: float, z: float) -> Origin:
    return Origin(xyz=(x, y, z), rpy=CYLINDER_ALONG_Y.rpy)


def _add_side_plate_link(
    part,
    *,
    length: float,
    height: float,
    plate_y: float,
    plate_thickness: float,
    material: str,
    pin_material: str,
    brace_material: str,
    brace_scale: float = 1.0,
) -> None:
    """Build a two-cheek side-plate link with rounded pin lobes."""

    bar_height = height * 0.52
    lobe_radius = height * 0.50
    cap_radius = height * 0.20
    cap_length = 0.003

    for side in (-1.0, 1.0):
        y = side * plate_y
        part.visual(
            Box((length, plate_thickness, bar_height)),
            origin=Origin(xyz=(length / 2.0, y, 0.0)),
            material=material,
            name=f"side_plate_{'n' if side < 0 else 'p'}",
        )
        for x, end_name in ((0.0, "near"), (length, "far")):
            part.visual(
                Cylinder(radius=lobe_radius, length=plate_thickness),
                origin=_pin_disk_origin(x, y, 0.0),
                material=material,
                name=f"{end_name}_lobe_{'n' if side < 0 else 'p'}",
            )
            part.visual(
                Cylinder(radius=cap_radius, length=plate_thickness * 0.90),
                origin=_pin_disk_origin(x, y, 0.0),
                material=pin_material,
                name=f"{end_name}_pin_cap_{'n' if side < 0 else 'p'}",
            )

    # A low cross spacer makes each pair of side plates read as one fabricated
    # link while keeping the actual hinge zones clear for the nested neighbor.
    part.visual(
        Box((0.070 * brace_scale, 2.0 * plate_y + plate_thickness, height * 0.18)),
        origin=Origin(xyz=(length * 0.48, 0.0, -height * 0.35)),
        material=brace_material,
        name="cross_spacer",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="nested_bracket_arm")

    model.material("graphite_cast", rgba=(0.08, 0.085, 0.09, 1.0))
    model.material("blued_steel", rgba=(0.10, 0.16, 0.22, 1.0))
    model.material("brushed_steel", rgba=(0.58, 0.60, 0.58, 1.0))
    model.material("dark_pin", rgba=(0.03, 0.032, 0.035, 1.0))
    model.material("rubber_black", rgba=(0.012, 0.012, 0.010, 1.0))
    model.material("tray_enamel", rgba=(0.75, 0.76, 0.70, 1.0))

    # A deliberately dense, bolted grounded foot and yoke.  The first pin sits
    # high above the base so the arm clears the foot through its normal travel.
    foot = model.part("foot")
    foot.visual(
        Box((0.34, 0.28, 0.034)),
        origin=Origin(xyz=(0.0, 0.0, 0.029)),
        material="graphite_cast",
        name="base_plate",
    )
    for x in (-0.115, 0.115):
        for y in (-0.095, 0.095):
            foot.visual(
                Cylinder(radius=0.018, length=0.018),
                origin=Origin(xyz=(x, y, 0.009)),
                material="rubber_black",
                name=f"rubber_pad_{x}_{y}",
            )
            foot.visual(
                Cylinder(radius=0.011, length=0.006),
                origin=Origin(xyz=(x, y, 0.048)),
                material="dark_pin",
                name=f"bolt_head_{x}_{y}",
            )
    foot.visual(
        Box((0.155, 0.160, 0.078)),
        origin=Origin(xyz=(0.010, 0.0, 0.084)),
        material="graphite_cast",
        name="pedestal_block",
    )
    foot.visual(
        Box((0.120, 0.028, 0.165)),
        origin=Origin(xyz=(0.025, -0.088, 0.157)),
        material="graphite_cast",
        name="clevis_cheek_n",
    )
    foot.visual(
        Box((0.120, 0.028, 0.165)),
        origin=Origin(xyz=(0.025, 0.088, 0.157)),
        material="graphite_cast",
        name="clevis_cheek_p",
    )
    for side in (-1.0, 1.0):
        foot.visual(
            Cylinder(radius=0.055, length=0.028),
            origin=_pin_disk_origin(0.020, side * 0.088, 0.230),
            material="graphite_cast",
            name=f"base_pin_lobe_{'n' if side < 0 else 'p'}",
        )
        foot.visual(
            Cylinder(radius=0.018, length=0.004),
            origin=_pin_disk_origin(0.020, side * 0.104, 0.230),
            material="dark_pin",
            name=f"base_pin_head_{'n' if side < 0 else 'p'}",
        )

    link_0 = model.part("long_link_0")
    _add_side_plate_link(
        link_0,
        length=0.430,
        height=0.086,
        plate_y=0.068,
        plate_thickness=0.012,
        material="blued_steel",
        pin_material="dark_pin",
        brace_material="brushed_steel",
        brace_scale=1.20,
    )

    link_1 = model.part("short_link")
    _add_side_plate_link(
        link_1,
        length=0.245,
        height=0.064,
        plate_y=0.057,
        plate_thickness=0.010,
        material="brushed_steel",
        pin_material="dark_pin",
        brace_material="blued_steel",
        brace_scale=0.75,
    )

    link_2 = model.part("long_link_1")
    _add_side_plate_link(
        link_2,
        length=0.335,
        height=0.056,
        plate_y=0.067,
        plate_thickness=0.010,
        material="brushed_steel",
        pin_material="dark_pin",
        brace_material="blued_steel",
        brace_scale=0.85,
    )

    tray = model.part("end_bracket")
    for side in (-1.0, 1.0):
        ear_y = side * 0.0575
        tray.visual(
            Box((0.055, 0.009, 0.058)),
            origin=Origin(xyz=(0.015, ear_y, -0.028)),
            material="tray_enamel",
            name=f"tray_ear_{'n' if side < 0 else 'p'}",
        )
        tray.visual(
            Cylinder(radius=0.027, length=0.009),
            origin=_pin_disk_origin(0.0, ear_y, 0.0),
            material="tray_enamel",
            name=f"tray_pivot_lobe_{'n' if side < 0 else 'p'}",
        )
        tray.visual(
            Cylinder(radius=0.010, length=0.0025),
            origin=_pin_disk_origin(0.0, side * 0.0535, 0.0),
            material="dark_pin",
            name=f"tray_pin_cap_{'n' if side < 0 else 'p'}",
        )
    tray.visual(
        Box((0.165, 0.118, 0.012)),
        origin=Origin(xyz=(0.085, 0.0, -0.055)),
        material="tray_enamel",
        name="tray_floor",
    )
    tray.visual(
        Box((0.132, 0.010, 0.038)),
        origin=Origin(xyz=(0.096, -0.044, -0.036)),
        material="tray_enamel",
        name="side_lip_n",
    )
    tray.visual(
        Box((0.132, 0.010, 0.038)),
        origin=Origin(xyz=(0.096, 0.044, -0.036)),
        material="tray_enamel",
        name="side_lip_p",
    )
    tray.visual(
        Box((0.014, 0.118, 0.040)),
        origin=Origin(xyz=(0.166, 0.0, -0.032)),
        material="tray_enamel",
        name="front_lip",
    )
    tray.visual(
        Box((0.035, 0.118, 0.018)),
        origin=Origin(xyz=(0.020, 0.0, -0.045)),
        material="tray_enamel",
        name="rear_bridge",
    )

    model.articulation(
        "foot_to_long_0",
        ArticulationType.REVOLUTE,
        parent=foot,
        child=link_0,
        origin=Origin(xyz=(0.020, 0.0, 0.230), rpy=(0.0, -0.42, 0.0)),
        axis=PIN_AXIS,
        motion_limits=MotionLimits(effort=38.0, velocity=1.4, lower=-0.85, upper=0.95),
    )
    model.articulation(
        "long_0_to_short",
        ArticulationType.REVOLUTE,
        parent=link_0,
        child=link_1,
        origin=Origin(xyz=(0.430, 0.0, 0.0), rpy=(0.0, 0.58, 0.0)),
        axis=PIN_AXIS,
        motion_limits=MotionLimits(effort=24.0, velocity=1.6, lower=-1.05, upper=1.05),
    )
    model.articulation(
        "short_to_long_1",
        ArticulationType.REVOLUTE,
        parent=link_1,
        child=link_2,
        origin=Origin(xyz=(0.245, 0.0, 0.0), rpy=(0.0, -0.50, 0.0)),
        axis=PIN_AXIS,
        motion_limits=MotionLimits(effort=18.0, velocity=1.7, lower=-1.05, upper=1.05),
    )
    model.articulation(
        "long_1_to_bracket",
        ArticulationType.REVOLUTE,
        parent=link_2,
        child=tray,
        origin=Origin(xyz=(0.335, 0.0, 0.0), rpy=(0.0, 0.34, 0.0)),
        axis=PIN_AXIS,
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=-1.20, upper=1.20),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    joints = list(object_model.articulations)

    ctx.check(
        "exactly four revolute joints",
        len(joints) == 4
        and all(joint.articulation_type == ArticulationType.REVOLUTE for joint in joints),
        details=f"joints={[joint.name for joint in joints]}",
    )
    ctx.check(
        "parallel pin axes",
        all(tuple(round(v, 6) for v in joint.axis) == PIN_AXIS for joint in joints),
        details=f"axes={[joint.axis for joint in joints]}",
    )

    long_0 = object_model.get_part("long_link_0")
    short = object_model.get_part("short_link")
    long_1 = object_model.get_part("long_link_1")
    tray = object_model.get_part("end_bracket")
    foot = object_model.get_part("foot")
    tray_joint = object_model.get_articulation("long_1_to_bracket")

    aabb_long_0 = ctx.part_world_aabb(long_0)
    aabb_short = ctx.part_world_aabb(short)
    aabb_long_1 = ctx.part_world_aabb(long_1)
    ctx.check(
        "alternating long short long links",
        aabb_long_0 is not None
        and aabb_short is not None
        and aabb_long_1 is not None
        and (aabb_long_0[1][0] - aabb_long_0[0][0]) > (aabb_short[1][0] - aabb_short[0][0]) + 0.10
        and (aabb_long_1[1][0] - aabb_long_1[0][0]) > (aabb_short[1][0] - aabb_short[0][0]) + 0.05,
        details=f"long0={aabb_long_0}, short={aabb_short}, long1={aabb_long_1}",
    )
    foot_box = ctx.part_world_aabb(foot)
    tray_box = ctx.part_world_aabb(tray)
    ctx.check(
        "light tray carried beyond dense base",
        foot_box is not None and tray_box is not None and tray_box[1][0] > foot_box[1][0] + 0.65,
        details=f"foot={foot_box}, tray={tray_box}",
    )
    with ctx.pose({tray_joint: 0.45}):
        moved_tray_box = ctx.part_world_aabb(tray)
        ctx.check(
            "end bracket rotates about final parallel pin",
            tray_box is not None
            and moved_tray_box is not None
            and abs(moved_tray_box[0][2] - tray_box[0][2]) > 0.015,
            details=f"rest={tray_box}, moved={moved_tray_box}",
        )

    return ctx.report()


object_model = build_object_model()
