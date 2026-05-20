from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _cyl_x(x: float, y: float, z: float) -> Origin:
    return Origin(xyz=(x, y, z), rpy=(0.0, math.pi / 2.0, 0.0))


def _cyl_y(x: float, y: float, z: float) -> Origin:
    return Origin(xyz=(x, y, z), rpy=(-math.pi / 2.0, 0.0, 0.0))


def _cyl_z(x: float, y: float, z: float) -> Origin:
    return Origin(xyz=(x, y, z))


def _annular_ring(outer_radius: float, inner_radius: float, thickness: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(thickness)
        .translate((0.0, 0.0, -thickness / 2.0))
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="ceiling_exam_lamp")

    painted = model.material("warm_white_painted_metal", rgba=(0.86, 0.84, 0.78, 1.0))
    satin = model.material("satin_stainless_steel", rgba=(0.62, 0.64, 0.64, 1.0))
    dark = model.material("black_rubber_cable", rgba=(0.02, 0.02, 0.018, 1.0))
    lens = model.material("warm_diffuser_lens", rgba=(1.0, 0.92, 0.48, 0.62))
    ceiling_mat = model.material("white_ceiling_panel", rgba=(0.95, 0.95, 0.92, 1.0))

    # Root: a ceiling escutcheon with a short fixed collar below it.
    ceiling_plate = model.part("ceiling_plate")
    ceiling_plate.visual(
        Box((0.46, 0.46, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.176)),
        material=ceiling_mat,
        name="ceiling_panel",
    )
    ceiling_plate.visual(
        Cylinder(radius=0.18, length=0.0375),
        origin=_cyl_z(0.0, 0.0, 0.15125),
        material=painted,
        name="round_plate",
    )
    ceiling_plate.visual(
        Cylinder(radius=0.105, length=0.085),
        origin=_cyl_z(0.0, 0.0, 0.09),
        material=painted,
        name="tapered_canopy",
    )
    ceiling_plate.visual(
        Cylinder(radius=0.066, length=0.0475),
        origin=_cyl_z(0.0, 0.0, 0.02375),
        material=satin,
        name="bottom_collar",
    )

    # Shoulder pan stage and the upper arm are one moving link.  The twin tubes
    # sit below the ceiling hub and terminate in a fork around the elbow joint.
    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        Cylinder(radius=0.070, length=0.090),
        origin=_cyl_z(0.0, 0.0, -0.045),
        material=satin,
        name="shoulder_hub",
    )
    for idx, y in enumerate((-0.058, 0.058)):
        upper_arm.visual(
            Cylinder(radius=0.014, length=0.760),
            origin=_cyl_x(0.43, y, -0.060),
            material=painted,
            name=f"upper_tube_{idx}",
        )
    upper_arm.visual(
        Cylinder(radius=0.006, length=0.750),
        origin=_cyl_x(0.40, 0.0, -0.087),
        material=dark,
        name="upper_cable",
    )
    for name, y in (("elbow_fork_0", -0.0575), ("elbow_fork_1", 0.0575)):
        upper_arm.visual(
            Cylinder(radius=0.065, length=0.040),
            origin=_cyl_y(0.85, y, -0.060),
            material=satin,
            name=name,
        )

    forearm = model.part("forearm")
    forearm.visual(
        Cylinder(radius=0.052, length=0.075),
        origin=_cyl_y(0.0, 0.0, 0.0),
        material=satin,
        name="elbow_knuckle",
    )
    for idx, y in enumerate((-0.022, 0.022)):
        forearm.visual(
            Cylinder(radius=0.014, length=0.590),
            origin=_cyl_x(0.335, y, -0.025),
            material=painted,
            name=f"forearm_tube_{idx}",
        )
    forearm.visual(
        Cylinder(radius=0.006, length=0.520),
        origin=_cyl_x(0.33, 0.0, -0.058),
        material=dark,
        name="forearm_cable",
    )
    for name, x in (("forearm_clip_0", 0.18), ("forearm_clip_1", 0.50)):
        forearm.visual(
            Box((0.018, 0.060, 0.040)),
            origin=Origin(xyz=(x, 0.0, -0.043)),
            material=dark,
            name=name,
        )
    forearm.visual(
        Cylinder(radius=0.055, length=0.050),
        origin=_cyl_z(0.65, 0.0, -0.025),
        material=satin,
        name="pan_socket",
    )

    # The pan link is a U-yoke: a vertical spindle, crossbar, two side cheeks,
    # and round bosses where the lamp tilts.
    yoke = model.part("yoke")
    yoke.visual(
        Cylinder(radius=0.035, length=0.080),
        origin=_cyl_z(0.0, 0.0, -0.040),
        material=satin,
        name="pan_stem",
    )
    yoke.visual(
        Box((0.090, 0.450, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, -0.090)),
        material=satin,
        name="yoke_crossbar",
    )
    for cheek_name, boss_name, y in (
        ("yoke_cheek_0", "tilt_boss_0", -0.205),
        ("yoke_cheek_1", "tilt_boss_1", 0.205),
    ):
        yoke.visual(
            Box((0.040, 0.035, 0.195)),
            origin=Origin(xyz=(0.0, y, -0.175)),
            material=satin,
            name=cheek_name,
        )
        yoke.visual(
            Cylinder(radius=0.045, length=0.035),
            origin=_cyl_y(0.0, y, -0.270),
            material=satin,
            name=boss_name,
        )

    lamp_head = model.part("lamp_head")
    lamp_head.visual(
        Cylinder(radius=0.025, length=0.375),
        origin=_cyl_y(0.0, 0.0, 0.0),
        material=satin,
        name="trunnion",
    )
    lamp_head.visual(
        Box((0.110, 0.300, 0.070)),
        origin=Origin(xyz=(0.0, 0.0, -0.035)),
        material=painted,
        name="rear_saddle",
    )
    lamp_head.visual(
        Cylinder(radius=0.175, length=0.085),
        origin=_cyl_z(0.0, 0.0, -0.105),
        material=painted,
        name="round_shell",
    )
    lamp_head.visual(
        Cylinder(radius=0.125, length=0.035),
        origin=_cyl_z(0.0, 0.0, -0.050),
        material=satin,
        name="rear_cap",
    )
    lamp_head.visual(
        mesh_from_cadquery(_annular_ring(0.176, 0.135, 0.014), "front_bezel"),
        origin=Origin(xyz=(0.0, 0.0, -0.151)),
        material=satin,
        name="front_bezel",
    )
    lamp_head.visual(
        Cylinder(radius=0.132, length=0.006),
        origin=_cyl_z(0.0, 0.0, -0.1505),
        material=lens,
        name="diffuser_lens",
    )
    lamp_head.visual(
        Cylinder(radius=0.015, length=0.073),
        origin=_cyl_z(0.0, 0.0, -0.190),
        material=satin,
        name="sterile_handle_stem",
    )
    lamp_head.visual(
        Sphere(radius=0.030),
        origin=Origin(xyz=(0.0, 0.0, -0.232)),
        material=painted,
        name="sterile_handle",
    )

    model.articulation(
        "shoulder",
        ArticulationType.REVOLUTE,
        parent=ceiling_plate,
        child=upper_arm,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.5, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "elbow",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=forearm,
        origin=Origin(xyz=(0.85, 0.0, -0.060)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=14.0, velocity=1.2, lower=-1.15, upper=1.25),
    )
    model.articulation(
        "pan",
        ArticulationType.REVOLUTE,
        parent=forearm,
        child=yoke,
        origin=Origin(xyz=(0.65, 0.0, -0.050)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.4, lower=-2.60, upper=2.60),
    )
    model.articulation(
        "tilt",
        ArticulationType.REVOLUTE,
        parent=yoke,
        child=lamp_head,
        origin=Origin(xyz=(0.0, 0.0, -0.270)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=1.0, lower=-0.95, upper=0.95),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    ceiling_plate = object_model.get_part("ceiling_plate")
    upper_arm = object_model.get_part("upper_arm")
    forearm = object_model.get_part("forearm")
    yoke = object_model.get_part("yoke")
    lamp_head = object_model.get_part("lamp_head")
    shoulder = object_model.get_articulation("shoulder")
    elbow = object_model.get_articulation("elbow")
    pan = object_model.get_articulation("pan")
    tilt = object_model.get_articulation("tilt")

    ctx.check(
        "four prompted revolute joints are present",
        all(j.articulation_type == ArticulationType.REVOLUTE for j in (shoulder, elbow, pan, tilt)),
    )
    ctx.check(
        "arm chain has five semantic links",
        {p.name for p in object_model.parts}
        == {"ceiling_plate", "upper_arm", "forearm", "yoke", "lamp_head"},
    )

    ctx.expect_contact(
        ceiling_plate,
        upper_arm,
        elem_a="bottom_collar",
        elem_b="shoulder_hub",
        contact_tol=0.001,
        name="shoulder hub seats against ceiling collar",
    )
    ctx.expect_overlap(
        ceiling_plate,
        upper_arm,
        axes="xy",
        elem_a="bottom_collar",
        elem_b="shoulder_hub",
        min_overlap=0.10,
        name="shoulder pivot is centered under plate",
    )
    ctx.expect_contact(
        upper_arm,
        forearm,
        elem_a="elbow_fork_0",
        elem_b="elbow_knuckle",
        contact_tol=0.002,
        name="forearm knuckle is captured by elbow fork",
    )
    ctx.expect_contact(
        forearm,
        yoke,
        elem_a="pan_socket",
        elem_b="pan_stem",
        contact_tol=0.001,
        name="pan stem seats in forearm socket",
    )
    ctx.expect_contact(
        yoke,
        lamp_head,
        elem_a="tilt_boss_0",
        elem_b="trunnion",
        contact_tol=0.002,
        name="lamp trunnion reaches the yoke tilt boss",
    )

    rest_pos = ctx.part_world_position(lamp_head)
    with ctx.pose({shoulder: 0.65}):
        swept_pos = ctx.part_world_position(lamp_head)
    ctx.check(
        "shoulder pan sweeps the arm sideways",
        rest_pos is not None and swept_pos is not None and swept_pos[1] > rest_pos[1] + 0.75,
        details=f"rest={rest_pos}, swept={swept_pos}",
    )

    with ctx.pose({elbow: 0.90}):
        folded_pos = ctx.part_world_position(lamp_head)
    ctx.check(
        "elbow bends the forearm downward",
        rest_pos is not None and folded_pos is not None and folded_pos[2] < rest_pos[2] - 0.30,
        details=f"rest={rest_pos}, folded={folded_pos}",
    )

    rest_aabb = ctx.part_world_aabb(lamp_head)
    with ctx.pose({tilt: 0.65}):
        tilted_aabb = ctx.part_world_aabb(lamp_head)
    if rest_aabb is not None and tilted_aabb is not None:
        rest_center_x = (rest_aabb[0][0] + rest_aabb[1][0]) / 2.0
        tilted_center_x = (tilted_aabb[0][0] + tilted_aabb[1][0]) / 2.0
        tilt_ok = tilted_center_x < rest_center_x - 0.04
    else:
        rest_center_x = tilted_center_x = None
        tilt_ok = False
    ctx.check(
        "tilt joint pitches the round head in the yoke",
        tilt_ok,
        details=f"rest_center_x={rest_center_x}, tilted_center_x={tilted_center_x}",
    )

    return ctx.report()


object_model = build_object_model()
