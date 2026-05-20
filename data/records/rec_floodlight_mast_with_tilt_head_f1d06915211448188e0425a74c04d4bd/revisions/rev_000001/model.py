from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


BLACK = Material("black_powdercoat", rgba=(0.02, 0.022, 0.024, 1.0))
DARK = Material("dark_cast_metal", rgba=(0.09, 0.095, 0.10, 1.0))
RUBBER = Material("matte_gasket", rgba=(0.005, 0.006, 0.007, 1.0))
GLASS = Material("cool_glass", rgba=(0.55, 0.78, 0.92, 0.45))
LED = Material("warm_led_phosphor", rgba=(1.0, 0.82, 0.30, 1.0))
WALL = Material("painted_wall", rgba=(0.74, 0.72, 0.68, 1.0))
SCREW = Material("brushed_screw_heads", rgba=(0.55, 0.55, 0.52, 1.0))


def cylinder_x(radius: float, length: float) -> Cylinder:
    return Cylinder(radius=radius, length=length)


def origin_x(xyz: tuple[float, float, float]) -> Origin:
    """Cylinder local +Z rotated onto world +X."""
    return Origin(xyz=xyz, rpy=(0.0, math.pi / 2.0, 0.0))


def origin_y(xyz: tuple[float, float, float]) -> Origin:
    """Cylinder local +Z rotated onto world +Y."""
    return Origin(xyz=xyz, rpy=(-math.pi / 2.0, 0.0, 0.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="security_perimeter_floodlight")
    for mat in (BLACK, DARK, RUBBER, GLASS, LED, WALL, SCREW):
        model.material(mat.name, rgba=mat.rgba)

    # Fixed wall plate, welded tubular standoff arm, and the stationary half of
    # the azimuth bearing are one rigid root assembly.
    mount = model.part("wall_mount")
    mount.visual(
        Box((0.025, 0.44, 0.54)),
        origin=Origin(xyz=(-0.029, 0.0, 0.0)),
        material=WALL,
        name="wall_patch",
    )
    mount.visual(
        Box((0.036, 0.22, 0.34)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=BLACK,
        name="mount_plate",
    )
    for sy in (-0.072, 0.072):
        for sz in (-0.118, 0.118):
            mount.visual(
                Cylinder(radius=0.017, length=0.008),
                origin=origin_x((0.020, sy, sz)),
                material=SCREW,
                name=f"screw_{sy:+.3f}_{sz:+.3f}",
            )
            mount.visual(
                Cylinder(radius=0.006, length=0.008),
                origin=origin_x((0.024, sy, sz)),
                material=DARK,
                name=f"screw_recess_{sy:+.3f}_{sz:+.3f}",
            )

    arm_tip_x = 0.435
    arm_z = 0.030
    mount.visual(
        Cylinder(radius=0.022, length=0.425),
        origin=origin_x((0.226, 0.0, arm_z)),
        material=BLACK,
        name="tubular_arm",
    )
    mount.visual(
        Cylinder(radius=0.058, length=0.055),
        origin=origin_x((0.039, 0.0, arm_z)),
        material=BLACK,
        name="wall_boss",
    )
    mount.visual(
        Box((0.330, 0.026, 0.024)),
        origin=Origin(xyz=(0.190, 0.0, -0.030), rpy=(0.0, -0.30, 0.0)),
        material=BLACK,
        name="diagonal_gusset",
    )
    mount.visual(
        Cylinder(radius=0.047, length=0.052),
        origin=Origin(xyz=(arm_tip_x, 0.0, 0.056)),
        material=BLACK,
        name="bearing_socket",
    )
    mount.visual(
        Box((0.060, 0.060, 0.040)),
        origin=Origin(xyz=(arm_tip_x - 0.030, 0.0, 0.040)),
        material=BLACK,
        name="tip_block",
    )

    # Pan unit: a compact turntable and U-yoke.  Its frame is at the lower face
    # of the rotating bearing disk and rotates continuously around local +Z.
    pan = model.part("pan_yoke")
    pan.visual(
        Cylinder(radius=0.045, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=DARK,
        name="turntable",
    )
    pan.visual(
        Cylinder(radius=0.027, length=0.068),
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
        material=BLACK,
        name="neck",
    )
    pan.visual(
        Box((0.125, 0.190, 0.030)),
        origin=Origin(xyz=(0.066, 0.0, 0.106)),
        material=BLACK,
        name="top_bridge",
    )
    for side, y in (("0", -0.102), ("1", 0.102)):
        pan.visual(
            Box((0.125, 0.018, 0.120)),
            origin=Origin(xyz=(0.135, y, 0.080)),
            material=BLACK,
            name=f"yoke_cheek_{side}",
        )
        pan.visual(
            Cylinder(radius=0.026, length=0.020),
            origin=origin_y((0.170, y, 0.080)),
            material=DARK,
            name=f"hinge_boss_{side}",
        )

    bearing_z = 0.082
    model.articulation(
        "azimuth_bearing",
        ArticulationType.CONTINUOUS,
        parent=mount,
        child=pan,
        origin=Origin(xyz=(arm_tip_x, 0.0, bearing_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=3.0),
    )

    # Flood head frame sits on the tilt hinge axis.  The lamp housing projects
    # along local +X, so positive motion about -Y pitches the beam upward.
    head = model.part("flood_head")
    head.visual(
        Box((0.220, 0.160, 0.118)),
        origin=Origin(xyz=(0.105, 0.0, 0.0)),
        material=DARK,
        name="rear_housing",
    )
    head.visual(
        Cylinder(radius=0.021, length=0.184),
        origin=origin_y((0.0, 0.0, 0.0)),
        material=DARK,
        name="tilt_axle",
    )
    for y in (-0.055, -0.028, 0.0, 0.028, 0.055):
        head.visual(
            Box((0.030, 0.006, 0.098)),
            origin=Origin(xyz=(-0.021, y, 0.0)),
            material=BLACK,
            name=f"heat_sink_{y:+.3f}",
        )
    head.visual(
        Box((0.012, 0.152, 0.100)),
        origin=Origin(xyz=(0.218, 0.0, 0.0)),
        material=GLASS,
        name="glass_lens",
    )
    head.visual(
        Box((0.014, 0.174, 0.016)),
        origin=Origin(xyz=(0.224, 0.0, 0.055)),
        material=BLACK,
        name="top_bezel",
    )
    head.visual(
        Box((0.014, 0.174, 0.016)),
        origin=Origin(xyz=(0.224, 0.0, -0.055)),
        material=BLACK,
        name="bottom_bezel",
    )
    for side, y in (("0", -0.083), ("1", 0.083)):
        head.visual(
            Box((0.014, 0.016, 0.112)),
            origin=Origin(xyz=(0.224, y, 0.0)),
            material=BLACK,
            name=f"side_bezel_{side}",
        )
    led_positions = [(-0.045, 0.025), (0.0, 0.025), (0.045, 0.025), (-0.045, -0.025), (0.0, -0.025), (0.045, -0.025)]
    for i, (y, z) in enumerate(led_positions):
        head.visual(
            Box((0.006, 0.020, 0.020)),
            origin=Origin(xyz=(0.2265, y, z)),
            material=LED,
            name=f"led_{i}",
        )
    head.visual(
        Box((0.108, 0.180, 0.012)),
        origin=Origin(xyz=(0.172, 0.0, 0.063), rpy=(0.0, -0.10, 0.0)),
        material=BLACK,
        name="rain_visor",
    )
    head.visual(
        Box((0.042, 0.042, 0.020)),
        origin=Origin(xyz=(-0.003, 0.0, 0.0)),
        material=DARK,
        name="hinge_hub",
    )

    model.articulation(
        "tilt_hinge",
        ArticulationType.REVOLUTE,
        parent=pan,
        child=head,
        origin=Origin(xyz=(0.170, 0.0, 0.080)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.5, lower=-0.75, upper=0.95),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    mount = object_model.get_part("wall_mount")
    pan = object_model.get_part("pan_yoke")
    head = object_model.get_part("flood_head")
    azimuth = object_model.get_articulation("azimuth_bearing")
    tilt = object_model.get_articulation("tilt_hinge")

    ctx.check(
        "azimuth joint is continuous",
        azimuth.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={azimuth.articulation_type}",
    )
    ctx.check(
        "tilt joint is limited revolute",
        tilt.articulation_type == ArticulationType.REVOLUTE
        and tilt.motion_limits is not None
        and tilt.motion_limits.lower < 0.0
        and tilt.motion_limits.upper > 0.0,
        details=f"type={tilt.articulation_type}, limits={tilt.motion_limits}",
    )

    ctx.expect_gap(
        pan,
        mount,
        axis="z",
        positive_elem="turntable",
        negative_elem="bearing_socket",
        max_gap=0.002,
        max_penetration=0.0,
        name="turntable sits on fixed bearing socket",
    )
    ctx.expect_overlap(
        pan,
        mount,
        axes="xy",
        elem_a="turntable",
        elem_b="bearing_socket",
        min_overlap=0.040,
        name="azimuth bearing disks are coaxial",
    )
    ctx.expect_within(
        head,
        pan,
        axes="y",
        inner_elem="rear_housing",
        outer_elem="top_bridge",
        margin=0.004,
        name="lamp body fits inside yoke width",
    )

    rest_pos = ctx.part_world_position(head)
    with ctx.pose({azimuth: 1.2}):
        panned_pos = ctx.part_world_position(head)
    ctx.check(
        "continuous pan swings flood head around arm tip",
        rest_pos is not None and panned_pos is not None and abs(panned_pos[1] - rest_pos[1]) > 0.10,
        details=f"rest={rest_pos}, panned={panned_pos}",
    )

    rest_aabb = ctx.part_world_aabb(head)
    with ctx.pose({tilt: 0.55}):
        tilted_aabb = ctx.part_world_aabb(head)
    if rest_aabb is not None and tilted_aabb is not None:
        rest_center_z = 0.5 * (rest_aabb[0][2] + rest_aabb[1][2])
        tilted_center_z = 0.5 * (tilted_aabb[0][2] + tilted_aabb[1][2])
        tilt_ok = tilted_center_z > rest_center_z + 0.035
    else:
        rest_center_z = tilted_center_z = None
        tilt_ok = False
    ctx.check(
        "positive tilt raises beam",
        tilt_ok,
        details=f"rest_center_z={rest_center_z}, tilted_center_z={tilted_center_z}",
    )

    return ctx.report()


object_model = build_object_model()
