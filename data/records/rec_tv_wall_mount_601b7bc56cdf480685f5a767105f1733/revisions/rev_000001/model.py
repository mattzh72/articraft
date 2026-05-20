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


BLACK = Material("satin_black_powdercoat", rgba=(0.015, 0.014, 0.013, 1.0))
EDGE_BLACK = Material("black_plastic_caps", rgba=(0.0, 0.0, 0.0, 1.0))
PIN_STEEL = Material("brushed_steel_pins", rgba=(0.55, 0.56, 0.54, 1.0))
WALL_DARK = Material("dark_grey_wall_plate", rgba=(0.08, 0.085, 0.085, 1.0))


def _origin(x: float, y: float, z: float, rpy=(0.0, 0.0, 0.0)) -> Origin:
    return Origin(xyz=(x, y, z), rpy=rpy)


def _add_vertical_barrel(part, name: str, x: float, y: float, z: float, radius: float, length: float, material) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=_origin(x, y, z),
        material=material,
        name=name,
    )


def _add_arm_link(part, length: float, *, material=BLACK) -> None:
    """A compact steel link with a central proximal barrel and a forked distal end."""
    barrel_radius = 0.028
    central_len = 0.048
    lug_len = 0.034
    lug_z = 0.041

    _add_vertical_barrel(part, "proximal_barrel", 0.0, 0.0, 0.0, barrel_radius, central_len, PIN_STEEL)

    # Main rectangular tube: deliberately overlaps the proximal barrel and a rear web
    # so the link reads as one welded part rather than floating hinge pieces.
    part.visual(
        Box((length - 0.095, 0.050, 0.045)),
        origin=_origin((0.018 + length - 0.077) * 0.5, 0.0, 0.0),
        material=material,
        name="link_tube",
    )
    part.visual(
        Box((0.030, 0.052, 0.106)),
        origin=_origin(length - 0.076, 0.0, 0.0),
        material=material,
        name="distal_web",
    )

    # Distal fork leaves the middle z-band open for the next link's center barrel.
    part.visual(
        Box((0.070, 0.052, 0.022)),
        origin=_origin(length - 0.041, 0.0, lug_z),
        material=material,
        name="distal_upper_bridge",
    )
    part.visual(
        Box((0.070, 0.052, 0.022)),
        origin=_origin(length - 0.041, 0.0, -lug_z),
        material=material,
        name="distal_lower_bridge",
    )
    _add_vertical_barrel(part, "distal_upper_knuckle", length, 0.0, lug_z, barrel_radius, lug_len, material)
    _add_vertical_barrel(part, "distal_lower_knuckle", length, 0.0, -lug_z, barrel_radius, lug_len, material)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="short_arm_tv_wall_bracket")

    wall_plate = model.part("wall_plate")
    wall_plate.visual(
        Box((0.035, 0.240, 0.420)),
        origin=_origin(0.0, 0.0, 0.250),
        material=WALL_DARK,
        name="back_plate",
    )
    wall_plate.visual(
        Box((0.018, 0.105, 0.330)),
        origin=_origin(0.026, 0.0, 0.250),
        material=BLACK,
        name="raised_spine",
    )
    wall_plate.visual(
        Box((0.014, 0.025, 0.390)),
        origin=_origin(0.023, 0.102, 0.250),
        material=BLACK,
        name="side_rib_0",
    )
    wall_plate.visual(
        Box((0.014, 0.025, 0.390)),
        origin=_origin(0.023, -0.102, 0.250),
        material=BLACK,
        name="side_rib_1",
    )

    # Four raised screw bosses on the wall plate, with dark screw recesses.
    for idx, (y, z) in enumerate(((-0.068, 0.095), (0.068, 0.095), (-0.068, 0.405), (0.068, 0.405))):
        wall_plate.visual(
            Cylinder(radius=0.018, length=0.009),
            origin=_origin(0.022, y, z, rpy=(0.0, math.pi / 2.0, 0.0)),
            material=PIN_STEEL,
            name=f"screw_boss_{idx}",
        )
        wall_plate.visual(
            Cylinder(radius=0.008, length=0.003),
            origin=_origin(0.028, y, z, rpy=(0.0, math.pi / 2.0, 0.0)),
            material=EDGE_BLACK,
            name=f"screw_recess_{idx}",
        )

    # Shoulder clevis protruding from the wall plate.
    shoulder_x = 0.080
    shoulder_z = 0.250
    lug_z = 0.041
    for suffix, dz in (("upper", lug_z), ("lower", -lug_z)):
        wall_plate.visual(
            Box((0.082, 0.064, 0.026)),
            origin=_origin(0.047, 0.0, shoulder_z + dz),
            material=BLACK,
            name=f"shoulder_{suffix}_strap",
        )
        _add_vertical_barrel(
            wall_plate,
            f"shoulder_{suffix}_knuckle",
            shoulder_x,
            0.0,
            shoulder_z + dz,
            0.030,
            0.034,
            BLACK,
        )

    arm_0 = model.part("arm_0")
    _add_arm_link(arm_0, 0.220)

    arm_1 = model.part("arm_1")
    _add_arm_link(arm_1, 0.180)

    pan_head = model.part("pan_head")
    _add_vertical_barrel(pan_head, "swivel_barrel", 0.0, 0.0, 0.0, 0.028, 0.048, PIN_STEEL)
    pan_head.visual(
        Box((0.068, 0.066, 0.030)),
        origin=_origin(0.034, 0.0, 0.0),
        material=BLACK,
        name="swivel_block",
    )
    pan_head.visual(
        Box((0.026, 0.022, 0.070)),
        origin=_origin(0.060, 0.042, 0.0),
        material=BLACK,
        name="side_web_0",
    )
    pan_head.visual(
        Box((0.026, 0.022, 0.070)),
        origin=_origin(0.060, -0.042, 0.0),
        material=BLACK,
        name="side_web_1",
    )
    pan_head.visual(
        Box((0.060, 0.165, 0.020)),
        origin=_origin(0.083, 0.0, 0.039),
        material=BLACK,
        name="upper_yoke_bridge",
    )
    pan_head.visual(
        Box((0.060, 0.165, 0.020)),
        origin=_origin(0.083, 0.0, -0.039),
        material=BLACK,
        name="lower_yoke_bridge",
    )
    pan_head.visual(
        Box((0.035, 0.031, 0.108)),
        origin=_origin(0.102, 0.068, 0.0),
        material=BLACK,
        name="tilt_cheek_0",
    )
    pan_head.visual(
        Box((0.035, 0.031, 0.108)),
        origin=_origin(0.102, -0.068, 0.0),
        material=BLACK,
        name="tilt_cheek_1",
    )

    tilt_head = model.part("tilt_head")
    tilt_head.visual(
        Cylinder(radius=0.022, length=0.105),
        origin=_origin(0.0, 0.0, 0.0, rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=PIN_STEEL,
        name="tilt_barrel",
    )
    tilt_head.visual(
        Box((0.130, 0.054, 0.036)),
        origin=_origin(0.065, 0.0, 0.0),
        material=BLACK,
        name="head_neck",
    )
    tilt_head.visual(
        Box((0.026, 0.180, 0.135)),
        origin=_origin(0.141, 0.0, 0.0),
        material=BLACK,
        name="mount_plate",
    )
    for idx, (y, z) in enumerate(((-0.055, -0.040), (0.055, -0.040), (-0.055, 0.040), (0.055, 0.040))):
        tilt_head.visual(
            Cylinder(radius=0.010, length=0.007),
            origin=_origin(0.1575, y, z, rpy=(0.0, math.pi / 2.0, 0.0)),
            material=PIN_STEEL,
            name=f"vesa_boss_{idx}",
        )
        tilt_head.visual(
            Cylinder(radius=0.0055, length=0.003),
            origin=_origin(0.1625, y, z, rpy=(0.0, math.pi / 2.0, 0.0)),
            material=EDGE_BLACK,
            name=f"vesa_hole_{idx}",
        )

    shoulder = model.articulation(
        "wall_to_arm_0",
        ArticulationType.REVOLUTE,
        parent=wall_plate,
        child=arm_0,
        origin=_origin(shoulder_x, 0.0, shoulder_z),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=30.0, velocity=1.5, lower=-1.35, upper=1.35),
    )
    elbow = model.articulation(
        "arm_0_to_arm_1",
        ArticulationType.REVOLUTE,
        parent=arm_0,
        child=arm_1,
        origin=_origin(0.220, 0.0, 0.0),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=25.0, velocity=1.5, lower=-2.20, upper=2.20),
    )
    swivel = model.articulation(
        "arm_1_to_pan",
        ArticulationType.REVOLUTE,
        parent=arm_1,
        child=pan_head,
        origin=_origin(0.180, 0.0, 0.0),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.2, lower=-1.40, upper=1.40),
    )
    tilt = model.articulation(
        "pan_to_tilt",
        ArticulationType.REVOLUTE,
        parent=pan_head,
        child=tilt_head,
        origin=_origin(0.102, 0.0, 0.0),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.0, lower=-0.45, upper=0.35),
    )

    # Keep local variables referenced so their semantic intent is visible in the authored graph.
    assert shoulder and elbow and swivel and tilt
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    joints = [
        object_model.get_articulation("wall_to_arm_0"),
        object_model.get_articulation("arm_0_to_arm_1"),
        object_model.get_articulation("arm_1_to_pan"),
        object_model.get_articulation("pan_to_tilt"),
    ]
    ctx.check(
        "four revolute bracket joints",
        len(object_model.articulations) == 4 and all(j.articulation_type == ArticulationType.REVOLUTE for j in joints),
        details=f"joints={[j.name for j in object_model.articulations]}",
    )

    wall_plate = object_model.get_part("wall_plate")
    arm_0 = object_model.get_part("arm_0")
    arm_1 = object_model.get_part("arm_1")
    pan_head = object_model.get_part("pan_head")
    tilt_head = object_model.get_part("tilt_head")

    ctx.expect_contact(
        wall_plate,
        arm_0,
        elem_a="shoulder_upper_knuckle",
        elem_b="proximal_barrel",
        contact_tol=0.001,
        name="wall clevis captures first arm barrel",
    )
    ctx.expect_contact(
        arm_0,
        arm_1,
        elem_a="distal_upper_knuckle",
        elem_b="proximal_barrel",
        contact_tol=0.001,
        name="elbow fork captures second arm barrel",
    )
    ctx.expect_contact(
        arm_1,
        pan_head,
        elem_a="distal_upper_knuckle",
        elem_b="swivel_barrel",
        contact_tol=0.001,
        name="outer arm captures pan swivel barrel",
    )
    ctx.expect_contact(
        pan_head,
        tilt_head,
        elem_a="tilt_cheek_0",
        elem_b="tilt_barrel",
        contact_tol=0.001,
        name="tilt yoke captures horizontal barrel",
    )

    arm_1_rest = ctx.part_world_position(arm_1)
    with ctx.pose({"wall_to_arm_0": 0.65}):
        arm_1_swung = ctx.part_world_position(arm_1)
    ctx.check(
        "wall joint swings the arm sideways",
        arm_1_rest is not None
        and arm_1_swung is not None
        and abs(arm_1_swung[1] - arm_1_rest[1]) > 0.10,
        details=f"rest={arm_1_rest}, swung={arm_1_swung}",
    )

    rest_plate_aabb = ctx.part_element_world_aabb(tilt_head, elem="mount_plate")
    with ctx.pose({"pan_to_tilt": 0.30}):
        tilted_plate_aabb = ctx.part_element_world_aabb(tilt_head, elem="mount_plate")
    if rest_plate_aabb is not None and tilted_plate_aabb is not None:
        rest_center_z = 0.5 * (rest_plate_aabb[0][2] + rest_plate_aabb[1][2])
        tilted_center_z = 0.5 * (tilted_plate_aabb[0][2] + tilted_plate_aabb[1][2])
    else:
        rest_center_z = tilted_center_z = 0.0
    ctx.check(
        "tilt joint pitches the mounting head",
        rest_plate_aabb is not None
        and tilted_plate_aabb is not None
        and abs(tilted_center_z - rest_center_z) > 0.025,
        details=f"rest_aabb={rest_plate_aabb}, tilted_aabb={tilted_plate_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
