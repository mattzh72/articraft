from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


OPEN_ANGLE = 0.35
HINGE_Y = 0.43
HINGE_Z = 0.205
MOVING_KNUCKLES = ("moving_knuckle_0", "moving_knuckle_1")
FRAME_PROP_PINS = ("frame_prop_pin_0", "frame_prop_pin_1")
LID_PROP_SOCKETS = ("lid_prop_socket_0", "lid_prop_socket_1")


def _ring_mesh(
    outer_x: float,
    outer_y: float,
    inner_x: float,
    inner_y: float,
    height: float,
    name: str,
):
    """Return a single-piece rectangular ring mesh centered on its local origin."""
    outer = cq.Workplane("XY").box(outer_x, outer_y, height)
    cutter = cq.Workplane("XY").box(inner_x, inner_y, height * 3.0)
    return mesh_from_cadquery(outer.cut(cutter), name, tolerance=0.002)


def _lid_open_origin(
    xyz: tuple[float, float, float],
    *,
    extra_rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> Origin:
    """Place a closed-lid feature at the nominal partly-open display angle.

    The child part frame remains exactly on the hinge line.  This transform only
    pitches the visible lid construction upward around that local hinge line so
    the default pose shows the hatch propped open; the lower joint limit returns
    the same geometry to a closed, horizontal position.
    """
    x, y, z = xyz
    c = math.cos(OPEN_ANGLE)
    s = math.sin(OPEN_ANGLE)
    return Origin(
        xyz=(x, y * c + z * s, -y * s + z * c),
        rpy=(extra_rpy[0] - OPEN_ANGLE, extra_rpy[1], extra_rpy[2]),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="roof_skylight_hatch")

    curb_mat = model.material("warm_white_curb", rgba=(0.82, 0.80, 0.72, 1.0))
    roof_mat = model.material("dark_roof_membrane", rgba=(0.08, 0.09, 0.10, 1.0))
    rubber_mat = model.material("black_rubber_gasket", rgba=(0.01, 0.012, 0.012, 1.0))
    metal_mat = model.material("brushed_aluminum", rgba=(0.62, 0.66, 0.68, 1.0))
    arm_mat = model.material("galvanized_steel", rgba=(0.47, 0.50, 0.51, 1.0))
    glass_mat = model.material("clear_blue_glass", rgba=(0.55, 0.82, 1.0, 0.32))

    curb = model.part("curb")
    curb.visual(
        _ring_mesh(1.58, 1.12, 1.04, 0.56, 0.018, "roof_flashing"),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=roof_mat,
        name="roof_flashing",
    )
    curb.visual(
        _ring_mesh(1.20, 0.72, 1.04, 0.56, 0.180, "curb_wall"),
        origin=Origin(xyz=(0.0, 0.0, 0.090)),
        material=curb_mat,
        name="curb_wall",
    )
    curb.visual(
        _ring_mesh(1.14, 0.66, 1.02, 0.54, 0.008, "top_gasket"),
        origin=Origin(xyz=(0.0, 0.0, 0.184)),
        material=rubber_mat,
        name="top_gasket",
    )

    # Hinge pedestals and fixed hinge knuckles live behind the raised curb, so
    # they read as real supports without intersecting the transparent sash.
    for i, x in enumerate((-0.45, 0.0, 0.45)):
        curb.visual(
            Box((0.16, 0.024, 0.205)),
            origin=Origin(xyz=(x, HINGE_Y, 0.1025)),
            material=metal_mat,
            name=f"hinge_pedestal_{i}",
        )
        curb.visual(
            Cylinder(radius=0.013, length=0.16),
            origin=Origin(xyz=(x, HINGE_Y, HINGE_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=metal_mat,
            name=f"fixed_knuckle_{i}",
        )
    curb.visual(
        Cylinder(radius=0.006, length=1.10),
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=metal_mat,
        name="hinge_pin",
    )

    # Clevis-like side brackets for the two prop-arm lower pivots.
    for i, x in enumerate((-0.631, 0.631)):
        curb.visual(
            Box((0.066, 0.090, 0.120)),
            origin=Origin(xyz=(x, -0.360, 0.075)),
            material=metal_mat,
            name=f"frame_prop_bracket_{i}",
        )
        pin_x = -0.675 if x < 0.0 else 0.675
        curb.visual(
            Cylinder(radius=0.008, length=0.046),
            origin=Origin(xyz=(pin_x, -0.360, 0.095), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=metal_mat,
            name=FRAME_PROP_PINS[i],
        )

    lid = model.part("lid")
    lid.visual(
        _ring_mesh(1.32, 0.84, 1.10, 0.62, 0.030, "lid_sash"),
        origin=_lid_open_origin((0.0, -0.46, 0.0)),
        material=metal_mat,
        name="sash_ring",
    )
    lid.visual(
        Box((1.14, 0.66, 0.010)),
        origin=_lid_open_origin((0.0, -0.46, 0.004)),
        material=glass_mat,
        name="clear_pane",
    )

    # Moving hinge knuckles and short leaves keep the sash clipped exactly to
    # the curb hinge line while leaving alternating gaps for the fixed knuckles.
    for i, x in enumerate((-0.225, 0.225)):
        lid.visual(
            Cylinder(radius=0.012, length=0.16),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=metal_mat,
            name=MOVING_KNUCKLES[i],
        )
        lid.visual(
            Box((0.16, 0.080, 0.004)),
            origin=_lid_open_origin((x, -0.040, -0.014)),
            material=metal_mat,
            name=f"hinge_leaf_{i}",
        )

    # Side rails with small clevis plates where the prop arms meet the lid.
    for i, x in enumerate((-0.665, 0.665)):
        lid.visual(
            Box((0.020, 0.56, 0.018)),
            origin=_lid_open_origin((x, -0.50, -0.026)),
            material=metal_mat,
            name=f"prop_lid_rail_{i}",
        )
        lid.visual(
            Cylinder(radius=0.018, length=0.020),
            origin=_lid_open_origin((x, -0.55, -0.030), extra_rpy=(0.0, math.pi / 2.0, 0.0)),
            material=metal_mat,
            name=LID_PROP_SOCKETS[i],
        )

    # Compute the default-open prop arm geometry to reach from the curb brackets
    # to the lid side sockets.  The arm joints are mimicked to the lid hinge.
    c = math.cos(OPEN_ANGLE)
    s = math.sin(OPEN_ANGLE)
    socket_y = HINGE_Y + (-0.55 * c + -0.030 * s)
    socket_z = HINGE_Z + (0.55 * s + -0.030 * c)
    pivot_y = -0.360
    pivot_z = 0.095
    arm_dy = socket_y - pivot_y
    arm_dz = socket_z - pivot_z
    arm_len = math.hypot(arm_dy, arm_dz)
    arm_roll = math.atan2(arm_dz, arm_dy)

    for i, x in enumerate((-0.690, 0.690)):
        arm = model.part(f"prop_arm_{i}")
        arm.visual(
            Cylinder(radius=0.021, length=0.026),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=arm_mat,
            name="lower_eye",
        )
        arm.visual(
            Box((0.014, arm_len - 0.032, 0.010)),
            origin=Origin(xyz=(0.0, arm_dy / 2.0, arm_dz / 2.0), rpy=(arm_roll, 0.0, 0.0)),
            material=arm_mat,
            name="arm_bar",
        )
        arm.visual(
            Cylinder(radius=0.017, length=0.026),
            origin=Origin(
                xyz=(0.0, arm_dy, arm_dz),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=arm_mat,
            name="upper_eye",
        )
        model.articulation(
            f"curb_to_prop_arm_{i}",
            ArticulationType.REVOLUTE,
            parent=curb,
            child=arm,
            origin=Origin(xyz=(x, pivot_y, pivot_z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=15.0, velocity=2.0, lower=-OPEN_ANGLE, upper=0.65),
            mimic=Mimic("curb_to_lid", multiplier=1.0, offset=0.0),
        )

    model.articulation(
        "curb_to_lid",
        ArticulationType.REVOLUTE,
        parent=curb,
        child=lid,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=1.5, lower=-OPEN_ANGLE, upper=0.65),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    curb = object_model.get_part("curb")
    lid = object_model.get_part("lid")
    arm_0 = object_model.get_part("prop_arm_0")
    arm_1 = object_model.get_part("prop_arm_1")
    lid_hinge = object_model.get_articulation("curb_to_lid")

    ctx.allow_overlap(
        curb,
        lid,
        elem_a="hinge_pin",
        elem_b=MOVING_KNUCKLES[0],
        reason="The hinge pin is intentionally captured inside the moving hinge barrel.",
    )
    ctx.allow_overlap(
        curb,
        lid,
        elem_a="hinge_pin",
        elem_b=MOVING_KNUCKLES[1],
        reason="The hinge pin is intentionally captured inside the moving hinge barrel.",
    )
    ctx.allow_overlap(
        curb,
        arm_0,
        elem_a=FRAME_PROP_PINS[0],
        elem_b="lower_eye",
        reason="The prop-arm lower eye intentionally rotates around its frame pivot pin.",
    )
    ctx.allow_overlap(
        curb,
        arm_1,
        elem_a=FRAME_PROP_PINS[1],
        elem_b="lower_eye",
        reason="The prop-arm lower eye intentionally rotates around its frame pivot pin.",
    )
    ctx.allow_overlap(
        lid,
        arm_0,
        elem_a=LID_PROP_SOCKETS[0],
        elem_b="upper_eye",
        reason="The prop-arm upper eye is intentionally captured on the lid-side pivot boss.",
    )
    ctx.allow_overlap(
        lid,
        arm_1,
        elem_a=LID_PROP_SOCKETS[1],
        elem_b="upper_eye",
        reason="The prop-arm upper eye is intentionally captured on the lid-side pivot boss.",
    )

    for i in (0, 1):
        ctx.expect_overlap(
            lid,
            curb,
            axes="x",
            elem_a=MOVING_KNUCKLES[i],
            elem_b="hinge_pin",
            min_overlap=0.12,
            name=f"moving hinge knuckle {i} is retained on pin",
        )
        ctx.expect_overlap(
            lid,
            curb,
            axes="yz",
            elem_a=MOVING_KNUCKLES[i],
            elem_b="hinge_pin",
            min_overlap=0.008,
            name=f"moving hinge knuckle {i} is coaxial with pin",
        )
    for i, arm in enumerate((arm_0, arm_1)):
        ctx.expect_overlap(
            arm,
            curb,
            axes="x",
            elem_a="lower_eye",
            elem_b=FRAME_PROP_PINS[i],
            min_overlap=0.010,
            name=f"prop arm {i} lower eye is retained on frame pin",
        )
        ctx.expect_overlap(
            arm,
            curb,
            axes="yz",
            elem_a="lower_eye",
            elem_b=FRAME_PROP_PINS[i],
            min_overlap=0.010,
            name=f"prop arm {i} lower eye is coaxial with frame pin",
        )
        ctx.expect_overlap(
            arm,
            lid,
            axes="x",
            elem_a="upper_eye",
            elem_b=LID_PROP_SOCKETS[i],
            min_overlap=0.002,
            name=f"prop arm {i} upper eye is retained on lid pivot",
        )
        ctx.expect_overlap(
            arm,
            lid,
            axes="yz",
            elem_a="upper_eye",
            elem_b=LID_PROP_SOCKETS[i],
            min_overlap=0.020,
            name=f"prop arm {i} upper eye is coaxial with lid pivot",
        )

    with ctx.pose({lid_hinge: -OPEN_ANGLE}):
        ctx.expect_gap(
            lid,
            curb,
            axis="z",
            positive_elem="sash_ring",
            negative_elem="top_gasket",
            min_gap=0.0,
            max_gap=0.010,
            name="closed lid seats just above curb gasket",
        )
        ctx.expect_overlap(
            lid,
            curb,
            axes="xy",
            elem_a="clear_pane",
            elem_b="curb_wall",
            min_overlap=0.50,
            name="clear lid spans the curb opening when closed",
        )

    default_lid_origin = ctx.part_world_position(lid)
    default_arm_0 = ctx.part_element_world_aabb(arm_0, elem="upper_eye")
    default_arm_1 = ctx.part_element_world_aabb(arm_1, elem="upper_eye")
    default_lid_aabb = ctx.part_world_aabb(lid)

    with ctx.pose({lid_hinge: 0.65}):
        open_lid_origin = ctx.part_world_position(lid)
        open_arm_0 = ctx.part_element_world_aabb(arm_0, elem="upper_eye")
        open_arm_1 = ctx.part_element_world_aabb(arm_1, elem="upper_eye")
        open_lid_aabb = ctx.part_world_aabb(lid)

    ctx.check(
        "lid remains clipped to hinge line",
        default_lid_origin is not None
        and open_lid_origin is not None
        and all(abs(a - b) < 1e-6 for a, b in zip(default_lid_origin, open_lid_origin)),
        details=f"default={default_lid_origin}, open={open_lid_origin}",
    )
    ctx.check(
        "lid opens upward",
        default_lid_aabb is not None
        and open_lid_aabb is not None
        and open_lid_aabb[1][2] > default_lid_aabb[1][2] + 0.20,
        details=f"default={default_lid_aabb}, open={open_lid_aabb}",
    )
    ctx.check(
        "prop arms rotate upward with lid",
        default_arm_0 is not None
        and default_arm_1 is not None
        and open_arm_0 is not None
        and open_arm_1 is not None
        and open_arm_0[1][2] > default_arm_0[1][2] + 0.08
        and open_arm_1[1][2] > default_arm_1[1][2] + 0.08,
        details=f"default0={default_arm_0}, open0={open_arm_0}, default1={default_arm_1}, open1={open_arm_1}",
    )

    return ctx.report()


object_model = build_object_model()
