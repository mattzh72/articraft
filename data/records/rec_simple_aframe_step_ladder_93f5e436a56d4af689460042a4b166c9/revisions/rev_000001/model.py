from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


HINGE_Z = 0.74
FRONT_FOOT_X = -0.23
REAR_FOOT_X = 0.28
LEG_BOTTOM_Z = 0.02
REAR_HINGE_UPPER = 0.59


def _beam_between(part, name, start, end, size_x, size_y, material):
    """Add a rectangular member whose local Z axis follows start->end."""
    sx, sy, sz = start
    ex, ey, ez = end
    dx = ex - sx
    dy = ey - sy
    dz = ez - sz
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if abs(dy) > 1e-6:
        raise ValueError("_beam_between is intentionally limited to X/Z members")
    pitch = math.atan2(dx, dz)
    part.visual(
        Box((size_x, size_y, length)),
        origin=Origin(
            xyz=((sx + ex) * 0.5, (sy + ey) * 0.5, (sz + ez) * 0.5),
            rpy=(0.0, pitch, 0.0),
        ),
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
    model = ArticulatedObject(name="compact_aframe_step_ladder")

    aluminum = model.material("brushed_aluminum", rgba=(0.72, 0.72, 0.68, 1.0))
    dark_aluminum = model.material("dark_hinge_metal", rgba=(0.18, 0.20, 0.22, 1.0))
    tread_blue = model.material("soft_blue_tread", rgba=(0.18, 0.36, 0.62, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.02, 0.02, 0.018, 1.0))
    yellow_label = model.material("caution_label", rgba=(0.95, 0.72, 0.08, 1.0))

    front = model.part("front_frame")
    rear = model.part("rear_frame")
    brace_0 = model.part("brace_0")
    brace_1 = model.part("brace_1")

    # Front climbing frame: two splayed rectangular rails, three compact treads,
    # rubber feet, and exposed top hinge hardware.
    for sign, rail_name, foot_name in (
        (-1.0, "front_rail_0", "front_foot_0"),
        (1.0, "front_rail_1", "front_foot_1"),
    ):
        y = sign * 0.215
        _beam_between(
            front,
            rail_name,
            (FRONT_FOOT_X, y, LEG_BOTTOM_Z),
            (-0.025, y, HINGE_Z),
            0.035,
            0.028,
            aluminum,
        )
        front.visual(
            Box((0.095, 0.060, 0.026)),
            origin=Origin(xyz=(FRONT_FOOT_X - 0.012, y, 0.013)),
            material=black_rubber,
            name=foot_name,
        )

    tread_specs = (
        ("bottom_tread", -0.160, 0.215, 0.155, 0.440),
        ("middle_tread", -0.095, 0.415, 0.145, 0.430),
        ("top_platform", -0.055, 0.630, 0.160, 0.430),
    )
    for tread_name, x, z, depth, width in tread_specs:
        front.visual(
            Box((depth, width, 0.036)),
            origin=Origin(xyz=(x, 0.0, z)),
            material=tread_blue,
            name=tread_name,
        )
        for groove_idx, groove_x in enumerate((-0.045, 0.0, 0.045)):
            front.visual(
                Box((0.010, width * 0.82, 0.006)),
                origin=Origin(xyz=(x + groove_x, 0.0, z + 0.021)),
                material=black_rubber,
                name=f"{tread_name}_grip_{groove_idx}",
            )

    front.visual(
        Box((0.030, 0.455, 0.035)),
        origin=Origin(xyz=(-0.042, 0.0, HINGE_Z - 0.005)),
        material=aluminum,
        name="front_top_bridge",
    )
    _cylinder_y(front, "hinge_pin", 0.006, 0.490, (0.0, 0.0, HINGE_Z), dark_aluminum)
    _cylinder_y(front, "front_barrel_0", 0.018, 0.050, (0.0, -0.190, HINGE_Z), dark_aluminum)
    _cylinder_y(front, "front_barrel_1", 0.018, 0.050, (0.0, 0.190, HINGE_Z), dark_aluminum)

    # Rear support frame is narrower so it nests inside the front rails when folded.
    for sign, rail_name, foot_name in (
        (-1.0, "rear_rail_0", "rear_foot_0"),
        (1.0, "rear_rail_1", "rear_foot_1"),
    ):
        y = sign * 0.255
        foot_y = sign * 0.285
        _beam_between(
            rear,
            rail_name,
            (REAR_FOOT_X, y, LEG_BOTTOM_Z - HINGE_Z),
            (0.025, y, 0.0),
            0.032,
            0.026,
            aluminum,
        )
        rear.visual(
            Box((0.095, 0.056, 0.026)),
            origin=Origin(xyz=(REAR_FOOT_X + 0.012, foot_y, LEG_BOTTOM_Z - HINGE_Z - 0.007)),
            material=black_rubber,
            name=foot_name,
        )

    for cross_name, x, z, width in (
        ("rear_top_bridge", 0.032, -0.055, 0.530),
        ("rear_middle_crossbar", 0.165, -0.430, 0.520),
        ("rear_low_crossbar", 0.225, -0.590, 0.515),
    ):
        rear.visual(
            Box((0.032, width, 0.030)),
            origin=Origin(xyz=(x, 0.0, z)),
            material=aluminum,
            name=cross_name,
        )
    _cylinder_y(rear, "rear_barrel", 0.017, 0.128, (0.0, 0.0, 0.0), dark_aluminum)
    rear.visual(
        Box((0.010, 0.120, 0.060)),
        origin=Origin(xyz=(0.012, 0.0, -0.030)),
        material=dark_aluminum,
        name="rear_hinge_leaf",
    )

    # Rear stop/latch pins for the folding spread braces. They sit just outside
    # the rear rails and are intentionally captured by the slotted brace ends in
    # the open pose.
    for idx, sign in enumerate((-1.0, 1.0)):
        _cylinder_y(
            rear,
            f"latch_pin_{idx}",
            0.007,
            0.120,
            (0.155, sign * 0.250, -0.360),
            dark_aluminum,
        )

    # Front pivot pins for paired side spread braces.
    for idx, sign in enumerate((-1.0, 1.0)):
        _cylinder_y(
            front,
            f"brace_pivot_pin_{idx}",
            0.007,
            0.150,
            (-0.115, sign * 0.232, 0.380),
            dark_aluminum,
        )

    # Paired flat spread-limit braces: one on each side, with black slot marks
    # showing the stow-friendly slotted end around the rear latch pins.
    for idx, brace in enumerate((brace_0, brace_1)):
        brace.visual(
            Box((0.242, 0.012, 0.012)),
            origin=Origin(xyz=(0.134, 0.0, 0.0)),
            material=dark_aluminum,
            name="bar",
        )
        brace.visual(
            Box((0.230, 0.014, 0.004)),
            origin=Origin(xyz=(0.155, 0.0, 0.011)),
            material=black_rubber,
            name="slot_mark",
        )
        brace.visual(
            Cylinder(radius=0.014, length=0.006),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=dark_aluminum,
            name="front_eye",
        )
        brace.visual(
            Box((0.070, 0.018, 0.018)),
            origin=Origin(xyz=(0.270, 0.0, 0.0)),
            material=dark_aluminum,
            name="slotted_end",
        )
        brace.visual(
            Box((0.050, 0.014, 0.004)),
            origin=Origin(xyz=(0.270, 0.0, 0.011)),
            material=yellow_label,
            name="lock_label",
        )

    rear_hinge = model.articulation(
        "front_to_rear",
        ArticulationType.REVOLUTE,
        parent=front,
        child=rear,
        origin=Origin(xyz=(0.0, 0.0, HINGE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=1.0, lower=0.0, upper=REAR_HINGE_UPPER),
    )

    for idx, (brace, sign) in enumerate(((brace_0, -1.0), (brace_1, 1.0))):
        model.articulation(
            f"front_to_brace_{idx}",
            ArticulationType.REVOLUTE,
            parent=front,
            child=brace,
            origin=Origin(xyz=(-0.115, sign * 0.300, 0.380)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=8.0, velocity=1.5, lower=0.0, upper=0.82),
            mimic=Mimic(joint=rear_hinge.name, multiplier=1.25, offset=0.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    front = object_model.get_part("front_frame")
    rear = object_model.get_part("rear_frame")
    brace_0 = object_model.get_part("brace_0")
    brace_1 = object_model.get_part("brace_1")
    rear_hinge = object_model.get_articulation("front_to_rear")

    ctx.allow_overlap(
        front,
        rear,
        elem_a="hinge_pin",
        elem_b="rear_barrel",
        reason="The visible steel hinge pin is intentionally captured through the rear hinge knuckle.",
    )
    ctx.expect_overlap(
        front,
        rear,
        axes="xyz",
        elem_a="hinge_pin",
        elem_b="rear_barrel",
        min_overlap=0.006,
        name="top hinge pin passes through rear barrel",
    )

    for idx, brace in enumerate((brace_0, brace_1)):
        ctx.allow_overlap(
            front,
            brace,
            elem_a=f"brace_pivot_pin_{idx}",
            elem_b="front_eye",
            reason="The spread brace eye is intentionally captured on the front pivot pin.",
        )
        ctx.expect_overlap(
            front,
            brace,
            axes="xyz",
            elem_a=f"brace_pivot_pin_{idx}",
            elem_b="front_eye",
            min_overlap=0.004,
            name=f"brace {idx} front pivot is captured",
        )
        ctx.allow_overlap(
            rear,
            brace,
            elem_a=f"latch_pin_{idx}",
            elem_b="slotted_end",
            reason="The open-state spread brace is intentionally seated over the rear stop pin.",
        )
        ctx.expect_overlap(
            rear,
            brace,
            axes="xyz",
            elem_a=f"latch_pin_{idx}",
            elem_b="slotted_end",
            min_overlap=0.004,
            name=f"brace {idx} rear latch is seated",
        )
        ctx.allow_overlap(
            rear,
            brace,
            elem_a=f"latch_pin_{idx}",
            elem_b="bar",
            reason="During folding the rear stop pin intentionally rides inside the long slotted spread-brace bar proxy.",
        )

    with ctx.pose({rear_hinge: 0.0}):
        open_box = ctx.part_world_aabb(rear)
        front_box = ctx.part_world_aabb(front)
        ctx.check(
            "open A-frame footprint is stable",
            open_box is not None
            and front_box is not None
            and (open_box[1][0] - front_box[0][0]) > 0.52,
            details=f"front={front_box}, rear={open_box}",
        )

    with ctx.pose({rear_hinge: REAR_HINGE_UPPER}):
        ctx.expect_overlap(
            front,
            rear,
            axes="x",
            min_overlap=0.20,
            name="folded rear frame nests beside front frame",
        )
        for idx, brace in enumerate((brace_0, brace_1)):
            ctx.expect_overlap(
                rear,
                brace,
                axes="xyz",
                elem_a=f"latch_pin_{idx}",
                elem_b="bar",
                min_overlap=0.004,
                name=f"brace {idx} stop pin retained while folded",
            )

    return ctx.report()


object_model = build_object_model()
