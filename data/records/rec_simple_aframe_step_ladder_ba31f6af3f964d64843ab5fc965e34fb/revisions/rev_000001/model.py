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


def _bar_between(
    part,
    name: str,
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    *,
    thickness: tuple[float, float],
    material: Material,
) -> None:
    """Add a rectangular tube whose local +Z axis runs from start to end."""

    sx, sy, sz = start
    ex, ey, ez = end
    dx, dy, dz = ex - sx, ey - sy, ez - sz
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    # The ladder rails and braces are all in X/Z or Y/Z planes.  Use a yaw/roll
    # free orientation by aligning local +Z to the X/Z projection.
    yaw = math.atan2(dy, dx) if abs(dy) > 1e-9 else 0.0
    pitch = math.atan2(dx, dz)
    part.visual(
        Box((thickness[0], thickness[1], length)),
        origin=Origin(
            xyz=((sx + ex) * 0.5, (sy + ey) * 0.5, (sz + ez) * 0.5),
            rpy=(0.0, pitch, yaw),
        ),
        material=material,
        name=name,
    )


def _cyl_y(part, name: str, xyz: tuple[float, float, float], *, radius: float, length: float, material: Material) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="retrofit_aframe_step_ladder")

    galvanized = model.material("aged_galvanized", rgba=(0.62, 0.64, 0.60, 1.0))
    dark_rail = model.material("dark_oxide_rail", rgba=(0.18, 0.20, 0.20, 1.0))
    tread_grip = model.material("worn_black_grip", rgba=(0.03, 0.035, 0.03, 1.0))
    brass = model.material("old_brass_bolts", rgba=(0.78, 0.58, 0.28, 1.0))
    rubber = model.material("aged_rubber", rgba=(0.015, 0.015, 0.012, 1.0))
    hatch_blue = model.material("service_blue", rgba=(0.12, 0.28, 0.42, 1.0))

    front = model.part("front_frame")
    top_z = 1.25
    width = 0.58
    side_y = width * 0.5
    front_foot_x = 0.32

    # Paired climbing rails, old-school rectangular tube construction.
    for i, y in enumerate((-side_y, side_y)):
        _bar_between(
            front,
            f"front_rail_{i}",
            (front_foot_x, y, 0.07),
            (0.02, y, top_z - 0.03),
            thickness=(0.045, 0.038),
            material=dark_rail,
        )
        front.visual(
            Box((0.16, 0.075, 0.070)),
            origin=Origin(xyz=(front_foot_x + 0.015, y, 0.035)),
            material=rubber,
            name=f"front_foot_{i}",
        )

    # Three grippy treads tie the two rails together and make the frame a single
    # structural assembly rather than a set of floating side members.
    for i, (z, depth) in enumerate(((0.35, 0.18), (0.65, 0.17), (0.94, 0.16))):
        frac = (z - 0.07) / (top_z - 0.10)
        x = front_foot_x * (1.0 - frac) + 0.02 * frac
        front.visual(
            Box((depth, width + 0.10, 0.045)),
            origin=Origin(xyz=(x + 0.015, 0.0, z)),
            material=galvanized,
            name=f"tread_{i}",
        )
        front.visual(
            Box((depth * 0.88, width + 0.04, 0.009)),
            origin=Origin(xyz=(x + 0.012, 0.0, z + 0.027)),
            material=tread_grip,
            name=f"tread_grip_{i}",
        )

    # Top tray/platform with flanged vintage sheet-metal edges and a visible
    # service-hatch socket.
    front.visual(
        Box((0.35, width + 0.12, 0.050)),
        origin=Origin(xyz=(0.075, 0.0, top_z + 0.060)),
        material=galvanized,
        name="top_tray",
    )
    front.visual(
        Box((0.036, width + 0.16, 0.11)),
        origin=Origin(xyz=(-0.115, 0.0, top_z + 0.030)),
        material=dark_rail,
        name="rear_tray_lip",
    )
    front.visual(
        Box((0.040, width + 0.16, 0.085)),
        origin=Origin(xyz=(0.252, 0.0, top_z + 0.025)),
        material=dark_rail,
        name="front_tray_lip",
    )
    front.visual(
        Box((0.29, 0.040, 0.075)),
        origin=Origin(xyz=(0.07, -0.375, top_z + 0.035)),
        material=dark_rail,
        name="tray_flange_0",
    )
    front.visual(
        Box((0.29, 0.040, 0.075)),
        origin=Origin(xyz=(0.07, 0.375, top_z + 0.035)),
        material=dark_rail,
        name="tray_flange_1",
    )

    # Pivot adapters and exposed split hinge barrels: modern bolted service
    # plates wrapped around an old ladder silhouette.
    for i, y in enumerate((-0.345, 0.345)):
        front.visual(
            Box((0.095, 0.038, 0.18)),
            origin=Origin(xyz=(-0.010, y, top_z - 0.045)),
            material=galvanized,
            name=f"pivot_adapter_{i}",
        )
        _cyl_y(front, f"front_hinge_knuckle_{i}", (0.0, y, top_z), radius=0.035, length=0.105, material=galvanized)
        for bx, bz in ((-0.030, top_z - 0.095), (0.025, top_z + 0.015)):
            _cyl_y(front, f"pivot_bolt_{i}_{0 if bx < 0 else 1}", (bx, y + (0.020 if y > 0 else -0.020), bz), radius=0.012, length=0.050, material=brass)

    # Receivers for the folding spread braces, attached to the side rails by
    # triangular-looking stacked plates.
    for i, y in enumerate((-0.345, 0.345)):
        x_anchor = 0.176
        z_anchor = 0.60
        sign = 1.0 if y > 0 else -1.0
        front.visual(
            Box((0.090, 0.060, 0.090)),
            origin=Origin(xyz=(x_anchor, y - sign * 0.070, z_anchor)),
            material=galvanized,
            name=f"brace_adapter_{i}",
        )
        front.visual(
            Box((0.028, 0.022, 0.080)),
            origin=Origin(xyz=(x_anchor, y, z_anchor)),
            material=dark_rail,
            name=f"brace_clevis_{i}",
        )
        _cyl_y(front, f"brace_pivot_bolt_{i}", (x_anchor, y, z_anchor), radius=0.018, length=0.070, material=brass)

    # A small hinged service hatch in the top tray.  The root has two short
    # hinge leaves; the hatch part carries the center leaf.
    front.visual(
        Box((0.030, 0.060, 0.010)),
        origin=Origin(xyz=(-0.078, -0.145, top_z + 0.090)),
        material=galvanized,
        name="hatch_leaf_0",
    )
    front.visual(
        Box((0.030, 0.060, 0.010)),
        origin=Origin(xyz=(-0.078, 0.145, top_z + 0.090)),
        material=galvanized,
        name="hatch_leaf_1",
    )

    rear = model.part("rear_frame")
    rear_foot = (-0.42, 0.0, -(top_z - 0.08))
    for i, y in enumerate((-side_y, side_y)):
        _bar_between(
            rear,
            f"rear_leg_{i}",
            (-0.045, y, -0.035),
            (rear_foot[0], y, rear_foot[2]),
            thickness=(0.042, 0.038),
            material=dark_rail,
        )
        rear.visual(
            Box((0.16, 0.075, 0.070)),
            origin=Origin(xyz=(rear_foot[0] - 0.010, y, rear_foot[2] - 0.035)),
            material=rubber,
            name=f"rear_foot_{i}",
        )

    # Rear cross braces keep the rear frame connected and visibly robust.
    for i, z_rel in enumerate((-0.43, -0.76, -1.02)):
        t = (-z_rel - 0.035) / ((top_z - 0.08) - 0.035)
        x_rel = -0.045 * (1.0 - t) + rear_foot[0] * t
        rear.visual(
            Box((0.055, width + 0.09, 0.040)),
            origin=Origin(xyz=(x_rel, 0.0, z_rel)),
            material=galvanized,
            name=f"rear_crossbar_{i}",
        )
    rear.visual(
        Box((0.055, width + 0.02, 0.050)),
        origin=Origin(xyz=(-0.060, 0.0, -0.160)),
        material=galvanized,
        name="rear_top_crossbar",
    )
    rear.visual(
        Box((0.065, 0.545, 0.045)),
        origin=Origin(xyz=(-0.025, 0.0, -0.015)),
        material=galvanized,
        name="rear_hinge_saddle",
    )
    _cyl_y(rear, "rear_hinge_barrel", (0.0, 0.0, 0.0), radius=0.030, length=0.430, material=galvanized)

    # Rear catch sockets for the open spread braces.  They are mounted to the
    # rear legs, not left hanging at the brace tips.
    rear.visual(
        Box((0.075, 0.030, 0.080)),
        origin=Origin(xyz=(-0.234, -0.319, -0.650)),
        material=galvanized,
        name="brace_catch_0",
    )
    rear.visual(
        Box((0.040, 0.028, 0.070)),
        origin=Origin(xyz=(-0.263, -0.345, -0.650)),
        material=brass,
        name="catch_pin_0",
    )
    rear.visual(
        Box((0.075, 0.030, 0.080)),
        origin=Origin(xyz=(-0.234, 0.319, -0.650)),
        material=galvanized,
        name="brace_catch_1",
    )
    rear.visual(
        Box((0.040, 0.028, 0.070)),
        origin=Origin(xyz=(-0.263, 0.345, -0.650)),
        material=brass,
        name="catch_pin_1",
    )

    leg_pivot = model.articulation(
        "leg_pivot",
        ArticulationType.REVOLUTE,
        parent=front,
        child=rear,
        origin=Origin(xyz=(0.0, 0.0, top_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.2, lower=-0.60, upper=0.0),
    )

    # Paired spread-limiting braces.  At the open limit they reach the rear
    # catch sockets; when the leg pivot closes they fold upward via mimic.
    for i, y in enumerate((-0.345, 0.345)):
        brace = model.part(f"spread_brace_{i}")
        brace.visual(
            Box((0.383, 0.024, 0.024)),
            origin=Origin(xyz=(-0.2135, 0.0, 0.0)),
            material=galvanized,
            name="brace_link",
        )
        brace.visual(
            Cylinder(radius=0.024, length=0.022),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=brass,
            name="front_eye",
        )
        brace.visual(
            Box((0.070, 0.034, 0.034)),
            origin=Origin(xyz=(-0.405, 0.0, 0.0)),
            material=dark_rail,
            name="rear_latch_tongue",
        )
        model.articulation(
            f"brace_fold_{i}",
            ArticulationType.REVOLUTE,
            parent=front,
            child=brace,
            origin=Origin(xyz=(0.176, y, 0.60)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=35.0, velocity=1.5, lower=0.0, upper=1.25),
            mimic=Mimic(joint=leg_pivot.name, multiplier=-2.0, offset=0.0),
        )

    hatch = model.part("service_hatch")
    hatch.visual(
        Box((0.165, 0.230, 0.012)),
        origin=Origin(xyz=(0.0825, 0.0, 0.008)),
        material=hatch_blue,
        name="hatch_panel",
    )
    hatch.visual(
        Box((0.145, 0.006, 0.008)),
        origin=Origin(xyz=(0.090, -0.070, 0.017)),
        material=brass,
        name="hatch_pull_0",
    )
    hatch.visual(
        Box((0.145, 0.006, 0.008)),
        origin=Origin(xyz=(0.090, 0.070, 0.017)),
        material=brass,
        name="hatch_pull_1",
    )
    _cyl_y(hatch, "hatch_barrel", (0.0, 0.0, 0.008), radius=0.010, length=0.095, material=galvanized)
    model.articulation(
        "hatch_hinge",
        ArticulationType.REVOLUTE,
        parent=front,
        child=hatch,
        origin=Origin(xyz=(-0.080, 0.0, top_z + 0.085)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=2.0, lower=0.0, upper=1.10),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    front = object_model.get_part("front_frame")
    rear = object_model.get_part("rear_frame")
    hatch = object_model.get_part("service_hatch")
    brace_0 = object_model.get_part("spread_brace_0")
    brace_1 = object_model.get_part("spread_brace_1")
    pivot = object_model.get_articulation("leg_pivot")
    hatch_hinge = object_model.get_articulation("hatch_hinge")

    for idx, brace in enumerate((brace_0, brace_1)):
        ctx.allow_overlap(
            front,
            brace,
            elem_a=f"brace_clevis_{idx}",
            elem_b="front_eye",
            reason="The spread-brace eye is intentionally captured between the bolted clevis cheeks.",
        )
        ctx.allow_overlap(
            front,
            brace,
            elem_a=f"brace_pivot_bolt_{idx}",
            elem_b="front_eye",
            reason="The brass pivot bolt is intentionally modeled through the spread-brace eye.",
        )
        ctx.expect_overlap(
            front,
            brace,
            axes="yz",
            min_overlap=0.018,
            elem_a=f"brace_clevis_{idx}",
            elem_b="front_eye",
            name=f"spread brace {idx} eye remains captured in front clevis",
        )
        ctx.expect_overlap(
            front,
            brace,
            axes="yz",
            min_overlap=0.018,
            elem_a=f"brace_pivot_bolt_{idx}",
            elem_b="front_eye",
            name=f"spread brace {idx} pivot bolt passes through eye",
        )
        ctx.allow_overlap(
            rear,
            brace,
            elem_a=f"catch_pin_{idx}",
            elem_b="rear_latch_tongue",
            reason="The latch tongue is represented seated over the open-state catch pin.",
        )
        ctx.allow_overlap(
            rear,
            brace,
            elem_a=f"brace_catch_{idx}",
            elem_b="rear_latch_tongue",
            reason="The latch tongue sits slightly inside the rear catch socket at the stable open stop.",
        )
        ctx.expect_overlap(
            brace,
            rear,
            axes="xz",
            min_overlap=0.020,
            elem_a="rear_latch_tongue",
            elem_b=f"catch_pin_{idx}",
            name=f"spread brace {idx} latch seats over catch pin",
        )
        ctx.expect_overlap(
            brace,
            rear,
            axes="xz",
            min_overlap=0.020,
            elem_a="rear_latch_tongue",
            elem_b=f"brace_catch_{idx}",
            name=f"spread brace {idx} tongue is guided by catch socket",
        )

    # In the default open state, the A-frame has a real fore-aft stance and the
    # spread brace reaches the rear catch region.
    open_front = ctx.part_world_aabb(front)
    open_rear = ctx.part_world_aabb(rear)
    ctx.check(
        "open stance spreads rear support behind climbing frame",
        open_front is not None and open_rear is not None and open_rear[0][0] < open_front[0][0] - 0.20,
        details=f"front={open_front}, rear={open_rear}",
    )
    ctx.expect_overlap(
        brace_0,
        rear,
        axes="xz",
        min_overlap=0.020,
        elem_a="rear_latch_tongue",
        elem_b="catch_pin_0",
        name="open spread brace reaches catch socket",
    )

    closed_rear = None
    folded_brace = None
    open_brace = ctx.part_world_aabb(brace_0)
    with ctx.pose({pivot: -0.60}):
        closed_rear = ctx.part_world_aabb(rear)
        folded_brace = ctx.part_world_aabb(brace_0)
    ctx.check(
        "leg pivot closes rear frame toward front feet",
        open_rear is not None
        and closed_rear is not None
        and closed_rear[1][0] > open_rear[1][0] + 0.22,
        details=f"open={open_rear}, closed={closed_rear}",
    )
    ctx.check(
        "mimic braces fold upward as ladder closes",
        open_brace is not None
        and folded_brace is not None
        and folded_brace[1][2] > open_brace[1][2] + 0.12,
        details=f"open={open_brace}, folded={folded_brace}",
    )

    closed_hatch = ctx.part_world_aabb(hatch)
    with ctx.pose({hatch_hinge: 1.0}):
        open_hatch = ctx.part_world_aabb(hatch)
    ctx.check(
        "service hatch hinges upward from top tray",
        closed_hatch is not None
        and open_hatch is not None
        and open_hatch[1][2] > closed_hatch[1][2] + 0.08,
        details=f"closed={closed_hatch}, open={open_hatch}",
    )

    return ctx.report()


object_model = build_object_model()
