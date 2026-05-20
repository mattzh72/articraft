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
    Sphere,
    TestContext,
    TestReport,
)


def _cylinder_between(part, name, start, end, radius, material):
    """Add a cylinder whose local Z axis spans two points in the part frame."""
    sx, sy, sz = start
    ex, ey, ez = end
    dx, dy, dz = ex - sx, ey - sy, ez - sz
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if length <= 0.0:
        raise ValueError(f"zero-length cylinder {name}")
    ux, uy, uz = dx / length, dy / length, dz / length
    yaw = math.atan2(uy, ux)
    pitch = math.atan2(math.sqrt(ux * ux + uy * uy), uz)
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
    model = ArticulatedObject(name="wide_bench_playground_swing")

    galvanized = model.material("galvanized_steel", rgba=(0.62, 0.64, 0.62, 1.0))
    dark_steel = model.material("dark_pivot_steel", rgba=(0.08, 0.09, 0.10, 1.0))
    blue = model.material("blue_powder_coat", rgba=(0.06, 0.22, 0.78, 1.0))
    wood = model.material("warm_seat_wood", rgba=(0.72, 0.43, 0.20, 1.0))
    dark_gap = model.material("dark_recess", rgba=(0.02, 0.018, 0.014, 1.0))
    black = model.material("black_rubber", rgba=(0.015, 0.015, 0.012, 1.0))

    # Root playground support: a broad A-frame with a high crossbeam and visible
    # clevis brackets below the beam for the upper hanger pivots.
    frame = model.part("frame")
    frame.visual(
        Cylinder(radius=0.065, length=2.95),
        origin=Origin(xyz=(0.0, 0.0, 2.35), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=galvanized,
        name="top_crossbeam",
    )
    for side, x in enumerate((-1.38, 1.38)):
        _cylinder_between(frame, f"leg_{side}_0", (x, -0.72, 0.04), (x, -0.035, 2.34), 0.042, galvanized)
        _cylinder_between(frame, f"leg_{side}_1", (x, 0.72, 0.04), (x, 0.035, 2.34), 0.042, galvanized)
        frame.visual(
            Cylinder(radius=0.030, length=1.55),
            origin=Origin(xyz=(x, 0.0, 0.045), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_steel,
            name=f"ground_foot_{side}",
        )
        frame.visual(
            Sphere(radius=0.075),
            origin=Origin(xyz=(x, 0.0, 2.34)),
            material=galvanized,
            name=f"top_node_{side}",
        )

    for side, x in enumerate((-1.06, 1.06)):
        for cheek, y in enumerate((-0.068, 0.068)):
            frame.visual(
                Box((0.13, 0.030, 0.26)),
                origin=Origin(xyz=(x, y, 2.225)),
                material=dark_steel,
                name=f"upper_clevis_{side}_{cheek}",
            )
        frame.visual(
            Cylinder(radius=0.034, length=0.22),
            origin=Origin(xyz=(x, 0.0, 2.13), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark_steel,
            name=f"upper_pin_{side}",
        )
        for support, sx in enumerate((x - 0.125, x + 0.125)):
            frame.visual(
                Box((0.040, 0.090, 0.250)),
                origin=Origin(xyz=(sx, 0.0, 2.225)),
                material=dark_steel,
                name=f"upper_pin_support_{side}_{support}",
            )

    # The two side suspension arms are a single rigid hanger assembly.  It
    # pivots at the upper pins and carries the lower bench pivot axle.
    arms = model.part("hanger_arms")
    for side, x in enumerate((-1.06, 1.06)):
        arms.visual(
            Cylinder(radius=0.056, length=0.20),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=blue,
            name=f"upper_barrel_{side}",
        )
        _cylinder_between(arms, f"side_arm_{side}", (x, 0.0, -0.050), (x, 0.0, -1.170), 0.032, blue)
        arms.visual(
            Box((0.032, 0.060, 0.120)),
            origin=Origin(xyz=(x, 0.0, -1.225)),
            material=blue,
            name=f"lower_fork_{side}",
        )
        arms.visual(
            Sphere(radius=0.044),
            origin=Origin(xyz=(x, 0.0, -1.25)),
            material=blue,
            name=f"lower_eye_{side}",
        )
    arms.visual(
        Cylinder(radius=0.022, length=2.12),
        origin=Origin(xyz=(0.0, 0.0, -0.16), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=blue,
        name="upper_spreader",
    )
    arms.visual(
        Cylinder(radius=0.026, length=2.22),
        origin=Origin(xyz=(0.0, 0.0, -1.25), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="lower_axle",
    )

    # The bench is wide and visibly rideable, with a slatted seat/back and
    # side bosses that rotate on the lower axle carried by the hanger arms.
    bench = model.part("bench")
    bench.visual(
        Box((1.76, 0.56, 0.075)),
        origin=Origin(xyz=(0.0, 0.16, -0.305)),
        material=wood,
        name="seat_pan",
    )
    for i, y in enumerate((-0.01, 0.14, 0.29)):
        bench.visual(
            Box((1.78, 0.014, 0.012)),
            origin=Origin(xyz=(0.0, y, -0.267)),
            material=dark_gap,
            name=f"seat_groove_{i}",
        )
    bench.visual(
        Box((1.76, 0.070, 0.55)),
        origin=Origin(xyz=(0.0, -0.165, -0.020), rpy=(0.18, 0.0, 0.0)),
        material=wood,
        name="back_panel",
    )
    for i, z in enumerate((-0.11, 0.07, 0.24)):
        bench.visual(
            Box((1.78, 0.040, 0.028)),
            origin=Origin(xyz=(0.0, -0.185, z), rpy=(0.18, 0.0, 0.0)),
            material=dark_gap,
            name=f"back_groove_{i}",
        )
    bench.visual(
        Box((1.82, 0.060, 0.070)),
        origin=Origin(xyz=(0.0, -0.090, -0.285)),
        material=dark_steel,
        name="rear_seat_rail",
    )
    bench.visual(
        Box((1.82, 0.050, 0.065)),
        origin=Origin(xyz=(0.0, 0.410, -0.285)),
        material=dark_steel,
        name="front_seat_rail",
    )
    for side, x in enumerate((-0.93, 0.93)):
        bench.visual(
            Cylinder(radius=0.060, length=0.22),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark_steel,
            name=f"pivot_boss_{side}",
        )
        bench.visual(
            Box((0.060, 0.034, 0.34)),
            origin=Origin(xyz=(x, 0.064, -0.170)),
            material=dark_steel,
            name=f"side_hanger_plate_{side}",
        )
        bench.visual(
            Box((0.080, 0.56, 0.075)),
            origin=Origin(xyz=(x, 0.16, -0.305)),
            material=dark_steel,
            name=f"seat_end_cap_{side}",
        )
    for side, x in enumerate((-0.90, 0.90)):
        bench.visual(
            Cylinder(radius=0.040, length=0.075),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=black,
            name=f"pivot_bushing_{side}",
        )

    model.articulation(
        "frame_to_arms",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=arms,
        origin=Origin(xyz=(0.0, 0.0, 2.13)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.6, lower=-0.55, upper=0.55),
    )
    model.articulation(
        "arms_to_bench",
        ArticulationType.REVOLUTE,
        parent=arms,
        child=bench,
        origin=Origin(xyz=(0.0, 0.0, -1.25)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.6, lower=-0.45, upper=0.45),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    arms = object_model.get_part("hanger_arms")
    bench = object_model.get_part("bench")
    upper = object_model.get_articulation("frame_to_arms")
    lower = object_model.get_articulation("arms_to_bench")

    # Coaxial hinge pins are intentionally nested inside their barrels/bushings.
    for side in (0, 1):
        ctx.allow_overlap(
            frame,
            arms,
            elem_a=f"upper_pin_{side}",
            elem_b=f"upper_barrel_{side}",
            reason="The upper hanger barrel rotates around the crossbeam clevis pin.",
        )
        ctx.expect_overlap(
            frame,
            arms,
            axes="x",
            elem_a=f"upper_pin_{side}",
            elem_b=f"upper_barrel_{side}",
            min_overlap=0.10,
            name=f"upper pivot {side} is captured on its pin",
        )
        ctx.allow_overlap(
            arms,
            bench,
            elem_a="lower_axle",
            elem_b=f"pivot_boss_{side}",
            reason="The bench side boss rotates around the lower hanger axle.",
        )
        ctx.allow_overlap(
            arms,
            bench,
            elem_a="lower_axle",
            elem_b=f"pivot_bushing_{side}",
            reason="The dark bushing is drawn around the lower axle as a captured bearing surface.",
        )
        ctx.allow_overlap(
            arms,
            bench,
            elem_a=f"lower_eye_{side}",
            elem_b=f"pivot_boss_{side}",
            reason="The hanger eye closely surrounds the bench boss at the lower pivot.",
        )
        ctx.expect_overlap(
            arms,
            bench,
            axes="x",
            elem_a="lower_axle",
            elem_b=f"pivot_boss_{side}",
            min_overlap=0.12,
            name=f"lower pivot {side} is captured on the axle",
        )
        ctx.expect_overlap(
            arms,
            bench,
            axes="x",
            elem_a=f"lower_eye_{side}",
            elem_b=f"pivot_boss_{side}",
            min_overlap=0.015,
            name=f"lower hanger eye {side} wraps the pivot boss",
        )

    ctx.expect_gap(
        frame,
        bench,
        axis="z",
        min_gap=0.80,
        positive_elem="top_crossbeam",
        name="bench hangs well below the overhead crossbeam",
    )
    ctx.expect_overlap(
        bench,
        frame,
        axes="x",
        min_overlap=1.55,
        elem_a="seat_pan",
        elem_b="top_crossbeam",
        name="wide bench sits under the crossbeam span",
    )

    rest_pos = ctx.part_world_position(bench)
    with ctx.pose({upper: 0.35, lower: -0.20}):
        swung_pos = ctx.part_world_position(bench)
        ctx.expect_gap(
            frame,
            bench,
            axis="z",
            min_gap=0.65,
            positive_elem="top_crossbeam",
            name="swung bench remains suspended below the frame",
        )
    ctx.check(
        "upper pivot swings the bench fore-aft",
        rest_pos is not None and swung_pos is not None and abs(swung_pos[1] - rest_pos[1]) > 0.25,
        details=f"rest={rest_pos}, swung={swung_pos}",
    )

    back_rest = ctx.part_world_aabb(bench)
    with ctx.pose({lower: 0.30}):
        back_tilted = ctx.part_world_aabb(bench)
    ctx.check(
        "lower pivot changes bench attitude",
        back_rest is not None
        and back_tilted is not None
        and abs(back_tilted[0][1] - back_rest[0][1]) > 0.05,
        details=f"rest_aabb={back_rest}, tilted_aabb={back_tilted}",
    )

    return ctx.report()


object_model = build_object_model()
