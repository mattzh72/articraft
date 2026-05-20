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
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
)


LINK_LENGTH = 0.92
UPPER_Z = 1.72
PIVOT_XS = (-0.68, 0.68)
PIVOT_YS = (-0.42, 0.42)
PRIMARY_PIVOT = (PIVOT_XS[0], PIVOT_YS[0], UPPER_Z - LINK_LENGTH)


def _bench_local(world_xyz: tuple[float, float, float]) -> tuple[float, float, float]:
    return (
        world_xyz[0] - PRIMARY_PIVOT[0],
        world_xyz[1] - PRIMARY_PIVOT[1],
        world_xyz[2] - PRIMARY_PIVOT[2],
    )


def _cylinder_between(
    part,
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    *,
    radius: float,
    material: Material,
    name: str,
) -> None:
    """Add a cylinder whose local +Z axis follows a segment in the YZ plane."""
    sx, sy, sz = start
    ex, ey, ez = end
    dx, dy, dz = ex - sx, ey - sy, ez - sz
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if abs(dx) > 1e-9:
        raise ValueError("_cylinder_between is intentionally only used in YZ planes")
    # Rotating local +Z around +X gives world direction (0, -sin(r), cos(r)).
    roll = math.atan2(-dy, dz)
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(
            xyz=((sx + ex) / 2.0, (sy + ey) / 2.0, (sz + ez) / 2.0),
            rpy=(roll, 0.0, 0.0),
        ),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="face_to_face_playground_glider")

    powder_red = model.material("powder_coated_red", rgba=(0.84, 0.07, 0.05, 1.0))
    yellow = model.material("safety_yellow_links", rgba=(0.96, 0.73, 0.08, 1.0))
    teal = model.material("teal_seat_plastic", rgba=(0.02, 0.43, 0.48, 1.0))
    dark = model.material("dark_rubber_tread", rgba=(0.03, 0.035, 0.035, 1.0))
    grey = model.material("galvanized_pivots", rgba=(0.58, 0.61, 0.62, 1.0))

    frame = model.part("overhead_frame")
    # Overhead beam and A-frame end supports.
    frame.visual(
        Cylinder(radius=0.045, length=2.20),
        origin=Origin(xyz=(0.0, 0.0, 2.10), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=powder_red,
        name="top_beam",
    )
    for x, suffix in ((-0.98, "0"), (0.98, "1")):
        _cylinder_between(
            frame,
            (x, 0.0, 2.08),
            (x, -0.85, 0.08),
            radius=0.035,
            material=powder_red,
            name=f"support_leg_{suffix}_0",
        )
        _cylinder_between(
            frame,
            (x, 0.0, 2.08),
            (x, 0.85, 0.08),
            radius=0.035,
            material=powder_red,
            name=f"support_leg_{suffix}_1",
        )
        frame.visual(
            Cylinder(radius=0.032, length=1.78),
            origin=Origin(xyz=(x, 0.0, 0.08), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=powder_red,
            name=f"ground_bar_{suffix}",
        )

    # Hangers place the four upper pivots below and outboard of the top beam.
    pivot_index = 0
    for x in PIVOT_XS:
        for y in PIVOT_YS:
            frame.visual(
                Cylinder(radius=0.024, length=abs(y) + 0.035),
                origin=Origin(
                    xyz=(x, y / 2.0, 2.035),
                    rpy=(-math.pi / 2.0, 0.0, 0.0),
                ),
                material=powder_red,
                name=f"pivot_spreader_{pivot_index}",
            )
            frame.visual(
                Box((0.052, 0.052, 0.255)),
                origin=Origin(xyz=(x, y, 1.905)),
                material=powder_red,
                name=f"pivot_drop_{pivot_index}",
            )
            # Clevis cheeks straddle, but do not intersect, the link eye.
            for side, dx in (("inner", -0.070), ("outer", 0.070)):
                if pivot_index == 0 and side == "inner":
                    frame.visual(
                        Box((0.020, 0.135, 0.185)),
                        origin=Origin(xyz=(x + dx, y, UPPER_Z)),
                        material=grey,
                        name="upper_clevis_0_inner",
                    )
                else:
                    frame.visual(
                        Box((0.020, 0.135, 0.185)),
                        origin=Origin(xyz=(x + dx, y, UPPER_Z)),
                        material=grey,
                        name=f"upper_clevis_{pivot_index}_{side}",
                    )
            frame.visual(
                Box((0.175, 0.130, 0.030)),
                origin=Origin(xyz=(x, y, UPPER_Z + 0.100)),
                material=grey,
                name=f"upper_yoke_bridge_{pivot_index}",
            )
            if pivot_index == 0:
                frame.visual(
                    Cylinder(radius=0.018, length=0.170),
                    origin=Origin(xyz=(x, y, UPPER_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
                    material=grey,
                    name="upper_pin_0",
                )
            else:
                frame.visual(
                    Cylinder(radius=0.018, length=0.170),
                    origin=Origin(xyz=(x, y, UPPER_Z), rpy=(0.0, math.pi / 2.0, 0.0)),
                    material=grey,
                    name=f"upper_pin_{pivot_index}",
                )
            pivot_index += 1

    def make_link(name: str) -> object:
        link = model.part(name)
        link.visual(
            Cylinder(radius=0.025, length=LINK_LENGTH - 0.10),
            origin=Origin(xyz=(0.0, 0.0, -LINK_LENGTH / 2.0)),
            material=yellow,
            name="strap",
        )
        link.visual(
            Cylinder(radius=0.050, length=0.078),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=grey,
            name="upper_eye",
        )
        link.visual(
            Cylinder(radius=0.050, length=0.078),
            origin=Origin(xyz=(0.0, 0.0, -LINK_LENGTH), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=grey,
            name="lower_eye",
        )
        return link

    drive_link = make_link("drive_link")
    parallel_links = [make_link(f"parallel_link_{i}") for i in range(3)]

    bench = model.part("bench_frame")
    # Moving rectangular bench cradle, centered under the four lower pivots.
    for y, suffix in ((-0.55, "0"), (0.55, "1")):
        bench.visual(
            Cylinder(radius=0.026, length=1.46),
            origin=Origin(
                xyz=_bench_local((0.0, y, 0.72)),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=powder_red,
            name=f"side_rail_{suffix}",
        )
    for x, suffix in ((-0.68, "0"), (0.68, "1")):
        bench.visual(
            Cylinder(radius=0.026, length=1.14),
            origin=Origin(
                xyz=_bench_local((x, 0.0, 0.72)),
                rpy=(-math.pi / 2.0, 0.0, 0.0),
            ),
            material=powder_red,
            name=f"end_rail_{suffix}",
        )
        bench.visual(
            Cylinder(radius=0.022, length=1.02),
            origin=Origin(
                xyz=_bench_local((x * 0.82, 0.0, 0.515)),
                rpy=(-math.pi / 2.0, 0.0, 0.0),
            ),
            material=powder_red,
            name=f"seat_cross_tube_{suffix}",
        )

    # Two seats face the center footwell; backs are on the outside edges.
    for y, suffix in ((-0.33, "0"), (0.33, "1")):
        bench.visual(
            Box((1.25, 0.28, 0.055)),
            origin=Origin(xyz=_bench_local((0.0, y, 0.565))),
            material=teal,
            name=f"seat_pan_{suffix}",
        )
        bench.visual(
            Box((1.08, 0.030, 0.075)),
            origin=Origin(xyz=_bench_local((0.0, y, 0.520))),
            material=powder_red,
            name=f"seat_underrail_{suffix}",
        )
        bench.visual(
            Cylinder(radius=0.018, length=1.22),
            origin=Origin(
                xyz=_bench_local((0.0, y, 0.505)),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=powder_red,
            name=f"seat_front_tube_{suffix}",
        )
        for x, post_suffix in ((-0.68, "0"), (0.68, "1")):
            bench.visual(
                Box((0.070, 0.070, 0.170)),
                origin=Origin(xyz=_bench_local((x, y, 0.645))),
                material=powder_red,
                name=f"seat_post_{suffix}_{post_suffix}",
            )

    for y, suffix in ((-0.68, "0"), (0.68, "1")):
        bench.visual(
            Box((1.25, 0.055, 0.36)),
            origin=Origin(xyz=_bench_local((0.0, y, 0.84))),
            material=teal,
            name=f"back_panel_{suffix}",
        )
        bench.visual(
            Box((1.12, 0.145, 0.050)),
            origin=Origin(xyz=_bench_local((0.0, y * 0.92, 0.700))),
            material=powder_red,
            name=f"back_base_rail_{suffix}",
        )
        bench.visual(
            Cylinder(radius=0.020, length=1.25),
            origin=Origin(
                xyz=_bench_local((0.0, y, 1.02)),
                rpy=(0.0, math.pi / 2.0, 0.0),
            ),
            material=powder_red,
            name=f"back_top_tube_{suffix}",
        )

    bench.visual(
        Box((1.04, 0.24, 0.035)),
        origin=Origin(xyz=_bench_local((0.0, 0.0, 0.405))),
        material=dark,
        name="center_footwell",
    )
    bench.visual(
        Cylinder(radius=0.020, length=1.12),
        origin=Origin(
            xyz=_bench_local((0.0, 0.0, 0.455)),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=powder_red,
        name="footwell_tube",
    )
    bench.visual(
        Box((1.00, 0.060, 0.045)),
        origin=Origin(xyz=_bench_local((0.0, 0.0, 0.430))),
        material=powder_red,
        name="footwell_mount",
    )
    for x, suffix in ((-0.58, "0"), (0.58, "1")):
        bench.visual(
            Box((0.24, 0.060, 0.320)),
            origin=Origin(xyz=_bench_local((x, 0.0, 0.570))),
            material=powder_red,
            name=f"footwell_bracket_{suffix}",
        )

    # Lower clevis brackets are part of the bench frame.  They visibly clip the
    # four link eyes while leaving a small mechanical running clearance.
    pivot_worlds: list[tuple[float, float, float]] = []
    for x in PIVOT_XS:
        for y in PIVOT_YS:
            pivot_worlds.append((x, y, UPPER_Z - LINK_LENGTH))
    for i, (x, y, z) in enumerate(pivot_worlds):
        for side, dx in (("inner", -0.070), ("outer", 0.070)):
            if i == 0 and side == "inner":
                bench.visual(
                    Box((0.020, 0.130, 0.170)),
                    origin=Origin(xyz=_bench_local((x + dx, y, z))),
                    material=grey,
                    name="lower_clip_0_inner",
                )
            else:
                bench.visual(
                    Box((0.020, 0.130, 0.170)),
                    origin=Origin(xyz=_bench_local((x + dx, y, z))),
                    material=grey,
                    name=f"lower_clip_{i}_{side}",
                )
        bench.visual(
            Box((0.055, 0.045, 0.120)),
            origin=Origin(xyz=_bench_local((x, y + (0.070 if y < 0 else -0.070), z - 0.060))),
            material=powder_red,
            name=f"clip_web_{i}",
        )
        bench.visual(
            Box((0.170, 0.150, 0.035)),
            origin=Origin(xyz=_bench_local((x, y, z - 0.095))),
            material=grey,
            name=f"lower_clip_bridge_{i}",
        )
        if i == 0:
            bench.visual(
                Cylinder(radius=0.018, length=0.170),
                origin=Origin(xyz=_bench_local((x, y, z)), rpy=(0.0, math.pi / 2.0, 0.0)),
                material=grey,
                name="lower_pin_0",
            )
        else:
            bench.visual(
                Cylinder(radius=0.018, length=0.170),
                origin=Origin(xyz=_bench_local((x, y, z)), rpy=(0.0, math.pi / 2.0, 0.0)),
                material=grey,
                name=f"lower_pin_{i}",
            )

    swing_limits = MotionLimits(effort=140.0, velocity=1.1, lower=-0.42, upper=0.42)
    swing_damping = MotionProperties(damping=1.5, friction=0.05)
    model.articulation(
        "beam_to_drive_link",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=drive_link,
        origin=Origin(xyz=(PIVOT_XS[0], PIVOT_YS[0], UPPER_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=swing_limits,
        motion_properties=swing_damping,
    )

    # The bench hangs from the primary lower pivot.  The mimic relation keeps
    # the seat carriage level while the upper links swing in a parallelogram.
    model.articulation(
        "drive_link_to_bench",
        ArticulationType.REVOLUTE,
        parent=drive_link,
        child=bench,
        origin=Origin(xyz=(0.0, 0.0, -LINK_LENGTH)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=swing_limits,
        motion_properties=swing_damping,
        mimic=Mimic(joint="beam_to_drive_link", multiplier=-1.0),
    )

    other_pivots = [
        (PIVOT_XS[0], PIVOT_YS[1], UPPER_Z),
        (PIVOT_XS[1], PIVOT_YS[0], UPPER_Z),
        (PIVOT_XS[1], PIVOT_YS[1], UPPER_Z),
    ]
    for i, (x, y, z) in enumerate(other_pivots):
        model.articulation(
            f"beam_to_parallel_link_{i}",
            ArticulationType.REVOLUTE,
            parent=frame,
            child=parallel_links[i],
            origin=Origin(xyz=(x, y, z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=swing_limits,
            motion_properties=swing_damping,
            mimic=Mimic(joint="beam_to_drive_link", multiplier=1.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("overhead_frame")
    bench = object_model.get_part("bench_frame")
    drive_link = object_model.get_part("drive_link")
    swing = object_model.get_articulation("beam_to_drive_link")

    link_pin_pairs = [
        (drive_link, "upper_pin_0", "lower_pin_0"),
        (object_model.get_part("parallel_link_0"), "upper_pin_1", "lower_pin_1"),
        (object_model.get_part("parallel_link_1"), "upper_pin_2", "lower_pin_2"),
        (object_model.get_part("parallel_link_2"), "upper_pin_3", "lower_pin_3"),
    ]
    for link, upper_pin, lower_pin in link_pin_pairs:
        ctx.allow_overlap(
            frame,
            link,
            elem_a=upper_pin,
            elem_b="upper_eye",
            reason="The visible hinge pin is intentionally captured through the link eye at the overhead clevis.",
        )
        ctx.allow_overlap(
            bench,
            link,
            elem_a=lower_pin,
            elem_b="lower_eye",
            reason="The visible lower hinge pin is intentionally captured through the link eye in the bench clip.",
        )

    ctx.expect_overlap(
        drive_link,
        bench,
        axes="yz",
        elem_a="lower_eye",
        elem_b="lower_clip_0_inner",
        min_overlap=0.055,
        name="primary lower eye remains inside bench clip at rest",
    )
    ctx.expect_overlap(
        drive_link,
        frame,
        axes="yz",
        elem_a="upper_eye",
        elem_b="upper_clevis_0_inner",
        min_overlap=0.055,
        name="drive link upper eye sits between overhead clevis plates",
    )
    ctx.expect_overlap(
        frame,
        drive_link,
        axes="xyz",
        elem_a="upper_pin_0",
        elem_b="upper_eye",
        min_overlap=0.030,
        name="upper hinge pin captures drive link eye",
    )
    ctx.expect_gap(
        frame,
        bench,
        axis="z",
        positive_elem="top_beam",
        negative_elem="center_footwell",
        min_gap=1.30,
        name="suspended bench hangs below the overhead beam",
    )

    rest_pos = ctx.part_world_position(bench)
    with ctx.pose({swing: 0.36}):
        ctx.expect_overlap(
            drive_link,
            bench,
            axes="yz",
            elem_a="lower_eye",
            elem_b="lower_clip_0_inner",
            min_overlap=0.050,
            name="primary lower eye remains clipped while swinging",
        )
        ctx.expect_overlap(
            bench,
            drive_link,
            axes="xyz",
            elem_a="lower_pin_0",
            elem_b="lower_eye",
            min_overlap=0.025,
            name="lower hinge pin stays through drive link eye while swinging",
        )
        swung_pos = ctx.part_world_position(bench)

    ctx.check(
        "bench swings as a unit on parallel links",
        rest_pos is not None
        and swung_pos is not None
        and swung_pos[1] > rest_pos[1] + 0.25
        and swung_pos[2] > rest_pos[2] + 0.04,
        details=f"rest={rest_pos}, swung={swung_pos}",
    )

    return ctx.report()


object_model = build_object_model()
