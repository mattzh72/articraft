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
    TorusGeometry,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="masonry_overshot_waterwheel")

    stone = model.material("varied_grey_stone", color=(0.44, 0.43, 0.39, 1.0))
    dark_mortar = model.material("dark_mortar", color=(0.18, 0.17, 0.16, 1.0))
    aged_oak = model.material("aged_oak", color=(0.55, 0.33, 0.16, 1.0))
    wet_wood = model.material("dark_wet_wood", color=(0.25, 0.15, 0.08, 1.0))
    iron = model.material("blackened_iron", color=(0.07, 0.07, 0.065, 1.0))
    water = model.material("clear_water", color=(0.24, 0.55, 0.92, 0.48))

    masonry = model.part("masonry")

    # Broad foundation tying the two stone sidewalls and the raised chute support
    # into one continuous masonry assembly.
    masonry.visual(
        Box((2.05, 1.04, 0.12)),
        origin=Origin(xyz=(-0.18, 0.0, 0.06)),
        material=stone,
        name="foundation",
    )

    # Two sidewalls are made as pier-and-lintel frames so the axle can pass
    # through a real open bay rather than being hidden inside a solid slab.
    for y in (-0.46, 0.46):
        masonry.visual(
            Box((0.28, 0.16, 0.94)),
            origin=Origin(xyz=(-0.62, y, 0.59)),
            material=stone,
            name=f"front_pier_{'n' if y < 0 else 's'}",
        )
        masonry.visual(
            Box((0.28, 0.16, 0.94)),
            origin=Origin(xyz=(0.50, y, 0.59)),
            material=stone,
            name=f"rear_pier_{'n' if y < 0 else 's'}",
        )
        masonry.visual(
            Box((1.40, 0.16, 0.22)),
            origin=Origin(xyz=(-0.06, y, 1.15)),
            material=stone,
            name=f"top_lintel_{'n' if y < 0 else 's'}",
        )
        masonry.visual(
            Box((1.34, 0.17, 0.16)),
            origin=Origin(xyz=(-0.06, y, 0.18)),
            material=stone,
            name=f"stone_sill_{'n' if y < 0 else 's'}",
        )
        # Bearing cheek blocks flank the open bay without intersecting the shaft.
        masonry.visual(
            Box((0.12, 0.10, 0.18)),
            origin=Origin(xyz=(-0.0975, y * 0.96, 0.66)),
            material=stone,
            name=f"bearing_cheek_{'n' if y < 0 else 's'}",
        )
        masonry.visual(
            Box((0.12, 0.10, 0.34)),
            origin=Origin(xyz=(-0.14, y * 0.96, 0.42)),
            material=stone,
            name=f"bearing_pedestal_{'n' if y < 0 else 's'}",
        )

        # Mortar lines on the outside faces make the support read as masonry
        # rather than a single cast block.
        outside_y = y + (0.083 if y > 0 else -0.083)
        for course, z in enumerate((0.38, 0.58, 0.78, 0.98)):
            masonry.visual(
                Box((1.36, 0.012, 0.018)),
                origin=Origin(xyz=(-0.06, outside_y, z)),
                material=dark_mortar,
                name=f"mortar_course_{'n' if y < 0 else 's'}_{course}",
            )
        for joint, x in enumerate((-0.74, -0.48, 0.38, 0.64)):
            masonry.visual(
                Box((0.018, 0.012, 0.76)),
                origin=Origin(xyz=(x, outside_y, 0.70)),
                material=dark_mortar,
                name=f"mortar_joint_{'n' if y < 0 else 's'}_{joint}",
            )

    # Sloped wooden feed chute above the wheel.  It is physically supported by
    # posts bearing on the masonry and includes fixed guide rails for the gate.
    chute_angle = math.atan2(-0.18, 0.78)
    chute_length = math.hypot(0.78, 0.18)
    chute_mid = (-0.63, 0.0, 1.25)
    chute_rpy = (0.0, -chute_angle, 0.0)

    def add_chute_bottom(name: str, x0: float, z0: float, x1: float, z1: float) -> None:
        dx = x1 - x0
        dz = z1 - z0
        angle = math.atan2(dz, dx)
        masonry.visual(
            Box((math.hypot(dx, dz), 0.50, 0.045)),
            origin=Origin(xyz=((x0 + x1) / 2.0, 0.0, (z0 + z1) / 2.0), rpy=(0.0, -angle, 0.0)),
            material=wet_wood,
            name=name,
        )

    # The bottom is split around the gate slot so the sliding plate does not
    # intersect a solid trough board.
    add_chute_bottom("chute_bottom_upstream", -1.02, 1.34, -0.80, 1.29)
    add_chute_bottom("chute_bottom_downstream", -0.64, 1.25, -0.24, 1.16)

    for y in (-0.270, 0.270):
        masonry.visual(
            Box((chute_length, 0.045, 0.18)),
            origin=Origin(xyz=(chute_mid[0], y, chute_mid[2] + 0.075), rpy=chute_rpy),
            material=aged_oak,
            name=f"chute_side_{'n' if y < 0 else 's'}",
        )
    masonry.visual(
        Box((0.22, 0.48, 0.055)),
        origin=Origin(xyz=(-0.18, 0.0, 1.25), rpy=chute_rpy),
        material=aged_oak,
        name="chute_lip",
    )
    for y in (-0.255, 0.255):
        masonry.visual(
            Box((0.12, 0.05, 0.16)),
            origin=Origin(xyz=(-0.25, y, 1.25)),
            material=aged_oak,
            name=f"lip_cleat_{'n' if y < 0 else 's'}",
        )
    masonry.visual(
        Box((0.20, 0.18, 0.025)),
        origin=Origin(xyz=(-0.10, 0.0, 1.225), rpy=(0.0, -0.15, 0.0)),
        material=water,
        name="water_ribbon",
    )

    for x, height in ((-0.94, 1.34), (-0.40, 1.22)):
        for y in (-0.29, 0.29):
            masonry.visual(
                Box((0.065, 0.065, height - 0.12)),
                origin=Origin(xyz=(x, y, (height + 0.12) / 2.0)),
                material=aged_oak,
                name=f"chute_post_{x:.2f}_{'n' if y < 0 else 's'}",
            )

    # Gate guide: a back board, two side rails, and stop bars.  The moving gate
    # stays between these rails throughout its vertical travel.
    masonry.visual(
        Box((0.018, 0.50, 0.50)),
        origin=Origin(xyz=(-0.755, 0.0, 1.14)),
        material=aged_oak,
        name="gate_back",
    )
    for y in (-0.238, 0.238):
        masonry.visual(
            Box((0.080, 0.035, 0.52)),
            origin=Origin(xyz=(-0.72, y, 1.14)),
            material=wet_wood,
            name=f"gate_rail_{'n' if y < 0 else 's'}",
        )
    for z, label in ((0.88, "lower"), (1.40, "upper")):
        masonry.visual(
            Box((0.085, 0.52, 0.035)),
            origin=Origin(xyz=(-0.72, 0.0, z)),
            material=wet_wood,
            name=f"gate_{label}_stop",
        )
    for y in (-0.238, 0.238):
        masonry.visual(
            Box((0.060, 0.035, 0.78)),
            origin=Origin(xyz=(-0.72, y, 0.49)),
            material=aged_oak,
            name=f"gate_support_{'n' if y < 0 else 's'}",
        )

    wheel = model.part("wheel")
    # The child frame is the axle center.  The shaft, wooden wheel, buckets and
    # outboard pulley are one rigid rotating shaft assembly.
    wheel.visual(
        Cylinder(radius=0.038, length=1.42),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=iron,
        name="shaft",
    )
    wheel.visual(
        Cylinder(radius=0.090, length=0.32),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=iron,
        name="hub",
    )

    rim_mesh = mesh_from_geometry(TorusGeometry(0.425, 0.026, radial_segments=20, tubular_segments=48), "waterwheel_rim")
    for y in (-0.145, 0.145):
        wheel.visual(
            rim_mesh,
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=aged_oak,
            name=f"rim_{'n' if y < 0 else 's'}",
        )

    for i in range(10):
        theta = 2.0 * math.pi * i / 10.0
        wheel.visual(
            Box((0.68, 0.052, 0.036)),
            origin=Origin(
                xyz=(0.19 * math.cos(theta), 0.0, 0.19 * math.sin(theta)),
                rpy=(0.0, -theta, 0.0),
            ),
            material=aged_oak,
            name=f"spoke_{i}",
        )

    for i in range(16):
        theta = 2.0 * math.pi * i / 16.0
        wheel.visual(
            Box((0.125, 0.335, 0.034)),
            origin=Origin(
                xyz=(0.435 * math.cos(theta), 0.0, 0.435 * math.sin(theta)),
                rpy=(0.0, -(theta + math.pi / 2.0), 0.0),
            ),
            material=wet_wood,
            name=f"bucket_{i}",
        )

    # Small outboard drive pulley fixed to the shaft end beyond one sidewall.
    for y, radius, width, label in ((0.630, 0.125, 0.070, "pulley_sheave"), (0.585, 0.146, 0.014, "pulley_flange_n"), (0.675, 0.146, 0.014, "pulley_flange_s")):
        wheel.visual(
            Cylinder(radius=radius, length=width),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=iron,
            name=label,
        )

    gate = model.part("chute_gate")
    gate.visual(
        Box((0.026, 0.392, 0.310)),
        origin=Origin(xyz=(0.0, 0.0, 0.155)),
        material=iron,
        name="gate_panel",
    )
    for y in (-0.205, 0.205):
        gate.visual(
            Box((0.034, 0.020, 0.350)),
            origin=Origin(xyz=(0.0, y, 0.175)),
            material=iron,
            name=f"gate_tongue_{'n' if y < 0 else 's'}",
        )
    gate.visual(
        Cylinder(radius=0.022, length=0.30),
        origin=Origin(xyz=(0.030, 0.0, 0.285), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=iron,
        name="gate_handle",
    )

    model.articulation(
        "axle_spin",
        ArticulationType.CONTINUOUS,
        parent=masonry,
        child=wheel,
        origin=Origin(xyz=(0.0, 0.0, 0.66)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=4.0),
    )

    model.articulation(
        "gate_slide",
        ArticulationType.PRISMATIC,
        parent=masonry,
        child=gate,
        origin=Origin(xyz=(-0.720, 0.0, 0.895)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=25.0, velocity=0.25, lower=0.0, upper=0.18),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    masonry = object_model.get_part("masonry")
    wheel = object_model.get_part("wheel")
    gate = object_model.get_part("chute_gate")
    axle = object_model.get_articulation("axle_spin")
    slide = object_model.get_articulation("gate_slide")

    ctx.check(
        "wheel and pulley share one continuous shaft assembly",
        axle.articulation_type == ArticulationType.CONTINUOUS
        and any(v.name == "pulley_sheave" for v in wheel.visuals),
        details="The small outboard pulley should rotate as part of the same shaft part as the wheel.",
    )

    ctx.expect_within(
        gate,
        masonry,
        axes="y",
        inner_elem="gate_panel",
        outer_elem="gate_back",
        margin=0.004,
        name="closed gate is laterally captured by guide",
    )
    ctx.expect_gap(
        gate,
        masonry,
        axis="x",
        min_gap=0.006,
        max_gap=0.035,
        positive_elem="gate_panel",
        negative_elem="gate_back",
        name="closed gate slides just proud of guide back",
    )
    ctx.expect_overlap(
        gate,
        masonry,
        axes="z",
        elem_a="gate_panel",
        elem_b="gate_back",
        min_overlap=0.28,
        name="closed gate remains inside vertical guide height",
    )

    rest_z = ctx.part_world_position(gate)[2]
    with ctx.pose({slide: 0.18}):
        ctx.expect_within(
            gate,
            masonry,
            axes="y",
            inner_elem="gate_panel",
            outer_elem="gate_back",
            margin=0.004,
            name="raised gate is laterally captured by guide",
        )
        ctx.expect_overlap(
            gate,
            masonry,
            axes="z",
            elem_a="gate_panel",
            elem_b="gate_back",
            min_overlap=0.27,
            name="raised gate retains vertical guide insertion",
        )
        raised_z = ctx.part_world_position(gate)[2]

    ctx.check(
        "gate slide raises the chute gate",
        raised_z > rest_z + 0.16,
        details=f"rest_z={rest_z}, raised_z={raised_z}",
    )

    rest_pos = ctx.part_world_position(wheel)
    with ctx.pose({axle: math.pi / 2.0}):
        spun_pos = ctx.part_world_position(wheel)
    ctx.check(
        "axle spin keeps wheel centered on horizontal shaft",
        rest_pos is not None and spun_pos is not None and abs(spun_pos[2] - rest_pos[2]) < 1.0e-6,
        details=f"rest={rest_pos}, spun={spun_pos}",
    )

    return ctx.report()


object_model = build_object_model()
