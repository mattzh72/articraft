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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pedestal_indexer")

    dark_iron = Material("dark_iron", color=(0.08, 0.085, 0.09, 1.0))
    satin_steel = Material("satin_steel", color=(0.58, 0.60, 0.58, 1.0))
    machined_aluminum = Material("machined_aluminum", color=(0.72, 0.74, 0.72, 1.0))
    blue_anodized = Material("blue_anodized", color=(0.08, 0.22, 0.45, 1.0))
    black_rubber = Material("black_rubber", color=(0.015, 0.015, 0.012, 1.0))
    safety_yellow = Material("safety_yellow", color=(0.95, 0.72, 0.08, 1.0))

    pedestal = model.part("pedestal")
    pedestal.visual(
        Cylinder(radius=0.34, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 0.0275)),
        material=dark_iron,
        name="floor_foot",
    )
    pedestal.visual(
        Cylinder(radius=0.275, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.080)),
        material=satin_steel,
        name="bearing_housing",
    )
    pedestal.visual(
        Cylinder(radius=0.225, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.102)),
        material=machined_aluminum,
        name="lower_race",
    )
    pedestal.visual(
        Cylinder(radius=0.080, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.095)),
        material=dark_iron,
        name="center_boss",
    )

    # Low clamp pads make the stationary casting read as bolted to the floor.
    for i in range(8):
        angle = i * math.tau / 8.0
        x = 0.285 * math.cos(angle)
        y = 0.285 * math.sin(angle)
        pedestal.visual(
            Cylinder(radius=0.016, length=0.008),
            origin=Origin(xyz=(x, y, 0.059)),
            material=dark_iron,
            name=f"base_bolt_{i}",
        )

    turntable_mast = model.part("turntable_mast")
    turntable_mast.visual(
        Cylinder(radius=0.300, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=satin_steel,
        name="turntable_disk",
    )
    turntable_mast.visual(
        Cylinder(radius=0.255, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.064)),
        material=machined_aluminum,
        name="top_face",
    )
    turntable_mast.visual(
        Cylinder(radius=0.155, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.069)),
        material=dark_iron,
        name="index_face",
    )

    # A bolt circle and index ticks on the rotating disk provide machined scale cues.
    for i in range(12):
        angle = i * math.tau / 12.0
        x = 0.235 * math.cos(angle)
        y = 0.235 * math.sin(angle)
        turntable_mast.visual(
            Cylinder(radius=0.011, length=0.007),
            origin=Origin(xyz=(x, y, 0.0695)),
            material=dark_iron,
            name=f"turntable_bolt_{i}",
        )
    for i in range(16):
        angle = i * math.tau / 16.0
        x = 0.185 * math.cos(angle)
        y = 0.185 * math.sin(angle)
        # Short chordal pads stand in for engraved index marks without floating.
        turntable_mast.visual(
            Box((0.030, 0.006, 0.004)),
            origin=Origin(xyz=(x, y, 0.0675), rpy=(0.0, 0.0, angle)),
            material=dark_iron,
            name=f"index_tick_{i}",
        )

    # The upper mast is rigidly part of the rotating assembly.
    turntable_mast.visual(
        Box((0.210, 0.075, 0.780)),
        origin=Origin(xyz=(0.0, 0.040, 0.460)),
        material=dark_iron,
        name="mast_wall",
    )
    turntable_mast.visual(
        Box((0.250, 0.105, 0.050)),
        origin=Origin(xyz=(0.0, 0.028, 0.095)),
        material=dark_iron,
        name="mast_foot",
    )
    turntable_mast.visual(
        Box((0.070, 0.110, 0.720)),
        origin=Origin(xyz=(-0.125, 0.035, 0.475)),
        material=dark_iron,
        name="side_rib_0",
    )
    turntable_mast.visual(
        Box((0.070, 0.110, 0.720)),
        origin=Origin(xyz=(0.125, 0.035, 0.475)),
        material=dark_iron,
        name="side_rib_1",
    )
    for i, z in enumerate((0.200, 0.415, 0.630, 0.845)):
        turntable_mast.visual(
            Box((0.260, 0.024, 0.035)),
            origin=Origin(xyz=(0.0, 0.003, z)),
            material=satin_steel,
            name=f"mast_cross_tie_{i}",
        )

    rail_x = 0.072
    rail_y = -0.040
    rail_z_center = 0.510
    rail_length = 0.700
    for side, x in enumerate((-rail_x, rail_x)):
        turntable_mast.visual(
            Cylinder(radius=0.010, length=rail_length),
            origin=Origin(xyz=(x, rail_y, rail_z_center)),
            material=machined_aluminum,
            name=f"guide_rail_{side}",
        )
        turntable_mast.visual(
            Box((0.018, 0.030, rail_length)),
            origin=Origin(xyz=(x, -0.015, rail_z_center)),
            material=satin_steel,
            name=f"rail_backer_{side}",
        )
        for j, z in enumerate((0.155, 0.875)):
            turntable_mast.visual(
                Box((0.050, 0.042, 0.045)),
                origin=Origin(xyz=(x, -0.018, z)),
                material=satin_steel,
                name=f"rail_standoff_{side}_{j}",
            )
    turntable_mast.visual(
        Box((0.210, 0.035, 0.035)),
        origin=Origin(xyz=(0.0, -0.024, 0.150)),
        material=safety_yellow,
        name="bottom_stop",
    )
    turntable_mast.visual(
        Box((0.210, 0.035, 0.035)),
        origin=Origin(xyz=(0.0, -0.024, 0.870)),
        material=safety_yellow,
        name="top_stop",
    )
    for side, x in enumerate((-rail_x, rail_x)):
        turntable_mast.visual(
            Cylinder(radius=0.016, length=0.018),
            origin=Origin(xyz=(x, -0.051, 0.168)),
            material=black_rubber,
            name=f"lower_bumper_{side}",
        )
        turntable_mast.visual(
            Cylinder(radius=0.016, length=0.018),
            origin=Origin(xyz=(x, -0.051, 0.850)),
            material=black_rubber,
            name=f"upper_bumper_{side}",
        )

    carriage = model.part("carriage")
    # The carriage frame is kept forward of the guide rails, with open cages
    # around each rail so the prismatic travel has actual clearance.
    carriage.visual(
        Box((0.235, 0.018, 0.175)),
        origin=Origin(xyz=(0.0, -0.080, 0.090)),
        material=blue_anodized,
        name="front_plate",
    )
    carriage.visual(
        Box((0.190, 0.070, 0.035)),
        origin=Origin(xyz=(0.0, -0.108, 0.150)),
        material=blue_anodized,
        name="tool_cap",
    )
    carriage.visual(
        Box((0.190, 0.052, 0.028)),
        origin=Origin(xyz=(0.0, -0.101, 0.008)),
        material=blue_anodized,
        name="lower_cap",
    )
    for side, x in enumerate((-rail_x, rail_x)):
        # Two cheeks flank the rail in X; front bridges sit clear in Y.
        for cheek, sx in enumerate((-0.020, 0.020)):
            carriage.visual(
                Box((0.010, 0.046, 0.205)),
                origin=Origin(xyz=(x + sx, -0.040, 0.092)),
                material=blue_anodized,
                name=f"bearing_cheek_{side}_{cheek}",
            )
        carriage.visual(
            Box((0.055, 0.014, 0.205)),
            origin=Origin(xyz=(x, -0.065, 0.092)),
            material=blue_anodized,
            name=f"front_bridge_{side}",
        )
        carriage.visual(
            Box((0.055, 0.014, 0.205)),
            origin=Origin(xyz=(x, -0.076, 0.092)),
            material=blue_anodized,
            name=f"outer_bridge_{side}",
        )
        carriage.visual(
            Box((0.064, 0.020, 0.022)),
            origin=Origin(xyz=(x, -0.080, 0.197)),
            material=machined_aluminum,
            name=f"upper_carriage_cap_{side}",
        )
        carriage.visual(
            Box((0.064, 0.020, 0.022)),
            origin=Origin(xyz=(x, -0.080, -0.014)),
            material=machined_aluminum,
            name=f"lower_carriage_cap_{side}",
        )

    model.articulation(
        "pedestal_to_turntable",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=turntable_mast,
        origin=Origin(xyz=(0.0, 0.0, 0.110)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=180.0, velocity=1.4, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "mast_to_carriage",
        ArticulationType.PRISMATIC,
        parent=turntable_mast,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.240)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=900.0, velocity=0.22, lower=0.0, upper=0.340),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal")
    turntable_mast = object_model.get_part("turntable_mast")
    carriage = object_model.get_part("carriage")
    turn_joint = object_model.get_articulation("pedestal_to_turntable")
    lift_joint = object_model.get_articulation("mast_to_carriage")
    lift_limits = lift_joint.motion_limits

    ctx.check(
        "turntable and mast rotate as one lower link",
        turn_joint.articulation_type == ArticulationType.REVOLUTE
        and turn_joint.child == "turntable_mast"
        and tuple(turn_joint.axis) == (0.0, 0.0, 1.0),
        details=f"type={turn_joint.articulation_type}, child={turn_joint.child}, axis={turn_joint.axis}",
    )
    ctx.check(
        "carriage has vertical guarded lift travel",
        lift_joint.articulation_type == ArticulationType.PRISMATIC
        and lift_limits is not None
        and lift_limits.lower == 0.0
        and lift_limits.upper == 0.340
        and tuple(lift_joint.axis) == (0.0, 0.0, 1.0),
        details=f"type={lift_joint.articulation_type}, limits={lift_limits}, axis={lift_joint.axis}",
    )

    ctx.expect_contact(
        turntable_mast,
        pedestal,
        elem_a="turntable_disk",
        elem_b="lower_race",
        contact_tol=1e-5,
        name="rotating turntable seats on lower bearing race",
    )
    ctx.expect_overlap(
        turntable_mast,
        pedestal,
        axes="xy",
        elem_a="turntable_disk",
        elem_b="lower_race",
        min_overlap=0.40,
        name="turntable bearing faces have broad concentric support",
    )

    def expect_rail_clearance(prefix: str) -> None:
        for side in (0, 1):
            rail = f"guide_rail_{side}"
            negative_cheek = f"bearing_cheek_{side}_0"
            positive_cheek = f"bearing_cheek_{side}_1"
            if side == 0:
                ctx.expect_gap(
                    turntable_mast,
                    carriage,
                    axis="x",
                    positive_elem=rail,
                    negative_elem=negative_cheek,
                    min_gap=0.004,
                    max_gap=0.008,
                    name=f"{prefix} rail {side} clears inner cheek",
                )
                ctx.expect_gap(
                    carriage,
                    turntable_mast,
                    axis="x",
                    positive_elem=positive_cheek,
                    negative_elem=rail,
                    min_gap=0.004,
                    max_gap=0.008,
                    name=f"{prefix} rail {side} clears outer cheek",
                )
            else:
                ctx.expect_gap(
                    turntable_mast,
                    carriage,
                    axis="x",
                    positive_elem=rail,
                    negative_elem=negative_cheek,
                    min_gap=0.004,
                    max_gap=0.008,
                    name=f"{prefix} rail {side} clears inner cheek",
                )
                ctx.expect_gap(
                    carriage,
                    turntable_mast,
                    axis="x",
                    positive_elem=positive_cheek,
                    negative_elem=rail,
                    min_gap=0.004,
                    max_gap=0.008,
                    name=f"{prefix} rail {side} clears outer cheek",
                )
            ctx.expect_gap(
                turntable_mast,
                carriage,
                axis="y",
                positive_elem=rail,
                negative_elem=f"front_bridge_{side}",
                min_gap=0.006,
                max_gap=0.012,
                name=f"{prefix} rail {side} clears front bridge",
            )

    lower_pos = ctx.part_world_position(carriage)
    with ctx.pose({lift_joint: 0.0}):
        expect_rail_clearance("lower lift")
        ctx.expect_gap(
            turntable_mast,
            carriage,
            axis="y",
            positive_elem="mast_wall",
            min_gap=0.015,
            max_gap=0.025,
            name="lower lift carriage clears mast wall",
        )
        ctx.expect_gap(
            carriage,
            turntable_mast,
            axis="z",
            negative_elem="bottom_stop",
            min_gap=0.035,
            max_gap=0.060,
            name="lower lift stays above bottom stop",
        )

    with ctx.pose({lift_joint: 0.340}):
        expect_rail_clearance("upper lift")
        ctx.expect_gap(
            turntable_mast,
            carriage,
            axis="y",
            positive_elem="mast_wall",
            min_gap=0.015,
            max_gap=0.025,
            name="upper lift carriage clears mast wall",
        )
        ctx.expect_gap(
            turntable_mast,
            carriage,
            axis="z",
            positive_elem="top_stop",
            min_gap=0.050,
            max_gap=0.080,
            name="upper lift stays below top stop",
        )
        upper_pos = ctx.part_world_position(carriage)

    ctx.check(
        "prismatic carriage travels upward",
        lower_pos is not None and upper_pos is not None and upper_pos[2] > lower_pos[2] + 0.30,
        details=f"lower={lower_pos}, upper={upper_pos}",
    )

    with ctx.pose({turn_joint: math.pi / 2.0, lift_joint: 0.170}):
        ctx.expect_contact(
            turntable_mast,
            pedestal,
            elem_a="turntable_disk",
            elem_b="lower_race",
            contact_tol=1e-5,
            name="rotated bearing remains seated",
        )
        ctx.expect_overlap(
            turntable_mast,
            pedestal,
            axes="xy",
            elem_a="turntable_disk",
            elem_b="lower_race",
            min_overlap=0.40,
            name="rotated bearing support remains concentric",
        )

    return ctx.report()


object_model = build_object_model()
