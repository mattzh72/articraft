from __future__ import annotations

from math import atan2, pi

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
    model = ArticulatedObject(name="record_turntable")

    satin_black = Material("satin_black", rgba=(0.015, 0.014, 0.013, 1.0))
    dark_rubber = Material("dark_rubber", rgba=(0.005, 0.005, 0.004, 1.0))
    brushed_aluminum = Material("brushed_aluminum", rgba=(0.72, 0.70, 0.66, 1.0))
    smoked_gray = Material("smoked_gray", rgba=(0.10, 0.11, 0.12, 1.0))
    warm_wood = Material("warm_wood", rgba=(0.42, 0.22, 0.09, 1.0))
    white_mark = Material("white_mark", rgba=(0.92, 0.92, 0.86, 1.0))

    plinth = model.part("plinth")
    plinth.visual(
        Box((0.46, 0.36, 0.065)),
        origin=Origin(xyz=(0.0, 0.0, -0.0325)),
        material=warm_wood,
        name="wood_body",
    )
    plinth.visual(
        Cylinder(radius=0.045, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=brushed_aluminum,
        name="bearing_pedestal",
    )
    for i, (x, y) in enumerate(
        ((-0.175, -0.130), (-0.175, 0.130), (0.175, -0.130), (0.175, 0.130))
    ):
        plinth.visual(
            Cylinder(radius=0.032, length=0.014),
            origin=Origin(xyz=(x, y, -0.071)),
            material=satin_black,
            name=f"foot_{i}",
        )

    platter = model.part("platter")
    platter.visual(
        Cylinder(radius=0.155, length=0.032),
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
        material=brushed_aluminum,
        name="platter_disc",
    )
    platter.visual(
        Cylinder(radius=0.135, length=0.005),
        origin=Origin(xyz=(0.0, 0.0, 0.034)),
        material=dark_rubber,
        name="rubber_mat",
    )
    platter.visual(
        Cylinder(radius=0.004, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.046)),
        material=brushed_aluminum,
        name="center_spindle",
    )
    platter.visual(
        Box((0.020, 0.006, 0.002)),
        origin=Origin(xyz=(0.116, 0.0, 0.0375)),
        material=white_mark,
        name="strobe_mark",
    )

    model.articulation(
        "platter_spin",
        ArticulationType.CONTINUOUS,
        parent=plinth,
        child=platter,
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.5, velocity=12.0),
    )

    tonearm_base = model.part("tonearm_base")
    tonearm_base.visual(
        Cylinder(radius=0.038, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=satin_black,
        name="mount_plate",
    )
    tonearm_base.visual(
        Cylinder(radius=0.021, length=0.075),
        origin=Origin(xyz=(0.0, 0.0, 0.0455)),
        material=brushed_aluminum,
        name="base_column",
    )

    base_xy = (0.170, 0.105)
    model.articulation(
        "base_mount",
        ArticulationType.FIXED,
        parent=plinth,
        child=tonearm_base,
        origin=Origin(xyz=(base_xy[0], base_xy[1], 0.0)),
    )

    # Orient the tonearm so its neutral pose reaches diagonally from the side
    # base toward the inner record area.
    stylus_target = (-0.045, -0.025)
    arm_yaw = atan2(stylus_target[1] - base_xy[1], stylus_target[0] - base_xy[0])

    tonearm = model.part("tonearm")
    tonearm.visual(
        Cylinder(radius=0.018, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=smoked_gray,
        name="pivot_collar",
    )
    tonearm.visual(
        Cylinder(radius=0.0045, length=0.216),
        origin=Origin(xyz=(0.108, 0.0, 0.010), rpy=(0.0, pi / 2.0, 0.0)),
        material=brushed_aluminum,
        name="arm_tube",
    )
    tonearm.visual(
        Cylinder(radius=0.013, length=0.038),
        origin=Origin(xyz=(-0.025, 0.0, 0.010), rpy=(0.0, pi / 2.0, 0.0)),
        material=smoked_gray,
        name="counterweight",
    )
    tonearm.visual(
        Box((0.044, 0.024, 0.006)),
        origin=Origin(xyz=(0.235, 0.0, 0.005)),
        material=satin_black,
        name="headshell",
    )
    tonearm.visual(
        Box((0.020, 0.014, 0.008)),
        origin=Origin(xyz=(0.249, 0.0, -0.002)),
        material=smoked_gray,
        name="cartridge",
    )
    tonearm.visual(
        Cylinder(radius=0.0018, length=0.030),
        origin=Origin(xyz=(0.252, 0.0, -0.013)),
        material=satin_black,
        name="stylus",
    )

    model.articulation(
        "tonearm_pivot",
        ArticulationType.REVOLUTE,
        parent=tonearm_base,
        child=tonearm,
        origin=Origin(xyz=(0.0, 0.0, 0.083), rpy=(0.0, 0.0, arm_yaw)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.25, velocity=1.4, lower=-0.45, upper=0.45),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    plinth = object_model.get_part("plinth")
    platter = object_model.get_part("platter")
    tonearm_base = object_model.get_part("tonearm_base")
    tonearm = object_model.get_part("tonearm")
    platter_spin = object_model.get_articulation("platter_spin")
    tonearm_pivot = object_model.get_articulation("tonearm_pivot")

    ctx.check(
        "platter uses continuous vertical spin",
        platter_spin.articulation_type == ArticulationType.CONTINUOUS
        and tuple(platter_spin.axis) == (0.0, 0.0, 1.0),
        details=f"type={platter_spin.articulation_type}, axis={platter_spin.axis}",
    )
    ctx.expect_origin_distance(
        platter,
        plinth,
        axes="xy",
        max_dist=0.001,
        name="platter centered on plinth main axis",
    )
    ctx.expect_within(
        platter,
        plinth,
        axes="xy",
        margin=0.0,
        elem_a="platter_disc",
        elem_b="wood_body",
        name="platter footprint stays inside plinth",
    )
    ctx.expect_gap(
        platter,
        plinth,
        axis="z",
        positive_elem="platter_disc",
        negative_elem="bearing_pedestal",
        max_gap=0.001,
        max_penetration=0.0,
        name="platter rests on bearing pedestal",
    )
    ctx.expect_overlap(
        platter,
        plinth,
        axes="xy",
        elem_a="platter_disc",
        elem_b="bearing_pedestal",
        min_overlap=0.08,
        name="platter centered over bearing",
    )

    ctx.expect_origin_distance(
        tonearm_base,
        platter,
        axes="xy",
        min_dist=0.18,
        max_dist=0.23,
        name="tonearm base sits off to one side",
    )
    ctx.expect_contact(
        tonearm,
        tonearm_base,
        elem_a="pivot_collar",
        elem_b="base_column",
        contact_tol=0.001,
        name="tonearm collar seats on base column",
    )
    ctx.check(
        "tonearm uses bounded vertical pivot",
        tonearm_pivot.articulation_type == ArticulationType.REVOLUTE
        and tuple(tonearm_pivot.axis) == (0.0, 0.0, 1.0)
        and tonearm_pivot.motion_limits is not None
        and tonearm_pivot.motion_limits.lower < 0.0
        and tonearm_pivot.motion_limits.upper > 0.0,
        details=(
            f"type={tonearm_pivot.articulation_type}, axis={tonearm_pivot.axis}, "
            f"limits={tonearm_pivot.motion_limits}"
        ),
    )

    def elem_center(part, elem: str) -> tuple[float, float, float] | None:
        aabb = ctx.part_element_world_aabb(part, elem=elem)
        if aabb is None:
            return None
        lo, hi = aabb
        return (
            (lo[0] + hi[0]) / 2.0,
            (lo[1] + hi[1]) / 2.0,
            (lo[2] + hi[2]) / 2.0,
        )

    with ctx.pose({tonearm_pivot: tonearm_pivot.motion_limits.lower}):
        lower_head = elem_center(tonearm, "headshell")
    with ctx.pose({tonearm_pivot: tonearm_pivot.motion_limits.upper}):
        upper_head = elem_center(tonearm, "headshell")
    ctx.check(
        "tonearm headshell sweeps around pivot",
        lower_head is not None
        and upper_head is not None
        and ((upper_head[0] - lower_head[0]) ** 2 + (upper_head[1] - lower_head[1]) ** 2)
        > 0.020**2,
        details=f"lower={lower_head}, upper={upper_head}",
    )

    return ctx.report()


object_model = build_object_model()
