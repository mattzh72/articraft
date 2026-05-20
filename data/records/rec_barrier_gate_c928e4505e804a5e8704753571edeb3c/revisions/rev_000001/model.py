from __future__ import annotations

import math

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
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="railway_level_crossing_barrier")

    painted_steel = model.material("painted_steel", rgba=(0.62, 0.66, 0.66, 1.0))
    dark_rubber = model.material("dark_rubber", rgba=(0.03, 0.035, 0.035, 1.0))
    hinge_dark = model.material("dark_hinge", rgba=(0.015, 0.017, 0.018, 1.0))
    warning_red = model.material("warning_red", rgba=(0.85, 0.03, 0.02, 1.0))
    barrier_white = model.material("barrier_white", rgba=(0.96, 0.95, 0.86, 1.0))
    lamp_amber = model.material("lamp_amber", rgba=(1.0, 0.58, 0.08, 1.0))

    cabinet = model.part("cabinet")
    cabinet.visual(
        Box((0.95, 0.65, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
        material=dark_rubber,
        name="plinth",
    )
    cabinet.visual(
        Box((0.80, 0.52, 1.00)),
        origin=Origin(xyz=(0.0, 0.0, 0.62)),
        material=painted_steel,
        name="cabinet_body",
    )
    cabinet.visual(
        Box((0.86, 0.58, 0.07)),
        origin=Origin(xyz=(0.0, 0.0, 1.155)),
        material=painted_steel,
        name="sloped_cap",
    )

    # Door seams, louvers, and signal lenses are shallow surface details embedded
    # slightly into the cabinet face so the cabinet remains one physical body.
    cabinet.visual(
        Box((0.018, 0.012, 0.78)),
        origin=Origin(xyz=(0.0, 0.263, 0.62)),
        material=hinge_dark,
        name="door_center_seam",
    )
    cabinet.visual(
        Box((0.66, 0.012, 0.018)),
        origin=Origin(xyz=(0.0, 0.263, 0.99)),
        material=hinge_dark,
        name="door_top_seam",
    )
    for i, z in enumerate((0.33, 0.39, 0.45, 0.51)):
        cabinet.visual(
            Box((0.30, 0.014, 0.018)),
            origin=Origin(xyz=(-0.20, 0.264, z)),
            material=hinge_dark,
            name=f"vent_slat_{i}",
        )
    for i, z in enumerate((0.82, 0.94)):
        cabinet.visual(
            Cylinder(radius=0.055, length=0.045),
            origin=Origin(xyz=(-0.27, 0.276, z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=lamp_amber if i == 0 else warning_red,
            name=f"warning_lens_{i}",
        )

    # Forked hinge bracket on the cabinet side.  The moving boom hub runs between
    # the two cheeks, leaving a visible mechanical clearance.
    cabinet.visual(
        Cylinder(radius=0.035, length=0.320),
        origin=Origin(xyz=(0.57, 0.0, 1.02), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=hinge_dark,
        name="pivot_pin",
    )
    cabinet.visual(
        Box((0.22, 0.040, 0.34)),
        origin=Origin(xyz=(0.495, -0.115, 1.02)),
        material=hinge_dark,
        name="rear_clevis_plate",
    )
    cabinet.visual(
        Box((0.22, 0.040, 0.34)),
        origin=Origin(xyz=(0.495, 0.115, 1.02)),
        material=hinge_dark,
        name="front_clevis_plate",
    )

    boom_arm = model.part("boom_arm")
    boom_arm.visual(
        Cylinder(radius=0.105, length=0.150),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=hinge_dark,
        name="pivot_hub",
    )
    boom_arm.visual(
        Box((2.80, 0.110, 0.100)),
        origin=Origin(xyz=(1.49, 0.0, 0.0)),
        material=barrier_white,
        name="arm_beam",
    )
    for i, x in enumerate((0.42, 1.12, 1.82, 2.52)):
        boom_arm.visual(
            Box((0.34, 0.008, 0.104)),
            origin=Origin(xyz=(x, 0.057, 0.0)),
            material=warning_red,
            name=f"front_red_band_{i}",
        )
        boom_arm.visual(
            Box((0.34, 0.008, 0.104)),
            origin=Origin(xyz=(x, -0.057, 0.0)),
            material=warning_red,
            name=f"rear_red_band_{i}",
        )
        boom_arm.visual(
            Box((0.34, 0.108, 0.008)),
            origin=Origin(xyz=(x, 0.0, 0.052)),
            material=warning_red,
            name=f"top_red_band_{i}",
        )

    boom_arm.visual(
        Cylinder(radius=0.030, length=0.280),
        origin=Origin(xyz=(3.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=hinge_dark,
        name="fold_pin",
    )
    boom_arm.visual(
        Box((0.12, 0.035, 0.12)),
        origin=Origin(xyz=(2.90, -0.068, 0.0)),
        material=hinge_dark,
        name="rear_fold_web",
    )
    boom_arm.visual(
        Box((0.12, 0.035, 0.12)),
        origin=Origin(xyz=(2.90, 0.068, 0.0)),
        material=hinge_dark,
        name="front_fold_web",
    )
    boom_arm.visual(
        Cylinder(radius=0.085, length=0.035),
        origin=Origin(xyz=(2.94, -0.095, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=hinge_dark,
        name="rear_fold_knuckle",
    )
    boom_arm.visual(
        Cylinder(radius=0.085, length=0.035),
        origin=Origin(xyz=(2.94, 0.095, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=hinge_dark,
        name="front_fold_knuckle",
    )

    end_section = model.part("end_section")
    end_section.visual(
        Cylinder(radius=0.075, length=0.090),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=hinge_dark,
        name="fold_barrel",
    )
    end_section.visual(
        Box((2.84, 0.100, 0.090)),
        origin=Origin(xyz=(1.485, 0.0, 0.0)),
        material=barrier_white,
        name="end_beam",
    )
    for i, x in enumerate((0.38, 1.08, 1.78, 2.48)):
        end_section.visual(
            Box((0.32, 0.008, 0.094)),
            origin=Origin(xyz=(x, 0.052, 0.0)),
            material=warning_red,
            name=f"front_red_band_{i}",
        )
        end_section.visual(
            Box((0.32, 0.008, 0.094)),
            origin=Origin(xyz=(x, -0.052, 0.0)),
            material=warning_red,
            name=f"rear_red_band_{i}",
        )
        end_section.visual(
            Box((0.32, 0.098, 0.008)),
            origin=Origin(xyz=(x, 0.0, 0.047)),
            material=warning_red,
            name=f"top_red_band_{i}",
        )
    end_section.visual(
        Cylinder(radius=0.040, length=0.055),
        origin=Origin(xyz=(2.91, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=warning_red,
        name="end_marker_lamp",
    )

    model.articulation(
        "cabinet_to_boom",
        ArticulationType.REVOLUTE,
        parent=cabinet,
        child=boom_arm,
        origin=Origin(xyz=(0.57, 0.0, 1.02)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=180.0, velocity=0.65, lower=0.0, upper=1.35),
    )

    model.articulation(
        "boom_to_end",
        ArticulationType.REVOLUTE,
        parent=boom_arm,
        child=end_section,
        origin=Origin(xyz=(3.0, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=0.8, lower=0.0, upper=2.70),
        mimic=Mimic(joint="cabinet_to_boom", multiplier=2.0, offset=0.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cabinet = object_model.get_part("cabinet")
    boom = object_model.get_part("boom_arm")
    end = object_model.get_part("end_section")
    boom_hinge = object_model.get_articulation("cabinet_to_boom")
    fold_hinge = object_model.get_articulation("boom_to_end")

    ctx.check(
        "boom hinge is revolute",
        boom_hinge.articulation_type == ArticulationType.REVOLUTE,
        details=str(boom_hinge.articulation_type),
    )
    ctx.check(
        "fold hinge is revolute",
        fold_hinge.articulation_type == ArticulationType.REVOLUTE,
        details=str(fold_hinge.articulation_type),
    )
    ctx.check(
        "fold hinge follows boom raise",
        fold_hinge.mimic is not None
        and fold_hinge.mimic.joint == "cabinet_to_boom"
        and fold_hinge.mimic.multiplier == 2.0,
        details=str(fold_hinge.mimic),
    )

    ctx.allow_overlap(
        cabinet,
        boom,
        elem_a="pivot_pin",
        elem_b="pivot_hub",
        reason="The cabinet hinge pin is intentionally captured through the boom pivot hub.",
    )
    ctx.allow_overlap(
        boom,
        end,
        elem_a="fold_pin",
        elem_b="fold_barrel",
        reason="The folding joint pin is intentionally captured through the end-section hinge barrel.",
    )

    with ctx.pose({boom_hinge: 0.0}):
        ctx.expect_overlap(
            cabinet,
            boom,
            axes="xyz",
            min_overlap=0.06,
            elem_a="pivot_pin",
            elem_b="pivot_hub",
            name="boom pivot hub is captured by cabinet pin",
        )
        ctx.expect_gap(
            end,
            boom,
            axis="x",
            min_gap=0.10,
            max_gap=0.25,
            positive_elem="end_beam",
            negative_elem="arm_beam",
            name="fold hinge separates the two beam sections",
        )
        ctx.expect_overlap(
            boom,
            end,
            axes="xyz",
            min_overlap=0.05,
            elem_a="fold_pin",
            elem_b="fold_barrel",
            name="fold barrel is captured by the fold pin",
        )

    rest_boom_aabb = ctx.part_element_world_aabb(boom, elem="arm_beam")
    rest_end_aabb = ctx.part_element_world_aabb(end, elem="end_beam")
    upper = boom_hinge.motion_limits.upper if boom_hinge.motion_limits else 1.35
    with ctx.pose({boom_hinge: upper}):
        raised_boom_aabb = ctx.part_element_world_aabb(boom, elem="arm_beam")
        raised_end_aabb = ctx.part_element_world_aabb(end, elem="end_beam")

    ctx.check(
        "boom arm raises upward",
        rest_boom_aabb is not None
        and raised_boom_aabb is not None
        and raised_boom_aabb[1][2] > rest_boom_aabb[1][2] + 2.0,
        details=f"rest={rest_boom_aabb}, raised={raised_boom_aabb}",
    )
    ctx.check(
        "folding end stays down relative to raised arm",
        raised_boom_aabb is not None
        and raised_end_aabb is not None
        and rest_end_aabb is not None
        and raised_end_aabb[0][2] < raised_boom_aabb[1][2] - 1.8
        and (raised_end_aabb[1][2] - raised_end_aabb[0][2]) > 2.0,
        details=f"rest_end={rest_end_aabb}, raised_boom={raised_boom_aabb}, raised_end={raised_end_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
