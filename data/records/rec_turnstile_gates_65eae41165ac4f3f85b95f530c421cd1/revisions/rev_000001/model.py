from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="three_arm_turnstile_gate")

    powder_coat = model.material("powder_coat", rgba=(0.23, 0.25, 0.28, 1.0))
    stainless = model.material("stainless", rgba=(0.77, 0.79, 0.81, 1.0))
    black_polymer = model.material("black_polymer", rgba=(0.08, 0.09, 0.10, 1.0))
    indicator_green = model.material("indicator_green", rgba=(0.20, 0.78, 0.42, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((1.12, 0.62, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=powder_coat,
        name="plinth",
    )
    frame.visual(
        Box((1.12, 0.20, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 0.075)),
        material=powder_coat,
        name="threshold_beam",
    )
    frame.visual(
        Box((0.16, 0.28, 1.18)),
        origin=Origin(xyz=(-0.48, 0.0, 0.64)),
        material=powder_coat,
        name="left_stanchion",
    )
    frame.visual(
        Box((0.16, 0.28, 1.18)),
        origin=Origin(xyz=(0.48, 0.0, 0.64)),
        material=powder_coat,
        name="right_stanchion",
    )
    frame.visual(
        Box((1.12, 0.28, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 1.18)),
        material=powder_coat,
        name="top_crosshead",
    )
    frame.visual(
        Cylinder(radius=0.11, length=0.80),
        origin=Origin(xyz=(0.0, 0.0, 0.45)),
        material=powder_coat,
        name="central_column",
    )
    frame.visual(
        Cylinder(radius=0.135, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 0.90)),
        material=powder_coat,
        name="bearing_head",
    )
    frame.visual(
        Box((0.08, 0.12, 0.22)),
        origin=Origin(xyz=(-0.44, 0.10, 0.92)),
        material=black_polymer,
        name="reader_pod",
    )
    frame.visual(
        Box((0.05, 0.004, 0.11)),
        origin=Origin(xyz=(-0.399, 0.142, 0.92)),
        material=indicator_green,
        name="status_light",
    )
    frame.inertial = Inertial.from_geometry(
        Box((1.12, 0.62, 1.23)),
        mass=165.0,
        origin=Origin(xyz=(0.0, 0.0, 0.615)),
    )

    rotor = model.part("rotor")
    rotor.visual(
        Cylinder(radius=0.038, length=0.12),
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
        material=stainless,
        name="spindle",
    )
    rotor.visual(
        Cylinder(radius=0.082, length=0.09),
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
        material=black_polymer,
        name="hub_barrel",
    )
    rotor.visual(
        Cylinder(radius=0.094, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.106)),
        material=stainless,
        name="hub_cap",
    )
    for index in range(3):
        angle = index * (2.0 * math.pi / 3.0)
        arm_origin = Origin(
            xyz=(0.19 * math.cos(angle), 0.19 * math.sin(angle), 0.026),
            rpy=(0.0, math.pi / 2.0, angle),
        )
        tip_origin = Origin(
            xyz=(0.367 * math.cos(angle), 0.367 * math.sin(angle), 0.026),
            rpy=(0.0, math.pi / 2.0, angle),
        )
        rotor.visual(
            Cylinder(radius=0.018, length=0.38),
            origin=arm_origin,
            material=stainless,
            name=f"arm_{index}",
        )
        rotor.visual(
            Cylinder(radius=0.028, length=0.026),
            origin=tip_origin,
            material=black_polymer,
            name=f"arm_tip_{index}",
        )
    rotor.inertial = Inertial.from_geometry(
        Cylinder(radius=0.20, length=0.18),
        mass=14.0,
        origin=Origin(xyz=(0.0, 0.0, 0.09)),
    )

    model.articulation(
        "rotor_spin",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=rotor,
        origin=Origin(xyz=(0.0, 0.0, 0.95)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=4.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    rotor = object_model.get_part("rotor")
    spin = object_model.get_articulation("rotor_spin")

    ctx.expect_overlap(
        rotor,
        frame,
        axes="xy",
        elem_a="hub_barrel",
        elem_b="bearing_head",
        min_overlap=0.15,
        name="rotor hub stays centered over the bearing head",
    )
    ctx.expect_gap(
        frame,
        rotor,
        axis="z",
        positive_elem="top_crosshead",
        negative_elem="arm_0",
        min_gap=0.04,
        name="resting arm clears the top crosshead",
    )
    ctx.expect_gap(
        rotor,
        frame,
        axis="z",
        positive_elem="arm_0",
        negative_elem="plinth",
        min_gap=0.82,
        name="resting arm sits well above the floor plinth",
    )

    with ctx.pose({spin: 2.0 * math.pi / 3.0}):
        ctx.expect_gap(
            frame,
            rotor,
            axis="z",
            positive_elem="top_crosshead",
            negative_elem="arm_0",
            min_gap=0.04,
            name="rotated arm still clears the top crosshead",
        )
        ctx.expect_overlap(
            rotor,
            frame,
            axes="xy",
            elem_a="hub_barrel",
            elem_b="bearing_head",
            min_overlap=0.15,
            name="rotated hub remains centered over the bearing head",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
