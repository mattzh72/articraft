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
    Sphere,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="three_arm_turnstile_gate")

    powder_coat = model.material("powder_coat", rgba=(0.24, 0.26, 0.29, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.70, 0.73, 0.76, 1.0))
    dark_handle = model.material("dark_handle", rgba=(0.15, 0.16, 0.18, 1.0))
    floor_gray = model.material("floor_gray", rgba=(0.52, 0.54, 0.56, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((0.88, 0.74, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=floor_gray,
        name="base_plinth",
    )

    post_height = 1.00
    post_size = 0.06
    post_center_z = 0.05 + post_height / 2.0
    for sx in (-0.41, 0.41):
        for sy in (-0.34, 0.34):
            frame.visual(
                Box((post_size, post_size, post_height)),
                origin=Origin(xyz=(sx, sy, post_center_z)),
                material=powder_coat,
                name=f"corner_post_{'p' if sx > 0 else 'n'}x_{'p' if sy > 0 else 'n'}y",
            )

    for y in (-0.34, 0.34):
        frame.visual(
            Box((0.88, 0.06, 0.10)),
            origin=Origin(xyz=(0.0, y, 0.48)),
            material=powder_coat,
            name=f"lower_side_rail_{'front' if y > 0 else 'rear'}",
        )
        frame.visual(
            Box((0.88, 0.06, 0.08)),
            origin=Origin(xyz=(0.0, y, 1.09)),
            material=powder_coat,
            name=f"upper_side_beam_{'front' if y > 0 else 'rear'}",
        )

    for x in (-0.41, 0.41):
        frame.visual(
            Box((0.06, 0.62, 0.10)),
            origin=Origin(xyz=(x, 0.0, 0.48)),
            material=powder_coat,
            name=f"lower_end_rail_{'right' if x > 0 else 'left'}",
        )
        frame.visual(
            Box((0.06, 0.62, 0.08)),
            origin=Origin(xyz=(x, 0.0, 1.09)),
            material=powder_coat,
            name=f"upper_end_beam_{'right' if x > 0 else 'left'}",
        )

    frame.visual(
        Cylinder(radius=0.055, length=0.85),
        origin=Origin(xyz=(0.0, 0.0, 0.475)),
        material=brushed_steel,
        name="column_shaft",
    )
    frame.visual(
        Cylinder(radius=0.07, length=0.02),
        origin=Origin(xyz=(0.0, 0.0, 0.91)),
        material=brushed_steel,
        name="bearing_cap",
    )
    frame.inertial = Inertial.from_geometry(
        Box((0.88, 0.74, 1.13)),
        mass=95.0,
        origin=Origin(xyz=(0.0, 0.0, 0.565)),
    )

    rotor = model.part("rotor")
    rotor.visual(
        Cylinder(radius=0.10, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
        material=brushed_steel,
        name="hub_plate",
    )
    rotor.visual(
        Cylinder(radius=0.055, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, 0.09)),
        material=brushed_steel,
        name="hub_boss",
    )

    arm_radius = 0.019
    arm_length = 0.32
    arm_center_radius = arm_length / 2.0
    tip_radius = 0.026
    tip_center_radius = arm_length + 0.015
    for index in range(3):
        angle = (2.0 * math.pi * index) / 3.0
        rotor.visual(
            Cylinder(radius=arm_radius, length=arm_length),
            origin=Origin(
                xyz=(
                    arm_center_radius * math.cos(angle),
                    arm_center_radius * math.sin(angle),
                    0.03,
                ),
                rpy=(0.0, math.pi / 2.0, angle),
            ),
            material=dark_handle,
            name=f"arm_{index}",
        )
        rotor.visual(
            Sphere(radius=tip_radius),
            origin=Origin(
                xyz=(
                    tip_center_radius * math.cos(angle),
                    tip_center_radius * math.sin(angle),
                    0.03,
                )
            ),
            material=dark_handle,
            name=f"arm_tip_{index}",
        )

    rotor.inertial = Inertial.from_geometry(
        Cylinder(radius=0.36, length=0.12),
        mass=14.0,
        origin=Origin(xyz=(0.0, 0.0, 0.06)),
    )

    model.articulation(
        "frame_to_rotor",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=rotor,
        origin=Origin(xyz=(0.0, 0.0, 0.92)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=60.0, velocity=6.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    rotor = object_model.get_part("rotor")
    spin = object_model.get_articulation("frame_to_rotor")

    ctx.expect_origin_distance(
        rotor,
        frame,
        axes="xy",
        max_dist=0.001,
        name="rotor stays centered on the support axis",
    )
    ctx.expect_gap(
        rotor,
        frame,
        axis="z",
        positive_elem="hub_plate",
        negative_elem="bearing_cap",
        max_gap=0.001,
        max_penetration=0.0,
        name="hub seats on the bearing cap without overlap",
    )
    ctx.expect_overlap(
        rotor,
        frame,
        axes="xy",
        elem_a="hub_plate",
        elem_b="bearing_cap",
        min_overlap=0.13,
        name="hub remains over the top bearing footprint",
    )

    with ctx.pose({spin: math.pi / 3.0}):
        ctx.expect_origin_distance(
            rotor,
            frame,
            axes="xy",
            max_dist=0.001,
            name="spun rotor remains centered in the fixed frame",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
