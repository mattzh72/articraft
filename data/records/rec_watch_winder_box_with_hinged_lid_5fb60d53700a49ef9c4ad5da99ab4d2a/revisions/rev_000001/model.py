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
    model = ArticulatedObject(name="watch_winder_presentation_box")

    walnut = model.material("walnut", rgba=(0.32, 0.22, 0.13, 1.0))
    brass = model.material("brass", rgba=(0.73, 0.62, 0.36, 1.0))
    lining = model.material("lining", rgba=(0.44, 0.38, 0.31, 1.0))
    cushion = model.material("cushion", rgba=(0.63, 0.57, 0.50, 1.0))
    hardware_dark = model.material("hardware_dark", rgba=(0.16, 0.16, 0.18, 1.0))
    glass = model.material("glass", rgba=(0.78, 0.84, 0.90, 0.35))

    base_w = 0.24
    base_d = 0.18
    floor_t = 0.016
    wall_t = 0.016
    base_h = 0.108

    lid_w = 0.248
    lid_d = 0.188
    lid_h = 0.078
    lid_t = 0.012

    hinge_y = -(base_d * 0.5) - 0.014
    hinge_z = base_h + 0.010

    body = model.part("body")
    body.visual(
        Box((base_w, base_d, floor_t)),
        origin=Origin(xyz=(0.0, 0.0, floor_t * 0.5)),
        material=walnut,
        name="base_floor",
    )
    body.visual(
        Box((base_w, wall_t, base_h)),
        origin=Origin(xyz=(0.0, (base_d * 0.5) - (wall_t * 0.5), base_h * 0.5)),
        material=walnut,
        name="front_wall",
    )
    body.visual(
        Box((base_w, wall_t, base_h)),
        origin=Origin(xyz=(0.0, -(base_d * 0.5) + (wall_t * 0.5), base_h * 0.5)),
        material=walnut,
        name="back_wall",
    )
    body.visual(
        Box((wall_t, base_d - (2.0 * wall_t), base_h)),
        origin=Origin(xyz=((base_w * 0.5) - (wall_t * 0.5), 0.0, base_h * 0.5)),
        material=walnut,
        name="right_wall",
    )
    body.visual(
        Box((wall_t, base_d - (2.0 * wall_t), base_h)),
        origin=Origin(xyz=(-(base_w * 0.5) + (wall_t * 0.5), 0.0, base_h * 0.5)),
        material=walnut,
        name="left_wall",
    )
    body.visual(
        Box((base_w - 0.028, base_d - 0.028, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, floor_t + 0.003)),
        material=lining,
        name="liner_floor",
    )
    body.visual(
        Cylinder(radius=0.006, length=0.020),
        origin=Origin(
            xyz=(0.0, -0.064, 0.063),
            rpy=(-math.pi * 0.5, 0.0, 0.0),
        ),
        material=brass,
        name="drive_spindle",
    )
    body.visual(
        Box((0.088, 0.018, 0.022)),
        origin=Origin(xyz=(0.0, hinge_y + 0.005, 0.101)),
        material=brass,
        name="body_hinge_leaf",
    )
    body.visual(
        Cylinder(radius=0.006, length=0.112),
        origin=Origin(
            xyz=(0.0, hinge_y + 0.002, hinge_z),
            rpy=(0.0, math.pi * 0.5, 0.0),
        ),
        material=brass,
        name="body_hinge_barrel",
    )
    body.inertial = Inertial.from_geometry(
        Box((base_w, base_d, base_h)),
        mass=2.8,
        origin=Origin(xyz=(0.0, 0.0, base_h * 0.5)),
    )

    lid = model.part("lid")
    lid.visual(
        Box((lid_w, lid_d, lid_t)),
        origin=Origin(xyz=(0.0, lid_d * 0.5, lid_h - (lid_t * 0.5) - 0.010)),
        material=walnut,
        name="lid_top",
    )
    lid.visual(
        Box((lid_w - 0.058, lid_d - 0.064, 0.003)),
        origin=Origin(xyz=(0.0, 0.100, lid_h - 0.0175)),
        material=glass,
        name="glass_panel",
    )
    lid.visual(
        Box((lid_t, lid_d, lid_h)),
        origin=Origin(xyz=((lid_w * 0.5) - (lid_t * 0.5), lid_d * 0.5, (lid_h * 0.5) - 0.010)),
        material=walnut,
        name="lid_right_rail",
    )
    lid.visual(
        Box((lid_t, lid_d, lid_h)),
        origin=Origin(xyz=(-(lid_w * 0.5) + (lid_t * 0.5), lid_d * 0.5, (lid_h * 0.5) - 0.010)),
        material=walnut,
        name="lid_left_rail",
    )
    lid.visual(
        Box((lid_w, lid_t, lid_h)),
        origin=Origin(xyz=(0.0, lid_d - (lid_t * 0.5), (lid_h * 0.5) - 0.010)),
        material=walnut,
        name="lid_front_rail",
    )
    for x_barrel in (-0.088, 0.088):
        lid.visual(
            Cylinder(radius=0.006, length=0.048),
            origin=Origin(
                xyz=(x_barrel, 0.0, 0.0),
                rpy=(0.0, math.pi * 0.5, 0.0),
            ),
            material=brass,
            name=f"lid_hinge_barrel_{'left' if x_barrel < 0.0 else 'right'}",
        )
    lid.inertial = Inertial.from_geometry(
        Box((lid_w, lid_d, lid_h)),
        mass=1.1,
        origin=Origin(xyz=(0.0, lid_d * 0.5, (lid_h * 0.5) - 0.010)),
    )

    cradle = model.part("cradle")
    cradle.visual(
        Cylinder(radius=0.016, length=0.014),
        origin=Origin(
            xyz=(0.0, 0.007, 0.0),
            rpy=(-math.pi * 0.5, 0.0, 0.0),
        ),
        material=hardware_dark,
        name="cradle_hub",
    )
    cradle.visual(
        Cylinder(radius=0.038, length=0.006),
        origin=Origin(
            xyz=(0.0, 0.017, 0.0),
            rpy=(-math.pi * 0.5, 0.0, 0.0),
        ),
        material=hardware_dark,
        name="backplate",
    )
    cradle.visual(
        Box((0.026, 0.014, 0.034)),
        origin=Origin(xyz=(0.0, 0.026, 0.0)),
        material=hardware_dark,
        name="cradle_mount",
    )
    cradle.visual(
        Box((0.046, 0.034, 0.056)),
        origin=Origin(xyz=(0.0, 0.044, 0.0)),
        material=cushion,
        name="cushion_core",
    )
    cradle.visual(
        Cylinder(radius=0.016, length=0.056),
        origin=Origin(xyz=(-0.024, 0.044, 0.0)),
        material=cushion,
        name="cushion_bolster_left",
    )
    cradle.visual(
        Cylinder(radius=0.016, length=0.056),
        origin=Origin(xyz=(0.024, 0.044, 0.0)),
        material=cushion,
        name="cushion_bolster_right",
    )
    cradle.inertial = Inertial.from_geometry(
        Box((0.080, 0.064, 0.076)),
        mass=0.35,
        origin=Origin(xyz=(0.0, 0.038, 0.0)),
    )

    model.articulation(
        "body_to_lid",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, hinge_y, hinge_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.5,
            lower=0.0,
            upper=1.35,
        ),
    )
    model.articulation(
        "body_to_cradle",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=cradle,
        origin=Origin(xyz=(0.0, -0.054, 0.063)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.5,
            velocity=4.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    cradle = object_model.get_part("cradle")
    lid_hinge = object_model.get_articulation("body_to_lid")
    cradle_spin = object_model.get_articulation("body_to_cradle")

    with ctx.pose({lid_hinge: 0.0}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="lid_front_rail",
            negative_elem="front_wall",
            max_gap=0.001,
            max_penetration=1e-6,
            name="closed lid front rail seats on the box rim",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="x",
            elem_a="lid_front_rail",
            elem_b="front_wall",
            min_overlap=0.20,
            name="closed lid spans the presentation opening",
        )
        closed_front_rail = ctx.part_element_world_aabb(lid, elem="lid_front_rail")

    with ctx.pose({lid_hinge: 1.2}):
        opened_front_rail = ctx.part_element_world_aabb(lid, elem="lid_front_rail")

    ctx.check(
        "lid opens upward from the rear hinge",
        closed_front_rail is not None
        and opened_front_rail is not None
        and opened_front_rail[0][2] > closed_front_rail[1][2] + 0.08
        and opened_front_rail[1][1] < 0.0,
        details=f"closed_front_rail={closed_front_rail}, opened_front_rail={opened_front_rail}",
    )

    with ctx.pose({cradle_spin: 0.0}):
        ctx.expect_gap(
            cradle,
            body,
            axis="y",
            positive_elem="cradle_hub",
            negative_elem="drive_spindle",
            max_gap=0.0005,
            max_penetration=0.0,
            name="cradle hub sits against the drive spindle",
        )
        rest_bolster = ctx.part_element_world_aabb(cradle, elem="cushion_bolster_right")

    with ctx.pose({cradle_spin: math.pi * 0.5, lid_hinge: 0.0}):
        ctx.expect_gap(
            cradle,
            body,
            axis="z",
            positive_elem="cushion_bolster_right",
            negative_elem="liner_floor",
            min_gap=0.0008,
            name="rotated cradle clears the lined floor",
        )
        ctx.expect_gap(
            lid,
            cradle,
            axis="z",
            min_gap=0.004,
            name="rotated cradle still clears the closed lid",
        )
        turned_bolster = ctx.part_element_world_aabb(cradle, elem="cushion_bolster_right")

    ctx.check(
        "cradle rotates about the spindle",
        rest_bolster is not None
        and turned_bolster is not None
        and turned_bolster[0][0] < rest_bolster[0][0] - 0.02,
        details=f"rest_bolster={rest_bolster}, turned_bolster={turned_bolster}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
