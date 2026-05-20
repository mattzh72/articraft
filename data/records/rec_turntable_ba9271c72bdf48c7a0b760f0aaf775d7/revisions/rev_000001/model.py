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
    model = ArticulatedObject(name="record_turntable")

    wood = model.material("wood", rgba=(0.34, 0.22, 0.14, 1.0))
    satin_black = model.material("satin_black", rgba=(0.08, 0.08, 0.09, 1.0))
    dark_gray = model.material("dark_gray", rgba=(0.20, 0.21, 0.23, 1.0))
    aluminum = model.material("aluminum", rgba=(0.76, 0.77, 0.79, 1.0))
    rubber = model.material("rubber", rgba=(0.10, 0.10, 0.10, 1.0))

    plinth_size = (0.46, 0.36, 0.05)
    top_plate_size = (0.43, 0.33, 0.006)
    platter_center = (-0.04, 0.0)
    tonearm_pivot = (0.145, -0.11)
    plinth_top_z = plinth_size[2] + top_plate_size[2]
    platter_joint_z = 0.068
    tonearm_joint_z = 0.096

    plinth = model.part("plinth")
    plinth.visual(
        Box(plinth_size),
        origin=Origin(xyz=(0.0, 0.0, plinth_size[2] * 0.5)),
        material=wood,
        name="plinth_body",
    )
    plinth.visual(
        Box(top_plate_size),
        origin=Origin(xyz=(0.0, 0.0, plinth_size[2] + top_plate_size[2] * 0.5)),
        material=satin_black,
        name="top_plate",
    )
    plinth.visual(
        Cylinder(radius=0.046, length=0.008),
        origin=Origin(xyz=(platter_center[0], platter_center[1], plinth_top_z - 0.002)),
        material=dark_gray,
        name="bearing_flange",
    )
    plinth.visual(
        Cylinder(radius=0.022, length=platter_joint_z - plinth_top_z),
        origin=Origin(
            xyz=(
                platter_center[0],
                platter_center[1],
                plinth_top_z + (platter_joint_z - plinth_top_z) * 0.5,
            )
        ),
        material=aluminum,
        name="center_support",
    )
    plinth.visual(
        Cylinder(radius=0.032, length=0.006),
        origin=Origin(xyz=(tonearm_pivot[0], tonearm_pivot[1], plinth_top_z - 0.003)),
        material=dark_gray,
        name="tonearm_mount_flange",
    )
    plinth.visual(
        Cylinder(radius=0.017, length=tonearm_joint_z - plinth_top_z),
        origin=Origin(
            xyz=(
                tonearm_pivot[0],
                tonearm_pivot[1],
                plinth_top_z + (tonearm_joint_z - plinth_top_z) * 0.5,
            )
        ),
        material=dark_gray,
        name="tonearm_mount_pedestal",
    )
    plinth.visual(
        Cylinder(radius=0.012, length=0.012),
        origin=Origin(xyz=(-0.175, -0.13, plinth_top_z + 0.006)),
        material=dark_gray,
        name="speed_selector",
    )
    plinth.visual(
        Cylinder(radius=0.005, length=0.006),
        origin=Origin(xyz=(-0.175, -0.13, plinth_top_z + 0.015)),
        material=aluminum,
        name="speed_selector_cap",
    )
    plinth.inertial = Inertial.from_geometry(
        Box((0.46, 0.36, 0.09)),
        mass=7.5,
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
    )

    platter = model.part("platter")
    platter.visual(
        Cylinder(radius=0.15, length=0.017),
        origin=Origin(xyz=(0.0, 0.0, 0.0085)),
        material=aluminum,
        name="platter_body",
    )
    platter.visual(
        Cylinder(radius=0.145, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, 0.0185)),
        material=rubber,
        name="record_mat",
    )
    platter.visual(
        Cylinder(radius=0.003, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.026)),
        material=aluminum,
        name="center_spindle",
    )
    platter.inertial = Inertial.from_geometry(
        Cylinder(radius=0.15, length=0.02),
        mass=1.9,
        origin=Origin(xyz=(0.0, 0.0, 0.01)),
    )

    tonearm = model.part("tonearm")
    tonearm.visual(
        Cylinder(radius=0.012, length=0.022),
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
        material=dark_gray,
        name="pivot_housing",
    )
    tonearm.visual(
        Cylinder(radius=0.0045, length=0.205),
        origin=Origin(xyz=(0.0, 0.113, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=aluminum,
        name="arm_tube",
    )
    tonearm.visual(
        Box((0.016, 0.028, 0.004)),
        origin=Origin(xyz=(0.0, 0.223, -0.001)),
        material=dark_gray,
        name="headshell",
    )
    tonearm.visual(
        Box((0.008, 0.012, 0.005)),
        origin=Origin(xyz=(0.0, 0.238, -0.004)),
        material=satin_black,
        name="cartridge_body",
    )
    tonearm.visual(
        Cylinder(radius=0.003, length=0.040),
        origin=Origin(xyz=(0.0, -0.02, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=aluminum,
        name="counterweight_stub",
    )
    tonearm.visual(
        Cylinder(radius=0.0055, length=0.010),
        origin=Origin(xyz=(0.0, -0.031, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_gray,
        name="counterweight_collar",
    )
    tonearm.visual(
        Cylinder(radius=0.011, length=0.022),
        origin=Origin(xyz=(0.0, -0.045, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_gray,
        name="counterweight",
    )
    tonearm.inertial = Inertial.from_geometry(
        Box((0.04, 0.30, 0.03)),
        mass=0.35,
        origin=Origin(xyz=(0.0, 0.09, 0.004)),
    )

    model.articulation(
        "platter_spin",
        ArticulationType.CONTINUOUS,
        parent=plinth,
        child=platter,
        origin=Origin(xyz=(platter_center[0], platter_center[1], platter_joint_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.8, velocity=12.0),
    )
    model.articulation(
        "tonearm_swing",
        ArticulationType.REVOLUTE,
        parent=plinth,
        child=tonearm,
        origin=Origin(xyz=(tonearm_pivot[0], tonearm_pivot[1], tonearm_joint_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=1.0,
            lower=0.0,
            upper=0.95,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    plinth = object_model.get_part("plinth")
    platter = object_model.get_part("platter")
    tonearm = object_model.get_part("tonearm")
    platter_spin = object_model.get_articulation("platter_spin")
    tonearm_swing = object_model.get_articulation("tonearm_swing")

    def aabb_center(aabb):
        return tuple((lo + hi) * 0.5 for lo, hi in zip(aabb[0], aabb[1]))

    with ctx.pose({platter_spin: 0.0, tonearm_swing: 0.0}):
        ctx.expect_gap(
            platter,
            plinth,
            axis="z",
            positive_elem="platter_body",
            negative_elem="center_support",
            max_gap=0.001,
            max_penetration=0.0,
            name="platter rests on a single central support",
        )
        ctx.expect_gap(
            tonearm,
            plinth,
            axis="z",
            positive_elem="arm_tube",
            negative_elem="top_plate",
            min_gap=0.03,
            name="tonearm clears the plinth surface",
        )
        parked_cartridge = ctx.part_element_world_aabb(tonearm, elem="cartridge_body")

    with ctx.pose({tonearm_swing: 0.65}):
        ctx.expect_overlap(
            tonearm,
            platter,
            axes="xy",
            elem_a="cartridge_body",
            elem_b="record_mat",
            min_overlap=0.006,
            name="tonearm swings the cartridge over the record area",
        )
        swung_cartridge = ctx.part_element_world_aabb(tonearm, elem="cartridge_body")

    ctx.check(
        "tonearm swings inward with positive motion",
        parked_cartridge is not None
        and swung_cartridge is not None
        and aabb_center(swung_cartridge)[0] < aabb_center(parked_cartridge)[0] - 0.10,
        details=f"parked={parked_cartridge}, swung={swung_cartridge}",
    )

    with ctx.pose({platter_spin: 1.7}):
        ctx.expect_gap(
            platter,
            plinth,
            axis="z",
            positive_elem="platter_body",
            negative_elem="center_support",
            max_gap=0.001,
            max_penetration=0.0,
            name="platter remains seated while spinning",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
