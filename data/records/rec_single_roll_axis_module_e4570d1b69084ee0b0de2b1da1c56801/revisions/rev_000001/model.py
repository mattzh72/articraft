from __future__ import annotations

from math import pi

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
    TorusGeometry,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fork_supported_roll_cartridge")

    painted_steel = model.material("painted_steel", rgba=(0.18, 0.20, 0.22, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.08, 0.09, 0.10, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.62, 0.64, 0.66, 1.0))
    roll_shell = model.material("roll_shell", rgba=(0.88, 0.86, 0.80, 1.0))
    cartridge_blue = model.material("cartridge_blue", rgba=(0.10, 0.34, 0.74, 1.0))

    frame = model.part("fork_frame")
    frame.visual(
        Box((0.52, 0.28, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
        material=painted_steel,
        name="base_plate",
    )

    arm_thickness = 0.055
    arm_gap = 0.360
    arm_center_x = arm_gap * 0.5 + arm_thickness * 0.5
    arm_height = 0.270
    arm_center_z = 0.04 + arm_height * 0.5 - 0.001
    frame.visual(
        Box((arm_thickness, 0.22, arm_height)),
        origin=Origin(xyz=(-arm_center_x, 0.0, arm_center_z)),
        material=painted_steel,
        name="fork_arm_0",
    )
    frame.visual(
        Box((arm_thickness, 0.22, arm_height)),
        origin=Origin(xyz=(arm_center_x, 0.0, arm_center_z)),
        material=painted_steel,
        name="fork_arm_1",
    )

    frame.visual(
        Box((0.47, 0.032, 0.060)),
        origin=Origin(xyz=(0.0, 0.096, 0.265)),
        material=painted_steel,
        name="rear_bridge",
    )

    bearing_mesh = mesh_from_geometry(
        TorusGeometry(radius=0.027, tube=0.006, radial_segments=32, tubular_segments=16).rotate_y(pi / 2.0),
        "bearing_collar",
    )
    frame.visual(
        bearing_mesh,
        origin=Origin(xyz=(-(arm_gap * 0.5), 0.0, 0.180)),
        material=brushed_steel,
        name="bearing_collar_0",
    )
    frame.visual(
        bearing_mesh,
        origin=Origin(xyz=(arm_gap * 0.5, 0.0, 0.180)),
        material=brushed_steel,
        name="bearing_collar_1",
    )

    for index, (x, y) in enumerate(
        ((-0.205, -0.095), (0.205, -0.095), (-0.205, 0.095), (0.205, 0.095))
    ):
        frame.visual(
            Cylinder(radius=0.012, length=0.006),
            origin=Origin(xyz=(x, y, 0.042)),
            material=dark_steel,
            name=f"base_bolt_{index}",
        )

    frame.inertial = Inertial.from_geometry(
        Box((0.52, 0.28, 0.31)),
        mass=8.0,
        origin=Origin(xyz=(0.0, 0.0, 0.155)),
    )

    cartridge = model.part("roll_cartridge")
    axis_origin = Origin(rpy=(0.0, pi / 2.0, 0.0))
    cartridge.visual(
        Cylinder(radius=0.075, length=0.300),
        origin=axis_origin,
        material=roll_shell,
        name="roll_shell",
    )
    cartridge.visual(
        Cylinder(radius=0.0125, length=0.354),
        origin=axis_origin,
        material=brushed_steel,
        name="axle",
    )
    cartridge.visual(
        Cylinder(radius=0.022, length=0.006),
        origin=Origin(xyz=(-0.174, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=brushed_steel,
        name="bearing_journal_0",
    )
    cartridge.visual(
        Cylinder(radius=0.022, length=0.006),
        origin=Origin(xyz=(0.174, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=brushed_steel,
        name="bearing_journal_1",
    )
    for index, x in enumerate((-0.153, 0.153)):
        cartridge.visual(
            Cylinder(radius=0.077, length=0.014),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=cartridge_blue,
            name=f"end_band_{index}",
        )
        cartridge.visual(
            Cylinder(radius=0.033, length=0.018),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=dark_steel,
            name=f"end_hub_{index}",
        )

    cartridge.visual(
        Box((0.235, 0.010, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.077)),
        material=cartridge_blue,
        name="index_stripe",
    )
    cartridge.inertial = Inertial.from_geometry(
        Cylinder(radius=0.078, length=0.354),
        mass=1.5,
        origin=axis_origin,
    )

    model.articulation(
        "roll_spin",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=cartridge,
        origin=Origin(xyz=(0.0, 0.0, 0.180)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=12.0, lower=-pi, upper=pi),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("fork_frame")
    cartridge = object_model.get_part("roll_cartridge")
    joint = object_model.get_articulation("roll_spin")

    ctx.allow_overlap(
        frame,
        cartridge,
        elem_a="bearing_collar_0",
        elem_b="bearing_journal_0",
        reason="The cartridge journal is intentionally seated in the fork bearing collar.",
    )
    ctx.allow_overlap(
        frame,
        cartridge,
        elem_a="bearing_collar_1",
        elem_b="bearing_journal_1",
        reason="The cartridge journal is intentionally seated in the fork bearing collar.",
    )

    ctx.expect_gap(
        frame,
        cartridge,
        axis="x",
        positive_elem="fork_arm_1",
        negative_elem="roll_shell",
        min_gap=0.020,
        name="roll shell clears the positive fork arm",
    )
    ctx.expect_gap(
        cartridge,
        frame,
        axis="x",
        positive_elem="roll_shell",
        negative_elem="fork_arm_0",
        min_gap=0.020,
        name="roll shell clears the negative fork arm",
    )
    ctx.expect_within(
        cartridge,
        frame,
        axes="yz",
        inner_elem="bearing_journal_0",
        outer_elem="bearing_collar_0",
        margin=0.002,
        name="journal is centered in one bearing collar",
    )
    ctx.expect_within(
        cartridge,
        frame,
        axes="yz",
        inner_elem="bearing_journal_1",
        outer_elem="bearing_collar_1",
        margin=0.002,
        name="journal is centered in the other bearing collar",
    )
    ctx.expect_overlap(
        cartridge,
        frame,
        axes="x",
        elem_a="bearing_journal_0",
        elem_b="bearing_collar_0",
        min_overlap=0.002,
        name="journal projects into one collar",
    )
    ctx.expect_overlap(
        cartridge,
        frame,
        axes="x",
        elem_a="bearing_journal_1",
        elem_b="bearing_collar_1",
        min_overlap=0.002,
        name="journal projects into the other collar",
    )

    rest_aabb = ctx.part_element_world_aabb(cartridge, elem="index_stripe")
    with ctx.pose({joint: pi / 2.0}):
        turned_aabb = ctx.part_element_world_aabb(cartridge, elem="index_stripe")
    if rest_aabb is not None and turned_aabb is not None:
        rest_center_z = (rest_aabb[0][2] + rest_aabb[1][2]) * 0.5
        turned_center_z = (turned_aabb[0][2] + turned_aabb[1][2]) * 0.5
        turned_center_y = (turned_aabb[0][1] + turned_aabb[1][1]) * 0.5
        ctx.check(
            "stripe moves around the supported axis",
            rest_center_z > turned_center_z + 0.045 and turned_center_y < -0.045,
            details=(
                f"rest_center_z={rest_center_z:.3f}, "
                f"turned_center_z={turned_center_z:.3f}, turned_center_y={turned_center_y:.3f}"
            ),
        )
    else:
        ctx.fail("stripe moves around the supported axis", "index stripe AABBs were unavailable")

    return ctx.report()


object_model = build_object_model()
