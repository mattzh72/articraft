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
    TorusGeometry,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="open_frame_rotary_module")

    dark_iron = model.material("dark_iron", rgba=(0.10, 0.11, 0.12, 1.0))
    machined = model.material("machined_steel", rgba=(0.70, 0.72, 0.70, 1.0))
    bearing_bronze = model.material("bearing_bronze", rgba=(0.72, 0.50, 0.22, 1.0))
    fixture_blue = model.material("fixture_blue", rgba=(0.05, 0.22, 0.45, 1.0))
    bolt_black = model.material("bolt_black", rgba=(0.02, 0.02, 0.02, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((0.58, 0.42, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, 0.02)),
        material=dark_iron,
        name="base_plate",
    )

    # Two tall side bearing supports leave the center open so the vertical
    # rotary shaft and axis line are visually obvious.
    for side_name, y in (("support_0", 0.18), ("support_1", -0.18)):
        frame.visual(
            Box((0.085, 0.060, 0.52)),
            origin=Origin(xyz=(0.0, y, 0.29)),
            material=dark_iron,
            name=side_name,
        )
        frame.visual(
            Box((0.18, 0.095, 0.035)),
            origin=Origin(xyz=(0.0, y, 0.057)),
            material=dark_iron,
            name=f"{side_name}_foot",
        )
        frame.visual(
            Box((0.11, 0.078, 0.025)),
            origin=Origin(xyz=(0.0, y, 0.562)),
            material=bolt_black,
            name=f"{side_name}_cap",
        )

    # Annular bearing collars are true torus meshes, not solid cylinders, so
    # the rotating shaft visibly passes through clear central holes.
    frame.visual(
        mesh_from_geometry(TorusGeometry(0.058, 0.013, radial_segments=40, tubular_segments=14), "lower_bearing"),
        origin=Origin(xyz=(0.0, 0.0, 0.220)),
        material=bearing_bronze,
        name="lower_bearing",
    )
    frame.visual(
        Box((0.080, 0.100, 0.030)),
        origin=Origin(xyz=(0.0, 0.110, 0.220)),
        material=dark_iron,
        name="lower_bearing_arm_0",
    )
    frame.visual(
        Box((0.080, 0.100, 0.030)),
        origin=Origin(xyz=(0.0, -0.110, 0.220)),
        material=dark_iron,
        name="lower_bearing_arm_1",
    )
    frame.visual(
        Box((0.015, 0.026, 0.045)),
        origin=Origin(xyz=(0.0385, 0.0, 0.220)),
        material=bearing_bronze,
        name="lower_bearing_pad_0",
    )
    frame.visual(
        Box((0.015, 0.026, 0.045)),
        origin=Origin(xyz=(-0.0385, 0.0, 0.220)),
        material=bearing_bronze,
        name="lower_bearing_pad_1",
    )
    frame.visual(
        mesh_from_geometry(TorusGeometry(0.058, 0.013, radial_segments=40, tubular_segments=14), "upper_bearing"),
        origin=Origin(xyz=(0.0, 0.0, 0.520)),
        material=bearing_bronze,
        name="upper_bearing",
    )
    frame.visual(
        Box((0.080, 0.100, 0.030)),
        origin=Origin(xyz=(0.0, 0.110, 0.520)),
        material=dark_iron,
        name="upper_bearing_arm_0",
    )
    frame.visual(
        Box((0.080, 0.100, 0.030)),
        origin=Origin(xyz=(0.0, -0.110, 0.520)),
        material=dark_iron,
        name="upper_bearing_arm_1",
    )
    frame.visual(
        Box((0.015, 0.026, 0.045)),
        origin=Origin(xyz=(0.0385, 0.0, 0.520)),
        material=bearing_bronze,
        name="upper_bearing_pad_0",
    )
    frame.visual(
        Box((0.015, 0.026, 0.045)),
        origin=Origin(xyz=(-0.0385, 0.0, 0.520)),
        material=bearing_bronze,
        name="upper_bearing_pad_1",
    )

    for i, (x, y) in enumerate(
        (
            (-0.22, -0.15),
            (-0.22, 0.15),
            (0.22, -0.15),
            (0.22, 0.15),
            (-0.055, -0.18),
            (0.055, -0.18),
            (-0.055, 0.18),
            (0.055, 0.18),
        )
    ):
        frame.visual(
            Cylinder(radius=0.012, length=0.010),
            origin=Origin(xyz=(x, y, 0.045)),
            material=bolt_black,
            name=f"bolt_{i}",
        )

    rotor = model.part("rotor")
    # The child frame is on the lower bearing centerline.  All rotor geometry is
    # offset from that frame so it rotates about the vertical column axis.
    rotor.visual(
        Cylinder(radius=0.031, length=0.55),
        origin=Origin(xyz=(0.0, 0.0, 0.165)),
        material=machined,
        name="shaft",
    )
    rotor.visual(
        Cylinder(radius=0.052, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, -0.055)),
        material=machined,
        name="lower_cap",
    )
    rotor.visual(
        Cylinder(radius=0.058, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.380)),
        material=machined,
        name="top_hub",
    )
    rotor.visual(
        Cylinder(radius=0.105, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.417)),
        material=fixture_blue,
        name="faceplate",
    )
    rotor.visual(
        Box((0.290, 0.075, 0.032)),
        origin=Origin(xyz=(0.105, 0.0, 0.445)),
        material=fixture_blue,
        name="tool_arm",
    )
    rotor.visual(
        Box((0.075, 0.115, 0.080)),
        origin=Origin(xyz=(0.270, 0.0, 0.450)),
        material=fixture_blue,
        name="tool_block",
    )
    rotor.visual(
        Box((0.120, 0.095, 0.035)),
        origin=Origin(xyz=(-0.095, 0.0, 0.450)),
        material=machined,
        name="counterweight",
    )
    for i, y in enumerate((-0.030, 0.030)):
        rotor.visual(
            Cylinder(radius=0.010, length=0.012),
            origin=Origin(xyz=(0.270, y, 0.496)),
            material=bolt_black,
            name=f"clamp_screw_{i}",
        )

    model.articulation(
        "frame_to_rotor",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=rotor,
        origin=Origin(xyz=(0.0, 0.0, 0.220)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=45.0,
            velocity=2.0,
            lower=-math.pi,
            upper=math.pi,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    rotor = object_model.get_part("rotor")
    joint = object_model.get_articulation("frame_to_rotor")

    ctx.check(
        "single vertical revolute joint",
        len(object_model.articulations) == 1
        and joint.articulation_type == ArticulationType.REVOLUTE
        and tuple(joint.axis) == (0.0, 0.0, 1.0),
        details=f"articulations={object_model.articulations}",
    )

    # The shaft is radially centered in both annular bearing collars while the
    # larger caps/hub sit outside the bearing stack with visible axial clearance.
    ctx.expect_within(
        rotor,
        frame,
        axes="xy",
        inner_elem="shaft",
        outer_elem="upper_bearing",
        margin=0.0,
        name="shaft runs through upper bearing center",
    )
    ctx.expect_within(
        rotor,
        frame,
        axes="xy",
        inner_elem="shaft",
        outer_elem="lower_bearing",
        margin=0.0,
        name="shaft runs through lower bearing center",
    )
    ctx.expect_gap(
        rotor,
        frame,
        axis="z",
        positive_elem="top_hub",
        negative_elem="upper_bearing",
        min_gap=0.020,
        max_gap=0.060,
        name="top hub clears upper bearing",
    )
    ctx.expect_gap(
        frame,
        rotor,
        axis="z",
        positive_elem="lower_bearing",
        negative_elem="lower_cap",
        min_gap=0.020,
        max_gap=0.060,
        name="lower cap clears lower bearing",
    )
    ctx.expect_contact(
        rotor,
        frame,
        elem_a="shaft",
        elem_b="lower_bearing_pad_0",
        contact_tol=0.0005,
        name="shaft seats against lower bearing pad",
    )
    ctx.expect_contact(
        rotor,
        frame,
        elem_a="shaft",
        elem_b="upper_bearing_pad_0",
        contact_tol=0.0005,
        name="shaft seats against upper bearing pad",
    )

    with ctx.pose({"frame_to_rotor": 0.0}):
        rest_aabb = ctx.part_element_world_aabb(rotor, elem="tool_block")
    with ctx.pose({"frame_to_rotor": math.pi / 2.0}):
        turned_aabb = ctx.part_element_world_aabb(rotor, elem="tool_block")

    if rest_aabb is not None and turned_aabb is not None:
        rest_center = tuple((rest_aabb[0][i] + rest_aabb[1][i]) * 0.5 for i in range(3))
        turned_center = tuple((turned_aabb[0][i] + turned_aabb[1][i]) * 0.5 for i in range(3))
    else:
        rest_center = None
        turned_center = None
    ctx.check(
        "tool block sweeps about vertical axis",
        rest_center is not None
        and turned_center is not None
        and rest_center[0] > 0.23
        and abs(rest_center[1]) < 0.02
        and abs(turned_center[0]) < 0.02
        and turned_center[1] > 0.23,
        details=f"rest={rest_center}, turned={turned_center}",
    )

    return ctx.report()


object_model = build_object_model()
