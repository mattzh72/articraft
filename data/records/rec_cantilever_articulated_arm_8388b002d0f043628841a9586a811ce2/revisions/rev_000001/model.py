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
    mesh_from_cadquery,
)
import cadquery as cq


def _tapered_rect_beam(length: float, start_y: float, start_z: float, end_y: float, end_z: float):
    """Rectangular tapered beam from local x=0 to x=length."""
    return (
        cq.Workplane("YZ")
        .rect(start_y, start_z)
        .workplane(offset=length)
        .rect(end_y, end_z)
        .loft(combine=True)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_mounted_transfer_arm")

    steel = model.material("powder_coated_steel", rgba=(0.09, 0.11, 0.12, 1.0))
    plate_blue = model.material("blue_grey_structural_links", rgba=(0.20, 0.29, 0.34, 1.0))
    dark_bushing = model.material("black_bushed_knuckles", rgba=(0.015, 0.016, 0.015, 1.0))
    bolt_metal = model.material("brushed_bolt_heads", rgba=(0.62, 0.60, 0.56, 1.0))
    warning = model.material("yellow_load_label", rgba=(0.95, 0.72, 0.08, 1.0))

    # The root frame is the shoulder pivot center.  The wall plate sits behind it
    # on negative X and the arm reaches out along positive X in the rest pose.
    wall_mount = model.part("wall_mount")
    wall_mount.visual(
        Box((0.070, 0.420, 0.720)),
        origin=Origin(xyz=(-0.220, 0.0, -0.060)),
        material=steel,
        name="backplate",
    )
    wall_mount.visual(
        Box((0.070, 0.320, 0.380)),
        origin=Origin(xyz=(-0.145, 0.0, -0.020)),
        material=plate_blue,
        name="raised_spine",
    )
    wall_mount.visual(
        Box((0.230, 0.058, 0.052)),
        origin=Origin(xyz=(-0.075, 0.130, -0.055), rpy=(0.0, 0.45, 0.0)),
        material=plate_blue,
        name="angled_strut_pos",
    )
    wall_mount.visual(
        Box((0.230, 0.058, 0.052)),
        origin=Origin(xyz=(-0.075, -0.130, -0.055), rpy=(0.0, 0.45, 0.0)),
        material=plate_blue,
        name="angled_strut_neg",
    )
    wall_mount.visual(
        Box((0.210, 0.050, 0.042)),
        origin=Origin(xyz=(-0.082, 0.130, 0.070), rpy=(0.0, -0.34, 0.0)),
        material=plate_blue,
        name="upper_strut_pos",
    )
    wall_mount.visual(
        Box((0.210, 0.050, 0.042)),
        origin=Origin(xyz=(-0.082, -0.130, 0.070), rpy=(0.0, -0.34, 0.0)),
        material=plate_blue,
        name="upper_strut_neg",
    )
    wall_mount.visual(
        Box((0.130, 0.060, 0.220)),
        origin=Origin(xyz=(0.0, 0.130, 0.0)),
        material=steel,
        name="shoulder_cheek_pos",
    )
    wall_mount.visual(
        Box((0.130, 0.060, 0.220)),
        origin=Origin(xyz=(0.0, -0.130, 0.0)),
        material=steel,
        name="shoulder_cheek_neg",
    )
    wall_mount.visual(
        Cylinder(radius=0.066, length=0.020),
        origin=Origin(xyz=(0.0, 0.164, 0.0), rpy=(math.pi / 2, 0.0, 0.0)),
        material=dark_bushing,
        name="shoulder_bearing_pos",
    )
    wall_mount.visual(
        Cylinder(radius=0.066, length=0.020),
        origin=Origin(xyz=(0.0, -0.164, 0.0), rpy=(math.pi / 2, 0.0, 0.0)),
        material=dark_bushing,
        name="shoulder_bearing_neg",
    )
    for idx, (y, z) in enumerate(((-0.145, -0.295), (0.145, -0.295), (-0.145, 0.195), (0.145, 0.195))):
        wall_mount.visual(
            Cylinder(radius=0.022, length=0.012),
            origin=Origin(xyz=(-0.179, y, z), rpy=(0.0, math.pi / 2, 0.0)),
            material=bolt_metal,
            name=f"backplate_bolt_{idx}",
        )
    wall_mount.visual(
        Box((0.008, 0.165, 0.038)),
        origin=Origin(xyz=(-0.111, 0.0, 0.115)),
        material=warning,
        name="load_label",
    )

    first_link = model.part("first_link")
    first_link.visual(
        Cylinder(radius=0.055, length=0.200),
        origin=Origin(rpy=(math.pi / 2, 0.0, 0.0)),
        material=dark_bushing,
        name="shoulder_barrel",
    )
    first_link.visual(
        mesh_from_cadquery(
            _tapered_rect_beam(0.680, 0.135, 0.155, 0.104, 0.112),
            "first_link_tapered_beam",
            tolerance=0.0015,
            angular_tolerance=0.08,
        ),
        origin=Origin(xyz=(0.045, 0.0, -0.020)),
        material=plate_blue,
        name="tapered_beam",
    )
    first_link.visual(
        Box((0.560, 0.050, 0.026)),
        origin=Origin(xyz=(0.365, 0.0, 0.058)),
        material=steel,
        name="top_reinforcing_rib",
    )
    first_link.visual(
        Box((0.080, 0.290, 0.180)),
        origin=Origin(xyz=(0.705, 0.0, -0.120)),
        material=steel,
        name="elbow_rear_bridge",
    )
    first_link.visual(
        Box((0.170, 0.055, 0.240)),
        origin=Origin(xyz=(0.820, 0.112, -0.120)),
        material=steel,
        name="elbow_cheek_pos",
    )
    first_link.visual(
        Box((0.170, 0.055, 0.240)),
        origin=Origin(xyz=(0.820, -0.112, -0.120)),
        material=steel,
        name="elbow_cheek_neg",
    )
    first_link.visual(
        Cylinder(radius=0.068, length=0.018),
        origin=Origin(xyz=(0.820, 0.146, -0.120), rpy=(math.pi / 2, 0.0, 0.0)),
        material=dark_bushing,
        name="elbow_cap_pos",
    )
    first_link.visual(
        Cylinder(radius=0.068, length=0.018),
        origin=Origin(xyz=(0.820, -0.146, -0.120), rpy=(math.pi / 2, 0.0, 0.0)),
        material=dark_bushing,
        name="elbow_cap_neg",
    )

    second_link = model.part("second_link")
    second_link.visual(
        Cylinder(radius=0.052, length=0.169),
        origin=Origin(rpy=(math.pi / 2, 0.0, 0.0)),
        material=dark_bushing,
        name="elbow_barrel",
    )
    second_link.visual(
        mesh_from_cadquery(
            _tapered_rect_beam(0.460, 0.108, 0.105, 0.132, 0.122),
            "second_link_tapered_beam",
            tolerance=0.0015,
            angular_tolerance=0.08,
        ),
        origin=Origin(xyz=(0.045, 0.0, 0.012)),
        material=plate_blue,
        name="short_beam",
    )
    second_link.visual(
        Box((0.090, 0.240, 0.120)),
        origin=Origin(xyz=(0.455, 0.0, 0.010)),
        material=steel,
        name="wrist_bridge",
    )
    second_link.visual(
        Box((0.120, 0.045, 0.170)),
        origin=Origin(xyz=(0.560, 0.100, 0.020)),
        material=steel,
        name="wrist_cheek_pos",
    )
    second_link.visual(
        Box((0.120, 0.045, 0.170)),
        origin=Origin(xyz=(0.560, -0.100, 0.020)),
        material=steel,
        name="wrist_cheek_neg",
    )
    second_link.visual(
        Cylinder(radius=0.050, length=0.016),
        origin=Origin(xyz=(0.560, 0.129, 0.020), rpy=(math.pi / 2, 0.0, 0.0)),
        material=dark_bushing,
        name="wrist_cap_pos",
    )
    second_link.visual(
        Cylinder(radius=0.050, length=0.016),
        origin=Origin(xyz=(0.560, -0.129, 0.020), rpy=(math.pi / 2, 0.0, 0.0)),
        material=dark_bushing,
        name="wrist_cap_neg",
    )

    end_plate = model.part("end_plate")
    end_plate.visual(
        Cylinder(radius=0.043, length=0.155),
        origin=Origin(rpy=(math.pi / 2, 0.0, 0.0)),
        material=dark_bushing,
        name="wrist_barrel",
    )
    end_plate.visual(
        Box((0.225, 0.100, 0.100)),
        origin=Origin(xyz=(0.140, 0.0, 0.0)),
        material=dark_bushing,
        name="wrist_neck",
    )
    end_plate.visual(
        Box((0.050, 0.300, 0.300)),
        origin=Origin(xyz=(0.275, 0.0, 0.0)),
        material=steel,
        name="square_plate",
    )
    end_plate.visual(
        Cylinder(radius=0.018, length=0.014),
        origin=Origin(xyz=(0.306, 0.0, 0.0), rpy=(0.0, math.pi / 2, 0.0)),
        material=dark_bushing,
        name="center_boss",
    )
    for idx, (y, z) in enumerate(((-0.095, -0.095), (0.095, -0.095), (-0.095, 0.095), (0.095, 0.095))):
        end_plate.visual(
            Cylinder(radius=0.014, length=0.012),
            origin=Origin(xyz=(0.306, y, z), rpy=(0.0, math.pi / 2, 0.0)),
            material=bolt_metal,
            name=f"plate_bolt_{idx}",
        )

    shoulder = model.articulation(
        "shoulder_pivot",
        ArticulationType.REVOLUTE,
        parent=wall_mount,
        child=first_link,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=450.0, velocity=0.7, lower=-0.65, upper=0.95),
    )
    elbow = model.articulation(
        "elbow_pivot",
        ArticulationType.REVOLUTE,
        parent=first_link,
        child=second_link,
        origin=Origin(xyz=(0.820, 0.0, -0.120)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=330.0, velocity=0.8, lower=-1.15, upper=1.15),
    )
    wrist = model.articulation(
        "wrist_pivot",
        ArticulationType.REVOLUTE,
        parent=second_link,
        child=end_plate,
        origin=Origin(xyz=(0.560, 0.0, 0.020)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.2, lower=-0.75, upper=0.75),
    )

    # Keep local names referenced so linters do not treat the joints as unused;
    # the returned model stores them for the compiler and tests.
    _ = (shoulder, elbow, wrist)
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    wall_mount = object_model.get_part("wall_mount")
    first_link = object_model.get_part("first_link")
    second_link = object_model.get_part("second_link")
    end_plate = object_model.get_part("end_plate")
    shoulder = object_model.get_articulation("shoulder_pivot")
    elbow = object_model.get_articulation("elbow_pivot")
    wrist = object_model.get_articulation("wrist_pivot")

    ctx.check(
        "main pivots are parallel horizontal axes",
        shoulder.axis == elbow.axis == wrist.axis == (0.0, -1.0, 0.0),
        details=f"axes: shoulder={shoulder.axis}, elbow={elbow.axis}, wrist={wrist.axis}",
    )
    ctx.check(
        "elbow package is visibly offset below the first link",
        elbow.origin.xyz[0] > 0.75 and elbow.origin.xyz[2] < -0.08,
        details=f"elbow origin={elbow.origin.xyz}",
    )
    ctx.expect_contact(
        wall_mount,
        first_link,
        contact_tol=0.001,
        elem_a="shoulder_cheek_pos",
        elem_b="shoulder_barrel",
        name="shoulder barrel contacts positive cheek bearing",
    )
    ctx.expect_contact(
        first_link,
        wall_mount,
        contact_tol=0.001,
        elem_a="shoulder_barrel",
        elem_b="shoulder_cheek_neg",
        name="shoulder barrel contacts negative cheek bearing",
    )
    ctx.expect_contact(
        first_link,
        second_link,
        contact_tol=0.001,
        elem_a="elbow_cheek_pos",
        elem_b="elbow_barrel",
        name="elbow barrel contacts positive cheek bearing",
    )
    ctx.expect_contact(
        second_link,
        first_link,
        contact_tol=0.001,
        elem_a="elbow_barrel",
        elem_b="elbow_cheek_neg",
        name="elbow barrel contacts negative cheek bearing",
    )
    ctx.expect_contact(
        second_link,
        end_plate,
        contact_tol=0.001,
        elem_a="wrist_cheek_pos",
        elem_b="wrist_barrel",
        name="wrist barrel contacts positive cheek bearing",
    )
    ctx.expect_contact(
        end_plate,
        second_link,
        contact_tol=0.001,
        elem_a="wrist_barrel",
        elem_b="wrist_cheek_neg",
        name="wrist barrel contacts negative cheek bearing",
    )

    rest_aabb = ctx.part_element_world_aabb(first_link, elem="tapered_beam")
    plate_aabb = ctx.part_element_world_aabb(end_plate, elem="square_plate")
    with ctx.pose({shoulder: 0.60, elbow: -0.45, wrist: 0.55}):
        raised_aabb = ctx.part_element_world_aabb(first_link, elem="tapered_beam")

    ctx.check(
        "positive shoulder motion lifts the first link",
        rest_aabb is not None and raised_aabb is not None and raised_aabb[1][2] > rest_aabb[1][2] + 0.10,
        details=f"rest={rest_aabb}, raised={raised_aabb}",
    )
    ctx.check(
        "end plate remains square and substantial",
        plate_aabb is not None
        and abs((plate_aabb[1][1] - plate_aabb[0][1]) - (plate_aabb[1][2] - plate_aabb[0][2])) < 0.010
        and (plate_aabb[1][1] - plate_aabb[0][1]) > 0.260,
        details=f"plate_aabb={plate_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
