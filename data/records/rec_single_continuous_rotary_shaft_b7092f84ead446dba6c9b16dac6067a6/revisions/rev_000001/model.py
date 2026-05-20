from __future__ import annotations

import math

import cadquery as cq

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
    mesh_from_cadquery,
)


SHAFT_AXIS_Z = 0.280
BASE_THICKNESS = 0.045
BEARING_AXIS_LOCAL_Z = SHAFT_AXIS_Z - BASE_THICKNESS
BEARING_CENTER_X = 0.360
BEARING_LENGTH = 0.125
SHAFT_RADIUS = 0.030


def _make_bearing_housing() -> cq.Workplane:
    """Pillow-block style bearing support with a real clear bore along X."""
    foot = cq.Workplane("XY").box(0.160, 0.280, 0.045).translate((0.0, 0.0, 0.0225))
    web = cq.Workplane("XY").box(0.115, 0.135, 0.190).translate((0.0, 0.0, 0.140))
    ring = (
        cq.Workplane("YZ")
        .center(0.0, BEARING_AXIS_LOCAL_Z)
        .circle(0.084)
        .extrude(BEARING_LENGTH / 2.0, both=True)
    )
    cap = (
        cq.Workplane("XY")
        .box(0.132, 0.165, 0.028)
        .translate((0.0, 0.0, BEARING_AXIS_LOCAL_Z + 0.078))
    )
    bore = (
        cq.Workplane("YZ")
        .center(0.0, BEARING_AXIS_LOCAL_Z)
        .circle(0.046)
        .extrude((BEARING_LENGTH + 0.060) / 2.0, both=True)
    )
    return foot.union(web).union(ring).union(cap).cut(bore)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_rotary_drum_module")

    model.material("painted_frame", rgba=(0.18, 0.22, 0.26, 1.0))
    model.material("frame_edge", rgba=(0.09, 0.10, 0.11, 1.0))
    model.material("rubber", rgba=(0.035, 0.035, 0.035, 1.0))
    model.material("cast_housing", rgba=(0.37, 0.40, 0.42, 1.0))
    model.material("brushed_steel", rgba=(0.72, 0.74, 0.76, 1.0))
    model.material("dark_steel", rgba=(0.18, 0.19, 0.20, 1.0))
    model.material("drum_blue", rgba=(0.05, 0.16, 0.24, 1.0))
    model.material("black_oxide", rgba=(0.02, 0.022, 0.024, 1.0))
    model.material("bearing_bronze", rgba=(0.72, 0.48, 0.22, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((1.050, 0.420, BASE_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS / 2.0)),
        material="painted_frame",
        name="base_plate",
    )
    for index, y in enumerate((-0.165, 0.165)):
        frame.visual(
            Box((0.980, 0.055, 0.052)),
            origin=Origin(xyz=(0.0, y, BASE_THICKNESS + 0.026)),
            material="frame_edge",
            name=f"side_rail_{index}",
        )
    for index, x in enumerate((-0.435, 0.435)):
        frame.visual(
            Box((0.070, 0.360, 0.036)),
            origin=Origin(xyz=(x, 0.0, BASE_THICKNESS + 0.018)),
            material="painted_frame",
            name=f"cross_tie_{index}",
        )
    for xi, x in enumerate((-0.420, 0.420)):
        for yi, y in enumerate((-0.145, 0.145)):
            frame.visual(
                Box((0.155, 0.105, 0.034)),
                origin=Origin(xyz=(x, y, -0.016)),
                material="rubber",
                name=f"foot_{xi}_{yi}",
            )

    bearing_mesh = mesh_from_cadquery(
        _make_bearing_housing(),
        "pillow_block_bearing_housing",
        tolerance=0.0008,
        angular_tolerance=0.06,
    )
    frame.visual(
        bearing_mesh,
        origin=Origin(xyz=(-BEARING_CENTER_X, 0.0, BASE_THICKNESS)),
        material="cast_housing",
        name="bearing_housing_0",
    )
    frame.visual(
        Box((0.100, 0.102, 0.035)),
        origin=Origin(xyz=(-BEARING_CENTER_X, 0.0, SHAFT_AXIS_Z - SHAFT_RADIUS - 0.0175)),
        material="bearing_bronze",
        name="bearing_insert_0",
    )
    frame.visual(
        bearing_mesh,
        origin=Origin(xyz=(BEARING_CENTER_X, 0.0, BASE_THICKNESS)),
        material="cast_housing",
        name="bearing_housing_1",
    )
    frame.visual(
        Box((0.100, 0.102, 0.035)),
        origin=Origin(xyz=(BEARING_CENTER_X, 0.0, SHAFT_AXIS_Z - SHAFT_RADIUS - 0.0175)),
        material="bearing_bronze",
        name="bearing_insert_1",
    )
    for index, x in enumerate((-BEARING_CENTER_X, BEARING_CENTER_X)):
        outer_sign = -1.0 if x < 0.0 else 1.0
        screw_face_x = x + outer_sign * (BEARING_LENGTH / 2.0 + 0.0045)
        for screw_index, (y_off, z_off) in enumerate(
            ((-0.048, -0.045), (0.048, -0.045), (-0.048, 0.045), (0.048, 0.045))
        ):
            frame.visual(
                Cylinder(radius=0.0085, length=0.012),
                origin=Origin(
                    xyz=(screw_face_x, y_off, SHAFT_AXIS_Z + z_off),
                    rpy=(0.0, math.pi / 2.0, 0.0),
                ),
                material="black_oxide",
                name=f"bearing_cap_screw_{index}_{screw_index}",
            )

    frame.inertial = Inertial.from_geometry(
        Box((1.050, 0.420, 0.340)),
        mass=22.0,
        origin=Origin(xyz=(0.0, 0.0, 0.155)),
    )

    rotor = model.part("rotor")
    x_axis = Origin(rpy=(0.0, math.pi / 2.0, 0.0))
    rotor.visual(
        Cylinder(radius=SHAFT_RADIUS, length=0.980),
        origin=x_axis,
        material="brushed_steel",
        name="shaft",
    )
    rotor.visual(
        Cylinder(radius=0.145, length=0.440),
        origin=x_axis,
        material="drum_blue",
        name="drum_barrel",
    )
    for index, x in enumerate((-0.228, 0.228)):
        rotor.visual(
            Cylinder(radius=0.136, length=0.016),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material="dark_steel",
            name=f"end_cover_{index}",
        )
        for screw_index in range(6):
            angle = screw_index * math.tau / 6.0
            rotor.visual(
                Cylinder(radius=0.0065, length=0.006),
                origin=Origin(
                    xyz=(x + (0.010 if x > 0.0 else -0.010), 0.108 * math.cos(angle), 0.108 * math.sin(angle)),
                    rpy=(0.0, math.pi / 2.0, 0.0),
                ),
                material="black_oxide",
                name=f"end_cover_screw_{index}_{screw_index}",
            )
    for index, x in enumerate((-0.244, 0.244)):
        rotor.visual(
            Cylinder(radius=0.074, length=0.044),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material="brushed_steel",
            name=f"hub_transition_{index}",
        )
    rotor.visual(
        Cylinder(radius=0.048, length=0.018),
        origin=Origin(xyz=(-0.268, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="brushed_steel",
        name="inboard_collar_0",
    )
    rotor.visual(
        Cylinder(radius=0.048, length=0.018),
        origin=Origin(xyz=(0.268, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material="brushed_steel",
        name="inboard_collar_1",
    )
    for index, x in enumerate((-0.463, 0.463)):
        rotor.visual(
            Cylinder(radius=0.052, length=0.030),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material="brushed_steel",
            name=f"outboard_collar_{index}",
        )
        rotor.visual(
            Cylinder(radius=0.005, length=0.010),
            origin=Origin(xyz=(x, 0.051, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material="black_oxide",
            name=f"collar_set_screw_{index}",
        )
    for index, x in enumerate((-0.497, 0.497)):
        rotor.visual(
            Cylinder(radius=0.037, length=0.014),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material="dark_steel",
            name=f"shaft_end_cap_{index}",
        )
    for index, x in enumerate((-0.165, 0.165)):
        rotor.visual(
            Cylinder(radius=0.149, length=0.012),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material="black_oxide",
            name=f"machined_band_{index}",
        )

    rotor.inertial = Inertial.from_geometry(
        Cylinder(radius=0.150, length=0.980),
        mass=10.5,
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
    )

    model.articulation(
        "frame_to_rotor",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=rotor,
        origin=Origin(xyz=(0.0, 0.0, SHAFT_AXIS_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=18.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    rotor = object_model.get_part("rotor")
    joint = object_model.get_articulation("frame_to_rotor")

    ctx.check(
        "single continuous rotor stage",
        len(object_model.articulations) == 1 and joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"articulations={object_model.articulations}",
    )
    ctx.expect_overlap(
        rotor,
        frame,
        axes="x",
        elem_a="shaft",
        elem_b="bearing_housing_0",
        min_overlap=0.105,
        name="shaft spans left bearing",
    )
    ctx.expect_overlap(
        rotor,
        frame,
        axes="x",
        elem_a="shaft",
        elem_b="bearing_housing_1",
        min_overlap=0.105,
        name="shaft spans right bearing",
    )
    ctx.expect_within(
        rotor,
        frame,
        axes="yz",
        inner_elem="shaft",
        outer_elem="bearing_housing_0",
        margin=0.0,
        name="shaft centered in left bearing envelope",
    )
    ctx.expect_within(
        rotor,
        frame,
        axes="yz",
        inner_elem="shaft",
        outer_elem="bearing_housing_1",
        margin=0.0,
        name="shaft centered in right bearing envelope",
    )
    ctx.expect_contact(
        rotor,
        frame,
        elem_a="shaft",
        elem_b="bearing_insert_0",
        contact_tol=0.0005,
        name="left bearing insert supports shaft",
    )
    ctx.expect_contact(
        rotor,
        frame,
        elem_a="shaft",
        elem_b="bearing_insert_1",
        contact_tol=0.0005,
        name="right bearing insert supports shaft",
    )
    ctx.expect_gap(
        rotor,
        frame,
        axis="x",
        positive_elem="inboard_collar_0",
        negative_elem="bearing_housing_0",
        min_gap=0.0015,
        name="left collar clears bearing housing",
    )
    ctx.expect_gap(
        frame,
        rotor,
        axis="x",
        positive_elem="bearing_housing_1",
        negative_elem="inboard_collar_1",
        min_gap=0.0015,
        name="right collar clears bearing housing",
    )
    ctx.expect_gap(
        rotor,
        frame,
        axis="x",
        positive_elem="drum_barrel",
        negative_elem="bearing_housing_0",
        min_gap=0.055,
        name="left drum face clear of bearing",
    )
    ctx.expect_gap(
        frame,
        rotor,
        axis="x",
        positive_elem="bearing_housing_1",
        negative_elem="drum_barrel",
        min_gap=0.055,
        name="right drum face clear of bearing",
    )

    rest_position = ctx.part_world_position(rotor)
    with ctx.pose({joint: math.pi / 2.0}):
        turned_position = ctx.part_world_position(rotor)
        ctx.expect_gap(
            frame,
            rotor,
            axis="x",
            positive_elem="bearing_housing_1",
            negative_elem="drum_barrel",
            min_gap=0.055,
            name="rotated drum still clears bearing",
        )
    ctx.check(
        "rotor spins about fixed shaft center",
        rest_position is not None and turned_position is not None and all(
            abs(a - b) < 1e-9 for a, b in zip(rest_position, turned_position)
        ),
        details=f"rest={rest_position}, turned={turned_position}",
    )

    return ctx.report()


object_model = build_object_model()
