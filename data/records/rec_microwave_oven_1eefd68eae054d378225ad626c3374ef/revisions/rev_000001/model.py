from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    KnobSkirt,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="countertop_microwave")

    cream = model.material("warm_white_enamel", rgba=(0.86, 0.84, 0.78, 1.0))
    dark = model.material("dark_tinted_glass", rgba=(0.03, 0.035, 0.04, 0.55))
    black = model.material("black_plastic", rgba=(0.01, 0.01, 0.012, 1.0))
    liner = model.material("dark_cavity_liner", rgba=(0.16, 0.16, 0.15, 1.0))
    steel = model.material("brushed_steel", rgba=(0.62, 0.62, 0.58, 1.0))
    glass = model.material("pale_glass", rgba=(0.72, 0.88, 0.96, 0.35))
    mark = model.material("white_print", rgba=(0.92, 0.92, 0.86, 1.0))

    width = 0.56
    depth = 0.42
    height = 0.32
    front_y = -depth / 2.0

    cavity_x_min = -0.235
    cavity_x_max = 0.135
    cavity_z_min = 0.055
    cavity_z_max = 0.262
    cavity_back_y = 0.185

    housing = model.part("housing")
    # Connected wall slabs leave a real open chamber volume while retaining the
    # rectangular outer appliance silhouette.
    housing.visual(
        Box((width, depth, cavity_z_min)),
        origin=Origin(xyz=(0.0, 0.0, cavity_z_min / 2.0)),
        material=cream,
        name="base_wall",
    )
    housing.visual(
        Box((width, depth, height - cavity_z_max)),
        origin=Origin(xyz=(0.0, 0.0, (height + cavity_z_max) / 2.0)),
        material=cream,
        name="top_wall",
    )
    housing.visual(
        Box((cavity_x_min + width / 2.0, depth, cavity_z_max - cavity_z_min)),
        origin=Origin(xyz=((-width / 2.0 + cavity_x_min) / 2.0, 0.0, (cavity_z_min + cavity_z_max) / 2.0)),
        material=cream,
        name="side_wall",
    )
    housing.visual(
        Box((cavity_x_max + width / 2.0, depth - cavity_back_y + front_y, cavity_z_max - cavity_z_min)),
        origin=Origin(xyz=((-width / 2.0 + cavity_x_max) / 2.0, (cavity_back_y + depth / 2.0) / 2.0, (cavity_z_min + cavity_z_max) / 2.0)),
        material=cream,
        name="rear_wall",
    )
    housing.visual(
        Box((width / 2.0 - cavity_x_max, depth, cavity_z_max - cavity_z_min)),
        origin=Origin(xyz=((cavity_x_max + width / 2.0) / 2.0, 0.0, (cavity_z_min + cavity_z_max) / 2.0)),
        material=cream,
        name="control_column",
    )

    # Dark liner panels sit exactly inside the chamber opening and make the
    # shell read as a hollow cooking cavity rather than a plain rectangular box.
    cavity_w = cavity_x_max - cavity_x_min
    cavity_h = cavity_z_max - cavity_z_min
    cavity_d = cavity_back_y - front_y
    cavity_cx = (cavity_x_min + cavity_x_max) / 2.0
    cavity_cy = (front_y + cavity_back_y) / 2.0
    cavity_cz = (cavity_z_min + cavity_z_max) / 2.0
    housing.visual(
        Box((cavity_w, cavity_d, 0.004)),
        origin=Origin(xyz=(cavity_cx, cavity_cy, cavity_z_min + 0.002)),
        material=liner,
        name="cavity_floor",
    )
    housing.visual(
        Box((cavity_w, 0.004, cavity_h)),
        origin=Origin(xyz=(cavity_cx, cavity_back_y - 0.002, cavity_cz)),
        material=liner,
        name="cavity_back",
    )
    housing.visual(
        Box((0.004, cavity_d, cavity_h)),
        origin=Origin(xyz=(cavity_x_min + 0.002, cavity_cy, cavity_cz)),
        material=liner,
        name="cavity_side",
    )
    housing.visual(
        Box((cavity_w, cavity_d, 0.004)),
        origin=Origin(xyz=(cavity_cx, cavity_cy, cavity_z_max - 0.002)),
        material=liner,
        name="cavity_ceiling",
    )

    # Right-side front control panel and printed dial legends.
    housing.visual(
        Box((0.122, 0.004, 0.258)),
        origin=Origin(xyz=(0.209, front_y - 0.002, 0.160)),
        material=black,
        name="control_panel",
    )
    for z in (0.215, 0.115):
        for angle in (-110, -70, -30, 10, 50, 90, 130):
            radius = 0.044
            a = math.radians(angle)
            x = 0.209 + radius * math.sin(a)
            zz = z + radius * math.cos(a)
            housing.visual(
                Box((0.003, 0.002, 0.010)),
                origin=Origin(
                    xyz=(x, front_y - 0.005, zz),
                    rpy=(0.0, 0.0, -a),
                ),
                material=mark,
                name=f"dial_tick_{z:.3f}_{angle}",
            )

    # A small metal hinge leaf on the case shows where the vertical hinge is
    # physically supported.
    housing.visual(
        Box((0.014, 0.005, 0.232)),
        origin=Origin(xyz=(-0.258, front_y - 0.0025, 0.160)),
        material=steel,
        name="hinge_leaf",
    )

    door = model.part("door")
    door_width = 0.392
    door_height = 0.240
    door_thick = 0.034
    door_shape = (
        cq.Workplane("XY")
        .box(door_width, door_thick, door_height)
        .translate((door_width / 2.0, 0.0, door_height / 2.0))
        .cut(
            cq.Workplane("XY")
            .box(0.245, door_thick + 0.004, 0.132)
            .translate((0.190, 0.0, 0.132))
        )
    )
    door.visual(
        mesh_from_cadquery(door_shape, "door_frame", tolerance=0.0008),
        material=black,
        name="door_frame",
    )
    door.visual(
        Box((0.265, 0.006, 0.154)),
        origin=Origin(xyz=(0.190, 0.000, 0.132)),
        material=dark,
        name="window_glass",
    )
    for z in (0.090, 0.116, 0.142, 0.168):
        door.visual(
            Box((0.238, 0.002, 0.003)),
            origin=Origin(xyz=(0.190, -0.004, z)),
            material=steel,
            name=f"window_mesh_{z:.3f}",
        )
    for x in (0.110, 0.160, 0.210, 0.260):
        door.visual(
            Box((0.003, 0.002, 0.126)),
            origin=Origin(xyz=(x, -0.004, 0.132)),
            material=steel,
            name=f"window_mesh_{x:.3f}",
        )
    door.visual(
        Cylinder(radius=0.010, length=0.162),
        origin=Origin(xyz=(0.355, -0.046, 0.140)),
        material=steel,
        name="pull_handle",
    )
    door.visual(
        Box((0.024, 0.038, 0.022)),
        origin=Origin(xyz=(0.355, -0.030, 0.070)),
        material=steel,
        name="handle_mount_0",
    )
    door.visual(
        Box((0.024, 0.038, 0.022)),
        origin=Origin(xyz=(0.355, -0.030, 0.210)),
        material=steel,
        name="handle_mount_1",
    )
    door.visual(
        Cylinder(radius=0.012, length=0.220),
        origin=Origin(xyz=(0.0, 0.0, 0.120)),
        material=steel,
        name="hinge_barrel",
    )

    hinge_x = cavity_x_min - 0.004
    door_joint_y = front_y - door_thick / 2.0
    model.articulation(
        "housing_to_door",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=door,
        origin=Origin(xyz=(hinge_x, door_joint_y, 0.040)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=0.0, upper=1.75),
    )

    knob_shape = KnobGeometry(
        0.062,
        0.030,
        body_style="skirted",
        top_diameter=0.050,
        skirt=KnobSkirt(0.069, 0.006, flare=0.05, chamfer=0.001),
        grip=KnobGrip(style="fluted", count=20, depth=0.0012),
        indicator=KnobIndicator(style="line", mode="engraved", depth=0.0007),
        center=False,
    )
    for name, z in (("time_dial", 0.215), ("power_dial", 0.115)):
        dial = model.part(name)
        dial.visual(
            mesh_from_geometry(knob_shape, name),
            material=steel,
            name="knob_cap",
        )
        model.articulation(
            f"panel_to_{name}",
            ArticulationType.REVOLUTE,
            parent=housing,
            child=dial,
            origin=Origin(xyz=(0.209, front_y, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=1.2, velocity=6.0, lower=-2.35, upper=2.35),
        )

    support_ring = model.part("support_ring")
    ring_shape = (
        cq.Workplane("XY")
        .circle(0.124)
        .circle(0.110)
        .extrude(0.012)
        .translate((0.0, 0.0, -0.006))
    )
    support_ring.visual(
        mesh_from_cadquery(ring_shape, "support_ring", tolerance=0.0008),
        material=steel,
        name="roller_ring",
    )
    model.articulation(
        "housing_to_ring",
        ArticulationType.FIXED,
        parent=housing,
        child=support_ring,
        origin=Origin(xyz=(cavity_cx, -0.010, cavity_z_min + 0.010)),
    )

    turntable = model.part("turntable")
    turntable.visual(
        Cylinder(radius=0.148, length=0.008),
        origin=Origin(),
        material=glass,
        name="glass_plate",
    )
    turntable.visual(
        Cylinder(radius=0.024, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, -0.009)),
        material=steel,
        name="center_hub",
    )
    model.articulation(
        "ring_to_turntable",
        ArticulationType.CONTINUOUS,
        parent=support_ring,
        child=turntable,
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1.5, velocity=4.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    door = object_model.get_part("door")
    support_ring = object_model.get_part("support_ring")
    turntable = object_model.get_part("turntable")
    time_dial = object_model.get_part("time_dial")
    power_dial = object_model.get_part("power_dial")

    door_hinge = object_model.get_articulation("housing_to_door")
    time_joint = object_model.get_articulation("panel_to_time_dial")
    power_joint = object_model.get_articulation("panel_to_power_dial")
    turntable_joint = object_model.get_articulation("ring_to_turntable")

    ctx.check(
        "door has vertical revolute hinge",
        door_hinge.articulation_type == ArticulationType.REVOLUTE
        and tuple(door_hinge.axis) == (0.0, 0.0, -1.0)
        and door_hinge.motion_limits is not None
        and door_hinge.motion_limits.upper is not None
        and door_hinge.motion_limits.upper > 1.5,
        details=f"type={door_hinge.articulation_type}, axis={door_hinge.axis}, limits={door_hinge.motion_limits}",
    )

    rest_door_aabb = ctx.part_world_aabb(door)
    with ctx.pose({door_hinge: door_hinge.motion_limits.upper}):
        open_door_aabb = ctx.part_world_aabb(door)
    ctx.check(
        "door swings outward from the front",
        rest_door_aabb is not None
        and open_door_aabb is not None
        and open_door_aabb[0][1] < rest_door_aabb[0][1] - 0.12,
        details=f"closed={rest_door_aabb}, open={open_door_aabb}",
    )

    ctx.check(
        "control dials use front-to-back rotary axes",
        all(
            joint.articulation_type == ArticulationType.REVOLUTE
            and tuple(joint.axis) == (0.0, 0.0, 1.0)
            and abs(joint.origin.rpy[0] - math.pi / 2.0) < 1e-6
            for joint in (time_joint, power_joint)
        ),
        details=f"time_axis={time_joint.axis}, power_axis={power_joint.axis}",
    )
    ctx.expect_contact(
        time_dial,
        housing,
        elem_a="knob_cap",
        elem_b="control_panel",
        contact_tol=0.003,
        name="time dial is seated in the front panel",
    )
    ctx.expect_contact(
        power_dial,
        housing,
        elem_a="knob_cap",
        elem_b="control_panel",
        contact_tol=0.003,
        name="power dial is seated in the front panel",
    )

    ctx.check(
        "turntable rotates about vertical axis",
        turntable_joint.articulation_type == ArticulationType.CONTINUOUS
        and tuple(turntable_joint.axis) == (0.0, 0.0, 1.0),
        details=f"type={turntable_joint.articulation_type}, axis={turntable_joint.axis}",
    )
    ctx.expect_within(
        turntable,
        housing,
        axes="xy",
        inner_elem="glass_plate",
        outer_elem="cavity_floor",
        margin=0.0,
        name="glass turntable fits inside the chamber floor",
    )
    ctx.expect_within(
        support_ring,
        housing,
        axes="xy",
        inner_elem="roller_ring",
        outer_elem="cavity_floor",
        margin=0.0,
        name="support ring fits inside the chamber floor",
    )
    ctx.expect_contact(
        turntable,
        support_ring,
        elem_a="glass_plate",
        elem_b="roller_ring",
        contact_tol=0.002,
        name="glass turntable sits on the support ring",
    )

    return ctx.report()


object_model = build_object_model()
