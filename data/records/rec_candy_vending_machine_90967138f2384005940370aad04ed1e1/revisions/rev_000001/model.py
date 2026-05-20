from __future__ import annotations

from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _add_hopper(part, *, clear_material: str, trim_material: str) -> None:
    shell_w = 0.28
    shell_d = 0.22
    shell_h = 0.46
    wall_t = 0.006
    collar_w = 0.22
    collar_d = 0.18
    collar_h = 0.10
    seam = 0.001

    shell_z = collar_h + shell_h / 2.0 - seam / 2.0

    part.visual(
        Box((shell_w, wall_t, shell_h)),
        origin=Origin(xyz=(0.0, shell_d / 2.0 - wall_t / 2.0, shell_z)),
        material=clear_material,
        name="front_wall",
    )
    part.visual(
        Box((shell_w, wall_t, shell_h)),
        origin=Origin(xyz=(0.0, -shell_d / 2.0 + wall_t / 2.0, shell_z)),
        material=clear_material,
        name="rear_wall",
    )
    part.visual(
        Box((wall_t, shell_d - 2.0 * wall_t + 2.0 * seam, shell_h)),
        origin=Origin(xyz=(shell_w / 2.0 - wall_t / 2.0, 0.0, shell_z)),
        material=clear_material,
        name="side_wall_0",
    )
    part.visual(
        Box((wall_t, shell_d - 2.0 * wall_t + 2.0 * seam, shell_h)),
        origin=Origin(xyz=(-shell_w / 2.0 + wall_t / 2.0, 0.0, shell_z)),
        material=clear_material,
        name="side_wall_1",
    )

    part.visual(
        Box((collar_w, collar_d, collar_h)),
        origin=Origin(xyz=(0.0, 0.0, collar_h / 2.0)),
        material=trim_material,
        name="hopper_collar",
    )
    part.visual(
        Box((shell_w + 0.012, shell_d + 0.012, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, collar_h + 0.007)),
        material=trim_material,
        name="shoulder_plate",
    )
    part.visual(
        Box((0.095, 0.060, 0.036)),
        origin=Origin(xyz=(0.0, collar_d / 2.0 + 0.025, 0.038)),
        material=trim_material,
        name="drop_spout",
    )
    part.visual(
        Box((shell_w + 0.010, wall_t, 0.010)),
        origin=Origin(xyz=(0.0, shell_d / 2.0, collar_h + shell_h - 0.005)),
        material=trim_material,
        name="top_rim_front",
    )
    part.visual(
        Box((shell_w + 0.010, wall_t, 0.010)),
        origin=Origin(xyz=(0.0, -shell_d / 2.0, collar_h + shell_h - 0.005)),
        material=trim_material,
        name="top_rim_rear",
    )
    part.visual(
        Box((wall_t, shell_d + 0.010, 0.010)),
        origin=Origin(xyz=(shell_w / 2.0, 0.0, collar_h + shell_h - 0.005)),
        material=trim_material,
        name="top_rim_side_0",
    )
    part.visual(
        Box((wall_t, shell_d + 0.010, 0.010)),
        origin=Origin(xyz=(-shell_w / 2.0, 0.0, collar_h + shell_h - 0.005)),
        material=trim_material,
        name="top_rim_side_1",
    )


def _add_lid(part, *, material: str) -> None:
    lid_w = 0.295
    lid_d = 0.235
    lid_t = 0.014

    part.visual(
        Box((lid_w, lid_d, lid_t)),
        origin=Origin(xyz=(0.0, lid_d / 2.0, lid_t / 2.0)),
        material=material,
        name="lid_panel",
    )
    part.visual(
        Box((0.080, 0.030, 0.020)),
        origin=Origin(xyz=(0.0, 0.180, 0.018)),
        material=material,
        name="lid_handle",
    )
    part.visual(
        Cylinder(radius=0.010, length=0.050),
        origin=Origin(xyz=(0.0, 0.000, 0.010), rpy=(pi / 2.0, 0.0, 0.0)),
        material=material,
        name="hinge_barrel",
    )


def _add_knob(part, *, body_material: str, shaft_material: str, mesh_name: str) -> None:
    knob_mesh = mesh_from_geometry(
        KnobGeometry(
            0.060,
            0.032,
            body_style="skirted",
            top_diameter=0.044,
            base_diameter=0.060,
            edge_radius=0.003,
            center=False,
        ),
        mesh_name,
    )
    part.visual(
        knob_mesh,
        origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
        material=body_material,
        name="knob_cap",
    )
    part.visual(
        Cylinder(radius=0.024, length=0.008),
        origin=Origin(xyz=(0.0, 0.004, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=shaft_material,
        name="knob_collar",
    )


def _add_cup_door(part, *, material: str, trim_material: str) -> None:
    part.visual(
        Box((0.180, 0.012, 0.160)),
        origin=Origin(xyz=(0.0, 0.006, 0.080)),
        material=material,
        name="door_panel",
    )
    part.visual(
        Box((0.080, 0.018, 0.020)),
        origin=Origin(xyz=(0.0, 0.018, 0.120)),
        material=trim_material,
        name="door_pull",
    )
    part.visual(
        Cylinder(radius=0.008, length=0.184),
        origin=Origin(xyz=(0.0, 0.004, 0.008), rpy=(0.0, pi / 2.0, 0.0)),
        material=trim_material,
        name="door_hinge",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dual_hopper_candy_vendor")

    red = model.material("red", rgba=(0.78, 0.09, 0.09, 1.0))
    dark = model.material("dark", rgba=(0.16, 0.16, 0.17, 1.0))
    metal = model.material("metal", rgba=(0.74, 0.75, 0.78, 1.0))
    clear = model.material("clear", rgba=(0.82, 0.92, 0.98, 0.34))

    chassis = model.part("chassis")
    chassis.visual(
        Cylinder(radius=0.235, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=dark,
        name="foot",
    )
    chassis.visual(
        Cylinder(radius=0.060, length=0.720),
        origin=Origin(xyz=(0.0, 0.0, 0.390)),
        material=metal,
        name="pedestal",
    )
    chassis.visual(
        Box((0.520, 0.320, 0.180)),
        origin=Origin(xyz=(0.0, 0.0, 0.840)),
        material=red,
        name="body",
    )
    chassis.visual(
        Box((0.540, 0.340, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, 0.955)),
        material=red,
        name="top_deck",
    )
    chassis.visual(
        Box((0.480, 0.040, 0.160)),
        origin=Origin(xyz=(0.0, 0.180, 0.845)),
        material=red,
        name="front_panel",
    )
    chassis.visual(
        Box((0.240, 0.200, 0.020)),
        origin=Origin(xyz=(0.0, 0.100, 0.740)),
        material=red,
        name="cup_top",
    )
    chassis.visual(
        Box((0.020, 0.200, 0.220)),
        origin=Origin(xyz=(0.110, 0.100, 0.630)),
        material=red,
        name="cup_side_0",
    )
    chassis.visual(
        Box((0.020, 0.200, 0.220)),
        origin=Origin(xyz=(-0.110, 0.100, 0.630)),
        material=red,
        name="cup_side_1",
    )
    chassis.visual(
        Box((0.240, 0.020, 0.220)),
        origin=Origin(xyz=(0.0, 0.010, 0.630)),
        material=red,
        name="cup_back",
    )
    chassis.visual(
        Box((0.240, 0.200, 0.020)),
        origin=Origin(xyz=(0.0, 0.100, 0.520)),
        material=red,
        name="cup_bottom",
    )
    chassis.visual(
        Box((0.220, 0.008, 0.030)),
        origin=Origin(xyz=(0.0, 0.196, 0.715)),
        material=metal,
        name="cup_frame_top",
    )
    chassis.visual(
        Box((0.220, 0.008, 0.030)),
        origin=Origin(xyz=(0.0, 0.196, 0.545)),
        material=metal,
        name="cup_frame_bottom",
    )
    chassis.visual(
        Box((0.030, 0.008, 0.170)),
        origin=Origin(xyz=(0.095, 0.196, 0.630)),
        material=metal,
        name="cup_frame_side_0",
    )
    chassis.visual(
        Box((0.030, 0.008, 0.170)),
        origin=Origin(xyz=(-0.095, 0.196, 0.630)),
        material=metal,
        name="cup_frame_side_1",
    )
    chassis.visual(
        Box((0.060, 0.050, 0.050)),
        origin=Origin(xyz=(-0.170, 0.187, 0.948)),
        material=red,
        name="outlet_mount_0",
    )
    chassis.visual(
        Box((0.060, 0.050, 0.050)),
        origin=Origin(xyz=(0.170, 0.187, 0.948)),
        material=red,
        name="outlet_mount_1",
    )
    chassis.visual(
        Box((0.080, 0.030, 0.038)),
        origin=Origin(xyz=(-0.170, 0.212, 0.985)),
        material=metal,
        name="outlet_0",
    )
    chassis.visual(
        Box((0.080, 0.030, 0.038)),
        origin=Origin(xyz=(0.170, 0.212, 0.985)),
        material=metal,
        name="outlet_1",
    )

    hopper_0 = model.part("hopper_0")
    _add_hopper(hopper_0, clear_material=clear.name, trim_material=red.name)
    hopper_1 = model.part("hopper_1")
    _add_hopper(hopper_1, clear_material=clear.name, trim_material=red.name)

    lid_0 = model.part("lid_0")
    _add_lid(lid_0, material=red.name)
    lid_1 = model.part("lid_1")
    _add_lid(lid_1, material=red.name)

    knob_0 = model.part("knob_0")
    _add_knob(knob_0, body_material=metal.name, shaft_material=dark.name, mesh_name="knob_0")
    knob_1 = model.part("knob_1")
    _add_knob(knob_1, body_material=metal.name, shaft_material=dark.name, mesh_name="knob_1")

    cup_door = model.part("cup_door")
    _add_cup_door(cup_door, material=metal.name, trim_material=dark.name)

    model.articulation(
        "chassis_to_hopper_0",
        ArticulationType.FIXED,
        parent=chassis,
        child=hopper_0,
        origin=Origin(xyz=(-0.170, 0.0, 0.980)),
    )
    model.articulation(
        "chassis_to_hopper_1",
        ArticulationType.FIXED,
        parent=chassis,
        child=hopper_1,
        origin=Origin(xyz=(0.170, 0.0, 0.980)),
    )
    model.articulation(
        "hopper_0_to_lid_0",
        ArticulationType.REVOLUTE,
        parent=hopper_0,
        child=lid_0,
        origin=Origin(xyz=(0.0, -0.110, 0.560)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=2.0, lower=0.0, upper=1.25),
    )
    model.articulation(
        "hopper_1_to_lid_1",
        ArticulationType.REVOLUTE,
        parent=hopper_1,
        child=lid_1,
        origin=Origin(xyz=(0.0, -0.110, 0.560)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=2.0, lower=0.0, upper=1.25),
    )
    model.articulation(
        "chassis_to_knob_0",
        ArticulationType.CONTINUOUS,
        parent=chassis,
        child=knob_0,
        origin=Origin(xyz=(-0.170, 0.200, 0.865)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=8.0),
    )
    model.articulation(
        "chassis_to_knob_1",
        ArticulationType.CONTINUOUS,
        parent=chassis,
        child=knob_1,
        origin=Origin(xyz=(0.170, 0.200, 0.865)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=8.0),
    )
    model.articulation(
        "chassis_to_cup_door",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=cup_door,
        origin=Origin(xyz=(0.0, 0.200, 0.545)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.5, lower=0.0, upper=1.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    hopper_0 = object_model.get_part("hopper_0")
    hopper_1 = object_model.get_part("hopper_1")
    lid_0 = object_model.get_part("lid_0")
    lid_1 = object_model.get_part("lid_1")
    knob_0 = object_model.get_part("knob_0")
    knob_1 = object_model.get_part("knob_1")
    cup_door = object_model.get_part("cup_door")

    lid_joint_0 = object_model.get_articulation("hopper_0_to_lid_0")
    lid_joint_1 = object_model.get_articulation("hopper_1_to_lid_1")
    knob_joint_0 = object_model.get_articulation("chassis_to_knob_0")
    knob_joint_1 = object_model.get_articulation("chassis_to_knob_1")
    cup_joint = object_model.get_articulation("chassis_to_cup_door")

    ctx.expect_gap(
        hopper_1,
        hopper_0,
        axis="x",
        min_gap=0.045,
        name="the two reservoirs remain separate containers",
    )
    ctx.expect_gap(
        hopper_0,
        "chassis",
        axis="z",
        max_gap=0.002,
        max_penetration=0.0,
        negative_elem="top_deck",
        name="hopper_0 sits on the shared top deck",
    )
    ctx.expect_gap(
        hopper_1,
        "chassis",
        axis="z",
        max_gap=0.002,
        max_penetration=0.0,
        negative_elem="top_deck",
        name="hopper_1 sits on the shared top deck",
    )
    ctx.expect_gap(
        lid_0,
        hopper_0,
        axis="z",
        positive_elem="lid_panel",
        max_gap=0.012,
        max_penetration=0.0,
        name="lid_0 closes onto hopper_0",
    )
    ctx.expect_gap(
        lid_1,
        hopper_1,
        axis="z",
        positive_elem="lid_panel",
        max_gap=0.012,
        max_penetration=0.0,
        name="lid_1 closes onto hopper_1",
    )

    lid_closed_0 = ctx.part_element_world_aabb(lid_0, elem="lid_panel")
    lid_closed_1 = ctx.part_element_world_aabb(lid_1, elem="lid_panel")
    lid_upper_0 = lid_joint_0.motion_limits.upper if lid_joint_0.motion_limits is not None else None
    lid_upper_1 = lid_joint_1.motion_limits.upper if lid_joint_1.motion_limits is not None else None
    lid_open_0 = None
    lid_open_1 = None
    if lid_upper_0 is not None:
        with ctx.pose({lid_joint_0: lid_upper_0}):
            lid_open_0 = ctx.part_element_world_aabb(lid_0, elem="lid_panel")
    if lid_upper_1 is not None:
        with ctx.pose({lid_joint_1: lid_upper_1}):
            lid_open_1 = ctx.part_element_world_aabb(lid_1, elem="lid_panel")

    ctx.check(
        "lid_0 opens upward on its rear hinge",
        lid_closed_0 is not None
        and lid_open_0 is not None
        and lid_open_0[1][2] > lid_closed_0[1][2] + 0.10,
        details=f"closed={lid_closed_0}, open={lid_open_0}",
    )
    ctx.check(
        "lid_1 opens upward on its rear hinge",
        lid_closed_1 is not None
        and lid_open_1 is not None
        and lid_open_1[1][2] > lid_closed_1[1][2] + 0.10,
        details=f"closed={lid_closed_1}, open={lid_open_1}",
    )

    for index, (knob_part, joint_name, outlet_name) in enumerate(
        (
            (knob_0, knob_joint_0, "outlet_0"),
            (knob_1, knob_joint_1, "outlet_1"),
        )
    ):
        outlet_aabb = ctx.part_element_world_aabb("chassis", elem=outlet_name)
        knob_pos = ctx.part_world_position(knob_part)
        joint = joint_name
        axis = tuple(round(v, 3) for v in joint.axis)
        lower = None if joint.motion_limits is None else joint.motion_limits.lower
        upper = None if joint.motion_limits is None else joint.motion_limits.upper
        aligned = (
            outlet_aabb is not None
            and knob_pos is not None
            and abs(((outlet_aabb[0][0] + outlet_aabb[1][0]) / 2.0) - knob_pos[0]) <= 0.020
            and 0.070 <= (((outlet_aabb[0][2] + outlet_aabb[1][2]) / 2.0) - knob_pos[2]) <= 0.170
        )
        ctx.check(
            f"knob_{index} stays centered under its outlet",
            aligned,
            details=f"outlet={outlet_aabb}, knob={knob_pos}",
        )
        ctx.check(
            f"knob_{index} uses continuous horizontal rotation",
            joint.articulation_type == ArticulationType.CONTINUOUS
            and abs(axis[1]) == 1.0
            and lower is None
            and upper is None,
            details=f"type={joint.articulation_type}, axis={axis}, limits=({lower}, {upper})",
        )

    door_closed = ctx.part_element_world_aabb(cup_door, elem="door_panel")
    door_upper = cup_joint.motion_limits.upper if cup_joint.motion_limits is not None else None
    door_open = None
    if door_upper is not None:
        with ctx.pose({cup_joint: door_upper}):
            door_open = ctx.part_element_world_aabb(cup_door, elem="door_panel")

    ctx.check(
        "pickup door swings outward from its lower hinge",
        door_closed is not None
        and door_open is not None
        and door_open[1][1] > door_closed[1][1] + 0.10
        and door_open[0][2] < door_closed[0][2] + 0.02,
        details=f"closed={door_closed}, open={door_open}",
    )

    return ctx.report()


object_model = build_object_model()
