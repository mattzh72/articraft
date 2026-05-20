from __future__ import annotations

import math

import cadquery as cq

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
    mesh_from_cadquery,
)


def _box_tube(outer_x: float, outer_y: float, height: float, wall: float) -> cq.Workplane:
    outer = cq.Workplane("XY").box(outer_x, outer_y, height)
    inner = cq.Workplane("XY").box(outer_x - 2.0 * wall, outer_y - 2.0 * wall, height + 0.004)
    return outer.cut(inner)


def _tray_shell(length: float, width: float, height: float, wall: float, floor: float) -> cq.Workplane:
    tray = cq.Workplane("XY").box(length, width, height)
    tray = (
        tray.faces(">Z")
        .workplane()
        .rect(length - 2.0 * wall, width - 2.0 * wall)
        .cutBlind(-(height - floor))
    )
    return tray


def _add_caster(
    model: ArticulatedObject,
    base,
    frame_material: Material,
    wheel_material: Material,
    index: int,
    x: float,
    y: float,
) -> None:
    axle_z = 0.030

    base.visual(
        Box((0.040, 0.026, 0.006)),
        origin=Origin(xyz=(x, y, 0.063)),
        material=frame_material,
        name=f"fork_cap_{index}",
    )
    for side, dx in enumerate((-0.015, 0.015)):
        base.visual(
            Box((0.005, 0.026, 0.028)),
            origin=Origin(xyz=(x + dx, y, 0.046)),
            material=frame_material,
            name=f"fork_arm_{index}_{side}",
        )

    caster = model.part(f"caster_{index}")
    caster.visual(
        Cylinder(radius=0.030, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=wheel_material,
        name="wheel_tire",
    )
    caster.visual(
        Cylinder(radius=0.013, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=frame_material,
        name="wheel_hub",
    )

    model.articulation(
        f"base_to_caster_{index}",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=caster,
        origin=Origin(xyz=(x, y, axle_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=12.0),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hospital_overbed_table")

    frame_material = model.material("frame_ivory", rgba=(0.86, 0.85, 0.80, 1.0))
    tray_material = model.material("tray_beige", rgba=(0.91, 0.89, 0.84, 1.0))
    trim_material = model.material("trim_taupe", rgba=(0.69, 0.67, 0.62, 1.0))
    metal_material = model.material("brushed_metal", rgba=(0.74, 0.76, 0.78, 1.0))
    lever_material = model.material("graphite", rgba=(0.25, 0.27, 0.29, 1.0))
    wheel_material = model.material("wheel_rubber", rgba=(0.10, 0.10, 0.11, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.55, 0.055, 0.030)),
        origin=Origin(xyz=(0.045, -0.180, 0.075)),
        material=frame_material,
        name="floor_rail_0",
    )
    base.visual(
        Box((0.55, 0.055, 0.030)),
        origin=Origin(xyz=(0.045, 0.180, 0.075)),
        material=frame_material,
        name="floor_rail_1",
    )
    base.visual(
        Box((0.050, 0.415, 0.030)),
        origin=Origin(xyz=(-0.275, 0.0, 0.075)),
        material=frame_material,
        name="crossbar",
    )
    base.visual(
        Box((0.110, 0.155, 0.050)),
        origin=Origin(xyz=(-0.200, 0.0, 0.105)),
        material=frame_material,
        name="column_saddle",
    )
    base.visual(
        Box((0.110, 0.090, 0.028)),
        origin=Origin(xyz=(-0.145, -0.115, 0.085)),
        material=frame_material,
        name="saddle_bridge_0",
    )
    base.visual(
        Box((0.110, 0.090, 0.028)),
        origin=Origin(xyz=(-0.145, 0.115, 0.085)),
        material=frame_material,
        name="saddle_bridge_1",
    )
    base.visual(
        mesh_from_cadquery(_box_tube(0.094, 0.064, 0.060, 0.008), "post_collar"),
        origin=Origin(xyz=(-0.200, 0.0, 0.159)),
        material=frame_material,
        name="post_collar",
    )
    base.visual(
        mesh_from_cadquery(_box_tube(0.085, 0.055, 0.420, 0.008), "outer_post"),
        origin=Origin(xyz=(-0.200, 0.0, 0.339)),
        material=metal_material,
        name="outer_post",
    )

    for index, (x, y) in enumerate(((-0.160, -0.180), (-0.160, 0.180), (0.275, -0.180), (0.275, 0.180))):
        _add_caster(model, base, frame_material, wheel_material, index, x, y)

    lift_column = model.part("lift_column")
    lift_column.visual(
        Box((0.059, 0.033, 0.780)),
        origin=Origin(xyz=(0.0, 0.0, -0.030)),
        material=metal_material,
        name="inner_post",
    )
    lift_column.visual(
        Box((0.285, 0.060, 0.040)),
        origin=Origin(xyz=(0.1425, 0.0, 0.360)),
        material=frame_material,
        name="head_arm",
    )
    lift_column.visual(
        Box((0.082, 0.100, 0.048)),
        origin=Origin(xyz=(0.280, 0.0, 0.364)),
        material=frame_material,
        name="head_block",
    )
    lift_column.visual(
        Box((0.024, 0.120, 0.050)),
        origin=Origin(xyz=(0.246, 0.0, 0.350)),
        material=frame_material,
        name="head_neck",
    )

    model.articulation(
        "base_to_lift_column",
        ArticulationType.PRISMATIC,
        parent=base,
        child=lift_column,
        origin=Origin(xyz=(-0.200, 0.0, 0.550)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=90.0,
            velocity=0.16,
            lower=0.0,
            upper=0.220,
        ),
    )

    tray = model.part("tray")
    tray.visual(
        mesh_from_cadquery(_tray_shell(0.540, 0.380, 0.028, 0.012, 0.006), "tray_shell"),
        origin=Origin(xyz=(0.060, 0.0, 0.014)),
        material=tray_material,
        name="tray_shell",
    )
    tray.visual(
        Box((0.160, 0.120, 0.024)),
        origin=Origin(xyz=(0.000, 0.0, 0.005)),
        material=trim_material,
        name="tray_pivot",
    )
    tray.visual(
        Box((0.022, 0.160, 0.018)),
        origin=Origin(xyz=(0.327, 0.0, 0.000)),
        material=trim_material,
        name="wing_bracket",
    )
    for x, name in ((-0.110, "lever_mount_0"), (0.110, "lever_mount_1")):
        tray.visual(
            Box((0.016, 0.014, 0.014)),
            origin=Origin(xyz=(x, 0.156, -0.004)),
            material=trim_material,
            name=name,
        )

    model.articulation(
        "lift_column_to_tray",
        ArticulationType.REVOLUTE,
        parent=lift_column,
        child=tray,
        origin=Origin(xyz=(0.280, 0.0, 0.395)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.4,
            lower=-0.12,
            upper=0.85,
        ),
    )

    wing = model.part("wing")
    wing.visual(
        mesh_from_cadquery(_tray_shell(0.220, 0.240, 0.022, 0.010, 0.005), "wing_shell"),
        origin=Origin(xyz=(0.110, 0.0, 0.017)),
        material=tray_material,
        name="wing_shell",
    )
    wing.visual(
        Cylinder(radius=0.008, length=0.180),
        origin=Origin(xyz=(0.0, 0.0, 0.011), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=trim_material,
        name="wing_barrel",
    )

    model.articulation(
        "tray_to_wing",
        ArticulationType.REVOLUTE,
        parent=tray,
        child=wing,
        origin=Origin(xyz=(0.338, 0.0, 0.006)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=1.2,
            lower=0.0,
            upper=1.35,
        ),
    )

    lever = model.part("lever")
    lever.visual(
        Box((0.180, 0.046, 0.008)),
        origin=Origin(xyz=(0.0, 0.024, -0.018)),
        material=lever_material,
        name="lever_paddle",
    )
    lever.visual(
        Cylinder(radius=0.008, length=0.160),
        origin=Origin(xyz=(0.0, 0.040, -0.022), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=lever_material,
        name="grip_bar",
    )
    for x, name in ((-0.072, "lever_arm_0"), (0.072, "lever_arm_1")):
        lever.visual(
            Box((0.012, 0.016, 0.020)),
            origin=Origin(xyz=(x, 0.010, -0.010)),
            material=lever_material,
            name=name,
        )

    model.articulation(
        "tray_to_lever",
        ArticulationType.REVOLUTE,
        parent=tray,
        child=lever,
        origin=Origin(xyz=(0.015, 0.155, 0.004)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=2.2,
            lower=0.0,
            upper=0.65,
        ),
    )

    return model


def _max_z(aabb):
    if aabb is None:
        return None
    return aabb[1][2]


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    lift_column = object_model.get_part("lift_column")
    tray = object_model.get_part("tray")
    wing = object_model.get_part("wing")
    lever = object_model.get_part("lever")

    lift_joint = object_model.get_articulation("base_to_lift_column")
    tray_joint = object_model.get_articulation("lift_column_to_tray")
    wing_joint = object_model.get_articulation("tray_to_wing")
    lever_joint = object_model.get_articulation("tray_to_lever")

    ctx.expect_within(
        lift_column,
        base,
        axes="xy",
        inner_elem="inner_post",
        outer_elem="outer_post",
        margin=0.0,
        name="inner post stays centered in the outer post",
    )
    ctx.expect_overlap(
        lift_column,
        base,
        axes="z",
        elem_a="inner_post",
        elem_b="outer_post",
        min_overlap=0.390,
        name="collapsed lift column remains deeply inserted",
    )

    rest_lift_pos = ctx.part_world_position(lift_column)
    tray_rest_aabb = ctx.part_element_world_aabb(tray, elem="tray_shell")
    wing_rest_aabb = ctx.part_element_world_aabb(wing, elem="wing_shell")
    lever_rest_aabb = ctx.part_element_world_aabb(lever, elem="grip_bar")

    with ctx.pose({lift_joint: 0.220}):
        ctx.expect_within(
            lift_column,
            base,
            axes="xy",
            inner_elem="inner_post",
            outer_elem="outer_post",
            margin=0.0,
            name="extended lift column stays centered in the outer post",
        )
        ctx.expect_overlap(
            lift_column,
            base,
            axes="z",
            elem_a="inner_post",
            elem_b="outer_post",
            min_overlap=0.180,
            name="extended lift column still retains insertion",
        )
        extended_lift_pos = ctx.part_world_position(lift_column)

    ctx.check(
        "lift column extends upward",
        rest_lift_pos is not None
        and extended_lift_pos is not None
        and extended_lift_pos[2] > rest_lift_pos[2] + 0.18,
        details=f"rest={rest_lift_pos}, extended={extended_lift_pos}",
    )

    ctx.expect_contact(
        tray,
        lift_column,
        elem_a="tray_pivot",
        elem_b="head_block",
        name="tray pivot bears on the support head",
    )

    tray_limits = tray_joint.motion_limits
    if tray_limits is not None and tray_limits.upper is not None:
        with ctx.pose({tray_joint: tray_limits.upper}):
            tray_tilted_aabb = ctx.part_element_world_aabb(tray, elem="tray_shell")
        ctx.check(
            "tray tilts upward at the front edge",
            _max_z(tray_rest_aabb) is not None
            and _max_z(tray_tilted_aabb) is not None
            and _max_z(tray_tilted_aabb) > _max_z(tray_rest_aabb) + 0.12,
            details=f"rest={tray_rest_aabb}, tilted={tray_tilted_aabb}",
        )

    wing_limits = wing_joint.motion_limits
    if wing_limits is not None and wing_limits.upper is not None:
        with ctx.pose({wing_joint: wing_limits.upper}):
            wing_raised_aabb = ctx.part_element_world_aabb(wing, elem="wing_shell")
        ctx.check(
            "reading wing lifts beside the tray",
            _max_z(wing_rest_aabb) is not None
            and _max_z(wing_raised_aabb) is not None
            and _max_z(wing_raised_aabb) > _max_z(wing_rest_aabb) + 0.10,
            details=f"rest={wing_rest_aabb}, raised={wing_raised_aabb}",
        )

    lever_limits = lever_joint.motion_limits
    if lever_limits is not None and lever_limits.upper is not None:
        with ctx.pose({lever_joint: lever_limits.upper}):
            lever_squeezed_aabb = ctx.part_element_world_aabb(lever, elem="grip_bar")
        ctx.check(
            "release lever squeezes upward",
            _max_z(lever_rest_aabb) is not None
            and _max_z(lever_squeezed_aabb) is not None
            and _max_z(lever_squeezed_aabb) > _max_z(lever_rest_aabb) + 0.015,
            details=f"rest={lever_rest_aabb}, squeezed={lever_squeezed_aabb}",
        )

    caster_joint_names = [f"base_to_caster_{index}" for index in range(4)]
    caster_ok = True
    caster_details = []
    for name in caster_joint_names:
        joint = object_model.get_articulation(name)
        caster_ok = caster_ok and joint.articulation_type == ArticulationType.CONTINUOUS and tuple(joint.axis) == (1.0, 0.0, 0.0)
        caster_details.append((name, str(joint.articulation_type), tuple(joint.axis)))
    ctx.check(
        "all caster wheels use continuous horizontal axles",
        caster_ok,
        details=str(caster_details),
    )

    return ctx.report()


object_model = build_object_model()
