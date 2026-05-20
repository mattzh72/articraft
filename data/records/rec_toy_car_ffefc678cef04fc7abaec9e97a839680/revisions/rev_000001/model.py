from __future__ import annotations

from math import pi

import cadquery as cq

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


BODY_LENGTH = 0.162
BODY_WIDTH = 0.074
BODY_HEIGHT = 0.066
WHEEL_RADIUS = 0.0145
WHEEL_WIDTH = 0.010
TRACK_HALF = 0.041
FRONT_AXLE_X = 0.047
REAR_AXLE_X = -0.047
AXLE_Z = 0.0165
PANEL_GAP = 0.0006


def _cq_box(
    size: tuple[float, float, float],
    *,
    center: tuple[float, float, float] = (0.0, 0.0, 0.0),
    rotate_y_deg: float = 0.0,
) -> cq.Workplane:
    solid = cq.Workplane("XY").box(*size)
    if rotate_y_deg:
        solid = solid.rotate((0.0, 0.0, 0.0), (0.0, 1.0, 0.0), rotate_y_deg)
    if center != (0.0, 0.0, 0.0):
        solid = solid.translate(center)
    return solid


def _arch_cutter(x_pos: float, y_pos: float, radius: float) -> cq.Workplane:
    cutter = cq.Workplane("XZ").center(x_pos, AXLE_Z).circle(radius).extrude(0.020, both=True)
    return cutter.translate((0.0, y_pos, 0.0))


def _body_shell_mesh() -> object:
    shell = _cq_box((0.154, 0.072, 0.022), center=(0.0, 0.0, 0.015))
    shell = shell.union(_cq_box((0.096, 0.060, 0.026), center=(-0.006, 0.0, 0.038)))
    shell = shell.union(_cq_box((0.070, 0.056, 0.012), center=(-0.010, 0.0, 0.053)))
    shell = shell.union(_cq_box((0.044, 0.060, 0.016), center=(-0.050, 0.0, 0.044)))

    shell = shell.cut(_cq_box((0.100, 0.100, 0.050), center=(0.088, 0.0, 0.055), rotate_y_deg=-24.0))
    shell = shell.cut(_cq_box((0.100, 0.100, 0.060), center=(-0.090, 0.0, 0.060), rotate_y_deg=30.0))

    inner = _cq_box((0.128, 0.058, 0.038), center=(-0.006, 0.0, 0.030))
    inner = inner.union(_cq_box((0.094, 0.046, 0.024), center=(-0.006, 0.0, 0.045)))
    shell = shell.cut(inner)
    shell = shell.cut(_cq_box((0.124, 0.054, 0.018), center=(-0.006, 0.0, 0.006)))

    door_cut_x = -0.009
    door_cut_z = 0.035
    shell = shell.cut(_cq_box((0.058, 0.010, 0.042), center=(door_cut_x, 0.033, door_cut_z)))
    shell = shell.cut(_cq_box((0.058, 0.010, 0.042), center=(door_cut_x, -0.033, door_cut_z)))
    shell = shell.cut(_cq_box((0.058, 0.052, 0.012), center=(0.041, 0.0, 0.038), rotate_y_deg=7.0))
    shell = shell.cut(_cq_box((0.044, 0.054, 0.024), center=(-0.057, 0.0, 0.041), rotate_y_deg=-50.0))

    for x_pos in (FRONT_AXLE_X, REAR_AXLE_X):
        shell = shell.cut(_arch_cutter(x_pos, 0.031, 0.018))
        shell = shell.cut(_arch_cutter(x_pos, -0.031, 0.018))

    shell = shell.union(_cq_box((0.132, 0.060, 0.004), center=(0.000, 0.0, 0.006)))
    shell = shell.union(_cq_box((0.006, 0.020, 0.008), center=(FRONT_AXLE_X, 0.028, 0.012)))
    shell = shell.union(_cq_box((0.006, 0.020, 0.008), center=(FRONT_AXLE_X, -0.028, 0.012)))
    shell = shell.union(_cq_box((0.006, 0.020, 0.008), center=(REAR_AXLE_X, 0.028, 0.012)))
    shell = shell.union(_cq_box((0.006, 0.020, 0.008), center=(REAR_AXLE_X, -0.028, 0.012)))

    return mesh_from_cadquery(shell, "toy_hatchback_body_shell")


def _hood_mesh() -> object:
    hood = _cq_box((0.054, 0.050, 0.0022), center=(0.027, 0.0, 0.0011), rotate_y_deg=6.9)
    return mesh_from_cadquery(hood, "toy_hatchback_hood")


def _undertray_mesh() -> object:
    tray = _cq_box((0.148, 0.058, 0.004), center=(0.0, 0.0, 0.004))
    tray = tray.union(_cq_box((0.112, 0.050, 0.012), center=(-0.008, 0.0, 0.010)))
    tray = tray.union(_cq_box((0.038, 0.044, 0.012), center=(0.046, 0.0, 0.010)))
    tray = tray.union(_cq_box((0.038, 0.046, 0.010), center=(-0.056, 0.0, 0.009)))
    tray = tray.union(_cq_box((0.020, 0.044, 0.018), center=(0.000, 0.0, 0.015)))
    tray = tray.union(_cq_box((0.018, 0.018, 0.020), center=(-0.014, 0.013, 0.018)))
    tray = tray.union(_cq_box((0.018, 0.018, 0.020), center=(-0.014, -0.013, 0.018)))

    front_support = _cq_box((0.012, 0.022, 0.006), center=(FRONT_AXLE_X, 0.0, 0.013))
    rear_support = _cq_box((0.012, 0.022, 0.006), center=(REAR_AXLE_X, 0.0, 0.013))
    front_axle = cq.Workplane("XZ").center(FRONT_AXLE_X, AXLE_Z).circle(0.0022).extrude(0.070, both=True)
    rear_axle = cq.Workplane("XZ").center(REAR_AXLE_X, AXLE_Z).circle(0.0022).extrude(0.070, both=True)

    tray = tray.union(front_support).union(rear_support).union(front_axle).union(rear_axle)
    return mesh_from_cadquery(tray, "toy_hatchback_undertray")


def _add_wheel(part, *, silver, rubber) -> None:
    wheel_rot = Origin(rpy=(-pi / 2.0, 0.0, 0.0))
    part.visual(
        Cylinder(radius=WHEEL_RADIUS, length=WHEEL_WIDTH),
        origin=wheel_rot,
        material=rubber,
        name="tire",
    )
    part.visual(
        Cylinder(radius=WHEEL_RADIUS * 0.76, length=WHEEL_WIDTH * 0.72),
        origin=wheel_rot,
        material=silver,
        name="rim",
    )
    part.visual(
        Cylinder(radius=WHEEL_RADIUS * 0.28, length=WHEEL_WIDTH * 0.96),
        origin=wheel_rot,
        material=silver,
        name="hub",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="toy_hatchback")

    body_red = model.material("body_red", rgba=(0.76, 0.13, 0.10, 1.0))
    chassis_black = model.material("chassis_black", rgba=(0.10, 0.10, 0.11, 1.0))
    glass_smoke = model.material("glass_smoke", rgba=(0.20, 0.24, 0.30, 0.92))
    wheel_silver = model.material("wheel_silver", rgba=(0.78, 0.79, 0.81, 1.0))
    rubber = model.material("rubber", rgba=(0.05, 0.05, 0.05, 1.0))

    body = model.part("body")
    body.visual(_body_shell_mesh(), material=body_red, name="shell")
    body.visual(
        Box((0.044, 0.056, 0.0026)),
        origin=Origin(xyz=(0.000, 0.0, 0.040), rpy=(0.0, 0.88, 0.0)),
        material=glass_smoke,
        name="windshield",
    )

    hood = model.part("hood")
    hood.visual(
        _hood_mesh(),
        origin=Origin(),
        material=body_red,
        name="hood_shell",
    )

    left_door = model.part("left_door")
    left_door.visual(
        Box((0.054, 0.0024, 0.026)),
        origin=Origin(xyz=(-0.027, -0.0012, 0.013)),
        material=body_red,
        name="door_panel",
    )
    left_door.visual(
        Box((0.050, 0.0016, 0.016)),
        origin=Origin(xyz=(-0.025, -0.0008, 0.034)),
        material=glass_smoke,
        name="door_window",
    )
    left_door.visual(
        Box((0.008, 0.0024, 0.004)),
        origin=Origin(xyz=(-0.033, -0.0012, 0.022)),
        material=body_red,
        name="door_handle",
    )

    right_door = model.part("right_door")
    right_door.visual(
        Box((0.054, 0.0024, 0.026)),
        origin=Origin(xyz=(-0.027, 0.0012, 0.013)),
        material=body_red,
        name="door_panel",
    )
    right_door.visual(
        Box((0.050, 0.0016, 0.016)),
        origin=Origin(xyz=(-0.025, 0.0008, 0.034)),
        material=glass_smoke,
        name="door_window",
    )
    right_door.visual(
        Box((0.008, 0.0024, 0.004)),
        origin=Origin(xyz=(-0.033, 0.0012, 0.022)),
        material=body_red,
        name="door_handle",
    )

    hatch = model.part("hatch")
    hatch.visual(
        Box((0.048, 0.056, 0.0022)),
        origin=Origin(xyz=(-0.024, 0.0, -0.016), rpy=(0.0, -1.06, 0.0)),
        material=body_red,
        name="hatch_panel",
    )
    hatch.visual(
        Box((0.032, 0.046, 0.0016)),
        origin=Origin(xyz=(-0.023, 0.0, -0.0145), rpy=(0.0, -1.06, 0.0)),
        material=glass_smoke,
        name="rear_window",
    )
    front_left_wheel = model.part("front_left_wheel")
    _add_wheel(front_left_wheel, silver=wheel_silver, rubber=rubber)
    front_left_wheel.visual(
        Cylinder(radius=0.0022, length=0.0064),
        origin=Origin(xyz=(0.0, -0.0082, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=chassis_black,
        name="axle_pin",
    )
    front_right_wheel = model.part("front_right_wheel")
    _add_wheel(front_right_wheel, silver=wheel_silver, rubber=rubber)
    front_right_wheel.visual(
        Cylinder(radius=0.0022, length=0.0064),
        origin=Origin(xyz=(0.0, 0.0082, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=chassis_black,
        name="axle_pin",
    )
    rear_left_wheel = model.part("rear_left_wheel")
    _add_wheel(rear_left_wheel, silver=wheel_silver, rubber=rubber)
    rear_left_wheel.visual(
        Cylinder(radius=0.0022, length=0.0064),
        origin=Origin(xyz=(0.0, -0.0082, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=chassis_black,
        name="axle_pin",
    )
    rear_right_wheel = model.part("rear_right_wheel")
    _add_wheel(rear_right_wheel, silver=wheel_silver, rubber=rubber)
    rear_right_wheel.visual(
        Cylinder(radius=0.0022, length=0.0064),
        origin=Origin(xyz=(0.0, 0.0082, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=chassis_black,
        name="axle_pin",
    )

    model.articulation(
        "hood_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=hood,
        origin=Origin(xyz=(0.010, 0.0, 0.041)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=3.0, lower=0.0, upper=1.20),
    )
    model.articulation(
        "left_door_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=left_door,
        origin=Origin(xyz=(0.019, BODY_WIDTH * 0.5 + 0.0014, 0.014)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=3.0, lower=0.0, upper=1.25),
    )
    model.articulation(
        "right_door_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=right_door,
        origin=Origin(xyz=(0.019, -BODY_WIDTH * 0.5 - 0.0014, 0.014)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=2.0, velocity=3.0, lower=0.0, upper=1.25),
    )
    model.articulation(
        "hatch_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=hatch,
        origin=Origin(xyz=(-0.028, 0.0, 0.054)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=2.5, velocity=2.5, lower=0.0, upper=1.25),
    )

    for joint_name, child_name, x_pos, y_pos in (
        ("front_left_spin", "front_left_wheel", FRONT_AXLE_X, TRACK_HALF),
        ("front_right_spin", "front_right_wheel", FRONT_AXLE_X, -TRACK_HALF),
        ("rear_left_spin", "rear_left_wheel", REAR_AXLE_X, TRACK_HALF),
        ("rear_right_spin", "rear_right_wheel", REAR_AXLE_X, -TRACK_HALF),
    ):
        model.articulation(
            joint_name,
            ArticulationType.CONTINUOUS,
            parent=body,
            child=child_name,
            origin=Origin(xyz=(x_pos, y_pos, AXLE_Z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=4.0, velocity=20.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    body = object_model.get_part("body")
    hood = object_model.get_part("hood")
    left_door = object_model.get_part("left_door")
    right_door = object_model.get_part("right_door")
    hatch = object_model.get_part("hatch")

    hood_hinge = object_model.get_articulation("hood_hinge")
    left_hinge = object_model.get_articulation("left_door_hinge")
    right_hinge = object_model.get_articulation("right_door_hinge")
    hatch_hinge = object_model.get_articulation("hatch_hinge")

    ctx.allow_isolated_part(
        hood,
        reason="The hood uses a simplified toy-car seam at the rear hinge line without explicit hinge barrels.",
    )
    ctx.allow_isolated_part(
        hatch,
        reason="The rear liftgate uses a simplified top hinge seam without explicit hinge struts or hinge barrels.",
    )
    ctx.expect_overlap(
        left_door,
        body,
        axes="xz",
        min_overlap=0.025,
        name="left door stays aligned with its aperture",
    )
    ctx.expect_overlap(
        right_door,
        body,
        axes="xz",
        min_overlap=0.025,
        name="right door stays aligned with its aperture",
    )
    ctx.expect_overlap(
        hood,
        body,
        axes="xy",
        min_overlap=0.030,
        name="hood covers the front compartment opening",
    )
    ctx.expect_overlap(
        hatch,
        body,
        axes="y",
        min_overlap=0.040,
        name="rear hatch spans the rear opening width",
    )

    left_closed = ctx.part_world_aabb(left_door)
    right_closed = ctx.part_world_aabb(right_door)
    hood_closed = ctx.part_world_aabb(hood)
    hatch_closed = ctx.part_world_aabb(hatch)

    with ctx.pose({left_hinge: left_hinge.motion_limits.upper}):
        left_open = ctx.part_world_aabb(left_door)
    with ctx.pose({right_hinge: right_hinge.motion_limits.upper}):
        right_open = ctx.part_world_aabb(right_door)
    with ctx.pose({hood_hinge: hood_hinge.motion_limits.upper}):
        hood_open = ctx.part_world_aabb(hood)
    with ctx.pose({hatch_hinge: hatch_hinge.motion_limits.upper}):
        hatch_open = ctx.part_world_aabb(hatch)

    ctx.check(
        "left door swings outward",
        left_closed is not None and left_open is not None and left_open[1][1] > left_closed[1][1] + 0.010,
        details=f"closed={left_closed}, open={left_open}",
    )
    ctx.check(
        "right door swings outward",
        right_closed is not None and right_open is not None and right_open[0][1] < right_closed[0][1] - 0.010,
        details=f"closed={right_closed}, open={right_open}",
    )
    ctx.check(
        "hood lifts upward",
        hood_closed is not None and hood_open is not None and hood_open[1][2] > hood_closed[1][2] + 0.015,
        details=f"closed={hood_closed}, open={hood_open}",
    )
    ctx.check(
        "rear hatch lifts upward",
        hatch_closed is not None and hatch_open is not None and hatch_open[1][2] > hatch_closed[1][2] + 0.015,
        details=f"closed={hatch_closed}, open={hatch_open}",
    )

    for joint_name in (
        "front_left_spin",
        "front_right_spin",
        "rear_left_spin",
        "rear_right_spin",
    ):
        joint = object_model.get_articulation(joint_name)
        limits = joint.motion_limits
        ctx.check(
            f"{joint_name} is continuous",
            joint.articulation_type == ArticulationType.CONTINUOUS
            and limits is not None
            and limits.lower is None
            and limits.upper is None,
            details=f"type={joint.articulation_type}, limits={limits}",
        )

    return ctx.report()


object_model = build_object_model()
