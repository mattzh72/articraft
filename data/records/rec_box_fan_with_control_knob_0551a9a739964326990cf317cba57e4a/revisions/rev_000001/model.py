from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    FanRotorBlade,
    FanRotorGeometry,
    FanRotorHub,
    KnobGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
    mesh_from_geometry,
)


HOUSING_W = 0.44
HOUSING_D = 0.18
HOUSING_H = 0.47
WALL = 0.018
FOOT_H = 0.022
BODY_CENTER_Z = FOOT_H + HOUSING_H / 2.0
FAN_RADIUS = 0.145
FAN_CENTER_Z = BODY_CENTER_Z + 0.004
HANDLE_PIVOT_X = HOUSING_W / 2.0 + 0.026
HANDLE_PIVOT_Z = BODY_CENTER_Z + 0.060
HANDLE_PIVOT_SPAN = 0.135
KNOB_X = 0.130
KNOB_Z = BODY_CENTER_Z - 0.120


def _housing_shape() -> cq.Workplane:
    front_panel_t = 0.010
    rear_frame_t = 0.008
    front_y = HOUSING_D / 2.0 - front_panel_t / 2.0
    rear_y = -HOUSING_D / 2.0 + rear_frame_t / 2.0

    shell = (
        cq.Workplane("XY")
        .box(HOUSING_W, HOUSING_D, HOUSING_H)
        .edges("|Z")
        .fillet(0.024)
        .cut(cq.Workplane("XY").box(HOUSING_W - 2.0 * WALL, HOUSING_D + 0.020, HOUSING_H - 2.0 * WALL))
        .translate((0.0, 0.0, BODY_CENTER_Z))
    )

    front_frame = (
        cq.Workplane("XY")
        .box(HOUSING_W * 0.92, front_panel_t, HOUSING_H * 0.92)
        .translate((0.0, front_y, BODY_CENTER_Z))
        .cut(
            cq.Workplane("XZ")
            .circle(FAN_RADIUS + 0.020)
            .extrude(front_panel_t + 0.004, both=True)
            .translate((0.0, front_y, BODY_CENTER_Z))
        )
    )

    front_lip = (
        cq.Workplane("XZ")
        .circle(FAN_RADIUS + 0.028)
        .extrude(0.014, both=True)
        .cut(
            cq.Workplane("XZ")
            .circle(FAN_RADIUS + 0.010)
            .extrude(0.020, both=True)
        )
        .translate((0.0, front_y + 0.006, BODY_CENTER_Z))
    )

    front_grille = cq.Workplane("XY")
    for z_offset in (-0.092, -0.040, 0.0, 0.040, 0.092):
        front_grille = front_grille.union(
            cq.Workplane("XY").box(FAN_RADIUS * 1.88, front_panel_t + 0.002, 0.010).translate((0.0, front_y, BODY_CENTER_Z + z_offset))
        )
    for x_offset in (-0.075, -0.028, 0.028, 0.075):
        front_grille = front_grille.union(
            cq.Workplane("XY").box(0.010, front_panel_t + 0.002, FAN_RADIUS * 1.78).translate((x_offset, front_y, BODY_CENTER_Z))
        )

    rear_frame = (
        cq.Workplane("XY")
        .box(HOUSING_W - 0.010, rear_frame_t, HOUSING_H - 0.014)
        .translate((0.0, rear_y, BODY_CENTER_Z))
        .cut(
            cq.Workplane("XY")
            .box(HOUSING_W * 0.72, rear_frame_t + 0.004, HOUSING_H * 0.72)
            .translate((0.0, rear_y, BODY_CENTER_Z))
        )
    )

    rear_slats = cq.Workplane("XY")
    for z_offset in (-0.110, -0.074, -0.038, 0.0, 0.038, 0.074, 0.110):
        rear_slats = rear_slats.union(
            cq.Workplane("XY").box(0.340, rear_frame_t + 0.002, 0.008).translate((0.0, rear_y, BODY_CENTER_Z + z_offset))
        )

    motor_bulge = (
        cq.Workplane("XZ")
        .circle(0.055)
        .extrude(0.060, both=True)
        .translate((0.0, rear_y - 0.020, BODY_CENTER_Z))
    )
    motor_neck = (
        cq.Workplane("XZ")
        .circle(0.020)
        .extrude(0.020, both=True)
        .translate((0.0, rear_y - 0.010, BODY_CENTER_Z))
    )

    control_pad = cq.Workplane("XY").box(0.092, 0.016, 0.078).translate((KNOB_X, front_y + 0.005, KNOB_Z))
    control_pad = control_pad.edges("|Y").fillet(0.007)

    hinge_pad = cq.Workplane("XY").box(0.090, 0.014, 0.030).translate((0.0, rear_y - 0.004, BODY_CENTER_Z + 0.168))

    pivot_pad = cq.Workplane("YZ").circle(0.016).extrude(0.012, both=True)
    side_pads = (
        pivot_pad.translate((HOUSING_W / 2.0 + 0.006, -HANDLE_PIVOT_SPAN / 2.0, HANDLE_PIVOT_Z))
        .union(pivot_pad.translate((HOUSING_W / 2.0 + 0.006, HANDLE_PIVOT_SPAN / 2.0, HANDLE_PIVOT_Z)))
    )

    feet = (
        cq.Workplane("XY")
        .box(0.118, 0.072, FOOT_H)
        .translate((-0.128, 0.0, FOOT_H / 2.0))
        .union(cq.Workplane("XY").box(0.118, 0.072, FOOT_H).translate((0.128, 0.0, FOOT_H / 2.0)))
    )

    return (
        shell.union(front_frame)
        .union(front_lip)
        .union(front_grille)
        .union(rear_frame)
        .union(rear_slats)
        .union(motor_bulge)
        .union(motor_neck)
        .union(control_pad)
        .union(hinge_pad)
        .union(side_pads)
        .union(feet)
        .combine()
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="utility_box_fan")

    plastic = model.material("plastic", rgba=(0.18, 0.21, 0.24, 1.0))
    dark_trim = model.material("dark_trim", rgba=(0.10, 0.11, 0.12, 1.0))
    blade_blue = model.material("blade_blue", rgba=(0.25, 0.42, 0.56, 0.92))
    knob_black = model.material("knob_black", rgba=(0.07, 0.07, 0.08, 1.0))
    steel = model.material("steel", rgba=(0.63, 0.66, 0.70, 1.0))
    rubber = model.material("rubber", rgba=(0.12, 0.12, 0.13, 1.0))

    housing = model.part("housing")
    housing.visual(
        mesh_from_cadquery(_housing_shape(), "housing_shell"),
        material=plastic,
        name="housing_shell",
    )

    blade = model.part("blade")
    blade.visual(
        mesh_from_geometry(
            FanRotorGeometry(
                FAN_RADIUS,
                0.044,
                5,
                thickness=0.020,
                blade_pitch_deg=27.0,
                blade_sweep_deg=18.0,
                blade=FanRotorBlade(shape="broad", tip_pitch_deg=11.0, camber=0.14),
                hub=FanRotorHub(style="spinner", rear_collar_height=0.010, rear_collar_radius=0.030, bore_diameter=0.010),
            ),
            "fan_rotor",
        ),
        material=blade_blue,
        name="rotor",
    )
    blade.visual(
        Cylinder(radius=0.010, length=0.090),
        origin=Origin(xyz=(0.0, 0.0, -0.045)),
        material=dark_trim,
        name="axle",
    )

    handle = model.part("handle")
    for y_offset, suffix in ((-HANDLE_PIVOT_SPAN / 2.0, "front"), (HANDLE_PIVOT_SPAN / 2.0, "rear")):
        handle.visual(
            Cylinder(radius=0.008, length=0.022),
            origin=Origin(xyz=(0.0, y_offset, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=dark_trim,
            name=f"{suffix}_pivot",
        )
        handle.visual(
            Box((0.072, 0.014, 0.018)),
            origin=Origin(xyz=(0.036, y_offset, 0.0)),
            material=dark_trim,
            name=f"{suffix}_arm",
        )
        handle.visual(
            Box((0.016, 0.014, 0.108)),
            origin=Origin(xyz=(0.072, y_offset, -0.054)),
            material=dark_trim,
            name=f"{suffix}_leg",
        )
    handle.visual(
        Cylinder(radius=0.010, length=HANDLE_PIVOT_SPAN - 0.012),
        origin=Origin(xyz=(0.072, 0.0, -0.108), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="handle_grip",
    )

    wall_hook = model.part("wall_hook")
    wall_hook.visual(
        Cylinder(radius=0.006, length=0.076),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="hinge_barrel",
    )
    for x_offset, suffix in ((-0.024, "left"), (0.024, "right")):
        wall_hook.visual(
            Box((0.010, 0.010, 0.074)),
            origin=Origin(xyz=(x_offset, -0.006, -0.037)),
            material=steel,
            name=f"{suffix}_leg",
        )
    wall_hook.visual(
        Box((0.062, 0.010, 0.010)),
        origin=Origin(xyz=(0.0, -0.006, -0.074)),
        material=steel,
        name="hook_bridge",
    )
    wall_hook.visual(
        Box((0.050, 0.024, 0.010)),
        origin=Origin(xyz=(0.0, -0.016, -0.082)),
        material=steel,
        name="hook_lip",
    )

    knob = model.part("knob")
    knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.040,
                0.026,
                body_style="skirted",
                top_diameter=0.031,
                base_diameter=0.043,
                edge_radius=0.002,
                crown_radius=0.003,
                center=False,
            ),
            "speed_knob",
        ),
        material=knob_black,
        name="knob_cap",
    )
    knob.visual(
        Box((0.004, 0.012, 0.002)),
        origin=Origin(xyz=(0.0, 0.011, 0.0245)),
        material=steel,
        name="knob_pointer",
    )

    model.articulation(
        "housing_to_blade",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=blade,
        origin=Origin(xyz=(0.0, 0.014, FAN_CENTER_Z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=28.0),
    )
    model.articulation(
        "housing_to_handle",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=handle,
        origin=Origin(xyz=(HANDLE_PIVOT_X, 0.0, HANDLE_PIVOT_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=2.5, lower=0.0, upper=1.18),
    )
    model.articulation(
        "housing_to_wall_hook",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=wall_hook,
        origin=Origin(xyz=(0.0, -(HOUSING_D / 2.0 + 0.010), BODY_CENTER_Z + 0.168)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=2.0, lower=0.0, upper=1.30),
    )
    model.articulation(
        "housing_to_knob",
        ArticulationType.CONTINUOUS,
        parent=housing,
        child=knob,
        origin=Origin(xyz=(KNOB_X, HOUSING_D / 2.0 + 0.015, KNOB_Z), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.4, velocity=10.0),
    )

    return model


def _aabb_top_z(aabb) -> float | None:
    if aabb is None:
        return None
    return float(aabb[1][2])


def _aabb_min_y(aabb) -> float | None:
    if aabb is None:
        return None
    return float(aabb[0][1])


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    housing = object_model.get_part("housing")
    blade = object_model.get_part("blade")
    handle = object_model.get_part("handle")
    wall_hook = object_model.get_part("wall_hook")
    knob = object_model.get_part("knob")

    blade_joint = object_model.get_articulation("housing_to_blade")
    handle_joint = object_model.get_articulation("housing_to_handle")
    hook_joint = object_model.get_articulation("housing_to_wall_hook")
    knob_joint = object_model.get_articulation("housing_to_knob")

    ctx.expect_within(
        blade,
        housing,
        axes="xz",
        margin=0.028,
        name="rotor footprint stays inside the square housing",
    )
    ctx.expect_overlap(
        knob,
        housing,
        axes="xz",
        min_overlap=0.030,
        name="speed knob sits on the front control area",
    )
    ctx.expect_gap(
        knob,
        housing,
        axis="y",
        min_gap=0.0,
        max_gap=0.040,
        name="speed knob projects from the front face",
    )
    ctx.allow_overlap(
        blade,
        housing,
        elem_a="axle",
        elem_b="housing_shell",
        reason="The blade shaft is intentionally represented as captured inside the fixed motor bearing sleeve.",
    )
    ctx.allow_overlap(
        housing,
        wall_hook,
        elem_a="housing_shell",
        elem_b="hinge_barrel",
        reason="The fold-down wall hook uses a simplified barrel hinge nested inside the rear hinge pad.",
    )

    handle_rest = ctx.part_world_aabb(handle)
    with ctx.pose({handle_joint: handle_joint.motion_limits.upper}):
        handle_raised = ctx.part_world_aabb(handle)
    handle_rest_top = _aabb_top_z(handle_rest)
    handle_raised_top = _aabb_top_z(handle_raised)
    ctx.check(
        "handle lifts upward from the side pivots",
        handle_rest_top is not None and handle_raised_top is not None and handle_raised_top > handle_rest_top + 0.05,
        details=f"rest_top={handle_rest_top}, raised_top={handle_raised_top}",
    )

    hook_rest = ctx.part_world_aabb(wall_hook)
    with ctx.pose({hook_joint: hook_joint.motion_limits.upper}):
        hook_deployed = ctx.part_world_aabb(wall_hook)
    hook_rest_min_y = _aabb_min_y(hook_rest)
    hook_deployed_min_y = _aabb_min_y(hook_deployed)
    ctx.check(
        "wall hook swings rearward when deployed",
        hook_rest_min_y is not None and hook_deployed_min_y is not None and hook_deployed_min_y < hook_rest_min_y - 0.04,
        details=f"rest_min_y={hook_rest_min_y}, deployed_min_y={hook_deployed_min_y}",
    )

    ctx.check(
        "blade uses continuous spin articulation",
        blade_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={blade_joint.articulation_type!r}",
    )
    ctx.check(
        "front speed knob uses continuous rotation",
        knob_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={knob_joint.articulation_type!r}",
    )

    return ctx.report()


object_model = build_object_model()
