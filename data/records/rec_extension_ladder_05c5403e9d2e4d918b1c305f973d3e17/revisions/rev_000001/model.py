from __future__ import annotations

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


def _rung(part, z: float, width: float, material: Material, name: str, y: float = 0.0) -> None:
    """Add a round ladder rung spanning the two side rails."""
    part.visual(
        Cylinder(radius=0.022, length=width),
        origin=Origin(xyz=(0.0, y, z), rpy=(0.0, 1.57079632679, 0.0)),
        material=material,
        name=name,
    )


def _pawl_mesh(name: str):
    """Single hooked lock-dog plate, extruded about its hinge axis."""
    # Built on the YZ plane.  X thickness is symmetric about the pivot.
    profile_yz = [
        (0.030, 0.035),
        (0.052, 0.000),
        (0.020, -0.055),
        (-0.045, -0.073),
        (-0.162, -0.090),
        (-0.168, -0.155),
        (-0.120, -0.190),
        (-0.054, -0.160),
        (-0.018, -0.088),
        (-0.020, 0.024),
    ]
    plate = cq.Workplane("YZ").polyline(profile_yz).close().extrude(0.034, both=True)
    rung_clearance = (
        cq.Workplane("YZ")
        .center(-0.125, -0.110)
        .circle(0.030)
        .extrude(0.090, both=True)
    )
    plate = plate.cut(rung_clearance)
    return mesh_from_cadquery(plate, name, tolerance=0.0008, angular_tolerance=0.08)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="firefighter_extension_ladder")

    aluminum = model.material("brushed_aluminum", rgba=(0.78, 0.82, 0.80, 1.0))
    darker_aluminum = model.material("shadowed_guide_aluminum", rgba=(0.45, 0.49, 0.48, 1.0))
    rubber = model.material("black_rubber", rgba=(0.02, 0.02, 0.018, 1.0))
    pawl_steel = model.material("zinc_yellow_lock_pawl", rgba=(0.95, 0.70, 0.12, 1.0))
    warning_red = model.material("red_stop_mark", rgba=(0.72, 0.05, 0.02, 1.0))

    base = model.part("base_section")
    # Firefighter ground-ladder base: long outside rails with round rungs.
    base.visual(
        Box((0.065, 0.080, 4.40)),
        origin=Origin(xyz=(-0.285, 0.0, 2.20)),
        material=aluminum,
        name="base_rail_0",
    )
    base.visual(
        Box((0.095, 0.100, 0.085)),
        origin=Origin(xyz=(-0.285, -0.010, 0.045)),
        material=rubber,
        name="rubber_foot_0",
    )
    base.visual(
        Box((0.065, 0.080, 4.40)),
        origin=Origin(xyz=(0.285, 0.0, 2.20)),
        material=aluminum,
        name="base_rail_1",
    )
    base.visual(
        Box((0.095, 0.100, 0.085)),
        origin=Origin(xyz=(0.285, -0.010, 0.045)),
        material=rubber,
        name="rubber_foot_1",
    )

    _rung(base, 0.36, 0.545, aluminum, "base_rung_0")
    _rung(base, 0.68, 0.545, aluminum, "base_rung_1")
    _rung(base, 1.00, 0.545, aluminum, "base_rung_2")
    _rung(base, 1.32, 0.545, aluminum, "base_rung_3")
    _rung(base, 1.64, 0.545, aluminum, "base_rung_4")
    _rung(base, 1.96, 0.545, aluminum, "base_rung_5")
    _rung(base, 2.28, 0.545, aluminum, "base_rung_6")
    _rung(base, 2.60, 0.545, aluminum, "base_rung_7")
    _rung(base, 2.92, 0.545, aluminum, "base_rung_8")
    _rung(base, 3.24, 0.545, aluminum, "base_rung_9")
    _rung(base, 3.56, 0.545, aluminum, "base_rung_10")
    _rung(base, 3.88, 0.545, aluminum, "base_rung_11")

    # Front guide channels that retain the fly rails while leaving a clear slide gap.
    base.visual(
        Box((0.030, 0.057, 4.05)),
        origin=Origin(xyz=(-0.2385, 0.0665, 2.22)),
        material=darker_aluminum,
        name="guide_web_0",
    )
    base.visual(
        Box((0.085, 0.018, 0.110)),
        origin=Origin(xyz=(-0.2385, 0.073, 0.225)),
        material=darker_aluminum,
        name="lower_guide_keeper_0",
    )
    base.visual(
        Box((0.085, 0.018, 0.110)),
        origin=Origin(xyz=(-0.2385, 0.073, 4.170)),
        material=darker_aluminum,
        name="upper_guide_keeper_0",
    )
    base.visual(
        Box((0.030, 0.057, 4.05)),
        origin=Origin(xyz=(0.2385, 0.0665, 2.22)),
        material=darker_aluminum,
        name="guide_web_1",
    )
    base.visual(
        Box((0.085, 0.018, 0.110)),
        origin=Origin(xyz=(0.2385, 0.073, 0.225)),
        material=darker_aluminum,
        name="lower_guide_keeper_1",
    )
    base.visual(
        Box((0.085, 0.018, 0.110)),
        origin=Origin(xyz=(0.2385, 0.073, 4.170)),
        material=darker_aluminum,
        name="upper_guide_keeper_1",
    )

    base.visual(
        Box((0.520, 0.020, 0.055)),
        origin=Origin(xyz=(0.0, -0.038, 4.34)),
        material=warning_red,
        name="base_stop_bar",
    )

    fly = model.part("fly_section")
    fly.visual(
        Box((0.055, 0.070, 4.10)),
        origin=Origin(xyz=(-0.220, 0.0, 2.05)),
        material=aluminum,
        name="fly_rail_0",
    )
    fly.visual(
        Box((0.055, 0.020, 0.260)),
        origin=Origin(xyz=(-0.220, -0.005, 1.43)),
        material=darker_aluminum,
        name="pawl_mount_plate_0",
    )
    fly.visual(
        Box((0.055, 0.070, 4.10)),
        origin=Origin(xyz=(0.220, 0.0, 2.05)),
        material=aluminum,
        name="fly_rail_1",
    )
    fly.visual(
        Box((0.055, 0.020, 0.260)),
        origin=Origin(xyz=(0.220, -0.005, 1.43)),
        material=darker_aluminum,
        name="pawl_mount_plate_1",
    )

    for i, z in enumerate([0.32 + 0.32 * n for n in range(12)]):
        _rung(fly, z, 0.425, aluminum, f"fly_rung_{i}")

    fly.visual(
        Box((0.440, 0.020, 0.055)),
        origin=Origin(xyz=(0.0, 0.044, 4.05)),
        material=warning_red,
        name="fly_top_stop",
    )

    slide = model.articulation(
        "base_to_fly_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=fly,
        origin=Origin(xyz=(0.0, 0.130, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=450.0, velocity=0.35, lower=0.0, upper=1.55),
    )
    slide.meta["description"] = "Fly section slides upward in the base guide channels with retained overlap."

    pawl_mesh = _pawl_mesh("hooked_lock_pawl")
    for x, name in [(-0.155, "pawl_0"), (0.155, "pawl_1")]:
        pawl = model.part(name)
        pawl.visual(
            pawl_mesh,
            origin=Origin(),
            material=pawl_steel,
            name="hook_plate",
        )
        pawl.visual(
            Cylinder(radius=0.024, length=0.150),
            origin=Origin(rpy=(0.0, 1.57079632679, 0.0)),
            material=darker_aluminum,
            name="pivot_barrel",
        )
        pawl.visual(
            Box((0.026, 0.018, 0.070)),
            origin=Origin(xyz=(0.0, 0.025, -0.020)),
            material=pawl_steel,
            name="heel_stop",
        )
        model.articulation(
            f"fly_to_{name}",
            ArticulationType.REVOLUTE,
            parent=fly,
            child=pawl,
            origin=Origin(xyz=(x, -0.005, 1.430)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=18.0, velocity=2.5, lower=-0.35, upper=0.75),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_section")
    fly = object_model.get_part("fly_section")
    pawl_0 = object_model.get_part("pawl_0")
    pawl_1 = object_model.get_part("pawl_1")
    slide = object_model.get_articulation("base_to_fly_slide")
    pawl_joint = object_model.get_articulation("fly_to_pawl_0")

    ctx.allow_overlap(
        fly,
        pawl_0,
        elem_a="pawl_mount_plate_0",
        elem_b="pivot_barrel",
        reason="The pawl pivot pin is intentionally captured through the fly hinge mounting plate.",
    )
    ctx.allow_overlap(
        fly,
        pawl_0,
        elem_a="fly_rail_0",
        elem_b="pivot_barrel",
        reason="The pawl pivot barrel passes through the fly side rail at the lock-dog hinge.",
    )
    ctx.allow_overlap(
        fly,
        pawl_1,
        elem_a="pawl_mount_plate_1",
        elem_b="pivot_barrel",
        reason="The pawl pivot pin is intentionally captured through the fly hinge mounting plate.",
    )
    ctx.allow_overlap(
        fly,
        pawl_1,
        elem_a="fly_rail_1",
        elem_b="pivot_barrel",
        reason="The pawl pivot barrel passes through the fly side rail at the lock-dog hinge.",
    )
    ctx.expect_overlap(
        pawl_0,
        fly,
        axes="xz",
        elem_a="pivot_barrel",
        elem_b="pawl_mount_plate_0",
        min_overlap=0.030,
        name="pawl pin passes through fly hinge plate",
    )
    ctx.expect_overlap(
        pawl_1,
        fly,
        axes="xz",
        elem_a="pivot_barrel",
        elem_b="pawl_mount_plate_1",
        min_overlap=0.030,
        name="second pawl pin passes through fly hinge plate",
    )
    ctx.expect_overlap(
        pawl_0,
        fly,
        axes="xz",
        elem_a="pivot_barrel",
        elem_b="fly_rail_0",
        min_overlap=0.030,
        name="pawl pin is retained in fly rail",
    )
    ctx.expect_overlap(
        pawl_1,
        fly,
        axes="xz",
        elem_a="pivot_barrel",
        elem_b="fly_rail_1",
        min_overlap=0.030,
        name="second pawl pin is retained in fly rail",
    )

    ctx.expect_within(
        fly,
        base,
        axes="x",
        inner_elem="fly_rail_0",
        outer_elem="base_rail_0",
        margin=0.120,
        name="fly rails sit inside base width",
    )
    ctx.expect_overlap(
        fly,
        base,
        axes="z",
        elem_a="fly_rail_0",
        elem_b="guide_web_0",
        min_overlap=3.5,
        name="collapsed fly is retained in guide channel",
    )
    ctx.expect_overlap(
        pawl_0,
        base,
        axes="xz",
        elem_a="hook_plate",
        elem_b="base_rung_3",
        min_overlap=0.015,
        name="lock pawl aligns with a base rung",
    )

    rest_pos = ctx.part_world_position(fly)
    with ctx.pose({slide: 1.55}):
        ctx.expect_overlap(
            fly,
            base,
            axes="z",
            elem_a="fly_rail_0",
            elem_b="guide_web_0",
            min_overlap=2.0,
            name="extended fly keeps long retained insertion",
        )
        extended_pos = ctx.part_world_position(fly)
    ctx.check(
        "fly section extends upward",
        rest_pos is not None and extended_pos is not None and extended_pos[2] > rest_pos[2] + 1.45,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    hook_rest_aabb = ctx.part_element_world_aabb(pawl_0, elem="hook_plate")
    hook_rest_y = None if hook_rest_aabb is None else 0.5 * (hook_rest_aabb[0][1] + hook_rest_aabb[1][1])
    with ctx.pose({pawl_joint: 0.70}):
        hook_lifted_aabb = ctx.part_element_world_aabb(pawl_0, elem="hook_plate")
        hook_lifted_y = None if hook_lifted_aabb is None else 0.5 * (hook_lifted_aabb[0][1] + hook_lifted_aabb[1][1])
    ctx.check(
        "pawl swings outward to release",
        hook_rest_y is not None and hook_lifted_y is not None and hook_lifted_y > hook_rest_y + 0.040,
        details=f"rest_y={hook_rest_y}, lifted_y={hook_lifted_y}",
    )

    return ctx.report()


object_model = build_object_model()
