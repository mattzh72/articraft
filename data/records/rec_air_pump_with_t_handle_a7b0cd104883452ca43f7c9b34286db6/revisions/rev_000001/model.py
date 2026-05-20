from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _shell_mesh(
    name: str,
    outer_profile: list[tuple[float, float]],
    inner_profile: list[tuple[float, float]],
):
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer_profile,
            inner_profile,
            segments=64,
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dual_head_floor_pump")

    powder_black = model.material("powder_black", rgba=(0.12, 0.12, 0.13, 1.0))
    pump_blue = model.material("pump_blue", rgba=(0.16, 0.34, 0.62, 1.0))
    steel = model.material("steel", rgba=(0.76, 0.78, 0.80, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.09, 1.0))
    light_face = model.material("light_face", rgba=(0.94, 0.95, 0.95, 1.0))
    warning_orange = model.material("warning_orange", rgba=(0.93, 0.44, 0.15, 1.0))
    glass = model.material("glass", rgba=(0.76, 0.84, 0.88, 0.40))

    base = model.part("base")

    barrel_shell = _shell_mesh(
        "barrel_shell",
        outer_profile=[
            (0.029, 0.000),
            (0.029, 0.530),
            (0.034, 0.548),
            (0.034, 0.560),
        ],
        inner_profile=[
            (0.0235, 0.018),
            (0.0235, 0.530),
            (0.0075, 0.548),
        ],
    )
    base.visual(
        Box((0.305, 0.068, 0.024)),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=powder_black,
        name="foot_bar",
    )
    base.visual(
        Box((0.118, 0.016, 0.005)),
        origin=Origin(xyz=(-0.072, 0.018, 0.0265)),
        material=rubber,
        name="left_tread",
    )
    base.visual(
        Box((0.118, 0.016, 0.005)),
        origin=Origin(xyz=(0.072, 0.018, 0.0265)),
        material=rubber,
        name="right_tread",
    )
    base.visual(
        Cylinder(radius=0.041, length=0.052),
        origin=Origin(xyz=(0.0, 0.0, 0.046)),
        material=powder_black,
        name="base_collar",
    )
    base.visual(
        barrel_shell,
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=pump_blue,
        name="barrel_shell",
    )
    base.visual(
        Box((0.028, 0.045, 0.056)),
        origin=Origin(xyz=(0.0, 0.041, 0.142)),
        material=powder_black,
        name="gauge_bracket",
    )
    base.visual(
        Cylinder(radius=0.046, length=0.024),
        origin=Origin(xyz=(0.0, 0.074, 0.142), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="gauge_bezel",
    )
    base.visual(
        Cylinder(radius=0.038, length=0.004),
        origin=Origin(xyz=(0.0, 0.086, 0.142), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=light_face,
        name="gauge_face",
    )
    base.visual(
        Cylinder(radius=0.005, length=0.006),
        origin=Origin(xyz=(0.0, 0.088, 0.142), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=warning_orange,
        name="needle_hub",
    )
    base.visual(
        Box((0.026, 0.003, 0.003)),
        origin=Origin(xyz=(0.009, 0.0895, 0.149)),
        material=warning_orange,
        name="needle",
    )
    base.visual(
        Cylinder(radius=0.038, length=0.001),
        origin=Origin(xyz=(0.0, 0.0905, 0.142), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=glass,
        name="gauge_lens",
    )
    base.visual(
        Cylinder(radius=0.007, length=0.030),
        origin=Origin(xyz=(0.045, -0.008, 0.120), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="hose_outlet",
    )
    base.visual(
        Box((0.020, 0.014, 0.014)),
        origin=Origin(xyz=(0.036, -0.008, 0.120)),
        material=powder_black,
        name="hose_mount",
    )

    handle = model.part("handle")
    handle.visual(
        Cylinder(radius=0.0075, length=0.540),
        origin=Origin(xyz=(0.0, 0.0, -0.190)),
        material=steel,
        name="rod",
    )
    handle.visual(
        Box((0.050, 0.024, 0.034)),
        origin=Origin(xyz=(0.0, 0.0, 0.095)),
        material=powder_black,
        name="yoke",
    )
    handle.visual(
        Cylinder(radius=0.007, length=0.290),
        origin=Origin(xyz=(0.0, 0.0, 0.108), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=powder_black,
        name="crossbar",
    )
    handle.visual(
        Cylinder(radius=0.012, length=0.012),
        origin=Origin(xyz=(-0.151, 0.0, 0.108), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=powder_black,
        name="left_endcap",
    )
    handle.visual(
        Cylinder(radius=0.012, length=0.012),
        origin=Origin(xyz=(0.151, 0.0, 0.108), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=powder_black,
        name="right_endcap",
    )

    grip = model.part("grip")
    grip.visual(
        _shell_mesh(
            "grip_sleeve",
            outer_profile=[
                (0.0155, -0.047),
                (0.0170, -0.036),
                (0.0170, 0.036),
                (0.0155, 0.047),
            ],
            inner_profile=[
                (0.0070, -0.047),
                (0.0070, 0.047),
            ],
        ),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber,
        name="grip_sleeve",
    )

    hose = model.part("hose")
    hose.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                [
                    (0.000, 0.000, 0.000),
                    (0.035, 0.000, -0.003),
                    (0.095, -0.005, -0.020),
                    (0.160, 0.004, -0.090),
                    (0.210, 0.020, -0.094),
                    (0.248, 0.020, -0.090),
                ],
                radius=0.006,
                samples_per_segment=18,
                radial_segments=18,
                cap_ends=True,
            ),
            "hose_tube",
        ),
        material=rubber,
        name="hose_tube",
    )
    hose.visual(
        Cylinder(radius=0.0075, length=0.014),
        origin=Origin(xyz=(0.006, 0.000, 0.000), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="base_ferrule",
    )
    hose.visual(
        Cylinder(radius=0.0080, length=0.014),
        origin=Origin(xyz=(0.254, 0.020, -0.090), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="head_ferrule",
    )

    valve_head = model.part("valve_head")
    valve_head.visual(
        Cylinder(radius=0.0075, length=0.012),
        origin=Origin(xyz=(0.006, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="hose_socket",
    )
    valve_head.visual(
        Box((0.034, 0.018, 0.016)),
        origin=Origin(xyz=(0.029, 0.0, 0.008)),
        material=powder_black,
        name="body",
    )
    valve_head.visual(
        Cylinder(radius=0.0038, length=0.014),
        origin=Origin(xyz=(0.053, -0.005, 0.006), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber,
        name="port_0",
    )
    valve_head.visual(
        Cylinder(radius=0.0038, length=0.014),
        origin=Origin(xyz=(0.053, 0.005, 0.006), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber,
        name="port_1",
    )
    valve_head.visual(
        Cylinder(radius=0.0035, length=0.004),
        origin=Origin(xyz=(0.020, -0.006, 0.018), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="pivot_boss_0",
    )
    valve_head.visual(
        Cylinder(radius=0.0035, length=0.004),
        origin=Origin(xyz=(0.020, 0.006, 0.018), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="pivot_boss_1",
    )

    clamp_lever = model.part("clamp_lever")
    clamp_lever.visual(
        Cylinder(radius=0.0033, length=0.008),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="pivot_barrel",
    )
    clamp_lever.visual(
        Box((0.032, 0.006, 0.008)),
        origin=Origin(xyz=(0.016, 0.0, 0.004)),
        material=warning_orange,
        name="lever_arm",
    )
    clamp_lever.visual(
        Cylinder(radius=0.0045, length=0.014),
        origin=Origin(xyz=(0.032, 0.0, 0.009), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=warning_orange,
        name="lever_tip",
    )

    model.articulation(
        "handle_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=handle,
        origin=Origin(xyz=(0.0, 0.0, 0.572)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=220.0,
            velocity=1.0,
            lower=0.0,
            upper=0.260,
        ),
    )
    model.articulation(
        "handle_to_grip",
        ArticulationType.CONTINUOUS,
        parent=handle,
        child=grip,
        origin=Origin(xyz=(0.082, 0.0, 0.108)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=8.0,
        ),
    )
    model.articulation(
        "base_to_hose",
        ArticulationType.FIXED,
        parent=base,
        child=hose,
        origin=Origin(xyz=(0.060, -0.008, 0.120)),
    )
    model.articulation(
        "hose_to_head",
        ArticulationType.FIXED,
        parent=hose,
        child=valve_head,
        origin=Origin(xyz=(0.260, 0.020, -0.090)),
    )
    model.articulation(
        "head_to_lever",
        ArticulationType.REVOLUTE,
        parent=valve_head,
        child=clamp_lever,
        origin=Origin(xyz=(0.020, 0.0, 0.018)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=4.0,
            lower=0.0,
            upper=1.10,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    handle = object_model.get_part("handle")
    grip = object_model.get_part("grip")
    hose = object_model.get_part("hose")
    valve_head = object_model.get_part("valve_head")
    clamp_lever = object_model.get_part("clamp_lever")

    handle_slide = object_model.get_articulation("handle_slide")
    grip_roll = object_model.get_articulation("handle_to_grip")
    lever_hinge = object_model.get_articulation("head_to_lever")

    ctx.allow_overlap(
        base,
        handle,
        elem_a="barrel_shell",
        elem_b="rod",
        reason="The pump rod is intentionally modeled as sliding inside the barrel shell along the main pumping axis.",
    )
    ctx.allow_overlap(
        handle,
        grip,
        elem_a="crossbar",
        elem_b="grip_sleeve",
        reason="The folding grip is intentionally represented as a sleeve rotating coaxially around the handle crossbar.",
    )

    ctx.expect_contact(
        hose,
        valve_head,
        elem_a="head_ferrule",
        elem_b="hose_socket",
        name="valve head stays supported by hose end",
    )
    ctx.expect_contact(
        clamp_lever,
        valve_head,
        elem_a="pivot_barrel",
        elem_b="pivot_boss_0",
        name="lever stays supported on first hinge boss",
    )
    ctx.expect_contact(
        clamp_lever,
        valve_head,
        elem_a="pivot_barrel",
        elem_b="pivot_boss_1",
        name="lever stays supported on second hinge boss",
    )
    ctx.expect_within(
        handle,
        grip,
        axes="yz",
        inner_elem="crossbar",
        outer_elem="grip_sleeve",
        margin=0.003,
        name="grip remains sleeved around crossbar",
    )
    ctx.expect_overlap(
        handle,
        grip,
        axes="x",
        elem_a="crossbar",
        elem_b="grip_sleeve",
        min_overlap=0.085,
        name="grip spans a real section of the crossbar",
    )

    handle_limits = handle_slide.motion_limits
    lever_limits = lever_hinge.motion_limits

    if handle_limits is not None and handle_limits.upper is not None:
        with ctx.pose({handle_slide: 0.0}):
            ctx.expect_within(
                handle,
                base,
                axes="xy",
                inner_elem="rod",
                outer_elem="barrel_shell",
                margin=0.0,
                name="rod stays centered in barrel at rest",
            )
            ctx.expect_overlap(
                handle,
                base,
                axes="z",
                elem_a="rod",
                elem_b="barrel_shell",
                min_overlap=0.40,
                name="rod is deeply inserted when the pump is down",
            )
            rest_handle_pos = ctx.part_world_position(handle)

        with ctx.pose({handle_slide: handle_limits.upper}):
            ctx.expect_within(
                handle,
                base,
                axes="xy",
                inner_elem="rod",
                outer_elem="barrel_shell",
                margin=0.0,
                name="rod stays centered in barrel at full stroke",
            )
            ctx.expect_overlap(
                handle,
                base,
                axes="z",
                elem_a="rod",
                elem_b="barrel_shell",
                min_overlap=0.19,
                name="rod remains inserted at full stroke",
            )
            raised_handle_pos = ctx.part_world_position(handle)

        ctx.check(
            "handle translates upward along pump axis",
            rest_handle_pos is not None
            and raised_handle_pos is not None
            and raised_handle_pos[2] > rest_handle_pos[2] + 0.20,
            details=f"rest={rest_handle_pos}, raised={raised_handle_pos}",
        )

    with ctx.pose({grip_roll: math.pi / 2.0}):
        ctx.expect_within(
            handle,
            grip,
            axes="yz",
            inner_elem="crossbar",
            outer_elem="grip_sleeve",
            margin=0.003,
            name="grip stays carried by the crossbar when rotated",
        )

    if lever_limits is not None and lever_limits.upper is not None:
        closed_tip = ctx.part_element_world_aabb(clamp_lever, elem="lever_tip")
        with ctx.pose({lever_hinge: lever_limits.upper}):
            open_tip = ctx.part_element_world_aabb(clamp_lever, elem="lever_tip")
        ctx.check(
            "clamp lever opens upward",
            closed_tip is not None
            and open_tip is not None
            and open_tip[1][2] > closed_tip[1][2] + 0.014,
            details=f"closed_tip={closed_tip}, open_tip={open_tip}",
        )

    return ctx.report()


object_model = build_object_model()
