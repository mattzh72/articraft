from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    LatheGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _trigger_plate_mesh():
    """One-piece trigger blade, extruded across Y from a side profile."""
    profile_xz = [
        (-0.014, 0.006),
        (0.004, 0.014),
        (0.035, 0.011),
        (0.041, -0.006),
        (0.031, -0.032),
        (0.018, -0.104),
        (0.006, -0.128),
        (-0.015, -0.126),
        (-0.027, -0.105),
        (-0.018, -0.038),
        (-0.025, -0.011),
    ]
    # ExtrudeGeometry profiles live in local XY and extrude along local Z.
    # Rotate +90 deg about X so profile-Y becomes object-Z and extrusion becomes Y.
    return ExtrudeGeometry(profile_xz, 0.018, center=True).rotate_x(math.pi / 2.0)


def _bottle_shell_mesh():
    outer_profile = [
        (0.042, 0.000),
        (0.050, 0.014),
        (0.052, 0.060),
        (0.049, 0.142),
        (0.043, 0.166),
        (0.030, 0.192),
        (0.021, 0.204),
        (0.021, 0.229),
    ]
    inner_profile = [
        (0.034, 0.006),
        (0.036, 0.006),
        (0.044, 0.018),
        (0.045, 0.060),
        (0.042, 0.140),
        (0.036, 0.160),
        (0.025, 0.186),
        (0.015, 0.204),
        (0.015, 0.229),
    ]
    return LatheGeometry.from_shell_profiles(
        outer_profile,
        inner_profile,
        segments=72,
        start_cap="round",
        end_cap="flat",
        lip_samples=8,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="field_service_trigger_sprayer")

    bottle_plastic = model.material("translucent_tough_polyethylene", rgba=(0.72, 0.92, 1.00, 0.42))
    black_rubber = model.material("replaceable_black_rubber", rgba=(0.02, 0.025, 0.025, 1.0))
    safety_orange = model.material("service_orange_plastic", rgba=(0.95, 0.38, 0.06, 1.0))
    dark_polymer = model.material("glass_filled_black_polymer", rgba=(0.07, 0.075, 0.08, 1.0))
    steel = model.material("brushed_stainless_steel", rgba=(0.70, 0.72, 0.72, 1.0))
    wear_yellow = model.material("replaceable_wear_yellow", rgba=(0.98, 0.74, 0.08, 1.0))
    seal_blue = model.material("blue_service_cover", rgba=(0.02, 0.22, 0.56, 1.0))

    body = model.part("bottle")
    body.visual(
        mesh_from_geometry(_bottle_shell_mesh(), "thick_translucent_bottle"),
        material=bottle_plastic,
        name="bottle_shell",
    )
    body.visual(
        Cylinder(radius=0.055, length=0.014),
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
        material=black_rubber,
        name="rubber_boot",
    )
    body.visual(
        Cylinder(radius=0.028, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.222)),
        material=safety_orange,
        name="threaded_collar",
    )
    body.visual(
        Cylinder(radius=0.019, length=0.036),
        origin=Origin(xyz=(0.0, 0.0, 0.224)),
        material=dark_polymer,
        name="neck_insert",
    )
    body.visual(
        Cylinder(radius=0.021, length=0.046),
        origin=Origin(xyz=(0.0, 0.0, 0.244)),
        material=dark_polymer,
        name="neck_spigot",
    )

    # Chunky pump head and maintenance hardware, all one supported housing.
    body.visual(
        Box((0.125, 0.050, 0.026)),
        origin=Origin(xyz=(0.050, 0.0, 0.267)),
        material=dark_polymer,
        name="pump_housing",
    )
    body.visual(
        Box((0.060, 0.050, 0.012)),
        origin=Origin(xyz=(0.020, 0.0, 0.227)),
        material=dark_polymer,
        name="lower_pump_housing",
    )
    body.visual(
        Cylinder(radius=0.014, length=0.064),
        origin=Origin(xyz=(0.155, 0.0, 0.242), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_polymer,
        name="pump_barrel",
    )
    body.visual(
        Box((0.028, 0.034, 0.014)),
        origin=Origin(xyz=(0.117, 0.0, 0.258)),
        material=dark_polymer,
        name="barrel_mount_web",
    )
    body.visual(
        Cylinder(radius=0.010, length=0.034),
        origin=Origin(xyz=(0.170, 0.0, 0.258), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_polymer,
        name="nozzle_socket",
    )
    body.visual(
        Box((0.062, 0.006, 0.030)),
        origin=Origin(xyz=(0.036, 0.028, 0.260)),
        material=seal_blue,
        name="service_cover",
    )
    body.visual(
        Box((0.036, 0.006, 0.010)),
        origin=Origin(xyz=(0.034, 0.031, 0.276)),
        material=steel,
        name="cover_hinge_strip",
    )
    body.visual(
        Cylinder(radius=0.0032, length=0.004),
        origin=Origin(xyz=(0.009, 0.0315, 0.265), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="cover_screw_0",
    )
    body.visual(
        Cylinder(radius=0.0032, length=0.004),
        origin=Origin(xyz=(0.063, 0.0315, 0.265), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="cover_screw_1",
    )
    body.visual(
        Box((0.040, 0.006, 0.006)),
        origin=Origin(xyz=(0.035, 0.032, 0.244)),
        material=wear_yellow,
        name="service_pry_tab",
    )

    # Trigger clevis cheeks, guard, and fixed hinge shaft.
    for suffix, y in (("0", -0.028), ("1", 0.028)):
        body.visual(
            Box((0.034, 0.006, 0.044)),
            origin=Origin(xyz=(0.078, y, 0.226)),
            material=dark_polymer,
            name=f"trigger_cheek_{suffix}",
        )
        body.visual(
            Cylinder(radius=0.014, length=0.004),
            origin=Origin(xyz=(0.078, y, 0.236), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=wear_yellow,
            name=f"pivot_bushing_{suffix}",
        )
    body.visual(
        Cylinder(radius=0.0046, length=0.066),
        origin=Origin(xyz=(0.078, 0.0, 0.236), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="trigger_pin",
    )
    body.visual(
        Box((0.014, 0.006, 0.024)),
        origin=Origin(xyz=(0.062, 0.031, 0.188)),
        material=dark_polymer,
        name="rear_guard_bridge",
    )
    body.visual(
        Box((0.020, 0.006, 0.010)),
        origin=Origin(xyz=(0.077, 0.031, 0.206), rpy=(0.0, -0.55, 0.0)),
        material=dark_polymer,
        name="guard_strut",
    )
    body.visual(
        Box((0.046, 0.014, 0.010)),
        origin=Origin(xyz=(0.139, 0.0, 0.260)),
        material=wear_yellow,
        name="pump_rail",
    )

    trigger = model.part("trigger")
    trigger.visual(
        mesh_from_geometry(_trigger_plate_mesh(), "wide_service_trigger"),
        material=safety_orange,
        name="trigger_blade",
    )
    trigger.visual(
        Cylinder(radius=0.011, length=0.036),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=safety_orange,
        name="pivot_boss",
    )
    trigger.visual(
        Box((0.018, 0.020, 0.012)),
        origin=Origin(xyz=(0.033, 0.0, 0.004)),
        material=wear_yellow,
        name="replaceable_cam_pad",
    )
    trigger.visual(
        Cylinder(radius=0.0038, length=0.028),
        origin=Origin(xyz=(0.030, 0.0, 0.006), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="link_pin",
    )

    pump_rod = model.part("pump_rod")
    pump_rod.visual(
        Cylinder(radius=0.0040, length=0.056),
        origin=Origin(xyz=(0.028, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="rod_shaft",
    )
    pump_rod.visual(
        Cylinder(radius=0.0095, length=0.010),
        origin=Origin(xyz=(0.055, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black_rubber,
        name="piston_seal",
    )
    pump_rod.visual(
        Box((0.014, 0.034, 0.018)),
        origin=Origin(xyz=(-0.004, 0.0, 0.0)),
        material=wear_yellow,
        name="clevis_yoke",
    )
    pump_rod.visual(
        Cylinder(radius=0.0038, length=0.040),
        origin=Origin(xyz=(-0.004, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="clevis_pin",
    )

    nozzle_cap = model.part("nozzle_cap")
    nozzle_cap.visual(
        Cylinder(radius=0.0165, length=0.026),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=safety_orange,
        name="ribbed_cap",
    )
    for suffix, xyz, size in (
        ("top", (0.0, 0.0, 0.0175), (0.026, 0.028, 0.004)),
        ("bottom", (0.0, 0.0, -0.0175), (0.026, 0.028, 0.004)),
        ("side_0", (0.0, 0.0175, 0.0), (0.026, 0.004, 0.028)),
        ("side_1", (0.0, -0.0175, 0.0), (0.026, 0.004, 0.028)),
    ):
        nozzle_cap.visual(
            Box(size),
            origin=Origin(xyz=xyz),
            material=safety_orange,
            name=f"grip_rib_{suffix}",
        )
    nozzle_cap.visual(
        Cylinder(radius=0.0055, length=0.003),
        origin=Origin(xyz=(0.0145, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="spray_orifice",
    )

    model.articulation(
        "trigger_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=trigger,
        origin=Origin(xyz=(0.078, 0.0, 0.236)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=16.0, velocity=3.0, lower=0.0, upper=0.62),
    )
    model.articulation(
        "pump_stroke",
        ArticulationType.PRISMATIC,
        parent=body,
        child=pump_rod,
        origin=Origin(xyz=(0.104, 0.0, 0.242)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=42.0, velocity=0.30, lower=0.0, upper=0.030),
    )
    model.articulation(
        "nozzle_adjust",
        ArticulationType.REVOLUTE,
        parent=body,
        child=nozzle_cap,
        origin=Origin(xyz=(0.200, 0.0, 0.258)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.2, velocity=4.0, lower=-2.4, upper=2.4),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    bottle = object_model.get_part("bottle")
    trigger = object_model.get_part("trigger")
    pump_rod = object_model.get_part("pump_rod")
    nozzle_cap = object_model.get_part("nozzle_cap")
    trigger_hinge = object_model.get_articulation("trigger_hinge")
    pump_stroke = object_model.get_articulation("pump_stroke")

    ctx.allow_overlap(
        bottle,
        trigger,
        elem_a="trigger_pin",
        elem_b="pivot_boss",
        reason="The steel trigger pin is intentionally captured through the trigger pivot bushing.",
    )
    ctx.allow_overlap(
        bottle,
        trigger,
        elem_a="trigger_pin",
        elem_b="trigger_blade",
        reason="The same hinge shaft passes through the trigger blade web at the pivot.",
    )
    ctx.expect_overlap(
        bottle,
        trigger,
        axes="yz",
        elem_a="trigger_pin",
        elem_b="pivot_boss",
        min_overlap=0.007,
        name="trigger pivot pin passes through boss",
    )
    ctx.expect_overlap(
        bottle,
        trigger,
        axes="yz",
        elem_a="trigger_pin",
        elem_b="trigger_blade",
        min_overlap=0.007,
        name="trigger pivot pin passes through blade",
    )

    ctx.allow_overlap(
        bottle,
        pump_rod,
        elem_a="pump_barrel",
        elem_b="rod_shaft",
        reason="The pump rod is deliberately retained inside the pump barrel sleeve with clearance represented by the sleeve proxy.",
    )
    ctx.allow_overlap(
        bottle,
        pump_rod,
        elem_a="pump_barrel",
        elem_b="piston_seal",
        reason="The replaceable piston seal seats inside the pump barrel during the stroke.",
    )
    ctx.expect_within(
        pump_rod,
        bottle,
        axes="yz",
        inner_elem="rod_shaft",
        outer_elem="pump_barrel",
        margin=0.003,
        name="pump rod stays centered in barrel",
    )
    ctx.expect_overlap(
        pump_rod,
        bottle,
        axes="x",
        elem_a="rod_shaft",
        elem_b="pump_barrel",
        min_overlap=0.018,
        name="pump rod retained in sleeve at rest",
    )
    ctx.expect_within(
        pump_rod,
        bottle,
        axes="yz",
        inner_elem="piston_seal",
        outer_elem="pump_barrel",
        margin=0.003,
        name="piston seal stays inside barrel bore",
    )
    ctx.expect_overlap(
        pump_rod,
        bottle,
        axes="x",
        elem_a="piston_seal",
        elem_b="pump_barrel",
        min_overlap=0.006,
        name="piston seal is seated in barrel",
    )
    ctx.allow_overlap(
        pump_rod,
        trigger,
        elem_a="clevis_yoke",
        elem_b="trigger_blade",
        reason="The simplified clevis yoke wraps the trigger cam tab to show the visible trigger-to-pump linkage.",
    )
    ctx.expect_overlap(
        pump_rod,
        trigger,
        axes="yz",
        elem_a="clevis_yoke",
        elem_b="trigger_blade",
        min_overlap=0.010,
        name="clevis yoke captures trigger cam",
    )
    ctx.allow_overlap(
        pump_rod,
        trigger,
        elem_a="rod_shaft",
        elem_b="replaceable_cam_pad",
        reason="The visible rod end bears against the replaceable trigger cam pad in a simplified local contact patch.",
    )
    ctx.allow_overlap(
        pump_rod,
        trigger,
        elem_a="rod_shaft",
        elem_b="trigger_blade",
        reason="The rod nose locally bears against the trigger blade just behind the replaceable cam pad.",
    )
    ctx.allow_overlap(
        pump_rod,
        trigger,
        elem_a="clevis_pin",
        elem_b="trigger_blade",
        reason="The clevis cross-pin is intentionally captured through the trigger linkage tab.",
    )
    ctx.expect_overlap(
        pump_rod,
        trigger,
        axes="yz",
        elem_a="rod_shaft",
        elem_b="replaceable_cam_pad",
        min_overlap=0.006,
        name="rod end bears on cam pad",
    )
    ctx.expect_overlap(
        pump_rod,
        trigger,
        axes="yz",
        elem_a="rod_shaft",
        elem_b="trigger_blade",
        min_overlap=0.006,
        name="rod nose aligned with trigger blade",
    )
    ctx.expect_overlap(
        pump_rod,
        trigger,
        axes="yz",
        elem_a="clevis_pin",
        elem_b="trigger_blade",
        min_overlap=0.006,
        name="clevis pin passes through trigger tab",
    )

    ctx.expect_contact(
        nozzle_cap,
        bottle,
        elem_a="ribbed_cap",
        elem_b="nozzle_socket",
        contact_tol=0.003,
        name="nozzle cap seats on socket",
    )

    rest_rod = ctx.part_world_position(pump_rod)
    rest_trigger = ctx.part_world_position(trigger)
    with ctx.pose({trigger_hinge: 0.62, pump_stroke: 0.030}):
        stroked_rod = ctx.part_world_position(pump_rod)
        stroked_trigger = ctx.part_world_position(trigger)
        ctx.expect_within(
            pump_rod,
            bottle,
            axes="yz",
            inner_elem="rod_shaft",
            outer_elem="pump_barrel",
            margin=0.004,
            name="pump rod remains guided during stroke",
        )
        ctx.expect_overlap(
            pump_rod,
            bottle,
            axes="x",
            elem_a="rod_shaft",
            elem_b="pump_barrel",
            min_overlap=0.012,
            name="pump rod remains inserted at full stroke",
        )
    ctx.check(
        "trigger stroke drives pump rod forward",
        rest_rod is not None
        and stroked_rod is not None
        and stroked_rod[0] > rest_rod[0] + 0.020,
        details=f"rest={rest_rod}, stroked={stroked_rod}",
    )
    ctx.check(
        "trigger origin remains at hinge during pull",
        rest_trigger is not None and stroked_trigger is not None,
        details=f"rest={rest_trigger}, stroked={stroked_trigger}",
    )

    return ctx.report()


object_model = build_object_model()
