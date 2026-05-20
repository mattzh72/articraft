from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _add_box(part, name: str, size, xyz, material: str) -> None:
    part.visual(Box(size), origin=Origin(xyz=xyz), material=material, name=name)


def _blade_mesh():
    # Thin utility-blade plate: a long trapezoid with a sharpened forward point,
    # authored in the local X/Z side profile and extruded as a 1.4 mm steel sheet.
    profile_xz = [
        (0.040, -0.0045),
        (0.112, -0.0045),
        (0.160, -0.0015),
        (0.176, 0.0000),
        (0.112, 0.0050),
        (0.040, 0.0040),
    ]
    blade = ExtrudeGeometry.centered(profile_xz, 0.0014, cap=True, closed=True)
    blade.rotate_x(math.pi / 2.0)
    return mesh_from_geometry(blade, "utility_blade")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="auto_retract_safety_knife")

    model.material("charcoal_polymer", rgba=(0.035, 0.040, 0.045, 1.0))
    model.material("rubber_grip", rgba=(0.005, 0.006, 0.007, 1.0))
    model.material("safety_orange", rgba=(1.0, 0.30, 0.035, 1.0))
    model.material("warm_guard", rgba=(0.95, 0.62, 0.18, 0.92))
    model.material("brushed_steel", rgba=(0.72, 0.74, 0.73, 1.0))
    model.material("dark_rail", rgba=(0.014, 0.016, 0.018, 1.0))

    handle_shell = model.part("handle_shell")
    # Straight molded shell with an open top slot, one internal blade channel,
    # and a separate captured T-slot guide under the nose for the guard.
    _add_box(handle_shell, "rear_bottom", (0.172, 0.040, 0.006), (-0.034, 0.0, -0.012), "charcoal_polymer")
    _add_box(handle_shell, "front_bottom_0", (0.078, 0.014, 0.006), (0.083, -0.013, -0.012), "charcoal_polymer")
    _add_box(handle_shell, "front_bottom_1", (0.078, 0.014, 0.006), (0.083, 0.013, -0.012), "charcoal_polymer")
    _add_box(handle_shell, "side_wall_0", (0.240, 0.006, 0.026), (0.0, -0.020, 0.000), "charcoal_polymer")
    _add_box(handle_shell, "side_wall_1", (0.240, 0.006, 0.026), (0.0, 0.020, 0.000), "charcoal_polymer")
    _add_box(handle_shell, "rear_top_bridge", (0.052, 0.040, 0.006), (-0.094, 0.0, 0.014), "charcoal_polymer")
    _add_box(handle_shell, "front_top_bridge", (0.060, 0.040, 0.006), (0.090, 0.0, 0.014), "charcoal_polymer")
    _add_box(handle_shell, "rear_end_cap", (0.008, 0.040, 0.026), (-0.120, 0.0, 0.000), "charcoal_polymer")
    _add_box(handle_shell, "front_nose_frame", (0.006, 0.040, 0.020), (0.120, 0.0, 0.001), "charcoal_polymer")

    for x in (-0.075, -0.045, -0.015, 0.015):
        _add_box(handle_shell, f"grip_rib_0_{x:+.3f}", (0.012, 0.003, 0.018), (x, -0.024, -0.001), "rubber_grip")
        _add_box(handle_shell, f"grip_rib_1_{x:+.3f}", (0.012, 0.003, 0.018), (x, 0.024, -0.001), "rubber_grip")

    _add_box(handle_shell, "blade_rail_0", (0.188, 0.011, 0.004), (0.010, -0.0125, -0.003), "dark_rail")
    _add_box(handle_shell, "blade_rail_1", (0.188, 0.011, 0.004), (0.010, 0.0125, -0.003), "dark_rail")

    _add_box(handle_shell, "guard_rail_0", (0.086, 0.006, 0.009), (0.078, -0.0115, -0.018), "dark_rail")
    _add_box(handle_shell, "guard_rail_1", (0.086, 0.006, 0.009), (0.078, 0.0115, -0.018), "dark_rail")
    _add_box(handle_shell, "guide_lip_0", (0.086, 0.005, 0.003), (0.078, -0.0065, -0.016), "dark_rail")
    _add_box(handle_shell, "guide_lip_1", (0.086, 0.005, 0.003), (0.078, 0.0065, -0.016), "dark_rail")

    blade_carrier = model.part("blade_carrier")
    _add_box(blade_carrier, "carrier_bar", (0.125, 0.012, 0.006), (0.028, 0.0, -0.003), "brushed_steel")
    blade_carrier.visual(_blade_mesh(), origin=Origin(), material="brushed_steel", name="blade_plate")
    _add_box(blade_carrier, "slider_stem", (0.020, 0.006, 0.023), (0.010, 0.0, 0.008), "safety_orange")
    _add_box(blade_carrier, "top_slider", (0.038, 0.026, 0.008), (0.010, 0.0, 0.0225), "safety_orange")
    for i, x in enumerate((-0.004, 0.006, 0.016, 0.026)):
        _add_box(blade_carrier, f"slider_grip_{i}", (0.004, 0.022, 0.004), (x, 0.0, 0.026), "rubber_grip")

    nose_guard = model.part("nose_guard")
    _add_box(nose_guard, "guide_foot", (0.050, 0.014, 0.004), (-0.020, 0.0, -0.022), "warm_guard")
    _add_box(nose_guard, "guide_neck", (0.050, 0.006, 0.006), (-0.020, 0.0, -0.019), "warm_guard")
    _add_box(nose_guard, "tongue_bridge", (0.024, 0.006, 0.006), (0.006, 0.0, -0.019), "warm_guard")
    _add_box(nose_guard, "guard_stem", (0.012, 0.006, 0.026), (0.018, 0.0, -0.009), "warm_guard")
    _add_box(nose_guard, "guard_side_0", (0.042, 0.004, 0.022), (0.034, -0.011, 0.000), "warm_guard")
    _add_box(nose_guard, "guard_side_1", (0.042, 0.004, 0.022), (0.034, 0.011, 0.000), "warm_guard")
    _add_box(nose_guard, "guard_lower_bar", (0.044, 0.026, 0.004), (0.036, 0.0, -0.010), "warm_guard")
    _add_box(nose_guard, "guard_upper_brow", (0.044, 0.026, 0.004), (0.036, 0.0, 0.010), "warm_guard")
    _add_box(nose_guard, "front_guard_bow", (0.006, 0.026, 0.022), (0.056, 0.0, 0.000), "warm_guard")

    model.articulation(
        "blade_slide",
        ArticulationType.PRISMATIC,
        parent=handle_shell,
        child=blade_carrier,
        origin=Origin(xyz=(-0.045, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=0.45, lower=0.0, upper=0.035),
    )

    model.articulation(
        "guard_slide",
        ArticulationType.PRISMATIC,
        parent=handle_shell,
        child=nose_guard,
        origin=Origin(xyz=(0.100, 0.0, 0.0)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=0.20, lower=0.0, upper=0.018),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    handle = object_model.get_part("handle_shell")
    carrier = object_model.get_part("blade_carrier")
    guard = object_model.get_part("nose_guard")
    blade_slide = object_model.get_articulation("blade_slide")
    guard_slide = object_model.get_articulation("guard_slide")

    ctx.check(
        "blade carrier uses a prismatic slide",
        blade_slide.articulation_type == ArticulationType.PRISMATIC,
        details=f"type={blade_slide.articulation_type}",
    )
    ctx.check(
        "nose guard uses an independent prismatic slide",
        guard_slide.articulation_type == ArticulationType.PRISMATIC,
        details=f"type={guard_slide.articulation_type}",
    )

    ctx.expect_within(
        carrier,
        handle,
        axes="y",
        inner_elem="carrier_bar",
        outer_elem="rear_bottom",
        margin=0.001,
        name="blade carrier is centered in the main channel",
    )
    ctx.expect_overlap(
        carrier,
        handle,
        axes="x",
        elem_a="carrier_bar",
        elem_b="blade_rail_0",
        min_overlap=0.085,
        name="collapsed blade carrier remains retained by handle rail",
    )
    ctx.expect_gap(
        carrier,
        handle,
        axis="z",
        positive_elem="top_slider",
        negative_elem="front_top_bridge",
        min_gap=0.001,
        name="thumb slider sits proud above the handle shell",
    )

    ctx.expect_overlap(
        guard,
        handle,
        axes="x",
        elem_a="guide_foot",
        elem_b="guide_lip_0",
        min_overlap=0.045,
        name="nose guard foot is retained in the front guide",
    )
    ctx.expect_gap(
        handle,
        guard,
        axis="z",
        positive_elem="guide_lip_0",
        negative_elem="guide_foot",
        min_gap=0.0015,
        max_gap=0.006,
        name="guide lip captures the guard foot from above",
    )
    ctx.expect_overlap(
        guard,
        handle,
        axes="y",
        elem_a="guide_foot",
        elem_b="guide_lip_0",
        min_overlap=0.0015,
        name="guard foot tucks under the side lip",
    )

    carrier_rest = ctx.part_world_position(carrier)
    guard_rest = ctx.part_world_position(guard)
    with ctx.pose({blade_slide: 0.035, guard_slide: 0.018}):
        ctx.expect_overlap(
            carrier,
            handle,
            axes="x",
            elem_a="carrier_bar",
            elem_b="blade_rail_0",
            min_overlap=0.050,
            name="extended carrier keeps hidden insertion in the handle",
        )
        ctx.expect_overlap(
            guard,
            handle,
            axes="x",
            elem_a="guide_foot",
            elem_b="guide_lip_0",
            min_overlap=0.040,
            name="slid nose guard stays clipped in the front guide",
        )
        carrier_extended = ctx.part_world_position(carrier)
        guard_retracted = ctx.part_world_position(guard)

    ctx.check(
        "blade slide moves the carrier forward",
        carrier_rest is not None
        and carrier_extended is not None
        and carrier_extended[0] > carrier_rest[0] + 0.030,
        details=f"rest={carrier_rest}, extended={carrier_extended}",
    )
    ctx.check(
        "nose guard slide retracts on its short guide",
        guard_rest is not None
        and guard_retracted is not None
        and guard_retracted[0] < guard_rest[0] - 0.014,
        details=f"rest={guard_rest}, retracted={guard_retracted}",
    )

    return ctx.report()


object_model = build_object_model()
