from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
)


def _cylinder_rpy_between(
    p0: tuple[float, float, float], p1: tuple[float, float, float]
) -> tuple[tuple[float, float, float], tuple[float, float, float], float]:
    """Return midpoint, rpy, and length for a local-Z cylinder spanning two points."""
    dx = p1[0] - p0[0]
    dy = p1[1] - p0[1]
    dz = p1[2] - p0[2]
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if length <= 0.0:
        raise ValueError("zero length cylinder")
    ux, uy, uz = dx / length, dy / length, dz / length
    pitch = math.atan2(math.sqrt(ux * ux + uy * uy), uz)
    yaw = math.atan2(uy, ux)
    midpoint = ((p0[0] + p1[0]) * 0.5, (p0[1] + p1[1]) * 0.5, (p0[2] + p1[2]) * 0.5)
    return midpoint, (0.0, pitch, yaw), length


def _tube(part, p0, p1, radius, *, material, name):
    midpoint, rpy, length = _cylinder_rpy_between(p0, p1)
    part.visual(Cylinder(radius=radius, length=length), origin=Origin(xyz=midpoint, rpy=rpy), material=material, name=name)


def _box(part, size, xyz, *, material, name, rpy=(0.0, 0.0, 0.0)):
    part.visual(Box(size), origin=Origin(xyz=xyz, rpy=rpy), material=material, name=name)


def _y_cylinder(part, radius, length, xyz, *, material, name):
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def _x_cylinder(part, radius, length, xyz, *, material, name):
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(0.0, math.pi / 2.0, 0.0)),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="retrofit_legacy_exercise_bike")

    frame_green = model.material("aged_green_enamel", color=(0.05, 0.22, 0.16, 1.0))
    cream = model.material("warm_cream_housing", color=(0.82, 0.75, 0.58, 1.0))
    dark = model.material("blackened_steel", color=(0.02, 0.023, 0.022, 1.0))
    rubber = model.material("matte_black_rubber", color=(0.005, 0.005, 0.005, 1.0))
    chrome = model.material("brushed_chrome", color=(0.78, 0.78, 0.73, 1.0))
    leather = model.material("aged_brown_leather", color=(0.32, 0.16, 0.07, 1.0))
    brass = model.material("dull_brass_fasteners", color=(0.72, 0.55, 0.23, 1.0))
    red = model.material("oxide_red_service_paint", color=(0.55, 0.09, 0.05, 1.0))

    frame = model.part("frame")

    # Floor stabilizers and old round-tube base.
    _y_cylinder(frame, 0.040, 0.74, (-0.90, 0.0, 0.055), material=frame_green, name="front_floor_tube")
    _y_cylinder(frame, 0.040, 0.74, (0.66, 0.0, 0.055), material=frame_green, name="rear_floor_tube")
    _tube(frame, (-0.92, 0.0, 0.090), (0.66, 0.0, 0.090), 0.030, material=frame_green, name="base_spine")
    for x, prefix in [(-0.90, "front"), (0.66, "rear")]:
        for y, suffix in [(-0.41, "foot_0"), (0.41, "foot_1")]:
            _box(frame, (0.18, 0.090, 0.035), (x, y * 0.90, 0.030), material=rubber, name=f"{prefix}_{suffix}")

    # Enclosed flywheel housing with layered cover plates and an explicit crank bearing.
    _y_cylinder(frame, 0.355, 0.175, (-0.43, 0.0, 0.455), material=dark, name="flywheel_outer_rim")
    _y_cylinder(frame, 0.330, 0.198, (-0.43, 0.0, 0.455), material=cream, name="flywheel_housing")
    _y_cylinder(frame, 0.295, 0.026, (-0.43, -0.114, 0.455), material=cream, name="near_side_cover")
    _y_cylinder(frame, 0.295, 0.030, (-0.43, 0.106, 0.455), material=cream, name="far_side_cover")
    _y_cylinder(frame, 0.085, 0.128, (-0.39, -0.151, 0.430), material=dark, name="crank_bearing_boss")
    _box(frame, (0.130, 0.022, 0.020), (-0.390, -0.216, 0.510), material=brass, name="bearing_retainer_top")
    _box(frame, (0.130, 0.022, 0.020), (-0.390, -0.216, 0.350), material=brass, name="bearing_retainer_bottom")
    _box(frame, (0.020, 0.022, 0.130), (-0.470, -0.216, 0.430), material=brass, name="bearing_retainer_side_0")
    _box(frame, (0.020, 0.022, 0.130), (-0.310, -0.216, 0.430), material=brass, name="bearing_retainer_side_1")
    _box(frame, (0.19, 0.014, 0.13), (-0.39, -0.118, 0.430), material=dark, name="near_crank_adapter_plate")
    _box(frame, (0.19, 0.020, 0.13), (-0.39, 0.104, 0.430), material=dark, name="far_crank_adapter_plate")

    # Cover fasteners and service bosses on the flywheel side plates.
    for i, angle in enumerate([0.25, 1.15, 2.15, 3.15, 4.15, 5.20]):
        x = -0.43 + 0.265 * math.cos(angle)
        z = 0.455 + 0.265 * math.sin(angle)
        _y_cylinder(frame, 0.011, 0.020, (x, -0.125, z), material=brass, name=f"cover_bolt_{i}")
    _box(frame, (0.245, 0.014, 0.190), (-0.620, -0.133, 0.570), material=dark, name="hatch_seat_frame")
    _box(frame, (0.205, 0.010, 0.150), (-0.620, -0.142, 0.570), material=cream, name="hatch_shadow_recess")
    for z in (0.510, 0.630):
        _box(frame, (0.034, 0.024, 0.038), (-0.750, -0.120, z), material=dark, name=f"hatch_hinge_lug_{int(z*1000)}")
    _box(frame, (0.040, 0.030, 0.040), (-0.500, -0.142, 0.570), material=dark, name="hatch_latch_keeper")

    # Structural retrofit frame: visible tubes meet gussets/adapters instead of floating into covers.
    _tube(frame, (-0.45, 0.0, 0.115), (-0.39, 0.0, 0.430), 0.034, material=frame_green, name="front_down_tube")
    _tube(frame, (-0.39, 0.0, 0.430), (0.392, 0.075, 0.650), 0.034, material=frame_green, name="top_frame_tube")
    _tube(frame, (0.58, 0.0, 0.085), (0.392, 0.075, 0.635), 0.032, material=frame_green, name="rear_stay_tube")
    _tube(frame, (-0.39, 0.0, 0.430), (-0.827, 0.075, 0.795), 0.032, material=frame_green, name="front_mast_tube")
    _tube(frame, (-0.43, 0.0, 0.780), (-0.827, 0.075, 0.910), 0.027, material=frame_green, name="housing_top_brace")
    _box(frame, (0.25, 0.170, 0.070), (-0.390, 0.0, 0.430), material=dark, name="crank_node_gusset")
    _box(frame, (0.175, 0.100, 0.050), (0.392, 0.080, 0.570), material=dark, name="seat_node_gusset")
    _box(frame, (0.160, 0.095, 0.050), (-0.827, 0.080, 0.745), material=dark, name="bar_node_gusset")

    # Seat post sleeve: four actual walls leave a square sliding bore.
    for dx, name in [(-0.039, "seat_sleeve_wall_0"), (0.039, "seat_sleeve_wall_1")]:
        _box(frame, (0.014, 0.090, 0.265), (0.330 + dx, 0.0, 0.715), material=frame_green, name=name)
    for dy, name in [(-0.039, "seat_sleeve_wall_2"), (0.039, "seat_sleeve_wall_3")]:
        _box(frame, (0.090, 0.014, 0.265), (0.330, dy, 0.715), material=frame_green, name=name)
    for dx, name in [(-0.050, "seat_clamp_side_0"), (0.050, "seat_clamp_side_1")]:
        _box(frame, (0.018, 0.110, 0.060), (0.330 + dx, 0.0, 0.842), material=dark, name=name)
    _box(frame, (0.116, 0.018, 0.060), (0.330, 0.050, 0.842), material=dark, name="seat_clamp_back")
    _box(frame, (0.038, 0.055, 0.090), (0.282, -0.068, 0.842), material=dark, name="seat_clamp_ear_0")
    _box(frame, (0.038, 0.055, 0.090), (0.378, -0.068, 0.842), material=dark, name="seat_clamp_ear_1")
    _x_cylinder(frame, 0.012, 0.140, (0.330, -0.067, 0.888), material=brass, name="seat_clamp_cross_bolt")

    # Handlebar post sleeve and clamp at the head of the frame.
    for dx, name in [(-0.039, "bar_sleeve_wall_0"), (0.039, "bar_sleeve_wall_1")]:
        _box(frame, (0.014, 0.090, 0.280), (-0.765 + dx, 0.0, 0.865), material=frame_green, name=name)
    for dy, name in [(-0.039, "bar_sleeve_wall_2"), (0.039, "bar_sleeve_wall_3")]:
        _box(frame, (0.090, 0.014, 0.280), (-0.765, dy, 0.865), material=frame_green, name=name)
    for dx, name in [(-0.050, "bar_clamp_side_0"), (0.050, "bar_clamp_side_1")]:
        _box(frame, (0.018, 0.112, 0.060), (-0.765 + dx, 0.0, 1.004), material=dark, name=name)
    _box(frame, (0.116, 0.018, 0.060), (-0.765, 0.050, 1.004), material=dark, name="bar_clamp_back")
    _box(frame, (0.038, 0.055, 0.050), (-0.813, -0.068, 1.004), material=dark, name="bar_clamp_ear_0")
    _box(frame, (0.038, 0.055, 0.050), (-0.717, -0.068, 1.004), material=dark, name="bar_clamp_ear_1")
    _x_cylinder(frame, 0.012, 0.140, (-0.765, -0.067, 1.034), material=brass, name="bar_clamp_cross_bolt")

    # Resistance adjuster bracket and threaded bearing boss.
    _tube(frame, (-0.130, 0.0, 0.645), (-0.195, 0.0, 0.815), 0.021, material=frame_green, name="resistance_knob_stem_brace")
    _box(frame, (0.110, 0.080, 0.022), (-0.195, 0.0, 0.820), material=dark, name="resistance_boss_plate")
    frame.visual(Cylinder(radius=0.020, length=0.070), origin=Origin(xyz=(-0.195, 0.0, 0.855)), material=brass, name="resistance_threaded_boss")

    # Rotating crank assembly captured by the explicit side bearing.
    crank = model.part("crankset")
    _y_cylinder(crank, 0.055, 0.060, (0.0, -0.040, 0.0), material=chrome, name="crank_hub")
    _y_cylinder(crank, 0.032, 0.120, (0.0, -0.020, 0.0), material=chrome, name="axle_stub")
    _box(crank, (0.035, 0.022, 0.330), (0.0, -0.060, -0.180), material=chrome, name="crank_arm")
    _box(crank, (0.070, 0.030, 0.040), (0.0, -0.060, -0.355), material=chrome, name="pedal_spindle_block")
    _box(crank, (0.180, 0.082, 0.035), (0.0, -0.105, -0.382), material=rubber, name="pedal_pad")
    model.articulation(
        "frame_to_crankset",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=crank,
        origin=Origin(xyz=(-0.390, -0.225, 0.430)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=10.0),
    )

    # Height-adjustable saddle post with retained insertion in its square sleeve.
    saddle = model.part("saddle_post")
    _box(saddle, (0.052, 0.052, 0.500), (0.0, 0.0, 0.020), material=chrome, name="seat_post_tube")
    _x_cylinder(saddle, 0.008, 0.220, (0.020, 0.0, 0.290), material=chrome, name="saddle_rail_0")
    _x_cylinder(saddle, 0.008, 0.220, (0.020, 0.052, 0.290), material=chrome, name="saddle_rail_1")
    _box(saddle, (0.210, 0.145, 0.046), (0.055, 0.0, 0.355), material=leather, name="saddle_rear_pad")
    _box(saddle, (0.185, 0.080, 0.040), (-0.095, 0.0, 0.345), material=leather, name="saddle_nose")
    _box(saddle, (0.078, 0.164, 0.075), (0.018, 0.0, 0.292), material=dark, name="saddle_adapter_clamp")
    for dz, name in [(-0.095, "seat_index_hole_0"), (-0.040, "seat_index_hole_1"), (0.015, "seat_index_hole_2")]:
        _y_cylinder(saddle, 0.009, 0.010, (0.0, -0.026, dz), material=dark, name=name)
    model.articulation(
        "frame_to_saddle_post",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=saddle,
        origin=Origin(xyz=(0.330, 0.0, 0.840)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.25, lower=0.0, upper=0.160),
    )

    # Pull-to-release saddle index pin, mounted in the clamp ears.
    seat_pin = model.part("saddle_pin")
    _y_cylinder(seat_pin, 0.010, 0.110, (0.0, 0.033, 0.0), material=chrome, name="pin_shaft")
    _y_cylinder(seat_pin, 0.034, 0.030, (0.0, -0.032, 0.0), material=dark, name="knob_cap")
    seat_pin.visual(Sphere(radius=0.022), origin=Origin(xyz=(0.0, -0.055, 0.0)), material=rubber, name="knob_bulb")
    model.articulation(
        "frame_to_saddle_pin",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=seat_pin,
        origin=Origin(xyz=(0.330, -0.077, 0.855)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=0.15, lower=0.0, upper=0.035),
    )

    # Height-adjustable handlebar post with supported retro loop bars.
    bars = model.part("handlebar_post")
    _box(bars, (0.052, 0.052, 0.480), (0.0, 0.0, 0.040), material=chrome, name="bar_post_tube")
    _tube(bars, (0.0, 0.0, 0.265), (-0.150, 0.0, 0.405), 0.018, material=chrome, name="bar_forward_stem")
    _y_cylinder(bars, 0.018, 0.520, (-0.150, 0.0, 0.405), material=chrome, name="handlebar_cross_tube")
    _tube(bars, (-0.150, -0.260, 0.405), (-0.205, -0.330, 0.365), 0.017, material=chrome, name="grip_bend_0")
    _tube(bars, (-0.150, 0.260, 0.405), (-0.205, 0.330, 0.365), 0.017, material=chrome, name="grip_bend_1")
    _y_cylinder(bars, 0.024, 0.125, (-0.205, -0.385, 0.365), material=rubber, name="grip_0")
    _y_cylinder(bars, 0.024, 0.125, (-0.205, 0.385, 0.365), material=rubber, name="grip_1")
    _box(bars, (0.080, 0.155, 0.070), (0.0, 0.0, 0.278), material=dark, name="bar_adapter_clamp")
    for dz, name in [(-0.070, "bar_index_hole_0"), (-0.015, "bar_index_hole_1"), (0.040, "bar_index_hole_2")]:
        _y_cylinder(bars, 0.009, 0.010, (0.0, -0.026, dz), material=dark, name=name)
    model.articulation(
        "frame_to_handlebar_post",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=bars,
        origin=Origin(xyz=(-0.765, 0.0, 1.005)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.22, lower=0.0, upper=0.120),
    )

    bar_pin = model.part("bar_pin")
    _y_cylinder(bar_pin, 0.010, 0.110, (0.0, 0.033, 0.0), material=chrome, name="pin_shaft")
    _y_cylinder(bar_pin, 0.034, 0.030, (0.0, -0.032, 0.0), material=dark, name="knob_cap")
    bar_pin.visual(Sphere(radius=0.022), origin=Origin(xyz=(0.0, -0.055, 0.0)), material=rubber, name="knob_bulb")
    model.articulation(
        "frame_to_bar_pin",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=bar_pin,
        origin=Origin(xyz=(-0.765, -0.077, 1.004)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=0.15, lower=0.0, upper=0.035),
    )

    # Turnable resistance knob on a threaded boss.
    knob = model.part("resistance_knob")
    knob.visual(Cylinder(radius=0.055, length=0.030), origin=Origin(xyz=(0.0, 0.0, 0.030)), material=dark, name="knob_cap")
    knob.visual(Cylinder(radius=0.018, length=0.040), origin=Origin(xyz=(0.0, 0.0, 0.000)), material=brass, name="knob_threaded_stem")
    for i, angle in enumerate([0.0, math.pi / 2.0, math.pi, 3.0 * math.pi / 2.0]):
        _box(knob, (0.022, 0.009, 0.018), (0.045 * math.cos(angle), 0.045 * math.sin(angle), 0.046), material=rubber, name=f"knob_grip_rib_{i}", rpy=(0.0, 0.0, angle))
    model.articulation(
        "frame_to_resistance_knob",
        ArticulationType.CONTINUOUS,
        parent=frame,
        child=knob,
        origin=Origin(xyz=(-0.195, 0.0, 0.890)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=5.0, velocity=3.0),
    )

    # Hinged service hatch, with separate knuckles and pull tab.
    hatch = model.part("service_hatch")
    _box(hatch, (0.205, 0.012, 0.150), (0.102, 0.0, 0.0), material=red, name="hatch_panel")
    _box(hatch, (0.048, 0.020, 0.070), (0.165, -0.016, 0.0), material=dark, name="hatch_pull_tab")
    hatch.visual(Cylinder(radius=0.012, length=0.052), origin=Origin(xyz=(0.0, -0.002, -0.048)), material=chrome, name="hinge_barrel_0")
    hatch.visual(Cylinder(radius=0.012, length=0.052), origin=Origin(xyz=(0.0, -0.002, 0.048)), material=chrome, name="hinge_barrel_1")
    for x, z, n in [(0.040, 0.055, "hatch_bolt_0"), (0.155, 0.055, "hatch_bolt_1"), (0.040, -0.055, "hatch_bolt_2"), (0.155, -0.055, "hatch_bolt_3")]:
        hatch.visual(Cylinder(radius=0.008, length=0.008), origin=Origin(xyz=(x, -0.005, z), rpy=(math.pi / 2.0, 0.0, 0.0)), material=brass, name=n)
    model.articulation(
        "frame_to_service_hatch",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=hatch,
        origin=Origin(xyz=(-0.750, -0.151, 0.570)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.2, lower=0.0, upper=1.35),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    crank = object_model.get_part("crankset")
    saddle = object_model.get_part("saddle_post")
    bars = object_model.get_part("handlebar_post")
    seat_pin = object_model.get_part("saddle_pin")
    bar_pin = object_model.get_part("bar_pin")
    knob = object_model.get_part("resistance_knob")
    hatch = object_model.get_part("service_hatch")

    crank_joint = object_model.get_articulation("frame_to_crankset")
    saddle_slide = object_model.get_articulation("frame_to_saddle_post")
    bar_slide = object_model.get_articulation("frame_to_handlebar_post")
    seat_pin_slide = object_model.get_articulation("frame_to_saddle_pin")
    bar_pin_slide = object_model.get_articulation("frame_to_bar_pin")
    knob_joint = object_model.get_articulation("frame_to_resistance_knob")
    hatch_hinge = object_model.get_articulation("frame_to_service_hatch")

    visual_contracts = [
        (frame, "crank_bearing_boss"),
        (frame, "seat_sleeve_wall_0"),
        (frame, "seat_sleeve_wall_2"),
        (frame, "bar_sleeve_wall_0"),
        (frame, "bar_sleeve_wall_2"),
        (crank, "axle_stub"),
        (saddle, "seat_post_tube"),
        (saddle, "seat_index_hole_2"),
        (bars, "bar_post_tube"),
        (seat_pin, "pin_shaft"),
        (bar_pin, "pin_shaft"),
    ]
    missing_visuals = []
    for part_obj, visual_name in visual_contracts:
        try:
            part_obj.get_visual(visual_name)
        except Exception:
            missing_visuals.append(f"{part_obj.name}:{visual_name}")
    ctx.check(
        "exact visual contracts exist",
        not missing_visuals,
        details=", ".join(missing_visuals),
    )

    ctx.allow_overlap(
        frame,
        crank,
        elem_a="crank_bearing_boss",
        elem_b="axle_stub",
        reason="The crank axle is intentionally captured inside the explicit bearing boss.",
    )
    ctx.expect_within(
        crank,
        frame,
        axes="xz",
        inner_elem="axle_stub",
        outer_elem="crank_bearing_boss",
        margin=0.001,
        name="crank axle is centered in bearing boss",
    )
    ctx.expect_overlap(
        crank,
        frame,
        axes="y",
        elem_a="axle_stub",
        elem_b="crank_bearing_boss",
        min_overlap=0.025,
        name="crank axle remains inserted in bearing boss",
    )

    ctx.allow_overlap(
        frame,
        knob,
        elem_a="resistance_threaded_boss",
        elem_b="knob_threaded_stem",
        reason="The resistance knob stem is threaded into the boss it rotates in.",
    )
    ctx.expect_within(
        knob,
        frame,
        axes="xy",
        inner_elem="knob_threaded_stem",
        outer_elem="resistance_threaded_boss",
        margin=0.002,
        name="resistance stem stays in threaded boss",
    )
    ctx.expect_overlap(
        knob,
        frame,
        axes="z",
        elem_a="knob_threaded_stem",
        elem_b="resistance_threaded_boss",
        min_overlap=0.015,
        name="resistance stem has threaded engagement",
    )

    for pin_part, post_part, pin_elem, post_elem, wall_elem, label in [
        (seat_pin, saddle, "pin_shaft", "seat_post_tube", "seat_sleeve_wall_2", "saddle"),
        (bar_pin, bars, "pin_shaft", "bar_post_tube", "bar_sleeve_wall_2", "bar"),
    ]:
        ctx.allow_overlap(
            frame,
            pin_part,
            elem_a=wall_elem,
            elem_b=pin_elem,
            reason=f"The {label} index pin passes through the clamp wall.",
        )
        ctx.allow_overlap(
            pin_part,
            post_part,
            elem_a=pin_elem,
            elem_b=post_elem,
            reason=f"The {label} index pin is intentionally seated in the post hole at rest.",
        )
        ctx.expect_within(
            pin_part,
            post_part,
            axes="xz",
            inner_elem=pin_elem,
            outer_elem=post_elem,
            margin=0.002,
            name=f"{label} pin aligns with post bore",
        )
        ctx.expect_overlap(
            pin_part,
            post_part,
            axes="y",
            elem_a=pin_elem,
            elem_b=post_elem,
            min_overlap=0.015,
            name=f"{label} pin is engaged at rest",
        )

    ctx.allow_overlap(
        seat_pin,
        saddle,
        elem_a="pin_shaft",
        elem_b="seat_index_hole_2",
        reason="The saddle index pin tip occupies the visible indexing hole when locked.",
    )
    ctx.expect_within(
        seat_pin,
        saddle,
        axes="xz",
        inner_elem="pin_shaft",
        outer_elem="seat_index_hole_2",
        margin=0.002,
        name="saddle pin tip centers on index hole",
    )
    ctx.expect_overlap(
        seat_pin,
        saddle,
        axes="y",
        elem_a="pin_shaft",
        elem_b="seat_index_hole_2",
        min_overlap=0.004,
        name="saddle pin tip enters index hole",
    )

    # Prompt-specific articulation checks.
    rest_pedal = ctx.part_element_world_aabb(crank, elem="pedal_pad")
    with ctx.pose({crank_joint: math.pi / 2.0}):
        moved_pedal = ctx.part_element_world_aabb(crank, elem="pedal_pad")
    ctx.check(
        "crankset rotates pedal through circle",
        rest_pedal is not None
        and moved_pedal is not None
        and moved_pedal[0][0] < rest_pedal[0][0] - 0.20
        and moved_pedal[0][2] > rest_pedal[0][2] + 0.20,
        details=f"rest={rest_pedal}, moved={moved_pedal}",
    )

    rest_saddle = ctx.part_world_position(saddle)
    with ctx.pose({saddle_slide: 0.160}):
        raised_saddle = ctx.part_world_position(saddle)
        ctx.expect_overlap(
            saddle,
            frame,
            axes="z",
            elem_a="seat_post_tube",
            elem_b="seat_sleeve_wall_0",
            min_overlap=0.045,
            name="raised saddle post retains insertion",
        )
    ctx.check(
        "saddle post slides upward",
        rest_saddle is not None and raised_saddle is not None and raised_saddle[2] > rest_saddle[2] + 0.140,
        details=f"rest={rest_saddle}, raised={raised_saddle}",
    )

    rest_bars = ctx.part_world_position(bars)
    with ctx.pose({bar_slide: 0.120}):
        raised_bars = ctx.part_world_position(bars)
        ctx.expect_overlap(
            bars,
            frame,
            axes="z",
            elem_a="bar_post_tube",
            elem_b="bar_sleeve_wall_0",
            min_overlap=0.050,
            name="raised bars retain insertion",
        )
    ctx.check(
        "handlebar post slides upward",
        rest_bars is not None and raised_bars is not None and raised_bars[2] > rest_bars[2] + 0.100,
        details=f"rest={rest_bars}, raised={raised_bars}",
    )

    rest_pin = ctx.part_world_position(seat_pin)
    with ctx.pose({seat_pin_slide: 0.035}):
        pulled_pin = ctx.part_world_position(seat_pin)
    ctx.check(
        "saddle pin pulls outward",
        rest_pin is not None and pulled_pin is not None and pulled_pin[1] < rest_pin[1] - 0.025,
        details=f"rest={rest_pin}, pulled={pulled_pin}",
    )

    rest_bar_pin = ctx.part_world_position(bar_pin)
    with ctx.pose({bar_pin_slide: 0.035}):
        pulled_bar_pin = ctx.part_world_position(bar_pin)
    ctx.check(
        "bar pin pulls outward",
        rest_bar_pin is not None and pulled_bar_pin is not None and pulled_bar_pin[1] < rest_bar_pin[1] - 0.025,
        details=f"rest={rest_bar_pin}, pulled={pulled_bar_pin}",
    )

    ctx.check(
        "resistance knob is a continuous turner",
        knob_joint.articulation_type == ArticulationType.CONTINUOUS,
        details=f"type={knob_joint.articulation_type}",
    )

    closed_hatch = ctx.part_element_world_aabb(hatch, elem="hatch_panel")
    with ctx.pose({hatch_hinge: 1.20}):
        open_hatch = ctx.part_element_world_aabb(hatch, elem="hatch_panel")
    ctx.check(
        "service hatch swings outward",
        closed_hatch is not None and open_hatch is not None and open_hatch[0][1] < closed_hatch[0][1] - 0.080,
        details=f"closed={closed_hatch}, open={open_hatch}",
    )

    return ctx.report()


object_model = build_object_model()
