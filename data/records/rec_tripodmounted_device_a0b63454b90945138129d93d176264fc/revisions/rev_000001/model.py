from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobBore,
    KnobGeometry,
    KnobGrip,
    KnobRelief,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TrunnionYokeGeometry,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _cylinder_between(part, start, end, radius, *, material, name):
    sx, sy, sz = start
    ex, ey, ez = end
    vx, vy, vz = ex - sx, ey - sy, ez - sz
    length = math.sqrt(vx * vx + vy * vy + vz * vz)
    if length <= 0.0:
        raise ValueError("zero-length cylinder")
    ux, uy, uz = vx / length, vy / length, vz / length
    yaw = math.atan2(uy, ux)
    pitch = math.atan2(math.sqrt(ux * ux + uy * uy), uz)
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(
            xyz=((sx + ex) * 0.5, (sy + ey) * 0.5, (sz + ez) * 0.5),
            rpy=(0.0, pitch, yaw),
        ),
        material=material,
        name=name,
    )


def _rounded_box(size, radius):
    return cq.Workplane("XY").box(*size).edges().fillet(radius)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_tripodmounted_device")

    graphite = model.material("satin_graphite_painted_metal", rgba=(0.10, 0.11, 0.12, 1.0))
    aluminum = model.material("dark_anodized_aluminum", rgba=(0.22, 0.23, 0.24, 1.0))
    polymer = model.material("matte_black_polymer", rgba=(0.025, 0.026, 0.028, 1.0))
    elastomer = model.material("soft_dark_elastomer", rgba=(0.012, 0.012, 0.011, 1.0))
    glass = model.material("smoked_glass", rgba=(0.015, 0.025, 0.035, 0.92))
    lens = model.material("coated_lens_blue_black", rgba=(0.02, 0.04, 0.075, 1.0))
    accent = model.material("champagne_trim", rgba=(0.70, 0.62, 0.48, 1.0))

    # Root tripod chassis: a connected column, hub, and three hinge sockets.
    base = model.part("tripod_base")
    base.visual(Cylinder(radius=0.030, length=0.470), origin=Origin(xyz=(0.0, 0.0, 0.885)), material=aluminum, name="center_column")
    base.visual(Cylinder(radius=0.066, length=0.075), origin=Origin(xyz=(0.0, 0.0, 0.625)), material=graphite, name="leg_hub")
    base.visual(Cylinder(radius=0.040, length=0.048), origin=Origin(xyz=(0.0, 0.0, 1.060)), material=polymer, name="column_lock_collar")
    base.visual(Cylinder(radius=0.045, length=0.010), origin=Origin(xyz=(0.0, 0.0, 1.087)), material=accent, name="collar_trim_ring")
    base.visual(Cylinder(radius=0.034, length=0.024), origin=Origin(xyz=(0.0, 0.0, 1.108)), material=graphite, name="pan_seat")
    for i in range(3):
        a = i * 2.0 * math.pi / 3.0 + math.pi / 6.0
        u = (math.cos(a), math.sin(a), 0.0)
        _cylinder_between(
            base,
            (u[0] * 0.046, u[1] * 0.046, 0.612),
            (u[0] * 0.080, u[1] * 0.080, 0.612),
            0.014,
            material=graphite,
            name=f"leg_mount_{i}",
        )

    # Three fixed, splayed tripod legs with real feet and mid-leg polymer collars.
    for i in range(3):
        a = i * 2.0 * math.pi / 3.0 + math.pi / 6.0
        u = (math.cos(a), math.sin(a), 0.0)
        t = (-math.sin(a), math.cos(a), 0.0)
        leg = model.part(f"leg_{i}")
        top = (u[0] * 0.094, u[1] * 0.094, 0.612)
        foot = (u[0] * 0.430, u[1] * 0.430, 0.034)
        _cylinder_between(
            leg,
            (top[0] - t[0] * 0.028, top[1] - t[1] * 0.028, top[2]),
            (top[0] + t[0] * 0.028, top[1] + t[1] * 0.028, top[2]),
            0.018,
            material=graphite,
            name="hinge_barrel",
        )
        _cylinder_between(leg, top, foot, 0.014, material=aluminum, name="leg_tube")
        mid0 = tuple(top[j] * 0.58 + foot[j] * 0.42 for j in range(3))
        leg_vec = (foot[0] - top[0], foot[1] - top[1], foot[2] - top[2])
        leg_len = math.sqrt(sum(v * v for v in leg_vec))
        n = tuple(v / leg_len for v in leg_vec)
        _cylinder_between(
            leg,
            tuple(mid0[j] - n[j] * 0.030 for j in range(3)),
            tuple(mid0[j] + n[j] * 0.030 for j in range(3)),
            0.018,
            material=polymer,
            name="leg_collar",
        )
        leg.visual(Cylinder(radius=0.019, length=0.032), origin=Origin(xyz=foot), material=graphite, name="foot_socket")
        leg.visual(Cylinder(radius=0.050, length=0.018), origin=Origin(xyz=(foot[0], foot[1], 0.009)), material=elastomer, name="rubber_foot")
        model.articulation(
            f"base_to_leg_{i}",
            ArticulationType.FIXED,
            parent=base,
            child=leg,
            origin=Origin(),
        )

    # Pan head: rotating base disc plus a machined yoke that carries the tilt axis.
    pan_head = model.part("pan_head")
    pan_head.visual(Cylinder(radius=0.066, length=0.026), origin=Origin(xyz=(0.0, 0.0, 0.013)), material=graphite, name="pan_disc")
    pan_head.visual(Cylinder(radius=0.052, length=0.014), origin=Origin(xyz=(0.0, 0.0, 0.033)), material=accent, name="pan_trim")
    yoke_mesh = mesh_from_geometry(
        TrunnionYokeGeometry(
            (0.160, 0.064, 0.110),
            span_width=0.102,
            trunnion_diameter=0.030,
            trunnion_center_z=0.075,
            base_thickness=0.018,
            corner_radius=0.004,
            center=False,
        ),
        "tilt_yoke",
    )
    pan_head.visual(yoke_mesh, origin=Origin(xyz=(0.0, 0.0, 0.038)), material=graphite, name="yoke_shell")
    pan_head.visual(Cylinder(radius=0.010, length=0.030), origin=Origin(xyz=(0.078, 0.0, 0.030), rpy=(0.0, math.pi / 2.0, 0.0)), material=graphite, name="pan_knob_boss")
    pan_head.visual(Cylinder(radius=0.016, length=0.038), origin=Origin(xyz=(0.091, 0.0, 0.113), rpy=(0.0, math.pi / 2.0, 0.0)), material=graphite, name="tilt_knob_boss")
    model.articulation(
        "pan_axis",
        ArticulationType.REVOLUTE,
        parent=base,
        child=pan_head,
        origin=Origin(xyz=(0.0, 0.0, 1.120)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=4.0, velocity=1.2, lower=-math.pi, upper=math.pi),
    )

    pan_knob = model.part("pan_knob")
    pan_knob.visual(Cylinder(radius=0.006, length=0.030), origin=Origin(xyz=(0.015, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)), material=graphite, name="knob_stem")
    pan_knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.034,
                0.020,
                body_style="faceted",
                grip=KnobGrip(style="ribbed", count=14, depth=0.0010, width=0.0015),
                bore=KnobBore(style="round", diameter=0.006),
            ),
            "pan_lock_knob",
        ),
        origin=Origin(xyz=(0.035, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=polymer,
        name="knob_cap",
    )
    model.articulation(
        "pan_lock",
        ArticulationType.REVOLUTE,
        parent=pan_head,
        child=pan_knob,
        origin=Origin(xyz=(0.093, 0.0, 0.030)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=2.0, lower=-1.2, upper=1.2),
    )

    # Tilt cradle: trunnion axle, floating quick-release plate, and bracket rails.
    tilt_cradle = model.part("tilt_cradle")
    tilt_cradle.visual(Cylinder(radius=0.0154, length=0.132), origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)), material=aluminum, name="tilt_axle")
    tilt_cradle.visual(Cylinder(radius=0.020, length=0.064), origin=Origin(xyz=(0.0, 0.0, 0.032)), material=graphite, name="cradle_neck")
    tilt_cradle.visual(
        mesh_from_cadquery(_rounded_box((0.172, 0.092, 0.014), 0.006), "quick_release_plate"),
        origin=Origin(xyz=(0.0, 0.0, 0.066)),
        material=graphite,
        name="head_plate",
    )
    tilt_cradle.visual(Box((0.040, 0.012, 0.122)), origin=Origin(xyz=(0.0, -0.038, 0.128)), material=polymer, name="bracket_spine")
    tilt_cradle.visual(Box((0.155, 0.026, 0.018)), origin=Origin(xyz=(0.0, -0.054, 0.083)), material=polymer, name="fixed_lower_jaw")
    tilt_cradle.visual(Box((0.138, 0.030, 0.006)), origin=Origin(xyz=(0.0, -0.055, 0.089)), material=elastomer, name="lower_pad")
    for side, x in enumerate((-0.083, 0.083)):
        tilt_cradle.visual(Box((0.014, 0.016, 0.112)), origin=Origin(xyz=(x, -0.040, 0.146)), material=polymer, name=f"guide_sleeve_{side}")
    tilt_cradle.visual(Box((0.178, 0.010, 0.012)), origin=Origin(xyz=(0.0, -0.027, 0.197)), material=polymer, name="top_bridge")
    model.articulation(
        "tilt_axis",
        ArticulationType.REVOLUTE,
        parent=pan_head,
        child=tilt_cradle,
        origin=Origin(xyz=(0.0, 0.0, 0.113)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=1.0, lower=-0.65, upper=0.95),
    )

    tilt_knob = model.part("tilt_knob")
    tilt_knob.visual(Cylinder(radius=0.006, length=0.034), origin=Origin(xyz=(0.017, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)), material=graphite, name="knob_stem")
    tilt_knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.042,
                0.024,
                body_style="lobed",
                top_diameter=0.038,
                crown_radius=0.0014,
                bore=KnobBore(style="round", diameter=0.006),
                body_reliefs=(KnobRelief(style="top_recess", width=0.014, depth=0.0012),),
            ),
            "tilt_lock_knob",
        ),
        origin=Origin(xyz=(0.041, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=polymer,
        name="knob_cap",
    )
    model.articulation(
        "tilt_lock",
        ArticulationType.REVOLUTE,
        parent=pan_head,
        child=tilt_knob,
        origin=Origin(xyz=(0.1085, 0.0, 0.113)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=2.0, lower=-1.4, upper=1.4),
    )

    # A premium consumer device clamped in the adjustable bracket.
    device = model.part("device")
    device.visual(
        mesh_from_cadquery(_rounded_box((0.168, 0.020, 0.106), 0.008), "device_body"),
        origin=Origin(xyz=(0.0, -0.055, 0.145)),
        material=polymer,
        name="body_shell",
    )
    device.visual(Box((0.145, 0.002, 0.076)), origin=Origin(xyz=(-0.010, -0.0650, 0.145)), material=glass, name="glass_face")
    device.visual(Cylinder(radius=0.014, length=0.004), origin=Origin(xyz=(0.060, -0.067, 0.172), rpy=(-math.pi / 2.0, 0.0, 0.0)), material=accent, name="lens_ring")
    device.visual(Cylinder(radius=0.008, length=0.005), origin=Origin(xyz=(0.060, -0.069, 0.172), rpy=(-math.pi / 2.0, 0.0, 0.0)), material=lens, name="lens_glass")
    model.articulation("cradle_to_device", ArticulationType.FIXED, parent=tilt_cradle, child=device, origin=Origin())

    clamp_jaw = model.part("clamp_jaw")
    clamp_jaw.visual(Box((0.180, 0.026, 0.014)), origin=Origin(xyz=(0.0, 0.0, 0.010)), material=polymer, name="upper_jaw")
    clamp_jaw.visual(Box((0.138, 0.030, 0.006)), origin=Origin(xyz=(0.0, 0.0, 0.000)), material=elastomer, name="upper_pad")
    for side, x in enumerate((-0.083, 0.083)):
        clamp_jaw.visual(Box((0.008, 0.010, 0.070)), origin=Origin(xyz=(x, 0.014, -0.030)), material=aluminum, name=f"slide_rod_{side}")
    model.articulation(
        "clamp_slide",
        ArticulationType.PRISMATIC,
        parent=tilt_cradle,
        child=clamp_jaw,
        origin=Origin(xyz=(0.0, -0.054, 0.201)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=0.12, lower=0.0, upper=0.038),
    )

    clamp_knob = model.part("clamp_knob")
    clamp_knob.visual(Cylinder(radius=0.0045, length=0.020), origin=Origin(xyz=(0.0, 0.0, -0.006)), material=graphite, name="threaded_screw")
    clamp_knob.visual(
        mesh_from_geometry(
            KnobGeometry(
                0.038,
                0.020,
                body_style="lobed",
                top_diameter=0.034,
                crown_radius=0.0012,
                bore=KnobBore(style="round", diameter=0.005),
            ),
            "clamp_adjust_knob",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=polymer,
        name="knob_cap",
    )
    model.articulation(
        "clamp_adjust",
        ArticulationType.REVOLUTE,
        parent=clamp_jaw,
        child=clamp_knob,
        origin=Origin(xyz=(0.0, 0.0, 0.023)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.6, velocity=3.0, lower=-math.pi, upper=math.pi),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("tripod_base")
    pan_head = object_model.get_part("pan_head")
    tilt_cradle = object_model.get_part("tilt_cradle")
    device = object_model.get_part("device")
    clamp_jaw = object_model.get_part("clamp_jaw")
    clamp_knob = object_model.get_part("clamp_knob")
    tilt_knob = object_model.get_part("tilt_knob")
    pan = object_model.get_articulation("pan_axis")
    tilt = object_model.get_articulation("tilt_axis")
    clamp = object_model.get_articulation("clamp_slide")

    ctx.check("three splayed tripod legs", all(object_model.get_part(f"leg_{i}") is not None for i in range(3)))
    ctx.check(
        "head carries pan tilt and clamp axes",
        pan.motion_limits is not None and tilt.motion_limits is not None and clamp.motion_limits is not None,
    )

    ctx.allow_overlap(
        pan_head,
        tilt_cradle,
        elem_a="yoke_shell",
        elem_b="tilt_axle",
        reason="The tilt axle is intentionally captured through the yoke trunnion bores.",
    )
    ctx.expect_overlap(
        tilt_cradle,
        pan_head,
        axes="x",
        elem_a="tilt_axle",
        elem_b="yoke_shell",
        min_overlap=0.10,
        name="tilt axle spans both yoke cheeks",
    )
    ctx.allow_overlap(
        pan_head,
        tilt_knob,
        elem_a="tilt_knob_boss",
        elem_b="knob_stem",
        reason="The tilt lock stem is captured in the side boss of the tripod head.",
    )
    ctx.expect_overlap(
        tilt_knob,
        pan_head,
        axes="x",
        elem_a="knob_stem",
        elem_b="tilt_knob_boss",
        min_overlap=0.001,
        name="tilt lock stem engages head boss",
    )

    for i in range(3):
        leg = object_model.get_part(f"leg_{i}")
        ctx.allow_overlap(
            base,
            leg,
            elem_a=f"leg_mount_{i}",
            elem_b="hinge_barrel",
            reason="Each leg hinge barrel is locally captured in the tripod hub socket.",
        )
        ctx.expect_overlap(
            leg,
            base,
            axes="z",
            elem_a="hinge_barrel",
            elem_b=f"leg_mount_{i}",
            min_overlap=0.020,
            name=f"leg {i} hinge barrel seated in hub",
        )

    for side in (0, 1):
        ctx.allow_overlap(
            tilt_cradle,
            clamp_jaw,
            elem_a=f"guide_sleeve_{side}",
            elem_b=f"slide_rod_{side}",
            reason="The clamp rods are represented as retained sliding members inside the bracket guide sleeves.",
        )
        ctx.expect_within(
            clamp_jaw,
            tilt_cradle,
            axes="xy",
            inner_elem=f"slide_rod_{side}",
            outer_elem=f"guide_sleeve_{side}",
            margin=0.003,
            name=f"clamp rod {side} centered in guide sleeve",
        )
        ctx.expect_overlap(
            clamp_jaw,
            tilt_cradle,
            axes="z",
            elem_a=f"slide_rod_{side}",
            elem_b=f"guide_sleeve_{side}",
            min_overlap=0.030,
            name=f"clamp rod {side} remains inserted",
        )

    ctx.allow_overlap(
        clamp_jaw,
        clamp_knob,
        elem_a="upper_jaw",
        elem_b="threaded_screw",
        reason="The clamp knob screw passes into the upper jaw as a captured tightening screw.",
    )
    ctx.expect_overlap(
        clamp_knob,
        clamp_jaw,
        axes="z",
        elem_a="threaded_screw",
        elem_b="upper_jaw",
        min_overlap=0.006,
        name="clamp screw engages upper jaw",
    )

    with ctx.pose({clamp: 0.0}):
        ctx.expect_gap(
            device,
            tilt_cradle,
            axis="z",
            positive_elem="body_shell",
            negative_elem="lower_pad",
            max_gap=0.004,
            max_penetration=0.001,
            name="device rests on lower elastomer pad",
        )
        ctx.expect_gap(
            clamp_jaw,
            device,
            axis="z",
            positive_elem="upper_pad",
            negative_elem="body_shell",
            max_gap=0.004,
            max_penetration=0.001,
            name="upper clamp pad seats on device",
        )

    rest_pos = ctx.part_world_position(clamp_jaw)
    with ctx.pose({clamp: 0.038}):
        open_pos = ctx.part_world_position(clamp_jaw)
        ctx.expect_gap(
            clamp_jaw,
            device,
            axis="z",
            positive_elem="upper_pad",
            negative_elem="body_shell",
            min_gap=0.030,
            max_gap=0.050,
            name="clamp opens above device",
        )
    ctx.check(
        "clamp slide moves upward",
        rest_pos is not None and open_pos is not None and open_pos[2] > rest_pos[2] + 0.030,
        details=f"rest={rest_pos}, open={open_pos}",
    )

    closed_device_aabb = ctx.part_world_aabb(device)
    with ctx.pose({tilt: 0.45}):
        tilted_device_aabb = ctx.part_world_aabb(device)
    if closed_device_aabb is not None and tilted_device_aabb is not None:
        c0 = (closed_device_aabb[0][1] + closed_device_aabb[1][1]) * 0.5
        c1 = (tilted_device_aabb[0][1] + tilted_device_aabb[1][1]) * 0.5
        ctx.check("tilt axis pitches mounted device", abs(c1 - c0) > 0.015, details=f"y before={c0}, after={c1}")
    else:
        ctx.fail("tilt axis pitches mounted device", "device aabb unavailable")

    return ctx.report()


object_model = build_object_model()
