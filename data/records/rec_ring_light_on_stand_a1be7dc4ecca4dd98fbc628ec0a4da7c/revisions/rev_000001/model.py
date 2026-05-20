from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    BoltPattern,
    Box,
    Cylinder,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TireGeometry,
    TireShoulder,
    TireSidewall,
    TireTread,
    TorusGeometry,
    TrunnionYokeGeometry,
    WheelBore,
    WheelFace,
    WheelGeometry,
    WheelHub,
    WheelRim,
    WheelSpokes,
    mesh_from_geometry,
)


def _hollow_tube_mesh(
    *,
    outer_radius: float,
    inner_radius: float,
    height: float,
    name: str,
    segments: int = 48,
):
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [(outer_radius, 0.0), (outer_radius, height)],
            [(inner_radius, 0.0), (inner_radius, height)],
            segments=segments,
            start_cap="flat",
            end_cap="flat",
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rolling_floor_ring_light")

    matte_black = model.material("matte_black", rgba=(0.015, 0.016, 0.018, 1.0))
    satin_black = model.material("satin_black", rgba=(0.045, 0.047, 0.052, 1.0))
    dark_plastic = model.material("dark_plastic", rgba=(0.025, 0.027, 0.030, 1.0))
    rubber = model.material("rubber", rgba=(0.008, 0.008, 0.009, 1.0))
    aluminum = model.material("brushed_aluminum", rgba=(0.63, 0.66, 0.69, 1.0))
    glowing_diffuser = model.material("warm_led_diffuser", rgba=(1.0, 0.86, 0.58, 0.82))
    cool_white = model.material("cool_white_label", rgba=(0.86, 0.88, 0.90, 1.0))

    outer_sleeve_mesh = _hollow_tube_mesh(
        outer_radius=0.038,
        inner_radius=0.028,
        height=0.72,
        name="outer_sleeve",
        segments=64,
    )
    clamp_collar_mesh = _hollow_tube_mesh(
        outer_radius=0.052,
        inner_radius=0.030,
        height=0.040,
        name="clamp_collar",
        segments=64,
    )

    rear_ring_mesh = mesh_from_geometry(
        TorusGeometry(radius=0.185, tube=0.030, radial_segments=24, tubular_segments=96),
        "rear_housing",
    )
    led_ring_mesh = mesh_from_geometry(
        TorusGeometry(radius=0.183, tube=0.023, radial_segments=18, tubular_segments=96),
        "led_diffuser",
    )
    inner_bezel_mesh = mesh_from_geometry(
        TorusGeometry(radius=0.151, tube=0.008, radial_segments=10, tubular_segments=80),
        "inner_bezel",
    )
    outer_bezel_mesh = mesh_from_geometry(
        TorusGeometry(radius=0.217, tube=0.008, radial_segments=10, tubular_segments=96),
        "outer_bezel",
    )
    yoke_mesh = mesh_from_geometry(
        TrunnionYokeGeometry(
            (0.62, 0.085, 0.380),
            span_width=0.490,
            trunnion_diameter=0.036,
            trunnion_center_z=0.290,
            base_thickness=0.040,
            corner_radius=0.010,
            center=False,
        ),
        "top_yoke",
    )
    wheel_mesh = mesh_from_geometry(
        WheelGeometry(
            0.032,
            0.028,
            rim=WheelRim(
                inner_radius=0.021,
                flange_height=0.003,
                flange_thickness=0.0025,
                bead_seat_depth=0.0015,
            ),
            hub=WheelHub(
                radius=0.012,
                width=0.020,
                cap_style="flat",
                bolt_pattern=BoltPattern(count=4, circle_diameter=0.014, hole_diameter=0.0025),
            ),
            face=WheelFace(dish_depth=0.0025, front_inset=0.001, rear_inset=0.001),
            spokes=WheelSpokes(style="straight", count=5, thickness=0.0025, window_radius=0.004),
            bore=WheelBore(style="round", diameter=0.010),
        ),
        "caster_wheel_rim",
    )
    tire_mesh = mesh_from_geometry(
        TireGeometry(
            0.045,
            0.030,
            inner_radius=0.032,
            tread=TireTread(style="ribbed", depth=0.0025, count=18, land_ratio=0.58),
            sidewall=TireSidewall(style="rounded", bulge=0.04),
            shoulder=TireShoulder(width=0.004, radius=0.002),
        ),
        "caster_tire",
    )

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.165, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
        material=satin_black,
        name="weighted_base_disk",
    )
    base.visual(
        Cylinder(radius=0.105, length=0.095),
        origin=Origin(xyz=(0.0, 0.0, 0.105)),
        material=matte_black,
        name="central_hub",
    )
    base.visual(
        outer_sleeve_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.120)),
        material=satin_black,
        name="outer_sleeve",
    )
    base.visual(
        clamp_collar_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.815)),
        material=matte_black,
        name="clamp_collar",
    )
    base.visual(
        Cylinder(radius=0.012, length=0.070),
        origin=Origin(xyz=(0.073, 0.0, 0.835), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=matte_black,
        name="clamp_knob_stub",
    )

    caster_radius = 0.43
    caster_angles = [i * math.tau / 5.0 + math.pi / 10.0 for i in range(5)]
    for index, angle in enumerate(caster_angles):
        arm_radius = 0.235
        base.visual(
            Box((0.420, 0.055, 0.025)),
            origin=Origin(
                xyz=(arm_radius * math.cos(angle), arm_radius * math.sin(angle), 0.130),
                rpy=(0.0, 0.0, angle),
            ),
            material=matte_black,
            name=f"base_spoke_{index}",
        )
        base.visual(
            Cylinder(radius=0.031, length=0.050),
            origin=Origin(
                xyz=(caster_radius * math.cos(angle), caster_radius * math.sin(angle), 0.145)
            ),
            material=satin_black,
            name=f"caster_socket_{index}",
        )

    mast = model.part("mast")
    mast.visual(
        Cylinder(radius=0.022, length=1.420),
        origin=Origin(xyz=(0.0, 0.0, 0.110)),
        material=aluminum,
        name="inner_pole",
    )
    for glide_index, (gx, gy, sx, sy) in enumerate(
        (
            (0.0245, 0.0, 0.008, 0.014),
            (-0.0245, 0.0, 0.008, 0.014),
            (0.0, 0.0245, 0.014, 0.008),
            (0.0, -0.0245, 0.014, 0.008),
        )
    ):
        mast.visual(
            Box((sx, sy, 0.100)),
            origin=Origin(xyz=(gx, gy, -0.500)),
            material=dark_plastic,
            name=f"glide_pad_{glide_index}",
        )
    mast.visual(
        Cylinder(radius=0.030, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.820)),
        material=matte_black,
        name="top_collar",
    )
    mast.visual(
        yoke_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.820)),
        material=matte_black,
        name="top_yoke",
    )

    light_head = model.part("light_head")
    vertical_ring = Origin(rpy=(math.pi / 2.0, 0.0, 0.0))
    light_head.visual(
        rear_ring_mesh,
        origin=vertical_ring,
        material=dark_plastic,
        name="rear_housing",
    )
    light_head.visual(
        led_ring_mesh,
        origin=Origin(xyz=(0.0, -0.014, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=glowing_diffuser,
        name="led_diffuser",
    )
    light_head.visual(
        Cylinder(radius=0.018, length=0.100),
        origin=Origin(xyz=(0.244, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=aluminum,
        name="trunnion_pin_pos",
    )
    light_head.visual(
        Cylinder(radius=0.018, length=0.100),
        origin=Origin(xyz=(-0.244, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=aluminum,
        name="trunnion_pin_neg",
    )
    light_head.visual(
        Box((0.080, 0.030, 0.050)),
        origin=Origin(xyz=(0.0, 0.028, -0.205)),
        material=dark_plastic,
        name="lower_control_pod",
    )
    light_head.visual(
        Box((0.022, 0.022, 0.082)),
        origin=Origin(xyz=(0.0, 0.016, -0.164)),
        material=dark_plastic,
        name="pod_bridge",
    )
    light_head.visual(
        Box((0.030, 0.004, 0.009)),
        origin=Origin(xyz=(0.0, -0.006, -0.205)),
        material=cool_white,
        name="control_label",
    )

    model.articulation(
        "mast_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=mast,
        origin=Origin(xyz=(0.0, 0.0, 0.820)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.20, lower=0.0, upper=0.40),
    )
    model.articulation(
        "head_tilt",
        ArticulationType.REVOLUTE,
        parent=mast,
        child=light_head,
        origin=Origin(xyz=(0.0, 0.0, 1.110)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.0, lower=-0.85, upper=0.85),
    )

    for index, angle in enumerate(caster_angles):
        caster = model.part(f"caster_{index}")
        caster.visual(
            Cylinder(radius=0.012, length=0.080),
            origin=Origin(xyz=(0.0, 0.0, 0.030)),
            material=aluminum,
            name="stem",
        )
        caster.visual(
            Cylinder(radius=0.036, length=0.018),
            origin=Origin(xyz=(0.0, 0.0, -0.010)),
            material=matte_black,
            name="swivel_bearing",
        )
        caster.visual(
            Box((0.008, 0.034, 0.084)),
            origin=Origin(xyz=(0.0245, 0.0, -0.057)),
            material=matte_black,
            name="fork_cheek_pos",
        )
        caster.visual(
            Box((0.008, 0.034, 0.084)),
            origin=Origin(xyz=(-0.0245, 0.0, -0.057)),
            material=matte_black,
            name="fork_cheek_neg",
        )
        caster.visual(
            Box((0.041, 0.034, 0.012)),
            origin=Origin(xyz=(0.0, 0.0, -0.005)),
            material=matte_black,
            name="fork_bridge",
        )
        caster.visual(
            Cylinder(radius=0.010, length=0.010),
            origin=Origin(xyz=(0.031, 0.0, -0.060), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=aluminum,
            name="axle_boss_pos",
        )
        caster.visual(
            Cylinder(radius=0.010, length=0.010),
            origin=Origin(xyz=(-0.031, 0.0, -0.060), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=aluminum,
            name="axle_boss_neg",
        )

        wheel = model.part(f"wheel_{index}")
        wheel.visual(wheel_mesh, material=aluminum, name="rim")
        wheel.visual(tire_mesh, material=rubber, name="tire")
        wheel.visual(
            Cylinder(radius=0.014, length=0.041),
            origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
            material=aluminum,
            name="hub_bushing",
        )

        model.articulation(
            f"caster_swivel_{index}",
            ArticulationType.CONTINUOUS,
            parent=base,
            child=caster,
            origin=Origin(
                xyz=(caster_radius * math.cos(angle), caster_radius * math.sin(angle), 0.105),
                rpy=(0.0, 0.0, angle + math.pi / 2.0),
            ),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=2.0, velocity=5.0),
        )
        model.articulation(
            f"wheel_spin_{index}",
            ArticulationType.CONTINUOUS,
            parent=caster,
            child=wheel,
            origin=Origin(xyz=(0.0, 0.0, -0.060)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=1.0, velocity=20.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    mast = object_model.get_part("mast")
    light_head = object_model.get_part("light_head")
    mast_slide = object_model.get_articulation("mast_slide")
    head_tilt = object_model.get_articulation("head_tilt")

    ctx.expect_within(
        mast,
        base,
        axes="xy",
        inner_elem="inner_pole",
        outer_elem="outer_sleeve",
        margin=0.006,
        name="inner mast is centered in the hollow sleeve",
    )
    ctx.expect_overlap(
        mast,
        base,
        axes="z",
        elem_a="inner_pole",
        elem_b="outer_sleeve",
        min_overlap=0.30,
        name="collapsed mast keeps generous insertion in sleeve",
    )
    for glide_index in range(4):
        ctx.allow_overlap(
            base,
            mast,
            elem_a="outer_sleeve",
            elem_b=f"glide_pad_{glide_index}",
            reason="Low-friction mast guide pads intentionally bear against the inside of the sleeve.",
        )
        ctx.expect_overlap(
            mast,
            base,
            axes="z",
            elem_a=f"glide_pad_{glide_index}",
            elem_b="outer_sleeve",
            min_overlap=0.080,
            name=f"mast glide pad {glide_index} is inside the sleeve length",
        )
    with ctx.pose({mast_slide: 0.40}):
        ctx.expect_within(
            mast,
            base,
            axes="xy",
            inner_elem="inner_pole",
            outer_elem="outer_sleeve",
            margin=0.006,
            name="extended mast stays centered in sleeve",
        )
        ctx.expect_overlap(
            mast,
            base,
            axes="z",
            elem_a="inner_pole",
            elem_b="outer_sleeve",
            min_overlap=0.18,
            name="extended mast remains retained in sleeve",
        )

    closed_aabb = ctx.part_world_aabb(light_head)
    with ctx.pose({head_tilt: 0.65}):
        tilted_aabb = ctx.part_world_aabb(light_head)
    if closed_aabb is not None and tilted_aabb is not None:
        closed_depth = closed_aabb[1][1] - closed_aabb[0][1]
        tilted_depth = tilted_aabb[1][1] - tilted_aabb[0][1]
        ctx.check(
            "tilt motion changes light head pitch",
            tilted_depth > closed_depth + 0.10,
            details=f"closed_depth={closed_depth:.3f}, tilted_depth={tilted_depth:.3f}",
        )
    else:
        ctx.fail("tilt motion changes light head pitch", "missing light-head AABB")

    ctx.allow_overlap(
        mast,
        light_head,
        elem_a="top_yoke",
        elem_b="trunnion_pin_pos",
        reason="The light-head trunnion pin is intentionally captured in the yoke bore.",
    )
    ctx.allow_overlap(
        mast,
        light_head,
        elem_a="top_yoke",
        elem_b="trunnion_pin_neg",
        reason="The light-head trunnion pin is intentionally captured in the yoke bore.",
    )
    ctx.expect_overlap(
        light_head,
        mast,
        axes="x",
        elem_a="trunnion_pin_pos",
        elem_b="top_yoke",
        min_overlap=0.035,
        name="positive trunnion pin reaches into yoke cheek",
    )
    ctx.expect_overlap(
        light_head,
        mast,
        axes="x",
        elem_a="trunnion_pin_neg",
        elem_b="top_yoke",
        min_overlap=0.035,
        name="negative trunnion pin reaches into yoke cheek",
    )

    for index in range(5):
        caster = object_model.get_part(f"caster_{index}")
        wheel = object_model.get_part(f"wheel_{index}")
        ctx.allow_overlap(
            base,
            caster,
            elem_a=f"caster_socket_{index}",
            elem_b="stem",
            reason="The caster swivel stem is intentionally seated inside the base socket.",
        )
        ctx.allow_overlap(
            base,
            caster,
            elem_a=f"base_spoke_{index}",
            elem_b="stem",
            reason="The caster stem passes through the spoke-mounted socket bore.",
        )
        ctx.allow_overlap(
            caster,
            wheel,
            elem_a="fork_cheek_pos",
            elem_b="hub_bushing",
            reason="The wheel bushing is represented as a captured axle sleeve inside the fork-cheek bore.",
        )
        ctx.allow_overlap(
            caster,
            wheel,
            elem_a="fork_cheek_neg",
            elem_b="hub_bushing",
            reason="The wheel bushing is represented as a captured axle sleeve inside the fork-cheek bore.",
        )
        ctx.expect_within(
            caster,
            base,
            axes="xy",
            inner_elem="stem",
            outer_elem=f"caster_socket_{index}",
            margin=0.004,
            name=f"caster {index} stem is centered in socket",
        )
        ctx.expect_overlap(
            caster,
            base,
            axes="z",
            elem_a="stem",
            elem_b=f"caster_socket_{index}",
            min_overlap=0.035,
            name=f"caster {index} stem remains inserted in socket",
        )
        ctx.expect_overlap(
            caster,
            base,
            axes="z",
            elem_a="stem",
            elem_b=f"base_spoke_{index}",
            min_overlap=0.020,
            name=f"caster {index} stem passes through spoke bore",
        )
        ctx.expect_overlap(
            wheel,
            caster,
            axes="z",
            elem_a="hub_bushing",
            elem_b="fork_cheek_pos",
            min_overlap=0.020,
            name=f"wheel {index} positive bushing is captured in fork cheek",
        )
        ctx.expect_overlap(
            wheel,
            caster,
            axes="z",
            elem_a="hub_bushing",
            elem_b="fork_cheek_neg",
            min_overlap=0.020,
            name=f"wheel {index} negative bushing is captured in fork cheek",
        )
        ctx.expect_contact(
            wheel,
            caster,
            elem_a="hub_bushing",
            elem_b="fork_cheek_pos",
            contact_tol=0.001,
            name=f"wheel {index} bushing bears on positive fork cheek",
        )
        ctx.expect_contact(
            wheel,
            caster,
            elem_a="hub_bushing",
            elem_b="fork_cheek_neg",
            contact_tol=0.001,
            name=f"wheel {index} bushing bears on negative fork cheek",
        )

    return ctx.report()


object_model = build_object_model()
