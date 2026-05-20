from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


LEG_HINGE_Z = 0.78
LEG_HINGE_R = 0.095
UPPER_DX = 0.28
UPPER_DZ = -0.38
LOWER_FOOT_Z = 0.397
LEG_THETA = math.atan2(UPPER_DX, UPPER_DZ)


def _cyl_x() -> tuple[float, float, float]:
    return (0.0, math.pi / 2.0, 0.0)


def _cyl_y() -> tuple[float, float, float]:
    return (-math.pi / 2.0, 0.0, 0.0)


def _radial(angle: float, radius: float, z: float) -> tuple[float, float, float]:
    return (radius * math.cos(angle), radius * math.sin(angle), z)


def _yawed(angle: float, xyz: tuple[float, float, float]) -> tuple[float, float, float]:
    x, y, z = xyz
    c = math.cos(angle)
    s = math.sin(angle)
    return (c * x - s * y, s * x + c * y, z)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="precision_tripod_camera")

    model.material("matte_black", rgba=(0.015, 0.016, 0.018, 1.0))
    model.material("anodized_dark", rgba=(0.055, 0.058, 0.064, 1.0))
    model.material("brushed_aluminum", rgba=(0.55, 0.56, 0.54, 1.0))
    model.material("rubber", rgba=(0.008, 0.008, 0.007, 1.0))
    model.material("camera_body", rgba=(0.085, 0.09, 0.10, 1.0))
    model.material("glass", rgba=(0.08, 0.18, 0.25, 0.72))
    model.material("screen", rgba=(0.015, 0.025, 0.033, 1.0))
    model.material("marking", rgba=(0.82, 0.84, 0.78, 1.0))

    clamp_knob_mesh = mesh_from_geometry(
        KnobGeometry(
            0.052,
            0.026,
            body_style="lobed",
            top_diameter=0.046,
            base_diameter=0.035,
            crown_radius=0.001,
        ),
        "lobed_clamp_knob",
    )
    small_knob_mesh = mesh_from_geometry(
        KnobGeometry(
            0.035,
            0.018,
            body_style="faceted",
            grip=KnobGrip(style="ribbed", count=14, depth=0.0008),
            indicator=KnobIndicator(style="dot", mode="raised", angle_deg=0.0),
        ),
        "small_lock_knob",
    )
    focus_ring_mesh = mesh_from_geometry(
        KnobGeometry(
            0.078,
            0.048,
            body_style="cylindrical",
            grip=KnobGrip(style="fluted", count=28, depth=0.0011),
        ),
        "fluted_focus_ring",
    )
    pan_handle_mesh = mesh_from_geometry(
        tube_from_spline_points(
            [
                (0.000, -0.086, 0.060),
                (-0.025, -0.160, 0.032),
                (-0.070, -0.260, -0.020),
                (-0.105, -0.335, -0.060),
            ],
            radius=0.0075,
            samples_per_segment=10,
            radial_segments=16,
            cap_ends=True,
        ),
        "pan_handle_tube",
    )

    hub = model.part("hub")
    hub.visual(
        Cylinder(radius=0.110, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, LEG_HINGE_Z - 0.025)),
        material="anodized_dark",
        name="spider_plate",
    )
    hub.visual(
        Sphere(radius=0.072),
        origin=Origin(xyz=(0.0, 0.0, LEG_HINGE_Z)),
        material="anodized_dark",
        name="cast_hub",
    )
    hub.visual(
        Cylinder(radius=0.034, length=0.410),
        origin=Origin(xyz=(0.0, 0.0, 0.955)),
        material="matte_black",
        name="outer_sleeve",
    )
    hub.visual(
        Cylinder(radius=0.048, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 1.120)),
        material="anodized_dark",
        name="clamp_collar",
    )
    for i, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        hub.visual(
            Box((0.125, 0.032, 0.046)),
            origin=Origin(
                xyz=_radial(angle, 0.056, LEG_HINGE_Z),
                rpy=(0.0, 0.0, angle),
            ),
            material="anodized_dark",
            name=f"leg_web_{i}",
        )
        hub.visual(
            Cylinder(radius=0.012, length=0.090),
            origin=Origin(
                xyz=_radial(angle, LEG_HINGE_R, LEG_HINGE_Z),
                rpy=(-math.pi / 2.0, 0.0, angle),
            ),
            material="brushed_aluminum",
            name=f"hinge_pin_{i}",
        )

    mast = model.part("mast")
    mast.visual(
        Cylinder(radius=0.021, length=0.620),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material="brushed_aluminum",
        name="inner_mast",
    )
    mast.visual(
        Cylinder(radius=0.045, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.334)),
        material="anodized_dark",
        name="top_flange",
    )
    mast.visual(
        Cylinder(radius=0.018, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.358)),
        material="brushed_aluminum",
        name="pan_stud",
    )

    model.articulation(
        "sleeve_to_mast",
        ArticulationType.PRISMATIC,
        parent=hub,
        child=mast,
        origin=Origin(xyz=(0.0, 0.0, 1.160)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=90.0, velocity=0.18, lower=0.0, upper=0.220),
    )

    mast_clamp = model.part("mast_clamp")
    mast_clamp.visual(
        Cylinder(radius=0.006, length=0.028),
        origin=Origin(xyz=(0.014, 0.0, 0.0), rpy=_cyl_x()),
        material="brushed_aluminum",
        name="clamp_shaft",
    )
    mast_clamp.visual(
        clamp_knob_mesh,
        origin=Origin(xyz=(0.041, 0.0, 0.0), rpy=_cyl_x()),
        material="matte_black",
        name="lobed_knob",
    )
    model.articulation(
        "collar_to_clamp",
        ArticulationType.REVOLUTE,
        parent=hub,
        child=mast_clamp,
        origin=Origin(xyz=(0.048, 0.0, 1.120)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=4.0, lower=-math.pi, upper=math.pi),
    )

    upper_legs = []
    lower_legs = []
    feet = []
    for i, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        upper = model.part(f"upper_leg_{i}")
        upper_legs.append(upper)
        upper.visual(
            Cylinder(radius=0.023, length=0.070),
            origin=Origin(rpy=_cyl_y()),
            material="anodized_dark",
            name="top_knuckle",
        )
        upper.visual(
            Cylinder(radius=0.012, length=math.hypot(UPPER_DX, UPPER_DZ) - 0.005),
            origin=Origin(
                xyz=(
                    UPPER_DX / 2.0 + 0.5 * 0.005 * math.sin(LEG_THETA),
                    -0.019,
                    UPPER_DZ / 2.0 + 0.5 * 0.005 * math.cos(LEG_THETA),
                ),
                rpy=(0.0, LEG_THETA, 0.0),
            ),
            material="matte_black",
            name="upper_tube_a",
        )
        upper.visual(
            Cylinder(radius=0.012, length=math.hypot(UPPER_DX, UPPER_DZ) - 0.005),
            origin=Origin(
                xyz=(
                    UPPER_DX / 2.0 + 0.5 * 0.005 * math.sin(LEG_THETA),
                    0.019,
                    UPPER_DZ / 2.0 + 0.5 * 0.005 * math.cos(LEG_THETA),
                ),
                rpy=(0.0, LEG_THETA, 0.0),
            ),
            material="matte_black",
            name="upper_tube_b",
        )
        upper.visual(
            Box((0.070, 0.075, 0.060)),
            origin=Origin(
                xyz=(UPPER_DX, 0.0, UPPER_DZ),
                rpy=(0.0, LEG_THETA, 0.0),
            ),
            material="anodized_dark",
            name="lower_socket",
        )
        upper.visual(
            Cylinder(radius=0.010, length=0.082),
            origin=Origin(
                xyz=(UPPER_DX - 0.030, 0.0, UPPER_DZ + 0.040),
                rpy=_cyl_y(),
            ),
            material="brushed_aluminum",
            name="cross_rivet",
        )
        model.articulation(
            f"hub_to_leg_{i}",
            ArticulationType.REVOLUTE,
            parent=hub,
            child=upper,
            origin=Origin(xyz=_radial(angle, LEG_HINGE_R, LEG_HINGE_Z), rpy=(0.0, 0.0, angle)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(effort=35.0, velocity=1.6, lower=-0.25, upper=0.55),
        )

        lower = model.part(f"lower_leg_{i}")
        lower_legs.append(lower)
        lower.visual(
            Cylinder(radius=0.014, length=0.442),
            origin=Origin(xyz=(0.0, 0.0, 0.161)),
            material="brushed_aluminum",
            name="inner_tube",
        )
        for j, z in enumerate((0.090, 0.170, 0.250)):
            lower.visual(
                Cylinder(radius=0.0155, length=0.006),
                origin=Origin(xyz=(0.0, 0.0, z)),
                material="marking",
                name=f"height_mark_{j}",
            )
        lower.visual(
            Sphere(radius=0.017),
            origin=Origin(xyz=(0.0, 0.0, LOWER_FOOT_Z)),
            material="anodized_dark",
            name="foot_socket",
        )
        model.articulation(
            f"leg_{i}_slide",
            ArticulationType.PRISMATIC,
            parent=upper,
            child=lower,
            origin=Origin(xyz=(UPPER_DX, 0.0, UPPER_DZ), rpy=(0.0, LEG_THETA, 0.0)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(effort=45.0, velocity=0.14, lower=0.0, upper=0.180),
        )

        foot = model.part(f"foot_{i}")
        feet.append(foot)
        foot_drop = 0.022
        foot_offset = (foot_drop * math.sin(LEG_THETA), 0.0, -foot_drop * math.cos(LEG_THETA))
        foot.visual(
            Sphere(radius=0.015),
            origin=Origin(),
            material="anodized_dark",
            name="swivel_ball",
        )
        foot.visual(
            Cylinder(radius=0.052, length=0.018),
            origin=Origin(xyz=foot_offset, rpy=(0.0, -LEG_THETA, 0.0)),
            material="rubber",
            name="rubber_pad",
        )
        model.articulation(
            f"leg_{i}_to_foot",
            ArticulationType.REVOLUTE,
            parent=lower,
            child=foot,
            origin=Origin(xyz=(0.0, 0.0, LOWER_FOOT_Z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=8.0, velocity=1.4, lower=-0.45, upper=0.45),
        )

        lock = model.part(f"leg_lock_{i}")
        lock.visual(
            Cylinder(radius=0.0045, length=0.035),
            origin=Origin(xyz=(0.0, 0.012, 0.0), rpy=_cyl_y()),
            material="brushed_aluminum",
            name="lock_shaft",
        )
        lock.visual(
            small_knob_mesh,
            origin=Origin(xyz=(0.0, 0.032, 0.0), rpy=_cyl_y()),
            material="matte_black",
            name="lock_knob",
        )
        model.articulation(
            f"socket_to_lock_{i}",
            ArticulationType.REVOLUTE,
            parent=upper,
            child=lock,
            origin=Origin(xyz=(UPPER_DX, 0.039, UPPER_DZ)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=1.2, velocity=4.0, lower=-math.pi, upper=math.pi),
        )

    pan_head = model.part("pan_head")
    pan_head.visual(
        Cylinder(radius=0.070, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material="anodized_dark",
        name="pan_disc",
    )
    pan_head.visual(
        Box((0.130, 0.180, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.0505)),
        material="anodized_dark",
        name="yoke_base",
    )
    pan_head.visual(
        Box((0.038, 0.018, 0.120)),
        origin=Origin(xyz=(0.0, -0.070, 0.105)),
        material="anodized_dark",
        name="tilt_cheek_a",
    )
    pan_head.visual(
        Cylinder(radius=0.018, length=0.014),
        origin=Origin(xyz=(0.0, -0.0602, 0.105), rpy=_cyl_y()),
        material="brushed_aluminum",
        name="tilt_bushing_a",
    )
    pan_head.visual(
        Box((0.038, 0.018, 0.120)),
        origin=Origin(xyz=(0.0, 0.070, 0.105)),
        material="anodized_dark",
        name="tilt_cheek_b",
    )
    pan_head.visual(
        Cylinder(radius=0.018, length=0.014),
        origin=Origin(xyz=(0.0, 0.0602, 0.105), rpy=_cyl_y()),
        material="brushed_aluminum",
        name="tilt_bushing_b",
    )
    pan_head.visual(
        pan_handle_mesh,
        origin=Origin(),
        material="matte_black",
        name="pan_handle",
    )
    pan_head.visual(
        Sphere(radius=0.012),
        origin=Origin(xyz=(-0.055, 0.040, 0.073)),
        material="glass",
        name="bubble_level",
    )
    model.articulation(
        "mast_to_pan",
        ArticulationType.REVOLUTE,
        parent=mast,
        child=pan_head,
        origin=Origin(xyz=(0.0, 0.0, 0.350)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.0, lower=-math.pi, upper=math.pi),
    )

    camera = model.part("camera")
    camera.visual(
        Box((0.160, 0.095, 0.085)),
        origin=Origin(xyz=(0.055, 0.0, 0.0)),
        material="camera_body",
        name="body_shell",
    )
    camera.visual(
        Cylinder(radius=0.014, length=0.110),
        origin=Origin(rpy=_cyl_y()),
        material="brushed_aluminum",
        name="tilt_trunnion",
    )
    camera.visual(
        Cylinder(radius=0.043, length=0.020),
        origin=Origin(xyz=(0.145, 0.0, 0.0), rpy=_cyl_x()),
        material="matte_black",
        name="front_bezel",
    )
    camera.visual(
        Cylinder(radius=0.032, length=0.044),
        origin=Origin(xyz=(0.177, 0.0, 0.0), rpy=_cyl_x()),
        material="matte_black",
        name="front_lens_barrel",
    )
    camera.visual(
        Cylinder(radius=0.029, length=0.004),
        origin=Origin(xyz=(0.201, 0.0, 0.0), rpy=_cyl_x()),
        material="glass",
        name="front_glass",
    )
    camera.visual(
        Box((0.007, 0.072, 0.046)),
        origin=Origin(xyz=(-0.0285, 0.0, 0.0)),
        material="screen",
        name="rear_screen",
    )
    camera.visual(
        Box((0.052, 0.020, 0.008)),
        origin=Origin(xyz=(0.040, 0.0, 0.0465)),
        material="matte_black",
        name="hot_shoe",
    )
    camera.visual(
        Box((0.040, 0.018, 0.072)),
        origin=Origin(xyz=(0.100, -0.056, -0.004)),
        material="rubber",
        name="side_grip",
    )
    model.articulation(
        "pan_to_camera",
        ArticulationType.REVOLUTE,
        parent=pan_head,
        child=camera,
        origin=Origin(xyz=(0.0, 0.0, 0.105)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.5, lower=-0.65, upper=0.75),
    )

    focus_ring = model.part("focus_ring")
    focus_ring.visual(
        focus_ring_mesh,
        origin=Origin(rpy=_cyl_x()),
        material="matte_black",
        name="fluted_ring",
    )
    model.articulation(
        "camera_to_focus",
        ArticulationType.REVOLUTE,
        parent=camera,
        child=focus_ring,
        origin=Origin(xyz=(0.179, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.8, velocity=3.0, lower=-1.2, upper=1.2),
    )

    shutter = model.part("shutter")
    shutter.visual(
        Cylinder(radius=0.010, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.0035)),
        material="brushed_aluminum",
        name="button_cap",
    )
    model.articulation(
        "camera_to_shutter",
        ArticulationType.PRISMATIC,
        parent=camera,
        child=shutter,
        origin=Origin(xyz=(0.095, -0.030, 0.043)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=0.6, velocity=0.04, lower=0.0, upper=0.004),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    hub = object_model.get_part("hub")
    mast = object_model.get_part("mast")
    ctx.allow_overlap(
        hub,
        mast,
        elem_a="outer_sleeve",
        elem_b="inner_mast",
        reason="The center mast is intentionally represented as a retained telescoping member inside the sleeve proxy.",
    )
    ctx.allow_overlap(
        hub,
        mast,
        elem_a="clamp_collar",
        elem_b="inner_mast",
        reason="The mast passes through the compression collar that locks the telescoping support.",
    )
    ctx.expect_within(
        mast,
        hub,
        axes="xy",
        inner_elem="inner_mast",
        outer_elem="outer_sleeve",
        margin=0.001,
        name="mast stays centered inside sleeve",
    )
    ctx.expect_overlap(
        mast,
        hub,
        axes="z",
        elem_a="inner_mast",
        elem_b="outer_sleeve",
        min_overlap=0.160,
        name="collapsed mast remains inserted",
    )
    ctx.expect_overlap(
        mast,
        hub,
        axes="z",
        elem_a="inner_mast",
        elem_b="clamp_collar",
        min_overlap=0.030,
        name="mast passes through locking collar",
    )

    for i in range(3):
        upper = object_model.get_part(f"upper_leg_{i}")
        lower = object_model.get_part(f"lower_leg_{i}")
        foot = object_model.get_part(f"foot_{i}")
        ctx.allow_overlap(
            hub,
            upper,
            elem_a=f"hinge_pin_{i}",
            elem_b="top_knuckle",
            reason="The leg hinge knuckle is captured around the metal pin at the spider hub.",
        )
        for tube_name in ("upper_tube_a", "upper_tube_b"):
            ctx.allow_overlap(
                hub,
                upper,
                elem_a=f"hinge_pin_{i}",
                elem_b=tube_name,
                reason="The upper tube root crowds around the captured hinge pin in the compact spider joint.",
            )
        ctx.allow_overlap(
            hub,
            upper,
            elem_a=f"leg_web_{i}",
            elem_b="top_knuckle",
            reason="The cast spider lug locally surrounds the leg hinge knuckle as a supported fork.",
        )
        for tube_name in ("upper_tube_a", "upper_tube_b"):
            ctx.allow_overlap(
                hub,
                upper,
                elem_a=f"leg_web_{i}",
                elem_b=tube_name,
                reason="The upper leg tube root is seated into the cast spider lug.",
            )
        ctx.allow_overlap(
            hub,
            upper,
            elem_a="spider_plate",
            elem_b="top_knuckle",
            reason="The top of the leg knuckle is tucked under the tripod spider plate at the hinge.",
        )
        for tube_name in ("upper_tube_a", "upper_tube_b"):
            ctx.allow_overlap(
                hub,
                upper,
                elem_a="spider_plate",
                elem_b=tube_name,
                reason="The upper leg tube is seated into the underside of the spider casting.",
            )
        ctx.expect_contact(
            hub,
            upper,
            elem_a=f"hinge_pin_{i}",
            elem_b="top_knuckle",
            contact_tol=0.001,
            name=f"leg {i} hinge is pinned",
        )
        ctx.expect_overlap(
            hub,
            upper,
            axes="z",
            elem_a=f"hinge_pin_{i}",
            elem_b="upper_tube_a",
            min_overlap=0.006,
            name=f"leg {i} hinge pin crosses tube root",
        )
        ctx.expect_overlap(
            hub,
            upper,
            axes="z",
            elem_a=f"leg_web_{i}",
            elem_b="top_knuckle",
            min_overlap=0.020,
            name=f"leg {i} hinge lug surrounds knuckle",
        )
        ctx.expect_overlap(
            hub,
            upper,
            axes="z",
            elem_a=f"leg_web_{i}",
            elem_b="upper_tube_a",
            min_overlap=0.004,
            name=f"leg {i} tube root seats in spider lug",
        )
        ctx.expect_overlap(
            hub,
            upper,
            axes="z",
            elem_a="spider_plate",
            elem_b="top_knuckle",
            min_overlap=0.010,
            name=f"leg {i} knuckle sits under spider plate",
        )
        ctx.allow_overlap(
            upper,
            lower,
            elem_a="lower_socket",
            elem_b="inner_tube",
            reason="The lower leg tube slides through the clamp socket with retained insertion.",
        )
        for tube_name in ("upper_tube_a", "upper_tube_b"):
            ctx.allow_overlap(
                lower,
                upper,
                elem_a="inner_tube",
                elem_b=tube_name,
                reason="The sliding lower tube nests between the twin upper leg tubes at the clamp end.",
            )
        ctx.allow_overlap(
            lower,
            upper,
            elem_a="inner_tube",
            elem_b="cross_rivet",
            reason="The clamp cross-rivet is represented as a local pin crossing the sliding tube proxy.",
        )
        ctx.expect_overlap(
            lower,
            upper,
            axes="z",
            elem_a="inner_tube",
            elem_b="lower_socket",
            min_overlap=0.030,
            name=f"leg {i} tube remains captured in socket",
        )
        ctx.expect_overlap(
            lower,
            upper,
            axes="z",
            elem_a="inner_tube",
            elem_b="upper_tube_a",
            min_overlap=0.020,
            name=f"leg {i} lower tube nests by upper tubes",
        )
        ctx.allow_overlap(
            lower,
            foot,
            elem_a="foot_socket",
            elem_b="swivel_ball",
            reason="The rubber foot uses a captured ball joint at the end of the lower leg.",
        )
        ctx.allow_overlap(
            lower,
            foot,
            elem_a="inner_tube",
            elem_b="swivel_ball",
            reason="The lower tube terminates inside the captured ball joint at the foot.",
        )
        ctx.allow_overlap(
            lower,
            foot,
            elem_a="foot_socket",
            elem_b="rubber_pad",
            reason="The rubber foot pad is slightly compressed under the captured socket ball.",
        )
        ctx.expect_contact(
            lower,
            foot,
            elem_a="foot_socket",
            elem_b="swivel_ball",
            contact_tol=0.001,
            name=f"foot {i} is attached by ball joint",
        )
        ctx.expect_overlap(
            lower,
            foot,
            axes="z",
            elem_a="inner_tube",
            elem_b="swivel_ball",
            min_overlap=0.004,
            name=f"foot {i} ball joint captures tube end",
        )
        ctx.expect_overlap(
            lower,
            foot,
            axes="z",
            elem_a="foot_socket",
            elem_b="rubber_pad",
            min_overlap=0.002,
            name=f"foot {i} rubber pad cups socket",
        )

        lock = object_model.get_part(f"leg_lock_{i}")
        ctx.allow_overlap(
            upper,
            lock,
            elem_a="lower_socket",
            elem_b="lock_shaft",
            reason="The clamp knob shaft passes through the lower leg socket casting.",
        )
        for tube_name in ("upper_tube_a", "upper_tube_b"):
            ctx.allow_overlap(
                lock,
                upper,
                elem_a="lock_shaft",
                elem_b=tube_name,
                reason="The clamp shaft is tucked through the crowded twin-tube clamp cluster.",
            )
        ctx.expect_overlap(
            upper,
            lock,
            axes="y",
            elem_a="lower_socket",
            elem_b="lock_shaft",
            min_overlap=0.003,
            name=f"leg {i} lock shaft passes through socket",
        )

        foot_aabb = ctx.part_world_aabb(foot)
        if foot_aabb is not None:
            min_pt, _ = foot_aabb
            ctx.check(
                f"foot {i} sits on ground plane",
                abs(min_pt[2]) < 0.018,
                details=f"foot bottom z={min_pt[2]:.4f}",
            )

    pan = object_model.get_part("pan_head")
    camera = object_model.get_part("camera")
    focus = object_model.get_part("focus_ring")
    ctx.allow_overlap(
        pan,
        camera,
        elem_a="tilt_bushing_a",
        elem_b="tilt_trunnion",
        reason="The camera trunnion is seated in the yoke bushing for the tilt axis.",
    )
    ctx.allow_overlap(
        pan,
        camera,
        elem_a="tilt_bushing_b",
        elem_b="tilt_trunnion",
        reason="The camera trunnion is seated in the opposite yoke bushing for the tilt axis.",
    )
    ctx.allow_overlap(
        object_model.get_part("mast"),
        pan,
        elem_a="pan_stud",
        elem_b="pan_disc",
        reason="The pan head rotates around the mast-mounted spindle captured inside the pan disc.",
    )
    ctx.expect_overlap(
        object_model.get_part("mast"),
        pan,
        axes="z",
        elem_a="pan_stud",
        elem_b="pan_disc",
        min_overlap=0.020,
        name="pan disc is captured on mast stud",
    )
    ctx.expect_within(
        camera,
        pan,
        axes="y",
        inner_elem="body_shell",
        outer_elem="yoke_base",
        margin=0.020,
        name="camera body is centered in yoke span",
    )
    ctx.expect_contact(
        camera,
        focus,
        elem_a="front_bezel",
        elem_b="fluted_ring",
        contact_tol=0.004,
        name="focus ring seats against front lens bezel",
    )
    ctx.allow_overlap(
        camera,
        focus,
        elem_a="front_lens_barrel",
        elem_b="fluted_ring",
        reason="The focus ring is modeled as a solid knurled sleeve around the lens barrel proxy.",
    )
    ctx.expect_overlap(
        camera,
        focus,
        axes="x",
        elem_a="front_lens_barrel",
        elem_b="fluted_ring",
        min_overlap=0.030,
        name="focus ring sleeve covers lens barrel",
    )

    mast_joint = object_model.get_articulation("sleeve_to_mast")
    pan_joint = object_model.get_articulation("mast_to_pan")
    tilt_joint = object_model.get_articulation("pan_to_camera")
    slide_joint = object_model.get_articulation("leg_0_slide")
    shutter_joint = object_model.get_articulation("camera_to_shutter")

    rest_mast = ctx.part_world_position(mast)
    with ctx.pose({mast_joint: 0.200}):
        raised_mast = ctx.part_world_position(mast)
        ctx.expect_overlap(
            mast,
            hub,
            axes="z",
            elem_a="inner_mast",
            elem_b="outer_sleeve",
            min_overlap=0.060,
            name="raised mast still has retained insertion",
        )
    ctx.check(
        "center support raises vertically",
        rest_mast is not None and raised_mast is not None and raised_mast[2] > rest_mast[2] + 0.18,
        details=f"rest={rest_mast}, raised={raised_mast}",
    )

    rest_foot = ctx.part_world_position(object_model.get_part("foot_0"))
    with ctx.pose({slide_joint: 0.140}):
        extended_foot = ctx.part_world_position(object_model.get_part("foot_0"))
    ctx.check(
        "leg slide lengthens outward and downward",
        rest_foot is not None
        and extended_foot is not None
        and extended_foot[0] > rest_foot[0] + 0.07
        and extended_foot[2] < rest_foot[2] - 0.08,
        details=f"rest={rest_foot}, extended={extended_foot}",
    )

    def _element_center(part_name: str, elem_name: str) -> tuple[float, float, float] | None:
        box = ctx.part_element_world_aabb(object_model.get_part(part_name), elem=elem_name)
        if box is None:
            return None
        lo, hi = box
        return ((lo[0] + hi[0]) * 0.5, (lo[1] + hi[1]) * 0.5, (lo[2] + hi[2]) * 0.5)

    rest_camera = _element_center("camera", "front_glass")
    with ctx.pose({pan_joint: 0.70, tilt_joint: 0.45}):
        aimed_camera = _element_center("camera", "front_glass")
    ctx.check(
        "pan and tilt move the instrument head",
        rest_camera is not None
        and aimed_camera is not None
        and abs(aimed_camera[0] - rest_camera[0]) + abs(aimed_camera[1] - rest_camera[1]) > 0.015,
        details=f"rest={rest_camera}, aimed={aimed_camera}",
    )

    shutter = object_model.get_part("shutter")
    rest_button = ctx.part_world_position(shutter)
    with ctx.pose({shutter_joint: 0.004}):
        pressed_button = ctx.part_world_position(shutter)
    ctx.check(
        "shutter button depresses downward",
        rest_button is not None and pressed_button is not None and pressed_button[2] < rest_button[2] - 0.003,
        details=f"rest={rest_button}, pressed={pressed_button}",
    )

    return ctx.report()


object_model = build_object_model()
