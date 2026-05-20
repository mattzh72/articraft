from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TireGeometry,
    TireSidewall,
    TireShoulder,
    TireTread,
    TorusGeometry,
    TrunnionYokeGeometry,
    mesh_from_cadquery,
    mesh_from_geometry,
)


def _hollow_tube_mesh(
    *,
    outer_radius: float,
    inner_radius: float,
    height: float,
    segments: int = 48,
) -> MeshGeometry:
    """Closed annular tube with an actual clear bore."""
    geom = MeshGeometry()
    outer_bottom = []
    outer_top = []
    inner_bottom = []
    inner_top = []
    for i in range(segments):
        a = 2.0 * math.pi * i / segments
        ca, sa = math.cos(a), math.sin(a)
        outer_bottom.append(geom.add_vertex(outer_radius * ca, outer_radius * sa, 0.0))
        outer_top.append(geom.add_vertex(outer_radius * ca, outer_radius * sa, height))
        inner_bottom.append(geom.add_vertex(inner_radius * ca, inner_radius * sa, 0.0))
        inner_top.append(geom.add_vertex(inner_radius * ca, inner_radius * sa, height))

    for i in range(segments):
        j = (i + 1) % segments
        # Outer wall.
        geom.add_face(outer_bottom[i], outer_bottom[j], outer_top[j])
        geom.add_face(outer_bottom[i], outer_top[j], outer_top[i])
        # Inner wall, reversed normals.
        geom.add_face(inner_bottom[j], inner_bottom[i], inner_top[i])
        geom.add_face(inner_bottom[j], inner_top[i], inner_top[j])
        # Annular top cap.
        geom.add_face(outer_top[i], outer_top[j], inner_top[j])
        geom.add_face(outer_top[i], inner_top[j], inner_top[i])
        # Annular bottom cap.
        geom.add_face(outer_bottom[j], outer_bottom[i], inner_bottom[i])
        geom.add_face(outer_bottom[j], inner_bottom[i], inner_bottom[j])
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rolling_tripod_photo_spotlight")

    matte_black = Material("matte_black", rgba=(0.015, 0.016, 0.018, 1.0))
    satin_black = Material("satin_black", rgba=(0.05, 0.052, 0.055, 1.0))
    dark_graphite = Material("dark_graphite", rgba=(0.12, 0.125, 0.13, 1.0))
    brushed_metal = Material("brushed_metal", rgba=(0.55, 0.56, 0.54, 1.0))
    rubber = Material("rubber", rgba=(0.01, 0.01, 0.01, 1.0))
    warm_lens = Material("warm_fresnel_lens", rgba=(1.0, 0.76, 0.32, 0.72))
    pale_led = Material("pale_led", rgba=(0.88, 0.92, 0.96, 1.0))
    for material in (
        matte_black,
        satin_black,
        dark_graphite,
        brushed_metal,
        rubber,
        warm_lens,
        pale_led,
    ):
        model.materials.append(material)

    base = model.part("base")
    # Rolling crown and the fixed lower sleeve for the telescoping mast.
    base.visual(
        Cylinder(radius=0.125, length=0.080),
        origin=Origin(xyz=(0.0, 0.0, 0.300)),
        material=dark_graphite,
        name="crown_hub",
    )
    base.visual(
        mesh_from_geometry(
            _hollow_tube_mesh(outer_radius=0.058, inner_radius=0.038, height=0.600),
            "lower_mast_sleeve",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.340)),
        material=satin_black,
        name="lower_mast_sleeve",
    )
    base.visual(
        mesh_from_geometry(
            _hollow_tube_mesh(outer_radius=0.078, inner_radius=0.034, height=0.040),
            "mast_clamp_collar",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.940)),
        material=dark_graphite,
        name="mast_clamp_collar",
    )
    base.visual(
        Box((0.090, 0.026, 0.036)),
        origin=Origin(xyz=(0.078, 0.0, 0.940)),
        material=dark_graphite,
        name="clamp_boss",
    )

    leg_length = 0.620
    leg_drop_angle = math.radians(24.0)
    foot_x = leg_length * math.cos(leg_drop_angle)
    foot_z = -leg_length * math.sin(leg_drop_angle)
    hinge_radius = 0.148
    hinge_z = 0.310
    wheel_radius = 0.048

    for i in range(3):
        yaw = 2.0 * math.pi * i / 3.0
        hx = hinge_radius * math.cos(yaw)
        hy = hinge_radius * math.sin(yaw)
        tangent_yaw = yaw + math.pi / 2.0
        # Root-side hinge barrels visibly attach each folding leg to the crown.
        base.visual(
            Cylinder(radius=0.017, length=0.082),
            origin=Origin(xyz=(hx, hy, hinge_z), rpy=(math.pi / 2.0, 0.0, tangent_yaw)),
            material=brushed_metal,
            name=f"crown_hinge_{i}",
        )

    for i in range(3):
        leg = model.part(f"leg_{i}")
        leg_tube_start = 0.070
        leg_tube_length = leg_length - leg_tube_start
        leg.visual(
            Cylinder(radius=0.014, length=0.070),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=brushed_metal,
            name="hinge_pin",
        )
        leg.visual(
            Box((0.095, 0.032, 0.028)),
            origin=Origin(
                xyz=(0.048 * math.cos(leg_drop_angle), 0.0, -0.048 * math.sin(leg_drop_angle)),
                rpy=(0.0, leg_drop_angle, 0.0),
            ),
            material=matte_black,
            name="hinge_to_leg_socket",
        )
        leg.visual(
            Box((leg_tube_length, 0.034, 0.026)),
            origin=Origin(
                xyz=(
                    (leg_tube_start + 0.5 * leg_tube_length) * math.cos(leg_drop_angle),
                    0.0,
                    -(leg_tube_start + 0.5 * leg_tube_length) * math.sin(leg_drop_angle),
                ),
                rpy=(0.0, leg_drop_angle, 0.0),
            ),
            material=matte_black,
            name="folding_leg_tube",
        )
        # Caster fork and axle at the end of the folding leg.
        wheel_z = foot_z - 0.080
        leg.visual(
            Box((0.055, 0.026, 0.070)),
            origin=Origin(xyz=(foot_x - 0.012, 0.0, foot_z + 0.070)),
            material=dark_graphite,
            name="caster_neck",
        )
        leg.visual(
            Box((0.070, 0.036, 0.050)),
            origin=Origin(xyz=(foot_x - 0.012, 0.0, foot_z + 0.018)),
            material=dark_graphite,
            name="caster_socket",
        )
        leg.visual(
            Box((0.044, 0.072, 0.018)),
            origin=Origin(xyz=(foot_x + 0.004, 0.0, foot_z + 0.026)),
            material=dark_graphite,
            name="caster_bridge",
        )
        leg.visual(
            Box((0.030, 0.010, 0.120)),
            origin=Origin(xyz=(foot_x + 0.012, 0.027, wheel_z + 0.040)),
            material=dark_graphite,
            name="caster_fork_a",
        )
        leg.visual(
            Box((0.030, 0.010, 0.120)),
            origin=Origin(xyz=(foot_x + 0.012, -0.027, wheel_z + 0.040)),
            material=dark_graphite,
            name="caster_fork_b",
        )
        leg.visual(
            Cylinder(radius=0.006, length=0.072),
            origin=Origin(xyz=(foot_x + 0.012, 0.0, wheel_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=brushed_metal,
            name="caster_axle",
        )

        yaw = 2.0 * math.pi * i / 3.0
        model.articulation(
            f"leg_hinge_{i}",
            ArticulationType.REVOLUTE,
            parent=base,
            child=leg,
            origin=Origin(
                xyz=(hinge_radius * math.cos(yaw), hinge_radius * math.sin(yaw), hinge_z),
                rpy=(0.0, 0.0, yaw),
            ),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=18.0, velocity=1.4, lower=-0.75, upper=0.18),
        )

        wheel = model.part(f"wheel_{i}")
        wheel.visual(
            mesh_from_geometry(
                TireGeometry(
                    wheel_radius,
                    0.030,
                    inner_radius=0.030,
                    tread=TireTread(style="ribbed", depth=0.0025, count=20, land_ratio=0.58),
                    sidewall=TireSidewall(style="rounded", bulge=0.05),
                    shoulder=TireShoulder(width=0.004, radius=0.002),
                ),
                f"caster_tire_{i}",
            ),
            origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
            material=rubber,
            name="tire",
        )
        wheel.visual(
            Cylinder(radius=0.031, length=0.038),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=brushed_metal,
            name="wheel_hub",
        )
        model.articulation(
            f"wheel_spin_{i}",
            ArticulationType.CONTINUOUS,
            parent=leg,
            child=wheel,
            origin=Origin(xyz=(foot_x + 0.012, 0.0, wheel_z)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=1.0, velocity=20.0),
        )

    mast = model.part("mast")
    mast.visual(
        Cylinder(radius=0.026, length=1.150),
        origin=Origin(xyz=(0.0, 0.0, 0.075)),
        material=brushed_metal,
        name="inner_mast",
    )
    mast.visual(
        Cylinder(radius=0.038, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, -0.480)),
        material=dark_graphite,
        name="lower_slide_bushing",
    )
    mast.visual(
        Cylinder(radius=0.047, length=0.038),
        origin=Origin(xyz=(0.0, 0.0, 0.665)),
        material=dark_graphite,
        name="pan_bearing_cap",
    )
    mast.visual(
        Cylinder(radius=0.064, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.692)),
        material=dark_graphite,
        name="top_turntable_plate",
    )
    model.articulation(
        "mast_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=mast,
        origin=Origin(xyz=(0.0, 0.0, 0.895)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.35, lower=0.0, upper=0.45),
    )

    yoke = model.part("yoke")
    yoke.visual(
        Cylinder(radius=0.074, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=dark_graphite,
        name="pan_disk",
    )
    yoke.visual(
        mesh_from_geometry(
            TrunnionYokeGeometry(
                (0.355, 0.115, 0.250),
                span_width=0.255,
                trunnion_diameter=0.040,
                trunnion_center_z=0.165,
                base_thickness=0.034,
                corner_radius=0.006,
                center=False,
            ),
            "tilt_yoke",
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.026)),
        material=matte_black,
        name="tilt_yoke",
    )
    model.articulation(
        "yoke_pan",
        ArticulationType.REVOLUTE,
        parent=mast,
        child=yoke,
        origin=Origin(xyz=(0.0, 0.0, 0.702)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=1.5, lower=-math.pi, upper=math.pi),
    )

    head = model.part("head")
    body_shape = (
        cq.Workplane("XY")
        .box(0.225, 0.235, 0.155)
        .edges("|Z")
        .fillet(0.018)
        .edges("#Z")
        .fillet(0.006)
    )
    head.visual(
        mesh_from_cadquery(body_shape, "spotlight_housing", tolerance=0.0015),
        origin=Origin(),
        material=satin_black,
        name="housing",
    )
    # Side trunnion bosses are captured in the yoke bores.
    head.visual(
        Cylinder(radius=0.022, length=0.065),
        origin=Origin(xyz=(0.132, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_metal,
        name="side_trunnion_a",
    )
    head.visual(
        Cylinder(radius=0.022, length=0.065),
        origin=Origin(xyz=(-0.132, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_metal,
        name="side_trunnion_b",
    )
    head.visual(
        mesh_from_geometry(TorusGeometry(radius=0.083, tube=0.009, radial_segments=18, tubular_segments=48), "front_bezel_ring"),
        origin=Origin(xyz=(0.0, 0.126, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_graphite,
        name="front_bezel",
    )
    head.visual(
        Cylinder(radius=0.074, length=0.008),
        origin=Origin(xyz=(0.0, 0.122, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=warm_lens,
        name="front_lens",
    )
    head.visual(
        Cylinder(radius=0.045, length=0.004),
        origin=Origin(xyz=(0.0, 0.127, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=pale_led,
        name="led_emitter",
    )
    head.visual(
        Box((0.090, 0.018, 0.038)),
        origin=Origin(xyz=(0.0, -0.124, 0.046)),
        material=dark_graphite,
        name="rear_heat_sink",
    )
    head.visual(
        Cylinder(radius=0.014, length=0.042),
        origin=Origin(xyz=(0.128, -0.128, 0.022), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_metal,
        name="handle_pivot_a",
    )
    head.visual(
        Box((0.030, 0.020, 0.030)),
        origin=Origin(xyz=(0.103, -0.118, 0.022)),
        material=dark_graphite,
        name="handle_pivot_mount_a",
    )
    head.visual(
        Cylinder(radius=0.014, length=0.042),
        origin=Origin(xyz=(-0.128, -0.128, 0.022), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_metal,
        name="handle_pivot_b",
    )
    head.visual(
        Box((0.030, 0.020, 0.030)),
        origin=Origin(xyz=(-0.103, -0.118, 0.022)),
        material=dark_graphite,
        name="handle_pivot_mount_b",
    )
    model.articulation(
        "head_tilt",
        ArticulationType.REVOLUTE,
        parent=yoke,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, 0.191)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=15.0, velocity=1.2, lower=-0.85, upper=0.85),
    )

    rear_handle = model.part("rear_handle")
    rear_handle.visual(
        Cylinder(radius=0.016, length=0.034),
        origin=Origin(xyz=(0.132, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_graphite,
        name="pivot_eye_a",
    )
    rear_handle.visual(
        Cylinder(radius=0.016, length=0.034),
        origin=Origin(xyz=(-0.132, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_graphite,
        name="pivot_eye_b",
    )
    rear_handle.visual(
        Box((0.026, 0.150, 0.026)),
        origin=Origin(xyz=(0.132, -0.075, 0.006)),
        material=matte_black,
        name="handle_arm_a",
    )
    rear_handle.visual(
        Box((0.026, 0.150, 0.026)),
        origin=Origin(xyz=(-0.132, -0.075, 0.006)),
        material=matte_black,
        name="handle_arm_b",
    )
    rear_handle.visual(
        Box((0.290, 0.032, 0.040)),
        origin=Origin(xyz=(0.0, -0.150, 0.006)),
        material=matte_black,
        name="carry_grip",
    )
    model.articulation(
        "handle_fold",
        ArticulationType.REVOLUTE,
        parent=head,
        child=rear_handle,
        origin=Origin(xyz=(0.0, -0.128, 0.022)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=2.0, lower=-1.35, upper=0.55),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    # Local hidden penetrations represent captured shafts, hinge barrels, and
    # sliding guide bushings rather than accidental collisions.
    for i in range(3):
        ctx.allow_overlap(
            "base",
            f"leg_{i}",
            elem_a=f"crown_hinge_{i}",
            elem_b="hinge_to_leg_socket",
            reason="The folding leg socket is intentionally wrapped around the crown hinge barrel.",
        )
        ctx.expect_overlap(
            "base",
            f"leg_{i}",
            elem_a=f"crown_hinge_{i}",
            elem_b="hinge_to_leg_socket",
            axes="xyz",
            min_overlap=0.010,
            name=f"leg {i} hinge socket captures crown barrel",
        )
        ctx.allow_overlap(
            "base",
            f"leg_{i}",
            elem_a=f"crown_hinge_{i}",
            elem_b="hinge_pin",
            reason="The metal leg hinge pin is intentionally coaxial with the crown hinge barrel.",
        )
        ctx.expect_overlap(
            "base",
            f"leg_{i}",
            elem_a=f"crown_hinge_{i}",
            elem_b="hinge_pin",
            axes="xyz",
            min_overlap=0.010,
            name=f"leg {i} hinge pin runs through crown barrel",
        )

        ctx.allow_overlap(
            f"leg_{i}",
            f"wheel_{i}",
            elem_a="caster_axle",
            elem_b="wheel_hub",
            reason="The caster axle is intentionally modeled through the wheel hub bore.",
        )
        ctx.expect_overlap(
            f"leg_{i}",
            f"wheel_{i}",
            elem_a="caster_axle",
            elem_b="wheel_hub",
            axes="xyz",
            min_overlap=0.008,
            name=f"wheel {i} hub is retained on axle",
        )

    ctx.allow_overlap(
        "base",
        "mast",
        elem_a="lower_mast_sleeve",
        elem_b="lower_slide_bushing",
        reason="The hidden guide bushing is seated in the lower sleeve to represent a retained telescoping fit.",
    )
    ctx.expect_within(
        "mast",
        "base",
        inner_elem="lower_slide_bushing",
        outer_elem="lower_mast_sleeve",
        axes="xy",
        margin=0.001,
        name="mast guide bushing stays centered in lower sleeve",
    )
    ctx.expect_overlap(
        "mast",
        "base",
        elem_a="lower_slide_bushing",
        elem_b="lower_mast_sleeve",
        axes="z",
        min_overlap=0.025,
        name="mast guide bushing remains inserted in lower sleeve",
    )

    ctx.allow_overlap(
        "head",
        "yoke",
        elem_a="side_trunnion_a",
        elem_b="tilt_yoke",
        reason="The side trunnion is intentionally captured in the yoke bore for the tilt hinge.",
    )
    ctx.expect_overlap(
        "head",
        "yoke",
        elem_a="side_trunnion_a",
        elem_b="tilt_yoke",
        axes="xyz",
        min_overlap=0.020,
        name="spotlight trunnion is seated in yoke bore",
    )
    ctx.allow_overlap(
        "head",
        "yoke",
        elem_a="side_trunnion_b",
        elem_b="tilt_yoke",
        reason="The opposing side trunnion is intentionally captured in the yoke bore for the tilt hinge.",
    )
    ctx.expect_overlap(
        "head",
        "yoke",
        elem_a="side_trunnion_b",
        elem_b="tilt_yoke",
        axes="xyz",
        min_overlap=0.020,
        name="opposing spotlight trunnion is seated in yoke bore",
    )

    for suffix in ("a", "b"):
        ctx.allow_overlap(
            "head",
            "rear_handle",
            elem_a=f"handle_pivot_{suffix}",
            elem_b=f"pivot_eye_{suffix}",
            reason="The rear handle eye rotates around a short housing pivot stub.",
        )
        ctx.expect_overlap(
            "head",
            "rear_handle",
            elem_a=f"handle_pivot_{suffix}",
            elem_b=f"pivot_eye_{suffix}",
            axes="xyz",
            min_overlap=0.010,
            name=f"rear handle pivot {suffix} is captured on housing stub",
        )
        ctx.allow_overlap(
            "head",
            "rear_handle",
            elem_a=f"handle_pivot_{suffix}",
            elem_b=f"handle_arm_{suffix}",
            reason="The handle arm root wraps locally around the short housing pivot.",
        )
        ctx.expect_overlap(
            "head",
            "rear_handle",
            elem_a=f"handle_pivot_{suffix}",
            elem_b=f"handle_arm_{suffix}",
            axes="xyz",
            min_overlap=0.006,
            name=f"rear handle arm {suffix} is seated around pivot root",
        )

    mast_slide = object_model.get_articulation("mast_slide")
    leg_hinge_0 = object_model.get_articulation("leg_hinge_0")
    head_tilt = object_model.get_articulation("head_tilt")
    handle_fold = object_model.get_articulation("handle_fold")

    rest_mast = ctx.part_world_position("mast")
    rest_wheel = ctx.part_world_position("wheel_0")
    rest_lens_aabb = ctx.part_element_world_aabb("head", elem="front_lens")
    rest_grip_aabb = ctx.part_element_world_aabb("rear_handle", elem="carry_grip")

    with ctx.pose({mast_slide: 0.45}):
        raised_mast = ctx.part_world_position("mast")
        ctx.expect_overlap(
            "mast",
            "base",
            elem_a="lower_slide_bushing",
            elem_b="lower_mast_sleeve",
            axes="z",
            min_overlap=0.025,
            name="extended mast bushing remains retained in sleeve",
        )

    with ctx.pose({leg_hinge_0: -0.60}):
        folded_wheel = ctx.part_world_position("wheel_0")

    with ctx.pose({head_tilt: 0.60}):
        tilted_lens_aabb = ctx.part_element_world_aabb("head", elem="front_lens")

    with ctx.pose({handle_fold: -1.00}):
        folded_grip_aabb = ctx.part_element_world_aabb("rear_handle", elem="carry_grip")

    ctx.check(
        "mast slides upward at full extension",
        rest_mast is not None and raised_mast is not None and raised_mast[2] > rest_mast[2] + 0.40,
        details=f"rest={rest_mast}, raised={raised_mast}",
    )
    ctx.check(
        "tripod leg folds upward on crown hinge",
        rest_wheel is not None and folded_wheel is not None and folded_wheel[2] > rest_wheel[2] + 0.25,
        details=f"rest={rest_wheel}, folded={folded_wheel}",
    )
    ctx.check(
        "head tilt lifts the front lens",
        rest_lens_aabb is not None
        and tilted_lens_aabb is not None
        and (tilted_lens_aabb[0][2] + tilted_lens_aabb[1][2]) * 0.5
        > (rest_lens_aabb[0][2] + rest_lens_aabb[1][2]) * 0.5 + 0.05,
        details=f"rest={rest_lens_aabb}, tilted={tilted_lens_aabb}",
    )
    ctx.check(
        "rear handle folds upward for carrying",
        rest_grip_aabb is not None
        and folded_grip_aabb is not None
        and (folded_grip_aabb[0][2] + folded_grip_aabb[1][2]) * 0.5
        > (rest_grip_aabb[0][2] + rest_grip_aabb[1][2]) * 0.5 + 0.10,
        details=f"rest={rest_grip_aabb}, folded={folded_grip_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
