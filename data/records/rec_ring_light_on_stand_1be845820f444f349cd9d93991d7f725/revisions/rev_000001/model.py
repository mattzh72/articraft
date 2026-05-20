from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    LatheGeometry,
    Material,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _hollow_tube(outer_radius: float, inner_radius: float, length: float, *, segments: int = 64):
    half = 0.5 * length
    return LatheGeometry.from_shell_profiles(
        [(outer_radius, -half), (outer_radius, half)],
        [(inner_radius, -half), (inner_radius, half)],
        segments=segments,
        start_cap="flat",
        end_cap="flat",
    )


def _circle_profile(radius: float, segments: int = 96):
    return [
        (radius * math.cos(math.tau * i / segments), radius * math.sin(math.tau * i / segments))
        for i in range(segments)
    ]


def _annulus(outer_radius: float, inner_radius: float, thickness: float, *, segments: int = 112):
    return ExtrudeWithHolesGeometry(
        _circle_profile(outer_radius, segments),
        [_circle_profile(inner_radius, segments)],
        thickness,
        center=True,
    )


def _cyl_x(length: float, radius: float, x: float = 0.0, y: float = 0.0, z: float = 0.0):
    return Cylinder(radius=radius, length=length), Origin(
        xyz=(x, y, z),
        rpy=(0.0, math.pi / 2.0, 0.0),
    )


def _cyl_y(length: float, radius: float, x: float = 0.0, y: float = 0.0, z: float = 0.0):
    return Cylinder(radius=radius, length=length), Origin(
        xyz=(x, y, z),
        rpy=(math.pi / 2.0, 0.0, 0.0),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="studio_ring_light_tripod")

    matte_black = model.material("matte_black", rgba=(0.025, 0.026, 0.028, 1.0))
    graphite = model.material("graphite", rgba=(0.13, 0.135, 0.145, 1.0))
    satin_metal = model.material("satin_metal", rgba=(0.45, 0.47, 0.50, 1.0))
    warm_diffuser = model.material("warm_diffuser", rgba=(1.0, 0.92, 0.70, 0.74))
    rubber = model.material("rubber", rgba=(0.012, 0.012, 0.013, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.058, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.105)),
        material=matte_black,
        name="tripod_hub",
    )
    base.visual(
        mesh_from_geometry(_hollow_tube(0.019, 0.0148, 0.660), "outer_sleeve_shell"),
        origin=Origin(xyz=(0.0, 0.0, 0.460)),
        material=matte_black,
        name="outer_sleeve",
    )
    base.visual(
        mesh_from_geometry(_hollow_tube(0.027, 0.0148, 0.058), "top_locking_collar"),
        origin=Origin(xyz=(0.0, 0.0, 0.810)),
        material=graphite,
        name="top_collar",
    )
    base.visual(
        Cylinder(radius=0.006, length=0.049),
        origin=Origin(xyz=(0.036, 0.0, 0.812), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_metal,
        name="clamp_screw",
    )
    base.visual(
        Sphere(radius=0.016),
        origin=Origin(xyz=(0.058, 0.0, 0.812)),
        material=rubber,
        name="clamp_knob",
    )

    leg_angles = (0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)
    for index, angle in enumerate(leg_angles):
        c = math.cos(angle)
        s = math.sin(angle)
        # Socket blocks protrude from the hub and provide a real hinge seat for each folding leg.
        base.visual(
            Box((0.030, 0.055, 0.035)),
            origin=Origin(xyz=(0.065 * c, 0.065 * s, 0.105), rpy=(0.0, 0.0, angle)),
            material=matte_black,
            name=f"hinge_socket_{index}",
        )

    upper_pole = model.part("upper_pole")
    upper_pole.visual(
        Cylinder(radius=0.0122, length=0.930),
        # The inner tube extends below the slide frame so it remains retained at full 300 mm extension.
        origin=Origin(xyz=(0.0, 0.0, 0.075)),
        material=graphite,
        name="inner_tube",
    )
    upper_pole.visual(
        Cylinder(radius=0.017, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.5375)),
        material=matte_black,
        name="top_cap",
    )

    model.articulation(
        "stand_extension",
        ArticulationType.PRISMATIC,
        parent=base,
        child=upper_pole,
        origin=Origin(xyz=(0.0, 0.0, 0.810)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=70.0, velocity=0.18, lower=0.0, upper=0.300),
    )

    for index, angle in enumerate(leg_angles):
        leg = model.part(f"leg_{index}")
        leg.visual(
            Cylinder(radius=0.012, length=0.055),
            origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=satin_metal,
            name="hinge_barrel",
        )
        leg.visual(
            mesh_from_geometry(
                tube_from_spline_points(
                    [
                        (0.010, 0.0, 0.000),
                        (0.195, 0.0, -0.046),
                        (0.445, 0.0, -0.095),
                    ],
                    radius=0.010,
                    samples_per_segment=18,
                    radial_segments=18,
                    cap_ends=True,
                ),
                f"folding_leg_tube_{index}",
            ),
            material=matte_black,
            name="leg_tube",
        )
        leg.visual(
            Sphere(radius=0.019),
            origin=Origin(xyz=(0.458, 0.0, -0.099)),
            material=rubber,
            name="foot_pad",
        )
        model.articulation(
            f"leg_hinge_{index}",
            ArticulationType.REVOLUTE,
            parent=base,
            child=leg,
            origin=Origin(xyz=(0.092 * math.cos(angle), 0.092 * math.sin(angle), 0.105), rpy=(0.0, 0.0, angle)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(effort=12.0, velocity=1.6, lower=0.0, upper=1.15),
        )

    tilt_bracket = model.part("tilt_bracket")
    tilt_bracket.visual(
        Box((0.076, 0.100, 0.035)),
        origin=Origin(xyz=(0.0, 0.030, 0.0175)),
        material=matte_black,
        name="receiver_block",
    )
    tilt_bracket.visual(
        Cylinder(radius=0.013, length=0.212),
        origin=Origin(xyz=(0.0, 0.060, 0.141)),
        material=matte_black,
        name="riser_post",
    )
    tilt_bracket.visual(
        Box((0.520, 0.030, 0.020)),
        origin=Origin(xyz=(0.0, 0.060, 0.255)),
        material=matte_black,
        name="rear_bridge",
    )
    for side, x in enumerate((-0.245, 0.245)):
        tilt_bracket.visual(
            Box((0.024, 0.085, 0.064)),
            origin=Origin(xyz=(x, 0.030, 0.255)),
            material=matte_black,
            name=f"side_cheek_{side}",
        )

    model.articulation(
        "pole_to_bracket",
        ArticulationType.FIXED,
        parent=upper_pole,
        child=tilt_bracket,
        origin=Origin(xyz=(0.0, 0.0, 0.555)),
    )

    ring_head = model.part("ring_head")
    ring_head.visual(
        mesh_from_geometry(
            TorusGeometry(radius=0.190, tube=0.026, radial_segments=22, tubular_segments=128),
            "ring_outer_body",
        ),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=matte_black,
        name="outer_body",
    )
    ring_head.visual(
        mesh_from_geometry(_annulus(0.205, 0.163, 0.008), "warm_diffuser_annulus"),
        origin=Origin(xyz=(0.0, -0.018, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=warm_diffuser,
        name="diffuser",
    )
    # Subtle ribs and a central hub make the hollow light ring read as a supported assembly.
    ring_head.visual(
        Box((0.342, 0.012, 0.014)),
        origin=Origin(),
        material=graphite,
        name="horizontal_spoke",
    )
    ring_head.visual(
        Box((0.014, 0.012, 0.342)),
        origin=Origin(),
        material=graphite,
        name="vertical_spoke",
    )
    ring_head.visual(
        Cylinder(radius=0.036, length=0.032),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=graphite,
        name="center_hub",
    )
    axle_geom, axle_origin = _cyl_x(0.520, 0.0085)
    ring_head.visual(
        axle_geom,
        origin=axle_origin,
        material=satin_metal,
        name="pivot_axle",
    )
    knob_geom, knob_pos_origin = _cyl_x(0.040, 0.021, x=0.280)
    ring_head.visual(
        knob_geom,
        origin=knob_pos_origin,
        material=rubber,
        name="lock_knob_0",
    )
    knob_geom, knob_neg_origin = _cyl_x(0.040, 0.021, x=-0.280)
    ring_head.visual(
        knob_geom,
        origin=knob_neg_origin,
        material=rubber,
        name="lock_knob_1",
    )
    ring_head.visual(
        Box((0.050, 0.026, 0.018)),
        origin=Origin(xyz=(0.0, 0.023, -0.205)),
        material=graphite,
        name="cable_port",
    )

    model.articulation(
        "ring_tilt",
        ArticulationType.REVOLUTE,
        parent=tilt_bracket,
        child=ring_head,
        origin=Origin(xyz=(0.0, 0.0, 0.255)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.2, lower=-math.pi / 3.0, upper=math.pi / 3.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    lift = object_model.get_articulation("stand_extension")
    tilt = object_model.get_articulation("ring_tilt")
    upper_pole = object_model.get_part("upper_pole")
    bracket = object_model.get_part("tilt_bracket")
    ring = object_model.get_part("ring_head")

    # The locking thumb screw lightly presses into the inner tube so the telescoping stage
    # is visibly retained by the collar instead of floating with an unrealistic clearance gap.
    ctx.allow_overlap(
        "base",
        upper_pole,
        elem_a="clamp_screw",
        elem_b="inner_tube",
        reason="The stand collar screw intentionally compresses the telescoping tube by a tiny local amount.",
    )
    ctx.expect_gap(
        "base",
        upper_pole,
        axis="x",
        positive_elem="clamp_screw",
        negative_elem="inner_tube",
        max_penetration=0.001,
        name="clamp screw lightly seats against inner tube",
    )

    # The trunnion axle is intentionally captured through the two solid cheek proxies.
    for cheek_name in ("side_cheek_0", "side_cheek_1"):
        ctx.allow_overlap(
            bracket,
            ring,
            elem_a=cheek_name,
            elem_b="pivot_axle",
            reason="The ring tilt axle is intentionally represented as captured through the bracket cheek bore.",
        )
        ctx.expect_within(
            ring,
            bracket,
            axes="yz",
            inner_elem="pivot_axle",
            outer_elem=cheek_name,
            margin=0.002,
            name=f"pivot axle centered in {cheek_name}",
        )
        ctx.expect_overlap(
            ring,
            bracket,
            axes="x",
            elem_a="pivot_axle",
            elem_b=cheek_name,
            min_overlap=0.010,
            name=f"pivot axle passes through {cheek_name}",
        )

    ctx.check(
        "stand extends 300 mm",
        lift.motion_limits is not None
        and lift.motion_limits.lower == 0.0
        and abs(lift.motion_limits.upper - 0.300) < 1e-6,
        details=f"limits={lift.motion_limits}",
    )
    ctx.check(
        "ring tilts about plus minus 60 degrees",
        tilt.motion_limits is not None
        and abs(tilt.motion_limits.lower + math.pi / 3.0) < 1e-6
        and abs(tilt.motion_limits.upper - math.pi / 3.0) < 1e-6,
        details=f"limits={tilt.motion_limits}",
    )

    ctx.expect_within(
        upper_pole,
        "base",
        axes="xy",
        inner_elem="inner_tube",
        outer_elem="top_collar",
        margin=0.001,
        name="inner tube is centered in top collar",
    )
    ctx.expect_overlap(
        upper_pole,
        "base",
        axes="z",
        elem_a="inner_tube",
        elem_b="outer_sleeve",
        min_overlap=0.250,
        name="collapsed mast remains deeply inserted",
    )

    rest_pos = ctx.part_world_position(upper_pole)
    with ctx.pose({lift: 0.300}):
        extended_pos = ctx.part_world_position(upper_pole)
        ctx.expect_within(
            upper_pole,
            "base",
            axes="xy",
            inner_elem="inner_tube",
            outer_elem="top_collar",
            margin=0.001,
            name="extended inner tube stays centered",
        )
        ctx.expect_overlap(
            upper_pole,
            "base",
            axes="z",
            elem_a="inner_tube",
            elem_b="outer_sleeve",
            min_overlap=0.060,
            name="extended mast still has retained insertion",
        )
    ctx.check(
        "upper pole moves upward on prismatic joint",
        rest_pos is not None and extended_pos is not None and extended_pos[2] > rest_pos[2] + 0.295,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    def _center_from_aabb(aabb):
        return tuple((aabb[0][i] + aabb[1][i]) * 0.5 for i in range(3))

    rest_diffuser_aabb = ctx.part_element_world_aabb(ring, elem="diffuser")
    with ctx.pose({tilt: math.pi / 3.0}):
        tilted_diffuser_aabb = ctx.part_element_world_aabb(ring, elem="diffuser")
    if rest_diffuser_aabb is not None and tilted_diffuser_aabb is not None:
        rest_depth = rest_diffuser_aabb[1][1] - rest_diffuser_aabb[0][1]
        tilted_depth = tilted_diffuser_aabb[1][1] - tilted_diffuser_aabb[0][1]
        ctx.check(
            "ring visibly pitches on horizontal tilt axis",
            tilted_depth > rest_depth + 0.250,
            details=f"rest_y_depth={rest_depth}, tilted_y_depth={tilted_depth}",
        )
    else:
        ctx.fail("ring visibly pitches on horizontal tilt axis", "diffuser AABB unavailable")

    for index in range(3):
        leg = object_model.get_part(f"leg_{index}")
        hinge = object_model.get_articulation(f"leg_hinge_{index}")
        foot_aabb = ctx.part_element_world_aabb(leg, elem="foot_pad")
        if foot_aabb is not None:
            foot_center = _center_from_aabb(foot_aabb)
            radial = math.hypot(foot_center[0], foot_center[1])
            ctx.check(
                f"leg {index} foot is deployed wide",
                radial > 0.50 and foot_center[2] < 0.035,
                details=f"foot_center={foot_center}, radial={radial}",
            )
        with ctx.pose({hinge: 1.15}):
            folded_foot_aabb = ctx.part_element_world_aabb(leg, elem="foot_pad")
        if foot_aabb is not None and folded_foot_aabb is not None:
            rest_center = _center_from_aabb(foot_aabb)
            folded_center = _center_from_aabb(folded_foot_aabb)
            ctx.check(
                f"leg {index} folds upward at hub",
                folded_center[2] > rest_center[2] + 0.300,
                details=f"rest={rest_center}, folded={folded_center}",
            )

    return ctx.report()


object_model = build_object_model()
