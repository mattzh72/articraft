from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fresnel_spot_tripod_lamp")

    black = model.material("satin_black", rgba=(0.005, 0.005, 0.004, 1.0))
    dark_metal = model.material("dark_burnished_metal", rgba=(0.10, 0.095, 0.085, 1.0))
    chrome = model.material("brushed_steel", rgba=(0.55, 0.52, 0.47, 1.0))
    rubber = model.material("matte_rubber", rgba=(0.015, 0.014, 0.013, 1.0))
    glass = model.material("pale_fresnel_glass", rgba=(0.56, 0.78, 0.92, 0.48))

    apex = model.part("apex_ring")

    apex.visual(
        mesh_from_geometry(TorusGeometry(0.085, 0.012, radial_segments=18, tubular_segments=72), "apex_ring_torus"),
        origin=Origin(xyz=(0.0, 0.0, 1.15)),
        material=dark_metal,
        name="apex_ring",
    )
    apex.visual(
        Cylinder(radius=0.046, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 1.15)),
        material=dark_metal,
        name="center_hub",
    )
    apex.visual(
        Cylinder(radius=0.058, length=0.034),
        origin=Origin(xyz=(0.0, 0.0, 1.195)),
        material=chrome,
        name="top_collar",
    )

    # A low fork bracket grows out of the ring and carries the lamp-head tilt axis.
    apex.visual(
        Box((0.340, 0.052, 0.060)),
        origin=Origin(xyz=(0.170, 0.0, 1.150)),
        material=dark_metal,
        name="yoke_stem",
    )
    apex.visual(
        Cylinder(radius=0.014, length=0.58),
        origin=Origin(xyz=(0.340, 0.0, 1.170), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="lower_yoke_crossbar",
    )
    for side, y, arm_name, boss_name, knob_name in (
        ("side_0", -0.265, "yoke_arm_side_0", "tilt_boss_side_0", "tilt_knob_side_0"),
        ("side_1", 0.265, "yoke_arm_side_1", "tilt_boss_side_1", "tilt_knob_side_1"),
    ):
        apex.visual(
            Box((0.038, 0.026, 0.520)),
            origin=Origin(xyz=(0.340, y, 1.300)),
            material=dark_metal,
            name=arm_name,
        )
        apex.visual(
            Cylinder(radius=0.034, length=0.030),
            origin=Origin(xyz=(0.340, y, 1.420), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=chrome,
            name=boss_name,
        )
        apex.visual(
            Cylinder(radius=0.044, length=0.026),
            origin=Origin(xyz=(0.340, y * 1.10, 1.420), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=black,
            name=knob_name,
        )

    leg_hinge_radius = 0.115
    leg_hinge_z = 1.15
    leg_angles = (math.radians(90.0), math.radians(210.0), math.radians(330.0))
    for index, angle in enumerate(leg_angles):
        radial = (math.cos(angle), math.sin(angle), 0.0)
        tangent = (-math.sin(angle), math.cos(angle), 0.0)
        hinge_xyz = (
            leg_hinge_radius * radial[0],
            leg_hinge_radius * radial[1],
            leg_hinge_z,
        )

        def local_point(x: float, y: float, z: float) -> tuple[float, float, float]:
            return (
                hinge_xyz[0] + x * radial[0] + y * tangent[0],
                hinge_xyz[1] + x * radial[1] + y * tangent[1],
                hinge_xyz[2] + z,
            )

        # Parent-side clevis ears and inward pads attach each folding leg to the apex ring.
        apex.visual(
            Box((0.070, 0.064, 0.036)),
            origin=Origin(xyz=local_point(-0.055, 0.0, 0.0), rpy=(0.0, 0.0, angle)),
            material=dark_metal,
            name=f"leg_socket_{index}",
        )
        for ear, y_offset in enumerate((-0.033, 0.033)):
            apex.visual(
                Box((0.050, 0.016, 0.052)),
                origin=Origin(xyz=local_point(0.002, y_offset, 0.0), rpy=(0.0, 0.0, angle)),
                material=chrome,
                name=f"leg_clevis_{index}_{ear}",
            )

        leg = model.part(f"leg_{index}")
        leg.visual(
            Cylinder(radius=0.016, length=0.050),
            origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=chrome,
            name="hinge_barrel",
        )
        leg.visual(
            Box((0.080, 0.044, 0.036)),
            origin=Origin(xyz=(0.045, 0.0, -0.032)),
            material=dark_metal,
            name="upper_lug",
        )
        leg_start = (0.045, 0.0, -0.045)
        leg_end = (0.800, 0.0, -1.060)
        leg_vec = (
            leg_end[0] - leg_start[0],
            leg_end[1] - leg_start[1],
            leg_end[2] - leg_start[2],
        )
        leg_len = math.sqrt(leg_vec[0] ** 2 + leg_vec[2] ** 2)
        leg_mid = (
            (leg_start[0] + leg_end[0]) * 0.5,
            0.0,
            (leg_start[2] + leg_end[2]) * 0.5,
        )
        leg_theta = math.atan2(leg_vec[0], leg_vec[2])
        leg.visual(
            Cylinder(radius=0.018, length=leg_len),
            origin=Origin(xyz=leg_mid, rpy=(0.0, leg_theta, 0.0)),
            material=chrome,
            name="leg_tube",
        )
        leg.visual(
            Cylinder(radius=0.060, length=0.028),
            origin=Origin(xyz=(0.820, 0.0, -1.075)),
            material=rubber,
            name="foot_pad",
        )

        model.articulation(
            f"apex_to_leg_{index}",
            ArticulationType.REVOLUTE,
            parent=apex,
            child=leg,
            origin=Origin(xyz=hinge_xyz, rpy=(0.0, 0.0, angle)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(effort=28.0, velocity=1.6, lower=0.0, upper=0.78),
        )

    head = model.part("fresnel_head")
    head.visual(
        Cylinder(radius=0.220, length=0.360),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="housing_shell",
    )
    head.visual(
        Cylinder(radius=0.200, length=0.030),
        origin=Origin(xyz=(-0.188, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="rear_cap",
    )
    head.visual(
        mesh_from_geometry(TorusGeometry(0.203, 0.018, radial_segments=18, tubular_segments=96), "front_bezel"),
        origin=Origin(xyz=(0.190, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_metal,
        name="front_bezel",
    )
    head.visual(
        Cylinder(radius=0.186, length=0.028),
        origin=Origin(xyz=(0.186, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=glass,
        name="fresnel_lens",
    )
    for i, radius in enumerate((0.040, 0.070, 0.100, 0.128, 0.154, 0.176)):
        head.visual(
            mesh_from_geometry(
                TorusGeometry(radius, 0.0028, radial_segments=8, tubular_segments=96),
                f"fresnel_ring_{i}",
            ),
            origin=Origin(xyz=(0.197, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=glass,
            name=f"fresnel_ring_{i}",
        )
    for side, y, trunnion_name in (
        ("side_0", -0.233, "side_trunnion_side_0"),
        ("side_1", 0.233, "side_trunnion_side_1"),
    ):
        head.visual(
            Cylinder(radius=0.040, length=0.034),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=chrome,
            name=trunnion_name,
        )

    model.articulation(
        "apex_to_head",
        ArticulationType.REVOLUTE,
        parent=apex,
        child=head,
        origin=Origin(xyz=(0.340, 0.0, 1.420)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=1.2, lower=-0.55, upper=0.75),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.
    # If overlap QC reports an intersection, classify it first: intentional
    # embeddings or nested fits should get a scoped allowance; unintended
    # collisions should be fixed in geometry, support, mount, or pose.

    apex = object_model.get_part("apex_ring")
    head = object_model.get_part("fresnel_head")
    head_joint = object_model.get_articulation("apex_to_head")

    def aabb_center(aabb):
        if aabb is None:
            return None
        lo, hi = aabb
        return tuple((lo[i] + hi[i]) * 0.5 for i in range(3))

    # The visible folding-leg hinge barrels are captured in close-fitting clevis ears.
    # These are local, mechanical hinge intersections rather than body collisions.
    for leg_index, clevis_name in (
        (1, "leg_clevis_1_0"),
        (1, "leg_clevis_1_1"),
        (2, "leg_clevis_2_0"),
        (2, "leg_clevis_2_1"),
    ):
        leg = object_model.get_part(f"leg_{leg_index}")
        ctx.allow_overlap(
            apex,
            leg,
            elem_a=clevis_name,
            elem_b="hinge_barrel",
            reason="The folding-leg hinge barrel is intentionally captured inside the apex clevis ear.",
        )
        ctx.expect_overlap(
            apex,
            leg,
            elem_a=clevis_name,
            elem_b="hinge_barrel",
            axes="z",
            min_overlap=0.025,
            name=f"leg {leg_index} hinge barrel shares clevis height",
        )

    # Tripod legs begin splayed for floor support and fold inward toward the apex.
    leg_0 = object_model.get_part("leg_0")
    foot_rest = aabb_center(ctx.part_element_world_aabb(leg_0, elem="foot_pad"))
    with ctx.pose({"apex_to_leg_0": 0.78}):
        foot_folded = aabb_center(ctx.part_element_world_aabb(leg_0, elem="foot_pad"))
    if foot_rest is None or foot_folded is None:
        ctx.fail("leg foot centers are measurable", "Could not measure leg_0 foot_pad AABBs.")
    else:
        rest_radius = math.hypot(foot_rest[0], foot_rest[1])
        folded_radius = math.hypot(foot_folded[0], foot_folded[1])
        ctx.check(
            "tripod leg folds inward",
            rest_radius > 0.70 and folded_radius < rest_radius - 0.35,
            details=f"rest_radius={rest_radius:.3f}, folded_radius={folded_radius:.3f}",
        )

    # The Fresnel head tilts about the yoke pivot; positive joint travel pitches the lens down.
    lens_rest = aabb_center(ctx.part_element_world_aabb(head, elem="fresnel_lens"))
    with ctx.pose({head_joint: 0.55}):
        lens_down = aabb_center(ctx.part_element_world_aabb(head, elem="fresnel_lens"))
    if lens_rest is None or lens_down is None:
        ctx.fail("head lens centers are measurable", "Could not measure Fresnel lens AABBs.")
    else:
        ctx.check(
            "head tilt pitches lens downward",
            lens_down[2] < lens_rest[2] - 0.08,
            details=f"rest_lens={lens_rest}, tilted_lens={lens_down}",
        )

    ctx.expect_overlap(
        head,
        apex,
        elem_a="side_trunnion_side_1",
        elem_b="tilt_boss_side_1",
        axes="xz",
        min_overlap=0.025,
        name="head trunnion aligns with yoke boss",
    )

    return ctx.report()


object_model = build_object_model()
