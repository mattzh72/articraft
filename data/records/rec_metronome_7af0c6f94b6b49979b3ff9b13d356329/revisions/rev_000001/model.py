from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
    superellipse_profile,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="portable_pyramid_metronome")

    walnut = model.material("walnut", rgba=(0.33, 0.20, 0.11, 1.0))
    dark_walnut = model.material("dark_walnut", rgba=(0.22, 0.13, 0.07, 1.0))
    brass = model.material("brass", rgba=(0.76, 0.63, 0.28, 1.0))
    nickel = model.material("nickel", rgba=(0.76, 0.78, 0.80, 1.0))
    ivory = model.material("ivory", rgba=(0.92, 0.90, 0.84, 1.0))
    black = model.material("black", rgba=(0.12, 0.12, 0.12, 1.0))
    rubber = model.material("rubber", rgba=(0.10, 0.10, 0.10, 1.0))

    def section_loop(
        size_x: float,
        size_y: float,
        radius: float,
        z: float,
        *,
        x_shift: float = 0.0,
        y_shift: float = 0.0,
    ) -> list[tuple[float, float, float]]:
        return [
            (x + x_shift, y + y_shift, z)
            for x, y in rounded_rect_profile(size_x, size_y, radius, corner_segments=6)
        ]

    body = model.part("body")
    body.visual(
        Box((0.120, 0.138, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=dark_walnut,
        name="base_plinth",
    )
    body.visual(
        Box((0.102, 0.098, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=walnut,
        name="base_riser",
    )

    housing_geom = section_loft(
        [
            section_loop(0.080, 0.096, 0.006, 0.024, x_shift=0.001),
            section_loop(0.068, 0.086, 0.005, 0.084, x_shift=-0.004),
            section_loop(0.050, 0.062, 0.004, 0.162, x_shift=-0.012),
            section_loop(0.028, 0.030, 0.003, 0.220, x_shift=-0.017),
        ]
    )
    body.visual(
        mesh_from_geometry(housing_geom, "metronome_housing"),
        material=walnut,
        name="housing_shell",
    )
    body.visual(
        Box((0.034, 0.036, 0.010)),
        origin=Origin(xyz=(-0.017, 0.0, 0.225)),
        material=dark_walnut,
        name="top_crown",
    )
    body.visual(
        Box((0.003, 0.032, 0.126)),
        origin=Origin(xyz=(0.019, 0.0, 0.115), rpy=(0.0, -0.18, 0.0)),
        material=ivory,
        name="tempo_scale",
    )
    body.visual(
        Box((0.032, 0.028, 0.010)),
        origin=Origin(xyz=(0.015, 0.0, 0.212)),
        material=dark_walnut,
        name="pivot_bridge",
    )
    body.visual(
        Box((0.014, 0.006, 0.018)),
        origin=Origin(xyz=(0.031, 0.008, 0.210)),
        material=nickel,
        name="pivot_left_cheek",
    )
    body.visual(
        Box((0.014, 0.006, 0.018)),
        origin=Origin(xyz=(0.031, -0.008, 0.210)),
        material=nickel,
        name="pivot_right_cheek",
    )
    body.visual(
        Cylinder(radius=0.009, length=0.010),
        origin=Origin(xyz=(-0.040, 0.0, 0.090), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brass,
        name="rear_key_boss",
    )
    body.visual(
        Box((0.004, 0.016, 0.006)),
        origin=Origin(xyz=(0.046, 0.062, -0.002)),
        material=dark_walnut,
        name="left_hinge_cap",
    )
    body.visual(
        Box((0.004, 0.016, 0.006)),
        origin=Origin(xyz=(0.046, -0.062, -0.002)),
        material=dark_walnut,
        name="right_hinge_cap",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.120, 0.138, 0.236)),
        mass=1.5,
        origin=Origin(xyz=(0.0, 0.0, 0.118)),
    )

    pendulum = model.part("pendulum")
    pendulum.visual(
        Cylinder(radius=0.005, length=0.018),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=nickel,
        name="pivot_barrel",
    )
    pendulum.visual(
        Box((0.0026, 0.0026, 0.236)),
        origin=Origin(xyz=(0.0, 0.0, -0.062)),
        material=nickel,
        name="pendulum_rod",
    )
    pendulum.visual(
        Cylinder(radius=0.012, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, -0.090), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brass,
        name="lower_bob",
    )
    pendulum.visual(
        Cylinder(radius=0.006, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, -0.154), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brass,
        name="lower_tip",
    )
    pendulum.inertial = Inertial.from_geometry(
        Box((0.028, 0.020, 0.236)),
        mass=0.08,
        origin=Origin(xyz=(0.0, 0.0, -0.060)),
    )

    slider = model.part("slider_weight")
    slider_ring = ExtrudeWithHolesGeometry(
        superellipse_profile(0.020, 0.020, exponent=2.0, segments=24),
        [superellipse_profile(0.006, 0.006, exponent=2.0, segments=18)],
        0.010,
        center=True,
    )
    slider.visual(
        mesh_from_geometry(slider_ring, "metronome_slider_weight"),
        material=brass,
        name="slider_ring",
    )
    slider.visual(
        Box((0.008, 0.005, 0.012)),
        origin=Origin(xyz=(0.008, 0.0, 0.0)),
        material=brass,
        name="slider_grip",
    )
    slider.inertial = Inertial.from_geometry(
        Cylinder(radius=0.010, length=0.010),
        mass=0.025,
    )

    left_leg = model.part("left_stabilizer_leg")
    left_leg.visual(
        Cylinder(radius=0.004, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, -0.002)),
        material=nickel,
        name="hinge_barrel",
    )
    left_leg.visual(
        Box((0.051, 0.008, 0.004)),
        origin=Origin(xyz=(-0.019, 0.017, -0.002), rpy=(0.0, 0.0, 2.412)),
        material=nickel,
        name="leg_arm",
    )
    left_leg.visual(
        Cylinder(radius=0.007, length=0.004),
        origin=Origin(xyz=(-0.038, 0.034, -0.002)),
        material=rubber,
        name="foot_pad",
    )
    left_leg.inertial = Inertial.from_geometry(
        Box((0.055, 0.020, 0.008)),
        mass=0.03,
        origin=Origin(xyz=(-0.020, 0.018, -0.002)),
    )

    right_leg = model.part("right_stabilizer_leg")
    right_leg.visual(
        Cylinder(radius=0.004, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, -0.002)),
        material=nickel,
        name="hinge_barrel",
    )
    right_leg.visual(
        Box((0.051, 0.008, 0.004)),
        origin=Origin(xyz=(-0.019, -0.017, -0.002), rpy=(0.0, 0.0, -2.412)),
        material=nickel,
        name="leg_arm",
    )
    right_leg.visual(
        Cylinder(radius=0.007, length=0.004),
        origin=Origin(xyz=(-0.038, -0.034, -0.002)),
        material=rubber,
        name="foot_pad",
    )
    right_leg.inertial = Inertial.from_geometry(
        Box((0.055, 0.020, 0.008)),
        mass=0.03,
        origin=Origin(xyz=(-0.020, -0.018, -0.002)),
    )

    winding_key = model.part("winding_key")
    winding_key.visual(
        Cylinder(radius=0.0032, length=0.014),
        origin=Origin(xyz=(-0.007, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brass,
        name="key_shaft",
    )
    winding_key.visual(
        Box((0.004, 0.030, 0.006)),
        origin=Origin(xyz=(-0.020, 0.0, 0.006)),
        material=brass,
        name="key_wing",
    )
    winding_key.visual(
        Box((0.006, 0.010, 0.016)),
        origin=Origin(xyz=(-0.016, 0.0, 0.006)),
        material=brass,
        name="key_hub",
    )
    winding_key.inertial = Inertial.from_geometry(
        Box((0.026, 0.030, 0.016)),
        mass=0.02,
        origin=Origin(xyz=(-0.016, 0.0, 0.006)),
    )

    model.articulation(
        "body_to_pendulum",
        ArticulationType.REVOLUTE,
        parent=body,
        child=pendulum,
        origin=Origin(xyz=(0.045, 0.0, 0.210)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.5,
            velocity=3.0,
            lower=-0.55,
            upper=0.55,
        ),
    )
    model.articulation(
        "pendulum_to_slider_weight",
        ArticulationType.PRISMATIC,
        parent=pendulum,
        child=slider,
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.2,
            velocity=0.08,
            lower=0.0,
            upper=0.040,
        ),
    )
    model.articulation(
        "body_to_left_stabilizer",
        ArticulationType.REVOLUTE,
        parent=body,
        child=left_leg,
        origin=Origin(xyz=(0.052, 0.062, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.4,
            velocity=2.0,
            lower=0.0,
            upper=0.76,
        ),
    )
    model.articulation(
        "body_to_right_stabilizer",
        ArticulationType.REVOLUTE,
        parent=body,
        child=right_leg,
        origin=Origin(xyz=(0.052, -0.062, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=0.4,
            velocity=2.0,
            lower=0.0,
            upper=0.76,
        ),
    )
    model.articulation(
        "body_to_winding_key",
        ArticulationType.CONTINUOUS,
        parent=body,
        child=winding_key,
        origin=Origin(xyz=(-0.045, 0.0, 0.090)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.2, velocity=12.0),
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

    body = object_model.get_part("body")
    pendulum = object_model.get_part("pendulum")
    slider = object_model.get_part("slider_weight")
    left_leg = object_model.get_part("left_stabilizer_leg")
    right_leg = object_model.get_part("right_stabilizer_leg")
    winding_key = object_model.get_part("winding_key")

    pendulum_hinge = object_model.get_articulation("body_to_pendulum")
    slider_joint = object_model.get_articulation("pendulum_to_slider_weight")
    left_leg_joint = object_model.get_articulation("body_to_left_stabilizer")
    right_leg_joint = object_model.get_articulation("body_to_right_stabilizer")
    key_joint = object_model.get_articulation("body_to_winding_key")

    def center_of_aabb(aabb):
        if aabb is None:
            return None
        lower, upper = aabb
        return tuple((lo + hi) * 0.5 for lo, hi in zip(lower, upper))

    ctx.check(
        "pendulum hinge uses transverse swing axis",
        tuple(round(v, 3) for v in pendulum_hinge.axis) == (1.0, 0.0, 0.0),
        details=f"axis={pendulum_hinge.axis}",
    )
    ctx.check(
        "slider moves along rod axis",
        tuple(round(v, 3) for v in slider_joint.axis) == (0.0, 0.0, 1.0),
        details=f"axis={slider_joint.axis}",
    )
    ctx.check(
        "stabilizer joints mirror about centerline",
        tuple(round(v, 3) for v in left_leg_joint.axis) == (0.0, 0.0, 1.0)
        and tuple(round(v, 3) for v in right_leg_joint.axis) == (0.0, 0.0, -1.0),
        details=f"left={left_leg_joint.axis}, right={right_leg_joint.axis}",
    )
    ctx.check(
        "winding key spins about rearward shaft axis",
        key_joint.articulation_type == ArticulationType.CONTINUOUS
        and tuple(round(v, 3) for v in key_joint.axis) == (-1.0, 0.0, 0.0),
        details=f"type={key_joint.articulation_type}, axis={key_joint.axis}",
    )

    ctx.expect_gap(
        left_leg,
        body,
        axis="x",
        positive_elem="hinge_barrel",
        negative_elem="left_hinge_cap",
        max_gap=0.0006,
        max_penetration=0.0006,
        name="left stabilizer hinge remains seated at body corner",
    )
    ctx.expect_gap(
        right_leg,
        body,
        axis="x",
        positive_elem="hinge_barrel",
        negative_elem="right_hinge_cap",
        max_gap=0.0006,
        max_penetration=0.0006,
        name="right stabilizer hinge remains seated at body corner",
    )
    ctx.expect_gap(
        body,
        winding_key,
        axis="x",
        positive_elem="rear_key_boss",
        negative_elem="key_shaft",
        max_gap=0.0006,
        max_penetration=0.0006,
        name="winding key shaft meets rear boss",
    )
    ctx.expect_origin_distance(
        slider,
        pendulum,
        axes="xy",
        max_dist=0.001,
        name="slider stays centered on pendulum rod",
    )

    slider_lower = slider_joint.motion_limits.lower if slider_joint.motion_limits else 0.0
    slider_upper = slider_joint.motion_limits.upper if slider_joint.motion_limits else 0.0
    with ctx.pose({pendulum_hinge: 0.0, slider_joint: slider_lower}):
        slider_low_pos = ctx.part_world_position(slider)
    with ctx.pose({pendulum_hinge: 0.0, slider_joint: slider_upper}):
        slider_high_pos = ctx.part_world_position(slider)
    ctx.check(
        "slider weight travels upward along pendulum rod",
        slider_low_pos is not None
        and slider_high_pos is not None
        and slider_high_pos[2] > slider_low_pos[2] + 0.030,
        details=f"lower={slider_low_pos}, upper={slider_high_pos}",
    )

    with ctx.pose({slider_joint: slider_upper, pendulum_hinge: -0.30}):
        left_swing = ctx.part_world_position(slider)
    with ctx.pose({slider_joint: slider_upper, pendulum_hinge: 0.30}):
        right_swing = ctx.part_world_position(slider)
    ctx.check(
        "pendulum swings side to side across the front face",
        left_swing is not None
        and right_swing is not None
        and left_swing[1] > 0.012
        and right_swing[1] < -0.012,
        details=f"left={left_swing}, right={right_swing}",
    )

    with ctx.pose({left_leg_joint: 0.0}):
        left_deployed = center_of_aabb(ctx.part_element_world_aabb(left_leg, elem="foot_pad"))
    with ctx.pose({left_leg_joint: 0.76}):
        left_folded = center_of_aabb(ctx.part_element_world_aabb(left_leg, elem="foot_pad"))
    ctx.check(
        "left stabilizer folds flush toward body side",
        left_deployed is not None
        and left_folded is not None
        and left_deployed[1] > left_folded[1] + 0.025,
        details=f"deployed={left_deployed}, folded={left_folded}",
    )

    with ctx.pose({right_leg_joint: 0.0}):
        right_deployed = center_of_aabb(ctx.part_element_world_aabb(right_leg, elem="foot_pad"))
    with ctx.pose({right_leg_joint: 0.76}):
        right_folded = center_of_aabb(ctx.part_element_world_aabb(right_leg, elem="foot_pad"))
    ctx.check(
        "right stabilizer folds flush toward body side",
        right_deployed is not None
        and right_folded is not None
        and right_deployed[1] < right_folded[1] - 0.025,
        details=f"deployed={right_deployed}, folded={right_folded}",
    )

    with ctx.pose({key_joint: 0.0}):
        key_rest = center_of_aabb(ctx.part_element_world_aabb(winding_key, elem="key_wing"))
    with ctx.pose({key_joint: math.pi / 2.0}):
        key_turned = center_of_aabb(ctx.part_element_world_aabb(winding_key, elem="key_wing"))
    ctx.check(
        "winding key wing rotates about the rear shaft",
        key_rest is not None
        and key_turned is not None
        and abs(key_rest[2] - key_turned[2]) > 0.004,
        details=f"rest={key_rest}, turned={key_turned}",
    )

    ctx.expect_gap(
        pendulum,
        body,
        axis="x",
        positive_elem="pendulum_rod",
        negative_elem="tempo_scale",
        min_gap=0.006,
        max_gap=0.020,
        name="pendulum rod clears front scale plane",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
