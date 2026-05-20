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
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def _circle_profile(radius: float, segments: int = 48) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos((2.0 * math.pi * i) / segments),
            radius * math.sin((2.0 * math.pi * i) / segments),
        )
        for i in range(segments)
    ]


def _ring_housing_mesh():
    outer_profile = _circle_profile(0.112, segments=64)
    inner_profile = _circle_profile(0.080, segments=64)

    annulus = ExtrudeWithHolesGeometry(
        outer_profile,
        [inner_profile],
        height=0.028,
        center=True,
    ).rotate_y(math.pi / 2.0).translate(0.032, 0.0, 0.0)

    front_bezel = TorusGeometry(
        radius=0.096,
        tube=0.012,
        radial_segments=18,
        tubular_segments=72,
    ).rotate_y(math.pi / 2.0).translate(0.036, 0.0, 0.0)

    rear_bezel = TorusGeometry(
        radius=0.096,
        tube=0.010,
        radial_segments=18,
        tubular_segments=72,
    ).rotate_y(math.pi / 2.0).translate(0.020, 0.0, 0.0)

    return annulus.merge(front_bezel).merge(rear_bezel)


def _tube_shell_mesh(outer_radius: float, inner_radius: float, length: float):
    half = 0.5 * length
    return LatheGeometry.from_shell_profiles(
        [(outer_radius, -half), (outer_radius, half)],
        [(inner_radius, -half), (inner_radius, half)],
        segments=56,
        start_cap="flat",
        end_cap="flat",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="magnifying_floor_lamp")

    powder_black = model.material("powder_black", rgba=(0.14, 0.14, 0.15, 1.0))
    dark_charcoal = model.material("dark_charcoal", rgba=(0.20, 0.21, 0.23, 1.0))
    satin_aluminum = model.material("satin_aluminum", rgba=(0.70, 0.72, 0.75, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.56, 0.58, 0.60, 1.0))
    clear_glass = model.material("clear_glass", rgba=(0.86, 0.94, 1.00, 0.35))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.160, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=powder_black,
        name="weighted_base",
    )
    base.visual(
        Cylinder(radius=0.060, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.046)),
        material=dark_charcoal,
        name="base_cap",
    )
    base.visual(
        Cylinder(radius=0.038, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.065)),
        material=powder_black,
        name="lower_collar",
    )
    base.visual(
        mesh_from_geometry(_tube_shell_mesh(0.022, 0.0185, 0.680), "outer_tube_shell"),
        origin=Origin(xyz=(0.0, 0.0, 0.430)),
        material=satin_aluminum,
        name="outer_tube",
    )
    base.visual(
        mesh_from_geometry(_tube_shell_mesh(0.029, 0.0185, 0.045), "top_socket_shell"),
        origin=Origin(xyz=(0.0, 0.0, 0.748)),
        material=powder_black,
        name="top_socket",
    )
    base.inertial = Inertial.from_geometry(
        Cylinder(radius=0.160, length=0.770),
        mass=8.0,
        origin=Origin(xyz=(0.0, 0.0, 0.385)),
    )

    upper_post = model.part("upper_post")
    upper_post.visual(
        Cylinder(radius=0.017, length=0.800),
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
        material=brushed_steel,
        name="inner_tube",
    )
    upper_post.visual(
        Box((0.028, 0.032, 0.050)),
        origin=Origin(xyz=(0.0, 0.0, 0.480)),
        material=powder_black,
        name="neck_block",
    )
    upper_post.visual(
        Cylinder(radius=0.028, length=0.020),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material=powder_black,
        name="stop_collar",
    )
    upper_post.visual(
        Box((0.028, 0.016, 0.044)),
        origin=Origin(xyz=(0.0, 0.033, 0.527)),
        material=powder_black,
        name="upper_yoke_left",
    )
    upper_post.visual(
        Box((0.028, 0.016, 0.044)),
        origin=Origin(xyz=(0.0, -0.033, 0.527)),
        material=powder_black,
        name="upper_yoke_right",
    )
    upper_post.visual(
        Box((0.020, 0.066, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.505)),
        material=powder_black,
        name="upper_yoke_bridge",
    )
    upper_post.inertial = Inertial.from_geometry(
        Box((0.060, 0.080, 0.860)),
        mass=1.8,
        origin=Origin(xyz=(0.0, 0.0, 0.120)),
    )

    model.articulation(
        "base_to_upper_post",
        ArticulationType.PRISMATIC,
        parent=base,
        child=upper_post,
        origin=Origin(xyz=(0.0, 0.0, 0.7705)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.18,
            lower=0.0,
            upper=0.260,
        ),
    )

    arm = model.part("arm")
    arm.visual(
        Cylinder(radius=0.012, length=0.050),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_charcoal,
        name="shoulder_barrel",
    )
    arm.visual(
        Box((0.175, 0.030, 0.014)),
        origin=Origin(xyz=(0.0875, 0.0, 0.0)),
        material=dark_charcoal,
        name="primary_arm",
    )
    arm.visual(
        Box((0.095, 0.024, 0.014)),
        origin=Origin(xyz=(0.060, 0.0, -0.010)),
        material=dark_charcoal,
        name="secondary_brace",
    )
    arm.visual(
        Box((0.020, 0.028, 0.032)),
        origin=Origin(xyz=(0.168, 0.0, 0.0)),
        material=powder_black,
        name="distal_block",
    )
    arm.visual(
        Box((0.026, 0.016, 0.050)),
        origin=Origin(xyz=(0.195, 0.033, 0.0)),
        material=powder_black,
        name="tilt_yoke_left",
    )
    arm.visual(
        Box((0.026, 0.016, 0.050)),
        origin=Origin(xyz=(0.195, -0.033, 0.0)),
        material=powder_black,
        name="tilt_yoke_right",
    )
    arm.visual(
        Box((0.020, 0.070, 0.012)),
        origin=Origin(xyz=(0.185, 0.0, -0.019)),
        material=powder_black,
        name="tilt_yoke_bridge",
    )
    arm.inertial = Inertial.from_geometry(
        Box((0.230, 0.080, 0.070)),
        mass=0.9,
        origin=Origin(xyz=(0.115, 0.0, 0.0)),
    )

    model.articulation(
        "upper_post_to_arm",
        ArticulationType.REVOLUTE,
        parent=upper_post,
        child=arm,
        origin=Origin(xyz=(0.0, 0.0, 0.525)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=14.0,
            velocity=1.6,
            lower=-0.70,
            upper=0.85,
        ),
    )

    ring = model.part("ring")
    ring.visual(
        mesh_from_geometry(_ring_housing_mesh(), "magnifier_ring_housing"),
        material=powder_black,
        name="ring_housing",
    )
    ring.visual(
        Box((0.024, 0.030, 0.160)),
        origin=Origin(xyz=(0.010, 0.0, 0.0)),
        material=powder_black,
        name="rear_mount_block",
    )
    ring.visual(
        Cylinder(radius=0.011, length=0.050),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_charcoal,
        name="tilt_trunnion",
    )
    ring.visual(
        Cylinder(radius=0.0815, length=0.008),
        origin=Origin(xyz=(0.032, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=clear_glass,
        name="lens_disc",
    )
    ring.inertial = Inertial.from_geometry(
        Box((0.070, 0.230, 0.230)),
        mass=1.3,
        origin=Origin(xyz=(0.032, 0.0, 0.0)),
    )

    model.articulation(
        "arm_to_ring",
        ArticulationType.REVOLUTE,
        parent=arm,
        child=ring,
        origin=Origin(xyz=(0.195, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.8,
            lower=-0.25,
            upper=1.25,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    upper_post = object_model.get_part("upper_post")
    arm = object_model.get_part("arm")
    ring = object_model.get_part("ring")

    post_slide = object_model.get_articulation("base_to_upper_post")
    shoulder = object_model.get_articulation("upper_post_to_arm")
    tilt = object_model.get_articulation("arm_to_ring")

    def _aabb_center(aabb):
        if aabb is None:
            return None
        lo, hi = aabb
        return (
            0.5 * (lo[0] + hi[0]),
            0.5 * (lo[1] + hi[1]),
            0.5 * (lo[2] + hi[2]),
        )

    with ctx.pose({post_slide: 0.0}):
        ctx.expect_within(
            upper_post,
            base,
            axes="xy",
            inner_elem="inner_tube",
            outer_elem="outer_tube",
            margin=0.006,
            name="inner post stays centered in the outer tube at rest",
        )
        ctx.expect_overlap(
            upper_post,
            base,
            axes="z",
            elem_a="inner_tube",
            elem_b="outer_tube",
            min_overlap=0.300,
            name="collapsed post remains deeply inserted",
        )
        ctx.expect_overlap(
            upper_post,
            base,
            axes="z",
            elem_a="inner_tube",
            elem_b="top_socket",
            min_overlap=0.030,
            name="inner post seats into the upper socket at rest",
        )

    rest_post_pos = ctx.part_world_position(upper_post)
    with ctx.pose({post_slide: 0.260}):
        ctx.expect_within(
            upper_post,
            base,
            axes="xy",
            inner_elem="inner_tube",
            outer_elem="outer_tube",
            margin=0.006,
            name="extended post stays centered in the outer tube",
        )
        ctx.expect_overlap(
            upper_post,
            base,
            axes="z",
            elem_a="inner_tube",
            elem_b="outer_tube",
            min_overlap=0.070,
            name="extended post retains insertion in the outer tube",
        )
        extended_post_pos = ctx.part_world_position(upper_post)

    ctx.check(
        "post extends upward",
        rest_post_pos is not None
        and extended_post_pos is not None
        and extended_post_pos[2] > rest_post_pos[2] + 0.20,
        details=f"rest={rest_post_pos}, extended={extended_post_pos}",
    )

    rest_ring_aabb = ctx.part_element_world_aabb(ring, elem="ring_housing")
    with ctx.pose({shoulder: 0.55}):
        raised_ring_aabb = ctx.part_element_world_aabb(ring, elem="ring_housing")
    rest_ring_center = _aabb_center(rest_ring_aabb)
    raised_ring_center = _aabb_center(raised_ring_aabb)
    ctx.check(
        "arm raises the lamp head",
        rest_ring_center is not None
        and raised_ring_center is not None
        and raised_ring_center[2] > rest_ring_center[2] + 0.08,
        details=f"rest={rest_ring_center}, raised={raised_ring_center}",
    )

    tilt_down_pose = (
        tilt.motion_limits.upper if tilt.motion_limits and tilt.motion_limits.upper is not None else 1.10
    )
    neutral_lens_aabb = ctx.part_element_world_aabb(ring, elem="lens_disc")
    with ctx.pose({tilt: tilt_down_pose}):
        tilted_lens_aabb = ctx.part_element_world_aabb(ring, elem="lens_disc")
    neutral_lens_center = _aabb_center(neutral_lens_aabb)
    tilted_lens_center = _aabb_center(tilted_lens_aabb)
    ctx.check(
        "ring tilt drops the lens face toward the work area",
        neutral_lens_center is not None
        and tilted_lens_center is not None
        and tilted_lens_center[2] < neutral_lens_center[2] - 0.02
        and tilted_lens_center[0] > neutral_lens_center[0] - 0.03,
        details=f"pose={tilt_down_pose}, neutral={neutral_lens_center}, tilted={tilted_lens_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
