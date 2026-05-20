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
    CylinderGeometry,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


def _circle_profile(radius: float, *, segments: int = 56) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos(2.0 * math.pi * i / segments),
            radius * math.sin(2.0 * math.pi * i / segments),
        )
        for i in range(segments)
    ]


def _ring_mesh(outer_radius: float, inner_radius: float, depth: float):
    ring = ExtrudeWithHolesGeometry(
        _circle_profile(outer_radius),
        [_circle_profile(inner_radius)],
        depth,
        center=True,
    )
    ring.rotate_y(math.pi / 2.0)
    return ring


def _tube_shell_mesh(
    outer_radius: float,
    inner_radius: float,
    z0: float,
    z1: float,
    *,
    segments: int = 56,
):
    shell = ExtrudeWithHolesGeometry(
        _circle_profile(outer_radius, segments=segments),
        [_circle_profile(inner_radius, segments=segments)],
        z1 - z0,
        center=False,
    )
    shell.translate(0.0, 0.0, z0)
    return shell


def _leg_geom() -> CylinderGeometry:
    leg = tube_from_spline_points(
        [
            (0.0, 0.0, 0.0),
            (0.15, 0.0, -0.060),
            (0.36, 0.0, -0.155),
        ],
        radius=0.011,
        samples_per_segment=18,
        radial_segments=18,
        cap_ends=True,
    )
    leg.merge(
        CylinderGeometry(radius=0.014, height=0.048, radial_segments=18).rotate_x(
            math.pi / 2.0
        )
    )
    return leg


def _handle_geom() -> CylinderGeometry:
    handle = tube_from_spline_points(
        [
            (0.0, 0.0, 0.0),
            (0.0, 0.030, -0.040),
            (0.0, 0.060, -0.095),
            (0.0, 0.060, -0.175),
        ],
        radius=0.009,
        samples_per_segment=18,
        radial_segments=18,
        cap_ends=True,
        up_hint=(1.0, 0.0, 0.0),
    )
    handle.merge(
        CylinderGeometry(radius=0.012, height=0.042, radial_segments=18).rotate_y(
            math.pi / 2.0
        )
    )
    handle.merge(
        CylinderGeometry(radius=0.013, height=0.090, radial_segments=18)
        .rotate_y(math.pi / 2.0)
        .translate(0.0, 0.060, -0.175)
    )
    return handle


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="ring_light_boom_stand")

    satin_black = model.material("satin_black", rgba=(0.14, 0.14, 0.15, 1.0))
    graphite = model.material("graphite", rgba=(0.22, 0.23, 0.25, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.09, 1.0))
    diffuser = model.material("diffuser", rgba=(0.94, 0.95, 0.94, 1.0))
    knob_black = model.material("knob_black", rgba=(0.10, 0.10, 0.11, 1.0))

    base_stand = model.part("base_stand")
    base_stand.visual(
        Cylinder(radius=0.050, length=0.090),
        origin=Origin(xyz=(0.0, 0.0, 0.145)),
        material=satin_black,
        name="base_hub",
    )
    base_stand.visual(
        Cylinder(radius=0.032, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, 0.225)),
        material=satin_black,
        name="lower_collar",
    )
    base_stand.visual(
        Cylinder(radius=0.024, length=0.340),
        origin=Origin(xyz=(0.0, 0.0, 0.430)),
        material=graphite,
        name="lower_mast",
    )
    base_stand.visual(
        mesh_from_geometry(
            _tube_shell_mesh(0.0245, 0.0195, 0.600, 0.955),
            "outer_sleeve_shell",
        ),
        material=graphite,
        name="outer_sleeve",
    )
    base_stand.visual(
        mesh_from_geometry(
            _tube_shell_mesh(0.0290, 0.0195, 0.955, 1.000),
            "mast_lock_collar_shell",
        ),
        material=satin_black,
        name="mast_lock_collar",
    )
    base_stand.visual(
        Cylinder(radius=0.005, length=0.040),
        origin=Origin(xyz=(0.036, 0.0, 0.978), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=graphite,
        name="lock_handle_stem",
    )
    base_stand.visual(
        Sphere(radius=0.014),
        origin=Origin(xyz=(0.062, 0.0, 0.978)),
        material=knob_black,
        name="lock_handle_knob",
    )
    base_stand.inertial = Inertial.from_geometry(
        Box((0.92, 0.92, 1.05)),
        mass=4.2,
        origin=Origin(xyz=(0.0, 0.0, 0.525)),
    )

    upper_mast_boom = model.part("upper_mast_boom")
    upper_mast_boom.visual(
        Cylinder(radius=0.016, length=0.800),
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
        material=graphite,
        name="inner_mast",
    )
    upper_mast_boom.visual(
        Cylinder(radius=0.023, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.430)),
        material=satin_black,
        name="boom_collar",
    )
    boom_structure = tube_from_spline_points(
        [
            (0.0, 0.0, 0.420),
            (0.070, 0.0, 0.380),
            (0.140, 0.0, 0.250),
        ],
        radius=0.014,
        samples_per_segment=18,
        radial_segments=18,
        cap_ends=True,
    )
    boom_structure.merge(
        tube_from_spline_points(
            [
                (0.140, -0.205, 0.420),
                (0.140, -0.205, 0.290),
                (0.140, 0.0, 0.250),
                (0.140, 0.205, 0.290),
                (0.140, 0.205, 0.420),
            ],
            radius=0.011,
            samples_per_segment=18,
            radial_segments=18,
            cap_ends=True,
            up_hint=(1.0, 0.0, 0.0),
        )
    )
    upper_mast_boom.visual(
        mesh_from_geometry(boom_structure, "boom_and_yoke"),
        material=satin_black,
        name="boom_and_yoke",
    )
    upper_mast_boom.inertial = Inertial.from_geometry(
        Box((0.30, 0.46, 0.86)),
        mass=1.3,
        origin=Origin(xyz=(0.08, 0.0, 0.140)),
    )

    ring_head = model.part("ring_head")
    ring_head.visual(
        mesh_from_geometry(_ring_mesh(0.190, 0.125, 0.050), "ring_light_annulus"),
        material=diffuser,
        name="ring_body",
    )
    ring_head.visual(
        Box((0.040, 0.090, 0.080)),
        origin=Origin(xyz=(-0.028, 0.0, -0.145)),
        material=satin_black,
        name="driver_box",
    )
    ring_head.visual(
        Cylinder(radius=0.015, length=0.034),
        origin=Origin(xyz=(-0.030, -0.198, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=knob_black,
        name="left_tilt_knob",
    )
    ring_head.visual(
        Box((0.034, 0.024, 0.060)),
        origin=Origin(xyz=(-0.020, 0.202, -0.010)),
        material=satin_black,
        name="handle_mount_lug",
    )
    ring_head.visual(
        Cylinder(radius=0.008, length=0.028),
        origin=Origin(
            xyz=(-0.020, 0.206182, -0.010),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=graphite,
        name="handle_hinge_boss",
    )
    ring_head.inertial = Inertial.from_geometry(
        Box((0.060, 0.400, 0.420)),
        mass=1.6,
        origin=Origin(xyz=(0.0, 0.0, -0.010)),
    )

    side_handle = model.part("side_handle")
    side_handle.visual(
        mesh_from_geometry(_handle_geom(), "folding_side_handle"),
        material=knob_black,
        name="side_handle_body",
    )
    side_handle.inertial = Inertial.from_geometry(
        Box((0.060, 0.120, 0.220)),
        mass=0.18,
        origin=Origin(xyz=(0.0, 0.040, -0.090)),
    )

    leg_angles = (0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)
    leg_parts = []
    for idx, angle in enumerate(leg_angles):
        leg = model.part(f"leg_{idx}")
        leg.visual(
            mesh_from_geometry(_leg_geom(), f"folding_leg_mesh_{idx}"),
            material=satin_black,
            name="leg_frame",
        )
        leg.visual(
            Sphere(radius=0.016),
            origin=Origin(xyz=(0.360, 0.0, -0.155)),
            material=rubber,
            name="foot_pad",
        )
        leg.inertial = Inertial.from_geometry(
            Box((0.39, 0.05, 0.20)),
            mass=0.28,
            origin=Origin(xyz=(0.190, 0.0, -0.080)),
        )
        leg_parts.append((leg, angle))

    model.articulation(
        "stand_to_upper_mast",
        ArticulationType.PRISMATIC,
        parent=base_stand,
        child=upper_mast_boom,
        origin=Origin(xyz=(0.0, 0.0, 0.960)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.20,
            lower=0.0,
            upper=0.220,
        ),
    )
    model.articulation(
        "boom_to_ring_head",
        ArticulationType.REVOLUTE,
        parent=upper_mast_boom,
        child=ring_head,
        origin=Origin(xyz=(0.200, 0.0, 0.420)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.5,
            lower=math.radians(-20.0),
            upper=math.radians(105.0),
        ),
    )
    model.articulation(
        "ring_to_side_handle",
        ArticulationType.REVOLUTE,
        parent=ring_head,
        child=side_handle,
        origin=Origin(xyz=(-0.020, 0.226, -0.010)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.5,
            lower=math.radians(-15.0),
            upper=math.radians(95.0),
        ),
    )
    for idx, (leg, angle) in enumerate(leg_parts):
        model.articulation(
            f"hub_to_leg_{idx}",
            ArticulationType.REVOLUTE,
            parent=base_stand,
            child=leg,
            origin=Origin(
                xyz=(0.064 * math.cos(angle), 0.064 * math.sin(angle), 0.160),
                rpy=(0.0, 0.0, angle),
            ),
            axis=(0.0, -1.0, 0.0),
            motion_limits=MotionLimits(
                effort=20.0,
                velocity=1.6,
                lower=0.0,
                upper=math.radians(70.0),
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base_stand = object_model.get_part("base_stand")
    upper_mast_boom = object_model.get_part("upper_mast_boom")
    ring_head = object_model.get_part("ring_head")
    side_handle = object_model.get_part("side_handle")
    mast_slide = object_model.get_articulation("stand_to_upper_mast")
    ring_tilt = object_model.get_articulation("boom_to_ring_head")
    handle_fold = object_model.get_articulation("ring_to_side_handle")
    leg_0 = object_model.get_part("leg_0")
    leg_joint_0 = object_model.get_articulation("hub_to_leg_0")

    ctx.expect_within(
        upper_mast_boom,
        base_stand,
        axes="xy",
        inner_elem="inner_mast",
        outer_elem="outer_sleeve",
        margin=0.004,
        name="inner mast stays centered in sleeve",
    )
    ctx.expect_overlap(
        upper_mast_boom,
        base_stand,
        axes="z",
        elem_a="inner_mast",
        elem_b="outer_sleeve",
        min_overlap=0.140,
        name="mast retains insertion at rest",
    )

    rest_mast_pos = ctx.part_world_position(upper_mast_boom)
    with ctx.pose({mast_slide: mast_slide.motion_limits.upper}):
        ctx.expect_within(
            upper_mast_boom,
            base_stand,
            axes="xy",
            inner_elem="inner_mast",
            outer_elem="outer_sleeve",
            margin=0.004,
            name="extended mast stays centered in sleeve",
        )
        ctx.expect_overlap(
            upper_mast_boom,
            base_stand,
            axes="z",
            elem_a="inner_mast",
            elem_b="outer_sleeve",
            min_overlap=0.100,
            name="mast retains insertion when extended",
        )
        extended_mast_pos = ctx.part_world_position(upper_mast_boom)
    ctx.check(
        "mast extends upward",
        rest_mast_pos is not None
        and extended_mast_pos is not None
        and extended_mast_pos[2] > rest_mast_pos[2] + 0.15,
        details=f"rest={rest_mast_pos}, extended={extended_mast_pos}",
    )

    def _aabb_center(aabb):
        if aabb is None:
            return None
        mins, maxs = aabb
        return tuple((mins[i] + maxs[i]) * 0.5 for i in range(3))

    rest_driver_center = _aabb_center(
        ctx.part_element_world_aabb(ring_head, elem="driver_box")
    )
    with ctx.pose({ring_tilt: math.radians(50.0)}):
        tilted_driver_center = _aabb_center(
            ctx.part_element_world_aabb(ring_head, elem="driver_box")
        )
    ctx.check(
        "ring head tilts upward",
        rest_driver_center is not None
        and tilted_driver_center is not None
        and tilted_driver_center[2] > rest_driver_center[2] + 0.03,
        details=f"rest={rest_driver_center}, tilted={tilted_driver_center}",
    )

    rest_foot_center = _aabb_center(ctx.part_element_world_aabb(leg_0, elem="foot_pad"))
    with ctx.pose({leg_joint_0: leg_joint_0.motion_limits.upper}):
        folded_foot_center = _aabb_center(ctx.part_element_world_aabb(leg_0, elem="foot_pad"))
    ctx.check(
        "leg folds upward from deployed pose",
        rest_foot_center is not None
        and folded_foot_center is not None
        and folded_foot_center[2] > rest_foot_center[2] + 0.12,
        details=f"rest={rest_foot_center}, folded={folded_foot_center}",
    )

    rest_handle_center = _aabb_center(
        ctx.part_element_world_aabb(side_handle, elem="side_handle_body")
    )
    with ctx.pose({handle_fold: math.radians(75.0)}):
        folded_handle_center = _aabb_center(
            ctx.part_element_world_aabb(side_handle, elem="side_handle_body")
        )
    ctx.check(
        "side handle folds upward",
        rest_handle_center is not None
        and folded_handle_center is not None
        and folded_handle_center[2] > rest_handle_center[2] + 0.04,
        details=f"rest={rest_handle_center}, folded={folded_handle_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
