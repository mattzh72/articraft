from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    CylinderGeometry,
    ExtrudeGeometry,
    KnobGeometry,
    KnobGrip,
    KnobIndicator,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
    section_loft,
    superellipse_profile,
    tube_from_spline_points,
)


def _xy_loop(
    *,
    width: float,
    depth: float,
    radius: float,
    z: float,
    cx: float = 0.0,
    cy: float = 0.0,
    segments: int = 10,
) -> list[tuple[float, float, float]]:
    return [
        (cx + x, cy + y, z)
        for x, y in rounded_rect_profile(width, depth, radius, corner_segments=segments)
    ]


def _yz_superellipse(
    *,
    x: float,
    width: float,
    height: float,
    zc: float,
    segments: int = 56,
) -> list[tuple[float, float, float]]:
    return [(x, y, zc + z) for y, z in superellipse_profile(width, height, 2.8, segments=segments)]


def _head_shell_mesh():
    sections = [
        _yz_superellipse(x=0.030, width=0.140, height=0.100, zc=0.015),
        _yz_superellipse(x=0.170, width=0.205, height=0.145, zc=0.000),
        _yz_superellipse(x=0.340, width=0.170, height=0.125, zc=-0.014),
        _yz_superellipse(x=0.480, width=0.110, height=0.090, zc=-0.028),
    ]
    return mesh_from_geometry(section_loft(sections), "long_nose_head_shell")


def _spiral_hook_mesh():
    geom = CylinderGeometry(radius=0.010, height=0.048, radial_segments=32).translate(
        0.0, 0.0, -0.024
    )
    geom.merge(
        CylinderGeometry(radius=0.018, height=0.024, radial_segments=36).translate(
            0.0, 0.0, -0.052
        )
    )

    pts: list[tuple[float, float, float]] = []
    turns = 1.45
    samples = 34
    for i in range(samples):
        t = i / (samples - 1)
        a = 2.0 * math.pi * turns * t
        radius = 0.015 + 0.030 * t
        z = -0.058 - 0.092 * t
        pts.append((radius * math.cos(a), radius * math.sin(a), z))
    pts.extend(
        [
            (0.050, -0.008, -0.158),
            (0.034, -0.024, -0.170),
            (0.012, -0.018, -0.164),
        ]
    )
    geom.merge(
        tube_from_spline_points(
            pts,
            radius=0.0052,
            samples_per_segment=9,
            radial_segments=18,
            cap_ends=True,
            up_hint=(0.0, 1.0, 0.0),
        )
    )
    return mesh_from_geometry(geom, "spiral_dough_hook")


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="soft_stand_mixer")

    enamel = model.material("warm_enamel", rgba=(0.88, 0.22, 0.18, 1.0))
    enamel_shadow = model.material("enamel_shadow", rgba=(0.58, 0.08, 0.07, 1.0))
    stainless = model.material("brushed_stainless", rgba=(0.86, 0.86, 0.82, 1.0))
    dark = model.material("black_rubber", rgba=(0.035, 0.035, 0.040, 1.0))
    chrome = model.material("polished_chrome", rgba=(0.96, 0.95, 0.90, 1.0))
    graphite = model.material("graphite_detail", rgba=(0.22, 0.23, 0.24, 1.0))

    base = model.part("base")
    base_shell = section_loft(
        [
            _xy_loop(width=0.550, depth=0.330, radius=0.100, z=0.000, cx=0.055),
            _xy_loop(width=0.600, depth=0.365, radius=0.115, z=0.030, cx=0.055),
            _xy_loop(width=0.535, depth=0.315, radius=0.090, z=0.060, cx=0.060),
        ]
    )
    base.visual(mesh_from_geometry(base_shell, "soft_rounded_base"), material=enamel, name="base_shell")
    base.visual(
        Box((0.450, 0.250, 0.010)),
        origin=Origin(xyz=(0.070, 0.0, -0.005)),
        material=dark,
        name="rubber_foot",
    )

    neck_mesh = section_loft(
        [
            _xy_loop(width=0.190, depth=0.165, radius=0.055, z=0.052, cx=-0.130),
            _xy_loop(width=0.155, depth=0.145, radius=0.050, z=0.175, cx=-0.135),
            _xy_loop(width=0.108, depth=0.118, radius=0.040, z=0.340, cx=-0.150),
        ]
    )
    base.visual(mesh_from_geometry(neck_mesh, "rear_pedestal_neck"), material=enamel, name="pedestal")

    # Bowl carriage guide rails, permanently attached to the base.
    for y in (-0.095, 0.095):
        base.visual(
            Box((0.280, 0.018, 0.018)),
            origin=Origin(xyz=(0.070, 1.39 * y, 0.069)),
            material=enamel_shadow,
            name=f"carriage_rail_{'n' if y < 0 else 'p'}",
        )
    base.visual(
        Box((0.210, 0.018, 0.012)),
        origin=Origin(xyz=(0.065, 0.0, 0.054)),
        material=enamel_shadow,
        name="carriage_slot",
    )

    # Rear hinge brackets and side mounting pads for the controls.
    base.visual(
        Box((0.115, 0.210, 0.018)),
        origin=Origin(xyz=(-0.150, 0.0, 0.326)),
        material=enamel,
        name="hinge_yoke_bridge",
    )
    for y in (-0.096, 0.096):
        base.visual(
            Box((0.050, 0.018, 0.080)),
            origin=Origin(xyz=(-0.150, y, 0.354)),
            material=enamel,
            name=f"hinge_cheek_{'n' if y < 0 else 'p'}",
        )
        base.visual(
            Cylinder(radius=0.026, length=0.050),
            origin=Origin(xyz=(-0.150, y, 0.390), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=chrome,
            name=f"hinge_knuckle_{'n' if y < 0 else 'p'}",
        )
    base.visual(
        Box((0.085, 0.022, 0.052)),
        origin=Origin(xyz=(-0.092, 0.076, 0.205)),
        material=enamel_shadow,
        name="speed_mount_pad",
    )
    base.visual(
        Box((0.075, 0.040, 0.030)),
        origin=Origin(xyz=(-0.130, -0.067, 0.286)),
        material=enamel_shadow,
        name="lock_mount_pad",
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.255, 0.225, 0.030)),
        origin=Origin(xyz=(0.050, 0.0, 0.015)),
        material=enamel_shadow,
        name="sliding_stage",
    )
    carriage.visual(
        Cylinder(radius=0.078, length=0.012),
        origin=Origin(xyz=(0.080, 0.0, 0.036)),
        material=chrome,
        name="carriage_pad",
    )
    carriage.visual(
        Box((0.050, 0.235, 0.028)),
        origin=Origin(xyz=(0.165, 0.0, 0.017)),
        material=enamel,
        name="front_lip",
    )

    bowl = model.part("bowl")
    bowl_profile_outer = [
        (0.038, 0.000),
        (0.055, 0.012),
        (0.087, 0.045),
        (0.125, 0.120),
        (0.142, 0.165),
        (0.156, 0.178),
        (0.146, 0.188),
    ]
    bowl_profile_inner = [
        (0.026, 0.010),
        (0.050, 0.022),
        (0.078, 0.050),
        (0.112, 0.122),
        (0.134, 0.166),
        (0.140, 0.177),
    ]
    bowl_geom = LatheGeometry.from_shell_profiles(
        bowl_profile_outer,
        bowl_profile_inner,
        segments=80,
        start_cap="flat",
        end_cap="round",
        lip_samples=12,
    )
    bowl.visual(mesh_from_geometry(bowl_geom, "rolled_rim_bowl"), material=stainless, name="bowl_shell")

    head = model.part("head")
    head.visual(_head_shell_mesh(), material=enamel, name="head_shell")
    head.visual(
        Cylinder(radius=0.022, length=0.105),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="hinge_barrel",
    )
    head.visual(
        Box((0.040, 0.100, 0.035)),
        origin=Origin(xyz=(0.018, 0.0, 0.012)),
        material=enamel,
        name="hinge_web",
    )
    head.visual(
        Cylinder(radius=0.047, length=0.018),
        origin=Origin(xyz=(0.489, 0.0, -0.030), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="nose_cap",
    )
    head.visual(
        Cylinder(radius=0.043, length=0.036),
        origin=Origin(xyz=(0.300, 0.0, -0.087)),
        material=chrome,
        name="drive_boss",
    )

    hook = model.part("hook")
    hook.visual(_spiral_hook_mesh(), material=stainless, name="spiral_hook")

    speed_selector = model.part("speed_selector")
    speed_mesh = mesh_from_geometry(
        KnobGeometry(
            0.052,
            0.024,
            body_style="skirted",
            top_diameter=0.042,
            edge_radius=0.0015,
            grip=KnobGrip(style="fluted", count=18, depth=0.0011),
            indicator=KnobIndicator(style="line", mode="raised", angle_deg=0.0),
            center=False,
        ),
        "base_speed_selector_knob",
    )
    speed_selector.visual(speed_mesh, material=graphite, name="selector_knob")

    head_lock = model.part("head_lock")
    head_lock.visual(
        Box((0.036, 0.020, 0.014)),
        origin=Origin(xyz=(0.0, -0.010, 0.0)),
        material=chrome,
        name="lock_stem",
    )
    head_lock.visual(
        Box((0.060, 0.016, 0.026)),
        origin=Origin(xyz=(0.0, -0.026, 0.0)),
        material=graphite,
        name="lock_tab",
    )

    model.articulation(
        "base_to_carriage",
        ArticulationType.PRISMATIC,
        parent=base,
        child=carriage,
        origin=Origin(xyz=(0.060, 0.0, 0.060)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.12, lower=0.0, upper=0.050),
    )
    model.articulation(
        "carriage_to_bowl",
        ArticulationType.FIXED,
        parent=carriage,
        child=bowl,
        origin=Origin(xyz=(0.080, 0.0, 0.042)),
    )
    model.articulation(
        "base_to_head",
        ArticulationType.REVOLUTE,
        parent=base,
        child=head,
        origin=Origin(xyz=(-0.150, 0.0, 0.390)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=22.0, velocity=1.2, lower=0.0, upper=math.radians(62.0)),
    )
    model.articulation(
        "head_to_hook",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=hook,
        origin=Origin(xyz=(0.300, 0.0, -0.105)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=7.0, velocity=20.0),
    )
    model.articulation(
        "base_to_speed_selector",
        ArticulationType.REVOLUTE,
        parent=base,
        child=speed_selector,
        origin=Origin(xyz=(-0.092, 0.087, 0.205), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.8, velocity=5.0, lower=-1.20, upper=1.20),
    )
    model.articulation(
        "base_to_head_lock",
        ArticulationType.PRISMATIC,
        parent=base,
        child=head_lock,
        origin=Origin(xyz=(-0.130, -0.087, 0.286)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=0.06, lower=0.0, upper=0.024),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    carriage = object_model.get_part("carriage")
    bowl = object_model.get_part("bowl")
    head = object_model.get_part("head")
    hook = object_model.get_part("hook")
    speed_selector = object_model.get_part("speed_selector")
    head_lock = object_model.get_part("head_lock")

    carriage_joint = object_model.get_articulation("base_to_carriage")
    bowl_mount = object_model.get_articulation("carriage_to_bowl")
    head_hinge = object_model.get_articulation("base_to_head")
    hook_spin = object_model.get_articulation("head_to_hook")
    speed_joint = object_model.get_articulation("base_to_speed_selector")
    lock_joint = object_model.get_articulation("base_to_head_lock")

    ctx.check(
        "required joint types",
        carriage_joint.articulation_type == ArticulationType.PRISMATIC
        and bowl_mount.articulation_type == ArticulationType.FIXED
        and head_hinge.articulation_type == ArticulationType.REVOLUTE
        and hook_spin.articulation_type == ArticulationType.CONTINUOUS
        and speed_joint.articulation_type == ArticulationType.REVOLUTE
        and lock_joint.articulation_type == ArticulationType.PRISMATIC,
    )
    ctx.check(
        "short bowl carriage travel",
        carriage_joint.motion_limits is not None
        and carriage_joint.motion_limits.lower == 0.0
        and 0.035 <= carriage_joint.motion_limits.upper <= 0.070,
    )
    ctx.check(
        "single rear tilt hinge",
        head_hinge.motion_limits is not None
        and head_hinge.motion_limits.lower == 0.0
        and head_hinge.axis == (0.0, -1.0, 0.0),
    )

    ctx.allow_overlap(
        bowl,
        hook,
        elem_a="bowl_shell",
        elem_b="spiral_hook",
        reason=(
            "The spiral hook intentionally hangs inside the visibly hollow bowl; "
            "the bowl shell is a closed thin-wall proxy so the contained hook is "
            "waived as a local hollow-vessel occupancy, not a surface intersection."
        ),
    )

    ctx.expect_contact(
        bowl,
        carriage,
        elem_a="bowl_shell",
        elem_b="carriage_pad",
        contact_tol=0.0015,
        name="bowl rests on carriage pad",
    )
    ctx.expect_within(
        hook,
        bowl,
        axes="xy",
        inner_elem="spiral_hook",
        outer_elem="bowl_shell",
        margin=0.006,
        name="spiral hook stays inside bowl opening",
    )

    rest_carriage = ctx.part_world_position(carriage)
    with ctx.pose({carriage_joint: carriage_joint.motion_limits.upper}):
        extended_carriage = ctx.part_world_position(carriage)
    ctx.check(
        "carriage slides forward",
        rest_carriage is not None
        and extended_carriage is not None
        and extended_carriage[0] > rest_carriage[0] + 0.035,
        details=f"rest={rest_carriage}, extended={extended_carriage}",
    )

    closed_head_aabb = ctx.part_world_aabb(head)
    with ctx.pose({head_hinge: head_hinge.motion_limits.upper}):
        raised_head_aabb = ctx.part_world_aabb(head)
    ctx.check(
        "head tilts upward",
        closed_head_aabb is not None
        and raised_head_aabb is not None
        and raised_head_aabb[1][2] > closed_head_aabb[1][2] + 0.040,
        details=f"closed={closed_head_aabb}, raised={raised_head_aabb}",
    )

    rest_lock = ctx.part_world_position(head_lock)
    with ctx.pose({lock_joint: lock_joint.motion_limits.upper}):
        shifted_lock = ctx.part_world_position(head_lock)
    ctx.check(
        "head lock translates outward",
        rest_lock is not None and shifted_lock is not None and shifted_lock[1] < rest_lock[1] - 0.015,
        details=f"rest={rest_lock}, shifted={shifted_lock}",
    )
    ctx.expect_origin_distance(
        speed_selector,
        object_model.get_part("base"),
        axes="y",
        min_dist=0.070,
        name="speed selector protrudes from base side",
    )

    return ctx.report()


object_model = build_object_model()
