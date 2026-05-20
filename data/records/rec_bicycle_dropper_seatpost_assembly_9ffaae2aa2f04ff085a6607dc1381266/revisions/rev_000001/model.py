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
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    section_loft,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    def tube_shell(name: str, *, outer_radius: float, inner_radius: float, length: float):
        outer_profile = [(outer_radius, 0.0), (outer_radius, length)]
        inner_profile = [(inner_radius, 0.0), (inner_radius, length)]
        return mesh_from_geometry(
            LatheGeometry.from_shell_profiles(
                outer_profile,
                inner_profile,
                segments=56,
            ),
            name,
        )

    def saddle_section(
        *,
        y: float,
        width: float,
        thickness: float,
        z_center: float,
        samples: int = 28,
    ) -> list[tuple[float, float, float]]:
        points: list[tuple[float, float, float]] = []
        for index in range(samples):
            angle = (2.0 * math.pi * index) / samples
            c = math.cos(angle)
            s = math.sin(angle)
            x = 0.5 * width * c
            z_scale = 1.0 if s >= 0.0 else 0.62
            z = z_center + 0.5 * thickness * s * z_scale
            points.append((x, y, z))
        return points

    def saddle_shell_mesh():
        return mesh_from_geometry(
            section_loft(
                [
                    saddle_section(y=-0.128, width=0.142, thickness=0.030, z_center=0.039),
                    saddle_section(y=-0.060, width=0.138, thickness=0.044, z_center=0.047),
                    saddle_section(y=0.010, width=0.118, thickness=0.040, z_center=0.038),
                    saddle_section(y=0.085, width=0.072, thickness=0.027, z_center=0.024),
                    saddle_section(y=0.138, width=0.028, thickness=0.016, z_center=0.018),
                ]
            ),
            "saddle_shell",
        )

    model = ArticulatedObject(name="dropper_seatpost")

    anodized_black = model.material("anodized_black", rgba=(0.10, 0.10, 0.11, 1.0))
    satin_black = model.material("satin_black", rgba=(0.14, 0.14, 0.15, 1.0))
    hardcoat_black = model.material("hardcoat_black", rgba=(0.08, 0.08, 0.09, 1.0))
    steel = model.material("steel", rgba=(0.66, 0.68, 0.71, 1.0))
    bolt_black = model.material("bolt_black", rgba=(0.18, 0.18, 0.19, 1.0))

    outer_tube = model.part("outer_tube")
    outer_tube.visual(
        tube_shell(
            "outer_tube_shell",
            outer_radius=0.0158,
            inner_radius=0.0142,
            length=0.228,
        ),
        material=anodized_black,
        name="outer_shell",
    )
    outer_tube.inertial = Inertial.from_geometry(
        Box((0.040, 0.040, 0.228)),
        mass=0.55,
        origin=Origin(xyz=(0.0, 0.0, 0.114)),
    )

    binder_ring = model.part("binder_ring")
    for x_pos, visual_name in ((0.01815, "binder_ring_shell"), (-0.01815, "binder_ring_shell_left")):
        binder_ring.visual(
            Box((0.0047, 0.041, 0.014)),
            origin=Origin(xyz=(x_pos, 0.0, 0.007)),
            material=satin_black,
            name=visual_name,
        )
    for y_pos, visual_name in ((0.01815, "binder_ring_front"), (-0.01815, "binder_ring_back")):
        binder_ring.visual(
            Box((0.041, 0.0047, 0.014)),
            origin=Origin(xyz=(0.0, y_pos, 0.007)),
            material=satin_black,
            name=visual_name,
        )
    for x_sign in (-1.0, 1.0):
        binder_ring.visual(
            Box((0.008, 0.009, 0.014)),
            origin=Origin(xyz=(0.008 * x_sign, -0.022, 0.007)),
            material=satin_black,
            name=f"binder_ear_{'left' if x_sign < 0.0 else 'right'}",
        )
    binder_ring.visual(
        Cylinder(radius=0.0032, length=0.032),
        origin=Origin(xyz=(0.0, -0.022, 0.007), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="binder_bolt",
    )
    binder_ring.visual(
        Cylinder(radius=0.0052, length=0.0045),
        origin=Origin(xyz=(0.018, -0.022, 0.007), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bolt_black,
        name="binder_bolt_head",
    )
    binder_ring.inertial = Inertial.from_geometry(
        Box((0.042, 0.050, 0.014)),
        mass=0.05,
        origin=Origin(xyz=(0.0, 0.0, 0.007)),
    )

    locking_collar = model.part("locking_collar")
    for x_pos, visual_name in ((0.01675, "collar_ring"), (-0.01675, "collar_ring_left")):
        locking_collar.visual(
            Box((0.0075, 0.041, 0.020)),
            origin=Origin(xyz=(x_pos, 0.0, 0.010)),
            material=satin_black,
            name=visual_name,
        )
    for y_pos, visual_name in ((0.01675, "collar_ring_front"), (-0.01675, "collar_ring_back")):
        locking_collar.visual(
            Box((0.041, 0.0075, 0.020)),
            origin=Origin(xyz=(0.0, y_pos, 0.010)),
            material=satin_black,
            name=visual_name,
        )
    locking_collar.visual(
        Box((0.014, 0.018, 0.020)),
        origin=Origin(xyz=(0.027, 0.0, 0.010)),
        material=satin_black,
        name="collar_boss",
    )
    locking_collar.visual(
        Cylinder(radius=0.0042, length=0.020),
        origin=Origin(xyz=(0.034, 0.0, 0.010), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="lever_pivot_barrel",
    )
    locking_collar.visual(
        Cylinder(radius=0.0030, length=0.018),
        origin=Origin(xyz=(0.026, 0.0, 0.004), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bolt_black,
        name="collar_fastener",
    )
    locking_collar.inertial = Inertial.from_geometry(
        Box((0.044, 0.042, 0.020)),
        mass=0.08,
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
    )

    lock_lever = model.part("lock_lever")
    lock_lever.visual(
        Box((0.028, 0.010, 0.008)),
        origin=Origin(xyz=(0.016, 0.0, -0.006)),
        material=hardcoat_black,
        name="lever_arm",
    )
    lock_lever.visual(
        Cylinder(radius=0.0050, length=0.022),
        origin=Origin(xyz=(0.034, 0.0, -0.010), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hardcoat_black,
        name="lever_paddle",
    )
    lock_lever.inertial = Inertial.from_geometry(
        Box((0.046, 0.022, 0.018)),
        mass=0.03,
        origin=Origin(xyz=(0.022, 0.0, -0.008)),
    )

    inner_post = model.part("inner_post")
    inner_post.visual(
        tube_shell(
            "inner_post_shell",
            outer_radius=0.0130,
            inner_radius=0.0112,
            length=0.290,
        ),
        material=hardcoat_black,
        name="stanchion_shell",
    )
    inner_post.visual(
        Box((0.028, 0.034, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.299)),
        material=anodized_black,
        name="head_block",
    )
    inner_post.visual(
        Box((0.054, 0.080, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.310)),
        material=steel,
        name="lower_cradle",
    )
    inner_post.visual(
        Box((0.052, 0.052, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.329)),
        material=steel,
        name="upper_clamp_plate",
    )
    for y_pos, label in ((-0.026, "rear"), (0.026, "front")):
        inner_post.visual(
            Cylinder(radius=0.0028, length=0.024),
            origin=Origin(xyz=(0.0, y_pos, 0.317)),
            material=steel,
            name=f"{label}_bolt_shank",
        )
        inner_post.visual(
            Cylinder(radius=0.0050, length=0.004),
            origin=Origin(xyz=(0.0, y_pos, 0.331)),
            material=bolt_black,
            name=f"{label}_bolt_head",
        )
    inner_post.inertial = Inertial.from_geometry(
        Box((0.060, 0.080, 0.335)),
        mass=0.45,
        origin=Origin(xyz=(0.0, 0.0, 0.168)),
    )

    saddle = model.part("saddle")
    saddle.visual(saddle_shell_mesh(), material=satin_black, name="saddle_shell")
    for x_pos, label in ((-0.023, "right"), (0.023, "left")):
        rail = tube_from_spline_points(
            [
                (x_pos, -0.112, 0.012),
                (x_pos, -0.070, 0.004),
                (x_pos, -0.032, -0.002),
                (x_pos, 0.032, -0.002),
                (x_pos, 0.082, 0.004),
                (x_pos, 0.112, 0.010),
            ],
            radius=0.0030,
            samples_per_segment=16,
            radial_segments=18,
            cap_ends=True,
        )
        saddle.visual(
            mesh_from_geometry(rail, f"saddle_{label}_rail"),
            material=steel,
            name=f"{label}_rail",
        )
        for y_mount, mount_name in ((-0.064, "rear"), (0.066, "front")):
            saddle.visual(
                Box((0.012, 0.034, 0.030)),
                origin=Origin(xyz=(x_pos, y_mount, 0.015)),
                material=hardcoat_black,
                name=f"{label}_{mount_name}_mount",
            )
    saddle.inertial = Inertial.from_geometry(
        Box((0.150, 0.280, 0.080)),
        mass=0.32,
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
    )

    model.articulation(
        "outer_tube_to_binder_ring",
        ArticulationType.FIXED,
        parent=outer_tube,
        child=binder_ring,
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
    )
    model.articulation(
        "outer_tube_to_locking_collar",
        ArticulationType.FIXED,
        parent=outer_tube,
        child=locking_collar,
        origin=Origin(xyz=(0.0, 0.0, 0.228)),
    )
    model.articulation(
        "outer_tube_to_inner_post",
        ArticulationType.PRISMATIC,
        parent=outer_tube,
        child=inner_post,
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=14.0,
            velocity=0.40,
            lower=0.0,
            upper=0.120,
        ),
    )
    model.articulation(
        "locking_collar_to_lever",
        ArticulationType.REVOLUTE,
        parent=locking_collar,
        child=lock_lever,
        origin=Origin(xyz=(0.034, 0.0, 0.010)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=4.0,
            lower=0.0,
            upper=1.25,
        ),
    )
    model.articulation(
        "inner_post_to_saddle",
        ArticulationType.FIXED,
        parent=inner_post,
        child=saddle,
        origin=Origin(xyz=(0.0, 0.0, 0.317)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    outer_tube = object_model.get_part("outer_tube")
    binder_ring = object_model.get_part("binder_ring")
    locking_collar = object_model.get_part("locking_collar")
    lock_lever = object_model.get_part("lock_lever")
    inner_post = object_model.get_part("inner_post")
    saddle = object_model.get_part("saddle")

    post_travel = object_model.get_articulation("outer_tube_to_inner_post")
    lever_joint = object_model.get_articulation("locking_collar_to_lever")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(
        binder_ring,
        outer_tube,
        contact_tol=0.001,
        name="binder ring clears but hugs outer tube",
    )
    ctx.expect_overlap(
        binder_ring,
        outer_tube,
        axes="xy",
        min_overlap=0.030,
        name="binder ring encircles outer tube",
    )
    ctx.expect_gap(
        locking_collar,
        outer_tube,
        axis="z",
        max_gap=0.0005,
        max_penetration=0.0,
        name="locking collar seats on outer tube",
    )
    ctx.expect_within(
        inner_post,
        outer_tube,
        axes="xy",
        inner_elem="stanchion_shell",
        outer_elem="outer_shell",
        margin=0.0015,
        name="inner post stays within outer tube diameter",
    )
    ctx.expect_within(
        inner_post,
        locking_collar,
        axes="xy",
        inner_elem="stanchion_shell",
        margin=0.0015,
        name="inner post stays within locking collar diameter",
    )
    ctx.expect_contact(
        saddle,
        inner_post,
        name="saddle rails contact clamp head",
    )
    ctx.check(
        "dropper joint axis is vertical",
        tuple(post_travel.axis) == (0.0, 0.0, 1.0),
        details=f"axis was {post_travel.axis}",
    )
    ctx.check(
        "lock lever axis is lateral",
        tuple(lever_joint.axis) == (0.0, 1.0, 0.0),
        details=f"axis was {lever_joint.axis}",
    )

    saddle_rest = ctx.part_world_position(saddle)
    with ctx.pose({post_travel: 0.120}):
        saddle_extended = ctx.part_world_position(saddle)
        ctx.expect_within(
            inner_post,
            outer_tube,
            axes="xy",
            inner_elem="stanchion_shell",
            outer_elem="outer_shell",
            margin=0.0015,
            name="inner post remains coaxial at full extension",
        )
    ctx.check(
        "dropper travel raises saddle",
        saddle_rest is not None
        and saddle_extended is not None
        and saddle_extended[2] > saddle_rest[2] + 0.11,
        details=f"rest={saddle_rest}, extended={locals().get('saddle_extended')}",
    )

    lever_rest = ctx.part_world_aabb(lock_lever)
    with ctx.pose({lever_joint: 1.10}):
        lever_open = ctx.part_world_aabb(lock_lever)
    ctx.check(
        "locking lever swings outward",
        lever_rest is not None
        and lever_open is not None
        and lever_open[0][2] < lever_rest[0][2] - 0.02,
        details=f"rest={lever_rest}, open={locals().get('lever_open')}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
