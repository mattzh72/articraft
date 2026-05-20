from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Cylinder,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

ASSETS = AssetContext.from_script(__file__)

BODY_OUTER_RADIUS = 0.0305
BODY_RING_TRACK_RADIUS = 0.0310
BODY_INNER_RADIUS = 0.0222
BODY_LENGTH = 0.038
FRONT_COLLAR_OUTER_RADIUS = 0.028
FRONT_COLLAR_LENGTH = 0.014
FRONT_COLLAR_CENTER_Z = BODY_LENGTH + FRONT_COLLAR_LENGTH / 2.0

REAR_MOUNT_OUTER_RADIUS = BODY_INNER_RADIUS
REAR_MOUNT_INNER_RADIUS = 0.0135
REAR_MOUNT_LENGTH = 0.004
REAR_MOUNT_CENTER_Z = REAR_MOUNT_LENGTH / 2.0
REAR_GLASS_RADIUS = REAR_MOUNT_INNER_RADIUS
REAR_GLASS_LENGTH = 0.002
REAR_GLASS_CENTER_Z = REAR_MOUNT_LENGTH + REAR_GLASS_LENGTH / 2.0

FOCUS_RING_INNER_RADIUS = BODY_RING_TRACK_RADIUS
FOCUS_RING_OUTER_RADIUS = 0.0365
FOCUS_RING_RIB_OUTER_RADIUS = 0.0380
FOCUS_RING_LENGTH = 0.026
FOCUS_RING_CENTER_Z = 0.024
FOCUS_RING_GUIDE_LENGTH = 0.004
FOCUS_RING_REAR_GUIDE_CENTER_Z = (
    FOCUS_RING_CENTER_Z - (FOCUS_RING_LENGTH / 2.0) + (FOCUS_RING_GUIDE_LENGTH / 2.0)
)
FOCUS_RING_FRONT_GUIDE_CENTER_Z = (
    FOCUS_RING_CENTER_Z + (FOCUS_RING_LENGTH / 2.0) - (FOCUS_RING_GUIDE_LENGTH / 2.0)
)

EXTENSION_OUTER_RADIUS = 0.0218
EXTENSION_INNER_RADIUS = 0.018
EXTENSION_LENGTH = 0.026
EXTENSION_CENTER_Z = 0.001
EXTENSION_FRONT_BEZEL_OUTER_RADIUS = 0.024
EXTENSION_FRONT_BEZEL_LENGTH = 0.004
EXTENSION_FRONT_BEZEL_CENTER_Z = 0.015
EXTENSION_RETENTION_RING_LENGTH = 0.0015
EXTENSION_RETENTION_RING_CENTER_Z = 0.01175
EXTENSION_GLASS_RADIUS = 0.017
EXTENSION_GLASS_LENGTH = 0.002
EXTENSION_GLASS_CENTER_Z = 0.010
EXTENSION_SLIDE = 0.018


def _annular_shell_mesh(filename: str, outer_radius: float, inner_radius: float, length: float):
    half = length / 2.0
    geometry = LatheGeometry.from_shell_profiles(
        [(outer_radius, -half), (outer_radius, half)],
        [(inner_radius, -half), (inner_radius, half)],
        segments=64,
        start_cap="flat",
        end_cap="flat",
    )
    return mesh_from_geometry(geometry, ASSETS.mesh_path(filename))


def _lens_element_mesh(filename: str, radius: float, thickness: float):
    half = thickness / 2.0
    geometry = LatheGeometry(
        [
            (0.0, -0.18 * half),
            (0.50 * radius, -half),
            (radius, -half),
            (radius, half),
            (0.50 * radius, half),
            (0.0, 0.18 * half),
        ],
        segments=64,
    )
    return mesh_from_geometry(geometry, ASSETS.mesh_path(filename))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="macro_lens", assets=ASSETS)

    body_metal = model.material("body_metal", rgba=(0.17, 0.17, 0.18, 1.0))
    ring_rubber = model.material("ring_rubber", rgba=(0.07, 0.07, 0.07, 1.0))
    trim_metal = model.material("trim_metal", rgba=(0.24, 0.24, 0.25, 1.0))
    coated_glass = model.material("coated_glass", rgba=(0.54, 0.69, 0.80, 0.38))

    body = model.part("body")
    body.visual(
        _annular_shell_mesh("body_shell.obj", BODY_OUTER_RADIUS, BODY_INNER_RADIUS, BODY_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, BODY_LENGTH / 2.0)),
        material=body_metal,
        name="body_shell",
    )
    body.visual(
        _annular_shell_mesh(
            "rear_mount.obj",
            REAR_MOUNT_OUTER_RADIUS,
            REAR_MOUNT_INNER_RADIUS,
            REAR_MOUNT_LENGTH,
        ),
        origin=Origin(xyz=(0.0, 0.0, REAR_MOUNT_CENTER_Z)),
        material=trim_metal,
        name="rear_mount",
    )
    body.visual(
        _lens_element_mesh("rear_glass.obj", REAR_GLASS_RADIUS, REAR_GLASS_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, REAR_GLASS_CENTER_Z)),
        material=coated_glass,
        name="rear_glass",
    )
    body.visual(
        _annular_shell_mesh(
            "rear_focus_track.obj",
            BODY_RING_TRACK_RADIUS,
            BODY_OUTER_RADIUS,
            FOCUS_RING_GUIDE_LENGTH,
        ),
        origin=Origin(xyz=(0.0, 0.0, FOCUS_RING_REAR_GUIDE_CENTER_Z)),
        material=trim_metal,
        name="rear_focus_track",
    )
    body.visual(
        _annular_shell_mesh(
            "front_focus_track.obj",
            BODY_RING_TRACK_RADIUS,
            BODY_OUTER_RADIUS,
            FOCUS_RING_GUIDE_LENGTH,
        ),
        origin=Origin(xyz=(0.0, 0.0, FOCUS_RING_FRONT_GUIDE_CENTER_Z)),
        material=trim_metal,
        name="front_focus_track",
    )
    body.visual(
        _annular_shell_mesh(
            "front_collar.obj",
            FRONT_COLLAR_OUTER_RADIUS,
            EXTENSION_OUTER_RADIUS,
            FRONT_COLLAR_LENGTH,
        ),
        origin=Origin(xyz=(0.0, 0.0, FRONT_COLLAR_CENTER_Z)),
        material=trim_metal,
        name="front_collar",
    )
    body.inertial = Inertial.from_geometry(
        Cylinder(radius=BODY_OUTER_RADIUS, length=BODY_LENGTH + FRONT_COLLAR_LENGTH),
        mass=0.42,
        origin=Origin(xyz=(0.0, 0.0, (BODY_LENGTH + FRONT_COLLAR_LENGTH) / 2.0)),
    )

    focus_ring = model.part("focus_ring")
    focus_ring.visual(
        _annular_shell_mesh(
            "focus_ring_shell.obj",
            FOCUS_RING_OUTER_RADIUS,
            FOCUS_RING_INNER_RADIUS,
            FOCUS_RING_LENGTH,
        ),
        material=ring_rubber,
        name="focus_ring_shell",
    )
    for index, offset in enumerate((-0.010, -0.006, -0.002, 0.002, 0.006, 0.010), start=1):
        focus_ring.visual(
            _annular_shell_mesh(
                f"focus_ring_rib_{index}.obj",
                FOCUS_RING_RIB_OUTER_RADIUS,
                FOCUS_RING_OUTER_RADIUS,
                0.0022,
            ),
            origin=Origin(xyz=(0.0, 0.0, offset)),
            material=ring_rubber,
            name=f"rib_{index}",
        )
    focus_ring.inertial = Inertial.from_geometry(
        Cylinder(radius=FOCUS_RING_OUTER_RADIUS, length=FOCUS_RING_LENGTH),
        mass=0.08,
    )

    extension_barrel = model.part("extension_barrel")
    extension_barrel.visual(
        _annular_shell_mesh(
            "extension_shell.obj",
            EXTENSION_OUTER_RADIUS,
            EXTENSION_INNER_RADIUS,
            EXTENSION_LENGTH,
        ),
        origin=Origin(xyz=(0.0, 0.0, EXTENSION_CENTER_Z)),
        material=body_metal,
        name="extension_shell",
    )
    extension_barrel.visual(
        _annular_shell_mesh(
            "extension_front_bezel.obj",
            EXTENSION_FRONT_BEZEL_OUTER_RADIUS,
            EXTENSION_INNER_RADIUS,
            EXTENSION_FRONT_BEZEL_LENGTH,
        ),
        origin=Origin(xyz=(0.0, 0.0, EXTENSION_FRONT_BEZEL_CENTER_Z)),
        material=trim_metal,
        name="front_bezel",
    )
    extension_barrel.visual(
        _annular_shell_mesh(
            "extension_retention_ring.obj",
            EXTENSION_INNER_RADIUS,
            EXTENSION_GLASS_RADIUS,
            EXTENSION_RETENTION_RING_LENGTH,
        ),
        origin=Origin(xyz=(0.0, 0.0, EXTENSION_RETENTION_RING_CENTER_Z)),
        material=trim_metal,
        name="retention_ring",
    )
    extension_barrel.visual(
        _lens_element_mesh("front_glass.obj", EXTENSION_GLASS_RADIUS, EXTENSION_GLASS_LENGTH),
        origin=Origin(xyz=(0.0, 0.0, EXTENSION_GLASS_CENTER_Z)),
        material=coated_glass,
        name="front_glass",
    )
    extension_barrel.inertial = Inertial.from_geometry(
        Cylinder(radius=EXTENSION_FRONT_BEZEL_OUTER_RADIUS, length=0.029),
        mass=0.14,
        origin=Origin(xyz=(0.0, 0.0, 0.0025)),
    )

    model.articulation(
        "focus_ring_spin",
        ArticulationType.REVOLUTE,
        parent=body,
        child=focus_ring,
        origin=Origin(xyz=(0.0, 0.0, FOCUS_RING_CENTER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.8, velocity=6.0, lower=-1.4, upper=1.4),
    )
    model.articulation(
        "extension_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=extension_barrel,
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=0.05,
            lower=0.0,
            upper=EXTENSION_SLIDE,
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    body = object_model.get_part("body")
    focus_ring = object_model.get_part("focus_ring")
    extension_barrel = object_model.get_part("extension_barrel")
    focus_ring_spin = object_model.get_articulation("focus_ring_spin")
    extension_slide = object_model.get_articulation("extension_slide")

    rear_mount = body.get_visual("rear_mount")
    rear_glass = body.get_visual("rear_glass")
    rear_focus_track = body.get_visual("rear_focus_track")
    front_focus_track = body.get_visual("front_focus_track")
    front_collar = body.get_visual("front_collar")
    focus_ring_shell = focus_ring.get_visual("focus_ring_shell")
    extension_shell = extension_barrel.get_visual("extension_shell")
    front_bezel = extension_barrel.get_visual("front_bezel")
    retention_ring = extension_barrel.get_visual("retention_ring")
    front_glass = extension_barrel.get_visual("front_glass")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts(max_pose_samples=24)
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.allow_overlap(
        body,
        extension_barrel,
        elem_a=front_collar,
        elem_b=extension_shell,
        reason=(
            "The inner extension barrel telescopes inside the front guide collar as a nested sleeve; "
            "the modeled shell volumes intentionally occupy the same guide region."
        ),
    )
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=48,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    ctx.expect_origin_distance(
        focus_ring,
        body,
        axes="xy",
        max_dist=0.001,
        name="focus_ring_concentric",
    )
    ctx.expect_overlap(
        focus_ring,
        body,
        axes="xy",
        min_overlap=0.060,
        name="focus_ring_covers_barrel",
    )
    ctx.expect_contact(
        focus_ring,
        body,
        elem_a=focus_ring_shell,
        elem_b=rear_focus_track,
        name="focus_ring_rear_track_contact",
    )
    ctx.expect_contact(
        focus_ring,
        body,
        elem_a=focus_ring_shell,
        elem_b=front_focus_track,
        name="focus_ring_front_track_contact",
    )
    ctx.expect_gap(
        focus_ring,
        body,
        axis="z",
        positive_elem=focus_ring_shell,
        negative_elem=rear_mount,
        min_gap=0.006,
        max_gap=0.008,
        name="focus_ring_clears_rear_mount",
    )
    ctx.expect_gap(
        body,
        focus_ring,
        axis="z",
        positive_elem=front_collar,
        negative_elem=focus_ring_shell,
        min_gap=0.0005,
        max_gap=0.0015,
        name="front_collar_starts_after_focus_ring",
    )
    ctx.expect_contact(
        body,
        body,
        elem_a=rear_mount,
        elem_b=rear_glass,
        name="rear_glass_retained",
    )

    ctx.expect_origin_distance(
        extension_barrel,
        body,
        axes="xy",
        max_dist=0.001,
        name="extension_barrel_concentric",
    )
    ctx.expect_contact(
        extension_barrel,
        body,
        elem_a=extension_shell,
        elem_b=front_collar,
        name="extension_guided_by_front_collar",
    )
    ctx.expect_within(
        extension_barrel,
        body,
        axes="xy",
        name="extension_stays_within_body_footprint",
    )
    ctx.expect_contact(
        extension_barrel,
        extension_barrel,
        elem_a=front_glass,
        elem_b=retention_ring,
        name="front_glass_retained",
    )
    ctx.expect_gap(
        extension_barrel,
        body,
        axis="z",
        positive_elem=front_bezel,
        negative_elem=front_collar,
        min_gap=0.0005,
        max_gap=0.0015,
        name="front_bezel_rest_gap",
    )

    focus_limits = focus_ring_spin.motion_limits
    extension_limits = extension_slide.motion_limits
    ctx.check(
        "focus_ring_joint_axis",
        tuple(focus_ring_spin.axis) == (0.0, 0.0, 1.0),
        details=f"Expected focus ring axis (0, 0, 1), got {focus_ring_spin.axis}.",
    )
    ctx.check(
        "extension_joint_axis",
        tuple(extension_slide.axis) == (0.0, 0.0, 1.0),
        details=f"Expected extension axis (0, 0, 1), got {extension_slide.axis}.",
    )
    ctx.check(
        "focus_ring_joint_range_realistic",
        focus_limits is not None
        and focus_limits.lower is not None
        and focus_limits.upper is not None
        and 2.0 <= (focus_limits.upper - focus_limits.lower) <= 3.2,
        details="Focus ring should have a realistic bounded throw for a manual-focus lens.",
    )
    ctx.check(
        "extension_joint_range_realistic",
        extension_limits is not None
        and extension_limits.lower is not None
        and extension_limits.upper is not None
        and 0.010 <= (extension_limits.upper - extension_limits.lower) <= 0.030,
        details="Extension barrel should have a compact but visible macro-style travel range.",
    )

    body_aabb = ctx.part_world_aabb(body)
    focus_ring_aabb = ctx.part_world_aabb(focus_ring)
    extension_aabb = ctx.part_world_aabb(extension_barrel)
    if body_aabb is None or focus_ring_aabb is None or extension_aabb is None:
        ctx.fail("lens_aabb_available", "Expected world AABBs for body, focus ring, and extension barrel.")
    else:
        body_diameter = body_aabb[1][0] - body_aabb[0][0]
        focus_ring_diameter = focus_ring_aabb[1][0] - focus_ring_aabb[0][0]
        extension_diameter = extension_aabb[1][0] - extension_aabb[0][0]
        rest_length = (
            max(body_aabb[1][2], focus_ring_aabb[1][2], extension_aabb[1][2])
            - min(body_aabb[0][2], focus_ring_aabb[0][2], extension_aabb[0][2])
        )
        ctx.check(
            "body_diameter_realistic",
            0.058 <= body_diameter <= 0.065,
            details=f"Expected a compact lens body diameter around 58-65 mm, got {body_diameter:.4f} m.",
        )
        ctx.check(
            "focus_ring_wider_than_body",
            focus_ring_diameter >= body_diameter + 0.010,
            details=(
                "Expected the rubberized focus ring to stand proud of the main barrel by at least 10 mm "
                f"in diameter; got body {body_diameter:.4f} m, ring {focus_ring_diameter:.4f} m."
            ),
        )
        ctx.check(
            "extension_barrel_nested_size",
            extension_diameter <= body_diameter - 0.010,
            details=(
                "Expected the inner extension barrel to be visibly smaller than the body bore; "
                f"got body {body_diameter:.4f} m and extension {extension_diameter:.4f} m."
            ),
        )
        ctx.check(
            "compact_macro_rest_length",
            0.055 <= rest_length <= 0.080,
            details=f"Expected a compact macro-lens rest length around 55-80 mm, got {rest_length:.4f} m.",
        )

    extension_rest_position = ctx.part_world_position(extension_barrel)
    if extension_rest_position is None:
        ctx.fail("extension_rest_position_available", "Expected a world position for the extension barrel at rest.")

    if focus_limits is not None and focus_limits.lower is not None and focus_limits.upper is not None:
        with ctx.pose({focus_ring_spin: focus_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="focus_ring_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="focus_ring_lower_no_floating")
            ctx.expect_contact(
                focus_ring,
                body,
                elem_a=focus_ring_shell,
                elem_b=rear_focus_track,
                name="focus_ring_lower_rear_track_contact",
            )
            ctx.expect_contact(
                focus_ring,
                body,
                elem_a=focus_ring_shell,
                elem_b=front_focus_track,
                name="focus_ring_lower_front_track_contact",
            )
        with ctx.pose({focus_ring_spin: focus_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="focus_ring_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="focus_ring_upper_no_floating")
            ctx.expect_contact(
                focus_ring,
                body,
                elem_a=focus_ring_shell,
                elem_b=rear_focus_track,
                name="focus_ring_upper_rear_track_contact",
            )
            ctx.expect_contact(
                focus_ring,
                body,
                elem_a=focus_ring_shell,
                elem_b=front_focus_track,
                name="focus_ring_upper_front_track_contact",
            )

    if extension_limits is not None and extension_limits.lower is not None and extension_limits.upper is not None:
        with ctx.pose({extension_slide: extension_limits.lower}):
            ctx.fail_if_parts_overlap_in_current_pose(name="extension_lower_no_overlap")
            ctx.fail_if_isolated_parts(name="extension_lower_no_floating")
            ctx.expect_contact(
                extension_barrel,
                body,
                elem_a=extension_shell,
                elem_b=front_collar,
                name="extension_lower_front_collar_contact",
            )
            ctx.expect_gap(
                extension_barrel,
                body,
                axis="z",
                positive_elem=front_bezel,
                negative_elem=front_collar,
                min_gap=0.0005,
                max_gap=0.0015,
                name="extension_lower_front_bezel_gap",
            )
        with ctx.pose({extension_slide: extension_limits.upper}):
            ctx.fail_if_parts_overlap_in_current_pose(name="extension_upper_no_overlap")
            ctx.fail_if_isolated_parts(name="extension_upper_no_floating")
            ctx.expect_within(
                extension_barrel,
                body,
                axes="xy",
                name="extension_upper_within_body_footprint",
            )
            ctx.expect_contact(
                extension_barrel,
                body,
                elem_a=extension_shell,
                elem_b=front_collar,
                name="extension_upper_front_collar_contact",
            )
            ctx.expect_gap(
                extension_barrel,
                body,
                axis="z",
                positive_elem=front_bezel,
                negative_elem=front_collar,
                min_gap=0.018,
                max_gap=0.0205,
                name="extension_upper_front_bezel_gap",
            )
            ctx.expect_gap(
                extension_barrel,
                focus_ring,
                axis="z",
                positive_elem=front_bezel,
                negative_elem=focus_ring_shell,
                min_gap=0.015,
                name="extension_clears_focus_ring_when_extended",
            )
            extension_extended_position = ctx.part_world_position(extension_barrel)
            if extension_rest_position is not None and extension_extended_position is not None:
                travel = extension_extended_position[2] - extension_rest_position[2]
                lateral_shift = abs(extension_extended_position[0] - extension_rest_position[0]) + abs(
                    extension_extended_position[1] - extension_rest_position[1]
                )
                ctx.check(
                    "extension_slide_travel_distance",
                    abs(travel - EXTENSION_SLIDE) <= 0.0005 and lateral_shift <= 1e-6,
                    details=(
                        f"Expected {EXTENSION_SLIDE:.4f} m of Z travel with no lateral drift; "
                        f"got travel {travel:.4f} m and lateral shift {lateral_shift:.6f} m."
                    ),
                )

            if focus_limits is not None and focus_limits.upper is not None:
                with ctx.pose({focus_ring_spin: focus_limits.upper, extension_slide: extension_limits.upper}):
                    ctx.fail_if_parts_overlap_in_current_pose(name="combined_pose_no_overlap")
                    ctx.fail_if_isolated_parts(name="combined_pose_no_floating")
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
