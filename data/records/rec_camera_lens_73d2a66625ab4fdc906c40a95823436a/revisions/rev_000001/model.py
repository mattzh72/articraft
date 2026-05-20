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
)


LENS_LENGTH = 0.180
FOCUS_RING_CENTER_Z = 0.052
IRIS_RING_CENTER_Z = -0.013
PL_MOUNT_CENTER_Z = -0.0972
FRONT_GLASS_CENTER_Z = 0.069


def _shell_mesh(name: str, outer_profile, inner_profile, *, segments: int = 72):
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer_profile,
            inner_profile,
            segments=segments,
            start_cap="flat",
            end_cap="flat",
        ),
        name,
    )


def _ring_shell(
    name: str,
    *,
    outer_radius: float,
    inner_radius: float,
    length: float,
    shoulder: float = 0.0015,
    segments: int = 64,
):
    half = length * 0.5
    shoulder = min(shoulder, half * 0.45)
    outer_profile = [
        (outer_radius - 0.0015, -half),
        (outer_radius, -half + shoulder),
        (outer_radius, half - shoulder),
        (outer_radius - 0.0015, half),
    ]
    inner_profile = [
        (inner_radius, -half),
        (inner_radius, -half + shoulder),
        (inner_radius, half - shoulder),
        (inner_radius, half),
    ]
    return _shell_mesh(name, outer_profile, inner_profile, segments=segments)


def _straight_shell(
    name: str,
    *,
    outer_radius: float,
    inner_radius: float,
    length: float,
    segments: int = 72,
):
    half = length * 0.5
    outer_profile = [(outer_radius, -half), (outer_radius, half)]
    inner_profile = [(inner_radius, -half), (inner_radius, half)]
    return _shell_mesh(name, outer_profile, inner_profile, segments=segments)


def _focus_ring_mesh():
    half = 0.024
    outer_profile = [
        (0.0555, -half),
        (0.0575, -0.020),
        (0.0588, -0.018),
        (0.0590, -0.010),
        (0.0590, 0.010),
        (0.0588, 0.018),
        (0.0575, 0.020),
        (0.0555, half),
    ]
    inner_profile = [(0.0510, z) for _, z in outer_profile]
    return _shell_mesh("focus_ring_shell", outer_profile, inner_profile, segments=84)


def _iris_ring_mesh():
    half = 0.015
    outer_profile = [
        (0.0515, -half),
        (0.0535, -0.011),
        (0.0548, -0.010),
        (0.0550, -0.005),
        (0.0550, 0.005),
        (0.0548, 0.010),
        (0.0535, 0.011),
        (0.0515, half),
    ]
    inner_profile = [(0.0490, z) for _, z in outer_profile]
    return _shell_mesh("iris_ring_shell", outer_profile, inner_profile, segments=80)


def _barrel_core_mesh():
    outer_profile = [
        (0.0330, -0.092),
        (0.0330, -0.078),
        (0.0410, -0.074),
        (0.0410, -0.031),
        (0.0490, -0.028),
        (0.0490, -0.025),
        (0.0430, -0.022),
        (0.0430, 0.000),
        (0.0490, 0.002),
        (0.0490, 0.005),
        (0.0430, 0.008),
        (0.0430, 0.025),
        (0.0510, 0.028),
        (0.0510, 0.031),
        (0.0460, 0.034),
        (0.0460, 0.070),
        (0.0510, 0.076),
        (0.0510, 0.079),
        (0.0540, 0.080),
        (0.0550, 0.088),
    ]
    inner_profile = [
        (0.0240, -0.092),
        (0.0240, -0.078),
        (0.0265, -0.074),
        (0.0280, -0.028),
        (0.0320, -0.028),
        (0.0320, 0.002),
        (0.0330, 0.006),
        (0.0340, 0.028),
        (0.0360, 0.028),
        (0.0360, 0.076),
        (0.0370, 0.080),
        (0.0380, 0.088),
    ]
    return _shell_mesh("barrel_core_shell", outer_profile, inner_profile, segments=88)


def _pl_mount_mesh():
    outer_profile = [
        (0.0395, -0.006),
        (0.0415, -0.003),
        (0.0445, 0.001),
        (0.0445, 0.006),
    ]
    inner_profile = [(0.0338, z) for _, z in outer_profile]
    return _shell_mesh("pl_mount_shell", outer_profile, inner_profile, segments=72)


def _front_retaining_ring_mesh():
    return _ring_shell(
        "front_retaining_ring",
        outer_radius=0.0360,
        inner_radius=0.0330,
        length=0.0040,
        shoulder=0.0008,
        segments=64,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cinema_prime_lens")

    barrel_alloy = model.material("barrel_alloy", rgba=(0.14, 0.15, 0.16, 1.0))
    grip_black = model.material("grip_black", rgba=(0.08, 0.09, 0.10, 1.0))
    mount_steel = model.material("mount_steel", rgba=(0.70, 0.72, 0.75, 1.0))
    witness_white = model.material("witness_white", rgba=(0.94, 0.95, 0.93, 1.0))
    accent_cyan = model.material("accent_cyan", rgba=(0.22, 0.78, 0.86, 1.0))
    coated_glass = model.material("coated_glass", rgba=(0.12, 0.24, 0.34, 0.68))

    barrel_core = model.part("barrel_core")
    barrel_core.visual(
        _barrel_core_mesh(),
        material=barrel_alloy,
        name="barrel_shell",
    )
    barrel_core.visual(
        Box((0.0020, 0.0022, 0.012)),
        origin=Origin(xyz=(0.0, 0.0441, 0.024)),
        material=witness_white,
        name="focus_index_mark",
    )
    barrel_core.visual(
        Box((0.0020, 0.0020, 0.010)),
        origin=Origin(xyz=(0.0, 0.0440, -0.002)),
        material=accent_cyan,
        name="iris_index_mark",
    )
    barrel_core.visual(
        Box((0.020, 0.0016, 0.010)),
        origin=Origin(xyz=(0.0, -0.0438, 0.014)),
        material=witness_white,
        name="nameplate_strip",
    )
    barrel_core.inertial = Inertial.from_geometry(
        Cylinder(radius=0.056, length=LENS_LENGTH),
        mass=2.2,
        origin=Origin(xyz=(0.0, 0.0, -0.002)),
    )

    focus_ring = model.part("focus_ring")
    focus_ring.visual(
        _focus_ring_mesh(),
        material=grip_black,
        name="focus_ring_shell",
    )
    focus_mark_zs = (-0.015, -0.009, -0.003, 0.003, 0.009, 0.015)
    focus_mark_lengths = (0.008, 0.010, 0.007, 0.009, 0.011, 0.007)
    for index, (z_pos, mark_len) in enumerate(zip(focus_mark_zs, focus_mark_lengths)):
        focus_ring.visual(
            Box((mark_len, 0.0016, 0.0032)),
            origin=Origin(xyz=(0.0, 0.0584, z_pos)),
            material=witness_white,
            name=f"distance_mark_{index:02d}",
        )
    focus_ring.visual(
        Box((0.011, 0.0018, 0.0042)),
        origin=Origin(xyz=(0.0, 0.0584, 0.012)),
        material=accent_cyan,
        name="distance_mark_accent",
    )
    focus_ring.inertial = Inertial.from_geometry(
        Cylinder(radius=0.059, length=0.048),
        mass=0.45,
    )

    iris_ring = model.part("iris_ring")
    iris_ring.visual(
        _iris_ring_mesh(),
        material=grip_black,
        name="iris_ring_shell",
    )
    iris_mark_zs = (-0.009, -0.003, 0.003, 0.009)
    iris_mark_lengths = (0.007, 0.009, 0.007, 0.009)
    for index, (z_pos, mark_len) in enumerate(zip(iris_mark_zs, iris_mark_lengths)):
        iris_ring.visual(
            Box((mark_len, 0.0015, 0.0028)),
            origin=Origin(xyz=(0.0, 0.05445, z_pos)),
            material=witness_white,
            name=f"iris_mark_{index:02d}",
        )
    iris_ring.inertial = Inertial.from_geometry(
        Cylinder(radius=0.055, length=0.030),
        mass=0.28,
    )

    pl_mount = model.part("pl_mount")
    pl_mount.visual(
        _pl_mount_mesh(),
        material=mount_steel,
        name="mount_plate",
    )
    pl_mount.visual(
        _ring_shell(
            "pl_mount_register_sleeve",
            outer_radius=0.0362,
            inner_radius=0.0330,
            length=0.0040,
            shoulder=0.0006,
            segments=72,
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.0042)),
        material=mount_steel,
        name="register_sleeve",
    )
    lug_radius = 0.0415
    for index, angle in enumerate((0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)):
        pl_mount.visual(
            Box((0.0075, 0.0110, 0.0040)),
            origin=Origin(
                xyz=(lug_radius * math.cos(angle), lug_radius * math.sin(angle), 0.0035),
                rpy=(0.0, 0.0, angle),
            ),
            material=mount_steel,
            name=f"bayonet_lug_{index:02d}",
        )
    pl_mount.visual(
        Box((0.010, 0.004, 0.003)),
        origin=Origin(xyz=(0.0, 0.0430, -0.0035)),
        material=witness_white,
        name="mount_alignment_dot",
    )
    pl_mount.inertial = Inertial.from_geometry(
        Cylinder(radius=0.045, length=0.012),
        mass=0.25,
    )

    front_glass = model.part("front_glass")
    front_glass.visual(
        _front_retaining_ring_mesh(),
        material=barrel_alloy,
        name="retaining_ring",
    )
    front_glass.visual(
        Cylinder(radius=0.0332, length=0.0040),
        material=coated_glass,
        name="front_element",
    )
    front_glass.inertial = Inertial.from_geometry(
        Cylinder(radius=0.036, length=0.004),
        mass=0.08,
    )

    model.articulation(
        "focus_rotation",
        ArticulationType.REVOLUTE,
        parent=barrel_core,
        child=focus_ring,
        origin=Origin(xyz=(0.0, 0.0, FOCUS_RING_CENTER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=3.0,
            velocity=3.5,
            lower=-2.2,
            upper=2.2,
        ),
    )
    model.articulation(
        "iris_rotation",
        ArticulationType.REVOLUTE,
        parent=barrel_core,
        child=iris_ring,
        origin=Origin(xyz=(0.0, 0.0, IRIS_RING_CENTER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1.8,
            velocity=2.5,
            lower=-0.7,
            upper=0.7,
        ),
    )
    model.articulation(
        "rear_mount_attachment",
        ArticulationType.FIXED,
        parent=barrel_core,
        child=pl_mount,
        origin=Origin(xyz=(0.0, 0.0, PL_MOUNT_CENTER_Z)),
    )
    model.articulation(
        "front_element_mount",
        ArticulationType.FIXED,
        parent=barrel_core,
        child=front_glass,
        origin=Origin(xyz=(0.0, 0.0, FRONT_GLASS_CENTER_Z)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    barrel_core = object_model.get_part("barrel_core")
    focus_ring = object_model.get_part("focus_ring")
    iris_ring = object_model.get_part("iris_ring")
    pl_mount = object_model.get_part("pl_mount")
    front_glass = object_model.get_part("front_glass")

    focus_rotation = object_model.get_articulation("focus_rotation")
    iris_rotation = object_model.get_articulation("iris_rotation")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.allow_overlap(
        barrel_core,
        focus_ring,
        elem_a="barrel_shell",
        elem_b="focus_ring_shell",
        reason="The focus ring is a concentric sleeve riding on the barrel bearing land; the visual shell model intentionally shares that bearing envelope.",
    )
    ctx.allow_overlap(
        barrel_core,
        iris_ring,
        elem_a="barrel_shell",
        elem_b="iris_ring_shell",
        reason="The iris ring is a concentric sleeve around the mid-barrel, modeled with shared bearing-envelope contact for a realistic cine-lens fit.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "focus joint axis is optical axis",
        tuple(focus_rotation.axis) == (0.0, 0.0, 1.0),
        details=f"axis={focus_rotation.axis}",
    )
    ctx.check(
        "iris joint axis is optical axis",
        tuple(iris_rotation.axis) == (0.0, 0.0, 1.0),
        details=f"axis={iris_rotation.axis}",
    )
    ctx.check(
        "focus ring has cinema-scale travel",
        focus_rotation.motion_limits is not None
        and focus_rotation.motion_limits.lower is not None
        and focus_rotation.motion_limits.upper is not None
        and focus_rotation.motion_limits.upper - focus_rotation.motion_limits.lower >= 4.0,
        details=f"limits={focus_rotation.motion_limits}",
    )
    ctx.check(
        "iris ring has limited throw",
        iris_rotation.motion_limits is not None
        and iris_rotation.motion_limits.lower is not None
        and iris_rotation.motion_limits.upper is not None
        and 0.9 <= iris_rotation.motion_limits.upper - iris_rotation.motion_limits.lower <= 1.8,
        details=f"limits={iris_rotation.motion_limits}",
    )

    ctx.expect_contact(focus_ring, barrel_core, name="focus ring rides on barrel")
    ctx.expect_contact(iris_ring, barrel_core, name="iris ring rides on barrel")
    ctx.expect_contact(pl_mount, barrel_core, name="pl mount seats on rear barrel")
    ctx.expect_contact(front_glass, barrel_core, name="front element retained in front barrel")

    ctx.expect_origin_distance(
        focus_ring,
        barrel_core,
        axes="xy",
        min_dist=0.0,
        max_dist=1e-6,
        name="focus ring remains concentric",
    )
    ctx.expect_origin_distance(
        iris_ring,
        barrel_core,
        axes="xy",
        min_dist=0.0,
        max_dist=1e-6,
        name="iris ring remains concentric",
    )
    ctx.expect_overlap(
        focus_ring,
        barrel_core,
        axes="xy",
        min_overlap=0.090,
        name="focus ring covers front barrel support",
    )
    ctx.expect_overlap(
        iris_ring,
        barrel_core,
        axes="xy",
        min_overlap=0.086,
        name="iris ring covers mid barrel support",
    )

    with ctx.pose({focus_rotation: 1.6, iris_rotation: -0.4}):
        ctx.expect_contact(focus_ring, barrel_core, name="focus ring stays on barrel when rotated")
        ctx.expect_contact(iris_ring, barrel_core, name="iris ring stays on barrel when rotated")
        ctx.expect_origin_distance(
            focus_ring,
            barrel_core,
            axes="xy",
            min_dist=0.0,
            max_dist=1e-6,
            name="focus ring stays concentric when rotated",
        )
        ctx.expect_origin_distance(
            iris_ring,
            barrel_core,
            axes="xy",
            min_dist=0.0,
            max_dist=1e-6,
            name="iris ring stays concentric when rotated",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
