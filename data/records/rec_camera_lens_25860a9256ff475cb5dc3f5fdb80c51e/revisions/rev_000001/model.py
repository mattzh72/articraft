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
    DomeGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="ultra_wide_prime_lens")

    def _save_mesh(name, geometry):
        return mesh_from_geometry(geometry, name)

    def _merge(*geometries):
        merged = geometries[0].copy()
        for geometry in geometries[1:]:
            merged.merge(geometry)
        return merged

    def _ring_shell(inner_radius: float, outer_radius: float, length: float, *, edge: float):
        half = length * 0.5
        outer_profile = [
            (outer_radius - 0.0004, -half),
            (outer_radius, -half + edge),
            (outer_radius, half - edge),
            (outer_radius - 0.0004, half),
        ]
        inner_profile = [
            (inner_radius, -half),
            (inner_radius + 0.00035, -half + edge),
            (inner_radius + 0.00035, half - edge),
            (inner_radius, half),
        ]
        return LatheGeometry.from_shell_profiles(
            outer_profile,
            inner_profile,
            segments=72,
            start_cap="flat",
            end_cap="flat",
        )

    def _body_shell_mesh():
        outer_profile = [
            (0.0315, 0.0000),
            (0.0338, 0.0018),
            (0.0338, 0.0060),
            (0.0312, 0.0085),
            (0.0310, 0.0125),
            (0.0323, 0.0150),
            (0.0323, 0.0300),
            (0.0338, 0.0310),
            (0.0342, 0.0360),
            (0.0350, 0.0440),
            (0.0362, 0.0510),
            (0.0380, 0.0560),
            (0.0408, 0.0580),
            (0.0408, 0.0840),
            (0.0448, 0.0850),
            (0.0478, 0.0880),
            (0.0508, 0.0920),
            (0.0520, 0.0960),
            (0.0495, 0.0980),
        ]
        inner_profile = [
            (0.0220, 0.0000),
            (0.0230, 0.0060),
            (0.0218, 0.0140),
            (0.0218, 0.0320),
            (0.0225, 0.0500),
            (0.0245, 0.0700),
            (0.0270, 0.0880),
            (0.0288, 0.0960),
        ]
        return LatheGeometry.from_shell_profiles(
            outer_profile,
            inner_profile,
            segments=88,
            start_cap="flat",
            end_cap="flat",
        )

    def _focus_ring_mesh():
        shell = _ring_shell(0.0412, 0.0468, 0.0240, edge=0.0013)
        ribs = [
            TorusGeometry(
                radius=0.0447,
                tube=0.00115,
                radial_segments=14,
                tubular_segments=56,
            ).translate(0.0, 0.0, z_pos)
            for z_pos in (-0.0038, 0.0, 0.0038)
        ]
        return _merge(shell, *ribs)

    def _aperture_ring_mesh():
        shell = _ring_shell(0.0327, 0.0362, 0.0120, edge=0.0009)
        ribs = [
            TorusGeometry(
                radius=0.0349,
                tube=0.0007,
                radial_segments=14,
                tubular_segments=48,
            ).translate(0.0, 0.0, z_pos)
            for z_pos in (-0.0032, 0.0, 0.0032)
        ]
        return _merge(shell, *ribs)

    body_black = model.material("body_black", rgba=(0.10, 0.10, 0.11, 1.0))
    rubber_black = model.material("rubber_black", rgba=(0.06, 0.06, 0.07, 1.0))
    metal_black = model.material("metal_black", rgba=(0.17, 0.17, 0.19, 1.0))
    mount_metal = model.material("mount_metal", rgba=(0.56, 0.57, 0.60, 1.0))
    coated_glass = model.material("coated_glass", rgba=(0.10, 0.17, 0.22, 1.0))
    engraving_white = model.material("engraving_white", rgba=(0.88, 0.89, 0.90, 1.0))

    lens_body = model.part("lens_body")
    lens_body.visual(
        _save_mesh("lens_body_shell", _body_shell_mesh()),
        material=body_black,
        name="body_shell",
    )
    lens_body.visual(
        Cylinder(radius=0.0338, length=0.0020),
        origin=Origin(xyz=(0.0, 0.0, 0.0010)),
        material=mount_metal,
        name="mount_flange",
    )
    lens_body.visual(
        Cylinder(radius=0.0215, length=0.0100),
        origin=Origin(xyz=(0.0, 0.0, 0.0050)),
        material=metal_black,
        name="rear_baffle",
    )
    lens_body.visual(
        Cylinder(radius=0.0332, length=0.0012),
        origin=Origin(xyz=(0.0, 0.0, 0.0150)),
        material=metal_black,
        name="aperture_rear_stop",
    )
    lens_body.visual(
        Cylinder(radius=0.0339, length=0.0012),
        origin=Origin(xyz=(0.0, 0.0, 0.0310)),
        material=metal_black,
        name="aperture_front_stop",
    )
    lens_body.visual(
        Cylinder(radius=0.0414, length=0.0014),
        origin=Origin(xyz=(0.0, 0.0, 0.0580)),
        material=metal_black,
        name="focus_rear_stop",
    )
    lens_body.visual(
        Cylinder(radius=0.0452, length=0.0014),
        origin=Origin(xyz=(0.0, 0.0, 0.0850)),
        material=metal_black,
        name="focus_front_stop",
    )
    lens_body.visual(
        Cylinder(radius=0.0292, length=0.0200),
        origin=Origin(xyz=(0.0, 0.0, 0.0860)),
        material=coated_glass,
        name="front_glass_core",
    )
    lens_body.visual(
        _save_mesh("front_element_dome", DomeGeometry(radius=0.0220, radial_segments=40, height_segments=18)),
        origin=Origin(xyz=(0.0, 0.0, 0.0740)),
        material=coated_glass,
        name="front_element_dome",
    )
    lens_body.inertial = Inertial.from_geometry(
        Cylinder(radius=0.052, length=0.098),
        mass=0.78,
        origin=Origin(xyz=(0.0, 0.0, 0.049)),
    )

    focus_ring = model.part("focus_ring")
    focus_ring.visual(
        _save_mesh("focus_ring_shell", _focus_ring_mesh()),
        material=rubber_black,
        name="focus_shell",
    )
    focus_ring.visual(
        Box((0.0016, 0.0024, 0.0090)),
        origin=Origin(xyz=(0.0465, 0.0, 0.0)),
        material=engraving_white,
        name="focus_mark",
    )
    focus_ring.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0468, length=0.024),
        mass=0.08,
    )

    aperture_ring = model.part("aperture_ring")
    aperture_ring.visual(
        _save_mesh("aperture_ring_shell", _aperture_ring_mesh()),
        material=metal_black,
        name="aperture_shell",
    )
    aperture_ring.visual(
        Box((0.0012, 0.0018, 0.0060)),
        origin=Origin(xyz=(0.0357, 0.0, 0.0)),
        material=engraving_white,
        name="aperture_mark",
    )
    aperture_ring.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0362, length=0.012),
        mass=0.04,
    )

    model.articulation(
        "focus_rotation",
        ArticulationType.REVOLUTE,
        parent=lens_body,
        child=focus_ring,
        origin=Origin(xyz=(0.0, 0.0, 0.0715)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=6.0,
            lower=-1.35,
            upper=1.35,
        ),
    )
    model.articulation(
        "aperture_rotation",
        ArticulationType.REVOLUTE,
        parent=lens_body,
        child=aperture_ring,
        origin=Origin(xyz=(0.0, 0.0, 0.0230)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1.0,
            velocity=4.0,
            lower=-0.85,
            upper=0.85,
        ),
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

    lens_body = object_model.get_part("lens_body")
    focus_ring = object_model.get_part("focus_ring")
    aperture_ring = object_model.get_part("aperture_ring")
    focus_rotation = object_model.get_articulation("focus_rotation")
    aperture_rotation = object_model.get_articulation("aperture_rotation")

    def _aabb_center(aabb):
        return tuple((aabb[0][axis] + aabb[1][axis]) * 0.5 for axis in range(3))

    body_aabb = ctx.part_world_aabb(lens_body)
    body_dims = None
    if body_aabb is not None:
        body_dims = tuple(body_aabb[1][axis] - body_aabb[0][axis] for axis in range(3))

    ctx.check(
        "lens body proportions read as an ultra-wide prime",
        body_dims is not None
        and 0.092 <= body_dims[2] <= 0.105
        and 0.094 <= body_dims[0] <= 0.108
        and abs(body_dims[0] - body_dims[1]) <= 0.004,
        details=f"body_dims={body_dims}",
    )
    ctx.check(
        "focus ring joint uses the lens axis",
        tuple(round(value, 6) for value in focus_rotation.axis) == (0.0, 0.0, 1.0),
        details=f"axis={focus_rotation.axis}",
    )
    ctx.check(
        "aperture ring joint uses the lens axis",
        tuple(round(value, 6) for value in aperture_rotation.axis) == (0.0, 0.0, 1.0),
        details=f"axis={aperture_rotation.axis}",
    )
    ctx.allow_isolated_part(
        focus_ring,
        reason="The focus ring is intentionally modeled as a close-clearance rotating sleeve around the main barrel.",
    )
    ctx.allow_isolated_part(
        aperture_ring,
        reason="The aperture ring is intentionally modeled as a close-clearance rotating sleeve around the rear barrel.",
    )

    with ctx.pose({focus_rotation: 0.0, aperture_rotation: 0.0}):
        ctx.expect_overlap(
            focus_ring,
            lens_body,
            axes="xy",
            elem_a="focus_shell",
            elem_b="body_shell",
            min_overlap=0.085,
            name="focus ring stays coaxial with the main barrel",
        )
        ctx.expect_gap(
            lens_body,
            focus_ring,
            axis="z",
            positive_elem="focus_front_stop",
            negative_elem="focus_shell",
            min_gap=0.0004,
            max_gap=0.0015,
            name="focus ring clears the front retaining stop",
        )
        ctx.expect_gap(
            focus_ring,
            lens_body,
            axis="z",
            positive_elem="focus_shell",
            negative_elem="focus_rear_stop",
            min_gap=0.0004,
            max_gap=0.0015,
            name="focus ring clears the rear retaining stop",
        )
        ctx.expect_overlap(
            aperture_ring,
            lens_body,
            axes="xy",
            elem_a="aperture_shell",
            elem_b="body_shell",
            min_overlap=0.060,
            name="aperture ring stays coaxial with the rear barrel",
        )
        ctx.expect_gap(
            aperture_ring,
            lens_body,
            axis="z",
            positive_elem="aperture_shell",
            negative_elem="aperture_rear_stop",
            min_gap=0.0006,
            max_gap=0.0016,
            name="aperture ring clears the rear retaining stop",
        )
        ctx.expect_gap(
            lens_body,
            aperture_ring,
            axis="z",
            positive_elem="aperture_front_stop",
            negative_elem="aperture_shell",
            min_gap=0.0006,
            max_gap=0.0016,
            name="aperture ring clears the front retaining stop",
        )

    focus_mark_rest = ctx.part_element_world_aabb(focus_ring, elem="focus_mark")
    with ctx.pose({focus_rotation: 1.0}):
        focus_mark_rotated = ctx.part_element_world_aabb(focus_ring, elem="focus_mark")

    aperture_mark_rest = ctx.part_element_world_aabb(aperture_ring, elem="aperture_mark")
    with ctx.pose({aperture_rotation: 0.65}):
        aperture_mark_rotated = ctx.part_element_world_aabb(aperture_ring, elem="aperture_mark")

    focus_rest_center = _aabb_center(focus_mark_rest) if focus_mark_rest is not None else None
    focus_rot_center = _aabb_center(focus_mark_rotated) if focus_mark_rotated is not None else None
    aperture_rest_center = _aabb_center(aperture_mark_rest) if aperture_mark_rest is not None else None
    aperture_rot_center = _aabb_center(aperture_mark_rotated) if aperture_mark_rotated is not None else None

    ctx.check(
        "focus ring visibly rotates around the barrel axis",
        focus_rest_center is not None
        and focus_rot_center is not None
        and abs(focus_rest_center[2] - focus_rot_center[2]) <= 1e-4
        and abs(math.hypot(focus_rest_center[0], focus_rest_center[1]) - math.hypot(focus_rot_center[0], focus_rot_center[1])) <= 0.002
        and math.dist(focus_rest_center[:2], focus_rot_center[:2]) >= 0.020,
        details=f"rest={focus_rest_center}, rotated={focus_rot_center}",
    )
    ctx.check(
        "aperture ring visibly rotates around the barrel axis",
        aperture_rest_center is not None
        and aperture_rot_center is not None
        and abs(aperture_rest_center[2] - aperture_rot_center[2]) <= 1e-4
        and abs(math.hypot(aperture_rest_center[0], aperture_rest_center[1]) - math.hypot(aperture_rot_center[0], aperture_rot_center[1])) <= 0.002
        and math.dist(aperture_rest_center[:2], aperture_rot_center[:2]) >= 0.010,
        details=f"rest={aperture_rest_center}, rotated={aperture_rot_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
