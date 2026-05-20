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


def _shell_mesh(
    name: str,
    outer_profile: list[tuple[float, float]],
    inner_profile: list[tuple[float, float]],
    *,
    segments: int = 72,
):
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


def _annulus_mesh(
    name: str,
    *,
    outer_radius: float,
    inner_radius: float,
    thickness: float,
    segments: int = 72,
):
    half_t = thickness * 0.5
    return _shell_mesh(
        name,
        [(outer_radius, -half_t), (outer_radius, half_t)],
        [(inner_radius, -half_t), (inner_radius, half_t)],
        segments=segments,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="variable_nd_filter_adapter")

    anodized_black = model.material("anodized_black", rgba=(0.08, 0.08, 0.09, 1.0))
    matte_charcoal = model.material("matte_charcoal", rgba=(0.13, 0.13, 0.14, 1.0))
    smoked_glass = model.material("smoked_glass", rgba=(0.18, 0.20, 0.22, 0.38))
    white_mark = model.material("white_mark", rgba=(0.92, 0.92, 0.92, 1.0))

    fixed_body_shell = _shell_mesh(
        "fixed_body_shell",
        [
            (0.0378, -0.0028),
            (0.0390, -0.0016),
            (0.0390, 0.0028),
            (0.0380, 0.0052),
            (0.0372, 0.0066),
        ],
        [
            (0.0310, -0.0028),
            (0.0310, -0.0003),
            (0.0306, 0.0015),
            (0.0306, 0.0066),
        ],
    )
    rear_thread_collar_mesh = _shell_mesh(
        "rear_thread_collar",
        [
            (0.0365, -0.0075),
            (0.0365, -0.0040),
            (0.0369, -0.0022),
        ],
        [
            (0.0310, -0.0075),
            (0.0310, -0.0022),
        ],
    )
    fixed_retainer_mesh = _annulus_mesh(
        "fixed_retainer",
        outer_radius=0.0312,
        inner_radius=0.0298,
        thickness=0.0010,
    )
    rotating_ring_shell = _shell_mesh(
        "rotating_density_ring",
        [
            (0.0435, -0.0020),
            (0.0442, -0.0005),
            (0.0442, 0.0045),
            (0.0436, 0.0068),
            (0.0432, 0.0082),
        ],
        [
            (0.0397, -0.0020),
            (0.0397, 0.0068),
            (0.0392, 0.0082),
        ],
    )
    rotating_retainer_mesh = _annulus_mesh(
        "rotating_retainer",
        outer_radius=0.0398,
        inner_radius=0.0302,
        thickness=0.0010,
    )

    fixed_adapter = model.part("fixed_adapter")
    fixed_adapter.visual(
        fixed_body_shell,
        material=anodized_black,
        name="fixed_body_shell",
    )
    fixed_adapter.visual(
        rear_thread_collar_mesh,
        material=anodized_black,
        name="rear_thread_collar",
    )
    fixed_adapter.visual(
        fixed_retainer_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.0015)),
        material=anodized_black,
        name="fixed_retainer",
    )
    fixed_adapter.visual(
        Cylinder(radius=0.0300, length=0.0010),
        origin=Origin(xyz=(0.0, 0.0, 0.0015)),
        material=smoked_glass,
        name="rear_polarizer",
    )
    fixed_adapter.visual(
        Box((0.0030, 0.0012, 0.0018)),
        origin=Origin(xyz=(0.0, 0.0380, 0.0049)),
        material=white_mark,
        name="fixed_reference_mark",
    )
    fixed_adapter.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0395, length=0.0145),
        mass=0.065,
        origin=Origin(xyz=(0.0, 0.0, -0.0002)),
    )

    density_ring = model.part("density_ring")
    density_ring.visual(
        rotating_ring_shell,
        material=matte_charcoal,
        name="outer_ring_shell",
    )
    density_ring.visual(
        rotating_retainer_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.0071)),
        material=matte_charcoal,
        name="front_retainer",
    )
    density_ring.visual(
        Cylinder(radius=0.0304, length=0.0010),
        origin=Origin(xyz=(0.0, 0.0, 0.0066)),
        material=smoked_glass,
        name="front_polarizer",
    )
    density_ring.visual(
        Box((0.0040, 0.0009, 0.0024)),
        origin=Origin(xyz=(0.0, 0.0440, 0.0034)),
        material=white_mark,
        name="outer_index_mark",
    )
    density_ring.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0445, length=0.0105),
        mass=0.045,
        origin=Origin(xyz=(0.0, 0.0, 0.0031)),
    )

    model.articulation(
        "density_ring_rotation",
        ArticulationType.CONTINUOUS,
        parent=fixed_adapter,
        child=density_ring,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=0.4, velocity=8.0),
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

    fixed_adapter = object_model.get_part("fixed_adapter")
    density_ring = object_model.get_part("density_ring")
    ring_joint = object_model.get_articulation("density_ring_rotation")

    ctx.expect_origin_distance(
        fixed_adapter,
        density_ring,
        axes="xy",
        max_dist=1e-6,
        name="density ring stays coaxial with the adapter body",
    )
    ctx.expect_overlap(
        fixed_adapter,
        density_ring,
        axes="xy",
        min_overlap=0.074,
        name="rotating ring remains concentrically nested around the fixed housing",
    )

    collar_aabb = ctx.part_element_world_aabb(fixed_adapter, elem="rear_thread_collar")
    ring_aabb = ctx.part_element_world_aabb(density_ring, elem="outer_ring_shell")
    ctx.check(
        "rear thread collar remains aft of the rotating ring",
        collar_aabb is not None
        and ring_aabb is not None
        and collar_aabb[1][2] <= ring_aabb[0][2] + 0.0005,
        details=f"collar_aabb={collar_aabb}, ring_aabb={ring_aabb}",
    )

    def _aabb_center(aabb):
        if aabb is None:
            return None
        lo, hi = aabb
        return (
            0.5 * (lo[0] + hi[0]),
            0.5 * (lo[1] + hi[1]),
            0.5 * (lo[2] + hi[2]),
        )

    ring_mark_rest = _aabb_center(
        ctx.part_element_world_aabb(density_ring, elem="outer_index_mark")
    )
    fixed_mark_rest = _aabb_center(
        ctx.part_element_world_aabb(fixed_adapter, elem="fixed_reference_mark")
    )
    with ctx.pose({ring_joint: math.pi / 2.0}):
        ring_mark_rotated = _aabb_center(
            ctx.part_element_world_aabb(density_ring, elem="outer_index_mark")
        )
        fixed_mark_rotated = _aabb_center(
            ctx.part_element_world_aabb(fixed_adapter, elem="fixed_reference_mark")
        )

    ctx.check(
        "outer density ring rotates continuously around the filter axis",
        ring_mark_rest is not None
        and ring_mark_rotated is not None
        and abs(ring_mark_rest[0]) < 0.005
        and ring_mark_rest[1] > 0.035
        and ring_mark_rotated[0] < -0.035
        and abs(ring_mark_rotated[1]) < 0.006,
        details=f"rest={ring_mark_rest}, rotated={ring_mark_rotated}",
    )
    ctx.check(
        "fixed rear adapter mark does not rotate with the density ring",
        fixed_mark_rest is not None
        and fixed_mark_rotated is not None
        and abs(fixed_mark_rest[0] - fixed_mark_rotated[0]) < 1e-6
        and abs(fixed_mark_rest[1] - fixed_mark_rotated[1]) < 1e-6
        and abs(fixed_mark_rest[2] - fixed_mark_rotated[2]) < 1e-6,
        details=f"rest={fixed_mark_rest}, rotated={fixed_mark_rotated}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
