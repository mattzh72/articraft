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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def circle_profile(radius: float, segments: int = 48, *, clockwise: bool = False) -> list[tuple[float, float]]:
    points = [
        (
            radius * math.cos((2.0 * math.pi * i) / segments),
            radius * math.sin((2.0 * math.pi * i) / segments),
        )
        for i in range(segments)
    ]
    return list(reversed(points)) if clockwise else points


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="monitor_stand_lazy_susan")

    platform_size = (0.46, 0.32, 0.018)
    base_radius = 0.13
    base_thickness = 0.012
    ring_outer_radius = 0.105
    ring_inner_radius = 0.078
    ring_thickness = 0.009
    mount_pad_radius = 0.095
    mount_pad_thickness = 0.006

    graphite = model.material("graphite", rgba=(0.17, 0.18, 0.19, 1.0))
    charcoal = model.material("charcoal", rgba=(0.24, 0.24, 0.25, 1.0))
    warm_gray = model.material("warm_gray", rgba=(0.55, 0.56, 0.58, 1.0))

    base = model.part("base_disc")
    base.visual(
        Cylinder(radius=base_radius, length=base_thickness),
        origin=Origin(xyz=(0.0, 0.0, base_thickness / 2.0)),
        material=graphite,
        name="bottom_disc",
    )

    bearing_ring_mesh = mesh_from_geometry(
        ExtrudeWithHolesGeometry(
            circle_profile(ring_outer_radius, 56),
            [circle_profile(ring_inner_radius, 56, clockwise=True)],
            ring_thickness,
            center=True,
            cap=True,
            closed=True,
        ),
        "bearing_ring",
    )
    base.visual(
        bearing_ring_mesh,
        origin=Origin(xyz=(0.0, 0.0, base_thickness + ring_thickness / 2.0)),
        material=charcoal,
        name="bearing_ring",
    )

    platform = model.part("top_platform")
    platform.visual(
        Cylinder(radius=mount_pad_radius, length=mount_pad_thickness),
        origin=Origin(xyz=(0.0, 0.0, mount_pad_thickness / 2.0)),
        material=charcoal,
        name="mount_pad",
    )
    platform.visual(
        Box(platform_size),
        origin=Origin(
            xyz=(0.0, 0.0, mount_pad_thickness + platform_size[2] / 2.0),
        ),
        material=warm_gray,
        name="platform_top",
    )

    model.articulation(
        "base_to_platform",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=platform,
        origin=Origin(xyz=(0.0, 0.0, base_thickness + ring_thickness)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=3.0),
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
    base = object_model.get_part("base_disc")
    platform = object_model.get_part("top_platform")
    turntable = object_model.get_articulation("base_to_platform")
    bottom_disc = base.get_visual("bottom_disc")
    bearing_ring = base.get_visual("bearing_ring")
    mount_pad = platform.get_visual("mount_pad")
    platform_top = platform.get_visual("platform_top")

    ctx.check("base part exists", base is not None)
    ctx.check("platform part exists", platform is not None)
    ctx.check(
        "turntable is continuous about vertical axis",
        turntable.articulation_type == ArticulationType.CONTINUOUS
        and tuple(round(v, 6) for v in turntable.axis) == (0.0, 0.0, 1.0)
        and turntable.motion_limits is not None
        and turntable.motion_limits.lower is None
        and turntable.motion_limits.upper is None,
        details=(
            f"type={turntable.articulation_type}, axis={turntable.axis}, "
            f"limits={turntable.motion_limits}"
        ),
    )

    with ctx.pose({turntable: 0.0}):
        ctx.expect_contact(
            platform,
            base,
            elem_a=mount_pad,
            elem_b=bearing_ring,
            contact_tol=1e-6,
            name="mount pad seats on bearing ring",
        )
        ctx.expect_gap(
            platform,
            base,
            axis="z",
            positive_elem=platform_top,
            negative_elem=bottom_disc,
            min_gap=0.010,
            name="top platform clears the bottom disc",
        )
        ctx.expect_origin_distance(
            platform,
            base,
            axes="xy",
            max_dist=1e-6,
            name="platform stays centered on the round base",
        )

        base_aabb_rest = ctx.part_element_world_aabb(base, elem="bottom_disc")
        top_aabb_rest = ctx.part_element_world_aabb(platform, elem="platform_top")
        rest_dx = top_aabb_rest[1][0] - top_aabb_rest[0][0] if top_aabb_rest else None
        rest_dy = top_aabb_rest[1][1] - top_aabb_rest[0][1] if top_aabb_rest else None
        base_dx = base_aabb_rest[1][0] - base_aabb_rest[0][0] if base_aabb_rest else None
        base_dy = base_aabb_rest[1][1] - base_aabb_rest[0][1] if base_aabb_rest else None
        ctx.check(
            "top platform overhangs the circular base footprint",
            rest_dx is not None
            and rest_dy is not None
            and base_dx is not None
            and base_dy is not None
            and rest_dx > base_dx + 0.15
            and rest_dy > base_dy + 0.05,
            details=(
                f"platform_xy=({rest_dx}, {rest_dy}), "
                f"base_xy=({base_dx}, {base_dy})"
            ),
        )

    with ctx.pose({turntable: math.pi / 2.0}):
        ctx.expect_contact(
            platform,
            base,
            elem_a=mount_pad,
            elem_b=bearing_ring,
            contact_tol=1e-6,
            name="mount pad stays seated after rotation",
        )
        ctx.expect_origin_distance(
            platform,
            base,
            axes="xy",
            max_dist=1e-6,
            name="rotation keeps the platform centered",
        )

        top_aabb_turned = ctx.part_element_world_aabb(platform, elem="platform_top")
        turned_dx = top_aabb_turned[1][0] - top_aabb_turned[0][0] if top_aabb_turned else None
        turned_dy = top_aabb_turned[1][1] - top_aabb_turned[0][1] if top_aabb_turned else None
        ctx.check(
            "platform footprint swaps x and y span at quarter turn",
            turned_dx is not None
            and turned_dy is not None
            and abs(turned_dx - 0.32) < 0.02
            and abs(turned_dy - 0.46) < 0.02,
            details=f"turned_xy=({turned_dx}, {turned_dy})",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
