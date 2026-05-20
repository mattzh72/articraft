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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    section_loft,
    superellipse_profile,
    tube_from_spline_points,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _yz_section(
    x_pos: float,
    *,
    width: float,
    height: float,
    z_center: float,
    exponent: float = 2.4,
    segments: int = 28,
) -> list[tuple[float, float, float]]:
    profile = superellipse_profile(width, height, exponent=exponent, segments=segments)
    return [(x_pos, y, z_center + z) for y, z in profile]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="yard_hydrant")

    hydrant_red = model.material("hydrant_red", rgba=(0.74, 0.15, 0.12, 1.0))
    galvanized = model.material("galvanized", rgba=(0.62, 0.66, 0.69, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.17, 0.18, 0.20, 1.0))
    rubber = model.material("rubber", rgba=(0.07, 0.07, 0.07, 1.0))
    concrete = model.material("concrete", rgba=(0.62, 0.61, 0.59, 1.0))

    standpipe = model.part("standpipe")
    standpipe.visual(
        Cylinder(radius=0.029, length=1.14),
        origin=Origin(xyz=(0.0, 0.0, 0.39)),
        material=galvanized,
        name="post_pipe",
    )
    standpipe.visual(
        Cylinder(radius=0.085, length=0.05),
        origin=Origin(xyz=(0.0, 0.0, 0.01)),
        material=concrete,
        name="ground_collar",
    )
    standpipe.visual(
        Cylinder(radius=0.043, length=0.09),
        origin=Origin(xyz=(0.0, 0.0, -0.155)),
        material=dark_steel,
        name="buried_anchor",
    )
    standpipe.visual(
        Cylinder(radius=0.037, length=0.07),
        origin=Origin(xyz=(0.0, 0.0, 0.985)),
        material=hydrant_red,
        name="head_neck",
    )

    head_casting_geom = section_loft(
        [
            _yz_section(-0.050, width=0.056, height=0.076, z_center=1.025),
            _yz_section(-0.012, width=0.072, height=0.104, z_center=1.041),
            _yz_section(0.040, width=0.070, height=0.094, z_center=1.039),
            _yz_section(0.086, width=0.050, height=0.060, z_center=1.025),
        ]
    )
    standpipe.visual(
        _mesh("hydrant_head_casting", head_casting_geom),
        material=hydrant_red,
        name="head_casting",
    )
    standpipe.visual(
        Box((0.040, 0.052, 0.014)),
        origin=Origin(xyz=(-0.006, 0.0, 1.091)),
        material=hydrant_red,
        name="top_cap",
    )
    standpipe.visual(
        Cylinder(radius=0.023, length=0.052),
        origin=Origin(xyz=(0.071, 0.0, 1.026), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hydrant_red,
        name="outlet_boss",
    )

    spout_geom = tube_from_spline_points(
        [
            (0.060, 0.0, 1.027),
            (0.132, 0.0, 1.031),
            (0.216, 0.0, 1.010),
        ],
        radius=0.014,
        samples_per_segment=18,
        radial_segments=20,
        cap_ends=False,
    )
    standpipe.visual(
        _mesh("hydrant_spout_tube", spout_geom),
        material=hydrant_red,
        name="spout_tube",
    )
    standpipe.visual(
        Cylinder(radius=0.017, length=0.010),
        origin=Origin(xyz=(0.218, 0.0, 1.010), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hydrant_red,
        name="spout_lip",
    )
    standpipe.visual(
        Box((0.018, 0.012, 0.038)),
        origin=Origin(xyz=(-0.006, -0.024, 1.109)),
        material=hydrant_red,
        name="left_pivot_lug",
    )
    standpipe.visual(
        Box((0.018, 0.012, 0.038)),
        origin=Origin(xyz=(-0.006, 0.024, 1.109)),
        material=hydrant_red,
        name="right_pivot_lug",
    )

    handle = model.part("handle")
    handle.visual(
        Cylinder(radius=0.011, length=0.034),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="handle_hub",
    )
    handle.visual(
        Box((0.130, 0.030, 0.014)),
        origin=Origin(xyz=(-0.072, 0.0, 0.0)),
        material=dark_steel,
        name="handle_web",
    )
    handle.visual(
        Box((0.235, 0.022, 0.012)),
        origin=Origin(xyz=(-0.215, 0.0, 0.0)),
        material=dark_steel,
        name="handle_arm",
    )
    handle.visual(
        Cylinder(radius=0.013, length=0.045),
        origin=Origin(xyz=(-0.332, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="handle_grip",
    )

    model.articulation(
        "handle_pivot",
        ArticulationType.REVOLUTE,
        parent=standpipe,
        child=handle,
        origin=Origin(xyz=(-0.006, 0.0, 1.109)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=30.0, velocity=1.8, lower=0.0, upper=1.05),
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

    standpipe = object_model.get_part("standpipe")
    handle = object_model.get_part("handle")
    pivot = object_model.get_articulation("handle_pivot")

    ctx.check("standpipe part present", standpipe is not None, details="standpipe part missing")
    ctx.check("handle part present", handle is not None, details="handle part missing")
    ctx.check(
        "handle pivots on horizontal top joint",
        pivot.parent == "standpipe"
        and pivot.child == "handle"
        and abs(pivot.axis[0]) < 1e-6
        and abs(pivot.axis[1] - 1.0) < 1e-6
        and abs(pivot.axis[2]) < 1e-6,
        details=f"parent={pivot.parent}, child={pivot.child}, axis={pivot.axis}",
    )

    with ctx.pose({pivot: 0.0}):
        ctx.expect_overlap(
            handle,
            standpipe,
            axes="xy",
            elem_a="handle_hub",
            elem_b="top_cap",
            min_overlap=0.018,
            name="handle hub sits over the standpipe top cap",
        )
        ctx.expect_gap(
            handle,
            standpipe,
            axis="z",
            positive_elem="handle_arm",
            negative_elem="top_cap",
            min_gap=0.004,
            max_gap=0.030,
            name="closed handle arm clears the top cap",
        )
        closed_grip_aabb = ctx.part_element_world_aabb(handle, elem="handle_grip")

    upper = pivot.motion_limits.upper if pivot.motion_limits is not None else None
    opened_grip_aabb = None
    if upper is not None:
        with ctx.pose({pivot: upper}):
            opened_grip_aabb = ctx.part_element_world_aabb(handle, elem="handle_grip")

    closed_grip_z = (
        0.5 * (closed_grip_aabb[0][2] + closed_grip_aabb[1][2]) if closed_grip_aabb is not None else None
    )
    opened_grip_z = (
        0.5 * (opened_grip_aabb[0][2] + opened_grip_aabb[1][2]) if opened_grip_aabb is not None else None
    )
    ctx.check(
        "handle raises when opened",
        closed_grip_z is not None and opened_grip_z is not None and opened_grip_z > closed_grip_z + 0.12,
        details=f"closed_grip_z={closed_grip_z}, opened_grip_z={opened_grip_z}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
