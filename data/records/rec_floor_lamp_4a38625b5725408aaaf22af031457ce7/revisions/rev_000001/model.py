from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _hollow_tube_mesh(
    name: str,
    *,
    outer_radius: float,
    inner_radius: float,
    height: float,
    segments: int = 64,
):
    """Thin cylindrical sleeve with real open bore and annular end faces."""
    shell = LatheGeometry.from_shell_profiles(
        [(outer_radius, 0.0), (outer_radius, height)],
        [(inner_radius, 0.0), (inner_radius, height)],
        segments=segments,
        start_cap="flat",
        end_cap="flat",
    )
    return mesh_from_geometry(shell, name)


def _cone_shade_mesh(name: str, *, tilt: float):
    """Open conical frustum lampshade, built around local +X after rotation."""
    length = 0.36
    shell = LatheGeometry.from_shell_profiles(
        [(0.058, 0.0), (0.190, length)],
        [(0.047, 0.006), (0.176, length - 0.010)],
        segments=96,
        start_cap="round",
        end_cap="round",
        lip_samples=8,
    )
    # Lathe is along +Z; rotate its axis to the shade's local +X and pitch it
    # downward so the rest pose already reads as a floor-lamp task shade.
    shell.rotate_y(math.pi / 2.0 + tilt)
    shell.translate(0.080 * math.cos(tilt), 0.0, -0.080 * math.sin(tilt))
    return mesh_from_geometry(shell, name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="telescoping_floor_lamp")

    dark_metal = model.material("satin_black_metal", rgba=(0.02, 0.022, 0.024, 1.0))
    chrome = model.material("brushed_chrome", rgba=(0.72, 0.72, 0.68, 1.0))
    shade_finish = model.material("warm_ivory_shade", rgba=(0.92, 0.84, 0.67, 1.0))
    warm_glass = model.material("warm_bulb_glass", rgba=(1.0, 0.86, 0.42, 0.72))

    # Root assembly: weighted round base, true hollow outer sleeve, and a
    # thickened clamp collar at the sleeve mouth.
    base_tube = model.part("base_tube")
    base_tube.visual(
        Cylinder(radius=0.230, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material=dark_metal,
        name="round_base",
    )
    base_tube.visual(
        Cylinder(radius=0.198, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=dark_metal,
        name="felt_underpad",
    )
    base_tube.visual(
        _hollow_tube_mesh(
            "outer_base_tube",
            outer_radius=0.037,
            inner_radius=0.027,
            height=0.850,
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
        material=dark_metal,
        name="outer_base_tube",
    )
    base_tube.visual(
        _hollow_tube_mesh(
            "top_clamp_collar",
            outer_radius=0.052,
            inner_radius=0.027,
            height=0.075,
        ),
        origin=Origin(xyz=(0.0, 0.0, 0.825)),
        material=chrome,
        name="top_clamp_collar",
    )

    # The child frame sits at the top mouth of the sleeve.  The visible post is
    # deliberately longer below that frame so it remains retained in the sleeve
    # at full extension.
    inner_post = model.part("inner_post")
    inner_post.visual(
        Cylinder(radius=0.020, length=1.300),
        origin=Origin(xyz=(0.0, 0.0, 0.100)),
        material=chrome,
        name="sliding_post",
    )
    inner_post.visual(
        Cylinder(radius=0.027, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, -0.040)),
        material=dark_metal,
        name="upper_guide_bushing",
    )
    inner_post.visual(
        Cylinder(radius=0.027, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, -0.500)),
        material=dark_metal,
        name="lower_guide_bushing",
    )
    inner_post.visual(
        Cylinder(radius=0.026, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.750)),
        material=chrome,
        name="post_cap",
    )
    inner_post.visual(
        Cylinder(radius=0.016, length=0.150),
        origin=Origin(xyz=(0.055, 0.0, 0.750), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=chrome,
        name="shade_arm",
    )
    inner_post.visual(
        Box((0.035, 0.112, 0.028)),
        origin=Origin(xyz=(0.116, 0.0, 0.750)),
        material=chrome,
        name="yoke_bridge",
    )
    inner_post.visual(
        Box((0.075, 0.012, 0.090)),
        origin=Origin(xyz=(0.150, 0.039, 0.750)),
        material=chrome,
        name="yoke_cheek_0",
    )
    inner_post.visual(
        Box((0.075, 0.012, 0.090)),
        origin=Origin(xyz=(0.150, -0.039, 0.750)),
        material=chrome,
        name="yoke_cheek_1",
    )

    model.articulation(
        "tube_to_post",
        ArticulationType.PRISMATIC,
        parent=base_tube,
        child=inner_post,
        origin=Origin(xyz=(0.0, 0.0, 0.895)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=95.0, velocity=0.18, lower=0.0, upper=0.350),
    )

    shade_tilt = math.radians(20.0)
    shade_axis_rpy = (0.0, math.pi / 2.0 + shade_tilt, 0.0)
    shade = model.part("shade")
    shade.visual(
        Cylinder(radius=0.024, length=0.066),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=chrome,
        name="hinge_knuckle",
    )
    shade.visual(
        Cylinder(radius=0.028, length=0.105),
        origin=Origin(
            xyz=(0.0525 * math.cos(shade_tilt), 0.0, -0.0525 * math.sin(shade_tilt)),
            rpy=shade_axis_rpy,
        ),
        material=chrome,
        name="shade_neck",
    )
    shade.visual(
        Cylinder(radius=0.052, length=0.045),
        origin=Origin(
            xyz=(0.090 * math.cos(shade_tilt), 0.0, -0.090 * math.sin(shade_tilt)),
            rpy=shade_axis_rpy,
        ),
        material=chrome,
        name="shade_collar",
    )
    shade.visual(
        _cone_shade_mesh("cone_shade", tilt=shade_tilt),
        material=shade_finish,
        name="cone_shade",
    )
    shade.visual(
        Cylinder(radius=0.024, length=0.085),
        origin=Origin(
            xyz=(0.142 * math.cos(shade_tilt), 0.0, -0.142 * math.sin(shade_tilt)),
            rpy=shade_axis_rpy,
        ),
        material=chrome,
        name="socket",
    )
    shade.visual(
        Sphere(radius=0.044),
        origin=Origin(xyz=(0.225 * math.cos(shade_tilt), 0.0, -0.225 * math.sin(shade_tilt))),
        material=warm_glass,
        name="bulb",
    )

    model.articulation(
        "post_to_shade",
        ArticulationType.REVOLUTE,
        parent=inner_post,
        child=shade,
        origin=Origin(xyz=(0.160, 0.0, 0.750)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=7.0, velocity=1.4, lower=-0.55, upper=0.55),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base_tube = object_model.get_part("base_tube")
    inner_post = object_model.get_part("inner_post")
    shade = object_model.get_part("shade")
    slide = object_model.get_articulation("tube_to_post")
    tilt = object_model.get_articulation("post_to_shade")

    # The dark guide bushings are intentionally captured inside the visible
    # hollow sleeve.  The mesh sleeve is used for visual realism, but the exact
    # overlap checker still treats the thin shell as a solid proxy at the guide
    # locations, so these local seated bearing interfaces are explicitly scoped.
    ctx.allow_overlap(
        base_tube,
        inner_post,
        elem_a="outer_base_tube",
        elem_b="lower_guide_bushing",
        reason="Lower guide bushing is intentionally captured inside the hollow telescoping sleeve.",
    )
    ctx.allow_overlap(
        base_tube,
        inner_post,
        elem_a="outer_base_tube",
        elem_b="upper_guide_bushing",
        reason="Upper guide bushing is intentionally captured inside the hollow telescoping sleeve.",
    )
    ctx.allow_overlap(
        base_tube,
        inner_post,
        elem_a="top_clamp_collar",
        elem_b="upper_guide_bushing",
        reason="Upper guide bushing is seated in the clamp collar at the sleeve mouth.",
    )

    ctx.expect_within(
        inner_post,
        base_tube,
        axes="xy",
        inner_elem="sliding_post",
        outer_elem="outer_base_tube",
        margin=0.004,
        name="sliding post is centered within the outer base tube",
    )
    ctx.expect_overlap(
        inner_post,
        base_tube,
        axes="z",
        elem_a="sliding_post",
        elem_b="outer_base_tube",
        min_overlap=0.40,
        name="collapsed post remains deeply inserted in the sleeve",
    )
    for guide_name in ("upper_guide_bushing", "lower_guide_bushing"):
        ctx.expect_within(
            inner_post,
            base_tube,
            axes="xy",
            inner_elem=guide_name,
            outer_elem="outer_base_tube",
            margin=0.001,
            name=f"{guide_name} is radially captured by the sleeve",
        )
        ctx.expect_overlap(
            inner_post,
            base_tube,
            axes="z",
            elem_a=guide_name,
            elem_b="outer_base_tube",
            min_overlap=0.040,
            name=f"{guide_name} remains engaged with the sleeve",
        )
    ctx.expect_within(
        inner_post,
        base_tube,
        axes="xy",
        inner_elem="upper_guide_bushing",
        outer_elem="top_clamp_collar",
        margin=0.001,
        name="upper guide bushing is radially captured by the collar",
    )
    ctx.expect_overlap(
        inner_post,
        base_tube,
        axes="z",
        elem_a="upper_guide_bushing",
        elem_b="top_clamp_collar",
        min_overlap=0.040,
        name="upper guide bushing is retained in the collar",
    )

    rest_position = ctx.part_world_position(inner_post)
    with ctx.pose({slide: 0.350}):
        ctx.expect_within(
            inner_post,
            base_tube,
            axes="xy",
            inner_elem="sliding_post",
            outer_elem="outer_base_tube",
            margin=0.004,
            name="extended post remains centered in the sleeve",
        )
        ctx.expect_overlap(
            inner_post,
            base_tube,
            axes="z",
            elem_a="sliding_post",
            elem_b="outer_base_tube",
            min_overlap=0.18,
            name="extended post keeps retained insertion in the sleeve",
        )
        extended_position = ctx.part_world_position(inner_post)

    ctx.check(
        "prismatic joint raises the lamp post",
        rest_position is not None
        and extended_position is not None
        and extended_position[2] > rest_position[2] + 0.30,
        details=f"rest={rest_position}, extended={extended_position}",
    )

    ctx.expect_within(
        shade,
        inner_post,
        axes="y",
        inner_elem="hinge_knuckle",
        outer_elem="yoke_bridge",
        margin=0.002,
        name="shade hinge knuckle sits between the yoke cheeks",
    )

    with ctx.pose({tilt: -0.45}):
        raised_aabb = ctx.part_world_aabb(shade)
    with ctx.pose({tilt: 0.45}):
        lowered_aabb = ctx.part_world_aabb(shade)

    ctx.check(
        "shade revolute hinge visibly tilts the cone",
        raised_aabb is not None
        and lowered_aabb is not None
        and lowered_aabb[0][2] < raised_aabb[0][2] - 0.12,
        details=f"raised={raised_aabb}, lowered={lowered_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
