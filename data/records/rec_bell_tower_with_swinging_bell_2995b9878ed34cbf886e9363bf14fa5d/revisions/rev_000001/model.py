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


def _build_bell_shell():
    outer_profile = [
        (0.165, 0.000),
        (0.162, 0.028),
        (0.146, 0.108),
        (0.112, 0.196),
        (0.062, 0.242),
        (0.030, 0.255),
    ]
    inner_profile = [
        (0.145, 0.012),
        (0.140, 0.040),
        (0.122, 0.110),
        (0.088, 0.194),
        (0.040, 0.238),
        (0.000, 0.248),
    ]
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(outer_profile, inner_profile, segments=72),
        "bell_shell",
    )


def _aabb_center(aabb):
    if aabb is None:
        return None
    lower, upper = aabb
    return tuple((lower[i] + upper[i]) * 0.5 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="alpine_bell_cote")

    aged_oak = model.material("aged_oak", rgba=(0.48, 0.33, 0.20, 1.0))
    roof_shingle = model.material("roof_shingle", rgba=(0.26, 0.18, 0.12, 1.0))
    brass = model.material("brass", rgba=(0.75, 0.60, 0.23, 1.0))
    iron = model.material("iron", rgba=(0.20, 0.20, 0.22, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((0.12, 0.24, 0.10)),
        origin=Origin(xyz=(-0.25, 0.0, 0.05)),
        material=aged_oak,
        name="left_runner",
    )
    frame.visual(
        Box((0.12, 0.24, 0.10)),
        origin=Origin(xyz=(0.25, 0.0, 0.05)),
        material=aged_oak,
        name="right_runner",
    )
    frame.visual(
        Box((0.58, 0.14, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 0.13)),
        material=aged_oak,
        name="base_sill",
    )
    frame.visual(
        Box((0.10, 0.14, 1.34)),
        origin=Origin(xyz=(-0.25, 0.0, 0.83)),
        material=aged_oak,
        name="left_post",
    )
    frame.visual(
        Box((0.10, 0.14, 1.34)),
        origin=Origin(xyz=(0.25, 0.0, 0.83)),
        material=aged_oak,
        name="right_post",
    )
    frame.visual(
        Box((0.60, 0.14, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, 1.42)),
        material=aged_oak,
        name="pivot_beam",
    )
    frame.visual(
        Box((0.62, 0.12, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, 1.52)),
        material=aged_oak,
        name="top_tie",
    )
    frame.visual(
        Box((0.40, 0.32, 0.05)),
        origin=Origin(xyz=(-0.18, 0.0, 1.68), rpy=(0.0, -0.82, 0.0)),
        material=roof_shingle,
        name="roof_left",
    )
    frame.visual(
        Box((0.40, 0.32, 0.05)),
        origin=Origin(xyz=(0.18, 0.0, 1.68), rpy=(0.0, 0.82, 0.0)),
        material=roof_shingle,
        name="roof_right",
    )
    frame.visual(
        Box((0.10, 0.34, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 1.81)),
        material=roof_shingle,
        name="ridge_cap",
    )
    frame.inertial = Inertial.from_geometry(
        Box((0.72, 0.34, 1.86)),
        mass=42.0,
        origin=Origin(xyz=(0.0, 0.0, 0.93)),
    )

    bell = model.part("bell")
    bell.visual(
        _build_bell_shell(),
        origin=Origin(xyz=(0.0, 0.0, -0.335)),
        material=brass,
        name="bell_shell",
    )
    bell.visual(
        Cylinder(radius=0.046, length=0.088),
        origin=Origin(xyz=(0.0, 0.0, -0.085)),
        material=brass,
        name="bell_crown",
    )
    bell.visual(
        Box((0.18, 0.08, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, -0.02)),
        material=iron,
        name="bell_headstock",
    )
    bell.visual(
        Cylinder(radius=0.018, length=0.40),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=iron,
        name="bell_axle",
    )
    bell.inertial = Inertial.from_geometry(
        Cylinder(radius=0.17, length=0.36),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, -0.18)),
    )

    model.articulation(
        "frame_to_bell",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=bell,
        origin=Origin(xyz=(0.0, 0.0, 1.33)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=1.2,
            lower=-0.55,
            upper=0.55,
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

    frame = object_model.get_part("frame")
    bell = object_model.get_part("bell")
    hinge = object_model.get_articulation("frame_to_bell")

    frame.get_visual("left_post")
    frame.get_visual("right_post")
    frame.get_visual("pivot_beam")
    bell.get_visual("bell_shell")
    bell.get_visual("bell_headstock")
    bell.get_visual("bell_axle")

    limits = hinge.motion_limits
    axis = tuple(round(value, 4) for value in hinge.axis)
    ctx.check(
        "bell hinge uses horizontal headstock axis",
        axis == (1.0, 0.0, 0.0),
        details=f"axis={hinge.axis}",
    )
    ctx.check(
        "bell hinge swings both directions from vertical",
        limits is not None
        and limits.lower is not None
        and limits.upper is not None
        and limits.lower < 0.0 < limits.upper,
        details=f"limits={limits}",
    )

    with ctx.pose({hinge: 0.0}):
        ctx.expect_within(
            bell,
            frame,
            axes="xy",
            inner_elem="bell_axle",
            outer_elem="pivot_beam",
            margin=0.03,
            name="bell axle stays under the pivot beam footprint",
        )
        ctx.expect_contact(
            bell,
            frame,
            elem_a="bell_axle",
            elem_b="left_post",
            name="bell axle bears on the left upright",
        )
        ctx.expect_contact(
            bell,
            frame,
            elem_a="bell_axle",
            elem_b="right_post",
            name="bell axle bears on the right upright",
        )
        ctx.expect_gap(
            frame,
            bell,
            axis="z",
            positive_elem="pivot_beam",
            negative_elem="bell_headstock",
            min_gap=0.0,
            max_gap=0.08,
            name="headstock hangs just below the pivot beam",
        )

    rest_center = None
    swung_center = None
    with ctx.pose({hinge: 0.0}):
        rest_center = _aabb_center(ctx.part_element_world_aabb(bell, elem="bell_shell"))
    with ctx.pose({hinge: 0.45}):
        swung_aabb = ctx.part_element_world_aabb(bell, elem="bell_shell")
        swung_center = _aabb_center(swung_aabb)
        ctx.check(
            "bell keeps clearance above the sill at full swing",
            swung_aabb is not None and swung_aabb[0][2] > 0.62,
            details=f"swung_aabb={swung_aabb}",
        )

    ctx.check(
        "positive hinge angle swings the bell toward positive y",
        rest_center is not None
        and swung_center is not None
        and swung_center[1] > rest_center[1] + 0.05,
        details=f"rest_center={rest_center}, swung_center={swung_center}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
