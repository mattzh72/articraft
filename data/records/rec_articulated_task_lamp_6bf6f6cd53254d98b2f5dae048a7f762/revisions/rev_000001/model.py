from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="piano_lamp")

    brass = model.material("satin_brass", rgba=(0.72, 0.59, 0.33, 1.0))
    diffuser = model.material("diffuser_white", rgba=(0.95, 0.93, 0.88, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.11, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=brass,
        name="base_plate",
    )
    base.visual(
        Cylinder(radius=0.026, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material=brass,
        name="swivel_hub",
    )
    base.inertial = Inertial.from_geometry(
        Cylinder(radius=0.11, length=0.028),
        mass=2.2,
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
    )

    post = model.part("post")
    post.visual(
        Cylinder(radius=0.024, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=brass,
        name="post_collar",
    )
    post.visual(
        Cylinder(radius=0.011, length=0.205),
        origin=Origin(xyz=(0.0, 0.0, 0.1115)),
        material=brass,
        name="post_stem",
    )
    post.visual(
        Cylinder(radius=0.015, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.225)),
        material=brass,
        name="upper_pivot_block",
    )
    post.visual(
        Box((0.020, 0.005, 0.032)),
        origin=Origin(xyz=(0.0, 0.011, 0.252)),
        material=brass,
        name="pivot_cheek_left",
    )
    post.visual(
        Box((0.020, 0.005, 0.032)),
        origin=Origin(xyz=(0.0, -0.011, 0.252)),
        material=brass,
        name="pivot_cheek_right",
    )
    post.inertial = Inertial.from_geometry(
        Cylinder(radius=0.018, length=0.245),
        mass=0.7,
        origin=Origin(xyz=(0.0, 0.0, 0.1225)),
    )

    head = model.part("head")
    head.visual(
        Box((0.016, 0.017, 0.028)),
        origin=Origin(xyz=(0.002, 0.0, 0.0)),
        material=brass,
        name="tilt_tongue",
    )
    head.visual(
        Box((0.022, 0.014, 0.020)),
        origin=Origin(xyz=(0.015, 0.0, 0.015)),
        material=brass,
        name="head_neck",
    )
    head.visual(
        Box((0.400, 0.055, 0.038)),
        origin=Origin(xyz=(0.222, 0.0, 0.029)),
        material=brass,
        name="housing_shell",
    )
    head.visual(
        Box((0.350, 0.040, 0.003)),
        origin=Origin(xyz=(0.227, 0.0, 0.0085)),
        material=diffuser,
        name="diffuser_panel",
    )
    head.inertial = Inertial.from_geometry(
        Box((0.430, 0.060, 0.050)),
        mass=0.9,
        origin=Origin(xyz=(0.215, 0.0, 0.026)),
    )

    model.articulation(
        "base_to_post",
        ArticulationType.REVOLUTE,
        parent=base,
        child=post,
        origin=Origin(xyz=(0.0, 0.0, 0.0275)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=1.8,
            lower=-2.2,
            upper=2.2,
        ),
    )
    model.articulation(
        "post_to_head",
        ArticulationType.REVOLUTE,
        parent=post,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, 0.252)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.6,
            lower=-0.45,
            upper=0.85,
        ),
    )

    return model


def run_tests() -> TestReport:
    def aabb_center(aabb):
        if aabb is None:
            return None
        return tuple((aabb[0][i] + aabb[1][i]) * 0.5 for i in range(3))

    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    post = object_model.get_part("post")
    head = object_model.get_part("head")
    swivel = object_model.get_articulation("base_to_post")
    tilt = object_model.get_articulation("post_to_head")
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    ctx.expect_contact(
        post,
        base,
        elem_a="post_collar",
        elem_b="swivel_hub",
        contact_tol=5e-4,
        name="post collar seats on swivel hub",
    )

    with ctx.pose({swivel: 0.0, tilt: 0.0}):
        ctx.expect_origin_gap(
            head,
            base,
            axis="z",
            min_gap=0.22,
            name="lamp head is elevated above the base",
        )
        rest_head_aabb = ctx.part_element_world_aabb(head, elem="housing_shell")

    with ctx.pose({swivel: 0.9, tilt: 0.0}):
        swiveled_head_aabb = ctx.part_element_world_aabb(head, elem="housing_shell")

    with ctx.pose({swivel: 0.0, tilt: 0.65}):
        tilted_head_aabb = ctx.part_element_world_aabb(head, elem="housing_shell")

    with ctx.pose({swivel: 0.0, tilt: -0.35}):
        ctx.expect_gap(
            head,
            base,
            axis="z",
            min_gap=0.09,
            name="downward-tilted housing still clears the base",
        )

    rest_center = aabb_center(rest_head_aabb)
    swiveled_center = aabb_center(swiveled_head_aabb)

    ctx.check(
        "swivel rotates the light bar around the post",
        rest_center is not None
        and swiveled_center is not None
        and swiveled_center[1] > rest_center[1] + 0.12,
        details=f"rest_center={rest_center}, swiveled_center={swiveled_center}",
    )
    ctx.check(
        "tilt raises the free end of the housing",
        rest_head_aabb is not None
        and tilted_head_aabb is not None
        and tilted_head_aabb[1][2] > rest_head_aabb[1][2] + 0.10,
        details=f"rest_aabb={rest_head_aabb}, tilted_aabb={tilted_head_aabb}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
