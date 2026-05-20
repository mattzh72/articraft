from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
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

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="carbon_dropper_seatpost", assets=ASSETS)

    carbon_fiber = model.material("carbon_fiber", rgba=(0.10, 0.11, 0.12, 1.0))
    matte_black = model.material("matte_black", rgba=(0.08, 0.08, 0.09, 1.0))
    hard_anodized = model.material("hard_anodized", rgba=(0.20, 0.21, 0.23, 1.0))
    titanium = model.material("titanium", rgba=(0.62, 0.64, 0.66, 1.0))

    outer_tube = model.part("outer_tube")
    outer_shell_mesh = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [
                (0.0164, 0.000),
                (0.0170, 0.010),
                (0.0170, 0.210),
                (0.0174, 0.228),
                (0.0184, 0.243),
                (0.0184, 0.255),
            ],
            [
                (0.0144, 0.002),
                (0.0144, 0.216),
                (0.0148, 0.243),
                (0.0148, 0.255),
            ],
            segments=72,
            start_cap="flat",
            end_cap="flat",
        ),
        ASSETS.mesh_path("dropper_outer_shell.obj"),
    )
    outer_tube.visual(outer_shell_mesh, material=carbon_fiber, name="outer_shell")
    outer_tube.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0184, length=0.255),
        mass=0.42,
        origin=Origin(xyz=(0.0, 0.0, 0.1275)),
    )

    inner_post = model.part("inner_post")
    inner_post.visual(
        Cylinder(radius=0.0133, length=0.220),
        origin=Origin(xyz=(0.0, 0.0, -0.007)),
        material=hard_anodized,
        name="stanchion",
    )
    inner_post.visual(
        Box((0.012, 0.018, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, 0.088)),
        material=matte_black,
        name="head_pillar",
    )
    inner_post.visual(
        Cylinder(radius=0.0154, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.066)),
        material=matte_black,
        name="stop_collar",
    )
    inner_post.inertial = Inertial.from_geometry(
        Cylinder(radius=0.0136, length=0.235),
        mass=0.28,
        origin=Origin(xyz=(0.0, 0.0, -0.003)),
    )

    saddle_clamp = model.part("saddle_clamp")
    saddle_clamp.visual(
        Box((0.016, 0.032, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=matte_black,
        name="clamp_base",
    )
    saddle_clamp.visual(
        Box((0.012, 0.009, 0.008)),
        origin=Origin(xyz=(0.0, 0.0105, 0.009)),
        material=matte_black,
        name="front_jaw",
    )
    saddle_clamp.visual(
        Box((0.012, 0.009, 0.008)),
        origin=Origin(xyz=(0.0, -0.0105, 0.009)),
        material=matte_black,
        name="rear_jaw",
    )
    saddle_clamp.visual(
        Box((0.010, 0.014, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=matte_black,
        name="center_bridge",
    )
    saddle_clamp.visual(
        Cylinder(radius=0.0032, length=0.050),
        origin=Origin(xyz=(0.0, 0.0105, 0.0154), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=titanium,
        name="front_rail",
    )
    saddle_clamp.visual(
        Cylinder(radius=0.0032, length=0.050),
        origin=Origin(xyz=(0.0, -0.0105, 0.0154), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=titanium,
        name="rear_rail",
    )
    saddle_clamp.visual(
        Cylinder(radius=0.0022, length=0.018),
        origin=Origin(xyz=(0.0, 0.0105, 0.0082), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=titanium,
        name="front_bolt",
    )
    saddle_clamp.visual(
        Cylinder(radius=0.0022, length=0.018),
        origin=Origin(xyz=(0.0, -0.0105, 0.0082), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=titanium,
        name="rear_bolt",
    )
    saddle_clamp.inertial = Inertial.from_geometry(
        Box((0.050, 0.032, 0.020)),
        mass=0.09,
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
    )

    model.articulation(
        "outer_to_inner",
        ArticulationType.PRISMATIC,
        parent=outer_tube,
        child=inner_post,
        origin=Origin(xyz=(0.0, 0.0, 0.192)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=1200.0,
            velocity=0.30,
            lower=0.0,
            upper=0.12,
        ),
    )
    model.articulation(
        "inner_to_clamp",
        ArticulationType.FIXED,
        parent=inner_post,
        child=saddle_clamp,
        origin=Origin(xyz=(0.0, 0.0, 0.096)),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    outer_tube = object_model.get_part("outer_tube")
    inner_post = object_model.get_part("inner_post")
    saddle_clamp = object_model.get_part("saddle_clamp")
    outer_to_inner = object_model.get_articulation("outer_to_inner")

    outer_shell = outer_tube.get_visual("outer_shell")
    stanchion = inner_post.get_visual("stanchion")
    head_pillar = inner_post.get_visual("head_pillar")
    stop_collar = inner_post.get_visual("stop_collar")
    clamp_base = saddle_clamp.get_visual("clamp_base")
    front_rail = saddle_clamp.get_visual("front_rail")
    rear_rail = saddle_clamp.get_visual("rear_rail")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Default exact visual sensor for joint mounting; keep unless scale makes it irrelevant.
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    # Default exact visual sensor for floating/disconnected subassemblies inside one part.
    ctx.warn_if_part_geometry_disconnected()
    # Default articulated-joint clearance gate; adapt only if the model is not articulated.
    ctx.check_articulation_overlaps(max_pose_samples=128)
    # Default broad overlap warning backstop; conservative and non-blocking by default.
    ctx.warn_if_overlaps(max_pose_samples=128, ignore_adjacent=True, ignore_fixed=True)

    ctx.expect_origin_distance(inner_post, outer_tube, axes="xy", max_dist=0.001)
    ctx.expect_within(inner_post, outer_tube, axes="xy", inner_elem=stanchion, outer_elem=outer_shell)

    ctx.expect_origin_distance(saddle_clamp, inner_post, axes="xy", max_dist=0.001)
    ctx.expect_within(inner_post, saddle_clamp, axes="xy", inner_elem=head_pillar, outer_elem=clamp_base)
    ctx.expect_contact(saddle_clamp, inner_post, elem_a=clamp_base, elem_b=head_pillar)
    ctx.expect_gap(
        saddle_clamp,
        inner_post,
        axis="z",
        positive_elem=front_rail,
        negative_elem=head_pillar,
        min_gap=0.012,
        max_gap=0.020,
    )
    ctx.expect_gap(
        saddle_clamp,
        inner_post,
        axis="z",
        positive_elem=rear_rail,
        negative_elem=head_pillar,
        min_gap=0.012,
        max_gap=0.020,
    )

    with ctx.pose({outer_to_inner: 0.0}):
        ctx.expect_contact(inner_post, outer_tube, elem_a=stop_collar, elem_b=outer_shell)
        ctx.expect_gap(
            saddle_clamp,
            outer_tube,
            axis="z",
            positive_elem=clamp_base,
            negative_elem=outer_shell,
            min_gap=0.032,
            max_gap=0.034,
        )
        ctx.expect_gap(
            inner_post,
            outer_tube,
            axis="z",
            positive_elem=head_pillar,
            negative_elem=outer_shell,
            min_gap=0.016,
            max_gap=0.018,
        )
        ctx.expect_overlap(inner_post, outer_tube, axes="z", min_overlap=0.17)

    with ctx.pose({outer_to_inner: 0.12}):
        ctx.expect_gap(
            inner_post,
            outer_tube,
            axis="z",
            positive_elem=stop_collar,
            negative_elem=outer_shell,
            min_gap=0.12,
            max_gap=0.126,
        )
        ctx.expect_gap(
            saddle_clamp,
            outer_tube,
            axis="z",
            positive_elem=clamp_base,
            negative_elem=outer_shell,
            min_gap=0.152,
            max_gap=0.154,
        )
        ctx.expect_gap(
            inner_post,
            outer_tube,
            axis="z",
            positive_elem=head_pillar,
            negative_elem=outer_shell,
            min_gap=0.136,
            max_gap=0.138,
        )
        ctx.expect_overlap(inner_post, outer_tube, axes="z", min_overlap=0.055)
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
