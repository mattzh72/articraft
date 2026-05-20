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
    tube_from_spline_points,
)

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="hydraulic_dropper_seatpost", assets=ASSETS)

    black_anodized = model.material("black_anodized", rgba=(0.12, 0.12, 0.13, 1.0))
    stanchion_silver = model.material("stanchion_silver", rgba=(0.70, 0.73, 0.76, 1.0))
    clamp_black = model.material("clamp_black", rgba=(0.10, 0.10, 0.11, 1.0))
    rail_black = model.material("rail_black", rgba=(0.08, 0.08, 0.09, 1.0))
    seal_gray = model.material("seal_gray", rgba=(0.20, 0.20, 0.21, 1.0))

    def _save_mesh(name: str, geometry):
        return mesh_from_geometry(geometry, ASSETS.mesh_path(name))

    outer_shell = _save_mesh(
        "dropper_outer_shell.obj",
        LatheGeometry.from_shell_profiles(
            [
                (0.0186, 0.000),
                (0.0188, 0.035),
                (0.0190, 0.240),
                (0.0208, 0.276),
                (0.0216, 0.304),
            ],
            [
                (0.0149, 0.000),
                (0.0149, 0.035),
                (0.0149, 0.240),
                (0.0152, 0.276),
                (0.0156, 0.304),
            ],
            segments=72,
            start_cap="flat",
            end_cap="flat",
        ),
    )
    seal_head = _save_mesh(
        "dropper_seal_head.obj",
        LatheGeometry.from_shell_profiles(
            [
                (0.0220, 0.000),
                (0.0220, 0.012),
            ],
            [
                (0.0150, 0.000),
                (0.0150, 0.012),
            ],
            segments=72,
            start_cap="flat",
            end_cap="flat",
        ),
    )
    actuator_loop = _save_mesh(
        "dropper_cable_loop.obj",
        tube_from_spline_points(
            [
                (0.014, 0.028, -0.002),
                (0.022, 0.041, -0.018),
                (0.023, 0.039, -0.060),
                (0.020, 0.034, -0.112),
                (0.014, 0.028, -0.162),
            ],
            radius=0.0025,
            samples_per_segment=16,
            radial_segments=16,
            cap_ends=True,
        ),
    )

    outer_tube = model.part("outer_tube")
    outer_tube.visual(outer_shell, material=black_anodized, name="outer_shell")
    outer_tube.visual(
        seal_head,
        origin=Origin(xyz=(0.0, 0.0, 0.298)),
        material=seal_gray,
        name="seal_head",
    )
    outer_tube.inertial = Inertial.from_geometry(
        Cylinder(radius=0.020, length=0.304),
        mass=0.72,
        origin=Origin(xyz=(0.0, 0.0, 0.152)),
    )

    inner_post = model.part("inner_post")
    inner_post.visual(
        Cylinder(radius=0.0149, length=0.280),
        origin=Origin(xyz=(0.0, 0.0, -0.010)),
        material=stanchion_silver,
        name="stanchion",
    )
    inner_post.visual(
        Cylinder(radius=0.0175, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=black_anodized,
        name="guide_flange",
    )
    inner_post.visual(
        Box((0.022, 0.028, 0.077)),
        origin=Origin(xyz=(0.0, 0.0, 0.1685)),
        material=black_anodized,
        name="head_mast",
    )
    inner_post.visual(
        Box((0.036, 0.020, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.211)),
        material=black_anodized,
        name="head_pad",
    )
    inner_post.inertial = Inertial.from_geometry(
        Box((0.040, 0.040, 0.330)),
        mass=0.58,
        origin=Origin(xyz=(0.0, 0.0, 0.075)),
    )

    clamp = model.part("clamp")
    clamp.visual(
        Box((0.050, 0.036, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, 0.000)),
        material=clamp_black,
        name="clamp_base",
    )
    clamp.visual(
        Box((0.034, 0.010, 0.008)),
        origin=Origin(xyz=(0.0, 0.022, 0.0102)),
        material=clamp_black,
        name="lower_cradle_left",
    )
    clamp.visual(
        Box((0.034, 0.010, 0.008)),
        origin=Origin(xyz=(0.0, -0.022, 0.0102)),
        material=clamp_black,
        name="lower_cradle_right",
    )
    clamp.visual(
        Box((0.032, 0.010, 0.006)),
        origin=Origin(xyz=(0.0, 0.022, 0.0240)),
        material=clamp_black,
        name="upper_cap_left",
    )
    clamp.visual(
        Box((0.032, 0.010, 0.006)),
        origin=Origin(xyz=(0.0, -0.022, 0.0240)),
        material=clamp_black,
        name="upper_cap_right",
    )
    clamp.visual(
        Box((0.008, 0.044, 0.018)),
        origin=Origin(xyz=(0.016, 0.0, 0.016)),
        material=clamp_black,
        name="front_bridge",
    )
    clamp.visual(
        Box((0.008, 0.044, 0.018)),
        origin=Origin(xyz=(-0.016, 0.0, 0.016)),
        material=clamp_black,
        name="rear_bridge",
    )
    clamp.inertial = Inertial.from_geometry(
        Box((0.054, 0.050, 0.034)),
        mass=0.16,
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
    )

    clamp.visual(
        Cylinder(radius=0.0034, length=0.084),
        origin=Origin(xyz=(0.0, 0.022, 0.0176), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rail_black,
        name="rail_left",
    )
    clamp.visual(
        Cylinder(radius=0.0034, length=0.084),
        origin=Origin(xyz=(0.0, -0.022, 0.0176), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rail_black,
        name="rail_right",
    )
    clamp.visual(
        Box((0.012, 0.044, 0.006)),
        origin=Origin(xyz=(-0.028, 0.0, 0.014)),
        material=rail_black,
        name="rail_bridge",
    )
    clamp.visual(
        Box((0.014, 0.010, 0.012)),
        origin=Origin(xyz=(0.010, 0.023, 0.000)),
        material=clamp_black,
        name="side_lug",
    )
    clamp.visual(
        Cylinder(radius=0.0045, length=0.018),
        origin=Origin(xyz=(0.014, 0.028, -0.004), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=clamp_black,
        name="entry_barrel",
    )
    clamp.visual(
        actuator_loop,
        material=clamp_black,
        name="housing_loop",
    )
    clamp.visual(
        Cylinder(radius=0.0040, length=0.014),
        origin=Origin(xyz=(0.014, 0.028, -0.164), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=clamp_black,
        name="lower_stop",
    )

    model.articulation(
        "dropper_travel",
        ArticulationType.PRISMATIC,
        parent=outer_tube,
        child=inner_post,
        origin=Origin(xyz=(0.0, 0.0, 0.304)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=900.0,
            velocity=0.45,
            lower=0.0,
            upper=0.120,
        ),
    )
    model.articulation(
        "head_to_clamp",
        ArticulationType.FIXED,
        parent=inner_post,
        child=clamp,
        origin=Origin(xyz=(0.0, 0.0, 0.222)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    outer_tube = object_model.get_part("outer_tube")
    inner_post = object_model.get_part("inner_post")
    clamp = object_model.get_part("clamp")
    dropper_travel = object_model.get_articulation("dropper_travel")

    outer_shell = outer_tube.get_visual("outer_shell")
    seal_head = outer_tube.get_visual("seal_head")
    stanchion = inner_post.get_visual("stanchion")
    guide_flange = inner_post.get_visual("guide_flange")
    head_mast = inner_post.get_visual("head_mast")
    head_pad = inner_post.get_visual("head_pad")
    clamp_base = clamp.get_visual("clamp_base")
    rail_left = clamp.get_visual("rail_left")
    lower_cradle_left = clamp.get_visual("lower_cradle_left")
    entry_barrel = clamp.get_visual("entry_barrel")
    side_lug = clamp.get_visual("side_lug")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.allow_overlap(
        inner_post,
        outer_tube,
        reason="dropper stanchion telescopes inside the hydraulic outer tube",
    )

    # Default exact visual sensor for joint mounting; keep unless scale makes it irrelevant.
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    # Default exact visual sensor for floating/disconnected subassemblies inside one part.
    ctx.warn_if_part_geometry_disconnected()
    # Default articulated-joint clearance gate; adapt only if the model is not articulated.
    ctx.check_articulation_overlaps(max_pose_samples=128)
    # Default broad overlap warning backstop; conservative and non-blocking by default.
    ctx.warn_if_overlaps(max_pose_samples=128, ignore_adjacent=True, ignore_fixed=True)

    # Use prompt-specific exact visual checks as the real completion criteria.
    # Cover each applicable category before returning:
    # - hero features are present and legible
    # - mounted parts are connected/seated, not floating
    # - important parts are in the right place
    # - key poses stay believable
    # - each new visible form or mechanism has a matching assertion
    # Resolve exact Part / Articulation / named Visual objects once here, then
    # pass those objects into ctx.expect_*, ctx.allow_*, and ctx.pose({joint: value}).
    # Prefer this object-first pattern over raw string test calls or global REFS bags.
    ctx.expect_within(
        inner_post,
        outer_tube,
        axes="xy",
        inner_elem=stanchion,
        outer_elem=outer_shell,
    )
    ctx.expect_overlap(
        inner_post,
        outer_tube,
        axes="xy",
        elem_a=stanchion,
        elem_b=outer_shell,
        min_overlap=0.0005,
    )
    ctx.expect_overlap(
        inner_post,
        outer_tube,
        axes="xy",
        elem_a=stanchion,
        elem_b=seal_head,
        min_overlap=0.0005,
    )
    ctx.expect_gap(
        clamp,
        outer_tube,
        axis="z",
        min_gap=0.180,
        positive_elem=clamp_base,
        negative_elem=outer_shell,
    )
    ctx.expect_contact(clamp, inner_post, elem_a=clamp_base, elem_b=head_pad)
    ctx.expect_overlap(
        clamp,
        inner_post,
        axes="xy",
        elem_a=clamp_base,
        elem_b=head_mast,
        min_overlap=0.0004,
    )
    ctx.expect_gap(
        clamp,
        clamp,
        axis="z",
        max_gap=0.0005,
        max_penetration=0.0,
        positive_elem=rail_left,
        negative_elem=lower_cradle_left,
    )
    ctx.expect_gap(
        clamp,
        inner_post,
        axis="y",
        min_gap=0.004,
        positive_elem=entry_barrel,
        negative_elem=stanchion,
    )
    with ctx.pose({dropper_travel: 0.120}):
        ctx.expect_within(
            inner_post,
            outer_tube,
            axes="xy",
            inner_elem=stanchion,
            outer_elem=outer_shell,
        )
        ctx.expect_gap(
            clamp,
            outer_tube,
            axis="z",
            min_gap=0.300,
            positive_elem=clamp_base,
            negative_elem=outer_shell,
        )
    with ctx.pose({dropper_travel: 0.0}):
        ctx.expect_within(
            inner_post,
            outer_tube,
            axes="xy",
            inner_elem=stanchion,
            outer_elem=outer_shell,
        )
        ctx.expect_gap(
            clamp,
            outer_tube,
            axis="z",
            min_gap=0.180,
            positive_elem=clamp_base,
            negative_elem=outer_shell,
        )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
