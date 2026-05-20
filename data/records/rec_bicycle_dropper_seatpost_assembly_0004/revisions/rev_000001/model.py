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
    ExtrudeGeometry,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)

ASSETS = AssetContext.from_script(__file__)


def _adapter_shell():
    geometry = LatheGeometry.from_shell_profiles(
        outer_profile=[
            (0.0188, -0.058),
            (0.0190, -0.020),
            (0.0194, -0.004),
            (0.0206, 0.000),
        ],
        inner_profile=[
            (0.0160, -0.058),
            (0.0160, -0.006),
            (0.0172, 0.000),
        ],
        segments=64,
        start_cap="flat",
        end_cap="flat",
    )
    return mesh_from_geometry(geometry, ASSETS.mesh_path("seatpost_adapter_shell.obj"))


def _clamp_ring_shell():
    geometry = LatheGeometry.from_shell_profiles(
        outer_profile=[
            (0.0206, 0.000),
            (0.0208, 0.015),
            (0.0204, 0.018),
        ],
        inner_profile=[
            (0.0166, 0.000),
            (0.0166, 0.016),
            (0.0170, 0.018),
        ],
        segments=64,
        start_cap="flat",
        end_cap="flat",
    )
    return mesh_from_geometry(geometry, ASSETS.mesh_path("seatpost_clamp_ring.obj"))


def _binder_lever_plate():
    profile = [
        (0.0025, 0.0050),
        (0.0100, 0.0085),
        (0.0280, 0.0100),
        (0.0490, 0.0070),
        (0.0620, 0.0030),
        (0.0660, -0.0010),
        (0.0560, -0.0050),
        (0.0280, -0.0065),
        (0.0100, -0.0105),
        (0.0030, -0.0060),
    ]
    geometry = ExtrudeGeometry.centered(profile, height=0.0050, cap=True, closed=True)
    geometry.rotate_x(math.pi / 2.0)
    return mesh_from_geometry(geometry, ASSETS.mesh_path("binder_lever_plate.obj"))


def _outer_post_shell():
    geometry = LatheGeometry.from_shell_profiles(
        outer_profile=[
            (0.0154, -0.125),
            (0.0154, 0.123),
            (0.0150, 0.125),
        ],
        inner_profile=[
            (0.0139, -0.125),
            (0.0139, 0.121),
            (0.0135, 0.125),
        ],
        segments=64,
        start_cap="flat",
        end_cap="flat",
    )
    return mesh_from_geometry(geometry, ASSETS.mesh_path("outer_post_shell.obj"))


def _seal_collar_shell():
    geometry = LatheGeometry.from_shell_profiles(
        outer_profile=[
            (0.0164, -0.006),
            (0.0164, 0.005),
            (0.0160, 0.006),
        ],
        inner_profile=[
            (0.0139, -0.006),
            (0.0139, 0.005),
            (0.0135, 0.006),
        ],
        segments=64,
        start_cap="flat",
        end_cap="flat",
    )
    return mesh_from_geometry(geometry, ASSETS.mesh_path("outer_post_seal_collar.obj"))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dropper_seatpost_assembly", assets=ASSETS)

    matte_black = model.material("matte_black", rgba=(0.10, 0.10, 0.11, 1.0))
    dark_alloy = model.material("dark_alloy", rgba=(0.18, 0.18, 0.19, 1.0))
    bolt_silver = model.material("bolt_silver", rgba=(0.72, 0.74, 0.77, 1.0))

    adapter = model.part("adapter")
    adapter.visual(_adapter_shell(), material=dark_alloy, name="adapter_shell")

    clamp = model.part("clamp_ring")
    clamp.visual(_clamp_ring_shell(), material=matte_black, name="ring_shell")
    clamp.visual(
        Box((0.0090, 0.0140, 0.0180)),
        origin=Origin(xyz=(0.0246, 0.0, 0.0090)),
        material=matte_black,
        name="jaw_block",
    )

    lever = model.part("binder_lever")
    lever.visual(
        Cylinder(radius=0.0033, length=0.0160),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=bolt_silver,
        name="pivot_barrel",
    )
    lever.visual(
        Cylinder(radius=0.0038, length=0.0120),
        origin=Origin(xyz=(-0.0016, 0.0, -0.0012), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_alloy,
        name="cam_lobe",
    )
    lever.visual(_binder_lever_plate(), material=matte_black, name="lever_plate")
    lever.visual(
        Box((0.0120, 0.0060, 0.0060)),
        origin=Origin(xyz=(0.0580, 0.0, 0.0005)),
        material=matte_black,
        name="handle_tip",
    )

    outer = model.part("outer_post")
    outer.visual(
        _outer_post_shell(),
        origin=Origin(xyz=(0.0, 0.0, 0.0350)),
        material=matte_black,
        name="outer_tube",
    )
    outer.visual(
        _seal_collar_shell(),
        origin=Origin(xyz=(0.0, 0.0, 0.1560)),
        material=dark_alloy,
        name="seal_collar",
    )

    inner = model.part("inner_post")
    inner.visual(
        Cylinder(radius=0.0132, length=0.2200),
        origin=Origin(xyz=(0.0, 0.0, -0.0300)),
        material=dark_alloy,
        name="inner_tube",
    )
    inner.visual(
        Cylinder(radius=0.0141, length=0.0080),
        origin=Origin(xyz=(0.0, 0.0, -0.1340)),
        material=matte_black,
        name="lower_guide_bushing",
    )
    inner.visual(
        Cylinder(radius=0.0141, length=0.0080),
        origin=Origin(xyz=(0.0, 0.0, -0.1180)),
        material=matte_black,
        name="upper_guide_bushing",
    )
    inner.visual(
        Box((0.0180, 0.0200, 0.0120)),
        origin=Origin(xyz=(0.0, 0.0, 0.0860)),
        material=dark_alloy,
        name="perch_bridge",
    )
    inner.visual(
        Box((0.0260, 0.0500, 0.0080)),
        origin=Origin(xyz=(0.0, 0.0, 0.0960)),
        material=matte_black,
        name="upper_yoke",
    )
    inner.visual(
        Cylinder(radius=0.0030, length=0.0340),
        origin=Origin(xyz=(0.0, 0.0140, 0.0890), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bolt_silver,
        name="front_bolt",
    )
    inner.visual(
        Cylinder(radius=0.0030, length=0.0340),
        origin=Origin(xyz=(0.0, -0.0140, 0.0890), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bolt_silver,
        name="rear_bolt",
    )

    model.articulation(
        "adapter_to_clamp",
        ArticulationType.FIXED,
        parent=adapter,
        child=clamp,
        origin=Origin(),
    )
    model.articulation(
        "clamp_to_binder_lever",
        ArticulationType.REVOLUTE,
        parent=clamp,
        child=lever,
        origin=Origin(xyz=(0.0325, 0.0, 0.0090)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=3.0,
            lower=-1.65,
            upper=0.0,
        ),
    )
    model.articulation(
        "clamp_to_outer_post",
        ArticulationType.FIXED,
        parent=clamp,
        child=outer,
        origin=Origin(),
    )
    model.articulation(
        "outer_to_inner_post",
        ArticulationType.PRISMATIC,
        parent=outer,
        child=inner,
        origin=Origin(xyz=(0.0, 0.0, 0.1620)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=0.40,
            lower=0.0,
            upper=0.110,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    adapter = object_model.get_part("adapter")
    clamp = object_model.get_part("clamp_ring")
    lever = object_model.get_part("binder_lever")
    outer = object_model.get_part("outer_post")
    inner = object_model.get_part("inner_post")
    binder_hinge = object_model.get_articulation("clamp_to_binder_lever")
    dropper_slide = object_model.get_articulation("outer_to_inner_post")

    adapter_shell = adapter.get_visual("adapter_shell")
    ring_shell = clamp.get_visual("ring_shell")
    jaw_block = clamp.get_visual("jaw_block")
    pivot_barrel = lever.get_visual("pivot_barrel")
    cam_lobe = lever.get_visual("cam_lobe")
    lever_plate = lever.get_visual("lever_plate")
    handle_tip = lever.get_visual("handle_tip")
    outer_tube = outer.get_visual("outer_tube")
    seal_collar = outer.get_visual("seal_collar")
    inner_tube = inner.get_visual("inner_tube")
    lower_guide_bushing = inner.get_visual("lower_guide_bushing")
    upper_guide_bushing = inner.get_visual("upper_guide_bushing")
    perch_bridge = inner.get_visual("perch_bridge")
    front_bolt = inner.get_visual("front_bolt")
    rear_bolt = inner.get_visual("rear_bolt")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.allow_overlap(
        inner,
        outer,
        reason="telescoping dropper post uses nested guide bushings and a sliding stanchion inside the outer tube",
    )

    # Default exact visual sensor for joint mounting; keep unless scale makes it irrelevant.
    ctx.warn_if_articulation_origin_near_geometry(tol=0.017)
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
    # Example:
    # lid = object_model.get_part("lid")
    # body = object_model.get_part("body")
    # lid_hinge = object_model.get_articulation("lid_hinge")
    # hinge_leaf = lid.get_visual("hinge_leaf")
    # body_leaf = body.get_visual("body_leaf")
    # ctx.expect_overlap(lid, body, axes="xy", min_overlap=0.05)
    # ctx.expect_gap(lid, body, axis="z", max_gap=0.001, max_penetration=0.0)
    # ctx.expect_contact(lid, body, elem_a=hinge_leaf, elem_b=body_leaf)
    # Add prompt-specific exact visual checks below; broad warn_if_* checks are not enough.
    ctx.expect_within(
        outer,
        clamp,
        axes="xy",
        inner_elem=outer_tube,
        outer_elem=ring_shell,
        name="outer post fits through clamp ring",
    )
    ctx.expect_within(
        outer,
        adapter,
        axes="xy",
        inner_elem=outer_tube,
        outer_elem=adapter_shell,
        name="outer post fits through sleeve adapter",
    )
    ctx.expect_gap(
        clamp,
        adapter,
        axis="z",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=ring_shell,
        negative_elem=adapter_shell,
        name="clamp ring seats on sleeve adapter",
    )
    ctx.expect_contact(
        lever,
        clamp,
        elem_a=cam_lobe,
        elem_b=jaw_block,
        name="binder lever cam seats against clamp jaw",
    )
    ctx.expect_overlap(
        lever,
        clamp,
        axes="yz",
        min_overlap=0.0005,
        elem_a=pivot_barrel,
        elem_b=jaw_block,
        name="lever pivot aligns with jaw block",
    )
    ctx.expect_gap(
        lever,
        clamp,
        axis="x",
        min_gap=0.012,
        positive_elem=lever_plate,
        negative_elem=ring_shell,
        name="binder lever plate projects clearly outside the clamp ring",
    )
    ctx.expect_within(
        inner,
        outer,
        axes="xy",
        inner_elem=inner_tube,
        outer_elem=outer_tube,
        name="inner stanchion nests inside outer tube",
    )
    ctx.expect_contact(
        inner,
        outer,
        elem_a=lower_guide_bushing,
        elem_b=outer_tube,
        name="lower guide bushing rides in the outer tube",
    )
    ctx.expect_gap(
        inner,
        outer,
        axis="z",
        min_gap=0.012,
        positive_elem=perch_bridge,
        negative_elem=seal_collar,
        name="saddle perch rises above outer collar",
    )
    ctx.expect_gap(
        inner,
        outer,
        axis="z",
        min_gap=0.070,
        positive_elem=front_bolt,
        negative_elem=seal_collar,
        name="front saddle bolt sits above collar",
    )
    ctx.expect_gap(
        inner,
        outer,
        axis="z",
        min_gap=0.070,
        positive_elem=rear_bolt,
        negative_elem=seal_collar,
        name="rear saddle bolt sits above collar",
    )

    with ctx.pose({binder_hinge: -1.55}):
        ctx.expect_gap(
            lever,
            clamp,
            axis="z",
            min_gap=0.030,
            positive_elem=handle_tip,
            negative_elem=ring_shell,
            name="binder lever opens upward clear of clamp",
        )

    with ctx.pose({dropper_slide: 0.110}):
        ctx.expect_within(
            inner,
            outer,
            axes="xy",
            inner_elem=inner_tube,
            outer_elem=outer_tube,
            name="inner tube stays coaxial at full dropper extension",
        )
        ctx.expect_gap(
            inner,
            outer,
            axis="z",
            min_gap=0.110,
            positive_elem=perch_bridge,
            negative_elem=seal_collar,
            name="dropper extension lifts the perch distinctly",
        )
        ctx.expect_contact(
            inner,
            outer,
            elem_a=upper_guide_bushing,
            elem_b=seal_collar,
            name="upper guide bushing remains guided at full extension",
        )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
