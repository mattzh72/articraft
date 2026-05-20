from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports. If the model needs mesh assets, create an
# `AssetContext` inside the editable section.
# >>> USER_CODE_START
import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)

ASSETS = AssetContext.from_script(__file__)

BASE_RADIUS = 0.078
BASE_THICKNESS = 0.018
MAST_RADIUS = 0.015
MAST_HEIGHT = 0.285

HUB_BODY_LENGTH = 0.050
HUB_BODY_WIDTH = 0.032
HUB_BODY_HEIGHT = 0.018
HUB_BODY_INNER_X = MAST_RADIUS + 0.006
HUB_COLLAR_OUTER_RADIUS = MAST_RADIUS + 0.009
HUB_COLLAR_INNER_RADIUS = MAST_RADIUS - 0.0008
HUB_CLAMP_GAP = 0.006
PIVOT_X = 0.055
PIVOT_BOSS_RADIUS = 0.013
PIVOT_BOSS_HEIGHT = 0.008

ARM_COLLAR_RADIUS = 0.013
ARM_COLLAR_HEIGHT = 0.008
ARM_BEAM_WIDTH = 0.018
ARM_BEAM_HEIGHT = 0.008

ARM_SWEEP = math.radians(60.0)

HUB_SPECS = (
    {
        "hub": "hub_lower",
        "arm": "arm_lower",
        "joint": "hub_lower_to_arm_lower",
        "height": 0.080,
        "yaw_deg": 0.0,
        "length": 0.112,
        "style": "pad_round",
    },
    {
        "hub": "hub_middle",
        "arm": "arm_middle",
        "joint": "hub_middle_to_arm_middle",
        "height": 0.155,
        "yaw_deg": 128.0,
        "length": 0.098,
        "style": "pad_square",
    },
    {
        "hub": "hub_upper",
        "arm": "arm_upper",
        "joint": "hub_upper_to_arm_upper",
        "height": 0.230,
        "yaw_deg": 246.0,
        "length": 0.106,
        "style": "fork",
    },
)


def _frame_shape() -> cq.Workplane:
    base = cq.Workplane("XY").circle(BASE_RADIUS).extrude(BASE_THICKNESS)
    base = (
        base.faces(">Z")
        .workplane(centerOption="CenterOfMass")
        .pushPoints(
            [
                (0.050, 0.0),
                (-0.025, 0.043),
                (-0.025, -0.043),
            ]
        )
        .hole(0.010)
    )

    mast = (
        cq.Workplane("XY")
        .circle(MAST_RADIUS)
        .extrude(MAST_HEIGHT)
        .translate((0.0, 0.0, BASE_THICKNESS))
    )
    top_cap = (
        cq.Workplane("XY")
        .circle(MAST_RADIUS * 1.18)
        .extrude(0.010)
        .translate((0.0, 0.0, BASE_THICKNESS + MAST_HEIGHT))
    )

    rib_seed = (
        cq.Workplane("XY")
        .box(0.042, 0.010, 0.050, centered=(False, True, False))
        .translate((0.006, 0.0, BASE_THICKNESS))
    )
    ribs = rib_seed
    for angle_deg in (120.0, 240.0):
        ribs = ribs.union(rib_seed.rotate((0, 0, 0), (0, 0, 1), angle_deg))

    return base.union(mast).union(top_cap).union(ribs)


def _hub_shape() -> cq.Workplane:
    collar = (
        cq.Workplane("XY")
        .circle(HUB_COLLAR_OUTER_RADIUS)
        .circle(HUB_COLLAR_INNER_RADIUS)
        .extrude(HUB_BODY_HEIGHT)
    )
    clamp_relief = (
        cq.Workplane("XY")
        .box(HUB_CLAMP_GAP, HUB_COLLAR_OUTER_RADIUS * 2.4, HUB_BODY_HEIGHT + 0.002, centered=(False, True, False))
        .translate((-HUB_CLAMP_GAP, 0.0, -0.001))
    )
    collar = collar.cut(clamp_relief)
    body = (
        cq.Workplane("XY")
        .box(HUB_BODY_LENGTH, HUB_BODY_WIDTH, HUB_BODY_HEIGHT, centered=(False, True, False))
        .translate((HUB_BODY_INNER_X, 0.0, 0.0))
    )
    body = body.faces(">X").edges("|Z").fillet(0.004)

    boss = (
        cq.Workplane("XY")
        .circle(PIVOT_BOSS_RADIUS)
        .extrude(PIVOT_BOSS_HEIGHT)
        .translate((PIVOT_X, 0.0, HUB_BODY_HEIGHT))
    )
    return collar.union(body).union(boss)


def _arm_shape(length: float, style: str) -> cq.Workplane:
    collar = cq.Workplane("XY").circle(ARM_COLLAR_RADIUS).extrude(ARM_COLLAR_HEIGHT)
    beam_length = length - 0.024
    beam = (
        cq.Workplane("XY")
        .box(beam_length, ARM_BEAM_WIDTH, ARM_BEAM_HEIGHT, centered=(False, True, False))
        .translate((ARM_COLLAR_RADIUS * 0.45, 0.0, 0.0))
    )

    arm = collar.union(beam)

    if style == "pad_round":
        tip = (
            cq.Workplane("XY")
            .circle(0.014)
            .extrude(0.006)
            .translate((length, 0.0, 0.001))
        )
        arm = arm.union(tip)
    elif style == "pad_square":
        tip = (
            cq.Workplane("XY")
            .box(0.024, 0.024, 0.006, centered=(True, True, False))
            .translate((length, 0.0, 0.001))
        )
        arm = arm.union(tip)
    elif style == "fork":
        fork_body = (
            cq.Workplane("XY")
            .box(0.028, 0.024, ARM_BEAM_HEIGHT, centered=(False, True, False))
            .translate((length - 0.020, 0.0, 0.0))
        )
        slot = (
            cq.Workplane("XY")
            .box(0.022, 0.010, ARM_BEAM_HEIGHT + 0.002, centered=(False, True, False))
            .translate((length - 0.014, 0.0, -0.001))
        )
        arm = arm.union(fork_body).cut(slot)

    return arm


def _arm_inertial_origin(length: float) -> Origin:
    return Origin(xyz=(length * 0.52, 0.0, ARM_BEAM_HEIGHT * 0.5))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="three_branch_rotary_fixture", assets=ASSETS)

    graphite = model.material("graphite", rgba=(0.22, 0.24, 0.27, 1.0))
    steel = model.material("steel", rgba=(0.58, 0.60, 0.63, 1.0))
    anodized = model.material("anodized_blue", rgba=(0.26, 0.35, 0.47, 1.0))
    rubber = model.material("rubber", rgba=(0.12, 0.12, 0.13, 1.0))

    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(_frame_shape(), "frame_shell.obj", assets=ASSETS),
        name="frame_shell",
        material=graphite,
    )
    frame.inertial = Inertial.from_geometry(
        Box((BASE_RADIUS * 2.0, BASE_RADIUS * 2.0, BASE_THICKNESS + MAST_HEIGHT + 0.010)),
        mass=4.8,
        origin=Origin(xyz=(0.0, 0.0, (BASE_THICKNESS + MAST_HEIGHT + 0.010) * 0.5)),
    )

    for spec in HUB_SPECS:
        hub = model.part(spec["hub"])
        hub.visual(
            mesh_from_cadquery(_hub_shape(), f"{spec['hub']}_shell.obj", assets=ASSETS),
            name="hub_shell",
            material=steel,
        )
        hub.inertial = Inertial.from_geometry(
            Box((0.068, HUB_BODY_WIDTH, HUB_BODY_HEIGHT + PIVOT_BOSS_HEIGHT)),
            mass=0.62,
            origin=Origin(
                xyz=(
                    HUB_BODY_INNER_X + HUB_BODY_LENGTH * 0.45,
                    0.0,
                    (HUB_BODY_HEIGHT + PIVOT_BOSS_HEIGHT) * 0.5,
                )
            ),
        )

        arm = model.part(spec["arm"])
        arm_material = rubber if spec["style"] == "pad_round" else anodized
        arm.visual(
            mesh_from_cadquery(
                _arm_shape(spec["length"], spec["style"]),
                f"{spec['arm']}_shell.obj",
                assets=ASSETS,
            ),
            name="arm_shell",
            material=arm_material,
        )
        arm.inertial = Inertial.from_geometry(
            Box((spec["length"] + 0.020, 0.026, ARM_COLLAR_HEIGHT)),
            mass=0.34,
            origin=_arm_inertial_origin(spec["length"]),
        )

        model.articulation(
            f"frame_to_{spec['hub']}",
            ArticulationType.FIXED,
            parent=frame,
            child=hub,
            origin=Origin(
                xyz=(0.0, 0.0, spec["height"]),
                rpy=(0.0, 0.0, math.radians(spec["yaw_deg"])),
            ),
        )

        model.articulation(
            spec["joint"],
            ArticulationType.REVOLUTE,
            parent=hub,
            child=arm,
            origin=Origin(xyz=(PIVOT_X, 0.0, HUB_BODY_HEIGHT + PIVOT_BOSS_HEIGHT)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=10.0,
                velocity=1.8,
                lower=-ARM_SWEEP,
                upper=ARM_SWEEP,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    frame = object_model.get_part("frame")
    hub_lower = object_model.get_part("hub_lower")
    hub_middle = object_model.get_part("hub_middle")
    hub_upper = object_model.get_part("hub_upper")
    arm_lower = object_model.get_part("arm_lower")
    arm_middle = object_model.get_part("arm_middle")
    arm_upper = object_model.get_part("arm_upper")

    lower_joint = object_model.get_articulation("hub_lower_to_arm_lower")
    middle_joint = object_model.get_articulation("hub_middle_to_arm_middle")
    upper_joint = object_model.get_articulation("hub_upper_to_arm_upper")
    frame_shell = frame.get_visual("frame_shell")
    hub_lower_shell = hub_lower.get_visual("hub_shell")
    hub_middle_shell = hub_middle.get_visual("hub_shell")
    hub_upper_shell = hub_upper.get_visual("hub_shell")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Preferred default QC stack:
    # 1) likely-failure broad-part floating check for isolated parts
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.allow_overlap(
        frame,
        hub_lower,
        elem_a=frame_shell,
        elem_b=hub_lower_shell,
        reason="Lower hub uses a split clamp collar that intentionally seats around the central mast.",
    )
    ctx.allow_overlap(
        frame,
        hub_middle,
        elem_a=frame_shell,
        elem_b=hub_middle_shell,
        reason="Middle hub uses a split clamp collar that intentionally seats around the central mast.",
    )
    ctx.allow_overlap(
        frame,
        hub_upper,
        elem_a=frame_shell,
        elem_b=hub_upper_shell,
        reason="Upper hub uses a split clamp collar that intentionally seats around the central mast.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    for part_name, part in (
        ("frame", frame),
        ("hub_lower", hub_lower),
        ("hub_middle", hub_middle),
        ("hub_upper", hub_upper),
        ("arm_lower", arm_lower),
        ("arm_middle", arm_middle),
        ("arm_upper", arm_upper),
    ):
        ctx.check(f"{part_name}_present", part is not None, f"missing part: {part_name}")

    for joint_name, joint in (
        ("hub_lower_to_arm_lower", lower_joint),
        ("hub_middle_to_arm_middle", middle_joint),
        ("hub_upper_to_arm_upper", upper_joint),
    ):
        ctx.check(f"{joint_name}_present", joint is not None, f"missing joint: {joint_name}")

    for hub in (hub_lower, hub_middle, hub_upper):
        ctx.expect_contact(hub, frame, name=f"{hub.name}_contacts_mast")
        ctx.expect_overlap(hub, frame, axes="xy", min_overlap=0.028, name=f"{hub.name}_wraps_frame")

    for arm, hub, joint in (
        (arm_lower, hub_lower, lower_joint),
        (arm_middle, hub_middle, middle_joint),
        (arm_upper, hub_upper, upper_joint),
    ):
        ctx.expect_contact(arm, hub, name=f"{arm.name}_seated_on_{hub.name}")
        with ctx.pose({joint: -ARM_SWEEP}):
            ctx.expect_contact(arm, hub, name=f"{arm.name}_contact_at_negative_limit")
        with ctx.pose({joint: ARM_SWEEP}):
            ctx.expect_contact(arm, hub, name=f"{arm.name}_contact_at_positive_limit")

    ctx.expect_origin_gap(hub_middle, hub_lower, axis="z", min_gap=0.070, name="middle_hub_above_lower_hub")
    ctx.expect_origin_gap(hub_upper, hub_middle, axis="z", min_gap=0.070, name="upper_hub_above_middle_hub")

    ctx.expect_origin_distance(arm_lower, arm_middle, axes="xy", min_dist=0.095, name="lower_middle_arm_spacing")
    ctx.expect_origin_distance(arm_middle, arm_upper, axes="xy", min_dist=0.090, name="middle_upper_arm_spacing")
    ctx.expect_origin_distance(arm_upper, arm_lower, axes="xy", min_dist=0.090, name="upper_lower_arm_spacing")

    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=36,
        ignore_adjacent=True,
        ignore_fixed=False,
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
