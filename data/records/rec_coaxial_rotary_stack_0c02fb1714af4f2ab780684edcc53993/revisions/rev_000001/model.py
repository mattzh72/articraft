from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_RADIUS = 0.22
BASE_HEIGHT = 0.08
BASE_DECK_RADIUS = 0.20
BASE_DECK_HEIGHT = 0.022

LOWER_OUTER_RADIUS = 0.25
LOWER_BODY_HEIGHT = 0.028
LOWER_RIM_HEIGHT = 0.008
LOWER_RIM_RADIUS = 0.238
LOWER_RING_INNER_CLEAR_RADIUS = 0.125
LOWER_HUB_RADIUS = 0.074
LOWER_SUPPORT_RADIUS = 0.05
LOWER_SUPPORT_HEIGHT = 0.03
LOWER_SUPPORT_PAD_RADIUS = 0.072
LOWER_SUPPORT_PAD_HEIGHT = 0.004
LOWER_TOTAL_HEIGHT = LOWER_BODY_HEIGHT + LOWER_SUPPORT_HEIGHT + LOWER_SUPPORT_PAD_HEIGHT

MIDDLE_OUTER_RADIUS = 0.135
MIDDLE_BODY_HEIGHT = 0.036
MIDDLE_TOP_COLLAR_HEIGHT = 0.008
MIDDLE_TOP_COLLAR_RADIUS = 0.118
MIDDLE_SUPPORT_RADIUS = 0.028
MIDDLE_SUPPORT_HEIGHT = 0.02
MIDDLE_SUPPORT_PAD_RADIUS = 0.043
MIDDLE_SUPPORT_PAD_HEIGHT = 0.004
MIDDLE_TOTAL_HEIGHT = (
    MIDDLE_BODY_HEIGHT + MIDDLE_TOP_COLLAR_HEIGHT + MIDDLE_SUPPORT_HEIGHT + MIDDLE_SUPPORT_PAD_HEIGHT
)

UPPER_FLANGE_RADIUS = 0.08
UPPER_FLANGE_HEIGHT = 0.016
UPPER_CAP_RADIUS = 0.035
UPPER_CAP_HEIGHT = 0.012


def make_radial_tab(
    *,
    base_radius: float,
    length: float,
    width: float,
    height: float,
    angle_deg: float,
    z_offset: float = 0.0,
) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .workplane(offset=z_offset)
        .transformed(rotate=(0.0, 0.0, angle_deg))
        .center(base_radius - 0.35 * length, 0.0)
        .rect(length, width)
        .extrude(height)
    )


def make_base_shape() -> cq.Workplane:
    housing = cq.Workplane("XY").circle(BASE_RADIUS).extrude(BASE_HEIGHT - BASE_DECK_HEIGHT)
    deck = (
        cq.Workplane("XY")
        .workplane(offset=BASE_HEIGHT - BASE_DECK_HEIGHT)
        .circle(BASE_DECK_RADIUS)
        .extrude(BASE_DECK_HEIGHT)
    )
    return housing.union(deck)


def make_lower_platter_shape() -> cq.Workplane:
    annulus_outer = cq.Workplane("XY").circle(LOWER_OUTER_RADIUS).extrude(LOWER_BODY_HEIGHT)
    annulus_inner = (
        cq.Workplane("XY").circle(LOWER_RING_INNER_CLEAR_RADIUS).extrude(LOWER_BODY_HEIGHT)
    )
    platter = annulus_outer.cut(annulus_inner)
    hub = cq.Workplane("XY").circle(LOWER_HUB_RADIUS).extrude(LOWER_BODY_HEIGHT)
    spokes = cq.Workplane("XY")
    for angle_deg in (0.0, 120.0, 240.0):
        spoke = (
            cq.Workplane("XY")
            .transformed(rotate=(0.0, 0.0, angle_deg))
            .center(0.098, 0.0)
            .rect(0.072, 0.028)
            .extrude(LOWER_BODY_HEIGHT)
        )
        spokes = spokes.union(spoke)
    rim = (
        cq.Workplane("XY")
        .workplane(offset=LOWER_BODY_HEIGHT)
        .circle(LOWER_RIM_RADIUS)
        .extrude(LOWER_RIM_HEIGHT)
    )
    grip = make_radial_tab(
        base_radius=LOWER_OUTER_RADIUS,
        length=0.04,
        width=0.07,
        height=LOWER_BODY_HEIGHT * 0.55,
        angle_deg=18.0,
        z_offset=0.0,
    )
    return platter.union(hub).union(spokes).union(rim).union(grip)


def make_lower_support_shape() -> cq.Workplane:
    column = (
        cq.Workplane("XY")
        .workplane(offset=LOWER_BODY_HEIGHT)
        .circle(LOWER_SUPPORT_RADIUS)
        .extrude(LOWER_SUPPORT_HEIGHT)
    )
    pad = (
        cq.Workplane("XY")
        .workplane(offset=LOWER_BODY_HEIGHT + LOWER_SUPPORT_HEIGHT)
        .circle(LOWER_SUPPORT_PAD_RADIUS)
        .extrude(LOWER_SUPPORT_PAD_HEIGHT)
    )
    return column.union(pad)


def make_middle_platter_shape() -> cq.Workplane:
    platter = cq.Workplane("XY").circle(MIDDLE_OUTER_RADIUS).extrude(MIDDLE_BODY_HEIGHT)
    collar = (
        cq.Workplane("XY")
        .workplane(offset=MIDDLE_BODY_HEIGHT)
        .circle(MIDDLE_TOP_COLLAR_RADIUS)
        .extrude(MIDDLE_TOP_COLLAR_HEIGHT)
    )
    grip = make_radial_tab(
        base_radius=MIDDLE_OUTER_RADIUS,
        length=0.03,
        width=0.05,
        height=MIDDLE_BODY_HEIGHT * 0.5,
        angle_deg=142.0,
        z_offset=0.0,
    )
    return platter.union(collar).union(grip)


def make_middle_support_shape() -> cq.Workplane:
    column = (
        cq.Workplane("XY")
        .workplane(offset=MIDDLE_BODY_HEIGHT + MIDDLE_TOP_COLLAR_HEIGHT)
        .circle(MIDDLE_SUPPORT_RADIUS)
        .extrude(MIDDLE_SUPPORT_HEIGHT)
    )
    pad = (
        cq.Workplane("XY")
        .workplane(offset=MIDDLE_BODY_HEIGHT + MIDDLE_TOP_COLLAR_HEIGHT + MIDDLE_SUPPORT_HEIGHT)
        .circle(MIDDLE_SUPPORT_PAD_RADIUS)
        .extrude(MIDDLE_SUPPORT_PAD_HEIGHT)
    )
    return column.union(pad)


def make_upper_flange_shape() -> cq.Workplane:
    flange = cq.Workplane("XY").circle(UPPER_FLANGE_RADIUS).extrude(UPPER_FLANGE_HEIGHT)
    cap = (
        cq.Workplane("XY")
        .workplane(offset=UPPER_FLANGE_HEIGHT)
        .circle(UPPER_CAP_RADIUS)
        .extrude(UPPER_CAP_HEIGHT)
    )
    pointer = make_radial_tab(
        base_radius=UPPER_FLANGE_RADIUS,
        length=0.022,
        width=0.022,
        height=UPPER_FLANGE_HEIGHT * 0.7,
        angle_deg=275.0,
        z_offset=0.0,
    )
    return flange.union(cap).union(pointer)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="coaxial_three_tier_stack")

    dark_body = model.material("dark_body", rgba=(0.18, 0.19, 0.21, 1.0))
    lower_metal = model.material("lower_metal", rgba=(0.43, 0.45, 0.48, 1.0))
    middle_metal = model.material("middle_metal", rgba=(0.58, 0.60, 0.63, 1.0))
    top_metal = model.material("top_metal", rgba=(0.70, 0.72, 0.75, 1.0))

    base = model.part("base_housing")
    base.visual(
        mesh_from_cadquery(make_base_shape(), "base_housing"),
        material=dark_body,
        name="base_shell",
    )

    lower_ring = model.part("lower_ring")
    lower_ring.visual(
        mesh_from_cadquery(make_lower_platter_shape(), "lower_platter_v3"),
        material=lower_metal,
        name="lower_platter",
    )
    lower_ring.visual(
        mesh_from_cadquery(make_lower_support_shape(), "lower_support_v3"),
        material=lower_metal,
        name="lower_support",
    )

    middle_stage = model.part("middle_stage")
    middle_stage.visual(
        mesh_from_cadquery(make_middle_platter_shape(), "middle_platter_v3"),
        material=middle_metal,
        name="middle_platter",
    )
    middle_stage.visual(
        mesh_from_cadquery(make_middle_support_shape(), "middle_support_v3"),
        material=middle_metal,
        name="middle_support",
    )

    upper_flange = model.part("upper_flange")
    upper_flange.visual(
        mesh_from_cadquery(make_upper_flange_shape(), "upper_flange_v3"),
        material=top_metal,
        name="upper_shell",
    )

    model.articulation(
        "base_to_lower_ring",
        ArticulationType.REVOLUTE,
        parent=base,
        child=lower_ring,
        origin=Origin(xyz=(0.0, 0.0, BASE_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=2.5,
            lower=-3.14159,
            upper=3.14159,
        ),
    )

    model.articulation(
        "lower_ring_to_middle_stage",
        ArticulationType.REVOLUTE,
        parent=lower_ring,
        child=middle_stage,
        origin=Origin(xyz=(0.0, 0.0, LOWER_TOTAL_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=2.5,
            lower=-3.14159,
            upper=3.14159,
        ),
    )

    model.articulation(
        "middle_stage_to_upper_flange",
        ArticulationType.REVOLUTE,
        parent=middle_stage,
        child=upper_flange,
        origin=Origin(xyz=(0.0, 0.0, MIDDLE_TOTAL_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=3.0,
            lower=-3.14159,
            upper=3.14159,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_housing")
    lower_ring = object_model.get_part("lower_ring")
    middle_stage = object_model.get_part("middle_stage")
    upper_flange = object_model.get_part("upper_flange")

    lower_joint = object_model.get_articulation("base_to_lower_ring")
    middle_joint = object_model.get_articulation("lower_ring_to_middle_stage")
    upper_joint = object_model.get_articulation("middle_stage_to_upper_flange")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
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
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "all_joint_axes_vertical",
        lower_joint.axis == (0.0, 0.0, 1.0)
        and middle_joint.axis == (0.0, 0.0, 1.0)
        and upper_joint.axis == (0.0, 0.0, 1.0),
        (
            f"lower={lower_joint.axis}, middle={middle_joint.axis}, "
            f"upper={upper_joint.axis}"
        ),
    )
    ctx.check(
        "joint_origins_are_coaxial",
        lower_joint.origin.xyz[:2] == (0.0, 0.0)
        and middle_joint.origin.xyz[:2] == (0.0, 0.0)
        and upper_joint.origin.xyz[:2] == (0.0, 0.0),
        (
            f"lower={lower_joint.origin.xyz}, middle={middle_joint.origin.xyz}, "
            f"upper={upper_joint.origin.xyz}"
        ),
    )
    ctx.check(
        "three_level_parent_chain",
        lower_joint.parent == base.name
        and lower_joint.child == lower_ring.name
        and middle_joint.parent == lower_ring.name
        and middle_joint.child == middle_stage.name
        and upper_joint.parent == middle_stage.name
        and upper_joint.child == upper_flange.name,
        (
            f"lower=({lower_joint.parent}->{lower_joint.child}), "
            f"middle=({middle_joint.parent}->{middle_joint.child}), "
            f"upper=({upper_joint.parent}->{upper_joint.child})"
        ),
    )

    ctx.expect_gap(
        lower_ring,
        base,
        axis="z",
        min_gap=0.0,
        max_gap=0.001,
        name="lower_ring_seated_on_base",
    )
    ctx.expect_gap(
        middle_stage,
        lower_ring,
        axis="z",
        min_gap=0.0,
        max_gap=0.001,
        positive_elem="middle_platter",
        negative_elem="lower_support",
        name="middle_stage_seated_on_lower_support",
    )
    ctx.expect_gap(
        upper_flange,
        middle_stage,
        axis="z",
        min_gap=0.0,
        max_gap=0.001,
        positive_elem="upper_shell",
        negative_elem="middle_support",
        name="upper_flange_seated_on_middle_support",
    )

    ctx.expect_gap(
        middle_stage,
        lower_ring,
        axis="z",
        min_gap=0.02,
        positive_elem="middle_platter",
        negative_elem="lower_platter",
        name="middle_platter_clears_lower_platter",
    )
    ctx.expect_gap(
        upper_flange,
        middle_stage,
        axis="z",
        min_gap=0.018,
        positive_elem="upper_shell",
        negative_elem="middle_platter",
        name="upper_flange_clears_middle_platter",
    )
    ctx.expect_within(
        middle_stage,
        lower_ring,
        axes="xy",
        margin=0.0,
        inner_elem="middle_platter",
        outer_elem="lower_platter",
        name="middle_stage_within_lower_ring_footprint",
    )
    ctx.expect_within(
        upper_flange,
        middle_stage,
        axes="xy",
        margin=0.0,
        inner_elem="upper_shell",
        outer_elem="middle_platter",
        name="upper_flange_within_middle_stage_footprint",
    )

    with ctx.pose(
        {
            lower_joint: 1.1,
            middle_joint: -0.75,
            upper_joint: 0.9,
        }
    ):
        ctx.fail_if_parts_overlap_in_current_pose(name="offset_pose_has_no_overlaps")
        ctx.expect_gap(
            lower_ring,
            base,
            axis="z",
            min_gap=0.0,
            max_gap=0.001,
            name="lower_ring_stays_seated_in_offset_pose",
        )
        ctx.expect_gap(
            middle_stage,
            lower_ring,
            axis="z",
            min_gap=0.0,
            max_gap=0.001,
            positive_elem="middle_platter",
            negative_elem="lower_support",
            name="middle_stage_stays_seated_in_offset_pose",
        )
        ctx.expect_gap(
            upper_flange,
            middle_stage,
            axis="z",
            min_gap=0.0,
            max_gap=0.001,
            positive_elem="upper_shell",
            negative_elem="middle_support",
            name="upper_flange_stays_seated_in_offset_pose",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
