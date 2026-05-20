from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports. If the model needs mesh assets, create an
# `AssetContext` inside the editable section.
# >>> USER_CODE_START
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
)

ASSETS = AssetContext.from_script(__file__)

BASE_LENGTH = 0.14
BASE_WIDTH = 0.09
BASE_THICKNESS = 0.008

HINGE_BLOCK_LENGTH = 0.022
HINGE_BLOCK_WIDTH = 0.036
HINGE_BLOCK_HEIGHT = 0.032
HINGE_X = -0.048
HINGE_Z = BASE_THICKNESS + 0.020

LINK_LENGTH = 0.085
LINK_WIDTH = 0.028
LINK_THICKNESS = 0.014

BRACKET_LENGTH = 0.040
BRACKET_WIDTH = 0.032
BRACKET_DECK_THICKNESS = 0.004
BRACKET_LIP_THICKNESS = 0.004
BRACKET_LIP_HEIGHT = 0.018

FOLD_LIMIT = 0.70


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="fold_out_arm", assets=ASSETS)

    powder_black = model.material("powder_black", rgba=(0.16, 0.17, 0.19, 1.0))
    anodized_gray = model.material("anodized_gray", rgba=(0.58, 0.61, 0.64, 1.0))
    bracket_gray = model.material("bracket_gray", rgba=(0.72, 0.74, 0.77, 1.0))

    base = model.part("base")
    base.visual(
        Box((BASE_LENGTH, BASE_WIDTH, BASE_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS / 2.0)),
        material=powder_black,
        name="base_plate",
    )
    base.visual(
        Box((HINGE_BLOCK_LENGTH, HINGE_BLOCK_WIDTH, HINGE_BLOCK_HEIGHT)),
        origin=Origin(
            xyz=(
                HINGE_X - HINGE_BLOCK_LENGTH / 2.0,
                0.0,
                BASE_THICKNESS + HINGE_BLOCK_HEIGHT / 2.0,
            )
        ),
        material=powder_black,
        name="hinge_block",
    )
    base.inertial = Inertial.from_geometry(
        Box((BASE_LENGTH, BASE_WIDTH, BASE_THICKNESS)),
        mass=1.3,
        origin=Origin(xyz=(0.0, 0.0, BASE_THICKNESS / 2.0)),
    )

    for index in range(1, 5):
        link = model.part(f"link_{index}")
        link.visual(
            Box((LINK_LENGTH, LINK_WIDTH, LINK_THICKNESS)),
            origin=Origin(xyz=(LINK_LENGTH / 2.0, 0.0, 0.0)),
            material=anodized_gray,
            name="link_body",
        )
        link.inertial = Inertial.from_geometry(
            Box((LINK_LENGTH, LINK_WIDTH, LINK_THICKNESS)),
            mass=0.22,
            origin=Origin(xyz=(LINK_LENGTH / 2.0, 0.0, 0.0)),
        )

    bracket = model.part("platform_bracket")
    bracket.visual(
        Box((BRACKET_LENGTH, BRACKET_WIDTH, BRACKET_DECK_THICKNESS)),
        origin=Origin(xyz=(BRACKET_LENGTH / 2.0, 0.0, 0.0)),
        material=bracket_gray,
        name="platform_deck",
    )
    bracket.visual(
        Box((BRACKET_LIP_THICKNESS, BRACKET_WIDTH, BRACKET_LIP_HEIGHT)),
        origin=Origin(
            xyz=(
                BRACKET_LENGTH - BRACKET_LIP_THICKNESS / 2.0,
                0.0,
                BRACKET_LIP_HEIGHT / 2.0,
            )
        ),
        material=bracket_gray,
        name="platform_lip",
    )
    bracket.inertial = Inertial.from_geometry(
        Box((BRACKET_LENGTH, BRACKET_WIDTH, BRACKET_LIP_HEIGHT)),
        mass=0.12,
        origin=Origin(xyz=(BRACKET_LENGTH / 2.0, 0.0, BRACKET_LIP_HEIGHT / 2.0)),
    )

    revolute_limits = MotionLimits(
        effort=8.0,
        velocity=1.6,
        lower=0.0,
        upper=FOLD_LIMIT,
    )

    model.articulation(
        "base_to_link_1",
        ArticulationType.REVOLUTE,
        parent=base,
        child="link_1",
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=revolute_limits,
    )
    model.articulation(
        "link_1_to_link_2",
        ArticulationType.REVOLUTE,
        parent="link_1",
        child="link_2",
        origin=Origin(xyz=(LINK_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=revolute_limits,
    )
    model.articulation(
        "link_2_to_link_3",
        ArticulationType.REVOLUTE,
        parent="link_2",
        child="link_3",
        origin=Origin(xyz=(LINK_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=revolute_limits,
    )
    model.articulation(
        "link_3_to_link_4",
        ArticulationType.REVOLUTE,
        parent="link_3",
        child="link_4",
        origin=Origin(xyz=(LINK_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=revolute_limits,
    )
    model.articulation(
        "link_4_to_platform_bracket",
        ArticulationType.FIXED,
        parent="link_4",
        child=bracket,
        origin=Origin(xyz=(LINK_LENGTH, 0.0, 0.0)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    base = object_model.get_part("base")
    link_1 = object_model.get_part("link_1")
    link_2 = object_model.get_part("link_2")
    link_3 = object_model.get_part("link_3")
    link_4 = object_model.get_part("link_4")
    platform_bracket = object_model.get_part("platform_bracket")

    base_to_link_1 = object_model.get_articulation("base_to_link_1")
    link_1_to_link_2 = object_model.get_articulation("link_1_to_link_2")
    link_2_to_link_3 = object_model.get_articulation("link_2_to_link_3")
    link_3_to_link_4 = object_model.get_articulation("link_3_to_link_4")

    hinge_block = base.get_visual("hinge_block")
    platform_deck = platform_bracket.get_visual("platform_deck")

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
    ctx.fail_if_parts_overlap_in_current_pose()

    for part_name, part in (
        ("base", base),
        ("link_1", link_1),
        ("link_2", link_2),
        ("link_3", link_3),
        ("link_4", link_4),
        ("platform_bracket", platform_bracket),
    ):
        ctx.check(f"{part_name}_present", part is not None, f"Missing part: {part_name}")

    for joint_name, joint in (
        ("base_to_link_1", base_to_link_1),
        ("link_1_to_link_2", link_1_to_link_2),
        ("link_2_to_link_3", link_2_to_link_3),
        ("link_3_to_link_4", link_3_to_link_4),
    ):
        axis_ok = tuple(joint.axis) == (0.0, -1.0, 0.0)
        limits = joint.motion_limits
        limits_ok = (
            limits is not None
            and abs(limits.lower - 0.0) < 1e-9
            and abs(limits.upper - FOLD_LIMIT) < 1e-9
        )
        ctx.check(joint_name + "_axis", axis_ok, f"{joint_name} should fold about -Y in one plane")
        ctx.check(
            joint_name + "_limits",
            limits_ok,
            f"{joint_name} should have 0..{FOLD_LIMIT:.2f} rad folding limits",
        )

    ctx.expect_gap(
        link_1,
        base,
        axis="x",
        max_gap=0.0005,
        max_penetration=0.0,
        negative_elem=hinge_block,
        name="link_1_seats_on_base_hinge_block",
    )
    ctx.expect_overlap(
        link_1,
        base,
        axes="yz",
        min_overlap=0.012,
        elem_b=hinge_block,
        name="link_1_overlaps_base_hinge_face",
    )
    ctx.expect_gap(
        link_2,
        link_1,
        axis="x",
        max_gap=0.0005,
        max_penetration=0.0,
        name="link_2_seats_on_link_1",
    )
    ctx.expect_overlap(
        link_2,
        link_1,
        axes="yz",
        min_overlap=0.012,
        name="link_2_joint_face_overlap",
    )
    ctx.expect_gap(
        link_3,
        link_2,
        axis="x",
        max_gap=0.0005,
        max_penetration=1e-6,
        name="link_3_seats_on_link_2",
    )
    ctx.expect_overlap(
        link_3,
        link_2,
        axes="yz",
        min_overlap=0.012,
        name="link_3_joint_face_overlap",
    )
    ctx.expect_gap(
        link_4,
        link_3,
        axis="x",
        max_gap=0.0005,
        max_penetration=0.0,
        name="link_4_seats_on_link_3",
    )
    ctx.expect_overlap(
        link_4,
        link_3,
        axes="yz",
        min_overlap=0.012,
        name="link_4_joint_face_overlap",
    )
    ctx.expect_gap(
        platform_bracket,
        link_4,
        axis="x",
        max_gap=0.0005,
        max_penetration=0.0,
        positive_elem=platform_deck,
        name="platform_bracket_mounts_to_link_4",
    )
    ctx.expect_overlap(
        platform_bracket,
        link_4,
        axes="yz",
        min_overlap=0.003,
        elem_a=platform_deck,
        name="platform_bracket_joint_face_overlap",
    )

    ctx.expect_origin_gap(
        link_2,
        link_1,
        axis="x",
        min_gap=LINK_LENGTH - 1e-6,
        max_gap=LINK_LENGTH + 1e-6,
        name="joint_spacing_link_1_to_link_2",
    )
    ctx.expect_origin_gap(
        link_3,
        link_2,
        axis="x",
        min_gap=LINK_LENGTH - 1e-6,
        max_gap=LINK_LENGTH + 1e-6,
        name="joint_spacing_link_2_to_link_3",
    )
    ctx.expect_origin_gap(
        link_4,
        link_3,
        axis="x",
        min_gap=LINK_LENGTH - 1e-6,
        max_gap=LINK_LENGTH + 1e-6,
        name="joint_spacing_link_3_to_link_4",
    )

    with ctx.pose(
        {
            base_to_link_1: 0.0,
            link_1_to_link_2: 0.0,
            link_2_to_link_3: 0.0,
            link_3_to_link_4: 0.0,
        }
    ):
        extended_platform = ctx.part_world_position(platform_bracket)
        extended_link_1 = ctx.part_world_position(link_1)
        extended_link_4 = ctx.part_world_position(link_4)
        ctx.check(
            "extended_pose_positions_available",
            extended_platform is not None and extended_link_1 is not None and extended_link_4 is not None,
            "Expected valid world positions in the straight pose",
        )
        if (
            extended_platform is not None
            and extended_link_1 is not None
            and extended_link_4 is not None
        ):
            ctx.check(
                "extended_pose_reads_as_long_reach",
                extended_platform[0] > extended_link_4[0] > extended_link_1[0],
                "Straight pose should progress outward along +X",
            )
            ctx.check(
                "extended_pose_stays_level",
                abs(extended_link_4[2] - extended_link_1[2]) < 1e-6,
                "Straight pose should keep the boxed links in one level plane",
            )

    with ctx.pose(
        {
            base_to_link_1: 0.65,
            link_1_to_link_2: 0.65,
            link_2_to_link_3: 0.65,
            link_3_to_link_4: 0.65,
        }
    ):
        packed_platform = ctx.part_world_position(platform_bracket)
        packed_link_2 = ctx.part_world_position(link_2)
        packed_link_4 = ctx.part_world_position(link_4)
        ctx.check(
            "packed_pose_positions_available",
            packed_platform is not None and packed_link_2 is not None and packed_link_4 is not None,
            "Expected valid world positions in the folded pose",
        )
        if packed_platform is not None and extended_platform is not None:
            ctx.check(
                "packed_pose_draws_platform_back_toward_base",
                packed_platform[0] < 0.08 and packed_platform[0] < extended_platform[0] - 0.15,
                "Folded pose should pull the platform bracket back near the base",
            )
            ctx.check(
                "packed_pose_lifts_platform_clear_of_base",
                packed_platform[2] > HINGE_Z + 0.10,
                "Folded pose should arch the platform upward rather than letting it sag into the base",
            )
        if packed_link_2 is not None and packed_link_4 is not None:
            ctx.check(
                "packed_pose_keeps_series_order",
                packed_link_4[2] > packed_link_2[2],
                "Later links should continue upward through the folded arch",
            )

    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=24,
        ignore_adjacent=True,
        ignore_fixed=True,
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
