from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


TONGUE_WIDTH = 0.014
EAR_THICKNESS = 0.010
FORK_WIDTH = TONGUE_WIDTH + 2.0 * EAR_THICKNESS
END_RADIUS = 0.012
LINK_BODY_WIDTH_X = 0.016
HINGE_Y_OFFSET = TONGUE_WIDTH / 2.0 + EAR_THICKNESS / 2.0

LINK_UPPER_STUB = 0.022
LINK_FORK_BRIDGE = 0.014
LINK_EAR_HEIGHT = 0.026

TOP_BRACKET_DROP = 0.030
TOP_PLATE_LENGTH = 0.100
TOP_PLATE_WIDTH = FORK_WIDTH + 0.010
TOP_PLATE_THICKNESS = 0.008

LINK_ONE_LENGTH = 0.115
LINK_TWO_LENGTH = 0.105
LINK_THREE_LENGTH = 0.095

PLATFORM_DROP = 0.055
PLATFORM_LENGTH = 0.060
PLATFORM_WIDTH = 0.038
PLATFORM_THICKNESS = 0.006
PLATFORM_LIP_THICKNESS = 0.006
PLATFORM_LIP_HEIGHT = 0.016
PLATFORM_GUSSET_HEIGHT = 0.018


def _y_cylinder_origin(xyz: tuple[float, float, float]) -> Origin:
    return Origin(xyz=xyz, rpy=(pi / 2.0, 0.0, 0.0))


def _add_link_visuals(part, *, length: float, material: str, prefix: str) -> None:
    spine_length = length - LINK_UPPER_STUB - LINK_FORK_BRIDGE - LINK_EAR_HEIGHT

    part.visual(
        Cylinder(radius=END_RADIUS, length=TONGUE_WIDTH),
        origin=_y_cylinder_origin((0.0, 0.0, 0.0)),
        material=material,
        name=f"{prefix}_proximal_barrel",
    )
    part.visual(
        Box((LINK_BODY_WIDTH_X, TONGUE_WIDTH, LINK_UPPER_STUB)),
        origin=Origin(xyz=(0.0, 0.0, -LINK_UPPER_STUB / 2.0)),
        material=material,
        name=f"{prefix}_upper_stub",
    )
    part.visual(
        Box((LINK_BODY_WIDTH_X, TONGUE_WIDTH, spine_length)),
        origin=Origin(xyz=(0.0, 0.0, -LINK_UPPER_STUB - spine_length / 2.0)),
        material=material,
        name=f"{prefix}_spine",
    )
    part.visual(
        Box((LINK_BODY_WIDTH_X, FORK_WIDTH, LINK_FORK_BRIDGE)),
        origin=Origin(
            xyz=(0.0, 0.0, -length + LINK_EAR_HEIGHT + LINK_FORK_BRIDGE / 2.0)
        ),
        material=material,
        name=f"{prefix}_fork_bridge",
    )
    for sign, side in ((-1.0, "left"), (1.0, "right")):
        part.visual(
            Box((LINK_BODY_WIDTH_X, EAR_THICKNESS, LINK_EAR_HEIGHT)),
            origin=Origin(
                xyz=(
                    0.0,
                    sign * HINGE_Y_OFFSET,
                    -length + LINK_EAR_HEIGHT / 2.0,
                )
            ),
            material=material,
            name=f"{prefix}_{side}_ear",
        )
        part.visual(
            Cylinder(radius=END_RADIUS, length=EAR_THICKNESS),
            origin=_y_cylinder_origin((0.0, sign * HINGE_Y_OFFSET, -length)),
            material=material,
            name=f"{prefix}_{side}_distal_barrel",
        )


def _add_top_bracket_visuals(part, *, material: str) -> None:
    part.visual(
        Box((TOP_PLATE_LENGTH, TOP_PLATE_WIDTH, TOP_PLATE_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, TOP_PLATE_THICKNESS / 2.0)),
        material=material,
        name="top_plate",
    )
    part.visual(
        Box((0.040, TOP_PLATE_WIDTH, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, -0.006)),
        material=material,
        name="mounting_pad",
    )
    for sign, side in ((-1.0, "left"), (1.0, "right")):
        part.visual(
            Box((0.022, EAR_THICKNESS, TOP_BRACKET_DROP)),
            origin=Origin(xyz=(0.0, sign * HINGE_Y_OFFSET, -TOP_BRACKET_DROP / 2.0)),
            material=material,
            name=f"{side}_hanger_ear",
        )
        part.visual(
            Cylinder(radius=END_RADIUS, length=EAR_THICKNESS),
            origin=_y_cylinder_origin((0.0, sign * HINGE_Y_OFFSET, -TOP_BRACKET_DROP)),
            material=material,
            name=f"{side}_hinge_barrel",
        )


def _add_platform_bracket_visuals(part, *, material: str) -> None:
    stem_length = PLATFORM_DROP - LINK_UPPER_STUB

    part.visual(
        Cylinder(radius=END_RADIUS, length=TONGUE_WIDTH),
        origin=_y_cylinder_origin((0.0, 0.0, 0.0)),
        material=material,
        name="platform_pivot_barrel",
    )
    part.visual(
        Box((LINK_BODY_WIDTH_X, TONGUE_WIDTH, LINK_UPPER_STUB)),
        origin=Origin(xyz=(0.0, 0.0, -LINK_UPPER_STUB / 2.0)),
        material=material,
        name="platform_upper_stub",
    )
    part.visual(
        Box((LINK_BODY_WIDTH_X, TONGUE_WIDTH, stem_length)),
        origin=Origin(xyz=(0.0, 0.0, -LINK_UPPER_STUB - stem_length / 2.0)),
        material=material,
        name="platform_stem",
    )
    part.visual(
        Box((0.020, PLATFORM_WIDTH, PLATFORM_GUSSET_HEIGHT)),
        origin=Origin(
            xyz=(0.004, 0.0, -PLATFORM_DROP + PLATFORM_GUSSET_HEIGHT / 2.0)
        ),
        material=material,
        name="platform_gusset",
    )
    part.visual(
        Box((PLATFORM_LENGTH, PLATFORM_WIDTH, PLATFORM_THICKNESS)),
        origin=Origin(
            xyz=(0.022, 0.0, -PLATFORM_DROP - PLATFORM_THICKNESS / 2.0)
        ),
        material=material,
        name="platform_deck",
    )
    part.visual(
        Box((PLATFORM_LIP_THICKNESS, PLATFORM_WIDTH, PLATFORM_LIP_HEIGHT)),
        origin=Origin(
            xyz=(
                0.022 + PLATFORM_LENGTH / 2.0 - PLATFORM_LIP_THICKNESS / 2.0,
                0.0,
                -PLATFORM_DROP - PLATFORM_THICKNESS + PLATFORM_LIP_HEIGHT / 2.0,
            )
        ),
        material=material,
        name="platform_lip",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="underslung_foldout_arm")

    model.material("powder_coat", rgba=(0.20, 0.22, 0.24, 1.0))
    model.material("brushed_aluminum", rgba=(0.70, 0.73, 0.76, 1.0))
    model.material("anodized_black", rgba=(0.12, 0.13, 0.14, 1.0))

    top_bracket = model.part("top_bracket")
    _add_top_bracket_visuals(top_bracket, material="powder_coat")

    link_one = model.part("link_one")
    _add_link_visuals(
        link_one,
        length=LINK_ONE_LENGTH,
        material="brushed_aluminum",
        prefix="link_one",
    )

    link_two = model.part("link_two")
    _add_link_visuals(
        link_two,
        length=LINK_TWO_LENGTH,
        material="brushed_aluminum",
        prefix="link_two",
    )

    link_three = model.part("link_three")
    _add_link_visuals(
        link_three,
        length=LINK_THREE_LENGTH,
        material="brushed_aluminum",
        prefix="link_three",
    )

    platform_bracket = model.part("platform_bracket")
    _add_platform_bracket_visuals(platform_bracket, material="anodized_black")

    common_limits = MotionLimits(lower=0.0, upper=1.30, effort=18.0, velocity=1.4)

    model.articulation(
        "top_to_link_one",
        ArticulationType.REVOLUTE,
        parent=top_bracket,
        child=link_one,
        origin=Origin(xyz=(0.0, 0.0, -TOP_BRACKET_DROP)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=common_limits,
    )
    model.articulation(
        "link_one_to_link_two",
        ArticulationType.REVOLUTE,
        parent=link_one,
        child=link_two,
        origin=Origin(xyz=(0.0, 0.0, -LINK_ONE_LENGTH)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.30, effort=14.0, velocity=1.6),
    )
    model.articulation(
        "link_two_to_link_three",
        ArticulationType.REVOLUTE,
        parent=link_two,
        child=link_three,
        origin=Origin(xyz=(0.0, 0.0, -LINK_TWO_LENGTH)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.30, effort=12.0, velocity=1.6),
    )
    model.articulation(
        "link_three_to_platform_bracket",
        ArticulationType.REVOLUTE,
        parent=link_three,
        child=platform_bracket,
        origin=Origin(xyz=(0.0, 0.0, -LINK_THREE_LENGTH)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.15, effort=8.0, velocity=1.8),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    top_bracket = object_model.get_part("top_bracket")
    link_one = object_model.get_part("link_one")
    link_two = object_model.get_part("link_two")
    link_three = object_model.get_part("link_three")
    platform_bracket = object_model.get_part("platform_bracket")

    top_to_link_one = object_model.get_articulation("top_to_link_one")
    link_one_to_link_two = object_model.get_articulation("link_one_to_link_two")
    link_two_to_link_three = object_model.get_articulation("link_two_to_link_three")
    link_three_to_platform = object_model.get_articulation("link_three_to_platform_bracket")

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
        "all hinges are revolute and parallel",
        all(
            joint.articulation_type == ArticulationType.REVOLUTE and joint.axis == (0.0, -1.0, 0.0)
            for joint in (
                top_to_link_one,
                link_one_to_link_two,
                link_two_to_link_three,
                link_three_to_platform,
            )
        ),
        details="Expected four revolute joints sharing the same -Y hinge axis.",
    )

    ctx.expect_contact(top_bracket, link_one, name="top bracket carries first link")
    ctx.expect_contact(link_one, link_two, name="link one supports link two")
    ctx.expect_contact(link_two, link_three, name="link two supports link three")
    ctx.expect_contact(link_three, platform_bracket, name="link three carries platform bracket")
    ctx.expect_origin_gap(
        top_bracket,
        link_one,
        axis="z",
        min_gap=0.020,
        name="first link hangs below top bracket",
    )

    with ctx.pose(
        {
            top_to_link_one: 0.95,
            link_one_to_link_two: 0.65,
            link_two_to_link_three: 0.70,
            link_three_to_platform: 0.30,
        }
    ):
        ctx.expect_origin_gap(
            platform_bracket,
            top_bracket,
            axis="x",
            min_gap=0.160,
            name="folded out pose reaches forward",
        )
        ctx.fail_if_parts_overlap_in_current_pose(name="opened_pose_clearance")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
