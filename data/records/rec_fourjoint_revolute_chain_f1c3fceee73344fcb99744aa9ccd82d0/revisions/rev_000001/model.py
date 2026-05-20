from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import isclose, pi

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


PLATE_THICK = 0.008
JOINT_GAP = 0.024
CHAIN_WIDTH = JOINT_GAP + 2.0 * PLATE_THICK
SIDE_Y = JOINT_GAP / 2.0 + PLATE_THICK / 2.0

LINK_1_LENGTH = 0.180
LINK_2_LENGTH = 0.170
LINK_3_LENGTH = 0.150
END_TAB_LENGTH = 0.058

LINK_1_HEIGHT = 0.038
LINK_2_HEIGHT = 0.035
LINK_3_HEIGHT = 0.032


def _add_ladder_link(
    part,
    *,
    length: float,
    height: float,
    material,
    prefix: str,
) -> None:
    part.visual(
        Box((length, PLATE_THICK, height)),
        origin=Origin(xyz=(length / 2.0, SIDE_Y, 0.0)),
        material=material,
        name=f"{prefix}_left_plate",
    )
    part.visual(
        Box((length, PLATE_THICK, height)),
        origin=Origin(xyz=(length / 2.0, -SIDE_Y, 0.0)),
        material=material,
        name=f"{prefix}_right_plate",
    )
    part.visual(
        Box((0.012, CHAIN_WIDTH, height * 0.60)),
        origin=Origin(xyz=(0.006, 0.0, 0.0)),
        material=material,
        name=f"{prefix}_prox_bridge",
    )
    part.visual(
        Box((0.010, CHAIN_WIDTH, height * 0.44)),
        origin=Origin(xyz=(length * 0.53, 0.0, 0.0)),
        material=material,
        name=f"{prefix}_mid_bridge",
    )
    part.visual(
        Box((0.012, CHAIN_WIDTH, height * 0.60)),
        origin=Origin(xyz=(length - 0.006, 0.0, 0.0)),
        material=material,
        name=f"{prefix}_distal_bridge",
    )
    part.visual(
        Box((length - 0.036, 0.012, height * 0.26)),
        origin=Origin(xyz=(length / 2.0, 0.0, 0.0)),
        material=material,
        name=f"{prefix}_spine",
    )


def _add_root_bracket(part, *, material) -> None:
    part.visual(
        Box((0.016, 0.076, 0.092)),
        origin=Origin(xyz=(-0.052, 0.0, 0.0)),
        material=material,
        name="back_plate",
    )
    part.visual(
        Box((0.064, CHAIN_WIDTH, 0.030)),
        origin=Origin(xyz=(-0.032, 0.0, 0.0)),
        material=material,
        name="center_saddle",
    )
    part.visual(
        Box((0.050, 0.076, 0.012)),
        origin=Origin(xyz=(-0.031, 0.0, 0.026)),
        material=material,
        name="upper_flange",
    )
    part.visual(
        Box((0.050, 0.076, 0.012)),
        origin=Origin(xyz=(-0.031, 0.0, -0.026)),
        material=material,
        name="lower_flange",
    )


def _add_end_tab(part, *, material) -> None:
    part.visual(
        Box((0.010, CHAIN_WIDTH, 0.024)),
        origin=Origin(xyz=(0.005, 0.0, 0.0)),
        material=material,
        name="tab_root",
    )
    part.visual(
        Box((0.036, 0.020, 0.022)),
        origin=Origin(xyz=(0.028, 0.0, 0.0)),
        material=material,
        name="tab_body",
    )
    part.visual(
        Cylinder(radius=0.010, length=0.020),
        origin=Origin(xyz=(END_TAB_LENGTH - 0.010, 0.0, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=material,
        name="tab_nose",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="ladder_frame_revolute_chain")

    bracket_finish = model.material("bracket_finish", rgba=(0.23, 0.25, 0.28, 1.0))
    link_light = model.material("link_light", rgba=(0.70, 0.72, 0.75, 1.0))
    link_mid = model.material("link_mid", rgba=(0.63, 0.66, 0.70, 1.0))
    end_finish = model.material("end_finish", rgba=(0.38, 0.40, 0.43, 1.0))

    root_bracket = model.part("root_bracket")
    _add_root_bracket(root_bracket, material=bracket_finish)

    link_1 = model.part("link_1")
    _add_ladder_link(
        link_1,
        length=LINK_1_LENGTH,
        height=LINK_1_HEIGHT,
        material=link_light,
        prefix="link_1",
    )

    link_2 = model.part("link_2")
    _add_ladder_link(
        link_2,
        length=LINK_2_LENGTH,
        height=LINK_2_HEIGHT,
        material=link_mid,
        prefix="link_2",
    )

    link_3 = model.part("link_3")
    _add_ladder_link(
        link_3,
        length=LINK_3_LENGTH,
        height=LINK_3_HEIGHT,
        material=link_light,
        prefix="link_3",
    )

    end_tab = model.part("end_tab")
    _add_end_tab(end_tab, material=end_finish)

    model.articulation(
        "root_to_link_1",
        ArticulationType.REVOLUTE,
        parent=root_bracket,
        child=link_1,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-1.0, upper=1.2, effort=18.0, velocity=1.5),
    )
    model.articulation(
        "link_1_to_link_2",
        ArticulationType.REVOLUTE,
        parent=link_1,
        child=link_2,
        origin=Origin(xyz=(LINK_1_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-1.4, upper=1.4, effort=14.0, velocity=1.8),
    )
    model.articulation(
        "link_2_to_link_3",
        ArticulationType.REVOLUTE,
        parent=link_2,
        child=link_3,
        origin=Origin(xyz=(LINK_2_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-1.4, upper=1.4, effort=14.0, velocity=1.8),
    )
    model.articulation(
        "link_3_to_end_tab",
        ArticulationType.REVOLUTE,
        parent=link_3,
        child=end_tab,
        origin=Origin(xyz=(LINK_3_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=-1.8, upper=1.8, effort=10.0, velocity=2.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    root_bracket = object_model.get_part("root_bracket")
    link_1 = object_model.get_part("link_1")
    link_2 = object_model.get_part("link_2")
    link_3 = object_model.get_part("link_3")
    end_tab = object_model.get_part("end_tab")

    root_to_link_1 = object_model.get_articulation("root_to_link_1")
    link_1_to_link_2 = object_model.get_articulation("link_1_to_link_2")
    link_2_to_link_3 = object_model.get_articulation("link_2_to_link_3")
    link_3_to_end_tab = object_model.get_articulation("link_3_to_end_tab")

    for joint_name, joint in (
        ("root_to_link_1", root_to_link_1),
        ("link_1_to_link_2", link_1_to_link_2),
        ("link_2_to_link_3", link_2_to_link_3),
        ("link_3_to_end_tab", link_3_to_end_tab),
    ):
        ctx.check(
            f"{joint_name}_axis_parallel",
            tuple(joint.axis) == (0.0, -1.0, 0.0),
            f"{joint_name} axis={joint.axis}",
        )

    ctx.expect_contact(root_bracket, link_1, contact_tol=1e-6, name="root_joint_contact")
    ctx.expect_contact(link_1, link_2, contact_tol=1e-6, name="joint_2_contact")
    ctx.expect_contact(link_2, link_3, contact_tol=1e-6, name="joint_3_contact")
    ctx.expect_contact(link_3, end_tab, contact_tol=1e-6, name="joint_4_contact")

    ctx.expect_origin_gap(
        link_2,
        link_1,
        axis="x",
        min_gap=LINK_1_LENGTH - 0.002,
        max_gap=LINK_1_LENGTH + 0.002,
        name="link_1_pitch_length",
    )
    ctx.expect_origin_gap(
        link_3,
        link_2,
        axis="x",
        min_gap=LINK_2_LENGTH - 0.002,
        max_gap=LINK_2_LENGTH + 0.002,
        name="link_2_pitch_length",
    )
    ctx.expect_origin_gap(
        end_tab,
        link_3,
        axis="x",
        min_gap=LINK_3_LENGTH - 0.002,
        max_gap=LINK_3_LENGTH + 0.002,
        name="link_3_pitch_length",
    )

    rest_tip = ctx.part_world_position(end_tab)
    with ctx.pose(
        {
            root_to_link_1: 0.55,
            link_1_to_link_2: -0.70,
            link_2_to_link_3: 0.80,
            link_3_to_end_tab: -0.45,
        }
    ):
        posed_tip = ctx.part_world_position(end_tab)
        ctx.check(
            "tip_stays_in_joint_plane",
            posed_tip is not None and isclose(posed_tip[1], 0.0, abs_tol=1e-6),
            f"posed tip origin={posed_tip}",
        )
        ctx.check(
            "tip_lifts_in_bent_pose",
            rest_tip is not None and posed_tip is not None and posed_tip[2] > rest_tip[2] + 0.06,
            f"rest tip={rest_tip}, posed tip={posed_tip}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
