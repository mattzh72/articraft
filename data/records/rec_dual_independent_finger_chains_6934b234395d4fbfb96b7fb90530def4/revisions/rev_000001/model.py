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


PALM_WIDTH = 0.135
PALM_DEPTH = 0.046
PALM_HEIGHT = 0.040
PALM_SPLIT_WIDTH = 0.028
PALM_SPLIT_DEPTH = 0.018

FINGER_ROOT_X = 0.038
FINGER_ROOT_Z = 0.006

PROXIMAL_LENGTH = 0.058
MIDDLE_LENGTH = 0.044
DISTAL_LENGTH = 0.032

PROXIMAL_WIDTH = 0.022
MIDDLE_WIDTH = 0.018
DISTAL_WIDTH = 0.015

PROXIMAL_THICKNESS = 0.017
MIDDLE_THICKNESS = 0.015
DISTAL_THICKNESS = 0.013


def _make_palm_shape() -> cq.Workplane:
    palm = cq.Workplane("XY").box(PALM_WIDTH, PALM_DEPTH, PALM_HEIGHT)
    palm = (
        palm.faces(">Y")
        .workplane(centerOption="CenterOfMass")
        .center(0.0, PALM_HEIGHT * 0.05)
        .rect(PALM_SPLIT_WIDTH, PALM_HEIGHT * 0.70)
        .cutBlind(-PALM_SPLIT_DEPTH)
    )
    palm = palm.edges("|Z").fillet(0.0035)
    return palm


def _make_phalanx_shape(
    *,
    length: float,
    width: float,
    thickness: float,
    root_block_len: float,
    tip_chamfer: float,
) -> cq.Workplane:
    main_body = (
        cq.Workplane("XY")
        .box(width, length, thickness, centered=(True, False, True))
        .edges(">Y")
        .chamfer(tip_chamfer)
        .edges("|Z")
        .fillet(min(width, thickness) * 0.16)
    )

    root_block = (
        cq.Workplane("XY")
        .box(width * 1.10, root_block_len, thickness * 1.15, centered=(True, False, True))
        .edges("|Z")
        .fillet(min(width, thickness) * 0.18)
    )

    knuckle = (
        cq.Workplane("YZ")
        .center(root_block_len * 0.48, 0.0)
        .circle(thickness * 0.40)
        .extrude(width * 1.12, both=True)
    )

    return root_block.union(main_body).union(knuckle)


def _make_tip_pad_shape(*, length: float, width: float, thickness: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(width, length, thickness, centered=(True, False, True))
        .edges("|Z")
        .fillet(min(width, thickness) * 0.24)
        .edges(">Y")
        .fillet(min(width, thickness) * 0.22)
    )


def _center_of_aabb(aabb):
    if aabb is None:
        return None
    mins, maxs = aabb
    return tuple((lo + hi) * 0.5 for lo, hi in zip(mins, maxs))


def _point_delta(a, b) -> float | None:
    if a is None or b is None:
        return None
    return max(abs(ai - bi) for ai, bi in zip(a, b))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="split_palm_dual_finger_mechanism")

    model.material("palm_body", rgba=(0.23, 0.25, 0.29, 1.0))
    model.material("finger_body", rgba=(0.72, 0.75, 0.78, 1.0))
    model.material("tip_pad", rgba=(0.12, 0.12, 0.13, 1.0))

    palm = model.part("palm")
    palm.visual(
        mesh_from_cadquery(_make_palm_shape(), "palm_shell"),
        material="palm_body",
        name="palm_shell",
    )

    left_proximal = model.part("left_proximal")
    left_proximal.visual(
        mesh_from_cadquery(
            _make_phalanx_shape(
                length=PROXIMAL_LENGTH,
                width=PROXIMAL_WIDTH,
                thickness=PROXIMAL_THICKNESS,
                root_block_len=0.016,
                tip_chamfer=0.005,
            ),
            "left_proximal_shell",
        ),
        material="finger_body",
        name="finger_shell",
    )

    left_middle = model.part("left_middle")
    left_middle.visual(
        mesh_from_cadquery(
            _make_phalanx_shape(
                length=MIDDLE_LENGTH,
                width=MIDDLE_WIDTH,
                thickness=MIDDLE_THICKNESS,
                root_block_len=0.013,
                tip_chamfer=0.004,
            ),
            "left_middle_shell",
        ),
        material="finger_body",
        name="finger_shell",
    )

    left_distal = model.part("left_distal")
    left_distal.visual(
        mesh_from_cadquery(
            _make_phalanx_shape(
                length=DISTAL_LENGTH,
                width=DISTAL_WIDTH,
                thickness=DISTAL_THICKNESS,
                root_block_len=0.011,
                tip_chamfer=0.003,
            ),
            "left_distal_shell",
        ),
        material="finger_body",
        name="finger_shell",
    )
    left_distal.visual(
        mesh_from_cadquery(
            _make_tip_pad_shape(length=0.013, width=0.012, thickness=0.008),
            "left_tip_pad",
        ),
        origin=Origin(xyz=(0.0, DISTAL_LENGTH - 0.008, -0.001)),
        material="tip_pad",
        name="tip_pad",
    )

    right_proximal = model.part("right_proximal")
    right_proximal.visual(
        mesh_from_cadquery(
            _make_phalanx_shape(
                length=PROXIMAL_LENGTH,
                width=PROXIMAL_WIDTH,
                thickness=PROXIMAL_THICKNESS,
                root_block_len=0.016,
                tip_chamfer=0.005,
            ),
            "right_proximal_shell",
        ),
        material="finger_body",
        name="finger_shell",
    )

    right_middle = model.part("right_middle")
    right_middle.visual(
        mesh_from_cadquery(
            _make_phalanx_shape(
                length=MIDDLE_LENGTH,
                width=MIDDLE_WIDTH,
                thickness=MIDDLE_THICKNESS,
                root_block_len=0.013,
                tip_chamfer=0.004,
            ),
            "right_middle_shell",
        ),
        material="finger_body",
        name="finger_shell",
    )

    right_distal = model.part("right_distal")
    right_distal.visual(
        mesh_from_cadquery(
            _make_phalanx_shape(
                length=DISTAL_LENGTH,
                width=DISTAL_WIDTH,
                thickness=DISTAL_THICKNESS,
                root_block_len=0.011,
                tip_chamfer=0.003,
            ),
            "right_distal_shell",
        ),
        material="finger_body",
        name="finger_shell",
    )
    right_distal.visual(
        mesh_from_cadquery(
            _make_tip_pad_shape(length=0.013, width=0.012, thickness=0.008),
            "right_tip_pad",
        ),
        origin=Origin(xyz=(0.0, DISTAL_LENGTH - 0.008, -0.001)),
        material="tip_pad",
        name="tip_pad",
    )

    palm_to_left_proximal = model.articulation(
        "palm_to_left_proximal",
        ArticulationType.REVOLUTE,
        parent=palm,
        child=left_proximal,
        origin=Origin(xyz=(-FINGER_ROOT_X, PALM_DEPTH * 0.5, FINGER_ROOT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=3.0, lower=0.0, upper=1.15),
    )
    left_proximal_to_middle = model.articulation(
        "left_proximal_to_middle",
        ArticulationType.REVOLUTE,
        parent=left_proximal,
        child=left_middle,
        origin=Origin(xyz=(0.0, PROXIMAL_LENGTH, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=3.5, lower=0.0, upper=1.35),
    )
    model.articulation(
        "left_middle_to_distal",
        ArticulationType.REVOLUTE,
        parent=left_middle,
        child=left_distal,
        origin=Origin(xyz=(0.0, MIDDLE_LENGTH, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=4.0, lower=0.0, upper=1.20),
    )

    palm_to_right_proximal = model.articulation(
        "palm_to_right_proximal",
        ArticulationType.REVOLUTE,
        parent=palm,
        child=right_proximal,
        origin=Origin(xyz=(FINGER_ROOT_X, PALM_DEPTH * 0.5, FINGER_ROOT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=3.0, lower=0.0, upper=1.15),
    )
    right_proximal_to_middle = model.articulation(
        "right_proximal_to_middle",
        ArticulationType.REVOLUTE,
        parent=right_proximal,
        child=right_middle,
        origin=Origin(xyz=(0.0, PROXIMAL_LENGTH, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=3.5, lower=0.0, upper=1.35),
    )
    model.articulation(
        "right_middle_to_distal",
        ArticulationType.REVOLUTE,
        parent=right_middle,
        child=right_distal,
        origin=Origin(xyz=(0.0, MIDDLE_LENGTH, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=4.0, lower=0.0, upper=1.20),
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

    palm = object_model.get_part("palm")
    left_proximal = object_model.get_part("left_proximal")
    left_middle = object_model.get_part("left_middle")
    left_distal = object_model.get_part("left_distal")
    right_proximal = object_model.get_part("right_proximal")
    right_middle = object_model.get_part("right_middle")
    right_distal = object_model.get_part("right_distal")

    palm_to_left_proximal = object_model.get_articulation("palm_to_left_proximal")
    left_proximal_to_middle = object_model.get_articulation("left_proximal_to_middle")
    left_middle_to_distal = object_model.get_articulation("left_middle_to_distal")
    palm_to_right_proximal = object_model.get_articulation("palm_to_right_proximal")
    right_proximal_to_middle = object_model.get_articulation("right_proximal_to_middle")
    right_middle_to_distal = object_model.get_articulation("right_middle_to_distal")

    left_tip_pad = left_distal.get_visual("tip_pad")
    right_tip_pad = right_distal.get_visual("tip_pad")

    all_axes_match = all(
        articulation.axis == (1.0, 0.0, 0.0)
        for articulation in (
            palm_to_left_proximal,
            left_proximal_to_middle,
            left_middle_to_distal,
            palm_to_right_proximal,
            right_proximal_to_middle,
            right_middle_to_distal,
        )
    )
    ctx.check(
        "all finger joints bend in the same plane",
        all_axes_match,
        details="Expected every revolute joint axis to be +X for planar YZ curling.",
    )

    ctx.expect_origin_distance(
        left_proximal,
        right_proximal,
        axes="x",
        min_dist=0.06,
        name="left and right root pivots are laterally split",
    )

    with ctx.pose(
        {
            palm_to_left_proximal: 0.0,
            left_proximal_to_middle: 0.0,
            left_middle_to_distal: 0.0,
            palm_to_right_proximal: 0.0,
            right_proximal_to_middle: 0.0,
            right_middle_to_distal: 0.0,
        }
    ):
        ctx.expect_gap(
            left_proximal,
            palm,
            axis="y",
            min_gap=0.0,
            max_gap=0.001,
            name="left proximal root seats on the palm front face",
        )
        ctx.expect_overlap(
            left_proximal,
            palm,
            axes="xz",
            min_overlap=0.012,
            name="left proximal root overlaps the left palm mount",
        )
        ctx.expect_gap(
            right_proximal,
            palm,
            axis="y",
            min_gap=0.0,
            max_gap=0.001,
            name="right proximal root seats on the palm front face",
        )
        ctx.expect_overlap(
            right_proximal,
            palm,
            axes="xz",
            min_overlap=0.012,
            name="right proximal root overlaps the right palm mount",
        )

        for positive_link, negative_link, check_name in (
            (left_middle, left_proximal, "left middle seats on the left proximal knuckle"),
            (left_distal, left_middle, "left distal seats on the left middle knuckle"),
            (right_middle, right_proximal, "right middle seats on the right proximal knuckle"),
            (right_distal, right_middle, "right distal seats on the right middle knuckle"),
        ):
            ctx.expect_gap(
                positive_link,
                negative_link,
                axis="y",
                min_gap=0.0,
                max_gap=0.001,
                name=check_name,
            )
            ctx.expect_overlap(
                positive_link,
                negative_link,
                axes="xz",
                min_overlap=0.010,
                name=f"{check_name} with shared hinge footprint",
            )

        rest_left_tip_aabb = ctx.part_element_world_aabb(left_distal, elem=left_tip_pad)
        rest_right_tip_aabb = ctx.part_element_world_aabb(right_distal, elem=right_tip_pad)
        rest_left_tip_center = _center_of_aabb(rest_left_tip_aabb)
        rest_right_tip_center = _center_of_aabb(rest_right_tip_aabb)

    with ctx.pose(
        {
            palm_to_left_proximal: 0.55,
            left_proximal_to_middle: 0.85,
            left_middle_to_distal: 0.65,
        }
    ):
        flex_left_tip_aabb = ctx.part_element_world_aabb(left_distal, elem=left_tip_pad)
        right_tip_during_left_aabb = ctx.part_element_world_aabb(right_distal, elem=right_tip_pad)
        flex_left_tip_center = _center_of_aabb(flex_left_tip_aabb)
        right_tip_during_left_center = _center_of_aabb(right_tip_during_left_aabb)

    ctx.check(
        "left finger curls upward",
        rest_left_tip_aabb is not None
        and flex_left_tip_aabb is not None
        and flex_left_tip_aabb[1][2] > rest_left_tip_aabb[1][2] + 0.018,
        details=f"rest={rest_left_tip_aabb}, flexed={flex_left_tip_aabb}",
    )
    right_motion_from_left = _point_delta(rest_right_tip_center, right_tip_during_left_center)
    ctx.check(
        "right finger stays fixed while only the left finger moves",
        right_motion_from_left is not None and right_motion_from_left < 1e-6,
        details=(
            f"rest_right_tip={rest_right_tip_center}, "
            f"right_tip_during_left_pose={right_tip_during_left_center}, "
            f"delta={right_motion_from_left}"
        ),
    )

    with ctx.pose(
        {
            palm_to_right_proximal: 0.52,
            right_proximal_to_middle: 0.82,
            right_middle_to_distal: 0.62,
        }
    ):
        flex_right_tip_aabb = ctx.part_element_world_aabb(right_distal, elem=right_tip_pad)
        left_tip_during_right_aabb = ctx.part_element_world_aabb(left_distal, elem=left_tip_pad)
        flex_right_tip_center = _center_of_aabb(flex_right_tip_aabb)
        left_tip_during_right_center = _center_of_aabb(left_tip_during_right_aabb)

    ctx.check(
        "right finger curls upward",
        rest_right_tip_aabb is not None
        and flex_right_tip_aabb is not None
        and flex_right_tip_aabb[1][2] > rest_right_tip_aabb[1][2] + 0.018,
        details=f"rest={rest_right_tip_aabb}, flexed={flex_right_tip_aabb}",
    )
    left_motion_from_right = _point_delta(rest_left_tip_center, left_tip_during_right_center)
    ctx.check(
        "left finger stays fixed while only the right finger moves",
        left_motion_from_right is not None and left_motion_from_right < 1e-6,
        details=(
            f"rest_left_tip={rest_left_tip_center}, "
            f"left_tip_during_right_pose={left_tip_during_right_center}, "
            f"delta={left_motion_from_right}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
