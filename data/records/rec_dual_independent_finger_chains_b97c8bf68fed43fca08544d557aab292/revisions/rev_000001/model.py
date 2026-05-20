from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import isclose

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


PALM_LENGTH = 0.170
PALM_DEPTH = 0.040
PALM_HEIGHT = 0.024
ROOT_SUPPORT_HEIGHT = 0.018

FINGER_SPACING = 0.062
ROOT_Y = PALM_DEPTH / 2.0 + 0.002

HINGE_RADIUS = 0.0055
ROOT_BARREL_LENGTH = 0.018
CHAIN_BARREL_LENGTH = 0.014

PROXIMAL_LENGTH = 0.055
MIDDLE_LENGTH = 0.043
DISTAL_LENGTH = 0.031

PROXIMAL_WIDTH = 0.0135
MIDDLE_WIDTH = 0.0125
DISTAL_WIDTH = 0.0115

PROXIMAL_THICKNESS = 0.0115
MIDDLE_THICKNESS = 0.0105
DISTAL_THICKNESS = 0.0095

JOINT_AXIS = (-1.0, 0.0, 0.0)
ROOT_Z = PALM_HEIGHT + HINGE_RADIUS + 0.001


def _cylinder_along_x(*, radius: float, length: float) -> cq.Workplane:
    return cq.Workplane("YZ").circle(radius).extrude(length / 2.0, both=True)


def _forward_half_cylinder_along_x(*, radius: float, length: float) -> cq.Workplane:
    full = _cylinder_along_x(radius=radius, length=length)
    cutter = (
        cq.Workplane("XY")
        .box(length + 0.010, 2.0 * radius + 0.010, 2.0 * radius + 0.010)
        .translate((0.0, -(radius + 0.005), 0.0))
    )
    return full.cut(cutter)


def _make_palm_shape() -> cq.Workplane:
    palm = cq.Workplane("XY").box(
        PALM_LENGTH,
        PALM_DEPTH,
        PALM_HEIGHT,
        centered=(True, True, False),
    )

    for root_x in (-FINGER_SPACING / 2.0, FINGER_SPACING / 2.0):
        root_block = (
            cq.Workplane("XY")
            .box(
                ROOT_BARREL_LENGTH + 0.006,
                0.012,
                ROOT_SUPPORT_HEIGHT,
                centered=(True, True, False),
            )
            .translate((root_x, ROOT_Y - 0.006, PALM_HEIGHT))
        )
        palm = palm.union(root_block)

    return palm


def _make_link_shape(
    *,
    length: float,
    body_width: float,
    body_thickness: float,
    barrel_length: float,
    child_barrel_length: float | None,
    fingertip: bool = False,
) -> cq.Workplane:
    barrel = _forward_half_cylinder_along_x(radius=HINGE_RADIUS, length=barrel_length)
    body = cq.Workplane("XY").box(
        body_width,
        length,
        body_thickness,
        centered=(True, False, True),
    )
    shoulder = (
        cq.Workplane("XY")
        .box(
            max(body_width + 0.004, barrel_length - 0.001),
            0.012,
            body_thickness + 0.002,
            centered=(True, False, True),
        )
        .translate((0.0, 0.0, 0.0))
    )

    link = barrel.union(body).union(shoulder)

    if fingertip:
        pad = (
            cq.Workplane("XY")
            .box(
                body_width + 0.005,
                0.017,
                body_thickness + 0.0015,
                centered=(True, False, True),
            )
            .translate((0.0, length - 0.006, 0.0))
        )
        nose = (
            cq.Workplane("XZ")
            .circle((body_thickness + 0.0015) / 2.0)
            .extrude((body_width + 0.004) / 2.0, both=True)
            .translate((0.0, length + 0.007, 0.0))
        )
        return link.union(pad).union(nose)

    knuckle_block = (
        cq.Workplane("XY")
        .box(
            max(child_barrel_length + 0.004, body_width + 0.004),
            0.012,
            body_thickness + 0.004,
            centered=(True, False, True),
        )
        .translate((0.0, length - 0.012, 0.0))
    )
    return link.union(knuckle_block)


def _add_mesh_part(
    model: ArticulatedObject,
    *,
    name: str,
    shape: cq.Workplane,
    material: str,
    inertial_box: tuple[float, float, float],
    inertial_origin: Origin,
):
    part = model.part(name)
    part.visual(mesh_from_cadquery(shape, name), material=material, name=f"{name}_shell")
    part.inertial = Inertial.from_geometry(
        Box(inertial_box),
        mass=0.08,
        origin=inertial_origin,
    )
    return part


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="forked_palm_two_finger_chain")

    model.material("palm_finish", rgba=(0.20, 0.23, 0.27, 1.0))
    model.material("finger_finish", rgba=(0.75, 0.78, 0.82, 1.0))

    palm = model.part("palm")
    palm.visual(
        mesh_from_cadquery(_make_palm_shape(), "palm"),
        material="palm_finish",
        name="palm_shell",
    )
    palm.inertial = Inertial.from_geometry(
        Box((PALM_LENGTH, PALM_DEPTH, PALM_HEIGHT + ROOT_SUPPORT_HEIGHT)),
        mass=0.55,
        origin=Origin(xyz=(0.0, 0.0, (PALM_HEIGHT + ROOT_SUPPORT_HEIGHT) / 2.0)),
    )

    left_proximal = _add_mesh_part(
        model,
        name="left_proximal",
        shape=_make_link_shape(
            length=PROXIMAL_LENGTH,
            body_width=PROXIMAL_WIDTH,
            body_thickness=PROXIMAL_THICKNESS,
            barrel_length=ROOT_BARREL_LENGTH,
            child_barrel_length=CHAIN_BARREL_LENGTH,
        ),
        material="finger_finish",
        inertial_box=(PROXIMAL_WIDTH + 0.004, PROXIMAL_LENGTH, PROXIMAL_THICKNESS + 0.004),
        inertial_origin=Origin(xyz=(0.0, PROXIMAL_LENGTH / 2.0, 0.0)),
    )
    left_middle = _add_mesh_part(
        model,
        name="left_middle",
        shape=_make_link_shape(
            length=MIDDLE_LENGTH,
            body_width=MIDDLE_WIDTH,
            body_thickness=MIDDLE_THICKNESS,
            barrel_length=CHAIN_BARREL_LENGTH,
            child_barrel_length=CHAIN_BARREL_LENGTH,
        ),
        material="finger_finish",
        inertial_box=(MIDDLE_WIDTH + 0.004, MIDDLE_LENGTH, MIDDLE_THICKNESS + 0.004),
        inertial_origin=Origin(xyz=(0.0, MIDDLE_LENGTH / 2.0, 0.0)),
    )
    left_distal = _add_mesh_part(
        model,
        name="left_distal",
        shape=_make_link_shape(
            length=DISTAL_LENGTH,
            body_width=DISTAL_WIDTH,
            body_thickness=DISTAL_THICKNESS,
            barrel_length=CHAIN_BARREL_LENGTH,
            child_barrel_length=None,
            fingertip=True,
        ),
        material="finger_finish",
        inertial_box=(DISTAL_WIDTH + 0.006, DISTAL_LENGTH + 0.014, DISTAL_THICKNESS + 0.004),
        inertial_origin=Origin(xyz=(0.0, DISTAL_LENGTH / 2.0 + 0.005, 0.0)),
    )

    right_proximal = _add_mesh_part(
        model,
        name="right_proximal",
        shape=_make_link_shape(
            length=PROXIMAL_LENGTH,
            body_width=PROXIMAL_WIDTH,
            body_thickness=PROXIMAL_THICKNESS,
            barrel_length=ROOT_BARREL_LENGTH,
            child_barrel_length=CHAIN_BARREL_LENGTH,
        ),
        material="finger_finish",
        inertial_box=(PROXIMAL_WIDTH + 0.004, PROXIMAL_LENGTH, PROXIMAL_THICKNESS + 0.004),
        inertial_origin=Origin(xyz=(0.0, PROXIMAL_LENGTH / 2.0, 0.0)),
    )
    right_middle = _add_mesh_part(
        model,
        name="right_middle",
        shape=_make_link_shape(
            length=MIDDLE_LENGTH,
            body_width=MIDDLE_WIDTH,
            body_thickness=MIDDLE_THICKNESS,
            barrel_length=CHAIN_BARREL_LENGTH,
            child_barrel_length=CHAIN_BARREL_LENGTH,
        ),
        material="finger_finish",
        inertial_box=(MIDDLE_WIDTH + 0.004, MIDDLE_LENGTH, MIDDLE_THICKNESS + 0.004),
        inertial_origin=Origin(xyz=(0.0, MIDDLE_LENGTH / 2.0, 0.0)),
    )
    right_distal = _add_mesh_part(
        model,
        name="right_distal",
        shape=_make_link_shape(
            length=DISTAL_LENGTH,
            body_width=DISTAL_WIDTH,
            body_thickness=DISTAL_THICKNESS,
            barrel_length=CHAIN_BARREL_LENGTH,
            child_barrel_length=None,
            fingertip=True,
        ),
        material="finger_finish",
        inertial_box=(DISTAL_WIDTH + 0.006, DISTAL_LENGTH + 0.014, DISTAL_THICKNESS + 0.004),
        inertial_origin=Origin(xyz=(0.0, DISTAL_LENGTH / 2.0 + 0.005, 0.0)),
    )

    for prefix, root_x, proximal, middle, distal in (
        ("left", -FINGER_SPACING / 2.0, left_proximal, left_middle, left_distal),
        ("right", FINGER_SPACING / 2.0, right_proximal, right_middle, right_distal),
    ):
        model.articulation(
            f"palm_to_{prefix}_proximal",
            ArticulationType.REVOLUTE,
            parent=palm,
            child=proximal,
            origin=Origin(xyz=(root_x, ROOT_Y, ROOT_Z)),
            axis=JOINT_AXIS,
            motion_limits=MotionLimits(lower=0.0, upper=1.15, effort=5.0, velocity=2.5),
        )
        model.articulation(
            f"{prefix}_proximal_to_middle",
            ArticulationType.REVOLUTE,
            parent=proximal,
            child=middle,
            origin=Origin(xyz=(0.0, PROXIMAL_LENGTH, 0.0)),
            axis=JOINT_AXIS,
            motion_limits=MotionLimits(lower=0.0, upper=1.30, effort=4.0, velocity=3.0),
        )
        model.articulation(
            f"{prefix}_middle_to_distal",
            ArticulationType.REVOLUTE,
            parent=middle,
            child=distal,
            origin=Origin(xyz=(0.0, MIDDLE_LENGTH, 0.0)),
            axis=JOINT_AXIS,
            motion_limits=MotionLimits(lower=0.0, upper=1.10, effort=3.0, velocity=3.0),
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

    joints = {
        "palm_to_left_proximal": ("palm", "left_proximal"),
        "left_proximal_to_middle": ("left_proximal", "left_middle"),
        "left_middle_to_distal": ("left_middle", "left_distal"),
        "palm_to_right_proximal": ("palm", "right_proximal"),
        "right_proximal_to_middle": ("right_proximal", "right_middle"),
        "right_middle_to_distal": ("right_middle", "right_distal"),
    }
    for joint_name, (parent_name, child_name) in joints.items():
        joint = object_model.get_articulation(joint_name)
        ctx.check(
            f"{joint_name} stays in its own finger chain",
            joint.parent == parent_name and joint.child == child_name and joint.axis == JOINT_AXIS,
            details=f"parent={joint.parent}, child={joint.child}, axis={joint.axis}",
        )

    ctx.expect_origin_gap(
        right_proximal,
        left_proximal,
        axis="x",
        min_gap=FINGER_SPACING - 0.004,
        max_gap=FINGER_SPACING + 0.004,
        name="two root knuckles stay side by side on the palm beam",
    )
    ctx.expect_origin_gap(
        left_proximal,
        palm,
        axis="z",
        min_gap=ROOT_Z - 0.001,
        max_gap=ROOT_Z + 0.001,
        name="left finger root sits above the grounded palm beam",
    )
    ctx.expect_origin_gap(
        right_proximal,
        palm,
        axis="z",
        min_gap=ROOT_Z - 0.001,
        max_gap=ROOT_Z + 0.001,
        name="right finger root sits above the grounded palm beam",
    )

    left_root = object_model.get_articulation("palm_to_left_proximal")
    left_mid = object_model.get_articulation("left_proximal_to_middle")
    left_tip = object_model.get_articulation("left_middle_to_distal")
    right_root = object_model.get_articulation("palm_to_right_proximal")
    right_mid = object_model.get_articulation("right_proximal_to_middle")
    right_tip = object_model.get_articulation("right_middle_to_distal")

    left_rest = ctx.part_world_position(left_distal)
    right_rest = ctx.part_world_position(right_distal)

    with ctx.pose({left_root: 0.70, left_mid: 0.85, left_tip: 0.60}):
        left_curled = ctx.part_world_position(left_distal)
        right_still = ctx.part_world_position(right_distal)

    ctx.check(
        "left finger curls downward when its own joints flex",
        left_rest is not None
        and left_curled is not None
        and left_curled[2] < left_rest[2] - 0.030
        and left_curled[1] < left_rest[1] - 0.010,
        details=f"rest={left_rest}, curled={left_curled}",
    )
    ctx.check(
        "left finger motion leaves the right finger uncoupled",
        right_rest is not None
        and right_still is not None
        and all(isclose(a, b, abs_tol=1e-6) for a, b in zip(right_rest, right_still)),
        details=f"right_rest={right_rest}, right_still={right_still}",
    )

    with ctx.pose({right_root: 0.70, right_mid: 0.85, right_tip: 0.60}):
        right_curled = ctx.part_world_position(right_distal)
        left_still = ctx.part_world_position(left_distal)

    ctx.check(
        "right finger curls downward when its own joints flex",
        right_rest is not None
        and right_curled is not None
        and right_curled[2] < right_rest[2] - 0.030
        and right_curled[1] < right_rest[1] - 0.010,
        details=f"rest={right_rest}, curled={right_curled}",
    )
    ctx.check(
        "right finger motion leaves the left finger uncoupled",
        left_rest is not None
        and left_still is not None
        and all(isclose(a, b, abs_tol=1e-6) for a, b in zip(left_rest, left_still)),
        details=f"left_rest={left_rest}, left_still={left_still}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
