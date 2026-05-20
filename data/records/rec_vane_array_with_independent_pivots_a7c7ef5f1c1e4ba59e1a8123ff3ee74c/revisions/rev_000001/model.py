from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

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


OUTER_WIDTH = 0.84
BODY_HEIGHT = 0.50
BODY_BOTTOM = 0.04
DEPTH = 0.18
WALL_THICKNESS = 0.04
INNER_WIDTH = OUTER_WIDTH - 2.0 * WALL_THICKNESS
OPENING_HEIGHT = BODY_HEIGHT - 2.0 * WALL_THICKNESS
BODY_TOP = BODY_BOTTOM + BODY_HEIGHT

FOOT_WIDTH = 0.16
FOOT_DEPTH = 0.10
FOOT_HEIGHT = 0.045
FOOT_X_OFFSET = 0.24

VANE_COUNT = 5
VANE_CHORD = 0.09
VANE_THICKNESS = 0.014
VANE_JOURNAL_RADIUS = 0.008
VANE_JOURNAL_LENGTH = 0.036
VANE_BLADE_SPAN = INNER_WIDTH - 2.0 * VANE_JOURNAL_LENGTH
VANE_LIMIT = 0.65

SUPPORT_PAD_THICKNESS = 0.03
SUPPORT_PAD_DEPTH = 0.07
SUPPORT_PAD_HEIGHT = 0.05
SUPPORT_PAD_INNER_PROJECTION = 0.012
SUPPORT_PAD_WALL_EMBED = SUPPORT_PAD_THICKNESS - SUPPORT_PAD_INNER_PROJECTION
SUPPORT_HUB_RADIUS = 0.018
SUPPORT_HUB_LENGTH = 0.024


def _vane_z_positions() -> tuple[float, ...]:
    opening_bottom = BODY_BOTTOM + WALL_THICKNESS
    pitch = OPENING_HEIGHT / (VANE_COUNT + 1)
    return tuple(opening_bottom + pitch * (index + 1) for index in range(VANE_COUNT))


VANE_ZS = _vane_z_positions()


def _make_support_bracket_shape() -> cq.Workplane:
    pad = cq.Workplane("XY").box(
        SUPPORT_PAD_THICKNESS,
        SUPPORT_PAD_DEPTH,
        SUPPORT_PAD_HEIGHT,
    )
    pad = pad.translate((-SUPPORT_PAD_THICKNESS / 2.0, 0.0, 0.0))

    outer_hub = cq.Solid.makeCylinder(
        SUPPORT_HUB_RADIUS,
        SUPPORT_HUB_LENGTH,
        cq.Vector(0.0, 0.0, 0.0),
        cq.Vector(-1.0, 0.0, 0.0),
    )
    inner_bore = cq.Solid.makeCylinder(
        VANE_JOURNAL_RADIUS,
        SUPPORT_PAD_THICKNESS,
        cq.Vector(0.0, 0.0, 0.0),
        cq.Vector(-1.0, 0.0, 0.0),
    )

    bracket = pad.union(cq.Workplane(obj=outer_hub))
    bracket = bracket.cut(cq.Workplane(obj=inner_bore))
    return bracket


def _make_vane_shape() -> cq.Workplane:
    blade = cq.Workplane("YZ").ellipse(VANE_CHORD / 2.0, VANE_THICKNESS / 2.0).extrude(VANE_BLADE_SPAN)
    blade = blade.translate((-VANE_BLADE_SPAN / 2.0, 0.0, 0.0))

    left_journal = cq.Solid.makeCylinder(
        VANE_JOURNAL_RADIUS,
        VANE_JOURNAL_LENGTH,
        cq.Vector(-VANE_BLADE_SPAN / 2.0, 0.0, 0.0),
        cq.Vector(-1.0, 0.0, 0.0),
    )
    right_journal = cq.Solid.makeCylinder(
        VANE_JOURNAL_RADIUS,
        VANE_JOURNAL_LENGTH,
        cq.Vector(VANE_BLADE_SPAN / 2.0, 0.0, 0.0),
        cq.Vector(1.0, 0.0, 0.0),
    )

    vane = blade.union(cq.Workplane(obj=left_journal))
    vane = vane.union(cq.Workplane(obj=right_journal))
    return vane


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="boxed_frame_vane_array")

    model.material("housing_finish", rgba=(0.19, 0.21, 0.24, 1.0))
    model.material("vane_finish", rgba=(0.77, 0.79, 0.82, 1.0))

    support_bracket_mesh = mesh_from_cadquery(_make_support_bracket_shape(), "support_bracket_mesh")
    vane_mesh = mesh_from_cadquery(_make_vane_shape(), "vane_body_mesh")

    housing = model.part("housing")
    body_center_z = BODY_BOTTOM + BODY_HEIGHT / 2.0
    housing.visual(
        Box((WALL_THICKNESS, DEPTH, BODY_HEIGHT)),
        origin=Origin(
            xyz=(-INNER_WIDTH / 2.0 - WALL_THICKNESS / 2.0, 0.0, body_center_z),
        ),
        material="housing_finish",
        name="left_side_wall",
    )
    housing.visual(
        Box((WALL_THICKNESS, DEPTH, BODY_HEIGHT)),
        origin=Origin(
            xyz=(INNER_WIDTH / 2.0 + WALL_THICKNESS / 2.0, 0.0, body_center_z),
        ),
        material="housing_finish",
        name="right_side_wall",
    )
    housing.visual(
        Box((INNER_WIDTH, DEPTH, WALL_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, BODY_BOTTOM + WALL_THICKNESS / 2.0)),
        material="housing_finish",
        name="bottom_rail",
    )
    housing.visual(
        Box((INNER_WIDTH, DEPTH, WALL_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, BODY_TOP - WALL_THICKNESS / 2.0)),
        material="housing_finish",
        name="top_rail",
    )
    housing.visual(
        Box((FOOT_WIDTH, FOOT_DEPTH, FOOT_HEIGHT)),
        origin=Origin(xyz=(-FOOT_X_OFFSET, 0.0, FOOT_HEIGHT / 2.0)),
        material="housing_finish",
        name="left_foot",
    )
    housing.visual(
        Box((FOOT_WIDTH, FOOT_DEPTH, FOOT_HEIGHT)),
        origin=Origin(xyz=(FOOT_X_OFFSET, 0.0, FOOT_HEIGHT / 2.0)),
        material="housing_finish",
        name="right_foot",
    )

    for index, vane_z in enumerate(VANE_ZS, start=1):
        housing.visual(
            support_bracket_mesh,
            origin=Origin(
                xyz=(INNER_WIDTH / 2.0, 0.0, vane_z),
                rpy=(0.0, 0.0, math.pi),
            ),
            material="housing_finish",
            name=f"right_support_{index}",
        )
        housing.visual(
            support_bracket_mesh,
            origin=Origin(xyz=(-INNER_WIDTH / 2.0, 0.0, vane_z)),
            material="housing_finish",
            name=f"left_support_{index}",
        )

    housing.inertial = Inertial.from_geometry(
        Box((OUTER_WIDTH, DEPTH, BODY_TOP)),
        mass=8.5,
        origin=Origin(xyz=(0.0, 0.0, BODY_TOP / 2.0)),
    )

    for index, vane_z in enumerate(VANE_ZS, start=1):
        vane = model.part(f"vane_{index}")
        vane.visual(vane_mesh, material="vane_finish", name="vane_body")
        vane.inertial = Inertial.from_geometry(
            Box(
                (
                    VANE_BLADE_SPAN + 2.0 * VANE_JOURNAL_LENGTH,
                    VANE_CHORD,
                    SUPPORT_HUB_RADIUS * 2.0,
                )
            ),
            mass=0.34,
        )

        model.articulation(
            f"housing_to_vane_{index}",
            ArticulationType.REVOLUTE,
            parent=housing,
            child=vane,
            origin=Origin(xyz=(0.0, 0.0, vane_z)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=1.5,
                velocity=2.0,
                lower=-VANE_LIMIT,
                upper=VANE_LIMIT,
            ),
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

    housing = object_model.get_part("housing")
    vane_parts = [object_model.get_part(f"vane_{index}") for index in range(1, VANE_COUNT + 1)]
    vane_joints = [
        object_model.get_articulation(f"housing_to_vane_{index}")
        for index in range(1, VANE_COUNT + 1)
    ]

    for index, (vane, joint) in enumerate(zip(vane_parts, vane_joints), start=1):
        ctx.check(
            f"vane_{index} is parented to the housing",
            joint.parent == housing.name and joint.child == vane.name,
            details=f"parent={joint.parent}, child={joint.child}",
        )

    def _span(aabb: tuple[tuple[float, float, float], tuple[float, float, float]], axis: int) -> float:
        return aabb[1][axis] - aabb[0][axis]

    def _center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]], axis: int) -> float:
        return 0.5 * (aabb[0][axis] + aabb[1][axis])

    first_vane = vane_parts[0]
    second_vane = vane_parts[1]
    first_joint = vane_joints[0]

    rest_aabb = ctx.part_element_world_aabb(first_vane, elem="vane_body")
    second_rest_aabb = ctx.part_element_world_aabb(second_vane, elem="vane_body")
    with ctx.pose({first_joint: VANE_LIMIT}):
        open_aabb = ctx.part_element_world_aabb(first_vane, elem="vane_body")
        second_after_aabb = ctx.part_element_world_aabb(second_vane, elem="vane_body")

    ctx.check(
        "first vane pitches about its long supported axis",
        rest_aabb is not None
        and open_aabb is not None
        and _span(open_aabb, 2) > _span(rest_aabb, 2) + 0.035
        and abs(_span(open_aabb, 0) - _span(rest_aabb, 0)) < 0.003,
        details=f"rest={rest_aabb}, open={open_aabb}",
    )
    ctx.check(
        "moving one vane leaves the neighboring vane untouched",
        second_rest_aabb is not None
        and second_after_aabb is not None
        and abs(_span(second_after_aabb, 2) - _span(second_rest_aabb, 2)) < 0.001
        and abs(_center(second_after_aabb, 2) - _center(second_rest_aabb, 2)) < 1e-6,
        details=f"rest={second_rest_aabb}, after={second_after_aabb}",
    )

    with ctx.pose({joint: VANE_LIMIT for joint in vane_joints}):
        for index in range(VANE_COUNT - 1):
            ctx.expect_gap(
                vane_parts[index + 1],
                vane_parts[index],
                axis="z",
                positive_elem="vane_body",
                negative_elem="vane_body",
                max_penetration=0.0,
                name=f"adjacent vanes {index + 1} and {index + 2} stay separated at full tilt",
            )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
