from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

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


HOUSING_DEPTH = 0.086
HOUSING_WIDTH = 0.056
HOUSING_HEIGHT = 0.066
HOUSING_CENTER_X = 0.004
HALF_LATERAL_OFFSET = 0.042
HOUSING_CENTER_Z = -0.022

OBJECTIVE_BARREL_RADIUS = 0.0205
OBJECTIVE_RING_RADIUS = 0.024
OBJECTIVE_BARREL_LENGTH = 0.046
OBJECTIVE_RING_LENGTH = 0.010
OBJECTIVE_CENTER_X = 0.070
OBJECTIVE_CENTER_Z = -0.020

EYEPIECE_RADIUS = 0.014
EYECUP_RADIUS = 0.018
EYEPIECE_LENGTH = 0.024
EYECUP_LENGTH = 0.014
EYEPIECE_CENTER_X = -0.051
EYEPIECE_CENTER_Z = -0.018

COVER_RADIUS = OBJECTIVE_RING_RADIUS
COVER_THICKNESS = 0.004
COVER_HINGE_X = OBJECTIVE_CENTER_X + (OBJECTIVE_BARREL_LENGTH / 2.0)
COVER_HINGE_Z = OBJECTIVE_CENTER_Z + OBJECTIVE_RING_RADIUS

HINGE_KNUCKLE_RADIUS = 0.009
LEFT_KNUCKLE_LENGTH = 0.014
RIGHT_KNUCKLE_LENGTH = 0.024
LEFT_KNUCKLE_CENTER_Z = 0.019


def _add_body_half(
    part,
    *,
    prefix: str,
    side: float,
    housing_material,
    rubber_material,
    metal_material,
    with_focus_pedestal: bool,
) -> None:
    lateral_center = side * HALF_LATERAL_OFFSET

    part.visual(
        Box((HOUSING_DEPTH, HOUSING_WIDTH, HOUSING_HEIGHT)),
        origin=Origin(xyz=(HOUSING_CENTER_X, lateral_center, HOUSING_CENTER_Z)),
        material=housing_material,
        name=f"{prefix}_housing",
    )
    part.visual(
        Box((0.040, 0.048, 0.012)),
        origin=Origin(xyz=(-0.004, lateral_center, 0.005)),
        material=housing_material,
        name=f"{prefix}_top_rail",
    )
    part.visual(
        Box((0.018, 0.044, 0.024)),
        origin=Origin(xyz=(0.054, lateral_center, OBJECTIVE_CENTER_Z)),
        material=housing_material,
        name=f"{prefix}_barrel_saddle",
    )
    part.visual(
        Cylinder(radius=OBJECTIVE_BARREL_RADIUS, length=OBJECTIVE_BARREL_LENGTH),
        origin=Origin(
            xyz=(OBJECTIVE_CENTER_X, lateral_center, OBJECTIVE_CENTER_Z),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=rubber_material,
        name=f"{prefix}_objective_barrel",
    )
    part.visual(
        Cylinder(radius=OBJECTIVE_RING_RADIUS, length=OBJECTIVE_RING_LENGTH),
        origin=Origin(
            xyz=(
                OBJECTIVE_CENTER_X + 0.018,
                lateral_center,
                OBJECTIVE_CENTER_Z,
            ),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=rubber_material,
        name=f"{prefix}_objective_ring",
    )
    part.visual(
        Cylinder(radius=EYEPIECE_RADIUS, length=EYEPIECE_LENGTH),
        origin=Origin(
            xyz=(EYEPIECE_CENTER_X, lateral_center, EYEPIECE_CENTER_Z),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=rubber_material,
        name=f"{prefix}_eyepiece",
    )
    part.visual(
        Cylinder(radius=EYECUP_RADIUS, length=EYECUP_LENGTH),
        origin=Origin(
            xyz=(EYEPIECE_CENTER_X - 0.005, lateral_center, EYEPIECE_CENTER_Z),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=rubber_material,
        name=f"{prefix}_eyecup",
    )

    if prefix == "left":
        part.visual(
            Box((0.028, 0.014, 0.016)),
            origin=Origin(xyz=(-0.006, -0.010, -0.018)),
            material=housing_material,
            name="left_lower_bridge_block",
        )
        part.visual(
            Box((0.028, 0.014, 0.016)),
            origin=Origin(xyz=(-0.006, -0.010, 0.018)),
            material=housing_material,
            name="left_upper_bridge_block",
        )
        part.visual(
            Cylinder(radius=HINGE_KNUCKLE_RADIUS, length=LEFT_KNUCKLE_LENGTH),
            origin=Origin(xyz=(0.0, 0.0, -LEFT_KNUCKLE_CENTER_Z)),
            material=metal_material,
            name="left_lower_knuckle",
        )
        part.visual(
            Cylinder(radius=HINGE_KNUCKLE_RADIUS, length=LEFT_KNUCKLE_LENGTH),
            origin=Origin(xyz=(0.0, 0.0, LEFT_KNUCKLE_CENTER_Z)),
            material=metal_material,
            name="left_upper_knuckle",
        )
        if with_focus_pedestal:
            part.visual(
                Cylinder(radius=0.011, length=0.008),
                origin=Origin(xyz=(0.0, 0.0, 0.030)),
                material=metal_material,
                name="focus_pedestal",
            )
    else:
        part.visual(
            Box((0.028, 0.014, 0.024)),
            origin=Origin(xyz=(-0.006, 0.010, 0.0)),
            material=housing_material,
            name="right_bridge_block",
        )
        part.visual(
            Cylinder(radius=0.0088, length=RIGHT_KNUCKLE_LENGTH),
            origin=Origin(),
            material=metal_material,
            name="right_center_knuckle",
        )


def _add_cover(part, *, material, hinge_material) -> None:
    part.visual(
        Cylinder(radius=COVER_RADIUS, length=COVER_THICKNESS),
        origin=Origin(
            xyz=(COVER_THICKNESS / 2.0, 0.0, -COVER_RADIUS),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=material,
        name="cover_disc",
    )
    part.visual(
        Cylinder(radius=0.0025, length=0.018),
        origin=Origin(xyz=(0.0035, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=hinge_material,
        name="cover_hinge_barrel",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="night_vision_binocular")

    housing_material = model.material("housing_olive", rgba=(0.17, 0.19, 0.15, 1.0))
    rubber_material = model.material("rubber_black", rgba=(0.08, 0.09, 0.09, 1.0))
    hinge_metal = model.material("hinge_metal", rgba=(0.24, 0.26, 0.28, 1.0))
    knob_material = model.material("knob_gray", rgba=(0.14, 0.15, 0.16, 1.0))

    left_body = model.part("left_body")
    _add_body_half(
        left_body,
        prefix="left",
        side=-1.0,
        housing_material=housing_material,
        rubber_material=rubber_material,
        metal_material=hinge_metal,
        with_focus_pedestal=True,
    )

    right_body = model.part("right_body")
    _add_body_half(
        right_body,
        prefix="right",
        side=1.0,
        housing_material=housing_material,
        rubber_material=rubber_material,
        metal_material=hinge_metal,
        with_focus_pedestal=False,
    )

    focus_knob = model.part("focus_knob")
    focus_knob.visual(
        Cylinder(radius=0.014, length=0.012),
        material=knob_material,
        name="knob_body",
    )
    focus_knob.visual(
        Cylinder(radius=0.017, length=0.006),
        material=knob_material,
        name="knob_grip_ring",
    )

    left_cover = model.part("left_cover")
    _add_cover(left_cover, material=rubber_material, hinge_material=hinge_metal)

    right_cover = model.part("right_cover")
    _add_cover(right_cover, material=rubber_material, hinge_material=hinge_metal)

    model.articulation(
        "interpupillary_hinge",
        ArticulationType.REVOLUTE,
        parent=left_body,
        child=right_body,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=1.2,
            lower=math.radians(-18.0),
            upper=math.radians(24.0),
        ),
    )

    model.articulation(
        "bridge_to_focus_knob",
        ArticulationType.REVOLUTE,
        parent=left_body,
        child=focus_knob,
        origin=Origin(xyz=(0.0, 0.0, 0.040)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=0.5,
            velocity=8.0,
            lower=-math.radians(180.0),
            upper=math.radians(180.0),
        ),
    )

    model.articulation(
        "left_objective_cover_hinge",
        ArticulationType.REVOLUTE,
        parent=left_body,
        child=left_cover,
        origin=Origin(xyz=(COVER_HINGE_X, -HALF_LATERAL_OFFSET, COVER_HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.2,
            velocity=4.0,
            lower=0.0,
            upper=math.radians(125.0),
        ),
    )

    model.articulation(
        "right_objective_cover_hinge",
        ArticulationType.REVOLUTE,
        parent=right_body,
        child=right_cover,
        origin=Origin(xyz=(COVER_HINGE_X, HALF_LATERAL_OFFSET, COVER_HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.2,
            velocity=4.0,
            lower=0.0,
            upper=math.radians(125.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    left_body = object_model.get_part("left_body")
    right_body = object_model.get_part("right_body")
    focus_knob = object_model.get_part("focus_knob")
    left_cover = object_model.get_part("left_cover")
    right_cover = object_model.get_part("right_cover")

    hinge = object_model.get_articulation("interpupillary_hinge")
    focus_joint = object_model.get_articulation("bridge_to_focus_knob")
    left_cover_hinge = object_model.get_articulation("left_objective_cover_hinge")
    right_cover_hinge = object_model.get_articulation("right_objective_cover_hinge")

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
        "interpupillary hinge axis is vertical",
        hinge.axis == (0.0, 0.0, 1.0),
        details=f"axis={hinge.axis}",
    )
    ctx.check(
        "focus knob spins on bridge axis",
        focus_joint.axis == (0.0, 0.0, 1.0),
        details=f"axis={focus_joint.axis}",
    )
    ctx.check(
        "objective covers share flip-up hinge axis",
        left_cover_hinge.axis == (0.0, -1.0, 0.0) and right_cover_hinge.axis == (0.0, -1.0, 0.0),
        details=f"left={left_cover_hinge.axis}, right={right_cover_hinge.axis}",
    )

    with ctx.pose(
        {
            hinge: 0.0,
            left_cover_hinge: 0.0,
            right_cover_hinge: 0.0,
            focus_joint: 0.0,
        }
    ):
        ctx.expect_gap(
            left_cover,
            left_body,
            axis="x",
            max_gap=0.0005,
            max_penetration=1e-6,
            positive_elem="cover_disc",
            negative_elem="left_objective_ring",
            name="left cover closes against left objective ring",
        )
        ctx.expect_gap(
            right_cover,
            right_body,
            axis="x",
            max_gap=0.0005,
            max_penetration=1e-6,
            positive_elem="cover_disc",
            negative_elem="right_objective_ring",
            name="right cover closes against right objective ring",
        )
        ctx.expect_gap(
            left_body,
            right_body,
            axis="z",
            max_gap=0.0005,
            max_penetration=0.0,
            positive_elem="left_upper_knuckle",
            negative_elem="right_center_knuckle",
            name="upper hinge knuckles stack cleanly",
        )
        ctx.expect_gap(
            right_body,
            left_body,
            axis="z",
            max_gap=0.0005,
            max_penetration=0.0,
            positive_elem="right_center_knuckle",
            negative_elem="left_lower_knuckle",
            name="lower hinge knuckles stack cleanly",
        )
        ctx.expect_gap(
            focus_knob,
            left_body,
            axis="z",
            max_gap=0.0005,
            max_penetration=0.0,
            positive_elem="knob_body",
            negative_elem="focus_pedestal",
            name="focus knob seats on bridge pedestal",
        )

    right_barrel_rest = ctx.part_element_world_aabb(right_body, elem="right_objective_barrel")
    with ctx.pose({hinge: math.radians(20.0)}):
        right_barrel_open = ctx.part_element_world_aabb(right_body, elem="right_objective_barrel")
    ctx.check(
        "interpupillary hinge swings right barrel outward",
        right_barrel_rest is not None
        and right_barrel_open is not None
        and right_barrel_open[1][1] > right_barrel_rest[1][1] + 0.01,
        details=f"rest={right_barrel_rest}, open={right_barrel_open}",
    )

    left_cover_rest = ctx.part_element_world_aabb(left_cover, elem="cover_disc")
    with ctx.pose({left_cover_hinge: math.radians(105.0)}):
        left_cover_open = ctx.part_element_world_aabb(left_cover, elem="cover_disc")
    ctx.check(
        "left cover flips above the lens line",
        left_cover_rest is not None
        and left_cover_open is not None
        and left_cover_open[1][2] > left_cover_rest[1][2] + 0.015,
        details=f"rest={left_cover_rest}, open={left_cover_open}",
    )

    right_cover_rest = ctx.part_element_world_aabb(right_cover, elem="cover_disc")
    with ctx.pose({right_cover_hinge: math.radians(105.0)}):
        right_cover_open = ctx.part_element_world_aabb(right_cover, elem="cover_disc")
    ctx.check(
        "right cover flips above the lens line",
        right_cover_rest is not None
        and right_cover_open is not None
        and right_cover_open[1][2] > right_cover_rest[1][2] + 0.015,
        details=f"rest={right_cover_rest}, open={right_cover_open}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
