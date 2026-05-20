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
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="farm_pedestrian_gate")

    galvanized = model.material("galvanized", rgba=(0.71, 0.73, 0.75, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.24, 0.26, 0.28, 1.0))
    black_iron = model.material("black_iron", rgba=(0.10, 0.11, 0.12, 1.0))
    concrete = model.material("concrete", rgba=(0.60, 0.60, 0.58, 1.0))

    post_frame = model.part("post_frame")
    post_frame.visual(
        Box((1.22, 0.12, 0.20)),
        origin=Origin(xyz=(0.53, 0.0, -0.12)),
        material=concrete,
        name="grade_beam",
    )
    post_frame.visual(
        Cylinder(radius=0.04, length=1.77),
        origin=Origin(xyz=(-0.06, 0.0, 0.665)),
        material=dark_steel,
        name="hinge_post",
    )
    post_frame.visual(
        Cylinder(radius=0.04, length=1.77),
        origin=Origin(xyz=(1.075, 0.0, 0.665)),
        material=dark_steel,
        name="latch_post",
    )
    post_frame.visual(
        Cylinder(radius=0.045, length=0.014),
        origin=Origin(xyz=(-0.06, 0.0, 1.557)),
        material=black_iron,
        name="hinge_post_cap",
    )
    post_frame.visual(
        Cylinder(radius=0.045, length=0.014),
        origin=Origin(xyz=(1.075, 0.0, 1.557)),
        material=black_iron,
        name="latch_post_cap",
    )
    for index, z_center in enumerate((0.20, 1.04)):
        post_frame.visual(
            Box((0.030, 0.016, 0.17)),
            origin=Origin(xyz=(-0.015, -0.024, z_center)),
            material=dark_steel,
            name=f"hinge_mount_{index}",
        )
    post_frame.visual(
        Box((0.022, 0.014, 0.22)),
        origin=Origin(xyz=(1.029, 0.032, 0.69)),
        material=black_iron,
        name="keeper_base",
    )
    post_frame.visual(
        Box((0.030, 0.020, 0.032)),
        origin=Origin(xyz=(1.027, 0.032, 0.745)),
        material=black_iron,
        name="keeper_upper_lip",
    )
    post_frame.visual(
        Box((0.030, 0.020, 0.032)),
        origin=Origin(xyz=(1.027, 0.032, 0.635)),
        material=black_iron,
        name="keeper_lower_lip",
    )
    post_frame.inertial = Inertial.from_geometry(
        Box((1.24, 0.18, 1.80)),
        mass=42.0,
        origin=Origin(xyz=(0.53, 0.0, 0.67)),
    )

    gate_leaf = model.part("gate_leaf")
    gate_leaf.visual(
        Box((0.965, 0.04, 0.04)),
        origin=Origin(xyz=(0.5175, 0.0, 0.96)),
        material=galvanized,
        name="top_rail",
    )
    gate_leaf.visual(
        Box((0.965, 0.04, 0.04)),
        origin=Origin(xyz=(0.5175, 0.0, -0.04)),
        material=galvanized,
        name="bottom_rail",
    )
    gate_leaf.visual(
        Box((0.04, 0.04, 1.04)),
        origin=Origin(xyz=(0.055, 0.0, 0.46)),
        material=galvanized,
        name="hinge_stile",
    )
    gate_leaf.visual(
        Box((0.04, 0.04, 1.04)),
        origin=Origin(xyz=(0.98, 0.0, 0.46)),
        material=galvanized,
        name="closing_stile",
    )
    gate_leaf.visual(
        Box((0.12, 0.020, 0.18)),
        origin=Origin(xyz=(0.935, 0.0, 0.49)),
        material=galvanized,
        name="latch_mount_plate",
    )
    gate_leaf.visual(
        Cylinder(radius=0.0065, length=0.014),
        origin=Origin(xyz=(0.956, 0.018, 0.49), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="pivot_spacer",
    )
    for index, x_center in enumerate((0.15, 0.26, 0.37, 0.48, 0.59, 0.70, 0.81, 0.92)):
        gate_leaf.visual(
            Cylinder(radius=0.0035, length=0.98),
            origin=Origin(xyz=(x_center, 0.0, 0.45)),
            material=galvanized,
            name=f"mesh_vertical_{index}",
        )
    for index, z_center in enumerate((0.07, 0.18, 0.29, 0.40, 0.51, 0.62, 0.73, 0.84)):
        gate_leaf.visual(
            Cylinder(radius=0.0035, length=0.93),
            origin=Origin(xyz=(0.52, 0.0, z_center), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=galvanized,
            name=f"mesh_horizontal_{index}",
        )
    for index, z_center in enumerate((0.0, 0.84)):
        gate_leaf.visual(
            Cylinder(radius=0.016, length=0.14),
            origin=Origin(xyz=(0.0, 0.0, z_center)),
            material=dark_steel,
            name=f"sleeve_{index}",
        )
        gate_leaf.visual(
            Box((0.060, 0.010, 0.060)),
            origin=Origin(xyz=(0.030, 0.0, z_center)),
            material=dark_steel,
            name=f"sleeve_strap_{index}",
        )
    gate_leaf.inertial = Inertial.from_geometry(
        Box((1.02, 0.06, 1.12)),
        mass=16.0,
        origin=Origin(xyz=(0.51, 0.0, 0.46)),
    )

    lever_latch = model.part("lever_latch")
    lever_latch.visual(
        Cylinder(radius=0.012, length=0.022),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=black_iron,
        name="pivot_boss",
    )
    lever_latch.visual(
        Box((0.024, 0.014, 0.120)),
        origin=Origin(xyz=(-0.006, 0.0, -0.060)),
        material=black_iron,
        name="lever_arm",
    )
    lever_latch.visual(
        Box((0.056, 0.012, 0.018)),
        origin=Origin(xyz=(0.028, 0.0, 0.0)),
        material=black_iron,
        name="latch_nose",
    )
    lever_latch.visual(
        Cylinder(radius=0.010, length=0.050),
        origin=Origin(xyz=(-0.004, 0.0, -0.126), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black_iron,
        name="handgrip",
    )
    lever_latch.visual(
        Box((0.014, 0.012, 0.030)),
        origin=Origin(xyz=(0.014, 0.0, 0.014)),
        material=black_iron,
        name="cam_rib",
    )
    lever_latch.inertial = Inertial.from_geometry(
        Box((0.10, 0.03, 0.17)),
        mass=0.45,
        origin=Origin(xyz=(0.01, 0.0, -0.04)),
    )

    model.articulation(
        "gate_hinge",
        ArticulationType.REVOLUTE,
        parent=post_frame,
        child=gate_leaf,
        origin=Origin(xyz=(0.0, 0.0, 0.20)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=25.0,
            velocity=1.2,
            lower=0.0,
            upper=1.75,
        ),
    )
    model.articulation(
        "latch_pivot",
        ArticulationType.REVOLUTE,
        parent=gate_leaf,
        child=lever_latch,
        origin=Origin(xyz=(0.956, 0.036, 0.49)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=4.0,
            lower=0.0,
            upper=0.80,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    post_frame = object_model.get_part("post_frame")
    gate_leaf = object_model.get_part("gate_leaf")
    lever_latch = object_model.get_part("lever_latch")
    gate_hinge = object_model.get_articulation("gate_hinge")
    latch_pivot = object_model.get_articulation("latch_pivot")

    def _aabb_center(aabb):
        if aabb is None:
            return None
        (min_x, min_y, min_z), (max_x, max_y, max_z) = aabb
        return (
            0.5 * (min_x + max_x),
            0.5 * (min_y + max_y),
            0.5 * (min_z + max_z),
        )

    with ctx.pose({gate_hinge: 0.0, latch_pivot: 0.0}):
        ctx.expect_gap(
            post_frame,
            gate_leaf,
            axis="x",
            positive_elem="keeper_base",
            negative_elem="closing_stile",
            min_gap=0.010,
            max_gap=0.040,
            name="gate leaf closes just short of the keeper",
        )
        ctx.expect_gap(
            post_frame,
            lever_latch,
            axis="x",
            positive_elem="keeper_base",
            negative_elem="latch_nose",
            min_gap=0.001,
            max_gap=0.012,
            name="lever nose sits within reach of the keeper",
        )
        ctx.expect_overlap(
            post_frame,
            lever_latch,
            axes="yz",
            elem_a="keeper_base",
            elem_b="latch_nose",
            min_overlap=0.008,
            name="keeper aligns with latch nose in the closed pose",
        )
        ctx.expect_gap(
            lever_latch,
            gate_leaf,
            axis="y",
            positive_elem="pivot_boss",
            negative_elem="latch_mount_plate",
            min_gap=0.002,
            max_gap=0.020,
            name="latch lever stands proud of the mounting plate",
        )

        closed_stile = _aabb_center(ctx.part_element_world_aabb(gate_leaf, elem="closing_stile"))
        rest_nose = _aabb_center(ctx.part_element_world_aabb(lever_latch, elem="latch_nose"))

    with ctx.pose({gate_hinge: 1.15, latch_pivot: 0.0}):
        open_stile = _aabb_center(ctx.part_element_world_aabb(gate_leaf, elem="closing_stile"))

    with ctx.pose({gate_hinge: 0.0, latch_pivot: 0.55}):
        lifted_nose = _aabb_center(ctx.part_element_world_aabb(lever_latch, elem="latch_nose"))

    ctx.check(
        "gate swings open around the post hinge",
        closed_stile is not None
        and open_stile is not None
        and open_stile[1] > closed_stile[1] + 0.55
        and open_stile[0] < closed_stile[0] - 0.40,
        details=f"closed={closed_stile}, open={open_stile}",
    )
    ctx.check(
        "lever latch lifts upward when actuated",
        rest_nose is not None
        and lifted_nose is not None
        and lifted_nose[2] > rest_nose[2] + 0.010,
        details=f"rest={rest_nose}, lifted={lifted_nose}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
