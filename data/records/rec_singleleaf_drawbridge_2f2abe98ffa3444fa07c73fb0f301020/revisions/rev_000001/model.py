from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
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
    model = ArticulatedObject(name="single_leaf_drawbridge")

    concrete = model.material("concrete", rgba=(0.62, 0.63, 0.65, 1.0))
    steel = model.material("steel", rgba=(0.32, 0.35, 0.38, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.18, 0.19, 0.21, 1.0))
    deck_paving = model.material("deck_paving", rgba=(0.20, 0.20, 0.19, 1.0))
    safety_yellow = model.material("safety_yellow", rgba=(0.77, 0.64, 0.14, 1.0))

    leaf_length = 8.0
    leaf_width = 4.2
    deck_top_z = 0.18
    girder_depth = 0.55
    girder_width = 0.18
    trunnion_radius = 0.18
    hinge_x = 0.0
    hinge_z = 0.0

    shore_frame = model.part("shore_frame")
    shore_frame.visual(
        Box((2.6, 5.0, 0.45)),
        origin=Origin(xyz=(-1.5, 0.0, deck_top_z - 0.225)),
        material=concrete,
        name="approach_slab",
    )
    shore_frame.visual(
        Box((0.90, 3.50, 0.70)),
        origin=Origin(xyz=(-0.45, 0.0, -0.17)),
        material=concrete,
        name="hinge_bulkhead",
    )
    shore_frame.visual(
        Box((0.95, 0.46, 1.02)),
        origin=Origin(xyz=(-0.18, 2.42, 0.12)),
        material=concrete,
        name="left_bearing_pedestal",
    )
    shore_frame.visual(
        Box((0.95, 0.46, 1.02)),
        origin=Origin(xyz=(-0.18, -2.42, 0.12)),
        material=concrete,
        name="right_bearing_pedestal",
    )
    shore_frame.visual(
        Cylinder(radius=0.30, length=0.26),
        origin=Origin(xyz=(0.04, 2.27, hinge_z), rpy=(1.57079632679, 0.0, 0.0)),
        material=steel,
        name="left_side_bearing",
    )
    shore_frame.visual(
        Cylinder(radius=0.30, length=0.26),
        origin=Origin(xyz=(0.04, -2.27, hinge_z), rpy=(1.57079632679, 0.0, 0.0)),
        material=steel,
        name="right_side_bearing",
    )
    shore_frame.visual(
        Box((0.82, 0.08, 0.08)),
        origin=Origin(xyz=(0.41, 1.70, -0.31)),
        material=dark_steel,
        name="left_stop_lip",
    )
    shore_frame.visual(
        Box((0.82, 0.08, 0.08)),
        origin=Origin(xyz=(0.41, -1.70, -0.31)),
        material=dark_steel,
        name="right_stop_lip",
    )
    shore_frame.visual(
        Box((0.08, 3.40, 0.08)),
        origin=Origin(xyz=(0.86, 0.0, -0.31)),
        material=dark_steel,
        name="toe_stop_lip",
    )
    shore_frame.visual(
        Box((0.42, 4.3, 0.06)),
        origin=Origin(xyz=(-0.41, 0.0, deck_top_z - 0.03)),
        material=deck_paving,
        name="approach_wearing_course",
    )
    shore_frame.visual(
        Box((0.06, 3.90, 0.03)),
        origin=Origin(xyz=(-0.22, 0.0, deck_top_z + 0.015)),
        material=safety_yellow,
        name="hinge_threshold_stripe",
    )
    shore_frame.inertial = Inertial.from_geometry(
        Box((2.8, 5.0, 1.15)),
        mass=24000.0,
        origin=Origin(xyz=(-1.25, 0.0, -0.10)),
    )

    bridge_leaf = model.part("bridge_leaf")
    bridge_leaf.visual(
        Box((leaf_length, leaf_width, 0.10)),
        origin=Origin(xyz=(leaf_length * 0.5, 0.0, deck_top_z - 0.05)),
        material=deck_paving,
        name="deck_plate",
    )
    bridge_leaf.visual(
        Box((leaf_length, girder_width, girder_depth)),
        origin=Origin(
            xyz=(leaf_length * 0.5, (leaf_width * 0.5) - (girder_width * 0.5), -0.025)
        ),
        material=steel,
        name="left_main_girder",
    )
    bridge_leaf.visual(
        Box((leaf_length, girder_width, girder_depth)),
        origin=Origin(
            xyz=(leaf_length * 0.5, -(leaf_width * 0.5) + (girder_width * 0.5), -0.025)
        ),
        material=steel,
        name="right_main_girder",
    )
    bridge_leaf.visual(
        Box((0.42, 3.60, 0.45)),
        origin=Origin(xyz=(0.21, 0.0, -0.02)),
        material=dark_steel,
        name="hinge_beam",
    )
    bridge_leaf.visual(
        Box((0.24, 4.00, 0.40)),
        origin=Origin(xyz=(leaf_length - 0.12, 0.0, -0.01)),
        material=dark_steel,
        name="toe_beam",
    )
    for index, x_pos in enumerate((2.0, 4.0, 6.0), start=1):
        bridge_leaf.visual(
            Box((0.16, 3.72, 0.35)),
            origin=Origin(xyz=(x_pos, 0.0, -0.02)),
            material=steel,
            name=f"cross_girder_{index}",
        )
    bridge_leaf.visual(
        Box((0.30, 0.24, 0.32)),
        origin=Origin(xyz=(0.15, 1.94, 0.0)),
        material=dark_steel,
        name="left_trunnion_gusset",
    )
    bridge_leaf.visual(
        Box((0.30, 0.24, 0.32)),
        origin=Origin(xyz=(0.15, -1.94, 0.0)),
        material=dark_steel,
        name="right_trunnion_gusset",
    )
    bridge_leaf.visual(
        Cylinder(radius=trunnion_radius, length=0.18),
        origin=Origin(xyz=(0.04, 1.93, hinge_z), rpy=(1.57079632679, 0.0, 0.0)),
        material=steel,
        name="left_trunnion_drum",
    )
    bridge_leaf.visual(
        Cylinder(radius=trunnion_radius, length=0.18),
        origin=Origin(xyz=(0.04, -1.93, hinge_z), rpy=(1.57079632679, 0.0, 0.0)),
        material=steel,
        name="right_trunnion_drum",
    )
    bridge_leaf.visual(
        Box((leaf_length - 0.35, 0.10, 0.16)),
        origin=Origin(xyz=(leaf_length * 0.5 + 0.175, 1.72, deck_top_z - 0.02)),
        material=safety_yellow,
        name="left_edge_beam",
    )
    bridge_leaf.visual(
        Box((leaf_length - 0.35, 0.10, 0.16)),
        origin=Origin(xyz=(leaf_length * 0.5 + 0.175, -1.72, deck_top_z - 0.02)),
        material=safety_yellow,
        name="right_edge_beam",
    )
    bridge_leaf.inertial = Inertial.from_geometry(
        Box((leaf_length, leaf_width, 0.75)),
        mass=13000.0,
        origin=Origin(xyz=(leaf_length * 0.5, 0.0, -0.02)),
    )

    model.articulation(
        "shore_to_leaf",
        ArticulationType.REVOLUTE,
        parent=shore_frame,
        child=bridge_leaf,
        origin=Origin(xyz=(hinge_x, 0.0, hinge_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=600000.0,
            velocity=0.35,
            lower=0.0,
            upper=1.20,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    shore_frame = object_model.get_part("shore_frame")
    bridge_leaf = object_model.get_part("bridge_leaf")
    hinge = object_model.get_articulation("shore_to_leaf")

    with ctx.pose({hinge: 0.0}):
        ctx.expect_gap(
            bridge_leaf,
            shore_frame,
            axis="z",
            positive_elem="hinge_beam",
            negative_elem="left_stop_lip",
            min_gap=0.0,
            max_gap=0.04,
            name="left stop lip stays just below the closed hinge beam",
        )
        ctx.expect_gap(
            bridge_leaf,
            shore_frame,
            axis="z",
            positive_elem="hinge_beam",
            negative_elem="right_stop_lip",
            min_gap=0.0,
            max_gap=0.04,
            name="right stop lip stays just below the closed hinge beam",
        )
        ctx.expect_gap(
            bridge_leaf,
            shore_frame,
            axis="z",
            positive_elem="deck_plate",
            negative_elem="toe_stop_lip",
            min_gap=0.30,
            max_gap=0.40,
            name="toe stop flange sits below the closed leaf deck",
        )

    closed_toe_aabb = ctx.part_element_world_aabb(bridge_leaf, elem="toe_beam")
    with ctx.pose({hinge: 1.0}):
        open_toe_aabb = ctx.part_element_world_aabb(bridge_leaf, elem="toe_beam")

    closed_toe_z = None
    open_toe_z = None
    if closed_toe_aabb is not None:
        closed_toe_z = 0.5 * (closed_toe_aabb[0][2] + closed_toe_aabb[1][2])
    if open_toe_aabb is not None:
        open_toe_z = 0.5 * (open_toe_aabb[0][2] + open_toe_aabb[1][2])
    ctx.check(
        "bridge leaf rotates upward in the open pose",
        closed_toe_z is not None
        and open_toe_z is not None
        and open_toe_z > closed_toe_z + 5.0,
        details=f"closed_toe_z={closed_toe_z}, open_toe_z={open_toe_z}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
