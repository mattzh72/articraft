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

    concrete = model.material("concrete", rgba=(0.63, 0.65, 0.66, 1.0))
    asphalt = model.material("asphalt", rgba=(0.15, 0.15, 0.16, 1.0))
    painted_steel = model.material("painted_steel", rgba=(0.33, 0.37, 0.40, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.20, 0.21, 0.23, 1.0))
    curb_yellow = model.material("curb_yellow", rgba=(0.73, 0.67, 0.24, 1.0))

    shore_frame = model.part("shore_frame")
    shore_frame.visual(
        Box((3.40, 5.60, 3.24)),
        origin=Origin(xyz=(-1.70, 0.0, 1.62)),
        material=concrete,
        name="abutment_core",
    )
    shore_frame.visual(
        Box((3.40, 4.40, 0.18)),
        origin=Origin(xyz=(-1.70, 0.0, 3.33)),
        material=asphalt,
        name="roadway_surface",
    )
    shore_frame.visual(
        Box((0.36, 5.60, 0.60)),
        origin=Origin(xyz=(-0.18, 0.0, 2.94)),
        material=concrete,
        name="cutwater_face",
    )

    for side_sign, side_name in ((1.0, "left"), (-1.0, "right")):
        shore_frame.visual(
            Box((0.90, 0.50, 0.72)),
            origin=Origin(xyz=(-0.15, side_sign * 2.30, 3.60)),
            material=painted_steel,
            name=f"{side_name}_bearing_pedestal",
        )
        shore_frame.visual(
            Box((0.72, 0.16, 0.80)),
            origin=Origin(xyz=(0.06, side_sign * 2.22, 3.12)),
            material=dark_steel,
            name=f"{side_name}_side_cheek",
        )
        shore_frame.visual(
            Cylinder(radius=0.24, length=0.18),
            origin=Origin(
                xyz=(0.20, side_sign * 2.30, 3.00),
                rpy=(1.5707963267948966, 0.0, 0.0),
            ),
            material=dark_steel,
            name=f"{side_name}_bearing_housing",
        )
        shore_frame.visual(
            Box((0.38, 0.18, 0.32)),
            origin=Origin(xyz=(0.24, side_sign * 2.30, 2.92)),
            material=dark_steel,
            name=f"{side_name}_bearing_cap",
        )

    shore_frame.inertial = Inertial.from_geometry(
        Box((3.40, 5.60, 4.00)),
        mass=85000.0,
        origin=Origin(xyz=(-1.70, 0.0, 2.00)),
    )

    bridge_leaf = model.part("bridge_leaf")
    bridge_leaf.visual(
        Box((11.00, 4.00, 0.12)),
        origin=Origin(xyz=(5.30, 0.0, 0.36)),
        material=asphalt,
        name="leaf_surface",
    )
    bridge_leaf.visual(
        Box((11.00, 4.00, 0.16)),
        origin=Origin(xyz=(5.30, 0.0, 0.26)),
        material=painted_steel,
        name="deck_plate",
    )
    bridge_leaf.visual(
        Box((0.88, 3.68, 0.52)),
        origin=Origin(xyz=(0.34, 0.0, 0.00)),
        material=dark_steel,
        name="heel_crossbeam",
    )
    bridge_leaf.visual(
        Cylinder(radius=0.20, length=3.84),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(1.5707963267948966, 0.0, 0.0)),
        material=dark_steel,
        name="trunnion_tube",
    )
    for side_sign, side_name in ((1.0, "left"), (-1.0, "right")):
        bridge_leaf.visual(
            Box((10.80, 0.22, 0.78)),
            origin=Origin(xyz=(5.38, side_sign * 1.89, -0.13)),
            material=dark_steel,
            name=f"{side_name}_main_girder",
        )
        bridge_leaf.visual(
            Box((10.80, 0.12, 0.10)),
            origin=Origin(xyz=(5.38, side_sign * 1.94, 0.47)),
            material=curb_yellow,
            name=f"{side_name}_curb",
        )
    for index, beam_x in enumerate((1.20, 3.40, 5.60, 7.80, 9.80)):
        bridge_leaf.visual(
            Box((0.22, 3.56, 0.18)),
            origin=Origin(xyz=(beam_x, 0.0, -0.02)),
            material=painted_steel,
            name=f"floorbeam_{index}",
        )
    bridge_leaf.visual(
        Box((0.56, 3.96, 0.20)),
        origin=Origin(xyz=(10.62, 0.0, 0.18)),
        material=painted_steel,
        name="tip_nose",
    )
    bridge_leaf.inertial = Inertial.from_geometry(
        Box((11.20, 4.00, 1.10)),
        mass=26000.0,
        origin=Origin(xyz=(5.20, 0.0, 0.05)),
    )

    model.articulation(
        "leaf_hinge",
        ArticulationType.REVOLUTE,
        parent=shore_frame,
        child=bridge_leaf,
        origin=Origin(xyz=(0.20, 0.0, 3.00)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=220000.0,
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
    leaf_hinge = object_model.get_articulation("leaf_hinge")

    ctx.expect_gap(
        bridge_leaf,
        shore_frame,
        axis="x",
        positive_elem="leaf_surface",
        negative_elem="roadway_surface",
        min_gap=0.0,
        max_gap=0.002,
        name="closed leaf meets the roadway apron at the hinge line",
    )
    ctx.expect_overlap(
        bridge_leaf,
        shore_frame,
        axes="y",
        elem_a="leaf_surface",
        elem_b="roadway_surface",
        min_overlap=3.9,
        name="closed leaf aligns with the shore roadway width",
    )

    closed_tip = ctx.part_element_world_aabb(bridge_leaf, elem="tip_nose")
    with ctx.pose({leaf_hinge: 1.20}):
        ctx.expect_gap(
            bridge_leaf,
            shore_frame,
            axis="z",
            positive_elem="tip_nose",
            negative_elem="roadway_surface",
            min_gap=8.0,
            name="opened leaf tip rises well above the shore frame",
        )
        opened_tip = ctx.part_element_world_aabb(bridge_leaf, elem="tip_nose")

    tip_rise_ok = (
        closed_tip is not None
        and opened_tip is not None
        and opened_tip[1][2] > closed_tip[1][2] + 8.0
    )
    ctx.check(
        "leaf rotates upward about the shore-side bearings",
        tip_rise_ok,
        details=f"closed_tip={closed_tip}, opened_tip={opened_tip}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
