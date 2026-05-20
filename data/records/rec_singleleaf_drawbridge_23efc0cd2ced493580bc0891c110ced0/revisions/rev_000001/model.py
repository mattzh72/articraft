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

    frame_steel = model.material("frame_steel", rgba=(0.23, 0.25, 0.28, 1.0))
    deck_steel = model.material("deck_steel", rgba=(0.39, 0.42, 0.45, 1.0))
    asphalt = model.material("asphalt", rgba=(0.11, 0.11, 0.12, 1.0))
    bearing_metal = model.material("bearing_metal", rgba=(0.62, 0.64, 0.67, 1.0))
    safety_yellow = model.material("safety_yellow", rgba=(0.86, 0.74, 0.12, 1.0))

    leaf_length = 7.2
    leaf_width = 4.28
    half_leaf_width = leaf_width * 0.5
    frame_inner_half_width = 2.25

    shore_frame = model.part("shore_frame")
    shore_frame.visual(
        Box((1.10, 5.35, 0.60)),
        origin=Origin(xyz=(0.0, 0.0, -1.15)),
        material=frame_steel,
        name="lower_sill",
    )
    shore_frame.visual(
        Box((1.10, 0.45, 9.20)),
        origin=Origin(xyz=(0.0, -2.675, 3.00)),
        material=frame_steel,
        name="left_column",
    )
    shore_frame.visual(
        Box((1.10, 0.45, 9.20)),
        origin=Origin(xyz=(0.0, 2.675, 3.00)),
        material=frame_steel,
        name="right_column",
    )
    shore_frame.visual(
        Box((1.10, 5.35, 0.50)),
        origin=Origin(xyz=(0.0, 0.0, 7.75)),
        material=frame_steel,
        name="header_beam",
    )
    shore_frame.visual(
        Box((0.75, 0.20, 1.15)),
        origin=Origin(xyz=(0.0, -2.35, 0.05)),
        material=frame_steel,
        name="left_bearing_pedestal",
    )
    shore_frame.visual(
        Box((0.75, 0.20, 1.15)),
        origin=Origin(xyz=(0.0, 2.35, 0.05)),
        material=frame_steel,
        name="right_bearing_pedestal",
    )
    shore_frame.visual(
        Cylinder(radius=0.24, length=0.16),
        origin=Origin(xyz=(0.0, -2.28, 0.0), rpy=(1.5707963267948966, 0.0, 0.0)),
        material=bearing_metal,
        name="left_bearing_cap",
    )
    shore_frame.visual(
        Cylinder(radius=0.24, length=0.16),
        origin=Origin(xyz=(0.0, 2.28, 0.0), rpy=(1.5707963267948966, 0.0, 0.0)),
        material=bearing_metal,
        name="right_bearing_cap",
    )
    shore_frame.inertial = Inertial.from_geometry(
        Box((1.20, 5.40, 9.40)),
        mass=18000.0,
        origin=Origin(xyz=(0.0, 0.0, 3.00)),
    )

    bridge_leaf = model.part("bridge_leaf")
    bridge_leaf.visual(
        Box((leaf_length, leaf_width, 0.12)),
        origin=Origin(xyz=(leaf_length * 0.5, 0.0, 0.0)),
        material=deck_steel,
        name="deck_plate",
    )
    bridge_leaf.visual(
        Box((leaf_length, 0.20, 0.44)),
        origin=Origin(xyz=(leaf_length * 0.5, -(half_leaf_width - 0.10), -0.16)),
        material=deck_steel,
        name="left_girder",
    )
    bridge_leaf.visual(
        Box((leaf_length, 0.20, 0.44)),
        origin=Origin(xyz=(leaf_length * 0.5, half_leaf_width - 0.10, -0.16)),
        material=deck_steel,
        name="right_girder",
    )
    bridge_leaf.visual(
        Box((0.35, leaf_width, 0.36)),
        origin=Origin(xyz=(0.175, 0.0, -0.12)),
        material=deck_steel,
        name="hinge_beam",
    )
    for rib_index, rib_x in enumerate((1.8, 3.6, 5.4), start=1):
        bridge_leaf.visual(
            Box((0.16, leaf_width - 0.40, 0.24)),
            origin=Origin(xyz=(rib_x, 0.0, -0.10)),
            material=deck_steel,
            name=f"cross_rib_{rib_index}",
        )
    bridge_leaf.visual(
        Box((6.85, 3.72, 0.03)),
        origin=Origin(xyz=(3.62, 0.0, 0.075)),
        material=asphalt,
        name="road_surface",
    )
    bridge_leaf.visual(
        Box((6.85, 0.08, 0.12)),
        origin=Origin(xyz=(3.62, -(half_leaf_width - 0.32), 0.09)),
        material=safety_yellow,
        name="left_curb",
    )
    bridge_leaf.visual(
        Box((6.85, 0.08, 0.12)),
        origin=Origin(xyz=(3.62, half_leaf_width - 0.32, 0.09)),
        material=safety_yellow,
        name="right_curb",
    )
    bridge_leaf.visual(
        Box((0.28, leaf_width, 0.08)),
        origin=Origin(xyz=(leaf_length - 0.14, 0.0, 0.08)),
        material=deck_steel,
        name="toe_plate",
    )
    bridge_leaf.visual(
        Cylinder(radius=0.16, length=0.06),
        origin=Origin(xyz=(0.10, -(half_leaf_width + 0.03), -0.02), rpy=(1.5707963267948966, 0.0, 0.0)),
        material=bearing_metal,
        name="left_trunnion",
    )
    bridge_leaf.visual(
        Cylinder(radius=0.16, length=0.06),
        origin=Origin(xyz=(0.10, half_leaf_width + 0.03, -0.02), rpy=(1.5707963267948966, 0.0, 0.0)),
        material=bearing_metal,
        name="right_trunnion",
    )
    bridge_leaf.inertial = Inertial.from_geometry(
        Box((leaf_length, leaf_width, 0.60)),
        mass=12000.0,
        origin=Origin(xyz=(leaf_length * 0.5, 0.0, -0.10)),
    )

    model.articulation(
        "shore_to_leaf",
        ArticulationType.REVOLUTE,
        parent=shore_frame,
        child=bridge_leaf,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=250000.0,
            velocity=0.45,
            lower=0.0,
            upper=1.25,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    leaf_width = 4.28
    shore_frame = object_model.get_part("shore_frame")
    bridge_leaf = object_model.get_part("bridge_leaf")
    hinge = object_model.get_articulation("shore_to_leaf")

    def _elem_aabb(part_name: str, elem_name: str):
        return ctx.part_element_world_aabb(part_name, elem=elem_name)

    left_column = _elem_aabb("shore_frame", "left_column")
    right_column = _elem_aabb("shore_frame", "right_column")
    leaf_box = ctx.part_world_aabb(bridge_leaf)

    centered_ok = False
    centered_details = "missing AABB data"
    if left_column is not None and right_column is not None and leaf_box is not None:
        left_inner_face = left_column[1][1]
        right_inner_face = right_column[0][1]
        left_margin = leaf_box[0][1] - left_inner_face
        right_margin = right_inner_face - leaf_box[1][1]
        centered_ok = left_margin > 0.05 and right_margin > 0.05 and abs(left_margin - right_margin) < 0.01
        centered_details = (
            f"left_margin={left_margin:.4f}, right_margin={right_margin:.4f}, "
            "target_symmetry_tol=0.01"
        )
    ctx.check("leaf centered between frame columns", centered_ok, centered_details)

    ctx.expect_gap(
        bridge_leaf,
        shore_frame,
        axis="z",
        positive_elem="deck_plate",
        negative_elem="lower_sill",
        min_gap=0.40,
        name="closed leaf clears lower sill",
    )

    toe_rest = ctx.part_element_world_aabb("bridge_leaf", elem="toe_plate")
    with ctx.pose({hinge: 1.10}):
        toe_open = ctx.part_element_world_aabb("bridge_leaf", elem="toe_plate")
        open_ok = False
        open_details = f"rest={toe_rest}, open={toe_open}"
        if toe_rest is not None and toe_open is not None:
            rest_center_x = 0.5 * (toe_rest[0][0] + toe_rest[1][0])
            rest_center_z = 0.5 * (toe_rest[0][2] + toe_rest[1][2])
            open_center_x = 0.5 * (toe_open[0][0] + toe_open[1][0])
            open_center_z = 0.5 * (toe_open[0][2] + toe_open[1][2])
            open_ok = open_center_z > rest_center_z + 5.0 and open_center_x < rest_center_x - 3.5
            open_details = (
                f"rest_center=({rest_center_x:.3f}, {rest_center_z:.3f}), "
                f"open_center=({open_center_x:.3f}, {open_center_z:.3f})"
            )
        ctx.check("leaf rotates upward into opening pose", open_ok, open_details)

        ctx.expect_gap(
            shore_frame,
            bridge_leaf,
            axis="z",
            positive_elem="header_beam",
            negative_elem="toe_plate",
            min_gap=0.20,
            name="opened toe plate stays below header",
        )

    ctx.expect_overlap(
        bridge_leaf,
        shore_frame,
        axes="y",
        elem_a="deck_plate",
        elem_b="header_beam",
        min_overlap=leaf_width,
        name="leaf remains centered under frame span",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
