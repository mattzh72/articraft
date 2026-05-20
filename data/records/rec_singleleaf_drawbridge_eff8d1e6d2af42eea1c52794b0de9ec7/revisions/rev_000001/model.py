from __future__ import annotations

from math import pi

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

    model.material("painted_steel", rgba=(0.18, 0.21, 0.23, 1.0))
    model.material("galvanized", rgba=(0.64, 0.67, 0.68, 1.0))
    model.material("dark_machined", rgba=(0.07, 0.08, 0.08, 1.0))
    model.material("safety_yellow", rgba=(0.95, 0.71, 0.12, 1.0))
    model.material("wear_steel", rgba=(0.50, 0.48, 0.43, 1.0))
    model.material("rubber_black", rgba=(0.02, 0.02, 0.018, 1.0))
    model.material("maintenance_blue", rgba=(0.10, 0.28, 0.45, 1.0))

    frame = model.part("support_frame")

    # A welded service frame: base skid, bearing pedestals, side pillow-block
    # yokes, and a rear maintenance platform.  Every visual overlaps or bears on
    # another frame member so the root reads as one connected field-service
    # assembly rather than a collection of floating details.
    frame.visual(
        Box((1.70, 3.45, 0.20)),
        origin=Origin(xyz=(-0.36, 0.0, 0.10)),
        material="painted_steel",
        name="base_skid",
    )
    frame.visual(
        Box((0.82, 2.85, 0.08)),
        origin=Origin(xyz=(-0.70, 0.0, 0.86)),
        material="galvanized",
        name="service_deck",
    )
    frame.visual(
        Box((0.12, 2.70, 0.72)),
        origin=Origin(xyz=(-0.30, 0.0, 0.51)),
        material="painted_steel",
        name="rear_cross_web",
    )
    frame.visual(
        Box((0.15, 2.90, 0.15)),
        origin=Origin(xyz=(-0.72, 0.0, 0.175)),
        material="painted_steel",
        name="low_cross_tie",
    )

    # Rear access guard rail built from chunky rectangular tube.
    for y_pos in (-1.28, -0.43, 0.43, 1.28):
        frame.visual(
            Box((0.06, 0.06, 0.66)),
            origin=Origin(xyz=(-1.02, y_pos, 1.18)),
            material="safety_yellow",
            name=f"rail_post_{len(frame.visuals)}",
        )
    frame.visual(
        Box((0.07, 2.74, 0.07)),
        origin=Origin(xyz=(-1.02, 0.0, 1.50)),
        material="safety_yellow",
        name="top_guard_rail",
    )
    frame.visual(
        Box((0.06, 2.74, 0.06)),
        origin=Origin(xyz=(-1.02, 0.0, 1.23)),
        material="safety_yellow",
        name="mid_guard_rail",
    )

    # Two heavy side bearing yokes.  The open rectangular windows clear the
    # rotating trunnion shaft while the caps and cheeks show replaceable split
    # bearing hardware.
    frame.visual(
        Box((0.58, 0.46, 0.56)),
        origin=Origin(xyz=(0.0, -1.30, 0.42)),
        material="painted_steel",
        name="side_0_pedestal",
    )
    frame.visual(
        Box((0.74, 0.52, 0.22)),
        origin=Origin(xyz=(0.0, -1.30, 0.69)),
        material="galvanized",
        name="side_0_lower_bearing",
    )
    frame.visual(
        Box((0.28, 0.42, 0.04)),
        origin=Origin(xyz=(0.0, -1.30, 0.80)),
        material="wear_steel",
        name="side_0_lower_liner",
    )
    frame.visual(
        Box((0.17, 0.52, 0.62)),
        origin=Origin(xyz=(-0.35, -1.30, 1.00)),
        material="painted_steel",
        name="side_0_rear_cheek",
    )
    frame.visual(
        Box((0.17, 0.52, 0.62)),
        origin=Origin(xyz=(0.35, -1.30, 1.00)),
        material="painted_steel",
        name="side_0_front_cheek",
    )
    frame.visual(
        Box((0.74, 0.52, 0.22)),
        origin=Origin(xyz=(0.0, -1.30, 1.34)),
        material="galvanized",
        name="side_0_split_cap",
    )
    frame.visual(
        Box((0.62, 0.40, 0.035)),
        origin=Origin(xyz=(0.0, -1.30, 1.465)),
        material="maintenance_blue",
        name="side_0_cap_cover",
    )
    for x_pos in (-0.24, 0.24):
        for y_offset in (-0.15, 0.15):
            frame.visual(
                Cylinder(radius=0.035, length=0.045),
                origin=Origin(xyz=(x_pos, -1.30 + y_offset, 1.503)),
                material="dark_machined",
                name=f"side_0_cap_bolt_{x_pos}_{y_offset}",
            )
    frame.visual(
        Cylinder(radius=0.022, length=0.10),
        origin=Origin(xyz=(0.0, -1.30, 1.532)),
        material="dark_machined",
        name="side_0_grease_stem",
    )
    frame.visual(
        Cylinder(radius=0.040, length=0.035),
        origin=Origin(xyz=(0.0, -1.30, 1.598)),
        material="dark_machined",
        name="side_0_grease_cap",
    )

    frame.visual(
        Box((0.58, 0.46, 0.56)),
        origin=Origin(xyz=(0.0, 1.30, 0.42)),
        material="painted_steel",
        name="side_1_pedestal",
    )
    frame.visual(
        Box((0.74, 0.52, 0.22)),
        origin=Origin(xyz=(0.0, 1.30, 0.69)),
        material="galvanized",
        name="side_1_lower_bearing",
    )
    frame.visual(
        Box((0.28, 0.42, 0.04)),
        origin=Origin(xyz=(0.0, 1.30, 0.80)),
        material="wear_steel",
        name="side_1_lower_liner",
    )
    frame.visual(
        Box((0.17, 0.52, 0.62)),
        origin=Origin(xyz=(-0.35, 1.30, 1.00)),
        material="painted_steel",
        name="side_1_rear_cheek",
    )
    frame.visual(
        Box((0.17, 0.52, 0.62)),
        origin=Origin(xyz=(0.35, 1.30, 1.00)),
        material="painted_steel",
        name="side_1_front_cheek",
    )
    frame.visual(
        Box((0.74, 0.52, 0.22)),
        origin=Origin(xyz=(0.0, 1.30, 1.34)),
        material="galvanized",
        name="side_1_split_cap",
    )
    frame.visual(
        Box((0.62, 0.40, 0.035)),
        origin=Origin(xyz=(0.0, 1.30, 1.465)),
        material="maintenance_blue",
        name="side_1_cap_cover",
    )
    for x_pos in (-0.24, 0.24):
        for y_offset in (-0.15, 0.15):
            frame.visual(
                Cylinder(radius=0.035, length=0.045),
                origin=Origin(xyz=(x_pos, 1.30 + y_offset, 1.503)),
                material="dark_machined",
                name=f"side_1_cap_bolt_{x_pos}_{y_offset}",
            )
    frame.visual(
        Cylinder(radius=0.022, length=0.10),
        origin=Origin(xyz=(0.0, 1.30, 1.532)),
        material="dark_machined",
        name="side_1_grease_stem",
    )
    frame.visual(
        Cylinder(radius=0.040, length=0.035),
        origin=Origin(xyz=(0.0, 1.30, 1.598)),
        material="dark_machined",
        name="side_1_grease_cap",
    )

    leaf = model.part("bridge_leaf")

    # The child frame is the trunnion centerline.  Closed geometry extends along
    # local +X, so positive rotation about -Y raises the free end.
    leaf.visual(
        Cylinder(radius=0.18, length=3.08),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material="dark_machined",
        name="trunnion_shaft",
    )
    leaf.visual(
        Cylinder(radius=0.245, length=1.80),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material="painted_steel",
        name="torque_tube",
    )
    leaf.visual(
        Box((5.30, 2.04, 0.16)),
        origin=Origin(xyz=(2.75, 0.0, -0.05)),
        material="painted_steel",
        name="deck_plate",
    )
    leaf.visual(
        Box((5.15, 0.18, 0.34)),
        origin=Origin(xyz=(2.82, -0.72, -0.20)),
        material="painted_steel",
        name="girder_0",
    )
    leaf.visual(
        Box((5.15, 0.18, 0.34)),
        origin=Origin(xyz=(2.82, 0.72, -0.20)),
        material="painted_steel",
        name="girder_1",
    )
    for x_pos in (0.35, 1.25, 2.15, 3.05, 3.95, 4.85):
        leaf.visual(
            Box((0.14, 2.00, 0.24)),
            origin=Origin(xyz=(x_pos, 0.0, -0.22)),
            material="painted_steel",
            name=f"crossmember_{x_pos}",
        )
    for y_pos in (-0.84, -0.42, 0.0, 0.42, 0.84):
        leaf.visual(
            Box((0.34, 0.09, 0.34)),
            origin=Origin(xyz=(0.20, y_pos, -0.09)),
            material="painted_steel",
            name=f"hinge_web_{y_pos}",
        )
    for y_pos in (-0.92, 0.92):
        leaf.visual(
            Box((4.65, 0.12, 0.22)),
            origin=Origin(xyz=(2.98, y_pos, 0.10)),
            material="safety_yellow",
            name=f"side_curb_{y_pos}",
        )
    for y_pos in (-0.45, 0.45):
        leaf.visual(
            Box((4.60, 0.34, 0.035)),
            origin=Origin(xyz=(2.95, y_pos, 0.045)),
            material="wear_steel",
            name=f"replaceable_wear_lane_{y_pos}",
        )
    for x_pos in (1.00, 2.05, 3.10, 4.15):
        for y_pos in (-0.58, -0.32, 0.32, 0.58):
            leaf.visual(
                Cylinder(radius=0.026, length=0.030),
                origin=Origin(xyz=(x_pos, y_pos, 0.075)),
                material="dark_machined",
                name=f"wear_bolt_{x_pos}_{y_pos}",
            )
    leaf.visual(
        Box((0.18, 1.72, 0.14)),
        origin=Origin(xyz=(5.35, 0.0, -0.02)),
        material="rubber_black",
        name="nose_bumper",
    )

    frame.inertial = Inertial.from_geometry(
        Box((1.70, 3.45, 1.65)),
        mass=5200.0,
        origin=Origin(xyz=(-0.20, 0.0, 0.82)),
    )
    leaf.inertial = Inertial.from_geometry(
        Box((5.50, 2.10, 0.55)),
        mass=3800.0,
        origin=Origin(xyz=(2.75, 0.0, -0.08)),
    )

    model.articulation(
        "leaf_pivot",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=leaf,
        origin=Origin(xyz=(0.0, 0.0, 1.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=1.20, effort=85000.0, velocity=0.18),
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
    # If overlap QC reports an intersection, classify it first: intentional
    # embeddings or nested fits should get a scoped allowance; unintended
    # collisions should be fixed in geometry, support, mount, or pose.

    frame = object_model.get_part("support_frame")
    leaf = object_model.get_part("bridge_leaf")
    pivot = object_model.get_articulation("leaf_pivot")

    with ctx.pose({pivot: 0.0}):
        ctx.expect_overlap(
            leaf,
            frame,
            axes="y",
            elem_a="trunnion_shaft",
            elem_b="side_0_split_cap",
            min_overlap=0.32,
            name="trunnion reaches side 0 bearing",
        )
        ctx.expect_overlap(
            leaf,
            frame,
            axes="y",
            elem_a="trunnion_shaft",
            elem_b="side_1_split_cap",
            min_overlap=0.32,
            name="trunnion reaches side 1 bearing",
        )
        ctx.expect_contact(
            leaf,
            frame,
            elem_a="trunnion_shaft",
            elem_b="side_0_lower_liner",
            contact_tol=0.002,
            name="side 0 liner supports trunnion",
        )
        ctx.expect_contact(
            leaf,
            frame,
            elem_a="trunnion_shaft",
            elem_b="side_1_lower_liner",
            contact_tol=0.002,
            name="side 1 liner supports trunnion",
        )
        ctx.expect_gap(
            leaf,
            frame,
            axis="z",
            positive_elem="trunnion_shaft",
            negative_elem="side_0_lower_bearing",
            min_gap=0.015,
            max_gap=0.055,
            name="shaft clears lower bearing saddle",
        )
        ctx.expect_gap(
            frame,
            leaf,
            axis="z",
            positive_elem="side_0_split_cap",
            negative_elem="trunnion_shaft",
            min_gap=0.020,
            max_gap=0.080,
            name="shaft clears removable bearing cap",
        )
        ctx.expect_gap(
            leaf,
            frame,
            axis="x",
            positive_elem="trunnion_shaft",
            negative_elem="side_0_rear_cheek",
            min_gap=0.030,
            max_gap=0.120,
            name="shaft clears rear bearing cheek",
        )
        ctx.expect_gap(
            frame,
            leaf,
            axis="x",
            positive_elem="side_0_front_cheek",
            negative_elem="trunnion_shaft",
            min_gap=0.030,
            max_gap=0.120,
            name="shaft clears front bearing cheek",
        )

    closed_pos = ctx.part_world_position(leaf)
    with ctx.pose({pivot: 1.05}):
        raised_pos = ctx.part_world_position(leaf)
        raised_aabb = ctx.part_world_aabb(leaf)
    ctx.check(
        "bridge leaf raises on positive pivot",
        closed_pos is not None
        and raised_pos is not None
        and raised_aabb is not None
        and raised_aabb[1][2] > 3.8,
        details=f"closed={closed_pos}, raised={raised_pos}, raised_aabb={raised_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
