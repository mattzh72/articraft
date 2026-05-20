from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    ConeGeometry,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="low_picket_garden_gate")

    weathered_wood = model.material("weathered_green_wood", rgba=(0.33, 0.49, 0.34, 1.0))
    post_wood = model.material("cedar_posts", rgba=(0.55, 0.39, 0.25, 1.0))
    dark_iron = model.material("black_iron", rgba=(0.02, 0.025, 0.025, 1.0))
    galvanized = model.material("galvanized_steel", rgba=(0.62, 0.64, 0.62, 1.0))
    earth = model.material("garden_ground", rgba=(0.25, 0.34, 0.18, 1.0))

    picket_cap_mesh = mesh_from_geometry(
        ConeGeometry(radius=0.032, height=0.080, radial_segments=4),
        "picket_cap",
    )
    ring_mesh = mesh_from_geometry(
        TorusGeometry(radius=0.055, tube=0.0065, radial_segments=18, tubular_segments=54),
        "ring_latch_loop",
    )

    frame = model.part("post_frame")
    frame.visual(
        Box((1.65, 1.40, 0.035)),
        origin=Origin(xyz=(0.58, 0.42, 0.0175)),
        material=earth,
        name="ground_strip",
    )
    for post_name, x_pos in (("hinge_post", -0.12), ("latch_post", 1.24)):
        frame.visual(
            Box((0.12, 0.12, 1.12)),
            origin=Origin(xyz=(x_pos, 0.0, 0.56)),
            material=post_wood,
            name=post_name,
        )
        frame.visual(
            Box((0.15, 0.15, 0.05)),
            origin=Origin(xyz=(x_pos, 0.0, 1.145)),
            material=post_wood,
            name=f"{post_name}_cap",
        )

    frame.visual(
        Box((0.055, 0.012, 0.12)),
        origin=Origin(xyz=(-0.0455, -0.052, 0.40)),
        material=dark_iron,
        name="lower_hinge_post_leaf",
    )
    frame.visual(
        Box((0.055, 0.012, 0.12)),
        origin=Origin(xyz=(-0.0455, -0.052, 0.74)),
        material=dark_iron,
        name="upper_hinge_post_leaf",
    )
    frame.visual(
        Box((0.035, 0.018, 0.12)),
        origin=Origin(xyz=(1.170, -0.050, 0.79)),
        material=dark_iron,
        name="latch_keeper_plate",
    )
    frame.visual(
        Box((0.060, 0.014, 0.018)),
        origin=Origin(xyz=(1.125, -0.065, 0.79)),
        material=dark_iron,
        name="latch_keeper_lip",
    )
    frame.visual(
        Cylinder(radius=0.045, length=0.012),
        origin=Origin(xyz=(0.175, 0.896, 0.040)),
        material=dark_iron,
        name="open_surface_socket",
    )
    frame.visual(
        Cylinder(radius=0.038, length=0.010),
        origin=Origin(xyz=(0.910, -0.065, 0.039)),
        material=dark_iron,
        name="closed_surface_socket",
    )

    leaf = model.part("gate_leaf")
    leaf.visual(
        Box((0.055, 0.050, 0.78)),
        origin=Origin(xyz=(0.045, 0.0, 0.39)),
        material=weathered_wood,
        name="hinge_stile",
    )
    leaf.visual(
        Box((0.055, 0.050, 0.78)),
        origin=Origin(xyz=(1.035, 0.0, 0.39)),
        material=weathered_wood,
        name="latch_stile",
    )
    leaf.visual(
        Box((1.04, 0.055, 0.065)),
        origin=Origin(xyz=(0.540, 0.0, 0.66)),
        material=weathered_wood,
        name="upper_rail",
    )
    leaf.visual(
        Box((1.04, 0.055, 0.065)),
        origin=Origin(xyz=(0.540, 0.0, 0.18)),
        material=weathered_wood,
        name="lower_rail",
    )
    leaf.visual(
        Cylinder(radius=0.025, length=0.96),
        origin=Origin(xyz=(0.55, 0.0, 0.415), rpy=(0.0, 1.107, 0.0)),
        material=weathered_wood,
        name="diagonal_brace",
    )
    for index, x_pos in enumerate((0.20, 0.38, 0.56, 0.74, 0.92)):
        leaf.visual(
            Box((0.042, 0.045, 0.68)),
            origin=Origin(xyz=(x_pos, 0.0, 0.43)),
            material=weathered_wood,
            name=f"picket_{index}",
        )
        leaf.visual(
            picket_cap_mesh,
            origin=Origin(xyz=(x_pos, 0.0, 0.81), rpy=(0.0, 0.0, math.pi / 4.0)),
            material=weathered_wood,
            name=f"picket_cap_{index}",
        )

    for hinge_name, z_pos in (("lower", 0.28), ("upper", 0.62)):
        leaf.visual(
            Box((0.33, 0.012, 0.035)),
            origin=Origin(xyz=(0.18, -0.031, z_pos)),
            material=dark_iron,
            name=f"{hinge_name}_hinge_strap",
        )
        leaf.visual(
            Cylinder(radius=0.018, length=0.11),
            origin=Origin(xyz=(0.0, -0.052, z_pos), rpy=(0.0, 0.0, 0.0)),
            material=dark_iron,
            name=f"{hinge_name}_hinge_barrel",
        )
        leaf.visual(
            Box((0.040, 0.020, 0.035)),
            origin=Origin(xyz=(0.006, -0.044, z_pos)),
            material=dark_iron,
            name=f"{hinge_name}_hinge_wrap",
        )

    leaf.visual(
        Box((0.18, 0.008, 0.10)),
        origin=Origin(xyz=(0.86, -0.031, 0.67)),
        material=dark_iron,
        name="ring_latch_plate",
    )
    leaf.visual(
        Box((0.19, 0.016, 0.024)),
        origin=Origin(xyz=(0.995, -0.041, 0.67)),
        material=dark_iron,
        name="latch_tongue",
    )

    leaf.visual(
        Box((0.200, 0.008, 0.050)),
        origin=Origin(xyz=(0.940, -0.029, 0.20)),
        material=dark_iron,
        name="lower_guide_plate",
    )
    leaf.visual(
        Box((0.012, 0.035, 0.050)),
        origin=Origin(xyz=(0.895, -0.0505, 0.20)),
        material=dark_iron,
        name="lower_guide_ear_0",
    )
    leaf.visual(
        Box((0.012, 0.035, 0.050)),
        origin=Origin(xyz=(0.925, -0.0505, 0.20)),
        material=dark_iron,
        name="lower_guide_ear_1",
    )
    leaf.visual(
        Box((0.200, 0.008, 0.050)),
        origin=Origin(xyz=(0.940, -0.029, 0.37)),
        material=dark_iron,
        name="upper_guide_plate",
    )
    leaf.visual(
        Box((0.012, 0.035, 0.050)),
        origin=Origin(xyz=(0.895, -0.0505, 0.37)),
        material=dark_iron,
        name="upper_guide_ear_0",
    )
    leaf.visual(
        Box((0.012, 0.035, 0.050)),
        origin=Origin(xyz=(0.925, -0.0505, 0.37)),
        material=dark_iron,
        name="upper_guide_ear_1",
    )

    ring = model.part("ring_latch")
    ring.visual(
        ring_mesh,
        origin=Origin(xyz=(0.0, -0.066, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_iron,
        name="ring_loop",
    )
    ring.visual(
        Cylinder(radius=0.016, length=0.065),
        origin=Origin(xyz=(0.0, -0.0325, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_iron,
        name="ring_hub",
    )
    ring.visual(
        Box((0.014, 0.014, 0.050)),
        origin=Origin(xyz=(0.0, -0.066, 0.034)),
        material=dark_iron,
        name="ring_spoke",
    )

    bolt = model.part("drop_bolt")
    bolt.visual(
        Cylinder(radius=0.009, length=0.50),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=galvanized,
        name="bolt_rod",
    )
    bolt.visual(
        Cylinder(radius=0.012, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, -0.2675)),
        material=galvanized,
        name="bolt_tip",
    )
    bolt.visual(
        Cylinder(radius=0.006, length=0.10),
        origin=Origin(xyz=(0.050, 0.0, 0.080), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=galvanized,
        name="bolt_handle",
    )

    model.articulation(
        "leaf_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=leaf,
        origin=Origin(xyz=(0.0, 0.0, 0.12)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=80.0, velocity=1.2, lower=0.0, upper=1.45),
    )
    model.articulation(
        "ring_pivot",
        ArticulationType.REVOLUTE,
        parent=leaf,
        child=ring,
        origin=Origin(xyz=(0.86, -0.030, 0.67)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=2.0, velocity=4.0, lower=-1.2, upper=1.2),
    )
    model.articulation(
        "bolt_slide",
        ArticulationType.PRISMATIC,
        parent=leaf,
        child=bolt,
        origin=Origin(xyz=(0.91, -0.065, 0.25)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=15.0, velocity=0.25, lower=0.0, upper=0.10),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    leaf_hinge = object_model.get_articulation("leaf_hinge")
    ring_pivot = object_model.get_articulation("ring_pivot")
    bolt_slide = object_model.get_articulation("bolt_slide")

    ctx.allow_overlap(
        "gate_leaf",
        "ring_latch",
        elem_a="ring_latch_plate",
        elem_b="ring_hub",
        reason="The ring latch hub is intentionally captured through the latch plate as its pivot bushing.",
    )

    ctx.check(
        "leaf swings on a vertical side hinge",
        leaf_hinge.articulation_type == ArticulationType.REVOLUTE
        and tuple(leaf_hinge.axis) == (0.0, 0.0, 1.0)
        and leaf_hinge.motion_limits is not None
        and leaf_hinge.motion_limits.upper is not None
        and leaf_hinge.motion_limits.upper > 1.3,
        details=f"axis={leaf_hinge.axis}, limits={leaf_hinge.motion_limits}",
    )
    ctx.check(
        "ring latch has its own revolute pivot",
        ring_pivot.articulation_type == ArticulationType.REVOLUTE
        and tuple(ring_pivot.axis) == (0.0, -1.0, 0.0),
        details=f"axis={ring_pivot.axis}",
    )
    ctx.check(
        "drop bolt is a vertical prismatic slide",
        bolt_slide.articulation_type == ArticulationType.PRISMATIC
        and tuple(bolt_slide.axis) == (0.0, 0.0, -1.0)
        and bolt_slide.motion_limits is not None
        and bolt_slide.motion_limits.upper is not None
        and bolt_slide.motion_limits.upper >= 0.09,
        details=f"axis={bolt_slide.axis}, limits={bolt_slide.motion_limits}",
    )

    ctx.expect_overlap(
        "drop_bolt",
        "gate_leaf",
        axes="z",
        elem_a="bolt_rod",
        elem_b="lower_guide_plate",
        min_overlap=0.04,
        name="bolt passes through lower guide bracket",
    )
    ctx.expect_overlap(
        "drop_bolt",
        "gate_leaf",
        axes="z",
        elem_a="bolt_rod",
        elem_b="upper_guide_plate",
        min_overlap=0.04,
        name="bolt passes through upper guide bracket",
    )
    ctx.expect_overlap(
        "ring_latch",
        "gate_leaf",
        axes="xz",
        elem_a="ring_hub",
        elem_b="ring_latch_plate",
        min_overlap=0.025,
        name="ring hub is seated in latch plate",
    )

    rest_pos = ctx.part_world_position("drop_bolt")
    with ctx.pose({bolt_slide: 0.10}):
        lowered_pos = ctx.part_world_position("drop_bolt")
    ctx.check(
        "bolt lowers toward the ground",
        rest_pos is not None and lowered_pos is not None and lowered_pos[2] < rest_pos[2] - 0.085,
        details=f"rest={rest_pos}, lowered={lowered_pos}",
    )

    def _aabb_center(aabb):
        if aabb is None:
            return None
        return (
            float((aabb[0][0] + aabb[1][0]) * 0.5),
            float((aabb[0][1] + aabb[1][1]) * 0.5),
            float((aabb[0][2] + aabb[1][2]) * 0.5),
        )

    closed_latch_center = _aabb_center(ctx.part_element_world_aabb("gate_leaf", elem="latch_stile"))
    with ctx.pose({leaf_hinge: 1.45}):
        open_latch_center = _aabb_center(ctx.part_element_world_aabb("gate_leaf", elem="latch_stile"))
    ctx.check(
        "leaf opens outward from the posts",
        closed_latch_center is not None
        and open_latch_center is not None
        and open_latch_center[1] > closed_latch_center[1] + 0.45,
        details=f"closed={closed_latch_center}, open={open_latch_center}",
    )

    ring_rest_center = _aabb_center(ctx.part_element_world_aabb("ring_latch", elem="ring_spoke"))
    with ctx.pose({ring_pivot: 0.9}):
        ring_turned_center = _aabb_center(ctx.part_element_world_aabb("ring_latch", elem="ring_spoke"))
    ctx.check(
        "ring latch visibly rotates",
        ring_rest_center is not None
        and ring_turned_center is not None
        and abs(ring_turned_center[0] - ring_rest_center[0]) > 0.020,
        details=f"rest={ring_rest_center}, turned={ring_turned_center}",
    )

    with ctx.pose({leaf_hinge: 1.45, bolt_slide: 0.10}):
        ctx.expect_overlap(
            "drop_bolt",
            "post_frame",
            axes="xy",
            elem_a="bolt_tip",
            elem_b="open_surface_socket",
            min_overlap=0.015,
            name="lowered bolt aligns with open hold socket",
        )

    return ctx.report()


object_model = build_object_model()
