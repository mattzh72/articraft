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


ARM_SWEEP_YAW = -0.72
BLADE_OPEN_ANGLE = math.radians(75.0)
GAUGE_TRAVEL_CHECK = 0.14
EXTENSION_OPEN_ANGLE = math.radians(-82.0)


def _cylinder_along_y(*, xyz: tuple[float, float, float]) -> Origin:
    return Origin(xyz=xyz, rpy=(math.pi / 2.0, 0.0, 0.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="heavy_paper_cutter")

    painted_steel = model.material("painted_steel", rgba=(0.21, 0.24, 0.28, 1.0))
    cast_iron = model.material("cast_iron", rgba=(0.31, 0.33, 0.36, 1.0))
    stainless = model.material("stainless", rgba=(0.77, 0.79, 0.81, 1.0))
    black_grip = model.material("black_grip", rgba=(0.10, 0.10, 0.11, 1.0))
    ruler_accent = model.material("ruler_accent", rgba=(0.87, 0.88, 0.85, 1.0))
    warning_red = model.material("warning_red", rgba=(0.67, 0.10, 0.10, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.54, 0.45, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=cast_iron,
        name="lower_plinth",
    )
    base.visual(
        Box((0.54, 0.41, 0.020)),
        origin=Origin(xyz=(0.0, 0.0, 0.028)),
        material=painted_steel,
        name="bed_plate",
    )
    base.visual(
        Box((0.40, 0.028, 0.035)),
        origin=Origin(xyz=(0.055, 0.191, 0.0555)),
        material=cast_iron,
        name="rear_fence",
    )
    base.visual(
        Box((0.08, 0.028, 0.020)),
        origin=Origin(xyz=(-0.145, 0.191, 0.048)),
        material=cast_iron,
        name="rear_fence_left_stub",
    )
    base.visual(
        Box((0.018, 0.310, 0.012)),
        origin=Origin(xyz=(-0.195, 0.020, 0.044)),
        material=stainless,
        name="cutting_edge_bar",
    )
    base.visual(
        Box((0.42, 0.032, 0.010)),
        origin=Origin(xyz=(0.040, -0.168, 0.043)),
        material=stainless,
        name="guide_deck",
    )
    base.visual(
        Box((0.45, 0.015, 0.003)),
        origin=Origin(xyz=(0.040, -0.132, 0.0395)),
        material=ruler_accent,
        name="front_scale_strip",
    )
    base.visual(
        Box((0.46, 0.010, 0.004)),
        origin=Origin(xyz=(-0.012, -0.012, 0.040), rpy=(0.0, 0.0, ARM_SWEEP_YAW)),
        material=warning_red,
        name="cut_line_strip",
    )
    base.visual(
        Box((0.060, 0.060, 0.010)),
        origin=Origin(xyz=(-0.237, 0.165, 0.043)),
        material=cast_iron,
        name="pivot_block",
    )
    base.visual(
        Cylinder(radius=0.020, length=0.010),
        origin=Origin(xyz=(-0.237, 0.165, 0.043)),
        material=stainless,
        name="pivot_collar",
    )
    base.visual(
        Box((0.014, 0.090, 0.018)),
        origin=Origin(xyz=(0.277, -0.120, 0.009)),
        material=cast_iron,
        name="hinge_mount_front",
    )
    base.visual(
        Box((0.014, 0.090, 0.018)),
        origin=Origin(xyz=(0.277, 0.120, 0.009)),
        material=cast_iron,
        name="hinge_mount_rear",
    )
    base.visual(
        Cylinder(radius=0.009, length=0.080),
        origin=_cylinder_along_y(xyz=(0.280, -0.120, 0.011)),
        material=stainless,
        name="hinge_knuckle_front",
    )
    base.visual(
        Cylinder(radius=0.009, length=0.080),
        origin=_cylinder_along_y(xyz=(0.280, 0.120, 0.011)),
        material=stainless,
        name="hinge_knuckle_rear",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.54, 0.45, 0.090)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, 0.045)),
    )

    blade_arm = model.part("blade_arm")
    blade_arm.visual(
        Cylinder(radius=0.028, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, -0.005)),
        material=stainless,
        name="pivot_hub",
    )
    blade_arm.visual(
        Box((0.058, 0.050, 0.038)),
        origin=Origin(xyz=(0.014, -0.010, 0.015)),
        material=painted_steel,
        name="pivot_cheek",
    )
    blade_arm.visual(
        Box((0.090, 0.058, 0.020)),
        origin=Origin(xyz=(0.050, 0.0, 0.022), rpy=(0.0, 0.0, ARM_SWEEP_YAW)),
        material=painted_steel,
        name="arm_root",
    )
    blade_arm.visual(
        Box((0.430, 0.052, 0.018)),
        origin=Origin(xyz=(0.260, 0.0, 0.028), rpy=(0.0, 0.0, ARM_SWEEP_YAW)),
        material=painted_steel,
        name="arm_body",
    )
    blade_arm.visual(
        Box((0.260, 0.050, 0.022)),
        origin=Origin(xyz=(0.145, 0.0, 0.026), rpy=(0.0, 0.0, ARM_SWEEP_YAW)),
        material=painted_steel,
        name="arm_spine",
    )
    blade_arm.visual(
        Box((0.390, 0.018, 0.010)),
        origin=Origin(xyz=(0.240, -0.018, 0.018), rpy=(0.0, 0.0, ARM_SWEEP_YAW)),
        material=stainless,
        name="blade_backer",
    )
    blade_arm.visual(
        Box((0.120, 0.060, 0.032)),
        origin=Origin(xyz=(0.470, 0.0, 0.040), rpy=(0.0, 0.0, ARM_SWEEP_YAW)),
        material=painted_steel,
        name="handle_housing",
    )
    blade_arm.visual(
        Box((0.180, 0.050, 0.022)),
        origin=Origin(xyz=(0.390, 0.0, 0.036), rpy=(0.0, 0.0, ARM_SWEEP_YAW)),
        material=painted_steel,
        name="handle_neck",
    )
    blade_arm.visual(
        Box((0.090, 0.034, 0.024)),
        origin=Origin(xyz=(0.515, 0.0, 0.055), rpy=(0.0, 0.0, ARM_SWEEP_YAW)),
        material=black_grip,
        name="handle_grip",
    )
    blade_arm.inertial = Inertial.from_geometry(
        Box((0.60, 0.10, 0.08)),
        mass=3.0,
        origin=Origin(xyz=(0.24, 0.0, 0.035)),
    )

    side_gauge = model.part("side_gauge")
    side_gauge.visual(
        Box((0.070, 0.045, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=cast_iron,
        name="gauge_carriage",
    )
    side_gauge.visual(
        Box((0.012, 0.075, 0.058)),
        origin=Origin(xyz=(0.0, 0.055, 0.035)),
        material=stainless,
        name="gauge_fence",
    )
    side_gauge.visual(
        Box((0.040, 0.030, 0.018)),
        origin=Origin(xyz=(0.014, -0.010, 0.021)),
        material=black_grip,
        name="gauge_cursor",
    )
    side_gauge.inertial = Inertial.from_geometry(
        Box((0.080, 0.090, 0.070)),
        mass=0.7,
        origin=Origin(xyz=(0.0, 0.038, 0.035)),
    )

    extension_table = model.part("extension_table")
    extension_table.visual(
        Cylinder(radius=0.009, length=0.150),
        origin=_cylinder_along_y(xyz=(0.0, 0.0, 0.0)),
        material=stainless,
        name="hinge_barrel",
    )
    extension_table.visual(
        Box((0.030, 0.320, 0.024)),
        origin=Origin(xyz=(0.020, 0.0, 0.012)),
        material=cast_iron,
        name="inner_rib",
    )
    extension_table.visual(
        Box((0.220, 0.350, 0.018)),
        origin=Origin(xyz=(0.100, 0.0, 0.018)),
        material=painted_steel,
        name="table_panel",
    )
    extension_table.inertial = Inertial.from_geometry(
        Box((0.22, 0.35, 0.050)),
        mass=1.8,
        origin=Origin(xyz=(0.100, 0.0, 0.018)),
    )

    model.articulation(
        "blade_pivot",
        ArticulationType.REVOLUTE,
        parent=base,
        child=blade_arm,
        origin=Origin(xyz=(-0.237, 0.165, 0.058)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=1.5,
            lower=0.0,
            upper=math.radians(85.0),
        ),
    )
    model.articulation(
        "gauge_slide",
        ArticulationType.PRISMATIC,
        parent=base,
        child=side_gauge,
        origin=Origin(xyz=(0.055, -0.168, 0.048)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=12.0,
            velocity=0.25,
            lower=-0.15,
            upper=0.16,
        ),
    )
    model.articulation(
        "extension_hinge",
        ArticulationType.REVOLUTE,
        parent=base,
        child=extension_table,
        origin=Origin(xyz=(0.280, 0.0, 0.011)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=15.0,
            velocity=1.5,
            lower=math.radians(-95.0),
            upper=0.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    blade_arm = object_model.get_part("blade_arm")
    side_gauge = object_model.get_part("side_gauge")
    extension_table = object_model.get_part("extension_table")

    blade_pivot = object_model.get_articulation("blade_pivot")
    gauge_slide = object_model.get_articulation("gauge_slide")
    extension_hinge = object_model.get_articulation("extension_hinge")

    bed_plate = base.get_visual("bed_plate")
    guide_deck = base.get_visual("guide_deck")
    pivot_collar = base.get_visual("pivot_collar")
    arm_body = blade_arm.get_visual("arm_body")
    pivot_hub = blade_arm.get_visual("pivot_hub")
    gauge_carriage = side_gauge.get_visual("gauge_carriage")
    gauge_fence = side_gauge.get_visual("gauge_fence")
    table_panel = extension_table.get_visual("table_panel")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "blade arm uses corner revolute pivot",
        blade_pivot.articulation_type == ArticulationType.REVOLUTE and blade_pivot.axis == (0.0, 0.0, 1.0),
        details=f"got type={blade_pivot.articulation_type} axis={blade_pivot.axis}",
    )
    ctx.check(
        "side gauge uses prismatic guide axis",
        gauge_slide.articulation_type == ArticulationType.PRISMATIC and gauge_slide.axis == (1.0, 0.0, 0.0),
        details=f"got type={gauge_slide.articulation_type} axis={gauge_slide.axis}",
    )
    ctx.check(
        "extension table uses side hinge axis",
        extension_hinge.articulation_type == ArticulationType.REVOLUTE and extension_hinge.axis == (0.0, 1.0, 0.0),
        details=f"got type={extension_hinge.articulation_type} axis={extension_hinge.axis}",
    )

    ctx.expect_contact(blade_arm, base, elem_a=pivot_hub, elem_b=pivot_collar)
    ctx.expect_gap(
        blade_arm,
        base,
        axis="z",
        positive_elem=pivot_hub,
        negative_elem=pivot_collar,
        max_gap=0.0005,
        max_penetration=0.0,
        name="blade pivot hub seats on collar",
    )
    ctx.expect_overlap(
        blade_arm,
        base,
        axes="xy",
        min_overlap=0.04,
        elem_a=arm_body,
        elem_b=bed_plate,
        name="blade arm spans the cutting bed",
    )

    ctx.expect_contact(side_gauge, base, elem_a=gauge_carriage, elem_b=guide_deck)
    ctx.expect_gap(
        side_gauge,
        base,
        axis="z",
        positive_elem=gauge_carriage,
        negative_elem=guide_deck,
        max_gap=0.0005,
        max_penetration=0.0,
        name="side gauge rides on guide deck",
    )
    ctx.expect_within(
        side_gauge,
        base,
        axes="y",
        inner_elem=gauge_fence,
        outer_elem=bed_plate,
        margin=0.0,
        name="side gauge fence stays within bed depth",
    )

    ctx.expect_contact(extension_table, base, elem_a=table_panel, elem_b=bed_plate)
    ctx.expect_gap(
        extension_table,
        base,
        axis="x",
        positive_elem=table_panel,
        negative_elem=bed_plate,
        max_gap=0.001,
        max_penetration=0.0,
        name="extension table closes flush to bed edge",
    )
    ctx.expect_within(
        extension_table,
        base,
        axes="y",
        inner_elem=table_panel,
        outer_elem=bed_plate,
        margin=0.03,
        name="extension table stays aligned with bed depth",
    )

    blade_rest_aabb = ctx.part_world_aabb(blade_arm)
    gauge_rest_pos = ctx.part_world_position(side_gauge)
    extension_rest_aabb = ctx.part_world_aabb(extension_table)
    assert blade_rest_aabb is not None
    assert gauge_rest_pos is not None
    assert extension_rest_aabb is not None

    with ctx.pose({blade_pivot: BLADE_OPEN_ANGLE}):
        blade_open_aabb = ctx.part_world_aabb(blade_arm)
        assert blade_open_aabb is not None
        ctx.check(
            "blade arm opens away from the bed",
            blade_open_aabb[1][1] > blade_rest_aabb[1][1] + 0.18,
            details=f"rest max y={blade_rest_aabb[1][1]:.4f}, open max y={blade_open_aabb[1][1]:.4f}",
        )

    with ctx.pose({gauge_slide: GAUGE_TRAVEL_CHECK}):
        gauge_moved_pos = ctx.part_world_position(side_gauge)
        assert gauge_moved_pos is not None
        ctx.check(
            "side gauge slides to set cut width",
            gauge_moved_pos[0] > gauge_rest_pos[0] + 0.12,
            details=f"rest x={gauge_rest_pos[0]:.4f}, moved x={gauge_moved_pos[0]:.4f}",
        )
        ctx.expect_contact(side_gauge, base, elem_a=gauge_carriage, elem_b=guide_deck)

    with ctx.pose({extension_hinge: EXTENSION_OPEN_ANGLE}):
        extension_open_aabb = ctx.part_world_aabb(extension_table)
        assert extension_open_aabb is not None
        ctx.check(
            "extension table folds upward on its hinge",
            extension_open_aabb[1][2] > extension_rest_aabb[1][2] + 0.14,
            details=(
                f"rest max z={extension_rest_aabb[1][2]:.4f}, "
                f"open max z={extension_open_aabb[1][2]:.4f}"
            ),
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
