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
    mesh_from_geometry,
    wire_from_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="snowmobile_front_ski_spindle_assembly")

    chassis_black = model.material("chassis_black", rgba=(0.15, 0.16, 0.17, 1.0))
    arm_black = model.material("arm_black", rgba=(0.12, 0.12, 0.13, 1.0))
    steel = model.material("steel", rgba=(0.60, 0.62, 0.66, 1.0))
    machined_steel = model.material("machined_steel", rgba=(0.73, 0.75, 0.78, 1.0))
    saddle_gray = model.material("saddle_gray", rgba=(0.47, 0.49, 0.52, 1.0))
    fastener_dark = model.material("fastener_dark", rgba=(0.28, 0.29, 0.31, 1.0))

    chassis = model.part("chassis")
    chassis.visual(
        Box((0.18, 0.34, 0.11)),
        origin=Origin(xyz=(-0.105, 0.0, 0.100)),
        material=chassis_black,
        name="side_rail",
    )
    chassis.visual(
        Box((0.11, 0.24, 0.045)),
        origin=Origin(xyz=(-0.050, 0.0, 0.160)),
        material=chassis_black,
        name="upper_frame_cap",
    )
    chassis.visual(
        Box((0.040, 0.080, 0.100)),
        origin=Origin(xyz=(-0.055, 0.0, 0.000)),
        material=steel,
        name="pivot_bulkhead",
    )
    chassis.visual(
        Box((0.090, 0.180, 0.045)),
        origin=Origin(xyz=(-0.070, 0.0, -0.055)),
        material=chassis_black,
        name="lower_gusset",
    )

    pivot_x = -0.008
    pivot_tab_x = 0.055
    pivot_tab_z = 0.060
    pivot_tab_thickness = 0.010
    pivot_half_spacing = 0.120
    tab_half_gap = 0.030

    for prefix, pivot_y in (("front", pivot_half_spacing), ("rear", -pivot_half_spacing)):
        chassis.visual(
            Box((0.030, 0.060, 0.050)),
            origin=Origin(xyz=(-0.055, pivot_y * 0.58, -0.005)),
            material=steel,
            name=f"{prefix}_pivot_web",
        )
        chassis.visual(
            Box((0.030, 0.060, 0.040)),
            origin=Origin(xyz=(-0.045, pivot_y, -0.005)),
            material=steel,
            name=f"{prefix}_pivot_support",
        )
        chassis.visual(
            Box((pivot_tab_x, pivot_tab_thickness, pivot_tab_z)),
            origin=Origin(xyz=(pivot_x, pivot_y - tab_half_gap, 0.0)),
            material=steel,
            name=f"{prefix}_pivot_aft_tab",
        )
        chassis.visual(
            Box((pivot_tab_x, pivot_tab_thickness, pivot_tab_z)),
            origin=Origin(xyz=(pivot_x, pivot_y + tab_half_gap, 0.0)),
            material=steel,
            name=f"{prefix}_pivot_fore_tab",
        )
        chassis.visual(
            Cylinder(radius=0.014, length=0.008),
            origin=Origin(
                xyz=(0.016, pivot_y - 0.039, 0.0),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=fastener_dark,
            name=f"{prefix}_pivot_bolt_head",
        )
        chassis.visual(
            Cylinder(radius=0.012, length=0.008),
            origin=Origin(
                xyz=(0.016, pivot_y + 0.039, 0.0),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=fastener_dark,
            name=f"{prefix}_pivot_bolt_nut",
        )

    chassis.visual(
        Cylinder(radius=0.018, length=0.330),
        origin=Origin(xyz=(-0.105, 0.0, 0.150), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=chassis_black,
        name="upper_cross_tube",
    )
    chassis.inertial = Inertial.from_geometry(
        Box((0.22, 0.40, 0.24)),
        mass=10.0,
        origin=Origin(xyz=(-0.060, 0.0, 0.060)),
    )

    wishbone = model.part("wishbone")
    arm_tube = wire_from_points(
        [
            (0.0, -pivot_half_spacing, 0.0),
            (0.340, 0.0, -0.045),
            (0.0, pivot_half_spacing, 0.0),
        ],
        radius=0.016,
        radial_segments=18,
        closed_path=False,
        cap_ends=True,
        corner_mode="fillet",
        corner_radius=0.040,
        corner_segments=12,
    )
    wishbone.visual(
        mesh_from_geometry(arm_tube, "wishbone_perimeter"),
        material=arm_black,
        name="wishbone_perimeter",
    )
    wishbone.visual(
        Cylinder(radius=0.013, length=0.160),
        origin=Origin(xyz=(0.070, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=arm_black,
        name="wishbone_crossbrace",
    )
    wishbone.visual(
        Cylinder(radius=0.024, length=0.050),
        origin=Origin(xyz=(0.0, pivot_half_spacing, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="front_inner_bushing",
    )
    wishbone.visual(
        Cylinder(radius=0.024, length=0.050),
        origin=Origin(xyz=(0.0, -pivot_half_spacing, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=steel,
        name="rear_inner_bushing",
    )
    wishbone.visual(
        Box((0.032, 0.060, 0.026)),
        origin=Origin(xyz=(0.348, 0.0, -0.046)),
        material=arm_black,
        name="outer_bridge",
    )
    wishbone.visual(
        Box((0.056, 0.014, 0.116)),
        origin=Origin(xyz=(0.388, -0.031, 0.018)),
        material=steel,
        name="kingpin_rear_ear",
    )
    wishbone.visual(
        Box((0.056, 0.014, 0.116)),
        origin=Origin(xyz=(0.388, 0.031, 0.018)),
        material=steel,
        name="kingpin_front_ear",
    )
    kingpin_loop = wire_from_points(
        [
            (0.364, -0.038, 0.022),
            (0.420, -0.038, 0.022),
            (0.420, 0.038, 0.022),
            (0.364, 0.038, 0.022),
        ],
        radius=0.011,
        radial_segments=16,
        closed_path=True,
        cap_ends=False,
        corner_mode="miter",
    )
    wishbone.visual(
        mesh_from_geometry(kingpin_loop, "upper_kingpin_sleeve"),
        material=fastener_dark,
        name="upper_kingpin_sleeve",
    )
    wishbone.inertial = Inertial.from_geometry(
        Box((0.46, 0.30, 0.14)),
        mass=4.2,
        origin=Origin(xyz=(0.200, 0.0, 0.0)),
    )

    saddle = model.part("saddle")
    saddle.visual(
        Cylinder(radius=0.016, length=0.250),
        origin=Origin(xyz=(0.0, 0.0, -0.040)),
        material=machined_steel,
        name="kingpost",
    )
    saddle.visual(
        Cylinder(radius=0.024, length=0.032),
        origin=Origin(xyz=(0.0, 0.0, 0.028)),
        material=saddle_gray,
        name="upper_bearing_cap",
    )
    saddle.visual(
        Box((0.042, 0.072, 0.110)),
        origin=Origin(xyz=(0.0, 0.0, -0.135)),
        material=saddle_gray,
        name="center_web",
    )
    saddle.visual(
        Box((0.098, 0.200, 0.014)),
        origin=Origin(xyz=(0.0, 0.0, -0.152)),
        material=saddle_gray,
        name="saddle_bridge",
    )
    saddle.visual(
        Box((0.012, 0.200, 0.090)),
        origin=Origin(xyz=(-0.046, 0.0, -0.180)),
        material=saddle_gray,
        name="inner_cheek",
    )
    saddle.visual(
        Box((0.012, 0.200, 0.090)),
        origin=Origin(xyz=(0.046, 0.0, -0.180)),
        material=saddle_gray,
        name="outer_cheek",
    )
    saddle.visual(
        Cylinder(radius=0.012, length=0.104),
        origin=Origin(xyz=(0.0, 0.0, -0.198), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=fastener_dark,
        name="ski_bolt_tube",
    )
    saddle.visual(
        Box((0.054, 0.030, 0.026)),
        origin=Origin(xyz=(0.0, 0.102, -0.146)),
        material=saddle_gray,
        name="front_saddle_nose",
    )
    saddle.visual(
        Box((0.054, 0.034, 0.026)),
        origin=Origin(xyz=(0.0, -0.082, -0.152)),
        material=saddle_gray,
        name="rear_ski_stop",
    )
    saddle.inertial = Inertial.from_geometry(
        Box((0.14, 0.24, 0.30)),
        mass=2.8,
        origin=Origin(xyz=(0.0, 0.0, -0.090)),
    )

    model.articulation(
        "chassis_to_wishbone",
        ArticulationType.REVOLUTE,
        parent=chassis,
        child=wishbone,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        # The wishbone extends mostly along local +X from the inner pivot axis.
        # Using -Y makes positive motion raise the outer end toward +Z.
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2200.0,
            velocity=2.2,
            lower=-0.30,
            upper=0.42,
        ),
    )
    model.articulation(
        "wishbone_to_saddle",
        ArticulationType.REVOLUTE,
        parent=wishbone,
        child=saddle,
        origin=Origin(xyz=(0.392, 0.0, 0.030)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=900.0,
            velocity=2.8,
            lower=-0.58,
            upper=0.58,
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

    chassis = object_model.get_part("chassis")
    wishbone = object_model.get_part("wishbone")
    saddle = object_model.get_part("saddle")
    arm_pivot = object_model.get_articulation("chassis_to_wishbone")
    kingpin = object_model.get_articulation("wishbone_to_saddle")

    ctx.check(
        "wishbone pivot axis is fore-aft",
        tuple(round(v, 3) for v in arm_pivot.axis) == (0.0, -1.0, 0.0),
        details=f"axis={arm_pivot.axis}",
    )
    ctx.check(
        "kingpin axis is vertical",
        tuple(round(v, 3) for v in kingpin.axis) == (0.0, 0.0, 1.0),
        details=f"axis={kingpin.axis}",
    )

    ctx.expect_gap(
        chassis,
        wishbone,
        axis="y",
        positive_elem="front_pivot_fore_tab",
        negative_elem="front_inner_bushing",
        max_gap=0.001,
        max_penetration=0.0,
        name="front bushing seats against fore chassis tab",
    )
    ctx.expect_gap(
        wishbone,
        chassis,
        axis="y",
        positive_elem="front_inner_bushing",
        negative_elem="front_pivot_aft_tab",
        max_gap=0.001,
        max_penetration=0.0,
        name="front bushing seats against aft chassis tab",
    )
    ctx.expect_gap(
        chassis,
        wishbone,
        axis="y",
        positive_elem="rear_pivot_fore_tab",
        negative_elem="rear_inner_bushing",
        max_gap=0.001,
        max_penetration=0.0,
        name="rear bushing seats against fore chassis tab",
    )
    ctx.expect_gap(
        wishbone,
        chassis,
        axis="y",
        positive_elem="rear_inner_bushing",
        negative_elem="rear_pivot_aft_tab",
        max_gap=0.001,
        max_penetration=0.0,
        name="rear bushing seats against aft chassis tab",
    )
    ctx.expect_overlap(
        saddle,
        wishbone,
        axes="xz",
        elem_a="kingpost",
        elem_b="upper_kingpin_sleeve",
        min_overlap=0.020,
        name="kingpost stays centered at the outer wishbone pivot",
    )
    ctx.expect_contact(
        saddle,
        wishbone,
        elem_a="upper_bearing_cap",
        elem_b="kingpin_front_ear",
        name="upper bearing cap bears against the front kingpin ear",
    )
    ctx.expect_contact(
        saddle,
        wishbone,
        elem_a="upper_bearing_cap",
        elem_b="kingpin_rear_ear",
        name="upper bearing cap bears against the rear kingpin ear",
    )

    rest_saddle_pos = ctx.part_world_position(saddle)
    with ctx.pose({arm_pivot: 0.25}):
        lifted_saddle_pos = ctx.part_world_position(saddle)
    ctx.check(
        "wishbone positive motion raises the spindle assembly",
        rest_saddle_pos is not None
        and lifted_saddle_pos is not None
        and lifted_saddle_pos[2] > rest_saddle_pos[2] + 0.06,
        details=f"rest={rest_saddle_pos}, lifted={lifted_saddle_pos}",
    )

    front_nose_rest = ctx.part_element_world_aabb(saddle, elem="front_saddle_nose")
    rear_stop_rest = ctx.part_element_world_aabb(saddle, elem="rear_ski_stop")
    with ctx.pose({kingpin: 0.40}):
        front_nose_steered = ctx.part_element_world_aabb(saddle, elem="front_saddle_nose")
        rear_stop_steered = ctx.part_element_world_aabb(saddle, elem="rear_ski_stop")
    nose_rest_center = None
    nose_steered_center = None
    rear_rest_center = None
    rear_steered_center = None
    if front_nose_rest is not None:
        nose_rest_center = tuple(
            (front_nose_rest[0][i] + front_nose_rest[1][i]) * 0.5 for i in range(3)
        )
    if front_nose_steered is not None:
        nose_steered_center = tuple(
            (front_nose_steered[0][i] + front_nose_steered[1][i]) * 0.5 for i in range(3)
        )
    if rear_stop_rest is not None:
        rear_rest_center = tuple(
            (rear_stop_rest[0][i] + rear_stop_rest[1][i]) * 0.5 for i in range(3)
        )
    if rear_stop_steered is not None:
        rear_steered_center = tuple(
            (rear_stop_steered[0][i] + rear_stop_steered[1][i]) * 0.5 for i in range(3)
        )
    ctx.check(
        "kingpin rotation yaws the ski saddle",
        nose_rest_center is not None
        and nose_steered_center is not None
        and rear_rest_center is not None
        and rear_steered_center is not None
        and nose_steered_center[0] < nose_rest_center[0] - 0.025
        and rear_steered_center[0] > rear_rest_center[0] + 0.020,
        details=(
            f"front_rest={nose_rest_center}, front_steered={nose_steered_center}, "
            f"rear_rest={rear_rest_center}, rear_steered={rear_steered_center}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
