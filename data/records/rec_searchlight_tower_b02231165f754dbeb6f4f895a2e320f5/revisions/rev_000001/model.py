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
    model = ArticulatedObject(name="searchlight_tower")

    tower_gray = model.material("tower_gray", rgba=(0.42, 0.45, 0.47, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.18, 0.19, 0.20, 1.0))
    service_black = model.material("service_black", rgba=(0.08, 0.09, 0.10, 1.0))
    safety_orange = model.material("safety_orange", rgba=(0.82, 0.40, 0.12, 1.0))
    lens_glass = model.material("lens_glass", rgba=(0.72, 0.85, 0.92, 0.75))

    tower_base = model.part("tower_base")
    tower_base.visual(
        Box((1.55, 1.15, 0.16)),
        origin=Origin(xyz=(0.0, 0.0, 0.08)),
        material=tower_gray,
        name="base_skid_frame",
    )
    tower_base.visual(
        Box((0.54, 0.54, 0.20)),
        origin=Origin(xyz=(0.0, 0.0, 0.26)),
        material=dark_steel,
        name="mast_pedestal",
    )
    tower_base.visual(
        Box((0.30, 0.30, 1.78)),
        origin=Origin(xyz=(0.0, 0.0, 1.25)),
        material=tower_gray,
        name="mast_column",
    )
    tower_base.visual(
        Box((0.16, 0.20, 1.10)),
        origin=Origin(xyz=(-0.23, 0.0, 1.03)),
        material=dark_steel,
        name="service_spine",
    )
    tower_base.visual(
        Box((0.62, 0.42, 0.59)),
        origin=Origin(xyz=(-0.28, -0.29, 0.445)),
        material=service_black,
        name="power_cabinet",
    )
    tower_base.visual(
        Cylinder(radius=0.19, length=0.14),
        origin=Origin(xyz=(0.0, 0.0, 2.21)),
        material=dark_steel,
        name="mast_bearing_housing",
    )
    tower_base.visual(
        Cylinder(radius=0.02, length=1.18),
        origin=Origin(xyz=(-0.325, 0.085, 1.03)),
        material=dark_steel,
        name="ladder_rail_left",
    )
    tower_base.visual(
        Cylinder(radius=0.02, length=1.18),
        origin=Origin(xyz=(-0.325, -0.085, 1.03)),
        material=dark_steel,
        name="ladder_rail_right",
    )
    for rung_index, rung_z in enumerate((0.62, 0.88, 1.14, 1.40)):
        tower_base.visual(
            Cylinder(radius=0.012, length=0.19),
            origin=Origin(
                xyz=(-0.325, 0.0, rung_z),
                rpy=(math.pi / 2.0, 0.0, 0.0),
            ),
            material=dark_steel,
            name=f"ladder_rung_{rung_index}",
        )
    tower_base.inertial = Inertial.from_geometry(
        Box((1.55, 1.15, 2.28)),
        mass=720.0,
        origin=Origin(xyz=(0.0, 0.0, 1.14)),
    )

    pan_carriage = model.part("pan_carriage")
    pan_carriage.visual(
        Cylinder(radius=0.23, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
        material=dark_steel,
        name="turntable_plate",
    )
    pan_carriage.visual(
        Box((0.28, 0.34, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 0.10)),
        material=service_black,
        name="pan_drive_box",
    )
    pan_carriage.visual(
        Box((0.26, 0.54, 0.10)),
        origin=Origin(xyz=(-0.02, 0.0, 0.17)),
        material=tower_gray,
        name="yoke_saddle",
    )
    pan_carriage.visual(
        Box((0.14, 0.10, 0.92)),
        origin=Origin(xyz=(-0.03, 0.30, 0.59)),
        material=tower_gray,
        name="left_yoke_arm",
    )
    pan_carriage.visual(
        Box((0.14, 0.10, 0.92)),
        origin=Origin(xyz=(-0.03, -0.30, 0.59)),
        material=tower_gray,
        name="right_yoke_arm",
    )
    pan_carriage.visual(
        Box((0.12, 0.58, 0.10)),
        origin=Origin(xyz=(-0.10, 0.0, 1.00)),
        material=dark_steel,
        name="rear_tie_bar",
    )
    pan_carriage.visual(
        Box((0.12, 0.10, 0.22)),
        origin=Origin(xyz=(0.08, 0.24, 0.63)),
        material=dark_steel,
        name="left_bearing_block",
    )
    pan_carriage.visual(
        Box((0.12, 0.10, 0.22)),
        origin=Origin(xyz=(0.08, -0.24, 0.63)),
        material=dark_steel,
        name="right_bearing_block",
    )
    pan_carriage.visual(
        Box((0.05, 0.07, 0.05)),
        origin=Origin(xyz=(0.08, 0.13, 0.245)),
        material=safety_orange,
        name="left_wear_pad",
    )
    pan_carriage.visual(
        Box((0.05, 0.07, 0.05)),
        origin=Origin(xyz=(0.08, -0.13, 0.245)),
        material=safety_orange,
        name="right_wear_pad",
    )
    pan_carriage.inertial = Inertial.from_geometry(
        Box((0.58, 0.70, 1.08)),
        mass=145.0,
        origin=Origin(xyz=(0.0, 0.0, 0.54)),
    )

    spotlight_head = model.part("spotlight_head")
    spotlight_head.visual(
        Cylinder(radius=0.038, length=0.38),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="trunnion_shaft",
    )
    spotlight_head.visual(
        Cylinder(radius=0.18, length=0.46),
        origin=Origin(xyz=(0.08, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=service_black,
        name="lamp_barrel",
    )
    spotlight_head.visual(
        Cylinder(radius=0.205, length=0.07),
        origin=Origin(xyz=(0.31, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="front_bezel",
    )
    spotlight_head.visual(
        Cylinder(radius=0.175, length=0.014),
        origin=Origin(xyz=(0.352, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=lens_glass,
        name="front_lens",
    )
    spotlight_head.visual(
        Box((0.24, 0.30, 0.24)),
        origin=Origin(xyz=(-0.20, 0.0, 0.0)),
        material=tower_gray,
        name="rear_housing",
    )
    spotlight_head.visual(
        Box((0.14, 0.28, 0.11)),
        origin=Origin(xyz=(-0.28, 0.0, 0.145)),
        material=dark_steel,
        name="ballast_pod",
    )
    spotlight_head.visual(
        Box((0.09, 0.20, 0.03)),
        origin=Origin(xyz=(-0.28, 0.0, 0.195)),
        material=safety_orange,
        name="service_hatch",
    )
    spotlight_head.visual(
        Box((0.08, 0.16, 0.12)),
        origin=Origin(xyz=(-0.34, 0.0, 0.0)),
        material=dark_steel,
        name="rear_connector_block",
    )
    spotlight_head.visual(
        Cylinder(radius=0.012, length=0.20),
        origin=Origin(xyz=(-0.16, 0.09, 0.185)),
        material=dark_steel,
        name="handle_post_left",
    )
    spotlight_head.visual(
        Cylinder(radius=0.012, length=0.20),
        origin=Origin(xyz=(-0.16, -0.09, 0.185)),
        material=dark_steel,
        name="handle_post_right",
    )
    spotlight_head.visual(
        Cylinder(radius=0.012, length=0.20),
        origin=Origin(
            xyz=(-0.16, 0.0, 0.285),
            rpy=(math.pi / 2.0, 0.0, 0.0),
        ),
        material=dark_steel,
        name="service_handle",
    )
    spotlight_head.inertial = Inertial.from_geometry(
        Box((0.72, 0.38, 0.45)),
        mass=92.0,
        origin=Origin(xyz=(0.01, 0.0, 0.02)),
    )

    model.articulation(
        "pan_axis",
        ArticulationType.CONTINUOUS,
        parent=tower_base,
        child=pan_carriage,
        origin=Origin(xyz=(0.0, 0.0, 2.28)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=3200.0, velocity=0.9),
    )
    model.articulation(
        "tilt_axis",
        ArticulationType.REVOLUTE,
        parent=pan_carriage,
        child=spotlight_head,
        origin=Origin(xyz=(0.08, 0.0, 0.63)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1800.0,
            velocity=1.2,
            lower=-0.55,
            upper=1.05,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower_base = object_model.get_part("tower_base")
    pan_carriage = object_model.get_part("pan_carriage")
    spotlight_head = object_model.get_part("spotlight_head")
    pan_axis = object_model.get_articulation("pan_axis")
    tilt_axis = object_model.get_articulation("tilt_axis")

    mast_bearing_housing = tower_base.get_visual("mast_bearing_housing")
    turntable_plate = pan_carriage.get_visual("turntable_plate")
    yoke_saddle = pan_carriage.get_visual("yoke_saddle")
    left_bearing_block = pan_carriage.get_visual("left_bearing_block")
    right_bearing_block = pan_carriage.get_visual("right_bearing_block")
    rear_tie_bar = pan_carriage.get_visual("rear_tie_bar")
    trunnion_shaft = spotlight_head.get_visual("trunnion_shaft")
    front_bezel = spotlight_head.get_visual("front_bezel")
    front_lens = spotlight_head.get_visual("front_lens")
    rear_housing = spotlight_head.get_visual("rear_housing")
    ballast_pod = spotlight_head.get_visual("ballast_pod")
    handle_post_left = spotlight_head.get_visual("handle_post_left")
    handle_post_right = spotlight_head.get_visual("handle_post_right")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.check(
        "expected_parts_and_axes_present",
        len(object_model.parts) == 3 and len(object_model.articulations) == 2,
        details=(
            f"expected 3 parts / 2 articulations, found "
            f"{len(object_model.parts)} parts / {len(object_model.articulations)} articulations"
        ),
    )

    ctx.expect_gap(
        pan_carriage,
        tower_base,
        axis="z",
        positive_elem=turntable_plate,
        negative_elem=mast_bearing_housing,
        max_gap=0.001,
        max_penetration=0.0,
        name="pan_bearing_seats_on_mast",
    )
    ctx.expect_overlap(
        pan_carriage,
        tower_base,
        axes="xy",
        min_overlap=0.30,
        elem_a=turntable_plate,
        elem_b=mast_bearing_housing,
        name="pan_bearing_has_real_support_area",
    )
    ctx.expect_contact(
        spotlight_head,
        pan_carriage,
        elem_a=trunnion_shaft,
        elem_b=left_bearing_block,
        name="left_trunnion_carried_by_bearing_block",
    )
    ctx.expect_contact(
        spotlight_head,
        pan_carriage,
        elem_a=trunnion_shaft,
        elem_b=right_bearing_block,
        name="right_trunnion_carried_by_bearing_block",
    )
    ctx.expect_contact(
        spotlight_head,
        spotlight_head,
        elem_a=handle_post_left,
        elem_b=rear_housing,
        name="left_handle_post_is_supported_by_housing",
    )
    ctx.expect_contact(
        spotlight_head,
        spotlight_head,
        elem_a=handle_post_right,
        elem_b=rear_housing,
        name="right_handle_post_is_supported_by_housing",
    )

    with ctx.pose({tilt_axis: -0.55}):
        ctx.expect_gap(
            spotlight_head,
            pan_carriage,
            axis="z",
            positive_elem=front_bezel,
            negative_elem=yoke_saddle,
            min_gap=0.035,
            name="front_bezel_clears_saddle_at_low_tilt",
        )

    with ctx.pose({tilt_axis: 1.05}):
        ctx.expect_gap(
            pan_carriage,
            spotlight_head,
            axis="z",
            positive_elem=rear_tie_bar,
            negative_elem=ballast_pod,
            min_gap=0.10,
            name="upper_tilt_clears_rear_tie_bar",
        )

    def _center(aabb):
        (min_pt, max_pt) = aabb
        return tuple((lo + hi) * 0.5 for lo, hi in zip(min_pt, max_pt))

    with ctx.pose({tilt_axis: 0.0}):
        lens_closed = ctx.part_element_world_aabb(spotlight_head, elem=front_lens)
    with ctx.pose({tilt_axis: 0.9}):
        lens_raised = ctx.part_element_world_aabb(spotlight_head, elem=front_lens)
    if lens_closed is None or lens_raised is None:
        ctx.fail("tilt_lifts_front", "front lens world AABB unavailable")
    else:
        closed_center = _center(lens_closed)
        raised_center = _center(lens_raised)
        ctx.check(
            "tilt_lifts_front",
            raised_center[2] > closed_center[2] + 0.20,
            details=(
                f"expected front lens to rise by > 0.20 m, got "
                f"{raised_center[2] - closed_center[2]:.3f} m"
            ),
        )

    with ctx.pose({pan_axis: 0.0, tilt_axis: 0.0}):
        lens_forward = ctx.part_element_world_aabb(spotlight_head, elem=front_lens)
    with ctx.pose({pan_axis: math.pi / 2.0, tilt_axis: 0.0}):
        lens_side = ctx.part_element_world_aabb(spotlight_head, elem=front_lens)
    if lens_forward is None or lens_side is None:
        ctx.fail("pan_swings_head_around_mast", "front lens world AABB unavailable")
    else:
        forward_center = _center(lens_forward)
        side_center = _center(lens_side)
        ctx.check(
            "pan_swings_head_around_mast",
            forward_center[0] > 0.25 and abs(side_center[0]) < 0.10 and side_center[1] > 0.25,
            details=(
                "expected the front lens to start forward of the mast and move to the left side "
                f"after 90 deg pan; got forward={forward_center}, side={side_center}"
            ),
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
