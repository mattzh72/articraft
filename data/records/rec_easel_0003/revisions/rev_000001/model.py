from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import os

try:
    os.getcwd()
except FileNotFoundError:
    os.chdir("/tmp")

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


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def _vec_sub(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _vec_len(v: tuple[float, float, float]) -> float:
    return math.sqrt((v[0] * v[0]) + (v[1] * v[1]) + (v[2] * v[2]))


def _leg_rpy_and_center(
    hinge_xyz: tuple[float, float, float],
    foot_xyz: tuple[float, float, float],
) -> tuple[tuple[float, float, float], tuple[float, float, float], float]:
    foot_vector = _vec_sub(foot_xyz, hinge_xyz)
    length = _vec_len(foot_vector)
    axis_z = tuple(-component / length for component in foot_vector)
    roll = -math.asin(_clamp(axis_z[1], -1.0, 1.0))
    pitch = math.atan2(axis_z[0], axis_z[2])
    center = (foot_vector[0] * 0.5, foot_vector[1] * 0.5, foot_vector[2] * 0.5)
    return (roll, pitch, 0.0), center, length


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tripod_field_easel")

    ash = model.material("ash", rgba=(0.72, 0.60, 0.42, 1.0))
    walnut = model.material("walnut", rgba=(0.47, 0.35, 0.22, 1.0))
    steel = model.material("steel", rgba=(0.23, 0.25, 0.27, 1.0))

    crown_z = 1.505
    front_hinge = (0.0, 0.040, crown_z)
    rear_left_hinge = (-0.038, -0.026, crown_z)
    rear_right_hinge = (0.038, -0.026, crown_z)

    frame = model.part("frame")
    frame.visual(
        Box((0.058, 0.030, 1.43)),
        origin=Origin(xyz=(0.0, 0.0, 0.825)),
        material=ash,
        name="mast",
    )
    frame.visual(
        Box((0.140, 0.080, 0.120)),
        origin=Origin(xyz=(0.0, 0.0, 1.595)),
        material=walnut,
        name="crown_block",
    )
    frame.visual(
        Box((0.124, 0.024, 0.210)),
        origin=Origin(xyz=(0.0, -0.015, 0.455)),
        material=walnut,
        name="lower_block",
    )
    frame.visual(
        Box((0.180, 0.020, 0.010)),
        origin=Origin(xyz=(0.0, 0.0135, 0.395)),
        material=steel,
        name="ledge_mount",
    )
    frame.visual(
        Cylinder(radius=0.0045, length=0.170),
        origin=Origin(xyz=(0.0, 0.028, 0.395), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="ledge_pin",
    )
    frame.visual(
        Box((0.050, 0.018, 0.024)),
        origin=Origin(xyz=(front_hinge[0], 0.031, front_hinge[2])),
        material=steel,
        name="front_hinge_tab",
    )
    frame.visual(
        Box((0.032, 0.018, 0.024)),
        origin=Origin(xyz=(front_hinge[0], 0.028, 1.523)),
        material=steel,
        name="front_hinge_bridge",
    )
    frame.visual(
        Box((0.044, 0.018, 0.024)),
        origin=Origin(xyz=(rear_left_hinge[0], -0.017, rear_left_hinge[2])),
        material=steel,
        name="rear_left_hinge_tab",
    )
    frame.visual(
        Box((0.044, 0.018, 0.024)),
        origin=Origin(xyz=(rear_right_hinge[0], -0.017, rear_right_hinge[2])),
        material=steel,
        name="rear_right_hinge_tab",
    )
    frame.inertial = Inertial.from_geometry(
        Box((0.140, 0.080, 1.56)),
        mass=3.8,
        origin=Origin(xyz=(0.0, 0.0, 0.88)),
    )

    def add_leg(
        name: str,
        hinge_xyz: tuple[float, float, float],
        foot_xyz: tuple[float, float, float],
        *,
        cap_y: float,
        lower: float,
        upper: float,
    ) -> None:
        rpy, _center, length = _leg_rpy_and_center(hinge_xyz, foot_xyz)
        foot_vector = _vec_sub(foot_xyz, hinge_xyz)
        bar_inset = 0.018
        foot_inset = 0.018
        bar_length = length - (bar_inset + foot_inset)
        unit_to_foot = tuple(component / length for component in foot_vector)
        bar_center = tuple(component * (bar_inset + (bar_length * 0.5)) for component in unit_to_foot)
        foot_center = tuple(component * (length - (foot_inset * 0.5)) for component in unit_to_foot)

        leg = model.part(name)
        leg.visual(
            Box((0.046, 0.018, 0.018)),
            origin=Origin(xyz=(0.0, cap_y, -0.008)),
            material=steel,
            name="hinge_cap",
        )
        leg.visual(
            Box((0.028, 0.020, bar_length)),
            origin=Origin(xyz=bar_center, rpy=rpy),
            material=ash,
            name="leg_bar",
        )
        leg.visual(
            Box((0.058, 0.034, 0.018)),
            origin=Origin(xyz=foot_center, rpy=rpy),
            material=walnut,
            name="foot",
        )
        leg.inertial = Inertial.from_geometry(
            Box((0.058, 0.034, bar_length)),
            mass=0.95,
            origin=Origin(xyz=bar_center, rpy=rpy),
        )
        model.articulation(
            f"{name}_hinge",
            ArticulationType.REVOLUTE,
            parent=frame,
            child=leg,
            origin=Origin(xyz=hinge_xyz),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(
                effort=14.0,
                velocity=1.0,
                lower=lower,
                upper=upper,
            ),
        )

    add_leg(
        "front_leg",
        front_hinge,
        (0.0, 0.435, 0.035),
        cap_y=0.009,
        lower=0.0,
        upper=0.28,
    )
    add_leg(
        "rear_left_leg",
        rear_left_hinge,
        (-0.340, -0.305, 0.030),
        cap_y=-0.009,
        lower=-0.28,
        upper=0.0,
    )
    add_leg(
        "rear_right_leg",
        rear_right_hinge,
        (0.340, -0.305, 0.030),
        cap_y=-0.009,
        lower=-0.28,
        upper=0.0,
    )

    canvas_rail = model.part("canvas_rail")
    canvas_rail.visual(
        Box((0.044, 0.018, 0.780)),
        origin=Origin(xyz=(0.0, 0.024, 0.390)),
        material=walnut,
        name="rail_strip",
    )
    canvas_rail.visual(
        Box((0.098, 0.014, 0.022)),
        origin=Origin(xyz=(0.0, 0.020, 0.736)),
        material=steel,
        name="top_clip",
    )
    canvas_rail.visual(
        Box((0.242, 0.052, 0.016)),
        origin=Origin(xyz=(0.0, 0.045, 0.094)),
        material=ash,
        name="rail_shelf",
    )
    canvas_rail.visual(
        Box((0.242, 0.014, 0.026)),
        origin=Origin(xyz=(0.0, 0.064, 0.099)),
        material=walnut,
        name="rail_lip",
    )
    canvas_rail.inertial = Inertial.from_geometry(
        Box((0.242, 0.064, 0.780)),
        mass=1.0,
        origin=Origin(xyz=(0.0, 0.032, 0.390)),
    )
    model.articulation(
        "canvas_rail_slide",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=canvas_rail,
        origin=Origin(xyz=(0.0, 0.0, 0.475)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=0.35,
            lower=0.0,
            upper=0.24,
        ),
    )

    display_ledge = model.part("display_ledge")
    display_ledge.visual(
        Cylinder(radius=0.0065, length=0.148),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="hinge_barrel",
    )
    display_ledge.visual(
        Box((0.170, 0.012, 0.012)),
        origin=Origin(xyz=(0.0, 0.012, -0.006)),
        material=walnut,
        name="tray_back",
    )
    display_ledge.visual(
        Box((0.282, 0.056, 0.014)),
        origin=Origin(xyz=(0.0, 0.036, -0.010)),
        material=ash,
        name="tray",
    )
    display_ledge.visual(
        Box((0.282, 0.012, 0.022)),
        origin=Origin(xyz=(0.0, 0.062, 0.002)),
        material=walnut,
        name="tray_lip",
    )
    display_ledge.inertial = Inertial.from_geometry(
        Box((0.282, 0.060, 0.030)),
        mass=0.55,
        origin=Origin(xyz=(0.0, 0.036, -0.004)),
    )
    model.articulation(
        "display_ledge_hinge",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=display_ledge,
        origin=Origin(xyz=(0.0, 0.028, 0.395)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=1.0,
            lower=-1.0,
            upper=0.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    front_leg = object_model.get_part("front_leg")
    rear_left_leg = object_model.get_part("rear_left_leg")
    rear_right_leg = object_model.get_part("rear_right_leg")
    canvas_rail = object_model.get_part("canvas_rail")
    display_ledge = object_model.get_part("display_ledge")

    front_leg_hinge = object_model.get_articulation("front_leg_hinge")
    rear_left_leg_hinge = object_model.get_articulation("rear_left_leg_hinge")
    rear_right_leg_hinge = object_model.get_articulation("rear_right_leg_hinge")
    canvas_rail_slide = object_model.get_articulation("canvas_rail_slide")
    display_ledge_hinge = object_model.get_articulation("display_ledge_hinge")

    mast = frame.get_visual("mast")
    lower_block = frame.get_visual("lower_block")
    ledge_mount = frame.get_visual("ledge_mount")
    ledge_pin = frame.get_visual("ledge_pin")
    front_hinge_tab = frame.get_visual("front_hinge_tab")
    rear_left_hinge_tab = frame.get_visual("rear_left_hinge_tab")
    rear_right_hinge_tab = frame.get_visual("rear_right_hinge_tab")

    front_hinge_cap = front_leg.get_visual("hinge_cap")
    rear_left_hinge_cap = rear_left_leg.get_visual("hinge_cap")
    rear_right_hinge_cap = rear_right_leg.get_visual("hinge_cap")
    front_foot = front_leg.get_visual("foot")
    rear_left_foot = rear_left_leg.get_visual("foot")
    rear_right_foot = rear_right_leg.get_visual("foot")

    rail_strip = canvas_rail.get_visual("rail_strip")
    rail_shelf = canvas_rail.get_visual("rail_shelf")
    top_clip = canvas_rail.get_visual("top_clip")

    hinge_barrel = display_ledge.get_visual("hinge_barrel")
    tray = display_ledge.get_visual("tray")
    tray_lip = display_ledge.get_visual("tray_lip")

    ctx.allow_overlap(
        frame,
        front_leg,
        reason="Front leg hinge cap wraps a crown pin with localized hinge hardware overlap.",
        elem_a=front_hinge_tab,
        elem_b=front_hinge_cap,
    )
    ctx.allow_overlap(
        frame,
        rear_left_leg,
        reason="Rear left leg hinge cap wraps a crown pin with localized hinge hardware overlap.",
        elem_a=rear_left_hinge_tab,
        elem_b=rear_left_hinge_cap,
    )
    ctx.allow_overlap(
        frame,
        rear_right_leg,
        reason="Rear right leg hinge cap wraps a crown pin with localized hinge hardware overlap.",
        elem_a=rear_right_hinge_tab,
        elem_b=rear_right_hinge_cap,
    )
    ctx.allow_overlap(
        frame,
        display_ledge,
        reason="Display ledge hinge barrel rotates around a steel pin; the pin volume stands in for unmodeled bore clearance.",
        elem_a=ledge_pin,
        elem_b=hinge_barrel,
    )

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_overlaps(max_pose_samples=96)

    ctx.expect_contact(frame, front_leg, elem_a=front_hinge_tab, elem_b=front_hinge_cap)
    ctx.expect_contact(frame, rear_left_leg, elem_a=rear_left_hinge_tab, elem_b=rear_left_hinge_cap)
    ctx.expect_contact(frame, rear_right_leg, elem_a=rear_right_hinge_tab, elem_b=rear_right_hinge_cap)

    ctx.expect_gap(
        canvas_rail,
        frame,
        axis="y",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=rail_strip,
        negative_elem=mast,
    )
    ctx.expect_within(
        canvas_rail,
        frame,
        axes="x",
        inner_elem=rail_strip,
        outer_elem=mast,
        margin=0.008,
    )
    ctx.expect_overlap(canvas_rail, frame, axes="x", elem_a=rail_strip, elem_b=mast, min_overlap=0.040)
    ctx.expect_origin_distance(canvas_rail, frame, axes="x", max_dist=0.005)

    ctx.expect_overlap(frame, display_ledge, axes="x", elem_a=ledge_pin, elem_b=hinge_barrel, min_overlap=0.14)
    ctx.expect_origin_distance(display_ledge, frame, axes="x", max_dist=0.01)
    ctx.expect_gap(
        canvas_rail,
        display_ledge,
        axis="z",
        min_gap=0.10,
        positive_elem=rail_shelf,
        negative_elem=tray,
    )
    ctx.expect_overlap(canvas_rail, display_ledge, axes="x", min_overlap=0.16)
    ctx.expect_within(
        display_ledge,
        frame,
        axes="x",
        inner_elem=hinge_barrel,
        outer_elem=lower_block,
        margin=0.02,
    )

    ctx.check(
        "leg_hinges_are_revolute",
        (
            front_leg_hinge.articulation_type == ArticulationType.REVOLUTE
            and rear_left_leg_hinge.articulation_type == ArticulationType.REVOLUTE
            and rear_right_leg_hinge.articulation_type == ArticulationType.REVOLUTE
        ),
        "All three legs should articulate as crown-mounted revolute hinges.",
    )
    ctx.check(
        "canvas_rail_is_prismatic_vertical",
        canvas_rail_slide.articulation_type == ArticulationType.PRISMATIC
        and tuple(canvas_rail_slide.axis) == (0.0, 0.0, 1.0),
        "Canvas rail should move on a vertical prismatic joint.",
    )
    ctx.check(
        "display_ledge_is_revolute",
        display_ledge_hinge.articulation_type == ArticulationType.REVOLUTE
        and tuple(display_ledge_hinge.axis) == (1.0, 0.0, 0.0),
        "Display ledge should fold on a horizontal revolute hinge.",
    )

    with ctx.pose({canvas_rail_slide: 0.24}):
        ctx.expect_gap(
            canvas_rail,
            frame,
            axis="y",
            max_gap=0.001,
            max_penetration=0.0,
            positive_elem=rail_strip,
            negative_elem=mast,
        )
        ctx.expect_gap(
            canvas_rail,
            display_ledge,
            axis="z",
            min_gap=0.22,
            positive_elem=rail_shelf,
            negative_elem=tray,
        )
        raised_shelf_aabb = ctx.part_element_world_aabb(canvas_rail, elem=rail_shelf)
        raised_clip_aabb = ctx.part_element_world_aabb(canvas_rail, elem=top_clip)
        ctx.check(
            "canvas_rail_raises_supports",
            raised_shelf_aabb is not None
            and raised_clip_aabb is not None
            and raised_shelf_aabb[0][2] > 0.80
            and raised_clip_aabb[1][2] > 1.44,
            "Raised prismatic rail should lift both the support shelf and top clip.",
        )

    with ctx.pose({display_ledge_hinge: -1.05}):
        ctx.expect_overlap(frame, display_ledge, axes="x", elem_a=ledge_pin, elem_b=hinge_barrel, min_overlap=0.14)
        folded_tray_aabb = ctx.part_element_world_aabb(display_ledge, elem=tray)
        ctx.check(
            "display_ledge_folds_down",
            folded_tray_aabb is not None and folded_tray_aabb[1][2] < 0.40,
            "Folded display ledge should swing below its hinge line.",
        )

    with ctx.pose(
        {
            front_leg_hinge: 0.28,
            rear_left_leg_hinge: -0.28,
            rear_right_leg_hinge: -0.28,
        }
    ):
        ctx.expect_contact(frame, front_leg, elem_a=front_hinge_tab, elem_b=front_hinge_cap)
        ctx.expect_contact(frame, rear_left_leg, elem_a=rear_left_hinge_tab, elem_b=rear_left_hinge_cap)
        ctx.expect_contact(frame, rear_right_leg, elem_a=rear_right_hinge_tab, elem_b=rear_right_hinge_cap)
        front_foot_aabb = ctx.part_element_world_aabb(front_leg, elem=front_foot)
        rear_left_foot_aabb = ctx.part_element_world_aabb(rear_left_leg, elem=rear_left_foot)
        rear_right_foot_aabb = ctx.part_element_world_aabb(rear_right_leg, elem=rear_right_foot)
        ctx.check(
            "tripod_footprint_reads_as_tripod",
            front_foot_aabb is not None
            and rear_left_foot_aabb is not None
            and rear_right_foot_aabb is not None
            and front_foot_aabb[0][1] > rear_left_foot_aabb[1][1] + 0.45
            and rear_left_foot_aabb[1][0] < -0.22
            and rear_right_foot_aabb[0][0] > 0.22,
            "Tripod should stand with one forward foot and two splayed rear feet.",
        )

    frame_aabb = ctx.part_world_aabb(frame)
    tray_aabb = ctx.part_element_world_aabb(display_ledge, elem=tray)
    tray_lip_aabb = ctx.part_element_world_aabb(display_ledge, elem=tray_lip)
    ctx.check(
        "overall_height_is_field_easel_scale",
        frame_aabb is not None and (frame_aabb[1][2] - frame_aabb[0][2]) > 1.45,
        "Field easel should be near human standing height.",
    )
    ctx.check(
        "display_ledge_has_retaining_lip",
        tray_aabb is not None
        and tray_lip_aabb is not None
        and tray_lip_aabb[1][2] > tray_aabb[1][2] + 0.005,
        "Display ledge should include a raised retaining lip.",
    )

    for joint in (
        front_leg_hinge,
        rear_left_leg_hinge,
        rear_right_leg_hinge,
        canvas_rail_slide,
        display_ledge_hinge,
    ):
        limits = joint.motion_limits
        if limits is None or limits.lower is None or limits.upper is None:
            continue
        with ctx.pose({joint: limits.lower}):
            ctx.fail_if_isolated_parts(name=f"{joint.name}_lower_no_floating")
            ctx.fail_if_parts_overlap_in_current_pose(name=f"{joint.name}_lower_no_unintended_overlap")
        with ctx.pose({joint: limits.upper}):
            ctx.fail_if_isolated_parts(name=f"{joint.name}_upper_no_floating")
            ctx.fail_if_parts_overlap_in_current_pose(name=f"{joint.name}_upper_no_unintended_overlap")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
