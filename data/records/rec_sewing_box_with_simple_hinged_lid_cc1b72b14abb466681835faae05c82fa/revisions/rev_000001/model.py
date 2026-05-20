from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
)


BODY_W = 0.50
BODY_D = 0.32
BODY_H = 0.205
WALL_T = 0.025
BOTTOM_T = 0.025

HINGE_Y = BODY_D / 2.0 + 0.008
HINGE_Z = BODY_H + 0.019
HINGE_R = 0.014
LID_CLOSED_GAP = 0.002
LID_OPEN_ANGLE = math.radians(105.0)


def _add_box(
    part,
    name: str,
    size: tuple[float, float, float],
    xyz: tuple[float, float, float],
    material: str,
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> None:
    part.visual(Box(size), origin=Origin(xyz=xyz, rpy=rpy), material=material, name=name)


def _add_cylinder_x(
    part,
    name: str,
    radius: float,
    length: float,
    xyz: tuple[float, float, float],
    material: str,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(0.0, math.pi / 2.0, 0.0)),
        material=material,
        name=name,
    )


def _add_cylinder_y(
    part,
    name: str,
    radius: float,
    length: float,
    xyz: tuple[float, float, float],
    material: str,
    *,
    positive: bool = True,
) -> None:
    roll = -math.pi / 2.0 if positive else math.pi / 2.0
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(roll, 0.0, 0.0)),
        material=material,
        name=name,
    )


def _add_z_screw(part, name: str, xyz: tuple[float, float, float], material: str) -> None:
    part.visual(
        Cylinder(radius=0.006, length=0.0025),
        origin=Origin(xyz=xyz),
        material=material,
        name=name,
    )


def _add_front_screw(part, name: str, xyz: tuple[float, float, float], material: str) -> None:
    _add_cylinder_y(part, name, 0.006, 0.003, xyz, material, positive=False)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="field_service_sewing_box")
    model.material("powder_blue", rgba=(0.10, 0.22, 0.34, 1.0))
    model.material("lid_blue", rgba=(0.13, 0.30, 0.45, 1.0))
    model.material("galvanized", rgba=(0.72, 0.74, 0.72, 1.0))
    model.material("dark_rubber", rgba=(0.015, 0.016, 0.015, 1.0))
    model.material("wear_white", rgba=(0.82, 0.83, 0.78, 1.0))
    model.material("shadow", rgba=(0.035, 0.040, 0.045, 1.0))

    body = model.part("body")

    # A shallow, open-topped service tray: thick walls, an inset floor, and
    # bolted wear rails so the cavity reads as usable storage instead of a
    # solid block.
    _add_box(body, "bottom_pan", (BODY_W, BODY_D, BOTTOM_T), (0.0, 0.0, BOTTOM_T / 2.0), "powder_blue")
    _add_box(
        body,
        "front_wall",
        (BODY_W, WALL_T, BODY_H - BOTTOM_T),
        (0.0, -BODY_D / 2.0 + WALL_T / 2.0, BOTTOM_T + (BODY_H - BOTTOM_T) / 2.0),
        "powder_blue",
    )
    _add_box(
        body,
        "rear_wall",
        (BODY_W, WALL_T, BODY_H - BOTTOM_T),
        (0.0, BODY_D / 2.0 - WALL_T / 2.0, BOTTOM_T + (BODY_H - BOTTOM_T) / 2.0),
        "powder_blue",
    )
    _add_box(
        body,
        "side_wall_0",
        (WALL_T, BODY_D, BODY_H - BOTTOM_T),
        (-BODY_W / 2.0 + WALL_T / 2.0, 0.0, BOTTOM_T + (BODY_H - BOTTOM_T) / 2.0),
        "powder_blue",
    )
    _add_box(
        body,
        "side_wall_1",
        (WALL_T, BODY_D, BODY_H - BOTTOM_T),
        (BODY_W / 2.0 - WALL_T / 2.0, 0.0, BOTTOM_T + (BODY_H - BOTTOM_T) / 2.0),
        "powder_blue",
    )

    _add_box(body, "front_rim_cap", (BODY_W, 0.030, 0.010), (0.0, -0.145, BODY_H + 0.005), "galvanized")
    _add_box(body, "rear_rim_cap", (BODY_W, 0.026, 0.010), (0.0, 0.132, BODY_H + 0.005), "galvanized")
    _add_box(body, "side_rim_cap_0", (0.030, BODY_D, 0.010), (-0.235, 0.0, BODY_H + 0.005), "galvanized")
    _add_box(body, "side_rim_cap_1", (0.030, BODY_D, 0.010), (0.235, 0.0, BODY_H + 0.005), "galvanized")

    # Interior dividers are welded to the floor so spools and needles have
    # separate service bays while the lid can expose everything at once.
    _add_box(body, "long_divider", (0.014, BODY_D - 0.080, 0.070), (0.0, -0.010, BOTTOM_T + 0.035), "wear_white")
    _add_box(body, "short_divider_0", (0.170, 0.012, 0.060), (-0.085, 0.040, BOTTOM_T + 0.030), "wear_white")
    _add_box(body, "short_divider_1", (0.170, 0.012, 0.060), (0.085, -0.060, BOTTOM_T + 0.030), "wear_white")
    _add_box(body, "needle_channel", (0.180, 0.018, 0.020), (0.110, 0.105, BOTTOM_T + 0.010), "shadow")

    # Replaceable wear strips and corner bumpers are intentionally proud and
    # bolted on; they overlap the shell within the same link as seated pads.
    for idx, x in enumerate((-0.175, 0.175)):
        _add_box(body, f"bottom_skid_{idx}", (0.205, 0.030, 0.012), (x, 0.0, 0.006), "dark_rubber")
        for y in (-0.095, 0.095):
            _add_z_screw(body, f"skid_screw_{idx}_{0 if y < 0 else 1}", (x, y, 0.0138), "galvanized")

    for idx, (x, y) in enumerate(
        (
            (-0.225, -0.140),
            (0.225, -0.140),
            (-0.225, 0.140),
            (0.225, 0.140),
        )
    ):
        _add_box(body, f"corner_bumper_{idx}", (0.040, 0.040, 0.030), (x, y, 0.040), "dark_rubber")

    # Rear hinge: fixed outer knuckles on bolted leaves.  The child lid link
    # carries the center knuckle and rotates exactly about this visible axis.
    for idx, x in enumerate((-0.172, 0.172)):
        _add_box(body, f"rear_hinge_leaf_{idx}", (0.135, 0.007, 0.060), (x, BODY_D / 2.0 + 0.0035, BODY_H + 0.004), "galvanized")
        _add_cylinder_x(body, f"outer_knuckle_{idx}", HINGE_R, 0.130, (x, HINGE_Y, HINGE_Z), "galvanized")
        _add_front_screw(body, f"rear_leaf_screw_{idx}_0", (x - 0.038, BODY_D / 2.0 + 0.006, BODY_H - 0.008), "galvanized")
        _add_front_screw(body, f"rear_leaf_screw_{idx}_1", (x + 0.038, BODY_D / 2.0 + 0.006, BODY_H - 0.008), "galvanized")

    _add_cylinder_x(body, "pin_head_0", 0.017, 0.010, (-0.242, HINGE_Y, HINGE_Z), "galvanized")
    _add_cylinder_x(body, "pin_head_1", 0.017, 0.010, (0.242, HINGE_Y, HINGE_Z), "galvanized")

    # A chunky front strike plate with visible fasteners gives the workhorse box
    # a serviceable latch target without adding a second motion to this prompt.
    _add_box(body, "front_strike_plate", (0.120, 0.008, 0.070), (0.0, -BODY_D / 2.0 - 0.004, BODY_H - 0.035), "galvanized")
    _add_box(body, "strike_hook", (0.065, 0.014, 0.020), (0.0, -BODY_D / 2.0 - 0.011, BODY_H - 0.063), "galvanized")
    _add_front_screw(body, "strike_screw_0", (-0.040, -BODY_D / 2.0 - 0.006, BODY_H - 0.045), "galvanized")
    _add_front_screw(body, "strike_screw_1", (0.040, -BODY_D / 2.0 - 0.006, BODY_H - 0.045), "galvanized")

    lid = model.part("lid")

    # The lid part frame is the hinge pin axis.  In the closed pose the sheet
    # extends forward along local -Y and slightly below the hinge center.
    lid_center_y = -0.185
    lid_center_z = (BODY_H + 0.010 + LID_CLOSED_GAP + 0.012) - HINGE_Z
    _add_box(lid, "lid_panel", (0.540, 0.320, 0.024), (0.0, lid_center_y, lid_center_z), "lid_blue")
    _add_box(lid, "front_drop_lip", (0.540, 0.020, 0.052), (0.0, -0.345, -0.030), "lid_blue")
    _add_box(lid, "side_drop_lip_0", (0.020, 0.330, 0.048), (-0.270, -0.170, -0.029), "lid_blue")
    _add_box(lid, "side_drop_lip_1", (0.020, 0.330, 0.048), (0.270, -0.170, -0.029), "lid_blue")
    _add_box(lid, "front_wear_strip", (0.440, 0.010, 0.014), (0.0, -0.356, -0.046), "dark_rubber")
    _add_box(lid, "top_service_label", (0.180, 0.070, 0.003), (0.0, -0.165, 0.0085), "wear_white")

    _add_box(lid, "center_hinge_leaf", (0.205, 0.060, 0.007), (0.0, -0.031, -0.009), "galvanized")
    _add_cylinder_x(lid, "center_knuckle", HINGE_R, 0.190, (0.0, 0.0, 0.0), "galvanized")
    for x in (-0.060, 0.060):
        _add_z_screw(lid, f"lid_leaf_screw_{0 if x < 0 else 1}", (x, -0.052, -0.007), "galvanized")

    _add_box(lid, "front_latch_tongue", (0.090, 0.009, 0.030), (0.0, -0.356, -0.068), "galvanized")
    _add_box(lid, "latch_pull_pad", (0.070, 0.014, 0.025), (0.0, -0.368, -0.030), "dark_rubber")
    _add_front_screw(lid, "latch_screw_0", (-0.025, -0.362, -0.030), "galvanized")
    _add_front_screw(lid, "latch_screw_1", (0.025, -0.362, -0.030), "galvanized")

    # Replaceable lid bumpers seat on the metal rim when closed.
    for idx, x in enumerate((-0.205, 0.205)):
        _add_box(lid, f"lid_bumper_{idx}", (0.048, 0.024, 0.010), (x, -0.298, -0.004), "dark_rubber")

    model.articulation(
        "lid_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=lid,
        origin=Origin(xyz=(0.0, HINGE_Y, HINGE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=14.0, velocity=2.0, lower=0.0, upper=LID_OPEN_ANGLE),
        motion_properties=MotionProperties(damping=0.08, friction=0.03),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    lid = object_model.get_part("lid")
    hinge = object_model.get_articulation("lid_hinge")

    ctx.check(
        "lid uses bounded rear hinge",
        hinge.motion_limits is not None
        and hinge.motion_limits.lower == 0.0
        and hinge.motion_limits.upper is not None
        and hinge.motion_limits.upper <= math.radians(110.0),
        details=f"limits={hinge.motion_limits}",
    )

    with ctx.pose({hinge: 0.0}):
        ctx.expect_gap(
            lid,
            body,
            axis="z",
            positive_elem="lid_panel",
            negative_elem="front_rim_cap",
            min_gap=0.001,
            max_gap=0.004,
            name="closed lid clears front service rim",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="xy",
            elem_a="lid_panel",
            elem_b="bottom_pan",
            min_overlap=0.28,
            name="closed lid covers the storage tray",
        )
        ctx.expect_overlap(
            lid,
            body,
            axes="yz",
            elem_a="center_knuckle",
            elem_b="outer_knuckle_0",
            min_overlap=0.020,
            name="hinge knuckles share the pin axis",
        )
        ctx.expect_gap(
            lid,
            body,
            axis="x",
            positive_elem="center_knuckle",
            negative_elem="outer_knuckle_0",
            min_gap=0.004,
            max_gap=0.020,
            name="left hinge knuckle has service clearance",
        )
        ctx.expect_gap(
            body,
            lid,
            axis="x",
            positive_elem="outer_knuckle_1",
            negative_elem="center_knuckle",
            min_gap=0.004,
            max_gap=0.020,
            name="right hinge knuckle has service clearance",
        )

        closed_aabb = ctx.part_element_world_aabb(lid, elem="front_drop_lip")

    with ctx.pose({hinge: LID_OPEN_ANGLE}):
        open_aabb = ctx.part_element_world_aabb(lid, elem="front_drop_lip")
        ctx.expect_overlap(
            lid,
            body,
            axes="yz",
            elem_a="center_knuckle",
            elem_b="outer_knuckle_0",
            min_overlap=0.020,
            name="opened lid stays on hinge axis",
        )

    ctx.check(
        "opening raises front lip for maintenance access",
        closed_aabb is not None and open_aabb is not None and open_aabb[1][2] > closed_aabb[1][2] + 0.25,
        details=f"closed={closed_aabb}, open={open_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
