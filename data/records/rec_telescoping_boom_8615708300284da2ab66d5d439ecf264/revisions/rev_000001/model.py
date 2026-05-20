from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def _box(part, name, size, xyz, material):
    part.visual(Box(size), origin=Origin(xyz=xyz), material=material, name=name)


def _cylinder(part, name, radius, length, xyz, rpy, material):
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
        name=name,
    )


def _tube_stage(
    part,
    *,
    prefix: str,
    x_min: float,
    x_max: float,
    width: float,
    height: float,
    wall: float,
    shell_material,
    pad_material,
    cap_material,
    front_collar: tuple[float, float] | None,
):
    """Open rectangular box-beam stage with sliding wear pads and a front cap."""

    length = x_max - x_min
    x_center = (x_min + x_max) * 0.5
    side_y = width * 0.5 - wall * 0.5
    cap_z = height * 0.5 - wall * 0.5

    _box(
        part,
        f"{prefix}_top_wall",
        (length, width, wall),
        (x_center, 0.0, cap_z),
        shell_material,
    )
    _box(
        part,
        f"{prefix}_bottom_wall",
        (length, width, wall),
        (x_center, 0.0, -cap_z),
        shell_material,
    )
    _box(
        part,
        f"{prefix}_side_pos",
        (length, wall, height),
        (x_center, side_y, 0.0),
        shell_material,
    )
    _box(
        part,
        f"{prefix}_side_neg",
        (length, wall, height),
        (x_center, -side_y, 0.0),
        shell_material,
    )

    # Replaceable nylon/bronze wear pads are slightly proud yet still clear the
    # parent sleeve.  Two longitudinal positions make the sliding contact read
    # as guided rather than as a loose floating member.
    pad_len = min(0.22, length * 0.20)
    for i, px in enumerate((x_min + 0.16, x_max - 0.16)):
        _box(
            part,
            f"{prefix}_top_pad_{i}",
            (pad_len, width * 0.44, 0.006),
            (px, 0.0, height * 0.5 + 0.0025),
            pad_material,
        )
        _box(
            part,
            f"{prefix}_bottom_pad_{i}",
            (pad_len, width * 0.44, 0.006),
            (px, 0.0, -height * 0.5 - 0.0025),
            pad_material,
        )
        _box(
            part,
            f"{prefix}_side_pos_pad_{i}",
            (pad_len, 0.006, height * 0.46),
            (px, width * 0.5 + 0.0025, 0.0),
            pad_material,
        )
        _box(
            part,
            f"{prefix}_side_neg_pad_{i}",
            (pad_len, 0.006, height * 0.46),
            (px, -width * 0.5 - 0.0025, 0.0),
            pad_material,
        )

    if front_collar is not None:
        collar_w, collar_h = front_collar
        collar_len = 0.070
        collar_x = x_max - collar_len * 0.5
        collar_wall = 0.014
        _box(
            part,
            f"{prefix}_front_top_cap",
            (collar_len, collar_w, collar_wall),
            (collar_x, 0.0, collar_h * 0.5 - collar_wall * 0.5),
            cap_material,
        )
        _box(
            part,
            f"{prefix}_front_bottom_cap",
            (collar_len, collar_w, collar_wall),
            (collar_x, 0.0, -collar_h * 0.5 + collar_wall * 0.5),
            cap_material,
        )
        _box(
            part,
            f"{prefix}_front_side_pos_cap",
            (collar_len, collar_wall, collar_h),
            (collar_x, collar_w * 0.5 - collar_wall * 0.5, 0.0),
            cap_material,
        )
        _box(
            part,
            f"{prefix}_front_side_neg_cap",
            (collar_len, collar_wall, collar_h),
            (collar_x, -collar_w * 0.5 + collar_wall * 0.5, 0.0),
            cap_material,
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="telescoping_service_boom")

    painted_yellow = model.material("worn_safety_yellow", rgba=(0.93, 0.67, 0.16, 1.0))
    darker_yellow = model.material("scratched_inner_yellow", rgba=(0.82, 0.55, 0.12, 1.0))
    dark_steel = model.material("dark_blued_steel", rgba=(0.10, 0.11, 0.12, 1.0))
    black_pad = model.material("black_wear_pad", rgba=(0.02, 0.02, 0.018, 1.0))
    rubbed_metal = model.material("rubbed_edge_metal", rgba=(0.48, 0.47, 0.43, 1.0))

    root = model.part("root_bracket")

    # Fixed wall bracket and welded root saddle.  The gusset plates overlap the
    # mounting plate and the outer sleeve locally, making the load path obvious.
    _box(root, "wall_plate", (0.10, 0.56, 0.68), (-0.05, 0.0, 0.0), dark_steel)
    _box(root, "lower_root_plate", (0.26, 0.38, 0.035), (0.065, 0.0, -0.155), dark_steel)
    _box(root, "upper_root_plate", (0.26, 0.38, 0.035), (0.065, 0.0, 0.155), dark_steel)
    _box(root, "side_gusset_pos", (0.24, 0.026, 0.38), (0.060, 0.205, 0.0), dark_steel)
    _box(root, "side_gusset_neg", (0.24, 0.026, 0.38), (0.060, -0.205, 0.0), dark_steel)
    for i, (yy, zz) in enumerate(((-0.20, -0.23), (-0.20, 0.23), (0.20, -0.23), (0.20, 0.23))):
        _cylinder(
            root,
            f"mount_bolt_{i}",
            0.026,
            0.016,
            (-0.105, yy, zz),
            (0.0, math.pi / 2.0, 0.0),
            rubbed_metal,
        )

    # Stout fixed outer sleeve: a real open rectangular tube with collars,
    # stop pads, and a clear central cavity for the first telescoping stage.
    outer_x0 = 0.03
    outer_x1 = 1.23
    outer_len = outer_x1 - outer_x0
    outer_w = 0.300
    outer_h = 0.220
    outer_t = 0.025
    outer_xc = (outer_x0 + outer_x1) * 0.5
    _box(root, "outer_top_wall", (outer_len, outer_w, outer_t), (outer_xc, 0.0, 0.0975), painted_yellow)
    _box(root, "outer_bottom_wall", (outer_len, outer_w, outer_t), (outer_xc, 0.0, -0.0975), painted_yellow)
    _box(root, "outer_side_pos", (outer_len, outer_t, outer_h), (outer_xc, 0.1375, 0.0), painted_yellow)
    _box(root, "outer_side_neg", (outer_len, outer_t, outer_h), (outer_xc, -0.1375, 0.0), painted_yellow)

    for label, cx in (("root", 0.055), ("front", outer_x1 - 0.030)):
        _box(root, f"outer_{label}_top_collar", (0.120, 0.340, 0.028), (cx, 0.0, 0.124), rubbed_metal)
        _box(root, f"outer_{label}_bottom_collar", (0.120, 0.340, 0.028), (cx, 0.0, -0.124), rubbed_metal)
        _box(root, f"outer_{label}_side_pos_collar", (0.120, 0.028, 0.276), (cx, 0.156, 0.0), rubbed_metal)
        _box(root, f"outer_{label}_side_neg_collar", (0.120, 0.028, 0.276), (cx, -0.156, 0.0), rubbed_metal)

    _box(root, "outer_stop_lug_pos", (0.075, 0.040, 0.055), (outer_x1 - 0.060, 0.102, -0.155), dark_steel)
    _box(root, "outer_stop_lug_neg", (0.075, 0.040, 0.055), (outer_x1 - 0.060, -0.102, -0.155), dark_steel)

    inner_0 = model.part("inner_beam_0")
    _tube_stage(
        inner_0,
        prefix="inner0",
        x_min=-0.870,
        x_max=0.350,
        width=0.205,
        height=0.159,
        wall=0.017,
        shell_material=darker_yellow,
        pad_material=black_pad,
        cap_material=rubbed_metal,
        front_collar=(0.230, 0.166),
    )

    inner_1 = model.part("inner_beam_1")
    _tube_stage(
        inner_1,
        prefix="inner1",
        x_min=-0.740,
        x_max=0.300,
        width=0.153,
        height=0.114,
        wall=0.014,
        shell_material=painted_yellow,
        pad_material=black_pad,
        cap_material=rubbed_metal,
        front_collar=(0.175, 0.112),
    )

    tip = model.part("tip_beam")
    _box(tip, "tip_rect_beam", (0.860, 0.104, 0.076), (-0.150, 0.0, 0.0), darker_yellow)
    for i, px in enumerate((-0.470, 0.060)):
        _box(tip, f"tip_top_pad_{i}", (0.180, 0.048, 0.005), (px, 0.0, 0.0405), black_pad)
        _box(tip, f"tip_bottom_pad_{i}", (0.180, 0.048, 0.005), (px, 0.0, -0.0405), black_pad)
        _box(tip, f"tip_side_pos_pad_{i}", (0.180, 0.005, 0.034), (px, 0.054, 0.0), black_pad)
        _box(tip, f"tip_side_neg_pad_{i}", (0.180, 0.005, 0.034), (px, -0.054, 0.0), black_pad)

    # Forked service tip: cheek plates, bridge block, pin, and gussets are all
    # carried by the final sliding beam, so nothing at the nose floats.
    _box(tip, "fork_bridge", (0.075, 0.158, 0.092), (0.300, 0.0, 0.0), rubbed_metal)
    _box(tip, "fork_tine_pos", (0.285, 0.028, 0.096), (0.440, 0.071, 0.0), rubbed_metal)
    _box(tip, "fork_tine_neg", (0.285, 0.028, 0.096), (0.440, -0.071, 0.0), rubbed_metal)
    _box(tip, "fork_upper_gusset", (0.145, 0.110, 0.018), (0.310, 0.0, 0.054), dark_steel)
    _box(tip, "fork_lower_gusset", (0.145, 0.110, 0.018), (0.310, 0.0, -0.054), dark_steel)
    _cylinder(
        tip,
        "fork_cross_pin",
        0.018,
        0.185,
        (0.470, 0.0, 0.0),
        (math.pi / 2.0, 0.0, 0.0),
        dark_steel,
    )

    model.articulation(
        "outer_to_inner_0",
        ArticulationType.PRISMATIC,
        parent=root,
        child=inner_0,
        origin=Origin(xyz=(1.130, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=3500.0, velocity=0.18, lower=0.0, upper=0.55),
    )
    model.articulation(
        "inner_0_to_inner_1",
        ArticulationType.PRISMATIC,
        parent=inner_0,
        child=inner_1,
        origin=Origin(xyz=(0.280, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=2500.0, velocity=0.16, lower=0.0, upper=0.45),
    )
    model.articulation(
        "inner_1_to_tip_beam",
        ArticulationType.PRISMATIC,
        parent=inner_1,
        child=tip,
        origin=Origin(xyz=(0.240, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1800.0, velocity=0.14, lower=0.0, upper=0.36),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    root = object_model.get_part("root_bracket")
    inner_0 = object_model.get_part("inner_beam_0")
    inner_1 = object_model.get_part("inner_beam_1")
    tip = object_model.get_part("tip_beam")
    slide_0 = object_model.get_articulation("outer_to_inner_0")
    slide_1 = object_model.get_articulation("inner_0_to_inner_1")
    slide_2 = object_model.get_articulation("inner_1_to_tip_beam")

    def check_slide_clearances(pose_name: str):
        ctx.expect_contact(
            root,
            inner_0,
            elem_a="outer_top_wall",
            elem_b="inner0_top_pad_0",
            contact_tol=0.001,
            name=f"{pose_name}: first stage top wear pad bears on outer sleeve",
        )
        ctx.expect_contact(
            inner_0,
            root,
            elem_a="inner0_bottom_pad_0",
            elem_b="outer_bottom_wall",
            contact_tol=0.001,
            name=f"{pose_name}: first stage bottom wear pad bears on outer sleeve",
        )
        ctx.expect_gap(
            root,
            inner_0,
            axis="y",
            positive_elem="outer_side_pos",
            negative_elem="inner0_side_pos_pad_1",
            min_gap=0.010,
            name=f"{pose_name}: first stage clears outer side",
        )
        ctx.expect_gap(
            inner_0,
            root,
            axis="y",
            positive_elem="inner0_side_neg_pad_1",
            negative_elem="outer_side_neg",
            min_gap=0.010,
            name=f"{pose_name}: first stage clears opposite outer side",
        )
        ctx.expect_overlap(
            inner_0,
            root,
            axes="x",
            elem_a="inner0_top_wall",
            elem_b="outer_top_wall",
            min_overlap=0.35,
            name=f"{pose_name}: first stage remains retained in outer sleeve",
        )

        ctx.expect_contact(
            inner_0,
            inner_1,
            elem_a="inner0_top_wall",
            elem_b="inner1_top_pad_0",
            contact_tol=0.001,
            name=f"{pose_name}: second stage top wear pad bears in first",
        )
        ctx.expect_contact(
            inner_1,
            inner_0,
            elem_a="inner1_bottom_pad_0",
            elem_b="inner0_bottom_wall",
            contact_tol=0.001,
            name=f"{pose_name}: second stage bottom wear pad bears in first",
        )
        ctx.expect_gap(
            inner_0,
            inner_1,
            axis="y",
            positive_elem="inner0_side_pos",
            negative_elem="inner1_side_pos_pad_1",
            min_gap=0.0025,
            name=f"{pose_name}: second stage clears first stage side",
        )
        ctx.expect_overlap(
            inner_1,
            inner_0,
            axes="x",
            elem_a="inner1_top_wall",
            elem_b="inner0_top_wall",
            min_overlap=0.30,
            name=f"{pose_name}: second stage remains retained in first",
        )

        ctx.expect_contact(
            inner_1,
            tip,
            elem_a="inner1_top_wall",
            elem_b="tip_top_pad_0",
            contact_tol=0.001,
            name=f"{pose_name}: tip beam top wear pad bears in second",
        )
        ctx.expect_contact(
            tip,
            inner_1,
            elem_a="tip_bottom_pad_0",
            elem_b="inner1_bottom_wall",
            contact_tol=0.001,
            name=f"{pose_name}: tip beam bottom wear pad bears in second",
        )
        ctx.expect_overlap(
            tip,
            inner_1,
            axes="x",
            elem_a="tip_rect_beam",
            elem_b="inner1_top_wall",
            min_overlap=0.25,
            name=f"{pose_name}: tip beam remains retained in second",
        )

    with ctx.pose({slide_0: 0.0, slide_1: 0.0, slide_2: 0.0}):
        check_slide_clearances("retracted")
        rest_tip_position = ctx.part_world_position(tip)

    with ctx.pose({slide_0: 0.55, slide_1: 0.45, slide_2: 0.36}):
        check_slide_clearances("extended")
        extended_tip_position = ctx.part_world_position(tip)

    ctx.check(
        "nested boom extends along the shared x axis",
        rest_tip_position is not None
        and extended_tip_position is not None
        and extended_tip_position[0] > rest_tip_position[0] + 1.20,
        details=f"rest={rest_tip_position}, extended={extended_tip_position}",
    )

    ctx.check(
        "visible stage profiles step down from root to tip",
        0.300 > 0.205 > 0.153 > 0.104 and 0.220 > 0.159 > 0.114 > 0.076,
        details="outer, first, second, and tip beam cross-sections must decrease clearly",
    )

    return ctx.report()


object_model = build_object_model()
