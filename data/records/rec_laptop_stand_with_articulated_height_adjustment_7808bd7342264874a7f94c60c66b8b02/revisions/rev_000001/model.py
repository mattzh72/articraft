from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Mimic,
    MotionLimits,
    Origin,
    SlotPatternPanelGeometry,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="prismatic_laptop_stand")

    aluminum = model.material("satin_aluminum", rgba=(0.72, 0.74, 0.72, 1.0))
    dark_aluminum = model.material("dark_anodized_aluminum", rgba=(0.10, 0.12, 0.13, 1.0))
    black = model.material("black_rubber", rgba=(0.01, 0.01, 0.01, 1.0))
    pin_metal = model.material("brushed_pin_metal", rgba=(0.55, 0.56, 0.54, 1.0))

    # Object frame: +X points toward the laptop, +Y spans the stand width,
    # and +Z is vertical.  The rear vertical guide is fixed to the base.
    spine_x = -0.18
    guide_top_z = 0.338
    spine_joint_z = 0.210
    lower_pivot_z = 0.180
    side_pivot_y = 0.190
    bracket_y = 0.177
    tray_joint = (0.020, 0.0, 0.376)
    tray_lug_x = 0.155
    tray_lug_z = -0.010
    brace_dx = tray_joint[0] + tray_lug_x
    brace_dz = tray_joint[2] + tray_lug_z - lower_pivot_z
    brace_length = math.hypot(brace_dx, brace_dz)
    brace_angle = math.atan2(brace_dz, brace_dx)

    base = model.part("base")
    base.visual(
        Box((0.52, 0.40, 0.028)),
        origin=Origin(xyz=(0.02, 0.0, 0.014)),
        material=dark_aluminum,
        name="base_plate",
    )
    base.visual(
        Box((0.014, 0.090, 0.310)),
        origin=Origin(xyz=(spine_x - 0.023, 0.0, 0.183)),
        material=aluminum,
        name="guide_rear_web",
    )
    for sign, suffix in ((1.0, "pos"), (-1.0, "neg")):
        base.visual(
            Box((0.070, 0.012, 0.310)),
            origin=Origin(xyz=(spine_x - 0.001, sign * 0.026, 0.183)),
            material=aluminum,
            name=f"guide_cheek_{suffix}",
        )
        base.visual(
            Box((0.11, 0.035, 0.008)),
            origin=Origin(xyz=(0.13, sign * 0.145, 0.032)),
            material=black,
            name=f"base_rubber_{suffix}",
        )

    spine = model.part("spine")
    spine.visual(
        Box((0.032, 0.040, 0.500)),
        origin=Origin(xyz=(0.0, 0.0, 0.100)),
        material=aluminum,
        name="sliding_post",
    )
    spine.visual(
        Box((0.030, 0.342, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, lower_pivot_z)),
        material=pin_metal,
        name="pivot_crossbar",
    )
    spine.visual(
        Box((0.040, 0.012, 0.040)),
        origin=Origin(xyz=(0.0, bracket_y, lower_pivot_z)),
        material=pin_metal,
        name="pivot_block_pos",
    )
    spine.visual(
        Box((0.040, 0.012, 0.040)),
        origin=Origin(xyz=(0.0, -bracket_y, lower_pivot_z)),
        material=pin_metal,
        name="pivot_block_neg",
    )
    spine.visual(
        Box((0.055, 0.115, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, 0.363)),
        material=aluminum,
        name="top_clamp",
    )

    tray = model.part("tray")
    tray_mesh = mesh_from_geometry(
        SlotPatternPanelGeometry(
            (0.430, 0.342),
            0.018,
            slot_size=(0.055, 0.008),
            pitch=(0.082, 0.030),
            frame=0.026,
            corner_radius=0.018,
            slot_angle_deg=0.0,
            stagger=True,
        ),
        "vented_tray_panel",
    )
    tray.visual(
        tray_mesh,
        origin=Origin(xyz=(0.205, 0.0, 0.009)),
        material=dark_aluminum,
        name="tray_panel",
    )
    tray.visual(
        Box((0.025, 0.335, 0.040)),
        origin=Origin(xyz=(0.420, 0.0, 0.038)),
        material=black,
        name="front_lip",
    )
    tray.visual(
        Box((0.042, 0.012, 0.020)),
        origin=Origin(xyz=(tray_lug_x, bracket_y, tray_lug_z)),
        material=pin_metal,
        name="brace_lug_pos",
    )
    tray.visual(
        Box((0.042, 0.012, 0.020)),
        origin=Origin(xyz=(tray_lug_x, -bracket_y, tray_lug_z)),
        material=pin_metal,
        name="brace_lug_neg",
    )

    for sign, suffix in ((1.0, "0"), (-1.0, "1")):
        brace = model.part(f"brace_{suffix}")
        brace.visual(
            Box((brace_length, 0.012, 0.012)),
            origin=Origin(
                xyz=(brace_dx / 2.0, 0.0, brace_dz / 2.0),
                rpy=(0.0, -brace_angle, 0.0),
            ),
            material=pin_metal,
            name="brace_bar",
        )
        brace.visual(
            Cylinder(radius=0.018, length=0.014),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
            material=pin_metal,
            name="lower_eye",
        )
        brace.visual(
            Cylinder(radius=0.018, length=0.014),
            origin=Origin(
                xyz=(brace_dx, 0.0, brace_dz),
                rpy=(-math.pi / 2.0, 0.0, 0.0),
            ),
            material=pin_metal,
            name="upper_eye",
        )

    model.articulation(
        "base_to_spine",
        ArticulationType.PRISMATIC,
        parent=base,
        child=spine,
        origin=Origin(xyz=(spine_x, 0.0, spine_joint_z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=0.18, lower=0.0, upper=0.160),
    )
    model.articulation(
        "spine_to_tray",
        ArticulationType.FIXED,
        parent=spine,
        child=tray,
        origin=Origin(xyz=tray_joint),
    )
    model.articulation(
        "spine_to_brace_0",
        ArticulationType.REVOLUTE,
        parent=spine,
        child="brace_0",
        origin=Origin(xyz=(0.0, side_pivot_y, lower_pivot_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.0, lower=-0.30, upper=0.18),
    )
    model.articulation(
        "spine_to_brace_1",
        ArticulationType.REVOLUTE,
        parent=spine,
        child="brace_1",
        origin=Origin(xyz=(0.0, -side_pivot_y, lower_pivot_z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.0, lower=-0.30, upper=0.18),
        mimic=Mimic(joint="spine_to_brace_0", multiplier=1.0, offset=0.0),
    )

    model.meta["guide_top_z"] = guide_top_z
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    spine = object_model.get_part("spine")
    tray = object_model.get_part("tray")
    brace_0 = object_model.get_part("brace_0")
    brace_1 = object_model.get_part("brace_1")
    spine_slide = object_model.get_articulation("base_to_spine")
    brace_drive = object_model.get_articulation("spine_to_brace_0")
    brace_follow = object_model.get_articulation("spine_to_brace_1")

    ctx.expect_gap(
        spine,
        object_model.get_part("base"),
        axis="z",
        positive_elem="sliding_post",
        negative_elem="base_plate",
        min_gap=0.025,
        name="spine clears base plate while sliding in guide",
    )
    ctx.expect_gap(
        tray,
        spine,
        axis="z",
        positive_elem="tray_panel",
        negative_elem="top_clamp",
        min_gap=0.0,
        max_gap=0.001,
        name="rigid tray rests on spine clamp",
    )
    ctx.expect_gap(
        brace_0,
        spine,
        axis="y",
        positive_elem="lower_eye",
        negative_elem="pivot_block_pos",
        min_gap=0.0,
        max_gap=0.002,
        name="positive brace is seated at spine pivot",
    )
    ctx.expect_gap(
        spine,
        brace_1,
        axis="y",
        positive_elem="pivot_block_neg",
        negative_elem="lower_eye",
        min_gap=0.0,
        max_gap=0.002,
        name="negative brace is seated at spine pivot",
    )
    ctx.expect_gap(
        brace_0,
        tray,
        axis="y",
        positive_elem="upper_eye",
        negative_elem="brace_lug_pos",
        min_gap=0.0,
        max_gap=0.002,
        name="positive brace meets tray corner lug",
    )
    ctx.expect_gap(
        tray,
        brace_1,
        axis="y",
        positive_elem="brace_lug_neg",
        negative_elem="upper_eye",
        min_gap=0.0,
        max_gap=0.002,
        name="negative brace meets tray corner lug",
    )

    rest_spine_pos = ctx.part_world_position(spine)
    rest_tray_pos = ctx.part_world_position(tray)
    with ctx.pose({spine_slide: 0.160}):
        high_spine_pos = ctx.part_world_position(spine)
        high_tray_pos = ctx.part_world_position(tray)
        ctx.expect_overlap(
            spine,
            object_model.get_part("base"),
            axes="z",
            elem_a="sliding_post",
            elem_b="guide_rear_web",
            min_overlap=0.08,
            name="extended spine remains retained in rear guide",
        )

    ctx.check(
        "spine extends vertically and carries the tray",
        rest_spine_pos is not None
        and high_spine_pos is not None
        and rest_tray_pos is not None
        and high_tray_pos is not None
        and high_spine_pos[2] > rest_spine_pos[2] + 0.150
        and high_tray_pos[2] > rest_tray_pos[2] + 0.150,
        details=(
            f"spine rest/high={rest_spine_pos}/{high_spine_pos}, "
            f"tray rest/high={rest_tray_pos}/{high_tray_pos}"
        ),
    )

    ctx.check(
        "side braces share the same driven rotation",
        getattr(brace_follow, "mimic", None) is not None
        and brace_follow.mimic.joint == "spine_to_brace_0"
        and abs(brace_follow.mimic.multiplier - 1.0) < 1e-9,
        details=f"follower mimic={getattr(brace_follow, 'mimic', None)}",
    )
    with ctx.pose({brace_drive: -0.20}):
        upper_0 = ctx.part_element_world_aabb(brace_0, elem="upper_eye")
        upper_1 = ctx.part_element_world_aabb(brace_1, elem="upper_eye")
    if upper_0 is not None and upper_1 is not None:
        center_0 = tuple((upper_0[0][i] + upper_0[1][i]) * 0.5 for i in range(3))
        center_1 = tuple((upper_1[0][i] + upper_1[1][i]) * 0.5 for i in range(3))
        aligned = (
            abs(center_0[0] - center_1[0]) < 0.002
            and abs(center_0[2] - center_1[2]) < 0.002
            and abs(center_0[1] + center_1[1]) < 0.002
        )
    else:
        aligned = False
        center_0 = center_1 = None
    ctx.check(
        "paired braces stay mirrored and aligned when rotated",
        aligned,
        details=f"upper eye centers after rotation: {center_0}, {center_1}",
    )

    return ctx.report()


object_model = build_object_model()
