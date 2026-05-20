from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="multi_link_flat_bar_lever")

    blued_steel = model.material("blued_steel", rgba=(0.05, 0.065, 0.075, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.02, 0.022, 0.024, 1.0))
    brushed = model.material("brushed_steel", rgba=(0.62, 0.62, 0.58, 1.0))
    drive_red = model.material("drive_red", rgba=(0.78, 0.08, 0.04, 1.0))
    coupler_orange = model.material("coupler_orange", rgba=(0.95, 0.42, 0.05, 1.0))
    follower_blue = model.material("follower_blue", rgba=(0.08, 0.23, 0.72, 1.0))
    output_green = model.material("output_green", rgba=(0.10, 0.48, 0.20, 1.0))
    rubber = model.material("rubber", rgba=(0.015, 0.015, 0.015, 1.0))

    bar_width = 0.060
    bar_thick = 0.012
    eye_radius = 0.034
    pin_radius = 0.010
    layer_gap = 0.018

    def add_bar(
        part,
        *,
        length: float,
        layer_z: float,
        material: Material,
        distal_pin: bool = True,
        handle: bool = False,
    ) -> None:
        """Add a straight flat bar whose local origin is the proximal pivot."""
        span_len = length - 2.0 * eye_radius + 0.008
        part.visual(
            Box((span_len, bar_width, bar_thick)),
            origin=Origin(xyz=(length / 2.0, 0.0, layer_z)),
            material=material,
            name="bar_span",
        )
        part.visual(
            Cylinder(radius=eye_radius, length=bar_thick),
            origin=Origin(xyz=(0.0, 0.0, layer_z)),
            material=material,
            name="proximal_eye",
        )
        part.visual(
            Cylinder(radius=eye_radius, length=bar_thick),
            origin=Origin(xyz=(length, 0.0, layer_z)),
            material=material,
            name="distal_eye",
        )
        if distal_pin:
            part.visual(
                Cylinder(radius=pin_radius, length=bar_thick + layer_gap + 0.008),
                origin=Origin(xyz=(length, 0.0, layer_gap / 2.0)),
                material=dark_steel,
                name="distal_pin",
            )
            part.visual(
                Cylinder(radius=pin_radius * 1.55, length=0.006),
                origin=Origin(xyz=(length, 0.0, layer_gap + bar_thick / 2.0 + 0.004)),
                material=brushed,
                name="pin_cap",
            )
        if handle:
            part.visual(
                Box((0.120, bar_width * 0.78, bar_thick)),
                origin=Origin(xyz=(-0.092, 0.0, layer_z)),
                material=material,
                name="handle_shank",
            )
            part.visual(
                Cylinder(radius=0.026, length=0.105),
                origin=Origin(xyz=(-0.172, 0.0, layer_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
                material=rubber,
                name="handle_grip",
            )

    base = model.part("base")
    base.visual(
        Box((0.82, 0.24, 0.026)),
        origin=Origin(xyz=(0.30, 0.0, 0.013)),
        material=blued_steel,
        name="base_plate",
    )
    base.visual(
        Box((0.11, 0.020, 0.070)),
        origin=Origin(xyz=(0.0, -0.052, 0.061)),
        material=blued_steel,
        name="pivot_cheek_0",
    )
    base.visual(
        Box((0.11, 0.020, 0.070)),
        origin=Origin(xyz=(0.0, 0.052, 0.061)),
        material=blued_steel,
        name="pivot_cheek_1",
    )
    base.visual(
        Cylinder(radius=0.018, length=0.092),
        origin=Origin(xyz=(0.0, 0.0, 0.064)),
        material=dark_steel,
        name="ground_pin",
    )
    for i, x in enumerate((-0.03, 0.63)):
        base.visual(
            Cylinder(radius=0.020, length=0.006),
            origin=Origin(xyz=(x, -0.085, 0.029)),
            material=brushed,
            name=f"mount_bolt_{i}_0",
        )
        base.visual(
            Cylinder(radius=0.020, length=0.006),
            origin=Origin(xyz=(x, 0.085, 0.029)),
            material=brushed,
            name=f"mount_bolt_{i}_1",
        )

    drive = model.part("drive_lever")
    add_bar(drive, length=0.320, layer_z=0.0, material=drive_red, handle=True)

    coupler = model.part("coupler_bar")
    add_bar(coupler, length=0.270, layer_z=layer_gap, material=coupler_orange)

    follower = model.part("follower_bar")
    add_bar(follower, length=0.250, layer_z=0.0, material=follower_blue)

    output = model.part("output_lever")
    add_bar(output, length=0.300, layer_z=layer_gap, material=output_green, distal_pin=False)
    output.visual(
        Box((0.115, bar_width * 0.82, bar_thick)),
        origin=Origin(xyz=(0.352, 0.0, layer_gap)),
        material=output_green,
        name="output_tip",
    )
    output.visual(
        Cylinder(radius=0.023, length=0.090),
        origin=Origin(xyz=(0.420, 0.0, layer_gap), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=rubber,
        name="output_grip",
    )

    pivot_z = 0.064
    model.articulation(
        "base_to_drive",
        ArticulationType.REVOLUTE,
        parent=base,
        child=drive,
        origin=Origin(xyz=(0.0, 0.0, pivot_z), rpy=(0.0, 0.0, math.radians(11.0))),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.4, lower=math.radians(-35.0), upper=math.radians(62.0)),
    )
    model.articulation(
        "drive_to_coupler",
        ArticulationType.REVOLUTE,
        parent=drive,
        child=coupler,
        origin=Origin(xyz=(0.320, 0.0, 0.0), rpy=(0.0, 0.0, math.radians(38.0))),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.0, lower=math.radians(-70.0), upper=math.radians(70.0)),
    )
    model.articulation(
        "coupler_to_follower",
        ArticulationType.REVOLUTE,
        parent=coupler,
        child=follower,
        origin=Origin(xyz=(0.270, 0.0, 0.0), rpy=(0.0, 0.0, math.radians(-58.0))),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=2.0, lower=math.radians(-80.0), upper=math.radians(65.0)),
    )
    model.articulation(
        "follower_to_output",
        ArticulationType.REVOLUTE,
        parent=follower,
        child=output,
        origin=Origin(xyz=(0.250, 0.0, 0.0), rpy=(0.0, 0.0, math.radians(42.0))),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=2.0, lower=math.radians(-60.0), upper=math.radians(80.0)),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    drive = object_model.get_part("drive_lever")
    coupler = object_model.get_part("coupler_bar")
    follower = object_model.get_part("follower_bar")
    output = object_model.get_part("output_lever")

    ctx.allow_overlap(
        base,
        drive,
        elem_a="ground_pin",
        elem_b="proximal_eye",
        reason="The base pivot pin is intentionally captured through the drive lever eye.",
    )
    ctx.expect_within(
        base,
        drive,
        axes="xy",
        inner_elem="ground_pin",
        outer_elem="proximal_eye",
        margin=0.001,
        name="base pivot pin sits inside drive eye",
    )
    ctx.expect_overlap(
        base,
        drive,
        axes="z",
        elem_a="ground_pin",
        elem_b="proximal_eye",
        min_overlap=0.010,
        name="base pivot pin passes through drive eye",
    )

    captured_pin_pairs = (
        (drive, coupler, "drive distal pin captures coupler eye"),
        (coupler, follower, "coupler distal pin captures follower eye"),
        (follower, output, "follower distal pin captures output eye"),
    )
    for parent, child, label in captured_pin_pairs:
        ctx.allow_overlap(
            parent,
            child,
            elem_a="distal_pin",
            elem_b="proximal_eye",
            reason=f"{label}; the small solid pin represents the hinge shaft through a flat-bar eye.",
        )
        ctx.expect_within(
            parent,
            child,
            axes="xy",
            inner_elem="distal_pin",
            outer_elem="proximal_eye",
            margin=0.001,
            name=f"{label} centered",
        )
        ctx.expect_overlap(
            parent,
            child,
            axes="z",
            elem_a="distal_pin",
            elem_b="proximal_eye",
            min_overlap=0.010,
            name=f"{label} retained through thickness",
        )

    ctx.expect_gap(
        coupler,
        drive,
        axis="z",
        positive_elem="proximal_eye",
        negative_elem="distal_eye",
        min_gap=0.004,
        max_gap=0.008,
        name="drive and coupler eyes are separated by a spacer gap",
    )
    ctx.expect_gap(
        coupler,
        follower,
        axis="z",
        positive_elem="distal_eye",
        negative_elem="proximal_eye",
        min_gap=0.004,
        max_gap=0.008,
        name="coupler and follower eyes are separated by a spacer gap",
    )
    ctx.expect_gap(
        output,
        follower,
        axis="z",
        positive_elem="proximal_eye",
        negative_elem="distal_eye",
        min_gap=0.004,
        max_gap=0.008,
        name="follower and output eyes are separated by a spacer gap",
    )

    revolute_names = (
        "base_to_drive",
        "drive_to_coupler",
        "coupler_to_follower",
        "follower_to_output",
    )
    ctx.check(
        "mechanism has four rotary pivots",
        all(object_model.get_articulation(name).articulation_type == ArticulationType.REVOLUTE for name in revolute_names),
        details=f"expected revolute joints: {revolute_names}",
    )

    def _aabb_center(aabb):
        lo, hi = aabb
        return tuple((lo[i] + hi[i]) / 2.0 for i in range(3))

    drive_joint = object_model.get_articulation("base_to_drive")
    rest_tip = _aabb_center(ctx.part_element_world_aabb(drive, elem="distal_eye"))
    with ctx.pose({drive_joint: math.radians(40.0)}):
        swept_tip = _aabb_center(ctx.part_element_world_aabb(drive, elem="distal_eye"))
    ctx.check(
        "drive lever sweeps about ground pivot",
        swept_tip[1] > rest_tip[1] + 0.12,
        details=f"rest distal eye={rest_tip}, swept distal eye={swept_tip}",
    )

    return ctx.report()


object_model = build_object_model()
