from __future__ import annotations

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


PALM_FRONT_X = 0.068
PIVOT_X = 0.092
PIVOT_Z = 0.070
LONG_Y = 0.050
HOOK_Y = -0.050


def _add_support_block(part, *, palm_front_x: float, pivot_x: float) -> None:
    """Machined root yoke whose part frame is on the hinge axis."""

    mount_min_x = palm_front_x - pivot_x
    mount_max_x = -0.007
    mount_size_x = mount_max_x - mount_min_x
    mount_center_x = (mount_min_x + mount_max_x) * 0.5

    part.visual(
        Box((mount_size_x, 0.038, 0.038)),
        origin=Origin(xyz=(mount_center_x, 0.0, -0.027)),
        material="dark_anodized",
        name="mount_block",
    )
    part.visual(
        Box((0.018, 0.006, 0.038)),
        origin=Origin(xyz=(-0.006, 0.0155, 0.0)),
        material="dark_anodized",
        name="side_cheek_0",
    )
    part.visual(
        Box((0.018, 0.006, 0.038)),
        origin=Origin(xyz=(-0.006, -0.0155, 0.0)),
        material="dark_anodized",
        name="side_cheek_1",
    )
    part.visual(
        Cylinder(radius=0.013, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material="brushed_steel",
        name="upper_ear",
    )
    part.visual(
        Cylinder(radius=0.013, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, -0.012)),
        material="brushed_steel",
        name="lower_ear",
    )
    part.visual(
        Box((0.010, 0.026, 0.006)),
        origin=Origin(xyz=(-0.019, 0.0, 0.013)),
        material="rubber_black",
        name="root_stop",
    )
    part.visual(
        Cylinder(radius=0.0042, length=0.002),
        origin=Origin(xyz=(mount_center_x - mount_size_x * 0.5 - 0.001, 0.010, -0.026), rpy=(0.0, 1.5708, 0.0)),
        material="brushed_steel",
        name="cap_screw_0",
    )
    part.visual(
        Cylinder(radius=0.0042, length=0.002),
        origin=Origin(xyz=(mount_center_x - mount_size_x * 0.5 - 0.001, -0.010, -0.026), rpy=(0.0, 1.5708, 0.0)),
        material="brushed_steel",
        name="cap_screw_1",
    )


def _add_finger_link(
    part,
    *,
    length: float,
    body_width: float = 0.014,
    pad: str | None = None,
) -> None:
    """Serial finger link with a center barrel and a distal clevis."""

    part.visual(
        Cylinder(radius=0.011, length=0.016),
        origin=Origin(),
        material="brushed_steel",
        name="proximal_barrel",
    )
    part.visual(
        Box((length - 0.028, body_width, 0.011)),
        origin=Origin(xyz=((0.010 + (length - 0.018)) * 0.5, 0.0, 0.0)),
        material="machined_aluminum",
        name="tapered_web",
    )
    part.visual(
        Box((0.014, 0.034, 0.006)),
        origin=Origin(xyz=(length - 0.021, 0.0, 0.0)),
        material="machined_aluminum",
        name="clevis_bridge",
    )
    part.visual(
        Box((0.018, 0.006, 0.038)),
        origin=Origin(xyz=(length - 0.006, 0.0155, 0.0)),
        material="machined_aluminum",
        name="distal_cheek_0",
    )
    part.visual(
        Box((0.018, 0.006, 0.038)),
        origin=Origin(xyz=(length - 0.006, -0.0155, 0.0)),
        material="machined_aluminum",
        name="distal_cheek_1",
    )
    part.visual(
        Cylinder(radius=0.013, length=0.008),
        origin=Origin(xyz=(length, 0.0, 0.012)),
        material="brushed_steel",
        name="upper_ear",
    )
    part.visual(
        Cylinder(radius=0.013, length=0.008),
        origin=Origin(xyz=(length, 0.0, -0.012)),
        material="brushed_steel",
        name="lower_ear",
    )

    if pad == "straight":
        part.visual(
            Box((0.032, 0.018, 0.012)),
            origin=Origin(xyz=(length + 0.018, -0.004, 0.0)),
            material="rubber_black",
            name="fingertip_pad",
        )
        part.visual(
            Cylinder(radius=0.0030, length=0.002),
            origin=Origin(xyz=(length + 0.012, -0.004, 0.007)),
            material="brushed_steel",
            name="pad_screw_0",
        )
        part.visual(
            Cylinder(radius=0.0030, length=0.002),
            origin=Origin(xyz=(length + 0.024, -0.004, 0.007)),
            material="brushed_steel",
            name="pad_screw_1",
        )
    elif pad == "hooked":
        part.visual(
            Box((0.026, 0.018, 0.012)),
            origin=Origin(xyz=(length + 0.010, 0.010, 0.0), rpy=(0.0, 0.0, 0.45)),
            material="rubber_black",
            name="hook_pad",
        )
        part.visual(
            Box((0.016, 0.020, 0.012)),
            origin=Origin(xyz=(length + 0.000, 0.024, 0.0), rpy=(0.0, 0.0, 1.05)),
            material="rubber_black",
            name="hook_lip",
        )
        part.visual(
            Cylinder(radius=0.0030, length=0.002),
            origin=Origin(xyz=(length + 0.006, 0.012, 0.007)),
            material="brushed_steel",
            name="pad_screw_0",
        )
        part.visual(
            Cylinder(radius=0.0030, length=0.002),
            origin=Origin(xyz=(length + 0.016, 0.016, 0.007)),
            material="brushed_steel",
            name="pad_screw_1",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_robot_pinch_hand")

    model.material("machined_aluminum", rgba=(0.72, 0.76, 0.77, 1.0))
    model.material("dark_anodized", rgba=(0.07, 0.08, 0.09, 1.0))
    model.material("brushed_steel", rgba=(0.58, 0.60, 0.62, 1.0))
    model.material("rubber_black", rgba=(0.015, 0.014, 0.013, 1.0))
    model.material("saddle_insert", rgba=(0.18, 0.20, 0.22, 1.0))
    model.material("warning_blue", rgba=(0.08, 0.22, 0.55, 1.0))

    palm = model.part("palm")
    palm.visual(
        Box((0.136, 0.150, 0.044)),
        origin=Origin(xyz=(0.0, 0.0, 0.022)),
        material="machined_aluminum",
        name="palm_body",
    )
    palm.visual(
        Box((0.092, 0.072, 0.003)),
        origin=Origin(xyz=(-0.010, 0.0, 0.0455)),
        material="saddle_insert",
        name="saddle_pad",
    )
    palm.visual(
        Box((0.004, 0.118, 0.030)),
        origin=Origin(xyz=(-0.064, 0.0, 0.023)),
        material="dark_anodized",
        name="rear_cover_line",
    )
    palm.visual(
        Box((0.090, 0.003, 0.004)),
        origin=Origin(xyz=(-0.010, 0.038, 0.046)),
        material="dark_anodized",
        name="saddle_seam_0",
    )
    palm.visual(
        Box((0.090, 0.003, 0.004)),
        origin=Origin(xyz=(-0.010, -0.038, 0.046)),
        material="dark_anodized",
        name="saddle_seam_1",
    )
    for i, y in enumerate((-0.055, 0.055)):
        palm.visual(
            Cylinder(radius=0.0045, length=0.002),
            origin=Origin(xyz=(-0.028, y, 0.045)),
            material="brushed_steel",
            name=f"palm_screw_{i}",
        )

    long_support = model.part("long_support")
    hook_support = model.part("hook_support")
    _add_support_block(long_support, palm_front_x=PALM_FRONT_X, pivot_x=PIVOT_X)
    _add_support_block(hook_support, palm_front_x=PALM_FRONT_X, pivot_x=PIVOT_X)

    long_proximal = model.part("long_proximal")
    long_middle = model.part("long_middle")
    long_tip = model.part("long_tip")
    _add_finger_link(long_proximal, length=0.075)
    _add_finger_link(long_middle, length=0.060)
    _add_finger_link(long_tip, length=0.048, body_width=0.012, pad="straight")

    hook_proximal = model.part("hook_proximal")
    hook_middle = model.part("hook_middle")
    hook_tip = model.part("hook_tip")
    _add_finger_link(hook_proximal, length=0.058, body_width=0.013)
    _add_finger_link(hook_middle, length=0.046, body_width=0.012)
    _add_finger_link(hook_tip, length=0.032, body_width=0.011, pad="hooked")

    model.articulation(
        "palm_to_long_support",
        ArticulationType.FIXED,
        parent=palm,
        child=long_support,
        origin=Origin(xyz=(PIVOT_X, LONG_Y, PIVOT_Z)),
    )
    model.articulation(
        "palm_to_hook_support",
        ArticulationType.FIXED,
        parent=palm,
        child=hook_support,
        origin=Origin(xyz=(PIVOT_X, HOOK_Y, PIVOT_Z)),
    )

    model.articulation(
        "long_root",
        ArticulationType.REVOLUTE,
        parent=long_support,
        child=long_proximal,
        origin=Origin(),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.5, lower=0.0, upper=0.28),
    )
    model.articulation(
        "long_middle_joint",
        ArticulationType.REVOLUTE,
        parent=long_proximal,
        child=long_middle,
        origin=Origin(xyz=(0.075, 0.0, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.7, lower=0.0, upper=0.23),
    )
    model.articulation(
        "long_tip_joint",
        ArticulationType.REVOLUTE,
        parent=long_middle,
        child=long_tip,
        origin=Origin(xyz=(0.060, 0.0, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=3.0, lower=0.0, upper=0.18),
    )

    model.articulation(
        "hook_root",
        ArticulationType.REVOLUTE,
        parent=hook_support,
        child=hook_proximal,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.5, lower=0.0, upper=0.35),
    )
    model.articulation(
        "hook_middle_joint",
        ArticulationType.REVOLUTE,
        parent=hook_proximal,
        child=hook_middle,
        origin=Origin(xyz=(0.058, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=12.0, velocity=2.7, lower=0.0, upper=0.35),
    )
    model.articulation(
        "hook_tip_joint",
        ArticulationType.REVOLUTE,
        parent=hook_middle,
        child=hook_tip,
        origin=Origin(xyz=(0.046, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=3.0, lower=0.0, upper=0.45),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    palm = object_model.get_part("palm")
    long_support = object_model.get_part("long_support")
    hook_support = object_model.get_part("hook_support")
    long_proximal = object_model.get_part("long_proximal")
    long_tip = object_model.get_part("long_tip")
    hook_proximal = object_model.get_part("hook_proximal")
    hook_tip = object_model.get_part("hook_tip")

    ctx.check(
        "two serial three-joint fingers",
        len([j for j in object_model.articulations if j.articulation_type == ArticulationType.REVOLUTE]) == 6,
        details="The pinch hand should expose three revolute joints on each finger.",
    )
    ctx.expect_contact(
        palm,
        long_support,
        elem_a="palm_body",
        elem_b="mount_block",
        name="long support block is face-mounted to the palm",
    )
    ctx.expect_contact(
        palm,
        hook_support,
        elem_a="palm_body",
        elem_b="mount_block",
        name="hook support block is face-mounted to the palm",
    )
    ctx.expect_gap(
        long_support,
        long_proximal,
        axis="z",
        positive_elem="upper_ear",
        negative_elem="proximal_barrel",
        max_penetration=0.000001,
        max_gap=0.0005,
        name="long root upper clevis clears the barrel",
    )
    ctx.expect_gap(
        long_proximal,
        long_support,
        axis="z",
        positive_elem="proximal_barrel",
        negative_elem="lower_ear",
        max_penetration=0.000001,
        max_gap=0.0005,
        name="long root lower clevis clears the barrel",
    )
    ctx.expect_gap(
        hook_support,
        hook_proximal,
        axis="z",
        positive_elem="upper_ear",
        negative_elem="proximal_barrel",
        max_penetration=0.000001,
        max_gap=0.0005,
        name="hook root upper clevis clears the barrel",
    )
    ctx.expect_gap(
        hook_proximal,
        hook_support,
        axis="z",
        positive_elem="proximal_barrel",
        negative_elem="lower_ear",
        max_penetration=0.000001,
        max_gap=0.0005,
        name="hook root lower clevis clears the barrel",
    )

    long_root = object_model.get_articulation("long_root")
    long_mid = object_model.get_articulation("long_middle_joint")
    long_tip_joint = object_model.get_articulation("long_tip_joint")
    hook_root = object_model.get_articulation("hook_root")
    hook_mid = object_model.get_articulation("hook_middle_joint")
    hook_tip_joint = object_model.get_articulation("hook_tip_joint")

    rest_long = ctx.part_world_position(long_tip)
    rest_hook = ctx.part_world_position(hook_tip)
    with ctx.pose(
        {
            long_root: 0.28,
            long_mid: 0.23,
            long_tip_joint: 0.18,
            hook_root: 0.35,
            hook_mid: 0.35,
            hook_tip_joint: 0.45,
        }
    ):
        flexed_long = ctx.part_world_position(long_tip)
        flexed_hook = ctx.part_world_position(hook_tip)
        ctx.expect_gap(
            long_tip,
            hook_tip,
            axis="x",
            positive_elem="fingertip_pad",
            negative_elem="hook_pad",
            min_gap=0.020,
            name="flexed straight pad clears hooked pad",
        )
        ctx.expect_gap(
            long_tip,
            hook_tip,
            axis="x",
            positive_elem="fingertip_pad",
            negative_elem="hook_lip",
            min_gap=0.020,
            name="flexed straight pad clears hooked lip",
        )

    ctx.check(
        "long finger bends inward",
        rest_long is not None and flexed_long is not None and flexed_long[1] < rest_long[1] - 0.008,
        details=f"rest={rest_long}, flexed={flexed_long}",
    )
    ctx.check(
        "hook finger bends inward",
        rest_hook is not None and flexed_hook is not None and flexed_hook[1] > rest_hook[1] + 0.008,
        details=f"rest={rest_hook}, flexed={flexed_hook}",
    )

    return ctx.report()


object_model = build_object_model()
