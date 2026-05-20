from __future__ import annotations

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


STROKE = 0.055


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wafer_handling_gripper")

    bead_blast = Material("bead_blast_aluminum", rgba=(0.62, 0.64, 0.63, 1.0))
    dark_anodized = Material("black_anodized_aluminum", rgba=(0.04, 0.045, 0.05, 1.0))
    hardened_steel = Material("polished_hardened_steel", rgba=(0.80, 0.82, 0.80, 1.0))
    ceramic = Material("white_ceramic_peek", rgba=(0.92, 0.88, 0.72, 1.0))
    fastener = Material("dark_fastener_heads", rgba=(0.015, 0.015, 0.018, 1.0))
    safety_mark = Material("engraved_gold_mark", rgba=(0.95, 0.68, 0.25, 1.0))

    body = model.part("body")
    # Dense machined center block and bolt-on front way deck.  The deck is a
    # continuous static support for the twin linear guideways and way covers.
    body.visual(
        Box((0.280, 0.115, 0.070)),
        origin=Origin(xyz=(0.0, -0.040, 0.035)),
        material=bead_blast,
        name="center_block",
    )
    body.visual(
        Box((0.205, 0.055, 0.055)),
        origin=Origin(xyz=(0.0, 0.025, 0.042)),
        material=bead_blast,
        name="front_boss",
    )
    body.visual(
        Box((0.425, 0.196, 0.020)),
        origin=Origin(xyz=(0.0, -0.007, 0.076)),
        material=bead_blast,
        name="top_cap_plate",
    )
    body.visual(
        Box((0.400, 0.092, 0.010)),
        origin=Origin(xyz=(0.0, 0.040, 0.075)),
        material=dark_anodized,
        name="way_base",
    )
    body.visual(
        Box((0.380, 0.010, 0.014)),
        origin=Origin(xyz=(0.0, 0.020, 0.087)),
        material=hardened_steel,
        name="rail_rear",
    )
    body.visual(
        Box((0.380, 0.010, 0.014)),
        origin=Origin(xyz=(0.0, 0.066, 0.087)),
        material=hardened_steel,
        name="rail_front",
    )
    body.visual(
        Box((0.365, 0.020, 0.006)),
        origin=Origin(xyz=(0.0, 0.043, 0.086)),
        material=dark_anodized,
        name="way_cover_strip",
    )
    for i, x in enumerate((-0.160, -0.120, -0.080, -0.040, 0.0, 0.040, 0.080, 0.120, 0.160)):
        body.visual(
            Box((0.018, 0.022, 0.007)),
            origin=Origin(xyz=(x, 0.043, 0.091)),
            material=dark_anodized,
            name=f"way_cover_rib_{i}",
        )
    body.visual(
        Box((0.018, 0.100, 0.020)),
        origin=Origin(xyz=(-0.212, 0.043, 0.089)),
        material=dark_anodized,
        name="end_stop_0",
    )
    body.visual(
        Cylinder(radius=0.006, length=0.004),
        origin=Origin(xyz=(-0.212, 0.020, 0.1008)),
        material=fastener,
        name="end_stop_0_screw_rear",
    )
    body.visual(
        Cylinder(radius=0.006, length=0.004),
        origin=Origin(xyz=(-0.212, 0.066, 0.1008)),
        material=fastener,
        name="end_stop_0_screw_front",
    )
    body.visual(
        Box((0.018, 0.100, 0.020)),
        origin=Origin(xyz=(0.212, 0.043, 0.089)),
        material=dark_anodized,
        name="end_stop_1",
    )
    body.visual(
        Cylinder(radius=0.006, length=0.004),
        origin=Origin(xyz=(0.212, 0.020, 0.1008)),
        material=fastener,
        name="end_stop_1_screw_rear",
    )
    body.visual(
        Cylinder(radius=0.006, length=0.004),
        origin=Origin(xyz=(0.212, 0.066, 0.1008)),
        material=fastener,
        name="end_stop_1_screw_front",
    )

    # Rear EOAT mounting features and small cover details.
    body.visual(
        Box((0.190, 0.012, 0.052)),
        origin=Origin(xyz=(0.0, -0.102, 0.040)),
        material=dark_anodized,
        name="mounting_flange",
    )
    body.visual(
        Cylinder(radius=0.012, length=0.006),
        origin=Origin(xyz=(-0.075, -0.109, 0.040), rpy=(1.57079632679, 0.0, 0.0)),
        material=hardened_steel,
        name="mount_boss_0",
    )
    body.visual(
        Cylinder(radius=0.012, length=0.006),
        origin=Origin(xyz=(0.075, -0.109, 0.040), rpy=(1.57079632679, 0.0, 0.0)),
        material=hardened_steel,
        name="mount_boss_1",
    )
    body.visual(
        Box((0.055, 0.020, 0.018)),
        origin=Origin(xyz=(0.0, -0.096, 0.075)),
        material=dark_anodized,
        name="sensor_cover",
    )
    body.visual(
        Box((0.080, 0.004, 0.003)),
        origin=Origin(xyz=(0.0, 0.083, 0.087)),
        material=safety_mark,
        name="pickup_zone_mark",
    )

    def add_jaw(name: str, side: float) -> object:
        """Create one mirrored sliding jaw carriage.

        side is +1 for the left jaw's inward-facing features and -1 for the
        right jaw's inward-facing features in each local carriage frame.
        """
        jaw = model.part(name)
        jaw.visual(
            Box((0.058, 0.075, 0.030)),
            origin=Origin(xyz=(0.0, 0.045, 0.113)),
            material=bead_blast,
            name="bearing_block",
        )
        jaw.visual(
            Box((0.052, 0.014, 0.018)),
            origin=Origin(xyz=(0.0, 0.020, 0.103)),
            material=dark_anodized,
            name="rear_bearing_shoe",
        )
        jaw.visual(
            Box((0.052, 0.014, 0.018)),
            origin=Origin(xyz=(0.0, 0.066, 0.103)),
            material=dark_anodized,
            name="front_bearing_shoe",
        )
        jaw.visual(
            Box((0.026, 0.070, 0.018)),
            origin=Origin(xyz=(side * 0.028, 0.092, 0.126)),
            material=bead_blast,
            name="jaw_neck",
        )
        jaw.visual(
            Box((0.018, 0.055, 0.095)),
            origin=Origin(xyz=(side * 0.043, 0.115, 0.143)),
            material=bead_blast,
            name="tall_finger",
        )
        jaw.visual(
            Box((0.012, 0.285, 0.011)),
            origin=Origin(xyz=(side * 0.052, 0.265, 0.112)),
            material=dark_anodized,
            name="lower_fork",
        )
        jaw.visual(
            Box((0.012, 0.285, 0.011)),
            origin=Origin(xyz=(side * 0.052, 0.265, 0.169)),
            material=dark_anodized,
            name="upper_fork",
        )
        jaw.visual(
            Box((0.016, 0.040, 0.015)),
            origin=Origin(xyz=(side * 0.052, 0.413, 0.112)),
            material=ceramic,
            name="lower_tip_pad",
        )
        jaw.visual(
            Box((0.016, 0.040, 0.015)),
            origin=Origin(xyz=(side * 0.052, 0.413, 0.169)),
            material=ceramic,
            name="upper_tip_pad",
        )
        for i, y in enumerate((0.028, 0.062)):
            jaw.visual(
                Cylinder(radius=0.0065, length=0.004),
                origin=Origin(xyz=(-side * 0.014, y, 0.130)),
                material=fastener,
                name=f"carriage_screw_{i}",
            )
        jaw.visual(
            Cylinder(radius=0.0045, length=0.0035),
            origin=Origin(xyz=(side * 0.052, 0.395, 0.178)),
            material=fastener,
            name="tip_clamp_screw",
        )
        return jaw

    left_jaw = add_jaw("left_jaw", 1.0)
    right_jaw = add_jaw("right_jaw", -1.0)

    model.articulation(
        "left_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=left_jaw,
        origin=Origin(xyz=(-0.130, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.18, lower=0.0, upper=STROKE),
    )
    model.articulation(
        "right_slide",
        ArticulationType.PRISMATIC,
        parent=body,
        child=right_jaw,
        origin=Origin(xyz=(0.130, 0.0, 0.0)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=80.0, velocity=0.18, lower=0.0, upper=STROKE),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    left_jaw = object_model.get_part("left_jaw")
    right_jaw = object_model.get_part("right_jaw")
    left_slide = object_model.get_articulation("left_slide")
    right_slide = object_model.get_articulation("right_slide")

    ctx.expect_contact(
        left_jaw,
        body,
        elem_a="rear_bearing_shoe",
        elem_b="rail_rear",
        contact_tol=0.0005,
        name="left rear shoe rides on rear rail",
    )
    ctx.expect_contact(
        left_jaw,
        body,
        elem_a="front_bearing_shoe",
        elem_b="rail_front",
        contact_tol=0.0005,
        name="left front shoe rides on front rail",
    )
    ctx.expect_contact(
        right_jaw,
        body,
        elem_a="rear_bearing_shoe",
        elem_b="rail_rear",
        contact_tol=0.0005,
        name="right rear shoe rides on rear rail",
    )
    ctx.expect_contact(
        right_jaw,
        body,
        elem_a="front_bearing_shoe",
        elem_b="rail_front",
        contact_tol=0.0005,
        name="right front shoe rides on front rail",
    )
    for jaw, side_name in ((left_jaw, "left"), (right_jaw, "right")):
        ctx.expect_within(
            jaw,
            body,
            axes="x",
            inner_elem="bearing_block",
            outer_elem="rail_front",
            margin=0.0,
            name=f"{side_name} carriage remains over rail length",
        )
        ctx.expect_gap(
            jaw,
            body,
            axis="z",
            positive_elem="bearing_block",
            negative_elem="way_cover_strip",
            min_gap=0.007,
            name=f"{side_name} carriage clears center way cover",
        )
        ctx.expect_gap(
            jaw,
            body,
            axis="y",
            positive_elem="lower_tip_pad",
            negative_elem="top_cap_plate",
            min_gap=0.25,
            name=f"{side_name} jaw tip projects into free pickup zone",
        )

    open_left = ctx.part_world_position(left_jaw)
    open_right = ctx.part_world_position(right_jaw)
    ctx.expect_gap(
        left_jaw,
        body,
        axis="x",
        positive_elem="bearing_block",
        negative_elem="end_stop_0",
        min_gap=0.035,
        name="left carriage clears outer stop at open travel",
    )
    ctx.expect_gap(
        body,
        right_jaw,
        axis="x",
        positive_elem="end_stop_1",
        negative_elem="bearing_block",
        min_gap=0.035,
        name="right carriage clears outer stop at open travel",
    )

    with ctx.pose({left_slide: STROKE, right_slide: STROKE}):
        closed_left = ctx.part_world_position(left_jaw)
        closed_right = ctx.part_world_position(right_jaw)
        ctx.expect_gap(
            right_jaw,
            left_jaw,
            axis="x",
            min_gap=0.028,
            name="closed jaws leave wafer pickup clearance",
        )
        for jaw, side_name in ((left_jaw, "left"), (right_jaw, "right")):
            ctx.expect_within(
                jaw,
                body,
                axes="x",
                inner_elem="bearing_block",
                outer_elem="rail_front",
                margin=0.0,
                name=f"{side_name} closed carriage remains on guideway",
            )
            ctx.expect_gap(
                jaw,
                body,
                axis="z",
                positive_elem="bearing_block",
                negative_elem="way_cover_strip",
                min_gap=0.007,
                name=f"{side_name} closed carriage clears way cover",
            )

    ctx.check(
        "left jaw moves toward center",
        open_left is not None
        and closed_left is not None
        and closed_left[0] > open_left[0] + STROKE * 0.8,
        details=f"open={open_left}, closed={closed_left}",
    )
    ctx.check(
        "right jaw moves toward center",
        open_right is not None
        and closed_right is not None
        and closed_right[0] < open_right[0] - STROKE * 0.8,
        details=f"open={open_right}, closed={closed_right}",
    )

    return ctx.report()


object_model = build_object_model()
