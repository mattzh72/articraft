from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    Mimic,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


FINGER_Y = (-0.045, 0.045)
BASE_HINGE_X = 0.075
HINGE_Z = 0.058
HINGE_RADIUS = 0.018
SEGMENT_WIDTH = 0.030
SEGMENT_THICKNESS = 0.022
PROXIMAL_LEN = 0.108
MIDDLE_LEN = 0.095
DISTAL_LEN = 0.078


def _add_segment_visuals(part, length: float, *, has_tip: bool, material: str) -> None:
    """Add the repeated rectangular phalanx and its vertical hinge boss."""
    # The bar begins inside its own hinge boss and stops just shy of the next
    # joint boss, leaving a small visible hinge clearance between adjacent links.
    bar_start = HINGE_RADIUS - 0.006
    bar_end = length - HINGE_RADIUS - 0.001
    part.visual(
        Cylinder(radius=HINGE_RADIUS, length=0.034),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material="steel",
        name="hinge_boss",
    )
    part.visual(
        Cylinder(radius=0.010, length=0.006),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material="dark_fastener",
        name="top_cap",
    )
    part.visual(
        Box((bar_end - bar_start, SEGMENT_WIDTH, SEGMENT_THICKNESS)),
        origin=Origin(xyz=((bar_start + bar_end) / 2.0, 0.0, 0.0)),
        material=material,
        name="phalanx_beam",
    )
    part.visual(
        Box((0.020, SEGMENT_WIDTH * 0.82, SEGMENT_THICKNESS + 0.006)),
        origin=Origin(xyz=(bar_start + 0.018, 0.0, 0.0)),
        material=material,
        name="web_rib",
    )
    if not has_tip:
        for name, y in (("clevis_a", 0.016), ("clevis_b", -0.016)):
            part.visual(
                Box((0.032, 0.006, SEGMENT_THICKNESS + 0.006)),
                origin=Origin(xyz=(length - 0.012, y, 0.0)),
                material=material,
                name=name,
            )
    if has_tip:
        part.visual(
            Box((0.030, SEGMENT_WIDTH + 0.012, SEGMENT_THICKNESS + 0.010)),
            origin=Origin(xyz=(length - 0.006, 0.0, 0.0)),
            material="rubber",
            name="grip_pad",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="parallel_finger_gripper")

    model.material("anodized_black", rgba=(0.02, 0.025, 0.03, 1.0))
    model.material("brushed_aluminum", rgba=(0.72, 0.74, 0.72, 1.0))
    model.material("blue_aluminum", rgba=(0.05, 0.22, 0.75, 1.0))
    model.material("steel", rgba=(0.55, 0.58, 0.60, 1.0))
    model.material("dark_fastener", rgba=(0.005, 0.005, 0.006, 1.0))
    model.material("rubber", rgba=(0.015, 0.014, 0.012, 1.0))

    base = model.part("base")
    base.visual(
        Box((0.205, 0.160, 0.030)),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material="anodized_black",
        name="base_plate",
    )
    base.visual(
        Box((0.060, 0.150, 0.012)),
        origin=Origin(xyz=(-0.055, 0.0, 0.036)),
        material="anodized_black",
        name="rear_rail",
    )
    for idx, y in enumerate(FINGER_Y):
        # A compact fork/clevis pair around each first hinge.  The child hinge
        # boss sits between these cheeks with side clearance.
        for side, sign in (("inner", -1.0), ("outer", 1.0)):
            base.visual(
                Box((0.034, 0.008, 0.046)),
                origin=Origin(
                    xyz=(
                        BASE_HINGE_X,
                        y + sign * (HINGE_RADIUS + 0.011),
                        0.030 + 0.046 / 2.0 - 0.0005,
                    )
                ),
                material="anodized_black",
                name=f"clevis_{idx}_{side}",
            )
        base.visual(
            Box((0.044, 0.064, 0.010)),
            origin=Origin(xyz=(BASE_HINGE_X - 0.005, y, 0.035)),
            material="anodized_black",
            name=f"bearing_pad_{idx}",
        )
        base.visual(
            Cylinder(radius=0.007, length=0.004),
            origin=Origin(xyz=(-0.070, y, 0.044)),
            material="dark_fastener",
            name=f"mount_screw_{idx}",
        )
        base.visual(
            Cylinder(radius=0.006, length=0.044),
            origin=Origin(xyz=(BASE_HINGE_X, y, 0.051)),
            material="steel",
            name=f"base_pin_{idx}",
        )

    segment_lengths = {
        "proximal": PROXIMAL_LEN,
        "middle": MIDDLE_LEN,
        "distal": DISTAL_LEN,
    }
    for finger_idx in range(2):
        for segment_name, length in segment_lengths.items():
            segment = model.part(f"{segment_name}_{finger_idx}")
            _add_segment_visuals(
                segment,
                length,
                has_tip=segment_name == "distal",
                material="blue_aluminum" if finger_idx == 0 else "brushed_aluminum",
            )

    bend_limits = MotionLimits(effort=8.0, velocity=2.4, lower=-0.20, upper=0.82)
    for finger_idx, y in enumerate(FINGER_Y):
        mimic_suffix = "" if finger_idx == 0 else "_0"
        model.articulation(
            f"base_to_proximal_{finger_idx}",
            ArticulationType.REVOLUTE,
            parent=base,
            child=f"proximal_{finger_idx}",
            origin=Origin(xyz=(BASE_HINGE_X, y, HINGE_Z)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=bend_limits,
            mimic=None
            if finger_idx == 0
            else Mimic(joint=f"base_to_proximal{mimic_suffix}", multiplier=1.0),
        )
        model.articulation(
            f"proximal_to_middle_{finger_idx}",
            ArticulationType.REVOLUTE,
            parent=f"proximal_{finger_idx}",
            child=f"middle_{finger_idx}",
            origin=Origin(xyz=(PROXIMAL_LEN, 0.0, 0.0)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=bend_limits,
            mimic=None
            if finger_idx == 0
            else Mimic(joint=f"proximal_to_middle{mimic_suffix}", multiplier=1.0),
        )
        model.articulation(
            f"middle_to_distal_{finger_idx}",
            ArticulationType.REVOLUTE,
            parent=f"middle_{finger_idx}",
            child=f"distal_{finger_idx}",
            origin=Origin(xyz=(MIDDLE_LEN, 0.0, 0.0)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=bend_limits,
            mimic=None
            if finger_idx == 0
            else Mimic(joint=f"middle_to_distal{mimic_suffix}", multiplier=1.0),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    proximal_0 = object_model.get_part("proximal_0")
    proximal_1 = object_model.get_part("proximal_1")
    middle_0 = object_model.get_part("middle_0")
    distal_0 = object_model.get_part("distal_0")

    for finger_idx in range(2):
        ctx.allow_overlap(
            "base",
            f"proximal_{finger_idx}",
            elem_a=f"base_pin_{finger_idx}",
            elem_b="hinge_boss",
            reason="The fixed base pin is intentionally captured inside the first hinge boss.",
        )
        ctx.expect_within(
            "base",
            f"proximal_{finger_idx}",
            axes="xy",
            inner_elem=f"base_pin_{finger_idx}",
            outer_elem="hinge_boss",
            margin=0.001,
            name=f"base pin {finger_idx} sits inside hinge boss",
        )
        ctx.expect_overlap(
            "base",
            f"proximal_{finger_idx}",
            axes="z",
            elem_a=f"base_pin_{finger_idx}",
            elem_b="hinge_boss",
            min_overlap=0.025,
            name=f"base pin {finger_idx} passes through hinge height",
        )
        for parent_name, child_name in (
            (f"proximal_{finger_idx}", f"middle_{finger_idx}"),
            (f"middle_{finger_idx}", f"distal_{finger_idx}"),
        ):
            for clevis_name in ("clevis_a", "clevis_b"):
                ctx.allow_overlap(
                    parent_name,
                    child_name,
                    elem_a=clevis_name,
                    elem_b="hinge_boss",
                    reason="The rectangular clevis tab intentionally captures the adjacent hinge boss.",
                )
                ctx.expect_overlap(
                    parent_name,
                    child_name,
                    axes="xz",
                    elem_a=clevis_name,
                    elem_b="hinge_boss",
                    min_overlap=0.010,
                    name=f"{parent_name} {clevis_name} captures {child_name}",
                )

    ctx.check(
        "two mirrored chain rows",
        len([p for p in object_model.parts if p.name.endswith("_0")]) == 3
        and len([p for p in object_model.parts if p.name.endswith("_1")]) == 3,
        details="Expected three phalanx links in each of the two parallel rows.",
    )
    ctx.check(
        "six serial hinge joints",
        sum(j.articulation_type == ArticulationType.REVOLUTE for j in object_model.articulations) == 6,
        details="Each of the two rows should have base, middle, and distal hinge joints.",
    )
    ctx.expect_origin_distance(
        proximal_0,
        proximal_1,
        axes="y",
        min_dist=0.085,
        max_dist=0.095,
        name="parallel rows keep fixed spacing",
    )
    ctx.expect_overlap(
        middle_0,
        proximal_0,
        axes="yz",
        min_overlap=0.020,
        name="adjacent phalanxes share hinge-height envelope",
    )

    driver_joints = {
        object_model.get_articulation("base_to_proximal_0"): 0.40,
        object_model.get_articulation("proximal_to_middle_0"): 0.32,
        object_model.get_articulation("middle_to_distal_0"): 0.24,
    }
    rest_tip_position = ctx.part_world_position(distal_0)
    with ctx.pose(driver_joints):
        curled_tip_position = ctx.part_world_position(distal_0)
    ctx.check(
        "serial chain curls through hinge stack",
        rest_tip_position is not None
        and curled_tip_position is not None
        and curled_tip_position[1] > rest_tip_position[1] + 0.035,
        details=f"rest={rest_tip_position}, curled={curled_tip_position}",
    )

    return ctx.report()


object_model = build_object_model()
