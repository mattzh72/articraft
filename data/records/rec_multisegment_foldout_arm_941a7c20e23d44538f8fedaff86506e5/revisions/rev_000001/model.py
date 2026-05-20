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


LINK_LEN = 0.180
HINGE_Z = 0.080
PIN_ROLL = math.pi / 2.0


steel = Material("dark_powder_coated_steel", color=(0.10, 0.12, 0.13, 1.0))
cover = Material("pressed_steel_covers", color=(0.17, 0.21, 0.23, 1.0))
edge = Material("worn_dark_edges", color=(0.045, 0.050, 0.052, 1.0))
pin_mat = Material("blackened_hardened_pins", color=(0.015, 0.015, 0.014, 1.0))
boss_mat = Material("machined_boss_faces", color=(0.55, 0.57, 0.55, 1.0))
stop_mat = Material("zinc_stop_tabs", color=(0.70, 0.64, 0.42, 1.0))


def _pin_origin(x: float = 0.0, y: float = 0.0, z: float = 0.0) -> Origin:
    return Origin(xyz=(x, y, z), rpy=(PIN_ROLL, 0.0, 0.0))


def _add_link_body(part, *, terminal: bool = False) -> None:
    """Add one short boxed boom link with a central input boss."""

    # Central boxed section with pressed cover and side seams.
    part.visual(
        Box((0.122, 0.048, 0.026)),
        origin=Origin(xyz=(0.077, 0.0, 0.0)),
        material=steel,
        name="boxed_web",
    )
    part.visual(
        Box((0.096, 0.038, 0.004)),
        origin=Origin(xyz=(0.078, 0.0, 0.015)),
        material=cover,
        name="stamped_cover",
    )
    part.visual(
        Box((0.112, 0.006, 0.032)),
        origin=Origin(xyz=(0.076, 0.025, 0.0)),
        material=edge,
        name="upper_side_seam",
    )
    part.visual(
        Box((0.112, 0.006, 0.032)),
        origin=Origin(xyz=(0.076, -0.025, 0.0)),
        material=edge,
        name="lower_side_seam",
    )

    # The child-side machined bushing/boss sits between the previous station's
    # side plates.  Its bore is represented by the parent pin overlap allowance.
    part.visual(
        Cylinder(radius=0.024, length=0.042),
        origin=_pin_origin(),
        material=boss_mat,
        name="near_boss",
    )
    part.visual(
        Cylinder(radius=0.014, length=0.050),
        origin=_pin_origin(),
        material=edge,
        name="boss_washer",
    )

    if terminal:
        # A forked pad at the free end, with two tines and machined side bosses.
        part.visual(
            Box((0.040, 0.068, 0.026)),
            origin=Origin(xyz=(LINK_LEN - 0.040, 0.0, 0.0)),
            material=steel,
            name="pad_bridge",
        )
        for side, y in (("upper", 0.030), ("lower", -0.030)):
            part.visual(
                Box((0.082, 0.018, 0.022)),
                origin=Origin(xyz=(LINK_LEN + 0.020, y, 0.0)),
                material=cover,
                name=f"{side}_fork_tine",
            )
            part.visual(
                Cylinder(radius=0.011, length=0.006),
                origin=_pin_origin(LINK_LEN + 0.046, y + (0.010 if y > 0 else -0.010), 0.0),
                material=boss_mat,
                name=f"{side}_pad_boss",
            )
        part.visual(
            Box((0.030, 0.012, 0.010)),
            origin=Origin(xyz=(LINK_LEN - 0.012, 0.0, 0.017)),
            material=stop_mat,
            name="terminal_stop_tab",
        )
        return

    # Outgoing fork station: the bridge ties the boxed web into the two local
    # side plates while leaving a real central pocket for the next link boss.
    part.visual(
        Box((0.038, 0.068, 0.026)),
        origin=Origin(xyz=(LINK_LEN - 0.050, 0.0, 0.0)),
        material=steel,
        name="fork_bridge",
    )
    for side, y in (("upper", 0.036), ("lower", -0.036)):
        part.visual(
            Box((0.076, 0.012, 0.070)),
            origin=Origin(xyz=(LINK_LEN, y, 0.0)),
            material=cover,
            name=f"{side}_side_plate",
        )
        part.visual(
            Box((0.022, 0.014, 0.008)),
            origin=Origin(xyz=(LINK_LEN - 0.024, y, 0.037)),
            material=stop_mat,
            name=f"{side}_stop_tab",
        )

    part.visual(
        Cylinder(radius=0.006, length=0.094),
        origin=_pin_origin(LINK_LEN, 0.0, 0.0),
        material=pin_mat,
        name="far_pin",
    )
    part.visual(
        Cylinder(radius=0.015, length=0.006),
        origin=_pin_origin(LINK_LEN, 0.050, 0.0),
        material=pin_mat,
        name="upper_pin_cap",
    )
    part.visual(
        Cylinder(radius=0.015, length=0.006),
        origin=_pin_origin(LINK_LEN, -0.050, 0.0),
        material=pin_mat,
        name="lower_pin_cap",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_fold_out_boom")

    base = model.part("base")
    base.visual(
        Box((0.280, 0.170, 0.012)),
        origin=Origin(xyz=(0.060, 0.0, 0.006)),
        material=steel,
        name="base_plate",
    )
    base.visual(
        Box((0.110, 0.064, 0.018)),
        origin=Origin(xyz=(-0.006, 0.0, 0.022)),
        material=cover,
        name="raised_mount",
    )
    for x, y in ((-0.040, -0.055), (-0.040, 0.055), (0.120, -0.055), (0.120, 0.055)):
        base.visual(
            Cylinder(radius=0.010, length=0.003),
            origin=Origin(xyz=(x, y, 0.013)),
            material=pin_mat,
            name=f"mount_bolt_{x}_{y}",
        )
    for side, y in (("upper", 0.036), ("lower", -0.036)):
        base.visual(
            Box((0.066, 0.012, 0.092)),
            origin=Origin(xyz=(0.0, y, HINGE_Z)),
            material=cover,
            name=f"{side}_base_plate",
        )
        base.visual(
            Box((0.052, 0.012, 0.050)),
            origin=Origin(xyz=(-0.010, y, 0.034)),
            material=steel,
            name=f"{side}_base_riser",
        )
        base.visual(
            Box((0.022, 0.014, 0.008)),
            origin=Origin(xyz=(-0.022, y, HINGE_Z + 0.037)),
            material=stop_mat,
            name=f"{side}_base_stop",
        )
    base.visual(
        Cylinder(radius=0.006, length=0.094),
        origin=_pin_origin(0.0, 0.0, HINGE_Z),
        material=pin_mat,
        name="base_pin",
    )
    base.visual(
        Cylinder(radius=0.015, length=0.006),
        origin=_pin_origin(0.0, 0.050, HINGE_Z),
        material=pin_mat,
        name="base_upper_cap",
    )
    base.visual(
        Cylinder(radius=0.015, length=0.006),
        origin=_pin_origin(0.0, -0.050, HINGE_Z),
        material=pin_mat,
        name="base_lower_cap",
    )

    links = []
    for idx in range(4):
        link = model.part(f"link_{idx}")
        _add_link_body(link, terminal=(idx == 3))
        links.append(link)

    fold = math.radians(60.0)
    model.articulation(
        "base_pivot",
        ArticulationType.REVOLUTE,
        parent=base,
        child=links[0],
        origin=Origin(xyz=(0.0, 0.0, HINGE_Z), rpy=(0.0, -fold, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.8, lower=0.0, upper=fold),
    )

    joint_specs = [
        ("pivot_0", links[0], links[1], 2.0 * fold, (0.0, -1.0, 0.0), 2.0 * fold),
        ("pivot_1", links[1], links[2], -2.0 * fold, (0.0, 1.0, 0.0), 2.0 * fold),
        ("pivot_2", links[2], links[3], 2.0 * fold, (0.0, -1.0, 0.0), 2.0 * fold),
    ]
    for name, parent, child, pitch, axis, upper in joint_specs:
        model.articulation(
            name,
            ArticulationType.REVOLUTE,
            parent=parent,
            child=child,
            origin=Origin(xyz=(LINK_LEN, 0.0, 0.0), rpy=(0.0, pitch, 0.0)),
            axis=axis,
            motion_limits=MotionLimits(effort=28.0, velocity=1.8, lower=0.0, upper=upper),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    links = [object_model.get_part(f"link_{idx}") for idx in range(4)]
    joints = [
        object_model.get_articulation("base_pivot"),
        object_model.get_articulation("pivot_0"),
        object_model.get_articulation("pivot_1"),
        object_model.get_articulation("pivot_2"),
    ]

    hinge_pairs = [
        (base, links[0], "base_pin"),
        (links[0], links[1], "far_pin"),
        (links[1], links[2], "far_pin"),
        (links[2], links[3], "far_pin"),
    ]
    for parent, child, pin_name in hinge_pairs:
        for boss_name in ("near_boss", "boss_washer"):
            ctx.allow_overlap(
                parent,
                child,
                elem_a=pin_name,
                elem_b=boss_name,
                reason="The hardened hinge pin is intentionally captured through the machined boss bore.",
            )
            ctx.expect_within(
                parent,
                child,
                axes="xz",
                inner_elem=pin_name,
                outer_elem=boss_name,
                margin=0.002,
                name=f"{parent.name}_{child.name}_{boss_name}_pin_centered",
            )
            ctx.expect_overlap(
                parent,
                child,
                axes="y",
                elem_a=pin_name,
                elem_b=boss_name,
                min_overlap=0.035,
                name=f"{parent.name}_{child.name}_{boss_name}_pin_engaged",
            )

    # Stowed configuration: folded zigzag over the base, compact but above it.
    stowed_tip = ctx.part_element_world_aabb(links[3], elem="upper_fork_tine")
    stowed_link_1 = ctx.part_world_position(links[1])
    stowed_link_2 = ctx.part_world_position(links[2])
    ctx.check(
        "stowed chain zigzags",
        stowed_link_1 is not None
        and stowed_link_2 is not None
        and stowed_link_1[2] > HINGE_Z + 0.12
        and abs(stowed_link_2[2] - HINGE_Z) < 0.025,
        details=f"link_1={stowed_link_1}, link_2={stowed_link_2}",
    )
    ctx.check(
        "stowed reach is compact",
        stowed_tip is not None and stowed_tip[1][0] < 0.46,
        details=f"terminal aabb={stowed_tip}",
    )
    ctx.expect_gap(
        links[1],
        base,
        axis="z",
        min_gap=0.015,
        positive_elem="boxed_web",
        negative_elem="base_plate",
        name="folded link clears base plate",
    )

    # Open configuration: all four pivots hit their stops to form a long reach.
    upper_pose = {joint: joint.motion_limits.upper for joint in joints}
    with ctx.pose(upper_pose):
        open_tip = ctx.part_element_world_aabb(links[3], elem="upper_fork_tine")
        open_link_1 = ctx.part_world_position(links[1])
        open_link_3 = ctx.part_world_position(links[3])
        ctx.check(
            "opened boom reaches long",
            open_tip is not None and open_tip[1][0] > 0.72,
            details=f"terminal aabb={open_tip}",
        )
        ctx.check(
            "opened hinges align in plane",
            open_link_1 is not None
            and open_link_3 is not None
            and abs(open_link_1[2] - HINGE_Z) < 0.015
            and abs(open_link_3[2] - HINGE_Z) < 0.015,
            details=f"link_1={open_link_1}, link_3={open_link_3}",
        )
        ctx.expect_gap(
            links[3],
            base,
            axis="z",
            min_gap=0.015,
            positive_elem="boxed_web",
            negative_elem="base_plate",
            name="opened boom stays above base",
        )

    return ctx.report()


object_model = build_object_model()
