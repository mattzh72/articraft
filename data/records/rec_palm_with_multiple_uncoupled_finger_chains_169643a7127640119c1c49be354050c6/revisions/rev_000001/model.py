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


FLEX_LIMIT = math.pi / 2.0
PALM_WIDTH = 0.105
PALM_LENGTH = 0.110
PALM_THICKNESS = 0.024
PALM_FRONT_Y = PALM_LENGTH / 2.0
FINGER_BASE_LEN = 0.018
PIN_RADIUS = 0.0045
PIN_MATERIAL = "dark_pin"
LINK_MATERIAL = "brushed_aluminum"
BASE_MATERIAL = "black_socket"


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="robotic_hand_palm")
    model.material("palm_gray", rgba=(0.42, 0.45, 0.47, 1.0))
    model.material(BASE_MATERIAL, rgba=(0.055, 0.058, 0.065, 1.0))
    model.material(LINK_MATERIAL, rgba=(0.78, 0.80, 0.78, 1.0))
    model.material(PIN_MATERIAL, rgba=(0.015, 0.016, 0.018, 1.0))
    model.material("rubber_tip", rgba=(0.02, 0.022, 0.024, 1.0))

    palm = model.part("palm")
    palm.visual(
        Box((PALM_WIDTH, PALM_LENGTH, PALM_THICKNESS)),
        origin=Origin(),
        material="palm_gray",
        name="palm_block",
    )
    # A slightly raised front rail makes the palm read as a rigid actuator block
    # with separate socket modules bolted to its leading edge.
    palm.visual(
        Box((PALM_WIDTH * 0.92, 0.010, PALM_THICKNESS + 0.004)),
        origin=Origin(xyz=(0.0, PALM_FRONT_Y - 0.005, 0.0)),
        material="palm_gray",
        name="front_rail",
    )

    flex_limits = MotionLimits(effort=8.0, velocity=2.5, lower=0.0, upper=FLEX_LIMIT)

    def add_pin(part, *, y: float, width: float, name: str) -> None:
        part.visual(
            Cylinder(radius=PIN_RADIUS, length=width + 0.006),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=PIN_MATERIAL,
            name=name,
        )

    def add_link(
        name: str,
        *,
        length: float,
        width: float,
        thickness: float,
        distal_pin: bool,
        tip: bool = False,
    ):
        link = model.part(name)
        link.visual(
            Box((width, length, thickness)),
            origin=Origin(xyz=(0.0, length / 2.0, 0.0)),
            material=LINK_MATERIAL,
            name=f"{name}_bar",
        )
        if distal_pin:
            add_pin(link, y=length - PIN_RADIUS, width=width, name=f"{name}_distal_pin")
        if tip:
            cap_len = 0.008
            link.visual(
                Box((width * 0.96, cap_len, thickness + 0.002)),
                origin=Origin(xyz=(0.0, length + cap_len / 2.0, 0.0)),
                material="rubber_tip",
                name=f"{name}_tip_pad",
            )
        return link

    finger_specs = [
        # x position, base/link width, proximal, middle, distal lengths
        (-0.036, 0.016, 0.042, 0.033, 0.026),
        (-0.012, 0.017, 0.048, 0.038, 0.030),
        (0.012, 0.017, 0.047, 0.037, 0.029),
        (0.036, 0.015, 0.040, 0.031, 0.024),
    ]

    for index, (x, width, proximal_len, middle_len, distal_len) in enumerate(finger_specs):
        base = model.part(f"finger_{index}_base")
        base.visual(
            Box((width + 0.004, FINGER_BASE_LEN, PALM_THICKNESS + 0.004)),
            origin=Origin(xyz=(0.0, -FINGER_BASE_LEN / 2.0, 0.0)),
            material=BASE_MATERIAL,
            name=f"finger_{index}_base_block",
        )
        add_pin(base, y=-PIN_RADIUS, width=width + 0.004, name=f"finger_{index}_base_pin")
        model.articulation(
            f"palm_to_finger_{index}_base",
            ArticulationType.FIXED,
            parent=palm,
            child=base,
            origin=Origin(xyz=(x, PALM_FRONT_Y + FINGER_BASE_LEN, 0.0)),
        )

        proximal = add_link(
            f"finger_{index}_proximal",
            length=proximal_len,
            width=width,
            thickness=0.017,
            distal_pin=True,
        )
        middle = add_link(
            f"finger_{index}_middle",
            length=middle_len,
            width=width * 0.92,
            thickness=0.015,
            distal_pin=True,
        )
        distal = add_link(
            f"finger_{index}_distal",
            length=distal_len,
            width=width * 0.84,
            thickness=0.013,
            distal_pin=False,
            tip=True,
        )

        model.articulation(
            f"finger_{index}_base_hinge",
            ArticulationType.REVOLUTE,
            parent=base,
            child=proximal,
            origin=Origin(),
            axis=(-1.0, 0.0, 0.0),
            motion_limits=flex_limits,
        )
        model.articulation(
            f"finger_{index}_middle_hinge",
            ArticulationType.REVOLUTE,
            parent=proximal,
            child=middle,
            origin=Origin(xyz=(0.0, proximal_len, 0.0)),
            axis=(-1.0, 0.0, 0.0),
            motion_limits=flex_limits,
        )
        model.articulation(
            f"finger_{index}_distal_hinge",
            ArticulationType.REVOLUTE,
            parent=middle,
            child=distal,
            origin=Origin(xyz=(0.0, middle_len, 0.0)),
            axis=(-1.0, 0.0, 0.0),
            motion_limits=flex_limits,
        )

    thumb_base_len = 0.018
    thumb_base = model.part("thumb_base")
    thumb_base.visual(
        Box((thumb_base_len, 0.028, PALM_THICKNESS + 0.004)),
        # The thumb socket frame is its outer hinge line; the block reaches back
        # inward to touch the palm side face.
        origin=Origin(xyz=(thumb_base_len / 2.0, 0.0, 0.0)),
        material=BASE_MATERIAL,
        name="thumb_base_block",
    )
    thumb_base.visual(
        Cylinder(radius=PIN_RADIUS, length=0.034),
        origin=Origin(xyz=(PIN_RADIUS, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=PIN_MATERIAL,
        name="thumb_base_pin",
    )
    model.articulation(
        "palm_to_thumb_base",
        ArticulationType.FIXED,
        parent=palm,
        child=thumb_base,
        origin=Origin(xyz=(-PALM_WIDTH / 2.0 - thumb_base_len, -0.012, 0.0)),
    )

    thumb_proximal = add_link(
        "thumb_proximal",
        length=0.038,
        width=0.018,
        thickness=0.017,
        distal_pin=True,
    )
    thumb_distal = add_link(
        "thumb_distal",
        length=0.030,
        width=0.015,
        thickness=0.014,
        distal_pin=False,
        tip=True,
    )

    model.articulation(
        "thumb_base_hinge",
        ArticulationType.REVOLUTE,
        parent=thumb_base,
        child=thumb_proximal,
        # Rotating the joint frame by 90 degrees makes the thumb project from
        # the side of the palm while keeping each link authored along local +Y.
        origin=Origin(rpy=(0.0, 0.0, math.pi / 2.0)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=flex_limits,
    )
    model.articulation(
        "thumb_distal_hinge",
        ArticulationType.REVOLUTE,
        parent=thumb_proximal,
        child=thumb_distal,
        origin=Origin(xyz=(0.0, 0.038, 0.0)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=flex_limits,
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.
    # If overlap QC reports an intersection, classify it first: intentional
    # embeddings or nested fits should get a scoped allowance; unintended
    # collisions should be fixed in geometry, support, mount, or pose.

    revolute_joints = [
        joint
        for joint in object_model.articulations
        if joint.articulation_type == ArticulationType.REVOLUTE
    ]
    ctx.check(
        "four fingers plus thumb use fourteen flexion hinges",
        len(revolute_joints) == 14,
        details=f"found {len(revolute_joints)} revolute joints",
    )
    for joint in revolute_joints:
        limits = joint.motion_limits
        ctx.check(
            f"{joint.name} has ninety degree flexion range",
            limits is not None
            and abs((limits.lower or 0.0) - 0.0) < 1e-6
            and abs((limits.upper or 0.0) - FLEX_LIMIT) < 1e-6,
            details=f"limits={limits}",
        )
        ctx.check(
            f"{joint.name} hinge axis crosses link width",
            tuple(round(v, 6) for v in joint.axis) == (-1.0, 0.0, 0.0),
            details=f"axis={joint.axis}",
        )

    for index in range(4):
        base = object_model.get_part(f"finger_{index}_base")
        proximal = object_model.get_part(f"finger_{index}_proximal")
        middle = object_model.get_part(f"finger_{index}_middle")
        distal = object_model.get_part(f"finger_{index}_distal")
        ctx.expect_contact(
            base,
            proximal,
            elem_a=f"finger_{index}_base_block",
            elem_b=f"finger_{index}_proximal_bar",
            name=f"finger {index} base hinge is seated",
        )
        ctx.expect_contact(
            proximal,
            middle,
            elem_a=f"finger_{index}_proximal_bar",
            elem_b=f"finger_{index}_middle_bar",
            name=f"finger {index} middle hinge is seated",
        )
        ctx.expect_contact(
            middle,
            distal,
            elem_a=f"finger_{index}_middle_bar",
            elem_b=f"finger_{index}_distal_bar",
            name=f"finger {index} distal hinge is seated",
        )

    ctx.expect_contact(
        "thumb_base",
        "thumb_proximal",
        elem_a="thumb_base_block",
        elem_b="thumb_proximal_bar",
        name="thumb base hinge is seated",
    )
    ctx.expect_contact(
        "thumb_proximal",
        "thumb_distal",
        elem_a="thumb_proximal_bar",
        elem_b="thumb_distal_bar",
        name="thumb distal hinge is seated",
    )

    distal = object_model.get_part("finger_1_distal")
    rest_aabb = ctx.part_world_aabb(distal)
    with ctx.pose(
        {
            "finger_1_base_hinge": FLEX_LIMIT,
            "finger_1_middle_hinge": FLEX_LIMIT,
            "finger_1_distal_hinge": FLEX_LIMIT,
        }
    ):
        flexed_aabb = ctx.part_world_aabb(distal)
    ctx.check(
        "finger flexion curls the chain downward",
        rest_aabb is not None
        and flexed_aabb is not None
        and flexed_aabb[0][2] < rest_aabb[0][2] - 0.020,
        details=f"rest={rest_aabb}, flexed={flexed_aabb}",
    )

    thumb_distal = object_model.get_part("thumb_distal")
    thumb_rest = ctx.part_world_aabb(thumb_distal)
    with ctx.pose({"thumb_base_hinge": FLEX_LIMIT, "thumb_distal_hinge": FLEX_LIMIT}):
        thumb_flexed = ctx.part_world_aabb(thumb_distal)
    ctx.check(
        "thumb flexion curls the thumb downward",
        thumb_rest is not None
        and thumb_flexed is not None
        and thumb_flexed[1][2] < thumb_rest[0][2] - 0.015,
        details=f"rest={thumb_rest}, flexed={thumb_flexed}",
    )

    return ctx.report()


object_model = build_object_model()
