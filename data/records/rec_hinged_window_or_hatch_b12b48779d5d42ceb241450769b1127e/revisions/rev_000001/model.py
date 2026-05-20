from __future__ import annotations

import math

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


SASH_JOINT = "frame_to_sash"
SASH_UPPER = 0.65


def _arm_angle(dy: float, dz: float) -> float:
    """Angle about +X that rotates local +Z toward a world/local YZ vector."""

    return math.atan2(-dy, dz)


def _rot_x(point: tuple[float, float, float], angle: float) -> tuple[float, float, float]:
    x, y, z = point
    c = math.cos(angle)
    s = math.sin(angle)
    return (x, y * c - z * s, y * s + z * c)


def _inv_rot_x(point: tuple[float, float, float], angle: float) -> tuple[float, float, float]:
    return _rot_x(point, -angle)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="greenhouse_vent_window")

    aluminium = Material("satin_aluminium", rgba=(0.72, 0.76, 0.74, 1.0))
    dark_aluminium = Material("dark_anodized_edges", rgba=(0.20, 0.23, 0.22, 1.0))
    glass = Material("greenhouse_glass", rgba=(0.62, 0.86, 0.92, 0.34))
    rubber = Material("black_gasket", rgba=(0.025, 0.030, 0.028, 1.0))
    hardware = Material("stainless_hardware", rgba=(0.82, 0.82, 0.78, 1.0))

    frame = model.part("frame")
    frame.visual(Box((1.05, 0.055, 0.050)), origin=Origin(xyz=(0.0, 0.0, 0.680)), material=aluminium, name="top_rail")
    frame.visual(Box((1.05, 0.055, 0.050)), origin=Origin(xyz=(0.0, 0.0, 0.025)), material=aluminium, name="bottom_rail")
    frame.visual(Box((0.050, 0.055, 0.700)), origin=Origin(xyz=(-0.500, 0.0, 0.350)), material=aluminium, name="jamb_0")
    frame.visual(Box((0.050, 0.055, 0.700)), origin=Origin(xyz=(0.500, 0.0, 0.350)), material=aluminium, name="jamb_1")
    frame.visual(Box((0.930, 0.010, 0.030)), origin=Origin(xyz=(0.0, 0.030, 0.655)), material=rubber, name="top_seal")
    frame.visual(Box((0.930, 0.010, 0.030)), origin=Origin(xyz=(0.0, 0.030, 0.055)), material=rubber, name="bottom_seal")
    frame.visual(Box((0.030, 0.010, 0.610)), origin=Origin(xyz=(-0.465, 0.030, 0.360)), material=rubber, name="side_seal_0")
    frame.visual(Box((0.030, 0.010, 0.610)), origin=Origin(xyz=(0.465, 0.030, 0.360)), material=rubber, name="side_seal_1")

    # A visible hinge pin fixed to the head rail marks the horizontal upper axis.
    frame.visual(
        Cylinder(radius=0.007, length=0.910),
        origin=Origin(xyz=(0.0, 0.072, 0.646), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hardware,
        name="top_hinge_pin",
    )
    frame.visual(
        Box((0.930, 0.055, 0.018)),
        origin=Origin(xyz=(0.0, 0.052, 0.655)),
        material=hardware,
        name="top_hinge_leaf",
    )

    sash = model.part("sash")
    sash_width = 0.860
    sash_height = 0.560
    sash_profile = 0.034
    sash_depth = 0.034
    sash.visual(
        Box((sash_width, sash_depth, sash_profile)),
        origin=Origin(xyz=(0.0, 0.0, -sash_profile / 2.0)),
        material=aluminium,
        name="sash_top_rail",
    )
    sash.visual(
        Box((sash_width, sash_depth, sash_profile)),
        origin=Origin(xyz=(0.0, 0.0, -sash_height + sash_profile / 2.0)),
        material=aluminium,
        name="sash_bottom_rail",
    )
    sash.visual(
        Box((sash_profile, sash_depth, sash_height)),
        origin=Origin(xyz=(-sash_width / 2.0 + sash_profile / 2.0, 0.0, -sash_height / 2.0)),
        material=aluminium,
        name="sash_stile_0",
    )
    sash.visual(
        Box((sash_profile, sash_depth, sash_height)),
        origin=Origin(xyz=(sash_width / 2.0 - sash_profile / 2.0, 0.0, -sash_height / 2.0)),
        material=aluminium,
        name="sash_stile_1",
    )
    sash.visual(
        Box((0.805, 0.006, 0.492)),
        origin=Origin(xyz=(0.0, -0.006, -0.290)),
        material=glass,
        name="glazing_pane",
    )
    sash.visual(Box((0.800, 0.005, 0.016)), origin=Origin(xyz=(0.0, 0.018, -0.055)), material=rubber, name="top_glass_gasket")
    sash.visual(Box((0.800, 0.005, 0.016)), origin=Origin(xyz=(0.0, 0.018, -0.520)), material=rubber, name="bottom_glass_gasket")
    sash.visual(Box((0.016, 0.005, 0.475)), origin=Origin(xyz=(-0.398, 0.018, -0.288)), material=rubber, name="side_glass_gasket_0")
    sash.visual(Box((0.016, 0.005, 0.475)), origin=Origin(xyz=(0.398, 0.018, -0.288)), material=rubber, name="side_glass_gasket_1")
    sash.visual(
        Box((0.810, 0.033, 0.020)),
        origin=Origin(xyz=(0.0, 0.0305, -0.010)),
        material=hardware,
        name="sash_hinge_leaf",
    )
    sash.visual(
        Box((0.140, 0.012, 0.024)),
        origin=Origin(xyz=(-0.255, 0.040, 0.010)),
        material=hardware,
        name="sash_hinge_tab_0",
    )
    sash.visual(
        Box((0.140, 0.012, 0.024)),
        origin=Origin(xyz=(0.255, 0.040, 0.010)),
        material=hardware,
        name="sash_hinge_tab_1",
    )
    sash.visual(
        Cylinder(radius=0.009, length=0.140),
        origin=Origin(xyz=(-0.255, 0.026, 0.011), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hardware,
        name="sash_hinge_knuckle_0",
    )
    sash.visual(
        Cylinder(radius=0.009, length=0.140),
        origin=Origin(xyz=(0.255, 0.026, 0.011), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hardware,
        name="sash_hinge_knuckle_1",
    )

    model.articulation(
        SASH_JOINT,
        ArticulationType.REVOLUTE,
        parent=frame,
        child=sash,
        origin=Origin(xyz=(0.0, 0.046, 0.635)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=25.0, velocity=1.4, lower=0.0, upper=SASH_UPPER),
    )

    hinge_world = (0.0, 0.046, 0.635)
    frame_low = (0.0, 0.065, 0.225)
    frame_high = (0.0, 0.065, 0.485)
    sash_high = (0.0, 0.058, -0.205)
    sash_low = (0.0, 0.058, -0.435)

    def sash_to_world(local: tuple[float, float, float], q: float) -> tuple[float, float, float]:
        rx = _rot_x(local, q)
        return (rx[0] + hinge_world[0], rx[1] + hinge_world[1], rx[2] + hinge_world[2])

    def world_to_sash(world: tuple[float, float, float], q: float) -> tuple[float, float, float]:
        rel = (world[0] - hinge_world[0], world[1] - hinge_world[1], world[2] - hinge_world[2])
        return _inv_rot_x(rel, q)

    def add_stay_arm(part, length: float, plate_x: float, material: Material) -> None:
        part.visual(Box((0.007, 0.014, length)), origin=Origin(xyz=(plate_x, 0.0, length / 2.0)), material=material, name="bar")
        part.visual(
            Box((0.002, 0.016, length * 0.46)),
            origin=Origin(xyz=(plate_x + (0.004 if plate_x >= 0.0 else -0.004), 0.0, length * 0.62)),
            material=dark_aluminium,
            name="slotted_track",
        )
        part.visual(
            Cylinder(radius=0.015, length=0.007),
            origin=Origin(xyz=(plate_x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=hardware,
            name="pivot_eye",
        )
        part.visual(
            Cylinder(radius=0.014, length=0.007),
            origin=Origin(xyz=(plate_x, 0.0, length), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=hardware,
            name="sliding_eye",
        )
        part.visual(
            Cylinder(radius=0.010, length=0.007),
            origin=Origin(xyz=(plate_x, 0.0, length * 0.55), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=hardware,
            name="center_rivet",
        )

    def add_pin(parent, name: str, xyz: tuple[float, float, float], sign: float, lane: float) -> None:
        parent.visual(
            Box((0.030, 0.050, 0.030)),
            origin=Origin(xyz=(sign * 0.465, xyz[1] - 0.010, xyz[2])),
            material=aluminium,
            name=f"{name}_bracket",
        )
        parent.visual(
            Cylinder(radius=0.010, length=0.018),
            origin=Origin(xyz=(sign * lane, xyz[1], xyz[2]), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=hardware,
            name=name,
        )

    for index, sign in enumerate((-1.0, 1.0)):
        # The fixed frame pins and the moving sash pins are visible clips that retain
        # the two short crossing stays on each side of the window.
        add_pin(frame, f"stay_low_pin_{index}", (sign * 0.0, frame_low[1], frame_low[2]), sign, 0.456)
        add_pin(frame, f"stay_high_pin_{index}", (sign * 0.0, frame_high[1], frame_high[2]), sign, 0.453)
        sash.visual(
            Box((0.030, 0.050, 0.030)),
            origin=Origin(xyz=(sign * 0.420, sash_high[1] - 0.018, sash_high[2])),
            material=aluminium,
            name=f"sash_high_pin_{index}_bracket",
        )
        sash.visual(
            Cylinder(radius=0.010, length=0.018),
            origin=Origin(xyz=(sign * 0.430, sash_high[1], sash_high[2]), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=hardware,
            name=f"sash_high_pin_{index}",
        )
        sash.visual(
            Box((0.030, 0.050, 0.030)),
            origin=Origin(xyz=(sign * 0.420, sash_low[1] - 0.018, sash_low[2])),
            material=aluminium,
            name=f"sash_low_pin_{index}_bracket",
        )
        sash.visual(
            Cylinder(radius=0.010, length=0.018),
            origin=Origin(xyz=(sign * 0.430, sash_low[1], sash_low[2]), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=hardware,
            name=f"sash_low_pin_{index}",
        )

        # Frame-pivoted arm: its slotted outer end follows the upper sash pin.
        upper_pin_closed = sash_to_world((sign * 0.430, sash_high[1], sash_high[2]), 0.0)
        upper_pin_open = sash_to_world((sign * 0.430, sash_high[1], sash_high[2]), SASH_UPPER)
        pivot_low = (sign * 0.456, frame_low[1], frame_low[2])
        v0 = (upper_pin_closed[1] - pivot_low[1], upper_pin_closed[2] - pivot_low[2])
        v1 = (upper_pin_open[1] - pivot_low[1], upper_pin_open[2] - pivot_low[2])
        alpha0 = _arm_angle(*v0)
        alpha1 = _arm_angle(*v1)
        arm_length = max(math.hypot(*v0), math.hypot(*v1)) + 0.055
        stay = model.part(f"stay_{index}_frame_arm")
        add_stay_arm(stay, arm_length, sign * 0.006, hardware)
        model.articulation(
            f"frame_to_stay_{index}",
            ArticulationType.REVOLUTE,
            parent=frame,
            child=stay,
            origin=Origin(xyz=pivot_low, rpy=(alpha0, 0.0, 0.0)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=4.0, velocity=2.0, lower=-0.05, upper=0.95),
            mimic=Mimic(joint=SASH_JOINT, multiplier=(alpha1 - alpha0) / SASH_UPPER),
        )

        # Sash-pivoted crossing arm: its slotted end remains over the upper frame pin.
        frame_pin_local_closed = world_to_sash((sign * 0.453, frame_high[1], frame_high[2]), 0.0)
        frame_pin_local_open = world_to_sash((sign * 0.453, frame_high[1], frame_high[2]), SASH_UPPER)
        pivot_sash = (sign * 0.430, sash_low[1], sash_low[2])
        w0 = (frame_pin_local_closed[1] - pivot_sash[1], frame_pin_local_closed[2] - pivot_sash[2])
        w1 = (frame_pin_local_open[1] - pivot_sash[1], frame_pin_local_open[2] - pivot_sash[2])
        beta0 = _arm_angle(*w0)
        beta1 = _arm_angle(*w1)
        cross_length = max(math.hypot(*w0), math.hypot(*w1)) + 0.055
        cross = model.part(f"stay_{index}_sash_arm")
        add_stay_arm(cross, cross_length, sign * 0.018, hardware)
        model.articulation(
            f"sash_to_stay_{index}",
            ArticulationType.REVOLUTE,
            parent=sash,
            child=cross,
            origin=Origin(xyz=pivot_sash, rpy=(beta0, 0.0, 0.0)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=4.0, velocity=2.0, lower=-0.65, upper=0.15),
            mimic=Mimic(joint=SASH_JOINT, multiplier=(beta1 - beta0) / SASH_UPPER),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    sash = object_model.get_part("sash")
    hinge = object_model.get_articulation(SASH_JOINT)

    for elem in ("sash_hinge_knuckle_0", "sash_hinge_knuckle_1"):
        ctx.allow_overlap(
            frame,
            sash,
            elem_a="top_hinge_pin",
            elem_b=elem,
            reason="The sash hinge knuckle is intentionally captured around the fixed top hinge pin.",
        )
        ctx.expect_overlap(
            frame,
            sash,
            axes="x",
            elem_a="top_hinge_pin",
            elem_b=elem,
            min_overlap=0.10,
            name=f"{elem} is retained on the hinge pin",
        )
        ctx.allow_overlap(
            frame,
            sash,
            elem_a="top_hinge_leaf",
            elem_b=elem,
            reason="The fixed hinge leaf locally nests beside the sash knuckle at the shared top hinge.",
        )
        ctx.expect_overlap(
            frame,
            sash,
            axes="x",
            elem_a="top_hinge_leaf",
            elem_b=elem,
            min_overlap=0.10,
            name=f"{elem} overlaps the fixed hinge leaf along the pin axis",
        )

    ctx.expect_within(sash, frame, axes="x", margin=0.015, name="sash fits between the side jambs")
    ctx.expect_overlap(sash, frame, axes="z", min_overlap=0.45, name="sash sits inside the rectangular frame height")

    for index in (0, 1):
        frame_arm = object_model.get_part(f"stay_{index}_frame_arm")
        sash_arm = object_model.get_part(f"stay_{index}_sash_arm")
        ctx.allow_overlap(
            frame,
            frame_arm,
            elem_a=f"stay_low_pin_{index}",
            elem_b="pivot_eye",
            reason="The fixed lower pivot pin passes through the stay arm eye.",
        )
        ctx.allow_overlap(
            frame,
            frame_arm,
            elem_a=f"stay_low_pin_{index}",
            elem_b="bar",
            reason="The fixed lower pivot pin passes through the reinforced end of the stay bar.",
        )
        ctx.allow_overlap(
            frame,
            frame_arm,
            elem_a=f"stay_low_pin_{index}_bracket",
            elem_b="pivot_eye",
            reason="The lower stay pivot eye is seated in the fixed jamb bracket.",
        )
        ctx.allow_overlap(
            frame,
            frame_arm,
            elem_a=f"stay_low_pin_{index}_bracket",
            elem_b="bar",
            reason="The first part of the stay bar lies inside the lower clevis-style jamb bracket.",
        )
        ctx.allow_overlap(
            frame,
            sash_arm,
            elem_a=f"stay_high_pin_{index}",
            elem_b="bar",
            reason="The upper frame pin is intentionally represented as captured in the slotted crossing arm.",
        )
        ctx.expect_overlap(
            frame_arm,
            frame,
            axes="yz",
            elem_a="pivot_eye",
            elem_b=f"stay_low_pin_{index}",
            min_overlap=0.015,
            name=f"frame stay arm {index} remains on lower jamb pivot",
        )
        ctx.expect_overlap(
            frame_arm,
            sash,
            axes="yz",
            elem_a="slotted_track",
            elem_b=f"sash_high_pin_{index}",
            min_overlap=0.006,
            name=f"closed frame arm {index} captures sash pin",
        )
        ctx.expect_overlap(
            sash_arm,
            frame,
            axes="yz",
            elem_a="slotted_track",
            elem_b=f"stay_high_pin_{index}",
            min_overlap=0.006,
            name=f"closed sash arm {index} captures frame pin",
        )

    rest_aabb = ctx.part_world_aabb(sash)
    with ctx.pose({hinge: SASH_UPPER}):
        open_aabb = ctx.part_world_aabb(sash)
        for index in (0, 1):
            frame_arm = object_model.get_part(f"stay_{index}_frame_arm")
            sash_arm = object_model.get_part(f"stay_{index}_sash_arm")
            ctx.expect_overlap(
                frame_arm,
                sash,
                axes="yz",
                elem_a="slotted_track",
                elem_b=f"sash_high_pin_{index}",
                min_overlap=0.004,
                name=f"vented frame arm {index} remains clipped to sash",
            )
            ctx.expect_overlap(
                sash_arm,
                frame,
                axes="yz",
                elem_a="slotted_track",
                elem_b=f"stay_high_pin_{index}",
                min_overlap=0.004,
                name=f"vented sash arm {index} remains clipped to frame",
            )

    ctx.check(
        "top hinge opens sash outward",
        rest_aabb is not None and open_aabb is not None and open_aabb[1][1] > rest_aabb[1][1] + 0.18,
        details=f"rest_aabb={rest_aabb}, open_aabb={open_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
