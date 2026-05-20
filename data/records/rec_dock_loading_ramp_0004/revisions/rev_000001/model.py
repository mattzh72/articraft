from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math
import os

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

os.chdir("/")


RAMP_RUN = 2.55
RAMP_RISE = 0.80
RAMP_ANGLE = math.atan2(RAMP_RISE, RAMP_RUN)
RAMP_LENGTH = math.hypot(RAMP_RUN, RAMP_RISE)
PLATFORM_LENGTH = 0.78
PLATFORM_Z = 0.80
OVERALL_WIDTH = 1.48
DECK_WIDTH = 1.18
STRINGER_X = 0.56
LEG_X = 0.40
LEG_Y = 3.04

def _surface_z(y: float) -> float:
    if y <= 0.0:
        return 0.0
    if y >= RAMP_RUN:
        return RAMP_RISE
    return RAMP_RISE * y / RAMP_RUN


def _midpoint(
    a: tuple[float, float, float],
    b: tuple[float, float, float],
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(
    a: tuple[float, float, float],
    b: tuple[float, float, float],
) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_cylinder(
    a: tuple[float, float, float],
    b: tuple[float, float, float],
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_member(part, a, b, radius: float, material, name: str | None = None) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _build_leg(part, *, steel, dark_steel, pin_color, rubber) -> None:
    part.visual(
        Box((0.075, 0.075, 0.78)),
        origin=Origin(xyz=(0.0, 0.0, -0.39)),
        material=steel,
        name="leg_post",
    )
    part.visual(
        Cylinder(radius=0.012, length=0.18),
        origin=Origin(xyz=(0.0, 0.0, -0.12), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=pin_color,
        name="locking_pin",
    )
    part.visual(
        Box((0.115, 0.095, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, -0.789)),
        material=dark_steel,
        name="caster_swivel_plate",
    )
    part.visual(
        Cylinder(radius=0.018, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, -0.825)),
        material=dark_steel,
        name="caster_kingpin",
    )
    part.visual(
        Box((0.082, 0.020, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, -0.842)),
        material=dark_steel,
        name="caster_bridge",
    )
    part.visual(
        Box((0.018, 0.084, 0.095)),
        origin=Origin(xyz=(0.028, 0.0, -0.887)),
        material=dark_steel,
        name="caster_fork_left",
    )
    part.visual(
        Box((0.018, 0.084, 0.095)),
        origin=Origin(xyz=(-0.028, 0.0, -0.887)),
        material=dark_steel,
        name="caster_fork_right",
    )
    part.visual(
        Cylinder(radius=0.012, length=0.085),
        origin=Origin(xyz=(0.0, 0.0, -0.915), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="caster_axle",
    )
    part.visual(
        Cylinder(radius=0.075, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, -0.915), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber,
        name="caster_wheel",
    )


def build_object_model() -> ArticulatedObject:
    os.chdir("/")
    model = ArticulatedObject(name="portable_yard_ramp")

    frame_blue = model.material("frame_blue", rgba=(0.18, 0.39, 0.67, 1.0))
    deck_steel = model.material("deck_steel", rgba=(0.63, 0.66, 0.70, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.22, 0.24, 0.27, 1.0))
    pin_orange = model.material("pin_orange", rgba=(0.86, 0.39, 0.08, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.09, 1.0))

    ramp_frame = model.part("ramp_frame")
    ramp_frame.inertial = Inertial.from_geometry(
        Box((OVERALL_WIDTH, RAMP_RUN + PLATFORM_LENGTH + 0.20, 1.10)),
        mass=420.0,
        origin=Origin(xyz=(0.0, 1.70, 0.55)),
    )

    ramp_frame.visual(
        Box((DECK_WIDTH, RAMP_LENGTH, 0.045)),
        origin=Origin(xyz=(0.0, RAMP_RUN * 0.5, RAMP_RISE * 0.5), rpy=(RAMP_ANGLE, 0.0, 0.0)),
        material=deck_steel,
        name="drive_surface",
    )
    for index in range(8):
        y = 0.28 + index * 0.27
        ramp_frame.visual(
            Box((DECK_WIDTH - 0.06, 0.032, 0.014)),
            origin=Origin(xyz=(0.0, y, _surface_z(y) + 0.020), rpy=(RAMP_ANGLE, 0.0, 0.0)),
            material=dark_steel,
            name=f"traction_bar_{index}",
        )

    ramp_frame.visual(
        Box((0.14, RAMP_LENGTH + 0.04, 0.14)),
        origin=Origin(xyz=(STRINGER_X, RAMP_RUN * 0.5, 0.33), rpy=(RAMP_ANGLE, 0.0, 0.0)),
        material=frame_blue,
        name="left_stringer",
    )
    ramp_frame.visual(
        Box((0.14, RAMP_LENGTH + 0.04, 0.14)),
        origin=Origin(xyz=(-STRINGER_X, RAMP_RUN * 0.5, 0.33), rpy=(RAMP_ANGLE, 0.0, 0.0)),
        material=frame_blue,
        name="right_stringer",
    )
    for index in range(7):
        y = 0.34 + index * 0.30
        ramp_frame.visual(
            Cylinder(radius=0.020, length=1.12),
            origin=Origin(xyz=(0.0, y, _surface_z(y) - 0.085), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark_steel,
            name=f"cross_rung_{index}",
        )

    ramp_frame.visual(
        Box((DECK_WIDTH, PLATFORM_LENGTH, 0.050)),
        origin=Origin(xyz=(0.0, RAMP_RUN + PLATFORM_LENGTH * 0.5, PLATFORM_Z)),
        material=deck_steel,
        name="top_platform",
    )
    ramp_frame.visual(
        Box((0.14, PLATFORM_LENGTH, 0.14)),
        origin=Origin(xyz=(STRINGER_X, RAMP_RUN + PLATFORM_LENGTH * 0.5, 0.73)),
        material=frame_blue,
        name="left_platform_side",
    )
    ramp_frame.visual(
        Box((0.14, PLATFORM_LENGTH, 0.14)),
        origin=Origin(xyz=(-STRINGER_X, RAMP_RUN + PLATFORM_LENGTH * 0.5, 0.73)),
        material=frame_blue,
        name="right_platform_side",
    )
    ramp_frame.visual(
        Box((1.10, 0.10, 0.13)),
        origin=Origin(xyz=(0.0, RAMP_RUN + 0.04, 0.74)),
        material=frame_blue,
        name="front_platform_beam",
    )
    ramp_frame.visual(
        Box((1.10, 0.10, 0.13)),
        origin=Origin(xyz=(0.0, RAMP_RUN + PLATFORM_LENGTH - 0.05, 0.74)),
        material=frame_blue,
        name="rear_platform_beam",
    )

    _add_member(
        ramp_frame,
        (0.660, -0.02, 0.090),
        (0.660, RAMP_RUN, 0.890),
        radius=0.030,
        material=frame_blue,
        name="left_rail",
    )
    _add_member(
        ramp_frame,
        (0.660, RAMP_RUN, 0.890),
        (0.660, RAMP_RUN + PLATFORM_LENGTH, 0.890),
        radius=0.030,
        material=frame_blue,
        name="left_platform_rail",
    )
    _add_member(
        ramp_frame,
        (-0.660, -0.02, 0.090),
        (-0.660, RAMP_RUN, 0.890),
        radius=0.030,
        material=frame_blue,
        name="right_rail",
    )
    _add_member(
        ramp_frame,
        (-0.660, RAMP_RUN, 0.890),
        (-0.660, RAMP_RUN + PLATFORM_LENGTH, 0.890),
        radius=0.030,
        material=frame_blue,
        name="right_platform_rail",
    )
    _add_member(
        ramp_frame,
        (0.665, RAMP_RUN, 0.890),
        (0.56, RAMP_RUN + 0.02, 0.785),
        radius=0.022,
        material=frame_blue,
        name="left_front_stanchion",
    )
    _add_member(
        ramp_frame,
        (0.665, RAMP_RUN + PLATFORM_LENGTH - 0.02, 0.890),
        (0.56, RAMP_RUN + PLATFORM_LENGTH - 0.02, 0.785),
        radius=0.022,
        material=frame_blue,
        name="left_rear_stanchion",
    )
    _add_member(
        ramp_frame,
        (-0.665, RAMP_RUN, 0.890),
        (-0.56, RAMP_RUN + 0.02, 0.785),
        radius=0.022,
        material=frame_blue,
        name="right_front_stanchion",
    )
    _add_member(
        ramp_frame,
        (-0.665, RAMP_RUN + PLATFORM_LENGTH - 0.02, 0.890),
        (-0.56, RAMP_RUN + PLATFORM_LENGTH - 0.02, 0.785),
        radius=0.022,
        material=frame_blue,
        name="right_rear_stanchion",
    )

    ramp_frame.visual(
        Box((0.11, 0.11, 0.34)),
        origin=Origin(xyz=(LEG_X, LEG_Y, 0.605)),
        material=frame_blue,
        name="left_sleeve",
    )
    ramp_frame.visual(
        Box((0.11, 0.11, 0.34)),
        origin=Origin(xyz=(-LEG_X, LEG_Y, 0.605)),
        material=frame_blue,
        name="right_sleeve",
    )
    ramp_frame.visual(
        Box((0.96, 0.09, 0.10)),
        origin=Origin(xyz=(0.0, LEG_Y, 0.60)),
        material=dark_steel,
        name="leg_crosshead",
    )
    _add_member(
        ramp_frame,
        (STRINGER_X - 0.02, 2.18, 0.57),
        (LEG_X, LEG_Y - 0.08, 0.69),
        radius=0.026,
        material=dark_steel,
        name="left_leg_brace",
    )
    _add_member(
        ramp_frame,
        (-STRINGER_X + 0.02, 2.18, 0.57),
        (-LEG_X, LEG_Y - 0.08, 0.69),
        radius=0.026,
        material=dark_steel,
        name="right_leg_brace",
    )

    ramp_frame.visual(
        Box((1.08, 0.22, 0.025)),
        origin=Origin(xyz=(0.0, RAMP_RUN + PLATFORM_LENGTH - 0.13, 0.792)),
        material=deck_steel,
        name="hook_plate",
    )
    ramp_frame.visual(
        Box((1.02, 0.05, 0.09)),
        origin=Origin(xyz=(0.0, RAMP_RUN + PLATFORM_LENGTH - 0.005, 0.735)),
        material=dark_steel,
        name="hook_flange",
    )
    ramp_frame.visual(
        Box((0.06, 0.14, 0.12)),
        origin=Origin(xyz=(0.48, RAMP_RUN + PLATFORM_LENGTH + 0.015, 0.748)),
        material=frame_blue,
        name="left_hook_gusset",
    )
    ramp_frame.visual(
        Box((0.06, 0.14, 0.12)),
        origin=Origin(xyz=(-0.48, RAMP_RUN + PLATFORM_LENGTH + 0.015, 0.748)),
        material=frame_blue,
        name="right_hook_gusset",
    )

    left_leg = model.part("left_leg")
    left_leg.inertial = Inertial.from_geometry(
        Box((0.18, 0.16, 1.02)),
        mass=28.0,
        origin=Origin(xyz=(0.0, 0.0, -0.51)),
    )
    _build_leg(left_leg, steel=deck_steel, dark_steel=dark_steel, pin_color=pin_orange, rubber=rubber)

    right_leg = model.part("right_leg")
    right_leg.inertial = Inertial.from_geometry(
        Box((0.18, 0.16, 1.02)),
        mass=28.0,
        origin=Origin(xyz=(0.0, 0.0, -0.51)),
    )
    _build_leg(right_leg, steel=deck_steel, dark_steel=dark_steel, pin_color=pin_orange, rubber=rubber)

    model.articulation(
        "left_leg_adjust",
        ArticulationType.PRISMATIC,
        parent=ramp_frame,
        child=left_leg,
        origin=Origin(xyz=(LEG_X, LEG_Y, 0.775)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=2200.0, velocity=0.10, lower=0.0, upper=0.16),
    )
    model.articulation(
        "right_leg_adjust",
        ArticulationType.PRISMATIC,
        parent=ramp_frame,
        child=right_leg,
        origin=Origin(xyz=(-LEG_X, LEG_Y, 0.775)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(effort=2200.0, velocity=0.10, lower=0.0, upper=0.16),
    )

    return model


def run_tests() -> TestReport:
    os.chdir("/")
    ctx = TestContext(object_model, asset_root="/")
    ramp_frame = object_model.get_part("ramp_frame")
    left_leg = object_model.get_part("left_leg")
    right_leg = object_model.get_part("right_leg")
    left_leg_adjust = object_model.get_articulation("left_leg_adjust")
    right_leg_adjust = object_model.get_articulation("right_leg_adjust")

    drive_surface = ramp_frame.get_visual("drive_surface")
    top_platform = ramp_frame.get_visual("top_platform")
    hook_plate = ramp_frame.get_visual("hook_plate")
    hook_flange = ramp_frame.get_visual("hook_flange")
    left_rail = ramp_frame.get_visual("left_rail")
    left_platform_rail = ramp_frame.get_visual("left_platform_rail")
    right_rail = ramp_frame.get_visual("right_rail")
    right_platform_rail = ramp_frame.get_visual("right_platform_rail")
    left_front_stanchion = ramp_frame.get_visual("left_front_stanchion")
    right_front_stanchion = ramp_frame.get_visual("right_front_stanchion")
    left_stringer = ramp_frame.get_visual("left_stringer")
    right_stringer = ramp_frame.get_visual("right_stringer")
    cross_rung_3 = ramp_frame.get_visual("cross_rung_3")
    left_sleeve = ramp_frame.get_visual("left_sleeve")
    right_sleeve = ramp_frame.get_visual("right_sleeve")

    left_post = left_leg.get_visual("leg_post")
    right_post = right_leg.get_visual("leg_post")
    left_pin = left_leg.get_visual("locking_pin")
    right_pin = right_leg.get_visual("locking_pin")
    left_wheel = left_leg.get_visual("caster_wheel")
    right_wheel = right_leg.get_visual("caster_wheel")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Default exact visual sensor for joint mounting; keep unless scale makes it irrelevant.
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    # Default exact visual sensor for floating/disconnected subassemblies inside one part.
    ctx.warn_if_part_geometry_disconnected()
    ctx.allow_overlap(ramp_frame, left_leg, reason="telescoping left support post nests inside the outer sleeve")
    ctx.allow_overlap(ramp_frame, right_leg, reason="telescoping right support post nests inside the outer sleeve")
    # Default articulated-joint clearance gate; adapt only if the model is not articulated.
    ctx.check_articulation_overlaps(max_pose_samples=128)
    # Default broad overlap warning backstop; conservative and non-blocking by default.
    ctx.warn_if_overlaps(max_pose_samples=128, ignore_adjacent=True, ignore_fixed=True)

    ctx.expect_gap(
        ramp_frame,
        ramp_frame,
        axis="z",
        max_gap=0.03,
        max_penetration=0.03,
        positive_elem=top_platform,
        negative_elem=hook_plate,
        name="dock hook plate stays vertically aligned with the top platform deck",
    )
    ctx.expect_overlap(
        ramp_frame,
        ramp_frame,
        axes="xy",
        min_overlap=0.20,
        elem_a=top_platform,
        elem_b=hook_plate,
        name="dock hook plate overlaps the front of the top platform in plan",
    )
    ctx.expect_gap(
        ramp_frame,
        ramp_frame,
        axis="y",
        min_gap=0.50,
        positive_elem=hook_plate,
        negative_elem=drive_surface,
        name="dock hook lip projects beyond the sloped drive surface at the loading end",
    )
    ctx.expect_gap(
        ramp_frame,
        ramp_frame,
        axis="z",
        max_gap=0.02,
        max_penetration=0.002,
        positive_elem=hook_plate,
        negative_elem=hook_flange,
        name="hook flange hangs just below the hook plate to catch a dock edge",
    )
    ctx.expect_gap(
        ramp_frame,
        ramp_frame,
        axis="x",
        min_gap=0.03,
        positive_elem=left_rail,
        negative_elem=drive_surface,
        name="left side rail stands outside the drive surface",
    )
    ctx.expect_gap(
        ramp_frame,
        ramp_frame,
        axis="x",
        min_gap=0.03,
        positive_elem=drive_surface,
        negative_elem=right_rail,
        name="right side rail stands outside the drive surface",
    )
    ctx.expect_contact(
        ramp_frame,
        ramp_frame,
        elem_a=left_rail,
        elem_b=left_platform_rail,
    )
    ctx.expect_contact(
        ramp_frame,
        ramp_frame,
        elem_a=right_rail,
        elem_b=right_platform_rail,
    )
    ctx.expect_contact(
        ramp_frame,
        ramp_frame,
        elem_a=left_platform_rail,
        elem_b=left_front_stanchion,
        name="left upper rail is supported by a front stanchion at the platform",
    )
    ctx.expect_contact(
        ramp_frame,
        ramp_frame,
        elem_a=right_platform_rail,
        elem_b=right_front_stanchion,
        name="right upper rail is supported by a front stanchion at the platform",
    )
    ctx.expect_contact(
        ramp_frame,
        ramp_frame,
        elem_a=cross_rung_3,
        elem_b=left_stringer,
        name="cross-brace rung lands on the left side structure",
    )
    ctx.expect_contact(
        ramp_frame,
        ramp_frame,
        elem_a=cross_rung_3,
        elem_b=right_stringer,
        name="cross-brace rung lands on the right side structure",
    )
    ctx.expect_within(
        left_leg,
        ramp_frame,
        axes="xy",
        inner_elem=left_post,
        outer_elem=left_sleeve,
    )
    ctx.expect_within(
        right_leg,
        ramp_frame,
        axes="xy",
        inner_elem=right_post,
        outer_elem=right_sleeve,
    )
    ctx.expect_gap(
        ramp_frame,
        left_leg,
        axis="z",
        min_gap=0.76,
        positive_elem=top_platform,
        negative_elem=left_wheel,
        name="left caster sits well below the top platform in the parked pose",
    )
    ctx.expect_gap(
        ramp_frame,
        right_leg,
        axis="z",
        min_gap=0.76,
        positive_elem=top_platform,
        negative_elem=right_wheel,
        name="right caster sits well below the top platform in the parked pose",
    )
    ctx.expect_overlap(
        left_leg,
        left_leg,
        axes="yz",
        min_overlap=0.02,
        elem_a=left_pin,
        elem_b=left_post,
        name="left locking pin passes through the telescoping leg post",
    )
    ctx.expect_overlap(
        right_leg,
        right_leg,
        axes="yz",
        min_overlap=0.02,
        elem_a=right_pin,
        elem_b=right_post,
        name="right locking pin passes through the telescoping leg post",
    )
    ctx.expect_overlap(
        left_leg,
        left_leg,
        axes="xy",
        min_overlap=0.05,
        elem_a=left_post,
        elem_b=left_wheel,
        name="left caster wheel stays centered beneath the leg post",
    )
    ctx.expect_overlap(
        right_leg,
        right_leg,
        axes="xy",
        min_overlap=0.05,
        elem_a=right_post,
        elem_b=right_wheel,
        name="right caster wheel stays centered beneath the leg post",
    )
    with ctx.pose({left_leg_adjust: 0.16, right_leg_adjust: 0.16}):
        ctx.expect_within(
            left_leg,
            ramp_frame,
            axes="xy",
            inner_elem=left_post,
            outer_elem=left_sleeve,
        )
        ctx.expect_within(
            right_leg,
            ramp_frame,
            axes="xy",
            inner_elem=right_post,
            outer_elem=right_sleeve,
        )
        ctx.expect_gap(
            ramp_frame,
            left_leg,
            axis="z",
            min_gap=0.90,
            positive_elem=top_platform,
            negative_elem=left_wheel,
            name="left caster drops farther below the platform when extended",
        )
        ctx.expect_gap(
            ramp_frame,
            right_leg,
            axis="z",
            min_gap=0.90,
            positive_elem=top_platform,
            negative_elem=right_wheel,
            name="right caster drops farther below the platform when extended",
        )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
