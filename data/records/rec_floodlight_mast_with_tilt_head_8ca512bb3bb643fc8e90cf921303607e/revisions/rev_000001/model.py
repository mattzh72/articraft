from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import atan2, pi, sqrt

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


BASE_TOP_Z = 0.55
POLE_HEIGHT = 10.80
PAN_LIMIT = 1.05
TILT_BIAS = 0.32
TILT_LOWER = -0.32
TILT_UPPER = 0.55


def _add_xz_brace(
    part,
    *,
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    thickness: float,
    material: str,
    name: str,
) -> None:
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    dz = end[2] - start[2]
    length = sqrt(dx * dx + dy * dy + dz * dz)
    pitch = atan2(-dz, dx)
    part.visual(
        Box((length, thickness, thickness)),
        origin=Origin(
            xyz=(
                0.5 * (start[0] + end[0]),
                0.5 * (start[1] + end[1]),
                0.5 * (start[2] + end[2]),
            ),
            rpy=(0.0, pitch, 0.0),
        ),
        material=material,
        name=name,
    )


def _add_yoke(part, *, steel: str) -> None:
    part.visual(
        Cylinder(radius=0.09, length=0.03),
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
        material=steel,
        name="turntable_base",
    )
    part.visual(
        Cylinder(radius=0.05, length=0.09),
        origin=Origin(xyz=(0.0, 0.0, 0.075)),
        material=steel,
        name="pan_knuckle",
    )
    part.visual(
        Box((0.12, 0.44, 0.05)),
        origin=Origin(xyz=(0.0, 0.0, 0.145)),
        material=steel,
        name="yoke_crossbar",
    )
    part.visual(
        Box((0.05, 0.04, 0.28)),
        origin=Origin(xyz=(0.0, 0.21, 0.31)),
        material=steel,
        name="upper_arm_pos",
    )
    part.visual(
        Box((0.05, 0.04, 0.28)),
        origin=Origin(xyz=(0.0, -0.21, 0.31)),
        material=steel,
        name="upper_arm_neg",
    )


def _add_head(part, *, housing: str, lens: str) -> None:
    part.visual(
        Box((0.18, 0.34, 0.24)),
        origin=Origin(xyz=(0.09, 0.0, 0.04)),
        material=housing,
        name="housing_shell",
    )
    part.visual(
        Box((0.014, 0.35, 0.25)),
        origin=Origin(xyz=(0.187, 0.0, 0.04)),
        material=housing,
        name="front_bezel",
    )
    part.visual(
        Box((0.004, 0.33, 0.23)),
        origin=Origin(xyz=(0.193, 0.0, 0.04)),
        material=lens,
        name="front_lens",
    )
    part.visual(
        Box((0.04, 0.35, 0.018)),
        origin=Origin(xyz=(0.17, 0.0, 0.169)),
        material=housing,
        name="top_visor",
    )
    for index, z in enumerate((-0.06, -0.01, 0.04, 0.09, 0.14), start=1):
        part.visual(
            Box((0.04, 0.30, 0.01)),
            origin=Origin(xyz=(-0.02, 0.0, z)),
            material=housing,
            name=f"rear_fin_{index}",
        )
    part.visual(
        Cylinder(radius=0.025, length=0.02),
        origin=Origin(xyz=(0.0, 0.18, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=housing,
        name="trunnion_pos",
    )
    part.visual(
        Cylinder(radius=0.025, length=0.02),
        origin=Origin(xyz=(0.0, -0.18, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=housing,
        name="trunnion_neg",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="stadium_four_head_lighting_mast")

    concrete = model.material("concrete", rgba=(0.67, 0.67, 0.64, 1.0))
    galvanized = model.material("galvanized_steel", rgba=(0.43, 0.46, 0.49, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.17, 0.18, 0.20, 1.0))
    lens = model.material("lamp_lens", rgba=(0.86, 0.89, 0.92, 1.0))

    base = model.part("concrete_base")
    base.visual(
        Box((2.40, 2.40, 0.28)),
        origin=Origin(xyz=(0.0, 0.0, 0.14)),
        material=concrete,
        name="footing_block",
    )
    base.visual(
        Box((1.30, 1.30, 0.27)),
        origin=Origin(xyz=(0.0, 0.0, 0.415)),
        material=concrete,
        name="plinth_block",
    )

    pole = model.part("mast_pole")
    pole.visual(
        Cylinder(radius=0.24, length=0.06),
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
        material=galvanized,
        name="base_flange",
    )
    pole.visual(
        Cylinder(radius=0.19, length=0.22),
        origin=Origin(xyz=(0.0, 0.0, 0.11)),
        material=galvanized,
        name="lower_reinforcement",
    )
    pole.visual(
        Cylinder(radius=0.15, length=10.70),
        origin=Origin(xyz=(0.0, 0.0, 5.35)),
        material=galvanized,
        name="main_shaft",
    )
    pole.visual(
        Cylinder(radius=0.18, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 10.75)),
        material=galvanized,
        name="top_spigot",
    )

    model.articulation(
        "base_to_pole",
        ArticulationType.FIXED,
        parent=base,
        child=pole,
        origin=Origin(xyz=(0.0, 0.0, BASE_TOP_Z)),
    )

    arm = model.part("cross_arm")
    arm.visual(
        Cylinder(radius=0.24, length=0.10),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        material=galvanized,
        name="pole_collar",
    )
    arm.visual(
        Box((0.24, 0.18, 0.26)),
        origin=Origin(xyz=(0.0, 0.0, 0.13)),
        material=galvanized,
        name="center_riser",
    )
    arm.visual(
        Cylinder(radius=0.09, length=2.80),
        origin=Origin(xyz=(0.0, 0.0, 0.35), rpy=(0.0, pi / 2.0, 0.0)),
        material=galvanized,
        name="main_cross_tube",
    )
    arm.visual(
        Box((0.18, 0.84, 0.10)),
        origin=Origin(xyz=(1.18, 0.0, 0.49)),
        material=galvanized,
        name="right_end_bracket",
    )
    arm.visual(
        Box((0.18, 0.84, 0.10)),
        origin=Origin(xyz=(-1.18, 0.0, 0.49)),
        material=galvanized,
        name="left_end_bracket",
    )
    _add_xz_brace(
        arm,
        start=(0.12, 0.08, 0.12),
        end=(0.98, 0.08, 0.43),
        thickness=0.04,
        material=galvanized,
        name="right_brace_pos",
    )
    _add_xz_brace(
        arm,
        start=(0.12, -0.08, 0.12),
        end=(0.98, -0.08, 0.43),
        thickness=0.04,
        material=galvanized,
        name="right_brace_neg",
    )
    _add_xz_brace(
        arm,
        start=(-0.12, 0.08, 0.12),
        end=(-0.98, 0.08, 0.43),
        thickness=0.04,
        material=galvanized,
        name="left_brace_pos",
    )
    _add_xz_brace(
        arm,
        start=(-0.12, -0.08, 0.12),
        end=(-0.98, -0.08, 0.43),
        thickness=0.04,
        material=galvanized,
        name="left_brace_neg",
    )

    model.articulation(
        "pole_to_cross_arm",
        ArticulationType.FIXED,
        parent=pole,
        child=arm,
        origin=Origin(xyz=(0.0, 0.0, POLE_HEIGHT)),
    )

    mounts = {
        "right_front": (1.18, 0.30, 0.0),
        "right_rear": (1.18, -0.30, 0.0),
        "left_front": (-1.18, 0.30, pi),
        "left_rear": (-1.18, -0.30, pi),
    }
    for name, (x, y, yaw) in mounts.items():
        yoke = model.part(f"{name}_yoke")
        _add_yoke(yoke, steel=dark_steel)

        head = model.part(f"{name}_head")
        _add_head(head, housing=dark_steel, lens=lens)

        model.articulation(
            f"cross_arm_to_{name}_pan",
            ArticulationType.REVOLUTE,
            parent=arm,
            child=yoke,
            origin=Origin(xyz=(x, y, 0.54), rpy=(0.0, 0.0, yaw)),
            axis=(0.0, 0.0, 1.0),
            motion_limits=MotionLimits(
                effort=120.0,
                velocity=0.7,
                lower=-PAN_LIMIT,
                upper=PAN_LIMIT,
            ),
        )
        model.articulation(
            f"{name}_yoke_to_head_tilt",
            ArticulationType.REVOLUTE,
            parent=yoke,
            child=head,
            origin=Origin(xyz=(0.0, 0.0, 0.29), rpy=(0.0, TILT_BIAS, 0.0)),
            axis=(0.0, 1.0, 0.0),
            motion_limits=MotionLimits(
                effort=80.0,
                velocity=0.7,
                lower=TILT_LOWER,
                upper=TILT_UPPER,
            ),
        )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    base = object_model.get_part("concrete_base")
    pole = object_model.get_part("mast_pole")
    arm = object_model.get_part("cross_arm")

    ctx.expect_contact(base, pole, name="pole seats on concrete plinth")
    ctx.expect_contact(pole, arm, name="cross-arm bracket sits on mast top")

    mounts = ("right_front", "right_rear", "left_front", "left_rear")
    for name in mounts:
        yoke = object_model.get_part(f"{name}_yoke")
        head = object_model.get_part(f"{name}_head")
        pan = object_model.get_articulation(f"cross_arm_to_{name}_pan")
        tilt = object_model.get_articulation(f"{name}_yoke_to_head_tilt")

        ctx.expect_contact(yoke, arm, name=f"{name} yoke mounts to end bracket")
        ctx.expect_contact(head, yoke, name=f"{name} flood head sits in its yoke")

        ctx.check(
            f"{name} pan axis is vertical",
            pan.axis == (0.0, 0.0, 1.0),
            details=f"axis={pan.axis}",
        )
        ctx.check(
            f"{name} tilt axis is horizontal",
            tilt.axis == (0.0, 1.0, 0.0),
            details=f"axis={tilt.axis}",
        )

        pan_limits = pan.motion_limits
        tilt_limits = tilt.motion_limits
        ctx.check(
            f"{name} pan limits are realistic",
            pan_limits is not None
            and pan_limits.lower is not None
            and pan_limits.upper is not None
            and abs(pan_limits.lower + PAN_LIMIT) < 1e-9
            and abs(pan_limits.upper - PAN_LIMIT) < 1e-9,
            details=f"limits={pan_limits}",
        )
        ctx.check(
            f"{name} tilt limits are realistic",
            tilt_limits is not None
            and tilt_limits.lower is not None
            and tilt_limits.upper is not None
            and abs(tilt_limits.lower - TILT_LOWER) < 1e-9
            and abs(tilt_limits.upper - TILT_UPPER) < 1e-9,
            details=f"limits={tilt_limits}",
        )

    def aabb_center(aabb):
        if aabb is None:
            return None
        lower, upper = aabb
        return tuple((lo + hi) * 0.5 for lo, hi in zip(lower, upper))

    right_front_head = object_model.get_part("right_front_head")
    right_front_pan = object_model.get_articulation("cross_arm_to_right_front_pan")
    right_front_tilt = object_model.get_articulation("right_front_yoke_to_head_tilt")

    rest_lens = aabb_center(ctx.part_element_world_aabb(right_front_head, elem="front_lens"))
    with ctx.pose({right_front_pan: 0.55}):
        panned_lens = aabb_center(
            ctx.part_element_world_aabb(right_front_head, elem="front_lens")
        )
    ctx.check(
        "right front head pans toward +y",
        rest_lens is not None
        and panned_lens is not None
        and panned_lens[1] > rest_lens[1] + 0.04,
        details=f"rest={rest_lens}, panned={panned_lens}",
    )

    with ctx.pose({right_front_tilt: 0.30}):
        tilted_lens = aabb_center(
            ctx.part_element_world_aabb(right_front_head, elem="front_lens")
        )
    ctx.check(
        "right front head tilts downward",
        rest_lens is not None
        and tilted_lens is not None
        and tilted_lens[2] < rest_lens[2] - 0.03,
        details=f"rest={rest_lens}, tilted={tilted_lens}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
