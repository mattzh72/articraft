from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _shell_mesh(name: str, outer_profile, inner_profile, *, segments: int = 64):
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer_profile,
            inner_profile,
            segments=segments,
        ),
        name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tilt_shift_architectural_prime_lens")

    anodized_black = model.material("anodized_black", rgba=(0.10, 0.10, 0.11, 1.0))
    matte_black = model.material("matte_black", rgba=(0.06, 0.06, 0.07, 1.0))
    satin_gray = model.material("satin_gray", rgba=(0.60, 0.61, 0.63, 1.0))
    coated_glass = model.material("coated_glass", rgba=(0.21, 0.30, 0.34, 0.72))

    outer_housing = model.part("outer_housing")
    outer_housing.visual(
        _shell_mesh(
            "outer_housing_shell",
            [
                (0.031, -0.060),
                (0.036, -0.060),
                (0.040, -0.054),
                (0.043, -0.045),
                (0.045, -0.020),
                (0.045, 0.004),
                (0.043, 0.010),
            ],
            [
                (0.024, -0.060),
                (0.028, -0.054),
                (0.034, -0.045),
                (0.037, -0.020),
                (0.037, 0.004),
                (0.035, 0.010),
            ],
        ),
        material=anodized_black,
        name="housing_shell",
    )
    outer_housing.visual(
        Cylinder(radius=0.034, length=0.003),
        origin=Origin(xyz=(0.0, 0.0, -0.0615)),
        material=satin_gray,
        name="mount_plate",
    )
    outer_housing.inertial = Inertial.from_geometry(
        Cylinder(radius=0.046, length=0.123),
        mass=0.55,
    )

    rotation_collar = model.part("rotation_collar")
    rotation_collar.visual(
        _shell_mesh(
            "rotation_collar_shell",
            [
                (0.0490, -0.007),
                (0.0512, -0.004),
                (0.0520, 0.006),
                (0.0512, 0.017),
                (0.0490, 0.022),
            ],
            [
                (0.0465, -0.007),
                (0.0465, 0.022),
            ],
            segments=56,
        ),
        material=anodized_black,
        name="collar_shell",
    )
    rotation_collar.visual(
        Box((0.010, 0.004, 0.012)),
        origin=Origin(xyz=(0.054, 0.0, 0.007)),
        material=matte_black,
        name="rotation_index",
    )
    for roller_index in range(3):
        angle = (2.0 * math.pi * roller_index) / 3.0
        rotation_collar.visual(
            Cylinder(radius=0.004, length=0.020),
            origin=Origin(
                xyz=(0.049 * math.cos(angle), 0.049 * math.sin(angle), 0.007),
                rpy=(0.0, 0.0, angle),
            ),
            material=satin_gray,
            name=f"bearing_roller_{roller_index}",
        )
    rotation_collar.inertial = Inertial.from_geometry(
        Cylinder(radius=0.052, length=0.029),
        mass=0.12,
    )

    shift_carriage = model.part("shift_carriage")
    shift_carriage.visual(
        _shell_mesh(
            "shift_carriage_ring",
            [
                (0.0305, -0.006),
                (0.0315, 0.000),
                (0.0305, 0.006),
            ],
            [
                (0.0275, -0.006),
                (0.0275, 0.006),
            ],
            segments=48,
        ),
        material=matte_black,
        name="carriage_ring",
    )
    shift_carriage.visual(
        Box((0.062, 0.006, 0.010)),
        origin=Origin(xyz=(0.0, 0.028, 0.0)),
        material=matte_black,
        name="carriage_tie_top",
    )
    shift_carriage.visual(
        Box((0.062, 0.006, 0.010)),
        origin=Origin(xyz=(0.0, -0.028, 0.0)),
        material=matte_black,
        name="carriage_tie_bottom",
    )
    shift_carriage.visual(
        Box((0.006, 0.020, 0.012)),
        origin=Origin(xyz=(0.028, 0.0, 0.0)),
        material=matte_black,
        name="guide_pad_pos",
    )
    shift_carriage.visual(
        Box((0.006, 0.020, 0.012)),
        origin=Origin(xyz=(-0.028, 0.0, 0.0)),
        material=matte_black,
        name="guide_pad_neg",
    )
    shift_carriage.inertial = Inertial.from_geometry(
        Box((0.068, 0.062, 0.016)),
        mass=0.18,
    )

    inner_optical_block = model.part("inner_optical_block")
    inner_optical_block.visual(
        _shell_mesh(
            "inner_optical_block_shell",
            [
                (0.020, -0.041),
                (0.021, -0.036),
                (0.023, -0.015),
                (0.025, 0.015),
                (0.029, 0.055),
                (0.031, 0.071),
                (0.029, 0.076),
            ],
            [
                (0.014, -0.041),
                (0.016, -0.030),
                (0.019, 0.000),
                (0.022, 0.055),
                (0.023, 0.071),
            ],
            segments=64,
        ),
        material=anodized_black,
        name="optical_shell",
    )
    inner_optical_block.visual(
        Cylinder(radius=0.0235, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.067)),
        material=coated_glass,
        name="front_glass",
    )
    inner_optical_block.visual(
        Cylinder(radius=0.017, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, -0.031)),
        material=coated_glass,
        name="rear_glass",
    )
    inner_optical_block.visual(
        Cylinder(radius=0.0045, length=0.010),
        origin=Origin(xyz=(0.0, 0.021, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=matte_black,
        name="trunnion_pos",
    )
    inner_optical_block.visual(
        Cylinder(radius=0.0045, length=0.010),
        origin=Origin(xyz=(0.0, -0.021, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=matte_black,
        name="trunnion_neg",
    )
    inner_optical_block.inertial = Inertial.from_geometry(
        Cylinder(radius=0.032, length=0.118),
        mass=0.42,
    )

    model.articulation(
        "housing_to_rotation_collar",
        ArticulationType.REVOLUTE,
        parent=outer_housing,
        child=rotation_collar,
        origin=Origin(),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=1.2,
            lower=-math.pi / 2.0,
            upper=math.pi / 2.0,
        ),
    )
    model.articulation(
        "collar_to_shift_carriage",
        ArticulationType.PRISMATIC,
        parent=rotation_collar,
        child=shift_carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=0.04,
            lower=-0.012,
            upper=0.012,
        ),
    )
    model.articulation(
        "carriage_to_optical_block",
        ArticulationType.REVOLUTE,
        parent=shift_carriage,
        child=inner_optical_block,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=0.6,
            lower=-math.radians(8.0),
            upper=math.radians(8.0),
        ),
    )

    return model


def run_tests() -> TestReport:
    def _aabb_center(aabb):
        if aabb is None:
            return None
        return tuple((aabb[0][i] + aabb[1][i]) * 0.5 for i in range(3))

    ctx = TestContext(object_model)
    outer_housing = object_model.get_part("outer_housing")
    rotation_collar = object_model.get_part("rotation_collar")
    shift_carriage = object_model.get_part("shift_carriage")
    inner_optical_block = object_model.get_part("inner_optical_block")

    collar_joint = object_model.get_articulation("housing_to_rotation_collar")
    shift_joint = object_model.get_articulation("collar_to_shift_carriage")
    tilt_joint = object_model.get_articulation("carriage_to_optical_block")

    shift_upper = shift_joint.motion_limits.upper or 0.0
    tilt_upper = tilt_joint.motion_limits.upper or 0.0
    collar_upper = collar_joint.motion_limits.upper or 0.0

    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    ctx.expect_overlap(
        rotation_collar,
        outer_housing,
        axes="xy",
        min_overlap=0.086,
        name="rotation collar stays coaxial around the outer housing",
    )
    ctx.expect_overlap(
        shift_carriage,
        rotation_collar,
        axes="xy",
        min_overlap=0.060,
        name="shift carriage nests inside the rotation collar at rest",
    )
    ctx.expect_within(
        inner_optical_block,
        outer_housing,
        axes="xy",
        margin=0.0,
        name="rest optical block stays within the housing silhouette",
    )

    rest_origin = ctx.part_world_position(inner_optical_block)
    with ctx.pose({shift_joint: shift_upper}):
        shifted_origin = ctx.part_world_position(inner_optical_block)
        ctx.expect_within(
            inner_optical_block,
            outer_housing,
            axes="xy",
            margin=0.0,
            name="shifted optical block still stays inside the housing silhouette",
        )
    ctx.check(
        "optical block shifts laterally rather than along the optical axis",
        rest_origin is not None
        and shifted_origin is not None
        and shifted_origin[0] > rest_origin[0] + 0.009
        and abs(shifted_origin[2] - rest_origin[2]) < 1e-4,
        details=f"rest_origin={rest_origin}, shifted_origin={shifted_origin}",
    )

    rest_front = _aabb_center(ctx.part_element_world_aabb(inner_optical_block, elem="front_glass"))
    with ctx.pose({tilt_joint: tilt_upper}):
        tilted_front = _aabb_center(
            ctx.part_element_world_aabb(inner_optical_block, elem="front_glass")
        )
    ctx.check(
        "positive tilt tips the front element toward +x in the default collar orientation",
        rest_front is not None
        and tilted_front is not None
        and tilted_front[0] > rest_front[0] + 0.008
        and abs(tilted_front[1] - rest_front[1]) < 0.004,
        details=f"rest_front={rest_front}, tilted_front={tilted_front}",
    )

    with ctx.pose({collar_joint: collar_upper, tilt_joint: tilt_upper}):
        rotated_tilt_front = _aabb_center(
            ctx.part_element_world_aabb(inner_optical_block, elem="front_glass")
        )
    ctx.check(
        "rotation collar reorients the tilt direction around the optical axis",
        rest_front is not None
        and rotated_tilt_front is not None
        and rotated_tilt_front[1] > rest_front[1] + 0.008
        and abs(rotated_tilt_front[0] - rest_front[0]) < 0.004,
        details=f"rest_front={rest_front}, rotated_tilt_front={rotated_tilt_front}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
