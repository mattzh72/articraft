from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    superellipse_profile,
)


def _mesh(name: str, geometry):
    return mesh_from_geometry(geometry, name)


def _build_arm_link(
    part,
    *,
    length: float,
    width: float,
    height: float,
    barrel_radius: float,
    barrel_length: float,
    material,
) -> None:
    ear_center_y = barrel_length * 0.5 + 0.00225
    part.visual(
        Cylinder(radius=barrel_radius, length=barrel_length),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=material,
        name="root_barrel",
    )
    part.visual(
        Box((0.026, width, height + 0.004)),
        origin=Origin(xyz=(0.013, 0.0, 0.0)),
        material=material,
        name="shoulder_block",
    )
    part.visual(
        Box((length - 0.032, width, height)),
        origin=Origin(xyz=((0.020 + (length - 0.012)) * 0.5, 0.0, 0.0)),
        material=material,
        name="link_beam",
    )
    part.visual(
        Box((0.012, width + 0.006, height + 0.002)),
        origin=Origin(xyz=(length - 0.016, 0.0, 0.0)),
        material=material,
        name="tip_block",
    )
    part.visual(
        Box((0.014, 0.0045, 0.018)),
        origin=Origin(xyz=(length - 0.007, ear_center_y, 0.0)),
        material=material,
        name="tip_ear_left",
    )
    part.visual(
        Box((0.014, 0.0045, 0.018)),
        origin=Origin(xyz=(length - 0.007, -ear_center_y, 0.0)),
        material=material,
        name="tip_ear_right",
    )


def _part_center(ctx: TestContext, part) -> tuple[float, float, float] | None:
    aabb = ctx.part_world_aabb(part)
    if aabb is None:
        return None
    lower, upper = aabb
    return tuple((lo + hi) * 0.5 for lo, hi in zip(lower, upper))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="music_stand_clip_lamp")

    matte_black = model.material("matte_black", rgba=(0.14, 0.14, 0.15, 1.0))
    graphite = model.material("graphite", rgba=(0.28, 0.29, 0.31, 1.0))
    warm_white = model.material("warm_white", rgba=(0.90, 0.88, 0.83, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.09, 1.0))

    clip_base = model.part("clip_base")
    clip_base.visual(
        Box((0.095, 0.026, 0.008)),
        origin=Origin(xyz=(0.0475, 0.0, 0.004)),
        material=matte_black,
        name="lower_jaw",
    )
    clip_base.visual(
        Box((0.074, 0.026, 0.008)),
        origin=Origin(xyz=(0.037, 0.0, 0.030)),
        material=matte_black,
        name="upper_jaw",
    )
    clip_base.visual(
        Box((0.016, 0.028, 0.034)),
        origin=Origin(xyz=(0.008, 0.0, 0.017)),
        material=matte_black,
        name="rear_spine",
    )
    clip_base.visual(
        Cylinder(radius=0.010, length=0.028),
        origin=Origin(xyz=(0.017, 0.0, 0.022), rpy=(pi / 2.0, 0.0, 0.0)),
        material=graphite,
        name="spring_barrel",
    )
    clip_base.visual(
        Box((0.010, 0.026, 0.018)),
        origin=Origin(xyz=(0.090, 0.0, 0.011)),
        material=matte_black,
        name="lower_lip",
    )
    clip_base.visual(
        Box((0.012, 0.022, 0.003)),
        origin=Origin(xyz=(0.074, 0.0, 0.026)),
        material=rubber,
        name="upper_pad",
    )
    clip_base.visual(
        Box((0.018, 0.022, 0.003)),
        origin=Origin(xyz=(0.074, 0.0, 0.0095)),
        material=rubber,
        name="lower_pad",
    )
    clip_base.visual(
        Box((0.014, 0.024, 0.016)),
        origin=Origin(xyz=(0.018, 0.0, 0.036)),
        material=matte_black,
        name="mount_stem",
    )
    clip_base.visual(
        Box((0.018, 0.024, 0.014)),
        origin=Origin(xyz=(0.018, 0.0, 0.046)),
        material=matte_black,
        name="mount_block",
    )
    clip_base.visual(
        Box((0.014, 0.0045, 0.020)),
        origin=Origin(xyz=(0.031, 0.00875, 0.052)),
        material=graphite,
        name="mount_ear_left",
    )
    clip_base.visual(
        Box((0.014, 0.0045, 0.020)),
        origin=Origin(xyz=(0.031, -0.00875, 0.052)),
        material=graphite,
        name="mount_ear_right",
    )

    lower_arm = model.part("lower_arm")
    _build_arm_link(
        lower_arm,
        length=0.104,
        width=0.013,
        height=0.010,
        barrel_radius=0.0068,
        barrel_length=0.013,
        material=graphite,
    )

    middle_arm = model.part("middle_arm")
    _build_arm_link(
        middle_arm,
        length=0.094,
        width=0.012,
        height=0.009,
        barrel_radius=0.0063,
        barrel_length=0.013,
        material=graphite,
    )

    upper_arm = model.part("upper_arm")
    _build_arm_link(
        upper_arm,
        length=0.082,
        width=0.011,
        height=0.008,
        barrel_radius=0.0058,
        barrel_length=0.013,
        material=graphite,
    )

    shade = model.part("shade")
    shade.visual(
        Cylinder(radius=0.0058, length=0.013),
        origin=Origin(rpy=(pi / 2.0, 0.0, 0.0)),
        material=graphite,
        name="tilt_barrel",
    )
    shade.visual(
        Box((0.012, 0.012, 0.012)),
        origin=Origin(xyz=(0.006, 0.0, 0.0)),
        material=graphite,
        name="tilt_bracket",
    )
    shade.visual(
        Cylinder(radius=0.010, length=0.008),
        origin=Origin(xyz=(0.010, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
        material=graphite,
        name="rear_collar",
    )
    shell_outer = superellipse_profile(0.044, 0.044, exponent=2.0, segments=40)
    shell_inner = superellipse_profile(0.036, 0.036, exponent=2.0, segments=40)
    shade.visual(
        _mesh(
            "lamp_shade_shell",
            ExtrudeWithHolesGeometry(
                shell_outer,
                [shell_inner],
                height=0.040,
                center=True,
            ).rotate_y(pi / 2.0),
        ),
        origin=Origin(xyz=(0.030, 0.0, 0.0)),
        material=matte_black,
        name="shade_shell",
    )
    shade.visual(
        _mesh(
            "lamp_shade_rear_cap",
            ExtrudeWithHolesGeometry(
                superellipse_profile(0.036, 0.036, exponent=2.0, segments=36),
                [superellipse_profile(0.010, 0.010, exponent=2.0, segments=24)],
                height=0.002,
                center=True,
            ).rotate_y(pi / 2.0),
        ),
        origin=Origin(xyz=(0.010, 0.0, 0.0)),
        material=warm_white,
        name="rear_cap",
    )

    model.articulation(
        "clip_to_lower",
        ArticulationType.REVOLUTE,
        parent=clip_base,
        child=lower_arm,
        origin=Origin(xyz=(0.035, 0.0, 0.052), rpy=(0.0, -1.00, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=2.5,
            lower=-0.85,
            upper=0.75,
        ),
    )
    model.articulation(
        "lower_to_middle",
        ArticulationType.REVOLUTE,
        parent=lower_arm,
        child=middle_arm,
        origin=Origin(xyz=(0.104, 0.0, 0.0), rpy=(0.0, 0.40, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.8,
            velocity=3.0,
            lower=-1.10,
            upper=1.20,
        ),
    )
    model.articulation(
        "middle_to_upper",
        ArticulationType.REVOLUTE,
        parent=middle_arm,
        child=upper_arm,
        origin=Origin(xyz=(0.094, 0.0, 0.0), rpy=(0.0, 0.45, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=1.5,
            velocity=3.2,
            lower=-1.15,
            upper=1.10,
        ),
    )
    model.articulation(
        "upper_to_shade",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=shade,
        origin=Origin(xyz=(0.082, 0.0, 0.0), rpy=(0.0, 1.05, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=0.9,
            velocity=3.5,
            lower=-0.35,
            upper=1.00,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    clip_base = object_model.get_part("clip_base")
    lower_arm = object_model.get_part("lower_arm")
    middle_arm = object_model.get_part("middle_arm")
    upper_arm = object_model.get_part("upper_arm")
    shade = object_model.get_part("shade")

    clip_to_lower = object_model.get_articulation("clip_to_lower")
    lower_to_middle = object_model.get_articulation("lower_to_middle")
    middle_to_upper = object_model.get_articulation("middle_to_upper")
    upper_to_shade = object_model.get_articulation("upper_to_shade")

    lower_root_barrel = lower_arm.get_visual("root_barrel")
    middle_root_barrel = middle_arm.get_visual("root_barrel")
    upper_root_barrel = upper_arm.get_visual("root_barrel")
    shade_tilt_barrel = shade.get_visual("tilt_barrel")

    mount_ear_left = clip_base.get_visual("mount_ear_left")
    mount_ear_right = clip_base.get_visual("mount_ear_right")
    lower_tip_ear_left = lower_arm.get_visual("tip_ear_left")
    lower_tip_ear_right = lower_arm.get_visual("tip_ear_right")
    middle_tip_ear_left = middle_arm.get_visual("tip_ear_left")
    middle_tip_ear_right = middle_arm.get_visual("tip_ear_right")
    upper_tip_ear_left = upper_arm.get_visual("tip_ear_left")
    upper_tip_ear_right = upper_arm.get_visual("tip_ear_right")

    ctx.expect_contact(
        lower_arm,
        clip_base,
        elem_a=lower_root_barrel,
        elem_b=mount_ear_left,
        name="lower arm barrel is supported by the left clip ear",
    )
    ctx.expect_contact(
        lower_arm,
        clip_base,
        elem_a=lower_root_barrel,
        elem_b=mount_ear_right,
        name="lower arm barrel is supported by the right clip ear",
    )
    ctx.expect_contact(
        middle_arm,
        lower_arm,
        elem_a=middle_root_barrel,
        elem_b=lower_tip_ear_left,
        name="middle arm barrel is supported by the lower arm left ear",
    )
    ctx.expect_contact(
        middle_arm,
        lower_arm,
        elem_a=middle_root_barrel,
        elem_b=lower_tip_ear_right,
        name="middle arm barrel is supported by the lower arm right ear",
    )
    ctx.expect_contact(
        upper_arm,
        middle_arm,
        elem_a=upper_root_barrel,
        elem_b=middle_tip_ear_left,
        name="upper arm barrel is supported by the middle arm left ear",
    )
    ctx.expect_contact(
        upper_arm,
        middle_arm,
        elem_a=upper_root_barrel,
        elem_b=middle_tip_ear_right,
        name="upper arm barrel is supported by the middle arm right ear",
    )
    ctx.expect_contact(
        shade,
        upper_arm,
        elem_a=shade_tilt_barrel,
        elem_b=upper_tip_ear_left,
        name="shade barrel is supported by the upper arm left ear",
    )
    ctx.expect_contact(
        shade,
        upper_arm,
        elem_a=shade_tilt_barrel,
        elem_b=upper_tip_ear_right,
        name="shade barrel is supported by the upper arm right ear",
    )

    clip_center = _part_center(ctx, clip_base)
    rest_shade_center = _part_center(ctx, shade)
    ctx.check(
        "shade sits above and ahead of the clip at rest",
        clip_center is not None
        and rest_shade_center is not None
        and rest_shade_center[0] > clip_center[0] + 0.16
        and rest_shade_center[2] > clip_center[2] + 0.11,
        details=f"clip_center={clip_center}, shade_center={rest_shade_center}",
    )

    with ctx.pose({clip_to_lower: 0.40}):
        raised_by_base = _part_center(ctx, shade)
    ctx.check(
        "base hinge positive motion raises the lamp head",
        rest_shade_center is not None
        and raised_by_base is not None
        and raised_by_base[2] > rest_shade_center[2] + 0.020,
        details=f"rest={rest_shade_center}, raised={raised_by_base}",
    )

    with ctx.pose({lower_to_middle: 0.55}):
        raised_by_middle = _part_center(ctx, shade)
    ctx.check(
        "middle hinge positive motion raises the lamp head",
        rest_shade_center is not None
        and raised_by_middle is not None
        and raised_by_middle[2] > rest_shade_center[2] + 0.012,
        details=f"rest={rest_shade_center}, raised={raised_by_middle}",
    )

    with ctx.pose({middle_to_upper: 0.60}):
        raised_by_upper = _part_center(ctx, shade)
    ctx.check(
        "upper hinge positive motion raises the lamp head",
        rest_shade_center is not None
        and raised_by_upper is not None
        and raised_by_upper[2] > rest_shade_center[2] + 0.010,
        details=f"rest={rest_shade_center}, raised={raised_by_upper}",
    )

    with ctx.pose({upper_to_shade: 0.70}):
        shade_tilted_up = _part_center(ctx, shade)
    ctx.check(
        "shade tilt positive motion lifts the shade nose",
        rest_shade_center is not None
        and shade_tilted_up is not None
        and shade_tilted_up[2] > rest_shade_center[2] + 0.010,
        details=f"rest={rest_shade_center}, tilted={shade_tilted_up}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
