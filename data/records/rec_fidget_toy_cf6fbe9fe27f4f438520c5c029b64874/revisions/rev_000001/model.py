from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
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
    rounded_rect_profile,
    sweep_profile_along_spline,
)


def _build_barrel_shell() -> object:
    outer_profile = [
        (0.00585, 0.0000),
        (0.00585, -0.0060),
        (0.00555, -0.0200),
        (0.00555, -0.1060),
        (0.00515, -0.1180),
        (0.00355, -0.1290),
        (0.00195, -0.1360),
    ]
    inner_profile = [
        (0.00270, 0.0000),
        (0.00270, -0.0035),
        (0.00460, -0.0110),
        (0.00460, -0.1120),
        (0.00305, -0.1250),
        (0.00100, -0.1360),
    ]
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer_profile,
            inner_profile,
            segments=72,
            start_cap="flat",
            end_cap="flat",
            lip_samples=8,
        ),
        "barrel_shell",
    )


def _build_tip_cone() -> object:
    profile = [
        (0.00000, -0.0056),
        (0.00040, -0.0047),
        (0.00090, -0.0023),
        (0.00145, 0.0008),
        (0.00185, 0.0038),
    ]
    return mesh_from_geometry(
        LatheGeometry(profile, segments=48, closed=True),
        "tip_cone",
    )


def _build_clip_strip() -> object:
    clip_path = [
        (0.0010, 0.0000, -0.0010),
        (0.0028, 0.0000, -0.0100),
        (0.0045, 0.0000, -0.0280),
        (0.0049, 0.0000, -0.0500),
        (0.0024, 0.0000, -0.0660),
    ]
    clip_profile = rounded_rect_profile(0.0048, 0.0011, radius=0.00018)
    return mesh_from_geometry(
        sweep_profile_along_spline(
            clip_path,
            profile=clip_profile,
            samples_per_segment=18,
            cap_profile=True,
            up_hint=(0.0, 1.0, 0.0),
        ),
        "clip_strip",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="click_pen")

    barrel_blue = model.material("barrel_blue", rgba=(0.09, 0.34, 0.78, 1.0))
    trim_silver = model.material("trim_silver", rgba=(0.78, 0.80, 0.84, 1.0))
    dark_plastic = model.material("dark_plastic", rgba=(0.16, 0.17, 0.19, 1.0))
    ink_tube = model.material("ink_tube", rgba=(0.70, 0.73, 0.78, 1.0))

    body = model.part("body")
    body.visual(
        _build_barrel_shell(),
        material=barrel_blue,
        name="barrel_shell",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.012, 0.012, 0.136)),
        mass=0.012,
        origin=Origin(xyz=(0.0, 0.0, -0.068)),
    )

    clip = model.part("clip")
    clip.visual(
        Box((0.0018, 0.0062, 0.0120)),
        origin=Origin(xyz=(0.0009, 0.0, -0.0055)),
        material=trim_silver,
        name="anchor",
    )
    clip.visual(
        _build_clip_strip(),
        material=trim_silver,
        name="spring",
    )
    clip.inertial = Inertial.from_geometry(
        Box((0.0070, 0.0065, 0.0680)),
        mass=0.002,
        origin=Origin(xyz=(0.0035, 0.0, -0.033)),
    )
    model.articulation(
        "body_to_clip",
        ArticulationType.FIXED,
        parent=body,
        child=clip,
        origin=Origin(xyz=(0.00555, 0.0, -0.017)),
    )

    plunger = model.part("plunger")
    plunger.visual(
        Cylinder(radius=0.00225, length=0.0200),
        origin=Origin(xyz=(0.0, 0.0, -0.0090)),
        material=dark_plastic,
        name="stem",
    )
    plunger.visual(
        Cylinder(radius=0.00270, length=0.0040),
        origin=Origin(xyz=(0.0, 0.0, -0.0014)),
        material=dark_plastic,
        name="guide",
    )
    plunger.visual(
        Cylinder(radius=0.00275, length=0.0016),
        origin=Origin(xyz=(0.0, 0.0, 0.0010)),
        material=dark_plastic,
        name="collar",
    )
    plunger.visual(
        Cylinder(radius=0.00320, length=0.0030),
        origin=Origin(xyz=(0.0, 0.0, 0.0032)),
        material=dark_plastic,
        name="button",
    )
    plunger.visual(
        Cylinder(radius=0.00240, length=0.0026),
        origin=Origin(xyz=(0.0, 0.0, -0.0188)),
        material=dark_plastic,
        name="cam",
    )
    plunger.inertial = Inertial.from_geometry(
        Box((0.007, 0.007, 0.026)),
        mass=0.003,
        origin=Origin(xyz=(0.0, 0.0, -0.008)),
    )
    model.articulation(
        "body_to_plunger",
        ArticulationType.PRISMATIC,
        parent=body,
        child=plunger,
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=0.05,
            lower=0.0,
            upper=0.0034,
        ),
    )

    tip = model.part("tip")
    tip.visual(
        Cylinder(radius=0.00125, length=0.110),
        origin=Origin(xyz=(0.0, 0.0, 0.055)),
        material=ink_tube,
        name="ink_tube",
    )
    tip.visual(
        Cylinder(radius=0.00460, length=0.0040),
        origin=Origin(xyz=(0.0, 0.0, 0.0300)),
        material=ink_tube,
        name="guide",
    )
    tip.visual(
        _build_tip_cone(),
        material=trim_silver,
        name="tip_cone",
    )
    tip.visual(
        Cylinder(radius=0.00035, length=0.0012),
        origin=Origin(xyz=(0.0, 0.0, -0.0058)),
        material=trim_silver,
        name="point",
    )
    tip.inertial = Inertial.from_geometry(
        Box((0.004, 0.004, 0.116)),
        mass=0.002,
        origin=Origin(xyz=(0.0, 0.0, 0.052)),
    )
    model.articulation(
        "body_to_tip",
        ArticulationType.PRISMATIC,
        parent=body,
        child=tip,
        origin=Origin(xyz=(0.0, 0.0, -0.131)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=5.0,
            velocity=0.04,
            lower=0.0,
            upper=0.0040,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    plunger = object_model.get_part("plunger")
    tip = object_model.get_part("tip")
    plunger_joint = object_model.get_articulation("body_to_plunger")
    tip_joint = object_model.get_articulation("body_to_tip")

    ctx.expect_within(
        plunger,
        body,
        axes="xy",
        inner_elem="stem",
        outer_elem="barrel_shell",
        margin=0.0009,
        name="plunger stem stays centered in top opening",
    )
    ctx.expect_overlap(
        plunger,
        body,
        axes="z",
        elem_a="stem",
        elem_b="barrel_shell",
        min_overlap=0.018,
        name="plunger stem remains engaged in barrel at rest",
    )
    ctx.expect_within(
        tip,
        body,
        axes="xy",
        inner_elem="ink_tube",
        outer_elem="barrel_shell",
        margin=0.0009,
        name="ink tube stays centered inside barrel",
    )
    ctx.expect_overlap(
        tip,
        body,
        axes="z",
        elem_a="ink_tube",
        elem_b="barrel_shell",
        min_overlap=0.090,
        name="tip refill remains inserted at rest",
    )

    plunger_limits = plunger_joint.motion_limits
    if plunger_limits is not None and plunger_limits.upper is not None:
        rest_button_aabb = ctx.part_element_world_aabb(plunger, elem="button")
        rest_plunger_pos = ctx.part_world_position(plunger)
        with ctx.pose({plunger_joint: plunger_limits.upper}):
            pressed_button_aabb = ctx.part_element_world_aabb(plunger, elem="button")
            pressed_plunger_pos = ctx.part_world_position(plunger)
            ctx.expect_within(
                plunger,
                body,
                axes="xy",
                inner_elem="stem",
                outer_elem="barrel_shell",
                margin=0.0009,
                name="plunger stays guided when pressed",
            )
            ctx.expect_overlap(
                plunger,
                body,
                axes="z",
                elem_a="stem",
                elem_b="barrel_shell",
                min_overlap=0.018,
                name="plunger stem remains engaged when pressed",
            )
        ctx.check(
            "button moves into cap",
            rest_plunger_pos is not None
            and pressed_plunger_pos is not None
            and pressed_plunger_pos[2] < rest_plunger_pos[2] - 0.0025,
            details=f"rest={rest_plunger_pos}, pressed={pressed_plunger_pos}",
        )
        ctx.check(
            "button crown sits lower when pressed",
            rest_button_aabb is not None
            and pressed_button_aabb is not None
            and pressed_button_aabb[1][2] < rest_button_aabb[1][2] - 0.0025,
            details=f"rest={rest_button_aabb}, pressed={pressed_button_aabb}",
        )

    tip_limits = tip_joint.motion_limits
    if tip_limits is not None and tip_limits.upper is not None:
        rest_point_aabb = ctx.part_element_world_aabb(tip, elem="point")
        with ctx.pose({tip_joint: tip_limits.upper}):
            extended_point_aabb = ctx.part_element_world_aabb(tip, elem="point")
            ctx.expect_within(
                tip,
                body,
                axes="xy",
                inner_elem="ink_tube",
                outer_elem="barrel_shell",
                margin=0.0009,
                name="tip cartridge stays centered when extended",
            )
            ctx.expect_overlap(
                tip,
                body,
                axes="z",
                elem_a="ink_tube",
                elem_b="barrel_shell",
                min_overlap=0.086,
                name="tip cartridge retains insertion when extended",
            )
        ctx.check(
            "tip extends out of the nose",
            rest_point_aabb is not None
            and extended_point_aabb is not None
            and extended_point_aabb[0][2] < rest_point_aabb[0][2] - 0.0035,
            details=f"rest={rest_point_aabb}, extended={extended_point_aabb}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
