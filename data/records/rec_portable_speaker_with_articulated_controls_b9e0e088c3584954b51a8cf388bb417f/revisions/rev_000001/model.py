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
    CylinderGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    tube_from_spline_points,
)


BODY_RADIUS = 0.055
BODY_LENGTH = 0.236
ENDCAP_LENGTH = 0.024
HINGE_X = 0.122
HINGE_Z = 0.073
BUTTON_MOUNT_Z = 0.063


def _build_button_collar_mesh():
    outer_profile = [
        (0.0100, 0.0000),
        (0.0097, 0.0020),
        (0.0092, 0.0070),
    ]
    inner_profile = [
        (0.0062, 0.0000),
        (0.0062, 0.0070),
    ]
    return mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            outer_profile,
            inner_profile,
            segments=40,
        ),
        "speaker_button_collar",
    )


def _build_handle_half_mesh(side: str):
    sign = 1.0 if side == "left" else -1.0
    path = [
        (0.000, 0.000, 0.000),
        (0.018 * sign, 0.000, 0.016),
        (0.050 * sign, 0.000, 0.040),
        (0.094 * sign, 0.000, 0.060),
        (0.1203 * sign, 0.000, 0.068),
    ]
    handle_geom = tube_from_spline_points(
        path,
        radius=0.0055,
        samples_per_segment=16,
        radial_segments=18,
        cap_ends=True,
    )
    handle_geom.merge(
        CylinderGeometry(radius=0.0085, height=0.016, radial_segments=24).rotate_x(math.pi / 2.0)
    )
    handle_geom.merge(
        CylinderGeometry(radius=0.0068, height=0.012, radial_segments=20)
        .rotate_x(math.pi / 2.0)
        .translate(0.010 * sign, 0.000, 0.007)
    )
    return mesh_from_geometry(handle_geom, f"{side}_speaker_handle_half")


def _add_button(
    model: ArticulatedObject,
    body,
    *,
    name: str,
    x: float,
    radius: float,
    height: float,
    travel: float,
    material,
) -> None:
    button = model.part(name)
    button.visual(
        Cylinder(radius=radius * 0.58, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=material,
        name="guide_stem",
    )
    button.visual(
        Cylinder(radius=radius, length=height),
        origin=Origin(xyz=(0.0, 0.0, 0.006 + height * 0.5)),
        material=material,
        name="button_cap",
    )
    button.inertial = Inertial.from_geometry(
        Box((radius * 2.0, radius * 2.0, 0.010)),
        mass=0.008,
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
    )
    model.articulation(
        f"speaker_body_to_{name}",
        ArticulationType.PRISMATIC,
        parent=body,
        child=button,
        origin=Origin(xyz=(x, 0.0, BUTTON_MOUNT_Z)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=6.0,
            velocity=0.08,
            lower=0.0,
            upper=travel,
        ),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="portable_bluetooth_speaker")

    fabric = model.material("fabric_black", rgba=(0.13, 0.14, 0.15, 1.0))
    rubber = model.material("rubber_charcoal", rgba=(0.20, 0.20, 0.22, 1.0))
    plastic = model.material("plastic_dark", rgba=(0.10, 0.10, 0.11, 1.0))
    button_material = model.material("button_silicone", rgba=(0.32, 0.34, 0.36, 1.0))
    emblem = model.material("metallic_logo", rgba=(0.62, 0.64, 0.66, 1.0))

    body = model.part("speaker_body")
    body.visual(
        Cylinder(radius=BODY_RADIUS, length=BODY_LENGTH),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=fabric,
        name="grille_band",
    )
    body.visual(
        Cylinder(radius=0.058, length=ENDCAP_LENGTH),
        origin=Origin(xyz=(-0.130, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber,
        name="left_endcap",
    )
    body.visual(
        Cylinder(radius=0.058, length=ENDCAP_LENGTH),
        origin=Origin(xyz=(0.130, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=rubber,
        name="right_endcap",
    )
    body.visual(
        Cylinder(radius=0.041, length=0.004),
        origin=Origin(xyz=(-0.142, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=plastic,
        name="left_radiator",
    )
    body.visual(
        Cylinder(radius=0.041, length=0.004),
        origin=Origin(xyz=(0.142, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=plastic,
        name="right_radiator",
    )
    body.visual(
        Box((0.176, 0.032, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.059)),
        material=rubber,
        name="control_spine",
    )
    body.visual(
        Box((0.034, 0.004, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, -0.058)),
        material=rubber,
        name="bottom_foot",
    )

    button_collar = _build_button_collar_mesh()
    for index, x in enumerate((-0.050, -0.025, 0.000, 0.025, 0.050)):
        body.visual(
            button_collar,
            origin=Origin(xyz=(x, 0.0, BUTTON_MOUNT_Z)),
            material=plastic,
            name=f"button_collar_{index}",
        )

    for side_sign, side_name in ((-1.0, "left"), (1.0, "right")):
        x_base = side_sign * HINGE_X
        x_outer = side_sign * (HINGE_X + 0.004)
        body.visual(
            Box((0.018, 0.028, 0.016)),
            origin=Origin(xyz=(x_base, 0.0, 0.061)),
            material=rubber,
            name=f"{side_name}_handle_bracket_base",
        )
        body.visual(
            Box((0.010, 0.006, 0.024)),
            origin=Origin(xyz=(x_outer, -0.011, HINGE_Z)),
            material=rubber,
            name=f"{side_name}_handle_bracket_front",
        )
        body.visual(
            Box((0.010, 0.006, 0.024)),
            origin=Origin(xyz=(x_outer, 0.011, HINGE_Z)),
            material=rubber,
            name=f"{side_name}_handle_bracket_rear",
        )

    body.visual(
        Box((0.030, 0.004, 0.010)),
        origin=Origin(xyz=(0.0, -0.055, 0.0)),
        material=emblem,
        name="logo_plate",
    )
    body.inertial = Inertial.from_geometry(
        Box((0.288, 0.124, 0.138)),
        mass=1.75,
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
    )

    left_handle = model.part("left_handle_half")
    left_handle.visual(
        _build_handle_half_mesh("left"),
        material=rubber,
        name="left_handle",
    )
    left_handle.inertial = Inertial.from_geometry(
        Box((0.124, 0.018, 0.074)),
        mass=0.085,
        origin=Origin(xyz=(0.062, 0.0, 0.037)),
    )

    right_handle = model.part("right_handle_half")
    right_handle.visual(
        _build_handle_half_mesh("right"),
        material=rubber,
        name="right_handle",
    )
    right_handle.inertial = Inertial.from_geometry(
        Box((0.124, 0.018, 0.074)),
        mass=0.085,
        origin=Origin(xyz=(-0.062, 0.0, 0.037)),
    )

    model.articulation(
        "speaker_body_to_left_handle",
        ArticulationType.REVOLUTE,
        parent=body,
        child=left_handle,
        origin=Origin(xyz=(-HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=1.8,
            lower=0.0,
            upper=1.0,
        ),
    )
    model.articulation(
        "speaker_body_to_right_handle",
        ArticulationType.REVOLUTE,
        parent=body,
        child=right_handle,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=1.8,
            lower=0.0,
            upper=1.0,
        ),
    )

    _add_button(
        model,
        body,
        name="power_button",
        x=-0.050,
        radius=0.0085,
        height=0.0035,
        travel=0.0020,
        material=button_material,
    )
    _add_button(
        model,
        body,
        name="volume_down_button",
        x=-0.025,
        radius=0.0088,
        height=0.0035,
        travel=0.0020,
        material=button_material,
    )
    _add_button(
        model,
        body,
        name="play_pause_button",
        x=0.000,
        radius=0.0105,
        height=0.0040,
        travel=0.0020,
        material=button_material,
    )
    _add_button(
        model,
        body,
        name="volume_up_button",
        x=0.025,
        radius=0.0088,
        height=0.0035,
        travel=0.0020,
        material=button_material,
    )
    _add_button(
        model,
        body,
        name="pairing_button",
        x=0.050,
        radius=0.0085,
        height=0.0035,
        travel=0.0020,
        material=button_material,
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

    body = object_model.get_part("speaker_body")
    left_handle = object_model.get_part("left_handle_half")
    right_handle = object_model.get_part("right_handle_half")
    left_joint = object_model.get_articulation("speaker_body_to_left_handle")
    right_joint = object_model.get_articulation("speaker_body_to_right_handle")

    button_specs = [
        ("power_button", "speaker_body_to_power_button"),
        ("volume_down_button", "speaker_body_to_volume_down_button"),
        ("play_pause_button", "speaker_body_to_play_pause_button"),
        ("volume_up_button", "speaker_body_to_volume_up_button"),
        ("pairing_button", "speaker_body_to_pairing_button"),
    ]

    ctx.check(
        "handle hinge axes mirror correctly",
        left_joint.axis == (0.0, -1.0, 0.0) and right_joint.axis == (0.0, 1.0, 0.0),
        details=f"left_axis={left_joint.axis}, right_axis={right_joint.axis}",
    )

    with ctx.pose({left_joint: 0.0, right_joint: 0.0}):
        ctx.expect_gap(
            right_handle,
            left_handle,
            axis="x",
            min_gap=0.0,
            max_gap=0.004,
            name="handle halves meet near the crown at rest",
        )
        ctx.expect_origin_gap(
            left_handle,
            body,
            axis="z",
            min_gap=0.05,
            name="left handle hinge sits above the enclosure centerline",
        )

    rest_left_aabb = ctx.part_world_aabb(left_handle)
    rest_right_aabb = ctx.part_world_aabb(right_handle)
    with ctx.pose({left_joint: 0.55, right_joint: 0.55}):
        lifted_left_aabb = ctx.part_world_aabb(left_handle)
        lifted_right_aabb = ctx.part_world_aabb(right_handle)

    ctx.check(
        "left handle lifts upward when its hinge rotates",
        rest_left_aabb is not None
        and lifted_left_aabb is not None
        and lifted_left_aabb[1][2] > rest_left_aabb[1][2] + 0.01,
        details=f"rest_aabb={rest_left_aabb}, lifted_aabb={lifted_left_aabb}",
    )
    ctx.check(
        "right handle lifts upward when its hinge rotates",
        rest_right_aabb is not None
        and lifted_right_aabb is not None
        and lifted_right_aabb[1][2] > rest_right_aabb[1][2] + 0.01,
        details=f"rest_aabb={rest_right_aabb}, lifted_aabb={lifted_right_aabb}",
    )

    for part_name, joint_name in button_specs:
        button = object_model.get_part(part_name)
        joint = object_model.get_articulation(joint_name)
        rest_pos = ctx.part_world_position(button)
        with ctx.pose({joint: joint.motion_limits.upper if joint.motion_limits is not None else 0.0}):
            pressed_pos = ctx.part_world_position(button)
        ctx.check(
            f"{part_name} presses downward",
            rest_pos is not None and pressed_pos is not None and pressed_pos[2] < rest_pos[2] - 0.0015,
            details=f"rest={rest_pos}, pressed={pressed_pos}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
