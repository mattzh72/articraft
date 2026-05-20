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
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def _x_cylinder(
    part,
    *,
    radius: float,
    length: float,
    x: float,
    y: float = 0.0,
    z: float = 0.0,
    material,
    name: str,
):
    return part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=(x, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=material,
        name=name,
    )


def _y_cylinder(
    part,
    *,
    radius: float,
    length: float,
    x: float = 0.0,
    y: float = 0.0,
    z: float = 0.0,
    material,
    name: str,
):
    return part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=(x, y, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def _add_barrel(
    model: ArticulatedObject,
    *,
    name: str,
    inner_sign: float,
    body_material,
    trim_material,
    with_diopter_mount: bool,
):
    barrel = model.part(name)

    _x_cylinder(
        barrel,
        radius=0.037,
        length=0.125,
        x=0.043,
        material=body_material,
        name="objective_housing",
    )
    _x_cylinder(
        barrel,
        radius=0.040,
        length=0.022,
        x=0.091,
        material=trim_material,
        name="objective_guard",
    )
    _x_cylinder(
        barrel,
        radius=0.039,
        length=0.018,
        x=0.018,
        material=trim_material,
        name="grip_band",
    )
    barrel.visual(
        Box((0.060, 0.050, 0.056)),
        origin=Origin(xyz=(-0.004, 0.000, 0.002)),
        material=body_material,
        name="prism_body",
    )
    barrel.visual(
        Box((0.040, 0.044, 0.020)),
        origin=Origin(xyz=(-0.008, 0.000, 0.026)),
        material=trim_material,
        name="upper_shoulder",
    )
    barrel.visual(
        Box((0.028, 0.038, 0.016)),
        origin=Origin(xyz=(0.006, 0.000, -0.022)),
        material=trim_material,
        name="lower_keel",
    )
    barrel.visual(
        Box((0.050, 0.016, 0.054)),
        origin=Origin(xyz=(-0.005, inner_sign * 0.043, 0.002)),
        material=body_material,
        name="bridge_lug",
    )
    _x_cylinder(
        barrel,
        radius=0.025,
        length=0.042,
        x=-0.043,
        material=body_material,
        name="eyepiece_tube",
    )

    if with_diopter_mount:
        _x_cylinder(
            barrel,
            radius=0.023,
            length=0.008,
            x=-0.060,
            material=trim_material,
            name="diopter_seat",
        )
    else:
        _x_cylinder(
            barrel,
            radius=0.027,
            length=0.014,
            x=-0.071,
            material=trim_material,
            name="eyecup_collar",
        )
        _x_cylinder(
            barrel,
            radius=0.028,
            length=0.018,
            x=-0.086,
            material=body_material,
            name="eyecup_shell",
        )
        _x_cylinder(
            barrel,
            radius=0.031,
            length=0.008,
            x=-0.099,
            material=trim_material,
            name="eyecup_lip",
        )

    barrel.inertial = Inertial.from_geometry(
        Box((0.215, 0.082, 0.082)),
        mass=0.62,
        origin=Origin(xyz=(0.000, 0.000, 0.000)),
    )
    return barrel


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="marine_waterproof_binoculars")

    armor_black = model.material("armor_black", rgba=(0.15, 0.17, 0.18, 1.0))
    trim_black = model.material("trim_black", rgba=(0.08, 0.09, 0.10, 1.0))
    knob_black = model.material("knob_black", rgba=(0.05, 0.05, 0.06, 1.0))
    dial_black = model.material("dial_black", rgba=(0.10, 0.10, 0.11, 1.0))

    bridge_body = model.part("bridge_body")
    bridge_body.visual(
        Box((0.060, 0.038, 0.046)),
        origin=Origin(xyz=(0.000, 0.000, 0.000)),
        material=armor_black,
        name="center_block",
    )
    bridge_body.visual(
        Box((0.050, 0.030, 0.020)),
        origin=Origin(xyz=(-0.028, 0.000, 0.018)),
        material=trim_black,
        name="upper_bridge",
    )
    bridge_body.visual(
        Box((0.036, 0.026, 0.016)),
        origin=Origin(xyz=(0.020, 0.000, -0.024)),
        material=trim_black,
        name="lower_bridge",
    )
    bridge_body.visual(
        Box((0.024, 0.018, 0.010)),
        origin=Origin(xyz=(-0.012, 0.000, 0.020)),
        material=trim_black,
        name="focus_pedestal",
    )
    bridge_body.visual(
        Box((0.030, 0.008, 0.024)),
        origin=Origin(xyz=(-0.024, 0.024, 0.046)),
        material=trim_black,
        name="knob_left_cheek",
    )
    bridge_body.visual(
        Box((0.030, 0.008, 0.024)),
        origin=Origin(xyz=(-0.024, -0.024, 0.046)),
        material=trim_black,
        name="knob_right_cheek",
    )
    bridge_body.visual(
        Box((0.012, 0.046, 0.024)),
        origin=Origin(xyz=(-0.041, 0.000, 0.040)),
        material=trim_black,
        name="focus_support_block",
    )
    bridge_body.inertial = Inertial.from_geometry(
        Box((0.090, 0.060, 0.080)),
        mass=0.34,
        origin=Origin(xyz=(0.000, 0.000, 0.010)),
    )

    left_barrel = _add_barrel(
        model,
        name="left_barrel",
        inner_sign=-1.0,
        body_material=armor_black,
        trim_material=trim_black,
        with_diopter_mount=False,
    )
    right_barrel = _add_barrel(
        model,
        name="right_barrel",
        inner_sign=1.0,
        body_material=armor_black,
        trim_material=trim_black,
        with_diopter_mount=True,
    )

    focus_wheel = model.part("focus_wheel")
    _y_cylinder(
        focus_wheel,
        radius=0.005,
        length=0.040,
        z=0.000,
        material=trim_black,
        name="focus_axle",
    )
    _y_cylinder(
        focus_wheel,
        radius=0.019,
        length=0.022,
        z=0.000,
        material=knob_black,
        name="focus_wheel_body",
    )
    _y_cylinder(
        focus_wheel,
        radius=0.021,
        length=0.012,
        z=0.000,
        material=dial_black,
        name="focus_tread",
    )
    focus_wheel.inertial = Inertial.from_geometry(
        Cylinder(radius=0.021, length=0.040),
        mass=0.05,
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
    )

    right_diopter = model.part("right_diopter_assembly")
    _x_cylinder(
        right_diopter,
        radius=0.027,
        length=0.022,
        x=0.000,
        material=dial_black,
        name="diopter_ring",
    )
    _x_cylinder(
        right_diopter,
        radius=0.028,
        length=0.018,
        x=-0.020,
        material=armor_black,
        name="eyecup_shell",
    )
    _x_cylinder(
        right_diopter,
        radius=0.031,
        length=0.008,
        x=-0.033,
        material=trim_black,
        name="eyecup_lip",
    )
    right_diopter.inertial = Inertial.from_geometry(
        Box((0.048, 0.064, 0.064)),
        mass=0.03,
        origin=Origin(xyz=(-0.018, 0.000, 0.000)),
    )

    model.articulation(
        "bridge_to_left_barrel",
        ArticulationType.FIXED,
        parent=bridge_body,
        child=left_barrel,
        origin=Origin(xyz=(0.000, 0.070, 0.000)),
    )
    model.articulation(
        "bridge_to_right_barrel",
        ArticulationType.FIXED,
        parent=bridge_body,
        child=right_barrel,
        origin=Origin(xyz=(0.000, -0.070, 0.000)),
    )
    model.articulation(
        "bridge_to_focus_wheel",
        ArticulationType.CONTINUOUS,
        parent=bridge_body,
        child=focus_wheel,
        origin=Origin(xyz=(-0.012, 0.000, 0.046)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=0.4, velocity=10.0),
    )
    model.articulation(
        "right_barrel_to_diopter",
        ArticulationType.CONTINUOUS,
        parent=right_barrel,
        child=right_diopter,
        origin=Origin(xyz=(-0.075, 0.000, 0.000)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=0.2, velocity=8.0),
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

    bridge_body = object_model.get_part("bridge_body")
    left_barrel = object_model.get_part("left_barrel")
    right_barrel = object_model.get_part("right_barrel")
    focus_wheel = object_model.get_part("focus_wheel")
    right_diopter = object_model.get_part("right_diopter_assembly")

    focus_joint = object_model.get_articulation("bridge_to_focus_wheel")
    diopter_joint = object_model.get_articulation("right_barrel_to_diopter")
    center_block = bridge_body.get_visual("center_block")
    knob_left_cheek = bridge_body.get_visual("knob_left_cheek")
    knob_right_cheek = bridge_body.get_visual("knob_right_cheek")
    left_bridge_lug = left_barrel.get_visual("bridge_lug")
    right_bridge_lug = right_barrel.get_visual("bridge_lug")
    focus_axle = focus_wheel.get_visual("focus_axle")
    diopter_ring = right_diopter.get_visual("diopter_ring")
    right_eyepiece_tube = right_barrel.get_visual("eyepiece_tube")

    ctx.expect_contact(
        left_barrel,
        bridge_body,
        elem_a=left_bridge_lug,
        elem_b=center_block,
        name="left barrel is mounted on the bridge",
    )
    ctx.expect_contact(
        right_barrel,
        bridge_body,
        elem_a=right_bridge_lug,
        elem_b=center_block,
        name="right barrel is mounted on the bridge",
    )
    ctx.expect_contact(
        focus_wheel,
        bridge_body,
        elem_a=focus_axle,
        elem_b=knob_left_cheek,
        name="focus wheel axle is supported by the left cheek",
    )
    ctx.expect_contact(
        focus_wheel,
        bridge_body,
        elem_a=focus_axle,
        elem_b=knob_right_cheek,
        name="focus wheel axle is supported by the right cheek",
    )
    ctx.expect_contact(
        right_diopter,
        right_barrel,
        elem_a=diopter_ring,
        elem_b=right_eyepiece_tube,
        name="diopter ring seats against the right eyepiece tube",
    )
    ctx.expect_origin_gap(
        left_barrel,
        right_barrel,
        axis="y",
        min_gap=0.13,
        max_gap=0.15,
        name="binocular barrels keep realistic center spacing",
    )

    ctx.check(
        "focus wheel uses continuous transverse rotation",
        focus_joint.articulation_type == ArticulationType.CONTINUOUS
        and focus_joint.axis == (0.0, 1.0, 0.0)
        and focus_joint.motion_limits is not None
        and focus_joint.motion_limits.lower is None
        and focus_joint.motion_limits.upper is None,
        details=(
            f"type={focus_joint.articulation_type}, axis={focus_joint.axis}, "
            f"limits={focus_joint.motion_limits}"
        ),
    )
    ctx.check(
        "right eyepiece diopter uses continuous axial rotation",
        diopter_joint.articulation_type == ArticulationType.CONTINUOUS
        and diopter_joint.axis == (1.0, 0.0, 0.0)
        and diopter_joint.motion_limits is not None
        and diopter_joint.motion_limits.lower is None
        and diopter_joint.motion_limits.upper is None,
        details=(
            f"type={diopter_joint.articulation_type}, axis={diopter_joint.axis}, "
            f"limits={diopter_joint.motion_limits}"
        ),
    )

    with ctx.pose({focus_joint: 1.6, diopter_joint: -1.2}):
        ctx.expect_contact(
            focus_wheel,
            bridge_body,
            elem_a=focus_axle,
            elem_b=knob_left_cheek,
            name="focus wheel remains supported while rotated",
        )
        ctx.expect_contact(
            right_diopter,
            right_barrel,
            elem_a=diopter_ring,
            elem_b=right_eyepiece_tube,
            name="diopter ring remains seated while rotated",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
