from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

import cadquery as cq

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
    mesh_from_cadquery,
)


BASE_RADIUS = 0.30
BASE_SUPPORT_RADIUS = 0.185
BASE_PEDESTAL_RADIUS = 0.095
BASE_TOTAL_HEIGHT = 0.100

LOWER_RADIUS = 0.36
LOWER_TOP_Z = 0.050

CHEEK_OFFSET_X = 0.215
CHEEK_AXIS_HEIGHT = 0.238

UPPER_RADIUS = 0.145


def _disk(radius: float, z: float, height: float) -> cq.Workplane:
    return cq.Workplane("XY").workplane(offset=z).circle(radius).extrude(height)


def _ring(outer_radius: float, inner_radius: float, z: float, height: float) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .workplane(offset=z)
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(height)
    )


def _base_shape() -> cq.Workplane:
    housing = (
        _disk(BASE_RADIUS, 0.000, 0.062)
        .union(_disk(0.230, 0.062, 0.028))
        .union(_disk(BASE_SUPPORT_RADIUS, 0.090, 0.010))
        .union(_disk(BASE_PEDESTAL_RADIUS, 0.068, 0.026))
    )
    return housing


def _lower_turntable_shape() -> cq.Workplane:
    platter = (
        _ring(BASE_SUPPORT_RADIUS - 0.006, 0.050, 0.000, 0.016)
        .union(_disk(LOWER_RADIUS, 0.010, LOWER_TOP_Z - 0.010))
        .union(_disk(0.155, 0.010, 0.028))
    )
    return platter


def _side_cheek_shape() -> cq.Workplane:
    foot = cq.Workplane("XY").box(0.160, 0.190, 0.032, centered=(True, True, False))

    web = (
        cq.Workplane("XY")
        .workplane(offset=0.032)
        .box(0.085, 0.180, 0.188, centered=(False, True, False))
        .translate((-0.065, 0.0, 0.0))
    )

    gusset_profile = (
        cq.Workplane("XZ")
        .moveTo(-0.065, 0.032)
        .lineTo(-0.065, 0.135)
        .lineTo(-0.010, 0.208)
        .lineTo(0.000, 0.208)
        .lineTo(0.000, 0.032)
        .close()
    )
    gusset = gusset_profile.extrude(0.150, both=True)

    support_column = _disk(0.070, 0.154, 0.084)
    upper_flange = _disk(0.112, 0.226, 0.012)

    cheek = foot.union(web).union(gusset).union(support_column).union(upper_flange)
    return cheek


def _upper_face_shape() -> cq.Workplane:
    face = (
        _ring(0.112, 0.040, 0.000, 0.010)
        .union(_disk(UPPER_RADIUS, 0.010, 0.026))
        .union(_disk(0.055, 0.036, 0.010))
    )
    return face


def _aabb_center(aabb):
    if aabb is None:
        return None
    minimum, maximum = aabb
    return tuple((minimum[i] + maximum[i]) * 0.5 for i in range(3))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="side_cheek_offset_dual_turntable")

    model.material("base_paint", rgba=(0.18, 0.19, 0.21, 1.0))
    model.material("turntable_gray", rgba=(0.67, 0.69, 0.72, 1.0))
    model.material("cheek_blue", rgba=(0.20, 0.33, 0.46, 1.0))
    model.material("upper_face_aluminum", rgba=(0.78, 0.80, 0.82, 1.0))
    model.material("indicator_red", rgba=(0.70, 0.16, 0.14, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_base_shape(), "base_housing"),
        material="base_paint",
        name="base_housing",
    )
    base.inertial = Inertial.from_geometry(
        Cylinder(radius=BASE_RADIUS, length=BASE_TOTAL_HEIGHT),
        mass=26.0,
        origin=Origin(xyz=(0.0, 0.0, BASE_TOTAL_HEIGHT * 0.5)),
    )

    lower_turntable = model.part("lower_turntable")
    lower_turntable.visual(
        mesh_from_cadquery(_lower_turntable_shape(), "lower_turntable_shell"),
        material="turntable_gray",
        name="lower_turntable_shell",
    )
    lower_turntable.inertial = Inertial.from_geometry(
        Cylinder(radius=LOWER_RADIUS, length=0.070),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, 0.015)),
    )

    side_cheek = model.part("side_cheek")
    side_cheek.visual(
        mesh_from_cadquery(_side_cheek_shape(), "side_cheek_body"),
        material="cheek_blue",
        name="side_cheek_body",
    )
    side_cheek.inertial = Inertial.from_geometry(
        Box((0.160, 0.190, CHEEK_AXIS_HEIGHT)),
        mass=10.0,
        origin=Origin(xyz=(-0.010, 0.0, CHEEK_AXIS_HEIGHT * 0.5)),
    )

    upper_face = model.part("upper_face")
    upper_face.visual(
        mesh_from_cadquery(_upper_face_shape(), "upper_face_plate"),
        material="upper_face_aluminum",
        name="upper_face_plate",
    )
    upper_face.visual(
        Box((0.090, 0.034, 0.010)),
        origin=Origin(xyz=(0.104, 0.0, 0.039)),
        material="indicator_red",
        name="upper_indicator",
    )
    upper_face.inertial = Inertial.from_geometry(
        Cylinder(radius=UPPER_RADIUS, length=0.050),
        mass=4.2,
        origin=Origin(xyz=(0.0, 0.0, 0.022)),
    )

    model.articulation(
        "base_to_lower_turntable",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=lower_turntable,
        origin=Origin(xyz=(0.0, 0.0, BASE_TOTAL_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=90.0, velocity=1.5),
    )
    model.articulation(
        "lower_turntable_to_side_cheek",
        ArticulationType.FIXED,
        parent=lower_turntable,
        child=side_cheek,
        origin=Origin(xyz=(CHEEK_OFFSET_X, 0.0, LOWER_TOP_Z)),
    )
    model.articulation(
        "side_cheek_to_upper_face",
        ArticulationType.CONTINUOUS,
        parent=side_cheek,
        child=upper_face,
        origin=Origin(xyz=(0.0, 0.0, CHEEK_AXIS_HEIGHT)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=25.0, velocity=2.5),
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
    base = object_model.get_part("base")
    lower_turntable = object_model.get_part("lower_turntable")
    side_cheek = object_model.get_part("side_cheek")
    upper_face = object_model.get_part("upper_face")

    lower_joint = object_model.get_articulation("base_to_lower_turntable")
    cheek_mount = object_model.get_articulation("lower_turntable_to_side_cheek")
    upper_joint = object_model.get_articulation("side_cheek_to_upper_face")

    ctx.check(
        "lower axis is vertical",
        tuple(lower_joint.axis) == (0.0, 0.0, 1.0),
        details=f"axis={lower_joint.axis}",
    )
    ctx.check(
        "upper axis is vertical",
        tuple(upper_joint.axis) == (0.0, 0.0, 1.0),
        details=f"axis={upper_joint.axis}",
    )
    ctx.check(
        "upper axis is offset from base axis",
        cheek_mount.origin.xyz[0] > 0.18 and abs(cheek_mount.origin.xyz[1]) < 1e-9,
        details=f"mount_origin={cheek_mount.origin.xyz}",
    )

    ctx.expect_contact(
        base,
        lower_turntable,
        name="lower turntable is seated on the base support ring",
    )
    ctx.expect_contact(
        lower_turntable,
        side_cheek,
        name="side cheek is mounted onto the lower turntable",
    )
    ctx.expect_contact(
        side_cheek,
        upper_face,
        name="upper rotary face is seated on the cheek support flange",
    )
    ctx.expect_overlap(
        base,
        lower_turntable,
        axes="xy",
        min_overlap=0.36,
        name="lower turntable stays broadly centered over the base",
    )

    cheek_rest = ctx.part_world_position(side_cheek)
    with ctx.pose({lower_joint: math.pi * 0.5}):
        cheek_quarter_turn = ctx.part_world_position(side_cheek)
    ctx.check(
        "lower turntable swings the cheek around the base axis",
        cheek_rest is not None
        and cheek_quarter_turn is not None
        and cheek_rest[0] > 0.18
        and abs(cheek_rest[1]) < 0.02
        and abs(cheek_quarter_turn[0]) < 0.02
        and cheek_quarter_turn[1] > 0.18,
        details=f"rest={cheek_rest}, quarter_turn={cheek_quarter_turn}",
    )

    cheek_before_upper_spin = ctx.part_world_position(side_cheek)
    indicator_rest = _aabb_center(ctx.part_element_world_aabb(upper_face, elem="upper_indicator"))
    with ctx.pose({upper_joint: math.pi * 0.5}):
        cheek_after_upper_spin = ctx.part_world_position(side_cheek)
        indicator_quarter_turn = _aabb_center(
            ctx.part_element_world_aabb(upper_face, elem="upper_indicator")
        )
    ctx.check(
        "upper face spins independently of the cheek support",
        cheek_before_upper_spin is not None
        and cheek_after_upper_spin is not None
        and indicator_rest is not None
        and indicator_quarter_turn is not None
        and max(
            abs(cheek_before_upper_spin[i] - cheek_after_upper_spin[i]) for i in range(3)
        )
        < 1e-6
        and abs(indicator_rest[0] - indicator_quarter_turn[0]) > 0.05
        and abs(indicator_rest[1] - indicator_quarter_turn[1]) > 0.05,
        details=(
            f"cheek_before={cheek_before_upper_spin}, "
            f"cheek_after={cheek_after_upper_spin}, "
            f"indicator_rest={indicator_rest}, "
            f"indicator_quarter_turn={indicator_quarter_turn}"
        ),
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
