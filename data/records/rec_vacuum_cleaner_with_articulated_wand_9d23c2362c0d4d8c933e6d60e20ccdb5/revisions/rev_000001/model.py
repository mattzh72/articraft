from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    CapsuleGeometry,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _capsule_x(radius: float, straight_length: float, mesh_name: str):
    """CapsuleGeometry is z-aligned; return a managed mesh rotated along +X."""
    geom = CapsuleGeometry(radius, straight_length, radial_segments=36, height_segments=8)
    geom.rotate_y(math.pi / 2.0)
    return mesh_from_geometry(geom, mesh_name)


def _add_x_cylinder(
    part,
    *,
    radius: float,
    x0: float,
    x1: float,
    material: Material,
    name: str,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=x1 - x0),
        origin=Origin(xyz=((x0 + x1) * 0.5, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=material,
        name=name,
    )


def _add_y_cylinder(
    part,
    *,
    radius: float,
    length: float,
    xyz: tuple[float, float, float],
    material: Material,
    name: str,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=material,
        name=name,
    )


def _add_wand_segment(
    part,
    *,
    length: float,
    tube_material: Material,
    joint_material: Material,
    proximal_name: str,
) -> None:
    """A rigid tube with a readable horizontal hinge barrel at its proximal end."""
    _add_y_cylinder(
        part,
        radius=0.043,
        length=0.220,
        xyz=(0.0, 0.0, 0.0),
        material=joint_material,
        name=proximal_name,
    )
    _add_x_cylinder(
        part,
        radius=0.023,
        x0=0.040,
        x1=length - 0.050,
        material=tube_material,
        name="tube",
    )
    _add_x_cylinder(
        part,
        radius=0.032,
        x0=0.038,
        x1=0.115,
        material=joint_material,
        name="proximal_cuff",
    )
    _add_x_cylinder(
        part,
        radius=0.032,
        x0=length - 0.125,
        x1=length - 0.050,
        material=joint_material,
        name="distal_cuff",
    )


def _add_distal_fork(
    part,
    *,
    length: float,
    material: Material,
) -> None:
    """Fork cheeks connect a parent tube to the hinge pin without touching the child barrel."""
    part.visual(
        Box((0.055, 0.300, 0.050)),
        origin=Origin(xyz=(length - 0.0775, 0.0, 0.0)),
        material=material,
        name="distal_fork_bridge",
    )
    part.visual(
        Box((0.070, 0.035, 0.105)),
        origin=Origin(xyz=(length - 0.020, 0.135, 0.0)),
        material=material,
        name="distal_fork_0",
    )
    part.visual(
        Box((0.070, 0.035, 0.105)),
        origin=Origin(xyz=(length - 0.020, -0.135, 0.0)),
        material=material,
        name="distal_fork_1",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="articulated_wand_vacuum")

    red = model.material("red_shell", rgba=(0.72, 0.06, 0.04, 1.0))
    dark = model.material("dark_plastic", rgba=(0.035, 0.038, 0.042, 1.0))
    rubber = model.material("matte_rubber", rgba=(0.010, 0.010, 0.012, 1.0))
    charcoal = model.material("charcoal_plastic", rgba=(0.12, 0.13, 0.14, 1.0))
    metal = model.material("brushed_metal", rgba=(0.72, 0.74, 0.76, 1.0))
    grey = model.material("nozzle_grey", rgba=(0.40, 0.42, 0.43, 1.0))
    blue = model.material("dust_bin_blue", rgba=(0.20, 0.48, 0.62, 0.55))

    body = model.part("body")
    body.visual(
        Box((0.720, 0.360, 0.105)),
        origin=Origin(xyz=(-0.020, 0.0, 0.075)),
        material=dark,
        name="lower_chassis",
    )
    body.visual(
        _capsule_x(0.160, 0.330, "vacuum_canister_shell"),
        origin=Origin(xyz=(-0.050, 0.0, 0.260)),
        material=red,
        name="canister_shell",
    )
    body.visual(
        Box((0.310, 0.330, 0.080)),
        origin=Origin(xyz=(0.020, 0.0, 0.365)),
        material=blue,
        name="dust_bin_window",
    )
    body.visual(
        Cylinder(radius=0.152, length=0.070),
        origin=Origin(xyz=(-0.385, 0.0, 0.260), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=charcoal,
        name="motor_cap",
    )
    body.visual(
        Box((0.125, 0.170, 0.095)),
        origin=Origin(xyz=(0.314, 0.0, 0.350)),
        material=charcoal,
        name="socket_block",
    )
    body.visual(
        Box((0.110, 0.035, 0.115)),
        origin=Origin(xyz=(0.372, 0.136, 0.350)),
        material=charcoal,
        name="socket_yoke_0",
    )
    body.visual(
        Box((0.110, 0.035, 0.115)),
        origin=Origin(xyz=(0.372, -0.136, 0.350)),
        material=charcoal,
        name="socket_yoke_1",
    )
    body.visual(
        Box((0.055, 0.305, 0.060)),
        origin=Origin(xyz=(0.330, 0.0, 0.350)),
        material=charcoal,
        name="yoke_bridge",
    )
    _add_y_cylinder(
        body,
        radius=0.012,
        length=0.300,
        xyz=(0.420, 0.0, 0.350),
        material=metal,
        name="socket_pin",
    )
    body.visual(
        Box((0.380, 0.060, 0.045)),
        origin=Origin(xyz=(-0.080, 0.0, 0.425)),
        material=dark,
        name="carry_handle",
    )
    _add_y_cylinder(
        body,
        radius=0.085,
        length=0.060,
        xyz=(-0.220, 0.205, 0.075),
        material=rubber,
        name="wheel_0",
    )
    _add_y_cylinder(
        body,
        radius=0.085,
        length=0.060,
        xyz=(-0.220, -0.205, 0.075),
        material=rubber,
        name="wheel_1",
    )
    _add_y_cylinder(
        body,
        radius=0.040,
        length=0.260,
        xyz=(-0.220, 0.0, 0.075),
        material=metal,
        name="wheel_axle",
    )

    wand_lengths = (0.380, 0.460, 0.360)
    wand_0 = model.part("wand_0")
    _add_wand_segment(
        wand_0,
        length=wand_lengths[0],
        tube_material=metal,
        joint_material=charcoal,
        proximal_name="body_elbow_barrel",
    )
    _add_y_cylinder(
        wand_0,
        radius=0.012,
        length=0.300,
        xyz=(wand_lengths[0], 0.0, 0.0),
        material=metal,
        name="distal_pin",
    )
    _add_distal_fork(wand_0, length=wand_lengths[0], material=charcoal)

    wand_1 = model.part("wand_1")
    _add_wand_segment(
        wand_1,
        length=wand_lengths[1],
        tube_material=metal,
        joint_material=charcoal,
        proximal_name="middle_elbow_barrel",
    )
    _add_y_cylinder(
        wand_1,
        radius=0.012,
        length=0.300,
        xyz=(wand_lengths[1], 0.0, 0.0),
        material=metal,
        name="distal_pin",
    )
    _add_distal_fork(wand_1, length=wand_lengths[1], material=charcoal)

    wand_2 = model.part("wand_2")
    _add_wand_segment(
        wand_2,
        length=wand_lengths[2],
        tube_material=metal,
        joint_material=charcoal,
        proximal_name="lower_elbow_barrel",
    )
    _add_y_cylinder(
        wand_2,
        radius=0.012,
        length=0.300,
        xyz=(wand_lengths[2], 0.0, 0.0),
        material=metal,
        name="distal_pin",
    )
    _add_distal_fork(wand_2, length=wand_lengths[2], material=charcoal)

    nozzle = model.part("nozzle")
    _add_y_cylinder(
        nozzle,
        radius=0.040,
        length=0.220,
        xyz=(0.0, 0.0, 0.0),
        material=charcoal,
        name="pitch_barrel",
    )
    nozzle.visual(
        Box((0.070, 0.120, 0.075)),
        origin=Origin(xyz=(0.035, 0.0, -0.075)),
        material=charcoal,
        name="hinge_neck",
    )
    nozzle.visual(
        Box((0.460, 0.320, 0.055)),
        origin=Origin(xyz=(0.230, 0.0, -0.120)),
        material=grey,
        name="nozzle_housing",
    )
    nozzle.visual(
        Box((0.050, 0.340, 0.036)),
        origin=Origin(xyz=(0.465, 0.0, -0.103)),
        material=rubber,
        name="front_bumper",
    )
    nozzle.visual(
        Box((0.300, 0.060, 0.008)),
        origin=Origin(xyz=(0.275, 0.0, -0.151)),
        material=dark,
        name="suction_slot",
    )
    nozzle.visual(
        Box((0.030, 0.285, 0.026)),
        origin=Origin(xyz=(0.420, 0.0, -0.152)),
        material=rubber,
        name="brush_strip",
    )
    _add_y_cylinder(
        nozzle,
        radius=0.030,
        length=0.040,
        xyz=(0.125, 0.180, -0.138),
        material=rubber,
        name="nozzle_wheel_0",
    )
    _add_y_cylinder(
        nozzle,
        radius=0.030,
        length=0.040,
        xyz=(0.125, -0.180, -0.138),
        material=rubber,
        name="nozzle_wheel_1",
    )

    model.articulation(
        "body_to_wand_0",
        ArticulationType.REVOLUTE,
        parent=body,
        child=wand_0,
        origin=Origin(xyz=(0.420, 0.0, 0.350), rpy=(0.0, -0.58, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.8, lower=-0.55, upper=0.70),
    )
    model.articulation(
        "wand_0_to_wand_1",
        ArticulationType.REVOLUTE,
        parent=wand_0,
        child=wand_1,
        origin=Origin(xyz=(wand_lengths[0], 0.0, 0.0), rpy=(0.0, 0.73, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=14.0, velocity=1.8, lower=-0.70, upper=0.85),
    )
    model.articulation(
        "wand_1_to_wand_2",
        ArticulationType.REVOLUTE,
        parent=wand_1,
        child=wand_2,
        origin=Origin(xyz=(wand_lengths[1], 0.0, 0.0), rpy=(0.0, 0.90, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=14.0, velocity=1.8, lower=-0.65, upper=0.75),
    )
    model.articulation(
        "wand_2_to_nozzle",
        ArticulationType.REVOLUTE,
        parent=wand_2,
        child=nozzle,
        origin=Origin(xyz=(wand_lengths[2], 0.0, 0.0), rpy=(0.0, -1.05, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.2, lower=-0.55, upper=0.55),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    wand_0 = object_model.get_part("wand_0")
    wand_1 = object_model.get_part("wand_1")
    wand_2 = object_model.get_part("wand_2")
    nozzle = object_model.get_part("nozzle")
    body_elbow = object_model.get_articulation("body_to_wand_0")
    elbow_0 = object_model.get_articulation("wand_0_to_wand_1")
    elbow_1 = object_model.get_articulation("wand_1_to_wand_2")
    nozzle_pitch = object_model.get_articulation("wand_2_to_nozzle")

    ctx.check(
        "articulated wand chain present",
        all(item is not None for item in (body, wand_0, wand_1, wand_2, nozzle)),
        "Expected body, three rigid wand links, and a floor nozzle.",
    )
    ctx.check(
        "four readable revolute pitch joints",
        all(item is not None for item in (body_elbow, elbow_0, elbow_1, nozzle_pitch)),
        "Expected two wand elbows plus the body elbow and nozzle pitch hinge.",
    )
    if None in (body, wand_0, wand_1, wand_2, nozzle, body_elbow, elbow_0, elbow_1, nozzle_pitch):
        return ctx.report()

    hinge_pairs = (
        (body, wand_0, "socket_pin", "body_elbow_barrel", "body hinge pin is captured inside the first wand barrel"),
        (wand_0, wand_1, "distal_pin", "middle_elbow_barrel", "wand elbow pin is captured inside the middle barrel"),
        (wand_1, wand_2, "distal_pin", "lower_elbow_barrel", "second wand elbow pin is captured inside the lower barrel"),
        (wand_2, nozzle, "distal_pin", "pitch_barrel", "nozzle pitch pin is captured inside the nozzle barrel"),
    )
    for parent_part, child_part, pin_elem, barrel_elem, reason in hinge_pairs:
        ctx.allow_overlap(
            parent_part,
            child_part,
            elem_a=pin_elem,
            elem_b=barrel_elem,
            reason=reason,
        )
        ctx.expect_within(
            parent_part,
            child_part,
            axes="xz",
            inner_elem=pin_elem,
            outer_elem=barrel_elem,
            margin=0.002,
            name=f"{pin_elem} retained in {barrel_elem}",
        )
        ctx.expect_overlap(
            parent_part,
            child_part,
            axes="y",
            elem_a=pin_elem,
            elem_b=barrel_elem,
            min_overlap=0.180,
            name=f"{pin_elem} passes through {barrel_elem}",
        )

    ctx.expect_origin_distance(
        wand_0,
        wand_1,
        axes="xz",
        min_dist=0.340,
        name="body elbow and first wand elbow are separated in profile",
    )
    ctx.expect_origin_distance(
        wand_1,
        wand_2,
        axes="xz",
        min_dist=0.420,
        name="wand elbow axes are visibly spaced",
    )
    ctx.expect_origin_distance(
        wand_2,
        nozzle,
        axes="xz",
        min_dist=0.320,
        name="lower elbow and nozzle pitch axis are spaced",
    )
    ctx.expect_origin_gap(
        nozzle,
        body,
        axis="x",
        min_gap=0.600,
        name="floor nozzle sits forward of main body",
    )

    nozzle_aabb = ctx.part_world_aabb(nozzle)
    ctx.check(
        "floor nozzle sits close to floor",
        nozzle_aabb is not None and 0.005 <= float(nozzle_aabb[0][2]) <= 0.080,
        details=f"nozzle_aabb={nozzle_aabb!r}",
    )

    rest_nozzle = ctx.part_world_position(nozzle)
    with ctx.pose({elbow_0: 0.45}):
        bent_nozzle = ctx.part_world_position(nozzle)
    ctx.check(
        "middle elbow bends the wand chain",
        rest_nozzle is not None
        and bent_nozzle is not None
        and abs(float(bent_nozzle[2]) - float(rest_nozzle[2])) > 0.080,
        details=f"rest={rest_nozzle!r}, bent={bent_nozzle!r}",
    )

    rest_aabb = ctx.part_world_aabb(nozzle)
    with ctx.pose({nozzle_pitch: 0.45}):
        pitched_aabb = ctx.part_world_aabb(nozzle)
    ctx.check(
        "floor nozzle pitch changes its profile height",
        rest_aabb is not None
        and pitched_aabb is not None
        and abs(float(pitched_aabb[1][2] - rest_aabb[1][2])) > 0.040,
        details=f"rest={rest_aabb!r}, pitched={pitched_aabb!r}",
    )

    return ctx.report()


object_model = build_object_model()
