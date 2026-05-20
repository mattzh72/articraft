from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    LatheGeometry,
    Material,
    MotionLimits,
    Origin,
    Sphere,
    TorusGeometry,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


ARM_MAT = Material("satin_black_metal", rgba=(0.015, 0.014, 0.013, 1.0))
CLAMP_MAT = Material("black_cast_clamp", rgba=(0.025, 0.024, 0.022, 1.0))
RUBBER_MAT = Material("soft_rubber", rgba=(0.004, 0.004, 0.004, 1.0))
SHADE_MAT = Material("deep_green_enamel", rgba=(0.02, 0.18, 0.12, 1.0))
RIM_MAT = Material("black_trim", rgba=(0.01, 0.01, 0.01, 1.0))
BULB_MAT = Material("warm_bulb", rgba=(1.0, 0.82, 0.36, 0.95))


def _cyl_x(radius: float, length: float) -> tuple[Cylinder, Origin]:
    return Cylinder(radius=radius, length=length), Origin(rpy=(0.0, math.pi / 2.0, 0.0))


def _cyl_y(radius: float, length: float) -> tuple[Cylinder, Origin]:
    return Cylinder(radius=radius, length=length), Origin(rpy=(math.pi / 2.0, 0.0, 0.0))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(
        name="clip_on_drafting_lamp",
        materials=[ARM_MAT, CLAMP_MAT, RUBBER_MAT, SHADE_MAT, RIM_MAT, BULB_MAT],
    )

    clamp = model.part("clamp")
    clamp.visual(Box((0.160, 0.075, 0.018)), origin=Origin(xyz=(0.020, 0.0, 0.009)), material=CLAMP_MAT, name="lower_jaw")
    clamp.visual(Box((0.150, 0.075, 0.016)), origin=Origin(xyz=(0.015, 0.0, 0.092)), material=CLAMP_MAT, name="upper_jaw")
    clamp.visual(Box((0.026, 0.075, 0.100)), origin=Origin(xyz=(-0.060, 0.0, 0.050)), material=CLAMP_MAT, name="rear_bridge")
    clamp.visual(Box((0.065, 0.060, 0.006)), origin=Origin(xyz=(0.055, 0.0, 0.021)), material=RUBBER_MAT, name="lower_pad")
    clamp.visual(Box((0.065, 0.060, 0.006)), origin=Origin(xyz=(0.055, 0.0, 0.081)), material=RUBBER_MAT, name="upper_pad")
    clamp.visual(Cylinder(radius=0.016, length=0.085), origin=Origin(xyz=(-0.025, 0.0, 0.1425)), material=ARM_MAT, name="post_tube")
    clamp.visual(Cylinder(radius=0.026, length=0.010), origin=Origin(xyz=(-0.025, 0.0, 0.190)), material=ARM_MAT, name="post_cap")

    first_link = model.part("first_link")
    first_link.visual(Cylinder(radius=0.035, length=0.036), origin=Origin(xyz=(0.0, 0.0, 0.018)), material=ARM_MAT, name="swivel_hub")
    rod_x, rod_x_origin = _cyl_x(0.0065, 0.315)
    first_link.visual(rod_x, origin=Origin(xyz=(0.175, 0.025, 0.028), rpy=rod_x_origin.rpy), material=ARM_MAT, name="upper_rod_0")
    first_link.visual(rod_x, origin=Origin(xyz=(0.175, -0.025, 0.028), rpy=rod_x_origin.rpy), material=ARM_MAT, name="upper_rod_1")
    lug_y, lug_y_origin = _cyl_y(0.023, 0.016)
    first_link.visual(lug_y, origin=Origin(xyz=(0.340, 0.026, 0.028), rpy=lug_y_origin.rpy), material=ARM_MAT, name="elbow_fork_0")
    first_link.visual(lug_y, origin=Origin(xyz=(0.340, -0.026, 0.028), rpy=lug_y_origin.rpy), material=ARM_MAT, name="elbow_fork_1")

    second_link = model.part("second_link")
    barrel_y, barrel_y_origin = _cyl_y(0.018, 0.036)
    second_link.visual(barrel_y, origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=barrel_y_origin.rpy), material=ARM_MAT, name="elbow_barrel")
    lower_rod_x, lower_rod_x_origin = _cyl_x(0.0060, 0.275)
    second_link.visual(lower_rod_x, origin=Origin(xyz=(0.155, 0.018, 0.0), rpy=lower_rod_x_origin.rpy), material=ARM_MAT, name="lower_rod_0")
    second_link.visual(lower_rod_x, origin=Origin(xyz=(0.155, -0.018, 0.0), rpy=lower_rod_x_origin.rpy), material=ARM_MAT, name="lower_rod_1")
    tip_lug_y, tip_lug_y_origin = _cyl_y(0.020, 0.014)
    second_link.visual(tip_lug_y, origin=Origin(xyz=(0.305, 0.026, 0.0), rpy=tip_lug_y_origin.rpy), material=ARM_MAT, name="shade_fork_0")
    second_link.visual(tip_lug_y, origin=Origin(xyz=(0.305, -0.026, 0.0), rpy=tip_lug_y_origin.rpy), material=ARM_MAT, name="shade_fork_1")

    shade = model.part("shade")
    shade_barrel_y, shade_barrel_y_origin = _cyl_y(0.016, 0.038)
    shade.visual(shade_barrel_y, origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=shade_barrel_y_origin.rpy), material=ARM_MAT, name="shade_barrel")
    shade.visual(Cylinder(radius=0.018, length=0.056), origin=Origin(xyz=(0.0, 0.0, -0.026)), material=ARM_MAT, name="neck")
    shade.visual(Cylinder(radius=0.036, length=0.012), origin=Origin(xyz=(0.0, 0.0, -0.050)), material=RIM_MAT, name="top_collar")
    shade_shell = LatheGeometry(
        [
            (0.038, -0.052),
            (0.095, -0.155),
            (0.088, -0.155),
            (0.030, -0.056),
        ],
        segments=64,
        closed=True,
    )
    shade.visual(mesh_from_geometry(shade_shell, "cone_shade_shell"), origin=Origin(), material=SHADE_MAT, name="shade_shell")
    shade.visual(mesh_from_geometry(TorusGeometry(radius=0.0915, tube=0.0040, radial_segments=12, tubular_segments=64), "shade_rim"), origin=Origin(xyz=(0.0, 0.0, -0.155)), material=RIM_MAT, name="rim")
    shade.visual(Cylinder(radius=0.014, length=0.035), origin=Origin(xyz=(0.0, 0.0, -0.070)), material=RIM_MAT, name="socket")
    shade.visual(Sphere(radius=0.025), origin=Origin(xyz=(0.0, 0.0, -0.105)), material=BULB_MAT, name="bulb")

    model.articulation(
        "post_swivel",
        ArticulationType.REVOLUTE,
        parent=clamp,
        child=first_link,
        origin=Origin(xyz=(-0.025, 0.0, 0.195)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=-1.65, upper=1.65),
    )
    model.articulation(
        "elbow",
        ArticulationType.REVOLUTE,
        parent=first_link,
        child=second_link,
        origin=Origin(xyz=(0.340, 0.0, 0.028)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=2.0, lower=-0.85, upper=1.25),
    )
    model.articulation(
        "shade_tilt",
        ArticulationType.REVOLUTE,
        parent=second_link,
        child=shade,
        origin=Origin(xyz=(0.305, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=3.0, velocity=1.5, lower=-0.95, upper=0.95),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    clamp = object_model.get_part("clamp")
    first_link = object_model.get_part("first_link")
    second_link = object_model.get_part("second_link")
    shade = object_model.get_part("shade")
    post_swivel = object_model.get_articulation("post_swivel")
    elbow = object_model.get_articulation("elbow")
    shade_tilt = object_model.get_articulation("shade_tilt")

    ctx.check(
        "three requested revolute joints",
        all(j.articulation_type == ArticulationType.REVOLUTE for j in (post_swivel, elbow, shade_tilt)),
        details="post swivel, elbow, and shade tilt must all be revolute joints",
    )
    ctx.expect_contact(first_link, clamp, elem_a="swivel_hub", elem_b="post_cap", contact_tol=0.001, name="swivel hub seats on post cap")
    ctx.expect_overlap(first_link, clamp, axes="xy", elem_a="swivel_hub", elem_b="post_cap", min_overlap=0.020, name="swivel hub centered over post")
    ctx.expect_within(second_link, first_link, axes="xz", inner_elem="elbow_barrel", outer_elem="elbow_fork_0", margin=0.003, name="elbow barrel coaxial with fork")
    ctx.expect_within(shade, second_link, axes="xz", inner_elem="shade_barrel", outer_elem="shade_fork_0", margin=0.004, name="shade barrel coaxial with fork")

    rest_elbow = ctx.part_world_position(second_link)
    with ctx.pose({post_swivel: 0.75}):
        swung_elbow = ctx.part_world_position(second_link)
    ctx.check(
        "post swivel sweeps arm sideways",
        rest_elbow is not None and swung_elbow is not None and swung_elbow[1] > rest_elbow[1] + 0.15,
        details=f"rest={rest_elbow}, swung={swung_elbow}",
    )

    rest_tip = ctx.part_world_position(shade)
    with ctx.pose({elbow: 0.80}):
        raised_tip = ctx.part_world_position(shade)
    ctx.check(
        "elbow raises second link tip",
        rest_tip is not None and raised_tip is not None and raised_tip[2] > rest_tip[2] + 0.12,
        details=f"rest={rest_tip}, raised={raised_tip}",
    )

    rest_shell = ctx.part_element_world_aabb(shade, elem="shade_shell")
    with ctx.pose({shade_tilt: 0.70}):
        tilted_shell = ctx.part_element_world_aabb(shade, elem="shade_shell")
    if rest_shell is not None and tilted_shell is not None:
        rest_center_x = (rest_shell[0][0] + rest_shell[1][0]) * 0.5
        tilted_center_x = (tilted_shell[0][0] + tilted_shell[1][0]) * 0.5
        shade_tilts = abs(tilted_center_x - rest_center_x) > 0.030
    else:
        shade_tilts = False
    ctx.check("shade tilt changes cone aiming direction", shade_tilts, details=f"rest={rest_shell}, tilted={tilted_shell}")

    return ctx.report()


object_model = build_object_model()
