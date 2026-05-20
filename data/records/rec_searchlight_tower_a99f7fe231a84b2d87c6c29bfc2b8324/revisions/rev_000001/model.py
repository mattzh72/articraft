from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    TrunnionYokeGeometry,
    mesh_from_geometry,
)


def _cylinder_between(part, start, end, radius, *, material, name):
    """Add a cylinder whose local Z axis runs from start to end."""
    sx, sy, sz = start
    ex, ey, ez = end
    dx, dy, dz = ex - sx, ey - sy, ez - sz
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    if length <= 0.0:
        raise ValueError("cylinder endpoints must differ")

    ux, uy, uz = dx / length, dy / length, dz / length
    yaw = math.atan2(uy, ux)
    pitch = math.atan2(math.sqrt(ux * ux + uy * uy), uz)
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(
            xyz=((sx + ex) * 0.5, (sy + ey) * 0.5, (sz + ez) * 0.5),
            rpy=(0.0, pitch, yaw),
        ),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="pan_tilt_searchlight_tower")

    concrete = model.material("weathered_concrete", rgba=(0.42, 0.42, 0.39, 1.0))
    safety_yellow = model.material("painted_yellow", rgba=(0.92, 0.70, 0.18, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.05, 0.055, 0.06, 1.0))
    black = model.material("matte_black", rgba=(0.01, 0.012, 0.014, 1.0))
    lens_glass = model.material("pale_blue_glass", rgba=(0.45, 0.72, 0.95, 0.55))
    reflector = model.material("warm_reflector", rgba=(1.0, 0.92, 0.62, 1.0))

    support = model.part("support")
    support.visual(
        Box((1.20, 1.00, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=concrete,
        name="concrete_pad",
    )
    support.visual(
        Box((0.58, 0.46, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 0.12)),
        material=dark_steel,
        name="base_plinth",
    )
    support.visual(
        Cylinder(radius=0.055, length=1.30),
        origin=Origin(xyz=(0.0, 0.0, 0.81)),
        material=dark_steel,
        name="central_mast",
    )
    support.visual(
        Cylinder(radius=0.095, length=0.22),
        origin=Origin(xyz=(0.0, 0.0, 0.27)),
        material=dark_steel,
        name="lower_sleeve",
    )
    support.visual(
        Box((0.66, 0.56, 0.07)),
        origin=Origin(xyz=(0.0, 0.0, 1.495)),
        material=dark_steel,
        name="top_platform",
    )
    support.visual(
        Cylinder(radius=0.175, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 1.5525)),
        material=dark_steel,
        name="fixed_bearing",
    )

    leg_anchors = (
        ((0.25, 0.19, 0.145), (0.27, 0.22, 1.475)),
        ((-0.25, 0.19, 0.145), (-0.27, 0.22, 1.475)),
        ((0.25, -0.19, 0.145), (0.27, -0.22, 1.475)),
        ((-0.25, -0.19, 0.145), (-0.27, -0.22, 1.475)),
    )
    for index, (start, end) in enumerate(leg_anchors):
        _cylinder_between(
            support,
            start,
            end,
            0.018,
            material=safety_yellow,
            name=f"tower_leg_{index}",
        )

    # Cross-braces make the fixed tower read as a broad support instead of a
    # narrow rotating pole.  Their ends are slightly buried in the plinth and
    # top platform, like welded tube stock.
    brace_pairs = (
        ((-0.25, 0.205, 0.22), (0.27, 0.215, 1.34)),
        ((0.25, 0.205, 0.22), (-0.27, 0.215, 1.34)),
        ((-0.25, -0.205, 0.22), (0.27, -0.215, 1.34)),
        ((0.25, -0.205, 0.22), (-0.27, -0.215, 1.34)),
    )
    for index, (start, end) in enumerate(brace_pairs):
        _cylinder_between(
            support,
            start,
            end,
            0.011,
            material=safety_yellow,
            name=f"cross_brace_{index}",
        )

    yoke = model.part("yoke")
    yoke.visual(
        mesh_from_geometry(
            TrunnionYokeGeometry(
                (0.50, 0.22, 0.42),
                span_width=0.32,
                trunnion_diameter=0.090,
                trunnion_center_z=0.28,
                base_thickness=0.060,
                corner_radius=0.018,
                center=False,
            ),
            "trunnion_yoke",
        ),
        origin=Origin(),
        material=safety_yellow,
        name="yoke_frame",
    )
    yoke.visual(
        Cylinder(radius=0.095, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.030)),
        material=black,
        name="turntable_collar",
    )
    yoke.visual(
        Cylinder(radius=0.055, length=0.075),
        origin=Origin(xyz=(0.0, 0.0, 0.0975)),
        material=dark_steel,
        name="pan_post",
    )

    lamp = model.part("lamp")
    lamp.visual(
        Cylinder(radius=0.130, length=0.320),
        origin=Origin(xyz=(0.0, 0.045, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=black,
        name="lamp_barrel",
    )
    lamp.visual(
        Cylinder(radius=0.150, length=0.050),
        origin=Origin(xyz=(0.0, 0.230, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="front_bezel",
    )
    lamp.visual(
        Cylinder(radius=0.115, length=0.016),
        origin=Origin(xyz=(0.0, 0.263, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=lens_glass,
        name="front_lens",
    )
    lamp.visual(
        Cylinder(radius=0.095, length=0.045),
        origin=Origin(xyz=(0.0, -0.1375, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="rear_cap",
    )
    lamp.visual(
        Cylinder(radius=0.075, length=0.010),
        origin=Origin(xyz=(0.0, 0.254, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=reflector,
        name="inner_reflector",
    )
    lamp.visual(
        Cylinder(radius=0.0455, length=0.470),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="tilt_trunnion",
    )
    lamp.visual(
        Cylinder(radius=0.057, length=0.016),
        origin=Origin(xyz=(-0.145, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="trunnion_boss_0",
    )
    lamp.visual(
        Cylinder(radius=0.057, length=0.016),
        origin=Origin(xyz=(0.145, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black,
        name="trunnion_boss_1",
    )

    pan = model.articulation(
        "pan",
        ArticulationType.REVOLUTE,
        parent=support,
        child=yoke,
        origin=Origin(xyz=(0.0, 0.0, 1.575)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=40.0, velocity=1.0, lower=-math.pi, upper=math.pi),
    )
    pan.meta["qc_samples"] = [-1.2, 0.0, 1.2]

    tilt = model.articulation(
        "tilt",
        ArticulationType.REVOLUTE,
        parent=yoke,
        child=lamp,
        origin=Origin(xyz=(0.0, 0.0, 0.28)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=0.8, lower=-0.45, upper=0.95),
    )
    tilt.meta["qc_samples"] = [-0.35, 0.0, 0.75]

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    support = object_model.get_part("support")
    yoke = object_model.get_part("yoke")
    lamp = object_model.get_part("lamp")
    pan = object_model.get_articulation("pan")
    tilt = object_model.get_articulation("tilt")

    def _center_z(aabb):
        return (aabb[0][2] + aabb[1][2]) * 0.5

    def _center_xy(aabb):
        return ((aabb[0][0] + aabb[1][0]) * 0.5, (aabb[0][1] + aabb[1][1]) * 0.5)

    def _dims_xy(aabb):
        return (aabb[1][0] - aabb[0][0], aabb[1][1] - aabb[0][1])

    ctx.allow_overlap(
        lamp,
        yoke,
        elem_a="tilt_trunnion",
        elem_b="yoke_frame",
        reason=(
            "The lamp's horizontal trunnion is intentionally captured in the "
            "yoke cheek bushings; the yoke mesh represents the bearing bores as "
            "a local solid sleeve for robustness."
        ),
    )

    ctx.check(
        "pan tilt joint count",
        len(object_model.articulations) == 2,
        details=f"articulations={[joint.name for joint in object_model.articulations]}",
    )
    ctx.check(
        "pan axis vertical",
        tuple(round(v, 6) for v in pan.axis) == (0.0, 0.0, 1.0),
        details=f"axis={pan.axis}",
    )
    ctx.check(
        "tilt axis horizontal",
        tuple(round(v, 6) for v in tilt.axis) == (1.0, 0.0, 0.0),
        details=f"axis={tilt.axis}",
    )

    ctx.expect_contact(
        yoke,
        support,
        elem_a="turntable_collar",
        elem_b="fixed_bearing",
        contact_tol=0.001,
        name="pan collar sits on fixed bearing",
    )
    ctx.expect_within(
        lamp,
        yoke,
        axes="xz",
        inner_elem="tilt_trunnion",
        outer_elem="yoke_frame",
        margin=0.006,
        name="trunnion is retained by the yoke cheeks",
    )
    ctx.expect_overlap(
        lamp,
        yoke,
        axes="x",
        elem_a="tilt_trunnion",
        elem_b="yoke_frame",
        min_overlap=0.42,
        name="trunnion spans both yoke arms",
    )
    pad_aabb = ctx.part_element_world_aabb(support, elem="concrete_pad")
    collar_aabb = ctx.part_element_world_aabb(yoke, elem="turntable_collar")
    ctx.check(
        "fixed support broader than pan member",
        pad_aabb is not None
        and collar_aabb is not None
        and (pad_aabb[1][0] - pad_aabb[0][0]) > 4.0 * (collar_aabb[1][0] - collar_aabb[0][0])
        and (pad_aabb[1][1] - pad_aabb[0][1]) > 4.0 * (collar_aabb[1][1] - collar_aabb[0][1]),
        details=f"pad_aabb={pad_aabb}, collar_aabb={collar_aabb}",
    )

    rest_lens = ctx.part_element_world_aabb(lamp, elem="front_lens")
    with ctx.pose({tilt: 0.70}):
        raised_lens = ctx.part_element_world_aabb(lamp, elem="front_lens")
    ctx.check(
        "positive tilt raises beam",
        rest_lens is not None and raised_lens is not None and _center_z(raised_lens) > _center_z(rest_lens) + 0.08,
        details=f"rest={rest_lens}, raised={raised_lens}",
    )

    rest_yoke = ctx.part_element_world_aabb(yoke, elem="yoke_frame")
    with ctx.pose({pan: 1.0}):
        panned_yoke = ctx.part_element_world_aabb(yoke, elem="yoke_frame")
    if rest_yoke is not None and panned_yoke is not None:
        rest_xy = _center_xy(rest_yoke)
        panned_xy = _center_xy(panned_yoke)
        moved = abs(rest_xy[0] - panned_xy[0]) + abs(rest_xy[1] - panned_xy[1])
        rest_dims = _dims_xy(rest_yoke)
        panned_dims = _dims_xy(panned_yoke)
        reoriented = abs(rest_dims[0] - panned_dims[0]) + abs(rest_dims[1] - panned_dims[1])
    else:
        moved = 0.0
        reoriented = 0.0
    ctx.check(
        "pan rotates yoke about mast",
        moved < 0.01 and reoriented > 0.20,
        details=f"rest={rest_yoke}, panned={panned_yoke}, center_shift={moved}, reoriented={reoriented}",
    )

    return ctx.report()


object_model = build_object_model()
