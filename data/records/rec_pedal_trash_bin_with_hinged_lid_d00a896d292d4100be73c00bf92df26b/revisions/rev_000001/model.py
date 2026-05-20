from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeGeometry,
    Material,
    MeshGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _superellipse_loop(
    width: float,
    depth: float,
    *,
    exponent: float = 3.0,
    segments: int = 72,
    front_bulge: float = 0.0,
) -> list[tuple[float, float]]:
    """Rounded D-like plan loop with an extra convex bow on the front side."""
    pts: list[tuple[float, float]] = []
    a = width * 0.5
    b = depth * 0.5
    for i in range(segments):
        t = 2.0 * math.pi * i / segments
        c = math.cos(t)
        s = math.sin(t)
        x = a * math.copysign(abs(c) ** (2.0 / exponent), c)
        y = b * math.copysign(abs(s) ** (2.0 / exponent), s)
        if y < 0.0 and a > 0.0:
            # Designer step bins often have a proud, continuous curved front.
            x_ratio = min(1.0, abs(x) / a)
            y -= front_bulge * (1.0 - x_ratio**1.8)
        return_y = y
        pts.append((x, return_y))
    return pts


def _add_loop(geom: MeshGeometry, loop: list[tuple[float, float]], z: float) -> list[int]:
    return [geom.add_vertex(x, y, z) for x, y in loop]


def _quad(geom: MeshGeometry, a: int, b: int, c: int, d: int) -> None:
    geom.add_face(a, b, c)
    geom.add_face(a, c, d)


def _connect_loops(
    geom: MeshGeometry,
    lower: list[int],
    upper: list[int],
    *,
    reverse: bool = False,
) -> None:
    count = len(lower)
    for i in range(count):
        j = (i + 1) % count
        if reverse:
            _quad(geom, lower[i], upper[i], upper[j], lower[j])
        else:
            _quad(geom, lower[i], lower[j], upper[j], upper[i])


def _cap_loop(geom: MeshGeometry, loop: list[int], z: float, *, reverse: bool = False) -> None:
    xs = [geom.vertices[i][0] for i in loop]
    ys = [geom.vertices[i][1] for i in loop]
    center = geom.add_vertex(sum(xs) / len(xs), sum(ys) / len(ys), z)
    for i in range(len(loop)):
        j = (i + 1) % len(loop)
        if reverse:
            geom.add_face(center, loop[j], loop[i])
        else:
            geom.add_face(center, loop[i], loop[j])


def _body_shell_geometry() -> MeshGeometry:
    """Thin-walled, open-top body with taper and a bowed front face."""
    geom = MeshGeometry()
    segments = 80

    outer_specs = (
        (0.000, 0.320, 0.250, 0.006),
        (0.080, 0.360, 0.285, 0.012),
        (0.360, 0.405, 0.315, 0.018),
        (0.620, 0.430, 0.330, 0.024),
    )
    inner_specs = (
        (0.055, 0.260, 0.190, 0.003),
        (0.120, 0.300, 0.225, 0.007),
        (0.360, 0.345, 0.255, 0.011),
        (0.620, 0.378, 0.276, 0.014),
    )

    outer_loops: list[list[int]] = []
    for z, width, depth, bulge in outer_specs:
        loop = _superellipse_loop(width, depth, segments=segments, front_bulge=bulge)
        outer_loops.append(_add_loop(geom, loop, z))

    inner_loops: list[list[int]] = []
    for z, width, depth, bulge in inner_specs:
        loop = _superellipse_loop(width, depth, segments=segments, front_bulge=bulge)
        inner_loops.append(_add_loop(geom, loop, z))

    for lower, upper in zip(outer_loops[:-1], outer_loops[1:]):
        _connect_loops(geom, lower, upper)
    for lower, upper in zip(inner_loops[:-1], inner_loops[1:]):
        _connect_loops(geom, lower, upper, reverse=True)

    outer_top = outer_loops[-1]
    inner_top = inner_loops[-1]
    for i in range(segments):
        j = (i + 1) % segments
        _quad(geom, outer_top[i], outer_top[j], inner_top[j], inner_top[i])

    _cap_loop(geom, outer_loops[0], outer_specs[0][0], reverse=True)
    _cap_loop(geom, inner_loops[0], inner_specs[0][0], reverse=False)
    return geom


def _lid_profile(width: float, depth: float, *, front_bow: float) -> list[tuple[float, float]]:
    half = width * 0.5
    profile: list[tuple[float, float]] = [(half, 0.0), (-half, 0.0)]
    side_steps = 5
    for k in range(1, side_steps + 1):
        u = k / side_steps
        profile.append((-half, -depth * u))

    front_steps = 18
    for k in range(1, front_steps):
        u = k / front_steps
        x = -half + 2.0 * half * u
        x_ratio = abs(x) / half if half else 0.0
        y = -depth - front_bow * (1.0 - x_ratio**1.8)
        profile.append((x, y))

    for k in range(side_steps, -1, -1):
        u = k / side_steps
        profile.append((half, -depth * u))
    return profile


def _pedal_pad_geometry() -> MeshGeometry:
    geom = ExtrudeGeometry(rounded_rect_profile(0.205, 0.090, 0.025, corner_segments=8), 0.026)
    geom.translate(0.0, -0.086, -0.017)
    return geom


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="designer_split_lid_step_bin")

    warm_white = model.material("warm_white", rgba=(0.86, 0.84, 0.78, 1.0))
    shadow_gray = model.material("shadow_gray", rgba=(0.10, 0.11, 0.11, 1.0))
    satin_lid = model.material("satin_lid", rgba=(0.18, 0.19, 0.20, 1.0))
    pedal_black = model.material("pedal_black", rgba=(0.02, 0.02, 0.018, 1.0))
    hinge_metal = model.material("hinge_metal", rgba=(0.55, 0.56, 0.55, 1.0))

    body = model.part("body")
    body.visual(
        mesh_from_geometry(_body_shell_geometry(), "curved_body_shell"),
        material=warm_white,
        name="body_shell",
    )

    # Integrated dark plinth and hardware pads are fixed to the body so the bin
    # reads as one grounded, non-floating appliance housing.
    plinth = ExtrudeGeometry(
        _superellipse_loop(0.350, 0.255, exponent=3.1, segments=64, front_bulge=0.010),
        0.040,
    )
    plinth.translate(0.0, -0.004, 0.020)
    body.visual(mesh_from_geometry(plinth, "dark_base_plinth"), material=shadow_gray, name="base_plinth")
    body.visual(
        Box((0.410, 0.020, 0.020)),
        origin=Origin(xyz=(0.0, 0.160, 0.623)),
        material=hinge_metal,
        name="rear_hinge_saddle",
    )
    for i, x in enumerate((-0.115, 0.115)):
        body.visual(
            Box((0.036, 0.110, 0.070)),
            origin=Origin(xyz=(x, -0.155, 0.065)),
            material=shadow_gray,
            name=f"pedal_bracket_{i}",
        )

    lid_depth = 0.305
    flap_width = 0.192
    hinge_y = 0.150
    hinge_z = 0.646
    for i, x in enumerate((-0.103, 0.103)):
        flap = model.part(f"lid_flap_{i}")
        flap_panel = ExtrudeGeometry(_lid_profile(flap_width, lid_depth, front_bow=0.014), 0.024)
        flap_panel.translate(0.0, 0.0, -0.005)
        flap.visual(
            mesh_from_geometry(flap_panel, f"lid_flap_{i}_panel"),
            material=satin_lid,
            name="lid_panel",
        )
        flap.visual(
            Cylinder(radius=0.011, length=0.150),
            origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=hinge_metal,
            name="hinge_knuckle",
        )
        flap.visual(
            Box((0.156, 0.018, 0.010)),
            origin=Origin(xyz=(0.0, -0.011, -0.010)),
            material=hinge_metal,
            name="hinge_leaf",
        )
        model.articulation(
            f"body_to_lid_flap_{i}",
            ArticulationType.REVOLUTE,
            parent=body,
            child=flap,
            origin=Origin(xyz=(x, hinge_y, hinge_z)),
            axis=(-1.0, 0.0, 0.0),
            motion_limits=MotionLimits(effort=4.0, velocity=2.5, lower=0.0, upper=1.30),
        )

    pedal = model.part("front_pedal")
    pedal.visual(
        Cylinder(radius=0.011, length=0.230),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=hinge_metal,
        name="pivot_bar",
    )
    pedal.visual(
        Box((0.072, 0.088, 0.018)),
        origin=Origin(xyz=(0.0, -0.043, -0.006)),
        material=pedal_black,
        name="pedal_neck",
    )
    pedal.visual(
        mesh_from_geometry(_pedal_pad_geometry(), "front_foot_pad"),
        material=pedal_black,
        name="foot_pad",
    )
    model.articulation(
        "body_to_front_pedal",
        ArticulationType.REVOLUTE,
        parent=body,
        child=pedal,
        origin=Origin(xyz=(0.0, -0.205, 0.065)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=15.0, velocity=3.0, lower=0.0, upper=0.42),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    pedal = object_model.get_part("front_pedal")
    flaps = [object_model.get_part("lid_flap_0"), object_model.get_part("lid_flap_1")]
    lid_joints = [
        object_model.get_articulation("body_to_lid_flap_0"),
        object_model.get_articulation("body_to_lid_flap_1"),
    ]
    pedal_joint = object_model.get_articulation("body_to_front_pedal")

    ctx.check(
        "three independent revolute controls",
        all(j.articulation_type == ArticulationType.REVOLUTE for j in (*lid_joints, pedal_joint)),
        details="Both lid flaps and the single front pedal should have separate revolute pivots.",
    )
    ctx.check(
        "lid hinge axes are separate rear axes",
        abs(lid_joints[0].origin.xyz[0] - lid_joints[1].origin.xyz[0]) > 0.15
        and abs(lid_joints[0].origin.xyz[1] - lid_joints[1].origin.xyz[1]) < 0.002
        and lid_joints[0].origin.xyz[1] > 0.13
        and lid_joints[1].origin.xyz[1] > 0.13,
        details=f"origins={[j.origin.xyz for j in lid_joints]}",
    )
    ctx.check(
        "pedal uses lower front transverse pivot",
        pedal_joint.axis == (1.0, 0.0, 0.0)
        and pedal_joint.origin.xyz[1] < -0.18
        and pedal_joint.origin.xyz[2] < 0.09,
        details=f"origin={pedal_joint.origin.xyz}, axis={pedal_joint.axis}",
    )
    for bracket_name in ("pedal_bracket_0", "pedal_bracket_1"):
        ctx.allow_overlap(
            body,
            pedal,
            elem_a=bracket_name,
            elem_b="pivot_bar",
            reason="The pedal pivot bar is intentionally captured in the fixed side bracket hole proxy.",
        )
        ctx.expect_overlap(
            body,
            pedal,
            axes="xyz",
            min_overlap=0.004,
            elem_a=bracket_name,
            elem_b="pivot_bar",
            name=f"{bracket_name} captures the pedal pivot bar",
        )

    for i, flap in enumerate(flaps):
        ctx.expect_gap(
            flap,
            body,
            axis="z",
            min_gap=0.004,
            max_gap=0.025,
            positive_elem="lid_panel",
            negative_elem="body_shell",
            name=f"lid_flap_{i} rests just above body rim",
        )
        ctx.expect_overlap(
            flap,
            body,
            axes="xy",
            min_overlap=0.12,
            elem_a="lid_panel",
            elem_b="body_shell",
            name=f"lid_flap_{i} covers its half of the opening",
        )

        rest_aabb = ctx.part_element_world_aabb(flap, elem="lid_panel")
        with ctx.pose({lid_joints[i]: 1.15}):
            raised_aabb = ctx.part_element_world_aabb(flap, elem="lid_panel")
        ctx.check(
            f"lid_flap_{i} rotates upward from rear hinge",
            rest_aabb is not None
            and raised_aabb is not None
            and raised_aabb[1][2] > rest_aabb[1][2] + 0.13,
            details=f"rest={rest_aabb}, raised={raised_aabb}",
        )

    rest_pad = ctx.part_element_world_aabb(pedal, elem="foot_pad")
    with ctx.pose({pedal_joint: 0.38}):
        pressed_pad = ctx.part_element_world_aabb(pedal, elem="foot_pad")
    ctx.check(
        "front pedal depresses about its own pivot",
        rest_pad is not None and pressed_pad is not None and pressed_pad[0][2] < rest_pad[0][2] - 0.020,
        details=f"rest={rest_pad}, pressed={pressed_pad}",
    )

    return ctx.report()


object_model = build_object_model()
