from __future__ import annotations

import math

import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    MotionProperties,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _mat(model: ArticulatedObject, name: str, rgba) -> Material:
    return model.material(name, rgba=rgba)


def _cyl_x(radius: float, length: float) -> tuple[Cylinder, Origin]:
    return Cylinder(radius=radius, length=length), Origin(rpy=(0.0, math.pi / 2.0, 0.0))


def _cyl_y(radius: float, length: float) -> tuple[Cylinder, Origin]:
    return Cylinder(radius=radius, length=length), Origin(rpy=(-math.pi / 2.0, 0.0, 0.0))


def _annular_bearing(outer_radius: float, inner_radius: float, length: float, name: str):
    """Centered hard-surface bearing race with a true through bore."""
    body = (
        cq.Workplane("XY")
        .circle(outer_radius)
        .circle(inner_radius)
        .extrude(length)
        .translate((0.0, 0.0, -length / 2.0))
    )
    # Eight small axial bolt counterbores give the housing a machined flange
    # reading without needing separate floating screw geometry.
    for i in range(8):
        a = i * math.tau / 8.0
        x = math.cos(a) * (outer_radius * 0.72)
        y = math.sin(a) * (outer_radius * 0.72)
        cutter = cq.Workplane("XY").center(x, y).circle(0.005).extrude(length * 1.4).translate(
            (0.0, 0.0, -length * 0.7)
        )
        body = body.cut(cutter)
    return mesh_from_cadquery(body, name, tolerance=0.0008, angular_tolerance=0.08)


def _spoked_rotor(radius: float, bore_radius: float, thickness: float, name: str):
    """One-piece rotating flange with lightening pockets and a central bore."""
    body = (
        cq.Workplane("XY")
        .circle(radius)
        .circle(bore_radius)
        .extrude(thickness)
        .translate((0.0, 0.0, -thickness / 2.0))
    )
    for i in range(6):
        a = i * math.tau / 6.0
        pocket = (
            cq.Workplane("XY")
            .center(math.cos(a) * radius * 0.48, math.sin(a) * radius * 0.48)
            .rect(radius * 0.36, radius * 0.13)
            .extrude(thickness * 1.4)
            .translate((0.0, 0.0, -thickness * 0.7))
            .rotate((0.0, 0.0, 0.0), (0.0, 0.0, 1.0), math.degrees(a))
        )
        body = body.cut(pocket)
    return mesh_from_cadquery(body, name, tolerance=0.0008, angular_tolerance=0.08)


def _rod_between(part, name: str, start, end, radius: float, material):
    sx, sy, sz = start
    ex, ey, ez = end
    dx, dy, dz = ex - sx, ey - sy, ez - sz
    length = math.sqrt(dx * dx + dy * dy + dz * dz)
    ux, uy, uz = dx / length, dy / length, dz / length
    pitch = math.acos(max(-1.0, min(1.0, uz)))
    yaw = math.atan2(uy, ux)
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(
            xyz=((sx + ex) / 2.0, (sy + ey) / 2.0, (sz + ez) / 2.0),
            rpy=(0.0, pitch, yaw),
        ),
        material=material,
        name=name,
    )


def _box(part, name: str, size, xyz, material, rpy=(0.0, 0.0, 0.0)):
    part.visual(Box(size), origin=Origin(xyz=xyz, rpy=rpy), material=material, name=name)


def _cylinder(part, name: str, radius: float, length: float, xyz, material, rpy=(0.0, 0.0, 0.0)):
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz, rpy=rpy),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rotary_branching_tree_study")

    dark_steel = _mat(model, "dark_blasted_steel", (0.12, 0.13, 0.13, 1.0))
    gunmetal = _mat(model, "gunmetal_casting", (0.23, 0.25, 0.25, 1.0))
    machined = _mat(model, "machined_bearing_steel", (0.58, 0.60, 0.58, 1.0))
    cover_mat = _mat(model, "removable_blackened_cover", (0.06, 0.065, 0.07, 1.0))
    stop_mat = _mat(model, "oxide_limit_stop", (0.42, 0.30, 0.13, 1.0))
    arm_mat = _mat(model, "satin_rigid_arm", (0.34, 0.38, 0.40, 1.0))
    fastener_mat = _mat(model, "dark_socket_fasteners", (0.03, 0.035, 0.04, 1.0))

    backbone = model.part("backbone")

    # Central fabricated spine and base: one root part with a visibly triangulated,
    # serviceable mechanical structure rather than a decorative enclosure.
    _box(backbone, "spine_tube", (0.12, 0.10, 1.22), (0.0, 0.0, 0.61), gunmetal)
    _box(backbone, "base_plate", (0.50, 0.36, 0.035), (0.0, 0.0, 0.0175), dark_steel)
    _box(backbone, "base_cross_rib", (0.42, 0.055, 0.065), (0.0, 0.0, 0.07), gunmetal)
    _box(backbone, "base_long_rib", (0.055, 0.30, 0.065), (0.0, 0.0, 0.07), gunmetal)
    _box(backbone, "top_cap_rail_0", (0.24, 0.050, 0.025), (0.0, 0.068, 1.2325), dark_steel)
    _box(backbone, "top_cap_rail_1", (0.24, 0.050, 0.025), (0.0, -0.068, 1.2325), dark_steel)
    _box(backbone, "rear_access_cover", (0.085, 0.008, 0.34), (0.0, -0.054, 0.66), cover_mat)
    _box(backbone, "front_access_cover", (0.086, 0.008, 0.24), (0.0, 0.054, 0.29), cover_mat)

    # Fasteners are slightly proud and locally embedded into their removable covers
    # so they read as bolted-on panels while remaining one supported root part.
    for ix, x in enumerate((-0.030, 0.030)):
        for iz, z in enumerate((0.53, 0.79)):
            _cylinder(
                backbone,
                f"rear_cover_screw_{ix}_{iz}",
                0.006,
                0.006,
                (x, -0.058, z),
                fastener_mat,
                rpy=(math.pi / 2.0, 0.0, 0.0),
            )
    for ix, x in enumerate((-0.030, 0.030)):
        for iz, z in enumerate((0.20, 0.38)):
            _cylinder(
                backbone,
                f"front_cover_screw_{ix}_{iz}",
                0.0055,
                0.006,
                (x, 0.058, z),
                fastener_mat,
                rpy=(math.pi / 2.0, 0.0, 0.0),
            )

    # Three offset bearing stations are connected by real brackets and a diagonal
    # tube backbone.  Their axes are deliberately separated: Z, Y, and X.
    upper_origin = (0.0, 0.0, 1.26)
    middle_origin = (0.0, 0.155, 0.76)
    lower_origin = (-0.155, 0.0, 0.38)

    # Upper horizontal bearing stack.
    backbone.visual(
        _annular_bearing(0.095, 0.034, 0.060, "upper_bearing_race"),
        origin=Origin(xyz=upper_origin),
        material=machined,
        name="upper_bearing_race",
    )
    _box(backbone, "upper_bridge_plate", (0.22, 0.16, 0.030), (0.0, 0.0, 1.205), dark_steel)
    _box(backbone, "upper_yoke_web", (0.055, 0.155, 0.105), (0.0, 0.0, 1.165), gunmetal)
    _box(backbone, "upper_stop_plus", (0.024, 0.026, 0.035), (0.030, 0.088, 1.305), stop_mat)
    _box(backbone, "upper_stop_minus", (0.024, 0.026, 0.035), (-0.030, 0.088, 1.305), stop_mat)

    # Middle front bearing, axis along world Y.
    backbone.visual(
        _annular_bearing(0.083, 0.031, 0.070, "middle_bearing_race"),
        origin=Origin(xyz=middle_origin, rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=machined,
        name="middle_bearing_race",
    )
    _box(backbone, "middle_side_cheek_0", (0.035, 0.120, 0.070), (0.055, 0.087, 0.76), dark_steel)
    _box(backbone, "middle_side_cheek_1", (0.035, 0.120, 0.070), (-0.055, 0.087, 0.76), dark_steel)
    _box(backbone, "middle_top_web", (0.10, 0.125, 0.030), (0.0, 0.087, 0.83), gunmetal)
    _box(backbone, "middle_bottom_web", (0.10, 0.125, 0.030), (0.0, 0.087, 0.69), gunmetal)
    _box(backbone, "middle_stop_top", (0.052, 0.026, 0.020), (0.055, 0.192, 0.823), stop_mat)
    _box(backbone, "middle_stop_bottom", (0.052, 0.026, 0.020), (0.055, 0.192, 0.697), stop_mat)

    # Lower side bearing, axis along world X.
    backbone.visual(
        _annular_bearing(0.078, 0.029, 0.070, "lower_bearing_race"),
        origin=Origin(xyz=lower_origin, rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined,
        name="lower_bearing_race",
    )
    _box(backbone, "lower_upper_saddle", (0.125, 0.13, 0.022), (-0.087, 0.0, 0.424), dark_steel)
    _box(backbone, "lower_lower_saddle", (0.125, 0.13, 0.022), (-0.087, 0.0, 0.336), dark_steel)
    _box(backbone, "lower_front_web", (0.125, 0.030, 0.095), (-0.087, 0.055, 0.38), gunmetal)
    _box(backbone, "lower_rear_web", (0.125, 0.030, 0.095), (-0.087, -0.055, 0.38), gunmetal)
    _box(backbone, "lower_stop_front", (0.025, 0.048, 0.022), (-0.165, 0.060, 0.425), stop_mat)
    _box(backbone, "lower_stop_rear", (0.025, 0.048, 0.022), (-0.165, -0.030, 0.425), stop_mat)

    # Triangulated external spine rods tie the bearing stations and base into
    # a clear load path.  The small overlaps at their endpoints are same-part
    # welded/sleeved joints.
    _rod_between(backbone, "front_upper_diag", (0.052, 0.060, 0.16), (0.052, 0.060, 1.17), 0.010, dark_steel)
    _rod_between(backbone, "rear_upper_diag", (-0.052, -0.060, 0.16), (-0.052, -0.060, 1.17), 0.010, dark_steel)
    _rod_between(backbone, "front_cross_diag", (-0.055, 0.060, 0.20), (0.055, 0.060, 1.08), 0.008, dark_steel)
    _rod_between(backbone, "rear_cross_diag", (0.055, -0.060, 0.20), (-0.055, -0.060, 1.08), 0.008, dark_steel)
    _rod_between(backbone, "middle_to_upper_tie", (0.045, 0.13, 0.81), (0.070, 0.04, 1.22), 0.007, dark_steel)
    _rod_between(backbone, "lower_to_middle_tie", (-0.13, 0.045, 0.43), (-0.030, 0.135, 0.72), 0.007, dark_steel)

    # Upper branch rotates in a horizontal envelope about the vertical bearing.
    upper_branch = model.part("upper_branch")
    _cylinder(upper_branch, "upper_shaft", 0.026, 0.088, (0.0, 0.0, 0.0), machined)
    upper_branch.visual(
        _spoked_rotor(0.069, 0.017, 0.020, "upper_rotor_flange"),
        origin=Origin(xyz=(0.0, 0.0, 0.054)),
        material=machined,
        name="upper_rotor_flange",
    )
    _box(upper_branch, "upper_arm_box", (0.62, 0.062, 0.045), (0.36, 0.0, 0.074), arm_mat)
    _box(upper_branch, "upper_arm_cap", (0.16, 0.088, 0.016), (0.69, 0.0, 0.074), cover_mat)
    _box(upper_branch, "upper_top_gusset", (0.23, 0.020, 0.082), (0.16, 0.041, 0.078), dark_steel)
    _box(upper_branch, "upper_bottom_gusset", (0.23, 0.020, 0.082), (0.16, -0.041, 0.078), dark_steel)
    _box(upper_branch, "upper_end_pad", (0.034, 0.085, 0.070), (0.69, 0.0, 0.074), gunmetal)
    for i, x in enumerate((0.14, 0.28, 0.50, 0.64)):
        _cylinder(upper_branch, f"upper_cap_screw_{i}", 0.0048, 0.010, (x, 0.0, 0.0865), fastener_mat)

    # Middle branch: independent pitch-like rotary axis, offset forward of the spine.
    middle_branch = model.part("middle_branch")
    _cylinder(
        middle_branch,
        "middle_shaft",
        0.024,
        0.100,
        (0.0, 0.0, 0.0),
        machined,
        rpy=(-math.pi / 2.0, 0.0, 0.0),
    )
    middle_branch.visual(
        _spoked_rotor(0.062, 0.016, 0.018, "middle_rotor_flange"),
        origin=Origin(xyz=(0.0, 0.056, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=machined,
        name="middle_rotor_flange",
    )
    _box(middle_branch, "middle_arm_box", (0.58, 0.050, 0.055), (0.34, 0.078, 0.0), arm_mat)
    _box(middle_branch, "middle_top_plate", (0.25, 0.018, 0.085), (0.17, 0.083, 0.045), dark_steel)
    _box(middle_branch, "middle_bottom_plate", (0.25, 0.018, 0.085), (0.17, 0.083, -0.045), dark_steel)
    _box(middle_branch, "middle_access_cover", (0.19, 0.010, 0.070), (0.47, 0.108, 0.0), cover_mat)
    _box(middle_branch, "middle_end_pad", (0.034, 0.078, 0.080), (0.64, 0.078, 0.0), gunmetal)
    for i, x in enumerate((0.42, 0.52)):
        for j, z in enumerate((-0.023, 0.023)):
            _cylinder(
                middle_branch,
                f"middle_cover_screw_{i}_{j}",
                0.0045,
                0.008,
                (x, 0.113, z),
                fastener_mat,
                rpy=(-math.pi / 2.0, 0.0, 0.0),
            )

    # Lower branch: separate side axis, with its own guide plates and stop window.
    lower_branch = model.part("lower_branch")
    _cylinder(
        lower_branch,
        "lower_shaft",
        0.023,
        0.096,
        (0.0, 0.0, 0.0),
        machined,
        rpy=(0.0, math.pi / 2.0, 0.0),
    )
    lower_branch.visual(
        _spoked_rotor(0.058, 0.015, 0.018, "lower_rotor_flange"),
        origin=Origin(xyz=(-0.044, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=machined,
        name="lower_rotor_flange",
    )
    _box(lower_branch, "lower_arm_box", (0.052, 0.50, 0.052), (-0.078, -0.305, 0.0), arm_mat)
    _box(lower_branch, "lower_side_plate_a", (0.084, 0.21, 0.016), (-0.083, -0.16, 0.028), dark_steel)
    _box(lower_branch, "lower_side_plate_b", (0.084, 0.21, 0.016), (-0.083, -0.16, -0.028), dark_steel)
    _box(lower_branch, "lower_access_cover", (0.012, 0.17, 0.064), (-0.108, -0.39, 0.0), cover_mat)
    _box(lower_branch, "lower_end_pad", (0.078, 0.034, 0.074), (-0.078, -0.570, 0.0), gunmetal)
    for i, y in enumerate((-0.34, -0.44)):
        for j, z in enumerate((-0.021, 0.021)):
            _cylinder(
                lower_branch,
                f"lower_cover_screw_{i}_{j}",
                0.0044,
                0.008,
                (-0.114, y, z),
                fastener_mat,
                rpy=(0.0, math.pi / 2.0, 0.0),
            )

    model.articulation(
        "upper_axis",
        ArticulationType.REVOLUTE,
        parent=backbone,
        child=upper_branch,
        origin=Origin(xyz=upper_origin),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.2, lower=-0.72, upper=0.72),
        motion_properties=MotionProperties(damping=0.8, friction=0.15),
    )
    model.articulation(
        "middle_axis",
        ArticulationType.REVOLUTE,
        parent=backbone,
        child=middle_branch,
        origin=Origin(xyz=middle_origin),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=95.0, velocity=1.0, lower=-0.58, upper=0.62),
        motion_properties=MotionProperties(damping=0.7, friction=0.12),
    )
    model.articulation(
        "lower_axis",
        ArticulationType.REVOLUTE,
        parent=backbone,
        child=lower_branch,
        origin=Origin(xyz=lower_origin),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=85.0, velocity=1.0, lower=-0.68, upper=0.55),
        motion_properties=MotionProperties(damping=0.7, friction=0.12),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    backbone = object_model.get_part("backbone")
    upper = object_model.get_part("upper_branch")
    middle = object_model.get_part("middle_branch")
    lower = object_model.get_part("lower_branch")
    upper_axis = object_model.get_articulation("upper_axis")
    middle_axis = object_model.get_articulation("middle_axis")
    lower_axis = object_model.get_articulation("lower_axis")

    # Bearings have true bores and keep the three rotating envelopes retained
    # at their supported axes without relying on broad collision allowances.
    ctx.expect_within(
        upper,
        backbone,
        axes="xy",
        inner_elem="upper_shaft",
        outer_elem="upper_bearing_race",
        margin=0.004,
        name="upper shaft is centered in upper bearing bore",
    )
    ctx.expect_within(
        middle,
        backbone,
        axes="xz",
        inner_elem="middle_shaft",
        outer_elem="middle_bearing_race",
        margin=0.004,
        name="middle shaft is centered in middle bearing bore",
    )
    ctx.expect_within(
        lower,
        backbone,
        axes="yz",
        inner_elem="lower_shaft",
        outer_elem="lower_bearing_race",
        margin=0.004,
        name="lower shaft is centered in lower bearing bore",
    )
    ctx.expect_overlap(
        upper,
        backbone,
        axes="z",
        elem_a="upper_shaft",
        elem_b="upper_bearing_race",
        min_overlap=0.045,
        name="upper bearing retains vertical shaft length",
    )
    ctx.expect_overlap(
        middle,
        backbone,
        axes="y",
        elem_a="middle_shaft",
        elem_b="middle_bearing_race",
        min_overlap=0.055,
        name="middle bearing retains forward shaft length",
    )
    ctx.expect_overlap(
        lower,
        backbone,
        axes="x",
        elem_a="lower_shaft",
        elem_b="lower_bearing_race",
        min_overlap=0.055,
        name="lower bearing retains side shaft length",
    )

    # Decisive pose checks prove independent rotary motion on three separated
    # axes.  The part origins remain bearing-centered while the end pads sweep.
    rest_upper = ctx.part_element_world_aabb(upper, elem="upper_end_pad")
    rest_middle = ctx.part_element_world_aabb(middle, elem="middle_end_pad")
    rest_lower = ctx.part_element_world_aabb(lower, elem="lower_end_pad")
    with ctx.pose({upper_axis: 0.60}):
        moved_upper = ctx.part_element_world_aabb(upper, elem="upper_end_pad")
    with ctx.pose({middle_axis: 0.50}):
        moved_middle = ctx.part_element_world_aabb(middle, elem="middle_end_pad")
    with ctx.pose({lower_axis: -0.50}):
        moved_lower = ctx.part_element_world_aabb(lower, elem="lower_end_pad")

    def _center(aabb):
        if aabb is None:
            return None
        lo, hi = aabb
        return tuple((lo[i] + hi[i]) / 2.0 for i in range(3))

    cu0, cu1 = _center(rest_upper), _center(moved_upper)
    cm0, cm1 = _center(rest_middle), _center(moved_middle)
    cl0, cl1 = _center(rest_lower), _center(moved_lower)
    ctx.check(
        "upper branch sweeps in horizontal plane",
        cu0 is not None and cu1 is not None and abs(cu1[1] - cu0[1]) > 0.12 and abs(cu1[2] - cu0[2]) < 0.01,
        details=f"rest={cu0}, moved={cu1}",
    )
    ctx.check(
        "middle branch rotates about forward axis",
        cm0 is not None and cm1 is not None and abs(cm1[2] - cm0[2]) > 0.10 and abs(cm1[1] - cm0[1]) < 0.01,
        details=f"rest={cm0}, moved={cm1}",
    )
    ctx.check(
        "lower branch rotates about side axis",
        cl0 is not None and cl1 is not None and abs(cl1[2] - cl0[2]) > 0.08 and abs(cl1[0] - cl0[0]) < 0.01,
        details=f"rest={cl0}, moved={cl1}",
    )

    return ctx.report()


object_model = build_object_model()
