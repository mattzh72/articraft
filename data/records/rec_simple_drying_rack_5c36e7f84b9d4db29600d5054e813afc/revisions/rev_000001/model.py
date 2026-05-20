from __future__ import annotations

from math import atan2, sqrt

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
)


def _cylinder_between(part, p0, p1, radius, *, material, name):
    """Add a cylinder whose local +Z axis is aligned from p0 to p1."""
    x0, y0, z0 = p0
    x1, y1, z1 = p1
    dx, dy, dz = x1 - x0, y1 - y0, z1 - z0
    length = sqrt(dx * dx + dy * dy + dz * dz)
    yaw = atan2(dy, dx)
    pitch = atan2(sqrt(dx * dx + dy * dy), dz)
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(
            xyz=((x0 + x1) * 0.5, (y0 + y1) * 0.5, (z0 + z1) * 0.5),
            rpy=(0.0, pitch, yaw),
        ),
        material=material,
        name=name,
    )


def _box(part, xyz, size, *, material, name, rpy=(0.0, 0.0, 0.0)):
    part.visual(Box(size), origin=Origin(xyz=xyz, rpy=rpy), material=material, name=name)


def _sphere(part, xyz, radius, *, material, name):
    part.visual(Sphere(radius=radius), origin=Origin(xyz=xyz), material=material, name=name)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="premium_simple_drying_rack")

    matte_graphite = model.material("matte_graphite", rgba=(0.10, 0.11, 0.12, 1.0))
    satin_aluminum = model.material("satin_aluminum", rgba=(0.72, 0.74, 0.73, 1.0))
    warm_hardware = model.material("warm_brushed_hardware", rgba=(0.62, 0.55, 0.44, 1.0))
    soft_rubber = model.material("soft_dark_rubber", rgba=(0.025, 0.026, 0.028, 1.0))
    seam_plastic = model.material("matte_warm_plastic", rgba=(0.82, 0.80, 0.74, 1.0))

    central = model.part("central_frame")

    top_z = 0.82
    half_len = 0.55
    half_width = 0.27

    # Main fixed drying deck: a premium satin/graphite tubular rectangle with
    # individual rails rather than a solid placeholder.
    _cylinder_between(
        central,
        (-half_len, -half_width, top_z),
        (half_len, -half_width, top_z),
        0.014,
        material=matte_graphite,
        name="near_side_rail",
    )
    _cylinder_between(
        central,
        (-half_len, half_width, top_z),
        (half_len, half_width, top_z),
        0.014,
        material=matte_graphite,
        name="far_side_rail",
    )
    for x, label in ((-half_len, "end_0"), (half_len, "end_1")):
        _cylinder_between(
            central,
            (x, -half_width, top_z),
            (x, half_width, top_z),
            0.011,
            material=matte_graphite,
            name=f"{label}_rail",
        )

    for i, x in enumerate((-0.42, -0.28, -0.14, 0.0, 0.14, 0.28, 0.42)):
        _cylinder_between(
            central,
            (x, -0.257, top_z),
            (x, 0.257, top_z),
            0.0065,
            material=satin_aluminum,
            name=f"cross_rail_{i}",
        )
        for sign, side in ((-1, "near"), (1, "far")):
            _box(
                central,
                (x, sign * half_width, top_z),
                (0.026, 0.038, 0.014),
                material=seam_plastic,
                name=f"{side}_rail_saddle_{i}",
            )

    # Interleaved side hinge barrels and low-profile straps for the two
    # fold-out wings.  The mating wing barrels occupy the deliberate gaps.
    root_hinge_segments = [(-0.54, -0.42), (-0.22, -0.10), (0.10, 0.22), (0.42, 0.54)]
    for sign, side in ((1, "wing_0"), (-1, "wing_1")):
        hinge_y = sign * 0.305
        for i, (x0, x1) in enumerate(root_hinge_segments):
            _cylinder_between(
                central,
                (x0, hinge_y, top_z),
                (x1, hinge_y, top_z),
                0.015,
                material=warm_hardware,
                name=f"{side}_root_barrel_{i}",
            )
            _box(
                central,
                ((x0 + x1) * 0.5, sign * 0.278, top_z),
                (x1 - x0 + 0.012, 0.026, 0.010),
                material=warm_hardware,
                name=f"{side}_hinge_strap_{i}",
            )
        for x, end in ((-0.558, "end_0"), (0.558, "end_1")):
            _cylinder_between(
                central,
                (x - 0.020, hinge_y, top_z),
                (x + 0.020, hinge_y, top_z),
                0.019,
                material=warm_hardware,
                name=f"{side}_hinge_washer_{end}",
            )

    # Folding leg hinge clevises and a restrained under-spine.  The clevises
    # leave clean daylight around the moving sleeves rather than hiding them.
    for sign, side in ((-1, "near"), (1, "far")):
        _cylinder_between(
            central,
            (-0.46, sign * 0.238, 0.748),
            (0.46, sign * 0.238, 0.748),
            0.008,
            material=matte_graphite,
            name=f"{side}_under_spine",
        )
    for x_sign, idx in ((-1, 0), (1, 1)):
        hinge_x = x_sign * 0.38
        _cylinder_between(
            central,
            (hinge_x, -0.245, 0.748),
            (hinge_x, 0.245, 0.748),
            0.009,
            material=matte_graphite,
            name=f"leg_{idx}_upper_axle",
        )
        for sign, side in ((-1, "near"), (1, "far")):
            _box(
                central,
                (hinge_x, sign * 0.238, 0.775),
                (0.040, 0.038, 0.090),
                material=matte_graphite,
                name=f"leg_{idx}_{side}_hanger",
            )
            _box(
                central,
                (hinge_x, sign * 0.221, 0.748),
                (0.050, 0.018, 0.044),
                material=warm_hardware,
                name=f"leg_{idx}_{side}_clevis",
            )

    # Small non-moving keeper blocks on the top frame show where folded wings
    # latch without adding visual clutter.
    for x, label in ((-0.532, "end_0"), (0.532, "end_1")):
        _box(
            central,
            (x, 0.0, top_z + 0.015),
            (0.045, 0.080, 0.012),
            material=seam_plastic,
            name=f"low_profile_latch_{label}",
        )

    def add_wing(part, *, sign: int, prefix: str):
        wing_hinge_segments = [(-0.42, -0.22), (-0.10, 0.10), (0.22, 0.42)]
        for i, (x0, x1) in enumerate(wing_hinge_segments):
            _cylinder_between(
                part,
                (x0, 0.0, 0.0),
                (x1, 0.0, 0.0),
                0.014,
                material=warm_hardware,
                name=f"{prefix}_barrel_{i}",
            )
            _box(
                part,
                ((x0 + x1) * 0.5, sign * 0.028, 0.0),
                (x1 - x0 + 0.010, 0.028, 0.010),
                material=warm_hardware,
                name=f"{prefix}_barrel_web_{i}",
            )

        _cylinder_between(
            part,
            (-0.50, sign * 0.048, 0.0),
            (0.50, sign * 0.048, 0.0),
            0.009,
            material=matte_graphite,
            name="inner_rail",
        )
        _cylinder_between(
            part,
            (-0.50, sign * 0.405, 0.0),
            (0.50, sign * 0.405, 0.0),
            0.011,
            material=matte_graphite,
            name="outer_rail",
        )
        for x, label in ((-0.50, "end_0"), (0.50, "end_1")):
            _cylinder_between(
                part,
                (x, sign * 0.048, 0.0),
                (x, sign * 0.405, 0.0),
                0.009,
                material=matte_graphite,
                name=f"{label}_rim",
            )
            _sphere(
                part,
                (x, sign * 0.405, 0.0),
                0.019,
                material=soft_rubber,
                name=f"{label}_soft_corner",
            )

        rail_positions = (-0.40, -0.28, -0.16, -0.04, 0.08, 0.20, 0.32, 0.44)
        for i, x in enumerate(rail_positions):
            _cylinder_between(
                part,
                (x, sign * 0.052, 0.0),
                (x, sign * 0.394, 0.0),
                0.0058,
                material=satin_aluminum,
                name=f"hanging_rail_{i}",
            )
            if i in (1, 4, 6):
                _box(
                    part,
                    (x, sign * 0.22, -0.006),
                    (0.052, 0.035, 0.012),
                    material=seam_plastic,
                    name=f"brace_socket_{i}",
                )

        for x, label in ((-0.18, "slot_0"), (0.18, "slot_1")):
            _box(
                part,
                (x, sign * 0.065, 0.012),
                (0.050, 0.032, 0.010),
                material=warm_hardware,
                name=f"fold_stop_{label}",
            )

    wing_0 = model.part("wing_0")
    add_wing(wing_0, sign=1, prefix="wing_0")

    wing_1 = model.part("wing_1")
    add_wing(wing_1, sign=-1, prefix="wing_1")

    def add_leg_frame(part, *, x_sign: int):
        _cylinder_between(
            part,
            (0.0, -0.200, 0.0),
            (0.0, 0.200, 0.0),
            0.013,
            material=warm_hardware,
            name="top_sleeve",
        )
        for sign, side in ((-1, "near"), (1, "far")):
            _cylinder_between(
                part,
                (0.0, sign * 0.185, -0.010),
                (x_sign * 0.142, sign * 0.320, -0.730),
                0.012,
                material=matte_graphite,
                name=f"{side}_leg",
            )
            _box(
                part,
                (x_sign * 0.145, sign * 0.348, -0.742),
                (0.090, 0.045, 0.026),
                material=soft_rubber,
                name="near_foot" if side == "near" else "far_foot",
            )

        _cylinder_between(
            part,
            (x_sign * 0.142, -0.355, -0.730),
            (x_sign * 0.142, 0.355, -0.730),
            0.011,
            material=matte_graphite,
            name="floor_bar",
        )
        _cylinder_between(
            part,
            (x_sign * 0.040, -0.223, -0.212),
            (x_sign * 0.111, 0.290, -0.572),
            0.006,
            material=satin_aluminum,
            name="diagonal_stay_0",
        )
        _cylinder_between(
            part,
            (x_sign * 0.040, 0.223, -0.212),
            (x_sign * 0.111, -0.290, -0.572),
            0.006,
            material=satin_aluminum,
            name="diagonal_stay_1",
        )

    leg_frame_0 = model.part("leg_frame_0")
    add_leg_frame(leg_frame_0, x_sign=-1)

    leg_frame_1 = model.part("leg_frame_1")
    add_leg_frame(leg_frame_1, x_sign=1)

    model.articulation(
        "central_to_wing_0",
        ArticulationType.REVOLUTE,
        parent=central,
        child=wing_0,
        origin=Origin(xyz=(0.0, 0.305, top_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.2, lower=0.0, upper=1.45),
    )
    model.articulation(
        "central_to_wing_1",
        ArticulationType.REVOLUTE,
        parent=central,
        child=wing_1,
        origin=Origin(xyz=(0.0, -0.305, top_z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=12.0, velocity=1.2, lower=0.0, upper=1.45),
    )
    model.articulation(
        "central_to_leg_0",
        ArticulationType.REVOLUTE,
        parent=central,
        child=leg_frame_0,
        origin=Origin(xyz=(-0.38, 0.0, 0.748)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=1.0, lower=0.0, upper=1.10),
    )
    model.articulation(
        "central_to_leg_1",
        ArticulationType.REVOLUTE,
        parent=central,
        child=leg_frame_1,
        origin=Origin(xyz=(0.38, 0.0, 0.748)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=20.0, velocity=1.0, lower=0.0, upper=1.10),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    central = object_model.get_part("central_frame")
    wing_0 = object_model.get_part("wing_0")
    wing_1 = object_model.get_part("wing_1")
    leg_0 = object_model.get_part("leg_frame_0")
    leg_1 = object_model.get_part("leg_frame_1")

    wing_0_joint = object_model.get_articulation("central_to_wing_0")
    wing_1_joint = object_model.get_articulation("central_to_wing_1")
    leg_0_joint = object_model.get_articulation("central_to_leg_0")
    leg_1_joint = object_model.get_articulation("central_to_leg_1")

    for leg, axle in ((leg_0, "leg_0_upper_axle"), (leg_1, "leg_1_upper_axle")):
        ctx.allow_overlap(
            central,
            leg,
            elem_a=axle,
            elem_b="top_sleeve",
            reason="The folding leg sleeve intentionally rotates around a captured axle through its bore.",
        )
        ctx.expect_overlap(
            central,
            leg,
            axes="y",
            min_overlap=0.30,
            name=f"{axle} has retained sleeve engagement",
        )

    ctx.expect_overlap(
        wing_0,
        central,
        axes="x",
        min_overlap=0.85,
        name="wing 0 spans the central frame length",
    )
    ctx.expect_overlap(
        wing_1,
        central,
        axes="x",
        min_overlap=0.85,
        name="wing 1 spans the central frame length",
    )
    wing0_open = ctx.part_world_aabb(wing_0)
    wing1_open = ctx.part_world_aabb(wing_1)
    leg0_open = ctx.part_world_aabb(leg_0)
    leg1_open = ctx.part_world_aabb(leg_1)
    ctx.check(
        "opposed folding feet share a level stance",
        leg0_open is not None
        and leg1_open is not None
        and abs(leg0_open[0][2] - leg1_open[0][2]) < 0.004
        and leg0_open[0][2] < 0.04
        and leg1_open[0][2] < 0.04,
        details=f"leg0={leg0_open}, leg1={leg1_open}",
    )
    with ctx.pose({wing_0_joint: 1.25, wing_1_joint: 1.25, leg_0_joint: 0.85, leg_1_joint: 0.85}):
        wing0_folded = ctx.part_world_aabb(wing_0)
        wing1_folded = ctx.part_world_aabb(wing_1)
        leg0_folded = ctx.part_world_aabb(leg_0)
        leg1_folded = ctx.part_world_aabb(leg_1)

    ctx.check(
        "wing 0 folds upward about its side hinge",
        wing0_open is not None
        and wing0_folded is not None
        and wing0_folded[1][2] > wing0_open[1][2] + 0.30,
        details=f"open={wing0_open}, folded={wing0_folded}",
    )
    ctx.check(
        "wing 1 folds upward about its side hinge",
        wing1_open is not None
        and wing1_folded is not None
        and wing1_folded[1][2] > wing1_open[1][2] + 0.30,
        details=f"open={wing1_open}, folded={wing1_folded}",
    )
    ctx.check(
        "leg frames fold inward under the rack",
        leg0_open is not None
        and leg1_open is not None
        and leg0_folded is not None
        and leg1_folded is not None
        and leg0_folded[1][0] > leg0_open[1][0] + 0.15
        and leg1_folded[0][0] < leg1_open[0][0] - 0.15,
        details=f"leg0 open={leg0_open}, folded={leg0_folded}; leg1 open={leg1_open}, folded={leg1_folded}",
    )

    return ctx.report()


object_model = build_object_model()
