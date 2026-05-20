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


def _midpoint(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    return ((a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, (a[2] + b[2]) * 0.5)


def _distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2)


def _rpy_for_cylinder(
    a: tuple[float, float, float], b: tuple[float, float, float]
) -> tuple[float, float, float]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    dz = b[2] - a[2]
    length_xy = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(length_xy, dz)
    return (0.0, pitch, yaw)


def _add_rod(part, a, b, *, radius: float, material, name: str | None = None) -> None:
    part.visual(
        Cylinder(radius=radius, length=_distance(a, b)),
        origin=Origin(xyz=_midpoint(a, b), rpy=_rpy_for_cylinder(a, b)),
        material=material,
        name=name,
    )


def _add_bar(
    part,
    a: tuple[float, float, float],
    b: tuple[float, float, float],
    *,
    width: float,
    height: float,
    material,
    name: str | None = None,
) -> None:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    length = math.hypot(dx, dy)
    yaw = math.atan2(dy, dx)
    part.visual(
        Box((length, width, height)),
        origin=Origin(xyz=_midpoint(a, b), rpy=(0.0, 0.0, yaw)),
        material=material,
        name=name,
    )


def _add_hatch_frame(part, *, cx: float, cy: float, seat_material) -> None:
    outer_x = 0.210
    outer_y = 0.100
    lip = 0.014
    z = 0.008
    h = 0.004
    part.visual(
        Box((outer_x, lip, h)),
        origin=Origin(xyz=(cx, cy - (outer_y - lip) * 0.5, z + h * 0.5)),
        material=seat_material,
    )
    part.visual(
        Box((outer_x, lip, h)),
        origin=Origin(xyz=(cx, cy + (outer_y - lip) * 0.5, z + h * 0.5)),
        material=seat_material,
    )
    part.visual(
        Box((lip, outer_y - 2.0 * lip, h)),
        origin=Origin(xyz=(cx - (outer_x - lip) * 0.5, cy, z + h * 0.5)),
        material=seat_material,
    )
    part.visual(
        Box((lip, outer_y - 2.0 * lip, h)),
        origin=Origin(xyz=(cx + (outer_x - lip) * 0.5, cy, z + h * 0.5)),
        material=seat_material,
    )


def _add_spindle_support(
    part,
    *,
    sx: float,
    base_material,
    plate_material,
    bolt_material,
    cap_name: str,
) -> None:
    part.visual(
        Box((0.108, 0.016, 0.018)),
        origin=Origin(xyz=(sx, -0.024, 0.009)),
        material=base_material,
        name=f"{cap_name}_rear_saddle",
    )
    for dx in (-0.030, 0.030):
        part.visual(
            Box((0.020, 0.042, 0.018)),
            origin=Origin(xyz=(sx + dx, 0.002, 0.009)),
            material=base_material,
        )
        part.visual(
            Box((0.014, 0.018, 0.046)),
            origin=Origin(xyz=(sx + dx * 0.85, 0.004, 0.032)),
            material=base_material,
        )
        part.visual(
            Box((0.010, 0.016, 0.020)),
            origin=Origin(xyz=(sx + dx * 0.72, 0.006, 0.058)),
            material=base_material,
        )
        part.visual(
            Box((0.014, 0.022, 0.004)),
            origin=Origin(xyz=(sx + dx * 0.72, 0.006, 0.070)),
            material=plate_material,
            name=f"{cap_name}_clamp_{'left' if dx < 0 else 'right'}",
        )
        part.visual(
            Cylinder(radius=0.0045, length=0.004),
            origin=Origin(xyz=(sx + dx * 0.72, 0.000, 0.074)),
            material=bolt_material,
        )
        part.visual(
            Cylinder(radius=0.0045, length=0.004),
            origin=Origin(xyz=(sx + dx * 0.72, 0.012, 0.074)),
            material=bolt_material,
        )
        _add_rod(
            part,
            (sx + dx * 0.95, -0.016, 0.018),
            (sx + dx * 0.72, 0.004, 0.052),
            radius=0.006,
            material=base_material,
        )
    part.visual(
        Box((0.014, 0.018, 0.006)),
        origin=Origin(xyz=(sx - 0.021, 0.006, 0.076)),
        material=plate_material,
        name=f"{cap_name}_bridge_left",
    )
    part.visual(
        Box((0.014, 0.018, 0.006)),
        origin=Origin(xyz=(sx + 0.021, 0.006, 0.076)),
        material=plate_material,
        name=f"{cap_name}_bridge_right",
    )
    part.visual(
        Cylinder(radius=0.0045, length=0.006),
        origin=Origin(xyz=(sx - 0.016, 0.006, 0.082)),
        material=bolt_material,
        name=f"{cap_name}_bolt_left",
    )
    part.visual(
        Cylinder(radius=0.0045, length=0.006),
        origin=Origin(xyz=(sx + 0.016, 0.006, 0.082)),
        material=bolt_material,
        name=f"{cap_name}_bolt_right",
    )


def _add_service_hatch(part, *, material, bolt_material) -> None:
    part.visual(
        Box((0.180, 0.072, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=material,
        name="hatch_base",
    )
    part.visual(
        Box((0.142, 0.046, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.008)),
        material=material,
        name="hatch_riser",
    )
    part.visual(
        Box((0.046, 0.010, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.011)),
        material=bolt_material,
        name="pull_handle",
    )
    part.visual(
        Box((0.010, 0.024, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
        material=bolt_material,
    )
    for bx in (-0.064, 0.064):
        for by in (-0.022, 0.022):
            part.visual(
                Cylinder(radius=0.0045, length=0.004),
                origin=Origin(xyz=(bx, by, 0.010)),
                material=bolt_material,
            )


def _add_wiper_part(part, *, sign: float, metal, dark_rubber) -> None:
    part.visual(
        Cylinder(radius=0.007, length=0.084),
        origin=Origin(xyz=(0.0, 0.0, -0.018)),
        material=metal,
        name="spindle_shaft",
    )
    part.visual(
        Cylinder(radius=0.013, length=0.012),
        origin=Origin(xyz=(0.0, 0.0, 0.002)),
        material=metal,
        name="spindle_collar",
    )
    part.visual(
        Cylinder(radius=0.010, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=metal,
    )

    _add_bar(
        part,
        (0.0, 0.0, 0.026),
        (sign * 0.050, 0.120, 0.026),
        width=0.018,
        height=0.008,
        material=metal,
        name="arm_base",
    )
    _add_bar(
        part,
        (sign * 0.050, 0.120, 0.025),
        (sign * 0.082, 0.305, 0.023),
        width=0.014,
        height=0.007,
        material=metal,
        name="arm_reach",
    )
    _add_rod(
        part,
        (sign * 0.018, 0.050, 0.034),
        (sign * 0.088, 0.280, 0.028),
        radius=0.004,
        material=metal,
        name="spring_bridge",
    )
    _add_bar(
        part,
        (sign * 0.095, 0.230, 0.026),
        (sign * 0.138, 0.534, 0.026),
        width=0.018,
        height=0.010,
        material=metal,
        name="blade_carrier",
    )
    _add_rod(
        part,
        (sign * 0.082, 0.305, 0.023),
        (sign * 0.098, 0.246, 0.026),
        radius=0.0045,
        material=metal,
        name="carrier_yoke",
    )
    _add_bar(
        part,
        (sign * 0.098, 0.238, 0.018),
        (sign * 0.140, 0.530, 0.018),
        width=0.006,
        height=0.006,
        material=dark_rubber,
        name="blade_rubber",
    )
    for t in (0.18, 0.50, 0.82):
        part.visual(
            Box((0.012, 0.010, 0.010)),
            origin=Origin(
                xyz=(
                    sign * (0.095 + (0.138 - 0.095) * t),
                    0.230 + (0.534 - 0.230) * t,
                    0.022,
                )
            ),
            material=metal,
        )
    _add_rod(
        part,
        (sign * 0.060, 0.170, 0.026),
        (sign * 0.104, 0.250, 0.026),
        radius=0.004,
        material=metal,
        name="secondary_stay",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="retrofit_wiper_assembly")

    painted_steel = model.material("painted_steel", rgba=(0.31, 0.34, 0.36, 1.0))
    cover_gray = model.material("cover_gray", rgba=(0.53, 0.54, 0.56, 1.0))
    zinc = model.material("zinc", rgba=(0.73, 0.74, 0.76, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.16, 0.17, 0.18, 1.0))
    rubber = model.material("rubber", rgba=(0.06, 0.06, 0.06, 1.0))

    cowl = model.part("cowl_frame")
    cowl.visual(
        Box((1.020, 0.040, 0.006)),
        origin=Origin(xyz=(0.0, -0.018, 0.003)),
        material=painted_steel,
        name="tray_floor_front",
    )
    cowl.visual(
        Box((1.020, 0.028, 0.006)),
        origin=Origin(xyz=(0.0, 0.030, 0.003)),
        material=painted_steel,
        name="tray_floor_rear",
    )
    cowl.visual(
        Box((0.240, 0.076, 0.006)),
        origin=Origin(xyz=(0.0, 0.002, 0.003)),
        material=painted_steel,
        name="center_bridge",
    )
    cowl.visual(
        Box((1.050, 0.050, 0.042)),
        origin=Origin(xyz=(0.0, -0.075, 0.021)),
        material=painted_steel,
        name="front_rail",
    )
    cowl.visual(
        Box((1.050, 0.026, 0.056)),
        origin=Origin(xyz=(0.0, 0.067, 0.028)),
        material=painted_steel,
        name="rear_rail",
    )
    cowl.visual(
        Box((0.030, 0.132, 0.040)),
        origin=Origin(xyz=(-0.510, 0.001, 0.020)),
        material=painted_steel,
    )
    cowl.visual(
        Box((0.030, 0.132, 0.040)),
        origin=Origin(xyz=(0.510, 0.001, 0.020)),
        material=painted_steel,
    )
    _add_hatch_frame(cowl, cx=-0.400, cy=0.012, seat_material=painted_steel)
    _add_hatch_frame(cowl, cx=0.400, cy=0.012, seat_material=painted_steel)

    cowl.visual(
        Box((0.180, 0.090, 0.033)),
        origin=Origin(xyz=(0.0, -0.006, 0.0165)),
        material=painted_steel,
        name="motor_pedestal",
    )
    for sx in (-0.078, 0.078):
        cowl.visual(
            Box((0.018, 0.064, 0.026)),
            origin=Origin(xyz=(sx, -0.006, 0.013)),
            material=painted_steel,
        )

    _add_spindle_support(
        cowl,
        sx=-0.240,
        base_material=painted_steel,
        plate_material=cover_gray,
        bolt_material=zinc,
        cap_name="left_bearing_cap",
    )
    _add_spindle_support(
        cowl,
        sx=0.240,
        base_material=painted_steel,
        plate_material=cover_gray,
        bolt_material=zinc,
        cap_name="right_bearing_cap",
    )

    cowl.inertial = Inertial.from_geometry(
        Box((1.050, 0.160, 0.072)),
        mass=11.0,
        origin=Origin(xyz=(0.0, 0.0, 0.036)),
    )

    motor = model.part("motor_pack")
    motor.visual(
        Box((0.180, 0.080, 0.008)),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=dark_steel,
        name="mount_plate",
    )
    motor.visual(
        Box((0.130, 0.080, 0.036)),
        origin=Origin(xyz=(0.000, 0.000, 0.026)),
        material=cover_gray,
        name="gearbox_housing",
    )
    motor.visual(
        Cylinder(radius=0.028, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.056)),
        material=zinc,
        name="output_pad",
    )
    motor.visual(
        Cylinder(radius=0.032, length=0.132),
        origin=Origin(xyz=(-0.126, 0.018, 0.028), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=cover_gray,
        name="motor_can",
    )
    motor.visual(
        Cylinder(radius=0.026, length=0.012),
        origin=Origin(xyz=(-0.198, 0.018, 0.028), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
    )
    motor.visual(
        Box((0.080, 0.040, 0.024)),
        origin=Origin(xyz=(-0.062, 0.009, 0.028)),
        material=cover_gray,
        name="motor_neck",
    )
    motor.visual(
        Box((0.042, 0.020, 0.020)),
        origin=Origin(xyz=(-0.060, 0.0, 0.022)),
        material=dark_steel,
    )
    for bx in (-0.060, 0.060):
        for by in (-0.024, 0.024):
            motor.visual(
                Cylinder(radius=0.0045, length=0.008),
                origin=Origin(xyz=(bx, by, 0.010)),
                material=zinc,
            )
    motor.inertial = Inertial.from_geometry(
        Box((0.250, 0.090, 0.080)),
        mass=3.6,
        origin=Origin(xyz=(0.000, 0.000, 0.040)),
    )

    crank = model.part("crank_train")
    crank.visual(
        Cylinder(radius=0.026, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, 0.005)),
        material=zinc,
        name="hub",
    )
    _add_bar(
        crank,
        (0.0, 0.0, 0.006),
        (-0.075, 0.000, 0.006),
        width=0.020,
        height=0.010,
        material=zinc,
        name="crank_arm",
    )
    crank.visual(
        Cylinder(radius=0.012, length=0.010),
        origin=Origin(xyz=(-0.075, 0.0, 0.005)),
        material=zinc,
    )
    _add_bar(
        crank,
        (-0.075, 0.000, 0.006),
        (-0.160, 0.024, 0.006),
        width=0.014,
        height=0.008,
        material=zinc,
        name="left_drag_link",
    )
    _add_bar(
        crank,
        (-0.160, 0.024, 0.006),
        (0.160, 0.024, 0.006),
        width=0.012,
        height=0.008,
        material=zinc,
        name="cross_link",
    )
    _add_bar(
        crank,
        (0.020, -0.010, 0.006),
        (0.100, -0.032, 0.006),
        width=0.014,
        height=0.008,
        material=dark_steel,
    )
    crank.visual(
        Box((0.036, 0.018, 0.010)),
        origin=Origin(xyz=(0.112, -0.036, 0.006)),
        material=dark_steel,
        name="counterweight_pad",
    )
    crank.inertial = Inertial.from_geometry(
        Box((0.340, 0.080, 0.020)),
        mass=0.7,
        origin=Origin(xyz=(0.000, 0.010, 0.010)),
    )

    left_wiper = model.part("left_wiper")
    _add_wiper_part(left_wiper, sign=1.0, metal=zinc, dark_rubber=rubber)
    left_wiper.inertial = Inertial.from_geometry(
        Box((0.190, 0.560, 0.060)),
        mass=0.9,
        origin=Origin(xyz=(0.080, 0.270, 0.010)),
    )

    right_wiper = model.part("right_wiper")
    _add_wiper_part(right_wiper, sign=-1.0, metal=zinc, dark_rubber=rubber)
    right_wiper.inertial = Inertial.from_geometry(
        Box((0.190, 0.560, 0.060)),
        mass=0.9,
        origin=Origin(xyz=(-0.080, 0.270, 0.010)),
    )

    left_hatch = model.part("left_service_hatch")
    _add_service_hatch(left_hatch, material=cover_gray, bolt_material=zinc)
    left_hatch.inertial = Inertial.from_geometry(
        Box((0.180, 0.072, 0.018)),
        mass=0.35,
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
    )

    right_hatch = model.part("right_service_hatch")
    _add_service_hatch(right_hatch, material=cover_gray, bolt_material=zinc)
    right_hatch.inertial = Inertial.from_geometry(
        Box((0.180, 0.072, 0.018)),
        mass=0.35,
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
    )

    model.articulation(
        "cowl_to_motor",
        ArticulationType.FIXED,
        parent=cowl,
        child=motor,
        origin=Origin(xyz=(0.0, -0.006, 0.033)),
    )
    model.articulation(
        "motor_to_crank",
        ArticulationType.REVOLUTE,
        parent=motor,
        child=crank,
        origin=Origin(xyz=(0.0, 0.0, 0.068)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=18.0,
            velocity=2.2,
            lower=-0.35,
            upper=0.35,
        ),
    )
    model.articulation(
        "cowl_to_left_wiper",
        ArticulationType.REVOLUTE,
        parent=cowl,
        child=left_wiper,
        origin=Origin(xyz=(-0.240, 0.006, 0.086)),
        axis=(0.0, 0.0, -1.0),
        motion_limits=MotionLimits(
            effort=14.0,
            velocity=2.0,
            lower=0.0,
            upper=0.65,
        ),
    )
    model.articulation(
        "cowl_to_right_wiper",
        ArticulationType.REVOLUTE,
        parent=cowl,
        child=right_wiper,
        origin=Origin(xyz=(0.240, 0.006, 0.086)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=14.0,
            velocity=2.0,
            lower=0.0,
            upper=0.65,
        ),
    )
    model.articulation(
        "cowl_to_left_hatch",
        ArticulationType.FIXED,
        parent=cowl,
        child=left_hatch,
        origin=Origin(xyz=(-0.400, 0.012, 0.012)),
    )
    model.articulation(
        "cowl_to_right_hatch",
        ArticulationType.FIXED,
        parent=cowl,
        child=right_hatch,
        origin=Origin(xyz=(0.400, 0.012, 0.012)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cowl = object_model.get_part("cowl_frame")
    motor = object_model.get_part("motor_pack")
    crank = object_model.get_part("crank_train")
    left_wiper = object_model.get_part("left_wiper")
    right_wiper = object_model.get_part("right_wiper")
    left_hatch = object_model.get_part("left_service_hatch")
    right_hatch = object_model.get_part("right_service_hatch")
    crank_joint = object_model.get_articulation("motor_to_crank")
    left_joint = object_model.get_articulation("cowl_to_left_wiper")
    right_joint = object_model.get_articulation("cowl_to_right_wiper")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts()
    # 2) noisier warning-tier sensor for same-part disconnected geometry islands
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    # 3) likely-failure rest-pose part-to-part overlap backstop for real 3D interpenetration
    # This is not an "inside / nested / footprint overlap" check.
    # Investigate all three. Warning-tier signals are not free passes.
    # Use `ctx.allow_overlap(...)` only for true intended penetration.
    # If parts are nested but should remain clear, prove that with exact
    # `expect_contact(...)`, `expect_gap(...)`, `expect_overlap(...)`, or
    # `expect_within(...)` checks instead.
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(motor, cowl, elem_a="mount_plate", elem_b="motor_pedestal")
    ctx.expect_contact(crank, motor, elem_a="hub", elem_b="output_pad")
    ctx.expect_overlap(left_wiper, cowl, axes="xy", min_overlap=0.010, elem_a="spindle_shaft", name="left_spindle_aligned_with_support")
    ctx.expect_overlap(right_wiper, cowl, axes="xy", min_overlap=0.010, elem_a="spindle_shaft", name="right_spindle_aligned_with_support")
    ctx.expect_contact(left_hatch, cowl, elem_a="hatch_base")
    ctx.expect_contact(right_hatch, cowl, elem_a="hatch_base")

    with ctx.pose({crank_joint: 0.35}):
        ctx.expect_within(crank, cowl, axes="xy", margin=0.0, name="crank_forward_sweep_within_tray")
    with ctx.pose({crank_joint: -0.35}):
        ctx.expect_within(crank, cowl, axes="xy", margin=0.0, name="crank_reverse_sweep_within_tray")
    with ctx.pose({left_joint: 0.55, right_joint: 0.55, crank_joint: 0.22}):
        ctx.expect_gap(
            left_wiper,
            cowl,
            axis="z",
            min_gap=0.004,
            positive_elem="blade_rubber",
            name="left_blade_clears_cowl_in_sweep",
        )
        ctx.expect_gap(
            right_wiper,
            cowl,
            axis="z",
            min_gap=0.004,
            positive_elem="blade_rubber",
            name="right_blade_clears_cowl_in_sweep",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
