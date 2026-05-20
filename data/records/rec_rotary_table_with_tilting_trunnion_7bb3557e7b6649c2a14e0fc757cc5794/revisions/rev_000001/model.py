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


def polar_points(radius: float, count: int, phase: float = 0.0) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos(phase + (2.0 * math.pi * i / count)),
            radius * math.sin(phase + (2.0 * math.pi * i / count)),
        )
        for i in range(count)
    ]


def box_solid(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def cylinder_x(
    radius: float,
    length: float,
    x_start: float,
    *,
    y: float = 0.0,
    z: float = 0.0,
) -> cq.Workplane:
    return cq.Workplane("YZ").circle(radius).extrude(length).translate((x_start, y, z))


def cylinder_y(
    radius: float,
    length: float,
    y_start: float,
    *,
    x: float = 0.0,
    z: float = 0.0,
) -> cq.Workplane:
    return cq.Workplane("XZ").circle(radius).extrude(length).translate((x, y_start, z))


def make_base_shape() -> cq.Workplane:
    foot = cq.Workplane("XY").circle(0.36).extrude(0.06)
    anchor_bolts = (
        cq.Workplane("XY")
        .pushPoints(polar_points(0.30, 8, math.pi / 8.0))
        .circle(0.013)
        .extrude(0.010)
        .translate((0.0, 0.0, 0.06))
    )

    lower_pedestal = cq.Workplane("XY").circle(0.28).extrude(0.075).translate((0.0, 0.0, 0.06))
    drum = cq.Workplane("XY").circle(0.25).extrude(0.225).translate((0.0, 0.0, 0.135))
    top_ring = (
        cq.Workplane("XY").circle(0.30).circle(0.12).extrude(0.040).translate((0.0, 0.0, 0.360))
    )
    top_bolt_circle = (
        cq.Workplane("XY")
        .pushPoints(polar_points(0.265, 12, math.pi / 12.0))
        .circle(0.0085)
        .extrude(0.012)
        .translate((0.0, 0.0, 0.388))
    )

    cover_points = [(-0.062, 0.207), (0.062, 0.207), (-0.062, 0.263), (0.062, 0.263)]

    front_cover = box_solid((0.18, 0.020, 0.11), (0.0, 0.254, 0.235))
    front_cover = front_cover.union(cylinder_y(0.030, 0.008, 0.264, z=0.235))
    front_cover = front_cover.union(
        cq.Workplane("XZ").pushPoints(cover_points).circle(0.006).extrude(0.006).translate((0.0, 0.264, 0.0))
    )

    rear_cover = box_solid((0.18, 0.020, 0.11), (0.0, -0.254, 0.235))
    rear_cover = rear_cover.union(cylinder_y(0.030, 0.008, -0.272, z=0.235))
    rear_cover = rear_cover.union(
        cq.Workplane("XZ")
        .pushPoints(cover_points)
        .circle(0.006)
        .extrude(0.006)
        .translate((0.0, -0.270, 0.0))
    )

    base = foot
    for extra in (
        anchor_bolts,
        lower_pedestal,
        drum,
        top_ring,
        front_cover,
        rear_cover,
    ):
        base = base.union(extra)

    center_well = cq.Workplane("XY").circle(0.105).extrude(0.100).translate((0.0, 0.0, 0.300))
    base = base.cut(center_well).cut(top_bolt_circle)
    return base


def make_yoke_shape() -> cq.Workplane:
    axis_z = 0.340

    platter = cq.Workplane("XY").circle(0.27).extrude(0.050)
    top_hub = cq.Workplane("XY").circle(0.11).extrude(0.058).translate((0.0, 0.0, 0.050))
    hub_shoulder = cq.Workplane("XY").circle(0.17).circle(0.078).extrude(0.018).translate((0.0, 0.0, 0.050))
    hub_bolts = (
        cq.Workplane("XY")
        .pushPoints(polar_points(0.083, 10, math.pi / 10.0))
        .circle(0.006)
        .extrude(0.008)
        .translate((0.0, 0.0, 0.108))
    )

    right_tower = box_solid((0.12, 0.26, 0.58), (0.34, 0.0, 0.355))
    left_tower = box_solid((0.12, 0.26, 0.58), (-0.34, 0.0, 0.355))

    right_front_buttress = box_solid((0.12, 0.09, 0.20), (0.34, 0.085, 0.145))
    right_rear_buttress = box_solid((0.12, 0.09, 0.20), (0.34, -0.085, 0.145))
    left_front_buttress = box_solid((0.12, 0.09, 0.20), (-0.34, 0.085, 0.145))
    left_rear_buttress = box_solid((0.12, 0.09, 0.20), (-0.34, -0.085, 0.145))

    right_housing = cylinder_x(0.105, 0.140, 0.280, z=axis_z)
    left_housing = cylinder_x(0.105, 0.140, -0.420, z=axis_z)
    right_inner_boss = cylinder_x(0.074, 0.020, 0.260, z=axis_z)
    left_inner_boss = cylinder_x(0.074, 0.020, -0.280, z=axis_z)

    right_cover = cylinder_x(0.094, 0.010, 0.422, z=axis_z)
    left_cover = cylinder_x(0.094, 0.010, -0.432, z=axis_z)

    yz_cover_points = [
        (0.062, axis_z),
        (-0.062, axis_z),
        (0.032, axis_z + 0.054),
        (0.032, axis_z - 0.054),
        (-0.032, axis_z + 0.054),
        (-0.032, axis_z - 0.054),
    ]
    right_cover_bolts = (
        cq.Workplane("YZ")
        .pushPoints(yz_cover_points)
        .circle(0.005)
        .extrude(0.006)
        .translate((0.432, 0.0, 0.0))
    )
    left_cover_bolts = (
        cq.Workplane("YZ")
        .pushPoints(yz_cover_points)
        .circle(0.005)
        .extrude(0.006)
        .translate((-0.438, 0.0, 0.0))
    )

    yoke = platter
    for extra in (
        top_hub,
        hub_shoulder,
        hub_bolts,
        right_tower,
        left_tower,
        right_front_buttress,
        right_rear_buttress,
        left_front_buttress,
        left_rear_buttress,
        right_housing,
        left_housing,
        right_inner_boss,
        left_inner_boss,
        right_cover,
        left_cover,
        right_cover_bolts,
        left_cover_bolts,
    ):
        yoke = yoke.union(extra)

    right_bore = cylinder_x(0.046, 0.160, 0.260, z=axis_z)
    left_bore = cylinder_x(0.046, 0.160, -0.420, z=axis_z)
    right_window = box_solid((0.082, 0.160, 0.22), (0.34, 0.0, 0.305))
    left_window = box_solid((0.082, 0.160, 0.22), (-0.34, 0.0, 0.305))

    return yoke.cut(right_bore).cut(left_bore).cut(right_window).cut(left_window)


def make_plate_body_shape() -> cq.Workplane:
    plate = cq.Workplane("XY").box(0.42, 0.32, 0.026).translate((0.0, 0.0, -0.035))
    plate = plate.edges("|Z").fillet(0.010)

    perimeter_rib = box_solid((0.38, 0.28, 0.024), (0.0, 0.0, -0.060))
    perimeter_rib = perimeter_rib.cut(box_solid((0.30, 0.20, 0.026), (0.0, 0.0, -0.060)))
    center_rib_x = box_solid((0.24, 0.026, 0.024), (0.0, 0.0, -0.060))
    center_rib_y = box_solid((0.026, 0.22, 0.024), (0.0, 0.0, -0.060))

    right_hanger = box_solid((0.040, 0.090, 0.100), (0.188, 0.0, -0.010))
    left_hanger = box_solid((0.040, 0.090, 0.100), (-0.188, 0.0, -0.010))
    right_hanger_cap = box_solid((0.055, 0.110, 0.028), (0.198, 0.0, 0.020))
    left_hanger_cap = box_solid((0.055, 0.110, 0.028), (-0.198, 0.0, 0.020))
    right_shoulder = box_solid((0.028, 0.086, 0.040), (0.236, 0.0, 0.000))
    left_shoulder = box_solid((0.028, 0.086, 0.040), (-0.236, 0.0, 0.000))

    for extra in (
        perimeter_rib,
        center_rib_x,
        center_rib_y,
        right_hanger,
        left_hanger,
        right_hanger_cap,
        left_hanger_cap,
        right_shoulder,
        left_shoulder,
    ):
        plate = plate.union(extra)

    slot_cuts = (
        cq.Workplane("XY")
        .pushPoints([(-0.11, 0.0), (0.0, 0.0), (0.11, 0.0)])
        .slot2D(0.24, 0.018, angle=90)
        .extrude(0.012)
        .translate((0.0, 0.0, -0.042))
    )
    counterbore_cuts = (
        cq.Workplane("XY")
        .pushPoints(
            [
                (-0.145, -0.10),
                (0.0, -0.10),
                (0.145, -0.10),
                (-0.145, 0.10),
                (0.0, 0.10),
                (0.145, 0.10),
            ]
        )
        .circle(0.009)
        .extrude(0.008)
        .translate((0.0, 0.0, -0.039))
    )
    return plate.cut(slot_cuts).cut(counterbore_cuts)


def make_trunnion_shape(side: str) -> cq.Workplane:
    if side not in {"left", "right"}:
        raise ValueError(f"unsupported side {side!r}")

    if side == "right":
        collar = cylinder_x(0.068, 0.010, 0.250)
        journal = cylinder_x(0.034, 0.160, 0.260)
        bolt_x = 0.248
    else:
        collar = cylinder_x(0.068, 0.010, -0.260)
        journal = cylinder_x(0.034, 0.160, -0.420)
        bolt_x = -0.252

    bolts = (
        cq.Workplane("YZ")
        .pushPoints(
            [
                (0.040, 0.040),
                (-0.040, 0.040),
                (0.040, -0.040),
                (-0.040, -0.040),
            ]
        )
        .circle(0.0045)
        .extrude(0.004)
        .translate((bolt_x, 0.0, 0.0))
    )
    return collar.union(journal).union(bolts)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dual_axis_rotary_fixture")

    base_color = model.material("base_cast_iron", rgba=(0.24, 0.25, 0.27, 1.0))
    yoke_color = model.material("support_gray", rgba=(0.42, 0.44, 0.47, 1.0))
    plate_color = model.material("machined_steel", rgba=(0.63, 0.65, 0.68, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(make_base_shape(), "fixture_base"),
        material=base_color,
        name="base_shell",
    )
    base.inertial = Inertial.from_geometry(
        Cylinder(radius=0.36, length=0.40),
        mass=185.0,
        origin=Origin(xyz=(0.0, 0.0, 0.20)),
    )

    yoke = model.part("yoke")
    yoke.visual(
        mesh_from_cadquery(make_yoke_shape(), "fixture_yoke"),
        material=yoke_color,
        name="yoke_shell",
    )
    yoke.inertial = Inertial.from_geometry(
        Box((0.82, 0.30, 0.62)),
        mass=92.0,
        origin=Origin(xyz=(0.0, 0.0, 0.29)),
    )

    plate = model.part("tool_plate")
    plate.visual(
        mesh_from_cadquery(make_plate_body_shape(), "fixture_tool_plate"),
        material=plate_color,
        name="plate_shell",
    )
    plate.inertial = Inertial.from_geometry(
        Box((0.44, 0.34, 0.32)),
        mass=46.0,
        origin=Origin(xyz=(0.0, 0.0, -0.12)),
    )

    right_trunnion = model.part("right_trunnion")
    right_trunnion.visual(
        mesh_from_cadquery(make_trunnion_shape("right"), "fixture_right_trunnion"),
        material=plate_color,
        name="right_trunnion_shell",
    )
    right_trunnion.inertial = Inertial.from_geometry(
        Cylinder(radius=0.068, length=0.17),
        mass=4.2,
        origin=Origin(xyz=(0.335, 0.0, 0.0)),
    )

    left_trunnion = model.part("left_trunnion")
    left_trunnion.visual(
        mesh_from_cadquery(make_trunnion_shape("left"), "fixture_left_trunnion"),
        material=plate_color,
        name="left_trunnion_shell",
    )
    left_trunnion.inertial = Inertial.from_geometry(
        Cylinder(radius=0.068, length=0.17),
        mass=4.2,
        origin=Origin(xyz=(-0.335, 0.0, 0.0)),
    )

    model.articulation(
        "base_rotation",
        ArticulationType.CONTINUOUS,
        parent=base,
        child=yoke,
        origin=Origin(xyz=(0.0, 0.0, 0.40)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=250.0, velocity=1.0),
    )

    model.articulation(
        "trunnion_tilt",
        ArticulationType.REVOLUTE,
        parent=yoke,
        child=plate,
        origin=Origin(xyz=(0.0, 0.0, 0.340)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=0.8,
            lower=-1.20,
            upper=1.20,
        ),
    )

    model.articulation(
        "plate_to_right_trunnion",
        ArticulationType.FIXED,
        parent=plate,
        child=right_trunnion,
        origin=Origin(),
    )

    model.articulation(
        "plate_to_left_trunnion",
        ArticulationType.FIXED,
        parent=plate,
        child=left_trunnion,
        origin=Origin(),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    yoke = object_model.get_part("yoke")
    plate = object_model.get_part("tool_plate")
    right_trunnion = object_model.get_part("right_trunnion")
    left_trunnion = object_model.get_part("left_trunnion")
    base_rotation = object_model.get_articulation("base_rotation")
    trunnion_tilt = object_model.get_articulation("trunnion_tilt")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()
    ctx.allow_overlap(
        right_trunnion,
        yoke,
        reason="captured right trunnion journal sits inside the bearing housing; nested coaxial fit is intentional",
    )
    ctx.allow_overlap(
        left_trunnion,
        yoke,
        reason="captured left trunnion journal sits inside the bearing housing; nested coaxial fit is intentional",
    )

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
    ctx.fail_if_articulation_overlaps(max_pose_samples=24)

    ctx.check(
        "vertical rotary axis is +z",
        tuple(round(v, 6) for v in base_rotation.axis) == (0.0, 0.0, 1.0),
        details=f"axis={base_rotation.axis}",
    )
    ctx.check(
        "trunnion axis is +x",
        tuple(round(v, 6) for v in trunnion_tilt.axis) == (1.0, 0.0, 0.0),
        details=f"axis={trunnion_tilt.axis}",
    )
    ctx.check(
        "trunnion limits straddle level",
        trunnion_tilt.motion_limits is not None
        and trunnion_tilt.motion_limits.lower is not None
        and trunnion_tilt.motion_limits.upper is not None
        and trunnion_tilt.motion_limits.lower < 0.0 < trunnion_tilt.motion_limits.upper,
        details=f"limits={trunnion_tilt.motion_limits}",
    )

    with ctx.pose({base_rotation: 0.0, trunnion_tilt: 0.0}):
        ctx.expect_contact(yoke, base, name="yoke rides on base thrust ring")
        ctx.expect_overlap(yoke, base, axes="xy", min_overlap=0.24, name="rotary stage footprint overlap")
        ctx.expect_contact(plate, right_trunnion, name="right trunnion is mounted to plate")
        ctx.expect_contact(plate, left_trunnion, name="left trunnion is mounted to plate")
        ctx.expect_contact(right_trunnion, yoke, name="right trunnion seats in yoke bearing")
        ctx.expect_contact(left_trunnion, yoke, name="left trunnion seats in yoke bearing")
        ctx.expect_overlap(right_trunnion, yoke, axes="yz", min_overlap=0.07, name="right trunnion sits within bearing span")
        ctx.expect_overlap(left_trunnion, yoke, axes="yz", min_overlap=0.07, name="left trunnion sits within bearing span")

    with ctx.pose({trunnion_tilt: 1.20}):
        ctx.expect_gap(
            plate,
            base,
            axis="z",
            min_gap=0.08,
            name="positive tilt clears base housing",
        )

    with ctx.pose({trunnion_tilt: -1.20}):
        ctx.expect_gap(
            plate,
            base,
            axis="z",
            min_gap=0.08,
            name="negative tilt clears base housing",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
