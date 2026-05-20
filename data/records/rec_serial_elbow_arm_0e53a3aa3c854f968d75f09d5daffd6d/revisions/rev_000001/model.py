from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _cylinder_y(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    x, y, z = center
    return (
        cq.Workplane("XZ")
        .center(x, z)
        .circle(radius)
        .extrude(length / 2.0, both=True)
        .translate((0.0, y, 0.0))
    )


def _cylinder_x(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    x, y, z = center
    return (
        cq.Workplane("YZ")
        .center(y, z)
        .circle(radius)
        .extrude(length / 2.0, both=True)
        .translate((x, 0.0, 0.0))
    )


def _make_base_shape() -> cq.Workplane:
    plate = _box((0.27, 0.20, 0.020), (-0.045, 0.0, -0.190))
    pedestal = _box((0.120, 0.122, 0.104), (-0.050, 0.0, -0.128))
    rear_spine = _box((0.040, 0.094, 0.082), (-0.060, 0.0, -0.042))
    left_buttress = _box((0.054, 0.024, 0.054), (-0.018, -0.047, -0.072))
    right_buttress = _box((0.054, 0.024, 0.054), (-0.018, 0.047, -0.072))
    left_cheek = _box((0.078, 0.016, 0.126), (0.004, -0.049, -0.006))
    right_cheek = _box((0.078, 0.016, 0.126), (0.004, 0.049, -0.006))
    left_cap = _cylinder_y(0.024, 0.010, (0.000, -0.057, 0.000))
    right_cap = _cylinder_y(0.024, 0.010, (0.000, 0.057, 0.000))
    left_bore = _cylinder_y(0.0102, 0.020, (0.000, -0.048, 0.000))
    right_bore = _cylinder_y(0.0102, 0.020, (0.000, 0.048, 0.000))
    shoulder_pocket = _box((0.086, 0.082, 0.132), (0.010, 0.0, -0.002))

    mount_holes = (
        cq.Workplane("XY")
        .pushPoints([(-0.120, -0.065), (-0.120, 0.065), (0.040, -0.065), (0.040, 0.065)])
        .circle(0.0065)
        .extrude(0.014, both=True)
        .translate((0.0, 0.0, -0.190))
    )

    return (
        plate.union(pedestal)
        .union(rear_spine)
        .union(left_buttress)
        .union(right_buttress)
        .union(left_cheek)
        .union(right_cheek)
        .union(left_cap)
        .union(right_cap)
        .cut(shoulder_pocket)
        .cut(left_bore)
        .cut(right_bore)
        .cut(mount_holes)
    )


def _make_upper_link_shape() -> cq.Workplane:
    shoulder_lug = _box((0.026, 0.0806, 0.056), (0.013, 0.0, 0.000))
    shoulder_bore = _cylinder_y(0.0102, 0.090, (0.000, 0.0, 0.000))

    arm_body = (
        cq.Workplane("YZ")
        .workplane(offset=0.028)
        .rect(0.038, 0.060)
        .workplane(offset=0.194)
        .rect(0.026, 0.044)
        .loft(combine=True)
    )

    elbow_bridge = _box((0.022, 0.030, 0.048), (0.236, 0.0, 0.000))
    left_plate = _box((0.060, 0.014, 0.080), (0.274, -0.030, 0.000))
    right_plate = _box((0.060, 0.014, 0.080), (0.274, 0.030, 0.000))
    left_plate_bore = _cylinder_y(0.0102, 0.016, (0.296, -0.030, 0.000))
    right_plate_bore = _cylinder_y(0.0102, 0.016, (0.296, 0.030, 0.000))

    return (
        shoulder_lug.union(arm_body)
        .union(elbow_bridge)
        .union(left_plate)
        .union(right_plate)
        .cut(shoulder_bore)
        .cut(left_plate_bore)
        .cut(right_plate_bore)
    )


def _make_tool_carrier_shape() -> cq.Workplane:
    elbow_lug = _box((0.020, 0.0446, 0.040), (0.010, 0.0, 0.000))
    elbow_bore = _cylinder_y(0.0102, 0.050, (0.000, 0.0, 0.000))

    carrier_body = (
        cq.Workplane("YZ")
        .workplane(offset=0.020)
        .rect(0.024, 0.040)
        .workplane(offset=0.072)
        .rect(0.020, 0.030)
        .loft(combine=True)
    )
    lower_gusset = _box((0.026, 0.020, 0.018), (0.078, 0.0, -0.017))
    tool_flange = _box((0.012, 0.054, 0.050), (0.106, 0.0, -0.004))

    flange_holes = _cylinder_x(0.0055, 0.018, (0.106, 0.0, -0.016)).union(
        _cylinder_x(0.0055, 0.018, (0.106, 0.0, 0.010))
    )
    return (
        elbow_lug.union(carrier_body)
        .union(lower_gusset)
        .union(tool_flange)
        .cut(elbow_bore)
        .cut(flange_holes)
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bench_manipulator_arm")

    base_color = model.material("base_color", rgba=(0.20, 0.21, 0.23, 1.0))
    link_color = model.material("link_color", rgba=(0.60, 0.62, 0.66, 1.0))
    tool_color = model.material("tool_color", rgba=(0.32, 0.34, 0.37, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_make_base_shape(), "base_yoke"),
        origin=Origin(),
        material=base_color,
        name="base_shell",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.26, 0.19, 0.20)),
        mass=8.2,
        origin=Origin(xyz=(-0.04, 0.0, -0.11)),
    )

    upper_link = model.part("upper_link")
    upper_link.visual(
        mesh_from_cadquery(_make_upper_link_shape(), "upper_link"),
        origin=Origin(),
        material=link_color,
        name="upper_link_shell",
    )
    upper_link.inertial = Inertial.from_geometry(
        Box((0.35, 0.13, 0.10)),
        mass=2.5,
        origin=Origin(xyz=(0.17, 0.0, 0.0)),
    )

    tool_carrier = model.part("tool_carrier")
    tool_carrier.visual(
        mesh_from_cadquery(_make_tool_carrier_shape(), "tool_carrier"),
        origin=Origin(),
        material=tool_color,
        name="tool_carrier_shell",
    )
    tool_carrier.inertial = Inertial.from_geometry(
        Box((0.14, 0.09, 0.07)),
        mass=0.95,
        origin=Origin(xyz=(0.07, 0.0, -0.004)),
    )

    model.articulation(
        "shoulder",
        ArticulationType.REVOLUTE,
        parent=base,
        child=upper_link,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=60.0, velocity=1.2, lower=-0.70, upper=0.60),
    )

    model.articulation(
        "elbow",
        ArticulationType.REVOLUTE,
        parent=upper_link,
        child=tool_carrier,
        origin=Origin(xyz=(0.296, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.5, lower=-1.05, upper=1.00),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    upper_link = object_model.get_part("upper_link")
    tool_carrier = object_model.get_part("tool_carrier")
    shoulder = object_model.get_articulation("shoulder")
    elbow = object_model.get_articulation("elbow")

    ctx.check_model_valid()
    ctx.check_mesh_assets_ready()

    # Preferred default QC stack:
    # 1) likely-failure grounded-component floating check for disconnected part groups
    ctx.fail_if_isolated_parts(contact_tol=0.001)
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
    ctx.fail_if_articulation_overlaps(max_pose_samples=24, name="articulation_clearance_sweep")

    ctx.check(
        "parallel_hinge_axes",
        tuple(shoulder.axis) == tuple(elbow.axis),
        details=f"shoulder axis={shoulder.axis}, elbow axis={elbow.axis}",
    )
    ctx.expect_contact(base, upper_link, contact_tol=0.001, name="shoulder_journals_are_supported")
    ctx.expect_contact(upper_link, tool_carrier, contact_tol=0.001, name="tool_carrier_supported_on_elbow_journals")

    with ctx.pose({shoulder: shoulder.motion_limits.upper, elbow: elbow.motion_limits.lower}):
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_in_raised_folded_pose")

    with ctx.pose({shoulder: shoulder.motion_limits.lower, elbow: elbow.motion_limits.upper}):
        ctx.fail_if_parts_overlap_in_current_pose(name="no_overlap_in_lowered_open_pose")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
