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


SHOULDER_AXIS_Z = 0.285
UPPER_ARM_LENGTH = 0.31
FOREARM_MOUNT_X = 0.258
SHOULDER_HUB_SPAN = 0.044
ELBOW_HUB_SPAN = 0.038


def _fuse_all(solids: list[cq.Workplane]) -> cq.Workplane:
    fused = solids[0]
    for solid in solids[1:]:
        fused = fused.union(solid)
    return fused.clean()


def _y_cylinder(radius: float, length: float, center_xyz: tuple[float, float, float]) -> cq.Workplane:
    x, y, z = center_xyz
    solid = cq.Solid.makeCylinder(
        radius,
        length,
        cq.Vector(x, y - length / 2.0, z),
        cq.Vector(0.0, 1.0, 0.0),
    )
    return cq.Workplane(obj=solid)


def _filleted_box(
    size_xyz: tuple[float, float, float],
    center_xyz: tuple[float, float, float],
    *,
    edge_selector: str,
    radius: float,
) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(*size_xyz)
        .edges(edge_selector)
        .fillet(radius)
        .translate(center_xyz)
    )


def _clevis_block(
    body_size_xyz: tuple[float, float, float],
    body_center_xyz: tuple[float, float, float],
    slot_size_xyz: tuple[float, float, float],
    slot_center_xyz: tuple[float, float, float],
) -> cq.Workplane:
    body = cq.Workplane("XY").box(*body_size_xyz).translate(body_center_xyz)
    slot = cq.Workplane("XY").box(*slot_size_xyz).translate(slot_center_xyz)
    return body.cut(slot).clean()


def _make_base_shape() -> cq.Workplane:
    foot = _filleted_box((0.26, 0.18, 0.02), (0.0, 0.0, 0.01), edge_selector="|Z", radius=0.008)
    column = _filleted_box((0.08, 0.09, 0.18), (-0.075, 0.0, 0.11), edge_selector="|Z", radius=0.012)
    shoulder_housing = _filleted_box((0.065, 0.08, 0.09), (-0.045, 0.0, 0.235), edge_selector="|Z", radius=0.010)
    finger_pos = cq.Workplane("XY").box(0.036, 0.012, 0.07).translate((-0.022, 0.022, SHOULDER_AXIS_Z))
    finger_neg = cq.Workplane("XY").box(0.036, 0.012, 0.07).translate((-0.022, -0.022, SHOULDER_AXIS_Z))
    return _fuse_all([foot, column, shoulder_housing, finger_pos, finger_neg])


def _make_upper_arm_shape() -> cq.Workplane:
    shoulder_hub = _y_cylinder(0.014, 0.032, (0.0, 0.0, 0.0))
    shoulder_neck = _filleted_box((0.06, 0.028, 0.046), (0.038, 0.0, 0.0), edge_selector="|X", radius=0.008)
    beam = _filleted_box((0.19, 0.046, 0.05), (0.157, 0.0, 0.0), edge_selector="|X", radius=0.010)
    elbow_block = _filleted_box((0.06, 0.05, 0.07), (0.282, 0.0, 0.0), edge_selector="|X", radius=0.008)
    finger_pos = cq.Workplane("XY").box(0.028, 0.012, 0.07).translate((0.304, 0.022, 0.0))
    finger_neg = cq.Workplane("XY").box(0.028, 0.012, 0.07).translate((0.304, -0.022, 0.0))
    return _fuse_all([shoulder_hub, shoulder_neck, beam, elbow_block, finger_pos, finger_neg])


def _make_forearm_shape() -> cq.Workplane:
    elbow_hub = _y_cylinder(0.012, 0.032, (0.0, 0.0, 0.0))
    root = _filleted_box((0.05, 0.026, 0.044), (0.032, 0.0, 0.0), edge_selector="|X", radius=0.007)
    beam = _filleted_box((0.13, 0.042, 0.044), (0.12, 0.0, 0.0), edge_selector="|X", radius=0.008)
    wrist_block = _filleted_box((0.07, 0.048, 0.056), (0.217, 0.0, 0.0), edge_selector="|X", radius=0.009)
    mount_pad = cq.Workplane("XY").box(0.008, 0.036, 0.036).translate((0.254, 0.0, 0.0))
    return _fuse_all([elbow_hub, root, beam, wrist_block, mount_pad])


def _make_end_plate_shape() -> cq.Workplane:
    plate = (
        cq.Workplane("YZ")
        .rect(0.082, 0.082)
        .extrude(0.016)
        .edges("|X")
        .fillet(0.007)
    )
    plate = plate.faces(">X").workplane(centerOption="CenterOfMass").circle(0.015).extrude(0.008)
    plate = plate.faces(">X").workplane(centerOption="CenterOfMass").circle(0.011).cutThruAll()
    plate = (
        plate.faces(">X")
        .workplane(centerOption="CenterOfMass")
        .pushPoints([(0.02, 0.02), (-0.02, 0.02), (0.02, -0.02), (-0.02, -0.02)])
        .circle(0.004)
        .cutThruAll()
    )
    return plate.clean()


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="serial_elbow_arm_module")

    base_gray = model.material("base_gray", color=(0.22, 0.23, 0.25))
    arm_gray = model.material("arm_gray", color=(0.70, 0.72, 0.75))
    forearm_gray = model.material("forearm_gray", color=(0.60, 0.63, 0.67))
    plate_black = model.material("plate_black", color=(0.14, 0.14, 0.15))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_make_base_shape(), "base_pedestal"),
        material=base_gray,
        name="pedestal",
    )
    base.inertial = Inertial.from_geometry(
        Box((0.24, 0.20, 0.31)),
        mass=14.0,
        origin=Origin(xyz=(0.0, 0.0, 0.155)),
    )

    upper_arm = model.part("upper_arm")
    upper_arm.visual(
        mesh_from_cadquery(_make_upper_arm_shape(), "upper_arm_link"),
        material=arm_gray,
        name="upper_arm_body",
    )
    upper_arm.inertial = Inertial.from_geometry(
        Box((0.35, 0.08, 0.09)),
        mass=4.2,
        origin=Origin(xyz=(0.175, 0.0, 0.0)),
    )

    forearm = model.part("forearm")
    forearm.visual(
        mesh_from_cadquery(_make_forearm_shape(), "forearm_link"),
        material=forearm_gray,
        name="forearm_body",
    )
    forearm.inertial = Inertial.from_geometry(
        Box((0.27, 0.07, 0.08)),
        mass=2.8,
        origin=Origin(xyz=(0.135, 0.0, 0.0)),
    )

    end_plate = model.part("end_plate")
    end_plate.visual(
        mesh_from_cadquery(_make_end_plate_shape(), "tool_end_plate"),
        material=plate_black,
        name="tool_plate",
    )
    end_plate.inertial = Inertial.from_geometry(
        Box((0.032, 0.09, 0.09)),
        mass=0.9,
        origin=Origin(xyz=(0.016, 0.0, 0.0)),
    )

    model.articulation(
        "shoulder",
        ArticulationType.REVOLUTE,
        parent=base,
        child=upper_arm,
        origin=Origin(xyz=(0.0, 0.0, SHOULDER_AXIS_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=1.8, lower=-1.0, upper=1.35),
    )
    model.articulation(
        "elbow",
        ArticulationType.REVOLUTE,
        parent=upper_arm,
        child=forearm,
        origin=Origin(xyz=(UPPER_ARM_LENGTH, 0.0, 0.0)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=90.0, velocity=2.2, lower=0.0, upper=2.25),
    )
    model.articulation(
        "forearm_to_end_plate",
        ArticulationType.FIXED,
        parent=forearm,
        child=end_plate,
        origin=Origin(xyz=(FOREARM_MOUNT_X, 0.0, 0.0)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    upper_arm = object_model.get_part("upper_arm")
    forearm = object_model.get_part("forearm")
    end_plate = object_model.get_part("end_plate")
    shoulder = object_model.get_articulation("shoulder")
    elbow = object_model.get_articulation("elbow")

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
    ctx.allow_overlap(
        base,
        upper_arm,
        reason="Shoulder joint is modeled as an interleaved clevis-and-hub envelope without a separate pin part.",
    )
    ctx.allow_overlap(
        upper_arm,
        forearm,
        reason="Elbow joint is modeled as an interleaved clevis-and-hub envelope without a separate pin part.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(base, upper_arm, contact_tol=5e-4, name="shoulder hub is carried by the base")
    ctx.expect_contact(upper_arm, forearm, contact_tol=5e-4, name="elbow hub is carried by the upper arm")
    ctx.expect_contact(forearm, end_plate, contact_tol=5e-4, name="end plate is mounted to the forearm")

    axes_ok = shoulder.axis == elbow.axis == (0.0, -1.0, 0.0)
    types_ok = (
        shoulder.articulation_type == ArticulationType.REVOLUTE
        and elbow.articulation_type == ArticulationType.REVOLUTE
    )
    ctx.check(
        "shoulder and elbow are parallel horizontal revolutes",
        axes_ok and types_ok,
        details=f"shoulder axis={shoulder.axis}, elbow axis={elbow.axis}",
    )

    with ctx.pose({shoulder: 0.6, elbow: 0.0}):
        end_plate_pos = ctx.part_world_position(end_plate)
        ctx.check(
            "positive shoulder rotation lifts the chain",
            end_plate_pos is not None and end_plate_pos[2] > SHOULDER_AXIS_Z + 0.12,
            details=f"end plate position in shoulder-lift pose: {end_plate_pos}",
        )

    with ctx.pose({shoulder: 0.0, elbow: 1.0}):
        end_plate_pos = ctx.part_world_position(end_plate)
        forearm_origin = ctx.part_world_position(forearm)
        ctx.check(
            "positive elbow rotation lifts the end plate above the elbow axis",
            end_plate_pos is not None
            and forearm_origin is not None
            and end_plate_pos[2] > forearm_origin[2] + 0.12,
            details=f"forearm origin={forearm_origin}, end plate position={end_plate_pos}",
        )

    with ctx.pose({shoulder: 0.55, elbow: 1.15}):
        ctx.fail_if_parts_overlap_in_current_pose(name="no part overlap in a lifted working pose")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
