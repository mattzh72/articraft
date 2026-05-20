from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


RAIL_L = 0.34
BASE_W = 0.082
BASE_T = 0.014
GUIDE_L = 0.24
GUIDE_W = 0.016
GUIDE_H = 0.012
GUIDE_Y = 0.022
MOTOR_L = 0.058
MOTOR_H = 0.034
STOP_L = 0.018
STOP_W = 0.050
STOP_H = 0.020

CAR_L = 0.108
CAR_W = 0.100
CAR_H = 0.044
CAR_REST_X = -0.020
CAR_TRAVEL = 0.090

HINGE_X = 0.064
HINGE_Z = 0.038

FRAME_OUT_W = 0.034
FRAME_SIDE_T = 0.009
FRAME_H = 0.034
FRAME_L = 0.096
BARREL_R = 0.007
BARREL_L = 0.020

RAM_L = 0.060
RAM_W = 0.022
RAM_H = 0.016
RAM_Z = -0.001
RAM_TRAVEL = 0.035


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _y_cylinder(radius: float, length: float, center: tuple[float, float, float]) -> cq.Workplane:
    return (
        cq.Workplane("XZ")
        .circle(radius)
        .extrude(length / 2.0, both=True)
        .translate(center)
    )


def _make_rail_shape() -> cq.Workplane:
    rail = _box((RAIL_L, BASE_W, BASE_T), (0.0, 0.0, BASE_T / 2.0))

    for y in (-GUIDE_Y, GUIDE_Y):
        rail = rail.union(
            _box((GUIDE_L, GUIDE_W, GUIDE_H), (0.0, y, BASE_T + GUIDE_H / 2.0))
        )

    rail = rail.union(
        _box(
            (MOTOR_L, BASE_W, MOTOR_H),
            (-RAIL_L / 2.0 + MOTOR_L / 2.0, 0.0, BASE_T + MOTOR_H / 2.0),
        )
    )
    rail = rail.union(
        _box(
            (STOP_L, STOP_W, STOP_H),
            (RAIL_L / 2.0 - STOP_L / 2.0, 0.0, BASE_T + STOP_H / 2.0),
        )
    )
    rail = rail.union(
        _box(
            (0.030, 0.034, 0.010),
            (-RAIL_L / 2.0 + MOTOR_L + 0.015, 0.0, BASE_T + 0.005),
        )
    )
    return rail


def _make_carriage_shape() -> cq.Workplane:
    carriage = _box((0.090, 0.014, 0.010), (-0.006, GUIDE_Y, 0.005))
    carriage = carriage.union(_box((0.090, 0.014, 0.010), (-0.006, -GUIDE_Y, 0.005)))
    carriage = carriage.union(_box((0.090, 0.042, 0.012), (-0.004, 0.0, 0.0155)))
    carriage = carriage.union(_box((0.030, 0.054, 0.010), (-0.046, 0.0, 0.018)))
    carriage = carriage.union(_box((0.028, 0.026, 0.014), (0.042, 0.0, 0.026)))
    carriage = carriage.union(_box((0.010, 0.008, 0.018), (HINGE_X - 0.005, 0.014, HINGE_Z)))
    carriage = carriage.union(_box((0.010, 0.008, 0.018), (HINGE_X - 0.005, -0.014, HINGE_Z)))
    return carriage


def _make_frame_shape() -> cq.Workplane:
    frame = _y_cylinder(BARREL_R, BARREL_L, (0.0, 0.0, 0.0))
    frame = frame.union(_box((0.014, 0.014, 0.014), (0.007, 0.0, 0.0)))
    frame = frame.union(_box((0.010, 0.020, 0.010), (0.015, 0.0, -0.006)))
    frame = frame.union(_box((0.074, 0.005, 0.024), (0.050, 0.0145, 0.0)))
    frame = frame.union(_box((0.074, 0.005, 0.024), (0.050, -0.0145, 0.0)))
    frame = frame.union(_box((0.070, 0.022, 0.004), (0.050, 0.0, -0.010)))
    frame = frame.union(_box((0.012, 0.034, 0.020), (0.088, 0.0, 0.0)))
    return frame


def _make_ram_shape() -> cq.Workplane:
    return _box((RAM_L, RAM_W, RAM_H), (RAM_L / 2.0, 0.0, RAM_Z))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="service_axis")

    model.material("rail_gray", rgba=(0.28, 0.30, 0.33, 1.0))
    model.material("carriage_gray", rgba=(0.70, 0.72, 0.74, 1.0))
    model.material("frame_blue", rgba=(0.25, 0.34, 0.44, 1.0))
    model.material("ram_black", rgba=(0.16, 0.17, 0.18, 1.0))

    rail = model.part("rail")
    rail.visual(
        mesh_from_cadquery(_make_rail_shape(), "rail"),
        material="rail_gray",
        name="rail_shell",
    )

    carriage = model.part("carriage")
    carriage.visual(
        mesh_from_cadquery(_make_carriage_shape(), "carriage"),
        material="carriage_gray",
        name="carriage_shell",
    )

    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(_make_frame_shape(), "frame"),
        material="frame_blue",
        name="frame_shell",
    )

    ram = model.part("ram")
    ram.visual(
        mesh_from_cadquery(_make_ram_shape(), "ram"),
        material="ram_black",
        name="ram_shell",
    )

    rail_to_carriage = model.articulation(
        "rail_to_carriage",
        ArticulationType.PRISMATIC,
        parent=rail,
        child=carriage,
        origin=Origin(
            xyz=(
                CAR_REST_X,
                0.0,
                BASE_T + GUIDE_H,
            )
        ),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=0.30,
            lower=0.0,
            upper=CAR_TRAVEL,
        ),
    )

    carriage_to_frame = model.articulation(
        "carriage_to_frame",
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=frame,
        origin=Origin(xyz=(HINGE_X, 0.0, HINGE_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=24.0,
            velocity=1.8,
            lower=-0.25,
            upper=1.05,
        ),
    )

    model.articulation(
        "frame_to_ram",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=ram,
        origin=Origin(xyz=(0.045, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=80.0,
            velocity=0.18,
            lower=0.0,
            upper=RAM_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    rail = object_model.get_part("rail")
    carriage = object_model.get_part("carriage")
    frame = object_model.get_part("frame")
    ram = object_model.get_part("ram")

    rail_to_carriage = object_model.get_articulation("rail_to_carriage")
    carriage_to_frame = object_model.get_articulation("carriage_to_frame")
    frame_to_ram = object_model.get_articulation("frame_to_ram")

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
        carriage,
        frame,
        reason="simplified clevis hinge omits a separate pin and uses a shared compact hinge envelope",
    )
    ctx.allow_overlap(
        frame,
        ram,
        reason="ram is represented as a close-running telescoping fit inside the compact nose guide without explicit machining clearance",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(carriage, rail, name="carriage rides on rail guides")
    ctx.expect_contact(frame, carriage, name="frame hinge is mounted in carriage clevis")
    ctx.expect_contact(ram, frame, name="ram is supported by frame slide deck")
    ctx.expect_within(ram, frame, axes="yz", margin=0.001, name="ram stays captured in frame section")

    with ctx.pose({rail_to_carriage: rail_to_carriage.motion_limits.lower}):
        x_retracted = ctx.part_world_position(carriage)[0]
    with ctx.pose({rail_to_carriage: rail_to_carriage.motion_limits.upper}):
        x_extended = ctx.part_world_position(carriage)[0]
    ctx.check(
        "carriage prismatic moves forward",
        x_extended > x_retracted + 0.08,
        f"expected carriage x to advance, got {x_retracted:.4f} -> {x_extended:.4f}",
    )

    with ctx.pose({carriage_to_frame: 0.0}):
        z_closed = ctx.part_world_aabb(frame)[1][2]
    with ctx.pose({carriage_to_frame: carriage_to_frame.motion_limits.upper}):
        z_open = ctx.part_world_aabb(frame)[1][2]
    ctx.check(
        "frame revolute lifts nose upward",
        z_open > z_closed + 0.03,
        f"expected frame max z to rise, got {z_closed:.4f} -> {z_open:.4f}",
    )

    with ctx.pose({frame_to_ram: frame_to_ram.motion_limits.lower}):
        ram_x_retracted = ctx.part_world_position(ram)[0]
    with ctx.pose({frame_to_ram: frame_to_ram.motion_limits.upper}):
        ram_x_extended = ctx.part_world_position(ram)[0]
    ctx.check(
        "ram prismatic extends outward",
        ram_x_extended > ram_x_retracted + 0.02,
        f"expected ram x to advance, got {ram_x_retracted:.4f} -> {ram_x_extended:.4f}",
    )

    with ctx.pose(
        {
            rail_to_carriage: rail_to_carriage.motion_limits.upper,
            carriage_to_frame: carriage_to_frame.motion_limits.upper,
            frame_to_ram: frame_to_ram.motion_limits.upper,
        }
    ):
        ctx.fail_if_parts_overlap_in_current_pose(name="no overlaps in fully deployed pose")

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
