from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
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


GUIDE_LENGTH = 2.20
GUIDE_CENTER_Y = 0.60
GUIDE_BODY_W = 0.16
GUIDE_BODY_H = 0.14
GUIDE_CAP_W = 0.06
GUIDE_CAP_H = 0.018
GUIDE_TOP_Z = GUIDE_BODY_H + GUIDE_CAP_H

TIE_X = 0.86
TIE_SIZE = (0.20, (2.0 * GUIDE_CENTER_Y) - GUIDE_BODY_W, 0.10)
FOOT_RADIUS = 0.045
FOOT_HEIGHT = 0.05

SHOE_SIZE = (0.28, 0.22, 0.08)
SHOE_CENTER_Z = GUIDE_TOP_Z + (SHOE_SIZE[2] / 2.0)

COLUMN_SIZE = (0.16, 0.08, 1.27)
COLUMN_BOTTOM_Z = SHOE_CENTER_Z + (SHOE_SIZE[2] / 2.0)
COLUMN_TOP_Z = COLUMN_BOTTOM_Z + COLUMN_SIZE[2]
COLUMN_CENTER_Z = COLUMN_BOTTOM_Z + (COLUMN_SIZE[2] / 2.0)

BEAM_SIZE = (0.20, 1.36, 0.18)
BEAM_CENTER_Z = COLUMN_TOP_Z + (BEAM_SIZE[2] / 2.0)
BEAM_TOP_Z = BEAM_CENTER_Z + (BEAM_SIZE[2] / 2.0)

TRUCK_BODY_SIZE = (0.22, 0.26, 0.10)
TRUCK_BODY_CENTER_Z_REL = (BEAM_SIZE[2] / 2.0) + (TRUCK_BODY_SIZE[2] / 2.0)
TRUCK_CHEEK_SIZE = (0.03, 0.20, 0.12)
TRUCK_NECK_SIZE = (0.07, 0.12, 0.16)
TRUCK_POD_SIZE = (0.10, 0.14, 0.20)
TRUCK_SENSOR_RADIUS = 0.025
TRUCK_SENSOR_LENGTH = 0.04

CROSSBEAM_TRAVEL = 0.66
TRUCK_TRAVEL = 0.42


def _add_box(
    part,
    size: tuple[float, float, float],
    xyz: tuple[float, float, float],
    *,
    material: str,
    name: str | None = None,
) -> None:
    part.visual(
        Box(size),
        origin=Origin(xyz=xyz),
        material=material,
        name=name,
    )


def _add_cylinder(
    part,
    radius: float,
    length: float,
    xyz: tuple[float, float, float],
    *,
    material: str,
    name: str | None = None,
) -> None:
    part.visual(
        Cylinder(radius=radius, length=length),
        origin=Origin(xyz=xyz),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="inspection_gantry")

    model.material("painted_frame", rgba=(0.84, 0.86, 0.88, 1.0))
    model.material("rail_steel", rgba=(0.57, 0.60, 0.65, 1.0))
    model.material("carriage_dark", rgba=(0.18, 0.20, 0.23, 1.0))
    model.material("sensor_blue", rgba=(0.22, 0.46, 0.78, 1.0))
    model.material("foot_black", rgba=(0.09, 0.10, 0.12, 1.0))

    base = model.part("base")
    _add_box(
        base,
        (GUIDE_LENGTH, GUIDE_BODY_W, GUIDE_BODY_H),
        (0.0, GUIDE_CENTER_Y, GUIDE_BODY_H / 2.0),
        material="painted_frame",
        name="left_guide_body",
    )
    _add_box(
        base,
        (GUIDE_LENGTH, GUIDE_BODY_W, GUIDE_BODY_H),
        (0.0, -GUIDE_CENTER_Y, GUIDE_BODY_H / 2.0),
        material="painted_frame",
        name="right_guide_body",
    )
    _add_box(
        base,
        (GUIDE_LENGTH - 0.14, GUIDE_CAP_W, GUIDE_CAP_H),
        (0.0, GUIDE_CENTER_Y, GUIDE_BODY_H + (GUIDE_CAP_H / 2.0)),
        material="rail_steel",
        name="left_guide_cap",
    )
    _add_box(
        base,
        (GUIDE_LENGTH - 0.14, GUIDE_CAP_W, GUIDE_CAP_H),
        (0.0, -GUIDE_CENTER_Y, GUIDE_BODY_H + (GUIDE_CAP_H / 2.0)),
        material="rail_steel",
        name="right_guide_cap",
    )
    _add_box(
        base,
        TIE_SIZE,
        (TIE_X, 0.0, TIE_SIZE[2] / 2.0),
        material="painted_frame",
        name="front_tie",
    )
    _add_box(
        base,
        TIE_SIZE,
        (-TIE_X, 0.0, TIE_SIZE[2] / 2.0),
        material="painted_frame",
        name="rear_tie",
    )
    for x in (-0.92, 0.92):
        for y in (-GUIDE_CENTER_Y, GUIDE_CENTER_Y):
            _add_cylinder(
                base,
                FOOT_RADIUS,
                FOOT_HEIGHT,
                (x, y, -FOOT_HEIGHT / 2.0),
                material="foot_black",
            )
    base.inertial = Inertial.from_geometry(
        Box((GUIDE_LENGTH, (2.0 * GUIDE_CENTER_Y) + GUIDE_BODY_W, GUIDE_TOP_Z + FOOT_HEIGHT)),
        mass=260.0,
        origin=Origin(xyz=(0.0, 0.0, (GUIDE_TOP_Z - FOOT_HEIGHT) / 2.0)),
    )

    crossbeam = model.part("crossbeam")
    _add_box(
        crossbeam,
        SHOE_SIZE,
        (0.0, GUIDE_CENTER_Y, SHOE_CENTER_Z - BEAM_CENTER_Z),
        material="carriage_dark",
        name="left_shoe",
    )
    _add_box(
        crossbeam,
        SHOE_SIZE,
        (0.0, -GUIDE_CENTER_Y, SHOE_CENTER_Z - BEAM_CENTER_Z),
        material="carriage_dark",
        name="right_shoe",
    )
    _add_box(
        crossbeam,
        COLUMN_SIZE,
        (0.0, GUIDE_CENTER_Y, COLUMN_CENTER_Z - BEAM_CENTER_Z),
        material="painted_frame",
        name="left_end_support",
    )
    _add_box(
        crossbeam,
        COLUMN_SIZE,
        (0.0, -GUIDE_CENTER_Y, COLUMN_CENTER_Z - BEAM_CENTER_Z),
        material="painted_frame",
        name="right_end_support",
    )
    _add_box(
        crossbeam,
        BEAM_SIZE,
        (0.0, 0.0, 0.0),
        material="painted_frame",
        name="crossbeam_body",
    )
    crossbeam.inertial = Inertial.from_geometry(
        Box((SHOE_SIZE[0], BEAM_SIZE[1], BEAM_TOP_Z - GUIDE_TOP_Z)),
        mass=145.0,
        origin=Origin(xyz=(0.0, 0.0, ((GUIDE_TOP_Z + BEAM_TOP_Z) / 2.0) - BEAM_CENTER_Z)),
    )

    truck = model.part("center_truck")
    _add_box(
        truck,
        TRUCK_BODY_SIZE,
        (0.0, 0.0, 0.0),
        material="carriage_dark",
        name="truck_body",
    )
    _add_box(
        truck,
        TRUCK_CHEEK_SIZE,
        (0.125, 0.0, -0.01),
        material="carriage_dark",
        name="left_cheek",
    )
    _add_box(
        truck,
        TRUCK_CHEEK_SIZE,
        (-0.125, 0.0, -0.01),
        material="carriage_dark",
        name="right_cheek",
    )
    _add_box(
        truck,
        TRUCK_NECK_SIZE,
        (-0.136, 0.0, -0.10),
        material="sensor_blue",
        name="sensor_neck",
    )
    _add_box(
        truck,
        TRUCK_POD_SIZE,
        (-0.15, 0.0, -0.28),
        material="sensor_blue",
        name="sensor_pod",
    )
    _add_cylinder(
        truck,
        TRUCK_SENSOR_RADIUS,
        TRUCK_SENSOR_LENGTH,
        (-0.15, 0.0, -0.40),
        material="rail_steel",
        name="sensor_lens",
    )
    truck.inertial = Inertial.from_geometry(
        Box((0.22, 0.26, 0.50)),
        mass=24.0,
        origin=Origin(xyz=(-0.05, 0.0, -0.15)),
    )

    model.articulation(
        "base_to_crossbeam",
        ArticulationType.PRISMATIC,
        parent=base,
        child=crossbeam,
        origin=Origin(xyz=(0.0, 0.0, BEAM_CENTER_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=-CROSSBEAM_TRAVEL,
            upper=CROSSBEAM_TRAVEL,
            effort=900.0,
            velocity=0.55,
        ),
    )
    model.articulation(
        "crossbeam_to_truck",
        ArticulationType.PRISMATIC,
        parent=crossbeam,
        child=truck,
        origin=Origin(xyz=(0.0, 0.0, TRUCK_BODY_CENTER_Z_REL)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            lower=-TRUCK_TRAVEL,
            upper=TRUCK_TRAVEL,
            effort=300.0,
            velocity=0.70,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    # `compile_model` automatically runs baseline sanity/QC:
    # - `check_model_valid()`
    # - exactly one root part
    # - `check_mesh_assets_ready()`
    # - disconnected floating-part-group detection
    # - disconnected within-part geometry-island detection
    # - current-pose real 3D overlap detection
    # Use `run_tests()` only for prompt-specific exact checks, targeted poses,
    # and explicit allowances such as `ctx.allow_overlap(...)`.

    base = object_model.get_part("base")
    crossbeam = object_model.get_part("crossbeam")
    truck = object_model.get_part("center_truck")
    base_to_crossbeam = object_model.get_articulation("base_to_crossbeam")
    crossbeam_to_truck = object_model.get_articulation("crossbeam_to_truck")

    ctx.check(
        "three primary parts exist",
        all(part is not None for part in (base, crossbeam, truck)),
        details="Missing one or more gantry parts.",
    )
    ctx.check(
        "crossbeam slides along +X",
        tuple(base_to_crossbeam.axis) == (1.0, 0.0, 0.0),
        details=f"axis={base_to_crossbeam.axis}",
    )
    ctx.check(
        "truck slides across beam along +Y",
        tuple(crossbeam_to_truck.axis) == (0.0, 1.0, 0.0),
        details=f"axis={crossbeam_to_truck.axis}",
    )

    ctx.expect_contact(
        crossbeam,
        base,
        name="crossbeam is supported on the twin base guides",
    )
    ctx.expect_contact(
        truck,
        crossbeam,
        name="center truck is supported by the crossbeam",
    )

    crossbeam_rest = ctx.part_world_position(crossbeam)
    with ctx.pose({base_to_crossbeam: CROSSBEAM_TRAVEL}):
        ctx.expect_contact(
            crossbeam,
            base,
            name="crossbeam stays supported at maximum base travel",
        )
        crossbeam_forward = ctx.part_world_position(crossbeam)

    truck_rest = ctx.part_world_position(truck)
    with ctx.pose({crossbeam_to_truck: TRUCK_TRAVEL}):
        ctx.expect_contact(
            truck,
            crossbeam,
            name="truck stays supported at maximum beam travel",
        )
        truck_outboard = ctx.part_world_position(truck)

    ctx.check(
        "crossbeam translates along the base guides",
        crossbeam_rest is not None
        and crossbeam_forward is not None
        and crossbeam_forward[0] > crossbeam_rest[0] + 0.40,
        details=f"rest={crossbeam_rest}, moved={crossbeam_forward}",
    )
    ctx.check(
        "truck translates across the beam",
        truck_rest is not None
        and truck_outboard is not None
        and truck_outboard[1] > truck_rest[1] + 0.25,
        details=f"rest={truck_rest}, moved={truck_outboard}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
