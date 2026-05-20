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


MAST_RAIL_X = 0.18
MAST_RAIL_SIZE = (0.09, 0.06, 1.76)
LIFT_LOWER_Z = 0.14
LIFT_TRAVEL = 0.76


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _fuse_all(shapes: list[cq.Workplane]) -> cq.Workplane:
    fused = shapes[0]
    for shape in shapes[1:]:
        fused = fused.union(shape)
    return fused


def _mast_rail(center_x: float) -> cq.Workplane:
    rail = _box(MAST_RAIL_SIZE, (center_x, 0.0, 0.98))
    slot_offset = 0.020 if center_x < 0.0 else -0.020
    slot = _box((0.050, 0.038, 1.60), (center_x + slot_offset, 0.0, 1.01))
    return rail.cut(slot)


def _mast_shape() -> cq.Workplane:
    shapes = [
        _mast_rail(-MAST_RAIL_X),
        _mast_rail(MAST_RAIL_X),
        _box((0.42, 0.07, 0.12), (0.0, -0.005, 0.10)),
        _box((0.45, 0.07, 0.10), (0.0, -0.005, 1.84)),
        _box((0.12, 0.09, 0.08), (-0.23, -0.045, 0.04)),
        _box((0.12, 0.09, 0.08), (0.23, -0.045, 0.04)),
    ]

    left_gusset = (
        cq.Workplane("YZ")
        .polyline([(-0.03, 0.16), (-0.03, 0.48), (0.015, 0.82), (0.015, 0.16)])
        .close()
        .extrude(0.05)
        .translate((-0.08, -0.02, 0.0))
    )
    right_gusset = (
        cq.Workplane("YZ")
        .polyline([(-0.03, 0.16), (-0.03, 0.48), (0.015, 0.82), (0.015, 0.16)])
        .close()
        .extrude(0.05)
        .translate((0.03, -0.02, 0.0))
    )
    return _fuse_all(shapes + [left_gusset, right_gusset])


def _carriage_shape() -> cq.Workplane:
    backrest_panel = _box((0.30, 0.016, 0.52), (0.0, 0.074, 0.63))
    for x_center in (-0.09, 0.0, 0.09):
        backrest_panel = backrest_panel.cut(_box((0.055, 0.03, 0.38), (x_center, 0.074, 0.63)))

    shapes = [
        _box((0.070, 0.035, 0.36), (-0.18, 0.0475, 0.19)),
        _box((0.070, 0.035, 0.36), (0.18, 0.0475, 0.19)),
        _box((0.43, 0.045, 0.060), (0.0, 0.085, 0.09)),
        _box((0.43, 0.040, 0.055), (0.0, 0.082, 0.34)),
        _box((0.028, 0.028, 0.55), (-0.12, 0.085, 0.63)),
        _box((0.028, 0.028, 0.55), (0.12, 0.085, 0.63)),
        _box((0.30, 0.028, 0.040), (0.0, 0.085, 0.90)),
        _box((0.016, 0.016, 0.52), (-0.06, 0.085, 0.625)),
        _box((0.016, 0.016, 0.52), (0.0, 0.085, 0.625)),
        _box((0.016, 0.016, 0.52), (0.06, 0.085, 0.625)),
        _box((0.055, 0.070, 0.14), (-0.14, 0.092, 0.00)),
        _box((0.055, 0.070, 0.14), (0.14, 0.092, 0.00)),
        _box((0.070, 0.62, 0.040), (-0.14, 0.40, -0.07)),
        _box((0.070, 0.62, 0.040), (0.14, 0.40, -0.07)),
        backrest_panel,
    ]

    carriage = _fuse_all(shapes)

    fork_tip_chamfer = (
        cq.Workplane("YZ")
        .polyline([(0.0, -0.02), (0.11, -0.02), (0.11, 0.02)])
        .close()
        .extrude(0.07)
    )
    carriage = carriage.cut(fork_tip_chamfer.translate((-0.175, 0.72, -0.065)))
    carriage = carriage.cut(fork_tip_chamfer.translate((0.105, 0.72, -0.065)))
    return carriage


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="narrow_mast_fork_carriage")
    model.material("mast_gray", rgba=(0.45, 0.47, 0.50, 1.0))
    model.material("carriage_dark", rgba=(0.16, 0.17, 0.18, 1.0))

    mast = model.part("mast")
    mast.visual(
        mesh_from_cadquery(_mast_shape(), "mast_frame"),
        name="mast_frame",
        material="mast_gray",
    )

    carriage = model.part("carriage")
    carriage.visual(
        mesh_from_cadquery(_carriage_shape(), "carriage_assembly"),
        name="carriage_assembly",
        material="carriage_dark",
    )

    model.articulation(
        "mast_lift",
        ArticulationType.PRISMATIC,
        parent=mast,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, LIFT_LOWER_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=LIFT_TRAVEL,
            effort=3000.0,
            velocity=0.25,
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

    mast = object_model.get_part("mast")
    carriage = object_model.get_part("carriage")
    lift = object_model.get_articulation("mast_lift")
    lift_upper = 0.0
    if lift.motion_limits is not None and lift.motion_limits.upper is not None:
        lift_upper = lift.motion_limits.upper

    ctx.expect_contact(
        carriage,
        mast,
        contact_tol=1e-6,
        name="carriage is physically supported on the mast guides at rest",
    )
    ctx.expect_within(
        carriage,
        mast,
        axes="x",
        margin=0.015,
        name="carriage stays within the mast width",
    )

    rest_pos = ctx.part_world_position(carriage)
    with ctx.pose({lift: lift_upper}):
        raised_pos = ctx.part_world_position(carriage)
        mast_aabb = ctx.part_world_aabb(mast)
        carriage_aabb = ctx.part_world_aabb(carriage)

        ctx.expect_contact(
            carriage,
            mast,
            contact_tol=1e-6,
            name="carriage remains supported on the mast guides at full lift",
        )
        ctx.expect_within(
            carriage,
            mast,
            axes="x",
            margin=0.015,
            name="carriage stays laterally captured at full lift",
        )
        ctx.expect_overlap(
            carriage,
            mast,
            axes="z",
            min_overlap=0.20,
            name="raised carriage still overlaps the mast in guide height",
        )
        ctx.check(
            "carriage remains below the mast crown at full lift",
            mast_aabb is not None
            and carriage_aabb is not None
            and carriage_aabb[1][2] < mast_aabb[1][2] - 0.05,
            details=f"mast_aabb={mast_aabb}, carriage_aabb={carriage_aabb}",
        )

    ctx.check(
        "prismatic joint raises the carriage vertically",
        rest_pos is not None
        and raised_pos is not None
        and raised_pos[2] > rest_pos[2] + 0.70
        and abs(raised_pos[0] - rest_pos[0]) < 1e-6
        and abs(raised_pos[1] - rest_pos[1]) < 1e-6,
        details=f"rest_pos={rest_pos}, raised_pos={raised_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
