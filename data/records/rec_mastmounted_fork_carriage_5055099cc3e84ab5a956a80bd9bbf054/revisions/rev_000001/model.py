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


MAST_REAR_BASE_SIZE = (0.92, 0.20, 0.12)
MAST_FRONT_FOOT_SIZE = (0.22, 0.06, 0.12)
MAST_RAIL_SIZE = (0.12, 0.08, 1.94)
RAIL_CENTER_X = 0.27
RAIL_FRONT_Y = MAST_RAIL_SIZE[1] / 2.0
RAIL_BASE_Z = 0.15
MAST_TOP_Z = RAIL_BASE_Z + MAST_RAIL_SIZE[2]

TOP_TIE_SIZE = (0.66, 0.08, 0.10)
MID_TIE_SIZE = (0.52, 0.08, 0.08)
REAR_SPINE_SIZE = (0.14, 0.12, 1.78)
RAIL_PEDESTAL_SIZE = (0.18, 0.08, 0.22)

CARRIAGE_LOWER_BEAM_SIZE = (0.74, 0.08, 0.12)
CARRIAGE_UPPER_BEAM_SIZE = (0.74, 0.08, 0.10)
CARRIAGE_SIDE_PLATE_SIZE = (0.10, 0.03, 0.52)
CARRIAGE_CENTER_PLATE_SIZE = (0.56, 0.02, 0.30)
CARRIAGE_CENTER_BRACE_SIZE = (0.12, 0.04, 0.42)
BACKREST_BAR_SIZE = (0.05, 0.03, 0.50)
BACKREST_TOP_SIZE = (0.58, 0.03, 0.04)
BACKREST_MID_SIZE = (0.46, 0.03, 0.04)
GUIDE_SHOE_SIZE = (0.03, 0.025, 0.14)
GUIDE_POST_SIZE = (0.05, 0.06, 0.56)
FORK_WIDTH = 0.09
FORK_Y_SHIFT = 0.08
CARRIAGE_BODY_Y = 0.15

CARRIAGE_REST_Z = 0.36
LIFT_TRAVEL = 0.75


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _build_mast_shape() -> cq.Workplane:
    mast = _box(MAST_REAR_BASE_SIZE, (0.0, -0.10, MAST_REAR_BASE_SIZE[2] / 2.0))

    for x_center in (-0.31, 0.31):
        mast = mast.union(
            _box(
                MAST_FRONT_FOOT_SIZE,
                (x_center, -0.01, MAST_FRONT_FOOT_SIZE[2] / 2.0),
            )
        )

    for x_center in (-RAIL_CENTER_X, RAIL_CENTER_X):
        mast = mast.union(
            _box(
                RAIL_PEDESTAL_SIZE,
                (x_center, -0.02, MAST_REAR_BASE_SIZE[2] + (RAIL_PEDESTAL_SIZE[2] / 2.0)),
            )
        )
        mast = mast.union(
            _box(
                MAST_RAIL_SIZE,
                (x_center, 0.0, RAIL_BASE_Z + (MAST_RAIL_SIZE[2] / 2.0)),
            )
        )

    mast = mast.union(_box(TOP_TIE_SIZE, (0.0, -0.02, MAST_TOP_Z + (TOP_TIE_SIZE[2] / 2.0))))
    mast = mast.union(_box(MID_TIE_SIZE, (0.0, -0.03, 1.14)))
    mast = mast.union(
        _box(
            REAR_SPINE_SIZE,
            (0.0, -0.06, MAST_REAR_BASE_SIZE[2] + (REAR_SPINE_SIZE[2] / 2.0)),
        )
    )

    return mast


def _build_fork_solid(x_center: float) -> cq.Workplane:
    fork_profile = [
        (0.06 + FORK_Y_SHIFT, 0.18),
        (0.10 + FORK_Y_SHIFT, 0.18),
        (0.10 + FORK_Y_SHIFT, -0.235),
        (0.88 + FORK_Y_SHIFT, -0.235),
        (1.03 + FORK_Y_SHIFT, -0.255),
        (1.03 + FORK_Y_SHIFT, -0.28),
        (0.10 + FORK_Y_SHIFT, -0.28),
        (0.06 + FORK_Y_SHIFT, -0.28),
    ]
    return (
        cq.Workplane("YZ")
        .polyline(fork_profile)
        .close()
        .extrude(FORK_WIDTH)
        .translate((x_center - (FORK_WIDTH / 2.0), 0.0, 0.0))
    )


def _build_carriage_shape() -> cq.Workplane:
    carriage = _box(CARRIAGE_LOWER_BEAM_SIZE, (0.0, CARRIAGE_BODY_Y, 0.0))
    carriage = carriage.union(_box(CARRIAGE_UPPER_BEAM_SIZE, (0.0, CARRIAGE_BODY_Y - 0.01, 0.42)))
    carriage = carriage.union(_box(CARRIAGE_CENTER_PLATE_SIZE, (0.0, CARRIAGE_BODY_Y + 0.02, 0.16)))
    carriage = carriage.union(_box(CARRIAGE_CENTER_BRACE_SIZE, (0.0, CARRIAGE_BODY_Y - 0.015, 0.18)))
    carriage = carriage.union(_box(BACKREST_MID_SIZE, (0.0, CARRIAGE_BODY_Y - 0.02, 0.81)))
    carriage = carriage.union(_box(BACKREST_TOP_SIZE, (0.0, CARRIAGE_BODY_Y - 0.02, 0.99)))

    for x_center in (-0.31, 0.31):
        carriage = carriage.union(_box(CARRIAGE_SIDE_PLATE_SIZE, (x_center, CARRIAGE_BODY_Y - 0.02, 0.21)))

    for x_center in (-0.22, 0.0, 0.22):
        carriage = carriage.union(_box(BACKREST_BAR_SIZE, (x_center, CARRIAGE_BODY_Y - 0.02, 0.72)))

    guide_shoe_y = RAIL_FRONT_Y + (GUIDE_SHOE_SIZE[1] / 2.0)
    outer_shoe_offset = (MAST_RAIL_SIZE[0] / 2.0) + (GUIDE_SHOE_SIZE[0] / 2.0)
    for x_center in (-RAIL_CENTER_X, RAIL_CENTER_X):
        shoe_x = x_center + (-outer_shoe_offset if x_center < 0.0 else outer_shoe_offset)
        carriage = carriage.union(_box(GUIDE_SHOE_SIZE, (shoe_x, guide_shoe_y, -0.02)))
        carriage = carriage.union(_box(GUIDE_SHOE_SIZE, (shoe_x, guide_shoe_y, 0.38)))
        post_x = x_center + (-0.045 if x_center < 0.0 else 0.045)
        carriage = carriage.union(_box(GUIDE_POST_SIZE, (post_x, 0.095, 0.18)))

    for x_center in (-0.18, 0.18):
        carriage = carriage.union(_build_fork_solid(x_center))

    return carriage


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mast_mounted_fork_carriage")
    model.material("mast_steel", rgba=(0.28, 0.30, 0.33, 1.0))
    model.material("carriage_steel", rgba=(0.40, 0.42, 0.45, 1.0))

    mast = model.part("mast")
    mast.visual(
        mesh_from_cadquery(_build_mast_shape(), "mast_frame"),
        material="mast_steel",
        name="mast_frame",
    )

    carriage = model.part("carriage")
    carriage.visual(
        mesh_from_cadquery(_build_carriage_shape(), "fork_carriage"),
        material="carriage_steel",
        name="fork_carriage",
    )

    model.articulation(
        "mast_to_carriage",
        ArticulationType.PRISMATIC,
        parent=mast,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, CARRIAGE_REST_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=LIFT_TRAVEL,
            effort=18000.0,
            velocity=0.35,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    mast = object_model.get_part("mast")
    carriage = object_model.get_part("carriage")
    lift = object_model.get_articulation("mast_to_carriage")

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

    ctx.expect_origin_distance(
        mast,
        carriage,
        axes="x",
        max_dist=0.001,
        name="carriage_centered_between_rails",
    )

    with ctx.pose({lift: 0.0}):
        ctx.expect_contact(
            carriage,
            mast,
            contact_tol=1e-6,
            name="carriage_guides_touch_mast_at_rest",
        )
        ctx.expect_overlap(
            carriage,
            mast,
            axes="xz",
            min_overlap=0.20,
            name="carriage_faces_mast_in_front_view",
        )

    with ctx.pose({lift: LIFT_TRAVEL}):
        ctx.expect_contact(
            carriage,
            mast,
            contact_tol=1e-6,
            name="carriage_guides_touch_mast_when_raised",
        )

    with ctx.pose({lift: 0.0}):
        low_pos = ctx.part_world_position(carriage)
    with ctx.pose({lift: LIFT_TRAVEL}):
        high_pos = ctx.part_world_position(carriage)

    ctx.check(
        "positive_prismatic_motion_raises_carriage",
        low_pos is not None and high_pos is not None and (high_pos[2] - low_pos[2]) > 0.70,
        details=f"low_z={low_pos} high_z={high_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
