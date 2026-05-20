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


BASE_LENGTH = 0.26
BASE_WIDTH = 0.16
BASE_THICKNESS = 0.016
RAIL_LENGTH = 0.22
RAIL_WIDTH = 0.028
RAIL_HEIGHT = 0.022
RAIL_OFFSET_Y = 0.045
RAIL_TOP_Z = BASE_THICKNESS + RAIL_HEIGHT

X_TRAVEL = 0.04
Z_TRAVEL = 0.10

CARRIAGE_RUNNER_LENGTH = 0.14
CARRIAGE_RUNNER_HEIGHT = 0.016
CARRIAGE_TOP_LENGTH = 0.18
CARRIAGE_TOP_WIDTH = 0.14
CARRIAGE_TOP_THICKNESS = 0.018
CARRIAGE_TOP_Z = CARRIAGE_RUNNER_HEIGHT

PEDESTAL_LENGTH = 0.082
PEDESTAL_WIDTH = 0.076
PEDESTAL_HEIGHT = 0.040
PEDESTAL_Z = CARRIAGE_TOP_Z + CARRIAGE_TOP_THICKNESS
PEDESTAL_TOP_Z = PEDESTAL_Z + PEDESTAL_HEIGHT
CHEEK_LENGTH = 0.072
CHEEK_WIDTH = 0.010
CHEEK_HEIGHT = 0.056
CHEEK_OFFSET_Y = 0.032

UPPER_SLIDER_LENGTH = 0.072
UPPER_SLIDER_WIDTH = 0.046
UPPER_SLIDER_HEIGHT = 0.026
UPPER_COLUMN_LENGTH = 0.058
UPPER_COLUMN_WIDTH = 0.034
UPPER_COLUMN_HEIGHT = 0.156
UPPER_TOP_CAP_LENGTH = 0.070
UPPER_TOP_CAP_WIDTH = 0.046
UPPER_TOP_CAP_HEIGHT = 0.016
UPPER_FRONT_PLATE_LENGTH = 0.082
UPPER_FRONT_PLATE_WIDTH = 0.010
UPPER_FRONT_PLATE_HEIGHT = 0.072


def _box(length: float, width: float, height: float, *, x: float = 0.0, y: float = 0.0, z: float = 0.0) -> cq.Workplane:
    return (
        cq.Workplane("XY")
        .box(length, width, height, centered=(True, True, False))
        .translate((x, y, z))
    )


def _guide_body_shape() -> cq.Workplane:
    body = _box(BASE_LENGTH, BASE_WIDTH, BASE_THICKNESS)
    body = body.union(_box(RAIL_LENGTH, RAIL_WIDTH, RAIL_HEIGHT, y=RAIL_OFFSET_Y, z=BASE_THICKNESS))
    body = body.union(_box(RAIL_LENGTH, RAIL_WIDTH, RAIL_HEIGHT, y=-RAIL_OFFSET_Y, z=BASE_THICKNESS))
    body = body.union(_box(0.030, 0.100, 0.010, x=-0.095, z=BASE_THICKNESS))
    body = body.union(_box(0.030, 0.100, 0.010, x=0.095, z=BASE_THICKNESS))
    body = body.cut(_box(0.180, 0.054, 0.020, z=0.006))
    return body


def _lower_carriage_shape() -> cq.Workplane:
    carriage = _box(CARRIAGE_RUNNER_LENGTH, RAIL_WIDTH, CARRIAGE_RUNNER_HEIGHT, y=RAIL_OFFSET_Y)
    carriage = carriage.union(
        _box(CARRIAGE_RUNNER_LENGTH, RAIL_WIDTH, CARRIAGE_RUNNER_HEIGHT, y=-RAIL_OFFSET_Y)
    )
    carriage = carriage.union(
        _box(
            CARRIAGE_TOP_LENGTH,
            CARRIAGE_TOP_WIDTH,
            CARRIAGE_TOP_THICKNESS,
            z=CARRIAGE_TOP_Z,
        )
    )
    carriage = carriage.union(
        _box(
            PEDESTAL_LENGTH,
            PEDESTAL_WIDTH,
            PEDESTAL_HEIGHT,
            z=PEDESTAL_Z,
        )
    )
    carriage = carriage.union(
        _box(
            CHEEK_LENGTH,
            CHEEK_WIDTH,
            CHEEK_HEIGHT,
            y=CHEEK_OFFSET_Y,
            z=PEDESTAL_Z,
        )
    )
    carriage = carriage.union(
        _box(
            CHEEK_LENGTH,
            CHEEK_WIDTH,
            CHEEK_HEIGHT,
            y=-CHEEK_OFFSET_Y,
            z=PEDESTAL_Z,
        )
    )
    return carriage


def _upper_guide_shape() -> cq.Workplane:
    guide = _box(UPPER_SLIDER_LENGTH, UPPER_SLIDER_WIDTH, UPPER_SLIDER_HEIGHT)
    guide = guide.union(
        _box(
            UPPER_COLUMN_LENGTH,
            UPPER_COLUMN_WIDTH,
            UPPER_COLUMN_HEIGHT,
            z=UPPER_SLIDER_HEIGHT,
        )
    )
    guide = guide.union(
        _box(
            UPPER_TOP_CAP_LENGTH,
            UPPER_TOP_CAP_WIDTH,
            UPPER_TOP_CAP_HEIGHT,
            z=UPPER_SLIDER_HEIGHT + UPPER_COLUMN_HEIGHT,
        )
    )
    guide = guide.union(
        _box(
            UPPER_FRONT_PLATE_LENGTH,
            UPPER_FRONT_PLATE_WIDTH,
            UPPER_FRONT_PLATE_HEIGHT,
            y=-(UPPER_COLUMN_WIDTH + UPPER_FRONT_PLATE_WIDTH) / 2.0,
            z=0.092,
        )
    )
    guide = guide.union(
        _box(
            0.052,
            0.018,
            0.044,
            y=-(UPPER_COLUMN_WIDTH + 0.018) / 2.0,
            z=0.074,
        )
    )
    return guide


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_service_xz_stage")

    model.material("body_dark", rgba=(0.22, 0.24, 0.27, 1.0))
    model.material("carriage_alloy", rgba=(0.66, 0.69, 0.73, 1.0))
    model.material("guide_silver", rgba=(0.78, 0.80, 0.83, 1.0))

    guide_body = model.part("guide_body")
    guide_body.visual(
        mesh_from_cadquery(_guide_body_shape(), "guide_body"),
        material="body_dark",
        name="guide_body_shell",
    )

    lower_carriage = model.part("lower_carriage")
    lower_carriage.visual(
        mesh_from_cadquery(_lower_carriage_shape(), "lower_carriage"),
        material="carriage_alloy",
        name="lower_carriage_shell",
    )

    upper_guide = model.part("upper_guide")
    upper_guide.visual(
        mesh_from_cadquery(_upper_guide_shape(), "upper_guide"),
        material="guide_silver",
        name="upper_guide_shell",
    )

    model.articulation(
        "body_to_lower_carriage",
        ArticulationType.PRISMATIC,
        parent=guide_body,
        child=lower_carriage,
        origin=Origin(xyz=(0.0, 0.0, RAIL_TOP_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=-X_TRAVEL,
            upper=X_TRAVEL,
            effort=120.0,
            velocity=0.20,
        ),
    )

    model.articulation(
        "lower_carriage_to_upper_guide",
        ArticulationType.PRISMATIC,
        parent=lower_carriage,
        child=upper_guide,
        origin=Origin(xyz=(0.0, 0.0, PEDESTAL_TOP_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            lower=0.0,
            upper=Z_TRAVEL,
            effort=90.0,
            velocity=0.16,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    guide_body = object_model.get_part("guide_body")
    lower_carriage = object_model.get_part("lower_carriage")
    upper_guide = object_model.get_part("upper_guide")
    x_slide = object_model.get_articulation("body_to_lower_carriage")
    z_slide = object_model.get_articulation("lower_carriage_to_upper_guide")

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

    ctx.check(
        "stage_uses_two_prismatic_joints",
        x_slide.articulation_type == ArticulationType.PRISMATIC
        and z_slide.articulation_type == ArticulationType.PRISMATIC,
        details=(
            f"Expected two prismatic joints, got "
            f"{x_slide.articulation_type!r} and {z_slide.articulation_type!r}"
        ),
    )
    ctx.check(
        "prismatic_axes_are_orthogonal_xz",
        tuple(x_slide.axis) == (1.0, 0.0, 0.0) and tuple(z_slide.axis) == (0.0, 0.0, 1.0),
        details=f"Got axes {x_slide.axis!r} and {z_slide.axis!r}",
    )
    ctx.expect_contact(
        lower_carriage,
        guide_body,
        contact_tol=0.0005,
        name="lower_carriage_supported_by_guide_body",
    )
    ctx.expect_overlap(
        lower_carriage,
        guide_body,
        axes="xy",
        min_overlap=0.10,
        name="lower_carriage_spans_guide_body",
    )
    ctx.expect_contact(
        upper_guide,
        lower_carriage,
        contact_tol=0.0005,
        name="upper_guide_seated_on_lower_carriage",
    )

    with ctx.pose({x_slide: 0.03}):
        body_pos = ctx.part_world_position(guide_body)
        lower_pos = ctx.part_world_position(lower_carriage)
        upper_pos = ctx.part_world_position(upper_guide)
        ctx.check(
            "x_stage_motion_moves_carriage_and_carried_guide_sideways",
            body_pos is not None
            and lower_pos is not None
            and upper_pos is not None
            and lower_pos[0] > body_pos[0] + 0.02
            and abs(lower_pos[1] - body_pos[1]) < 1e-6
            and abs(upper_pos[0] - lower_pos[0]) < 1e-6
            and abs(upper_pos[2] - (PEDESTAL_TOP_Z + RAIL_TOP_Z)) < 1e-6,
            details=f"body={body_pos}, lower={lower_pos}, upper={upper_pos}",
        )

    with ctx.pose({z_slide: 0.08}):
        lower_pos = ctx.part_world_position(lower_carriage)
        upper_pos = ctx.part_world_position(upper_guide)
        ctx.check(
            "z_stage_motion_is_vertical_relative_to_lower_carriage",
            lower_pos is not None
            and upper_pos is not None
            and abs(upper_pos[0] - lower_pos[0]) < 1e-6
            and abs(upper_pos[1] - lower_pos[1]) < 1e-6
            and upper_pos[2] > lower_pos[2] + 0.15,
            details=f"lower={lower_pos}, upper={upper_pos}",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
