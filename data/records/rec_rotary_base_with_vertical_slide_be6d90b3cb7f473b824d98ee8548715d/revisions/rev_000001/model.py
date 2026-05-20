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


PLATE_W = 0.170
PLATE_H = 0.340
PLATE_T = 0.016

PIVOT_Y = 0.047992
PIVOT_Z = 0.115

DRUM_R = 0.026
DRUM_H = 0.024

SLIDE_LOW_Z = 0.060
SLIDE_TRAVEL = 0.120


def _centered_box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _centered_cylinder(radius: float, height: float, center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").circle(radius).extrude(height).translate((center[0], center[1], center[2] - height / 2.0))


def make_backplate_shape() -> cq.Workplane:
    plate = _centered_box((PLATE_W, PLATE_T, PLATE_H), (0.0, PLATE_T / 2.0, PLATE_H / 2.0))
    center_pad = _centered_box((0.072, 0.014, 0.112), (0.0, 0.023, PIVOT_Z))
    contact_pad = _centered_box((0.052, 0.004, 0.088), (0.0, 0.032, PIVOT_Z))
    upper_boss = _centered_box((0.050, 0.012, 0.038), (0.0, 0.024, PIVOT_Z + 0.046))
    lower_boss = _centered_box((0.050, 0.012, 0.038), (0.0, 0.024, PIVOT_Z - 0.046))
    side_rib_left = _centered_box((0.014, 0.018, 0.118), (-0.030, 0.025, PIVOT_Z))
    side_rib_right = _centered_box((0.014, 0.018, 0.118), (0.030, 0.025, PIVOT_Z))
    top_pad = _centered_box((0.096, 0.010, 0.060), (0.0, PLATE_T + 0.005, 0.278))
    bottom_pad = _centered_box((0.096, 0.010, 0.060), (0.0, PLATE_T + 0.005, 0.062))

    return (
        plate.union(center_pad)
        .union(contact_pad)
        .union(upper_boss)
        .union(lower_boss)
        .union(side_rib_left)
        .union(side_rib_right)
        .union(top_pad)
        .union(bottom_pad)
    )


def make_rotary_frame_shape() -> cq.Workplane:
    rear_mount = _centered_box((0.050, 0.012, 0.084), (0.0, 0.000, 0.0))
    drum = _centered_cylinder(DRUM_R, DRUM_H, (0.0, 0.012, 0.0))
    arm = _centered_box((0.058, 0.024, 0.024), (0.0, 0.028, 0.006))
    riser = _centered_box((0.038, 0.024, 0.080), (0.0, 0.046, 0.044))
    lower_gusset = _centered_box((0.046, 0.026, 0.042), (0.0, 0.064, 0.082))
    guide_body = _centered_box((0.060, 0.034, 0.188), (0.0, 0.086, 0.144))
    top_cap = _centered_box((0.068, 0.028, 0.018), (0.0, 0.086, 0.241))

    return rear_mount.union(drum).union(arm).union(riser).union(lower_gusset).union(guide_body).union(top_cap)


def make_guide_rail_shape() -> cq.Workplane:
    return _centered_box((0.026, 0.010, 0.180), (0.0, 0.108, 0.145))


def make_carriage_shape() -> cq.Workplane:
    body = _centered_box((0.058, 0.028, 0.052), (0.0, 0.0, 0.0))
    face_plate = _centered_box((0.040, 0.018, 0.036), (0.0, 0.020, 0.0))
    top_lug = _centered_box((0.030, 0.014, 0.014), (0.0, 0.002, 0.026))
    return body.union(face_plate).union(top_lug)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="wall_backed_rotary_column")

    backplate_mat = model.material("backplate_gray", rgba=(0.28, 0.30, 0.33, 1.0))
    frame_mat = model.material("anodized_aluminum", rgba=(0.76, 0.78, 0.80, 1.0))
    rail_mat = model.material("rail_steel", rgba=(0.55, 0.58, 0.62, 1.0))
    carriage_mat = model.material("carriage_orange", rgba=(0.86, 0.43, 0.18, 1.0))

    backplate = model.part("backplate")
    backplate.visual(
        mesh_from_cadquery(make_backplate_shape(), "backplate_body"),
        material=backplate_mat,
        name="backplate_body",
    )

    rotary_column = model.part("rotary_column")
    rotary_column.visual(
        mesh_from_cadquery(make_rotary_frame_shape(), "rotary_stage_body"),
        material=frame_mat,
        name="stage_body",
    )
    rotary_column.visual(
        mesh_from_cadquery(make_guide_rail_shape(), "rotary_stage_rail"),
        material=rail_mat,
        name="guide_rail",
    )

    carriage = model.part("carriage")
    carriage.visual(
        mesh_from_cadquery(make_carriage_shape(), "slide_carriage"),
        material=carriage_mat,
        name="carriage_body",
    )

    model.articulation(
        "backplate_to_rotary_column",
        ArticulationType.REVOLUTE,
        parent=backplate,
        child=rotary_column,
        origin=Origin(xyz=(0.0, PIVOT_Y, PIVOT_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=45.0,
            velocity=1.2,
            lower=-1.35,
            upper=1.35,
        ),
    )

    model.articulation(
        "rotary_column_to_carriage",
        ArticulationType.PRISMATIC,
        parent=rotary_column,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.127, SLIDE_LOW_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=220.0,
            velocity=0.18,
            lower=0.0,
            upper=SLIDE_TRAVEL,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    backplate = object_model.get_part("backplate")
    rotary_column = object_model.get_part("rotary_column")
    carriage = object_model.get_part("carriage")
    swing = object_model.get_articulation("backplate_to_rotary_column")
    lift = object_model.get_articulation("rotary_column_to_carriage")

    backplate_body = backplate.get_visual("backplate_body")
    stage_body = rotary_column.get_visual("stage_body")
    guide_rail = rotary_column.get_visual("guide_rail")
    carriage_body = carriage.get_visual("carriage_body")

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
        "rotary joint axis is vertical",
        tuple(swing.axis) == (0.0, 0.0, 1.0),
        f"axis={swing.axis}",
    )
    ctx.check(
        "slide joint axis is vertical",
        tuple(lift.axis) == (0.0, 0.0, 1.0),
        f"axis={lift.axis}",
    )
    ctx.check(
        "rotary joint swings to both sides",
        swing.motion_limits is not None
        and swing.motion_limits.lower is not None
        and swing.motion_limits.upper is not None
        and swing.motion_limits.lower < 0.0 < swing.motion_limits.upper,
        f"limits={swing.motion_limits}",
    )
    ctx.check(
        "slide joint lifts upward from a low home",
        lift.motion_limits is not None
        and lift.motion_limits.lower == 0.0
        and lift.motion_limits.upper is not None
        and lift.motion_limits.upper >= 0.10,
        f"limits={lift.motion_limits}",
    )

    ctx.expect_contact(
        rotary_column,
        backplate,
        elem_a=stage_body,
        elem_b=backplate_body,
        name="rotary stage is physically carried by the backplate yoke",
    )
    ctx.expect_contact(
        carriage,
        rotary_column,
        elem_a=carriage_body,
        elem_b=guide_rail,
        name="carriage is captured on the guide rail",
    )

    def aabb_center(aabb: tuple[tuple[float, float, float], tuple[float, float, float]] | None) -> tuple[float, float, float] | None:
        if aabb is None:
            return None
        low, high = aabb
        return tuple((low[i] + high[i]) / 2.0 for i in range(3))

    with ctx.pose({swing: 1.0}):
        rail_aabb = ctx.part_element_world_aabb(rotary_column, elem=guide_rail)
        rail_center = aabb_center(rail_aabb)
        ctx.check(
            "positive rotary motion swings the upright leftward",
            rail_center is not None and rail_center[0] < -0.08,
            f"guide rail center at +1.0 rad = {rail_center}",
        )
        ctx.expect_gap(
            carriage,
            backplate,
            axis="y",
            min_gap=0.005,
            positive_elem=carriage_body,
            negative_elem=backplate_body,
            name="swung carriage remains proud of the wall bracket",
        )

    base_pos = ctx.part_world_position(carriage)
    with ctx.pose({lift: 0.10}):
        raised_pos = ctx.part_world_position(carriage)
        ctx.expect_contact(
            carriage,
            rotary_column,
            elem_a=carriage_body,
            elem_b=guide_rail,
            name="raised carriage stays in contact with the guide rail",
        )
    ctx.check(
        "prismatic joint raises the carriage in +Z",
        base_pos is not None
        and raised_pos is not None
        and (raised_pos[2] - base_pos[2]) > 0.095,
        f"base={base_pos}, raised={raised_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
