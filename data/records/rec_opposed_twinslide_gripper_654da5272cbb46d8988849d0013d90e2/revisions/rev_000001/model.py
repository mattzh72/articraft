from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


BASE_W = 0.22
BASE_D = 0.13
BASE_T = 0.016
HOUSING_H = 0.08
POCKET_DEPTH = 0.020
SUPPORT_TOP_Z = HOUSING_H - POCKET_DEPTH
RAIL_W = 0.034
RAIL_D = 0.096
CENTER_SPINE_W = 0.034
CENTER_SPINE_H = 0.044
OUTER_CHEEK_W = 0.024
OUTER_CHEEK_H = 0.044
END_BEAM_D = 0.022
END_BEAM_H = 0.038

CARRIAGE_W = 0.050
CARRIAGE_D = 0.082
CARRIAGE_H = 0.024
FINGER_LOWER = (0.012, 0.030, 0.020)
FINGER_UPPER = (0.008, 0.026, 0.014)
FINGER_EMBED = 0.001

SLIDE_CENTER_X = 0.046
SLIDE_Z = SUPPORT_TOP_Z + (CARRIAGE_H / 2.0)
SLIDE_STROKE = 0.024


def make_housing_shape() -> cq.Workplane:
    def raised_block(
        width: float,
        depth: float,
        height: float,
        *,
        x: float = 0.0,
        y: float = 0.0,
    ) -> cq.Workplane:
        return (
            cq.Workplane("XY")
            .box(width, depth, height, centered=(True, True, False))
            .translate((x, y, BASE_T))
        )

    housing = cq.Workplane("XY").box(BASE_W, BASE_D, BASE_T, centered=(True, True, False))

    rail_height = SUPPORT_TOP_Z - BASE_T
    housing = housing.union(
        raised_block(RAIL_W, RAIL_D, rail_height, x=-SLIDE_CENTER_X)
    )
    housing = housing.union(
        raised_block(RAIL_W, RAIL_D, rail_height, x=SLIDE_CENTER_X)
    )
    housing = housing.union(raised_block(CENTER_SPINE_W, BASE_D - 0.008, CENTER_SPINE_H))
    housing = housing.union(
        raised_block(
            OUTER_CHEEK_W,
            BASE_D - 0.008,
            OUTER_CHEEK_H,
            x=-(BASE_W / 2.0 - OUTER_CHEEK_W / 2.0 - 0.010),
        )
    )
    housing = housing.union(
        raised_block(
            OUTER_CHEEK_W,
            BASE_D - 0.008,
            OUTER_CHEEK_H,
            x=(BASE_W / 2.0 - OUTER_CHEEK_W / 2.0 - 0.010),
        )
    )
    housing = housing.union(
        raised_block(
            BASE_W - 0.020,
            END_BEAM_D,
            END_BEAM_H,
            y=(BASE_D / 2.0 - END_BEAM_D / 2.0 - 0.010),
        )
    )
    housing = housing.union(
        raised_block(
            BASE_W - 0.020,
            END_BEAM_D,
            END_BEAM_H,
            y=-(BASE_D / 2.0 - END_BEAM_D / 2.0 - 0.010),
        )
    )
    return housing


def make_carriage_body_shape() -> cq.Workplane:
    body = cq.Workplane("XY").box(CARRIAGE_W, CARRIAGE_D, CARRIAGE_H)
    body = body.edges("|Z").fillet(0.003)
    return body


def add_carriage_part(
    model: ArticulatedObject,
    *,
    name: str,
    body_mesh_name: str,
    finger_direction: float,
) -> None:
    carriage = model.part(name)
    carriage.visual(
        mesh_from_cadquery(make_carriage_body_shape(), body_mesh_name),
        material="carriage_steel",
        name="body",
    )

    lower_x = finger_direction * (
        (CARRIAGE_W / 2.0) + (FINGER_LOWER[0] / 2.0) - FINGER_EMBED
    )
    lower_z = (CARRIAGE_H / 2.0) + (FINGER_LOWER[2] / 2.0) - FINGER_EMBED
    carriage.visual(
        Box(FINGER_LOWER),
        origin=Origin(xyz=(lower_x, 0.0, lower_z)),
        material="finger_steel",
        name="finger_lower",
    )

    upper_x = finger_direction * (
        (CARRIAGE_W / 2.0)
        + FINGER_LOWER[0]
        + (FINGER_UPPER[0] / 2.0)
        - (2.0 * FINGER_EMBED)
    )
    upper_z = (
        (CARRIAGE_H / 2.0)
        + FINGER_LOWER[2]
        + (FINGER_UPPER[2] / 2.0)
        - (2.0 * FINGER_EMBED)
    )
    carriage.visual(
        Box(FINGER_UPPER),
        origin=Origin(xyz=(upper_x, 0.0, upper_z)),
        material="finger_steel",
        name="finger_upper",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="block_gripper")

    model.material("housing_cast", rgba=(0.22, 0.24, 0.27, 1.0))
    model.material("carriage_steel", rgba=(0.66, 0.68, 0.71, 1.0))
    model.material("finger_steel", rgba=(0.44, 0.46, 0.49, 1.0))

    housing = model.part("housing")
    housing.visual(
        mesh_from_cadquery(make_housing_shape(), "gripper_housing"),
        material="housing_cast",
        name="housing_shell",
    )

    add_carriage_part(
        model,
        name="left_carriage",
        body_mesh_name="left_carriage_body",
        finger_direction=1.0,
    )
    add_carriage_part(
        model,
        name="right_carriage",
        body_mesh_name="right_carriage_body",
        finger_direction=-1.0,
    )

    left_carriage = model.get_part("left_carriage")
    right_carriage = model.get_part("right_carriage")

    model.articulation(
        "housing_to_left_carriage",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=left_carriage,
        origin=Origin(xyz=(-SLIDE_CENTER_X, 0.0, SLIDE_Z)),
        axis=(-1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=220.0,
            velocity=0.18,
            lower=0.0,
            upper=SLIDE_STROKE,
        ),
    )
    model.articulation(
        "housing_to_right_carriage",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=right_carriage,
        origin=Origin(xyz=(SLIDE_CENTER_X, 0.0, SLIDE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=220.0,
            velocity=0.18,
            lower=0.0,
            upper=SLIDE_STROKE,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    left_carriage = object_model.get_part("left_carriage")
    right_carriage = object_model.get_part("right_carriage")
    left_slide = object_model.get_articulation("housing_to_left_carriage")
    right_slide = object_model.get_articulation("housing_to_right_carriage")

    left_body = left_carriage.get_visual("body")
    right_body = right_carriage.get_visual("body")
    left_upper = left_carriage.get_visual("finger_upper")
    right_upper = right_carriage.get_visual("finger_upper")

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
        "mirrored slides share common axis",
        left_slide.axis == (-1.0, 0.0, 0.0)
        and right_slide.axis == (1.0, 0.0, 0.0)
        and abs(left_slide.origin.xyz[1] - right_slide.origin.xyz[1]) < 1e-9
        and abs(left_slide.origin.xyz[2] - right_slide.origin.xyz[2]) < 1e-9,
        details=(
            f"left axis={left_slide.axis}, right axis={right_slide.axis}, "
            f"left origin={left_slide.origin.xyz}, right origin={right_slide.origin.xyz}"
        ),
    )

    with ctx.pose({left_slide: 0.0, right_slide: 0.0}):
        ctx.expect_contact(
            left_carriage,
            housing,
            name="left carriage is seated on housing",
        )
        ctx.expect_contact(
            right_carriage,
            housing,
            name="right carriage is seated on housing",
        )
        ctx.expect_gap(
            right_carriage,
            left_carriage,
            axis="x",
            positive_elem=right_body,
            negative_elem=left_body,
            min_gap=0.040,
            max_gap=0.044,
            name="wide carriage bodies stay clearly separated",
        )
        ctx.expect_gap(
            right_carriage,
            left_carriage,
            axis="x",
            positive_elem=right_upper,
            negative_elem=left_upper,
            min_gap=0.004,
            max_gap=0.008,
            name="short stepped fingers nearly meet when closed",
        )

        left_closed = ctx.part_world_position(left_carriage)
        right_closed = ctx.part_world_position(right_carriage)

    with ctx.pose(
        {
            left_slide: left_slide.motion_limits.upper,
            right_slide: right_slide.motion_limits.upper,
        }
    ):
        ctx.expect_contact(
            left_carriage,
            housing,
            name="left carriage remains supported through full stroke",
        )
        ctx.expect_contact(
            right_carriage,
            housing,
            name="right carriage remains supported through full stroke",
        )
        ctx.expect_gap(
            right_carriage,
            left_carriage,
            axis="x",
            positive_elem=right_upper,
            negative_elem=left_upper,
            min_gap=0.050,
            name="mirrored slides open the finger gap",
        )

        left_open = ctx.part_world_position(left_carriage)
        right_open = ctx.part_world_position(right_carriage)
        outward_ok = (
            left_closed is not None
            and right_closed is not None
            and left_open is not None
            and right_open is not None
            and left_open[0] < left_closed[0]
            and right_open[0] > right_closed[0]
        )
        ctx.check(
            "positive stroke moves both carriages outward",
            outward_ok,
            details=(
                f"left closed/open={left_closed}/{left_open}, "
                f"right closed/open={right_closed}/{right_open}"
            ),
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
