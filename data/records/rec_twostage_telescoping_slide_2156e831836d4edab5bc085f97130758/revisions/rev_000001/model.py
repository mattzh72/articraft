from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


OUTER_LEN = 0.72
OUTER_W = 0.060
OUTER_H = 0.050
OUTER_BOTTOM_T = 0.006
OUTER_WALL_T = 0.006
OUTER_LIP_T = 0.006
OUTER_LIP_W = 0.012
OUTER_SLOT_W = OUTER_W - 2.0 * (OUTER_WALL_T + OUTER_LIP_W)
OUTER_LIP_Z0 = 0.034

RAIL_LEN = 0.62
RAIL_LOWER_W = 0.030
RAIL_LOWER_H = 0.020
RAIL_LOWER_Z0 = OUTER_BOTTOM_T
RAIL_STEM_W = 0.014
RAIL_STEM_H = 0.024
RAIL_STEM_Z0 = RAIL_LOWER_Z0 + RAIL_LOWER_H
RAIL_CAP_W = 0.026
RAIL_CAP_H = 0.008
RAIL_CAP_Z0 = RAIL_STEM_Z0 + RAIL_STEM_H

STAGE_REAR_STOP_X0 = 0.000
STAGE_FRONT_STOP_X0 = 0.446
STAGE_STOP_LEN = 0.010
STAGE_STOP_W = 0.006
STAGE_STOP_Y = 0.018
STAGE_STOP_H = 0.014
STAGE_STOP_Z0 = 0.022

CARRIAGE_REAR_COLLAR_X0 = 0.190
CARRIAGE_FRONT_COLLAR_X0 = 0.464
CARRIAGE_COLLAR_LEN = 0.008
CARRIAGE_COLLAR_W = 0.030
CARRIAGE_COLLAR_H = 0.008
CARRIAGE_COLLAR_Z0 = 0.056

BUFFER_LEN = 0.006
BUFFER_W = 0.006
BUFFER_H = 0.014
BUFFER_Z0 = STAGE_STOP_Z0
BUFFER_Y = 0.021
REAR_BUFFER_X0 = 0.002
FRONT_BUFFER_X0 = 0.708

RAIL_HOME_X = 0.010
RAIL_TRAVEL = 0.240

CARRIAGE_LEN = 0.102
CARRIAGE_W = 0.039
CARRIAGE_CHEEK_W = 0.006
CARRIAGE_CHEEK_Y = 0.016
CARRIAGE_CHEEK_H = 0.018
CARRIAGE_BRIDGE_Z0 = 0.014
CARRIAGE_BRIDGE_H = 0.006
CARRIAGE_NOSE_X0 = 0.072
CARRIAGE_NOSE_LEN = 0.030
CARRIAGE_NOSE_H = 0.010
CARRIAGE_NOSE_Z0 = 0.020
CARRIAGE_HOME_X = 0.200
CARRIAGE_TRAVEL = 0.160
CARRIAGE_Z = 0.054


def _box_origin(
    x0: float,
    length: float,
    width: float,
    z0: float,
    height: float,
    *,
    y: float = 0.0,
) -> Origin:
    return Origin(xyz=(x0 + 0.5 * length, y, z0 + 0.5 * height))


def _add_box_visual(
    part,
    *,
    name: str,
    size: tuple[float, float, float],
    x0: float,
    z0: float,
    material: str,
    y: float = 0.0,
):
    sx, sy, sz = size
    part.visual(
        Box(size),
        origin=_box_origin(x0, sx, sy, z0, sz, y=y),
        material=material,
        name=name,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="precision_linear_extension")

    model.material("outer_steel", rgba=(0.24, 0.27, 0.30, 1.0))
    model.material("rail_steel", rgba=(0.68, 0.70, 0.73, 1.0))
    model.material("carriage_aluminum", rgba=(0.83, 0.85, 0.87, 1.0))
    model.material("buffer_black", rgba=(0.10, 0.11, 0.12, 1.0))
    model.material("stop_dark", rgba=(0.18, 0.19, 0.21, 1.0))

    outer = model.part("outer_beam")
    inner = model.part("inner_rail")
    carriage = model.part("nose_carriage")

    _add_box_visual(
        outer,
        name="outer_bottom",
        size=(OUTER_LEN, OUTER_W, OUTER_BOTTOM_T),
        x0=0.0,
        z0=0.0,
        material="outer_steel",
    )
    _add_box_visual(
        outer,
        name="outer_left_wall",
        size=(OUTER_LEN, OUTER_WALL_T, OUTER_H),
        x0=0.0,
        y=-(OUTER_W - OUTER_WALL_T) / 2.0,
        z0=0.0,
        material="outer_steel",
    )
    _add_box_visual(
        outer,
        name="outer_right_wall",
        size=(OUTER_LEN, OUTER_WALL_T, OUTER_H),
        x0=0.0,
        y=(OUTER_W - OUTER_WALL_T) / 2.0,
        z0=0.0,
        material="outer_steel",
    )
    _add_box_visual(
        outer,
        name="outer_left_lip",
        size=(OUTER_LEN, OUTER_LIP_W, OUTER_LIP_T),
        x0=0.0,
        y=-(OUTER_SLOT_W + OUTER_LIP_W) / 2.0,
        z0=OUTER_LIP_Z0,
        material="outer_steel",
    )
    _add_box_visual(
        outer,
        name="outer_right_lip",
        size=(OUTER_LEN, OUTER_LIP_W, OUTER_LIP_T),
        x0=0.0,
        y=(OUTER_SLOT_W + OUTER_LIP_W) / 2.0,
        z0=OUTER_LIP_Z0,
        material="outer_steel",
    )
    _add_box_visual(
        outer,
        name="rear_left_buffer",
        size=(BUFFER_LEN, BUFFER_W, BUFFER_H),
        x0=REAR_BUFFER_X0,
        y=-BUFFER_Y,
        z0=BUFFER_Z0,
        material="buffer_black",
    )
    _add_box_visual(
        outer,
        name="rear_right_buffer",
        size=(BUFFER_LEN, BUFFER_W, BUFFER_H),
        x0=REAR_BUFFER_X0,
        y=BUFFER_Y,
        z0=BUFFER_Z0,
        material="buffer_black",
    )
    _add_box_visual(
        outer,
        name="front_left_buffer",
        size=(BUFFER_LEN, BUFFER_W, BUFFER_H),
        x0=FRONT_BUFFER_X0,
        y=-BUFFER_Y,
        z0=BUFFER_Z0,
        material="buffer_black",
    )
    _add_box_visual(
        outer,
        name="front_right_buffer",
        size=(BUFFER_LEN, BUFFER_W, BUFFER_H),
        x0=FRONT_BUFFER_X0,
        y=BUFFER_Y,
        z0=BUFFER_Z0,
        material="buffer_black",
    )

    _add_box_visual(
        inner,
        name="inner_lower_body",
        size=(RAIL_LEN, RAIL_LOWER_W, RAIL_LOWER_H),
        x0=0.0,
        z0=RAIL_LOWER_Z0,
        material="rail_steel",
    )
    _add_box_visual(
        inner,
        name="inner_stem",
        size=(RAIL_LEN, RAIL_STEM_W, RAIL_STEM_H),
        x0=0.0,
        z0=RAIL_STEM_Z0,
        material="rail_steel",
    )
    _add_box_visual(
        inner,
        name="inner_rail_body",
        size=(RAIL_LEN, RAIL_CAP_W, RAIL_CAP_H),
        x0=0.0,
        z0=RAIL_CAP_Z0,
        material="rail_steel",
    )
    _add_box_visual(
        inner,
        name="rear_left_stage_stop",
        size=(STAGE_STOP_LEN, STAGE_STOP_W, STAGE_STOP_H),
        x0=STAGE_REAR_STOP_X0,
        y=-STAGE_STOP_Y,
        z0=STAGE_STOP_Z0,
        material="stop_dark",
    )
    _add_box_visual(
        inner,
        name="rear_right_stage_stop",
        size=(STAGE_STOP_LEN, STAGE_STOP_W, STAGE_STOP_H),
        x0=STAGE_REAR_STOP_X0,
        y=STAGE_STOP_Y,
        z0=STAGE_STOP_Z0,
        material="stop_dark",
    )
    _add_box_visual(
        inner,
        name="front_left_stage_stop",
        size=(STAGE_STOP_LEN, STAGE_STOP_W, STAGE_STOP_H),
        x0=STAGE_FRONT_STOP_X0,
        y=-STAGE_STOP_Y,
        z0=STAGE_STOP_Z0,
        material="stop_dark",
    )
    _add_box_visual(
        inner,
        name="front_right_stage_stop",
        size=(STAGE_STOP_LEN, STAGE_STOP_W, STAGE_STOP_H),
        x0=STAGE_FRONT_STOP_X0,
        y=STAGE_STOP_Y,
        z0=STAGE_STOP_Z0,
        material="stop_dark",
    )
    _add_box_visual(
        inner,
        name="rear_carriage_collar",
        size=(CARRIAGE_COLLAR_LEN, CARRIAGE_COLLAR_W, CARRIAGE_COLLAR_H),
        x0=CARRIAGE_REAR_COLLAR_X0,
        z0=CARRIAGE_COLLAR_Z0,
        material="stop_dark",
    )
    _add_box_visual(
        inner,
        name="front_carriage_collar",
        size=(CARRIAGE_COLLAR_LEN, CARRIAGE_COLLAR_W, CARRIAGE_COLLAR_H),
        x0=CARRIAGE_FRONT_COLLAR_X0,
        z0=CARRIAGE_COLLAR_Z0,
        material="stop_dark",
    )

    _add_box_visual(
        carriage,
        name="left_cheek",
        size=(CARRIAGE_LEN, CARRIAGE_CHEEK_W, CARRIAGE_CHEEK_H),
        x0=0.0,
        y=-CARRIAGE_CHEEK_Y,
        z0=0.0,
        material="carriage_aluminum",
    )
    _add_box_visual(
        carriage,
        name="right_cheek",
        size=(CARRIAGE_LEN, CARRIAGE_CHEEK_W, CARRIAGE_CHEEK_H),
        x0=0.0,
        y=CARRIAGE_CHEEK_Y,
        z0=0.0,
        material="carriage_aluminum",
    )
    _add_box_visual(
        carriage,
        name="carriage_body",
        size=(CARRIAGE_LEN, CARRIAGE_W, CARRIAGE_BRIDGE_H),
        x0=0.0,
        z0=CARRIAGE_BRIDGE_Z0,
        material="carriage_aluminum",
    )
    _add_box_visual(
        carriage,
        name="nose_block",
        size=(CARRIAGE_NOSE_LEN, CARRIAGE_W, CARRIAGE_NOSE_H),
        x0=CARRIAGE_NOSE_X0,
        z0=CARRIAGE_NOSE_Z0,
        material="carriage_aluminum",
    )

    beam_to_rail = model.articulation(
        "beam_to_rail",
        ArticulationType.PRISMATIC,
        parent=outer,
        child=inner,
        origin=Origin(xyz=(RAIL_HOME_X, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=0.45,
            lower=0.0,
            upper=RAIL_TRAVEL,
        ),
    )
    model.articulation(
        "rail_to_carriage",
        ArticulationType.PRISMATIC,
        parent=inner,
        child=carriage,
        origin=Origin(xyz=(CARRIAGE_HOME_X, 0.0, CARRIAGE_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=90.0,
            velocity=0.55,
            lower=0.0,
            upper=CARRIAGE_TRAVEL,
        ),
    )

    beam_to_rail.meta["description"] = "Primary telescoping rail stage"
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    outer = object_model.get_part("outer_beam")
    inner = object_model.get_part("inner_rail")
    carriage = object_model.get_part("nose_carriage")
    beam_to_rail = object_model.get_articulation("beam_to_rail")
    rail_to_carriage = object_model.get_articulation("rail_to_carriage")

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
    ctx.fail_if_articulation_overlaps(max_pose_samples=24)

    ctx.check("outer beam present", outer is not None)
    ctx.check("inner rail present", inner is not None)
    ctx.check("nose carriage present", carriage is not None)
    ctx.check(
        "beam_to_rail axis is slide axis",
        tuple(round(v, 6) for v in beam_to_rail.axis) == (1.0, 0.0, 0.0),
        details=f"axis={beam_to_rail.axis}",
    )
    ctx.check(
        "rail_to_carriage axis is slide axis",
        tuple(round(v, 6) for v in rail_to_carriage.axis) == (1.0, 0.0, 0.0),
        details=f"axis={rail_to_carriage.axis}",
    )

    with ctx.pose({beam_to_rail: 0.0, rail_to_carriage: 0.0}):
        ctx.expect_contact(
            outer,
            inner,
            name="inner rail is physically supported by the outer beam at home",
        )
        ctx.expect_contact(
            inner,
            carriage,
            name="nose carriage is physically supported by the inner rail at home",
        )
        ctx.expect_overlap(
            inner,
            outer,
            axes="yz",
            min_overlap=0.039,
            name="inner rail stays captured in outer beam cross section at home",
        )
        ctx.expect_overlap(
            carriage,
            inner,
            axes="yz",
            min_overlap=0.010,
            name="nose carriage sits on the inner rail at home",
        )
        ctx.expect_gap(
            inner,
            outer,
            axis="x",
            positive_elem="rear_left_stage_stop",
            negative_elem="rear_left_buffer",
            min_gap=0.0015,
            max_gap=0.0035,
            name="rear stage stop rests just off the rear buffer at home",
        )
        ctx.expect_gap(
            carriage,
            inner,
            axis="x",
            positive_elem="carriage_body",
            negative_elem="rear_carriage_collar",
            min_gap=0.0015,
            max_gap=0.0040,
            name="carriage rear collar captures the home position",
        )
        ctx.expect_gap(
            carriage,
            outer,
            axis="z",
            negative_elem="outer_left_lip",
            min_gap=0.010,
            max_gap=0.020,
            name="carriage stays above the outer beam guide lips at home",
        )

    with ctx.pose({beam_to_rail: RAIL_TRAVEL, rail_to_carriage: CARRIAGE_TRAVEL}):
        ctx.expect_contact(
            outer,
            inner,
            name="inner rail remains supported by the outer beam at full extension",
        )
        ctx.expect_contact(
            inner,
            carriage,
            name="nose carriage remains supported by the inner rail at full extension",
        )
        ctx.expect_overlap(
            inner,
            outer,
            axes="x",
            min_overlap=0.450,
            name="inner rail keeps long engagement in the outer beam at full extension",
        )
        ctx.expect_overlap(
            carriage,
            inner,
            axes="x",
            min_overlap=0.100,
            name="carriage keeps substantial engagement on the inner rail at full extension",
        )
        ctx.expect_overlap(
            inner,
            outer,
            axes="yz",
            min_overlap=0.039,
            name="inner rail stays captured in outer beam cross section at full extension",
        )
        ctx.expect_overlap(
            carriage,
            inner,
            axes="yz",
            min_overlap=0.010,
            name="nose carriage stays aligned on the rail at full extension",
        )
        ctx.expect_gap(
            outer,
            inner,
            axis="x",
            positive_elem="front_left_buffer",
            negative_elem="front_left_stage_stop",
            min_gap=0.0015,
            max_gap=0.0035,
            name="front stage stop lands just shy of the front buffer at full extension",
        )
        ctx.expect_gap(
            inner,
            carriage,
            axis="x",
            positive_elem="front_carriage_collar",
            negative_elem="carriage_body",
            min_gap=0.0015,
            max_gap=0.0035,
            name="front carriage collar retains the nose carriage at full extension",
        )
        ctx.expect_gap(
            carriage,
            outer,
            axis="z",
            negative_elem="outer_left_lip",
            min_gap=0.010,
            max_gap=0.020,
            name="carriage stays above the outer beam guide lips at full extension",
        )
        ctx.expect_within(
            carriage,
            inner,
            axes="x",
            margin=0.0,
            name="carriage body remains within the rail's supported running length",
        )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
