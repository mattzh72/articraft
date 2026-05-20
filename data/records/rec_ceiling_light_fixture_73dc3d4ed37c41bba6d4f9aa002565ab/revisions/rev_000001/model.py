from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


RAIL_LENGTH = 2.0
RAIL_WIDTH = 0.036
RAIL_HEIGHT = 0.018
TOP_WALL_THICKNESS = 0.003
SIDE_WALL_THICKNESS = 0.003
FIXTURE_X_POSITIONS = (-0.72, -0.36, 0.0, 0.36, 0.72)


def _fixture_names(index: int) -> tuple[str, str, str, str, str, str]:
    prefix = f"fixture_{index}"
    return (
        f"{prefix}_carriage",
        f"{prefix}_swivel",
        f"{prefix}_head",
        f"{prefix}_slide",
        f"{prefix}_pan",
        f"{prefix}_tilt",
    )


def _aabb_center(aabb):
    if aabb is None:
        return None
    lower, upper = aabb
    return (
        0.5 * (lower[0] + upper[0]),
        0.5 * (lower[1] + upper[1]),
        0.5 * (lower[2] + upper[2]),
    )


def _build_fixture(
    model: ArticulatedObject,
    rail,
    *,
    index: int,
    x_position: float,
) -> None:
    carriage_name, swivel_name, head_name, slide_name, pan_name, tilt_name = _fixture_names(index)

    carriage = model.part(carriage_name)
    carriage.visual(
        Box((0.050, 0.020, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.010)),
        material="dark_steel",
        name="shoe",
    )
    carriage.visual(
        Box((0.018, 0.008, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, -0.001)),
        material="dark_steel",
        name="neck",
    )
    carriage.visual(
        Cylinder(radius=0.016, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, -0.009)),
        material="dark_steel",
        name="turntable_plate",
    )

    swivel = model.part(swivel_name)
    swivel.visual(
        Cylinder(radius=0.0155, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, -0.002)),
        material="steel_trim",
        name="top_cap",
    )
    swivel.visual(
        Cylinder(radius=0.0055, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, -0.016)),
        material="steel_trim",
        name="stem",
    )
    swivel.visual(
        Box((0.032, 0.022, 0.012)),
        origin=Origin(xyz=(0.014, 0.0, -0.034)),
        material="steel_trim",
        name="knuckle_block",
    )
    swivel.visual(
        Box((0.010, 0.004, 0.020)),
        origin=Origin(xyz=(0.019, -0.013, -0.047)),
        material="steel_trim",
        name="left_ear",
    )
    swivel.visual(
        Box((0.010, 0.004, 0.020)),
        origin=Origin(xyz=(0.019, 0.013, -0.047)),
        material="steel_trim",
        name="right_ear",
    )

    head = model.part(head_name)
    head.visual(
        Box((0.018, 0.022, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material="steel_trim",
        name="collar_block",
    )
    head.visual(
        Cylinder(radius=0.014, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, -0.030)),
        material="matte_black",
        name="neck_tube",
    )
    head.visual(
        Cylinder(radius=0.028, length=0.064),
        origin=Origin(xyz=(0.0, 0.0, -0.092)),
        material="matte_black",
        name="lamp_body",
    )
    head.visual(
        Cylinder(radius=0.031, length=0.010),
        origin=Origin(xyz=(0.0, 0.0, -0.129)),
        material="matte_black",
        name="front_bezel",
    )
    head.visual(
        Cylinder(radius=0.024, length=0.002),
        origin=Origin(xyz=(0.0, 0.0, -0.135)),
        material="lens_glass",
        name="lens",
    )

    model.articulation(
        slide_name,
        ArticulationType.PRISMATIC,
        parent=rail,
        child=carriage,
        origin=Origin(xyz=(x_position, 0.0, 0.010)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=0.20,
            lower=-0.12,
            upper=0.12,
        ),
    )
    model.articulation(
        pan_name,
        ArticulationType.REVOLUTE,
        parent=carriage,
        child=swivel,
        origin=Origin(xyz=(0.0, 0.0, -0.011)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=10.0,
            velocity=2.5,
            lower=-pi,
            upper=pi,
        ),
    )
    model.articulation(
        tilt_name,
        ArticulationType.REVOLUTE,
        parent=swivel,
        child=head,
        origin=Origin(xyz=(0.019, 0.0, -0.049)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=8.0,
            velocity=2.0,
            lower=-1.30,
            upper=0.45,
        ),
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="track_lighting_system")

    model.material("rail_white", rgba=(0.96, 0.96, 0.94, 1.0))
    model.material("matte_black", rgba=(0.14, 0.14, 0.15, 1.0))
    model.material("dark_steel", rgba=(0.38, 0.40, 0.43, 1.0))
    model.material("steel_trim", rgba=(0.60, 0.61, 0.62, 1.0))
    model.material("lens_glass", rgba=(0.88, 0.92, 0.96, 0.65))

    rail = model.part("rail")
    rail.visual(
        Box((RAIL_LENGTH, RAIL_WIDTH, TOP_WALL_THICKNESS)),
        origin=Origin(xyz=(0.0, 0.0, RAIL_HEIGHT - 0.5 * TOP_WALL_THICKNESS)),
        material="rail_white",
        name="top_wall",
    )
    rail.visual(
        Box((RAIL_LENGTH, SIDE_WALL_THICKNESS, RAIL_HEIGHT - TOP_WALL_THICKNESS)),
        origin=Origin(
            xyz=(
                0.0,
                -0.5 * (RAIL_WIDTH - SIDE_WALL_THICKNESS),
                0.5 * (RAIL_HEIGHT - TOP_WALL_THICKNESS),
            )
        ),
        material="rail_white",
        name="left_wall",
    )
    rail.visual(
        Box((RAIL_LENGTH, SIDE_WALL_THICKNESS, RAIL_HEIGHT - TOP_WALL_THICKNESS)),
        origin=Origin(
            xyz=(
                0.0,
                0.5 * (RAIL_WIDTH - SIDE_WALL_THICKNESS),
                0.5 * (RAIL_HEIGHT - TOP_WALL_THICKNESS),
            )
        ),
        material="rail_white",
        name="right_wall",
    )
    rail.visual(
        Box((0.004, RAIL_WIDTH, RAIL_HEIGHT)),
        origin=Origin(xyz=(-0.5 * RAIL_LENGTH + 0.002, 0.0, 0.5 * RAIL_HEIGHT)),
        material="rail_white",
        name="left_endcap",
    )
    rail.visual(
        Box((0.004, RAIL_WIDTH, RAIL_HEIGHT)),
        origin=Origin(xyz=(0.5 * RAIL_LENGTH - 0.002, 0.0, 0.5 * RAIL_HEIGHT)),
        material="rail_white",
        name="right_endcap",
    )

    for index, x_position in enumerate(FIXTURE_X_POSITIONS, start=1):
        _build_fixture(model, rail, index=index, x_position=x_position)

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
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

    rail = object_model.get_part("rail")

    for index in range(1, 6):
        carriage_name, swivel_name, head_name, slide_name, pan_name, tilt_name = _fixture_names(index)
        carriage = object_model.get_part(carriage_name)
        swivel = object_model.get_part(swivel_name)
        head = object_model.get_part(head_name)
        slide = object_model.get_articulation(slide_name)
        pan = object_model.get_articulation(pan_name)
        tilt = object_model.get_articulation(tilt_name)

        ctx.check(f"{carriage_name} present", carriage is not None)
        ctx.check(f"{swivel_name} present", swivel is not None)
        ctx.check(f"{head_name} present", head is not None)

        ctx.expect_contact(
            carriage,
            rail,
            elem_a="shoe",
            elem_b="top_wall",
            name=f"{carriage_name} rides in contact with the rail track",
        )
        ctx.expect_contact(
            swivel,
            carriage,
            elem_a="top_cap",
            elem_b="turntable_plate",
            name=f"{swivel_name} seats on the carriage turntable",
        )
        ctx.expect_contact(
            head,
            swivel,
            elem_a="collar_block",
            elem_b="left_ear",
            name=f"{head_name} is supported by the left tilt ear",
        )
        ctx.expect_contact(
            head,
            swivel,
            elem_a="collar_block",
            elem_b="right_ear",
            name=f"{head_name} is supported by the right tilt ear",
        )
        ctx.expect_origin_gap(
            rail,
            head,
            axis="z",
            min_gap=0.04,
            name=f"{head_name} hangs below the ceiling rail",
        )

        ctx.check(
            f"{slide_name} uses a rail-axis prismatic joint",
            slide.axis == (1.0, 0.0, 0.0)
            and slide.motion_limits is not None
            and slide.motion_limits.lower is not None
            and slide.motion_limits.upper is not None
            and slide.motion_limits.lower < 0.0 < slide.motion_limits.upper,
            details=f"axis={slide.axis}, limits={slide.motion_limits}",
        )
        ctx.check(
            f"{pan_name} pans on a vertical revolute joint",
            pan.axis == (0.0, 0.0, 1.0)
            and pan.motion_limits is not None
            and pan.motion_limits.lower is not None
            and pan.motion_limits.upper is not None
            and pan.motion_limits.lower <= -3.0
            and pan.motion_limits.upper >= 3.0,
            details=f"axis={pan.axis}, limits={pan.motion_limits}",
        )
        ctx.check(
            f"{tilt_name} tilts on a horizontal collar joint",
            tilt.axis == (0.0, 1.0, 0.0)
            and tilt.motion_limits is not None
            and tilt.motion_limits.lower is not None
            and tilt.motion_limits.upper is not None
            and tilt.motion_limits.lower < -1.0
            and 0.2 < tilt.motion_limits.upper < 0.8,
            details=f"axis={tilt.axis}, limits={tilt.motion_limits}",
        )

    carriage_3 = object_model.get_part("fixture_3_carriage")
    head_3 = object_model.get_part("fixture_3_head")
    slide_3 = object_model.get_articulation("fixture_3_slide")
    pan_3 = object_model.get_articulation("fixture_3_pan")
    tilt_3 = object_model.get_articulation("fixture_3_tilt")

    rest_carriage_pos = ctx.part_world_position(carriage_3)
    with ctx.pose({slide_3: 0.12}):
        extended_carriage_pos = ctx.part_world_position(carriage_3)
    ctx.check(
        "center fixture carriage slides along the rail",
        rest_carriage_pos is not None
        and extended_carriage_pos is not None
        and extended_carriage_pos[0] > rest_carriage_pos[0] + 0.10
        and abs(extended_carriage_pos[1] - rest_carriage_pos[1]) < 1e-6
        and abs(extended_carriage_pos[2] - rest_carriage_pos[2]) < 1e-6,
        details=f"rest={rest_carriage_pos}, extended={extended_carriage_pos}",
    )

    with ctx.pose({tilt_3: 0.0, pan_3: 0.0}):
        neutral_bezel = _aabb_center(ctx.part_element_world_aabb(head_3, elem="front_bezel"))
    with ctx.pose({tilt_3: -1.0, pan_3: 0.0}):
        tilted_bezel = _aabb_center(ctx.part_element_world_aabb(head_3, elem="front_bezel"))
    ctx.check(
        "center fixture head tilts at the collar",
        neutral_bezel is not None
        and tilted_bezel is not None
        and tilted_bezel[0] > neutral_bezel[0] + 0.07
        and tilted_bezel[2] > neutral_bezel[2] + 0.03,
        details=f"neutral={neutral_bezel}, tilted={tilted_bezel}",
    )

    with ctx.pose({tilt_3: -1.0, pan_3: 0.0}):
        pan_start_bezel = _aabb_center(ctx.part_element_world_aabb(head_3, elem="front_bezel"))
    with ctx.pose({tilt_3: -1.0, pan_3: 1.15}):
        pan_end_bezel = _aabb_center(ctx.part_element_world_aabb(head_3, elem="front_bezel"))
    ctx.check(
        "center fixture pans around the vertical clip joint",
        pan_start_bezel is not None
        and pan_end_bezel is not None
        and abs(pan_end_bezel[1] - pan_start_bezel[1]) > 0.07,
        details=f"start={pan_start_bezel}, end={pan_end_bezel}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
