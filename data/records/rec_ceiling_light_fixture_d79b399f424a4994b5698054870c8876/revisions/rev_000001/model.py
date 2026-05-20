from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math

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


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="recessed_wall_wash_ceiling_light")

    ceiling_white = model.material("ceiling_white", rgba=(0.95, 0.95, 0.94, 1.0))
    housing_white = model.material("housing_white", rgba=(0.93, 0.93, 0.92, 1.0))
    sheet_metal = model.material("sheet_metal", rgba=(0.72, 0.74, 0.76, 1.0))
    matte_black = model.material("matte_black", rgba=(0.10, 0.10, 0.11, 1.0))
    steel = model.material("steel", rgba=(0.78, 0.79, 0.81, 1.0))

    ceiling_panel = model.part("ceiling_panel")
    ceiling_panel.visual(
        Box((0.80, 0.12, 0.018)),
        origin=Origin(xyz=(0.0, 0.14, 0.009)),
        material=ceiling_white,
        name="panel_room_band",
    )
    ceiling_panel.visual(
        Box((0.80, 0.12, 0.018)),
        origin=Origin(xyz=(0.0, -0.14, 0.009)),
        material=ceiling_white,
        name="panel_wall_band",
    )
    ceiling_panel.visual(
        Box((0.08, 0.16, 0.018)),
        origin=Origin(xyz=(-0.36, 0.0, 0.009)),
        material=ceiling_white,
        name="panel_left_band",
    )
    ceiling_panel.visual(
        Box((0.08, 0.16, 0.018)),
        origin=Origin(xyz=(0.36, 0.0, 0.009)),
        material=ceiling_white,
        name="panel_right_band",
    )
    ceiling_panel.inertial = Inertial.from_geometry(
        Box((0.80, 0.40, 0.018)),
        mass=3.5,
        origin=Origin(xyz=(0.0, 0.0, 0.009)),
    )

    housing = model.part("housing")
    housing.visual(
        Box((0.60, 0.004, 0.151)),
        origin=Origin(xyz=(0.0, 0.058, 0.0795)),
        material=sheet_metal,
        name="room_wall",
    )
    housing.visual(
        Box((0.60, 0.004, 0.151)),
        origin=Origin(xyz=(0.0, -0.058, 0.0795)),
        material=sheet_metal,
        name="wall_side_wall",
    )
    housing.visual(
        Box((0.004, 0.12, 0.151)),
        origin=Origin(xyz=(-0.298, 0.0, 0.0795)),
        material=sheet_metal,
        name="left_end_wall",
    )
    housing.visual(
        Box((0.004, 0.12, 0.151)),
        origin=Origin(xyz=(0.298, 0.0, 0.0795)),
        material=sheet_metal,
        name="right_end_wall",
    )
    housing.visual(
        Box((0.60, 0.12, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.153)),
        material=sheet_metal,
        name="top_plate",
    )
    housing.visual(
        Box((0.64, 0.02, 0.004)),
        origin=Origin(xyz=(0.0, 0.07, 0.002)),
        material=housing_white,
        name="trim_room",
    )
    housing.visual(
        Box((0.64, 0.02, 0.004)),
        origin=Origin(xyz=(0.0, -0.07, 0.002)),
        material=housing_white,
        name="trim_wall",
    )
    housing.visual(
        Box((0.02, 0.12, 0.004)),
        origin=Origin(xyz=(-0.31, 0.0, 0.002)),
        material=housing_white,
        name="trim_left",
    )
    housing.visual(
        Box((0.02, 0.12, 0.004)),
        origin=Origin(xyz=(0.31, 0.0, 0.002)),
        material=housing_white,
        name="trim_right",
    )
    housing.visual(
        Box((0.014, 0.112, 0.008)),
        origin=Origin(xyz=(-0.291, 0.0, 0.066)),
        material=sheet_metal,
        name="left_track",
    )
    housing.visual(
        Box((0.014, 0.112, 0.008)),
        origin=Origin(xyz=(0.291, 0.0, 0.066)),
        material=sheet_metal,
        name="right_track",
    )
    housing.visual(
        Box((0.02, 0.11, 0.010)),
        origin=Origin(xyz=(-0.023, 0.0, 0.148)),
        material=sheet_metal,
        name="bridge_left",
    )
    housing.visual(
        Box((0.02, 0.11, 0.010)),
        origin=Origin(xyz=(0.023, 0.0, 0.148)),
        material=sheet_metal,
        name="bridge_right",
    )
    housing.inertial = Inertial.from_geometry(
        Box((0.64, 0.16, 0.155)),
        mass=4.0,
        origin=Origin(xyz=(0.0, 0.0, 0.0775)),
    )

    baffle = model.part("baffle")
    baffle.visual(
        Box((0.008, 0.082, 0.058)),
        origin=Origin(xyz=(-0.280, 0.0, 0.033)),
        material=matte_black,
        name="left_cheek",
    )
    baffle.visual(
        Box((0.008, 0.082, 0.058)),
        origin=Origin(xyz=(0.280, 0.0, 0.033)),
        material=matte_black,
        name="right_cheek",
    )
    baffle.visual(
        Box((0.552, 0.010, 0.050)),
        origin=Origin(xyz=(0.0, 0.034, 0.029)),
        material=matte_black,
        name="room_lip",
    )
    baffle.visual(
        Box((0.552, 0.008, 0.028)),
        origin=Origin(xyz=(0.0, -0.033, 0.018)),
        material=matte_black,
        name="wall_lip",
    )
    baffle.visual(
        Box((0.552, 0.056, 0.004)),
        origin=Origin(xyz=(0.0, 0.001, 0.039), rpy=(0.52, 0.0, 0.0)),
        material=matte_black,
        name="spreader_plate",
    )
    fin_positions = (
        (-0.020, 0.020),
        (-0.006, 0.024),
        (0.008, 0.028),
        (0.022, 0.032),
    )
    for idx, (y_pos, z_pos) in enumerate(fin_positions, start=1):
        baffle.visual(
            Box((0.548, 0.003, 0.025)),
            origin=Origin(xyz=(0.0, y_pos, z_pos), rpy=(0.52, 0.0, 0.0)),
            material=matte_black,
            name=f"louver_fin_{idx}",
        )
    baffle.visual(
        Box((0.552, 0.010, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.057)),
        material=matte_black,
        name="carrier_bar",
    )
    baffle.inertial = Inertial.from_geometry(
        Box((0.56, 0.09, 0.06)),
        mass=0.8,
        origin=Origin(xyz=(0.0, 0.0, 0.033)),
    )

    thumb_screw = model.part("thumb_screw")
    thumb_screw.visual(
        Cylinder(radius=0.0035, length=0.087),
        origin=Origin(xyz=(0.0, 0.0, 0.0435)),
        material=steel,
        name="stem",
    )
    thumb_screw.visual(
        Cylinder(radius=0.011, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=steel,
        name="thumb_head",
    )
    thumb_screw.visual(
        Cylinder(radius=0.008, length=0.004),
        origin=Origin(xyz=(0.0, 0.0, 0.087)),
        material=steel,
        name="clamp_washer",
    )
    thumb_screw.visual(
        Cylinder(radius=0.0025, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.012), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=steel,
        name="thumb_bar",
    )
    thumb_screw.inertial = Inertial.from_geometry(
        Cylinder(radius=0.012, length=0.092),
        mass=0.08,
        origin=Origin(xyz=(0.0, 0.0, 0.046)),
    )

    model.articulation(
        "panel_to_housing",
        ArticulationType.FIXED,
        parent=ceiling_panel,
        child=housing,
        origin=Origin(),
    )

    model.articulation(
        "housing_to_baffle",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=baffle,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=20.0,
            velocity=0.05,
            lower=-0.015,
            upper=0.015,
        ),
    )

    model.articulation(
        "baffle_to_thumb_screw",
        ArticulationType.FIXED,
        parent=baffle,
        child=thumb_screw,
        origin=Origin(xyz=(0.0, 0.030, 0.054)),
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

    ceiling_panel = object_model.get_part("ceiling_panel")
    housing = object_model.get_part("housing")
    baffle = object_model.get_part("baffle")
    thumb_screw = object_model.get_part("thumb_screw")
    slide = object_model.get_articulation("housing_to_baffle")

    lower = slide.motion_limits.lower if slide.motion_limits is not None else None
    upper = slide.motion_limits.upper if slide.motion_limits is not None else None

    ctx.expect_contact(
        housing,
        ceiling_panel,
        elem_a="trim_room",
        elem_b="panel_room_band",
        name="housing trim seats in the ceiling opening",
    )
    ctx.expect_contact(
        baffle,
        housing,
        elem_a="left_cheek",
        elem_b="left_track",
        name="baffle is physically supported by the left track",
    )
    ctx.expect_contact(
        thumb_screw,
        baffle,
        elem_a="thumb_head",
        elem_b="room_lip",
        name="thumb screw bears on the baffle lip",
    )
    ctx.expect_gap(
        housing,
        thumb_screw,
        axis="x",
        positive_elem="bridge_right",
        negative_elem="clamp_washer",
        min_gap=0.004,
        name="thumb screw washer clears the right side of the slot",
    )
    ctx.expect_gap(
        thumb_screw,
        housing,
        axis="x",
        positive_elem="clamp_washer",
        negative_elem="bridge_left",
        min_gap=0.004,
        name="thumb screw washer clears the left side of the slot",
    )

    lower_baffle_pos = None
    upper_baffle_pos = None
    lower_thumb_pos = None
    upper_thumb_pos = None

    if lower is not None:
        with ctx.pose({slide: lower}):
            ctx.expect_gap(
                baffle,
                housing,
                axis="y",
                positive_elem="wall_lip",
                negative_elem="wall_side_wall",
                min_gap=0.003,
                max_gap=0.006,
                name="baffle clears the wall-side housing at minimum spread",
            )
            lower_baffle_pos = ctx.part_world_position(baffle)
            lower_thumb_pos = ctx.part_world_position(thumb_screw)

    if upper is not None:
        with ctx.pose({slide: upper}):
            ctx.expect_gap(
                housing,
                baffle,
                axis="y",
                positive_elem="room_wall",
                negative_elem="room_lip",
                min_gap=0.001,
                max_gap=0.004,
                name="baffle clears the room-side housing at maximum spread",
            )
            upper_baffle_pos = ctx.part_world_position(baffle)
            upper_thumb_pos = ctx.part_world_position(thumb_screw)

    ctx.check(
        "baffle slides toward the room side across its travel",
        lower_baffle_pos is not None
        and upper_baffle_pos is not None
        and upper_baffle_pos[1] > lower_baffle_pos[1] + 0.025,
        details=f"lower={lower_baffle_pos}, upper={upper_baffle_pos}",
    )
    ctx.check(
        "thumb screw travels with the baffle carriage",
        lower_thumb_pos is not None
        and upper_thumb_pos is not None
        and upper_thumb_pos[1] > lower_thumb_pos[1] + 0.025,
        details=f"lower={lower_thumb_pos}, upper={upper_thumb_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
