from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    ExtrudeWithHolesGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def _offset_profile(
    profile: list[tuple[float, float]],
    *,
    dx: float = 0.0,
    dy: float = 0.0,
) -> list[tuple[float, float]]:
    return [(x + dx, y + dy) for x, y in profile]


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="vent_only_skylight_sunroof_cassette")

    frame_aluminum = model.material("frame_aluminum", rgba=(0.24, 0.26, 0.29, 1.0))
    rail_steel = model.material("rail_steel", rgba=(0.60, 0.62, 0.66, 1.0))
    tray_black = model.material("tray_black", rgba=(0.09, 0.10, 0.11, 1.0))
    seal_black = model.material("seal_black", rgba=(0.05, 0.05, 0.06, 1.0))
    glass_tint = model.material("glass_tint", rgba=(0.22, 0.34, 0.42, 0.52))

    outer_width = 0.46
    outer_length = 0.62
    total_height = 0.065
    floor_thickness = 0.010
    wall_height = 0.045
    bezel_thickness = 0.010
    side_wall_thickness = 0.030

    fixed_open_width = 0.352
    fixed_open_length = 0.176
    fixed_open_center_y = -0.190

    vent_open_width = 0.352
    vent_open_length = 0.156
    vent_open_center_y = 0.005

    cassette_frame = model.part("cassette_frame")
    cassette_frame.inertial = Inertial.from_geometry(
        Box((outer_width, outer_length, total_height)),
        mass=8.5,
        origin=Origin(xyz=(0.0, 0.0, total_height * 0.5)),
    )

    top_bezel = ExtrudeWithHolesGeometry(
        rounded_rect_profile(outer_width, outer_length, 0.018, corner_segments=8),
        [
            _offset_profile(
                rounded_rect_profile(fixed_open_width, fixed_open_length, 0.008, corner_segments=6),
                dy=fixed_open_center_y,
            ),
            _offset_profile(
                rounded_rect_profile(vent_open_width, vent_open_length, 0.008, corner_segments=6),
                dy=vent_open_center_y,
            ),
        ],
        height=bezel_thickness,
        center=True,
    )
    cassette_frame.visual(
        mesh_from_geometry(top_bezel, "sunroof_top_bezel"),
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
        material=frame_aluminum,
        name="top_bezel",
    )
    cassette_frame.visual(
        Box((outer_width, outer_length, floor_thickness)),
        origin=Origin(xyz=(0.0, 0.0, floor_thickness * 0.5)),
        material=tray_black,
        name="lower_pan",
    )
    cassette_frame.visual(
        Box((side_wall_thickness, outer_length, wall_height)),
        origin=Origin(xyz=(outer_width * 0.5 - side_wall_thickness * 0.5, 0.0, 0.0325)),
        material=frame_aluminum,
        name="right_side_wall",
    )
    cassette_frame.visual(
        Box((side_wall_thickness, outer_length, wall_height)),
        origin=Origin(xyz=(-outer_width * 0.5 + side_wall_thickness * 0.5, 0.0, 0.0325)),
        material=frame_aluminum,
        name="left_side_wall",
    )
    cassette_frame.visual(
        Box((outer_width - 2.0 * side_wall_thickness, 0.030, wall_height)),
        origin=Origin(xyz=(0.0, -0.295, 0.0325)),
        material=frame_aluminum,
        name="front_header",
    )
    cassette_frame.visual(
        Box((outer_width - 2.0 * side_wall_thickness, 0.030, wall_height)),
        origin=Origin(xyz=(0.0, -0.085, 0.0325)),
        material=frame_aluminum,
        name="center_mullion",
    )
    cassette_frame.visual(
        Box((outer_width - 2.0 * side_wall_thickness, 0.040, wall_height)),
        origin=Origin(xyz=(0.0, 0.290, 0.0325)),
        material=frame_aluminum,
        name="rear_header",
    )

    cassette_frame.visual(
        Box((0.014, 0.350, 0.025)),
        origin=Origin(xyz=(-0.165, 0.090, 0.0225)),
        material=rail_steel,
        name="left_guide_rail",
    )
    cassette_frame.visual(
        Box((0.014, 0.350, 0.025)),
        origin=Origin(xyz=(0.165, 0.090, 0.0225)),
        material=rail_steel,
        name="right_guide_rail",
    )

    cassette_frame.visual(
        Box((0.014, 0.176, 0.008)),
        origin=Origin(xyz=(-0.165, fixed_open_center_y, 0.051)),
        material=seal_black,
        name="fixed_left_ledge",
    )
    cassette_frame.visual(
        Box((0.014, 0.176, 0.008)),
        origin=Origin(xyz=(0.165, fixed_open_center_y, 0.051)),
        material=seal_black,
        name="fixed_right_ledge",
    )
    cassette_frame.visual(
        Box((0.330, 0.012, 0.008)),
        origin=Origin(xyz=(0.0, -0.272, 0.051)),
        material=seal_black,
        name="fixed_front_ledge",
    )
    cassette_frame.visual(
        Box((0.330, 0.012, 0.008)),
        origin=Origin(xyz=(0.0, -0.108, 0.051)),
        material=seal_black,
        name="fixed_rear_ledge",
    )

    fixed_glass = model.part("fixed_glass")
    fixed_glass.inertial = Inertial.from_geometry(
        Box((0.344, 0.172, 0.009)),
        mass=1.3,
    )
    fixed_glass.visual(
        Box((0.344, 0.172, 0.003)),
        origin=Origin(xyz=(0.0, 0.0, -0.0045)),
        material=seal_black,
        name="fixed_seal_bed",
    )
    fixed_glass.visual(
        Box((0.340, 0.168, 0.006)),
        origin=Origin(),
        material=glass_tint,
        name="fixed_pane",
    )

    vent_insert = model.part("vent_insert")
    vent_insert.inertial = Inertial.from_geometry(
        Box((0.344, 0.200, 0.028)),
        mass=1.5,
        origin=Origin(xyz=(0.0, 0.020, -0.007)),
    )
    vent_insert.visual(
        Box((0.344, 0.146, 0.003)),
        origin=Origin(xyz=(0.0, 0.0, -0.0085)),
        material=seal_black,
        name="vent_seal_bed",
    )
    vent_insert.visual(
        Box((0.018, 0.146, 0.014)),
        origin=Origin(xyz=(-0.163, 0.0, 0.0)),
        material=frame_aluminum,
        name="vent_left_frame",
    )
    vent_insert.visual(
        Box((0.018, 0.146, 0.014)),
        origin=Origin(xyz=(0.163, 0.0, 0.0)),
        material=frame_aluminum,
        name="vent_right_frame",
    )
    vent_insert.visual(
        Box((0.344, 0.014, 0.014)),
        origin=Origin(xyz=(0.0, -0.064, 0.0)),
        material=frame_aluminum,
        name="vent_front_frame",
    )
    vent_insert.visual(
        Box((0.344, 0.014, 0.014)),
        origin=Origin(xyz=(0.0, 0.064, 0.0)),
        material=frame_aluminum,
        name="vent_rear_frame",
    )
    vent_insert.visual(
        Box((0.310, 0.126, 0.005)),
        origin=Origin(xyz=(0.0, 0.0, 0.0025)),
        material=glass_tint,
        name="vent_pane",
    )
    vent_insert.visual(
        Box((0.014, 0.180, 0.014)),
        origin=Origin(xyz=(-0.165, 0.050, -0.014)),
        material=rail_steel,
        name="left_slider",
    )
    vent_insert.visual(
        Box((0.014, 0.180, 0.014)),
        origin=Origin(xyz=(0.165, 0.050, -0.014)),
        material=rail_steel,
        name="right_slider",
    )
    model.articulation(
        "frame_to_fixed_glass",
        ArticulationType.FIXED,
        parent=cassette_frame,
        child=fixed_glass,
        origin=Origin(xyz=(0.0, fixed_open_center_y, 0.058)),
    )
    model.articulation(
        "frame_to_vent_insert",
        ArticulationType.PRISMATIC,
        parent=cassette_frame,
        child=vent_insert,
        origin=Origin(xyz=(0.0, vent_open_center_y, 0.056)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=40.0,
            velocity=0.15,
            lower=0.0,
            upper=0.130,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    cassette_frame = object_model.get_part("cassette_frame")
    fixed_glass = object_model.get_part("fixed_glass")
    vent_insert = object_model.get_part("vent_insert")
    vent_slide = object_model.get_articulation("frame_to_vent_insert")

    ctx.expect_contact(
        fixed_glass,
        cassette_frame,
        elem_a="fixed_seal_bed",
        elem_b="fixed_front_ledge",
        name="fixed glass seats on the front ledge",
    )
    ctx.expect_contact(
        fixed_glass,
        cassette_frame,
        elem_a="fixed_seal_bed",
        elem_b="fixed_rear_ledge",
        name="fixed glass seats on the rear ledge",
    )

    with ctx.pose({vent_slide: 0.0}):
        ctx.expect_contact(
            vent_insert,
            cassette_frame,
            elem_a="left_slider",
            elem_b="left_guide_rail",
            name="left slider rests on the left guide rail when closed",
        )
        ctx.expect_contact(
            vent_insert,
            cassette_frame,
            elem_a="right_slider",
            elem_b="right_guide_rail",
            name="right slider rests on the right guide rail when closed",
        )
        ctx.expect_overlap(
            vent_insert,
            cassette_frame,
            axes="x",
            elem_a="left_slider",
            elem_b="left_guide_rail",
            min_overlap=0.012,
            name="left slider is centered over the left guide rail",
        )
        ctx.expect_overlap(
            vent_insert,
            cassette_frame,
            axes="x",
            elem_a="right_slider",
            elem_b="right_guide_rail",
            min_overlap=0.012,
            name="right slider is centered over the right guide rail",
        )
        ctx.expect_overlap(
            vent_insert,
            cassette_frame,
            axes="y",
            elem_a="left_slider",
            elem_b="left_guide_rail",
            min_overlap=0.160,
            name="closed vent insert retains substantial left rail engagement",
        )
        ctx.expect_overlap(
            vent_insert,
            cassette_frame,
            axes="y",
            elem_a="right_slider",
            elem_b="right_guide_rail",
            min_overlap=0.160,
            name="closed vent insert retains substantial right rail engagement",
        )
        rest_pos = ctx.part_world_position(vent_insert)

    with ctx.pose({vent_slide: 0.130}):
        ctx.expect_contact(
            vent_insert,
            cassette_frame,
            elem_a="left_slider",
            elem_b="left_guide_rail",
            name="left slider stays supported on the left guide rail when open",
        )
        ctx.expect_contact(
            vent_insert,
            cassette_frame,
            elem_a="right_slider",
            elem_b="right_guide_rail",
            name="right slider stays supported on the right guide rail when open",
        )
        ctx.expect_overlap(
            vent_insert,
            cassette_frame,
            axes="y",
            elem_a="left_slider",
            elem_b="left_guide_rail",
            min_overlap=0.150,
            name="open vent insert still retains left rail insertion",
        )
        ctx.expect_overlap(
            vent_insert,
            cassette_frame,
            axes="y",
            elem_a="right_slider",
            elem_b="right_guide_rail",
            min_overlap=0.150,
            name="open vent insert still retains right rail insertion",
        )
        open_pos = ctx.part_world_position(vent_insert)

    ctx.check(
        "vent insert slides rearward",
        rest_pos is not None and open_pos is not None and open_pos[1] > rest_pos[1] + 0.10,
        details=f"rest={rest_pos}, open={open_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
