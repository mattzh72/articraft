from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    ExtrudeGeometry,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    rounded_rect_profile,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="retractable_sunroof_cassette")

    aluminum = model.material("aluminum", rgba=(0.73, 0.75, 0.78, 1.0))
    dark_coat = model.material("dark_coat", rgba=(0.18, 0.19, 0.21, 1.0))
    rail_finish = model.material("rail_finish", rgba=(0.32, 0.34, 0.36, 1.0))
    seal_rubber = model.material("seal_rubber", rgba=(0.08, 0.08, 0.09, 1.0))

    outer_width = 0.92
    border_width = 0.09
    channel_center_x = 0.384
    inner_guide_center_x = 0.358
    outer_wall_center_x = 0.451

    front_edge_y = -0.22
    opening_front_y = -0.17
    opening_rear_y = 0.21
    rear_end_y = 0.56

    cassette = model.part("cassette")

    cassette.visual(
        Box((border_width, rear_end_y - front_edge_y, 0.004)),
        origin=Origin(xyz=(-0.415, 0.17, 0.002)),
        material=dark_coat,
        name="left_gutter_floor",
    )
    cassette.visual(
        Box((border_width, rear_end_y - front_edge_y, 0.004)),
        origin=Origin(xyz=(0.415, 0.17, 0.002)),
        material=dark_coat,
        name="right_gutter_floor",
    )
    cassette.visual(
        Box((outer_width, opening_front_y - front_edge_y, 0.004)),
        origin=Origin(xyz=(0.0, -0.195, 0.002)),
        material=dark_coat,
        name="front_floor_bridge",
    )
    cassette.visual(
        Box((outer_width, rear_end_y - opening_rear_y, 0.004)),
        origin=Origin(xyz=(0.0, 0.385, 0.002)),
        material=dark_coat,
        name="rear_storage_floor",
    )

    cassette.visual(
        Box((0.018, rear_end_y - front_edge_y, 0.030)),
        origin=Origin(xyz=(-outer_wall_center_x, 0.17, 0.019)),
        material=rail_finish,
        name="left_outer_wall",
    )
    cassette.visual(
        Box((0.018, rear_end_y - front_edge_y, 0.030)),
        origin=Origin(xyz=(outer_wall_center_x, 0.17, 0.019)),
        material=rail_finish,
        name="right_outer_wall",
    )
    cassette.visual(
        Box((0.016, rear_end_y - (-0.20), 0.016)),
        origin=Origin(xyz=(-inner_guide_center_x, 0.18, 0.012)),
        material=rail_finish,
        name="left_inner_guide",
    )
    cassette.visual(
        Box((0.016, rear_end_y - (-0.20), 0.016)),
        origin=Origin(xyz=(inner_guide_center_x, 0.18, 0.012)),
        material=rail_finish,
        name="right_inner_guide",
    )
    cassette.visual(
        Box((0.040, rear_end_y - (-0.20), 0.004)),
        origin=Origin(xyz=(-channel_center_x, 0.18, 0.006)),
        material=aluminum,
        name="left_rail_shelf",
    )
    cassette.visual(
        Box((0.040, rear_end_y - (-0.20), 0.004)),
        origin=Origin(xyz=(channel_center_x, 0.18, 0.006)),
        material=aluminum,
        name="right_rail_shelf",
    )

    cassette.visual(
        Box((outer_width, opening_front_y - front_edge_y, 0.004)),
        origin=Origin(xyz=(0.0, -0.195, 0.032)),
        material=rail_finish,
        name="front_header",
    )
    cassette.visual(
        Box((border_width, opening_rear_y - opening_front_y, 0.004)),
        origin=Origin(xyz=(-0.415, 0.02, 0.032)),
        material=dark_coat,
        name="left_roof_border",
    )
    cassette.visual(
        Box((border_width, opening_rear_y - opening_front_y, 0.004)),
        origin=Origin(xyz=(0.415, 0.02, 0.032)),
        material=dark_coat,
        name="right_roof_border",
    )
    cassette.visual(
        Box((outer_width, rear_end_y - opening_rear_y, 0.004)),
        origin=Origin(xyz=(0.0, 0.385, 0.032)),
        material=dark_coat,
        name="rear_cavity_cover",
    )
    cassette.visual(
        Box((outer_width, 0.020, 0.030)),
        origin=Origin(xyz=(0.0, 0.57, 0.019)),
        material=rail_finish,
        name="rear_bulkhead",
    )
    cassette.visual(
        Box((0.740, 0.020, 0.004)),
        origin=Origin(xyz=(0.0, -0.170, 0.030)),
        material=seal_rubber,
        name="front_seal_strip",
    )
    cassette.visual(
        Box((0.740, 0.020, 0.004)),
        origin=Origin(xyz=(0.0, 0.210, 0.030)),
        material=seal_rubber,
        name="rear_seal_strip",
    )
    cassette.inertial = Inertial.from_geometry(
        Box((outer_width, rear_end_y - front_edge_y, 0.036)),
        mass=11.0,
        origin=Origin(xyz=(0.0, 0.17, 0.018)),
    )

    panel = model.part("panel")
    panel_skin_mesh = mesh_from_geometry(
        ExtrudeGeometry(
            rounded_rect_profile(0.734, 0.392, radius=0.020),
            0.010,
            cap=True,
            center=True,
            closed=True,
        ),
        "sunroof_panel_skin",
    )
    panel.visual(
        panel_skin_mesh,
        material=aluminum,
        name="panel_skin",
    )
    panel.visual(
        Box((0.024, 0.396, 0.012)),
        origin=Origin(xyz=(-0.378, 0.0, -0.010)),
        material=rail_finish,
        name="left_runner",
    )
    panel.visual(
        Box((0.024, 0.396, 0.012)),
        origin=Origin(xyz=(0.378, 0.0, -0.010)),
        material=rail_finish,
        name="right_runner",
    )
    panel.visual(
        Box((0.700, 0.018, 0.004)),
        origin=Origin(xyz=(0.0, -0.188, -0.002)),
        material=seal_rubber,
        name="leading_edge_seal",
    )
    panel.visual(
        Box((0.700, 0.018, 0.004)),
        origin=Origin(xyz=(0.0, 0.188, -0.002)),
        material=seal_rubber,
        name="trailing_edge_seal",
    )
    panel.inertial = Inertial.from_geometry(
        Box((0.756, 0.400, 0.022)),
        mass=5.8,
        origin=Origin(xyz=(0.0, 0.0, -0.001)),
    )

    model.articulation(
        "cassette_to_panel",
        ArticulationType.PRISMATIC,
        parent=cassette,
        child=panel,
        origin=Origin(xyz=(0.0, 0.02, 0.024)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=180.0,
            velocity=0.25,
            lower=0.0,
            upper=0.34,
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
    cassette = object_model.get_part("cassette")
    panel = object_model.get_part("panel")
    slide = object_model.get_articulation("cassette_to_panel")

    rear_cover = cassette.get_visual("rear_cavity_cover")
    rear_floor = cassette.get_visual("rear_storage_floor")
    left_shelf = cassette.get_visual("left_rail_shelf")
    right_shelf = cassette.get_visual("right_rail_shelf")
    panel_skin = panel.get_visual("panel_skin")
    left_runner = panel.get_visual("left_runner")
    right_runner = panel.get_visual("right_runner")

    ctx.check(
        "sunroof cassette parts exist",
        cassette is not None and panel is not None and slide is not None,
        details="Expected cassette root, sliding panel, and prismatic guide articulation.",
    )

    upper = slide.motion_limits.upper if slide.motion_limits is not None else 0.34
    with ctx.pose({slide: 0.0}):
        ctx.expect_within(
            panel,
            cassette,
            axes="x",
            inner_elem=left_runner,
            outer_elem=left_shelf,
            margin=0.0,
            name="left runner sits laterally on left rail when closed",
        )
        ctx.expect_within(
            panel,
            cassette,
            axes="x",
            inner_elem=right_runner,
            outer_elem=right_shelf,
            margin=0.0,
            name="right runner sits laterally on right rail when closed",
        )
        ctx.expect_overlap(
            panel,
            cassette,
            axes="y",
            elem_a=left_runner,
            elem_b=left_shelf,
            min_overlap=0.35,
            name="left runner remains captured along left rail when closed",
        )
        ctx.expect_overlap(
            panel,
            cassette,
            axes="y",
            elem_a=right_runner,
            elem_b=right_shelf,
            min_overlap=0.35,
            name="right runner remains captured along right rail when closed",
        )
        closed_pos = ctx.part_world_position(panel)

    with ctx.pose({slide: upper}):
        ctx.expect_overlap(
            panel,
            cassette,
            axes="xy",
            elem_a=panel_skin,
            elem_b=rear_cover,
            min_overlap=0.18,
            name="open panel stores below rear roof cover",
        )
        ctx.expect_gap(
            cassette,
            panel,
            axis="z",
            positive_elem=rear_cover,
            negative_elem=panel_skin,
            min_gap=0.0005,
            max_gap=0.006,
            name="rear cover clears stored panel vertically",
        )
        ctx.expect_gap(
            panel,
            cassette,
            axis="z",
            positive_elem=panel_skin,
            negative_elem=rear_floor,
            min_gap=0.012,
            max_gap=0.030,
            name="stored panel rides above rear cassette floor",
        )
        ctx.expect_overlap(
            panel,
            cassette,
            axes="y",
            elem_a=left_runner,
            elem_b=left_shelf,
            min_overlap=0.05,
            name="left runner retains insertion on left rail when open",
        )
        ctx.expect_overlap(
            panel,
            cassette,
            axes="y",
            elem_a=right_runner,
            elem_b=right_shelf,
            min_overlap=0.05,
            name="right runner retains insertion on right rail when open",
        )
        open_pos = ctx.part_world_position(panel)

    ctx.check(
        "panel slides rearward into storage cavity",
        closed_pos is not None
        and open_pos is not None
        and open_pos[1] > closed_pos[1] + 0.30,
        details=f"closed={closed_pos}, open={open_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
