from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="library_card_catalog_cabinet")

    oak = model.material("warm_oak", color=(0.50, 0.27, 0.11, 1.0))
    dark_oak = model.material("dark_oak_endgrain", color=(0.30, 0.15, 0.06, 1.0))
    brass = model.material("aged_brass", color=(0.82, 0.62, 0.25, 1.0))
    paper = model.material("typed_label_paper", color=(0.92, 0.86, 0.68, 1.0))
    shadow = model.material("dark_drawer_void", color=(0.055, 0.035, 0.020, 1.0))

    cols = 6
    rows = 8
    width = 0.90
    depth = 0.52
    height = 1.48
    side_t = 0.035
    top_t = 0.040
    bottom_t = 0.040
    divider_w = 0.018
    divider_h = 0.018
    back_t = 0.025
    front_y = -depth / 2.0
    face_depth = 0.050
    usable_w = width - 2.0 * side_t - (cols - 1) * divider_w
    usable_h = height - top_t - bottom_t - (rows - 1) * divider_h
    opening_w = usable_w / cols
    opening_h = usable_h / rows

    face_w = opening_w - 0.010
    face_h = opening_h - 0.012
    face_t = 0.020
    drawer_box_w = face_w - 0.016
    drawer_body_h = face_h - 0.035
    drawer_depth = 0.400
    drawer_side_t = 0.008
    drawer_bottom_t = 0.008
    drawer_back_t = 0.010
    rail_w = 0.010
    rail_h = 0.010
    # Long enough to disappear into the back panel, so every rail is visibly
    # carried by the carcass rather than floating as a loose strip.
    rail_depth = 0.470
    travel = 0.280

    cabinet = model.part("carcass")

    # Tall rectangular outer wooden carcass.
    cabinet.visual(
        Box((side_t, depth, height)),
        origin=Origin(xyz=(-width / 2.0 + side_t / 2.0, 0.0, height / 2.0)),
        material=oak,
        name="side_0",
    )
    cabinet.visual(
        Box((side_t, depth, height)),
        origin=Origin(xyz=(width / 2.0 - side_t / 2.0, 0.0, height / 2.0)),
        material=oak,
        name="side_1",
    )
    cabinet.visual(
        Box((width, depth, top_t)),
        origin=Origin(xyz=(0.0, 0.0, height - top_t / 2.0)),
        material=dark_oak,
        name="top_slab",
    )
    cabinet.visual(
        Box((width, depth, bottom_t)),
        origin=Origin(xyz=(0.0, 0.0, bottom_t / 2.0)),
        material=dark_oak,
        name="bottom_slab",
    )
    cabinet.visual(
        Box((width, back_t, height)),
        origin=Origin(xyz=(0.0, depth / 2.0 - back_t / 2.0, height / 2.0)),
        material=oak,
        name="back_panel",
    )

    # Front grid mullions and rails that define all forty-eight drawer openings.
    inner_x0 = -width / 2.0 + side_t
    inner_z0 = bottom_t
    grid_center_y = front_y + face_depth / 2.0
    for c in range(1, cols):
        x = inner_x0 + c * opening_w + (c - 0.5) * divider_w
        cabinet.visual(
            Box((divider_w, face_depth, height - top_t - bottom_t)),
            origin=Origin(xyz=(x, grid_center_y, height / 2.0)),
            material=dark_oak,
            name=f"vertical_mullion_{c}",
        )

    for r in range(1, rows):
        z = inner_z0 + r * opening_h + (r - 0.5) * divider_h
        cabinet.visual(
            Box((width - 2.0 * side_t, face_depth, divider_h)),
            origin=Origin(xyz=(0.0, grid_center_y, z)),
            material=dark_oak,
            name=f"horizontal_mullion_{r}",
        )

    def slot_center(col: int, row: int) -> tuple[float, float]:
        x = inner_x0 + col * (opening_w + divider_w) + opening_w / 2.0
        z = inner_z0 + row * (opening_h + divider_h) + opening_h / 2.0
        return x, z

    for row in range(rows):
        for col in range(cols):
            x, z = slot_center(col, row)
            rail_center_y = front_y + 0.035 + rail_depth / 2.0
            rail_center_z = z - drawer_body_h / 2.0 - rail_h / 2.0
            for side in (-1, 1):
                cabinet.visual(
                    Box((rail_w, rail_depth, rail_h)),
                    origin=Origin(
                        xyz=(
                            x + side * (drawer_box_w / 2.0 - rail_w / 2.0),
                            rail_center_y,
                            rail_center_z,
                        )
                    ),
                    material=oak,
                    name=f"guide_rail_{row}_{col}_{0 if side < 0 else 1}",
                )

    # Forty-eight independently sliding card drawers.
    for row in range(rows):
        for col in range(cols):
            x, z = slot_center(col, row)
            drawer = model.part(f"drawer_{row}_{col}")

            # The child frame is at the closed drawer-front center; +Y is the
            # hidden insertion direction, and the joint slides the drawer out
            # toward -Y.
            drawer.visual(
                Box((face_w, face_t, face_h)),
                origin=Origin(),
                material=oak,
                name="front_panel",
            )
            drawer.visual(
                Box((drawer_box_w, drawer_depth, drawer_bottom_t)),
                origin=Origin(
                    xyz=(
                        0.0,
                        face_t / 2.0 + drawer_depth / 2.0,
                        -drawer_body_h / 2.0 + drawer_bottom_t / 2.0,
                    )
                ),
                material=oak,
                name="bottom_board",
            )
            drawer.visual(
                Box((drawer_side_t, drawer_depth, drawer_body_h - drawer_bottom_t)),
                origin=Origin(
                    xyz=(
                        -drawer_box_w / 2.0 + drawer_side_t / 2.0,
                        face_t / 2.0 + drawer_depth / 2.0,
                        drawer_bottom_t / 2.0,
                    )
                ),
                material=oak,
                name="side_board_0",
            )
            drawer.visual(
                Box((drawer_side_t, drawer_depth, drawer_body_h - drawer_bottom_t)),
                origin=Origin(
                    xyz=(
                        drawer_box_w / 2.0 - drawer_side_t / 2.0,
                        face_t / 2.0 + drawer_depth / 2.0,
                        drawer_bottom_t / 2.0,
                    )
                ),
                material=oak,
                name="side_board_1",
            )
            drawer.visual(
                Box((drawer_box_w, drawer_back_t, drawer_body_h)),
                origin=Origin(
                    xyz=(
                        0.0,
                        face_t / 2.0 + drawer_depth - drawer_back_t / 2.0,
                        0.0,
                    )
                ),
                material=oak,
                name="back_board",
            )

            label_y = -face_t / 2.0 - 0.0025 + 0.0005
            drawer.visual(
                Box((face_w * 0.58, 0.004, face_h * 0.23)),
                origin=Origin(xyz=(0.0, label_y, face_h * 0.12)),
                material=paper,
                name="label_card",
            )
            drawer.visual(
                Box((face_w * 0.64, 0.005, 0.006)),
                origin=Origin(xyz=(0.0, label_y - 0.0008, face_h * 0.25)),
                material=brass,
                name="label_top_rail",
            )
            drawer.visual(
                Box((face_w * 0.64, 0.005, 0.006)),
                origin=Origin(xyz=(0.0, label_y - 0.0008, -face_h * 0.01)),
                material=brass,
                name="label_bottom_rail",
            )
            drawer.visual(
                Box((0.006, 0.005, face_h * 0.26)),
                origin=Origin(xyz=(-face_w * 0.32, label_y - 0.0008, face_h * 0.12)),
                material=brass,
                name="label_side_0",
            )
            drawer.visual(
                Box((0.006, 0.005, face_h * 0.26)),
                origin=Origin(xyz=(face_w * 0.32, label_y - 0.0008, face_h * 0.12)),
                material=brass,
                name="label_side_1",
            )
            drawer.visual(
                Box((face_w * 0.36, 0.012, 0.018)),
                origin=Origin(xyz=(0.0, -face_t / 2.0 - 0.006, -face_h * 0.22)),
                material=brass,
                name="finger_pull",
            )

            model.articulation(
                f"slide_{row}_{col}",
                ArticulationType.PRISMATIC,
                parent=cabinet,
                child=drawer,
                origin=Origin(xyz=(x, front_y - face_t / 2.0 - 0.002, z)),
                axis=(0.0, -1.0, 0.0),
                motion_limits=MotionLimits(effort=35.0, velocity=0.25, lower=0.0, upper=travel),
            )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    carcass = object_model.get_part("carcass")
    drawers = [object_model.get_part(f"drawer_{row}_{col}") for row in range(8) for col in range(6)]
    slides = [object_model.get_articulation(f"slide_{row}_{col}") for row in range(8) for col in range(6)]

    ctx.check("forty eight drawers", len(drawers) == 48)
    ctx.check("forty eight drawer slides", len(slides) == 48)
    ctx.check(
        "all slides are outward prismatic drawers",
        all(
            slide.articulation_type == ArticulationType.PRISMATIC
            and tuple(slide.axis) == (0.0, -1.0, 0.0)
            and slide.motion_limits is not None
            and slide.motion_limits.lower == 0.0
            and slide.motion_limits.upper is not None
            and slide.motion_limits.upper >= 0.27
            for slide in slides
        ),
    )
    rail_names = [visual.name for visual in carcass.visuals if visual.name.startswith("guide_rail_")]
    ctx.check("two guide rails for each drawer", len(rail_names) == 96)

    for row, col in ((0, 0), (0, 5), (3, 2), (7, 0), (7, 5)):
        drawer = object_model.get_part(f"drawer_{row}_{col}")
        slide = object_model.get_articulation(f"slide_{row}_{col}")
        rail = f"guide_rail_{row}_{col}_0"
        ctx.expect_gap(
            drawer,
            carcass,
            axis="z",
            max_gap=0.001,
            max_penetration=0.0,
            positive_elem="bottom_board",
            negative_elem=rail,
            name=f"drawer_{row}_{col} rests on guide rail",
        )
        ctx.expect_overlap(
            drawer,
            carcass,
            axes="y",
            min_overlap=0.34,
            elem_a="bottom_board",
            elem_b=rail,
            name=f"drawer_{row}_{col} rail engagement when closed",
        )
        rest_position = ctx.part_world_position(drawer)
        with ctx.pose({slide: slide.motion_limits.upper}):
            ctx.expect_overlap(
                drawer,
                carcass,
                axes="y",
                min_overlap=0.075,
                elem_a="bottom_board",
                elem_b=rail,
                name=f"drawer_{row}_{col} remains on rail when extended",
            )
            extended_position = ctx.part_world_position(drawer)
        ctx.check(
            f"drawer_{row}_{col} slides out toward the user",
            rest_position is not None
            and extended_position is not None
            and extended_position[1] < rest_position[1] - 0.25,
            details=f"rest={rest_position}, extended={extended_position}",
        )

    return ctx.report()


object_model = build_object_model()
