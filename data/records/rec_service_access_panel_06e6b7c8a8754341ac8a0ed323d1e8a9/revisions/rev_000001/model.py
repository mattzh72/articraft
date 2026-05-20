from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="service_access_panel")

    painted_steel = model.material("painted_steel", rgba=(0.66, 0.68, 0.70, 1.0))
    dark_frame = model.material("dark_frame", rgba=(0.28, 0.30, 0.32, 1.0))
    latch_black = model.material("latch_black", rgba=(0.10, 0.11, 0.12, 1.0))

    outer_width = 1.05
    outer_height = 1.35
    face_thickness = 0.035
    opening_width = 0.56
    opening_height = 0.86

    side_band = (outer_width - opening_width) * 0.5
    top_band = (outer_height - opening_height) * 0.5

    bezel_outer_width = 0.70
    bezel_outer_height = 1.00
    bezel_band = 0.08
    bezel_thickness = 0.012

    visible_opening_width = bezel_outer_width - 2.0 * bezel_band
    visible_opening_height = bezel_outer_height - 2.0 * bezel_band

    stop_lip_depth = 0.008
    stop_lip_band = 0.020

    frame = model.part("equipment_face")
    frame.inertial = Inertial.from_geometry(
        Box((outer_width, face_thickness, outer_height)),
        mass=24.0,
        origin=Origin(),
    )

    frame.visual(
        Box((outer_width, face_thickness, top_band)),
        origin=Origin(xyz=(0.0, 0.0, opening_height * 0.5 + top_band * 0.5)),
        material=dark_frame,
        name="top_face",
    )
    frame.visual(
        Box((outer_width, face_thickness, top_band)),
        origin=Origin(xyz=(0.0, 0.0, -(opening_height * 0.5 + top_band * 0.5))),
        material=dark_frame,
        name="bottom_face",
    )
    frame.visual(
        Box((side_band, face_thickness, opening_height)),
        origin=Origin(xyz=(-(opening_width * 0.5 + side_band * 0.5), 0.0, 0.0)),
        material=dark_frame,
        name="hinge_jamb",
    )
    frame.visual(
        Box((side_band, face_thickness, opening_height)),
        origin=Origin(xyz=(opening_width * 0.5 + side_band * 0.5, 0.0, 0.0)),
        material=dark_frame,
        name="latch_jamb",
    )

    bezel_y = face_thickness * 0.5 + bezel_thickness * 0.5
    frame.visual(
        Box((bezel_outer_width, bezel_thickness, bezel_band)),
        origin=Origin(xyz=(0.0, bezel_y, visible_opening_height * 0.5 + bezel_band * 0.5)),
        material=painted_steel,
        name="top_bezel",
    )
    frame.visual(
        Box((bezel_outer_width, bezel_thickness, bezel_band)),
        origin=Origin(xyz=(0.0, bezel_y, -(visible_opening_height * 0.5 + bezel_band * 0.5))),
        material=painted_steel,
        name="bottom_bezel",
    )
    frame.visual(
        Box((bezel_band, bezel_thickness, bezel_outer_height)),
        origin=Origin(xyz=(-(visible_opening_width * 0.5 + bezel_band * 0.5), bezel_y, 0.0)),
        material=painted_steel,
        name="hinge_bezel",
    )
    frame.visual(
        Box((bezel_band, bezel_thickness, bezel_outer_height)),
        origin=Origin(xyz=(visible_opening_width * 0.5 + bezel_band * 0.5, bezel_y, 0.0)),
        material=painted_steel,
        name="latch_bezel",
    )
    frame.visual(
        Box((0.028, bezel_thickness, 0.90)),
        origin=Origin(xyz=(-0.284, bezel_y, 0.0)),
        material=dark_frame,
        name="hinge_cover",
    )

    stop_y = -(face_thickness * 0.5) + stop_lip_depth * 0.5
    frame.visual(
        Box((opening_width, stop_lip_depth, stop_lip_band)),
        origin=Origin(xyz=(0.0, stop_y, opening_height * 0.5 - stop_lip_band * 0.5)),
        material=latch_black,
        name="rear_stop_top",
    )
    frame.visual(
        Box((opening_width, stop_lip_depth, stop_lip_band)),
        origin=Origin(xyz=(0.0, stop_y, -(opening_height * 0.5 - stop_lip_band * 0.5))),
        material=latch_black,
        name="rear_stop_bottom",
    )
    frame.visual(
        Box((stop_lip_band, stop_lip_depth, opening_height)),
        origin=Origin(xyz=(-(opening_width * 0.5 - stop_lip_band * 0.5), stop_y, 0.0)),
        material=latch_black,
        name="rear_stop_hinge",
    )
    frame.visual(
        Box((stop_lip_band, stop_lip_depth, opening_height)),
        origin=Origin(xyz=(opening_width * 0.5 - stop_lip_band * 0.5, stop_y, 0.0)),
        material=latch_black,
        name="rear_stop_latch",
    )

    door = model.part("service_door")
    door_width = 0.536
    door_height = 0.828
    door_skin_thickness = 0.018
    inner_pan_thickness = 0.008

    door.visual(
        Box((door_width, door_skin_thickness, door_height)),
        origin=Origin(xyz=(door_width * 0.5, 0.009, 0.0)),
        material=painted_steel,
        name="door_skin",
    )
    door.visual(
        Box((0.486, inner_pan_thickness, 0.764)),
        origin=Origin(xyz=(0.264, -0.004, 0.0)),
        material=dark_frame,
        name="door_pan",
    )
    door.visual(
        Box((0.018, 0.010, 0.180)),
        origin=Origin(xyz=(door_width - 0.014, 0.023, 0.0)),
        material=latch_black,
        name="latch_pull",
    )
    door.inertial = Inertial.from_geometry(
        Box((door_width, 0.028, door_height)),
        mass=8.0,
        origin=Origin(xyz=(door_width * 0.5, 0.004, 0.0)),
    )

    model.articulation(
        "frame_to_door",
        ArticulationType.REVOLUTE,
        parent=frame,
        child=door,
        origin=Origin(xyz=(-0.270, 0.0, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=20.0, velocity=1.4, lower=0.0, upper=2.10),
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

    frame = object_model.get_part("equipment_face")
    door = object_model.get_part("service_door")
    hinge = object_model.get_articulation("frame_to_door")

    with ctx.pose({hinge: 0.0}):
        ctx.expect_gap(
            door,
            frame,
            axis="x",
            positive_elem="door_skin",
            negative_elem="hinge_bezel",
            max_gap=0.0005,
            max_penetration=1e-5,
            name="hinge side closes tightly against the hinge bezel",
        )
        ctx.expect_gap(
            frame,
            door,
            axis="x",
            positive_elem="latch_bezel",
            negative_elem="door_skin",
            min_gap=0.003,
            max_gap=0.0055,
            name="latch-side reveal stays narrow",
        )
        ctx.expect_gap(
            frame,
            door,
            axis="z",
            positive_elem="top_bezel",
            negative_elem="door_skin",
            min_gap=0.005,
            max_gap=0.0075,
            name="top reveal is realistic",
        )
        ctx.expect_gap(
            door,
            frame,
            axis="y",
            positive_elem="door_pan",
            negative_elem="rear_stop_latch",
            min_gap=0.001,
            max_gap=0.003,
            name="door rests just in front of the stop lip",
        )

    closed_skin = ctx.part_element_world_aabb(door, elem="door_skin")
    with ctx.pose({hinge: 1.60}):
        opened_skin = ctx.part_element_world_aabb(door, elem="door_skin")

    ctx.check(
        "door swings outward from the equipment face",
        closed_skin is not None
        and opened_skin is not None
        and opened_skin[1][1] > closed_skin[1][1] + 0.20,
        details=f"closed={closed_skin}, opened={opened_skin}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
