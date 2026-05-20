from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import cadquery as cq

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


GUIDE_LENGTH = 0.58
GUIDE_DEPTH = 0.12
GUIDE_HEIGHT = 0.09
GUIDE_CENTER_Z = 0.255
GUIDE_CAVITY_DEPTH = 0.078
GUIDE_CAVITY_HEIGHT = 0.054
SLOT_LENGTH = 0.50
SLOT_WIDTH = 0.028
SLOT_HEIGHT = 0.050
GUIDE_SIDE_WALL = (GUIDE_DEPTH - GUIDE_CAVITY_DEPTH) / 2.0
GUIDE_CAP_THICKNESS = (GUIDE_HEIGHT - GUIDE_CAVITY_HEIGHT) / 2.0
BOTTOM_RAIL_WIDTH = (GUIDE_CAVITY_DEPTH - SLOT_WIDTH) / 2.0

SUPPORT_X = 0.24
FOOT_LENGTH = 0.16
FOOT_DEPTH = 0.22
FOOT_HEIGHT = 0.03
UPRIGHT_LENGTH = 0.08
UPRIGHT_DEPTH = 0.12
UPRIGHT_HEIGHT = GUIDE_CENTER_Z - GUIDE_HEIGHT / 2.0 - FOOT_HEIGHT

SHUTTLE_LENGTH = 0.14
SHUTTLE_DEPTH = 0.072
SHUTTLE_HEIGHT = GUIDE_CAVITY_HEIGHT
NECK_LENGTH = 0.08
NECK_DEPTH = 0.018
NECK_HEIGHT = 0.064
PLATE_LENGTH = 0.16
PLATE_DEPTH = 0.11
PLATE_HEIGHT = 0.02
PLATE_CENTER_Z = -(SHUTTLE_HEIGHT / 2.0 + NECK_HEIGHT + PLATE_HEIGHT / 2.0)

TRAVEL_LIMIT = 0.19


def _box(size: tuple[float, float, float], center: tuple[float, float, float]) -> cq.Workplane:
    return cq.Workplane("XY").box(*size).translate(center)


def _build_support(center_x: float) -> cq.Workplane:
    foot = _box(
        (FOOT_LENGTH, FOOT_DEPTH, FOOT_HEIGHT),
        (center_x, 0.0, FOOT_HEIGHT / 2.0),
    )
    upright = _box(
        (UPRIGHT_LENGTH, UPRIGHT_DEPTH, UPRIGHT_HEIGHT),
        (center_x, 0.0, FOOT_HEIGHT + UPRIGHT_HEIGHT / 2.0),
    )
    return foot.union(upright)


def _build_guide_channel() -> cq.Workplane:
    channel = _box(
        (GUIDE_LENGTH, GUIDE_DEPTH, GUIDE_HEIGHT),
        (0.0, 0.0, GUIDE_CENTER_Z),
    )
    cavity = _box(
        (GUIDE_LENGTH + 0.04, GUIDE_CAVITY_DEPTH, GUIDE_CAVITY_HEIGHT),
        (0.0, 0.0, GUIDE_CENTER_Z),
    )
    slot = _box(
        (SLOT_LENGTH, SLOT_WIDTH, SLOT_HEIGHT),
        (0.0, 0.0, GUIDE_CENTER_Z - GUIDE_HEIGHT / 2.0 + SLOT_HEIGHT / 2.0),
    )
    return channel.cut(cavity).cut(slot)


def _build_shuttle_block() -> cq.Workplane:
    return cq.Workplane("XY").box(SHUTTLE_LENGTH, SHUTTLE_DEPTH, SHUTTLE_HEIGHT)


def _build_hanger_neck() -> cq.Workplane:
    neck_center_z = -(SHUTTLE_HEIGHT / 2.0 + NECK_HEIGHT / 2.0)
    return _box((NECK_LENGTH, NECK_DEPTH, NECK_HEIGHT), (0.0, 0.0, neck_center_z))


def _build_carriage_plate() -> cq.Workplane:
    plate = (
        cq.Workplane("XY")
        .box(PLATE_LENGTH, PLATE_DEPTH, PLATE_HEIGHT)
        .faces(">Z")
        .workplane()
        .pushPoints([(-0.045, 0.0), (0.045, 0.0)])
        .hole(0.012)
        .edges("|Z")
        .fillet(0.01)
    )
    return plate.translate((0.0, 0.0, PLATE_CENTER_Z))


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bridge_guided_linear_carriage")

    model.material("frame_steel", rgba=(0.23, 0.25, 0.28, 1.0))
    model.material("guide_finish", rgba=(0.38, 0.41, 0.45, 1.0))
    model.material("slider_steel", rgba=(0.70, 0.72, 0.75, 1.0))
    model.material("mount_plate", rgba=(0.80, 0.82, 0.84, 1.0))

    frame = model.part("frame")
    frame.visual(
        mesh_from_cadquery(_build_support(-SUPPORT_X), "left_support"),
        material="frame_steel",
        name="left_support",
    )
    frame.visual(
        mesh_from_cadquery(_build_support(SUPPORT_X), "right_support"),
        material="frame_steel",
        name="right_support",
    )
    frame.visual(
        Box((GUIDE_LENGTH, GUIDE_DEPTH, GUIDE_CAP_THICKNESS)),
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                GUIDE_CENTER_Z + GUIDE_CAVITY_HEIGHT / 2.0 + GUIDE_CAP_THICKNESS / 2.0,
            )
        ),
        material="guide_finish",
        name="guide_top",
    )
    frame.visual(
        Box((GUIDE_LENGTH, GUIDE_SIDE_WALL, GUIDE_HEIGHT)),
        origin=Origin(
            xyz=(0.0, GUIDE_CAVITY_DEPTH / 2.0 + GUIDE_SIDE_WALL / 2.0, GUIDE_CENTER_Z)
        ),
        material="guide_finish",
        name="guide_right_wall",
    )
    frame.visual(
        Box((GUIDE_LENGTH, GUIDE_SIDE_WALL, GUIDE_HEIGHT)),
        origin=Origin(
            xyz=(0.0, -(GUIDE_CAVITY_DEPTH / 2.0 + GUIDE_SIDE_WALL / 2.0), GUIDE_CENTER_Z)
        ),
        material="guide_finish",
        name="guide_left_wall",
    )
    frame.visual(
        Box((GUIDE_LENGTH, BOTTOM_RAIL_WIDTH, GUIDE_CAP_THICKNESS)),
        origin=Origin(
            xyz=(
                0.0,
                SLOT_WIDTH / 2.0 + BOTTOM_RAIL_WIDTH / 2.0,
                GUIDE_CENTER_Z - GUIDE_CAVITY_HEIGHT / 2.0 - GUIDE_CAP_THICKNESS / 2.0,
            )
        ),
        material="guide_finish",
        name="guide_right_rail",
    )
    frame.visual(
        Box((GUIDE_LENGTH, BOTTOM_RAIL_WIDTH, GUIDE_CAP_THICKNESS)),
        origin=Origin(
            xyz=(
                0.0,
                -(SLOT_WIDTH / 2.0 + BOTTOM_RAIL_WIDTH / 2.0),
                GUIDE_CENTER_Z - GUIDE_CAVITY_HEIGHT / 2.0 - GUIDE_CAP_THICKNESS / 2.0,
            )
        ),
        material="guide_finish",
        name="guide_left_rail",
    )
    frame.inertial = Inertial.from_geometry(
        Box((0.64, FOOT_DEPTH, GUIDE_CENTER_Z + GUIDE_HEIGHT / 2.0)),
        mass=18.0,
        origin=Origin(xyz=(0.0, 0.0, (GUIDE_CENTER_Z + GUIDE_HEIGHT / 2.0) / 2.0)),
    )

    slider = model.part("slider")
    slider.visual(
        mesh_from_cadquery(_build_shuttle_block(), "shuttle_block"),
        material="slider_steel",
        name="shuttle_block",
    )
    slider.visual(
        mesh_from_cadquery(_build_hanger_neck(), "hanger_neck"),
        material="slider_steel",
        name="hanger_neck",
    )
    slider.visual(
        mesh_from_cadquery(_build_carriage_plate(), "carriage_plate"),
        material="mount_plate",
        name="carriage_plate",
    )
    slider.inertial = Inertial.from_geometry(
        Box((PLATE_LENGTH, PLATE_DEPTH, SHUTTLE_HEIGHT + NECK_HEIGHT + PLATE_HEIGHT)),
        mass=4.0,
        origin=Origin(
            xyz=(
                0.0,
                0.0,
                (PLATE_CENTER_Z - PLATE_HEIGHT / 2.0 + SHUTTLE_HEIGHT / 2.0) / 2.0,
            )
        ),
    )

    model.articulation(
        "frame_to_slider",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=slider,
        origin=Origin(xyz=(0.0, 0.0, GUIDE_CENTER_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=-TRAVEL_LIMIT,
            upper=TRAVEL_LIMIT,
            effort=400.0,
            velocity=0.4,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    frame = object_model.get_part("frame")
    slider = object_model.get_part("slider")
    slide = object_model.get_articulation("frame_to_slider")
    guide_top = frame.get_visual("guide_top")
    guide_left_wall = frame.get_visual("guide_left_wall")
    guide_right_wall = frame.get_visual("guide_right_wall")
    guide_right_rail = frame.get_visual("guide_right_rail")
    shuttle = slider.get_visual("shuttle_block")
    plate = slider.get_visual("carriage_plate")

    ctx.expect_contact(
        frame,
        slider,
        elem_a=guide_top,
        elem_b=shuttle,
        contact_tol=1e-6,
        name="shuttle is carried by the guide roof",
    )
    ctx.expect_gap(
        frame,
        slider,
        axis="y",
        positive_elem=guide_right_wall,
        negative_elem=shuttle,
        min_gap=0.0025,
        max_gap=0.0035,
        name="right wall leaves a slim running clearance",
    )
    ctx.expect_gap(
        slider,
        frame,
        axis="y",
        positive_elem=shuttle,
        negative_elem=guide_left_wall,
        min_gap=0.0025,
        max_gap=0.0035,
        name="left wall leaves a slim running clearance",
    )
    ctx.expect_overlap(
        slider,
        frame,
        axes="x",
        elem_a=shuttle,
        elem_b=guide_top,
        min_overlap=0.12,
        name="shuttle is fully inserted at rest",
    )
    ctx.expect_gap(
        frame,
        slider,
        axis="z",
        positive_elem=guide_right_rail,
        negative_elem=plate,
        min_gap=0.04,
        max_gap=0.05,
        name="carriage plate hangs below the guide slot",
    )

    rest_pos = ctx.part_world_position(slider)
    upper = slide.motion_limits.upper if slide.motion_limits is not None else None
    with ctx.pose({slide: upper}):
        ctx.expect_contact(
            frame,
            slider,
            elem_a=guide_top,
            elem_b=shuttle,
            contact_tol=1e-6,
            name="shuttle stays supported at max extension",
        )
        ctx.expect_overlap(
            slider,
            frame,
            axes="x",
            elem_a=shuttle,
            elem_b=guide_top,
            min_overlap=0.06,
            name="shuttle retains insertion at max extension",
        )
        extended_pos = ctx.part_world_position(slider)

    ctx.check(
        "slider extends positively along the guide axis",
        rest_pos is not None
        and extended_pos is not None
        and extended_pos[0] > rest_pos[0] + 0.18
        and abs(extended_pos[1] - rest_pos[1]) < 1e-6
        and abs(extended_pos[2] - rest_pos[2]) < 1e-6,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
