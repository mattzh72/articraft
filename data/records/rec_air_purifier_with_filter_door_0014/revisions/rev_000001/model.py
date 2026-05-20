from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

ASSETS = AssetContext.from_script(__file__)

BODY_W = 0.72
BODY_D = 0.28
BODY_H = 0.62
WALL = 0.022
FRAME_D = 0.018
SIDE_MARGIN = 0.040
CENTER_DIVIDER = 0.030
TOP_MARGIN = 0.060
BOTTOM_MARGIN = 0.060

OPENING_W = (BODY_W - (2.0 * SIDE_MARGIN) - CENTER_DIVIDER) / 2.0
OPENING_H = BODY_H - TOP_MARGIN - BOTTOM_MARGIN
OPENING_Z = BOTTOM_MARGIN + (OPENING_H / 2.0)
LEFT_OPENING_X = -((CENTER_DIVIDER / 2.0) + (OPENING_W / 2.0))
RIGHT_OPENING_X = -LEFT_OPENING_X
FRONT_FACE_Y = BODY_D / 2.0
FRAME_Y = FRONT_FACE_Y - (FRAME_D / 2.0)

FILTER_W = OPENING_W - 0.034
FILTER_H = OPENING_H - 0.034
FILTER_D = BODY_D - 0.042
FILTER_CENTER_Y = -0.001
FILTER_FRONT_Y = FILTER_CENTER_Y + (FILTER_D / 2.0)

DOOR_W = OPENING_W + 0.014
DOOR_H = OPENING_H + 0.014
DOOR_T = 0.018
LEFT_HINGE_X = -(BODY_W / 2.0) + SIDE_MARGIN
RIGHT_HINGE_X = (BODY_W / 2.0) - SIDE_MARGIN
HINGE_Z = BODY_H / 2.0


def _add_filter_face(part, bay_name: str, center_x: float, filter_material: str, rib_material: str) -> None:
    part.visual(
        Box((FILTER_W, FILTER_D, FILTER_H)),
        origin=Origin(xyz=(center_x, FILTER_CENTER_Y, OPENING_Z)),
        material=filter_material,
        name=f"{bay_name}_filter_core",
    )

    rib_count = 7
    rib_step = FILTER_W / (rib_count + 1)
    rib_depth = 0.008
    rib_height = FILTER_H - 0.026
    rib_y = FILTER_FRONT_Y - (rib_depth / 2.0)
    for index in range(rib_count):
        rib_x = center_x - (FILTER_W / 2.0) + ((index + 1) * rib_step)
        part.visual(
            Box((0.008, rib_depth, rib_height)),
            origin=Origin(xyz=(rib_x, rib_y, OPENING_Z)),
            material=rib_material,
            name=f"{bay_name}_rib_{index + 1}",
        )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="double_bay_air_purifier", assets=ASSETS)

    housing = model.material("housing", rgba=(0.89, 0.91, 0.93, 1.0))
    trim = model.material("trim", rgba=(0.77, 0.80, 0.84, 1.0))
    door_finish = model.material("door_finish", rgba=(0.54, 0.58, 0.61, 1.0))
    handle_finish = model.material("handle_finish", rgba=(0.20, 0.23, 0.25, 1.0))
    filter_media = model.material("filter_media", rgba=(0.60, 0.72, 0.64, 1.0))
    filter_ribs = model.material("filter_ribs", rgba=(0.87, 0.92, 0.88, 1.0))
    vent_finish = model.material("vent_finish", rgba=(0.70, 0.75, 0.79, 1.0))

    body = model.part("body")
    body.visual(
        Box((BODY_W, WALL, BODY_H)),
        origin=Origin(xyz=(0.0, -(BODY_D / 2.0) + (WALL / 2.0), BODY_H / 2.0)),
        material=housing,
        name="rear_shell",
    )
    body.visual(
        Box((WALL, BODY_D, BODY_H)),
        origin=Origin(xyz=(-(BODY_W / 2.0) + (WALL / 2.0), 0.0, BODY_H / 2.0)),
        material=housing,
        name="left_shell",
    )
    body.visual(
        Box((WALL, BODY_D, BODY_H)),
        origin=Origin(xyz=((BODY_W / 2.0) - (WALL / 2.0), 0.0, BODY_H / 2.0)),
        material=housing,
        name="right_shell",
    )
    body.visual(
        Box((BODY_W, BODY_D, WALL)),
        origin=Origin(xyz=(0.0, 0.0, WALL / 2.0)),
        material=housing,
        name="bottom_shell",
    )
    body.visual(
        Box((BODY_W, BODY_D, WALL)),
        origin=Origin(xyz=(0.0, 0.0, BODY_H - (WALL / 2.0))),
        material=housing,
        name="top_shell",
    )
    body.visual(
        Box((CENTER_DIVIDER, BODY_D, BODY_H)),
        origin=Origin(xyz=(0.0, 0.0, BODY_H / 2.0)),
        material=trim,
        name="center_partition",
    )
    body.visual(
        Box((BODY_W, FRAME_D, TOP_MARGIN)),
        origin=Origin(xyz=(0.0, FRAME_Y, BODY_H - (TOP_MARGIN / 2.0))),
        material=trim,
        name="front_top_strip",
    )
    body.visual(
        Box((BODY_W, FRAME_D, BOTTOM_MARGIN)),
        origin=Origin(xyz=(0.0, FRAME_Y, BOTTOM_MARGIN / 2.0)),
        material=trim,
        name="front_bottom_strip",
    )
    body.visual(
        Box((SIDE_MARGIN, FRAME_D, OPENING_H)),
        origin=Origin(
            xyz=(
                -(BODY_W / 2.0) + (SIDE_MARGIN / 2.0),
                FRAME_Y,
                OPENING_Z,
            )
        ),
        material=trim,
        name="left_frame_strip",
    )
    body.visual(
        Box((CENTER_DIVIDER, FRAME_D, OPENING_H)),
        origin=Origin(xyz=(0.0, FRAME_Y, OPENING_Z)),
        material=trim,
        name="center_frame_strip",
    )
    body.visual(
        Box((SIDE_MARGIN, FRAME_D, OPENING_H)),
        origin=Origin(
            xyz=(
                (BODY_W / 2.0) - (SIDE_MARGIN / 2.0),
                FRAME_Y,
                OPENING_Z,
            )
        ),
        material=trim,
        name="right_frame_strip",
    )

    _add_filter_face(body, "left", LEFT_OPENING_X, filter_media, filter_ribs)
    _add_filter_face(body, "right", RIGHT_OPENING_X, filter_media, filter_ribs)

    vent_span = 0.44
    vent_count = 7
    vent_step = vent_span / (vent_count - 1)
    for index in range(vent_count):
        vent_x = -(vent_span / 2.0) + (index * vent_step)
        body.visual(
            Box((0.030, 0.150, 0.006)),
            origin=Origin(xyz=(vent_x, -0.010, BODY_H + 0.003)),
            material=vent_finish,
            name=f"top_vent_bar_{index + 1}",
        )

    body.visual(
        Box((0.160, 0.055, 0.004)),
        origin=Origin(xyz=(0.0, 0.085, BODY_H + 0.002)),
        material=handle_finish,
        name="control_panel",
    )

    left_door = model.part("left_door")
    left_door.visual(
        Box((DOOR_W, DOOR_T, DOOR_H)),
        origin=Origin(xyz=(DOOR_W / 2.0, DOOR_T / 2.0, 0.0)),
        material=door_finish,
        name="door_panel",
    )
    left_door.visual(
        Box((0.018, 0.026, 0.150)),
        origin=Origin(xyz=(DOOR_W - 0.034, DOOR_T + 0.013, 0.0)),
        material=handle_finish,
        name="handle",
    )

    right_door = model.part("right_door")
    right_door.visual(
        Box((DOOR_W, DOOR_T, DOOR_H)),
        origin=Origin(xyz=(-(DOOR_W / 2.0), DOOR_T / 2.0, 0.0)),
        material=door_finish,
        name="door_panel",
    )
    right_door.visual(
        Box((0.018, 0.026, 0.150)),
        origin=Origin(xyz=(-(DOOR_W - 0.034), DOOR_T + 0.013, 0.0)),
        material=handle_finish,
        name="handle",
    )

    model.articulation(
        "left_door_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=left_door,
        origin=Origin(xyz=(LEFT_HINGE_X, FRONT_FACE_Y, HINGE_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.8, lower=0.0, upper=1.45),
    )
    model.articulation(
        "right_door_hinge",
        ArticulationType.REVOLUTE,
        parent=body,
        child=right_door,
        origin=Origin(xyz=(RIGHT_HINGE_X, FRONT_FACE_Y, HINGE_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=8.0, velocity=1.8, lower=-1.45, upper=0.0),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    body = object_model.get_part("body")
    left_door = object_model.get_part("left_door")
    right_door = object_model.get_part("right_door")
    left_hinge = object_model.get_articulation("left_door_hinge")
    right_hinge = object_model.get_articulation("right_door_hinge")

    left_filter = body.get_visual("left_filter_core")
    right_filter = body.get_visual("right_filter_core")
    left_panel = left_door.get_visual("door_panel")
    right_panel = right_door.get_visual("door_panel")
    left_handle = left_door.get_visual("handle")
    right_handle = right_door.get_visual("handle")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    # Default exact visual sensor for joint mounting; keep unless scale makes it irrelevant.
    ctx.warn_if_articulation_origin_near_geometry(tol=0.015)
    # Default exact visual sensor for floating/disconnected subassemblies inside one part.
    ctx.warn_if_part_geometry_disconnected()
    # Default articulated-joint clearance gate; adapt only if the model is not articulated.
    ctx.check_articulation_overlaps(max_pose_samples=128)
    # Default broad overlap warning backstop; conservative and non-blocking by default.
    ctx.warn_if_overlaps(max_pose_samples=128, ignore_adjacent=True, ignore_fixed=True)

    # Use prompt-specific exact visual checks as the real completion criteria.
    # Cover each applicable category before returning:
    # - hero features are present and legible
    # - mounted parts are connected/seated, not floating
    # - important parts are in the right place
    # - key poses stay believable
    # - each new visible form or mechanism has a matching assertion
    # Resolve exact Part / Articulation / named Visual objects once here, then
    # pass those objects into ctx.expect_*, ctx.allow_*, and ctx.pose({joint: value}).
    # Prefer this object-first pattern over raw string test calls or global REFS bags.
    # Example:
    # lid = object_model.get_part("lid")
    # body = object_model.get_part("body")
    # lid_hinge = object_model.get_articulation("lid_hinge")
    # hinge_leaf = lid.get_visual("hinge_leaf")
    # body_leaf = body.get_visual("body_leaf")
    # ctx.expect_overlap(lid, body, axes="xy", min_overlap=0.05)
    # ctx.expect_gap(lid, body, axis="z", max_gap=0.001, max_penetration=0.0)
    # ctx.expect_contact(lid, body, elem_a=hinge_leaf, elem_b=body_leaf)
    # Add prompt-specific exact visual checks below; broad warn_if_* checks are not enough.
    ctx.expect_overlap(
        left_door,
        body,
        axes="xz",
        min_overlap=0.12,
        elem_a=left_panel,
        elem_b=left_filter,
        name="left door covers the left filter bay",
    )
    ctx.expect_overlap(
        right_door,
        body,
        axes="xz",
        min_overlap=0.12,
        elem_a=right_panel,
        elem_b=right_filter,
        name="right door covers the right filter bay",
    )
    ctx.expect_gap(
        left_door,
        body,
        axis="y",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=left_panel,
        name="left door sits flush on the purifier face",
    )
    ctx.expect_gap(
        right_door,
        body,
        axis="y",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=right_panel,
        name="right door sits flush on the purifier face",
    )
    ctx.expect_gap(
        right_door,
        left_door,
        axis="x",
        min_gap=0.001,
        max_gap=0.024,
        positive_elem=right_panel,
        negative_elem=left_panel,
        name="the two front doors meet with a narrow center seam",
    )
    ctx.expect_overlap(
        left_door,
        right_door,
        axes="z",
        min_overlap=0.14,
        elem_a=left_handle,
        elem_b=right_handle,
        name="both door handles sit at matching grasp height",
    )
    with ctx.pose({left_hinge: 1.20}):
        ctx.expect_gap(
            left_door,
            body,
            axis="y",
            min_gap=0.020,
            positive_elem=left_panel,
            negative_elem=left_filter,
            name="opening the left door exposes the left filter bay",
        )
        ctx.expect_overlap(
            right_door,
            body,
            axes="xz",
            min_overlap=0.12,
            elem_a=right_panel,
            elem_b=right_filter,
            name="the right bay stays sealed while only the left door opens",
        )
    with ctx.pose({right_hinge: -1.20}):
        ctx.expect_gap(
            right_door,
            body,
            axis="y",
            min_gap=0.020,
            positive_elem=right_panel,
            negative_elem=right_filter,
            name="opening the right door exposes the right filter bay",
        )
        ctx.expect_overlap(
            left_door,
            body,
            axes="xz",
            min_overlap=0.12,
            elem_a=left_panel,
            elem_b=left_filter,
            name="the left bay stays sealed while only the right door opens",
        )
    with ctx.pose({left_hinge: 1.20, right_hinge: -1.20}):
        ctx.expect_gap(
            left_door,
            body,
            axis="y",
            min_gap=0.020,
            positive_elem=left_panel,
            negative_elem=left_filter,
            name="left door swings clear of the left filter",
        )
        ctx.expect_gap(
            right_door,
            body,
            axis="y",
            min_gap=0.020,
            positive_elem=right_panel,
            negative_elem=right_filter,
            name="right door swings clear of the right filter",
        )
    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
