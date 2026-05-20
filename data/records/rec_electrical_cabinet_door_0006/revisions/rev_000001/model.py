from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math
import os
import pathlib
from pathlib import Path

_ORIG_OS_GETCWD = os.getcwd


def _safe_getcwd() -> str:
    try:
        return _ORIG_OS_GETCWD()
    except FileNotFoundError:
        try:
            os.chdir("/")
        except FileNotFoundError:
            pass
        return "/"


os.getcwd = _safe_getcwd

if not os.path.isabs(__file__):
    __file__ = f"/{__file__.lstrip('./')}"

try:
    os.chdir(os.path.dirname(__file__) or "/")
except FileNotFoundError:
    os.chdir("/")

_ORIG_PATH_CWD = Path.cwd.__func__


@classmethod
def _safe_path_cwd(cls):
    try:
        return _ORIG_PATH_CWD(cls)
    except FileNotFoundError:
        return cls("/")


Path.cwd = _safe_path_cwd
if hasattr(pathlib, "PosixPath"):
    pathlib.PosixPath.cwd = _safe_path_cwd

from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)

ASSETS = AssetContext.from_script("/tmp/model.py")

BODY_W = 0.60
BODY_H = 0.80
BODY_D = 0.25
SHEET_T = 0.003
INNER_W = BODY_W - 2.0 * SHEET_T
INNER_H = BODY_H - 2.0 * SHEET_T

SEAL_RAIL = 0.024
SEAL_DEPTH = 0.010
SEAL_CENTER_Y = 0.113

DOOR_W = 0.620
DOOR_H = 0.820
DOOR_LEFT_FROM_AXIS = 0.005
DOOR_CENTER_X = DOOR_LEFT_FROM_AXIS + DOOR_W / 2.0
DOOR_SKIN_CENTER_Y = -0.0085
DOOR_SKIN_T = 0.003

RETURN_OUTER_W = 0.530
RETURN_OUTER_H = 0.744
RETURN_RAIL = 0.022
RETURN_DEPTH = 0.022
RETURN_CENTER_Y = -0.035
PAN_CONNECTOR_DEPTH = 0.026
PAN_CONNECTOR_CENTER_Y = -0.0215

GASKET_OUTER_W = 0.584
GASKET_OUTER_H = 0.784
GASKET_RAIL = 0.010
GASKET_DEPTH = 0.004
GASKET_CENTER_Y = -0.012

HINGE_AXIS_X = -0.308
HINGE_AXIS_Y = 0.132
HINGE_RADIUS = 0.008
HINGE_LENGTH = 0.100
HINGE_BODY_KNUCKLE_LENGTH = 0.030
HINGE_DOOR_KNUCKLE_LENGTH = 0.040
HINGE_BODY_KNUCKLE_OFFSET = 0.035
HINGE_POSITIONS_Z = (-0.26, 0.0, 0.26)
HINGE_BODY_LEAF_SIZE = (0.008, 0.008, HINGE_LENGTH)
HINGE_BODY_LEAF_CENTER_X = -0.304
HINGE_BODY_LEAF_CENTER_Y = 0.120
HINGE_DOOR_LEAF_SIZE = (0.008, 0.008, HINGE_LENGTH)
HINGE_DOOR_LEAF_CENTER_X = 0.004
HINGE_DOOR_LEAF_CENTER_Y = -0.016
HINGE_DOOR_BRIDGE_SIZE = (0.016, 0.004, 0.040)
HINGE_DOOR_BRIDGE_CENTER_X = 0.008
HINGE_DOOR_BRIDGE_CENTER_Y = -0.010

LATCH_AXIS_X = DOOR_CENTER_X + DOOR_W / 2.0 - 0.070
LATCH_AXIS_Y = 0.000
LATCH_ESCUTCHEON_SIZE = (0.040, 0.007, 0.115)
LATCH_ESCUTCHEON_CENTER_Y = -0.0035
LATCH_HUB_R = 0.007
LATCH_HUB_LENGTH = 0.008
LATCH_PADDLE_SIZE = (0.014, 0.010, 0.102)
LATCH_PADDLE_CENTER_Y = 0.013
LATCH_GRIP_SIZE = (0.040, 0.010, 0.014)
LATCH_GRIP_CENTER_Y = 0.016
LATCH_GRIP_CENTER_Z = 0.032


def _add_rect_frame(
    part,
    *,
    center_x: float,
    outer_w: float,
    outer_h: float,
    rail: float,
    depth: float,
    center_y: float,
    prefix: str,
    material,
) -> None:
    span_w = outer_w - 2.0 * rail
    part.visual(
        Box((rail, depth, outer_h)),
        origin=Origin(xyz=(center_x - outer_w / 2.0 + rail / 2.0, center_y, 0.0)),
        material=material,
        name=f"{prefix}_left",
    )
    part.visual(
        Box((rail, depth, outer_h)),
        origin=Origin(xyz=(center_x + outer_w / 2.0 - rail / 2.0, center_y, 0.0)),
        material=material,
        name=f"{prefix}_right",
    )
    part.visual(
        Box((span_w, depth, rail)),
        origin=Origin(xyz=(center_x, center_y, outer_h / 2.0 - rail / 2.0)),
        material=material,
        name=f"{prefix}_top",
    )
    part.visual(
        Box((span_w, depth, rail)),
        origin=Origin(xyz=(center_x, center_y, -outer_h / 2.0 + rail / 2.0)),
        material=material,
        name=f"{prefix}_bottom",
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="nema_4x_stainless_cabinet", assets=ASSETS)

    stainless = model.material("stainless", rgba=(0.71, 0.73, 0.75, 1.0))
    stainless_shadow = model.material("stainless_shadow", rgba=(0.58, 0.61, 0.64, 1.0))
    gasket_black = model.material("gasket_black", rgba=(0.10, 0.10, 0.11, 1.0))
    latch_metal = model.material("latch_metal", rgba=(0.42, 0.44, 0.46, 1.0))

    body = model.part("body")
    body.visual(
        Box((BODY_W, SHEET_T, BODY_H)),
        origin=Origin(xyz=(0.0, -BODY_D / 2.0 + SHEET_T / 2.0, 0.0)),
        material=stainless,
        name="back_panel",
    )
    body.visual(
        Box((SHEET_T, BODY_D - SHEET_T, BODY_H)),
        origin=Origin(xyz=(-BODY_W / 2.0 + SHEET_T / 2.0, SHEET_T / 2.0, 0.0)),
        material=stainless,
        name="left_wall",
    )
    body.visual(
        Box((SHEET_T, BODY_D - SHEET_T, BODY_H)),
        origin=Origin(xyz=(BODY_W / 2.0 - SHEET_T / 2.0, SHEET_T / 2.0, 0.0)),
        material=stainless,
        name="right_wall",
    )
    body.visual(
        Box((BODY_W - 2.0 * SHEET_T, BODY_D - SHEET_T, SHEET_T)),
        origin=Origin(xyz=(0.0, SHEET_T / 2.0, BODY_H / 2.0 - SHEET_T / 2.0)),
        material=stainless,
        name="top_wall",
    )
    body.visual(
        Box((BODY_W - 2.0 * SHEET_T, BODY_D - SHEET_T, SHEET_T)),
        origin=Origin(xyz=(0.0, SHEET_T / 2.0, -BODY_H / 2.0 + SHEET_T / 2.0)),
        material=stainless,
        name="bottom_wall",
    )
    _add_rect_frame(
        body,
        center_x=0.0,
        outer_w=INNER_W,
        outer_h=INNER_H,
        rail=SEAL_RAIL,
        depth=SEAL_DEPTH,
        center_y=SEAL_CENTER_Y,
        prefix="opening_seal",
        material=stainless_shadow,
    )
    for hinge_name, z_pos in zip(("top", "mid", "bottom"), HINGE_POSITIONS_Z):
        body.visual(
            Box(HINGE_BODY_LEAF_SIZE),
            origin=Origin(xyz=(HINGE_BODY_LEAF_CENTER_X, HINGE_BODY_LEAF_CENTER_Y, z_pos)),
            material=stainless_shadow,
            name=f"hinge_body_leaf_{hinge_name}",
        )
        body.visual(
            Cylinder(radius=HINGE_RADIUS, length=HINGE_BODY_KNUCKLE_LENGTH),
            origin=Origin(xyz=(HINGE_AXIS_X, HINGE_AXIS_Y, z_pos - HINGE_BODY_KNUCKLE_OFFSET)),
            material=stainless,
            name=f"hinge_body_knuckle_lower_{hinge_name}",
        )
        body.visual(
            Cylinder(radius=HINGE_RADIUS, length=HINGE_BODY_KNUCKLE_LENGTH),
            origin=Origin(xyz=(HINGE_AXIS_X, HINGE_AXIS_Y, z_pos + HINGE_BODY_KNUCKLE_OFFSET)),
            material=stainless,
            name=f"hinge_body_knuckle_upper_{hinge_name}",
        )
    body.inertial = Inertial.from_geometry(Box((BODY_W, BODY_D, BODY_H)), mass=20.0)

    door = model.part("door")
    door.visual(
        Box((DOOR_W, DOOR_SKIN_T, DOOR_H)),
        origin=Origin(xyz=(DOOR_CENTER_X, DOOR_SKIN_CENTER_Y, 0.0)),
        material=stainless,
        name="door_skin",
    )
    _add_rect_frame(
        door,
        center_x=DOOR_CENTER_X,
        outer_w=RETURN_OUTER_W,
        outer_h=RETURN_OUTER_H,
        rail=RETURN_RAIL,
        depth=RETURN_DEPTH,
        center_y=RETURN_CENTER_Y,
        prefix="door_return",
        material=stainless_shadow,
    )
    _add_rect_frame(
        door,
        center_x=DOOR_CENTER_X,
        outer_w=GASKET_OUTER_W,
        outer_h=GASKET_OUTER_H,
        rail=GASKET_RAIL,
        depth=GASKET_DEPTH,
        center_y=GASKET_CENTER_Y,
        prefix="gasket",
        material=gasket_black,
    )
    door.visual(
        Box((RETURN_RAIL, PAN_CONNECTOR_DEPTH, RETURN_OUTER_H)),
        origin=Origin(
            xyz=(DOOR_CENTER_X - RETURN_OUTER_W / 2.0 + RETURN_RAIL / 2.0, PAN_CONNECTOR_CENTER_Y, 0.0)
        ),
        material=stainless_shadow,
        name="pan_left",
    )
    door.visual(
        Box((RETURN_RAIL, PAN_CONNECTOR_DEPTH, RETURN_OUTER_H)),
        origin=Origin(
            xyz=(DOOR_CENTER_X + RETURN_OUTER_W / 2.0 - RETURN_RAIL / 2.0, PAN_CONNECTOR_CENTER_Y, 0.0)
        ),
        material=stainless_shadow,
        name="pan_right",
    )
    door.visual(
        Box((RETURN_OUTER_W - 2.0 * RETURN_RAIL, PAN_CONNECTOR_DEPTH, RETURN_RAIL)),
        origin=Origin(
            xyz=(DOOR_CENTER_X, PAN_CONNECTOR_CENTER_Y, RETURN_OUTER_H / 2.0 - RETURN_RAIL / 2.0)
        ),
        material=stainless_shadow,
        name="pan_top",
    )
    door.visual(
        Box((RETURN_OUTER_W - 2.0 * RETURN_RAIL, PAN_CONNECTOR_DEPTH, RETURN_RAIL)),
        origin=Origin(
            xyz=(DOOR_CENTER_X, PAN_CONNECTOR_CENTER_Y, -RETURN_OUTER_H / 2.0 + RETURN_RAIL / 2.0)
        ),
        material=stainless_shadow,
        name="pan_bottom",
    )
    for hinge_name, z_pos in zip(("top", "mid", "bottom"), HINGE_POSITIONS_Z):
        door.visual(
            Box(HINGE_DOOR_LEAF_SIZE),
            origin=Origin(xyz=(HINGE_DOOR_LEAF_CENTER_X, HINGE_DOOR_LEAF_CENTER_Y, z_pos)),
            material=stainless_shadow,
            name=f"hinge_door_leaf_{hinge_name}",
        )
        door.visual(
            Box(HINGE_DOOR_BRIDGE_SIZE),
            origin=Origin(xyz=(HINGE_DOOR_BRIDGE_CENTER_X, HINGE_DOOR_BRIDGE_CENTER_Y, z_pos)),
            material=stainless_shadow,
            name=f"hinge_door_bridge_{hinge_name}",
        )
        door.visual(
            Cylinder(radius=HINGE_RADIUS, length=HINGE_DOOR_KNUCKLE_LENGTH),
            origin=Origin(xyz=(0.0, 0.0, z_pos)),
            material=stainless,
            name=f"hinge_door_knuckle_{hinge_name}",
        )
    door.visual(
        Box(LATCH_ESCUTCHEON_SIZE),
        origin=Origin(xyz=(LATCH_AXIS_X, LATCH_ESCUTCHEON_CENTER_Y, 0.0)),
        material=latch_metal,
        name="latch_escutcheon",
    )
    door.inertial = Inertial.from_geometry(
        Box((DOOR_W, 0.040, DOOR_H)),
        mass=8.0,
        origin=Origin(xyz=(DOOR_CENTER_X, -0.015, 0.0)),
    )

    latch = model.part("latch")
    latch.visual(
        Cylinder(radius=LATCH_HUB_R, length=LATCH_HUB_LENGTH),
        origin=Origin(xyz=(0.0, LATCH_HUB_LENGTH / 2.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=latch_metal,
        name="hub",
    )
    latch.visual(
        Box(LATCH_PADDLE_SIZE),
        origin=Origin(xyz=(0.0, LATCH_PADDLE_CENTER_Y, 0.0)),
        material=latch_metal,
        name="paddle",
    )
    latch.visual(
        Box(LATCH_GRIP_SIZE),
        origin=Origin(xyz=(0.0, LATCH_GRIP_CENTER_Y, LATCH_GRIP_CENTER_Z)),
        material=latch_metal,
        name="grip",
    )
    latch.inertial = Inertial.from_geometry(
        Box((0.040, 0.024, 0.110)),
        mass=0.25,
        origin=Origin(xyz=(0.0, 0.012, 0.0)),
    )

    model.articulation(
        "body_to_door",
        ArticulationType.REVOLUTE,
        parent=body,
        child=door,
        origin=Origin(xyz=(HINGE_AXIS_X, HINGE_AXIS_Y, 0.0)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=30.0,
            velocity=1.8,
            lower=0.0,
            upper=math.radians(115.0),
        ),
    )
    model.articulation(
        "door_to_latch",
        ArticulationType.REVOLUTE,
        parent=door,
        child=latch,
        origin=Origin(xyz=(LATCH_AXIS_X, LATCH_AXIS_Y, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(
            effort=2.0,
            velocity=4.0,
            lower=0.0,
            upper=math.pi / 2.0,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root, seed=0)
    body = object_model.get_part("body")
    door = object_model.get_part("door")
    latch = object_model.get_part("latch")
    body_to_door = object_model.get_articulation("body_to_door")
    door_to_latch = object_model.get_articulation("door_to_latch")

    back_panel = body.get_visual("back_panel")
    opening_seal_left = body.get_visual("opening_seal_left")
    opening_seal_right = body.get_visual("opening_seal_right")
    opening_seal_top = body.get_visual("opening_seal_top")
    opening_seal_bottom = body.get_visual("opening_seal_bottom")
    hinge_body_knuckle_upper_top = body.get_visual("hinge_body_knuckle_upper_top")
    hinge_body_knuckle_lower_bottom = body.get_visual("hinge_body_knuckle_lower_bottom")

    door_skin = door.get_visual("door_skin")
    gasket_left = door.get_visual("gasket_left")
    gasket_right = door.get_visual("gasket_right")
    gasket_top = door.get_visual("gasket_top")
    gasket_bottom = door.get_visual("gasket_bottom")
    hinge_door_knuckle_top = door.get_visual("hinge_door_knuckle_top")
    hinge_door_knuckle_bottom = door.get_visual("hinge_door_knuckle_bottom")
    latch_escutcheon = door.get_visual("latch_escutcheon")

    latch_hub = latch.get_visual("hub")
    latch_paddle = latch.get_visual("paddle")

    ctx.check_model_valid()
    ctx.check_mesh_files_exist()
    ctx.allow_overlap(
        door,
        latch,
        elem_a=latch_escutcheon,
        elem_b=latch_hub,
        reason="Quarter-turn latch spindle passes through the escutcheon bore; the bore is implied rather than cut through the plate.",
    )
    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()
    ctx.fail_if_parts_overlap_in_current_pose()
    ctx.fail_if_articulation_overlaps(max_pose_samples=32)
    ctx.fail_if_parts_overlap_in_sampled_poses(
        max_pose_samples=32,
        ignore_adjacent=False,
        ignore_fixed=True,
    )

    back_panel_aabb = ctx.part_element_world_aabb(body, elem=back_panel)
    door_skin_aabb = ctx.part_element_world_aabb(door, elem=door_skin)
    assert back_panel_aabb is not None
    assert door_skin_aabb is not None
    ctx.check(
        "body_size_realistic",
        abs((back_panel_aabb[1][0] - back_panel_aabb[0][0]) - BODY_W) < 0.01
        and abs((back_panel_aabb[1][2] - back_panel_aabb[0][2]) - BODY_H) < 0.01,
        details=str(back_panel_aabb),
    )
    ctx.check(
        "door_size_realistic",
        abs((door_skin_aabb[1][0] - door_skin_aabb[0][0]) - DOOR_W) < 0.01
        and abs((door_skin_aabb[1][2] - door_skin_aabb[0][2]) - DOOR_H) < 0.01,
        details=str(door_skin_aabb),
    )

    ctx.expect_contact(body, door, elem_a=hinge_body_knuckle_upper_top, elem_b=hinge_door_knuckle_top)
    ctx.expect_contact(body, door, elem_a=hinge_body_knuckle_lower_bottom, elem_b=hinge_door_knuckle_bottom)
    ctx.expect_contact(door, latch, elem_a=latch_escutcheon, elem_b=latch_hub)
    ctx.expect_overlap(door, body, axes="xz", min_overlap=0.55, elem_a=door_skin)
    ctx.expect_gap(
        door,
        body,
        axis="y",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=gasket_left,
        negative_elem=opening_seal_left,
    )
    ctx.expect_gap(
        door,
        body,
        axis="y",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=gasket_right,
        negative_elem=opening_seal_right,
    )
    ctx.expect_gap(
        door,
        body,
        axis="y",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=gasket_top,
        negative_elem=opening_seal_top,
    )
    ctx.expect_gap(
        door,
        body,
        axis="y",
        max_gap=0.001,
        max_penetration=0.0,
        positive_elem=gasket_bottom,
        negative_elem=opening_seal_bottom,
    )

    door_limits = body_to_door.motion_limits
    latch_limits = door_to_latch.motion_limits
    assert door_limits is not None and door_limits.upper is not None
    assert latch_limits is not None and latch_limits.upper is not None

    closed_skin_aabb = ctx.part_element_world_aabb(door, elem=door_skin)
    closed_paddle_aabb = ctx.part_element_world_aabb(latch, elem=latch_paddle)
    assert closed_skin_aabb is not None
    assert closed_paddle_aabb is not None

    with ctx.pose({body_to_door: door_limits.upper}):
        ctx.fail_if_parts_overlap_in_current_pose(name="door_open_no_overlap")
        ctx.fail_if_isolated_parts(name="door_open_no_floating")
        ctx.expect_contact(body, door, elem_a=hinge_body_knuckle_upper_top, elem_b=hinge_door_knuckle_top)
        open_skin_aabb = ctx.part_element_world_aabb(door, elem=door_skin)
        assert open_skin_aabb is not None
        ctx.check(
            "door_swings_outward",
            open_skin_aabb[1][1] > closed_skin_aabb[1][1] + 0.45,
            details=f"closed y={closed_skin_aabb[1][1]:.3f}, open y={open_skin_aabb[1][1]:.3f}",
        )

    with ctx.pose({door_to_latch: latch_limits.upper}):
        ctx.fail_if_parts_overlap_in_current_pose(name="latch_turn_no_overlap")
        ctx.fail_if_isolated_parts(name="latch_turn_no_floating")
        ctx.expect_contact(door, latch, elem_a=latch_escutcheon, elem_b=latch_hub)
        turned_paddle_aabb = ctx.part_element_world_aabb(latch, elem=latch_paddle)
        assert turned_paddle_aabb is not None
        closed_x = closed_paddle_aabb[1][0] - closed_paddle_aabb[0][0]
        closed_z = closed_paddle_aabb[1][2] - closed_paddle_aabb[0][2]
        turned_x = turned_paddle_aabb[1][0] - turned_paddle_aabb[0][0]
        turned_z = turned_paddle_aabb[1][2] - turned_paddle_aabb[0][2]
        ctx.check(
            "latch_quarter_turn_reads_correctly",
            closed_z > closed_x * 3.0 and turned_x > turned_z * 3.0,
            details=f"closed x/z={closed_x:.3f}/{closed_z:.3f}, turned x/z={turned_x:.3f}/{turned_z:.3f}",
        )

    with ctx.pose({body_to_door: door_limits.upper, door_to_latch: latch_limits.upper}):
        ctx.fail_if_parts_overlap_in_current_pose(name="open_door_and_turned_latch_no_overlap")
        ctx.fail_if_isolated_parts(name="open_door_and_turned_latch_no_floating")
        ctx.expect_contact(body, door, elem_a=hinge_body_knuckle_lower_bottom, elem_b=hinge_door_knuckle_bottom)
        ctx.expect_contact(door, latch, elem_a=latch_escutcheon, elem_b=latch_hub)

    return ctx.report()


# >>> USER_CODE_END

object_model = build_object_model()
